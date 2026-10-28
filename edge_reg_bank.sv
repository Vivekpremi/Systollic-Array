`timescale 1ns/1ps
//=============================================================================
// edge_reg_bank.sv  --  input blocks -> operands at the array edges
//
// WHAT IT DOES
//   Holds one input block per lane and feeds its elements, one at a time, into
//   the edge of the systolic array:
//     row lane i    : a block of row i of A    -> enters PE[i][0] (left edge)
//     column lane j : a block of column j of B -> enters PE[0][j] (top edge)
//   Element k of a block is the k-th term of the dot product (k = 0 first).
//   Each element's L is turned into the PE's log operand by `descaling`
//   (L << scale); sign, zero flag and the block's E go along with it.
//
// BLOCK FORMAT (BLOCK_WIDTH bits, 48 at the defaults; same as the output block)
//   block[MANT_LSB  +: ARRAY_SIZE*BLOCK_MANTISSA_WIDTH]  L of each element, signed,
//                                                        element 0 in the lowest bits
//   block[ZERO_LSB  +: ARRAY_SIZE]                       zero flag of each element
//   block[SIGN_LSB  +: ARRAY_SIZE]                       sign of each element (1 = negative)
//   block[SCALE_LSB +: SCALE_FACTOR_WIDTH]               scale
//   block[EXP_LSB   +: EXP_WIDTH]                        E, signed
//   remaining top bits                                   unused (0)
//   value = (-1)^sign * 2^(E + L * 2^scale / 2^FRAC_WIDTH), or 0 if zero = 1
//   Defaults: L [23:0], zero [27:24], sign [31:28], scale [35:32], E [43:36].
//
// LOADING
//   row_we[i] / col_we[i] store a whole block in one clock and restart that
//   lane at element 0. The lane then shows element 0 with valid = 1 at once.
//
// PACING (one element per product)
//   set_i_ready[i] pulses when PE[i][0] finishes a product. On its rising
//   edge, row lane i AND column lane i both move to their next element. After
//   the last element the lane's valid goes low (block used up).
//
// LIMITATIONS / NOTES
//   - Load all lanes in the SAME clock (the AXI wrapper and the testbench do).
//     A lane becomes valid as soon as it is written, so PEs on lanes loaded
//     earlier would start early and the systolic timing would break.
//   - Column lane i is paced by PE[i][0], not by the PE it feeds (PE[0][i]);
//     this works because all PEs take the same number of clocks per product.
//   - A zero element still uses its slot: valid/last/pointer are unchanged,
//     the PE just skips the add.
//   - scale must be <= MANTISSA_WIDTH - BLOCK_MANTISSA_WIDTH, or L << scale
//     wraps (see descaling).
//=============================================================================
module edge_reg_bank #(
    parameter ARRAY_SIZE           = 4,    // lanes per side = elements per block
    parameter MANTISSA_WIDTH       = 16,   // bits of the PE log operand (descaling output)
    parameter EXP_WIDTH            = 8,    // bits of E
    parameter BLOCK_MANTISSA_WIDTH = 6,    // bits of one L (signed)
    parameter SCALE_FACTOR_WIDTH   = 4,    // bits of the scale
    parameter LOG2_ARRAY_SIZE      = $clog2(ARRAY_SIZE),   // bits of an element pointer
    // ---- block geometry (identical in format_convertor and the testbenches) ----
    parameter MANT_BUS_WIDTH = ARRAY_SIZE*BLOCK_MANTISSA_WIDTH,   // all L fields
    parameter BLOCK_BITS     = MANT_BUS_WIDTH + 2*ARRAY_SIZE
                               + SCALE_FACTOR_WIDTH + EXP_WIDTH,  // bits used
    parameter BLOCK_WIDTH    = ((BLOCK_BITS + 7)/8)*8             // rounded up to whole bytes
)(
    input  wire clk,
    input  wire rst_n,

    // ---- write side: one block per lane ----
    input  wire                    row_we   [ARRAY_SIZE-1:0],   // store row_block[i] this clock
    input  wire                    col_we   [ARRAY_SIZE-1:0],   // store col_block[i] this clock
    input  wire [BLOCK_WIDTH-1:0]   row_block [ARRAY_SIZE-1:0], // block of row i of A
    input  wire [BLOCK_WIDTH-1:0]   col_block [ARRAY_SIZE-1:0], // block of column i of B

    // ---- pacing from the array: PE[i][0] finished a product (1-clock pulse) ----
    input  wire                    set_i_ready [ARRAY_SIZE-1:0],

    // ---- current element of each lane, to the array edges ----
    output wire signed [MANTISSA_WIDTH-1:0] mantissa_row_0 [ARRAY_SIZE-1:0],   // L << scale
    output wire signed [MANTISSA_WIDTH-1:0] mantissa_col_0 [ARRAY_SIZE-1:0],
    output wire signed [EXP_WIDTH-1:0]   exponent_row_0 [ARRAY_SIZE-1:0],      // block E
    output wire signed [EXP_WIDTH-1:0]   exponent_col_0 [ARRAY_SIZE-1:0],
    output wire                          sign_row_0     [ARRAY_SIZE-1:0],      // 1 = negative
    output wire                          sign_col_0     [ARRAY_SIZE-1:0],
    output wire                          zero_row_0     [ARRAY_SIZE-1:0],      // 1 = exactly 0
    output wire                          zero_col_0     [ARRAY_SIZE-1:0],
    output wire                          valid_in_row_0 [ARRAY_SIZE-1:0],      // lane has an element to give
    output wire                          valid_in_col_0 [ARRAY_SIZE-1:0],
    output wire                          last_in_row_0  [ARRAY_SIZE-1:0],      // that element is the last one
    output wire                          last_in_col_0  [ARRAY_SIZE-1:0],

    // ---- status ----
    output wire                          bank_primed    // every lane has a block loaded
);

    // bit position of each field inside a block
    localparam MANT_LSB  = 0;
    localparam ZERO_LSB  = MANT_LSB  + MANT_BUS_WIDTH;
    localparam SIGN_LSB  = ZERO_LSB  + ARRAY_SIZE;
    localparam SCALE_LSB = SIGN_LSB  + ARRAY_SIZE;
    localparam EXP_LSB   = SCALE_LSB + SCALE_FACTOR_WIDTH;

    wire lane_primed [ARRAY_SIZE-1:0];   // both lanes i have a block

    genvar li;
    generate
        for (li = 0; li < ARRAY_SIZE; li = li + 1) begin : lane

            // ---- storage: the whole block in one register per lane ----
            //   row_block_r / col_block_r : the stored blocks
            //   row_loaded / col_loaded   : a block has been written since reset
            // The *_w wires are just slices of the stored block (no logic).
            reg [BLOCK_WIDTH-1:0]         row_block_r, col_block_r;
            reg                          row_loaded,  col_loaded;

            wire [MANT_BUS_WIDTH-1:0]   row_mants_w = row_block_r[MANT_LSB  +: MANT_BUS_WIDTH];
            wire [MANT_BUS_WIDTH-1:0]   col_mants_w = col_block_r[MANT_LSB  +: MANT_BUS_WIDTH];
            wire [SCALE_FACTOR_WIDTH-1:0]row_scale_w = row_block_r[SCALE_LSB +: SCALE_FACTOR_WIDTH];
            wire [SCALE_FACTOR_WIDTH-1:0]col_scale_w = col_block_r[SCALE_LSB +: SCALE_FACTOR_WIDTH];
            wire signed [EXP_WIDTH-1:0]  row_exp_w   = row_block_r[EXP_LSB   +: EXP_WIDTH];
            wire signed [EXP_WIDTH-1:0]  col_exp_w   = col_block_r[EXP_LSB   +: EXP_WIDTH];
            wire [ARRAY_SIZE-1:0]        row_sv_w    = row_block_r[SIGN_LSB  +: ARRAY_SIZE];   // sign bits
            wire [ARRAY_SIZE-1:0]        col_sv_w    = col_block_r[SIGN_LSB  +: ARRAY_SIZE];
            wire [ARRAY_SIZE-1:0]        row_z_w     = row_block_r[ZERO_LSB  +: ARRAY_SIZE];   // zero flags
            wire [ARRAY_SIZE-1:0]        col_z_w     = col_block_r[ZERO_LSB  +: ARRAY_SIZE];

            //   row_ptr / col_ptr   : element now shown on the lane (0..ARRAY_SIZE-1)
            //   set_i_ready_q       : set_i_ready one clock ago (to find its rising edge)
            //   row_done / col_done : all elements of the block have been used
            reg [LOG2_ARRAY_SIZE-1:0]    row_ptr, col_ptr;
            reg                          set_i_ready_q;
            reg                          row_done, col_done;

            wire demand_rise = set_i_ready[li] & ~set_i_ready_q;   // PE[i][0] just finished a product

            always @(posedge clk or negedge rst_n) begin
                if (!rst_n) begin
                    row_block_r    <= '0; col_block_r  <= '0;
                    row_loaded    <= 1'b0; col_loaded <= 1'b0;
                    row_ptr       <= '0; col_ptr     <= '0;
                    row_done      <= 1'b0; col_done   <= 1'b0;
                    set_i_ready_q <= 1'b0;
                end else begin
                    set_i_ready_q <= set_i_ready[li];

                    // write: store the block, restart at element 0
                    if (row_we[li]) begin
                        row_block_r <= row_block[li];
                        row_loaded <= 1'b1;
                        row_ptr    <= '0;
                        row_done   <= 1'b0;
                    end
                    if (col_we[li]) begin
                        col_block_r <= col_block[li];
                        col_loaded <= 1'b1;
                        col_ptr    <= '0;
                        col_done   <= 1'b0;
                    end

                    // read: next element on each demand pulse (row i and column i together);
                    // after the last element the lane is done
                    if (demand_rise) begin
                        if (row_loaded && !row_done) begin
                            if (row_ptr == LOG2_ARRAY_SIZE'(ARRAY_SIZE-1)) row_done <= 1'b1;
                            else                                           row_ptr  <= row_ptr + 1'b1;
                        end
                        if (col_loaded && !col_done) begin
                            if (col_ptr == LOG2_ARRAY_SIZE'(ARRAY_SIZE-1)) col_done <= 1'b1;
                            else                                           col_ptr  <= col_ptr + 1'b1;
                        end
                    end
                end
            end

            // ---- current element: its L, turned into the PE log operand ----
            wire signed [BLOCK_MANTISSA_WIDTH-1:0] row_mant_sel =
                    row_mants_w[row_ptr*BLOCK_MANTISSA_WIDTH +: BLOCK_MANTISSA_WIDTH];
            wire signed [BLOCK_MANTISSA_WIDTH-1:0] col_mant_sel =
                    col_mants_w[col_ptr*BLOCK_MANTISSA_WIDTH +: BLOCK_MANTISSA_WIDTH];

            descaling #(
                .MANTISSA_WIDTH(BLOCK_MANTISSA_WIDTH),
                .SCALE_FACTOR_WIDTH(SCALE_FACTOR_WIDTH),
                .PE_MANTISSA_WIDTH(MANTISSA_WIDTH)
            ) u_descale_row (
                .mant(row_mant_sel),
                .inp_scale_factor(row_scale_w),
                .inp_pe(mantissa_row_0[li])
            );

            descaling #(
                .MANTISSA_WIDTH(BLOCK_MANTISSA_WIDTH),
                .SCALE_FACTOR_WIDTH(SCALE_FACTOR_WIDTH),
                .PE_MANTISSA_WIDTH(MANTISSA_WIDTH)
            ) u_descale_col (
                .mant(col_mant_sel),
                .inp_scale_factor(col_scale_w),
                .inp_pe(mantissa_col_0[li])
            );

            // ---- edge outputs ----
            assign exponent_row_0[li] = row_exp_w;
            assign exponent_col_0[li] = col_exp_w;

            assign sign_row_0[li] = row_sv_w[row_ptr];   // sign of the current element
            assign sign_col_0[li] = col_sv_w[col_ptr];

            assign zero_row_0[li] = row_z_w[row_ptr];    // zero flag of the current element
            assign zero_col_0[li] = col_z_w[col_ptr];

            // valid while a loaded block still has elements left
            assign valid_in_row_0[li] = row_loaded && !row_done;
            assign valid_in_col_0[li] = col_loaded && !col_done;

            // last = the current element is element ARRAY_SIZE-1
            assign last_in_row_0[li] = valid_in_row_0[li] &&
                                        (row_ptr == LOG2_ARRAY_SIZE'(ARRAY_SIZE-1));
            assign last_in_col_0[li] = valid_in_col_0[li] &&
                                        (col_ptr == LOG2_ARRAY_SIZE'(ARRAY_SIZE-1));

            assign lane_primed[li] = row_loaded && col_loaded;
        end
    endgenerate

    // bank_primed = every row lane and column lane has a block
    reg  all_primed;
    integer k;
    always @(*) begin
        all_primed = 1'b1;
        for (k = 0; k < ARRAY_SIZE; k = k + 1)
            if (!lane_primed[k]) all_primed = 1'b0;
    end
    assign bank_primed = all_primed;

endmodule

