`timescale 1ns/1ps
//=============================================================================
// edge_reg_bank.sv
//
// Register bank + descalers sitting on the LEFT (row) and TOP (column) edges
// of the systolic array. This is the future landing zone for AXI-Stream beats;
// for now it is filled directly by the testbench via a simple per-lane
// write-enable + packed-block interface.
//
// Per lane (ARRAY_SIZE row lanes + ARRAY_SIZE col lanes) it holds one block:
//     4 x BLOCK_MANTISSA_WIDTH-bit mantissas   (packed)
//     1 x SCALE_FACTOR_WIDTH-bit  scale factor (shared across the 4 mantissas)
//     1 x EXP_WIDTH-bit           exponent     (shared across the 4 mantissas)
//
// A `descaling` instance per lane expands the CURRENTLY-SELECTED 5-bit mantissa
// (mant << scale_factor) into the MANTISSA_WIDTH-bit value the PE expects on
// mantissa_row_0[i] / mantissa_col_0[i].
//
// READ PACING: exactly one mantissa is consumed per demand pulse. The array
// exposes set_i_ready[i] (= demand_from_mem[i] = anti_log_valid), which is a
// ONE-CYCLE PULSE per operand (real CORDIC drives valid_out = (state==out),
// high for a single cycle). We advance on its rising edge -- for a one-cycle
// pulse this is equivalent to "advance while high", but edge-detect stays
// correct even if the signal ever widened to multiple cycles. Per user spec,
// row i and column i pull together, so a single set_i_ready[i] advances BOTH
// the row-lane and col-lane read pointers for index i.
//=============================================================================
module edge_reg_bank #(
    parameter ARRAY_SIZE           = 4,
    parameter MANTISSA_WIDTH       = 16,   // width the PE consumes (descaling.inp_pe)
    parameter EXP_WIDTH            = 8,
    parameter BLOCK_MANTISSA_WIDTH = 5,    // width of one packed BFP mantissa
    parameter SCALE_FACTOR_WIDTH   = 4,
    parameter LOG2_ARRAY_SIZE      = $clog2(ARRAY_SIZE),
    parameter MANT_BUS_WIDTH      = ARRAY_SIZE*BLOCK_MANTISSA_WIDTH, // mantissas only (20b)
    // one BLOCK = {exp[8], scale[4], mant3..mant0[20]} = 32 bits exactly
    parameter BLOCK_WIDTH           = EXP_WIDTH + SCALE_FACTOR_WIDTH + MANT_BUS_WIDTH
)(
    input  wire clk,
    input  wire rst_n,

    // ---- Testbench/driver write side: ONE packed 32-bit BLOCK per lane ----
    //   block[31:24] = exponent
    //   block[23:20] = scale_factor
    //   block[19:0]  = 4 packed mantissas, mantissa[0] in the LSBs
    input  wire                    row_we   [ARRAY_SIZE-1:0], // write-enable, row lane i
    input  wire                    col_we   [ARRAY_SIZE-1:0], // write-enable, col lane i
    input  wire [BLOCK_WIDTH-1:0]   row_block [ARRAY_SIZE-1:0], // packed block, row lane i
    input  wire [BLOCK_WIDTH-1:0]   col_block [ARRAY_SIZE-1:0],

    // ---- Demand from the array (one-cycle pulse, one per row/col index) ----
    input  wire                    set_i_ready [ARRAY_SIZE-1:0],

    // ---- To the systolic array edges ----
    output wire [MANTISSA_WIDTH-1:0]     mantissa_row_0 [ARRAY_SIZE-1:0],
    output wire [MANTISSA_WIDTH-1:0]     mantissa_col_0 [ARRAY_SIZE-1:0],
    output wire [EXP_WIDTH-1:0]          exponent_row_0 [ARRAY_SIZE-1:0],
    output wire [EXP_WIDTH-1:0]          exponent_col_0 [ARRAY_SIZE-1:0],
    output wire                          valid_in_row_0 [ARRAY_SIZE-1:0],
    output wire                          valid_in_col_0 [ARRAY_SIZE-1:0],
    output wire                          last_in_row_0  [ARRAY_SIZE-1:0],
    output wire                          last_in_col_0  [ARRAY_SIZE-1:0],

    // ---- Status ----
    output wire                          bank_primed,   // every lane has a block loaded

    // ---- Debug observability (read pointers per lane) ----
    output wire [LOG2_ARRAY_SIZE-1:0]    dbg_row_ptr [ARRAY_SIZE-1:0],
    output wire [LOG2_ARRAY_SIZE-1:0]    dbg_col_ptr [ARRAY_SIZE-1:0]
);

    // field offsets within the 32-bit block
    localparam MANT_LSB  = 0;
    localparam SCALE_LSB = MANT_BUS_WIDTH;                       // 20
    localparam EXP_LSB   = MANT_BUS_WIDTH + SCALE_FACTOR_WIDTH;  // 24

    wire lane_primed [ARRAY_SIZE-1:0];

    genvar li;
    generate
        for (li = 0; li < ARRAY_SIZE; li = li + 1) begin : lane

            // ----------------- storage: one 32-bit block per lane -----------------
            // The whole block arrives as one atomic write, so it lives in one reg;
            // the fields below are free wire slices off the flop outputs (no gates).
            reg [BLOCK_WIDTH-1:0]         row_block_r, col_block_r;
            reg                          row_loaded,  col_loaded;

            wire [MANT_BUS_WIDTH-1:0]   row_mants_w = row_block_r[MANT_LSB  +: MANT_BUS_WIDTH];
            wire [MANT_BUS_WIDTH-1:0]   col_mants_w = col_block_r[MANT_LSB  +: MANT_BUS_WIDTH];
            wire [SCALE_FACTOR_WIDTH-1:0]row_scale_w = row_block_r[SCALE_LSB +: SCALE_FACTOR_WIDTH];
            wire [SCALE_FACTOR_WIDTH-1:0]col_scale_w = col_block_r[SCALE_LSB +: SCALE_FACTOR_WIDTH];
            wire [EXP_WIDTH-1:0]         row_exp_w   = row_block_r[EXP_LSB   +: EXP_WIDTH];
            wire [EXP_WIDTH-1:0]         col_exp_w   = col_block_r[EXP_LSB   +: EXP_WIDTH];

            // read pointers (0..ARRAY_SIZE-1), and rising-edge detect on demand
            reg [LOG2_ARRAY_SIZE-1:0]    row_ptr, col_ptr;
            reg                          set_i_ready_q;   // registered demand for edge detect
            reg                          row_done, col_done; // whole block consumed

            wire demand_rise = set_i_ready[li] & ~set_i_ready_q;

            // ----------------- write + read-pointer update -----------------
            always @(posedge clk or negedge rst_n) begin
                if (!rst_n) begin
                    row_block_r    <= '0; col_block_r  <= '0;
                    row_loaded    <= 1'b0; col_loaded <= 1'b0;
                    row_ptr       <= '0; col_ptr     <= '0;
                    row_done      <= 1'b0; col_done   <= 1'b0;
                    set_i_ready_q <= 1'b0;
                end else begin
                    set_i_ready_q <= set_i_ready[li];

                    // ---- write side: store the whole 32-bit block atomically,
                    //      reset that lane's read pointer ----
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

                    // ---- read side: advance one mantissa per demand rising edge ----
                    // row i and col i advance together off the same set_i_ready[i].
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

            // ----------------- current selected mantissa -> descaler -----------------
            wire [BLOCK_MANTISSA_WIDTH-1:0] row_mant_sel =
                    row_mants_w[row_ptr*BLOCK_MANTISSA_WIDTH +: BLOCK_MANTISSA_WIDTH];
            wire [BLOCK_MANTISSA_WIDTH-1:0] col_mant_sel =
                    col_mants_w[col_ptr*BLOCK_MANTISSA_WIDTH +: BLOCK_MANTISSA_WIDTH];

            descaling #(
                .MANTISSA_WIDTH(BLOCK_MANTISSA_WIDTH),
                .SCALE_FACTOR_WIDTH(SCALE_FACTOR_WIDTH)
            ) u_descale_row (
                .mant(row_mant_sel),
                .inp_scale_factor(row_scale_w),
                .inp_pe(mantissa_row_0[li])
            );

            descaling #(
                .MANTISSA_WIDTH(BLOCK_MANTISSA_WIDTH),
                .SCALE_FACTOR_WIDTH(SCALE_FACTOR_WIDTH)
            ) u_descale_col (
                .mant(col_mant_sel),
                .inp_scale_factor(col_scale_w),
                .inp_pe(mantissa_col_0[li])
            );

            // ----------------- edge outputs -----------------
            assign exponent_row_0[li] = row_exp_w;
            assign exponent_col_0[li] = col_exp_w;

            // valid while the lane has a loaded block that isn't fully consumed
            assign valid_in_row_0[li] = row_loaded && !row_done;
            assign valid_in_col_0[li] = col_loaded && !col_done;

            // last asserted on the final mantissa of the block (pointer at ARRAY_SIZE-1)
            assign last_in_row_0[li] = valid_in_row_0[li] &&
                                        (row_ptr == LOG2_ARRAY_SIZE'(ARRAY_SIZE-1));
            assign last_in_col_0[li] = valid_in_col_0[li] &&
                                        (col_ptr == LOG2_ARRAY_SIZE'(ARRAY_SIZE-1));

            assign lane_primed[li] = row_loaded && col_loaded;

            assign dbg_row_ptr[li] = row_ptr;
            assign dbg_col_ptr[li] = col_ptr;
        end
    endgenerate

    // aggregate primed flag
    reg  all_primed;
    integer k;
    always @(*) begin
        all_primed = 1'b1;
        for (k = 0; k < ARRAY_SIZE; k = k + 1)
            if (!lane_primed[k]) all_primed = 1'b0;
    end
    assign bank_primed = all_primed;

endmodule
