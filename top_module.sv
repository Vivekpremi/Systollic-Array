`timescale 1ns/1ps

//=============================================================================
// top_module.sv  --  the accelerator core: one 4x4 x 4x4 tile, blocks in, blocks out
//
// WHAT IT DOES
//   C = A x B for one tile (ARRAY_SIZE x ARRAY_SIZE), in the block format:
//     input : ARRAY_SIZE row blocks (row i of A) + ARRAY_SIZE column blocks
//             (column j of B), written with row_we / col_we
//     output: ARRAY_SIZE output blocks, block j = column j of C, one per
//             block_ready pulse, in the SAME format as the inputs (so a
//             result can be fed back in as an operand)
//
// DATA PATH
//   edge_reg_bank -> systolic_array -> iface -> format_convertor
//     edge_reg_bank   : stores the blocks, feeds one element per product into
//                       the array edges (L -> L << scale)
//     systolic_array  : 4x4 PEs, PE[i][j] accumulates C[i][j]
//     iface           : sends each finished column of results, 2 per clock
//     format_convertor: turns a column of results into one output block
//
// BLOCK FORMAT (BLOCK_WIDTH bits, 48 at the defaults)
//   value = (-1)^sign * 2^(E + L * 2^scale / 2^FRAC_WIDTH), or 0 if zero = 1
//   L of each element [MANT_BUS_WIDTH-1:0] (element 0 lowest), then
//   ARRAY_SIZE zero flags, ARRAY_SIZE sign bits, scale, E (signed), unused
//   top bits = 0. Defaults: L [23:0], zero [27:24], sign [31:28],
//   scale [35:32], E [43:36].
//
// USE
//   reset -> write all 2*ARRAY_SIZE blocks in the SAME clock -> wait for
//   ARRAY_SIZE block_ready pulses (columns 0, 1, 2, 3) -> reset before the
//   next tile (the PE accumulators clear only on reset).
//
// FLAGS
//   exp_overflow / exp_underflow : with each output block: its E did not fit
//                                  and was clamped
//   pe_overflow[i*ARRAY_SIZE+j]  : PE[i][j] lost top bits (sticky until reset)
//   acc_overflow                 : OR of all pe_overflow bits
//
// LIMITATIONS
//   ARRAY_SIZE must be 4 for now (iface pairs results; format_convertor only
//   divides by 1..4), FRAC_WIDTH must be 8 (CORDIC table). Number range
//   limits: see docs/numeric_range.md.
//=============================================================================
module top_module #(
    parameter MANTISSA_WIDTH = 16,   // bits of the PE log operand (INT.FRAC, 8.8 by default)
    parameter FRAC_WIDTH = 8,        // fraction bits of every log value (must be 8)
    parameter EXP_WIDTH = 8,         // bits of a block exponent E (signed)
    parameter DATA_WIDTH = 32,       // PE product width; accumulator DATA_WIDTH+1
    parameter ARRAY_SIZE = 4,        // PEs per side = elements per block (must be 4 for now)
    parameter LOG2_ARRAY_SIZE = $clog2(ARRAY_SIZE),
    // L: residual log per element, signed; the same width for input and output blocks
    parameter BLOCK_MANTISSA_WIDTH = 6,
    parameter SCALE_FACTOR_WIDTH = 4,   // bits of the block scale
    // ---- block geometry (identical in edge_reg_bank, format_convertor, testbenches) ----
    parameter MANT_BUS_WIDTH = ARRAY_SIZE*BLOCK_MANTISSA_WIDTH,   // all L fields
    parameter BLOCK_BITS     = MANT_BUS_WIDTH + 2*ARRAY_SIZE
                               + SCALE_FACTOR_WIDTH + EXP_WIDTH,  // bits used
    parameter BLOCK_WIDTH    = ((BLOCK_BITS + 7)/8)*8             // rounded up to whole bytes
)(
    input wire clk,
    input wire rst_n,    // async, active low; also clears the tile (one tile per reset)

    // ---- input blocks: write all lanes in the same clock ----
    input  wire                    row_we    [ARRAY_SIZE-1:0],   // store row_block[i]
    input  wire                    col_we    [ARRAY_SIZE-1:0],   // store col_block[i]
    input  wire [BLOCK_WIDTH-1:0]  row_block [ARRAY_SIZE-1:0],   // row i of A
    input  wire [BLOCK_WIDTH-1:0]  col_block [ARRAY_SIZE-1:0],   // column i of B

    // ---- debug / testbench outputs (not used by the AXI wrapper) ----
    output wire [ARRAY_SIZE-1:0] valid_out [ARRAY_SIZE-1:0],   // valid_out[i][j]: PE[i][j] result final
    output wire signed [DATA_WIDTH:0] mantissa_out_1,          // iface -> FC: Res_out pair
    output wire signed [DATA_WIDTH:0] mantissa_out_2,
    output wire signed [EXP_WIDTH:0] exp_m_out_1,              // iface -> FC: exp_out pair
    output wire signed [EXP_WIDTH:0] exp_m_out_2,
    output wire valid_man_out,                                 // iface -> FC: a pair is valid
    output wire set_i_ready[ARRAY_SIZE-1:0],                   // array -> bank pacing pulses
    output wire signed [DATA_WIDTH:0] Res_out [ARRAY_SIZE-1:0][ARRAY_SIZE-1:0],   // every PE accumulator
    output wire mantissas_sent_out,                            // iface -> FC: last pair of a column

    // ---- output blocks ----
    output wire block_ready,                                   // 1-clock pulse: a new output block
    output wire [SCALE_FACTOR_WIDTH-1:0] scale_factor,         // the block's fields, separately
    output wire signed [EXP_WIDTH-1:0] max_exp,                //   (E)
    output wire [MANT_BUS_WIDTH-1:0] mantissa_out,             //   (all L)
    output wire [BLOCK_WIDTH-1:0] block_out,                   // the packed output block (input format)
    output wire [ARRAY_SIZE-1:0] sign_out,
    output wire [ARRAY_SIZE-1:0] zero_out,
    output wire exp_overflow,     // this block's E was too large and clamped
    output wire exp_underflow,    // this block's E was too small and clamped
    output wire [ARRAY_SIZE*ARRAY_SIZE-1:0] pe_overflow,   // PE[i][j] overflow (sticky)
    output wire acc_overflow,     // OR of pe_overflow

    output wire signed [EXP_WIDTH:0] exp_out [ARRAY_SIZE-1:0][ARRAY_SIZE-1:0],   // every PE's Er + Ec (debug)

    output wire bank_primed       // all lanes of edge_reg_bank have a block
);

// ---- edge_reg_bank -> systolic_array: the current element of each lane ----
wire signed [MANTISSA_WIDTH-1:0] mantissa_col_0 [ARRAY_SIZE-1:0];
wire signed [MANTISSA_WIDTH-1:0] mantissa_row_0 [ARRAY_SIZE-1:0];
wire signed [EXP_WIDTH-1:0]      exponent_col_0 [ARRAY_SIZE-1:0];
wire signed [EXP_WIDTH-1:0]      exponent_row_0 [ARRAY_SIZE-1:0];
wire                      sign_col_0     [ARRAY_SIZE-1:0];
wire                      sign_row_0     [ARRAY_SIZE-1:0];
wire                      zero_col_0     [ARRAY_SIZE-1:0];
wire                      zero_row_0     [ARRAY_SIZE-1:0];
wire                      valid_in_row_0 [ARRAY_SIZE-1:0];
wire                      valid_in_col_0 [ARRAY_SIZE-1:0];
wire                      last_in_row_0  [ARRAY_SIZE-1:0];
wire                      last_in_col_0  [ARRAY_SIZE-1:0];

edge_reg_bank #(
    .ARRAY_SIZE(ARRAY_SIZE), .MANTISSA_WIDTH(MANTISSA_WIDTH), .EXP_WIDTH(EXP_WIDTH),
    .BLOCK_MANTISSA_WIDTH(BLOCK_MANTISSA_WIDTH), .SCALE_FACTOR_WIDTH(SCALE_FACTOR_WIDTH)
)
bank (
    .clk(clk),
    .rst_n(rst_n),
    .row_we(row_we),
    .col_we(col_we),
    .row_block(row_block),
    .col_block(col_block),
    .set_i_ready(set_i_ready),
    .mantissa_row_0(mantissa_row_0),
    .mantissa_col_0(mantissa_col_0),
    .exponent_row_0(exponent_row_0),
    .exponent_col_0(exponent_col_0),
    .sign_row_0(sign_row_0),
    .sign_col_0(sign_col_0),
    .zero_row_0(zero_row_0),
    .zero_col_0(zero_col_0),
    .valid_in_row_0(valid_in_row_0),
    .valid_in_col_0(valid_in_col_0),
    .last_in_row_0(last_in_row_0),
    .last_in_col_0(last_in_col_0),
    .bank_primed(bank_primed)
);

systolic_array #(
    .DATA_WIDTH(DATA_WIDTH), .EXP_WIDTH(EXP_WIDTH),
    .MANTISSA_WIDTH(MANTISSA_WIDTH), .FRAC_WIDTH(FRAC_WIDTH),
    .ARRAY_SIZE(ARRAY_SIZE)
)
SA (
    .clk(clk),
    .rst_n(rst_n),
    .mantissa_col_0(mantissa_col_0),
    .mantissa_row_0(mantissa_row_0),
    .exponent_col_0(exponent_col_0),
    .exponent_row_0(exponent_row_0),
    .sign_col_0(sign_col_0),
    .sign_row_0(sign_row_0),
    .zero_col_0(zero_col_0),
    .zero_row_0(zero_row_0),
    .valid_in_row_0(valid_in_row_0),
    .valid_in_col_0(valid_in_col_0),
    .exp_out(exp_out),
    .Res_out(Res_out),
    .valid_out(valid_out),
    .last_in_row_0(last_in_row_0),
    .last_in_col_0(last_in_col_0),
    .set_i_ready(set_i_ready),
    .pe_overflow(pe_overflow)
);

assign acc_overflow = |pe_overflow;

// ---------------------------------------------------------------------------
// When iface may start the next column (start_op)
//   block_cnt : output blocks produced so far in this tile (0..ARRAY_SIZE-1)
//   Before PE[0][1] has finished (valid_out[0][1] = 0): start when PE[0][0]
//   has finished, i.e. column 0 can begin.
//   After that: start each next column when the previous column's block is
//   out (block_ready). block_cnt != ARRAY_SIZE-1 stops a start after the
//   last column's block.
//   (iface itself then waits until the whole column is finished.)
// ---------------------------------------------------------------------------
reg [LOG2_ARRAY_SIZE-1:0] block_cnt;
wire start_op = ((valid_out[0][1]) ? block_ready : valid_out[0][0]) && (block_cnt != LOG2_ARRAY_SIZE'(ARRAY_SIZE-1));

always @(posedge clk or negedge rst_n) begin
    if (!rst_n) begin
        block_cnt <= 0;
    end
    else begin
        if(block_ready) begin
            block_cnt <= (block_cnt == LOG2_ARRAY_SIZE'(ARRAY_SIZE-1)) ? 0 : block_cnt + 1;
        end
    end
end

iface #(
    .DATA_WIDTH(DATA_WIDTH), .EXP_WIDTH(EXP_WIDTH), .ARRAY_SIZE(ARRAY_SIZE)
) iface(
    .clk(clk),
    .rst_n(rst_n),
    .start(start_op),
    .valid_column (valid_out),
    .mantissa (Res_out),
    .exponent (exp_out),

    .mantissa_out_1 (mantissa_out_1),
    .mantissa_out_2 (mantissa_out_2),
    .exp_m_out_1 (exp_m_out_1),
    .exp_m_out_2 (exp_m_out_2),
    .valid_man_out (valid_man_out),
    .mantissas_sent_out (mantissas_sent_out)
);

format_convertor #(
    .DATA_WIDTH(DATA_WIDTH), .FRAC_WIDTH(FRAC_WIDTH), .EXP_WIDTH(EXP_WIDTH), .ARRAY_SIZE(ARRAY_SIZE),
    .BLOCK_MANTISSA_WIDTH(BLOCK_MANTISSA_WIDTH), .SCALE_FACTOR_WIDTH(SCALE_FACTOR_WIDTH)
) FC(
    .clk(clk),
    .rst_n(rst_n),
    .mant0(mantissa_out_1),
    .mant1(mantissa_out_2),
    .exp_m_1(exp_m_out_1),
    .exp_m_2(exp_m_out_2),
    .mantissa_valid(valid_man_out),
    .all_mantissas_sent(mantissas_sent_out),

    .block_out(block_out),
    .max_exp(max_exp),
    .scale_factor(scale_factor),
    .mantissa_out(mantissa_out),
    .sign_out(sign_out),
    .zero_out(zero_out),
    .block_ready(block_ready),
    .exp_overflow(exp_overflow),
    .exp_underflow(exp_underflow)
);

endmodule

