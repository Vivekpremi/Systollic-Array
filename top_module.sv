`timescale 1ns/1ps

//=============================================================================
// top_module.sv  (edge_reg_bank integrated)
//
// edge_reg_bank is now the front door. Operands enter as one packed 32-bit
// BLOCK per lane (mantissas[19:0] + scale[23:20] + exp[31:24]) plus per-lane
// write-enables. The bank descales + demand-paces them into the systolic array
// (set_i_ready from the array closes the demand loop). iface + format_convertor
// are unchanged. The old already-descaled mantissa_*_0 inputs are removed.
//=============================================================================
module top_module #(
    parameter MANTISSA_WIDTH = 16,
    parameter FRAC_WIDTH = 8,
    parameter EXP_WIDTH = 8,
    parameter DATA_WIDTH = 32,
    parameter ARRAY_SIZE = 4,
    parameter LOG2_ARRAY_SIZE = $clog2(ARRAY_SIZE),
    parameter BLOCK_MANTISSA_WIDTH = 6,   // INPUT block: residual log L, two's complement
    parameter OUT_MANTISSA_WIDTH   = 5,   // OUTPUT block: format_convertor's mantissa (still unsigned)
    parameter SCALE_FACTOR_WIDTH = 4,
    parameter MANT_BUS_WIDTH = ARRAY_SIZE*BLOCK_MANTISSA_WIDTH,
    parameter BLOCK_BITS     = MANT_BUS_WIDTH + 2*ARRAY_SIZE
                               + SCALE_FACTOR_WIDTH + EXP_WIDTH,
    parameter BLOCK_WIDTH    = ((BLOCK_BITS + 7)/8)*8
)(
    input wire clk,
    input wire rst_n,

    // ---- operand input: one packed 32-bit BLOCK per lane + write-enable ----
    input  wire                    row_we    [ARRAY_SIZE-1:0],
    input  wire                    col_we    [ARRAY_SIZE-1:0],
    input  wire [BLOCK_WIDTH-1:0]  row_block [ARRAY_SIZE-1:0],
    input  wire [BLOCK_WIDTH-1:0]  col_block [ARRAY_SIZE-1:0],

    // ---- observability / outputs (same as before) ----
    output wire [ARRAY_SIZE-1:0] valid_out [ARRAY_SIZE-1:0],
    output wire valid_exp_out,
    output wire signed [EXP_WIDTH-1:0] exponent_out_1,
    output wire signed [EXP_WIDTH-1:0] exponent_out_2,
    output wire signed [DATA_WIDTH:0] mantissa_out_1,
    output wire signed [DATA_WIDTH:0] mantissa_out_2,
    output wire signed [EXP_WIDTH-1:0] exp_m_out_1,
    output wire signed [EXP_WIDTH-1:0] exp_m_out_2,
    output wire valid_man_out,
    output wire [1:0] iface_state_out,
    output wire set_i_ready[ARRAY_SIZE-1:0],
    output wire signed [DATA_WIDTH:0] Res_out [ARRAY_SIZE-1:0][ARRAY_SIZE-1:0],
    output wire last_exp_sent,
    output wire mantissas_sent_out,
    output wire block_ready,
    output wire [SCALE_FACTOR_WIDTH-1:0] scale_factor,
    output wire signed [EXP_WIDTH-1:0] max_exp,
    output wire [ARRAY_SIZE*OUT_MANTISSA_WIDTH -1 :0] mantissa_out,
    output wire max_exp_calculated,

    // ---- per-PE exponent, exposed so the testbench can CHECK the
    //      hardware exponent path instead of trusting its own copy ----
    output wire signed [EXP_WIDTH-1:0] exp_out [ARRAY_SIZE-1:0][ARRAY_SIZE-1:0],

    // ---- bank status ----
    output wire bank_primed
);

// ---- bank <-> array edge wires (descaled operands the bank produces) ----
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
wire [LOG2_ARRAY_SIZE-1:0] dbg_row_ptr [ARRAY_SIZE-1:0];
wire [LOG2_ARRAY_SIZE-1:0] dbg_col_ptr [ARRAY_SIZE-1:0];

wire demand_from_mem [ARRAY_SIZE-1:0];

// ---- edge register bank (front door) ----
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
    .bank_primed(bank_primed),
    .dbg_row_ptr(dbg_row_ptr),
    .dbg_col_ptr(dbg_col_ptr)
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
    .demand_from_mem(demand_from_mem)
);

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
    .DATA_WIDTH(DATA_WIDTH), .EXP_WIDTH(EXP_WIDTH), .ARRAY_SIZE(ARRAY_SIZE),
    .BLOCK_MANTISSA_WIDTH(OUT_MANTISSA_WIDTH), .SCALE_FACTOR_WIDTH(SCALE_FACTOR_WIDTH)
) iface(
    .clk(clk),
    .rst_n(rst_n),
    .start(start_op),
    .valid_column (valid_out),
    .mantissa (Res_out),
    .exponent (exp_out),
    .max_exp_calculated (max_exp_calculated),

    .valid_exp_out (valid_exp_out),
    .exp_out_1 (exponent_out_1),
    .exp_out_2 (exponent_out_2),
    .mantissa_out_1 (mantissa_out_1),
    .mantissa_out_2 (mantissa_out_2),
    .exp_m_out_1 (exp_m_out_1),
    .exp_m_out_2 (exp_m_out_2),
    .valid_man_out (valid_man_out),
    .state_out (iface_state_out),
    .last_exp_sent (last_exp_sent),
    .mantissas_sent_out (mantissas_sent_out)
);

format_convertor #(
    .DATA_WIDTH(DATA_WIDTH), .FRAC_WIDTH(FRAC_WIDTH), .EXP_WIDTH(EXP_WIDTH), .ARRAY_SIZE(ARRAY_SIZE),
    .BLOCK_MANTISSA_WIDTH(OUT_MANTISSA_WIDTH), .SCALE_FACTOR_WIDTH(SCALE_FACTOR_WIDTH)
) FC(
    .clk(clk),
    .rst_n(rst_n),
    .mant0(mantissa_out_1),
    .mant1(mantissa_out_2),
    .exp_m_1(exp_m_out_1),
    .exp_m_2(exp_m_out_2),
    .mantissa_valid(valid_man_out),
    .all_mantissas_sent(mantissas_sent_out),
    .exp0(exponent_out_1),
    .exp1(exponent_out_2),
    .exp_valid(valid_exp_out),
    .last_exp_sent(last_exp_sent),

    .max_exp(max_exp),
    .scale_factor(scale_factor),
    .mantissa_out(mantissa_out),
    .block_ready(block_ready),
    .max_exp_calculated(max_exp_calculated)
);

endmodule

