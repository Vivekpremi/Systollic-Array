
`timescale 1ns/1ps

module top_module #(
    parameter MANTISSA_WIDTH = 16,
    parameter EXP_WIDTH = 8,
    parameter DATA_WIDTH = 32,
    parameter ARRAY_SIZE = 4,
    parameter LOG2_ARRAY_SIZE = $clog2(ARRAY_SIZE),
    parameter BLOCK_MANTISSA_WIDTH = 5,
    parameter SCALE_FACTOR_WIDTH = 4
)(
    input wire clk,
    input wire rst_n,
    input wire [MANTISSA_WIDTH-1:0] mantissa_col_0 [ARRAY_SIZE-1:0],
    input wire [MANTISSA_WIDTH-1:0] mantissa_row_0 [ARRAY_SIZE-1:0],
    input wire [EXP_WIDTH-1:0] exponent_col_0 [ARRAY_SIZE-1:0],
    input wire [EXP_WIDTH-1:0] exponent_row_0 [ARRAY_SIZE-1:0],
    input wire valid_in_row_0[ARRAY_SIZE-1:0],
    input wire valid_in_col_0[ARRAY_SIZE-1:0],
    input wire last_in_row_0[ARRAY_SIZE-1:0],
    input wire last_in_col_0[ARRAY_SIZE-1:0],
    input wire start,
//    input wire max_exp_calculated,

    output wire [ARRAY_SIZE-1:0] valid_out [ARRAY_SIZE-1:0],
    output wire valid_exp_out,
    output wire [EXP_WIDTH-1:0] exponent_out_1,
    output wire [EXP_WIDTH-1:0] exponent_out_2,
    output wire [DATA_WIDTH-1:0] mantissa_out_1,
    output wire [DATA_WIDTH-1:0] mantissa_out_2,
    output wire [EXP_WIDTH-1:0] exp_m_out_1,
    output wire [EXP_WIDTH-1:0] exp_m_out_2,
    output wire valid_man_out,
    output wire [1:0] iface_state_out,
    output wire set_i_ready[ARRAY_SIZE-1:0],
    output wire [DATA_WIDTH-1:0] Res_out [ARRAY_SIZE-1:0][ARRAY_SIZE-1:0],
    output wire last_exp_sent,
    output wire mantissas_sent_out,
    output wire block_ready,
    output wire [SCALE_FACTOR_WIDTH-1:0] scale_factor,
    output wire [EXP_WIDTH-1:0] max_exp,
    output wire [ARRAY_SIZE*BLOCK_MANTISSA_WIDTH -1 :0] mantissa_out,
    output wire max_exp_calculated
);
// // Block Storing Refisters
// reg [31:0] block_col [15:0]; // 31:27(m1), 26:22(m2), 21:17(m3), 16:12(m4)
// reg [31:0] block_row [15:0];
// //Add Descalers
// genvar i;
// wire [4:0] mantissa_col_scaled [15:0];
// wire [4:0] mantissa_row_scaled [15:0];
// wire [3:0] scale_factor_col [15:0];
// wire [3:0] scale_factor_row [15:0];

// genvar j;

// descaling descaler(
//     .mant(),
//     .inp_scale_factor(),
//     .inp_pe()
// );


wire [EXP_WIDTH-1:0] exp_out [ARRAY_SIZE-1:0][ARRAY_SIZE-1:0];
systolic_array #(
    .DATA_WIDTH(DATA_WIDTH),
    .EXP_WIDTH(EXP_WIDTH),
    .MANTISSA_WIDTH(MANTISSA_WIDTH)
)
SA (
    .clk(clk),
    .rst_n(rst_n),
    .mantissa_col_0(mantissa_col_0),
    .mantissa_row_0(mantissa_row_0),
    .exponent_col_0(exponent_col_0),
    .exponent_row_0(exponent_row_0),
    .valid_in_row_0(valid_in_row_0),
    .valid_in_col_0(valid_in_col_0),
    .exp_out(exp_out),
    .Res_out(Res_out),
    .valid_out(valid_out),
    .last_in_row_0(last_in_row_0),
    .last_in_col_0(last_in_col_0),
    .set_i_ready(set_i_ready)
);

reg [LOG2_ARRAY_SIZE-1:0] block_cnt;
wire start_op = ((valid_out[0][1]) ? block_ready : valid_out[0][0]) && (block_cnt != LOG2_ARRAY_SIZE'(ARRAY_SIZE-1));

always @(posedge clk or negedge rst_n) begin
    if (!rst_n) begin
        block_cnt <= 0;
    end
    else begin
        if(block_ready) begin
            block_cnt <= (block_cnt == LOG2_ARRAY_SIZE'(ARRAY_SIZE-1)) ? 0 : block_cnt + 1; // Increment block count when block is ready, wrap around after ARRAY_SIZE-1
        end
    end
end
iface iface(
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



format_convertor FC(
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



