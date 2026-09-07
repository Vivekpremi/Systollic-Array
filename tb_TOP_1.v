
`timescale 1ns/1ps
//`include "PE.v"
module tb_TOP_1;

    parameter DATA_WIDTH = 32;
    parameter EXP_WIDTH = 8;
    parameter MANTISSA_WIDTH = 16;


    reg clk;
    reg rst_n;
    reg [MANTISSA_WIDTH-1:0] mantissa_col_0 [15:0];

    reg [MANTISSA_WIDTH-1:0] mantissa_row_0 [15:0];
    reg [EXP_WIDTH-1:0] exponent_col_0 [15:0];
    reg [EXP_WIDTH-1:0] exponent_row_0 [15:0];
    reg valid_in_row_0[15:0];
    reg valid_in_col_0[15:0];
    reg last_in_row_0[15:0];
    reg last_in_col_0[15:0];                 
    wire [15:0] valid_out [15:0];
    wire set_i_ready[15:0];

    wire [DATA_WIDTH-1:0] mantissa_out_1;
    wire [DATA_WIDTH-1:0] mantissa_out_2;
    wire [7:0] exp_m_out_1;
    wire [7:0] exp_m_out_2;
    wire [7:0] exponent_out_1;
    wire [7:0] exponent_out_2;
    wire valid_exp_out;
    wire valid_man_out;
    wire [DATA_WIDTH-1:0] Res_out [15:0][15:0];
    wire last_exp_sent;
    wire mantissas_sent_out;
    wire block_ready;
    wire [4:0] scale_factor;
    wire [7:0] max_exp;
    wire [16*5 -1 :0] mantissa_out;
    wire max_exp_calculated;
top_module #(
    .MANTISSA_WIDTH(16),
    .EXP_WIDTH(8),
    .DATA_WIDTH(32)
)
uut (
    .clk(clk),
    .rst_n(rst_n),
    .mantissa_col_0(mantissa_col_0),
    .mantissa_row_0(mantissa_row_0),
    .exponent_col_0(exponent_col_0),
    .exponent_row_0(exponent_row_0),
    .valid_in_row_0(valid_in_row_0),
    .valid_in_col_0(valid_in_col_0),
    .last_in_row_0(last_in_row_0),
    .last_in_col_0(last_in_col_0),
    .max_exp_calculated(max_exp_calculated),

    .valid_out(valid_out),
    .valid_exp_out(valid_exp_out),
    .exponent_out_1(exponent_out_1),
    .exponent_out_2(exponent_out_2),
    .mantissa_out_1(mantissa_out_1),
    .mantissa_out_2(mantissa_out_2),
    .exp_m_out_1(exp_m_out_1),
    .exp_m_out_2(exp_m_out_2),
    .valid_man_out(valid_man_out),
    .set_i_ready(set_i_ready),
    .iface_state_out(iface_state_out),
    .Res_out(Res_out),
    .last_exp_sent(last_exp_sent),
    .mantissas_sent_out(mantissas_sent_out),
    .block_ready(block_ready),
    .scale_factor(scale_factor),
    .max_exp(max_exp),
    .mantissa_out(mantissa_out)
);


wire [1:0]iface_state_out;
// clock generation
initial begin
    clk = 0;
    forever #5 clk = ~clk; // 100MHz clock
end
// reset generation
initial begin
    rst_n = 0;
    #20 rst_n = 1; // Release reset after 20ns
end
// matrix A and B inputs
reg [15:0] A [0:15][0:15];
reg [15:0] B [0:15][0:15];

initial begin
    A[0][0]  = 16'h0001; A[0][1]  = 16'h0002; A[0][2]  = 16'h0003; A[0][3]  = 16'h0004; A[0][4]  = 16'h0005; A[0][5]  = 16'h0006; A[0][6]  = 16'h0007; A[0][7]  = 16'h0008; A[0][8]  = 16'h0009; A[0][9]  = 16'h000A; A[0][10]  = 16'h000B; A[0][11]  = 16'h000C; A[0][12]  = 16'h000D; A[0][13]  = 16'h000E; A[0][14]  = 16'h000F; A[0][15]  = 16'h0010;
    A[1][0]  = 16'h0001; A[1][1]  = 16'h0002; A[1][2]  = 16'h0003; A[1][3]  = 16'h0004; A[1][4]  = 16'h0005; A[1][5]  = 16'h0006; A[1][6]  = 16'h0007; A[1][7]  = 16'h0008; A[1][8]  = 16'h0009; A[1][9]  = 16'h000A; A[1][10]  = 16'h000B; A[1][11]  = 16'h000C; A[1][12]  = 16'h000D; A[1][13]  = 16'h000E; A[1][14]  = 16'h000F; A[1][15]  = 16'h0010;
    A[2][0]  = 16'h0001; A[2][1]  = 16'h0002; A[2][2]  = 16'h0003; A[2][3]  = 16'h0004; A[2][4]  = 16'h0005; A[2][5]  = 16'h0006; A[2][6]  = 16'h0007; A[2][7]  = 16'h0008; A[2][8]  = 16'h0009; A[2][9]  = 16'h000A; A[2][10]  = 16'h000B; A[2][11]  = 16'h000C; A[2][12]  = 16'h000D; A[2][13]  = 16'h000E; A[2][14]  = 16'h000F; A[2][15]  = 16'h0010;
    A[3][0]  = 16'h0001; A[3][1]  = 16'h0002; A[3][2]  = 16'h0003; A[3][3]  = 16'h0004; A[3][4]  = 16'h0005; A[3][5]  = 16'h0006; A[3][6]  = 16'h0007; A[3][7]  = 16'h0008; A[3][8]  = 16'h0009; A[3][9]  = 16'h000A; A[3][10]  = 16'h000B; A[3][11]  = 16'h000C; A[3][12]  = 16'h000D; A[3][13]  = 16'h000E; A[3][14]  = 16'h000F; A[3][15]  = 16'h0010;
    A[4][0]  = 16'h0001; A[4][1]  = 16'h0002; A[4][2]  = 16'h0003; A[4][3]  = 16'h0004; A[4][4]  = 16'h0005; A[4][5]  = 16'h0006; A[4][6]  = 16'h0007; A[4][7]  = 16'h0008; A[4][8]  = 16'h0009; A[4][9]  = 16'h000A; A[4][10]  = 16'h000B; A[4][11]  = 16'h000C; A[4][12]  = 16'h000D; A[4][13]  = 16'h000E; A[4][14]  = 16'h000F; A[4][15]  = 16'h0010;
    A[5][0]  = 16'h0001; A[5][1]  = 16'h0002; A[5][2]  = 16'h0003; A[5][3]  = 16'h0004; A[5][4]  = 16'h0005; A[5][5]  = 16'h0006; A[5][6]  = 16'h0007; A[5][7]  = 16'h0008; A[5][8]  = 16'h0009; A[5][9]  = 16'h000A; A[5][10]  = 16'h000B; A[5][11]  = 16'h000C; A[5][12]  = 16'h000D; A[5][13]  = 16'h000E; A[5][14]  = 16'h000F; A[5][15]  = 16'h0010;
    A[6][0]  = 16'h0001; A[6][1]  = 16'h0002; A[6][2]  = 16'h0003; A[6][3]  = 16'h0004; A[6][4]  = 16'h0005; A[6][5]  = 16'h0006; A[6][6]  = 16'h0007; A[6][7]  = 16'h0008; A[6][8]  = 16'h0009; A[6][9]  = 16'h000A; A[6][10]  = 16'h000B; A[6][11]  = 16'h000C; A[6][12]  = 16'h000D; A[6][13]  = 16'h000E; A[6][14]  = 16'h000F; A[6][15]  = 16'h0010;
    A[7][0]  = 16'h0001; A[7][1]  = 16'h0002; A[7][2]  = 16'h0003; A[7][3]  = 16'h0004; A[7][4]  = 16'h0005; A[7][5]  = 16'h0006; A[7][6]  = 16'h0007; A[7][7]  = 16'h0008; A[7][8]  = 16'h0009; A[7][9]  = 16'h000A; A[7][10]  = 16'h000B; A[7][11]  = 16'h000C; A[7][12]  = 16'h000D; A[7][13]  = 16'h000E; A[7][14]  = 16'h000F; A[7][15]  = 16'h0010;
    A[8][0]  = 16'h0001; A[8][1]  = 16'h0002; A[8][2]  = 16'h0003; A[8][3]  = 16'h0004; A[8][4]  = 16'h0005; A[8][5]  = 16'h0006; A[8][6]  = 16'h0007; A[8][7]  = 16'h0008; A[8][8]  = 16'h0009; A[8][9]  = 16'h000A; A[8][10]  = 16'h000B; A[8][11]  = 16'h000C; A[8][12]  = 16'h000D; A[8][13]  = 16'h000E; A[8][14]  = 16'h000F; A[8][15]  = 16'h0010;
    A[9][0]  = 16'h0001; A[9][1]  = 16'h0002; A[9][2]  = 16'h0003; A[9][3]  = 16'h0004; A[9][4]  = 16'h0005; A[9][5]  = 16'h0006; A[9][6]  = 16'h0007; A[9][7]  = 16'h0008; A[9][8]  = 16'h0009; A[9][9]  = 16'h000A; A[9][10]  = 16'h000B; A[9][11]  = 16'h000C; A[9][12]  = 16'h000D; A[9][13]  = 16'h000E; A[9][14]  = 16'h000F; A[9][15]  = 16'h0010;
    A[10][0] = 16'h0001; A[10][1] = 16'h0002; A[10][2] = 16'h0003; A[10][3] = 16'h0004; A[10][4] = 16'h0005; A[10][5] = 16'h0006; A[10][6] = 16'h0007; A[10][7] = 16'h0008; A[10][8] = 16'h0009; A[10][9] = 16'h000A; A[10][10] = 16'h000B; A[10][11] = 16'h000C; A[10][12] = 16'h000D; A[10][13] = 16'h000E; A[10][14] = 16'h000F; A[10][15] = 16'h0010;
    A[11][0] = 16'h0001; A[11][1] = 16'h0002; A[11][2] = 16'h0003; A[11][3] = 16'h0004; A[11][4] = 16'h0005; A[11][5] = 16'h0006; A[11][6] = 16'h0007; A[11][7] = 16'h0008; A[11][8] = 16'h0009; A[11][9] = 16'h000A; A[11][10] = 16'h000B; A[11][11] = 16'h000C; A[11][12] = 16'h000D; A[11][13] = 16'h000E; A[11][14] = 16'h000F; A[11][15] = 16'h0010;
    A[12][0] = 16'h0001; A[12][1] = 16'h0002; A[12][2] = 16'h0003; A[12][3] = 16'h0004; A[12][4] = 16'h0005; A[12][5] = 16'h0006; A[12][6] = 16'h0007; A[12][7] = 16'h0008; A[12][8] = 16'h0009; A[12][9] = 16'h000A; A[12][10] = 16'h000B; A[12][11] = 16'h000C; A[12][12] = 16'h000D; A[12][13] = 16'h000E; A[12][14] = 16'h000F; A[12][15] = 16'h0010;
    A[13][0] = 16'h0001; A[13][1] = 16'h0002; A[13][2] = 16'h0003; A[13][3] = 16'h0004; A[13][4] = 16'h0005; A[13][5] = 16'h0006; A[13][6] = 16'h0007; A[13][7] = 16'h0008; A[13][8] = 16'h0009; A[13][9] = 16'h000A; A[13][10] = 16'h000B; A[13][11] = 16'h000C; A[13][12] = 16'h000D; A[13][13] = 16'h000E; A[13][14] = 16'h000F; A[13][15] = 16'h0010;
    A[14][0] = 16'h0001; A[14][1] = 16'h0002; A[14][2] = 16'h0003; A[14][3] = 16'h0004; A[14][4] = 16'h0005; A[14][5] = 16'h0006; A[14][6] = 16'h0007; A[14][7] = 16'h0008; A[14][8] = 16'h0009; A[14][9] = 16'h000A; A[14][10] = 16'h000B; A[14][11] = 16'h000C; A[14][12] = 16'h000D; A[14][13] = 16'h000E; A[14][14] = 16'h000F; A[14][15] = 16'h0010;
    A[15][0] = 16'h0001; A[15][1] = 16'h0002; A[15][2] = 16'h0003; A[15][3] = 16'h0004; A[15][4] = 16'h0005; A[15][5] = 16'h0006; A[15][6] = 16'h0007; A[15][7] = 16'h0008; A[15][8] = 16'h0009; A[15][9] = 16'h000A; A[15][10] = 16'h000B; A[15][11] = 16'h000C; A[15][12] = 16'h000D; A[15][13] = 16'h000E; A[15][14] = 16'h000F; A[15][15] = 16'h0010;
    
    B[0][0]  = 16'h0001; B[0][1]  = 16'h0002; B[0][2]  = 16'h0003; B[0][3]  = 16'h0004; B[0][4]  = 16'h0005; B[0][5]  = 16'h0006; B[0][6]  = 16'h0007; B[0][7]  = 16'h0008; B[0][8]  = 16'h0009; B[0][9]  = 16'h000A; B[0][10]  = 16'h000B; B[0][11]  = 16'h000C; B[0][12]  = 16'h000D; B[0][13]  = 16'h000E; B[0][14]  = 16'h000F; B[0][15]  = 16'h0010;
    B[1][0]  = 16'h0001; B[1][1]  = 16'h0002; B[1][2]  = 16'h0003; B[1][3]  = 16'h0004; B[1][4]  = 16'h0005; B[1][5]  = 16'h0006; B[1][6]  = 16'h0007; B[1][7]  = 16'h0008; B[1][8]  = 16'h0009; B[1][9]  = 16'h000A; B[1][10]  = 16'h000B; B[1][11]  = 16'h000C; B[1][12]  = 16'h000D; B[1][13]  = 16'h000E; B[1][14]  = 16'h000F; B[1][15]  = 16'h0010;
    B[2][0]  = 16'h0001; B[2][1]  = 16'h0002; B[2][2]  = 16'h0003; B[2][3]  = 16'h0004; B[2][4]  = 16'h0005; B[2][5]  = 16'h0006; B[2][6]  = 16'h0007; B[2][7]  = 16'h0008; B[2][8]  = 16'h0009; B[2][9]  = 16'h000A; B[2][10]  = 16'h000B; B[2][11]  = 16'h000C; B[2][12]  = 16'h000D; B[2][13]  = 16'h000E; B[2][14]  = 16'h000F; B[2][15]  = 16'h0010;
    B[3][0]  = 16'h0001; B[3][1]  = 16'h0002; B[3][2]  = 16'h0003; B[3][3]  = 16'h0004; B[3][4]  = 16'h0005; B[3][5]  = 16'h0006; B[3][6]  = 16'h0007; B[3][7]  = 16'h0008; B[3][8]  = 16'h0009; B[3][9]  = 16'h000A; B[3][10]  = 16'h000B; B[3][11]  = 16'h000C; B[3][12]  = 16'h000D; B[3][13]  = 16'h000E; B[3][14]  = 16'h000F; B[3][15]  = 16'h0010;
    B[4][0]  = 16'h0001; B[4][1]  = 16'h0002; B[4][2]  = 16'h0003; B[4][3]  = 16'h0004; B[4][4]  = 16'h0005; B[4][5]  = 16'h0006; B[4][6]  = 16'h0007; B[4][7]  = 16'h0008; B[4][8]  = 16'h0009; B[4][9]  = 16'h000A; B[4][10]  = 16'h000B; B[4][11]  = 16'h000C; B[4][12]  = 16'h000D; B[4][13]  = 16'h000E; B[4][14]  = 16'h000F; B[4][15]  = 16'h0010;
    B[5][0]  = 16'h0001; B[5][1]  = 16'h0002; B[5][2]  = 16'h0003; B[5][3]  = 16'h0004; B[5][4]  = 16'h0005; B[5][5]  = 16'h0006; B[5][6]  = 16'h0007; B[5][7]  = 16'h0008; B[5][8]  = 16'h0009; B[5][9]  = 16'h000A; B[5][10]  = 16'h000B; B[5][11]  = 16'h000C; B[5][12]  = 16'h000D; B[5][13]  = 16'h000E; B[5][14]  = 16'h000F; B[5][15]  = 16'h0010;
    B[6][0]  = 16'h0001; B[6][1]  = 16'h0002; B[6][2]  = 16'h0003; B[6][3]  = 16'h0004; B[6][4]  = 16'h0005; B[6][5]  = 16'h0006; B[6][6]  = 16'h0007; B[6][7]  = 16'h0008; B[6][8]  = 16'h0009; B[6][9]  = 16'h000A; B[6][10]  = 16'h000B; B[6][11]  = 16'h000C; B[6][12]  = 16'h000D; B[6][13]  = 16'h000E; B[6][14]  = 16'h000F; B[6][15]  = 16'h0010;
    B[7][0]  = 16'h0001; B[7][1]  = 16'h0002; B[7][2]  = 16'h0003; B[7][3]  = 16'h0004; B[7][4]  = 16'h0005; B[7][5]  = 16'h0006; B[7][6]  = 16'h0007; B[7][7]  = 16'h0008; B[7][8]  = 16'h0009; B[7][9]  = 16'h000A; B[7][10]  = 16'h000B; B[7][11]  = 16'h000C; B[7][12]  = 16'h000D; B[7][13]  = 16'h000E; B[7][14]  = 16'h000F; B[7][15]  = 16'h0010;
    B[8][0]  = 16'h0001; B[8][1]  = 16'h0002; B[8][2]  = 16'h0003; B[8][3]  = 16'h0004; B[8][4]  = 16'h0005; B[8][5]  = 16'h0006; B[8][6]  = 16'h0007; B[8][7]  = 16'h0008; B[8][8]  = 16'h0009; B[8][9]  = 16'h000A; B[8][10]  = 16'h000B; B[8][11]  = 16'h000C; B[8][12]  = 16'h000D; B[8][13]  = 16'h000E; B[8][14]  = 16'h000F; B[8][15]  = 16'h0010;
    B[9][0]  = 16'h0001; B[9][1]  = 16'h0002; B[9][2]  = 16'h0003; B[9][3]  = 16'h0004; B[9][4]  = 16'h0005; B[9][5]  = 16'h0006; B[9][6]  = 16'h0007; B[9][7]  = 16'h0008; B[9][8]  = 16'h0009; B[9][9]  = 16'h000A; B[9][10]  = 16'h000B; B[9][11]  = 16'h000C; B[9][12]  = 16'h000D; B[9][13]  = 16'h000E; B[9][14]  = 16'h000F; B[9][15]  = 16'h0010;
    B[10][0] = 16'h0001; B[10][1] = 16'h0002; B[10][2] = 16'h0003; B[10][3] = 16'h0004; B[10][4] = 16'h0005; B[10][5] = 16'h0006; B[10][6] = 16'h0007; B[10][7] = 16'h0008; B[10][8] = 16'h0009; B[10][9] = 16'h000A; B[10][10] = 16'h000B; B[10][11] = 16'h000C; B[10][12] = 16'h000D; B[10][13] = 16'h000E; B[10][14] = 16'h000F; B[10][15] = 16'h0010;
    B[11][0] = 16'h0001; B[11][1] = 16'h0002; B[11][2] = 16'h0003; B[11][3] = 16'h0004; B[11][4] = 16'h0005; B[11][5] = 16'h0006; B[11][6] = 16'h0007; B[11][7] = 16'h0008; B[11][8] = 16'h0009; B[11][9] = 16'h000A; B[11][10] = 16'h000B; B[11][11] = 16'h000C; B[11][12] = 16'h000D; B[11][13] = 16'h000E; B[11][14] = 16'h000F; B[11][15] = 16'h0010;
    B[12][0] = 16'h0001; B[12][1] = 16'h0002; B[12][2] = 16'h0003; B[12][3] = 16'h0004; B[12][4] = 16'h0005; B[12][5] = 16'h0006; B[12][6] = 16'h0007; B[12][7] = 16'h0008; B[12][8] = 16'h0009; B[12][9] = 16'h000A; B[12][10] = 16'h000B; B[12][11] = 16'h000C; B[12][12] = 16'h000D; B[12][13] = 16'h000E; B[12][14] = 16'h000F; B[12][15] = 16'h0010;
    B[13][0] = 16'h0001; B[13][1] = 16'h0002; B[13][2] = 16'h0003; B[13][3] = 16'h0004; B[13][4] = 16'h0005; B[13][5] = 16'h0006; B[13][6] = 16'h0007; B[13][7] = 16'h0008; B[13][8] = 16'h0009; B[13][9] = 16'h000A; B[13][10] = 16'h000B; B[13][11] = 16'h000C; B[13][12] = 16'h000D; B[13][13] = 16'h000E; B[13][14] = 16'h000F; B[13][15] = 16'h0010;
    B[14][0] = 16'h0001; B[14][1] = 16'h0002; B[14][2] = 16'h0003; B[14][3] = 16'h0004; B[14][4] = 16'h0005; B[14][5] = 16'h0006; B[14][6] = 16'h0007; B[14][7] = 16'h0008; B[14][8] = 16'h0009; B[14][9] = 16'h000A; B[14][10] = 16'h000B; B[14][11] = 16'h000C; B[14][12] = 16'h000D; B[14][13] = 16'h000E; B[14][14] = 16'h000F; B[14][15] = 16'h0010;
    B[15][0] = 16'h0001; B[15][1] = 16'h0002; B[15][2] = 16'h0003; B[15][3] = 16'h0004; B[15][4] = 16'h0005; B[15][5] = 16'h0006; B[15][6] = 16'h0007; B[15][7] = 16'h0008; B[15][8] = 16'h0009; B[15][9] = 16'h000A; B[15][10] = 16'h000B; B[15][11] = 16'h000C; B[15][12] = 16'h000D; B[15][13] = 16'h000E; B[15][14] = 16'h000F; B[15][15] = 16'h0010;

i = 0;
j = 0;


end
integer i, j;
reg [4:0] k;
reg [4:0] t[15:0]; // to keep track of which data to send for each column/row
//when set_i_ready is high for a column/row, we can send the next data for that column/row. We will use k to keep track of which data to send next for the current column/row. We will first fill the first column and row, then move to the next ones as set_i_ready signals are received.
//introduce bubbles
always @(posedge clk or negedge rst_n) begin
    if(!rst_n) begin
        k <= 0; // reset k on reset
    end else begin
    if(k == 16) begin
        k <= k; // reset k after filling one column/row
    end else if(set_i_ready[0]) begin
        k <= k + 1; // move to next data for the current column/row
    end
end
end
parameter [4:0] a [0:15] = '{
    5'b0000, 5'b0001, 5'b0010, 5'b0011,
    5'b0100, 5'b0101, 5'b0110, 5'b0111,
    5'b1000, 5'b1001, 5'b1010, 5'b1011,
    5'b1100, 5'b1101, 5'b1110, 5'b1111
};
genvar set_idx;

generate

    for(set_idx = 0; set_idx < 16; set_idx = set_idx + 1) begin : set_input_logic
        always @(posedge clk or negedge rst_n) begin
                if(!rst_n) begin
                    mantissa_col_0[set_idx] <= 0;
                    exponent_col_0[set_idx] <= 0; // example exponent
                    mantissa_row_0[set_idx] <= 0;
                    exponent_row_0[set_idx] <= 0; // example exponent
                    valid_in_row_0[set_idx] <= 0;    
                    valid_in_col_0[set_idx] <= 0;
                    t[set_idx] <= 0; // reset data index for this column/row
                end 
                else if(set_idx ==0 && (set_i_ready[0] || t[set_idx] == 0) && t[set_idx] < 16) begin
                        mantissa_col_0[set_idx] <= B[t[set_idx][3:0]][0]; // send first data for this column
                        exponent_col_0[set_idx] <= 8'h01; // example exponent
                        mantissa_row_0[set_idx] <= A[0][t[set_idx][3:0]]; // send first data for this row
                        exponent_row_0[set_idx] <= 8'h02; // example exponent
                        valid_in_row_0[set_idx] <= 1;    
                        valid_in_col_0[set_idx] <= 1;
                        t[set_idx] <= t[set_idx] + 1; // move to next data for this column/row
                        if(t[set_idx] == 15) begin
                            last_in_row_0[set_idx] <= 1;    
                            last_in_col_0[set_idx] <= 1;
                        end
                    end
                else if(set_i_ready[set_idx-1] && t[set_idx] < 16) begin
                        mantissa_col_0[set_idx] <= B[t[set_idx][3:0]][set_idx];
                        exponent_col_0[set_idx] <= 8'h01; // example exponent
                        mantissa_row_0[set_idx] <= A[set_idx][t[set_idx][3:0]];
                        exponent_row_0[set_idx] <= 8'h02; // example exponent
                        valid_in_row_0[set_idx] <= 1;    
                        valid_in_col_0[set_idx] <= 1;
                        t[set_idx] <= t[set_idx] + 1; // move to next data for this column/row

                        if(t[set_idx] == 15) begin
                            last_in_row_0[set_idx] <= 1;    
                            last_in_col_0[set_idx] <= 1;
                        end
                    end
            else begin
                        mantissa_col_0[set_idx] <= 0;
                        exponent_col_0[set_idx] <= 0; // example exponent
                        mantissa_row_0[set_idx] <= 0;
                        exponent_row_0[set_idx] <= 0; // example exponent
                        valid_in_row_0[set_idx] <= 0;    
                        valid_in_col_0[set_idx] <= 0;
                        last_in_row_0[set_idx] <= 0;
                        last_in_col_0[set_idx] <= 0;
                    end
                end
            end
endgenerate


//print results after some delay to allow processing
integer fd;

initial begin
    fd = $fopen("res.txt", "w");
    if (fd == 0) begin
        $display("File open failed");
        $finish;
    end
end

always @(negedge clk) begin
    if(k == 5'b10000 && t[15] == 5'b10000) $display("All data sent to PE.");

    #100000;
    $display("Results processed.");

    for(i = 0; i < 16; i = i + 1) begin
        for(j = 0; j < 16; j = j + 1) begin
            $fwrite(fd, "Res_out[%0d][%0d] = %h; ", i, j, Res_out[i][j]);
        end
        $fwrite(fd, "\n");
    end

    $fclose(fd);
    $finish;
end
// vcd
initial begin
    $dumpfile("TOP_1_tb.vcd");
    $dumpvars(0, tb_TOP);
end

endmodule
