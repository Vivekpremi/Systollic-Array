`timescale 1ns/1ps
//`include "exponential_CORDIC.v"
//PE of block floating point Systolic array
module PE #(
    parameter EXP_WIDTH = 8
)(
    input wire clk,
    input wire rst_n,
    input wire valid_in,
    input wire [15:0] mantissa_col_i_j, 
    input  wire [EXP_WIDTH-1:0] exponent_col_j,
    input wire [15:0] mantissa_row_i_j,
    input wire [EXP_WIDTH-1:0] exponent_row_i,
    input wire last_row_in,
    input wire last_col_in,
    
    output wire [15:0] mantissa_row_i_j_out,
    output wire last_out,
    output wire [15:0] mantissa_col_i_j_out,
    output wire [EXP_WIDTH-1:0] exponent_col_j_out,
    output wire [EXP_WIDTH-1:0] exponent_row_i_out,
    output wire [31:0] Res_out,
    output wire valid_out_row_i,
    output wire valid_out_col_j,
    output wire demand_from_mem,
    output wire [EXP_WIDTH-1:0] exp_out
);

//exponent adder
wire [EXP_WIDTH-1:0] exp_sum;
assign exp_sum = exponent_col_j + exponent_row_i;

wire [EXP_WIDTH-1:0] exp_out_d;
assign exp_out_d =  exp_sum ;

//mantissa adder
wire [16:0] mantissa_sum;
assign mantissa_sum = mantissa_col_i_j + mantissa_row_i_j;

// Anti log of mantissa sum
wire anti_log_valid;
wire [10:0] anti_log_fractional_part;

//multiply with ln2 since e^xln2 = 2^x i.e. antilog base 2
// ln2 in form of shift and add is 0.69314718 = 1>>1 + 1>>3 + 1>>4 + 1>>7

wire [7:0] fractional_part_ln2 = (mantissa_sum[7:0] >> 1) + (mantissa_sum[7:0] >> 3) + (mantissa_sum[7:0] >> 4) + (mantissa_sum[7:0] >> 7);

exponential_CORDIC  #(
.Int_WIDTH (9), // Integer width
.Frac_WIDTH (8),  // Fractional width 
.DATA_WIDTH (17) // Total width
)
anti_log (
    .clk(clk),
    .rst(rst_n),
    .valid_in(valid_in),
    .x(fractional_part_ln2),
    .valid_out(anti_log_valid),
    .exp_result(anti_log_fractional_part)
);
//shifter to multiply 2^ii to 2^.ff
wire [31:0] multiplication_result;
assign multiplication_result = {21'b0,anti_log_fractional_part} << mantissa_sum[16:8];


//accumalator
reg [31:0] accum;
reg accum_valid;
reg last_out_q;

always @(posedge clk or negedge rst_n) begin
if(!rst_n) begin
    accum <= 0;
    accum_valid <= 0;
end 
else if (anti_log_valid) begin
    accum <= accum + multiplication_result;
    accum_valid <= 1;
    last_out_q <= last_row_in && last_col_in;
end else begin
    accum <= accum;
    accum_valid <= 0;
    last_out_q <= last_out_q ;
end
end



//pipeline registers
reg [7:0] exp_out_q;
reg [15:0] mantissa_col_i_j_out_q;
reg [15:0] mantissa_row_i_j_out_q;
reg [EXP_WIDTH-1:0] exponent_col_j_out_q;
reg [EXP_WIDTH-1:0] exponent_row_i_out_q;

always @(posedge clk or negedge rst_n) begin
if(!rst_n) begin
    exp_out_q <= 0;
    mantissa_col_i_j_out_q <= 0;
    mantissa_row_i_j_out_q <= 0;
    exponent_col_j_out_q <= 0;
    exponent_row_i_out_q <= 0;

end else if(valid_in) begin
    exp_out_q <= exp_out_d;
    mantissa_col_i_j_out_q <= mantissa_col_i_j;
    mantissa_row_i_j_out_q <= mantissa_row_i_j;
    exponent_col_j_out_q <= exponent_col_j;
    exponent_row_i_out_q <= exponent_row_i;
    end
    else begin
    exp_out_q <= exp_out_q;
    mantissa_col_i_j_out_q <= mantissa_col_i_j_out_q;
    mantissa_row_i_j_out_q <= mantissa_row_i_j_out_q;
    exponent_col_j_out_q <= exponent_col_j_out_q;
    exponent_row_i_out_q <= exponent_row_i_out_q;
    end
end

//output assignment
assign mantissa_row_i_j_out = mantissa_row_i_j_out_q;
assign mantissa_col_i_j_out = mantissa_col_i_j_out_q;
assign exp_out = exp_out_q;
assign exponent_col_j_out = exponent_col_j_out_q;
assign exponent_row_i_out = exponent_row_i_out_q;
assign Res_out = accum;
assign valid_out_row_i = accum_valid;
assign valid_out_col_j = accum_valid;
assign last_out = last_out_q; // Output is valid and final when accum is valid and it's the final PE
assign demand_from_mem = anti_log_valid; // Request new data when current data is valid

endmodule
