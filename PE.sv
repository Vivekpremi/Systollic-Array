`timescale 1ns/1ps
//`include "exponential_CORDIC.v"
//PE of block floating point Systolic array
module PE #(
    parameter EXP_WIDTH = 8
)(
    input wire clk,
    input wire rst_n,
    input wire valid_in,
    input wire signed [15:0] mantissa_col_i_j, 
    input wire signed [EXP_WIDTH-1:0] exponent_col_j,
    input wire signed [15:0] mantissa_row_i_j,
    input wire signed [EXP_WIDTH-1:0] exponent_row_i,
    input wire sign_col_i_j,              // VALUE sign of the column operand
    input wire sign_row_i_j,              // VALUE sign of the row operand
    input wire zero_col_i_j,              // column operand is exactly ZERO
    input wire zero_row_i_j,              // row operand is exactly ZERO
    input wire last_row_in,
    input wire last_col_in,
    
    output wire signed [15:0] mantissa_row_i_j_out,
    output wire last_out,
    output wire signed [15:0] mantissa_col_i_j_out,
    output wire signed [EXP_WIDTH-1:0] exponent_col_j_out,
    output wire signed [EXP_WIDTH-1:0] exponent_row_i_out,
    output wire sign_col_i_j_out,         // pass-through to next PE
    output wire sign_row_i_j_out,
    output wire zero_col_i_j_out,
    output wire zero_row_i_j_out,
    output wire signed [31:0] Res_out,
    output wire valid_out_row_i,
    output wire valid_out_col_j,
    output wire demand_from_mem,
    output wire signed [EXP_WIDTH-1:0] exp_out
);

//exponent adder (SIGNED: E can be negative for |v| < 1)
wire signed [EXP_WIDTH-1:0] exp_sum;
assign exp_sum = exponent_col_j + exponent_row_i;

wire signed [EXP_WIDTH-1:0] exp_out_d;
assign exp_out_d =  exp_sum ;

//mantissa adder -- SIGNED log add.  L_a + L_b = log2(|a|*|b|).
// Two's complement makes this a plain signed add, and the [16:8]/[7:0] split
// below then gives FLOOR(integer part) with an always-positive fraction, which
// is exactly what the antilog needs:  2^-0.7 = 2^-1 * 2^+0.3
wire signed [16:0] mantissa_sum;
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
// Register the INTEGER part of the log (mantissa_sum[16:8]) at the cycle the
// operand is latched by the CORDIC (valid_in). The CORDIC has multi-cycle
// latency and the operand bus is gated to 0 between operands, so reading
// mantissa_sum[16:8] combinationally at anti_log_valid time would see 0 and
// drop the exponent. Operations can't overlap (demand_from_mem = anti_log_valid
// gates the next operand on the current finishing), so a single held register
// stays aligned with the in-flight antilog.
reg signed [8:0] int_part_held;
// The VALUE sign of the in-flight product needs the SAME treatment: it is
// consumed at anti_log_valid time, ~10 cycles after the operand bus was gated
// to 0.  prod_sign = s_v(row) XOR s_v(col)  -- (-)(-)=+, (-)(+)=-.
reg sign_held;
// ZERO: if EITHER operand is zero the product is zero, so the accumulate is
// skipped entirely.  Like int_part_held and sign_held this MUST be latched at
// valid_in and held across the CORDIC latency -- the operand bus is gated to 0
// in between, so reading it live at anti_log_valid time would be wrong.
reg zero_held;
always @(posedge clk or negedge rst_n) begin
    if(!rst_n) begin
        int_part_held <= 0;
        sign_held     <= 1'b0;
        zero_held     <= 1'b0;
    end else if(valid_in) begin
        int_part_held <= mantissa_sum[16:8];
        sign_held     <= sign_row_i_j ^ sign_col_i_j;
        zero_held     <= zero_row_i_j | zero_col_i_j;   // OR: either zero -> product zero
    end else begin
        int_part_held <= int_part_held;
        sign_held     <= sign_held;
        zero_held     <= zero_held;
    end
end

//shifter to multiply 2^ii to 2^.ff  (uses the HELD integer part, not the live bus)
// BIDIRECTIONAL: a negative integer part (|product| < 1) is a RIGHT shift.
wire [31:0] antilog_ext = {21'b0, anti_log_fractional_part};
wire [8:0]  shift_amt   = int_part_held[8] ? (~int_part_held + 9'd1) : int_part_held;
wire [31:0] multiplication_result =
        int_part_held[8] ? (antilog_ext >> shift_amt)
                         : (antilog_ext << shift_amt);


//accumalator -- SIGNED. The antilog is always positive, so the product's sign
//lives entirely in sign_held and decides add vs subtract.
reg signed [31:0] accum;
reg accum_valid;
reg last_out_q;

always @(posedge clk or negedge rst_n) begin
if(!rst_n) begin
    accum <= 0;
    accum_valid <= 0;
end 
else if (anti_log_valid) begin
    // A zero operand contributes exactly nothing -- no rounding, no shift.
    // accum_valid / last_out_q still advance so the pipeline and drain are
    // unaffected; only the addition is suppressed.
    if (!zero_held)
        accum <= sign_held ? (accum - $signed({1'b0, multiplication_result[30:0]}))
                           : (accum + $signed({1'b0, multiplication_result[30:0]}));
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
reg signed [EXP_WIDTH-1:0] exponent_col_j_out_q;
reg signed [EXP_WIDTH-1:0] exponent_row_i_out_q;
reg sign_col_i_j_out_q;
reg sign_row_i_j_out_q;
reg zero_col_i_j_out_q;
reg zero_row_i_j_out_q;

always @(posedge clk or negedge rst_n) begin
if(!rst_n) begin
    exp_out_q <= 0;
    mantissa_col_i_j_out_q <= 0;
    mantissa_row_i_j_out_q <= 0;
    exponent_col_j_out_q <= 0;
    exponent_row_i_out_q <= 0;
    sign_col_i_j_out_q <= 0;
    sign_row_i_j_out_q <= 0;
    zero_col_i_j_out_q <= 0;
    zero_row_i_j_out_q <= 0;

end else if(valid_in) begin
    exp_out_q <= exp_out_d;
    mantissa_col_i_j_out_q <= mantissa_col_i_j;
    mantissa_row_i_j_out_q <= mantissa_row_i_j;
    exponent_col_j_out_q <= exponent_col_j;
    exponent_row_i_out_q <= exponent_row_i;
    sign_col_i_j_out_q <= sign_col_i_j;
    sign_row_i_j_out_q <= sign_row_i_j;
    zero_col_i_j_out_q <= zero_col_i_j;
    zero_row_i_j_out_q <= zero_row_i_j;
    end
    else begin
    exp_out_q <= exp_out_q;
    mantissa_col_i_j_out_q <= mantissa_col_i_j_out_q;
    mantissa_row_i_j_out_q <= mantissa_row_i_j_out_q;
    exponent_col_j_out_q <= exponent_col_j_out_q;
    exponent_row_i_out_q <= exponent_row_i_out_q;
    sign_col_i_j_out_q <= sign_col_i_j_out_q;
    sign_row_i_j_out_q <= sign_row_i_j_out_q;
    zero_col_i_j_out_q <= zero_col_i_j_out_q;
    zero_row_i_j_out_q <= zero_row_i_j_out_q;
    end
end

//output assignment
assign mantissa_row_i_j_out = mantissa_row_i_j_out_q;
assign mantissa_col_i_j_out = mantissa_col_i_j_out_q;
assign exp_out = exp_out_q;
assign exponent_col_j_out = exponent_col_j_out_q;
assign exponent_row_i_out = exponent_row_i_out_q;
assign sign_col_i_j_out = sign_col_i_j_out_q;
assign sign_row_i_j_out = sign_row_i_j_out_q;
assign zero_col_i_j_out = zero_col_i_j_out_q;
assign zero_row_i_j_out = zero_row_i_j_out_q;
assign Res_out = accum;
assign valid_out_row_i = accum_valid;
assign valid_out_col_j = accum_valid;
assign last_out = last_out_q; // Output is valid and final when accum is valid and it's the final PE
assign demand_from_mem = anti_log_valid; // Request new data when current data is valid

endmodule
