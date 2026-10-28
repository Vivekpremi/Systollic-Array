`timescale 1ns/1ps
//=============================================================================
// PE.sv  --  one processing element of the systolic array
//
// WHAT IT DOES
//   Multiplies one row operand by one column operand per step and adds the
//   product into its own accumulator. After all ARRAY_SIZE steps of a tile the
//   accumulator holds one element of C = A x B. It also passes both operands
//   on: the row operand to the PE on the right, the column operand to the PE
//   below.
//
// NUMBER FORMAT OF THE OPERANDS (after edge_reg_bank's descaler)
//   mantissa_* = log2(|v| / 2^E) in fixed point with FRAC_WIDTH fraction bits
//                (= L * 2^scale of the block), signed, MANTISSA_WIDTH bits
//   exponent_* = the block exponent E (signed, EXP_WIDTH bits)
//   sign_*     = 1 if the value is negative;  zero_* = 1 if the value is 0
//
// THE MULTIPLY, STEP BY STEP
//   1. log add:   s = mantissa_row + mantissa_col = log2(|a*b| / 2^(Er+Ec))
//   2. split s:   s = n + f, n = integer part (floor, signed), f = fraction 0..1
//                 (two's complement makes this a plain bit split, e.g.
//                  -0.7 = -1 + 0.3)
//   3. antilog:   2^f = e^(f*ln2). f*ln2 is done with shifts, the e^x by
//                 exponential_CORDIC (about 10 clocks). Result 1.0..2.0.
//   4. shift:     |a*b| / 2^(Er+Ec) = 2^f * 2^n  -> shift 2^f left by n
//                 (n >= 0) or right by |n| (n < 0)
//   5. add:       accum += product, or -= if sign_row XOR sign_col = 1.
//                 Skipped if either operand is zero.
//   The exponent of the result is exp_out = Er + Ec (the same for every step
//   of a tile, since a block shares one E).
//
// OUTPUT
//   real value of C[i][j] = Res_out / 2^FRAC_WIDTH * 2^exp_out
//   Res_out is final when last_out = 1.
//
// LIMITATIONS / NOTES
//   - One operand pair at a time: a new pair is taken only when the CORDIC
//     is idle. demand_from_mem pulses when a product finishes; the array uses
//     PE[i][0]'s pulse to let the edge bank present the next operands.
//   - The accumulator is cleared only by reset: ONE TILE PER RESET.
//   - Product limit: 2^f * 2^n must fit DATA_WIDTH bits with FRAC_WIDTH
//     fraction bits, i.e. n <= DATA_WIDTH - FRAC_WIDTH - 1 (product < 2^24 at
//     the defaults, relative to 2^(Er+Ec)). Larger products lose top bits.
//   - Sum limit: the accumulator is DATA_WIDTH+1 bits signed.
//   - Both overflows set overflow_out (sticky until reset); the arithmetic is
//     not changed by the flag.
//   - Small products: a right shift drops bits, so products below 2^-FRAC_WIDTH
//     (relative) become 0.
//   - f*ln2 uses 1/2 + 1/8 + 1/16 + 1/128 = 0.6953 instead of 0.6931 (0.3%
//     high), and each shift rounds down.
//=============================================================================
module PE #(
    parameter EXP_WIDTH      = 8,    // bits of a block exponent E (signed)
    parameter MANTISSA_WIDTH = 16,   // bits of the log operand (INT.FRAC = 8.8 by default)
    parameter FRAC_WIDTH     = 8,    // fraction bits of the log; must match exponential_CORDIC (8)
    parameter DATA_WIDTH     = 32,   // bits of the product; the accumulator has DATA_WIDTH+1
    // ---- derived ----
    // INT_WIDTH: integer bits of one log operand (the "8" in 8.8)
    parameter INT_WIDTH      = MANTISSA_WIDTH - FRAC_WIDTH,
    // SUM_INT_WIDTH: integer bits of the SUM of two logs (one carry bit more);
    // this is the width of n, the shift amount
    parameter SUM_INT_WIDTH  = INT_WIDTH + 1
)(
    input wire clk,
    input wire rst_n,                                        // async, active low
    input wire valid_in,                                     // both operands are present this clock
    input wire signed [MANTISSA_WIDTH-1:0] mantissa_col_i_j, // column operand, log form
    input wire signed [EXP_WIDTH-1:0] exponent_col_j,        // column block exponent
    input wire signed [MANTISSA_WIDTH-1:0] mantissa_row_i_j, // row operand, log form
    input wire signed [EXP_WIDTH-1:0] exponent_row_i,        // row block exponent
    input wire sign_col_i_j,              // column operand is negative
    input wire sign_row_i_j,              // row operand is negative
    input wire zero_col_i_j,              // column operand is exactly 0
    input wire zero_row_i_j,              // row operand is exactly 0
    input wire last_row_in,               // the row operand stream has reached its last element
    input wire last_col_in,               // the column operand stream has reached its last element

    // operands passed on (registered at valid_in): row -> right, column -> down
    output wire signed [MANTISSA_WIDTH-1:0] mantissa_row_i_j_out,
    output wire last_out,                 // Res_out is final (stays 1 until reset)
    output wire signed [MANTISSA_WIDTH-1:0] mantissa_col_i_j_out,
    output wire signed [EXP_WIDTH-1:0] exponent_col_j_out,
    output wire signed [EXP_WIDTH-1:0] exponent_row_i_out,
    output wire sign_col_i_j_out,
    output wire sign_row_i_j_out,
    output wire zero_col_i_j_out,
    output wire zero_row_i_j_out,
    output wire signed [DATA_WIDTH:0] Res_out,   // the accumulator, FRAC_WIDTH fraction bits
    output wire valid_out_row_i,          // 1-clock pulse after each product: valid_in of the PE on the right
    output wire valid_out_col_j,          // same pulse: valid_in of the PE below
    output wire demand_from_mem,          // 1-clock pulse when a product finishes (ready for the next pair)
    output wire overflow_out,             // sticky: a product or the accumulator lost its top bits
    output wire signed [EXP_WIDTH:0] exp_out     // Er + Ec (EXP_WIDTH+1 bits, cannot overflow)
);

// ---------------------------------------------------------------------------
// exponent of the product: Er + Ec. One bit wider than E, so it never wraps
// (-256..254 at EXP_WIDTH = 8).
// ---------------------------------------------------------------------------
wire signed [EXP_WIDTH:0] exp_sum;
assign exp_sum = exponent_col_j + exponent_row_i;

wire signed [EXP_WIDTH:0] exp_out_d;
assign exp_out_d =  exp_sum ;

// ---------------------------------------------------------------------------
// STEP 1: log add.  mantissa_sum = log2(|a*b| / 2^(Er+Ec)), FRAC_WIDTH
// fraction bits, one bit wider than an operand (signed).
// Its top SUM_INT_WIDTH bits are n (floor of the log); the low FRAC_WIDTH
// bits are f (always 0..1, also for a negative log: -0.7 = -1 + 0.3).
// ---------------------------------------------------------------------------
wire signed [MANTISSA_WIDTH:0] mantissa_sum;
assign mantissa_sum = mantissa_col_i_j + mantissa_row_i_j;

// ---------------------------------------------------------------------------
// STEP 3: antilog of the fraction, 2^f = e^(f * ln2)
//   frac_part           : f, FRAC_WIDTH bits (0..255 = 0.0..0.996)
//   fractional_part_ln2 : f * ln2, using ln2 ~ 1/2 + 1/8 + 1/16 + 1/128
//   anti_log_fractional_part : 2^f from the CORDIC, FRAC_WIDTH fraction bits,
//                              value 1.0..2.0 (256..511 at FRAC_WIDTH = 8)
//   anti_log_valid      : 1-clock pulse when that result is ready
// ---------------------------------------------------------------------------
wire anti_log_valid;
wire [FRAC_WIDTH+2:0] anti_log_fractional_part;

wire [FRAC_WIDTH-1:0] frac_part = mantissa_sum[FRAC_WIDTH-1:0];
wire [FRAC_WIDTH-1:0] fractional_part_ln2 = (frac_part >> 1) + (frac_part >> 3)
                                          + (frac_part >> 4) + (frac_part >> 7);

exponential_CORDIC  #(
.Frac_WIDTH (FRAC_WIDTH)   // fraction bits of x and of the result (must be 8)
)
anti_log (
    .clk(clk),
    .rst(rst_n),
    .valid_in(valid_in),
    .x(fractional_part_ln2),
    .valid_out(anti_log_valid),
    .exp_result(anti_log_fractional_part)
);

// ---------------------------------------------------------------------------
// Values held while the CORDIC works (about 10 clocks).
// The operand bus is forced to 0 between operands, so everything the final
// step needs is captured at valid_in:
//   int_part_held : n, the integer part of the log sum (signed)
//   sign_held     : sign of the product = sign_row XOR sign_col
//   zero_held     : the product is 0 (either operand is 0)
// Only one product is in flight at a time, so one register each is enough.
// ---------------------------------------------------------------------------
reg signed [SUM_INT_WIDTH-1:0] int_part_held;
reg sign_held;
reg zero_held;
always @(posedge clk or negedge rst_n) begin
    if(!rst_n) begin
        int_part_held <= 0;
        sign_held     <= 1'b0;
        zero_held     <= 1'b0;
    end else if(valid_in) begin
        int_part_held <= mantissa_sum[MANTISSA_WIDTH:FRAC_WIDTH];
        sign_held     <= sign_row_i_j ^ sign_col_i_j;
        zero_held     <= zero_row_i_j | zero_col_i_j;
    end else begin
        int_part_held <= int_part_held;
        sign_held     <= sign_held;
        zero_held     <= zero_held;
    end
end

// ---------------------------------------------------------------------------
// STEP 4: |product| = 2^f * 2^n, as a DATA_WIDTH-bit number with FRAC_WIDTH
// fraction bits
//   antilog_ext : 2^f widened to DATA_WIDTH bits
//   shift_amt   : |n|
//   multiplication_result : 2^f << n  (n >= 0)   or   2^f >> |n|  (n < 0)
// A left shift past the top loses bits (see the overflow check below); a
// right shift drops bits at the bottom (rounds down).
// ---------------------------------------------------------------------------
wire [DATA_WIDTH-1:0] antilog_ext =
        {{(DATA_WIDTH-(FRAC_WIDTH+3)){1'b0}}, anti_log_fractional_part};
wire [SUM_INT_WIDTH-1:0] shift_amt = int_part_held[SUM_INT_WIDTH-1]
        ? (~int_part_held + {{(SUM_INT_WIDTH-1){1'b0}},1'b1}) : int_part_held;
wire [DATA_WIDTH-1:0] multiplication_result =
        int_part_held[SUM_INT_WIDTH-1] ? (antilog_ext >> shift_amt)
                         : (antilog_ext << shift_amt);

// ---------------------------------------------------------------------------
// STEP 5: accumulator (signed, DATA_WIDTH+1 bits)
//   accum       : running sum of the signed products = Res_out
//   accum_valid : 1-clock pulse after each product (passed on as valid_out)
//   last_out_q  : set by the product that was the last of both streams
// The product is always positive; its sign (sign_held) chooses add or
// subtract. {1'b0, product} keeps all DATA_WIDTH product bits as a positive
// number.
// ---------------------------------------------------------------------------
reg signed [DATA_WIDTH:0] accum;
reg accum_valid;
reg last_out_q;

// ---------------------------------------------------------------------------
// OVERFLOW FLAG (only a flag; the arithmetic is not changed)
//   prod_ovf : on a left shift, antilog_ext << shift_amt keeps only
//              DATA_WIDTH bits. The bits that fall off are
//              antilog_ext >> (DATA_WIDTH - shift_amt); any 1 there is lost.
//              A shift of DATA_WIDTH or more loses everything. Computed in
//              32 bits so it works for any SUM_INT_WIDTH.
//   acc_ovf  : the same add/subtract done one bit wider; if the top two bits
//              differ, the DATA_WIDTH+1-bit accumulator has wrapped.
//   ovf_q    : sticky flag, set by either (zero products are skipped)
// ---------------------------------------------------------------------------
wire        prod_left = ~int_part_held[SUM_INT_WIDTH-1];
wire [31:0] sh32      = 32'(shift_amt);
wire        prod_ovf  = prod_left &&
                        ((sh32 >= 32'(DATA_WIDTH)) ? (antilog_ext != 0)
                                                   : ((antilog_ext >> (32'(DATA_WIDTH) - sh32)) != 0));
wire signed [DATA_WIDTH+1:0] acc_ext  = {accum[DATA_WIDTH], accum};
wire signed [DATA_WIDTH+1:0] prod_ext = {2'b00, multiplication_result};
wire signed [DATA_WIDTH+1:0] acc_next = sign_held ? (acc_ext - prod_ext) : (acc_ext + prod_ext);
wire        acc_ovf   = (acc_next[DATA_WIDTH+1] != acc_next[DATA_WIDTH]);

reg ovf_q;
always @(posedge clk or negedge rst_n) begin
    if (!rst_n)
        ovf_q <= 1'b0;
    else if (anti_log_valid && !zero_held && (prod_ovf || acc_ovf))
        ovf_q <= 1'b1;           // stays set until reset, like accum
end
assign overflow_out = ovf_q;

always @(posedge clk or negedge rst_n) begin
if(!rst_n) begin
    accum <= 0;
    accum_valid <= 0;
    last_out_q <= 1'b0;
end
else if (anti_log_valid) begin
    // a zero operand adds nothing; valid / last still advance as normal
    if (!zero_held)
        accum <= sign_held ? (accum - $signed({1'b0, multiplication_result}))
                           : (accum + $signed({1'b0, multiplication_result}));
    accum_valid <= 1;
    last_out_q <= last_row_in && last_col_in;
end else begin
    accum <= accum;
    accum_valid <= 0;
    last_out_q <= last_out_q ;
end
end



// ---------------------------------------------------------------------------
// Pipeline registers: the operands (and Er + Ec) captured at valid_in. They
// are what the next PEs receive: row data to the right, column data below.
// ---------------------------------------------------------------------------
reg signed [EXP_WIDTH:0] exp_out_q;
reg signed [MANTISSA_WIDTH-1:0] mantissa_col_i_j_out_q;
reg signed [MANTISSA_WIDTH-1:0] mantissa_row_i_j_out_q;
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

// ---------------------------------------------------------------------------
// outputs
// ---------------------------------------------------------------------------
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
assign last_out = last_out_q;              // final result of the tile is in Res_out
assign demand_from_mem = anti_log_valid;   // ready for the next operand pair

endmodule

