// HARI OM

// Shifter module used for all shifting operations

`timescale 1ns/1ps

module shifter #(
    parameter DATA_WIDTH = 32,
    parameter EXP_WIDTH = 8
    
)(
    input [DATA_WIDTH-1:0] mant,
    input [EXP_WIDTH-1:0] exp,
    input left,
    output [DATA_WIDTH-1:0] out

);

assign out = left ? (mant << exp) : (mant >> exp);

endmodule

module shifter_quantizer #(
  parameter DATA_WIDTH = 32,
    parameter EXP_WIDTH = 8,
    parameter BLOCK_MANTISSA_WIDTH = 5,
    parameter SCALE_FACTOR_WIDTH = 4
    )(

input [DATA_WIDTH-1:0] mant,
input [SCALE_FACTOR_WIDTH-1:0] scale_factor,
output [BLOCK_MANTISSA_WIDTH-1:0] out

);

wire [DATA_WIDTH-1:0] scaled_mant;
assign scaled_mant = (mant >> scale_factor);

// the output block mantissa is {1'b0, VAL_BITS value bits}; saturate at VAL_MAX.
// At BLOCK_MANTISSA_WIDTH=5 this is exactly the old {1'b0, >15 ? 4'b1111 : [3:0]}.
localparam VAL_BITS = BLOCK_MANTISSA_WIDTH - 1;
localparam VAL_MAX  = (1 << VAL_BITS) - 1;

assign out = {1'b0, (scaled_mant > VAL_MAX) ? {VAL_BITS{1'b1}}
                                            : scaled_mant[VAL_BITS-1:0]};

endmodule


// One-hot -> binary position encoder, any width.
//
// Replaces the hand-written encoder_32_5, which spelled out one OR term per
// output bit for exactly 32 inputs -- that is why DATA_WIDTH was a knob in
// name only: the encode path ignored it.
//
// Same structure, generated: output bit k is the OR of every input whose index
// has bit k set.  (b >> k) & 1 is evaluated at elaboration time (both are
// genvars), so this synthesises to the identical OR tree.
// Verified against the old code: the OR-term sets match for all 5 output bits
// at WIDTH=32, and 200k random vectors through old vs new log_finder agree.
module onehot_encoder #(
    parameter WIDTH = 32
)(
    input  [WIDTH-1:0] inp,
    output [WIDTH-1:0] outp
);
    localparam IDX_BITS = $clog2(WIDTH);   // 32 -> 5, 33 -> 6

    genvar k, b;
    generate
        for (k = 0; k < IDX_BITS; k = k + 1) begin : enc_bit
            wire [WIDTH-1:0] terms;
            for (b = 0; b < WIDTH; b = b + 1) begin : enc_term
                assign terms[b] = (((b >> k) & 1) != 0) ? inp[b] : 1'b0;
            end
            assign outp[k] = |terms;
        end
        if (WIDTH > IDX_BITS) begin : enc_pad
            assign outp[WIDTH-1:IDX_BITS] = {(WIDTH-IDX_BITS){1'b0}};
        end
    endgenerate
endmodule

// Reverse a Number (used in leading one detector)
module rev_num #(
    parameter WIDTH = 32
)(
    input  [WIDTH-1:0] org,
    output [WIDTH-1:0] rev
);

    genvar i;
    generate
        for (i = 0; i < WIDTH; i = i + 1) begin : rev_bit
            assign rev[WIDTH-1-i] = org[i];
        end
    endgenerate
endmodule

// Leading One Detector
module lead_one_detector #(
    parameter WIDTH = 32
)(
    input  [WIDTH-1:0] seq,
    output [WIDTH-1:0] one_hot
);
    wire [WIDTH-1:0] seq_rev;
    wire [WIDTH-1:0] one_hot_rev;
    rev_num #(.WIDTH(WIDTH)) rev_block (
        .org(seq),
        .rev(seq_rev)
    );

    // isolate-lowest-set-bit on the reversed word == highest set bit of seq
    assign one_hot_rev = seq_rev & (~seq_rev + 1'b1);

    rev_num #(.WIDTH(WIDTH)) rev_block_1 (
        .org(one_hot_rev),
        .rev(one_hot)
    );

endmodule

// Log module
// log_mant = (position of leading one) + (FRAC_BITS bits immediately below it).
// Normalise by left-shifting the leading one up to the MSB, then slice the
// FRAC_BITS bits under it.  At WIDTH=32, FRAC_BITS=8 this is bit-for-bit the
// old `31 - exp_mant` / `raw_mant[30:23]` / `{24'b0, ...}` code.
module log_finder #(
    parameter WIDTH     = 32,
    parameter FRAC_BITS = 8
)(
    input  [WIDTH-1:0] shifted_mant,
    output [WIDTH-1:0] log_mant
);

// wide enough to hold the shift amount WIDTH-1
localparam SH_EXP_W = $clog2(WIDTH) + 1;

wire [WIDTH-1:0] lead_one_hot;
wire [WIDTH-1:0] exp_mant;
wire [WIDTH-1:0] raw_mant;

lead_one_detector #(.WIDTH(WIDTH)) lod_0(
    .seq(shifted_mant),
    .one_hot(lead_one_hot)
);

onehot_encoder #(.WIDTH(WIDTH)) enc1(
    .inp(lead_one_hot),
    .outp(exp_mant)
);

wire [SH_EXP_W-1:0] shift_exp = SH_EXP_W'(WIDTH-1) - exp_mant[SH_EXP_W-1:0];
shifter #(.DATA_WIDTH(WIDTH), .EXP_WIDTH(SH_EXP_W)) sh1(
    .mant(shifted_mant),
    .exp(shift_exp),
    .left(1'b1),
    .out(raw_mant)
);

// bits [WIDTH-2 : WIDTH-1-FRAC_BITS] -- the FRAC_BITS just under the leading one
assign log_mant = {{(WIDTH-FRAC_BITS){1'b0}}, raw_mant[WIDTH-2 -: FRAC_BITS]} + exp_mant;

endmodule


// Module for rounding to nearest 2 power
module round_2_power #(
        parameter WIDTH = 32
    )(
        input  [WIDTH-1:0] inp_num,
        output [WIDTH-1:0] out_num
    );

    wire [WIDTH-1:0] lead_one_hot;
    lead_one_detector #(.WIDTH(WIDTH)) lod_1(
        .seq(inp_num),
        .one_hot(lead_one_hot)
    );

    assign out_num = (((lead_one_hot >> 1) & inp_num) != 0) 
                 ? (lead_one_hot << 1) 
                 : lead_one_hot;
    
endmodule


// Expands a compressed two's-complement log mantissa back to the 16-bit 8.8
// log-domain value the PE consumes.
//
// SIGNED: `mant` is TWO'S COMPLEMENT (the residual log L can be negative when
// |v| < 2^E). It must be SIGN-extended, not zero-extended, and shifted
// arithmetically -- zero-extending a negative mantissa silently produces a
// large positive log (e.g. mant=-6, scale=3 would give +208 instead of -48).
module descaling #(
    parameter MANTISSA_WIDTH = 6,          // 6-bit two's complement (was 5 unsigned)
    parameter SCALE_FACTOR_WIDTH = 4,
    // width the PE consumes.  Was hardcoded 16, which made the MANTISSA_WIDTH
    // knob a no-op here and silently truncated whenever the PE bus != 16.
    parameter PE_MANTISSA_WIDTH = 16
)(
    input  signed [MANTISSA_WIDTH-1:0] mant,
    input         [SCALE_FACTOR_WIDTH-1:0] inp_scale_factor,
    output signed [PE_MANTISSA_WIDTH-1:0] inp_pe
);

assign inp_pe = $signed({{(PE_MANTISSA_WIDTH-MANTISSA_WIDTH){mant[MANTISSA_WIDTH-1]}}, mant}) <<< inp_scale_factor;

endmodule

