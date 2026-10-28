// HARI OM

//=============================================================================
// submodules_FC.sv  --  small building blocks
//
//   shifter            left or right shift by a variable amount
//   onehot_encoder     one-hot word -> index of its 1 bit
//   rev_num            reverse the bit order of a word
//   lead_one_detector  keep only the highest 1 bit of a word
//   log_finder         log2 of an unsigned number (Mitchell's method)
//                      -> used by format_convertor
//   descaling          block L -> PE log operand (L << scale)
//                      -> used by edge_reg_bank
//=============================================================================

`timescale 1ns/1ps

// ---------------------------------------------------------------------------
// shifter: out = left ? mant << exp : mant >> exp   (logical shifts)
// ---------------------------------------------------------------------------
module shifter #(
    parameter DATA_WIDTH = 32,   // bits of the value
    parameter EXP_WIDTH = 8      // bits of the shift amount

)(
    input [DATA_WIDTH-1:0] mant,
    input [EXP_WIDTH-1:0] exp,
    input left,
    output [DATA_WIDTH-1:0] out

);

assign out = left ? (mant << exp) : (mant >> exp);

endmodule


// ---------------------------------------------------------------------------
// onehot_encoder: index of the single 1 bit in inp.
//   Output bit k = OR of every input bit whose index has bit k set.
//   Example (WIDTH 8): inp = 0010_0000 (bit 5)  ->  outp = 5.
//   Only the low $clog2(WIDTH) output bits are used; the rest are 0.
//   If inp is all zero, outp = 0.
// ---------------------------------------------------------------------------
module onehot_encoder #(
    parameter WIDTH = 32
)(
    input  [WIDTH-1:0] inp,
    output [WIDTH-1:0] outp
);
    localparam IDX_BITS = $clog2(WIDTH);   // bits needed for an index: 32 -> 5, 33 -> 6

    genvar k, b;
    generate
        for (k = 0; k < IDX_BITS; k = k + 1) begin : enc_bit
            wire [WIDTH-1:0] terms;        // inputs that have bit k set in their index
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

// ---------------------------------------------------------------------------
// rev_num: rev = org with the bit order reversed (bit 0 <-> bit WIDTH-1)
// ---------------------------------------------------------------------------
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

// ---------------------------------------------------------------------------
// lead_one_detector: one_hot = seq with only its highest 1 bit kept.
//   x & (~x + 1) keeps the LOWEST 1 bit of x, so the word is reversed, its
//   lowest 1 kept, and reversed back.
//   Example: seq = 0110_1000  ->  one_hot = 0100_0000.   seq = 0 -> 0.
// ---------------------------------------------------------------------------
module lead_one_detector #(
    parameter WIDTH = 32
)(
    input  [WIDTH-1:0] seq,
    output [WIDTH-1:0] one_hot
);
    wire [WIDTH-1:0] seq_rev;       // seq reversed
    wire [WIDTH-1:0] one_hot_rev;   // lowest 1 of seq_rev = highest 1 of seq (still reversed)
    rev_num #(.WIDTH(WIDTH)) rev_block (
        .org(seq),
        .rev(seq_rev)
    );

    assign one_hot_rev = seq_rev & (~seq_rev + 1'b1);

    rev_num #(.WIDTH(WIDTH)) rev_block_1 (
        .org(one_hot_rev),
        .rev(one_hot)
    );

endmodule

// ---------------------------------------------------------------------------
// log_finder: log2 of an unsigned number, Mitchell's method
//
//   Any v > 0 is v = 2^p * (1 + x), 0 <= x < 1, p = position of the leading 1.
//   log2(v) = p + log2(1 + x) ~ p + x
//   (log2(1+x) and x are equal at x = 0 and x = 1, so the straight line x is
//    a close fit in between.)
//   x is just the bits under the leading 1, read as a binary fraction.
//
//   log_mant = p * 2^FRAC_BITS + (the FRAC_BITS bits under the leading 1)
//              i.e. log2(v) in fixed point with FRAC_BITS fraction bits.
//   Example: v = 1280 = 1024 * 1.25  ->  p = 10, x = 0.25 = 64/256
//            log_mant = 10*256 + 64 = 2624 (10.25; exact log2 is 10.32)
//
//   Signals:
//     lead_one_hot : v with only its leading 1 kept
//     exp_mant     : p, the position of that 1
//     shift_exp    : WIDTH-1-p, the left shift that moves the leading 1 to the top bit
//     raw_mant     : v shifted so its leading 1 is in bit WIDTH-1
//
//   LIMITATIONS
//   - Always low by up to 0.086 octave (largest at x ~ 0.44): the value is up
//     to about 6% too small. The bits below the FRAC_BITS kept are dropped
//     (at most 1/2^FRAC_BITS more).
//   - log2(0) does not exist; v = 0 gives 0 (same as v = 1). The caller must
//     handle zero separately (format_convertor uses a zero flag).
//   - Only the low $clog2(WIDTH) + FRAC_BITS bits of log_mant can be non-zero.
// ---------------------------------------------------------------------------
module log_finder #(
    parameter WIDTH     = 32,   // bits of the input
    parameter FRAC_BITS = 8     // fraction bits of the log
)(
    input  [WIDTH-1:0] shifted_mant,   // v (unsigned)
    output [WIDTH-1:0] log_mant        // log2(v), FRAC_BITS fraction bits
);

// bits needed for a shift of up to WIDTH-1
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

// p in the integer bits, the FRAC_BITS bits under the leading 1 as the fraction
assign log_mant = (exp_mant << FRAC_BITS)
                + {{(WIDTH-FRAC_BITS){1'b0}}, raw_mant[WIDTH-2 -: FRAC_BITS]};

endmodule


// ---------------------------------------------------------------------------
// descaling: block residual L -> the log operand the PE uses
//
//   inp_pe = L * 2^scale  (= L << scale, sign-extended first)
//   This is log2(|v| / 2^E) in fixed point with FRAC_WIDTH fraction bits.
//   L is signed (two's complement), so it is sign-extended and shifted
//   arithmetically. Example: L = -6, scale = 3 -> -48 (= -0.1875 octave).
//
//   LIMITATION: L * 2^scale must fit PE_MANTISSA_WIDTH signed bits, so
//   scale <= PE_MANTISSA_WIDTH - MANTISSA_WIDTH (16 - 6 = 10 at the
//   defaults). A larger scale wraps silently.
// ---------------------------------------------------------------------------
module descaling #(
    parameter MANTISSA_WIDTH = 6,          // bits of L (signed) = BLOCK_MANTISSA_WIDTH
    parameter SCALE_FACTOR_WIDTH = 4,      // bits of the block scale
    parameter PE_MANTISSA_WIDTH = 16       // bits of the PE log operand
)(
    input  signed [MANTISSA_WIDTH-1:0] mant,             // L
    input         [SCALE_FACTOR_WIDTH-1:0] inp_scale_factor,  // scale
    output signed [PE_MANTISSA_WIDTH-1:0] inp_pe         // L << scale
);

assign inp_pe = $signed({{(PE_MANTISSA_WIDTH-MANTISSA_WIDTH){mant[MANTISSA_WIDTH-1]}}, mant}) <<< inp_scale_factor;

endmodule

