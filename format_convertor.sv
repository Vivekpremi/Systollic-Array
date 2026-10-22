// HARI OM

`timescale 1ns/1ps

//=============================================================================
// format_convertor.sv  --  accumulator -> output BLOCK encoder
//
// Emits a packed block with EXACTLY the same layout and semantics as the INPUT
// block that edge_reg_bank consumes, so a tile's output can be fed straight
// back in as a tile's input:
//
//     v = (-1)^s * 2^E * 2^L        (or 0 when the zero flag is set)
//
//     block[MANT_LSB  +: ARRAY_SIZE*BLOCK_MANTISSA_WIDTH] = L[], TWO'S COMPLEMENT
//     block[ZERO_LSB  +: ARRAY_SIZE]                      = zero flags
//     block[SIGN_LSB  +: ARRAY_SIZE]                      = value signs
//     block[SCALE_LSB +: SCALE_FACTOR_WIDTH]              = shared scale factor
//     block[EXP_LSB   +: EXP_WIDTH]                       = shared exponent E, SIGNED
//
// E is chosen by GEOMETRIC-MEAN centring -- E = round(mean of log2|v| over the
// NON-ZERO elements) -- matching how the input blocks are built, so L straddles
// zero and the signed residual field is used in both directions.
//
// HOW THIS DIFFERS FROM THE PREVIOUS VERSION
// The old encoder aligned every mantissa to the column's MAX exponent, took the
// log of the aligned integer, and emitted that as an UNSIGNED mantissa. That
// made L a magnitude measured upward from a floor (always >= 0) rather than a
// signed deviation around a centre, so the output could not be decoded by the
// input decoder. It also carried no sign and no zero flag, so a negative or
// zero result was unrepresentable. The alignment shift is gone: per-element
// ABSOLUTE logs are computed instead, which is what centring needs anyway.
//
// FSM:  CALC_MAX_EXP -> COLLECT -> ENCODE -> CALC_MAX_EXP
//   CALC_MAX_EXP : drain iface's exponent pre-pass; assert max_exp_calculated
//   COLLECT      : per mantissa pair, record sign / zero / absolute log
//   ENCODE       : centre, pick the scale factor, quantise, assert block_ready
//=============================================================================
module format_convertor #(
    parameter ARRAY_SIZE = 4,
    parameter BLOCK_MANTISSA_WIDTH = 6,   // residual log L, two's complement -- SAME as the input block
    parameter DATA_WIDTH = 32,
    parameter FRAC_WIDTH = 8,             // fractional bits of the log format
    parameter EXP_WIDTH = 8,
    parameter SCALE_FACTOR_WIDTH = 4,
    // ---- derived block geometry, identical to edge_reg_bank's ----
    parameter MANT_BUS_WIDTH = ARRAY_SIZE*BLOCK_MANTISSA_WIDTH,
    parameter BLOCK_BITS     = MANT_BUS_WIDTH + 2*ARRAY_SIZE
                               + SCALE_FACTOR_WIDTH + EXP_WIDTH,
    parameter BLOCK_WIDTH    = ((BLOCK_BITS + 7)/8)*8
)(
    input clk,
    input rst_n,
    input signed [DATA_WIDTH:0] mant0,
    input signed [DATA_WIDTH:0] mant1,
    input signed [EXP_WIDTH:0] exp_m_1,
    input signed [EXP_WIDTH:0] exp_m_2,
    input mantissa_valid,
    input all_mantissas_sent,
    input signed [EXP_WIDTH:0] exp0,
    input signed [EXP_WIDTH:0] exp1,
    input exp_valid,
    input last_exp_sent,

    // ---- the output block, ready to be re-fed as an input block ----
    output [BLOCK_WIDTH-1:0] block_out,

    // ---- decomposed view of the same block (observability / legacy) ----
    output signed [EXP_WIDTH-1:0]            max_exp,       // = E, signed
    output [SCALE_FACTOR_WIDTH-1:0]          scale_factor,
    output [MANT_BUS_WIDTH-1:0]              mantissa_out,  // L[], two's complement
    output [ARRAY_SIZE-1:0]                  sign_out,
    output [ARRAY_SIZE-1:0]                  zero_out,
    output block_ready,
    // block exponent out of the EXP_WIDTH range: E was CLAMPED, not wrapped.
    // Status only (not part of the block), held with the block until the next one.
    output exp_overflow,
    output exp_underflow,
    output reg max_exp_calculated
);

// ---- field offsets, mirroring edge_reg_bank exactly ----
localparam MANT_LSB  = 0;
localparam ZERO_LSB  = MANT_LSB  + MANT_BUS_WIDTH;
localparam SIGN_LSB  = ZERO_LSB  + ARRAY_SIZE;
localparam SCALE_LSB = SIGN_LSB  + ARRAY_SIZE;
localparam EXP_LSB   = SCALE_LSB + SCALE_FACTOR_WIDTH;

// log path width: |accumulator| needs DATA_WIDTH+1 bits
localparam LOGW   = DATA_WIDTH + 1;
localparam LIDX   = $clog2(ARRAY_SIZE);
// ONE signed working width for every log quantity (per-element log, the running
// sum, the mean, residuals, the scale search). Mixing widths here is what
// produced a pile of WIDTHEXPAND/WIDTHTRUNC warnings.
//   |log2(v)| <= 2^(EXP_WIDTH-1) + LOGW, scaled by 2^FRAC_WIDTH, summed over
//   ARRAY_SIZE elements, plus a sign bit and a little headroom.
// log_finder's PORT is LOGW bits, but its VALUE is at most
// ((LOGW-1)<<FRAC_WIDTH)+2^FRAC_WIDTH-1, so only LGW bits are meaningful.
// Slicing to LGW instead of zero-extending from LOGW is what keeps ALOGW small.
localparam LGW    = $clog2(LOGW) + FRAC_WIDTH + 1;
// |log2(v)| <= 2^(EXP_WIDTH-1) + LOGW, scaled by 2^FRAC_WIDTH, summed over
// ARRAY_SIZE elements, plus sign and one bit of headroom.
// Sizing this from LOGW instead made it 46 bits and produced a 46-bit divider.
localparam ALOGW  = FRAC_WIDTH + $clog2((1 << EXP_WIDTH) + LOGW) + LIDX + 2;
// signed residual field limits, at the working width for clean comparison
localparam signed [ALOGW-1:0] LMAX =  (1 << (BLOCK_MANTISSA_WIDTH-1)) - 1;
localparam signed [ALOGW-1:0] LMIN = -(1 << (BLOCK_MANTISSA_WIDTH-1));

localparam ST_EXP = 2'd0, ST_COLLECT = 2'd1, ST_ENCODE = 2'd2;
reg [1:0] state;

reg [LIDX:0]               idx;        // element write pointer (0..ARRAY_SIZE)
reg signed [ALOGW-1:0]     alog  [0:ARRAY_SIZE-1];   // absolute log2|v|, FRAC fixed pt
reg                        sgn_r [0:ARRAY_SIZE-1];
reg                        zer_r [0:ARRAY_SIZE-1];
reg signed [ALOGW-1:0]     alog_sum;   // running sum over NON-ZERO elements
reg [LIDX:0]               nz_cnt;     // how many non-zero (0..ARRAY_SIZE)
reg                        blk_rdy_q;
reg                        ovf_q, unf_q;   // held with the block

// held output fields
reg signed [EXP_WIDTH-1:0]       E_q;
reg [SCALE_FACTOR_WIDTH-1:0]     scale_q;
reg [MANT_BUS_WIDTH-1:0]         mant_q;
reg [ARRAY_SIZE-1:0]             sign_q, zero_q;

// ---------------------------------------------------------------------------
// per-element decode of the two incoming accumulator values
//   sign : the accumulator's sign bit
//   zero : exactly zero (log2(0) is unrepresentable, so it travels as a flag)
//   abs  : magnitude, which is what the log path must see.  Feeding the raw
//          two's-complement word into a leading-one detector made a negative
//          accumulator look enormous (bit DATA_WIDTH set).
// ---------------------------------------------------------------------------
wire                  s0 = mant0[DATA_WIDTH];
wire                  s1 = mant1[DATA_WIDTH];
wire                  z0 = (mant0 == 0);
wire                  z1 = (mant1 == 0);
wire [LOGW-1:0]       a0 = s0 ? (~mant0 + 1'b1) : mant0;
wire [LOGW-1:0]       a1 = s1 ? (~mant1 + 1'b1) : mant1;

wire [LOGW-1:0] lg0, lg1;
log_finder #(.WIDTH(LOGW), .FRAC_BITS(FRAC_WIDTH)) lgf0(.shifted_mant(a0), .log_mant(lg0));
log_finder #(.WIDTH(LOGW), .FRAC_BITS(FRAC_WIDTH)) lgf1(.shifted_mant(a1), .log_mant(lg1));

// Absolute log of the VALUE, not of the raw accumulator word:
//   value   = Res_out / 2^FRAC_WIDTH * 2^exp_m
//   log2(v) = log2(Res_out) - FRAC_WIDTH + exp_m
// all in FRAC_WIDTH fixed point.
function automatic signed [ALOGW-1:0] mk_alog(input [LOGW-1:0] lg,
                                             input signed [EXP_WIDTH:0] em);
    logic signed [ALOGW-1:0] lg_s, em_s;
    begin
        lg_s   = $signed({{(ALOGW-LGW){1'b0}}, lg[LGW-1:0]});  // log is >= 0
        em_s   = ALOGW'(em);                                    // sign-extends
        mk_alog = lg_s + ((em_s - ALOGW'(FRAC_WIDTH)) <<< FRAC_WIDTH);
    end
endfunction

wire signed [ALOGW-1:0] alog0 = mk_alog(lg0, exp_m_1);
wire signed [ALOGW-1:0] alog1 = mk_alog(lg1, exp_m_2);

// ---------------------------------------------------------------------------
// ENCODE combinational work: centre, size the scale factor, quantise.
// ---------------------------------------------------------------------------
// E = round(mean of log2|v| over non-zeros), as an INTEGER exponent.
// nz_cnt is not a power of two in general, so this is a real divide. It runs
// ONCE PER BLOCK, not per element -- but for synthesis replace it with a
// reciprocal multiply (recip[nz_cnt] is an ARRAY_SIZE-entry constant table).
// DIVIDE-FREE mean.  nz_cnt is 0..ARRAY_SIZE, so instead of one variable
// divider we build one CONSTANT multiply per possible count and mux the
// results. Each constant multiply synthesises to shift-adds; for a power-of-two
// count it collapses to a plain shift (/1 is free, /2 is >>1, /4 is >>2), and
// only the non-power-of-two counts cost a short adder chain.
// A single `alog_sum / nz_cnt` cost ~20k cells on its own.
localparam RW = FRAC_WIDTH + 8;          // reciprocal fixed-point bits
localparam PW = ALOGW + RW + 2;          // full width of sum * reciprocal
wire signed [PW-1:0] mean_cand [0:ARRAY_SIZE];
genvar gi;
generate
    assign mean_cand[0] = '0;            // no non-zero elements: unused
    for (gi = 1; gi <= ARRAY_SIZE; gi = gi + 1) begin : mean_g
        // (1<<RW)/gi is folded at ELABORATION (gi is a genvar), so this is a
        // multiply by a literal, not a divide.
        localparam [RW:0] RECIP = (1 << RW) / gi;
        // Verilog sizes a*b as max(width(a),width(b)), NOT the sum, so both
        // operands must be widened to the full product width FIRST or the
        // product truncates and >>> RW shifts the remainder away (E came out 0).
        wire signed [PW-1:0] sum_ext  = {{(PW-ALOGW){alog_sum[ALOGW-1]}}, alog_sum};
        wire signed [PW-1:0] rcp_ext  = {{(PW-RW-1){1'b0}}, RECIP};
        wire signed [PW-1:0] prod     = sum_ext * rcp_ext;
        assign mean_cand[gi] = (prod >>> RW);
    end
endgenerate
wire signed [ALOGW-1:0] mean_alog = mean_cand[nz_cnt[LIDX:0]][ALOGW-1:0];
// round-to-nearest on the way down to an INTEGER exponent
wire signed [ALOGW-1:0] mean_rnd  = mean_alog + ALOGW'(1 << (FRAC_WIDTH-1));
wire signed [ALOGW-1:0] mean_int  = mean_rnd >>> FRAC_WIDTH;
// EXPONENT RANGE. The per-PE exponent is EXP_WIDTH+1 bits (sum of two block
// exponents), but the block's E field is EXP_WIDTH, the same as the input
// block. A centre outside [-2^(EXP_WIDTH-1), 2^(EXP_WIDTH-1)-1] cannot be
// stored. It used to be SLICED (wrapped): 201 became -55, which forced the
// residuals to absorb a 256-octave gap at scale 13 -- an unbounded error that
// happened to be 18% on test 120. Clamping keeps E as close as the field allows
// and the flag tells the controller the block is out of range.
localparam signed [ALOGW-1:0] EMAX_W =  (1 << (EXP_WIDTH-1)) - 1;
localparam signed [ALOGW-1:0] EMIN_W = -(1 << (EXP_WIDTH-1));
wire e_ovf = (mean_int > EMAX_W);
wire e_unf = (mean_int < EMIN_W);
wire signed [EXP_WIDTH-1:0] E_c   = e_ovf ? {1'b0, {(EXP_WIDTH-1){1'b1}}}     //  2^(W-1)-1
                                  : e_unf ? {1'b1, {(EXP_WIDTH-1){1'b0}}}     // -2^(W-1)
                                  : mean_int[EXP_WIDTH-1:0];

// residual logs about that centre, still in FRAC_WIDTH fixed point
wire signed [ALOGW-1:0] resid [0:ARRAY_SIZE-1];
genvar g;
generate
    for (g = 0; g < ARRAY_SIZE; g = g + 1) begin : res_g
        assign resid[g] = alog[g] - (ALOGW'(E_c) <<< FRAC_WIDTH);
    end
endgenerate

// The scale factor is the smallest s for which every residual fits the signed
// BLOCK_MANTISSA_WIDTH field [LMIN, LMAX] = [-2^(W-1), 2^(W-1)-1] after
// quantisation by 2^s. The field is ASYMMETRIC, so the positive and negative
// extremes are tested against their OWN limits:
//     max positive residual  <= LMAX    << s
//     max negative magnitude <= |LMIN|  << s
// With round-to-nearest, a residual >= LMIN<<s still quantises to >= LMIN, so
// the negative bound is exact.
// BUG FIXED: both signs used to be tested against LMAX<<s. A residual of
// exactly -32<<5 (test 136: 0.5 next to values ~27) was rejected at s=5 and
// forced s=6, doubling the step for the whole block (25.56 -> 22.63, 10.7%).
// The tb's input-block encoder already used the exact range, so the input and
// output encoders now agree.
reg signed [ALOGW-1:0] need_p;   // largest positive residual
reg signed [ALOGW-1:0] need_n;   // largest negative residual, as a magnitude
reg [SCALE_FACTOR_WIDTH-1:0] scale_c;
integer m, sidx;
always @(*) begin
    need_p = 0;
    need_n = 0;
    for (m = 0; m < ARRAY_SIZE; m = m + 1)
        if (!zer_r[m]) begin
            if (resid[m] < 0) begin
                if ((-resid[m]) > need_n) need_n = (-resid[m]);
            end else begin
                if (resid[m] > need_p)    need_p = resid[m];
            end
        end
    scale_c = {SCALE_FACTOR_WIDTH{1'b1}};          // saturate if nothing fits
    for (sidx = (1 << SCALE_FACTOR_WIDTH) - 1; sidx >= 0; sidx = sidx - 1)
        if ((need_p <= ALOGW'(((1 << (BLOCK_MANTISSA_WIDTH-1)) - 1) << sidx)) &&
            (need_n <= ALOGW'(  (1 << (BLOCK_MANTISSA_WIDTH-1))      << sidx)))
            scale_c = sidx[SCALE_FACTOR_WIDTH-1:0];
end

// quantise each residual by the chosen scale, round to nearest, then saturate
reg signed [ALOGW-1:0]              q_wide [0:ARRAY_SIZE-1];
reg signed [BLOCK_MANTISSA_WIDTH-1:0] q_sat [0:ARRAY_SIZE-1];
integer n;
always @(*) begin
    for (n = 0; n < ARRAY_SIZE; n = n + 1) begin
        // round-to-nearest: add half a step before the arithmetic shift
        q_wide[n] = (scale_c == 0) ? resid[n]
                  : ((resid[n] + (ALOGW'(1) <<< (scale_c - 1))) >>> scale_c);
        if      (q_wide[n] >  LMAX) q_sat[n] = LMAX[BLOCK_MANTISSA_WIDTH-1:0];
        else if (q_wide[n] <  LMIN) q_sat[n] = LMIN[BLOCK_MANTISSA_WIDTH-1:0];
        else                        q_sat[n] = q_wide[n][BLOCK_MANTISSA_WIDTH-1:0];
    end
end

// ---------------------------------------------------------------------------
// FSM
// ---------------------------------------------------------------------------
integer p;
always @(posedge clk or negedge rst_n) begin
    if (!rst_n) begin
        state              <= ST_EXP;
        idx                <= 0;
        alog_sum           <= 0;
        nz_cnt             <= 0;
        blk_rdy_q          <= 0;
        ovf_q              <= 1'b0;
        unf_q              <= 1'b0;
        max_exp_calculated <= 0;
        E_q                <= 0;
        scale_q            <= 0;
        mant_q             <= 0;
        sign_q             <= 0;
        zero_q             <= 0;
        for (p = 0; p < ARRAY_SIZE; p = p + 1) begin
            alog[p]  <= 0;
            sgn_r[p] <= 1'b0;
            zer_r[p] <= 1'b1;
        end
    end else begin
        blk_rdy_q <= 1'b0;

        case (state)
        // iface sends an exponent pre-pass before the mantissas; this encoder
        // does not need it (it uses the per-element exp_m that arrives WITH
        // each mantissa), but iface waits on max_exp_calculated to advance.
        ST_EXP: begin
            if (exp_valid && last_exp_sent) begin
                max_exp_calculated <= 1'b1;
                idx                <= 0;
                alog_sum           <= 0;
                nz_cnt             <= 0;
                state              <= ST_COLLECT;
            end else begin
                max_exp_calculated <= 1'b0;
            end
        end

        ST_COLLECT: begin
            max_exp_calculated <= 1'b0;
            if (mantissa_valid) begin
                alog [idx[LIDX-1:0]]   <= alog0;
                sgn_r[idx[LIDX-1:0]]   <= s0;
                zer_r[idx[LIDX-1:0]]   <= z0;
                alog [idx[LIDX-1:0]+1'b1] <= alog1;
                sgn_r[idx[LIDX-1:0]+1'b1] <= s1;
                zer_r[idx[LIDX-1:0]+1'b1] <= z1;

                alog_sum <= alog_sum + (z0 ? ALOGW'(0) : alog0)
                                     + (z1 ? ALOGW'(0) : alog1);
                nz_cnt   <= nz_cnt   + (z0 ? (LIDX+1)'(0) : (LIDX+1)'(1))
                                     + (z1 ? (LIDX+1)'(0) : (LIDX+1)'(1));

                if (all_mantissas_sent) begin
                    idx   <= 0;
                    state <= ST_ENCODE;
                end else begin
                    idx <= idx + 2;
                end
            end
        end

        // one cycle to let alog[]/alog_sum/nz_cnt settle, then latch the block
        ST_ENCODE: begin
            E_q     <= E_c;
            ovf_q   <= e_ovf;
            unf_q   <= e_unf;
            scale_q <= scale_c;
            for (p = 0; p < ARRAY_SIZE; p = p + 1) begin
                mant_q[p*BLOCK_MANTISSA_WIDTH +: BLOCK_MANTISSA_WIDTH]
                        <= zer_r[p] ? {BLOCK_MANTISSA_WIDTH{1'b0}}
                                    : q_sat[p][BLOCK_MANTISSA_WIDTH-1:0];
                sign_q[p] <= sgn_r[p];
                zero_q[p] <= zer_r[p];
            end
            blk_rdy_q <= 1'b1;
            state     <= ST_EXP;
        end

        default: state <= ST_EXP;
        endcase
    end
end

// ---------------------------------------------------------------------------
// outputs
// ---------------------------------------------------------------------------
assign block_out = { {(BLOCK_WIDTH-BLOCK_BITS){1'b0}},
                     E_q, scale_q, sign_q, zero_q, mant_q };

assign max_exp      = E_q;
assign scale_factor = scale_q;
assign mantissa_out = mant_q;
assign sign_out     = sign_q;
assign zero_out     = zero_q;
assign block_ready  = blk_rdy_q;
assign exp_overflow  = ovf_q;
assign exp_underflow = unf_q;

endmodule

