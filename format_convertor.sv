// HARI OM

`timescale 1ns/1ps

//=============================================================================
// format_convertor.sv  --  PE results  ->  one output block
//
// WHAT IT DOES
//   Takes the ARRAY_SIZE results of one column of the array and packs them into
//   one block, in exactly the same format as the input blocks, so a result can
//   be fed back in as an operand.
//
// INPUT (from iface, two elements per clock)
//   Each PE result is the pair (Res_out, exp_out). Its real value is
//       v = Res_out / 2^FRAC_WIDTH * 2^exp_out
//   Res_out is signed, DATA_WIDTH+1 bits; exp_out is signed, EXP_WIDTH+1 bits.
//
// OUTPUT BLOCK  (same as the input block)
//   v = (-1)^sign * 2^(E + L * 2^scale / 2^FRAC_WIDTH),   or 0 if zero = 1
//
//     block[MANT_LSB  +: ARRAY_SIZE*BLOCK_MANTISSA_WIDTH]  L of each element, signed
//     block[ZERO_LSB  +: ARRAY_SIZE]                       zero flag of each element
//     block[SIGN_LSB  +: ARRAY_SIZE]                       sign of each element (1 = negative)
//     block[SCALE_LSB +: SCALE_FACTOR_WIDTH]               shared scale
//     block[EXP_LSB   +: EXP_WIDTH]                        shared exponent E, signed
//
// HOW (all logs are fixed point with FRAC_WIDTH fraction bits)
//   1. For each element: sign, zero flag, and its absolute log
//          alog = log2|v| = log2|Res_out| - FRAC_WIDTH + exp_out
//   2. E = round(average of alog over the non-zero elements).
//      If E does not fit EXP_WIDTH bits it is clamped and exp_overflow /
//      exp_underflow is set.
//   3. Residual of each element:  resid = alog - E * 2^FRAC_WIDTH
//   4. scale = the smallest s for which every residual fits in L after
//      dividing by 2^s.
//   5. L = round(resid / 2^scale), saturated to L_MIN..L_MAX.
//
// FSM:  ST_COLLECT -> ST_ENCODE -> ST_COLLECT   (once per column)
//   ST_COLLECT : wait for results (mantissa_valid); take two per clock, store
//                sign / zero / alog, keep the running sum and count of the
//                non-zero ones; all_mantissas_sent marks the last pair
//   ST_ENCODE  : compute E, scale and L (combinational), register the block,
//                pulse block_ready for one clock, clear the column state
//
// LIMITATIONS / NOTES
//   - log2|Res_out| uses Mitchell's method (log2(1+x) ~ x): it is up to
//     0.086 octave too low, so a decoded value can be about 6% too small.
//   - If even scale = SCALE_MAX cannot fit a residual, that L saturates.
//   - ARRAY_SIZE must be even (results arrive in pairs) and at most 4: only
//     the divides by n = 1..4 are built (see STEP 2).
//   - Each element's exponent arrives with it on exp_m_1 / exp_m_2.
//=============================================================================
module format_convertor #(
    parameter ARRAY_SIZE = 4,             // elements per block (= rows of the array)
    parameter BLOCK_MANTISSA_WIDTH = 6,   // bits of each L (signed), same as the input block
    parameter DATA_WIDTH = 32,            // Res_out has DATA_WIDTH+1 bits
    parameter FRAC_WIDTH = 8,             // fraction bits of every log value
    parameter EXP_WIDTH = 8,              // bits of the block exponent E (signed)
    parameter SCALE_FACTOR_WIDTH = 4,     // bits of the block scale
    // ---- block geometry, identical to edge_reg_bank ----
    parameter MANT_BUS_WIDTH = ARRAY_SIZE*BLOCK_MANTISSA_WIDTH,   // all L fields together
    parameter BLOCK_BITS     = MANT_BUS_WIDTH + 2*ARRAY_SIZE
                               + SCALE_FACTOR_WIDTH + EXP_WIDTH,  // bits actually used
    parameter BLOCK_WIDTH    = ((BLOCK_BITS + 7)/8)*8             // rounded up to whole bytes
)(
    input clk,
    input rst_n,
    // ---- two results per clock, from iface ----
    input signed [DATA_WIDTH:0] mant0,          // Res_out of element idx
    input signed [DATA_WIDTH:0] mant1,          // Res_out of element idx+1
    input signed [EXP_WIDTH:0] exp_m_1,         // exp_out of element idx
    input signed [EXP_WIDTH:0] exp_m_2,         // exp_out of element idx+1
    input mantissa_valid,                       // mant0/mant1/exp_m_* are valid this clock
    input all_mantissas_sent,                   // this is the last pair of the column

    // ---- the output block ----
    output [BLOCK_WIDTH-1:0] block_out,

    // ---- the same block, field by field ----
    output signed [EXP_WIDTH-1:0]            max_exp,       // E
    output [SCALE_FACTOR_WIDTH-1:0]          scale_factor,  // scale
    output [MANT_BUS_WIDTH-1:0]              mantissa_out,  // all L, signed, element 0 in the LSBs
    output [ARRAY_SIZE-1:0]                  sign_out,
    output [ARRAY_SIZE-1:0]                  zero_out,
    output block_ready,                     // one-clock pulse: block_out is new
    // E did not fit EXP_WIDTH bits and was clamped (status, not part of the block;
    // held until the next block)
    output exp_overflow,                    // E was too large, set to E_MAX
    output exp_underflow                    // E was too small, set to E_MIN
);

// ---- bit position of each field inside the block (same as edge_reg_bank) ----
localparam MANT_LSB  = 0;
localparam ZERO_LSB  = MANT_LSB  + MANT_BUS_WIDTH;
localparam SIGN_LSB  = ZERO_LSB  + ARRAY_SIZE;
localparam SCALE_LSB = SIGN_LSB  + ARRAY_SIZE;

// ---- widths ----
// LOGW : width of |Res_out| (the magnitude of a DATA_WIDTH+1-bit signed value)
localparam LOGW   = DATA_WIDTH + 1;
// LIDX : bits needed to address one element of the block (log2 of ARRAY_SIZE).
//        Counters that must reach ARRAY_SIZE itself (idx, nz_cnt) use LIDX+1.
localparam LIDX   = $clog2(ARRAY_SIZE);
// LGW  : bits that log_finder's output really uses. log2|Res_out| < LOGW, so
//        its integer part needs $clog2(LOGW) bits, plus FRAC_WIDTH fraction
//        bits, plus one spare.
localparam LGW    = $clog2(LOGW) + FRAC_WIDTH + 1;
// ALOGW: signed width used for EVERY log quantity below (alog, their sum, the
//        mean, residuals, scale search). It must hold the sum of ARRAY_SIZE logs:
//          |alog| < 2^EXP_WIDTH + LOGW octaves   (exp_out range + log2|Res_out|)
//          times 2^FRAC_WIDTH (fixed point)      -> + FRAC_WIDTH bits
//          summed over ARRAY_SIZE elements       -> + LIDX bits
//          sign + 1 bit of headroom              -> + 2 bits
localparam ALOGW  = FRAC_WIDTH + $clog2((1 << EXP_WIDTH) + LOGW) + LIDX + 2;
// L_MAX / L_MIN of a signed BLOCK_MANTISSA_WIDTH field, at width ALOGW for comparisons
localparam signed [ALOGW-1:0] LMAX =  (1 << (BLOCK_MANTISSA_WIDTH-1)) - 1;
localparam signed [ALOGW-1:0] LMIN = -(1 << (BLOCK_MANTISSA_WIDTH-1));

localparam ST_COLLECT = 2'd1, ST_ENCODE = 2'd2;
reg [1:0] state;

// ---- per-column storage, filled in ST_COLLECT ----
reg [LIDX:0]               idx;                     // next element slot to write (steps by 2)
reg signed [ALOGW-1:0]     alog  [0:ARRAY_SIZE-1];  // absolute log of each element, log2|v|, fixed point
reg                        sgn_r [0:ARRAY_SIZE-1];  // sign of each element
reg                        zer_r [0:ARRAY_SIZE-1];  // zero flag of each element
reg signed [ALOGW-1:0]     alog_sum;                // sum of alog over the non-zero elements
reg [LIDX:0]               nz_cnt;                  // number of non-zero elements, 0..ARRAY_SIZE
reg                        blk_rdy_q;               // block_ready pulse
reg                        ovf_q, unf_q;            // exp_overflow / exp_underflow, held with the block

// ---- output block fields, registered in ST_ENCODE ----
reg signed [EXP_WIDTH-1:0]       E_q;
reg [SCALE_FACTOR_WIDTH-1:0]     scale_q;
reg [MANT_BUS_WIDTH-1:0]         mant_q;
reg [ARRAY_SIZE-1:0]             sign_q, zero_q;

// ---------------------------------------------------------------------------
// STEP 1: sign, zero flag and absolute log of the two incoming elements
//   sN : sign = top bit of Res_out
//   zN : zero flag = Res_out is exactly 0 (log2(0) does not exist)
//   aN : |Res_out|  (two's complement negate when negative; the log circuit
//        needs the magnitude)
//   lgN: log2|Res_out| in fixed point, from log_finder (Mitchell's method)
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

// alog = log2 of the element's real value, in FRAC_WIDTH fixed point:
//   v       = Res_out / 2^FRAC_WIDTH * 2^exp_out
//   log2|v| = log2|Res_out| - FRAC_WIDTH + exp_out
// lg is the first term; (exp_out - FRAC_WIDTH) is an integer, so it is
// shifted left by FRAC_WIDTH to line it up with the fixed-point fraction.
// Example (FRAC_WIDTH = 8): Res_out = 1280, exp_out = 2  ->  v = 5 * 4 = 20.
//   lg = 2624 (10.25),  (2 - 8) << 8 = -1536,  alog = 1088 = 4.25  (exact 4.32)
function automatic signed [ALOGW-1:0] mk_alog(input [LOGW-1:0] lg,
                                             input signed [EXP_WIDTH:0] em);
    logic signed [ALOGW-1:0] lg_s, em_s;
    begin
        lg_s   = $signed({{(ALOGW-LGW){1'b0}}, lg[LGW-1:0]});  // log2|Res_out| >= 0
        em_s   = ALOGW'(em);                                    // sign-extended exp_out
        mk_alog = lg_s + ((em_s - ALOGW'(FRAC_WIDTH)) <<< FRAC_WIDTH);
    end
endfunction

wire signed [ALOGW-1:0] alog0 = mk_alog(lg0, exp_m_1);
wire signed [ALOGW-1:0] alog1 = mk_alog(lg1, exp_m_2);

// ---------------------------------------------------------------------------
// STEP 2: E = round(mean of alog over the non-zero elements)
//
// The mean needs alog_sum / n, where n = nz_cnt (0..ARRAY_SIZE). There is no
// divider and no multiplier: one result is built for every possible n with
// shifts and adds only, and nz_cnt selects one. All results round down
// (floor), like a >>> shift.
//
// HOW x / n IS DONE WITH SHIFTS            (x = alog_sum, sign-extended to PW)
//   n = 2^k  : x / n = x >>> k                               exact, no gates
//   n odd    : x / n ~= (x * R) >>> 16,   R = floor(2^16 / n)
//              x * R is written as a few shifted copies of x added together
//   n = m*2^k: (m odd) do x / m, then shift right k more: (x * R(m)) >>> (16+k)
//
// TABLE for n = 1 .. 16   ("err" = how much R / 2^16 is below 1/n, relative;
//                          the mean comes out at most |mean| * err low, plus
//                          1 unit from the floor)
//   n  R      x * R as shifts and adds                         adds  err
//   1  -      x                                                  0   exact
//   2  -      x >>> 1                                            0   exact
//   3  21845  m1 = x+(x<<<2); m2 = m1+(m1<<<4); m3 = m2+(m2<<<8)  3   0.0015%
//             (21845 = 5*17*257), then >>> 16
//   4  -      x >>> 2                                            0   exact
//   5  13107  m1 = x+(x<<<1); m2 = m1+(m1<<<4); m3 = m2+(m2<<<8)  3   0.0015%
//             (13107 = 3*17*257), then >>> 16
//   6  -      (x / 3 product m3) >>> 17                          3   0.0015%
//   7  9362   (x<<<13)+(x<<<10)+(x<<<7)+(x<<<4)+(x<<<1)           4   0.0031%
//             then >>> 16
//   8  -      x >>> 3                                            0   exact
//   9  7281   (x<<<13)-(x<<<10)+(x<<<7)-(x<<<4)+x                 4   0.011%
//             then >>> 16
//  10  -      (x / 5 product m3) >>> 17                          3   0.0015%
//  11  5957   (x<<<13)-(x<<<11)-(x<<<8)+(x<<<6)+(x<<<2)+x         5   0.014%
//             then >>> 16
//  12  -      (x / 3 product m3) >>> 18                          3   0.0015%
//  13  5041   (x<<<12)+(x<<<10)-(x<<<6)-(x<<<4)+x                 4   0.0046%
//             then >>> 16
//  14  -      (x / 7 product) >>> 17                             4   0.0031%
//  15  4369   m1 = x+(x<<<4); m2 = m1+(m1<<<8)                    2   0.0015%
//             (4369 = 17*257), then >>> 16
//  16  -      x >>> 4                                            0   exact
//
// ONLY n = 1..4 ARE BUILT BELOW, so ARRAY_SIZE must be <= 4 for now. A larger
// ARRAY_SIZE stops elaboration with an error; to support it, add the cases
// from the table above.
//   RW : 16, the fraction bits of R (= FRAC_WIDTH + 8)
//   PW : working width, wide enough for x * R
// ---------------------------------------------------------------------------
localparam RW = FRAC_WIDTH + 8;
localparam PW = ALOGW + RW + 2;
wire signed [PW-1:0] mean_cand [0:ARRAY_SIZE];   // mean_cand[n] = alog_sum / n
genvar gi;
generate
    assign mean_cand[0] = '0;            // n = 0 (all elements zero): E is unused
    for (gi = 1; gi <= ARRAY_SIZE; gi = gi + 1) begin : mean_g
        // alog_sum sign-extended to PW bits
        wire signed [PW-1:0] sum_ext = {{(PW-ALOGW){alog_sum[ALOGW-1]}}, alog_sum};
        if (gi == 1) begin : div1
            assign mean_cand[gi] = sum_ext;
        end else if (gi == 2) begin : div2
            assign mean_cand[gi] = sum_ext >>> 1;
        end else if (gi == 3) begin : div3
            // x * 21845 in 3 shift-and-add steps (21845 = 5 * 17 * 257):
            //   m1 = x  * 5   = x  + (x  << 2)
            //   m2 = m1 * 17  = m1 + (m1 << 4)
            //   m3 = m2 * 257 = m2 + (m2 << 8)
            // then >>> 16. Max |m3| < 2^20 * 2^15 = 2^35, inside PW bits.
            wire signed [PW-1:0] m1 = sum_ext + (sum_ext <<< 2);
            wire signed [PW-1:0] m2 = m1      + (m1      <<< 4);
            wire signed [PW-1:0] m3 = m2      + (m2      <<< 8);
            assign mean_cand[gi] = m3 >>> 16;
        end else if (gi == 4) begin : div4
            assign mean_cand[gi] = sum_ext >>> 2;
        end else begin : div_missing
            $error("format_convertor: no divide-by-%0d built (ARRAY_SIZE = %0d). Only n = 1..4 exist; add it from the STEP 2 table.", gi, ARRAY_SIZE);
            assign mean_cand[gi] = '0;
        end
    end
endgenerate
wire signed [ALOGW-1:0] mean_alog = mean_cand[nz_cnt[LIDX:0]][ALOGW-1:0];   // mean log, fixed point
// round to the nearest integer: add 0.5, then drop the fraction bits
wire signed [ALOGW-1:0] mean_rnd  = mean_alog + ALOGW'(1 << (FRAC_WIDTH-1));
wire signed [ALOGW-1:0] mean_int  = mean_rnd >>> FRAC_WIDTH;               // E before clamping

// E must fit EXP_WIDTH signed bits: E_MIN = -2^(EXP_WIDTH-1) .. E_MAX = 2^(EXP_WIDTH-1)-1.
// The PE exponents are EXP_WIDTH+1 bits, so a column's mean can be outside that
// range. It is then clamped to the nearest end (not wrapped) and flagged; the
// block's values are only approximate in that case.
localparam signed [ALOGW-1:0] EMAX_W =  (1 << (EXP_WIDTH-1)) - 1;
localparam signed [ALOGW-1:0] EMIN_W = -(1 << (EXP_WIDTH-1));
wire e_ovf = (mean_int > EMAX_W);
wire e_unf = (mean_int < EMIN_W);
wire signed [EXP_WIDTH-1:0] E_c   = e_ovf ? {1'b0, {(EXP_WIDTH-1){1'b1}}}     // E_MAX
                                  : e_unf ? {1'b1, {(EXP_WIDTH-1){1'b0}}}     // E_MIN
                                  : mean_int[EXP_WIDTH-1:0];

// ---------------------------------------------------------------------------
// STEP 3: residual of each element = how far its log is from E (fixed point)
//   resid = alog - E * 2^FRAC_WIDTH
// ---------------------------------------------------------------------------
wire signed [ALOGW-1:0] resid [0:ARRAY_SIZE-1];
genvar g;
generate
    for (g = 0; g < ARRAY_SIZE; g = g + 1) begin : res_g
        assign resid[g] = alog[g] - (ALOGW'(E_c) <<< FRAC_WIDTH);
    end
endgenerate

// ---------------------------------------------------------------------------
// STEP 4: scale = the smallest s for which every residual fits in L
//
// After dividing by 2^s, L must lie in L_MIN..L_MAX. The signed field reaches
// one step further below zero than above it, so each side is checked
// against its own limit:
//     largest positive residual            <= L_MAX  * 2^s
//     largest negative residual (magnitude) <= |L_MIN| * 2^s
// Smallest s = finest step = least rounding error.
// If nothing fits, scale = SCALE_MAX and STEP 5 saturates the out-of-range L.
//   need_p : largest positive residual of the non-zero elements
//   need_n : largest negative residual, as a positive number
// ---------------------------------------------------------------------------
reg signed [ALOGW-1:0] need_p;
reg signed [ALOGW-1:0] need_n;
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
    scale_c = {SCALE_FACTOR_WIDTH{1'b1}};          // SCALE_MAX if nothing fits
    for (sidx = (1 << SCALE_FACTOR_WIDTH) - 1; sidx >= 0; sidx = sidx - 1)
        if ((need_p <= ALOGW'(((1 << (BLOCK_MANTISSA_WIDTH-1)) - 1) << sidx)) &&
            (need_n <= ALOGW'(  (1 << (BLOCK_MANTISSA_WIDTH-1))      << sidx)))
            scale_c = sidx[SCALE_FACTOR_WIDTH-1:0];
end

// ---------------------------------------------------------------------------
// STEP 5: L = round(resid / 2^scale), saturated to L_MIN..L_MAX
//   q_wide : the rounded quotient at full width (add half a step, then shift)
//   q_sat  : q_wide limited to the L field
// Rounding to nearest keeps the error within half a step:
//   |log error| <= 2^scale / 2^(FRAC_WIDTH+1) octave
// ---------------------------------------------------------------------------
reg signed [ALOGW-1:0]              q_wide [0:ARRAY_SIZE-1];
reg signed [BLOCK_MANTISSA_WIDTH-1:0] q_sat [0:ARRAY_SIZE-1];
integer n;
always @(*) begin
    for (n = 0; n < ARRAY_SIZE; n = n + 1) begin
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
        state              <= ST_COLLECT;
        idx                <= 0;
        alog_sum           <= 0;
        nz_cnt             <= 0;
        blk_rdy_q          <= 0;
        ovf_q              <= 1'b0;
        unf_q              <= 1'b0;
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
        // two elements per clock: store them, add the non-zero ones to the sum
        ST_COLLECT: begin
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

        // the stored column is complete: register the block computed by STEPS 2-5
        ST_ENCODE: begin
            E_q     <= E_c;
            ovf_q   <= e_ovf;
            unf_q   <= e_unf;
            scale_q <= scale_c;
            for (p = 0; p < ARRAY_SIZE; p = p + 1) begin
                mant_q[p*BLOCK_MANTISSA_WIDTH +: BLOCK_MANTISSA_WIDTH]
                        <= zer_r[p] ? {BLOCK_MANTISSA_WIDTH{1'b0}}      // zero element: L = 0
                                    : q_sat[p][BLOCK_MANTISSA_WIDTH-1:0];
                sign_q[p] <= sgn_r[p];
                zero_q[p] <= zer_r[p];
            end
            blk_rdy_q <= 1'b1;
            // clear the column state for the next column
            idx       <= 0;
            alog_sum  <= 0;
            nz_cnt    <= 0;
            state     <= ST_COLLECT;
        end

        default: state <= ST_COLLECT;
        endcase
    end
end

// ---------------------------------------------------------------------------
// outputs (unused top bits of the byte-aligned block are 0)
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
