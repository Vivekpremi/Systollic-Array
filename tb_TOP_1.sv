`timescale 1ns/1ps
//=============================================================================
// tb_TOP_1.sv  --  main testbench of the core (top_module), 140 tests
//
// WHAT IT DOES
//   Each test is a 4x4 A and a 4x4 B of real numbers written out explicitly.
//   For each test:
//     reset -> encode A's rows and B's columns into blocks (make_block, the
//     same encoding the software driver must use) -> load all 8 blocks in one
//     clock -> wait 200 clocks -> check -> report
//
// BLOCK ENCODING (make_block)
//   sign = v < 0, zero = v == 0
//   E    = round(mean of log2|v| over the non-zero elements)
//   L    = round((round(log2|v| * 2^FRAC_WIDTH) - E * 2^FRAC_WIDTH) / 2^scale),
//          scale = the smallest that fits every L in BLOCK_MANTISSA_WIDTH bits
//   Row lane i gets row i of A; column lane j gets column j of B.
//
// CHECKS (a test FAILS if any fails)
//   Res_out   : C[i][j] = Res_out / 2^FRAC_WIDTH * 2^(E_row + E_col), compared
//               with the exact A x B; error / sum|terms| must be <= 10%
//               (this measure is not fooled by cancellation)
//   exponent  : the hardware exp_out must equal E_row + E_col
//   blocks    : the 4 output blocks, decoded, must be within 10% of the
//               largest |C| in their column, and exp_overflow / underflow
//               must match an ideal software encoding of the exact C
//               (E / scale / L differences are reported, not failed: the
//               hardware log is approximate)
//   overflow  : every PE whose inputs must overflow must raise pe_overflow
//
// OUTPUT
//   terminal       : one line per test + a summary
//   test_report.txt: matrices, results, errors, block check, per test
//   TOP_1_tb.vcd   : waveform
//
// TESTS
//   1-26 hand-made (powers of two, near 1, mid range, wide range, signs),
//   27-136 random families and probes (benign, near 1, below 1, signed,
//   NN-like weights, heavy tails, wide 1e-2..1e2 and 1e-4..1e4, spread inside
//   a block, small-product margins, cancellation, overflow probes),
//   137-140 columns with 1, 2 and 3 non-zero results (every divide path of
//   format_convertor).
//   Expected at the default parameters: 125 pass. The 15 failures (62, 65,
//   78, 80-93) have a very wide range of values inside one block (wide-range
//   and heavy-tail data), more than a 6-bit L can hold accurately.
//
// LIMITATION: the stimulus is 4x4, so ARRAY_SIZE must be 4.
//=============================================================================
module tb_TOP_1;
    parameter DATA_WIDTH=32, EXP_WIDTH=8, MANTISSA_WIDTH=16, FRAC_WIDTH=8, ARRAY_SIZE=4;
    parameter BLOCK_MANTISSA_WIDTH = 6, SCALE_FACTOR_WIDTH=4;
    localparam MBW = ARRAY_SIZE*BLOCK_MANTISSA_WIDTH;   // 24 (input block L field)
    localparam BLOCK_BITS = MBW + 2*ARRAY_SIZE + SCALE_FACTOR_WIDTH + EXP_WIDTH;
    localparam BW  = ((BLOCK_BITS + 7)/8)*8;
    localparam MANT_LSB  = 0;
    localparam ZERO_LSB  = MANT_LSB  + MBW;
    localparam SIGN_LSB  = ZERO_LSB  + ARRAY_SIZE;
    localparam SCALE_LSB = SIGN_LSB  + ARRAY_SIZE;
    localparam EXP_LSB   = SCALE_LSB + SCALE_FACTOR_WIDTH;
    localparam LMAX =  (1<<(BLOCK_MANTISSA_WIDTH-1)) - 1;
    localparam LMIN = -(1<<(BLOCK_MANTISSA_WIDTH-1));
    localparam LOG_ONE = (1<<FRAC_WIDTH);           // 1.0 in the log fixed point

    reg clk, rst_n;
    reg row_we [ARRAY_SIZE-1:0], col_we [ARRAY_SIZE-1:0];
    reg [BW-1:0] row_block [ARRAY_SIZE-1:0], col_block [ARRAY_SIZE-1:0];

    wire [ARRAY_SIZE-1:0] valid_out [ARRAY_SIZE-1:0];
    wire valid_man_out, mantissas_sent_out;
    wire block_ready, bank_primed;
    wire [EXP_WIDTH:0]   exp_m_out_1, exp_m_out_2;   // per-PE exponent path: EXP_WIDTH+1
    wire [EXP_WIDTH-1:0] max_exp;                                                      // block E field: EXP_WIDTH
    wire signed [DATA_WIDTH:0] mantissa_out_1, mantissa_out_2;
    wire set_i_ready [ARRAY_SIZE-1:0];
    wire signed [DATA_WIDTH:0] Res_out [ARRAY_SIZE-1:0][ARRAY_SIZE-1:0];
    wire [SCALE_FACTOR_WIDTH-1:0] scale_factor;
    wire [MBW-1:0] mantissa_out;              // L[], now TWO'S COMPLEMENT
    wire [ARRAY_SIZE-1:0] sign_out, zero_out;  // per-element sign / zero
    wire [BW-1:0] block_out;                   // SAME format as row_block/col_block
    wire exp_overflow, exp_underflow;          // output block E clamped (out of range)
    wire [ARRAY_SIZE*ARRAY_SIZE-1:0] pe_overflow;  // sticky per-PE overflow, PE[i][j] at i*ARRAY_SIZE+j
    wire acc_overflow;                         // OR of pe_overflow
    wire signed [EXP_WIDTH:0] exp_out_hw [ARRAY_SIZE-1:0][ARRAY_SIZE-1:0];   // per-PE exponent from HW

    top_module #(
        .MANTISSA_WIDTH       (MANTISSA_WIDTH),
        .FRAC_WIDTH           (FRAC_WIDTH),
        .EXP_WIDTH            (EXP_WIDTH),
        .DATA_WIDTH           (DATA_WIDTH),
        .ARRAY_SIZE           (ARRAY_SIZE),
        .BLOCK_MANTISSA_WIDTH (BLOCK_MANTISSA_WIDTH),
        .SCALE_FACTOR_WIDTH   (SCALE_FACTOR_WIDTH)
    ) uut (
        .clk(clk), .rst_n(rst_n),
        .row_we(row_we), .col_we(col_we), .row_block(row_block), .col_block(col_block),
        .valid_out(valid_out),
        .mantissa_out_1(mantissa_out_1), .mantissa_out_2(mantissa_out_2),
        .exp_m_out_1(exp_m_out_1), .exp_m_out_2(exp_m_out_2),
        .valid_man_out(valid_man_out), .set_i_ready(set_i_ready),
        .Res_out(Res_out),
        .mantissas_sent_out(mantissas_sent_out),
        .block_ready(block_ready), .scale_factor(scale_factor), .max_exp(max_exp),
        .mantissa_out(mantissa_out),
        .block_out(block_out), .sign_out(sign_out), .zero_out(zero_out),
        .exp_overflow(exp_overflow), .exp_underflow(exp_underflow),
        .pe_overflow(pe_overflow), .acc_overflow(acc_overflow),
        .exp_out(exp_out_hw),
        .bank_primed(bank_primed)
    );

    initial begin clk=0; forever #5 clk=~clk; end

    real Afp [0:3][0:3], Bfp [0:3][0:3], Cexp [0:3][0:3];
    integer i,j,k;
    integer ntests, nfail;
    real    acc_sum;
    reg [SCALE_FACTOR_WIDTH-1:0] sf_row, sf_col;
    integer E_row [0:3], E_col [0:3];   // shared exponent chosen per lane

    // ---------------- block encoder (the software driver must do the same) ----------------
    // v = (-1)^sign * 2^(E + L * 2^scale / 2^FRAC_WIDTH)
    //   E = round(mean(log2|v|)) over the non-zero elements (shared, signed)
    //   L = (log2|v| - E) in FRAC_WIDTH fixed point, divided by 2^scale and rounded
    function real absr(input real v); begin absr = (v<0.0) ? -v : v; end endfunction

    function integer log8p8(input real v);   // round(log2(v)*256), v>0
        real l;
        begin
            if (v <= 0.0) log8p8 = 0;
            else begin l = $ln(v)/$ln(2.0); log8p8 = $rtoi(l*$itor(LOG_ONE) + ((l<0.0)?-0.5:0.5)); end
        end
    endfunction

    function integer sdiv_round(input integer x, input integer s); // round(x / 2^s), signed
        integer half;
        begin
            half = (s==0) ? 0 : (1<<(s-1));
            sdiv_round = (x >= 0) ? ((x + half) >>> s) : -(((-x) + half) >>> s);
        end
    endfunction

    // pick_scale_m: smallest scale s for which every L fits LMIN..LMAX.
    // Zero elements (act = 0) do not count: their L is a dummy 0.
    // SMAX: largest scale the SCALE_FACTOR_WIDTH field can hold.
    localparam SMAX = (1 << SCALE_FACTOR_WIDTH) - 1;

    function integer pick_scale_m(input integer l0,l1,l2,l3, input [3:0] act);
        integer s,m0,m1,m2,m3;
        reg     found;
        begin
            pick_scale_m = SMAX; found = 1'b0;
            for (s=0; s<=SMAX; s=s+1) begin
                m0=sdiv_round(l0,s); m1=sdiv_round(l1,s);
                m2=sdiv_round(l2,s); m3=sdiv_round(l3,s);
                if ( (!act[0] || (m0>=LMIN && m0<=LMAX)) &&
                     (!act[1] || (m1>=LMIN && m1<=LMAX)) &&
                     (!act[2] || (m2>=LMIN && m2<=LMAX)) &&
                     (!act[3] || (m3>=LMIN && m3<=LMAX)) && !found) begin
                    pick_scale_m = s; found = 1'b1;
                end
            end
        end
    endfunction

    function integer pick_scale(input integer l0,l1,l2,l3);
        integer s,m0,m1,m2,m3;
        reg     found;
        begin
            pick_scale = SMAX; found = 1'b0;
            for (s=0; s<=SMAX; s=s+1) begin
                m0=sdiv_round(l0,s); m1=sdiv_round(l1,s);
                m2=sdiv_round(l2,s); m3=sdiv_round(l3,s);
                if (m0>=LMIN && m0<=LMAX && m1>=LMIN && m1<=LMAX &&
                    m2>=LMIN && m2<=LMAX && m3>=LMIN && m3<=LMAX && !found) begin
                    pick_scale = s; found = 1'b1;
                end
            end
        end
    endfunction

    // E is the mean of log2|v| over NON-ZERO elements only.  Including a zero
    // would feed log2(0) = -inf into the mean and yield INT_MIN.
    function integer calc_E(input real v0,v1,v2,v3);
        real sumlog; integer n;
        begin
            sumlog = 0.0; n = 0;
            if (v0 != 0.0) begin sumlog = sumlog + $ln(absr(v0))/$ln(2.0); n = n+1; end
            if (v1 != 0.0) begin sumlog = sumlog + $ln(absr(v1))/$ln(2.0); n = n+1; end
            if (v2 != 0.0) begin sumlog = sumlog + $ln(absr(v2))/$ln(2.0); n = n+1; end
            if (v3 != 0.0) begin sumlog = sumlog + $ln(absr(v3))/$ln(2.0); n = n+1; end
            if (n == 0) calc_E = 0;                      // all-zero block: E unused
            else begin
                sumlog = sumlog / n;
                calc_E = $rtoi(sumlog + ((sumlog<0.0)?-0.5:0.5));
            end
        end
    endfunction

    function [BW-1:0] make_block(input real v0,v1,v2,v3);
        integer l0,l1,l2,l3,s,m0,m1,m2,m3,E;
        real meanlog;
        reg [3:0] sv, zf;
        begin
            // value signs
            sv[0] = (v0<0.0); sv[1] = (v1<0.0); sv[2] = (v2<0.0); sv[3] = (v3<0.0);
            zf[0] = (v0==0.0); zf[1] = (v1==0.0); zf[2] = (v2==0.0); zf[3] = (v3==0.0);
            // shared exponent E = round(mean of log2|v|)
            E = calc_E(v0,v1,v2,v3);
            // residual logs
            // zero elements get a dummy L of 0; the z flag is what matters
            l0 = zf[0] ? 0 : (log8p8(absr(v0)) - E*LOG_ONE);
            l1 = zf[1] ? 0 : (log8p8(absr(v1)) - E*LOG_ONE);
            l2 = zf[2] ? 0 : (log8p8(absr(v2)) - E*LOG_ONE);
            l3 = zf[3] ? 0 : (log8p8(absr(v3)) - E*LOG_ONE);
            s  = pick_scale_m(l0,l1,l2,l3, ~zf);   // zeros excluded from the fit
            m0=sdiv_round(l0,s); m1=sdiv_round(l1,s);
            m2=sdiv_round(l2,s); m3=sdiv_round(l3,s);
            make_block = '0;
            make_block[MANT_LSB + 0*BLOCK_MANTISSA_WIDTH +: BLOCK_MANTISSA_WIDTH] = m0[BLOCK_MANTISSA_WIDTH-1:0];
            make_block[MANT_LSB + 1*BLOCK_MANTISSA_WIDTH +: BLOCK_MANTISSA_WIDTH] = m1[BLOCK_MANTISSA_WIDTH-1:0];
            make_block[MANT_LSB + 2*BLOCK_MANTISSA_WIDTH +: BLOCK_MANTISSA_WIDTH] = m2[BLOCK_MANTISSA_WIDTH-1:0];
            make_block[MANT_LSB + 3*BLOCK_MANTISSA_WIDTH +: BLOCK_MANTISSA_WIDTH] = m3[BLOCK_MANTISSA_WIDTH-1:0];
            make_block[ZERO_LSB  +: ARRAY_SIZE] = zf;          // zero flags
            make_block[SIGN_LSB  +: ARRAY_SIZE] = sv;
            make_block[SCALE_LSB +: SCALE_FACTOR_WIDTH] = s[SCALE_FACTOR_WIDTH-1:0];
            make_block[EXP_LSB   +: EXP_WIDTH] = E[EXP_WIDTH-1:0];       // signed, two's complement
        end
    endfunction

    task clrwe; integer w; begin for(w=0;w<4;w=w+1) begin row_we[w]=0; col_we[w]=0; end end endtask

    // ---------------- one test: reset, load, multiply, check ----------------
    task run_test(input integer id, input [8*44:1] name);
        real tot_err, a, e, d, sacc, cnd, wcond, tm;
        integer bad, sflip;
        string bad_str;
        begin
            for(i=0;i<4;i=i+1) for(j=0;j<4;j=j+1) begin
                Cexp[i][j]=0.0;
                for(k=0;k<4;k=k+1) Cexp[i][j]=Cexp[i][j]+Afp[i][k]*Bfp[k][j];
            end

            // reset before every multiply
            rst_n=0; clrwe;
            for(i=0;i<4;i=i+1) begin row_block[i]=0; col_block[i]=0; end
            repeat(1) @(posedge clk);
            rst_n=1;

            for(i=0;i<4;i=i+1) begin
                row_block[i] = make_block(Afp[i][0],Afp[i][1],Afp[i][2],Afp[i][3]);
                col_block[i] = make_block(Bfp[0][i],Bfp[1][i],Bfp[2][i],Bfp[3][i]);
                E_row[i]     = calc_E(Afp[i][0],Afp[i][1],Afp[i][2],Afp[i][3]);
                E_col[i]     = calc_E(Bfp[0][i],Bfp[1][i],Bfp[2][i],Bfp[3][i]);
            end
            sf_row = row_block[0][SCALE_LSB +: SCALE_FACTOR_WIDTH];
            sf_col = col_block[0][SCALE_LSB +: SCALE_FACTOR_WIDTH];
            for(i=0;i<4;i=i+1) begin row_we[i]=1; col_we[i]=1; end
            @(posedge clk); clrwe;
            if(!bank_primed) $display("  WARN: bank_primed low after load");

            repeat(200) @(posedge clk);

            tot_err=0.0; bad=0; bad_str="";
            wcond=0.0; sflip=0; t_experr=0;
            for(i=0;i<4;i=i+1) for(j=0;j<4;j=j+1) begin
                a = ($itor($signed(Res_out[i][j]))/$itor(LOG_ONE))*(2.0**(E_row[i]+E_col[j]));
                e = Cexp[i][j];
                d = (a>e)?(a-e):(e-a);
                got_m[i][j]=a; abserr[i][j]=d;
                exp_hw[i][j]  = 32'(signed'(exp_out_hw[i][j]));
                exp_ref[i][j] = E_row[i] + E_col[j];
                if (exp_hw[i][j] !== exp_ref[i][j]) t_experr = t_experr + 1;
                sacc = 0.0;
                for(k=0;k<4;k=k+1) begin
                    tm = Afp[i][k]*Bfp[k][j];
                    sacc = sacc + ((tm<0.0)? -tm : tm);
                end
                if (sacc>0.0) begin cnd = d/sacc; if (cnd>wcond) wcond=cnd; end
                if (e*a < 0.0) sflip = sflip + 1;
                if(e>0.5 || e<-0.5) begin
                    tot_err = tot_err + d/((e<0.0)?-e:e);
                    if (d > 0.10*((e<0.0)?-e:e)) bad = bad + 1;
                end
            end
            t_id=id; t_name=name; t_cond=wcond; t_flip=sflip;
            fc_check; pe_ovf_check; account;
            make_txt;
            t_nstr = $sformatf("%0s", name);
            $display("  %4d | %-32s | %8.3f%% | %2d  | %2d  | %8.3f%% | %0s",
                      id, t_nstr, wcond*100.0, sflip, t_experr, t_fc_norm*100.0,
                      t_fail ? "FAIL" : "PASS");
            if (t_experr!=0) s_experr = s_experr + 1;
            if (t_fail) nfail = nfail + 1;
            ntests = ntests + 1;
            acc_sum = acc_sum + 100.0*(1.0-tot_err/16.0);
        end
    endtask



    // ================= REPORTING =================
    // Detailed per-test dump goes to a text file; the terminal gets only a
    // one-line table row per test plus the final summary.
    integer fh;
    real    got_m  [0:3][0:3];     // hardware result (scaled by 2^(Ea+Eb))
    real    abserr [0:3][0:3];     // |got - expected|
    real    t_cond;                // worst condition-aware error for this test
    integer t_flip;                // sign flips in this test
    integer exp_hw [0:3][0:3];     // what the hardware produced
    integer exp_ref[0:3][0:3];     // what it should be (E_row+E_col)
    integer t_experr;              // mismatching exponents this test
    // ---- output-block (format conversion) check state ----
    integer t_fcerr;               // failing block elements + structural errors this test
    real    t_fc_norm;             // worst block error this test
    real    fc_dec [0:3][0:3];     // [elem][block] value decoded from the hardware block
    integer t_fcoor;               // blocks correctly flagged out of range this test
    // IDEAL block, encoded in software from EXPECTED C (exact log2, no hardware
    // approximations) -- [block] for per-block fields, [elem][block] per element
    integer x_E   [0:3], x_S [0:3];
    reg     x_ovf [0:3], x_unf [0:3];
    integer x_L   [0:3][0:3];
    reg     x_sg  [0:3][0:3], x_z [0:3][0:3];
    integer g_L   [0:3][0:3];          // hardware L, unpacked from mantissa_out
    real    fc_nerr [0:3][0:3];        // |decoded - expected C| / max|expected C in column|
    integer m_E, m_S, m_L, m_SG, m_Z, m_OV, m_UN;   // field mismatches this test
    integer t_fcval;                   // values over the 10% tolerance this test
    // ---- PE overflow check state ----
    integer ov_hw   [0:3][0:3];        // pe_overflow from the hardware
    integer ov_need [0:3][0:3];        // 1 if some product MUST overflow (from the input blocks)
    integer t_ovf_hw, t_ovf_miss;      // PEs flagged / must-flag PEs not flagged, this test
    integer c_ovf, c_ovf_miss;         // tests with a flag / tests with a missed flag, whole run
    integer c_oor;                 // blocks flagged out of range, whole run
    // non-zero-count coverage: cov_n[n] = output blocks that had n non-zero
    // elements (hardware zero flags). format_convertor divides by n to get E,
    // with a separate circuit for each n, so every n = 1..4 must be exercised.
    integer cov_n [0:4];
    reg     fc_n_bad;              // set if the number of blocks produced != ARRAY_SIZE
    reg     t_fail;                // this test failed ANY check
    integer c_res, c_exp, c_fc;    // per-check failure counts, whole run
    real    s_fcworst;             // worst block-normalised error, whole run
    reg [8*44:1] t_name;
    string       t_nstr;
    integer t_id;

    // ======================================================================
    // OUTPUT BLOCK CAPTURE
    // format_convertor emits one packed output block per column, flagged by
    // block_ready. Every block is captured as it is produced; fc_n counts how
    // many arrived.
    // ======================================================================
    localparam MAX_BLK = 16;
    integer                       fc_n;
    integer                       fc_exp   [0:MAX_BLK-1];   // max_exp, signed
    integer                       fc_scale [0:MAX_BLK-1];   // scale_factor
    reg [MBW-1:0]                 fc_mant  [0:MAX_BLK-1];   // packed L[], signed
    reg                           fc_ovf   [0:MAX_BLK-1];   // exp_overflow with the block
    reg                           fc_unf   [0:MAX_BLK-1];   // exp_underflow with the block
    reg [ARRAY_SIZE-1:0]          fc_sign  [0:MAX_BLK-1];
    reg [ARRAY_SIZE-1:0]          fc_zero  [0:MAX_BLK-1];
    integer                       fc_time  [0:MAX_BLK-1];   // $time, for ordering

    // async reset, matching the RTL's `negedge rst_n` style (a synchronous
    // reset here makes rst_n both sync and async -> SYNCASYNCNET).
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            fc_n <= 0;
        end else if (block_ready && fc_n < MAX_BLK) begin
            fc_exp  [fc_n] <= 32'(signed'(max_exp));
            fc_scale[fc_n] <= 32'(scale_factor);
            fc_mant [fc_n] <= mantissa_out;
            fc_sign [fc_n] <= sign_out;
            fc_zero [fc_n] <= zero_out;
            fc_ovf  [fc_n] <= exp_overflow;
            fc_unf  [fc_n] <= exp_underflow;
            fc_time [fc_n] <= 32'($time);   // $time is 64-bit; fc_time is integer
            fc_n           <= fc_n + 1;
        end
    end

    // ======================================================================
    // OUTPUT BLOCK CHECK
    // Expected = the IDEAL block, encoded in software from EXPECTED C (the true
    // A x B), exactly like the input blocks are built: exact log2, E =
    // round(mean log2|v|), E clamped to the EXP_WIDTH range (flagged), smallest
    // scale that fits, L rounded to nearest. Block j is column j of C.
    // Got = the block format_convertor produced.
    // PASS/FAIL is decided on VALUES, like the Res_out check: each hardware
    // block is decoded back to numbers and compared with EXPECTED C, the error
    // normalised by the largest |EXPECTED C| in that column, tolerance 10%.
    // exp_overflow / exp_underflow must also match. The other fields are
    // REPORTED, not failed: the hardware log is approximate (Mitchell), so E or
    // L differing by 1 from the ideal is normal.
    // Blocks whose ideal E is out of range are skipped in the value check --
    // they cannot be represented in this format.
    // ======================================================================
    task fc_ideal(input integer b);
        integer m, lg [0:3], emax, emin;
        reg [3:0] zf;
        begin
            emax =  (1 << (EXP_WIDTH-1)) - 1;
            emin = -(1 << (EXP_WIDTH-1));
            for (m = 0; m < 4; m = m + 1) begin
                x_z [m][b] = (Cexp[m][b] == 0.0);
                x_sg[m][b] = (Cexp[m][b] <  0.0);
                zf[m]      = x_z[m][b];
            end
            x_E[b]   = calc_E(Cexp[0][b], Cexp[1][b], Cexp[2][b], Cexp[3][b]);
            x_ovf[b] = (x_E[b] > emax);
            x_unf[b] = (x_E[b] < emin);
            if (x_ovf[b]) x_E[b] = emax;
            if (x_unf[b]) x_E[b] = emin;
            for (m = 0; m < 4; m = m + 1)
                lg[m] = x_z[m][b] ? 0 : (log8p8(absr(Cexp[m][b])) - x_E[b]*LOG_ONE);
            x_S[b] = pick_scale_m(lg[0], lg[1], lg[2], lg[3], ~zf);
            for (m = 0; m < 4; m = m + 1) begin
                x_L[m][b] = x_z[m][b] ? 0 : sdiv_round(lg[m], x_S[b]);
                if (x_L[m][b] > LMAX) x_L[m][b] = LMAX;
                if (x_L[m][b] < LMIN) x_L[m][b] = LMIN;
            end
        end
    endtask

    task fc_check;
        integer b, m, lraw, nzh;
        real    ev, dv, lgv, mx, d, nrm;
        reg     skip;
        begin
            t_fc_norm = 0.0; t_fcoor = 0; t_fcval = 0;
            m_E = 0; m_S = 0; m_L = 0; m_SG = 0; m_Z = 0; m_OV = 0; m_UN = 0;
            fc_n_bad = (fc_n != ARRAY_SIZE);
            for (b = 0; b < 4; b = b + 1) begin
                fc_ideal(b);
                mx = 0.0;
                for (m = 0; m < 4; m = m + 1) if (absr(Cexp[m][b]) > mx) mx = absr(Cexp[m][b]);
                skip = x_ovf[b] || x_unf[b];          // ideal block itself out of range
                if (skip) t_fcoor = t_fcoor + 1;
                if (b < fc_n) begin
                    nzh = 0;
                    for (m = 0; m < 4; m = m + 1) if (!fc_zero[b][m]) nzh = nzh + 1;
                    cov_n[nzh] = cov_n[nzh] + 1;
                    if (fc_exp[b]   != x_E[b])   m_E  = m_E  + 1;
                    if (fc_scale[b] != x_S[b])   m_S  = m_S  + 1;
                    if (fc_ovf[b]   != x_ovf[b]) m_OV = m_OV + 1;
                    if (fc_unf[b]   != x_unf[b]) m_UN = m_UN + 1;
                end
                for (m = 0; m < 4; m = m + 1) begin
                    ev = Cexp[m][b];
                    dv = 0.0; g_L[m][b] = 0;
                    if (b < fc_n) begin
                        lraw = (32'(fc_mant[b]) >> (m*BLOCK_MANTISSA_WIDTH))
                               & ((1<<BLOCK_MANTISSA_WIDTH)-1);
                        if (lraw >= (1<<(BLOCK_MANTISSA_WIDTH-1)))
                            lraw = lraw - (1<<BLOCK_MANTISSA_WIDTH);
                        g_L[m][b] = lraw;
                        if (lraw            != x_L[m][b])  m_L  = m_L  + 1;
                        if (fc_sign[b][m]   != x_sg[m][b]) m_SG = m_SG + 1;
                        if (fc_zero[b][m]   != x_z[m][b])  m_Z  = m_Z  + 1;
                        if (!fc_zero[b][m]) begin
                            lgv = $itor(lraw * (1<<fc_scale[b])) / $itor(LOG_ONE);
                            dv  = 2.0 ** ($itor(fc_exp[b]) + lgv);
                            if (fc_sign[b][m]) dv = -dv;
                        end
                    end
                    fc_dec[m][b] = dv;
                    d   = absr(dv - ev);
                    nrm = (mx > 0.0) ? d/mx : 0.0;
                    fc_nerr[m][b] = nrm;
                    if (!skip && nrm > t_fc_norm) t_fc_norm = nrm;
                    if (!skip && nrm > 0.10)      t_fcval   = t_fcval + 1;
                end
            end
            // FAIL on: wrong block count, values over tolerance, wrong flags
            t_fcerr = (fc_n_bad ? 1 : 0) + t_fcval + m_OV + m_UN;
            if (t_fc_norm > s_fcworst) s_fcworst = t_fc_norm;
            c_oor = c_oor + t_fcoor;
        end
    endtask

    // One place that decides pass/fail, so run_test and run_case cannot drift.
    // ======================================================================
    // PE OVERFLOW CHECK
    // A product overflows its DATA_WIDTH register when k = floor(L_row + L_col)
    // is at least DATA_WIDTH - FRAC_WIDTH (the antilog A is >= 2^FRAC_WIDTH, so
    // A << k needs at least FRAC_WIDTH + 1 + k bits). k is computed exactly from
    // the input blocks as the PE sees them (L << scale, in 1/2^FRAC units).
    // Every such PE MUST raise pe_overflow. A flag without a must-flag is
    // allowed: the accumulator can overflow on its own (Limit 4, sum form).
    // ======================================================================
    function integer lbus(input [BW-1:0] blk, input integer idx);   // L << scale, signed
        integer l, sc;
        begin
            l  = 32'(blk[MANT_LSB + idx*BLOCK_MANTISSA_WIDTH +: BLOCK_MANTISSA_WIDTH]);
            if (l >= (1 << (BLOCK_MANTISSA_WIDTH-1))) l = l - (1 << BLOCK_MANTISSA_WIDTH);
            sc = 32'(blk[SCALE_LSB +: SCALE_FACTOR_WIDTH]);
            lbus = l * (1 << sc);
        end
    endfunction

    task pe_ovf_check;
        integer i4, j4, k4, ksum;
        begin
            t_ovf_hw = 0; t_ovf_miss = 0;
            for (i4 = 0; i4 < 4; i4 = i4 + 1) for (j4 = 0; j4 < 4; j4 = j4 + 1) begin
                ov_hw[i4][j4]   = pe_overflow[i4*ARRAY_SIZE + j4] ? 1 : 0;
                ov_need[i4][j4] = 0;
                for (k4 = 0; k4 < 4; k4 = k4 + 1)
                    if (!row_block[i4][ZERO_LSB + k4] && !col_block[j4][ZERO_LSB + k4]) begin
                        ksum = (lbus(row_block[i4], k4) + lbus(col_block[j4], k4)) >>> FRAC_WIDTH;
                        if (ksum >= DATA_WIDTH - FRAC_WIDTH) ov_need[i4][j4] = 1;
                    end
                t_ovf_hw = t_ovf_hw + ov_hw[i4][j4];
                if ((ov_need[i4][j4] != 0) && (ov_hw[i4][j4] == 0)) t_ovf_miss = t_ovf_miss + 1;
            end
            if (t_ovf_hw   != 0) c_ovf      = c_ovf + 1;
            if (t_ovf_miss != 0) c_ovf_miss = c_ovf_miss + 1;
        end
    endtask

    task account;
        begin
            if (t_cond > 0.10)  c_res = c_res + 1;
            if (t_experr != 0)  c_exp = c_exp + 1;
            if (t_fcerr  != 0)  c_fc  = c_fc  + 1;
            t_fail = (t_cond > 0.10) || (t_experr != 0) || (t_fcerr != 0) || (t_ovf_miss != 0);
        end
    endtask

    task make_txt;
        integer i3,j3;
        begin
            $fwrite(fh,"\n");
            $fwrite(fh,"================================================================================\n");
            $fwrite(fh,"  TEST %0d  :  %0s\n", t_id, t_name);
            $fwrite(fh,"================================================================================\n\n");

            $fwrite(fh,"  MATRIX A  (row blocks)                                    shared exp E_row\n");
            $fwrite(fh,"  ------------------------------------------------------    ----------------\n");
            for(i3=0;i3<4;i3=i3+1)
                $fwrite(fh,"   [ %12.5f %12.5f %12.5f %12.5f ]        E = %0d\n",
                        Afp[i3][0],Afp[i3][1],Afp[i3][2],Afp[i3][3], E_row[i3]);

            $fwrite(fh,"\n  MATRIX B  (column blocks)\n");
            $fwrite(fh,"  ------------------------------------------------------\n");
            for(i3=0;i3<4;i3=i3+1)
                $fwrite(fh,"   [ %12.5f %12.5f %12.5f %12.5f ]\n",
                        Bfp[i3][0],Bfp[i3][1],Bfp[i3][2],Bfp[i3][3]);
            $fwrite(fh,"        E_col =");
            for(j3=0;j3<4;j3=j3+1) $fwrite(fh," %8d", E_col[j3]);
            $fwrite(fh,"\n");

            $fwrite(fh,"\n  EXPECTED  C = A x B\n");
            $fwrite(fh,"  ------------------------------------------------------\n");
            for(i3=0;i3<4;i3=i3+1)
                $fwrite(fh,"   [ %12.4f %12.4f %12.4f %12.4f ]\n",
                        Cexp[i3][0],Cexp[i3][1],Cexp[i3][2],Cexp[i3][3]);

            $fwrite(fh,"\n  GOT       C  (hardware, Res_out/256 x 2^(E_row+E_col))\n");
            $fwrite(fh,"  ------------------------------------------------------\n");
            for(i3=0;i3<4;i3=i3+1)
                $fwrite(fh,"   [ %12.4f %12.4f %12.4f %12.4f ]\n",
                        got_m[i3][0],got_m[i3][1],got_m[i3][2],got_m[i3][3]);

            $fwrite(fh,"\n  ABSOLUTE ERROR  |got - expected|\n");
            $fwrite(fh,"  ------------------------------------------------------\n");
            for(i3=0;i3<4;i3=i3+1)
                $fwrite(fh,"   [ %12.4f %12.4f %12.4f %12.4f ]\n",
                        abserr[i3][0],abserr[i3][1],abserr[i3][2],abserr[i3][3]);

            $fwrite(fh,"\n  EXPONENT CHECK   exp_out (hardware)  vs  E_row+E_col (expected)\n");
            $fwrite(fh,"  ------------------------------------------------------\n");
            for(i3=0;i3<4;i3=i3+1) begin
                $fwrite(fh,"   got  [ %5d %5d %5d %5d ]     exp [ %5d %5d %5d %5d ]\n",
                    exp_hw[i3][0],exp_hw[i3][1],exp_hw[i3][2],exp_hw[i3][3],
                    exp_ref[i3][0],exp_ref[i3][1],exp_ref[i3][2],exp_ref[i3][3]);
            end
            $fwrite(fh,"    exponent mismatches : %0d of 16  %0s\n",
                    t_experr, (t_experr==0)?"(OK)":"(*** MISMATCH ***)");

            $fwrite(fh,"\n  PE OVERFLOW CHECK   pe_overflow (hardware)  vs  must-flag (from the input blocks)\n");
            $fwrite(fh,"  ------------------------------------------------------\n");
            for(i3=0;i3<4;i3=i3+1)
                $fwrite(fh,"   got  [ %5d %5d %5d %5d ]     must [ %5d %5d %5d %5d ]\n",
                    ov_hw[i3][0],ov_hw[i3][1],ov_hw[i3][2],ov_hw[i3][3],
                    ov_need[i3][0],ov_need[i3][1],ov_need[i3][2],ov_need[i3][3]);
            $fwrite(fh,"    flagged : %0d of 16   missed : %0d  %0s\n", t_ovf_hw, t_ovf_miss,
                    (t_ovf_miss==0)?"(OK)":"(*** MISSED OVERFLOW ***)");
            if (t_ovf_hw != 0)
                $fwrite(fh,"    flagged PEs hold invalid results (top bits lost); a flag with must = 0 is an accumulator overflow\n");

            fc_dump;

            $fwrite(fh,"\n  SUMMARY\n");
            $fwrite(fh,"  ------------------------------------------------------\n");
            $fwrite(fh,"    worst condition-aware error : %.3f %%\n", t_cond*100.0);
            $fwrite(fh,"       ( |err| / sum|terms| , immune to cancellation )\n");
            $fwrite(fh,"    sign flips                  : %0d of 16 elements\n", t_flip);
            $fwrite(fh,"    exponent mismatches         : %0d of 16\n", t_experr);
            $fwrite(fh,"    block values over 10%%       : %0d of 16\n", t_fcval);
            $fwrite(fh,"    worst block error           : %.3f %%\n", t_fc_norm*100.0);
            $fwrite(fh,"    exp_overflow/underflow wrong: %0d\n", m_OV + m_UN);
            $fwrite(fh,"    PE overflow flagged / missed: %0d / %0d\n", t_ovf_hw, t_ovf_miss);
            $fwrite(fh,"    tolerance                   : 10.000 %%\n");
            $fwrite(fh,"    VERDICT                     : %0s\n", t_fail ? "FAIL" : "PASS");
            $fwrite(fh,"\n");
        end
    endtask

    // OUTPUT BLOCK CHECK section of the per-test report. Same layout as the
    // matrices above: 4 blocks side by side, block j = column j of C.
    task fc_dump;
        integer r;
        begin
            $fwrite(fh,"\n  OUTPUT BLOCK CHECK   hardware block  vs  ideal block (software, from EXPECTED C)\n");
            $fwrite(fh,"  ------------------------------------------------------\n");
            $fwrite(fh,"   one block per column of C: block 0 = column 0, ...   blocks produced : %0d of 4 %0s\n\n",
                    fc_n, fc_n_bad ? "(*** WRONG ***)" : "(OK)");
            $fwrite(fh,"                        block0  block1  block2  block3\n");
            $fwrite(fh,"   exponent E     got  [ %6d  %6d  %6d  %6d ]\n", fc_exp[0], fc_exp[1], fc_exp[2], fc_exp[3]);
            $fwrite(fh,"                  exp  [ %6d  %6d  %6d  %6d ]\n", x_E[0], x_E[1], x_E[2], x_E[3]);
            $fwrite(fh,"   scale          got  [ %6d  %6d  %6d  %6d ]\n", fc_scale[0], fc_scale[1], fc_scale[2], fc_scale[3]);
            $fwrite(fh,"                  exp  [ %6d  %6d  %6d  %6d ]\n", x_S[0], x_S[1], x_S[2], x_S[3]);
            $fwrite(fh,"   exp_overflow   got  [ %6d  %6d  %6d  %6d ]\n", fc_ovf[0], fc_ovf[1], fc_ovf[2], fc_ovf[3]);
            $fwrite(fh,"                  exp  [ %6d  %6d  %6d  %6d ]\n", x_ovf[0], x_ovf[1], x_ovf[2], x_ovf[3]);
            $fwrite(fh,"   exp_underflow  got  [ %6d  %6d  %6d  %6d ]\n", fc_unf[0], fc_unf[1], fc_unf[2], fc_unf[3]);
            $fwrite(fh,"                  exp  [ %6d  %6d  %6d  %6d ]\n", x_unf[0], x_unf[1], x_unf[2], x_unf[3]);

            $fwrite(fh,"\n   mantissa L   (row i = element i of the column)\n");
            for (r = 0; r < 4; r = r + 1)
                $fwrite(fh,"                  got  [ %6d  %6d  %6d  %6d ]     exp [ %6d  %6d  %6d  %6d ]\n",
                        g_L[r][0], g_L[r][1], g_L[r][2], g_L[r][3],
                        x_L[r][0], x_L[r][1], x_L[r][2], x_L[r][3]);
            $fwrite(fh,"\n   sign   (1 = negative)\n");
            for (r = 0; r < 4; r = r + 1)
                $fwrite(fh,"                  got  [ %6d  %6d  %6d  %6d ]     exp [ %6d  %6d  %6d  %6d ]\n",
                        fc_sign[0][r], fc_sign[1][r], fc_sign[2][r], fc_sign[3][r],
                        x_sg[r][0], x_sg[r][1], x_sg[r][2], x_sg[r][3]);
            $fwrite(fh,"\n   zero   (1 = value is exactly 0)\n");
            for (r = 0; r < 4; r = r + 1)
                $fwrite(fh,"                  got  [ %6d  %6d  %6d  %6d ]     exp [ %6d  %6d  %6d  %6d ]\n",
                        fc_zero[0][r], fc_zero[1][r], fc_zero[2][r], fc_zero[3][r],
                        x_z[r][0], x_z[r][1], x_z[r][2], x_z[r][3]);

            $fwrite(fh,"\n    mismatches : exponent %0d/4  scale %0d/4  mantissa %0d/16  sign %0d/16  zero %0d/16\n",
                    m_E, m_S, m_L, m_SG, m_Z);
            $fwrite(fh,"                 exp_overflow %0d/4  exp_underflow %0d/4   %0s\n", m_OV, m_UN,
                    (m_OV + m_UN == 0) ? "(flags OK)" : "(*** FLAG MISMATCH ***)");
            $fwrite(fh,"    (exponent/scale/mantissa/sign/zero are shown, not failed: the hardware\n");
            $fwrite(fh,"     log is approximate, so a difference of 1 from the ideal is normal)\n");

            $fwrite(fh,"\n  DECODED C  (value read back from the hardware blocks)\n");
            $fwrite(fh,"  ------------------------------------------------------\n");
            for (r = 0; r < 4; r = r + 1)
                $fwrite(fh,"   [ %12.4f %12.4f %12.4f %12.4f ]\n",
                        fc_dec[r][0], fc_dec[r][1], fc_dec[r][2], fc_dec[r][3]);

            $fwrite(fh,"\n  BLOCK ERROR  |decoded - expected C| / largest |expected C| in that column\n");
            $fwrite(fh,"  ------------------------------------------------------\n");
            for (r = 0; r < 4; r = r + 1)
                $fwrite(fh,"   [ %11.3f%% %11.3f%% %11.3f%% %11.3f%% ]\n",
                        fc_nerr[r][0]*100.0, fc_nerr[r][1]*100.0, fc_nerr[r][2]*100.0, fc_nerr[r][3]*100.0);
            $fwrite(fh,"    values over 10%% : %0d of 16      worst : %.3f %%   %0s\n",
                    t_fcval, t_fc_norm*100.0, (t_fcval == 0) ? "(OK)" : "(*** OVER TOLERANCE ***)");
            if (t_fcoor != 0)
                $fwrite(fh,"    %0d block(s) out of exponent range: E clamped + flagged, values skipped\n", t_fcoor);
        end
    endtask

    // ---------------- appended stress / marginal cases ----------------
    // Condition-aware pass criterion: |err| / sum|terms|.  Plain relative error
    // is meaningless when a dot product cancels (tiny |sum|, finite |err|), so
    // a cancelling case would "fail" for a non-reason. 10% of sum|terms| is the
    // bar here.  Cases are also flagged if any element's SIGN comes out wrong.
    integer s_run, s_fail, s_flip, s_experr;
    real    s_worst, s_sum;

    task run_case(input integer id, input [8*44:1] name);
        real a,e,d,sa,cond,wc,term;
        integer sf,i2,j2,k2;
        begin
            for(i=0;i<4;i=i+1) for(j=0;j<4;j=j+1) begin
                Cexp[i][j]=0.0;
                for(k=0;k<4;k=k+1) Cexp[i][j]=Cexp[i][j]+Afp[i][k]*Bfp[k][j];
            end
            rst_n=0; clrwe;
            for(i=0;i<4;i=i+1) begin row_block[i]=0; col_block[i]=0; end
            repeat(4) @(posedge clk);
            rst_n=1; @(posedge clk); @(posedge clk);
            for(i=0;i<4;i=i+1) begin
                row_block[i] = make_block(Afp[i][0],Afp[i][1],Afp[i][2],Afp[i][3]);
                col_block[i] = make_block(Bfp[0][i],Bfp[1][i],Bfp[2][i],Bfp[3][i]);
                E_row[i]     = calc_E(Afp[i][0],Afp[i][1],Afp[i][2],Afp[i][3]);
                E_col[i]     = calc_E(Bfp[0][i],Bfp[1][i],Bfp[2][i],Bfp[3][i]);
            end
            for(i=0;i<4;i=i+1) begin row_we[i]=1; col_we[i]=1; end
            @(posedge clk); clrwe;
            repeat(200) @(posedge clk);

            wc=0.0; sf=0; t_experr=0;
            for(i2=0;i2<4;i2=i2+1) for(j2=0;j2<4;j2=j2+1) begin
                a = ($itor($signed(Res_out[i2][j2]))/$itor(LOG_ONE))*(2.0**(E_row[i2]+E_col[j2]));
                e = Cexp[i2][j2];
                d = (a>e)?(a-e):(e-a);
                got_m[i2][j2]  = a;
                abserr[i2][j2] = d;
                exp_hw[i2][j2]  = 32'(signed'(exp_out_hw[i2][j2]));
                exp_ref[i2][j2] = E_row[i2] + E_col[j2];
                if (exp_hw[i2][j2] !== exp_ref[i2][j2]) t_experr = t_experr + 1;
                sa = 0.0;
                for(k2=0;k2<4;k2=k2+1) begin
                    term = Afp[i2][k2]*Bfp[k2][j2];
                    sa = sa + ((term<0.0)? -term : term);
                end
                if (sa>0.0) begin cond = d/sa; if (cond>wc) wc=cond; end
                if (e*a < 0.0) sf = sf + 1;
            end

            t_id=id; t_name=name; t_cond=wc; t_flip=sf;
            fc_check; pe_ovf_check; account;
            make_txt;

            s_run = s_run+1; s_sum = s_sum+wc;
            if (wc > s_worst) s_worst = wc;
            if (t_fail)    s_fail = s_fail+1;
            if (sf > 0)    s_flip = s_flip+1;
            // terminal: one compact table row only
            t_nstr = $sformatf("%0s", name);
            $display("  %4d | %-32s | %8.3f%% | %2d  | %2d  | %8.3f%% | %0s",
                      id, t_nstr, wc*100.0, sf, t_experr, t_fc_norm*100.0,
                      t_fail ? "FAIL" : "PASS");
            if (t_experr!=0) s_experr = s_experr + 1;
        end
    endtask

    initial begin
        clrwe; ntests=0; nfail=0; acc_sum=0.0;
        s_run=0; s_fail=0; s_flip=0; s_experr=0; s_worst=0.0; s_sum=0.0;
        for (i=0;i<5;i=i+1) cov_n[i]=0;
        c_res=0; c_exp=0; c_fc=0; c_oor=0; c_ovf=0; c_ovf_miss=0; t_ovf_hw=0; t_ovf_miss=0; t_fcoor=0; s_fcworst=0.0; fc_n_bad=1'b0; t_fcerr=0; t_fail=1'b0;
        fh = $fopen("test_report.txt","w");
        if (fh == 0) begin $display("ERROR: cannot open test_report.txt"); $finish; end
        $fwrite(fh,"================================================================================\n");
        $fwrite(fh,"            BFP / LNS SYSTOLIC ARRAY  -  DETAILED TEST REPORT\n");
        $fwrite(fh,"================================================================================\n");
        $fwrite(fh,"  Datapath : 4x4 output-stationary, log-domain (LNS) multiply\n");
        $fwrite(fh,"  Encoding : v = (-1)^s * 2^E * 2^L   (E shared per block, L per element)\n");
        $fwrite(fh,"  Metric   : condition-aware error = |got-exp| / sum|terms|\n");
        $fwrite(fh,"  Tolerance: 10%%\n");
        $fwrite(fh,"================================================================================\n");
        $display("");
        $display(" ------------------------------------------------------------------------------------");
        $display("  TEST | NAME                             | COND-ERR | SF  | EXP |  BLK-ERR | RESULT");
        $display(" ------------------------------------------------------------------------------------");
        for(i=0;i<4;i=i+1) begin row_block[i]=0; col_block[i]=0; end
        rst_n=0; #20 rst_n=1;
        @(posedge clk);

        // ---------------- T1: all ones ----------------
        Afp[0][0]=1.00; Afp[0][1]=1.00; Afp[0][2]=1.00; Afp[0][3]=1.00;
        Afp[1][0]=1.00; Afp[1][1]=1.00; Afp[1][2]=1.00; Afp[1][3]=1.00;
        Afp[2][0]=1.00; Afp[2][1]=1.00; Afp[2][2]=1.00; Afp[2][3]=1.00;
        Afp[3][0]=1.00; Afp[3][1]=1.00; Afp[3][2]=1.00; Afp[3][3]=1.00;
        Bfp[0][0]=1.00; Bfp[0][1]=1.00; Bfp[0][2]=1.00; Bfp[0][3]=1.00;
        Bfp[1][0]=1.00; Bfp[1][1]=1.00; Bfp[1][2]=1.00; Bfp[1][3]=1.00;
        Bfp[2][0]=1.00; Bfp[2][1]=1.00; Bfp[2][2]=1.00; Bfp[2][3]=1.00;
        Bfp[3][0]=1.00; Bfp[3][1]=1.00; Bfp[3][2]=1.00; Bfp[3][3]=1.00;
        run_test(1, "all ones");

        // ---------------- T2: all twos ----------------
        Afp[0][0]=2.00; Afp[0][1]=2.00; Afp[0][2]=2.00; Afp[0][3]=2.00;
        Afp[1][0]=2.00; Afp[1][1]=2.00; Afp[1][2]=2.00; Afp[1][3]=2.00;
        Afp[2][0]=2.00; Afp[2][1]=2.00; Afp[2][2]=2.00; Afp[2][3]=2.00;
        Afp[3][0]=2.00; Afp[3][1]=2.00; Afp[3][2]=2.00; Afp[3][3]=2.00;
        Bfp[0][0]=2.00; Bfp[0][1]=2.00; Bfp[0][2]=2.00; Bfp[0][3]=2.00;
        Bfp[1][0]=2.00; Bfp[1][1]=2.00; Bfp[1][2]=2.00; Bfp[1][3]=2.00;
        Bfp[2][0]=2.00; Bfp[2][1]=2.00; Bfp[2][2]=2.00; Bfp[2][3]=2.00;
        Bfp[3][0]=2.00; Bfp[3][1]=2.00; Bfp[3][2]=2.00; Bfp[3][3]=2.00;
        run_test(2, "all twos");

        // ---------------- T3: powers of two ----------------
        Afp[0][0]=2.00; Afp[0][1]=4.00; Afp[0][2]=8.00; Afp[0][3]=16.00;
        Afp[1][0]=4.00; Afp[1][1]=8.00; Afp[1][2]=16.00; Afp[1][3]=2.00;
        Afp[2][0]=8.00; Afp[2][1]=16.00; Afp[2][2]=2.00; Afp[2][3]=4.00;
        Afp[3][0]=16.00; Afp[3][1]=2.00; Afp[3][2]=4.00; Afp[3][3]=8.00;
        Bfp[0][0]=2.00; Bfp[0][1]=8.00; Bfp[0][2]=4.00; Bfp[0][3]=16.00;
        Bfp[1][0]=16.00; Bfp[1][1]=4.00; Bfp[1][2]=8.00; Bfp[1][3]=2.00;
        Bfp[2][0]=4.00; Bfp[2][1]=2.00; Bfp[2][2]=16.00; Bfp[2][3]=8.00;
        Bfp[3][0]=8.00; Bfp[3][1]=16.00; Bfp[3][2]=2.00; Bfp[3][3]=4.00;
        run_test(3, "powers of two");

        // ---------------- T4: small integers ----------------
        Afp[0][0]=1.00; Afp[0][1]=2.00; Afp[0][2]=3.00; Afp[0][3]=4.00;
        Afp[1][0]=4.00; Afp[1][1]=3.00; Afp[1][2]=2.00; Afp[1][3]=1.00;
        Afp[2][0]=2.00; Afp[2][1]=4.00; Afp[2][2]=1.00; Afp[2][3]=3.00;
        Afp[3][0]=3.00; Afp[3][1]=1.00; Afp[3][2]=4.00; Afp[3][3]=2.00;
        Bfp[0][0]=1.00; Bfp[0][1]=3.00; Bfp[0][2]=2.00; Bfp[0][3]=4.00;
        Bfp[1][0]=2.00; Bfp[1][1]=1.00; Bfp[1][2]=4.00; Bfp[1][3]=3.00;
        Bfp[2][0]=4.00; Bfp[2][1]=2.00; Bfp[2][2]=3.00; Bfp[2][3]=1.00;
        Bfp[3][0]=3.00; Bfp[3][1]=4.00; Bfp[3][2]=1.00; Bfp[3][3]=2.00;
        run_test(4, "small integers");

        // ---------------- T5: near-1 tight ----------------
        Afp[0][0]=1.10; Afp[0][1]=1.20; Afp[0][2]=1.30; Afp[0][3]=1.40;
        Afp[1][0]=1.40; Afp[1][1]=1.30; Afp[1][2]=1.20; Afp[1][3]=1.10;
        Afp[2][0]=1.15; Afp[2][1]=1.25; Afp[2][2]=1.35; Afp[2][3]=1.45;
        Afp[3][0]=1.45; Afp[3][1]=1.35; Afp[3][2]=1.25; Afp[3][3]=1.15;
        Bfp[0][0]=1.12; Bfp[0][1]=1.22; Bfp[0][2]=1.32; Bfp[0][3]=1.42;
        Bfp[1][0]=1.42; Bfp[1][1]=1.32; Bfp[1][2]=1.22; Bfp[1][3]=1.12;
        Bfp[2][0]=1.18; Bfp[2][1]=1.28; Bfp[2][2]=1.38; Bfp[2][3]=1.48;
        Bfp[3][0]=1.48; Bfp[3][1]=1.38; Bfp[3][2]=1.28; Bfp[3][3]=1.18;
        run_test(5, "near-1 tight");

        // ---------------- T6: near-1 mixed ----------------
        Afp[0][0]=1.48; Afp[0][1]=1.61; Afp[0][2]=1.19; Afp[0][3]=1.83;
        Afp[1][0]=1.83; Afp[1][1]=1.19; Afp[1][2]=1.61; Afp[1][3]=1.48;
        Afp[2][0]=1.27; Afp[2][1]=1.94; Afp[2][2]=1.05; Afp[2][3]=1.72;
        Afp[3][0]=1.72; Afp[3][1]=1.05; Afp[3][2]=1.94; Afp[3][3]=1.27;
        Bfp[0][0]=1.48; Bfp[0][1]=1.61; Bfp[0][2]=1.19; Bfp[0][3]=1.83;
        Bfp[1][0]=1.83; Bfp[1][1]=1.19; Bfp[1][2]=1.61; Bfp[1][3]=1.48;
        Bfp[2][0]=1.27; Bfp[2][1]=1.94; Bfp[2][2]=1.05; Bfp[2][3]=1.72;
        Bfp[3][0]=1.72; Bfp[3][1]=1.05; Bfp[3][2]=1.94; Bfp[3][3]=1.27;
        run_test(6, "near-1 mixed");

        // ---------------- T7: near-1 all unique ----------------
        Afp[0][0]=1.03; Afp[0][1]=1.09; Afp[0][2]=1.17; Afp[0][3]=1.23;
        Afp[1][0]=1.31; Afp[1][1]=1.37; Afp[1][2]=1.43; Afp[1][3]=1.51;
        Afp[2][0]=1.59; Afp[2][1]=1.63; Afp[2][2]=1.71; Afp[2][3]=1.79;
        Afp[3][0]=1.87; Afp[3][1]=1.91; Afp[3][2]=1.97; Afp[3][3]=1.99;
        Bfp[0][0]=1.98; Bfp[0][1]=1.92; Bfp[0][2]=1.86; Bfp[0][3]=1.78;
        Bfp[1][0]=1.72; Bfp[1][1]=1.66; Bfp[1][2]=1.58; Bfp[1][3]=1.52;
        Bfp[2][0]=1.46; Bfp[2][1]=1.38; Bfp[2][2]=1.32; Bfp[2][3]=1.26;
        Bfp[3][0]=1.18; Bfp[3][1]=1.12; Bfp[3][2]=1.06; Bfp[3][3]=1.02;
        run_test(7, "near-1 all unique");

        // ---------------- T8: near-1 interleaved ----------------
        Afp[0][0]=1.07; Afp[0][1]=1.93; Afp[0][2]=1.21; Afp[0][3]=1.77;
        Afp[1][0]=1.85; Afp[1][1]=1.13; Afp[1][2]=1.69; Afp[1][3]=1.35;
        Afp[2][0]=1.41; Afp[2][1]=1.65; Afp[2][2]=1.29; Afp[2][3]=1.89;
        Afp[3][0]=1.97; Afp[3][1]=1.25; Afp[3][2]=1.53; Afp[3][3]=1.11;
        Bfp[0][0]=1.55; Bfp[0][1]=1.33; Bfp[0][2]=1.81; Bfp[0][3]=1.15;
        Bfp[1][0]=1.27; Bfp[1][1]=1.75; Bfp[1][2]=1.39; Bfp[1][3]=1.63;
        Bfp[2][0]=1.95; Bfp[2][1]=1.17; Bfp[2][2]=1.61; Bfp[2][3]=1.45;
        Bfp[3][0]=1.09; Bfp[3][1]=1.87; Bfp[3][2]=1.23; Bfp[3][3]=1.71;
        run_test(8, "near-1 interleaved");

        // ---------------- T9: near-1 extremes ----------------
        Afp[0][0]=1.01; Afp[0][1]=1.99; Afp[0][2]=1.33; Afp[0][3]=1.67;
        Afp[1][0]=1.99; Afp[1][1]=1.01; Afp[1][2]=1.67; Afp[1][3]=1.33;
        Afp[2][0]=1.50; Afp[2][1]=1.75; Afp[2][2]=1.25; Afp[2][3]=1.95;
        Afp[3][0]=1.05; Afp[3][1]=1.45; Afp[3][2]=1.85; Afp[3][3]=1.55;
        Bfp[0][0]=1.11; Bfp[0][1]=1.44; Bfp[0][2]=1.77; Bfp[0][3]=1.22;
        Bfp[1][0]=1.88; Bfp[1][1]=1.33; Bfp[1][2]=1.66; Bfp[1][3]=1.02;
        Bfp[2][0]=1.55; Bfp[2][1]=1.99; Bfp[2][2]=1.08; Bfp[2][3]=1.42;
        Bfp[3][0]=1.31; Bfp[3][1]=1.62; Bfp[3][2]=1.93; Bfp[3][3]=1.24;
        run_test(9, "near-1 extremes");

        // ---------------- T10: mid-range mixed ----------------
        Afp[0][0]=2.50; Afp[0][1]=3.70; Afp[0][2]=1.40; Afp[0][3]=4.20;
        Afp[1][0]=4.20; Afp[1][1]=1.40; Afp[1][2]=3.70; Afp[1][3]=2.50;
        Afp[2][0]=3.10; Afp[2][1]=2.80; Afp[2][2]=4.60; Afp[2][3]=1.90;
        Afp[3][0]=1.90; Afp[3][1]=4.60; Afp[3][2]=2.80; Afp[3][3]=3.10;
        Bfp[0][0]=1.50; Bfp[0][1]=2.80; Bfp[0][2]=3.40; Bfp[0][3]=1.30;
        Bfp[1][0]=3.40; Bfp[1][1]=1.30; Bfp[1][2]=2.80; Bfp[1][3]=1.50;
        Bfp[2][0]=2.20; Bfp[2][1]=4.10; Bfp[2][2]=1.70; Bfp[2][3]=3.60;
        Bfp[3][0]=3.60; Bfp[3][1]=1.70; Bfp[3][2]=4.10; Bfp[3][3]=2.20;
        run_test(10, "mid-range mixed");

        // ---------------- T11: mid-range all unique ----------------
        Afp[0][0]=1.50; Afp[0][1]=1.85; Afp[0][2]=2.20; Afp[0][3]=2.55;
        Afp[1][0]=2.90; Afp[1][1]=3.25; Afp[1][2]=3.60; Afp[1][3]=3.95;
        Afp[2][0]=4.30; Afp[2][1]=4.65; Afp[2][2]=5.00; Afp[2][3]=5.35;
        Afp[3][0]=5.70; Afp[3][1]=6.05; Afp[3][2]=6.40; Afp[3][3]=6.75;
        Bfp[0][0]=6.00; Bfp[0][1]=5.70; Bfp[0][2]=5.40; Bfp[0][3]=5.10;
        Bfp[1][0]=4.80; Bfp[1][1]=4.50; Bfp[1][2]=4.20; Bfp[1][3]=3.90;
        Bfp[2][0]=3.60; Bfp[2][1]=3.30; Bfp[2][2]=3.00; Bfp[2][3]=2.70;
        Bfp[3][0]=2.40; Bfp[3][1]=2.10; Bfp[3][2]=1.80; Bfp[3][3]=1.50;
        run_test(11, "mid-range all unique");

        // ---------------- T12: mid-range repeating ----------------
        Afp[0][0]=3.30; Afp[0][1]=2.20; Afp[0][2]=4.40; Afp[0][3]=1.10;
        Afp[1][0]=1.10; Afp[1][1]=4.40; Afp[1][2]=2.20; Afp[1][3]=3.30;
        Afp[2][0]=5.50; Afp[2][1]=6.60; Afp[2][2]=1.10; Afp[2][3]=2.20;
        Afp[3][0]=2.20; Afp[3][1]=1.10; Afp[3][2]=6.60; Afp[3][3]=5.50;
        Bfp[0][0]=2.60; Bfp[0][1]=3.90; Bfp[0][2]=1.70; Bfp[0][3]=4.80;
        Bfp[1][0]=4.80; Bfp[1][1]=1.70; Bfp[1][2]=3.90; Bfp[1][3]=2.60;
        Bfp[2][0]=6.10; Bfp[2][1]=1.30; Bfp[2][2]=5.20; Bfp[2][3]=2.40;
        Bfp[3][0]=2.40; Bfp[3][1]=5.20; Bfp[3][2]=1.30; Bfp[3][3]=6.10;
        run_test(12, "mid-range repeating");

        // ---------------- T13: mid-range scattered ----------------
        Afp[0][0]=2.07; Afp[0][1]=5.83; Afp[0][2]=3.41; Afp[0][3]=1.62;
        Afp[1][0]=4.95; Afp[1][1]=1.28; Afp[1][2]=6.14; Afp[1][3]=2.73;
        Afp[2][0]=3.86; Afp[2][1]=6.52; Afp[2][2]=1.95; Afp[2][3]=4.31;
        Afp[3][0]=1.44; Afp[3][1]=3.09; Afp[3][2]=5.27; Afp[3][3]=6.88;
        Bfp[0][0]=5.11; Bfp[0][1]=2.46; Bfp[0][2]=6.73; Bfp[0][3]=1.85;
        Bfp[1][0]=3.52; Bfp[1][1]=6.07; Bfp[1][2]=2.19; Bfp[1][3]=4.64;
        Bfp[2][0]=1.73; Bfp[2][1]=4.28; Bfp[2][2]=5.96; Bfp[2][3]=3.35;
        Bfp[3][0]=6.41; Bfp[3][1]=1.59; Bfp[3][2]=3.82; Bfp[3][3]=5.04;
        run_test(13, "mid-range scattered");

        // ---------------- T14: larger values ----------------
        Afp[0][0]=5.50; Afp[0][1]=10.20; Afp[0][2]=3.30; Afp[0][3]=7.70;
        Afp[1][0]=7.70; Afp[1][1]=3.30; Afp[1][2]=10.20; Afp[1][3]=5.50;
        Afp[2][0]=8.90; Afp[2][1]=4.60; Afp[2][2]=6.10; Afp[2][3]=11.30;
        Afp[3][0]=11.30; Afp[3][1]=6.10; Afp[3][2]=4.60; Afp[3][3]=8.90;
        Bfp[0][0]=4.40; Bfp[0][1]=8.80; Bfp[0][2]=6.60; Bfp[0][3]=2.20;
        Bfp[1][0]=2.20; Bfp[1][1]=6.60; Bfp[1][2]=8.80; Bfp[1][3]=4.40;
        Bfp[2][0]=9.50; Bfp[2][1]=3.70; Bfp[2][2]=5.30; Bfp[2][3]=7.10;
        Bfp[3][0]=7.10; Bfp[3][1]=5.30; Bfp[3][2]=3.70; Bfp[3][3]=9.50;
        run_test(14, "larger values");

        // ---------------- T15: teens ----------------
        Afp[0][0]=12.50; Afp[0][1]=18.30; Afp[0][2]=9.70; Afp[0][3]=15.10;
        Afp[1][0]=15.10; Afp[1][1]=9.70; Afp[1][2]=18.30; Afp[1][3]=12.50;
        Afp[2][0]=11.80; Afp[2][1]=16.40; Afp[2][2]=13.90; Afp[2][3]=10.60;
        Afp[3][0]=10.60; Afp[3][1]=13.90; Afp[3][2]=16.40; Afp[3][3]=11.80;
        Bfp[0][0]=11.20; Bfp[0][1]=7.40; Bfp[0][2]=16.80; Bfp[0][3]=13.60;
        Bfp[1][0]=13.60; Bfp[1][1]=16.80; Bfp[1][2]=7.40; Bfp[1][3]=11.20;
        Bfp[2][0]=14.70; Bfp[2][1]=9.30; Bfp[2][2]=12.10; Bfp[2][3]=17.50;
        Bfp[3][0]=17.50; Bfp[3][1]=12.10; Bfp[3][2]=9.30; Bfp[3][3]=14.70;
        run_test(15, "teens");

        // ---------------- T16: large all unique ----------------
        Afp[0][0]=8.00; Afp[0][1]=9.70; Afp[0][2]=11.40; Afp[0][3]=13.10;
        Afp[1][0]=14.80; Afp[1][1]=16.50; Afp[1][2]=18.20; Afp[1][3]=19.90;
        Afp[2][0]=21.60; Afp[2][1]=23.30; Afp[2][2]=25.00; Afp[2][3]=26.70;
        Afp[3][0]=28.40; Afp[3][1]=30.10; Afp[3][2]=31.80; Afp[3][3]=33.50;
        Bfp[0][0]=30.00; Bfp[0][1]=28.70; Bfp[0][2]=27.40; Bfp[0][3]=26.10;
        Bfp[1][0]=24.80; Bfp[1][1]=23.50; Bfp[1][2]=22.20; Bfp[1][3]=20.90;
        Bfp[2][0]=19.60; Bfp[2][1]=18.30; Bfp[2][2]=17.00; Bfp[2][3]=15.70;
        Bfp[3][0]=14.40; Bfp[3][1]=13.10; Bfp[3][2]=11.80; Bfp[3][3]=10.50;
        run_test(16, "large all unique");

        // ---------------- T17: twenties thirties ----------------
        Afp[0][0]=25.40; Afp[0][1]=31.70; Afp[0][2]=19.20; Afp[0][3]=28.90;
        Afp[1][0]=28.90; Afp[1][1]=19.20; Afp[1][2]=31.70; Afp[1][3]=25.40;
        Afp[2][0]=22.60; Afp[2][1]=34.10; Afp[2][2]=26.80; Afp[2][3]=20.50;
        Afp[3][0]=20.50; Afp[3][1]=26.80; Afp[3][2]=34.10; Afp[3][3]=22.60;
        Bfp[0][0]=22.60; Bfp[0][1]=17.30; Bfp[0][2]=33.10; Bfp[0][3]=26.50;
        Bfp[1][0]=26.50; Bfp[1][1]=33.10; Bfp[1][2]=17.30; Bfp[1][3]=22.60;
        Bfp[2][0]=29.40; Bfp[2][1]=21.70; Bfp[2][2]=24.90; Bfp[2][3]=35.20;
        Bfp[3][0]=35.20; Bfp[3][1]=24.90; Bfp[3][2]=21.70; Bfp[3][3]=29.40;
        run_test(17, "twenties thirties");

        // ---------------- T18: wide range in block ----------------
        Afp[0][0]=1.20; Afp[0][1]=6.50; Afp[0][2]=2.80; Afp[0][3]=14.00;
        Afp[1][0]=14.00; Afp[1][1]=2.80; Afp[1][2]=6.50; Afp[1][3]=1.20;
        Afp[2][0]=3.60; Afp[2][1]=11.20; Afp[2][2]=1.80; Afp[2][3]=8.40;
        Afp[3][0]=8.40; Afp[3][1]=1.80; Afp[3][2]=11.20; Afp[3][3]=3.60;
        Bfp[0][0]=1.50; Bfp[0][1]=9.00; Bfp[0][2]=3.20; Bfp[0][3]=11.50;
        Bfp[1][0]=11.50; Bfp[1][1]=3.20; Bfp[1][2]=9.00; Bfp[1][3]=1.50;
        Bfp[2][0]=2.40; Bfp[2][1]=13.10; Bfp[2][2]=1.70; Bfp[2][3]=7.30;
        Bfp[3][0]=7.30; Bfp[3][1]=1.70; Bfp[3][2]=13.10; Bfp[3][3]=2.40;
        run_test(18, "wide range in block");

        // ---------------- T19: very wide range ----------------
        Afp[0][0]=1.10; Afp[0][1]=15.00; Afp[0][2]=2.30; Afp[0][3]=60.00;
        Afp[1][0]=60.00; Afp[1][1]=2.30; Afp[1][2]=15.00; Afp[1][3]=1.10;
        Afp[2][0]=4.70; Afp[2][1]=28.00; Afp[2][2]=1.60; Afp[2][3]=42.00;
        Afp[3][0]=42.00; Afp[3][1]=1.60; Afp[3][2]=28.00; Afp[3][3]=4.70;
        Bfp[0][0]=1.30; Bfp[0][1]=12.00; Bfp[0][2]=4.50; Bfp[0][3]=45.00;
        Bfp[1][0]=45.00; Bfp[1][1]=4.50; Bfp[1][2]=12.00; Bfp[1][3]=1.30;
        Bfp[2][0]=3.20; Bfp[2][1]=22.00; Bfp[2][2]=1.90; Bfp[2][3]=38.00;
        Bfp[3][0]=38.00; Bfp[3][1]=1.90; Bfp[3][2]=22.00; Bfp[3][3]=3.20;
        run_test(19, "very wide range");

        // ---------------- T20: extreme spread unique ----------------
        Afp[0][0]=1.05; Afp[0][1]=1.50; Afp[0][2]=2.85; Afp[0][3]=5.10;
        Afp[1][0]=8.25; Afp[1][1]=12.30; Afp[1][2]=17.25; Afp[1][3]=23.10;
        Afp[2][0]=29.85; Afp[2][1]=37.50; Afp[2][2]=46.05; Afp[2][3]=55.50;
        Afp[3][0]=65.85; Afp[3][1]=77.10; Afp[3][2]=89.25; Afp[3][3]=102.30;
        Bfp[0][0]=91.10; Bfp[0][1]=79.50; Bfp[0][2]=68.70; Bfp[0][3]=58.70;
        Bfp[1][0]=49.50; Bfp[1][1]=41.10; Bfp[1][2]=33.50; Bfp[1][3]=26.70;
        Bfp[2][0]=20.70; Bfp[2][1]=15.50; Bfp[2][2]=11.10; Bfp[2][3]=7.50;
        Bfp[3][0]=4.70; Bfp[3][1]=2.70; Bfp[3][2]=1.50; Bfp[3][3]=1.10;
        run_test(20, "extreme spread unique");


        // ================= SIGNED / SUB-1 TESTS =================

        // ---------------- T21: all negative ----------------
        Afp[0][0]=-1.50; Afp[0][1]=-2.50; Afp[0][2]=-3.50; Afp[0][3]=-4.50;
        Afp[1][0]=-4.50; Afp[1][1]=-3.50; Afp[1][2]=-2.50; Afp[1][3]=-1.50;
        Afp[2][0]=-2.00; Afp[2][1]=-3.00; Afp[2][2]=-4.00; Afp[2][3]=-5.00;
        Afp[3][0]=-5.00; Afp[3][1]=-4.00; Afp[3][2]=-3.00; Afp[3][3]=-2.00;
        Bfp[0][0]=-2.00; Bfp[0][1]=-3.00; Bfp[0][2]=-1.50; Bfp[0][3]=-4.00;
        Bfp[1][0]=-4.00; Bfp[1][1]=-1.50; Bfp[1][2]=-3.00; Bfp[1][3]=-2.00;
        Bfp[2][0]=-2.50; Bfp[2][1]=-3.50; Bfp[2][2]=-4.50; Bfp[2][3]=-1.50;
        Bfp[3][0]=-1.50; Bfp[3][1]=-4.50; Bfp[3][2]=-3.50; Bfp[3][3]=-2.50;
        run_test(21, "all negative");

        // ---------------- T22: mixed sign, no cancellation ----------------
        Afp[0][0]= 2.50; Afp[0][1]=-3.70; Afp[0][2]= 1.40; Afp[0][3]=-4.20;
        Afp[1][0]=-4.20; Afp[1][1]= 1.40; Afp[1][2]=-3.70; Afp[1][3]= 2.50;
        Afp[2][0]= 3.10; Afp[2][1]=-2.80; Afp[2][2]= 4.60; Afp[2][3]=-1.90;
        Afp[3][0]=-1.90; Afp[3][1]= 4.60; Afp[3][2]=-2.80; Afp[3][3]= 3.10;
        Bfp[0][0]=-1.50; Bfp[0][1]= 2.80; Bfp[0][2]=-3.40; Bfp[0][3]= 1.30;
        Bfp[1][0]= 3.40; Bfp[1][1]=-1.30; Bfp[1][2]= 2.80; Bfp[1][3]=-1.50;
        Bfp[2][0]=-2.20; Bfp[2][1]= 4.10; Bfp[2][2]=-1.70; Bfp[2][3]= 3.60;
        Bfp[3][0]= 3.60; Bfp[3][1]=-1.70; Bfp[3][2]= 4.10; Bfp[3][3]=-2.20;
        run_test(22, "mixed sign");

        // ---------------- T23: mixed sign WITH cancellation ----------------
        Afp[0][0]= 2.00; Afp[0][1]=-3.00; Afp[0][2]= 4.00; Afp[0][3]=-1.50;
        Afp[1][0]= 1.50; Afp[1][1]=-2.00; Afp[1][2]= 3.00; Afp[1][3]=-4.00;
        Afp[2][0]=-2.50; Afp[2][1]= 3.50; Afp[2][2]=-1.25; Afp[2][3]= 2.75;
        Afp[3][0]= 3.25; Afp[3][1]=-1.75; Afp[3][2]= 2.25; Afp[3][3]=-3.75;
        Bfp[0][0]=-1.00; Bfp[0][1]= 2.00; Bfp[0][2]=-0.50; Bfp[0][3]= 3.00;
        Bfp[1][0]= 2.00; Bfp[1][1]=-0.50; Bfp[1][2]= 3.00; Bfp[1][3]=-1.00;
        Bfp[2][0]=-0.50; Bfp[2][1]= 3.00; Bfp[2][2]=-1.00; Bfp[2][3]= 2.00;
        Bfp[3][0]= 3.00; Bfp[3][1]=-1.00; Bfp[3][2]= 2.00; Bfp[3][3]=-0.50;
        run_test(23, "mixed sign + cancellation");

        // ---------------- T24: sub-1 values (negative logs) ----------------
        Afp[0][0]=0.25; Afp[0][1]=0.50; Afp[0][2]=0.75; Afp[0][3]=0.90;
        Afp[1][0]=0.90; Afp[1][1]=0.75; Afp[1][2]=0.50; Afp[1][3]=0.25;
        Afp[2][0]=0.40; Afp[2][1]=0.60; Afp[2][2]=0.80; Afp[2][3]=0.95;
        Afp[3][0]=0.95; Afp[3][1]=0.80; Afp[3][2]=0.60; Afp[3][3]=0.40;
        Bfp[0][0]=0.30; Bfp[0][1]=0.55; Bfp[0][2]=0.70; Bfp[0][3]=0.85;
        Bfp[1][0]=0.85; Bfp[1][1]=0.70; Bfp[1][2]=0.55; Bfp[1][3]=0.30;
        Bfp[2][0]=0.45; Bfp[2][1]=0.65; Bfp[2][2]=0.35; Bfp[2][3]=0.95;
        Bfp[3][0]=0.95; Bfp[3][1]=0.35; Bfp[3][2]=0.65; Bfp[3][3]=0.45;
        run_test(24, "sub-1 values");

        // ---------------- T25: sub-1 AND negative ----------------
        Afp[0][0]=-0.25; Afp[0][1]= 0.50; Afp[0][2]=-0.75; Afp[0][3]= 0.90;
        Afp[1][0]= 0.90; Afp[1][1]=-0.75; Afp[1][2]= 0.50; Afp[1][3]=-0.25;
        Afp[2][0]=-0.40; Afp[2][1]= 0.60; Afp[2][2]=-0.80; Afp[2][3]= 0.95;
        Afp[3][0]= 0.95; Afp[3][1]=-0.80; Afp[3][2]= 0.60; Afp[3][3]=-0.40;
        Bfp[0][0]= 0.30; Bfp[0][1]=-0.55; Bfp[0][2]= 0.70; Bfp[0][3]=-0.85;
        Bfp[1][0]=-0.85; Bfp[1][1]= 0.70; Bfp[1][2]=-0.55; Bfp[1][3]= 0.30;
        Bfp[2][0]= 0.45; Bfp[2][1]=-0.65; Bfp[2][2]= 0.35; Bfp[2][3]=-0.95;
        Bfp[3][0]=-0.95; Bfp[3][1]= 0.35; Bfp[3][2]=-0.65; Bfp[3][3]= 0.45;
        run_test(25, "sub-1 and negative");

        // ---------------- T26: values straddling 1.0 ----------------
        Afp[0][0]=0.50; Afp[0][1]=2.00; Afp[0][2]=0.80; Afp[0][3]=3.20;
        Afp[1][0]=3.20; Afp[1][1]=0.80; Afp[1][2]=2.00; Afp[1][3]=0.50;
        Afp[2][0]=1.60; Afp[2][1]=0.60; Afp[2][2]=2.40; Afp[2][3]=0.90;
        Afp[3][0]=0.90; Afp[3][1]=2.40; Afp[3][2]=0.60; Afp[3][3]=1.60;
        Bfp[0][0]=0.70; Bfp[0][1]=1.80; Bfp[0][2]=0.40; Bfp[0][3]=2.60;
        Bfp[1][0]=2.60; Bfp[1][1]=0.40; Bfp[1][2]=1.80; Bfp[1][3]=0.70;
        Bfp[2][0]=1.20; Bfp[2][1]=0.95; Bfp[2][2]=3.00; Bfp[2][3]=0.55;
        Bfp[3][0]=0.55; Bfp[3][1]=3.00; Bfp[3][2]=0.95; Bfp[3][3]=1.20;
        run_test(26, "straddling 1.0");

        // ---- Test 27: random 1..10 (benign) ----
        Afp[0][0]=3.142000000; Afp[0][1]=5.898000000; Afp[0][2]=4.330000000; Afp[0][3]=6.435000000;
        Afp[1][0]=6.631000000; Afp[1][1]=1.590000000; Afp[1][2]=1.119000000; Afp[1][3]=8.537000000;
        Afp[2][0]=3.334000000; Afp[2][1]=3.109000000; Afp[2][2]=9.961000000; Afp[2][3]=5.232000000;
        Afp[3][0]=8.528000000; Afp[3][1]=5.287000000; Afp[3][2]=6.752000000; Afp[3][3]=2.356000000;
        Bfp[0][0]=6.714000000; Bfp[0][1]=8.812000000; Bfp[0][2]=5.709000000; Bfp[0][3]=7.671000000;
        Bfp[1][0]=7.043000000; Bfp[1][1]=1.576000000; Bfp[1][2]=7.824000000; Bfp[1][3]=6.320000000;
        Bfp[2][0]=3.711000000; Bfp[2][1]=1.279000000; Bfp[2][2]=8.790000000; Bfp[2][3]=5.255000000;
        Bfp[3][0]=7.469000000; Bfp[3][1]=8.909000000; Bfp[3][2]=7.427000000; Bfp[3][3]=9.290000000;
        run_case(27, "random 1..10 (benign)");

        // ---- Test 28: random 1..10 (benign) ----
        Afp[0][0]=4.555000000; Afp[0][1]=8.208000000; Afp[0][2]=5.002000000; Afp[0][3]=9.420000000;
        Afp[1][0]=8.910000000; Afp[1][1]=1.877000000; Afp[1][2]=2.224000000; Afp[1][3]=2.953000000;
        Afp[2][0]=9.689000000; Afp[2][1]=4.925000000; Afp[2][2]=6.640000000; Afp[2][3]=3.709000000;
        Afp[3][0]=5.565000000; Afp[3][1]=4.473000000; Afp[3][2]=4.158000000; Afp[3][3]=6.266000000;
        Bfp[0][0]=6.258000000; Bfp[0][1]=9.138000000; Bfp[0][2]=7.138000000; Bfp[0][3]=9.361000000;
        Bfp[1][0]=8.708000000; Bfp[1][1]=9.919000000; Bfp[1][2]=7.041000000; Bfp[1][3]=2.468000000;
        Bfp[2][0]=8.746000000; Bfp[2][1]=9.682000000; Bfp[2][2]=9.142000000; Bfp[2][3]=6.122000000;
        Bfp[3][0]=7.424000000; Bfp[3][1]=2.900000000; Bfp[3][2]=8.484000000; Bfp[3][3]=6.162000000;
        run_case(28, "random 1..10 (benign)");

        // ---- Test 29: random 1..10 (benign) ----
        Afp[0][0]=3.565000000; Afp[0][1]=1.571000000; Afp[0][2]=8.685000000; Afp[0][3]=9.908000000;
        Afp[1][0]=1.797000000; Afp[1][1]=8.205000000; Afp[1][2]=4.694000000; Afp[1][3]=2.357000000;
        Afp[2][0]=3.645000000; Afp[2][1]=7.919000000; Afp[2][2]=8.855000000; Afp[2][3]=1.398000000;
        Afp[3][0]=6.531000000; Afp[3][1]=1.404000000; Afp[3][2]=7.466000000; Afp[3][3]=3.979000000;
        Bfp[0][0]=8.928000000; Bfp[0][1]=9.826000000; Bfp[0][2]=5.549000000; Bfp[0][3]=9.987000000;
        Bfp[1][0]=3.787000000; Bfp[1][1]=1.693000000; Bfp[1][2]=6.398000000; Bfp[1][3]=1.282000000;
        Bfp[2][0]=2.776000000; Bfp[2][1]=4.671000000; Bfp[2][2]=6.494000000; Bfp[2][3]=2.406000000;
        Bfp[3][0]=1.382000000; Bfp[3][1]=8.810000000; Bfp[3][2]=3.824000000; Bfp[3][3]=9.628000000;
        run_case(29, "random 1..10 (benign)");

        // ---- Test 30: random 1..10 (benign) ----
        Afp[0][0]=9.070000000; Afp[0][1]=4.400000000; Afp[0][2]=5.144000000; Afp[0][3]=5.681000000;
        Afp[1][0]=6.795000000; Afp[1][1]=6.361000000; Afp[1][2]=6.033000000; Afp[1][3]=6.581000000;
        Afp[2][0]=9.466000000; Afp[2][1]=5.563000000; Afp[2][2]=4.881000000; Afp[2][3]=7.483000000;
        Afp[3][0]=3.139000000; Afp[3][1]=3.710000000; Afp[3][2]=9.800000000; Afp[3][3]=5.690000000;
        Bfp[0][0]=5.936000000; Bfp[0][1]=1.103000000; Bfp[0][2]=4.737000000; Bfp[0][3]=6.220000000;
        Bfp[1][0]=1.180000000; Bfp[1][1]=6.542000000; Bfp[1][2]=6.690000000; Bfp[1][3]=1.541000000;
        Bfp[2][0]=6.646000000; Bfp[2][1]=5.196000000; Bfp[2][2]=7.114000000; Bfp[2][3]=4.173000000;
        Bfp[3][0]=7.363000000; Bfp[3][1]=7.642000000; Bfp[3][2]=1.200000000; Bfp[3][3]=1.545000000;
        run_case(30, "random 1..10 (benign)");

        // ---- Test 31: random 1..10 (benign) ----
        Afp[0][0]=7.084000000; Afp[0][1]=9.670000000; Afp[0][2]=3.260000000; Afp[0][3]=5.107000000;
        Afp[1][0]=6.334000000; Afp[1][1]=3.880000000; Afp[1][2]=4.276000000; Afp[1][3]=3.814000000;
        Afp[2][0]=4.322000000; Afp[2][1]=6.361000000; Afp[2][2]=3.704000000; Afp[2][3]=4.394000000;
        Afp[3][0]=7.950000000; Afp[3][1]=1.242000000; Afp[3][2]=6.123000000; Afp[3][3]=7.617000000;
        Bfp[0][0]=3.790000000; Bfp[0][1]=3.003000000; Bfp[0][2]=8.234000000; Bfp[0][3]=3.148000000;
        Bfp[1][0]=2.687000000; Bfp[1][1]=4.917000000; Bfp[1][2]=7.283000000; Bfp[1][3]=1.917000000;
        Bfp[2][0]=3.898000000; Bfp[2][1]=4.004000000; Bfp[2][2]=8.502000000; Bfp[2][3]=4.946000000;
        Bfp[3][0]=8.700000000; Bfp[3][1]=2.524000000; Bfp[3][2]=4.030000000; Bfp[3][3]=6.852000000;
        run_case(31, "random 1..10 (benign)");

        // ---- Test 32: random 1..10 (benign) ----
        Afp[0][0]=8.964000000; Afp[0][1]=5.060000000; Afp[0][2]=3.025000000; Afp[0][3]=2.088000000;
        Afp[1][0]=5.767000000; Afp[1][1]=2.717000000; Afp[1][2]=8.261000000; Afp[1][3]=8.546000000;
        Afp[2][0]=2.652000000; Afp[2][1]=3.507000000; Afp[2][2]=8.265000000; Afp[2][3]=6.777000000;
        Afp[3][0]=8.256000000; Afp[3][1]=4.108000000; Afp[3][2]=2.167000000; Afp[3][3]=3.627000000;
        Bfp[0][0]=8.145000000; Bfp[0][1]=3.441000000; Bfp[0][2]=4.117000000; Bfp[0][3]=4.752000000;
        Bfp[1][0]=4.778000000; Bfp[1][1]=4.686000000; Bfp[1][2]=9.286000000; Bfp[1][3]=2.404000000;
        Bfp[2][0]=1.042000000; Bfp[2][1]=9.489000000; Bfp[2][2]=8.920000000; Bfp[2][3]=9.882000000;
        Bfp[3][0]=4.909000000; Bfp[3][1]=9.551000000; Bfp[3][2]=9.346000000; Bfp[3][3]=2.999000000;
        run_case(32, "random 1..10 (benign)");

        // ---- Test 33: random 1..10 (benign) ----
        Afp[0][0]=7.710000000; Afp[0][1]=8.530000000; Afp[0][2]=6.967000000; Afp[0][3]=5.671000000;
        Afp[1][0]=3.601000000; Afp[1][1]=4.070000000; Afp[1][2]=3.047000000; Afp[1][3]=1.613000000;
        Afp[2][0]=6.298000000; Afp[2][1]=3.583000000; Afp[2][2]=8.292000000; Afp[2][3]=1.406000000;
        Afp[3][0]=9.132000000; Afp[3][1]=7.243000000; Afp[3][2]=9.315000000; Afp[3][3]=9.069000000;
        Bfp[0][0]=9.097000000; Bfp[0][1]=6.193000000; Bfp[0][2]=1.118000000; Bfp[0][3]=7.708000000;
        Bfp[1][0]=2.546000000; Bfp[1][1]=3.699000000; Bfp[1][2]=6.966000000; Bfp[1][3]=5.725000000;
        Bfp[2][0]=4.724000000; Bfp[2][1]=9.451000000; Bfp[2][2]=6.509000000; Bfp[2][3]=4.072000000;
        Bfp[3][0]=3.272000000; Bfp[3][1]=8.755000000; Bfp[3][2]=5.295000000; Bfp[3][3]=8.041000000;
        run_case(33, "random 1..10 (benign)");

        // ---- Test 34: random 1..10 (benign) ----
        Afp[0][0]=4.167000000; Afp[0][1]=2.776000000; Afp[0][2]=5.812000000; Afp[0][3]=8.351000000;
        Afp[1][0]=2.542000000; Afp[1][1]=8.125000000; Afp[1][2]=9.296000000; Afp[1][3]=8.254000000;
        Afp[2][0]=8.411000000; Afp[2][1]=1.068000000; Afp[2][2]=6.657000000; Afp[2][3]=8.763000000;
        Afp[3][0]=1.449000000; Afp[3][1]=3.443000000; Afp[3][2]=3.417000000; Afp[3][3]=5.745000000;
        Bfp[0][0]=4.807000000; Bfp[0][1]=5.256000000; Bfp[0][2]=7.988000000; Bfp[0][3]=1.016000000;
        Bfp[1][0]=1.494000000; Bfp[1][1]=2.142000000; Bfp[1][2]=2.122000000; Bfp[1][3]=1.616000000;
        Bfp[2][0]=9.772000000; Bfp[2][1]=8.690000000; Bfp[2][2]=1.775000000; Bfp[2][3]=5.519000000;
        Bfp[3][0]=3.843000000; Bfp[3][1]=3.831000000; Bfp[3][2]=4.162000000; Bfp[3][3]=6.822000000;
        run_case(34, "random 1..10 (benign)");

        // ---- Test 35: random 1..10 (benign) ----
        Afp[0][0]=6.280000000; Afp[0][1]=4.248000000; Afp[0][2]=2.720000000; Afp[0][3]=3.959000000;
        Afp[1][0]=2.114000000; Afp[1][1]=6.000000000; Afp[1][2]=7.444000000; Afp[1][3]=4.422000000;
        Afp[2][0]=1.719000000; Afp[2][1]=2.607000000; Afp[2][2]=4.359000000; Afp[2][3]=6.440000000;
        Afp[3][0]=8.044000000; Afp[3][1]=4.422000000; Afp[3][2]=8.210000000; Afp[3][3]=6.606000000;
        Bfp[0][0]=4.884000000; Bfp[0][1]=4.352000000; Bfp[0][2]=5.465000000; Bfp[0][3]=7.326000000;
        Bfp[1][0]=4.785000000; Bfp[1][1]=7.247000000; Bfp[1][2]=5.148000000; Bfp[1][3]=3.206000000;
        Bfp[2][0]=5.823000000; Bfp[2][1]=7.257000000; Bfp[2][2]=1.644000000; Bfp[2][3]=4.824000000;
        Bfp[3][0]=4.833000000; Bfp[3][1]=8.917000000; Bfp[3][2]=9.428000000; Bfp[3][3]=4.368000000;
        run_case(35, "random 1..10 (benign)");

        // ---- Test 36: random 1..10 (benign) ----
        Afp[0][0]=9.081000000; Afp[0][1]=8.118000000; Afp[0][2]=3.360000000; Afp[0][3]=5.177000000;
        Afp[1][0]=2.108000000; Afp[1][1]=8.319000000; Afp[1][2]=6.961000000; Afp[1][3]=8.986000000;
        Afp[2][0]=8.132000000; Afp[2][1]=7.008000000; Afp[2][2]=7.604000000; Afp[2][3]=6.075000000;
        Afp[3][0]=1.928000000; Afp[3][1]=6.290000000; Afp[3][2]=1.044000000; Afp[3][3]=2.292000000;
        Bfp[0][0]=7.969000000; Bfp[0][1]=1.399000000; Bfp[0][2]=1.826000000; Bfp[0][3]=1.894000000;
        Bfp[1][0]=8.924000000; Bfp[1][1]=2.612000000; Bfp[1][2]=1.211000000; Bfp[1][3]=8.574000000;
        Bfp[2][0]=2.092000000; Bfp[2][1]=8.595000000; Bfp[2][2]=7.062000000; Bfp[2][3]=8.526000000;
        Bfp[3][0]=9.572000000; Bfp[3][1]=6.212000000; Bfp[3][2]=8.189000000; Bfp[3][3]=1.326000000;
        run_case(36, "random 1..10 (benign)");

        // ---- Test 37: random near-1 (best case) ----
        Afp[0][0]=1.053500000; Afp[0][1]=1.002300000; Afp[0][2]=1.043000000; Afp[0][3]=0.921300000;
        Afp[1][0]=1.049800000; Afp[1][1]=1.086900000; Afp[1][2]=0.912200000; Afp[1][3]=0.964800000;
        Afp[2][0]=1.012800000; Afp[2][1]=1.065600000; Afp[2][2]=0.948400000; Afp[2][3]=0.936000000;
        Afp[3][0]=0.950000000; Afp[3][1]=1.023200000; Afp[3][2]=1.050700000; Afp[3][3]=0.978700000;
        Bfp[0][0]=0.973500000; Bfp[0][1]=0.979300000; Bfp[0][2]=0.970100000; Bfp[0][3]=0.983600000;
        Bfp[1][0]=0.916700000; Bfp[1][1]=1.000100000; Bfp[1][2]=1.094600000; Bfp[1][3]=0.982600000;
        Bfp[2][0]=1.049500000; Bfp[2][1]=0.932100000; Bfp[2][2]=1.038200000; Bfp[2][3]=1.051200000;
        Bfp[3][0]=1.034800000; Bfp[3][1]=1.003400000; Bfp[3][2]=0.996700000; Bfp[3][3]=1.028600000;
        run_case(37, "random near-1 (best case)");

        // ---- Test 38: random near-1 (best case) ----
        Afp[0][0]=1.079500000; Afp[0][1]=0.929900000; Afp[0][2]=0.919200000; Afp[0][3]=1.049600000;
        Afp[1][0]=1.083300000; Afp[1][1]=1.003500000; Afp[1][2]=0.988600000; Afp[1][3]=1.043800000;
        Afp[2][0]=0.937200000; Afp[2][1]=0.953500000; Afp[2][2]=0.939800000; Afp[2][3]=1.017100000;
        Afp[3][0]=0.963000000; Afp[3][1]=0.946500000; Afp[3][2]=1.038200000; Afp[3][3]=1.090700000;
        Bfp[0][0]=0.959200000; Bfp[0][1]=1.041100000; Bfp[0][2]=0.982600000; Bfp[0][3]=1.070700000;
        Bfp[1][0]=1.016900000; Bfp[1][1]=0.953400000; Bfp[1][2]=0.943500000; Bfp[1][3]=0.904600000;
        Bfp[2][0]=0.995900000; Bfp[2][1]=0.976600000; Bfp[2][2]=0.934400000; Bfp[2][3]=0.972100000;
        Bfp[3][0]=0.964400000; Bfp[3][1]=1.054800000; Bfp[3][2]=0.928700000; Bfp[3][3]=1.098200000;
        run_case(38, "random near-1 (best case)");

        // ---- Test 39: random near-1 (best case) ----
        Afp[0][0]=0.995900000; Afp[0][1]=1.019800000; Afp[0][2]=0.993600000; Afp[0][3]=1.066900000;
        Afp[1][0]=1.064300000; Afp[1][1]=1.011400000; Afp[1][2]=0.996300000; Afp[1][3]=1.044100000;
        Afp[2][0]=1.071300000; Afp[2][1]=0.980100000; Afp[2][2]=1.046700000; Afp[2][3]=1.092100000;
        Afp[3][0]=0.993500000; Afp[3][1]=0.945900000; Afp[3][2]=0.947000000; Afp[3][3]=1.043500000;
        Bfp[0][0]=1.035100000; Bfp[0][1]=1.091700000; Bfp[0][2]=1.070800000; Bfp[0][3]=0.948400000;
        Bfp[1][0]=0.937900000; Bfp[1][1]=0.951700000; Bfp[1][2]=0.937400000; Bfp[1][3]=1.040900000;
        Bfp[2][0]=1.071700000; Bfp[2][1]=1.080000000; Bfp[2][2]=0.951000000; Bfp[2][3]=1.073000000;
        Bfp[3][0]=0.962700000; Bfp[3][1]=0.984700000; Bfp[3][2]=1.045800000; Bfp[3][3]=0.917200000;
        run_case(39, "random near-1 (best case)");

        // ---- Test 40: random near-1 (best case) ----
        Afp[0][0]=0.918500000; Afp[0][1]=1.066800000; Afp[0][2]=0.958400000; Afp[0][3]=0.971300000;
        Afp[1][0]=1.016100000; Afp[1][1]=1.035100000; Afp[1][2]=0.901400000; Afp[1][3]=0.967000000;
        Afp[2][0]=0.987200000; Afp[2][1]=0.997200000; Afp[2][2]=0.942000000; Afp[2][3]=1.017000000;
        Afp[3][0]=1.091100000; Afp[3][1]=0.978200000; Afp[3][2]=1.008900000; Afp[3][3]=0.923800000;
        Bfp[0][0]=0.955000000; Bfp[0][1]=1.033100000; Bfp[0][2]=0.922500000; Bfp[0][3]=1.077400000;
        Bfp[1][0]=1.081800000; Bfp[1][1]=0.919400000; Bfp[1][2]=1.088300000; Bfp[1][3]=0.974800000;
        Bfp[2][0]=1.054500000; Bfp[2][1]=1.051500000; Bfp[2][2]=0.959100000; Bfp[2][3]=1.035200000;
        Bfp[3][0]=1.030800000; Bfp[3][1]=1.061200000; Bfp[3][2]=0.953100000; Bfp[3][3]=1.050800000;
        run_case(40, "random near-1 (best case)");

        // ---- Test 41: random near-1 (best case) ----
        Afp[0][0]=1.092300000; Afp[0][1]=1.034600000; Afp[0][2]=1.007200000; Afp[0][3]=0.922700000;
        Afp[1][0]=0.998800000; Afp[1][1]=0.970400000; Afp[1][2]=1.043600000; Afp[1][3]=1.035700000;
        Afp[2][0]=1.013300000; Afp[2][1]=0.936400000; Afp[2][2]=1.029100000; Afp[2][3]=1.026200000;
        Afp[3][0]=0.935800000; Afp[3][1]=1.078000000; Afp[3][2]=1.031100000; Afp[3][3]=0.924600000;
        Bfp[0][0]=1.086400000; Bfp[0][1]=0.928300000; Bfp[0][2]=0.966300000; Bfp[0][3]=1.044100000;
        Bfp[1][0]=1.019500000; Bfp[1][1]=1.011000000; Bfp[1][2]=1.029500000; Bfp[1][3]=0.991500000;
        Bfp[2][0]=0.962500000; Bfp[2][1]=0.935300000; Bfp[2][2]=0.913700000; Bfp[2][3]=1.043200000;
        Bfp[3][0]=1.050900000; Bfp[3][1]=1.008600000; Bfp[3][2]=1.047900000; Bfp[3][3]=0.971800000;
        run_case(41, "random near-1 (best case)");

        // ---- Test 42: random near-1 (best case) ----
        Afp[0][0]=0.953200000; Afp[0][1]=0.976700000; Afp[0][2]=1.074500000; Afp[0][3]=0.908400000;
        Afp[1][0]=1.000900000; Afp[1][1]=0.949400000; Afp[1][2]=1.053800000; Afp[1][3]=0.970800000;
        Afp[2][0]=0.966600000; Afp[2][1]=0.980700000; Afp[2][2]=1.008300000; Afp[2][3]=1.054300000;
        Afp[3][0]=0.970600000; Afp[3][1]=1.069400000; Afp[3][2]=0.922400000; Afp[3][3]=0.954100000;
        Bfp[0][0]=0.919900000; Bfp[0][1]=0.922500000; Bfp[0][2]=1.055800000; Bfp[0][3]=1.045500000;
        Bfp[1][0]=0.937000000; Bfp[1][1]=0.937800000; Bfp[1][2]=0.983300000; Bfp[1][3]=1.048700000;
        Bfp[2][0]=1.063100000; Bfp[2][1]=1.049700000; Bfp[2][2]=1.018400000; Bfp[2][3]=0.929300000;
        Bfp[3][0]=0.979700000; Bfp[3][1]=0.938700000; Bfp[3][2]=1.005500000; Bfp[3][3]=1.013700000;
        run_case(42, "random near-1 (best case)");

        // ---- Test 43: random near-1 (best case) ----
        Afp[0][0]=0.940400000; Afp[0][1]=0.950000000; Afp[0][2]=1.056300000; Afp[0][3]=0.906000000;
        Afp[1][0]=1.060600000; Afp[1][1]=1.078200000; Afp[1][2]=1.089900000; Afp[1][3]=0.976600000;
        Afp[2][0]=1.010500000; Afp[2][1]=1.016600000; Afp[2][2]=1.026700000; Afp[2][3]=1.095400000;
        Afp[3][0]=1.037300000; Afp[3][1]=0.959900000; Afp[3][2]=1.072000000; Afp[3][3]=0.996800000;
        Bfp[0][0]=1.020300000; Bfp[0][1]=1.045400000; Bfp[0][2]=0.900500000; Bfp[0][3]=1.054100000;
        Bfp[1][0]=1.032400000; Bfp[1][1]=0.998400000; Bfp[1][2]=1.004700000; Bfp[1][3]=0.992100000;
        Bfp[2][0]=0.938700000; Bfp[2][1]=1.005900000; Bfp[2][2]=0.907400000; Bfp[2][3]=1.000100000;
        Bfp[3][0]=1.029200000; Bfp[3][1]=0.988800000; Bfp[3][2]=1.013200000; Bfp[3][3]=1.091800000;
        run_case(43, "random near-1 (best case)");

        // ---- Test 44: random near-1 (best case) ----
        Afp[0][0]=1.078400000; Afp[0][1]=0.927100000; Afp[0][2]=1.058500000; Afp[0][3]=1.024700000;
        Afp[1][0]=0.910100000; Afp[1][1]=0.972000000; Afp[1][2]=0.946700000; Afp[1][3]=0.915600000;
        Afp[2][0]=1.007800000; Afp[2][1]=1.086000000; Afp[2][2]=0.964600000; Afp[2][3]=1.074100000;
        Afp[3][0]=1.038900000; Afp[3][1]=0.926900000; Afp[3][2]=1.071700000; Afp[3][3]=1.020200000;
        Bfp[0][0]=1.085400000; Bfp[0][1]=1.043200000; Bfp[0][2]=1.047900000; Bfp[0][3]=0.968700000;
        Bfp[1][0]=1.061300000; Bfp[1][1]=1.086300000; Bfp[1][2]=1.072300000; Bfp[1][3]=0.987400000;
        Bfp[2][0]=1.051400000; Bfp[2][1]=0.997000000; Bfp[2][2]=0.921800000; Bfp[2][3]=0.908500000;
        Bfp[3][0]=0.915600000; Bfp[3][1]=0.940100000; Bfp[3][2]=0.932200000; Bfp[3][3]=0.999400000;
        run_case(44, "random near-1 (best case)");

        // ---- Test 45: random sub-1 ----
        Afp[0][0]=0.722000000; Afp[0][1]=0.578000000; Afp[0][2]=0.476000000; Afp[0][3]=0.678000000;
        Afp[1][0]=0.371000000; Afp[1][1]=0.513000000; Afp[1][2]=0.774000000; Afp[1][3]=0.457000000;
        Afp[2][0]=0.261000000; Afp[2][1]=0.900000000; Afp[2][2]=0.741000000; Afp[2][3]=0.427000000;
        Afp[3][0]=0.430000000; Afp[3][1]=0.571000000; Afp[3][2]=0.631000000; Afp[3][3]=0.299000000;
        Bfp[0][0]=0.102000000; Bfp[0][1]=0.286000000; Bfp[0][2]=0.797000000; Bfp[0][3]=0.228000000;
        Bfp[1][0]=0.509000000; Bfp[1][1]=0.274000000; Bfp[1][2]=0.286000000; Bfp[1][3]=0.252000000;
        Bfp[2][0]=0.459000000; Bfp[2][1]=0.250000000; Bfp[2][2]=0.124000000; Bfp[2][3]=0.198000000;
        Bfp[3][0]=0.250000000; Bfp[3][1]=0.536000000; Bfp[3][2]=0.153000000; Bfp[3][3]=0.120000000;
        run_case(45, "random sub-1");

        // ---- Test 46: random sub-1 ----
        Afp[0][0]=0.499000000; Afp[0][1]=0.463000000; Afp[0][2]=0.726000000; Afp[0][3]=0.145000000;
        Afp[1][0]=0.459000000; Afp[1][1]=0.453000000; Afp[1][2]=0.124000000; Afp[1][3]=0.959000000;
        Afp[2][0]=0.295000000; Afp[2][1]=0.184000000; Afp[2][2]=0.522000000; Afp[2][3]=0.247000000;
        Afp[3][0]=0.654000000; Afp[3][1]=0.408000000; Afp[3][2]=0.210000000; Afp[3][3]=0.146000000;
        Bfp[0][0]=0.748000000; Bfp[0][1]=0.345000000; Bfp[0][2]=0.801000000; Bfp[0][3]=0.514000000;
        Bfp[1][0]=0.930000000; Bfp[1][1]=0.367000000; Bfp[1][2]=0.322000000; Bfp[1][3]=0.337000000;
        Bfp[2][0]=0.825000000; Bfp[2][1]=0.660000000; Bfp[2][2]=0.407000000; Bfp[2][3]=0.183000000;
        Bfp[3][0]=0.707000000; Bfp[3][1]=0.963000000; Bfp[3][2]=0.627000000; Bfp[3][3]=0.103000000;
        run_case(46, "random sub-1");

        // ---- Test 47: random sub-1 ----
        Afp[0][0]=0.127000000; Afp[0][1]=0.181000000; Afp[0][2]=0.252000000; Afp[0][3]=0.133000000;
        Afp[1][0]=0.148000000; Afp[1][1]=0.682000000; Afp[1][2]=0.901000000; Afp[1][3]=0.279000000;
        Afp[2][0]=0.967000000; Afp[2][1]=0.524000000; Afp[2][2]=0.815000000; Afp[2][3]=0.916000000;
        Afp[3][0]=0.937000000; Afp[3][1]=0.130000000; Afp[3][2]=0.371000000; Afp[3][3]=0.640000000;
        Bfp[0][0]=0.942000000; Bfp[0][1]=0.178000000; Bfp[0][2]=0.361000000; Bfp[0][3]=0.856000000;
        Bfp[1][0]=0.202000000; Bfp[1][1]=0.447000000; Bfp[1][2]=0.397000000; Bfp[1][3]=0.705000000;
        Bfp[2][0]=0.926000000; Bfp[2][1]=0.255000000; Bfp[2][2]=0.758000000; Bfp[2][3]=0.753000000;
        Bfp[3][0]=0.844000000; Bfp[3][1]=0.592000000; Bfp[3][2]=0.922000000; Bfp[3][3]=0.423000000;
        run_case(47, "random sub-1");

        // ---- Test 48: random sub-1 ----
        Afp[0][0]=0.469000000; Afp[0][1]=0.304000000; Afp[0][2]=0.794000000; Afp[0][3]=0.528000000;
        Afp[1][0]=0.340000000; Afp[1][1]=0.251000000; Afp[1][2]=0.741000000; Afp[1][3]=0.639000000;
        Afp[2][0]=0.732000000; Afp[2][1]=0.444000000; Afp[2][2]=0.534000000; Afp[2][3]=0.237000000;
        Afp[3][0]=0.733000000; Afp[3][1]=0.120000000; Afp[3][2]=0.516000000; Afp[3][3]=0.775000000;
        Bfp[0][0]=0.703000000; Bfp[0][1]=0.186000000; Bfp[0][2]=0.311000000; Bfp[0][3]=0.851000000;
        Bfp[1][0]=0.672000000; Bfp[1][1]=0.882000000; Bfp[1][2]=0.876000000; Bfp[1][3]=0.500000000;
        Bfp[2][0]=0.898000000; Bfp[2][1]=0.752000000; Bfp[2][2]=0.397000000; Bfp[2][3]=0.429000000;
        Bfp[3][0]=0.164000000; Bfp[3][1]=0.455000000; Bfp[3][2]=0.951000000; Bfp[3][3]=0.193000000;
        run_case(48, "random sub-1");

        // ---- Test 49: random sub-1 ----
        Afp[0][0]=0.606000000; Afp[0][1]=0.198000000; Afp[0][2]=0.172000000; Afp[0][3]=0.678000000;
        Afp[1][0]=0.314000000; Afp[1][1]=0.143000000; Afp[1][2]=0.236000000; Afp[1][3]=0.674000000;
        Afp[2][0]=0.621000000; Afp[2][1]=0.110000000; Afp[2][2]=0.305000000; Afp[2][3]=0.961000000;
        Afp[3][0]=0.296000000; Afp[3][1]=0.601000000; Afp[3][2]=0.473000000; Afp[3][3]=0.795000000;
        Bfp[0][0]=0.638000000; Bfp[0][1]=0.802000000; Bfp[0][2]=0.576000000; Bfp[0][3]=0.267000000;
        Bfp[1][0]=0.258000000; Bfp[1][1]=0.170000000; Bfp[1][2]=0.835000000; Bfp[1][3]=0.200000000;
        Bfp[2][0]=0.121000000; Bfp[2][1]=0.960000000; Bfp[2][2]=0.277000000; Bfp[2][3]=0.895000000;
        Bfp[3][0]=0.176000000; Bfp[3][1]=0.514000000; Bfp[3][2]=0.298000000; Bfp[3][3]=0.838000000;
        run_case(49, "random sub-1");

        // ---- Test 50: random sub-1 ----
        Afp[0][0]=0.648000000; Afp[0][1]=0.671000000; Afp[0][2]=0.778000000; Afp[0][3]=0.876000000;
        Afp[1][0]=0.408000000; Afp[1][1]=0.637000000; Afp[1][2]=0.497000000; Afp[1][3]=0.199000000;
        Afp[2][0]=0.843000000; Afp[2][1]=0.629000000; Afp[2][2]=0.825000000; Afp[2][3]=0.283000000;
        Afp[3][0]=0.580000000; Afp[3][1]=0.513000000; Afp[3][2]=0.748000000; Afp[3][3]=0.169000000;
        Bfp[0][0]=0.408000000; Bfp[0][1]=0.531000000; Bfp[0][2]=0.164000000; Bfp[0][3]=0.592000000;
        Bfp[1][0]=0.754000000; Bfp[1][1]=0.476000000; Bfp[1][2]=0.677000000; Bfp[1][3]=0.639000000;
        Bfp[2][0]=0.291000000; Bfp[2][1]=0.412000000; Bfp[2][2]=0.986000000; Bfp[2][3]=0.398000000;
        Bfp[3][0]=0.483000000; Bfp[3][1]=0.175000000; Bfp[3][2]=0.294000000; Bfp[3][3]=0.247000000;
        run_case(50, "random sub-1");

        // ---- Test 51: random signed mild ----
        Afp[0][0]=-6.358000000; Afp[0][1]=7.588000000; Afp[0][2]=4.207000000; Afp[0][3]=-7.419000000;
        Afp[1][0]=4.905000000; Afp[1][1]=4.816000000; Afp[1][2]=-3.752000000; Afp[1][3]=-3.108000000;
        Afp[2][0]=-2.170000000; Afp[2][1]=-3.766000000; Afp[2][2]=-5.836000000; Afp[2][3]=-3.169000000;
        Afp[3][0]=-3.656000000; Afp[3][1]=2.226000000; Afp[3][2]=6.453000000; Afp[3][3]=2.858000000;
        Bfp[0][0]=3.796000000; Bfp[0][1]=4.663000000; Bfp[0][2]=-2.919000000; Bfp[0][3]=7.858000000;
        Bfp[1][0]=7.657000000; Bfp[1][1]=3.872000000; Bfp[1][2]=4.845000000; Bfp[1][3]=-4.158000000;
        Bfp[2][0]=3.711000000; Bfp[2][1]=4.241000000; Bfp[2][2]=6.843000000; Bfp[2][3]=6.485000000;
        Bfp[3][0]=-2.086000000; Bfp[3][1]=-5.215000000; Bfp[3][2]=-7.701000000; Bfp[3][3]=-3.224000000;
        run_case(51, "random signed mild");

        // ---- Test 52: random signed mild ----
        Afp[0][0]=4.129000000; Afp[0][1]=7.875000000; Afp[0][2]=-4.941000000; Afp[0][3]=-2.079000000;
        Afp[1][0]=7.427000000; Afp[1][1]=-5.621000000; Afp[1][2]=-4.700000000; Afp[1][3]=3.483000000;
        Afp[2][0]=4.509000000; Afp[2][1]=4.037000000; Afp[2][2]=-4.577000000; Afp[2][3]=-7.475000000;
        Afp[3][0]=-7.721000000; Afp[3][1]=5.053000000; Afp[3][2]=5.311000000; Afp[3][3]=2.100000000;
        Bfp[0][0]=-2.102000000; Bfp[0][1]=-7.382000000; Bfp[0][2]=4.805000000; Bfp[0][3]=4.226000000;
        Bfp[1][0]=-3.762000000; Bfp[1][1]=-6.103000000; Bfp[1][2]=-4.449000000; Bfp[1][3]=4.665000000;
        Bfp[2][0]=6.818000000; Bfp[2][1]=3.698000000; Bfp[2][2]=-5.463000000; Bfp[2][3]=-5.132000000;
        Bfp[3][0]=4.913000000; Bfp[3][1]=-6.966000000; Bfp[3][2]=2.184000000; Bfp[3][3]=-4.926000000;
        run_case(52, "random signed mild");

        // ---- Test 53: random signed mild ----
        Afp[0][0]=6.713000000; Afp[0][1]=2.615000000; Afp[0][2]=-4.228000000; Afp[0][3]=7.374000000;
        Afp[1][0]=3.171000000; Afp[1][1]=3.087000000; Afp[1][2]=7.649000000; Afp[1][3]=3.897000000;
        Afp[2][0]=3.285000000; Afp[2][1]=-7.663000000; Afp[2][2]=5.663000000; Afp[2][3]=2.213000000;
        Afp[3][0]=3.501000000; Afp[3][1]=-4.971000000; Afp[3][2]=-6.425000000; Afp[3][3]=-6.458000000;
        Bfp[0][0]=-5.787000000; Bfp[0][1]=-7.653000000; Bfp[0][2]=-4.588000000; Bfp[0][3]=4.771000000;
        Bfp[1][0]=5.277000000; Bfp[1][1]=-3.048000000; Bfp[1][2]=7.745000000; Bfp[1][3]=-2.899000000;
        Bfp[2][0]=-6.140000000; Bfp[2][1]=6.048000000; Bfp[2][2]=-2.853000000; Bfp[2][3]=-3.082000000;
        Bfp[3][0]=-3.866000000; Bfp[3][1]=-7.249000000; Bfp[3][2]=-5.144000000; Bfp[3][3]=-5.208000000;
        run_case(53, "random signed mild");

        // ---- Test 54: random signed mild ----
        Afp[0][0]=6.673000000; Afp[0][1]=4.339000000; Afp[0][2]=-3.685000000; Afp[0][3]=-2.691000000;
        Afp[1][0]=2.761000000; Afp[1][1]=7.627000000; Afp[1][2]=-7.607000000; Afp[1][3]=-7.636000000;
        Afp[2][0]=-7.880000000; Afp[2][1]=-2.386000000; Afp[2][2]=2.623000000; Afp[2][3]=4.836000000;
        Afp[3][0]=5.185000000; Afp[3][1]=5.413000000; Afp[3][2]=5.228000000; Afp[3][3]=4.654000000;
        Bfp[0][0]=6.998000000; Bfp[0][1]=-6.540000000; Bfp[0][2]=7.597000000; Bfp[0][3]=-3.804000000;
        Bfp[1][0]=-3.901000000; Bfp[1][1]=-2.782000000; Bfp[1][2]=-4.456000000; Bfp[1][3]=-2.687000000;
        Bfp[2][0]=-2.948000000; Bfp[2][1]=-6.764000000; Bfp[2][2]=5.151000000; Bfp[2][3]=2.555000000;
        Bfp[3][0]=3.955000000; Bfp[3][1]=-3.939000000; Bfp[3][2]=5.820000000; Bfp[3][3]=6.215000000;
        run_case(54, "random signed mild");

        // ---- Test 55: random signed mild ----
        Afp[0][0]=7.173000000; Afp[0][1]=3.721000000; Afp[0][2]=3.619000000; Afp[0][3]=-5.612000000;
        Afp[1][0]=-6.393000000; Afp[1][1]=6.373000000; Afp[1][2]=-3.694000000; Afp[1][3]=4.717000000;
        Afp[2][0]=4.007000000; Afp[2][1]=-7.919000000; Afp[2][2]=7.967000000; Afp[2][3]=-2.416000000;
        Afp[3][0]=5.299000000; Afp[3][1]=2.356000000; Afp[3][2]=4.304000000; Afp[3][3]=-4.200000000;
        Bfp[0][0]=-2.332000000; Bfp[0][1]=7.303000000; Bfp[0][2]=6.393000000; Bfp[0][3]=6.268000000;
        Bfp[1][0]=2.071000000; Bfp[1][1]=5.032000000; Bfp[1][2]=4.636000000; Bfp[1][3]=-5.506000000;
        Bfp[2][0]=-3.940000000; Bfp[2][1]=-4.226000000; Bfp[2][2]=-2.837000000; Bfp[2][3]=-3.255000000;
        Bfp[3][0]=3.698000000; Bfp[3][1]=-2.170000000; Bfp[3][2]=5.206000000; Bfp[3][3]=6.970000000;
        run_case(55, "random signed mild");

        // ---- Test 56: random signed mild ----
        Afp[0][0]=-4.648000000; Afp[0][1]=-5.057000000; Afp[0][2]=3.349000000; Afp[0][3]=-7.342000000;
        Afp[1][0]=5.316000000; Afp[1][1]=-6.942000000; Afp[1][2]=-4.584000000; Afp[1][3]=4.297000000;
        Afp[2][0]=6.995000000; Afp[2][1]=-4.836000000; Afp[2][2]=-4.333000000; Afp[2][3]=5.516000000;
        Afp[3][0]=4.617000000; Afp[3][1]=-5.842000000; Afp[3][2]=-2.386000000; Afp[3][3]=2.396000000;
        Bfp[0][0]=-3.946000000; Bfp[0][1]=5.788000000; Bfp[0][2]=-6.959000000; Bfp[0][3]=5.865000000;
        Bfp[1][0]=-7.305000000; Bfp[1][1]=-5.891000000; Bfp[1][2]=-2.311000000; Bfp[1][3]=-7.417000000;
        Bfp[2][0]=-5.443000000; Bfp[2][1]=-3.080000000; Bfp[2][2]=6.946000000; Bfp[2][3]=3.708000000;
        Bfp[3][0]=5.969000000; Bfp[3][1]=-3.214000000; Bfp[3][2]=4.056000000; Bfp[3][3]=-5.492000000;
        run_case(56, "random signed mild");

        // ---- Test 57: random signed mild ----
        Afp[0][0]=5.720000000; Afp[0][1]=3.889000000; Afp[0][2]=-4.682000000; Afp[0][3]=7.146000000;
        Afp[1][0]=-4.961000000; Afp[1][1]=-3.102000000; Afp[1][2]=-7.626000000; Afp[1][3]=-7.569000000;
        Afp[2][0]=7.053000000; Afp[2][1]=3.119000000; Afp[2][2]=-3.962000000; Afp[2][3]=-4.306000000;
        Afp[3][0]=-3.421000000; Afp[3][1]=4.335000000; Afp[3][2]=-3.710000000; Afp[3][3]=3.580000000;
        Bfp[0][0]=6.393000000; Bfp[0][1]=-3.322000000; Bfp[0][2]=3.038000000; Bfp[0][3]=-3.473000000;
        Bfp[1][0]=-3.293000000; Bfp[1][1]=5.656000000; Bfp[1][2]=-6.104000000; Bfp[1][3]=5.591000000;
        Bfp[2][0]=5.587000000; Bfp[2][1]=4.654000000; Bfp[2][2]=-2.472000000; Bfp[2][3]=4.941000000;
        Bfp[3][0]=-3.611000000; Bfp[3][1]=6.114000000; Bfp[3][2]=-4.335000000; Bfp[3][3]=4.076000000;
        run_case(57, "random signed mild");

        // ---- Test 58: random signed mild ----
        Afp[0][0]=6.549000000; Afp[0][1]=-6.301000000; Afp[0][2]=7.186000000; Afp[0][3]=-5.501000000;
        Afp[1][0]=-4.094000000; Afp[1][1]=-7.105000000; Afp[1][2]=5.776000000; Afp[1][3]=-7.179000000;
        Afp[2][0]=4.249000000; Afp[2][1]=-5.497000000; Afp[2][2]=-5.740000000; Afp[2][3]=5.505000000;
        Afp[3][0]=5.249000000; Afp[3][1]=7.016000000; Afp[3][2]=7.542000000; Afp[3][3]=6.072000000;
        Bfp[0][0]=4.537000000; Bfp[0][1]=5.403000000; Bfp[0][2]=-3.772000000; Bfp[0][3]=-5.375000000;
        Bfp[1][0]=6.278000000; Bfp[1][1]=5.324000000; Bfp[1][2]=7.491000000; Bfp[1][3]=3.035000000;
        Bfp[2][0]=4.792000000; Bfp[2][1]=5.317000000; Bfp[2][2]=2.641000000; Bfp[2][3]=-5.639000000;
        Bfp[3][0]=-2.995000000; Bfp[3][1]=-2.715000000; Bfp[3][2]=-7.539000000; Bfp[3][3]=4.384000000;
        run_case(58, "random signed mild");

        // ---- Test 59: NN weights gauss(0,1) ----
        Afp[0][0]=0.736000000; Afp[0][1]=0.324400000; Afp[0][2]=-0.936000000; Afp[0][3]=2.191500000;
        Afp[1][0]=1.801400000; Afp[1][1]=1.633000000; Afp[1][2]=-1.055600000; Afp[1][3]=0.155400000;
        Afp[2][0]=-0.197400000; Afp[2][1]=2.555400000; Afp[2][2]=0.508800000; Afp[2][3]=1.198100000;
        Afp[3][0]=-0.665600000; Afp[3][1]=-0.045100000; Afp[3][2]=0.491700000; Afp[3][3]=2.866100000;
        Bfp[0][0]=0.412300000; Bfp[0][1]=-1.573000000; Bfp[0][2]=0.373300000; Bfp[0][3]=-0.261000000;
        Bfp[1][0]=-0.479500000; Bfp[1][1]=-1.595900000; Bfp[1][2]=0.170400000; Bfp[1][3]=1.092000000;
        Bfp[2][0]=0.878100000; Bfp[2][1]=-0.142400000; Bfp[2][2]=0.046600000; Bfp[2][3]=-0.599300000;
        Bfp[3][0]=-0.394000000; Bfp[3][1]=-0.687500000; Bfp[3][2]=-0.963700000; Bfp[3][3]=-0.053300000;
        run_case(59, "NN weights gauss(0,1)");

        // ---- Test 60: NN weights gauss(0,1) ----
        Afp[0][0]=1.133900000; Afp[0][1]=-1.202600000; Afp[0][2]=-1.524200000; Afp[0][3]=-0.039800000;
        Afp[1][0]=-1.622800000; Afp[1][1]=0.792900000; Afp[1][2]=-0.058700000; Afp[1][3]=1.252100000;
        Afp[2][0]=-0.851000000; Afp[2][1]=-0.508000000; Afp[2][2]=0.584400000; Afp[2][3]=0.176100000;
        Afp[3][0]=-0.437700000; Afp[3][1]=-0.532300000; Afp[3][2]=0.060700000; Afp[3][3]=-1.184200000;
        Bfp[0][0]=1.857000000; Bfp[0][1]=-0.563200000; Bfp[0][2]=-0.136800000; Bfp[0][3]=-0.955700000;
        Bfp[1][0]=1.228100000; Bfp[1][1]=0.344400000; Bfp[1][2]=-0.061300000; Bfp[1][3]=-2.252700000;
        Bfp[2][0]=0.172100000; Bfp[2][1]=0.561600000; Bfp[2][2]=1.627800000; Bfp[2][3]=-0.200700000;
        Bfp[3][0]=-1.630200000; Bfp[3][1]=0.158200000; Bfp[3][2]=0.740300000; Bfp[3][3]=1.011300000;
        run_case(60, "NN weights gauss(0,1)");

        // ---- Test 61: NN weights gauss(0,1) ----
        Afp[0][0]=-0.665700000; Afp[0][1]=-1.186800000; Afp[0][2]=0.287300000; Afp[0][3]=0.460100000;
        Afp[1][0]=-1.470300000; Afp[1][1]=-1.467200000; Afp[1][2]=0.077200000; Afp[1][3]=0.091100000;
        Afp[2][0]=1.521800000; Afp[2][1]=0.871900000; Afp[2][2]=-0.853400000; Afp[2][3]=0.699000000;
        Afp[3][0]=1.373500000; Afp[3][1]=-0.051600000; Afp[3][2]=-0.129900000; Afp[3][3]=1.548400000;
        Bfp[0][0]=0.813100000; Bfp[0][1]=0.010100000; Bfp[0][2]=-0.194200000; Bfp[0][3]=-0.580500000;
        Bfp[1][0]=1.183200000; Bfp[1][1]=0.246400000; Bfp[1][2]=-1.251000000; Bfp[1][3]=2.351500000;
        Bfp[2][0]=1.445600000; Bfp[2][1]=1.072800000; Bfp[2][2]=-1.383100000; Bfp[2][3]=1.166600000;
        Bfp[3][0]=-1.396400000; Bfp[3][1]=0.501700000; Bfp[3][2]=-0.330300000; Bfp[3][3]=0.632200000;
        run_case(61, "NN weights gauss(0,1)");

        // ---- Test 62: NN weights gauss(0,1) ----
        Afp[0][0]=-1.715100000; Afp[0][1]=0.528500000; Afp[0][2]=-0.407400000; Afp[0][3]=0.597100000;
        Afp[1][0]=-0.387700000; Afp[1][1]=0.226500000; Afp[1][2]=-0.523500000; Afp[1][3]=1.194600000;
        Afp[2][0]=-1.294200000; Afp[2][1]=-0.380600000; Afp[2][2]=-0.055800000; Afp[2][3]=0.217000000;
        Afp[3][0]=0.897400000; Afp[3][1]=0.150800000; Afp[3][2]=0.428200000; Afp[3][3]=1.224600000;
        Bfp[0][0]=-0.163200000; Bfp[0][1]=1.686300000; Bfp[0][2]=-1.184600000; Bfp[0][3]=-0.876400000;
        Bfp[1][0]=-0.110300000; Bfp[1][1]=-1.209300000; Bfp[1][2]=-0.769000000; Bfp[1][3]=0.382300000;
        Bfp[2][0]=1.641500000; Bfp[2][1]=0.685100000; Bfp[2][2]=-0.458400000; Bfp[2][3]=-0.000300000;
        Bfp[3][0]=1.182900000; Bfp[3][1]=-0.587600000; Bfp[3][2]=-0.766700000; Bfp[3][3]=-0.753000000;
        run_case(62, "NN weights gauss(0,1)");

        // ---- Test 63: NN weights gauss(0,1) ----
        Afp[0][0]=2.749500000; Afp[0][1]=2.788500000; Afp[0][2]=0.469700000; Afp[0][3]=0.831600000;
        Afp[1][0]=0.510300000; Afp[1][1]=-0.002100000; Afp[1][2]=-1.143000000; Afp[1][3]=0.003600000;
        Afp[2][0]=0.033700000; Afp[2][1]=2.273600000; Afp[2][2]=-0.131000000; Afp[2][3]=0.078400000;
        Afp[3][0]=-0.095800000; Afp[3][1]=0.016100000; Afp[3][2]=-0.519900000; Afp[3][3]=-1.948200000;
        Bfp[0][0]=0.260000000; Bfp[0][1]=-0.175100000; Bfp[0][2]=-0.386000000; Bfp[0][3]=-0.760200000;
        Bfp[1][0]=-0.832900000; Bfp[1][1]=0.144600000; Bfp[1][2]=-0.180800000; Bfp[1][3]=0.502100000;
        Bfp[2][0]=-0.303100000; Bfp[2][1]=-0.305900000; Bfp[2][2]=0.291800000; Bfp[2][3]=-0.066800000;
        Bfp[3][0]=0.635600000; Bfp[3][1]=-0.146000000; Bfp[3][2]=1.430600000; Bfp[3][3]=0.827600000;
        run_case(63, "NN weights gauss(0,1)");

        // ---- Test 64: NN weights gauss(0,1) ----
        Afp[0][0]=-1.679600000; Afp[0][1]=-0.330100000; Afp[0][2]=-1.409200000; Afp[0][3]=-0.075000000;
        Afp[1][0]=1.299400000; Afp[1][1]=0.743100000; Afp[1][2]=-2.551900000; Afp[1][3]=-0.200000000;
        Afp[2][0]=0.375900000; Afp[2][1]=0.014600000; Afp[2][2]=-1.020400000; Afp[2][3]=-2.042200000;
        Afp[3][0]=-1.388700000; Afp[3][1]=0.744100000; Afp[3][2]=-0.924600000; Afp[3][3]=-0.369700000;
        Bfp[0][0]=-1.324800000; Bfp[0][1]=0.292500000; Bfp[0][2]=0.840100000; Bfp[0][3]=0.154200000;
        Bfp[1][0]=-0.057900000; Bfp[1][1]=-0.770600000; Bfp[1][2]=-0.759400000; Bfp[1][3]=0.129300000;
        Bfp[2][0]=-0.907500000; Bfp[2][1]=1.131100000; Bfp[2][2]=-0.099200000; Bfp[2][3]=-2.505000000;
        Bfp[3][0]=-0.666100000; Bfp[3][1]=0.097800000; Bfp[3][2]=-0.180300000; Bfp[3][3]=0.296700000;
        run_case(64, "NN weights gauss(0,1)");

        // ---- Test 65: NN weights gauss(0,1) ----
        Afp[0][0]=0.086900000; Afp[0][1]=1.322600000; Afp[0][2]=-0.569600000; Afp[0][3]=-0.477800000;
        Afp[1][0]=0.189300000; Afp[1][1]=0.413000000; Afp[1][2]=0.511400000; Afp[1][3]=1.063200000;
        Afp[2][0]=-0.067400000; Afp[2][1]=2.073300000; Afp[2][2]=-0.837200000; Afp[2][3]=-0.361400000;
        Afp[3][0]=-0.259300000; Afp[3][1]=0.119900000; Afp[3][2]=-0.179000000; Afp[3][3]=-1.658100000;
        Bfp[0][0]=-1.053000000; Bfp[0][1]=1.215200000; Bfp[0][2]=-0.099100000; Bfp[0][3]=0.697000000;
        Bfp[1][0]=0.090200000; Bfp[1][1]=0.655600000; Bfp[1][2]=-1.128000000; Bfp[1][3]=-0.884800000;
        Bfp[2][0]=-0.065300000; Bfp[2][1]=-0.463500000; Bfp[2][2]=0.116400000; Bfp[2][3]=-1.134800000;
        Bfp[3][0]=-0.854300000; Bfp[3][1]=0.817800000; Bfp[3][2]=-0.609500000; Bfp[3][3]=0.274000000;
        run_case(65, "NN weights gauss(0,1)");

        // ---- Test 66: NN weights gauss(0,1) ----
        Afp[0][0]=-1.138900000; Afp[0][1]=0.883300000; Afp[0][2]=-0.481300000; Afp[0][3]=-2.167700000;
        Afp[1][0]=0.720800000; Afp[1][1]=1.978200000; Afp[1][2]=0.491500000; Afp[1][3]=-1.501600000;
        Afp[2][0]=0.540200000; Afp[2][1]=-0.070000000; Afp[2][2]=0.356500000; Afp[2][3]=-2.113300000;
        Afp[3][0]=0.967800000; Afp[3][1]=0.932400000; Afp[3][2]=0.787000000; Afp[3][3]=-0.272200000;
        Bfp[0][0]=1.336700000; Bfp[0][1]=-1.717200000; Bfp[0][2]=-0.514200000; Bfp[0][3]=0.222100000;
        Bfp[1][0]=0.404200000; Bfp[1][1]=1.838100000; Bfp[1][2]=-0.768900000; Bfp[1][3]=0.409200000;
        Bfp[2][0]=-2.149000000; Bfp[2][1]=0.805700000; Bfp[2][2]=-0.004800000; Bfp[2][3]=0.192100000;
        Bfp[3][0]=0.829000000; Bfp[3][1]=-0.274000000; Bfp[3][2]=-2.052700000; Bfp[3][3]=1.798600000;
        run_case(66, "NN weights gauss(0,1)");

        // ---- Test 67: NN weights gauss(0,1) ----
        Afp[0][0]=-0.084300000; Afp[0][1]=0.414900000; Afp[0][2]=0.569200000; Afp[0][3]=-0.484800000;
        Afp[1][0]=0.418400000; Afp[1][1]=2.318400000; Afp[1][2]=0.278400000; Afp[1][3]=2.133200000;
        Afp[2][0]=-0.424000000; Afp[2][1]=0.736800000; Afp[2][2]=0.266400000; Afp[2][3]=1.221500000;
        Afp[3][0]=-0.936900000; Afp[3][1]=-0.371900000; Afp[3][2]=-0.840500000; Afp[3][3]=-0.039300000;
        Bfp[0][0]=1.219800000; Bfp[0][1]=-1.986000000; Bfp[0][2]=0.035800000; Bfp[0][3]=-1.850900000;
        Bfp[1][0]=0.685400000; Bfp[1][1]=0.326400000; Bfp[1][2]=0.166400000; Bfp[1][3]=0.555700000;
        Bfp[2][0]=2.128500000; Bfp[2][1]=-0.427700000; Bfp[2][2]=1.096200000; Bfp[2][3]=-0.656100000;
        Bfp[3][0]=0.785000000; Bfp[3][1]=0.705400000; Bfp[3][2]=2.034100000; Bfp[3][3]=0.750100000;
        run_case(67, "NN weights gauss(0,1)");

        // ---- Test 68: NN weights gauss(0,1) ----
        Afp[0][0]=-0.080200000; Afp[0][1]=0.820600000; Afp[0][2]=0.630500000; Afp[0][3]=-0.223600000;
        Afp[1][0]=0.763300000; Afp[1][1]=-0.403600000; Afp[1][2]=-0.242300000; Afp[1][3]=-0.408600000;
        Afp[2][0]=0.809800000; Afp[2][1]=-0.601100000; Afp[2][2]=-0.819700000; Afp[2][3]=1.205400000;
        Afp[3][0]=-0.633100000; Afp[3][1]=-0.420900000; Afp[3][2]=0.930400000; Afp[3][3]=0.920800000;
        Bfp[0][0]=2.001300000; Bfp[0][1]=0.312800000; Bfp[0][2]=-0.338700000; Bfp[0][3]=0.506900000;
        Bfp[1][0]=-0.231100000; Bfp[1][1]=0.433900000; Bfp[1][2]=-0.509000000; Bfp[1][3]=1.143700000;
        Bfp[2][0]=-0.752000000; Bfp[2][1]=0.498000000; Bfp[2][2]=-0.292700000; Bfp[2][3]=1.116900000;
        Bfp[3][0]=0.192400000; Bfp[3][1]=-1.055700000; Bfp[3][2]=0.128900000; Bfp[3][3]=-1.140800000;
        run_case(68, "NN weights gauss(0,1)");

        // ---- Test 69: NN weights gauss(0,0.1) ----
        Afp[0][0]=0.044450000; Afp[0][1]=0.041920000; Afp[0][2]=0.065770000; Afp[0][3]=-0.093410000;
        Afp[1][0]=-0.089030000; Afp[1][1]=0.081080000; Afp[1][2]=0.055140000; Afp[1][3]=-0.079810000;
        Afp[2][0]=-0.061580000; Afp[2][1]=0.072940000; Afp[2][2]=0.125180000; Afp[2][3]=0.008510000;
        Afp[3][0]=-0.116020000; Afp[3][1]=0.015280000; Afp[3][2]=-0.107910000; Afp[3][3]=0.033870000;
        Bfp[0][0]=-0.013820000; Bfp[0][1]=-0.062680000; Bfp[0][2]=0.049020000; Bfp[0][3]=-0.005760000;
        Bfp[1][0]=0.041420000; Bfp[1][1]=-0.046190000; Bfp[1][2]=0.021770000; Bfp[1][3]=-0.074580000;
        Bfp[2][0]=0.109970000; Bfp[2][1]=0.058880000; Bfp[2][2]=0.102510000; Bfp[2][3]=-0.163840000;
        Bfp[3][0]=0.199860000; Bfp[3][1]=-0.194950000; Bfp[3][2]=0.092230000; Bfp[3][3]=0.032000000;
        run_case(69, "NN weights gauss(0,0.1)");

        // ---- Test 70: NN weights gauss(0,0.1) ----
        Afp[0][0]=0.131310000; Afp[0][1]=0.104410000; Afp[0][2]=-0.157490000; Afp[0][3]=0.049530000;
        Afp[1][0]=0.105210000; Afp[1][1]=-0.356060000; Afp[1][2]=-0.090500000; Afp[1][3]=-0.182840000;
        Afp[2][0]=-0.039130000; Afp[2][1]=-0.012300000; Afp[2][2]=0.033860000; Afp[2][3]=-0.027620000;
        Afp[3][0]=0.161660000; Afp[3][1]=0.096660000; Afp[3][2]=-0.028120000; Afp[3][3]=0.144660000;
        Bfp[0][0]=0.067700000; Bfp[0][1]=0.012310000; Bfp[0][2]=-0.005930000; Bfp[0][3]=-0.027070000;
        Bfp[1][0]=-0.234960000; Bfp[1][1]=-0.015700000; Bfp[1][2]=0.034730000; Bfp[1][3]=0.152300000;
        Bfp[2][0]=-0.005380000; Bfp[2][1]=-0.139000000; Bfp[2][2]=0.015850000; Bfp[2][3]=-0.153970000;
        Bfp[3][0]=0.065000000; Bfp[3][1]=0.005850000; Bfp[3][2]=-0.257750000; Bfp[3][3]=-0.060020000;
        run_case(70, "NN weights gauss(0,0.1)");

        // ---- Test 71: NN weights gauss(0,0.1) ----
        Afp[0][0]=0.027410000; Afp[0][1]=0.130110000; Afp[0][2]=0.134080000; Afp[0][3]=0.139810000;
        Afp[1][0]=-0.114350000; Afp[1][1]=0.041050000; Afp[1][2]=-0.026300000; Afp[1][3]=-0.047370000;
        Afp[2][0]=-0.023820000; Afp[2][1]=-0.177790000; Afp[2][2]=0.051650000; Afp[2][3]=-0.062780000;
        Afp[3][0]=0.005910000; Afp[3][1]=0.059750000; Afp[3][2]=0.086610000; Afp[3][3]=-0.063440000;
        Bfp[0][0]=-0.093330000; Bfp[0][1]=0.077380000; Bfp[0][2]=-0.027810000; Bfp[0][3]=-0.082970000;
        Bfp[1][0]=-0.087990000; Bfp[1][1]=0.146030000; Bfp[1][2]=0.013980000; Bfp[1][3]=-0.082600000;
        Bfp[2][0]=0.205300000; Bfp[2][1]=0.102430000; Bfp[2][2]=-0.017080000; Bfp[2][3]=-0.133220000;
        Bfp[3][0]=-0.208590000; Bfp[3][1]=0.142430000; Bfp[3][2]=-0.063330000; Bfp[3][3]=0.029500000;
        run_case(71, "NN weights gauss(0,0.1)");

        // ---- Test 72: NN weights gauss(0,0.1) ----
        Afp[0][0]=0.075110000; Afp[0][1]=0.054830000; Afp[0][2]=-0.076770000; Afp[0][3]=-0.056340000;
        Afp[1][0]=0.136940000; Afp[1][1]=-0.047270000; Afp[1][2]=-0.141910000; Afp[1][3]=0.038160000;
        Afp[2][0]=-0.111270000; Afp[2][1]=-0.082730000; Afp[2][2]=-0.121620000; Afp[2][3]=0.003660000;
        Afp[3][0]=-0.188390000; Afp[3][1]=0.010370000; Afp[3][2]=0.062660000; Afp[3][3]=0.163890000;
        Bfp[0][0]=-0.058100000; Bfp[0][1]=0.185480000; Bfp[0][2]=0.103360000; Bfp[0][3]=0.050870000;
        Bfp[1][0]=-0.147660000; Bfp[1][1]=0.055340000; Bfp[1][2]=0.022990000; Bfp[1][3]=0.074560000;
        Bfp[2][0]=0.086140000; Bfp[2][1]=0.095210000; Bfp[2][2]=-0.002860000; Bfp[2][3]=-0.001490000;
        Bfp[3][0]=0.029220000; Bfp[3][1]=0.060000000; Bfp[3][2]=0.106220000; Bfp[3][3]=-0.029260000;
        run_case(72, "NN weights gauss(0,0.1)");

        // ---- Test 73: NN weights gauss(0,0.1) ----
        Afp[0][0]=-0.075790000; Afp[0][1]=-0.033100000; Afp[0][2]=-0.098410000; Afp[0][3]=-0.154420000;
        Afp[1][0]=-0.002560000; Afp[1][1]=-0.004750000; Afp[1][2]=-0.107580000; Afp[1][3]=0.042980000;
        Afp[2][0]=0.169780000; Afp[2][1]=0.048810000; Afp[2][2]=-0.034940000; Afp[2][3]=0.075660000;
        Afp[3][0]=-0.093610000; Afp[3][1]=-0.093900000; Afp[3][2]=-0.198000000; Afp[3][3]=0.004440000;
        Bfp[0][0]=-0.091040000; Bfp[0][1]=-0.010840000; Bfp[0][2]=0.116640000; Bfp[0][3]=0.049940000;
        Bfp[1][0]=-0.081130000; Bfp[1][1]=0.093810000; Bfp[1][2]=-0.006750000; Bfp[1][3]=0.009620000;
        Bfp[2][0]=0.056980000; Bfp[2][1]=0.013790000; Bfp[2][2]=0.162230000; Bfp[2][3]=0.108690000;
        Bfp[3][0]=-0.064520000; Bfp[3][1]=-0.035870000; Bfp[3][2]=-0.034690000; Bfp[3][3]=-0.137570000;
        run_case(73, "NN weights gauss(0,0.1)");

        // ---- Test 74: NN weights gauss(0,0.1) ----
        Afp[0][0]=0.154310000; Afp[0][1]=0.063370000; Afp[0][2]=0.045630000; Afp[0][3]=0.341330000;
        Afp[1][0]=0.051550000; Afp[1][1]=0.063840000; Afp[1][2]=0.001100000; Afp[1][3]=0.199090000;
        Afp[2][0]=0.054980000; Afp[2][1]=-0.119580000; Afp[2][2]=0.062740000; Afp[2][3]=0.085200000;
        Afp[3][0]=-0.010260000; Afp[3][1]=-0.032980000; Afp[3][2]=0.168700000; Afp[3][3]=-0.091450000;
        Bfp[0][0]=0.140830000; Bfp[0][1]=-0.064440000; Bfp[0][2]=-0.200800000; Bfp[0][3]=-0.183040000;
        Bfp[1][0]=-0.009060000; Bfp[1][1]=0.084920000; Bfp[1][2]=0.045110000; Bfp[1][3]=0.044090000;
        Bfp[2][0]=0.013410000; Bfp[2][1]=-0.042680000; Bfp[2][2]=-0.003710000; Bfp[2][3]=-0.100390000;
        Bfp[3][0]=0.121340000; Bfp[3][1]=-0.117420000; Bfp[3][2]=-0.074650000; Bfp[3][3]=-0.178090000;
        run_case(74, "NN weights gauss(0,0.1)");

        // ---- Test 75: NN heavy-tail (pareto) ----
        Afp[0][0]=1.029400000; Afp[0][1]=1.228300000; Afp[0][2]=1.358700000; Afp[0][3]=1.453500000;
        Afp[1][0]=2.095500000; Afp[1][1]=4.092000000; Afp[1][2]=2.455100000; Afp[1][3]=5.815600000;
        Afp[2][0]=2.211700000; Afp[2][1]=1.341600000; Afp[2][2]=1.604200000; Afp[2][3]=2.510000000;
        Afp[3][0]=3.078900000; Afp[3][1]=1.360600000; Afp[3][2]=1.340100000; Afp[3][3]=1.887900000;
        Bfp[0][0]=1.188700000; Bfp[0][1]=4.179600000; Bfp[0][2]=1.000000000; Bfp[0][3]=2.008600000;
        Bfp[1][0]=1.557700000; Bfp[1][1]=1.155600000; Bfp[1][2]=1.755800000; Bfp[1][3]=1.534900000;
        Bfp[2][0]=8.865100000; Bfp[2][1]=4.219500000; Bfp[2][2]=1.801900000; Bfp[2][3]=3.924900000;
        Bfp[3][0]=4.196200000; Bfp[3][1]=1.127900000; Bfp[3][2]=3.225600000; Bfp[3][3]=3.302100000;
        run_case(75, "NN heavy-tail (pareto)");

        // ---- Test 76: NN heavy-tail (pareto) ----
        Afp[0][0]=22.842400000; Afp[0][1]=2.586800000; Afp[0][2]=1.393400000; Afp[0][3]=1.166600000;
        Afp[1][0]=2.024500000; Afp[1][1]=1.170900000; Afp[1][2]=2.570000000; Afp[1][3]=1.767200000;
        Afp[2][0]=3.205700000; Afp[2][1]=1.341600000; Afp[2][2]=1.272100000; Afp[2][3]=4.783800000;
        Afp[3][0]=2.063000000; Afp[3][1]=1.372700000; Afp[3][2]=1.730300000; Afp[3][3]=1.464100000;
        Bfp[0][0]=1.986900000; Bfp[0][1]=1.623900000; Bfp[0][2]=1.251300000; Bfp[0][3]=13.423800000;
        Bfp[1][0]=4.235500000; Bfp[1][1]=2.579900000; Bfp[1][2]=1.242800000; Bfp[1][3]=1.154900000;
        Bfp[2][0]=2.400100000; Bfp[2][1]=3.164400000; Bfp[2][2]=2.024100000; Bfp[2][3]=1.173600000;
        Bfp[3][0]=2.338600000; Bfp[3][1]=1.638500000; Bfp[3][2]=1.365600000; Bfp[3][3]=1.808900000;
        run_case(76, "NN heavy-tail (pareto)");

        // ---- Test 77: NN heavy-tail (pareto) ----
        Afp[0][0]=1.487500000; Afp[0][1]=2.008900000; Afp[0][2]=2.530900000; Afp[0][3]=14.975600000;
        Afp[1][0]=1.081700000; Afp[1][1]=1.758300000; Afp[1][2]=6.948200000; Afp[1][3]=2.040900000;
        Afp[2][0]=1.782100000; Afp[2][1]=3.589800000; Afp[2][2]=1.021400000; Afp[2][3]=2.854800000;
        Afp[3][0]=1.916900000; Afp[3][1]=1.795200000; Afp[3][2]=1.281500000; Afp[3][3]=1.903500000;
        Bfp[0][0]=1.093400000; Bfp[0][1]=1.643800000; Bfp[0][2]=1.219800000; Bfp[0][3]=5.981600000;
        Bfp[1][0]=2.970400000; Bfp[1][1]=1.532400000; Bfp[1][2]=2.553900000; Bfp[1][3]=1.382500000;
        Bfp[2][0]=1.639400000; Bfp[2][1]=1.124600000; Bfp[2][2]=3.672000000; Bfp[2][3]=2.618900000;
        Bfp[3][0]=2.589400000; Bfp[3][1]=1.059700000; Bfp[3][2]=3.130300000; Bfp[3][3]=1.008100000;
        run_case(77, "NN heavy-tail (pareto)");

        // ---- Test 78: NN heavy-tail (pareto) ----
        Afp[0][0]=1.742200000; Afp[0][1]=1.562100000; Afp[0][2]=1.041600000; Afp[0][3]=1.440700000;
        Afp[1][0]=4.281300000; Afp[1][1]=1.920500000; Afp[1][2]=1.286200000; Afp[1][3]=3.072800000;
        Afp[2][0]=1.340100000; Afp[2][1]=4.552900000; Afp[2][2]=1.407100000; Afp[2][3]=1.055300000;
        Afp[3][0]=10.773900000; Afp[3][1]=1.050400000; Afp[3][2]=1.214700000; Afp[3][3]=2.672000000;
        Bfp[0][0]=1.085000000; Bfp[0][1]=1.688300000; Bfp[0][2]=3.692300000; Bfp[0][3]=1.811800000;
        Bfp[1][0]=1.330100000; Bfp[1][1]=2.575200000; Bfp[1][2]=3.203500000; Bfp[1][3]=1.781300000;
        Bfp[2][0]=1.929600000; Bfp[2][1]=1.296200000; Bfp[2][2]=1.417500000; Bfp[2][3]=24.718400000;
        Bfp[3][0]=10.802400000; Bfp[3][1]=1.001100000; Bfp[3][2]=1.156800000; Bfp[3][3]=1.185500000;
        run_case(78, "NN heavy-tail (pareto)");

        // ---- Test 79: NN heavy-tail (pareto) ----
        Afp[0][0]=2.856700000; Afp[0][1]=1.161000000; Afp[0][2]=1.131700000; Afp[0][3]=2.632600000;
        Afp[1][0]=1.131200000; Afp[1][1]=12.501400000; Afp[1][2]=1.499500000; Afp[1][3]=4.598200000;
        Afp[2][0]=3.133500000; Afp[2][1]=1.167400000; Afp[2][2]=2.791900000; Afp[2][3]=1.429500000;
        Afp[3][0]=1.182300000; Afp[3][1]=1.031900000; Afp[3][2]=2.247500000; Afp[3][3]=1.731400000;
        Bfp[0][0]=1.272300000; Bfp[0][1]=3.294900000; Bfp[0][2]=1.157500000; Bfp[0][3]=1.480300000;
        Bfp[1][0]=1.062800000; Bfp[1][1]=1.333200000; Bfp[1][2]=4.220900000; Bfp[1][3]=1.587600000;
        Bfp[2][0]=1.117600000; Bfp[2][1]=1.197100000; Bfp[2][2]=1.992100000; Bfp[2][3]=1.242200000;
        Bfp[3][0]=4.282700000; Bfp[3][1]=1.042500000; Bfp[3][2]=3.874900000; Bfp[3][3]=1.912600000;
        run_case(79, "NN heavy-tail (pareto)");

        // ---- Test 80: wide range 1e-2..1e2 ----
        Afp[0][0]=19.450650000; Afp[0][1]=0.261140000; Afp[0][2]=0.018900000; Afp[0][3]=0.048820000;
        Afp[1][0]=3.215560000; Afp[1][1]=0.012700000; Afp[1][2]=0.027590000; Afp[1][3]=13.463290000;
        Afp[2][0]=32.516720000; Afp[2][1]=26.574820000; Afp[2][2]=22.372760000; Afp[2][3]=1.660160000;
        Afp[3][0]=31.700880000; Afp[3][1]=6.170740000; Afp[3][2]=1.696630000; Afp[3][3]=0.504380000;
        Bfp[0][0]=0.012510000; Bfp[0][1]=0.022830000; Bfp[0][2]=0.099690000; Bfp[0][3]=0.697620000;
        Bfp[1][0]=0.030490000; Bfp[1][1]=76.185400000; Bfp[1][2]=2.022950000; Bfp[1][3]=9.153120000;
        Bfp[2][0]=37.224890000; Bfp[2][1]=0.080340000; Bfp[2][2]=35.043010000; Bfp[2][3]=67.949790000;
        Bfp[3][0]=0.117450000; Bfp[3][1]=0.031760000; Bfp[3][2]=15.853370000; Bfp[3][3]=36.229920000;
        run_case(80, "wide range 1e-2..1e2");

        // ---- Test 81: wide range 1e-2..1e2 ----
        Afp[0][0]=15.631740000; Afp[0][1]=0.429790000; Afp[0][2]=2.562040000; Afp[0][3]=0.028540000;
        Afp[1][0]=0.086700000; Afp[1][1]=0.473670000; Afp[1][2]=9.526070000; Afp[1][3]=0.022650000;
        Afp[2][0]=51.659030000; Afp[2][1]=0.013160000; Afp[2][2]=75.629080000; Afp[2][3]=0.088680000;
        Afp[3][0]=18.855180000; Afp[3][1]=18.714990000; Afp[3][2]=65.717470000; Afp[3][3]=0.759920000;
        Bfp[0][0]=6.239950000; Bfp[0][1]=0.147280000; Bfp[0][2]=21.450230000; Bfp[0][3]=0.027250000;
        Bfp[1][0]=34.274260000; Bfp[1][1]=2.340330000; Bfp[1][2]=0.017690000; Bfp[1][3]=58.247500000;
        Bfp[2][0]=0.183670000; Bfp[2][1]=0.018730000; Bfp[2][2]=0.108680000; Bfp[2][3]=0.012860000;
        Bfp[3][0]=0.133420000; Bfp[3][1]=6.914030000; Bfp[3][2]=89.965430000; Bfp[3][3]=3.353080000;
        run_case(81, "wide range 1e-2..1e2");

        // ---- Test 82: wide range 1e-2..1e2 ----
        Afp[0][0]=26.062880000; Afp[0][1]=19.460730000; Afp[0][2]=7.225560000; Afp[0][3]=0.344760000;
        Afp[1][0]=0.101660000; Afp[1][1]=0.042090000; Afp[1][2]=1.541250000; Afp[1][3]=0.065560000;
        Afp[2][0]=9.113690000; Afp[2][1]=0.191400000; Afp[2][2]=2.075160000; Afp[2][3]=1.533970000;
        Afp[3][0]=3.248580000; Afp[3][1]=0.041810000; Afp[3][2]=0.105930000; Afp[3][3]=5.683430000;
        Bfp[0][0]=0.071220000; Bfp[0][1]=4.852510000; Bfp[0][2]=72.472560000; Bfp[0][3]=0.360180000;
        Bfp[1][0]=70.247880000; Bfp[1][1]=2.864730000; Bfp[1][2]=59.921440000; Bfp[1][3]=1.273290000;
        Bfp[2][0]=10.260880000; Bfp[2][1]=5.372710000; Bfp[2][2]=0.060390000; Bfp[2][3]=1.179310000;
        Bfp[3][0]=2.106900000; Bfp[3][1]=0.715430000; Bfp[3][2]=4.363810000; Bfp[3][3]=43.294580000;
        run_case(82, "wide range 1e-2..1e2");

        // ---- Test 83: wide range 1e-2..1e2 ----
        Afp[0][0]=15.130950000; Afp[0][1]=17.740490000; Afp[0][2]=0.037480000; Afp[0][3]=6.495220000;
        Afp[1][0]=3.232490000; Afp[1][1]=56.670020000; Afp[1][2]=0.010240000; Afp[1][3]=0.048940000;
        Afp[2][0]=79.899680000; Afp[2][1]=0.308920000; Afp[2][2]=6.645460000; Afp[2][3]=2.328300000;
        Afp[3][0]=0.390790000; Afp[3][1]=0.568190000; Afp[3][2]=1.772610000; Afp[3][3]=87.261230000;
        Bfp[0][0]=1.359930000; Bfp[0][1]=14.484100000; Bfp[0][2]=0.163620000; Bfp[0][3]=7.029620000;
        Bfp[1][0]=2.621600000; Bfp[1][1]=9.413440000; Bfp[1][2]=32.965830000; Bfp[1][3]=0.020860000;
        Bfp[2][0]=73.833430000; Bfp[2][1]=2.758910000; Bfp[2][2]=0.109440000; Bfp[2][3]=0.720150000;
        Bfp[3][0]=0.024400000; Bfp[3][1]=0.084040000; Bfp[3][2]=22.444910000; Bfp[3][3]=6.991190000;
        run_case(83, "wide range 1e-2..1e2");

        // ---- Test 84: wide range 1e-2..1e2 ----
        Afp[0][0]=29.189040000; Afp[0][1]=0.044610000; Afp[0][2]=0.027080000; Afp[0][3]=0.329190000;
        Afp[1][0]=0.961660000; Afp[1][1]=6.970950000; Afp[1][2]=0.156050000; Afp[1][3]=8.894350000;
        Afp[2][0]=2.172090000; Afp[2][1]=0.142770000; Afp[2][2]=14.158290000; Afp[2][3]=0.019710000;
        Afp[3][0]=2.337870000; Afp[3][1]=0.056390000; Afp[3][2]=0.014810000; Afp[3][3]=0.819200000;
        Bfp[0][0]=1.200480000; Bfp[0][1]=2.819320000; Bfp[0][2]=0.316930000; Bfp[0][3]=52.241720000;
        Bfp[1][0]=13.696020000; Bfp[1][1]=0.041290000; Bfp[1][2]=0.010450000; Bfp[1][3]=69.557310000;
        Bfp[2][0]=0.440880000; Bfp[2][1]=0.026290000; Bfp[2][2]=0.051390000; Bfp[2][3]=6.319330000;
        Bfp[3][0]=0.050870000; Bfp[3][1]=0.310670000; Bfp[3][2]=5.098040000; Bfp[3][3]=0.256560000;
        run_case(84, "wide range 1e-2..1e2");

        // ---- Test 85: wide range 1e-2..1e2 ----
        Afp[0][0]=4.112500000; Afp[0][1]=62.150570000; Afp[0][2]=14.851010000; Afp[0][3]=3.257380000;
        Afp[1][0]=3.180540000; Afp[1][1]=0.173100000; Afp[1][2]=3.644560000; Afp[1][3]=3.948630000;
        Afp[2][0]=2.228970000; Afp[2][1]=0.918090000; Afp[2][2]=10.140240000; Afp[2][3]=0.065750000;
        Afp[3][0]=7.140920000; Afp[3][1]=82.027830000; Afp[3][2]=0.013920000; Afp[3][3]=0.101940000;
        Bfp[0][0]=87.705830000; Bfp[0][1]=0.021370000; Bfp[0][2]=18.671410000; Bfp[0][3]=2.107950000;
        Bfp[1][0]=0.018980000; Bfp[1][1]=3.300640000; Bfp[1][2]=0.010730000; Bfp[1][3]=24.960530000;
        Bfp[2][0]=0.161980000; Bfp[2][1]=49.637720000; Bfp[2][2]=38.952270000; Bfp[2][3]=0.035180000;
        Bfp[3][0]=0.409160000; Bfp[3][1]=0.040740000; Bfp[3][2]=10.745650000; Bfp[3][3]=0.024070000;
        run_case(85, "wide range 1e-2..1e2");

        // ---- Test 86: wide range 1e-2..1e2 ----
        Afp[0][0]=0.019720000; Afp[0][1]=0.029140000; Afp[0][2]=27.227490000; Afp[0][3]=0.097340000;
        Afp[1][0]=0.195920000; Afp[1][1]=0.609200000; Afp[1][2]=1.611670000; Afp[1][3]=0.449530000;
        Afp[2][0]=61.443870000; Afp[2][1]=0.230400000; Afp[2][2]=0.801240000; Afp[2][3]=28.091400000;
        Afp[3][0]=0.094270000; Afp[3][1]=0.049650000; Afp[3][2]=22.135800000; Afp[3][3]=0.183570000;
        Bfp[0][0]=36.959550000; Bfp[0][1]=19.428600000; Bfp[0][2]=0.310230000; Bfp[0][3]=0.214550000;
        Bfp[1][0]=0.025440000; Bfp[1][1]=31.181120000; Bfp[1][2]=0.588960000; Bfp[1][3]=0.233720000;
        Bfp[2][0]=0.050840000; Bfp[2][1]=0.080100000; Bfp[2][2]=31.297780000; Bfp[2][3]=0.033310000;
        Bfp[3][0]=1.653230000; Bfp[3][1]=3.153790000; Bfp[3][2]=1.206640000; Bfp[3][3]=1.022010000;
        run_case(86, "wide range 1e-2..1e2");

        // ---- Test 87: wide range 1e-2..1e2 ----
        Afp[0][0]=0.389000000; Afp[0][1]=0.058410000; Afp[0][2]=19.232780000; Afp[0][3]=38.667230000;
        Afp[1][0]=1.802030000; Afp[1][1]=0.101550000; Afp[1][2]=44.571770000; Afp[1][3]=0.064110000;
        Afp[2][0]=0.048280000; Afp[2][1]=3.187560000; Afp[2][2]=21.173020000; Afp[2][3]=1.945800000;
        Afp[3][0]=0.027460000; Afp[3][1]=82.378800000; Afp[3][2]=0.042140000; Afp[3][3]=0.236900000;
        Bfp[0][0]=89.768950000; Bfp[0][1]=14.744640000; Bfp[0][2]=10.543780000; Bfp[0][3]=0.020090000;
        Bfp[1][0]=25.533660000; Bfp[1][1]=0.398400000; Bfp[1][2]=0.011040000; Bfp[1][3]=14.335560000;
        Bfp[2][0]=52.624060000; Bfp[2][1]=0.015500000; Bfp[2][2]=0.655620000; Bfp[2][3]=40.363140000;
        Bfp[3][0]=0.213740000; Bfp[3][1]=0.162850000; Bfp[3][2]=13.054040000; Bfp[3][3]=0.581910000;
        run_case(87, "wide range 1e-2..1e2");

        // ---- Test 88: very wide 1e-4..1e4 ----
        Afp[0][0]=9652.655158000; Afp[0][1]=704.601769000; Afp[0][2]=738.770510000; Afp[0][3]=45.274203000;
        Afp[1][0]=0.003619000; Afp[1][1]=31.341223000; Afp[1][2]=4.252553000; Afp[1][3]=0.728383000;
        Afp[2][0]=3.402435000; Afp[2][1]=0.027678000; Afp[2][2]=0.003824000; Afp[2][3]=1983.843475000;
        Afp[3][0]=0.086098000; Afp[3][1]=0.000702000; Afp[3][2]=6477.679172000; Afp[3][3]=1.219895000;
        Bfp[0][0]=25.509282000; Bfp[0][1]=0.051705000; Bfp[0][2]=0.001553000; Bfp[0][3]=0.045656000;
        Bfp[1][0]=465.312897000; Bfp[1][1]=0.015530000; Bfp[1][2]=1985.508918000; Bfp[1][3]=0.027244000;
        Bfp[2][0]=1498.989980000; Bfp[2][1]=0.002704000; Bfp[2][2]=0.354294000; Bfp[2][3]=0.001074000;
        Bfp[3][0]=0.001542000; Bfp[3][1]=1.514286000; Bfp[3][2]=0.083232000; Bfp[3][3]=1.995699000;
        run_case(88, "very wide 1e-4..1e4");

        // ---- Test 89: very wide 1e-4..1e4 ----
        Afp[0][0]=0.005387000; Afp[0][1]=0.030457000; Afp[0][2]=0.037562000; Afp[0][3]=1536.065989000;
        Afp[1][0]=0.000444000; Afp[1][1]=0.213834000; Afp[1][2]=684.545200000; Afp[1][3]=3.775913000;
        Afp[2][0]=0.011108000; Afp[2][1]=9501.453442000; Afp[2][2]=0.000736000; Afp[2][3]=0.001246000;
        Afp[3][0]=0.000137000; Afp[3][1]=0.326008000; Afp[3][2]=741.014987000; Afp[3][3]=0.018144000;
        Bfp[0][0]=0.019249000; Bfp[0][1]=242.715187000; Bfp[0][2]=0.000417000; Bfp[0][3]=0.000138000;
        Bfp[1][0]=20.316467000; Bfp[1][1]=0.000507000; Bfp[1][2]=1.654834000; Bfp[1][3]=0.001439000;
        Bfp[2][0]=6.513605000; Bfp[2][1]=646.347157000; Bfp[2][2]=0.421042000; Bfp[2][3]=0.015365000;
        Bfp[3][0]=9.698192000; Bfp[3][1]=39.930058000; Bfp[3][2]=0.000285000; Bfp[3][3]=0.008298000;
        run_case(89, "very wide 1e-4..1e4");

        // ---- Test 90: very wide 1e-4..1e4 ----
        Afp[0][0]=0.103113000; Afp[0][1]=1.112996000; Afp[0][2]=0.015446000; Afp[0][3]=18.437051000;
        Afp[1][0]=0.011056000; Afp[1][1]=1731.079697000; Afp[1][2]=0.065140000; Afp[1][3]=0.005923000;
        Afp[2][0]=0.000424000; Afp[2][1]=0.001029000; Afp[2][2]=0.005302000; Afp[2][3]=1152.040248000;
        Afp[3][0]=0.215164000; Afp[3][1]=0.350126000; Afp[3][2]=4760.621628000; Afp[3][3]=5405.571821000;
        Bfp[0][0]=133.538064000; Bfp[0][1]=0.008348000; Bfp[0][2]=0.041576000; Bfp[0][3]=1.098570000;
        Bfp[1][0]=0.011612000; Bfp[1][1]=0.059802000; Bfp[1][2]=0.005017000; Bfp[1][3]=0.008500000;
        Bfp[2][0]=99.028566000; Bfp[2][1]=6.713461000; Bfp[2][2]=0.366733000; Bfp[2][3]=0.593414000;
        Bfp[3][0]=8.079719000; Bfp[3][1]=30.011178000; Bfp[3][2]=0.000152000; Bfp[3][3]=558.317239000;
        run_case(90, "very wide 1e-4..1e4");

        // ---- Test 91: very wide 1e-4..1e4 ----
        Afp[0][0]=0.502586000; Afp[0][1]=0.054428000; Afp[0][2]=4659.151023000; Afp[0][3]=113.011734000;
        Afp[1][0]=0.063612000; Afp[1][1]=0.000157000; Afp[1][2]=0.314532000; Afp[1][3]=0.329578000;
        Afp[2][0]=0.099639000; Afp[2][1]=0.000605000; Afp[2][2]=4.111221000; Afp[2][3]=0.010908000;
        Afp[3][0]=124.456183000; Afp[3][1]=1.691676000; Afp[3][2]=0.021994000; Afp[3][3]=405.986548000;
        Bfp[0][0]=0.381241000; Bfp[0][1]=0.000473000; Bfp[0][2]=0.017339000; Bfp[0][3]=0.000264000;
        Bfp[1][0]=0.273419000; Bfp[1][1]=1861.737792000; Bfp[1][2]=3333.110702000; Bfp[1][3]=16.443780000;
        Bfp[2][0]=0.001716000; Bfp[2][1]=0.008265000; Bfp[2][2]=0.004227000; Bfp[2][3]=0.095969000;
        Bfp[3][0]=12.750308000; Bfp[3][1]=0.000151000; Bfp[3][2]=0.823423000; Bfp[3][3]=0.254022000;
        run_case(91, "very wide 1e-4..1e4");

        // ---- Test 92: very wide 1e-4..1e4 ----
        Afp[0][0]=19.509223000; Afp[0][1]=409.257488000; Afp[0][2]=0.149569000; Afp[0][3]=6.638498000;
        Afp[1][0]=0.014984000; Afp[1][1]=0.431469000; Afp[1][2]=12.028679000; Afp[1][3]=0.000262000;
        Afp[2][0]=1.293052000; Afp[2][1]=1.041556000; Afp[2][2]=0.002940000; Afp[2][3]=0.009775000;
        Afp[3][0]=392.035661000; Afp[3][1]=0.062327000; Afp[3][2]=0.003466000; Afp[3][3]=0.018920000;
        Bfp[0][0]=1.477612000; Bfp[0][1]=0.155023000; Bfp[0][2]=0.064344000; Bfp[0][3]=0.038084000;
        Bfp[1][0]=0.005404000; Bfp[1][1]=1.087497000; Bfp[1][2]=6.298019000; Bfp[1][3]=0.000516000;
        Bfp[2][0]=5.305798000; Bfp[2][1]=0.028917000; Bfp[2][2]=6741.518476000; Bfp[2][3]=3192.372941000;
        Bfp[3][0]=0.001122000; Bfp[3][1]=37.157659000; Bfp[3][2]=0.057777000; Bfp[3][3]=0.000776000;
        run_case(92, "very wide 1e-4..1e4");

        // ---- Test 93: very wide 1e-4..1e4 ----
        Afp[0][0]=1011.819811000; Afp[0][1]=1847.369615000; Afp[0][2]=196.678580000; Afp[0][3]=12.388013000;
        Afp[1][0]=6670.791031000; Afp[1][1]=2.788026000; Afp[1][2]=25.546326000; Afp[1][3]=16.657502000;
        Afp[2][0]=747.676395000; Afp[2][1]=704.719794000; Afp[2][2]=0.027172000; Afp[2][3]=0.004600000;
        Afp[3][0]=2.327926000; Afp[3][1]=5.219785000; Afp[3][2]=0.066923000; Afp[3][3]=768.174817000;
        Bfp[0][0]=616.061229000; Bfp[0][1]=31.224251000; Bfp[0][2]=1.916192000; Bfp[0][3]=36.521346000;
        Bfp[1][0]=0.003644000; Bfp[1][1]=289.321681000; Bfp[1][2]=0.001042000; Bfp[1][3]=1690.235310000;
        Bfp[2][0]=10.100843000; Bfp[2][1]=0.016533000; Bfp[2][2]=0.003341000; Bfp[2][3]=6.281734000;
        Bfp[3][0]=0.514264000; Bfp[3][1]=37.429037000; Bfp[3][2]=0.000606000; Bfp[3][3]=1005.486535000;
        run_case(93, "very wide 1e-4..1e4");

        // ---- Test 94: exact 1.0 everywhere ----
        Afp[0][0]=1.000000000; Afp[0][1]=1.000000000; Afp[0][2]=1.000000000; Afp[0][3]=1.000000000;
        Afp[1][0]=1.000000000; Afp[1][1]=1.000000000; Afp[1][2]=1.000000000; Afp[1][3]=1.000000000;
        Afp[2][0]=1.000000000; Afp[2][1]=1.000000000; Afp[2][2]=1.000000000; Afp[2][3]=1.000000000;
        Afp[3][0]=1.000000000; Afp[3][1]=1.000000000; Afp[3][2]=1.000000000; Afp[3][3]=1.000000000;
        Bfp[0][0]=1.000000000; Bfp[0][1]=1.000000000; Bfp[0][2]=1.000000000; Bfp[0][3]=1.000000000;
        Bfp[1][0]=1.000000000; Bfp[1][1]=1.000000000; Bfp[1][2]=1.000000000; Bfp[1][3]=1.000000000;
        Bfp[2][0]=1.000000000; Bfp[2][1]=1.000000000; Bfp[2][2]=1.000000000; Bfp[2][3]=1.000000000;
        Bfp[3][0]=1.000000000; Bfp[3][1]=1.000000000; Bfp[3][2]=1.000000000; Bfp[3][3]=1.000000000;
        run_case(94, "exact 1.0 everywhere");

        // ---- Test 95: exact powers of two ----
        Afp[0][0]=1.000000000; Afp[0][1]=2.000000000; Afp[0][2]=4.000000000; Afp[0][3]=8.000000000;
        Afp[1][0]=1.000000000; Afp[1][1]=2.000000000; Afp[1][2]=4.000000000; Afp[1][3]=8.000000000;
        Afp[2][0]=1.000000000; Afp[2][1]=2.000000000; Afp[2][2]=4.000000000; Afp[2][3]=8.000000000;
        Afp[3][0]=1.000000000; Afp[3][1]=2.000000000; Afp[3][2]=4.000000000; Afp[3][3]=8.000000000;
        Bfp[0][0]=1.000000000; Bfp[0][1]=1.000000000; Bfp[0][2]=1.000000000; Bfp[0][3]=1.000000000;
        Bfp[1][0]=2.000000000; Bfp[1][1]=2.000000000; Bfp[1][2]=2.000000000; Bfp[1][3]=2.000000000;
        Bfp[2][0]=4.000000000; Bfp[2][1]=4.000000000; Bfp[2][2]=4.000000000; Bfp[2][3]=4.000000000;
        Bfp[3][0]=8.000000000; Bfp[3][1]=8.000000000; Bfp[3][2]=8.000000000; Bfp[3][3]=8.000000000;
        run_case(95, "exact powers of two");

        // ---- Test 96: all identical 3.7 ----
        Afp[0][0]=3.700000000; Afp[0][1]=3.700000000; Afp[0][2]=3.700000000; Afp[0][3]=3.700000000;
        Afp[1][0]=3.700000000; Afp[1][1]=3.700000000; Afp[1][2]=3.700000000; Afp[1][3]=3.700000000;
        Afp[2][0]=3.700000000; Afp[2][1]=3.700000000; Afp[2][2]=3.700000000; Afp[2][3]=3.700000000;
        Afp[3][0]=3.700000000; Afp[3][1]=3.700000000; Afp[3][2]=3.700000000; Afp[3][3]=3.700000000;
        Bfp[0][0]=3.700000000; Bfp[0][1]=3.700000000; Bfp[0][2]=3.700000000; Bfp[0][3]=3.700000000;
        Bfp[1][0]=3.700000000; Bfp[1][1]=3.700000000; Bfp[1][2]=3.700000000; Bfp[1][3]=3.700000000;
        Bfp[2][0]=3.700000000; Bfp[2][1]=3.700000000; Bfp[2][2]=3.700000000; Bfp[2][3]=3.700000000;
        Bfp[3][0]=3.700000000; Bfp[3][1]=3.700000000; Bfp[3][2]=3.700000000; Bfp[3][3]=3.700000000;
        run_case(96, "all identical 3.7");

        // ---- Test 97: perfect cancellation ----
        Afp[0][0]=1.000000000; Afp[0][1]=-1.000000000; Afp[0][2]=1.000000000; Afp[0][3]=-1.000000000;
        Afp[1][0]=1.000000000; Afp[1][1]=-1.000000000; Afp[1][2]=1.000000000; Afp[1][3]=-1.000000000;
        Afp[2][0]=1.000000000; Afp[2][1]=-1.000000000; Afp[2][2]=1.000000000; Afp[2][3]=-1.000000000;
        Afp[3][0]=1.000000000; Afp[3][1]=-1.000000000; Afp[3][2]=1.000000000; Afp[3][3]=-1.000000000;
        Bfp[0][0]=1.000000000; Bfp[0][1]=1.000000000; Bfp[0][2]=1.000000000; Bfp[0][3]=1.000000000;
        Bfp[1][0]=1.000000000; Bfp[1][1]=1.000000000; Bfp[1][2]=1.000000000; Bfp[1][3]=1.000000000;
        Bfp[2][0]=1.000000000; Bfp[2][1]=1.000000000; Bfp[2][2]=1.000000000; Bfp[2][3]=1.000000000;
        Bfp[3][0]=1.000000000; Bfp[3][1]=1.000000000; Bfp[3][2]=1.000000000; Bfp[3][3]=1.000000000;
        run_case(97, "perfect cancellation");

        // ---- Test 98: alternating signs equal mag ----
        Afp[0][0]=5.000000000; Afp[0][1]=-5.000000000; Afp[0][2]=5.000000000; Afp[0][3]=-5.000000000;
        Afp[1][0]=5.000000000; Afp[1][1]=-5.000000000; Afp[1][2]=5.000000000; Afp[1][3]=-5.000000000;
        Afp[2][0]=5.000000000; Afp[2][1]=-5.000000000; Afp[2][2]=5.000000000; Afp[2][3]=-5.000000000;
        Afp[3][0]=5.000000000; Afp[3][1]=-5.000000000; Afp[3][2]=5.000000000; Afp[3][3]=-5.000000000;
        Bfp[0][0]=5.000000000; Bfp[0][1]=5.000000000; Bfp[0][2]=5.000000000; Bfp[0][3]=5.000000000;
        Bfp[1][0]=5.000000000; Bfp[1][1]=5.000000000; Bfp[1][2]=5.000000000; Bfp[1][3]=5.000000000;
        Bfp[2][0]=5.000000000; Bfp[2][1]=5.000000000; Bfp[2][2]=5.000000000; Bfp[2][3]=5.000000000;
        Bfp[3][0]=5.000000000; Bfp[3][1]=5.000000000; Bfp[3][2]=5.000000000; Bfp[3][3]=5.000000000;
        run_case(98, "alternating signs equal mag");

        // ---- Test 99: all negative ----
        Afp[0][0]=-2.500000000; Afp[0][1]=-2.500000000; Afp[0][2]=-2.500000000; Afp[0][3]=-2.500000000;
        Afp[1][0]=-2.500000000; Afp[1][1]=-2.500000000; Afp[1][2]=-2.500000000; Afp[1][3]=-2.500000000;
        Afp[2][0]=-2.500000000; Afp[2][1]=-2.500000000; Afp[2][2]=-2.500000000; Afp[2][3]=-2.500000000;
        Afp[3][0]=-2.500000000; Afp[3][1]=-2.500000000; Afp[3][2]=-2.500000000; Afp[3][3]=-2.500000000;
        Bfp[0][0]=-3.500000000; Bfp[0][1]=-3.500000000; Bfp[0][2]=-3.500000000; Bfp[0][3]=-3.500000000;
        Bfp[1][0]=-3.500000000; Bfp[1][1]=-3.500000000; Bfp[1][2]=-3.500000000; Bfp[1][3]=-3.500000000;
        Bfp[2][0]=-3.500000000; Bfp[2][1]=-3.500000000; Bfp[2][2]=-3.500000000; Bfp[2][3]=-3.500000000;
        Bfp[3][0]=-3.500000000; Bfp[3][1]=-3.500000000; Bfp[3][2]=-3.500000000; Bfp[3][3]=-3.500000000;
        run_case(99, "all negative");

        // ---- Test 100: margin int_part=-4 ----
        Afp[0][0]=0.250000000; Afp[0][1]=0.250000000; Afp[0][2]=4.000000000; Afp[0][3]=4.000000000;
        Afp[1][0]=0.250000000; Afp[1][1]=0.250000000; Afp[1][2]=4.000000000; Afp[1][3]=4.000000000;
        Afp[2][0]=0.250000000; Afp[2][1]=0.250000000; Afp[2][2]=4.000000000; Afp[2][3]=4.000000000;
        Afp[3][0]=0.250000000; Afp[3][1]=0.250000000; Afp[3][2]=4.000000000; Afp[3][3]=4.000000000;
        Bfp[0][0]=0.250000000; Bfp[0][1]=0.250000000; Bfp[0][2]=0.250000000; Bfp[0][3]=0.250000000;
        Bfp[1][0]=0.250000000; Bfp[1][1]=0.250000000; Bfp[1][2]=0.250000000; Bfp[1][3]=0.250000000;
        Bfp[2][0]=4.000000000; Bfp[2][1]=4.000000000; Bfp[2][2]=4.000000000; Bfp[2][3]=4.000000000;
        Bfp[3][0]=4.000000000; Bfp[3][1]=4.000000000; Bfp[3][2]=4.000000000; Bfp[3][3]=4.000000000;
        run_case(100, "margin int_part=-4");

        // ---- Test 101: margin int_part=-6 ----
        Afp[0][0]=0.125000000; Afp[0][1]=0.125000000; Afp[0][2]=8.000000000; Afp[0][3]=8.000000000;
        Afp[1][0]=0.125000000; Afp[1][1]=0.125000000; Afp[1][2]=8.000000000; Afp[1][3]=8.000000000;
        Afp[2][0]=0.125000000; Afp[2][1]=0.125000000; Afp[2][2]=8.000000000; Afp[2][3]=8.000000000;
        Afp[3][0]=0.125000000; Afp[3][1]=0.125000000; Afp[3][2]=8.000000000; Afp[3][3]=8.000000000;
        Bfp[0][0]=0.125000000; Bfp[0][1]=0.125000000; Bfp[0][2]=0.125000000; Bfp[0][3]=0.125000000;
        Bfp[1][0]=0.125000000; Bfp[1][1]=0.125000000; Bfp[1][2]=0.125000000; Bfp[1][3]=0.125000000;
        Bfp[2][0]=8.000000000; Bfp[2][1]=8.000000000; Bfp[2][2]=8.000000000; Bfp[2][3]=8.000000000;
        Bfp[3][0]=8.000000000; Bfp[3][1]=8.000000000; Bfp[3][2]=8.000000000; Bfp[3][3]=8.000000000;
        run_case(101, "margin int_part=-6");

        // ---- Test 102: margin int_part=-8 EDGE ----
        Afp[0][0]=0.062500000; Afp[0][1]=0.062500000; Afp[0][2]=16.000000000; Afp[0][3]=16.000000000;
        Afp[1][0]=0.062500000; Afp[1][1]=0.062500000; Afp[1][2]=16.000000000; Afp[1][3]=16.000000000;
        Afp[2][0]=0.062500000; Afp[2][1]=0.062500000; Afp[2][2]=16.000000000; Afp[2][3]=16.000000000;
        Afp[3][0]=0.062500000; Afp[3][1]=0.062500000; Afp[3][2]=16.000000000; Afp[3][3]=16.000000000;
        Bfp[0][0]=0.062500000; Bfp[0][1]=0.062500000; Bfp[0][2]=0.062500000; Bfp[0][3]=0.062500000;
        Bfp[1][0]=0.062500000; Bfp[1][1]=0.062500000; Bfp[1][2]=0.062500000; Bfp[1][3]=0.062500000;
        Bfp[2][0]=16.000000000; Bfp[2][1]=16.000000000; Bfp[2][2]=16.000000000; Bfp[2][3]=16.000000000;
        Bfp[3][0]=16.000000000; Bfp[3][1]=16.000000000; Bfp[3][2]=16.000000000; Bfp[3][3]=16.000000000;
        run_case(102, "margin int_part=-8 EDGE");

        // ---- Test 103: margin int_part=-9 ----
        Afp[0][0]=0.044194174; Afp[0][1]=0.044194174; Afp[0][2]=22.627417000; Afp[0][3]=22.627417000;
        Afp[1][0]=0.044194174; Afp[1][1]=0.044194174; Afp[1][2]=22.627417000; Afp[1][3]=22.627417000;
        Afp[2][0]=0.044194174; Afp[2][1]=0.044194174; Afp[2][2]=22.627417000; Afp[2][3]=22.627417000;
        Afp[3][0]=0.044194174; Afp[3][1]=0.044194174; Afp[3][2]=22.627417000; Afp[3][3]=22.627417000;
        Bfp[0][0]=0.044194174; Bfp[0][1]=0.044194174; Bfp[0][2]=0.044194174; Bfp[0][3]=0.044194174;
        Bfp[1][0]=0.044194174; Bfp[1][1]=0.044194174; Bfp[1][2]=0.044194174; Bfp[1][3]=0.044194174;
        Bfp[2][0]=22.627417000; Bfp[2][1]=22.627417000; Bfp[2][2]=22.627417000; Bfp[2][3]=22.627417000;
        Bfp[3][0]=22.627417000; Bfp[3][1]=22.627417000; Bfp[3][2]=22.627417000; Bfp[3][3]=22.627417000;
        run_case(103, "margin int_part=-9");

        // ---- Test 104: margin int_part=-10 CLIFF ----
        Afp[0][0]=0.031250000; Afp[0][1]=0.031250000; Afp[0][2]=32.000000000; Afp[0][3]=32.000000000;
        Afp[1][0]=0.031250000; Afp[1][1]=0.031250000; Afp[1][2]=32.000000000; Afp[1][3]=32.000000000;
        Afp[2][0]=0.031250000; Afp[2][1]=0.031250000; Afp[2][2]=32.000000000; Afp[2][3]=32.000000000;
        Afp[3][0]=0.031250000; Afp[3][1]=0.031250000; Afp[3][2]=32.000000000; Afp[3][3]=32.000000000;
        Bfp[0][0]=0.031250000; Bfp[0][1]=0.031250000; Bfp[0][2]=0.031250000; Bfp[0][3]=0.031250000;
        Bfp[1][0]=0.031250000; Bfp[1][1]=0.031250000; Bfp[1][2]=0.031250000; Bfp[1][3]=0.031250000;
        Bfp[2][0]=32.000000000; Bfp[2][1]=32.000000000; Bfp[2][2]=32.000000000; Bfp[2][3]=32.000000000;
        Bfp[3][0]=32.000000000; Bfp[3][1]=32.000000000; Bfp[3][2]=32.000000000; Bfp[3][3]=32.000000000;
        run_case(104, "margin int_part=-10 CLIFF");

        // ---- Test 105: margin int_part=-12 ----
        Afp[0][0]=0.015625000; Afp[0][1]=0.015625000; Afp[0][2]=64.000000000; Afp[0][3]=64.000000000;
        Afp[1][0]=0.015625000; Afp[1][1]=0.015625000; Afp[1][2]=64.000000000; Afp[1][3]=64.000000000;
        Afp[2][0]=0.015625000; Afp[2][1]=0.015625000; Afp[2][2]=64.000000000; Afp[2][3]=64.000000000;
        Afp[3][0]=0.015625000; Afp[3][1]=0.015625000; Afp[3][2]=64.000000000; Afp[3][3]=64.000000000;
        Bfp[0][0]=0.015625000; Bfp[0][1]=0.015625000; Bfp[0][2]=0.015625000; Bfp[0][3]=0.015625000;
        Bfp[1][0]=0.015625000; Bfp[1][1]=0.015625000; Bfp[1][2]=0.015625000; Bfp[1][3]=0.015625000;
        Bfp[2][0]=64.000000000; Bfp[2][1]=64.000000000; Bfp[2][2]=64.000000000; Bfp[2][3]=64.000000000;
        Bfp[3][0]=64.000000000; Bfp[3][1]=64.000000000; Bfp[3][2]=64.000000000; Bfp[3][3]=64.000000000;
        run_case(105, "margin int_part=-12");

        // ---- Test 106: margin int_part=-16 ----
        Afp[0][0]=0.003906250; Afp[0][1]=0.003906250; Afp[0][2]=256.000000000; Afp[0][3]=256.000000000;
        Afp[1][0]=0.003906250; Afp[1][1]=0.003906250; Afp[1][2]=256.000000000; Afp[1][3]=256.000000000;
        Afp[2][0]=0.003906250; Afp[2][1]=0.003906250; Afp[2][2]=256.000000000; Afp[2][3]=256.000000000;
        Afp[3][0]=0.003906250; Afp[3][1]=0.003906250; Afp[3][2]=256.000000000; Afp[3][3]=256.000000000;
        Bfp[0][0]=0.003906250; Bfp[0][1]=0.003906250; Bfp[0][2]=0.003906250; Bfp[0][3]=0.003906250;
        Bfp[1][0]=0.003906250; Bfp[1][1]=0.003906250; Bfp[1][2]=0.003906250; Bfp[1][3]=0.003906250;
        Bfp[2][0]=256.000000000; Bfp[2][1]=256.000000000; Bfp[2][2]=256.000000000; Bfp[2][3]=256.000000000;
        Bfp[3][0]=256.000000000; Bfp[3][1]=256.000000000; Bfp[3][2]=256.000000000; Bfp[3][3]=256.000000000;
        run_case(106, "margin int_part=-16");

        // ---- Test 107: margin int_part=-20 DEAD ----
        Afp[0][0]=0.000976562; Afp[0][1]=0.000976562; Afp[0][2]=1024.000000000; Afp[0][3]=1024.000000000;
        Afp[1][0]=0.000976562; Afp[1][1]=0.000976562; Afp[1][2]=1024.000000000; Afp[1][3]=1024.000000000;
        Afp[2][0]=0.000976562; Afp[2][1]=0.000976562; Afp[2][2]=1024.000000000; Afp[2][3]=1024.000000000;
        Afp[3][0]=0.000976562; Afp[3][1]=0.000976562; Afp[3][2]=1024.000000000; Afp[3][3]=1024.000000000;
        Bfp[0][0]=0.000976562; Bfp[0][1]=0.000976562; Bfp[0][2]=0.000976562; Bfp[0][3]=0.000976562;
        Bfp[1][0]=0.000976562; Bfp[1][1]=0.000976562; Bfp[1][2]=0.000976562; Bfp[1][3]=0.000976562;
        Bfp[2][0]=1024.000000000; Bfp[2][1]=1024.000000000; Bfp[2][2]=1024.000000000; Bfp[2][3]=1024.000000000;
        Bfp[3][0]=1024.000000000; Bfp[3][1]=1024.000000000; Bfp[3][2]=1024.000000000; Bfp[3][3]=1024.000000000;
        run_case(107, "margin int_part=-20 DEAD");

        // ---- Test 108: spread 2x in block ----
        Afp[0][0]=0.707106780; Afp[0][1]=0.707106780; Afp[0][2]=1.414214000; Afp[0][3]=1.414214000;
        Afp[1][0]=0.707106780; Afp[1][1]=0.707106780; Afp[1][2]=1.414214000; Afp[1][3]=1.414214000;
        Afp[2][0]=0.707106780; Afp[2][1]=0.707106780; Afp[2][2]=1.414214000; Afp[2][3]=1.414214000;
        Afp[3][0]=0.707106780; Afp[3][1]=0.707106780; Afp[3][2]=1.414214000; Afp[3][3]=1.414214000;
        Bfp[0][0]=1.000000000; Bfp[0][1]=1.000000000; Bfp[0][2]=1.000000000; Bfp[0][3]=1.000000000;
        Bfp[1][0]=1.000000000; Bfp[1][1]=1.000000000; Bfp[1][2]=1.000000000; Bfp[1][3]=1.000000000;
        Bfp[2][0]=1.000000000; Bfp[2][1]=1.000000000; Bfp[2][2]=1.000000000; Bfp[2][3]=1.000000000;
        Bfp[3][0]=1.000000000; Bfp[3][1]=1.000000000; Bfp[3][2]=1.000000000; Bfp[3][3]=1.000000000;
        run_case(108, "spread 2x in block");

        // ---- Test 109: spread 4x in block ----
        Afp[0][0]=0.500000000; Afp[0][1]=0.500000000; Afp[0][2]=2.000000000; Afp[0][3]=2.000000000;
        Afp[1][0]=0.500000000; Afp[1][1]=0.500000000; Afp[1][2]=2.000000000; Afp[1][3]=2.000000000;
        Afp[2][0]=0.500000000; Afp[2][1]=0.500000000; Afp[2][2]=2.000000000; Afp[2][3]=2.000000000;
        Afp[3][0]=0.500000000; Afp[3][1]=0.500000000; Afp[3][2]=2.000000000; Afp[3][3]=2.000000000;
        Bfp[0][0]=1.000000000; Bfp[0][1]=1.000000000; Bfp[0][2]=1.000000000; Bfp[0][3]=1.000000000;
        Bfp[1][0]=1.000000000; Bfp[1][1]=1.000000000; Bfp[1][2]=1.000000000; Bfp[1][3]=1.000000000;
        Bfp[2][0]=1.000000000; Bfp[2][1]=1.000000000; Bfp[2][2]=1.000000000; Bfp[2][3]=1.000000000;
        Bfp[3][0]=1.000000000; Bfp[3][1]=1.000000000; Bfp[3][2]=1.000000000; Bfp[3][3]=1.000000000;
        run_case(109, "spread 4x in block");

        // ---- Test 110: spread 10x in block ----
        Afp[0][0]=0.316227770; Afp[0][1]=0.316227770; Afp[0][2]=3.162278000; Afp[0][3]=3.162278000;
        Afp[1][0]=0.316227770; Afp[1][1]=0.316227770; Afp[1][2]=3.162278000; Afp[1][3]=3.162278000;
        Afp[2][0]=0.316227770; Afp[2][1]=0.316227770; Afp[2][2]=3.162278000; Afp[2][3]=3.162278000;
        Afp[3][0]=0.316227770; Afp[3][1]=0.316227770; Afp[3][2]=3.162278000; Afp[3][3]=3.162278000;
        Bfp[0][0]=1.000000000; Bfp[0][1]=1.000000000; Bfp[0][2]=1.000000000; Bfp[0][3]=1.000000000;
        Bfp[1][0]=1.000000000; Bfp[1][1]=1.000000000; Bfp[1][2]=1.000000000; Bfp[1][3]=1.000000000;
        Bfp[2][0]=1.000000000; Bfp[2][1]=1.000000000; Bfp[2][2]=1.000000000; Bfp[2][3]=1.000000000;
        Bfp[3][0]=1.000000000; Bfp[3][1]=1.000000000; Bfp[3][2]=1.000000000; Bfp[3][3]=1.000000000;
        run_case(110, "spread 10x in block");

        // ---- Test 111: spread 16x in block ----
        Afp[0][0]=0.250000000; Afp[0][1]=0.250000000; Afp[0][2]=4.000000000; Afp[0][3]=4.000000000;
        Afp[1][0]=0.250000000; Afp[1][1]=0.250000000; Afp[1][2]=4.000000000; Afp[1][3]=4.000000000;
        Afp[2][0]=0.250000000; Afp[2][1]=0.250000000; Afp[2][2]=4.000000000; Afp[2][3]=4.000000000;
        Afp[3][0]=0.250000000; Afp[3][1]=0.250000000; Afp[3][2]=4.000000000; Afp[3][3]=4.000000000;
        Bfp[0][0]=1.000000000; Bfp[0][1]=1.000000000; Bfp[0][2]=1.000000000; Bfp[0][3]=1.000000000;
        Bfp[1][0]=1.000000000; Bfp[1][1]=1.000000000; Bfp[1][2]=1.000000000; Bfp[1][3]=1.000000000;
        Bfp[2][0]=1.000000000; Bfp[2][1]=1.000000000; Bfp[2][2]=1.000000000; Bfp[2][3]=1.000000000;
        Bfp[3][0]=1.000000000; Bfp[3][1]=1.000000000; Bfp[3][2]=1.000000000; Bfp[3][3]=1.000000000;
        run_case(111, "spread 16x in block");

        // ---- Test 112: spread 100x in block ----
        Afp[0][0]=0.100000000; Afp[0][1]=0.100000000; Afp[0][2]=10.000000000; Afp[0][3]=10.000000000;
        Afp[1][0]=0.100000000; Afp[1][1]=0.100000000; Afp[1][2]=10.000000000; Afp[1][3]=10.000000000;
        Afp[2][0]=0.100000000; Afp[2][1]=0.100000000; Afp[2][2]=10.000000000; Afp[2][3]=10.000000000;
        Afp[3][0]=0.100000000; Afp[3][1]=0.100000000; Afp[3][2]=10.000000000; Afp[3][3]=10.000000000;
        Bfp[0][0]=1.000000000; Bfp[0][1]=1.000000000; Bfp[0][2]=1.000000000; Bfp[0][3]=1.000000000;
        Bfp[1][0]=1.000000000; Bfp[1][1]=1.000000000; Bfp[1][2]=1.000000000; Bfp[1][3]=1.000000000;
        Bfp[2][0]=1.000000000; Bfp[2][1]=1.000000000; Bfp[2][2]=1.000000000; Bfp[2][3]=1.000000000;
        Bfp[3][0]=1.000000000; Bfp[3][1]=1.000000000; Bfp[3][2]=1.000000000; Bfp[3][3]=1.000000000;
        run_case(112, "spread 100x in block");

        // ---- Test 113: spread 1000x in block ----
        Afp[0][0]=0.031622780; Afp[0][1]=0.031622780; Afp[0][2]=31.622777000; Afp[0][3]=31.622777000;
        Afp[1][0]=0.031622780; Afp[1][1]=0.031622780; Afp[1][2]=31.622777000; Afp[1][3]=31.622777000;
        Afp[2][0]=0.031622780; Afp[2][1]=0.031622780; Afp[2][2]=31.622777000; Afp[2][3]=31.622777000;
        Afp[3][0]=0.031622780; Afp[3][1]=0.031622780; Afp[3][2]=31.622777000; Afp[3][3]=31.622777000;
        Bfp[0][0]=1.000000000; Bfp[0][1]=1.000000000; Bfp[0][2]=1.000000000; Bfp[0][3]=1.000000000;
        Bfp[1][0]=1.000000000; Bfp[1][1]=1.000000000; Bfp[1][2]=1.000000000; Bfp[1][3]=1.000000000;
        Bfp[2][0]=1.000000000; Bfp[2][1]=1.000000000; Bfp[2][2]=1.000000000; Bfp[2][3]=1.000000000;
        Bfp[3][0]=1.000000000; Bfp[3][1]=1.000000000; Bfp[3][2]=1.000000000; Bfp[3][3]=1.000000000;
        run_case(113, "spread 1000x in block");

        // ---- Test 114: spread 10000x in block ----
        Afp[0][0]=0.010000000; Afp[0][1]=0.010000000; Afp[0][2]=100.000000000; Afp[0][3]=100.000000000;
        Afp[1][0]=0.010000000; Afp[1][1]=0.010000000; Afp[1][2]=100.000000000; Afp[1][3]=100.000000000;
        Afp[2][0]=0.010000000; Afp[2][1]=0.010000000; Afp[2][2]=100.000000000; Afp[2][3]=100.000000000;
        Afp[3][0]=0.010000000; Afp[3][1]=0.010000000; Afp[3][2]=100.000000000; Afp[3][3]=100.000000000;
        Bfp[0][0]=1.000000000; Bfp[0][1]=1.000000000; Bfp[0][2]=1.000000000; Bfp[0][3]=1.000000000;
        Bfp[1][0]=1.000000000; Bfp[1][1]=1.000000000; Bfp[1][2]=1.000000000; Bfp[1][3]=1.000000000;
        Bfp[2][0]=1.000000000; Bfp[2][1]=1.000000000; Bfp[2][2]=1.000000000; Bfp[2][3]=1.000000000;
        Bfp[3][0]=1.000000000; Bfp[3][1]=1.000000000; Bfp[3][2]=1.000000000; Bfp[3][3]=1.000000000;
        run_case(114, "spread 10000x in block");

        // ---- Test 115: magnitude 1e-30 uniform ----
        Afp[0][0]=0.000000000; Afp[0][1]=0.000000000; Afp[0][2]=0.000000000; Afp[0][3]=0.000000000;
        Afp[1][0]=0.000000000; Afp[1][1]=0.000000000; Afp[1][2]=0.000000000; Afp[1][3]=0.000000000;
        Afp[2][0]=0.000000000; Afp[2][1]=0.000000000; Afp[2][2]=0.000000000; Afp[2][3]=0.000000000;
        Afp[3][0]=0.000000000; Afp[3][1]=0.000000000; Afp[3][2]=0.000000000; Afp[3][3]=0.000000000;
        Bfp[0][0]=0.000000000; Bfp[0][1]=0.000000000; Bfp[0][2]=0.000000000; Bfp[0][3]=0.000000000;
        Bfp[1][0]=0.000000000; Bfp[1][1]=0.000000000; Bfp[1][2]=0.000000000; Bfp[1][3]=0.000000000;
        Bfp[2][0]=0.000000000; Bfp[2][1]=0.000000000; Bfp[2][2]=0.000000000; Bfp[2][3]=0.000000000;
        Bfp[3][0]=0.000000000; Bfp[3][1]=0.000000000; Bfp[3][2]=0.000000000; Bfp[3][3]=0.000000000;
        run_case(115, "magnitude 1e-30 uniform");

        // ---- Test 116: magnitude 1e-15 uniform ----
        Afp[0][0]=0.000000000; Afp[0][1]=0.000000000; Afp[0][2]=0.000000000; Afp[0][3]=0.000000000;
        Afp[1][0]=0.000000000; Afp[1][1]=0.000000000; Afp[1][2]=0.000000000; Afp[1][3]=0.000000000;
        Afp[2][0]=0.000000000; Afp[2][1]=0.000000000; Afp[2][2]=0.000000000; Afp[2][3]=0.000000000;
        Afp[3][0]=0.000000000; Afp[3][1]=0.000000000; Afp[3][2]=0.000000000; Afp[3][3]=0.000000000;
        Bfp[0][0]=0.000000000; Bfp[0][1]=0.000000000; Bfp[0][2]=0.000000000; Bfp[0][3]=0.000000000;
        Bfp[1][0]=0.000000000; Bfp[1][1]=0.000000000; Bfp[1][2]=0.000000000; Bfp[1][3]=0.000000000;
        Bfp[2][0]=0.000000000; Bfp[2][1]=0.000000000; Bfp[2][2]=0.000000000; Bfp[2][3]=0.000000000;
        Bfp[3][0]=0.000000000; Bfp[3][1]=0.000000000; Bfp[3][2]=0.000000000; Bfp[3][3]=0.000000000;
        run_case(116, "magnitude 1e-15 uniform");

        // ---- Test 117: magnitude 1e-5 uniform ----
        Afp[0][0]=0.000010000; Afp[0][1]=0.000010000; Afp[0][2]=0.000010000; Afp[0][3]=0.000010000;
        Afp[1][0]=0.000010000; Afp[1][1]=0.000010000; Afp[1][2]=0.000010000; Afp[1][3]=0.000010000;
        Afp[2][0]=0.000010000; Afp[2][1]=0.000010000; Afp[2][2]=0.000010000; Afp[2][3]=0.000010000;
        Afp[3][0]=0.000010000; Afp[3][1]=0.000010000; Afp[3][2]=0.000010000; Afp[3][3]=0.000010000;
        Bfp[0][0]=0.000010000; Bfp[0][1]=0.000010000; Bfp[0][2]=0.000010000; Bfp[0][3]=0.000010000;
        Bfp[1][0]=0.000010000; Bfp[1][1]=0.000010000; Bfp[1][2]=0.000010000; Bfp[1][3]=0.000010000;
        Bfp[2][0]=0.000010000; Bfp[2][1]=0.000010000; Bfp[2][2]=0.000010000; Bfp[2][3]=0.000010000;
        Bfp[3][0]=0.000010000; Bfp[3][1]=0.000010000; Bfp[3][2]=0.000010000; Bfp[3][3]=0.000010000;
        run_case(117, "magnitude 1e-5 uniform");

        // ---- Test 118: magnitude 1e5 uniform ----
        Afp[0][0]=100000.000000000; Afp[0][1]=100000.000000000; Afp[0][2]=100000.000000000; Afp[0][3]=100000.000000000;
        Afp[1][0]=100000.000000000; Afp[1][1]=100000.000000000; Afp[1][2]=100000.000000000; Afp[1][3]=100000.000000000;
        Afp[2][0]=100000.000000000; Afp[2][1]=100000.000000000; Afp[2][2]=100000.000000000; Afp[2][3]=100000.000000000;
        Afp[3][0]=100000.000000000; Afp[3][1]=100000.000000000; Afp[3][2]=100000.000000000; Afp[3][3]=100000.000000000;
        Bfp[0][0]=100000.000000000; Bfp[0][1]=100000.000000000; Bfp[0][2]=100000.000000000; Bfp[0][3]=100000.000000000;
        Bfp[1][0]=100000.000000000; Bfp[1][1]=100000.000000000; Bfp[1][2]=100000.000000000; Bfp[1][3]=100000.000000000;
        Bfp[2][0]=100000.000000000; Bfp[2][1]=100000.000000000; Bfp[2][2]=100000.000000000; Bfp[2][3]=100000.000000000;
        Bfp[3][0]=100000.000000000; Bfp[3][1]=100000.000000000; Bfp[3][2]=100000.000000000; Bfp[3][3]=100000.000000000;
        run_case(118, "magnitude 1e5 uniform");

        // ---- Test 119: magnitude 1e15 uniform ----
        Afp[0][0]=1000000000000000.000000000; Afp[0][1]=1000000000000000.000000000; Afp[0][2]=1000000000000000.000000000; Afp[0][3]=1000000000000000.000000000;
        Afp[1][0]=1000000000000000.000000000; Afp[1][1]=1000000000000000.000000000; Afp[1][2]=1000000000000000.000000000; Afp[1][3]=1000000000000000.000000000;
        Afp[2][0]=1000000000000000.000000000; Afp[2][1]=1000000000000000.000000000; Afp[2][2]=1000000000000000.000000000; Afp[2][3]=1000000000000000.000000000;
        Afp[3][0]=1000000000000000.000000000; Afp[3][1]=1000000000000000.000000000; Afp[3][2]=1000000000000000.000000000; Afp[3][3]=1000000000000000.000000000;
        Bfp[0][0]=1000000000000000.000000000; Bfp[0][1]=1000000000000000.000000000; Bfp[0][2]=1000000000000000.000000000; Bfp[0][3]=1000000000000000.000000000;
        Bfp[1][0]=1000000000000000.000000000; Bfp[1][1]=1000000000000000.000000000; Bfp[1][2]=1000000000000000.000000000; Bfp[1][3]=1000000000000000.000000000;
        Bfp[2][0]=1000000000000000.000000000; Bfp[2][1]=1000000000000000.000000000; Bfp[2][2]=1000000000000000.000000000; Bfp[2][3]=1000000000000000.000000000;
        Bfp[3][0]=1000000000000000.000000000; Bfp[3][1]=1000000000000000.000000000; Bfp[3][2]=1000000000000000.000000000; Bfp[3][3]=1000000000000000.000000000;
        run_case(119, "magnitude 1e15 uniform");

        // ---- Test 120: magnitude 1e30 uniform ----
        Afp[0][0]=1000000000000000019884624838656.000000000; Afp[0][1]=1000000000000000019884624838656.000000000; Afp[0][2]=1000000000000000019884624838656.000000000; Afp[0][3]=1000000000000000019884624838656.000000000;
        Afp[1][0]=1000000000000000019884624838656.000000000; Afp[1][1]=1000000000000000019884624838656.000000000; Afp[1][2]=1000000000000000019884624838656.000000000; Afp[1][3]=1000000000000000019884624838656.000000000;
        Afp[2][0]=1000000000000000019884624838656.000000000; Afp[2][1]=1000000000000000019884624838656.000000000; Afp[2][2]=1000000000000000019884624838656.000000000; Afp[2][3]=1000000000000000019884624838656.000000000;
        Afp[3][0]=1000000000000000019884624838656.000000000; Afp[3][1]=1000000000000000019884624838656.000000000; Afp[3][2]=1000000000000000019884624838656.000000000; Afp[3][3]=1000000000000000019884624838656.000000000;
        Bfp[0][0]=1000000000000000019884624838656.000000000; Bfp[0][1]=1000000000000000019884624838656.000000000; Bfp[0][2]=1000000000000000019884624838656.000000000; Bfp[0][3]=1000000000000000019884624838656.000000000;
        Bfp[1][0]=1000000000000000019884624838656.000000000; Bfp[1][1]=1000000000000000019884624838656.000000000; Bfp[1][2]=1000000000000000019884624838656.000000000; Bfp[1][3]=1000000000000000019884624838656.000000000;
        Bfp[2][0]=1000000000000000019884624838656.000000000; Bfp[2][1]=1000000000000000019884624838656.000000000; Bfp[2][2]=1000000000000000019884624838656.000000000; Bfp[2][3]=1000000000000000019884624838656.000000000;
        Bfp[3][0]=1000000000000000019884624838656.000000000; Bfp[3][1]=1000000000000000019884624838656.000000000; Bfp[3][2]=1000000000000000019884624838656.000000000; Bfp[3][3]=1000000000000000019884624838656.000000000;
        run_case(120, "magnitude 1e30 uniform");

        // ---- Test 121: row tiny col huge (1e-6 x 1e6) ----
        Afp[0][0]=0.000001000; Afp[0][1]=0.000001000; Afp[0][2]=0.000001000; Afp[0][3]=0.000001000;
        Afp[1][0]=0.000001000; Afp[1][1]=0.000001000; Afp[1][2]=0.000001000; Afp[1][3]=0.000001000;
        Afp[2][0]=0.000001000; Afp[2][1]=0.000001000; Afp[2][2]=0.000001000; Afp[2][3]=0.000001000;
        Afp[3][0]=0.000001000; Afp[3][1]=0.000001000; Afp[3][2]=0.000001000; Afp[3][3]=0.000001000;
        Bfp[0][0]=1000000.000000000; Bfp[0][1]=1000000.000000000; Bfp[0][2]=1000000.000000000; Bfp[0][3]=1000000.000000000;
        Bfp[1][0]=1000000.000000000; Bfp[1][1]=1000000.000000000; Bfp[1][2]=1000000.000000000; Bfp[1][3]=1000000.000000000;
        Bfp[2][0]=1000000.000000000; Bfp[2][1]=1000000.000000000; Bfp[2][2]=1000000.000000000; Bfp[2][3]=1000000.000000000;
        Bfp[3][0]=1000000.000000000; Bfp[3][1]=1000000.000000000; Bfp[3][2]=1000000.000000000; Bfp[3][3]=1000000.000000000;
        run_case(121, "row tiny col huge (1e-6 x 1e6)");

        // ---- Test 122: overflow probe 2^12 ----
        Afp[0][0]=64.000000000; Afp[0][1]=64.000000000; Afp[0][2]=64.000000000; Afp[0][3]=64.000000000;
        Afp[1][0]=64.000000000; Afp[1][1]=64.000000000; Afp[1][2]=64.000000000; Afp[1][3]=64.000000000;
        Afp[2][0]=64.000000000; Afp[2][1]=64.000000000; Afp[2][2]=64.000000000; Afp[2][3]=64.000000000;
        Afp[3][0]=64.000000000; Afp[3][1]=64.000000000; Afp[3][2]=64.000000000; Afp[3][3]=64.000000000;
        Bfp[0][0]=64.000000000; Bfp[0][1]=64.000000000; Bfp[0][2]=64.000000000; Bfp[0][3]=64.000000000;
        Bfp[1][0]=64.000000000; Bfp[1][1]=64.000000000; Bfp[1][2]=64.000000000; Bfp[1][3]=64.000000000;
        Bfp[2][0]=64.000000000; Bfp[2][1]=64.000000000; Bfp[2][2]=64.000000000; Bfp[2][3]=64.000000000;
        Bfp[3][0]=64.000000000; Bfp[3][1]=64.000000000; Bfp[3][2]=64.000000000; Bfp[3][3]=64.000000000;
        run_case(122, "overflow probe 2^12");

        // ---- Test 123: overflow probe 2^16 ----
        Afp[0][0]=256.000000000; Afp[0][1]=256.000000000; Afp[0][2]=256.000000000; Afp[0][3]=256.000000000;
        Afp[1][0]=256.000000000; Afp[1][1]=256.000000000; Afp[1][2]=256.000000000; Afp[1][3]=256.000000000;
        Afp[2][0]=256.000000000; Afp[2][1]=256.000000000; Afp[2][2]=256.000000000; Afp[2][3]=256.000000000;
        Afp[3][0]=256.000000000; Afp[3][1]=256.000000000; Afp[3][2]=256.000000000; Afp[3][3]=256.000000000;
        Bfp[0][0]=256.000000000; Bfp[0][1]=256.000000000; Bfp[0][2]=256.000000000; Bfp[0][3]=256.000000000;
        Bfp[1][0]=256.000000000; Bfp[1][1]=256.000000000; Bfp[1][2]=256.000000000; Bfp[1][3]=256.000000000;
        Bfp[2][0]=256.000000000; Bfp[2][1]=256.000000000; Bfp[2][2]=256.000000000; Bfp[2][3]=256.000000000;
        Bfp[3][0]=256.000000000; Bfp[3][1]=256.000000000; Bfp[3][2]=256.000000000; Bfp[3][3]=256.000000000;
        run_case(123, "overflow probe 2^16");

        // ---- Test 124: overflow probe 2^18 ----
        Afp[0][0]=512.000000000; Afp[0][1]=512.000000000; Afp[0][2]=512.000000000; Afp[0][3]=512.000000000;
        Afp[1][0]=512.000000000; Afp[1][1]=512.000000000; Afp[1][2]=512.000000000; Afp[1][3]=512.000000000;
        Afp[2][0]=512.000000000; Afp[2][1]=512.000000000; Afp[2][2]=512.000000000; Afp[2][3]=512.000000000;
        Afp[3][0]=512.000000000; Afp[3][1]=512.000000000; Afp[3][2]=512.000000000; Afp[3][3]=512.000000000;
        Bfp[0][0]=512.000000000; Bfp[0][1]=512.000000000; Bfp[0][2]=512.000000000; Bfp[0][3]=512.000000000;
        Bfp[1][0]=512.000000000; Bfp[1][1]=512.000000000; Bfp[1][2]=512.000000000; Bfp[1][3]=512.000000000;
        Bfp[2][0]=512.000000000; Bfp[2][1]=512.000000000; Bfp[2][2]=512.000000000; Bfp[2][3]=512.000000000;
        Bfp[3][0]=512.000000000; Bfp[3][1]=512.000000000; Bfp[3][2]=512.000000000; Bfp[3][3]=512.000000000;
        run_case(124, "overflow probe 2^18");

        // ---- Test 125: overflow probe 2^19 ----
        Afp[0][0]=724.077300000; Afp[0][1]=724.077300000; Afp[0][2]=724.077300000; Afp[0][3]=724.077300000;
        Afp[1][0]=724.077300000; Afp[1][1]=724.077300000; Afp[1][2]=724.077300000; Afp[1][3]=724.077300000;
        Afp[2][0]=724.077300000; Afp[2][1]=724.077300000; Afp[2][2]=724.077300000; Afp[2][3]=724.077300000;
        Afp[3][0]=724.077300000; Afp[3][1]=724.077300000; Afp[3][2]=724.077300000; Afp[3][3]=724.077300000;
        Bfp[0][0]=724.077300000; Bfp[0][1]=724.077300000; Bfp[0][2]=724.077300000; Bfp[0][3]=724.077300000;
        Bfp[1][0]=724.077300000; Bfp[1][1]=724.077300000; Bfp[1][2]=724.077300000; Bfp[1][3]=724.077300000;
        Bfp[2][0]=724.077300000; Bfp[2][1]=724.077300000; Bfp[2][2]=724.077300000; Bfp[2][3]=724.077300000;
        Bfp[3][0]=724.077300000; Bfp[3][1]=724.077300000; Bfp[3][2]=724.077300000; Bfp[3][3]=724.077300000;
        run_case(125, "overflow probe 2^19");

        // ---- Test 126: single huge outlier in block ----
        Afp[0][0]=1.000000000; Afp[0][1]=1.000000000; Afp[0][2]=1.000000000; Afp[0][3]=1000.000000000;
        Afp[1][0]=1.000000000; Afp[1][1]=1.000000000; Afp[1][2]=1.000000000; Afp[1][3]=1000.000000000;
        Afp[2][0]=1.000000000; Afp[2][1]=1.000000000; Afp[2][2]=1.000000000; Afp[2][3]=1000.000000000;
        Afp[3][0]=1.000000000; Afp[3][1]=1.000000000; Afp[3][2]=1.000000000; Afp[3][3]=1000.000000000;
        Bfp[0][0]=1.000000000; Bfp[0][1]=1.000000000; Bfp[0][2]=1.000000000; Bfp[0][3]=1.000000000;
        Bfp[1][0]=1.000000000; Bfp[1][1]=1.000000000; Bfp[1][2]=1.000000000; Bfp[1][3]=1.000000000;
        Bfp[2][0]=1.000000000; Bfp[2][1]=1.000000000; Bfp[2][2]=1.000000000; Bfp[2][3]=1.000000000;
        Bfp[3][0]=1.000000000; Bfp[3][1]=1.000000000; Bfp[3][2]=1.000000000; Bfp[3][3]=1.000000000;
        run_case(126, "single huge outlier in block");

        // ---- Test 127: single tiny outlier in block ----
        Afp[0][0]=1.000000000; Afp[0][1]=1.000000000; Afp[0][2]=1.000000000; Afp[0][3]=0.001000000;
        Afp[1][0]=1.000000000; Afp[1][1]=1.000000000; Afp[1][2]=1.000000000; Afp[1][3]=0.001000000;
        Afp[2][0]=1.000000000; Afp[2][1]=1.000000000; Afp[2][2]=1.000000000; Afp[2][3]=0.001000000;
        Afp[3][0]=1.000000000; Afp[3][1]=1.000000000; Afp[3][2]=1.000000000; Afp[3][3]=0.001000000;
        Bfp[0][0]=1.000000000; Bfp[0][1]=1.000000000; Bfp[0][2]=1.000000000; Bfp[0][3]=1.000000000;
        Bfp[1][0]=1.000000000; Bfp[1][1]=1.000000000; Bfp[1][2]=1.000000000; Bfp[1][3]=1.000000000;
        Bfp[2][0]=1.000000000; Bfp[2][1]=1.000000000; Bfp[2][2]=1.000000000; Bfp[2][3]=1.000000000;
        Bfp[3][0]=1.000000000; Bfp[3][1]=1.000000000; Bfp[3][2]=1.000000000; Bfp[3][3]=1.000000000;
        run_case(127, "single tiny outlier in block");

        // ---- Test 128: mixed 1e-6 and 1e6 in block ----
        Afp[0][0]=0.000001000; Afp[0][1]=1000000.000000000; Afp[0][2]=0.000001000; Afp[0][3]=1000000.000000000;
        Afp[1][0]=0.000001000; Afp[1][1]=1000000.000000000; Afp[1][2]=0.000001000; Afp[1][3]=1000000.000000000;
        Afp[2][0]=0.000001000; Afp[2][1]=1000000.000000000; Afp[2][2]=0.000001000; Afp[2][3]=1000000.000000000;
        Afp[3][0]=0.000001000; Afp[3][1]=1000000.000000000; Afp[3][2]=0.000001000; Afp[3][3]=1000000.000000000;
        Bfp[0][0]=1.000000000; Bfp[0][1]=1.000000000; Bfp[0][2]=1.000000000; Bfp[0][3]=1.000000000;
        Bfp[1][0]=1.000000000; Bfp[1][1]=1.000000000; Bfp[1][2]=1.000000000; Bfp[1][3]=1.000000000;
        Bfp[2][0]=1.000000000; Bfp[2][1]=1.000000000; Bfp[2][2]=1.000000000; Bfp[2][3]=1.000000000;
        Bfp[3][0]=1.000000000; Bfp[3][1]=1.000000000; Bfp[3][2]=1.000000000; Bfp[3][3]=1.000000000;
        run_case(128, "mixed 1e-6 and 1e6 in block");

        // ---- Test 129: ZERO single element (known gap) ----
        Afp[0][0]=0.000000000; Afp[0][1]=1.000000000; Afp[0][2]=2.000000000; Afp[0][3]=3.000000000;
        Afp[1][0]=0.000000000; Afp[1][1]=1.000000000; Afp[1][2]=2.000000000; Afp[1][3]=3.000000000;
        Afp[2][0]=0.000000000; Afp[2][1]=1.000000000; Afp[2][2]=2.000000000; Afp[2][3]=3.000000000;
        Afp[3][0]=0.000000000; Afp[3][1]=1.000000000; Afp[3][2]=2.000000000; Afp[3][3]=3.000000000;
        Bfp[0][0]=1.000000000; Bfp[0][1]=1.000000000; Bfp[0][2]=1.000000000; Bfp[0][3]=1.000000000;
        Bfp[1][0]=1.000000000; Bfp[1][1]=1.000000000; Bfp[1][2]=1.000000000; Bfp[1][3]=1.000000000;
        Bfp[2][0]=1.000000000; Bfp[2][1]=1.000000000; Bfp[2][2]=1.000000000; Bfp[2][3]=1.000000000;
        Bfp[3][0]=1.000000000; Bfp[3][1]=1.000000000; Bfp[3][2]=1.000000000; Bfp[3][3]=1.000000000;
        run_case(129, "ZERO single element (known gap)");

        // ---- Test 130: ZERO whole block (known gap) ----
        Afp[0][0]=0.000000000; Afp[0][1]=0.000000000; Afp[0][2]=0.000000000; Afp[0][3]=0.000000000;
        Afp[1][0]=0.000000000; Afp[1][1]=0.000000000; Afp[1][2]=0.000000000; Afp[1][3]=0.000000000;
        Afp[2][0]=0.000000000; Afp[2][1]=0.000000000; Afp[2][2]=0.000000000; Afp[2][3]=0.000000000;
        Afp[3][0]=0.000000000; Afp[3][1]=0.000000000; Afp[3][2]=0.000000000; Afp[3][3]=0.000000000;
        Bfp[0][0]=1.000000000; Bfp[0][1]=1.000000000; Bfp[0][2]=1.000000000; Bfp[0][3]=1.000000000;
        Bfp[1][0]=1.000000000; Bfp[1][1]=1.000000000; Bfp[1][2]=1.000000000; Bfp[1][3]=1.000000000;
        Bfp[2][0]=1.000000000; Bfp[2][1]=1.000000000; Bfp[2][2]=1.000000000; Bfp[2][3]=1.000000000;
        Bfp[3][0]=1.000000000; Bfp[3][1]=1.000000000; Bfp[3][2]=1.000000000; Bfp[3][3]=1.000000000;
        run_case(130, "ZERO whole block (known gap)");

        // ---- Test 131: near-cancel mixed sign ----
        Afp[0][0]=3.000000000; Afp[0][1]=-2.900000000; Afp[0][2]=3.100000000; Afp[0][3]=-3.200000000;
        Afp[1][0]=3.000000000; Afp[1][1]=-2.900000000; Afp[1][2]=3.100000000; Afp[1][3]=-3.200000000;
        Afp[2][0]=3.000000000; Afp[2][1]=-2.900000000; Afp[2][2]=3.100000000; Afp[2][3]=-3.200000000;
        Afp[3][0]=3.000000000; Afp[3][1]=-2.900000000; Afp[3][2]=3.100000000; Afp[3][3]=-3.200000000;
        Bfp[0][0]=1.000000000; Bfp[0][1]=1.000000000; Bfp[0][2]=1.000000000; Bfp[0][3]=1.000000000;
        Bfp[1][0]=1.000000000; Bfp[1][1]=1.000000000; Bfp[1][2]=1.000000000; Bfp[1][3]=1.000000000;
        Bfp[2][0]=1.000000000; Bfp[2][1]=1.000000000; Bfp[2][2]=1.000000000; Bfp[2][3]=1.000000000;
        Bfp[3][0]=1.000000000; Bfp[3][1]=1.000000000; Bfp[3][2]=1.000000000; Bfp[3][3]=1.000000000;
        run_case(131, "near-cancel mixed sign");

        // ---- Test 132: cancellation risk signed ----
        Afp[0][0]=3.687000000; Afp[0][1]=3.954000000; Afp[0][2]=2.427000000; Afp[0][3]=2.437000000;
        Afp[1][0]=1.636000000; Afp[1][1]=-1.200000000; Afp[1][2]=-4.393000000; Afp[1][3]=2.804000000;
        Afp[2][0]=3.350000000; Afp[2][1]=1.017000000; Afp[2][2]=-4.439000000; Afp[2][3]=-2.240000000;
        Afp[3][0]=2.215000000; Afp[3][1]=1.928000000; Afp[3][2]=2.594000000; Afp[3][3]=4.108000000;
        Bfp[0][0]=-3.554000000; Bfp[0][1]=4.770000000; Bfp[0][2]=-4.410000000; Bfp[0][3]=-1.519000000;
        Bfp[1][0]=-4.720000000; Bfp[1][1]=3.933000000; Bfp[1][2]=2.189000000; Bfp[1][3]=-3.034000000;
        Bfp[2][0]=4.583000000; Bfp[2][1]=3.610000000; Bfp[2][2]=2.162000000; Bfp[2][3]=-2.945000000;
        Bfp[3][0]=1.299000000; Bfp[3][1]=-3.975000000; Bfp[3][2]=-3.999000000; Bfp[3][3]=-4.317000000;
        run_case(132, "cancellation risk signed");

        // ---- Test 133: cancellation risk signed ----
        Afp[0][0]=-4.280000000; Afp[0][1]=4.053000000; Afp[0][2]=2.329000000; Afp[0][3]=1.483000000;
        Afp[1][0]=3.513000000; Afp[1][1]=2.101000000; Afp[1][2]=-3.722000000; Afp[1][3]=4.461000000;
        Afp[2][0]=-2.901000000; Afp[2][1]=-2.505000000; Afp[2][2]=3.626000000; Afp[2][3]=-2.239000000;
        Afp[3][0]=2.329000000; Afp[3][1]=2.616000000; Afp[3][2]=-2.208000000; Afp[3][3]=3.438000000;
        Bfp[0][0]=1.448000000; Bfp[0][1]=4.256000000; Bfp[0][2]=-2.490000000; Bfp[0][3]=-4.372000000;
        Bfp[1][0]=4.205000000; Bfp[1][1]=3.945000000; Bfp[1][2]=-3.119000000; Bfp[1][3]=-4.785000000;
        Bfp[2][0]=3.161000000; Bfp[2][1]=-4.829000000; Bfp[2][2]=-5.000000000; Bfp[2][3]=-3.224000000;
        Bfp[3][0]=4.403000000; Bfp[3][1]=2.458000000; Bfp[3][2]=1.300000000; Bfp[3][3]=2.502000000;
        run_case(133, "cancellation risk signed");

        // ---- Test 134: cancellation risk signed ----
        Afp[0][0]=-2.544000000; Afp[0][1]=-4.034000000; Afp[0][2]=3.898000000; Afp[0][3]=-4.119000000;
        Afp[1][0]=2.275000000; Afp[1][1]=-3.097000000; Afp[1][2]=1.760000000; Afp[1][3]=3.204000000;
        Afp[2][0]=2.697000000; Afp[2][1]=3.656000000; Afp[2][2]=4.854000000; Afp[2][3]=4.812000000;
        Afp[3][0]=2.984000000; Afp[3][1]=1.077000000; Afp[3][2]=3.180000000; Afp[3][3]=-1.395000000;
        Bfp[0][0]=2.961000000; Bfp[0][1]=4.078000000; Bfp[0][2]=4.243000000; Bfp[0][3]=-4.918000000;
        Bfp[1][0]=3.173000000; Bfp[1][1]=3.864000000; Bfp[1][2]=4.814000000; Bfp[1][3]=-1.037000000;
        Bfp[2][0]=-1.959000000; Bfp[2][1]=2.463000000; Bfp[2][2]=3.798000000; Bfp[2][3]=-2.259000000;
        Bfp[3][0]=3.842000000; Bfp[3][1]=-4.607000000; Bfp[3][2]=1.001000000; Bfp[3][3]=4.916000000;
        run_case(134, "cancellation risk signed");

        // ---- Test 135: cancellation risk signed ----
        Afp[0][0]=-4.339000000; Afp[0][1]=-2.556000000; Afp[0][2]=1.901000000; Afp[0][3]=4.233000000;
        Afp[1][0]=-2.036000000; Afp[1][1]=-3.811000000; Afp[1][2]=-3.455000000; Afp[1][3]=-1.301000000;
        Afp[2][0]=1.851000000; Afp[2][1]=-4.163000000; Afp[2][2]=2.078000000; Afp[2][3]=-4.370000000;
        Afp[3][0]=-1.633000000; Afp[3][1]=1.289000000; Afp[3][2]=-1.123000000; Afp[3][3]=1.106000000;
        Bfp[0][0]=1.898000000; Bfp[0][1]=3.662000000; Bfp[0][2]=3.692000000; Bfp[0][3]=4.360000000;
        Bfp[1][0]=4.245000000; Bfp[1][1]=4.976000000; Bfp[1][2]=1.205000000; Bfp[1][3]=3.279000000;
        Bfp[2][0]=-4.419000000; Bfp[2][1]=4.815000000; Bfp[2][2]=-2.077000000; Bfp[2][3]=-1.722000000;
        Bfp[3][0]=4.854000000; Bfp[3][1]=-3.775000000; Bfp[3][2]=-4.934000000; Bfp[3][3]=1.498000000;
        run_case(135, "cancellation risk signed");

        // ---- Test 136: cancellation risk signed ----
        Afp[0][0]=-4.830000000; Afp[0][1]=3.460000000; Afp[0][2]=-3.387000000; Afp[0][3]=4.528000000;
        Afp[1][0]=-2.573000000; Afp[1][1]=-2.886000000; Afp[1][2]=-2.037000000; Afp[1][3]=-2.810000000;
        Afp[2][0]=2.184000000; Afp[2][1]=4.946000000; Afp[2][2]=-3.910000000; Afp[2][3]=-4.798000000;
        Afp[3][0]=-3.916000000; Afp[3][1]=-1.822000000; Afp[3][2]=4.695000000; Afp[3][3]=-4.537000000;
        Bfp[0][0]=-4.649000000; Bfp[0][1]=-2.244000000; Bfp[0][2]=-2.127000000; Bfp[0][3]=-2.808000000;
        Bfp[1][0]=-4.943000000; Bfp[1][1]=-4.589000000; Bfp[1][2]=3.270000000; Bfp[1][3]=3.172000000;
        Bfp[2][0]=2.506000000; Bfp[2][1]=-3.197000000; Bfp[2][2]=-2.397000000; Bfp[2][3]=-2.595000000;
        Bfp[3][0]=-1.699000000; Bfp[3][1]=-1.530000000; Bfp[3][2]=1.322000000; Bfp[3][3]=-1.702000000;
        run_case(136, "cancellation risk signed");

        // ================= NON-ZERO COUNT TESTS =================
        // format_convertor takes E as the mean log over the NON-ZERO elements
        // of each output column, so it divides by n = 1, 2, 3 or 4. The tests
        // above only ever produce n = 4 (or an all-zero column). These make
        // columns of C with 1, 2 and 3 non-zero elements.

        // ---- Test 137: one column per n (diagonal A) ----
        // A diagonal -> C[i][j] = A[i][i] * B[i][j], so C has zeros exactly
        // where B has them: column 0 n=3, column 1 n=2, column 2 n=1, column 3 n=4
        Afp[0][0]=1.500000000; Afp[0][1]=0.0; Afp[0][2]=0.0; Afp[0][3]=0.0;
        Afp[1][0]=0.0; Afp[1][1]=-2.250000000; Afp[1][2]=0.0; Afp[1][3]=0.0;
        Afp[2][0]=0.0; Afp[2][1]=0.0; Afp[2][2]=3.000000000; Afp[2][3]=0.0;
        Afp[3][0]=0.0; Afp[3][1]=0.0; Afp[3][2]=0.0; Afp[3][3]=0.750000000;
        Bfp[0][0]=2.000000000; Bfp[0][1]=0.0; Bfp[0][2]=0.0; Bfp[0][3]=1.100000000;
        Bfp[1][0]=0.0; Bfp[1][1]=1.750000000; Bfp[1][2]=0.0; Bfp[1][3]=2.200000000;
        Bfp[2][0]=1.250000000; Bfp[2][1]=0.0; Bfp[2][2]=4.000000000; Bfp[2][3]=-3.300000000;
        Bfp[3][0]=3.500000000; Bfp[3][1]=2.500000000; Bfp[3][2]=0.0; Bfp[3][3]=0.900000000;
        run_case(137, "nonzero count n=3,2,1,4");

        // ---- Test 138: one zero row in A -> every column n=3 ----
        Afp[0][0]=1.200000000; Afp[0][1]=2.500000000; Afp[0][2]=0.800000000; Afp[0][3]=3.100000000;
        Afp[1][0]=2.000000000; Afp[1][1]=1.100000000; Afp[1][2]=1.700000000; Afp[1][3]=0.600000000;
        Afp[2][0]=0.0; Afp[2][1]=0.0; Afp[2][2]=0.0; Afp[2][3]=0.0;
        Afp[3][0]=0.900000000; Afp[3][1]=3.300000000; Afp[3][2]=2.200000000; Afp[3][3]=1.400000000;
        Bfp[0][0]=1.300000000; Bfp[0][1]=0.700000000; Bfp[0][2]=2.100000000; Bfp[0][3]=1.800000000;
        Bfp[1][0]=2.400000000; Bfp[1][1]=1.600000000; Bfp[1][2]=0.900000000; Bfp[1][3]=1.200000000;
        Bfp[2][0]=0.500000000; Bfp[2][1]=2.800000000; Bfp[2][2]=1.500000000; Bfp[2][3]=2.600000000;
        Bfp[3][0]=1.700000000; Bfp[3][1]=1.100000000; Bfp[3][2]=3.000000000; Bfp[3][3]=0.800000000;
        run_case(138, "nonzero count n=3 (zero row)");

        // ---- Test 139: two zero rows in A -> every column n=2 ----
        Afp[0][0]=0.0; Afp[0][1]=0.0; Afp[0][2]=0.0; Afp[0][3]=0.0;
        Afp[1][0]=2.000000000; Afp[1][1]=-1.100000000; Afp[1][2]=1.700000000; Afp[1][3]=0.600000000;
        Afp[2][0]=0.0; Afp[2][1]=0.0; Afp[2][2]=0.0; Afp[2][3]=0.0;
        Afp[3][0]=0.900000000; Afp[3][1]=3.300000000; Afp[3][2]=2.200000000; Afp[3][3]=1.400000000;
        Bfp[0][0]=1.300000000; Bfp[0][1]=0.700000000; Bfp[0][2]=2.100000000; Bfp[0][3]=1.800000000;
        Bfp[1][0]=2.400000000; Bfp[1][1]=1.600000000; Bfp[1][2]=0.900000000; Bfp[1][3]=1.200000000;
        Bfp[2][0]=0.500000000; Bfp[2][1]=2.800000000; Bfp[2][2]=1.500000000; Bfp[2][3]=2.600000000;
        Bfp[3][0]=1.700000000; Bfp[3][1]=1.100000000; Bfp[3][2]=3.000000000; Bfp[3][3]=0.800000000;
        run_case(139, "nonzero count n=2 (2 zero rows)");

        // ---- Test 140: three zero rows in A -> every column n=1 ----
        Afp[0][0]=0.0; Afp[0][1]=0.0; Afp[0][2]=0.0; Afp[0][3]=0.0;
        Afp[1][0]=0.0; Afp[1][1]=0.0; Afp[1][2]=0.0; Afp[1][3]=0.0;
        Afp[2][0]=1.200000000; Afp[2][1]=2.500000000; Afp[2][2]=0.800000000; Afp[2][3]=-3.100000000;
        Afp[3][0]=0.0; Afp[3][1]=0.0; Afp[3][2]=0.0; Afp[3][3]=0.0;
        Bfp[0][0]=1.300000000; Bfp[0][1]=0.700000000; Bfp[0][2]=2.100000000; Bfp[0][3]=1.800000000;
        Bfp[1][0]=2.400000000; Bfp[1][1]=1.600000000; Bfp[1][2]=0.900000000; Bfp[1][3]=1.200000000;
        Bfp[2][0]=0.500000000; Bfp[2][1]=2.800000000; Bfp[2][2]=1.500000000; Bfp[2][3]=2.600000000;
        Bfp[3][0]=1.700000000; Bfp[3][1]=1.100000000; Bfp[3][2]=3.000000000; Bfp[3][3]=0.800000000;
        run_case(140, "nonzero count n=1 (3 zero rows)");
        $display(" ------------------------------------------------------------------------------------");
        $display("");
        $display(" ============================ OVERALL SUMMARY ============================");
        $display("   total tests run        : %0d", ntests + s_run);
        $display("   passed                 : %0d", (ntests+s_run)-(nfail+s_fail));
        $display("   failed (any check)     : %0d", nfail + s_fail);
        $display("     Res_out  >10%% cond-err : %0d", c_res);
        $display("     exponent mismatches    : %0d", c_exp);
        $display("     output-block errors    : %0d", c_fc);
        $display("     missed PE overflows    : %0d", c_ovf_miss);
        $display("   tests with PE overflow flagged: %0d", c_ovf);
        $display("   blocks flagged out of range: %0d  (E clamped, reported, not a failure)", c_oor);
        $display("   blocks by non-zero count: n=0:%0d  n=1:%0d  n=2:%0d  n=3:%0d  n=4:%0d  %0s",
                 cov_n[0], cov_n[1], cov_n[2], cov_n[3], cov_n[4],
                 (cov_n[1]!=0 && cov_n[2]!=0 && cov_n[3]!=0 && cov_n[4]!=0) ? "(all divide paths used)" : "(*** SOME DIVIDE PATH NOT USED ***)");
        $display("   tests with sign flips  : %0d", s_flip);
        $display("   tests w/ exponent errors: %0d", s_experr);
        $display("   worst cond-err         : %.3f%%", (s_worst>0.0?s_worst:0.0)*100.0);
        $display("   worst block-normalised  : %.3f%%", s_fcworst*100.0);
        $display("   mean cond-err (appended): %.3f%%", (s_sum/s_run)*100.0);
        $display("   RESULT                 : %0s", ((nfail+s_fail)==0)?"ALL PASSED":"SOME FAILED");
        $display(" =========================================================================");
        $display("   detailed per-test report written to : test_report.txt");
        $display("");

        $fwrite(fh,"\n================================================================================\n");
        $fwrite(fh,"                              OVERALL SUMMARY\n");
        $fwrite(fh,"================================================================================\n");
        $fwrite(fh,"   total tests run         : %0d\n", ntests + s_run);
        $fwrite(fh,"   passed                  : %0d\n", (ntests+s_run)-(nfail+s_fail));
        $fwrite(fh,"   failed (any check)      : %0d\n", nfail + s_fail);
        $fwrite(fh,"     Res_out  >10%% cond-err  : %0d\n", c_res);
        $fwrite(fh,"     exponent mismatches     : %0d\n", c_exp);
        $fwrite(fh,"     output-block errors     : %0d\n", c_fc);
        $fwrite(fh,"     missed PE overflows     : %0d\n", c_ovf_miss);
        $fwrite(fh,"   tests with PE overflow flagged : %0d\n", c_ovf);
        $fwrite(fh,"   blocks flagged out of range : %0d  (E clamped, reported, not a failure)\n", c_oor);
        $fwrite(fh,"   blocks by non-zero count : n=0:%0d  n=1:%0d  n=2:%0d  n=3:%0d  n=4:%0d\n",
                cov_n[0], cov_n[1], cov_n[2], cov_n[3], cov_n[4]);
        $fwrite(fh,"   worst block-normalised    : %.3f %%\n", s_fcworst*100.0);
        $fwrite(fh,"   tests with sign flips   : %0d\n", s_flip);
        $fwrite(fh,"   worst cond-err          : %.3f %%\n", s_worst*100.0);
        $fwrite(fh,"   RESULT                  : %0s\n", ((nfail+s_fail)==0)?"ALL PASSED":"SOME FAILED");
        $fwrite(fh,"================================================================================\n");
        $fclose(fh);
        $finish;
    end

    initial begin $dumpfile("TOP_1_tb.vcd"); $dumpvars(0, tb_TOP_1); end
    initial begin #4000000; $display("WATCHDOG"); $finish; end
endmodule

