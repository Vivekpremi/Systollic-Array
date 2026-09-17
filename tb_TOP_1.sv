`timescale 1ns/1ps
//=============================================================================
// tb_TOP_1.sv -- 20-test floating-point regression suite
//                (blocks -> edge_reg_bank -> systolic_array -> iface -> FC)
//
// Every test is a RAW 4x4 A and 4x4 B written out explicitly. Each test does:
//     reset -> encode+load blocks -> run -> compare -> report
//
// Block encoding (done here, mirroring what the SW driver must do):
//     log   = round(log2(v) * 256)                  8.8 fixed point
//     scale = smallest s in 0..15 with all (log>>s) <= 31
//     mant  = round(log / 2^s)                       5 bits each
//     block = {exp[31:24], scale[23:20], m3,m2,m1,m0 [19:0]}   m0 in LSBs
// HW descaler reconstructs mant<<scale and reads it as 8.8.
//
// Row lane i gets A[i][0..3];  col lane j gets B[0..3][j].
// Res_out = 256 * true product.
//
// Difficulty ramps T1 -> T20: exact powers of two, near-1 floats, mid-range,
// larger values, then wide dynamic range inside one block (worst case for the
// shared scale factor).
//=============================================================================
module tb_TOP_1;
    parameter DATA_WIDTH=32, EXP_WIDTH=8, MANTISSA_WIDTH=16, ARRAY_SIZE=4;
    parameter BLOCK_MANTISSA_WIDTH=5, SCALE_FACTOR_WIDTH=4;
    localparam MBW = ARRAY_SIZE*BLOCK_MANTISSA_WIDTH;   // 20
    localparam BW  = EXP_WIDTH+SCALE_FACTOR_WIDTH+MBW;   // 32

    reg clk, rst_n;
    reg row_we [ARRAY_SIZE-1:0], col_we [ARRAY_SIZE-1:0];
    reg [BW-1:0] row_block [ARRAY_SIZE-1:0], col_block [ARRAY_SIZE-1:0];

    wire [ARRAY_SIZE-1:0] valid_out [ARRAY_SIZE-1:0];
    wire valid_exp_out, valid_man_out, last_exp_sent, mantissas_sent_out;
    wire block_ready, max_exp_calculated, bank_primed;
    wire [EXP_WIDTH-1:0] exponent_out_1, exponent_out_2, exp_m_out_1, exp_m_out_2, max_exp;
    wire [DATA_WIDTH-1:0] mantissa_out_1, mantissa_out_2;
    wire [1:0] iface_state_out;
    wire set_i_ready [ARRAY_SIZE-1:0];
    wire [DATA_WIDTH-1:0] Res_out [ARRAY_SIZE-1:0][ARRAY_SIZE-1:0];
    wire [SCALE_FACTOR_WIDTH-1:0] scale_factor;
    wire [MBW-1:0] mantissa_out;

    top_module #(.MANTISSA_WIDTH(16), .EXP_WIDTH(8), .DATA_WIDTH(32)) uut (
        .clk(clk), .rst_n(rst_n),
        .row_we(row_we), .col_we(col_we), .row_block(row_block), .col_block(col_block),
        .valid_out(valid_out), .valid_exp_out(valid_exp_out),
        .exponent_out_1(exponent_out_1), .exponent_out_2(exponent_out_2),
        .mantissa_out_1(mantissa_out_1), .mantissa_out_2(mantissa_out_2),
        .exp_m_out_1(exp_m_out_1), .exp_m_out_2(exp_m_out_2),
        .valid_man_out(valid_man_out), .set_i_ready(set_i_ready),
        .iface_state_out(iface_state_out), .Res_out(Res_out),
        .last_exp_sent(last_exp_sent), .mantissas_sent_out(mantissas_sent_out),
        .block_ready(block_ready), .scale_factor(scale_factor), .max_exp(max_exp),
        .mantissa_out(mantissa_out), .max_exp_calculated(max_exp_calculated),
        .bank_primed(bank_primed)
    );

    initial begin clk=0; forever #5 clk=~clk; end

    real Afp [0:3][0:3], Bfp [0:3][0:3], Cexp [0:3][0:3];
    integer i,j,k;
    integer ntests, nfail;
    real    acc_sum;
    reg [SCALE_FACTOR_WIDTH-1:0] sf_row, sf_col;

    // ---------------- block encoder ----------------
    function integer log8p8(input real v);
        real l;
        begin
            if (v <= 0.0) log8p8 = 0;
            else begin l = $ln(v)/$ln(2.0); log8p8 = $rtoi(l*256.0 + 0.5); end
        end
    endfunction

    function integer pick_scale(input integer l0,l1,l2,l3);
        integer s, mx, m0,m1,m2,m3;
        begin
            pick_scale = 15;
            for (s=0; s<16; s=s+1) begin
                m0=(l0+(1<<s)/2)>>s; m1=(l1+(1<<s)/2)>>s;
                m2=(l2+(1<<s)/2)>>s; m3=(l3+(1<<s)/2)>>s;
                mx = m0; if(m1>mx) mx=m1; if(m2>mx) mx=m2; if(m3>mx) mx=m3;
                if (mx <= 31 && pick_scale==15) pick_scale = s;
            end
        end
    endfunction

    function [BW-1:0] make_block(input real v0,v1,v2,v3, input [EXP_WIDTH-1:0] e);
        integer l0,l1,l2,l3,s,m0,m1,m2,m3;
        begin
            l0=log8p8(v0); l1=log8p8(v1); l2=log8p8(v2); l3=log8p8(v3);
            s = pick_scale(l0,l1,l2,l3);
            m0=(l0+(1<<s)/2)>>s; m1=(l1+(1<<s)/2)>>s;
            m2=(l2+(1<<s)/2)>>s; m3=(l3+(1<<s)/2)>>s;
            make_block = { e, s[3:0], m3[4:0], m2[4:0], m1[4:0], m0[4:0] };
        end
    endfunction

    task clrwe; integer w; begin for(w=0;w<4;w=w+1) begin row_we[w]=0; col_we[w]=0; end end endtask

    // ---------------- one test: reset, load, multiply, check ----------------
    task run_test(input integer id, input [8*32:1] name,
                  input [EXP_WIDTH-1:0] erow, input [EXP_WIDTH-1:0] ecol);
        real tot_err, a, e, d;
        integer bad;
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
                row_block[i] = make_block(Afp[i][0],Afp[i][1],Afp[i][2],Afp[i][3], erow);
                col_block[i] = make_block(Bfp[0][i],Bfp[1][i],Bfp[2][i],Bfp[3][i], ecol);
            end
            sf_row = row_block[0][23:20];
            sf_col = col_block[0][23:20];
            for(i=0;i<4;i=i+1) begin row_we[i]=1; col_we[i]=1; end
            @(posedge clk); clrwe;
            if(!bank_primed) $display("  WARN: bank_primed low after load");

            repeat(200) @(posedge clk);

            $display("");
            $display("--- T%0d: %0s   (row SF=%0d exp=%02h | col SF=%0d exp=%02h)",
                      id, name, sf_row, erow, sf_col, ecol);
            $display("   A                                   B");
            for(i=0;i<4;i=i+1)
                $display("   [%7.2f %7.2f %7.2f %7.2f]     [%7.2f %7.2f %7.2f %7.2f]",
                    Afp[i][0],Afp[i][1],Afp[i][2],Afp[i][3],
                    Bfp[i][0],Bfp[i][1],Bfp[i][2],Bfp[i][3]);
            $display("   expected C                          got C");
            for(i=0;i<4;i=i+1)
                $display("   [%8.2f %8.2f %8.2f %8.2f]   [%8.2f %8.2f %8.2f %8.2f]",
                    Cexp[i][0],Cexp[i][1],Cexp[i][2],Cexp[i][3],
                    $itor(Res_out[i][0])/256.0,$itor(Res_out[i][1])/256.0,
                    $itor(Res_out[i][2])/256.0,$itor(Res_out[i][3])/256.0);

            tot_err=0.0; bad=0; bad_str="";
            for(i=0;i<4;i=i+1) for(j=0;j<4;j=j+1) begin
                a = $itor(Res_out[i][j])/256.0; e = Cexp[i][j];
                d = (a>e)?(a-e):(e-a);
                if(e!=0.0) begin
                    tot_err = tot_err + d/e;
                    if (d > 0.10*e) begin
                        bad = bad + 1;
                        bad_str = {bad_str, $sformatf(" [%0d,%0d]", i,j)};
                    end
                end
            end
            $display("   accuracy = %6.2f%%   outside-10%%-tol = %0d/16  %0s %0s",
                      100.0*(1.0-tot_err/16.0), bad, bad_str , (bad==0)?"PASS":"FAIL");
            acc_sum = acc_sum + 100.0*(1.0-tot_err/16.0);
            ntests  = ntests + 1;
            if (bad!=0) nfail = nfail + 1;
        end
    endtask

    initial begin
        clrwe; ntests=0; nfail=0; acc_sum=0.0;
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
        run_test(1, "all ones", 8'h00, 8'h00);

        // ---------------- T2: all twos ----------------
        Afp[0][0]=2.00; Afp[0][1]=2.00; Afp[0][2]=2.00; Afp[0][3]=2.00;
        Afp[1][0]=2.00; Afp[1][1]=2.00; Afp[1][2]=2.00; Afp[1][3]=2.00;
        Afp[2][0]=2.00; Afp[2][1]=2.00; Afp[2][2]=2.00; Afp[2][3]=2.00;
        Afp[3][0]=2.00; Afp[3][1]=2.00; Afp[3][2]=2.00; Afp[3][3]=2.00;
        Bfp[0][0]=2.00; Bfp[0][1]=2.00; Bfp[0][2]=2.00; Bfp[0][3]=2.00;
        Bfp[1][0]=2.00; Bfp[1][1]=2.00; Bfp[1][2]=2.00; Bfp[1][3]=2.00;
        Bfp[2][0]=2.00; Bfp[2][1]=2.00; Bfp[2][2]=2.00; Bfp[2][3]=2.00;
        Bfp[3][0]=2.00; Bfp[3][1]=2.00; Bfp[3][2]=2.00; Bfp[3][3]=2.00;
        run_test(2, "all twos", 8'h01, 8'h02);

        // ---------------- T3: powers of two ----------------
        Afp[0][0]=2.00; Afp[0][1]=4.00; Afp[0][2]=8.00; Afp[0][3]=16.00;
        Afp[1][0]=4.00; Afp[1][1]=8.00; Afp[1][2]=16.00; Afp[1][3]=2.00;
        Afp[2][0]=8.00; Afp[2][1]=16.00; Afp[2][2]=2.00; Afp[2][3]=4.00;
        Afp[3][0]=16.00; Afp[3][1]=2.00; Afp[3][2]=4.00; Afp[3][3]=8.00;
        Bfp[0][0]=2.00; Bfp[0][1]=8.00; Bfp[0][2]=4.00; Bfp[0][3]=16.00;
        Bfp[1][0]=16.00; Bfp[1][1]=4.00; Bfp[1][2]=8.00; Bfp[1][3]=2.00;
        Bfp[2][0]=4.00; Bfp[2][1]=2.00; Bfp[2][2]=16.00; Bfp[2][3]=8.00;
        Bfp[3][0]=8.00; Bfp[3][1]=16.00; Bfp[3][2]=2.00; Bfp[3][3]=4.00;
        run_test(3, "powers of two", 8'h03, 8'h04);

        // ---------------- T4: small integers ----------------
        Afp[0][0]=1.00; Afp[0][1]=2.00; Afp[0][2]=3.00; Afp[0][3]=4.00;
        Afp[1][0]=4.00; Afp[1][1]=3.00; Afp[1][2]=2.00; Afp[1][3]=1.00;
        Afp[2][0]=2.00; Afp[2][1]=4.00; Afp[2][2]=1.00; Afp[2][3]=3.00;
        Afp[3][0]=3.00; Afp[3][1]=1.00; Afp[3][2]=4.00; Afp[3][3]=2.00;
        Bfp[0][0]=1.00; Bfp[0][1]=3.00; Bfp[0][2]=2.00; Bfp[0][3]=4.00;
        Bfp[1][0]=2.00; Bfp[1][1]=1.00; Bfp[1][2]=4.00; Bfp[1][3]=3.00;
        Bfp[2][0]=4.00; Bfp[2][1]=2.00; Bfp[2][2]=3.00; Bfp[2][3]=1.00;
        Bfp[3][0]=3.00; Bfp[3][1]=4.00; Bfp[3][2]=1.00; Bfp[3][3]=2.00;
        run_test(4, "small integers", 8'h05, 8'h06);

        // ---------------- T5: near-1 tight ----------------
        Afp[0][0]=1.10; Afp[0][1]=1.20; Afp[0][2]=1.30; Afp[0][3]=1.40;
        Afp[1][0]=1.40; Afp[1][1]=1.30; Afp[1][2]=1.20; Afp[1][3]=1.10;
        Afp[2][0]=1.15; Afp[2][1]=1.25; Afp[2][2]=1.35; Afp[2][3]=1.45;
        Afp[3][0]=1.45; Afp[3][1]=1.35; Afp[3][2]=1.25; Afp[3][3]=1.15;
        Bfp[0][0]=1.12; Bfp[0][1]=1.22; Bfp[0][2]=1.32; Bfp[0][3]=1.42;
        Bfp[1][0]=1.42; Bfp[1][1]=1.32; Bfp[1][2]=1.22; Bfp[1][3]=1.12;
        Bfp[2][0]=1.18; Bfp[2][1]=1.28; Bfp[2][2]=1.38; Bfp[2][3]=1.48;
        Bfp[3][0]=1.48; Bfp[3][1]=1.38; Bfp[3][2]=1.28; Bfp[3][3]=1.18;
        run_test(5, "near-1 tight", 8'h10, 8'h11);

        // ---------------- T6: near-1 mixed ----------------
        Afp[0][0]=1.48; Afp[0][1]=1.61; Afp[0][2]=1.19; Afp[0][3]=1.83;
        Afp[1][0]=1.83; Afp[1][1]=1.19; Afp[1][2]=1.61; Afp[1][3]=1.48;
        Afp[2][0]=1.27; Afp[2][1]=1.94; Afp[2][2]=1.05; Afp[2][3]=1.72;
        Afp[3][0]=1.72; Afp[3][1]=1.05; Afp[3][2]=1.94; Afp[3][3]=1.27;
        Bfp[0][0]=1.48; Bfp[0][1]=1.61; Bfp[0][2]=1.19; Bfp[0][3]=1.83;
        Bfp[1][0]=1.83; Bfp[1][1]=1.19; Bfp[1][2]=1.61; Bfp[1][3]=1.48;
        Bfp[2][0]=1.27; Bfp[2][1]=1.94; Bfp[2][2]=1.05; Bfp[2][3]=1.72;
        Bfp[3][0]=1.72; Bfp[3][1]=1.05; Bfp[3][2]=1.94; Bfp[3][3]=1.27;
        run_test(6, "near-1 mixed", 8'h12, 8'h13);

        // ---------------- T7: near-1 all unique ----------------
        Afp[0][0]=1.03; Afp[0][1]=1.09; Afp[0][2]=1.17; Afp[0][3]=1.23;
        Afp[1][0]=1.31; Afp[1][1]=1.37; Afp[1][2]=1.43; Afp[1][3]=1.51;
        Afp[2][0]=1.59; Afp[2][1]=1.63; Afp[2][2]=1.71; Afp[2][3]=1.79;
        Afp[3][0]=1.87; Afp[3][1]=1.91; Afp[3][2]=1.97; Afp[3][3]=1.99;
        Bfp[0][0]=1.98; Bfp[0][1]=1.92; Bfp[0][2]=1.86; Bfp[0][3]=1.78;
        Bfp[1][0]=1.72; Bfp[1][1]=1.66; Bfp[1][2]=1.58; Bfp[1][3]=1.52;
        Bfp[2][0]=1.46; Bfp[2][1]=1.38; Bfp[2][2]=1.32; Bfp[2][3]=1.26;
        Bfp[3][0]=1.18; Bfp[3][1]=1.12; Bfp[3][2]=1.06; Bfp[3][3]=1.02;
        run_test(7, "near-1 all unique", 8'h14, 8'h15);

        // ---------------- T8: near-1 interleaved ----------------
        Afp[0][0]=1.07; Afp[0][1]=1.93; Afp[0][2]=1.21; Afp[0][3]=1.77;
        Afp[1][0]=1.85; Afp[1][1]=1.13; Afp[1][2]=1.69; Afp[1][3]=1.35;
        Afp[2][0]=1.41; Afp[2][1]=1.65; Afp[2][2]=1.29; Afp[2][3]=1.89;
        Afp[3][0]=1.97; Afp[3][1]=1.25; Afp[3][2]=1.53; Afp[3][3]=1.11;
        Bfp[0][0]=1.55; Bfp[0][1]=1.33; Bfp[0][2]=1.81; Bfp[0][3]=1.15;
        Bfp[1][0]=1.27; Bfp[1][1]=1.75; Bfp[1][2]=1.39; Bfp[1][3]=1.63;
        Bfp[2][0]=1.95; Bfp[2][1]=1.17; Bfp[2][2]=1.61; Bfp[2][3]=1.45;
        Bfp[3][0]=1.09; Bfp[3][1]=1.87; Bfp[3][2]=1.23; Bfp[3][3]=1.71;
        run_test(8, "near-1 interleaved", 8'h16, 8'h17);

        // ---------------- T9: near-1 extremes ----------------
        Afp[0][0]=1.01; Afp[0][1]=1.99; Afp[0][2]=1.33; Afp[0][3]=1.67;
        Afp[1][0]=1.99; Afp[1][1]=1.01; Afp[1][2]=1.67; Afp[1][3]=1.33;
        Afp[2][0]=1.50; Afp[2][1]=1.75; Afp[2][2]=1.25; Afp[2][3]=1.95;
        Afp[3][0]=1.05; Afp[3][1]=1.45; Afp[3][2]=1.85; Afp[3][3]=1.55;
        Bfp[0][0]=1.11; Bfp[0][1]=1.44; Bfp[0][2]=1.77; Bfp[0][3]=1.22;
        Bfp[1][0]=1.88; Bfp[1][1]=1.33; Bfp[1][2]=1.66; Bfp[1][3]=1.02;
        Bfp[2][0]=1.55; Bfp[2][1]=1.99; Bfp[2][2]=1.08; Bfp[2][3]=1.42;
        Bfp[3][0]=1.31; Bfp[3][1]=1.62; Bfp[3][2]=1.93; Bfp[3][3]=1.24;
        run_test(9, "near-1 extremes", 8'h18, 8'h19);

        // ---------------- T10: mid-range mixed ----------------
        Afp[0][0]=2.50; Afp[0][1]=3.70; Afp[0][2]=1.40; Afp[0][3]=4.20;
        Afp[1][0]=4.20; Afp[1][1]=1.40; Afp[1][2]=3.70; Afp[1][3]=2.50;
        Afp[2][0]=3.10; Afp[2][1]=2.80; Afp[2][2]=4.60; Afp[2][3]=1.90;
        Afp[3][0]=1.90; Afp[3][1]=4.60; Afp[3][2]=2.80; Afp[3][3]=3.10;
        Bfp[0][0]=1.50; Bfp[0][1]=2.80; Bfp[0][2]=3.40; Bfp[0][3]=1.30;
        Bfp[1][0]=3.40; Bfp[1][1]=1.30; Bfp[1][2]=2.80; Bfp[1][3]=1.50;
        Bfp[2][0]=2.20; Bfp[2][1]=4.10; Bfp[2][2]=1.70; Bfp[2][3]=3.60;
        Bfp[3][0]=3.60; Bfp[3][1]=1.70; Bfp[3][2]=4.10; Bfp[3][3]=2.20;
        run_test(10, "mid-range mixed", 8'h20, 8'h21);

        // ---------------- T11: mid-range all unique ----------------
        Afp[0][0]=1.50; Afp[0][1]=1.85; Afp[0][2]=2.20; Afp[0][3]=2.55;
        Afp[1][0]=2.90; Afp[1][1]=3.25; Afp[1][2]=3.60; Afp[1][3]=3.95;
        Afp[2][0]=4.30; Afp[2][1]=4.65; Afp[2][2]=5.00; Afp[2][3]=5.35;
        Afp[3][0]=5.70; Afp[3][1]=6.05; Afp[3][2]=6.40; Afp[3][3]=6.75;
        Bfp[0][0]=6.00; Bfp[0][1]=5.70; Bfp[0][2]=5.40; Bfp[0][3]=5.10;
        Bfp[1][0]=4.80; Bfp[1][1]=4.50; Bfp[1][2]=4.20; Bfp[1][3]=3.90;
        Bfp[2][0]=3.60; Bfp[2][1]=3.30; Bfp[2][2]=3.00; Bfp[2][3]=2.70;
        Bfp[3][0]=2.40; Bfp[3][1]=2.10; Bfp[3][2]=1.80; Bfp[3][3]=1.50;
        run_test(11, "mid-range all unique", 8'h22, 8'h23);

        // ---------------- T12: mid-range repeating ----------------
        Afp[0][0]=3.30; Afp[0][1]=2.20; Afp[0][2]=4.40; Afp[0][3]=1.10;
        Afp[1][0]=1.10; Afp[1][1]=4.40; Afp[1][2]=2.20; Afp[1][3]=3.30;
        Afp[2][0]=5.50; Afp[2][1]=6.60; Afp[2][2]=1.10; Afp[2][3]=2.20;
        Afp[3][0]=2.20; Afp[3][1]=1.10; Afp[3][2]=6.60; Afp[3][3]=5.50;
        Bfp[0][0]=2.60; Bfp[0][1]=3.90; Bfp[0][2]=1.70; Bfp[0][3]=4.80;
        Bfp[1][0]=4.80; Bfp[1][1]=1.70; Bfp[1][2]=3.90; Bfp[1][3]=2.60;
        Bfp[2][0]=6.10; Bfp[2][1]=1.30; Bfp[2][2]=5.20; Bfp[2][3]=2.40;
        Bfp[3][0]=2.40; Bfp[3][1]=5.20; Bfp[3][2]=1.30; Bfp[3][3]=6.10;
        run_test(12, "mid-range repeating", 8'h24, 8'h25);

        // ---------------- T13: mid-range scattered ----------------
        Afp[0][0]=2.07; Afp[0][1]=5.83; Afp[0][2]=3.41; Afp[0][3]=1.62;
        Afp[1][0]=4.95; Afp[1][1]=1.28; Afp[1][2]=6.14; Afp[1][3]=2.73;
        Afp[2][0]=3.86; Afp[2][1]=6.52; Afp[2][2]=1.95; Afp[2][3]=4.31;
        Afp[3][0]=1.44; Afp[3][1]=3.09; Afp[3][2]=5.27; Afp[3][3]=6.88;
        Bfp[0][0]=5.11; Bfp[0][1]=2.46; Bfp[0][2]=6.73; Bfp[0][3]=1.85;
        Bfp[1][0]=3.52; Bfp[1][1]=6.07; Bfp[1][2]=2.19; Bfp[1][3]=4.64;
        Bfp[2][0]=1.73; Bfp[2][1]=4.28; Bfp[2][2]=5.96; Bfp[2][3]=3.35;
        Bfp[3][0]=6.41; Bfp[3][1]=1.59; Bfp[3][2]=3.82; Bfp[3][3]=5.04;
        run_test(13, "mid-range scattered", 8'h26, 8'h27);

        // ---------------- T14: larger values ----------------
        Afp[0][0]=5.50; Afp[0][1]=10.20; Afp[0][2]=3.30; Afp[0][3]=7.70;
        Afp[1][0]=7.70; Afp[1][1]=3.30; Afp[1][2]=10.20; Afp[1][3]=5.50;
        Afp[2][0]=8.90; Afp[2][1]=4.60; Afp[2][2]=6.10; Afp[2][3]=11.30;
        Afp[3][0]=11.30; Afp[3][1]=6.10; Afp[3][2]=4.60; Afp[3][3]=8.90;
        Bfp[0][0]=4.40; Bfp[0][1]=8.80; Bfp[0][2]=6.60; Bfp[0][3]=2.20;
        Bfp[1][0]=2.20; Bfp[1][1]=6.60; Bfp[1][2]=8.80; Bfp[1][3]=4.40;
        Bfp[2][0]=9.50; Bfp[2][1]=3.70; Bfp[2][2]=5.30; Bfp[2][3]=7.10;
        Bfp[3][0]=7.10; Bfp[3][1]=5.30; Bfp[3][2]=3.70; Bfp[3][3]=9.50;
        run_test(14, "larger values", 8'h30, 8'h31);

        // ---------------- T15: teens ----------------
        Afp[0][0]=12.50; Afp[0][1]=18.30; Afp[0][2]=9.70; Afp[0][3]=15.10;
        Afp[1][0]=15.10; Afp[1][1]=9.70; Afp[1][2]=18.30; Afp[1][3]=12.50;
        Afp[2][0]=11.80; Afp[2][1]=16.40; Afp[2][2]=13.90; Afp[2][3]=10.60;
        Afp[3][0]=10.60; Afp[3][1]=13.90; Afp[3][2]=16.40; Afp[3][3]=11.80;
        Bfp[0][0]=11.20; Bfp[0][1]=7.40; Bfp[0][2]=16.80; Bfp[0][3]=13.60;
        Bfp[1][0]=13.60; Bfp[1][1]=16.80; Bfp[1][2]=7.40; Bfp[1][3]=11.20;
        Bfp[2][0]=14.70; Bfp[2][1]=9.30; Bfp[2][2]=12.10; Bfp[2][3]=17.50;
        Bfp[3][0]=17.50; Bfp[3][1]=12.10; Bfp[3][2]=9.30; Bfp[3][3]=14.70;
        run_test(15, "teens", 8'h32, 8'h33);

        // ---------------- T16: large all unique ----------------
        Afp[0][0]=8.00; Afp[0][1]=9.70; Afp[0][2]=11.40; Afp[0][3]=13.10;
        Afp[1][0]=14.80; Afp[1][1]=16.50; Afp[1][2]=18.20; Afp[1][3]=19.90;
        Afp[2][0]=21.60; Afp[2][1]=23.30; Afp[2][2]=25.00; Afp[2][3]=26.70;
        Afp[3][0]=28.40; Afp[3][1]=30.10; Afp[3][2]=31.80; Afp[3][3]=33.50;
        Bfp[0][0]=30.00; Bfp[0][1]=28.70; Bfp[0][2]=27.40; Bfp[0][3]=26.10;
        Bfp[1][0]=24.80; Bfp[1][1]=23.50; Bfp[1][2]=22.20; Bfp[1][3]=20.90;
        Bfp[2][0]=19.60; Bfp[2][1]=18.30; Bfp[2][2]=17.00; Bfp[2][3]=15.70;
        Bfp[3][0]=14.40; Bfp[3][1]=13.10; Bfp[3][2]=11.80; Bfp[3][3]=10.50;
        run_test(16, "large all unique", 8'h34, 8'h35);

        // ---------------- T17: twenties thirties ----------------
        Afp[0][0]=25.40; Afp[0][1]=31.70; Afp[0][2]=19.20; Afp[0][3]=28.90;
        Afp[1][0]=28.90; Afp[1][1]=19.20; Afp[1][2]=31.70; Afp[1][3]=25.40;
        Afp[2][0]=22.60; Afp[2][1]=34.10; Afp[2][2]=26.80; Afp[2][3]=20.50;
        Afp[3][0]=20.50; Afp[3][1]=26.80; Afp[3][2]=34.10; Afp[3][3]=22.60;
        Bfp[0][0]=22.60; Bfp[0][1]=17.30; Bfp[0][2]=33.10; Bfp[0][3]=26.50;
        Bfp[1][0]=26.50; Bfp[1][1]=33.10; Bfp[1][2]=17.30; Bfp[1][3]=22.60;
        Bfp[2][0]=29.40; Bfp[2][1]=21.70; Bfp[2][2]=24.90; Bfp[2][3]=35.20;
        Bfp[3][0]=35.20; Bfp[3][1]=24.90; Bfp[3][2]=21.70; Bfp[3][3]=29.40;
        run_test(17, "twenties thirties", 8'h36, 8'h37);

        // ---------------- T18: wide range in block ----------------
        Afp[0][0]=1.20; Afp[0][1]=6.50; Afp[0][2]=2.80; Afp[0][3]=14.00;
        Afp[1][0]=14.00; Afp[1][1]=2.80; Afp[1][2]=6.50; Afp[1][3]=1.20;
        Afp[2][0]=3.60; Afp[2][1]=11.20; Afp[2][2]=1.80; Afp[2][3]=8.40;
        Afp[3][0]=8.40; Afp[3][1]=1.80; Afp[3][2]=11.20; Afp[3][3]=3.60;
        Bfp[0][0]=1.50; Bfp[0][1]=9.00; Bfp[0][2]=3.20; Bfp[0][3]=11.50;
        Bfp[1][0]=11.50; Bfp[1][1]=3.20; Bfp[1][2]=9.00; Bfp[1][3]=1.50;
        Bfp[2][0]=2.40; Bfp[2][1]=13.10; Bfp[2][2]=1.70; Bfp[2][3]=7.30;
        Bfp[3][0]=7.30; Bfp[3][1]=1.70; Bfp[3][2]=13.10; Bfp[3][3]=2.40;
        run_test(18, "wide range in block", 8'h40, 8'h41);

        // ---------------- T19: very wide range ----------------
        Afp[0][0]=1.10; Afp[0][1]=15.00; Afp[0][2]=2.30; Afp[0][3]=60.00;
        Afp[1][0]=60.00; Afp[1][1]=2.30; Afp[1][2]=15.00; Afp[1][3]=1.10;
        Afp[2][0]=4.70; Afp[2][1]=28.00; Afp[2][2]=1.60; Afp[2][3]=42.00;
        Afp[3][0]=42.00; Afp[3][1]=1.60; Afp[3][2]=28.00; Afp[3][3]=4.70;
        Bfp[0][0]=1.30; Bfp[0][1]=12.00; Bfp[0][2]=4.50; Bfp[0][3]=45.00;
        Bfp[1][0]=45.00; Bfp[1][1]=4.50; Bfp[1][2]=12.00; Bfp[1][3]=1.30;
        Bfp[2][0]=3.20; Bfp[2][1]=22.00; Bfp[2][2]=1.90; Bfp[2][3]=38.00;
        Bfp[3][0]=38.00; Bfp[3][1]=1.90; Bfp[3][2]=22.00; Bfp[3][3]=3.20;
        run_test(19, "very wide range", 8'h42, 8'h43);

        // ---------------- T20: extreme spread unique ----------------
        Afp[0][0]=1.05; Afp[0][1]=1.50; Afp[0][2]=2.85; Afp[0][3]=5.10;
        Afp[1][0]=8.25; Afp[1][1]=12.30; Afp[1][2]=17.25; Afp[1][3]=23.10;
        Afp[2][0]=29.85; Afp[2][1]=37.50; Afp[2][2]=46.05; Afp[2][3]=55.50;
        Afp[3][0]=65.85; Afp[3][1]=77.10; Afp[3][2]=89.25; Afp[3][3]=102.30;
        Bfp[0][0]=91.10; Bfp[0][1]=79.50; Bfp[0][2]=68.70; Bfp[0][3]=58.70;
        Bfp[1][0]=49.50; Bfp[1][1]=41.10; Bfp[1][2]=33.50; Bfp[1][3]=26.70;
        Bfp[2][0]=20.70; Bfp[2][1]=15.50; Bfp[2][2]=11.10; Bfp[2][3]=7.50;
        Bfp[3][0]=4.70; Bfp[3][1]=2.70; Bfp[3][2]=1.50; Bfp[3][3]=1.10;
        run_test(20, "extreme spread unique", 8'h44, 8'h45);

        $display("");
        $display("================= SUMMARY =================");
        $display("  tests run     : %0d", ntests);
        $display("  tests failed  : %0d   (10%% tolerance)", nfail);
        $display("  mean accuracy : %.2f%%", acc_sum/ntests);
        $display("  RESULT        : %0s", (nfail==0)?"ALL PASSED":"SOME FAILED");
        $display("===========================================");
        $finish;
    end

    initial begin $dumpfile("TOP_1_tb.vcd"); $dumpvars(0, tb_TOP_1); end
    initial begin #4000000; $display("WATCHDOG"); $finish; end
endmodule
