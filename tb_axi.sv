`timescale 1ns/1ps
//=============================================================================
// tb_axi.sv -- smoke test of bfp_accel_axi (AXI-Lite + AXI-Stream plumbing)
//
// Default parameters only. Two tiles, back to back:
//   tile 1: A = B = all ones -> C = 4 everywhere -> output E = 2, L = 0, scale 0
//   tile 2: A = B = all twos -> C = 16 everywhere -> output E = 4
// Tile 2 also proves the per-tile core reset: without it the accumulators
// would still hold tile 1's sums.
//=============================================================================
module tb_axi;
    localparam N = 4, BMW = 6, SFW = 4, EW = 8;
    localparam MBW = N*BMW;
    localparam ZERO_LSB = MBW, SIGN_LSB = ZERO_LSB + N, SCALE_LSB = SIGN_LSB + N, EXP_LSB = SCALE_LSB + SFW;
    localparam BW = (((MBW + 2*N + SFW + EW) + 7)/8)*8;   // 48
    localparam AW = 64;

    reg clk = 0, rstn = 0;
    always #5 clk = ~clk;

    reg  [7:0]  awaddr = 0, araddr = 0;
    reg         awvalid = 0, wvalid = 0, bready = 0, arvalid = 0, rready = 0;
    reg  [31:0] wdata = 0;
    wire        awready, wready, bvalid, arready, rvalid;
    wire [1:0]  bresp, rresp;
    wire [31:0] rdata;

    reg  [AW-1:0] s_tdata = 0;
    reg           s_tvalid = 0, s_tlast = 0;
    wire          s_tready;
    wire [AW-1:0] m_tdata;
    wire [AW/8-1:0] m_tkeep;
    wire          m_tvalid, m_tlast;
    reg           m_tready = 0;
    wire          irq;

    bfp_accel_axi dut (
        .aclk(clk), .aresetn(rstn),
        .s_axi_awaddr(awaddr), .s_axi_awprot(3'd0), .s_axi_awvalid(awvalid), .s_axi_awready(awready),
        .s_axi_wdata(wdata), .s_axi_wstrb(4'hF), .s_axi_wvalid(wvalid), .s_axi_wready(wready),
        .s_axi_bresp(bresp), .s_axi_bvalid(bvalid), .s_axi_bready(bready),
        .s_axi_araddr(araddr), .s_axi_arprot(3'd0), .s_axi_arvalid(arvalid), .s_axi_arready(arready),
        .s_axi_rdata(rdata), .s_axi_rresp(rresp), .s_axi_rvalid(rvalid), .s_axi_rready(rready),
        .s_axis_tdata(s_tdata), .s_axis_tvalid(s_tvalid), .s_axis_tready(s_tready), .s_axis_tlast(s_tlast),
        .m_axis_tdata(m_tdata), .m_axis_tkeep(m_tkeep), .m_axis_tvalid(m_tvalid),
        .m_axis_tready(m_tready), .m_axis_tlast(m_tlast),
        .irq(irq)
    );

    integer errors = 0;

    task axil_write(input [7:0] a, input [31:0] d);
        begin
            @(negedge clk); awaddr = a; wdata = d; awvalid = 1; wvalid = 1; bready = 1;
            while (!(awready && wready)) @(negedge clk);
            @(negedge clk); awvalid = 0; wvalid = 0;
            while (!bvalid) @(negedge clk);
            @(negedge clk); bready = 0;
        end
    endtask

    task axil_read(input [7:0] a, output [31:0] d);
        begin
            @(negedge clk); araddr = a; arvalid = 1; rready = 1;
            while (!arready) @(negedge clk);
            @(negedge clk); arvalid = 0;
            while (!rvalid) @(negedge clk);
            d = rdata;
            @(negedge clk); rready = 0;
        end
    endtask

    // every block of the tile is the same: all elements = 2^e  (L = 0, scale 0)
    task send_tile(input integer e);
        integer bt;
        reg [BW-1:0] blk;
        begin
            blk = '0;
            blk[EXP_LSB +: EW] = e[EW-1:0];
            for (bt = 0; bt < 2*N; bt = bt + 1) begin
                @(negedge clk); s_tdata = AW'(blk); s_tvalid = 1; s_tlast = (bt == 2*N-1);
                @(posedge clk); while (!s_tready) @(posedge clk);
            end
            @(negedge clk); s_tvalid = 0; s_tlast = 0;
        end
    endtask

    task run_tile(input integer name_e, input integer exp_out_e);
        reg [31:0] st;
        integer got, tmo;
        reg [BW-1:0] want;
        begin
            send_tile(name_e);
            axil_read(8'h04, st);
            if (!st[2]) begin $display("  FAIL: IN_READY not set (STATUS=%h)", st); errors = errors + 1; end
            axil_write(8'h00, 32'h1);                         // START
            tmo = 0; st = 0;
            while (!st[1] && !st[3] && tmo < 200) begin axil_read(8'h04, st); tmo = tmo + 1; end
            if (!st[1]) begin $display("  FAIL: no DONE (STATUS=%h)", st); errors = errors + 1; end
            if (st[3])  begin $display("  FAIL: TIMEOUT"); errors = errors + 1; end
            if (st[7:4] != 0) begin $display("  FAIL: unexpected flags STATUS=%h", st); errors = errors + 1; end
            want = '0; want[EXP_LSB +: EW] = exp_out_e[EW-1:0];
            // receive N output blocks
            m_tready = 1; got = 0;
            while (got < N) begin
                @(posedge clk);
                if (m_tvalid && m_tready) begin
                    if (m_tdata[BW-1:0] !== want || m_tdata[AW-1:BW] !== '0) begin
                        $display("  FAIL: out beat %0d = %h, want %h", got, m_tdata, AW'(want)); errors = errors + 1;
                    end
                    if (m_tlast != (got == N-1)) begin $display("  FAIL: TLAST on beat %0d", got); errors = errors + 1; end
                    got = got + 1;
                end
            end
            @(negedge clk); m_tready = 0;
            axil_read(8'h20, st);
            $display("  tile inputs 2^%0d: output E = %0d expected, %0d cycles, STATUS ok", name_e, exp_out_e, st);
        end
    endtask

    reg [31:0] r;
    initial begin
        repeat (5) @(posedge clk); rstn = 1; repeat (3) @(posedge clk);
        axil_read(8'h30, r); $display("  ID   = %h", r); if (r !== 32'hBF5A0001) begin $display("  FAIL: ID"); errors = errors + 1; end
        axil_read(8'h34, r); $display("  CFG0 = %h  (scale %0d, E %0d, L %0d, N %0d)", r, r[31:24], r[23:16], r[15:8], r[7:0]);
        axil_read(8'h38, r); $display("  CFG1 = %h  (block %0d, frac %0d, mant %0d, data %0d)", r, r[31:24], r[23:16], r[15:8], r[7:0]);
        run_tile(0, 2);      // ones: C = 4  = 2^2
        run_tile(1, 4);      // twos: C = 16 = 2^4 (needs the per-tile reset)
        $display("  RESULT: %0s (%0d errors)", (errors == 0) ? "PASS" : "FAIL", errors);
        $finish;
    end
    initial begin #2000000; $display("  WATCHDOG"); $finish; end
endmodule
