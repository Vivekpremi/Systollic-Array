`timescale 1ns/1ps
//=============================================================================
// bfp_accel_axi.sv  --  AXI wrapper around top_module (the accelerator core)
//
// WHAT IT DOES
//   Puts the core behind standard AXI interfaces so the ARM (software) and an
//   AXI DMA can use it. Full register map and stream format:
//   docs/axi_interface.md.
//     s_axi_*  : AXI4-Lite slave, 32-bit   - registers: START, STATUS, flags, config
//     s_axis_* : AXI4-Stream in            - 2*ARRAY_SIZE input blocks per tile
//                                            (row blocks 0..N-1, then column blocks 0..N-1)
//     m_axis_* : AXI4-Stream out           - ARRAY_SIZE output blocks per tile
//                                            (block j = column j of C), 2 flag bits each
//     irq      : level, high when a tile finished (DONE or TIMEOUT) and IRQ_EN[0] = 1
//
// ONE TILE  (FSM: S_IDLE -> S_RST -> S_LOAD -> S_RUN -> S_SEND -> S_IDLE)
//   S_IDLE : core held in reset; input beats are stored in blk_in[]. After the
//            8th beat (with TLAST) IN_READY = 1. Wait for START.
//   S_RST  : 2 clocks of core reset (clears every PE accumulator)
//   S_LOAD : all 2*ARRAY_SIZE blocks go into the core in ONE clock
//   S_RUN  : each core block_ready stores one output block (+ its E flags) in
//            blk_out[]; after the last one DONE = 1. If the tile takes
//            TIMEOUT_LIM cycles, TIMEOUT = 1 and back to S_IDLE.
//   S_SEND : the stored blocks go out on m_axis, TLAST on the last
//   A tile takes 109 clocks from START to DONE at the default parameters.
//
// STREAM BEATS (AXIS_WIDTH bits, 64 at the defaults)
//   in : [BLOCK_WIDTH-1:0] = one block, upper bits ignored. Exactly 2*N beats,
//        TLAST on the last; a TLAST too early or missing -> IN_LEN_ERR and the
//        input is dropped.
//   out: [BLOCK_WIDTH-1:0] = one output block, [BLOCK_WIDTH] = its
//        exp_overflow, [BLOCK_WIDTH+1] = its exp_underflow, rest 0.
//
// LIMITATIONS / NOTES
//   - One tile at a time: no input is accepted while a tile runs or its
//     results wait to be sent (TREADY low); the DMA simply waits.
//   - ABORT is ignored in S_SEND: an AXI-Stream packet cannot be taken back,
//     so the output beats always complete with TLAST.
//   - There is no timeout in S_SEND: if the DMA never takes the results, the
//     wrapper waits (only aresetn gets it out).
//   - WSTRB is ignored: every register write writes the whole 32-bit word.
//   - Only the low 8 address bits are decoded (registers repeat every 256 bytes).
//=============================================================================
module bfp_accel_axi #(
    parameter MANTISSA_WIDTH       = 16,
    parameter FRAC_WIDTH           = 8,
    parameter EXP_WIDTH            = 8,
    parameter DATA_WIDTH           = 32,
    parameter ARRAY_SIZE           = 4,
    parameter BLOCK_MANTISSA_WIDTH = 6,
    parameter SCALE_FACTOR_WIDTH   = 4,
    parameter AXIL_ADDR_WIDTH      = 8,      // AXI-Lite address bits decoded
    // block geometry, identical to top_module / edge_reg_bank
    parameter MANT_BUS_WIDTH = ARRAY_SIZE*BLOCK_MANTISSA_WIDTH,
    parameter BLOCK_BITS     = MANT_BUS_WIDTH + 2*ARRAY_SIZE + SCALE_FACTOR_WIDTH + EXP_WIDTH,
    parameter BLOCK_WIDTH    = ((BLOCK_BITS + 7)/8)*8,
    // stream width: smallest AXI DMA width (32..512) that holds a block + 2
    // flag bits. 64 at the defaults (48-bit block).
    parameter AXIS_WIDTH     = (BLOCK_WIDTH + 2 <=  32) ?  32 :
                               (BLOCK_WIDTH + 2 <=  64) ?  64 :
                               (BLOCK_WIDTH + 2 <= 128) ? 128 :
                               (BLOCK_WIDTH + 2 <= 256) ? 256 : 512
)(
    input  wire                         aclk,      // one clock for everything (core included)
    input  wire                         aresetn,   // active low, synchronous for the wrapper; also resets the core

    // ---- AXI4-Lite slave (registers) ----
    input  wire [AXIL_ADDR_WIDTH-1:0]   s_axi_awaddr,
    input  wire [2:0]                   s_axi_awprot,
    input  wire                         s_axi_awvalid,
    output wire                         s_axi_awready,
    input  wire [31:0]                  s_axi_wdata,
    input  wire [3:0]                   s_axi_wstrb,
    input  wire                         s_axi_wvalid,
    output wire                         s_axi_wready,
    output wire [1:0]                   s_axi_bresp,
    output wire                         s_axi_bvalid,
    input  wire                         s_axi_bready,
    input  wire [AXIL_ADDR_WIDTH-1:0]   s_axi_araddr,
    input  wire [2:0]                   s_axi_arprot,
    input  wire                         s_axi_arvalid,
    output wire                         s_axi_arready,
    output wire [31:0]                  s_axi_rdata,
    output wire [1:0]                   s_axi_rresp,
    output wire                         s_axi_rvalid,
    input  wire                         s_axi_rready,

    // ---- AXI4-Stream in: input blocks (from the DMA, MM2S) ----
    input  wire [AXIS_WIDTH-1:0]        s_axis_tdata,
    input  wire                         s_axis_tvalid,
    output wire                         s_axis_tready,
    input  wire                         s_axis_tlast,

    // ---- AXI4-Stream out: output blocks (to the DMA, S2MM) ----
    output wire [AXIS_WIDTH-1:0]        m_axis_tdata,
    output wire [AXIS_WIDTH/8-1:0]      m_axis_tkeep,
    output wire                         m_axis_tvalid,
    input  wire                         m_axis_tready,
    output wire                         m_axis_tlast,

    output wire                         irq        // tile finished (DONE or TIMEOUT), if IRQ_EN[0]
);

    localparam N     = ARRAY_SIZE;               // output blocks per tile
    localparam NIN   = 2*N;                      // input blocks per tile (N rows + N columns)
    localparam CW    = $clog2(NIN) + 1;          // width of the beat counters (counts up to NIN)
    localparam [31:0] ID_VALUE = 32'hBF5A0001;   // read-only ID register: "BF5A" + version 0001

    // ======================================================================
    // registers
    // ======================================================================
    localparam [2:0] S_IDLE = 3'd0, S_RST = 3'd1, S_LOAD = 3'd2, S_RUN = 3'd3, S_SEND = 3'd4;
    reg  [2:0]             state;
    reg  [1:0]             rst_cnt;               // clocks spent in S_RST
    reg                    core_rst_req;          // 1 = hold the core in reset
    reg  [BLOCK_WIDTH-1:0] blk_in  [0:NIN-1];     // staged input blocks: 0..N-1 rows of A, N..2N-1 columns of B
    reg  [CW-1:0]          in_cnt;                // input beats received so far
    reg                    in_ready, in_len_err;  // STATUS.IN_READY (all 2N blocks staged) / IN_LEN_ERR (bad packet length)
    reg  [BLOCK_WIDTH-1:0] blk_out [0:N-1];       // output blocks captured from the core
    reg  [N-1:0]           out_ovf, out_unf;      // exp_overflow / exp_underflow of each output block
    reg  [CW-1:0]          out_cnt;               // output blocks captured (S_RUN) / sent (S_SEND)
    reg                    done, timeout;         // STATUS.DONE / STATUS.TIMEOUT
    reg  [31:0]            cycles, timeout_lim;   // CYCLES (clocks since START) / TIMEOUT_LIM (default 4096)
    reg                    irq_en, irq_pend;      // IRQ_EN register / pending interrupt
    reg  [63:0]            pe_ovf_q;              // core pe_overflow at DONE (PE_OVF_LO/HI)

    // ======================================================================
    // core
    // ======================================================================
    // the core is reset by the system reset OR by the wrapper (idle, S_RST, abort, timeout)
    wire core_rst_n = aresetn & ~core_rst_req;

    wire                     row_we    [N-1:0];
    wire                     col_we    [N-1:0];
    wire [BLOCK_WIDTH-1:0]   row_block [N-1:0];
    wire [BLOCK_WIDTH-1:0]   col_block [N-1:0];
    wire                     load_now = (state == S_LOAD);

    genvar g;
    generate
        for (g = 0; g < N; g = g + 1) begin : lanes
            assign row_we[g]    = load_now;
            assign col_we[g]    = load_now;
            assign row_block[g] = blk_in[g];
            assign col_block[g] = blk_in[N + g];
        end
    endgenerate

    wire [BLOCK_WIDTH-1:0]   core_block_out;
    wire                     core_block_ready;
    wire                     core_exp_ovf, core_exp_unf;
    wire [N*N-1:0]           core_pe_ovf;
    wire                     core_acc_ovf;

    top_module #(
        .MANTISSA_WIDTH(MANTISSA_WIDTH), .FRAC_WIDTH(FRAC_WIDTH), .EXP_WIDTH(EXP_WIDTH),
        .DATA_WIDTH(DATA_WIDTH), .ARRAY_SIZE(ARRAY_SIZE),
        .BLOCK_MANTISSA_WIDTH(BLOCK_MANTISSA_WIDTH), .SCALE_FACTOR_WIDTH(SCALE_FACTOR_WIDTH)
    ) core (
        .clk(aclk), .rst_n(core_rst_n),
        .row_we(row_we), .col_we(col_we), .row_block(row_block), .col_block(col_block),
        .block_out(core_block_out), .block_ready(core_block_ready),
        .exp_overflow(core_exp_ovf), .exp_underflow(core_exp_unf),
        .pe_overflow(core_pe_ovf), .acc_overflow(core_acc_ovf)
        // debug / observability outputs are left open on purpose
    );

    // ======================================================================
    // AXI4-Lite: write channel (address and data may arrive in any order)
    // ======================================================================
    reg                       aw_have, w_have, bvalid_q;
    reg [AXIL_ADDR_WIDTH-1:0] aw_addr;
    reg [31:0]                w_data;
    assign s_axi_awready = ~aw_have & ~bvalid_q;
    assign s_axi_wready  = ~w_have  & ~bvalid_q;
    assign s_axi_bvalid  = bvalid_q;
    assign s_axi_bresp   = 2'b00;
    wire   wr_fire = aw_have & w_have & ~bvalid_q;
    wire [7:0] wr_off = 8'(aw_addr);

    // decoded write strobes (one cycle)
    wire wr_ctrl    = wr_fire && (wr_off[7:2] == 6'h00);
    wire wr_irq_en  = wr_fire && (wr_off[7:2] == 6'h02);
    wire wr_irq_st  = wr_fire && (wr_off[7:2] == 6'h03);
    wire wr_tmo     = wr_fire && (wr_off[7:2] == 6'h09);
    wire start_cmd  = wr_ctrl & w_data[0];
    wire abort_cmd  = wr_ctrl & w_data[1];

    always @(posedge aclk) begin
        if (!aresetn) begin
            aw_have <= 1'b0; w_have <= 1'b0; bvalid_q <= 1'b0;
            aw_addr <= '0;   w_data <= '0;
        end else begin
            if (s_axi_awvalid && s_axi_awready) begin aw_have <= 1'b1; aw_addr <= s_axi_awaddr; end
            if (s_axi_wvalid  && s_axi_wready)  begin w_have  <= 1'b1; w_data  <= s_axi_wdata;  end
            if (wr_fire) begin aw_have <= 1'b0; w_have <= 1'b0; bvalid_q <= 1'b1; end
            if (bvalid_q && s_axi_bready) bvalid_q <= 1'b0;
        end
    end

    // ======================================================================
    // AXI4-Lite: read channel
    // ======================================================================
    reg        rvalid_q;
    reg [31:0] rdata_q;
    assign s_axi_arready = ~rvalid_q;
    assign s_axi_rvalid  = rvalid_q;
    assign s_axi_rdata   = rdata_q;
    assign s_axi_rresp   = 2'b00;

    wire [31:0] status_w = {24'd0, in_len_err, (|pe_ovf_q), (|out_unf), (|out_ovf),
                            timeout, in_ready, done, (state != S_IDLE)};
    wire [31:0] cfg0_w = {8'(SCALE_FACTOR_WIDTH), 8'(EXP_WIDTH), 8'(BLOCK_MANTISSA_WIDTH), 8'(ARRAY_SIZE)};
    wire [31:0] cfg1_w = {8'(BLOCK_WIDTH), 8'(FRAC_WIDTH), 8'(MANTISSA_WIDTH), 8'(DATA_WIDTH)};

    reg [31:0] rd_mux;
    wire [7:0] rd_off = 8'(s_axi_araddr);
    always @(*) begin
        case (rd_off[7:2])
            6'h01:   rd_mux = status_w;
            6'h02:   rd_mux = {31'd0, irq_en};
            6'h03:   rd_mux = {31'd0, irq_pend};
            6'h04:   rd_mux = pe_ovf_q[31:0];
            6'h05:   rd_mux = pe_ovf_q[63:32];
            6'h06:   rd_mux = 32'(out_ovf);
            6'h07:   rd_mux = 32'(out_unf);
            6'h08:   rd_mux = cycles;
            6'h09:   rd_mux = timeout_lim;
            6'h0C:   rd_mux = ID_VALUE;
            6'h0D:   rd_mux = cfg0_w;
            6'h0E:   rd_mux = cfg1_w;
            default: rd_mux = 32'd0;
        endcase
    end

    always @(posedge aclk) begin
        if (!aresetn) begin
            rvalid_q <= 1'b0; rdata_q <= '0;
        end else begin
            if (s_axi_arvalid && s_axi_arready) begin rvalid_q <= 1'b1; rdata_q <= rd_mux; end
            else if (rvalid_q && s_axi_rready)  rvalid_q <= 1'b0;
        end
    end

    // ======================================================================
    // input stream: stage 2*N blocks while idle
    // ======================================================================
    assign s_axis_tready = (state == S_IDLE) && !in_ready;
    wire in_fire = s_axis_tvalid && s_axis_tready;

    // ======================================================================
    // output stream: send the N captured blocks with their flags
    // ======================================================================
    wire [CW-1:0] send_idx = out_cnt;
    wire [AXIS_WIDTH-1:0] send_beat =
        AXIS_WIDTH'({ out_unf[send_idx[$clog2(N)-1:0]], out_ovf[send_idx[$clog2(N)-1:0]],
                      blk_out[send_idx[$clog2(N)-1:0]] });
    assign m_axis_tvalid = (state == S_SEND);
    assign m_axis_tdata  = send_beat;
    assign m_axis_tkeep  = {(AXIS_WIDTH/8){1'b1}};
    assign m_axis_tlast  = (state == S_SEND) && (out_cnt == CW'(N-1));
    wire out_fire = m_axis_tvalid && m_axis_tready;

    // ======================================================================
    // control FSM
    // ======================================================================
    integer b;
    always @(posedge aclk) begin
        if (!aresetn) begin
            state <= S_IDLE; rst_cnt <= '0; core_rst_req <= 1'b1;
            in_cnt <= '0; in_ready <= 1'b0; in_len_err <= 1'b0;
            out_cnt <= '0; out_ovf <= '0; out_unf <= '0;
            done <= 1'b0; timeout <= 1'b0; cycles <= '0; timeout_lim <= 32'd4096;
            irq_en <= 1'b0; irq_pend <= 1'b0; pe_ovf_q <= '0;
            for (b = 0; b < NIN; b = b + 1) blk_in[b] <= '0;
            for (b = 0; b < N;   b = b + 1) blk_out[b] <= '0;
        end else begin
            // ---- register writes ----
            if (wr_irq_en) irq_en <= w_data[0];
            if (wr_irq_st && w_data[0]) irq_pend <= 1'b0;
            if (wr_tmo)    timeout_lim <= w_data;

            // ABORT cancels a tile that has not finished (IDLE with staged
            // input, RST, LOAD, RUN). It is IGNORED while the results are being
            // sent (S_SEND): an AXI-Stream packet cannot be taken back or cut
            // short, so the 4 output beats always finish with TLAST and the
            // DMA receiving them is never left waiting.
            if (abort_cmd && state != S_SEND) begin
                state <= S_IDLE; core_rst_req <= 1'b1;
                in_cnt <= '0; in_ready <= 1'b0; in_len_err <= 1'b0; out_cnt <= '0;
            end else begin
                case (state)
                S_IDLE: begin
                    // core stays in reset while idle: accumulators are clear
                    core_rst_req <= 1'b1;
                    if (in_fire) begin
                        blk_in[in_cnt[CW-2:0]] <= s_axis_tdata[BLOCK_WIDTH-1:0];
                        if (in_cnt == CW'(NIN-1)) begin
                            in_cnt <= '0;
                            if (s_axis_tlast) begin in_ready <= 1'b1; in_len_err <= 1'b0; end
                            else               in_len_err <= 1'b1;   // no TLAST: discard
                        end else if (s_axis_tlast) begin
                            in_cnt <= '0; in_len_err <= 1'b1;        // early TLAST: discard
                        end else begin
                            in_cnt <= in_cnt + 1'b1;
                        end
                    end
                    if (start_cmd && in_ready) begin
                        done <= 1'b0; timeout <= 1'b0; cycles <= '0;
                        out_ovf <= '0; out_unf <= '0; out_cnt <= '0; pe_ovf_q <= '0;
                        rst_cnt <= '0;
                        state <= S_RST;
                    end
                end
                S_RST: begin                      // two cycles of core reset
                    core_rst_req <= 1'b1;
                    cycles  <= cycles + 1;
                    rst_cnt <= rst_cnt + 1'b1;
                    if (rst_cnt == 2'd1) begin core_rst_req <= 1'b0; state <= S_LOAD; end
                end
                S_LOAD: begin                     // row_we/col_we high this cycle
                    cycles <= cycles + 1;
                    in_ready <= 1'b0;
                    state <= S_RUN;
                end
                S_RUN: begin
                    cycles <= cycles + 1;
                    if (core_block_ready && (out_cnt < CW'(N))) begin
                        blk_out[out_cnt[$clog2(N)-1:0]] <= core_block_out;
                        out_ovf[out_cnt[$clog2(N)-1:0]] <= core_exp_ovf;
                        out_unf[out_cnt[$clog2(N)-1:0]] <= core_exp_unf;
                        out_cnt <= out_cnt + 1'b1;
                        if (out_cnt == CW'(N-1)) begin
                            pe_ovf_q <= 64'(core_pe_ovf);
                            done     <= 1'b1;
                            if (irq_en) irq_pend <= 1'b1;
                            out_cnt  <= '0;
                            state    <= S_SEND;
                        end
                    end else if (cycles >= timeout_lim) begin
                        timeout <= 1'b1;
                        if (irq_en) irq_pend <= 1'b1;
                        core_rst_req <= 1'b1;
                        state <= S_IDLE;
                    end
                end
                S_SEND: begin
                    if (out_fire) begin
                        if (out_cnt == CW'(N-1)) begin out_cnt <= '0; state <= S_IDLE; end
                        else                      out_cnt <= out_cnt + 1'b1;
                    end
                end
                default: state <= S_IDLE;
                endcase
            end
        end
    end

    assign irq = irq_pend;

endmodule
