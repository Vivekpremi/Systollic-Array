`timescale 1ns/1ps
//=============================================================================
// bfp_accel_axi_top.v -- plain-Verilog top for Vivado block designs
//
// Vivado's "Add Module" (module reference) needs a Verilog or VHDL top file.
// This file only re-exposes bfp_accel_axi (SystemVerilog) with flat ports and
// the interface attributes Vivado uses to infer:
//   s_axi   -> AXI4-Lite slave      s_axis -> AXI4-Stream slave
//   m_axis  -> AXI4-Stream master   aclk / aresetn / irq
// No logic: every port and parameter goes straight through.
// This is the file to "Add Module" in the block design. Keep PE.sv as a
// Verilog Header in the project (systolic_array.sv includes it).
// The AXIS_WIDTH formula must match bfp_accel_axi.sv.
//=============================================================================
module bfp_accel_axi_top #(
    parameter MANTISSA_WIDTH       = 16,
    parameter FRAC_WIDTH           = 8,
    parameter EXP_WIDTH            = 8,
    parameter DATA_WIDTH           = 32,
    parameter ARRAY_SIZE           = 4,
    parameter BLOCK_MANTISSA_WIDTH = 6,
    parameter SCALE_FACTOR_WIDTH   = 4,
    parameter BLOCK_BITS  = ARRAY_SIZE*BLOCK_MANTISSA_WIDTH + 2*ARRAY_SIZE + SCALE_FACTOR_WIDTH + EXP_WIDTH,
    parameter BLOCK_WIDTH = ((BLOCK_BITS + 7)/8)*8,
    parameter AXIS_WIDTH  = (BLOCK_WIDTH + 2 <=  32) ?  32 :
                            (BLOCK_WIDTH + 2 <=  64) ?  64 :
                            (BLOCK_WIDTH + 2 <= 128) ? 128 :
                            (BLOCK_WIDTH + 2 <= 256) ? 256 : 512
)(
    (* X_INTERFACE_INFO = "xilinx.com:signal:clock:1.0 aclk CLK" *)
    (* X_INTERFACE_PARAMETER = "ASSOCIATED_BUSIF s_axi:s_axis:m_axis, ASSOCIATED_RESET aresetn" *)
    input  wire                    aclk,
    (* X_INTERFACE_INFO = "xilinx.com:signal:reset:1.0 aresetn RST" *)
    (* X_INTERFACE_PARAMETER = "POLARITY ACTIVE_LOW" *)
    input  wire                    aresetn,

    input  wire [7:0]              s_axi_awaddr,
    input  wire [2:0]              s_axi_awprot,
    input  wire                    s_axi_awvalid,
    output wire                    s_axi_awready,
    input  wire [31:0]             s_axi_wdata,
    input  wire [3:0]              s_axi_wstrb,
    input  wire                    s_axi_wvalid,
    output wire                    s_axi_wready,
    output wire [1:0]              s_axi_bresp,
    output wire                    s_axi_bvalid,
    input  wire                    s_axi_bready,
    input  wire [7:0]              s_axi_araddr,
    input  wire [2:0]              s_axi_arprot,
    input  wire                    s_axi_arvalid,
    output wire                    s_axi_arready,
    output wire [31:0]             s_axi_rdata,
    output wire [1:0]              s_axi_rresp,
    output wire                    s_axi_rvalid,
    input  wire                    s_axi_rready,

    input  wire [AXIS_WIDTH-1:0]   s_axis_tdata,
    input  wire                    s_axis_tvalid,
    output wire                    s_axis_tready,
    input  wire                    s_axis_tlast,

    output wire [AXIS_WIDTH-1:0]   m_axis_tdata,
    output wire [AXIS_WIDTH/8-1:0] m_axis_tkeep,
    output wire                    m_axis_tvalid,
    input  wire                    m_axis_tready,
    output wire                    m_axis_tlast,

    (* X_INTERFACE_INFO = "xilinx.com:signal:interrupt:1.0 irq INTERRUPT" *)
    (* X_INTERFACE_PARAMETER = "SENSITIVITY LEVEL_HIGH" *)
    output wire                    irq
);

    bfp_accel_axi #(
        .MANTISSA_WIDTH(MANTISSA_WIDTH), .FRAC_WIDTH(FRAC_WIDTH), .EXP_WIDTH(EXP_WIDTH),
        .DATA_WIDTH(DATA_WIDTH), .ARRAY_SIZE(ARRAY_SIZE),
        .BLOCK_MANTISSA_WIDTH(BLOCK_MANTISSA_WIDTH), .SCALE_FACTOR_WIDTH(SCALE_FACTOR_WIDTH)
    ) u_accel (
        .aclk(aclk), .aresetn(aresetn),
        .s_axi_awaddr(s_axi_awaddr), .s_axi_awprot(s_axi_awprot), .s_axi_awvalid(s_axi_awvalid),
        .s_axi_awready(s_axi_awready), .s_axi_wdata(s_axi_wdata), .s_axi_wstrb(s_axi_wstrb),
        .s_axi_wvalid(s_axi_wvalid), .s_axi_wready(s_axi_wready), .s_axi_bresp(s_axi_bresp),
        .s_axi_bvalid(s_axi_bvalid), .s_axi_bready(s_axi_bready), .s_axi_araddr(s_axi_araddr),
        .s_axi_arprot(s_axi_arprot), .s_axi_arvalid(s_axi_arvalid), .s_axi_arready(s_axi_arready),
        .s_axi_rdata(s_axi_rdata), .s_axi_rresp(s_axi_rresp), .s_axi_rvalid(s_axi_rvalid),
        .s_axi_rready(s_axi_rready),
        .s_axis_tdata(s_axis_tdata), .s_axis_tvalid(s_axis_tvalid), .s_axis_tready(s_axis_tready),
        .s_axis_tlast(s_axis_tlast),
        .m_axis_tdata(m_axis_tdata), .m_axis_tkeep(m_axis_tkeep), .m_axis_tvalid(m_axis_tvalid),
        .m_axis_tready(m_axis_tready), .m_axis_tlast(m_axis_tlast),
        .irq(irq)
    );

endmodule
