# Timing constraint for synthesizing bfp_accel_axi_top (the AXI wrapper) as top: 100 MHz.
create_clock -period 10.000 -name clk [get_ports aclk]
