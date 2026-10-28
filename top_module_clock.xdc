# Timing constraint for synthesizing top_module on its own (100 MHz).
# Add as a constraints file: Add Sources > Add or create constraints.
create_clock -period 10.000 -name clk [get_ports clk]
