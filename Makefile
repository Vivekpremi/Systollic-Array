# =============================================================================
# Systolic Array (BFP / LNS) -- build & simulation
#
#   make            build + run the main testbench (tb_TOP_1)
#   make wave       build + run, then open the VCD in gtkwave
#   make lint       lint the RTL only (no simulation)
#   make clean      remove build artifacts and waveforms
#
# Verilator auto-discovers modules from files in this directory (hence
# -Wno-MULTITOP), so only the top-level sources need listing. PE.sv is pulled
# in by `include from systolic_array.sv -- it must exist but is never listed.
# It IS listed as a prerequisite below so that editing it forces a rebuild.
# =============================================================================

# ---- tools ----
VERILATOR ?= verilator
GTKWAVE   ?= gtkwave

# ---- warning suppressions (as used on the command line) ----
WAIVERS := -Wall \
           -Wno-UNUSED \
           -Wno-UNDRIVEN \
           -Wno-PINMISSING \
           -Wno-UNUSEDSIGNAL \
           -Wno-BLKSEQ \
           -Wno-CMPCONST \
           -Wno-UNSIGNED \
           -Wno-ASCRANGE \
           -Wno-DECLFILENAME \
           -Wno-MULTITOP

# ---- sources ----
# Listed on the command line:
SRCS := tb_TOP_1.sv top_module.sv submodules_FC.sv

# Found automatically by Verilator, but tracked here so edits trigger a rebuild:
DEPS := PE.sv systolic_array.sv iface.sv format_convertor.sv exponential_CORDIC.sv

# RTL only (for lint; excludes the testbench).
# PE.sv is deliberately NOT listed -- systolic_array.sv `includes it, so listing
# both would define module PE twice (MODDUP error).
RTL := top_module.sv submodules_FC.sv systolic_array.sv iface.sv \
       format_convertor.sv exponential_CORDIC.sv

TOP  := tb_TOP_1
BIN  := obj_dir/V$(TOP)
VCD  := TOP_1_tb.vcd

.PHONY: all run wave lint clean help

all: run

# ---- build ----
$(BIN): $(SRCS) $(DEPS)
	$(VERILATOR) $(WAIVERS) --trace --binary $(SRCS)

# ---- build + run ----
run: $(BIN)
	./$(BIN)

# ---- build + run + view waveform ----
wave: run
	$(GTKWAVE) $(VCD) &

# ---- lint only ----
lint:
	$(VERILATOR) --lint-only $(WAIVERS) $(RTL)

# ---- cleanup ----
clean:
	rm -rf obj_dir $(VCD)

help:
	@echo "make        - build and run $(TOP)"
	@echo "make wave   - build, run, open $(VCD) in gtkwave"
	@echo "make lint   - lint RTL only"
	@echo "make clean  - remove obj_dir and waveforms"