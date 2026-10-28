# =============================================================================
# Systolic Array (BFP / LNS)  --  build & simulation
#
# Run a testbench by its name:
#     make tb_TOP_1        core testbench: 140 numeric tests       (also: make, make run)
#     make tb_axi          AXI wrapper smoke test: 2 tiles
#     make tb_axi_full     140 tests + 20 robustness tests, all through AXI
#     make test_all        all three, one after another
# Run `make help` for every target and knob.
#
# Each testbench builds into its own folder obj_<name>/, so they never mix.
# PE.sv is pulled in by `include from systolic_array.sv, so it is never put
# on the verilator command line (that would define module PE twice). It is a
# prerequisite, so editing it still forces a rebuild.
# =============================================================================

# ---- tools ----
VERILATOR ?= verilator
GTKWAVE   ?= gtkwave

# Extra flags appended to every verilator call, for host/toolchain flags you
# don't want baked into the recipes:   make tb_TOP_1 EXTRA_FLAGS="-CFLAGS -std=c++20"
EXTRA_FLAGS ?=

# Run-time options passed to the simulation binary, e.g. a waveform for
# tb_axi_full:                          make tb_axi_full PLUSARGS=+dump
PLUSARGS ?=

# =============================================================================
# PARAMETER KNOBS
#
# Each knob is named EXACTLY after the RTL parameter it sets:
#
#     make tb_TOP_1 DATA_WIDTH=48
#     make tb_axi_full BLOCK_MANTISSA_WIDTH=8 DATA_WIDTH=48
#     make lint EXP_WIDTH=12
#
# `?=` means "default, unless overridden"; a VAR=value pair on the make command
# line overrides it (no dashes, the `=` is required).
#
# PARAMS turns each knob into a Verilator -G override of the TOP module's
# parameter. tb_TOP_1 and tb_axi_full have all 7 parameters and pass them
# down to the design. tb_axi is a fixed smoke test at the defaults, so the
# knobs are not applied to it.
# =============================================================================

# ARRAY_SIZE -- rows/cols of the PE array. Must be 4 for now (see CONSTRAINTS).
ARRAY_SIZE           ?= 4
# EXP_WIDTH -- shared block exponent, signed.
EXP_WIDTH            ?= 8
# MANTISSA_WIDTH -- log-domain bus into the PE; this is INT+FRAC total.
MANTISSA_WIDTH       ?= 16
# FRAC_WIDTH -- fractional bits of that log. Must be 8 (see CONSTRAINTS).
FRAC_WIDTH           ?= 8
# DATA_WIDTH -- product width. The accumulator is DATA_WIDTH+1 (guard bit).
DATA_WIDTH           ?= 32
# BLOCK_MANTISSA_WIDTH -- residual log L per element, two's complement, in
# BOTH the input and the output block (one format). The biggest accuracy lever.
BLOCK_MANTISSA_WIDTH ?= 6
# SCALE_FACTOR_WIDTH -- shared per-block scale.
SCALE_FACTOR_WIDTH   ?= 4

PARAM_LIST := ARRAY_SIZE EXP_WIDTH MANTISSA_WIDTH FRAC_WIDTH DATA_WIDTH \
              BLOCK_MANTISSA_WIDTH SCALE_FACTOR_WIDTH

# One-line description per knob, printed by `make help` / `make params`.
DESC_ARRAY_SIZE           := rows/cols of the PE array; the tile is N x N (must be 4 for now)
DESC_EXP_WIDTH            := shared block exponent, signed (per-PE sum is E_row+E_col)
DESC_MANTISSA_WIDTH       := log-domain bus into the PE, INT+FRAC total (8.8 by default)
DESC_FRAC_WIDTH           := fractional bits of that log; must be 8
DESC_DATA_WIDTH           := product width; the PE accumulator is DATA_WIDTH+1 (guard bit)
DESC_BLOCK_MANTISSA_WIDTH := residual log L per element, two's complement; input AND output block
DESC_SCALE_FACTOR_WIDTH   := shared per-block scale (one L step = 2^scale / 2^FRAC_WIDTH octave)

# -GNAME=value for every knob above ($($(p)) = the VALUE of the knob named p).
PARAMS := $(foreach p,$(PARAM_LIST),-G$(p)=$($(p)))

# =============================================================================
# CONSTRAINTS -- enforced by the design, not by this Makefile
#
#   ARRAY_SIZE  must be 4 for now:
#       - iface sends the results in PAIRS (reads [row_idx+1], steps by 2),
#         so it must be even and >= 4;
#       - format_convertor only builds the divide by n = 1..4 for the mean
#         (a bigger value stops elaboration with an $error; the shift-and-add
#         table for n up to 16 is in its comments);
#       - the testbenches' stimulus is explicit 4x4 matrices.
#
#   FRAC_WIDTH  must be 8.
#       exponential_CORDIC's e[] table is 8 constants (the shift-add encodings
#       of e^(2^-k), k = 0..7). Other values fail elaboration.
# =============================================================================

# ---- warning suppressions (the agreed set) ----
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
# the core (PE.sv is `included by systolic_array.sv, so not listed here)
CORE := top_module.sv submodules_FC.sv systolic_array.sv iface.sv \
        format_convertor.sv exponential_CORDIC.sv edge_reg_bank.sv
# the AXI wrapper: Verilog shell for Vivado + the SystemVerilog wrapper
AXI  := bfp_accel_axi_top.v bfp_accel_axi.sv

# the testbenches, and what each one needs
TBS := tb_TOP_1 tb_axi tb_axi_full

SRCS_tb_TOP_1    := tb_TOP_1.sv $(CORE)
SRCS_tb_axi      := tb_axi.sv $(AXI) $(CORE)
SRCS_tb_axi_full := tb_axi_full.sv $(AXI) $(CORE)

# knobs (-G) per testbench; tb_axi has no parameters
GPARAMS_tb_TOP_1    := $(PARAMS)
GPARAMS_tb_axi      :=
GPARAMS_tb_axi_full := $(PARAMS)

# waveform support: tb_TOP_1 always writes TOP_1_tb.vcd; tb_axi_full writes
# axi_full.vcd only when run with PLUSARGS=+dump
TRACE_tb_TOP_1    := --trace
TRACE_tb_axi      :=
TRACE_tb_axi_full := --trace

.PHONY: all run wave lint lint_axi params sweep test_all clean help $(TBS)

all: tb_TOP_1
run: tb_TOP_1

# ---- one build rule + one run rule per testbench ----
# obj_<tb>/V<tb> is the simulation binary; `make <tb>` builds it if any
# source changed, then runs it.
# --top-module is REQUIRED: -G overrides apply to the top module only.
# The knobs are part of the command, not the file list, so make cannot tell
# that a binary is stale when only a knob changed: `make clean` first when
# switching configurations.
define TB_RULES
obj_$(1)/V$(1): $$(SRCS_$(1)) PE.sv
	$$(VERILATOR) $$(WAIVERS) $$(GPARAMS_$(1)) $$(EXTRA_FLAGS) $$(TRACE_$(1)) --binary \
	    --Mdir obj_$(1) --top-module $(1) $$(SRCS_$(1))

$(1): obj_$(1)/V$(1)
	./obj_$(1)/V$(1) $$(PLUSARGS)
endef
$(foreach t,$(TBS),$(eval $(call TB_RULES,$(t))))

# ---- all testbenches, one after another ----
test_all: $(TBS)

# ---- tb_TOP_1 + waveform viewer ----
wave: tb_TOP_1
	$(GTKWAVE) TOP_1_tb.vcd &

# ---- lint only ----
# lint     : the core alone (top = top_module)
# lint_axi : the AXI wrapper + core, as Vivado will see it (top = bfp_accel_axi_top)
lint:
	$(VERILATOR) --lint-only $(WAIVERS) $(PARAMS) $(EXTRA_FLAGS) \
	    --top-module top_module $(CORE)

lint_axi:
	$(VERILATOR) --lint-only $(WAIVERS) $(PARAMS) $(EXTRA_FLAGS) \
	    --top-module bfp_accel_axi_top $(AXI) $(CORE)

# ---- show the parameters in force ----
params:
	@echo "parameters for this build:"
	@$(foreach p,$(PARAM_LIST),printf "  %-22s = %-3s %s\n" "$(p)" "$($(p))" "$(DESC_$(p))";)
	@echo ""
	@echo "constraints: ARRAY_SIZE = 4 for now;  FRAC_WIDTH = 8"

# ---- lint the core for a set of knob combinations ----
sweep:
	@fail=0; \
	for cfg in \
	    "" \
	    "EXP_WIDTH=12" "EXP_WIDTH=6" \
	    "DATA_WIDTH=48" "DATA_WIDTH=24" \
	    "MANTISSA_WIDTH=20" "MANTISSA_WIDTH=12" \
	    "BLOCK_MANTISSA_WIDTH=8" "BLOCK_MANTISSA_WIDTH=4" \
	    "SCALE_FACTOR_WIDTH=6" "SCALE_FACTOR_WIDTH=2" \
	    "BLOCK_MANTISSA_WIDTH=8 DATA_WIDTH=48 EXP_WIDTH=12" ; do \
	  if $(MAKE) --no-print-directory lint $$cfg >/dev/null 2>&1; then \
	    printf "  PASS  %s\n" "$${cfg:-defaults}"; \
	  else \
	    printf "  FAIL  %s\n" "$${cfg:-defaults}"; fail=1; \
	  fi; \
	done; \
	exit $$fail

# ---- cleanup (reports are kept) ----
clean:
	rm -rf $(foreach t,$(TBS),obj_$(t)) obj_dir TOP_1_tb.vcd axi_full.vcd

# =============================================================================
help:
	@echo "Systolic Array (BFP / LNS) -- make targets"
	@echo ""
	@echo "  TESTBENCHES (build if needed, then run)"
	@echo "  make tb_TOP_1     core: 140 numeric tests            -> test_report.txt"
	@echo "  make tb_axi       AXI wrapper smoke test, 2 tiles     -> terminal only"
	@echo "  make tb_axi_full  140 tests + 20 robustness via AXI  -> axi_full_report.txt"
	@echo "  make test_all     all three, one after another"
	@echo "  make / make run   same as make tb_TOP_1"
	@echo "  make wave         make tb_TOP_1, then open TOP_1_tb.vcd in gtkwave"
	@echo ""
	@echo "  OTHER"
	@echo "  make lint         lint the core (top_module), no simulation"
	@echo "  make lint_axi     lint the AXI wrapper + core (bfp_accel_axi_top)"
	@echo "  make params       print the parameter values in force"
	@echo "  make sweep        lint the core for every knob combination in the list"
	@echo "  make clean        remove the build folders and waveforms (reports are kept)"
	@echo "  make help         this message"
	@echo ""
	@echo "PARAMETER KNOBS -- each is named after the RTL parameter it sets."
	@echo "Override with VAR=value on the command line (no dashes)."
	@echo "Applied to tb_TOP_1, tb_axi_full, lint, lint_axi (tb_axi is fixed):"
	@echo ""
	@$(foreach p,$(PARAM_LIST),printf "  %-22s = %-3s %s\n" "$(p)" "$($(p))" "$(DESC_$(p))";)
	@echo ""
	@echo "  examples:"
	@echo "    make tb_TOP_1 DATA_WIDTH=48"
	@echo "    make tb_axi_full BLOCK_MANTISSA_WIDTH=8 DATA_WIDTH=48 EXP_WIDTH=12"
	@echo "    make tb_axi_full PLUSARGS=+dump        # also write axi_full.vcd"
	@echo "    make clean && make tb_TOP_1 BLOCK_MANTISSA_WIDTH=8   # knobs are not"
	@echo "                                                         # tracked, clean first"
	@echo ""
	@echo "CONSTRAINTS"
	@echo "  ARRAY_SIZE  must be 4 for now  (iface pairs results; FC divides by 1..4 only;"
	@echo "                                  stimulus is 4x4)"
	@echo "  FRAC_WIDTH  must be 8          (CORDIC e[] table is 8 constants)"
