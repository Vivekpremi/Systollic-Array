# =============================================================================
# Systolic Array (BFP / LNS)  --  build & simulation
#
# Run `make help` for the full command and knob list.
#
# Verilator auto-discovers modules from files in this directory (hence
# -Wno-MULTITOP), so only the top-level sources need listing. PE.sv is pulled
# in by `include from systolic_array.sv -- it must exist but is never listed.
# It IS listed as a prerequisite below so that editing it forces a rebuild.
# =============================================================================

# ---- tools ----
VERILATOR ?= verilator
GTKWAVE   ?= gtkwave

# Extra flags appended to every verilator call, for host/toolchain flags you
# don't want baked into the recipes:   make EXTRA_FLAGS="-CFLAGS -fcoroutines"
EXTRA_FLAGS ?=

# =============================================================================
# PARAMETER KNOBS
#
# Each knob is named EXACTLY after the RTL parameter it sets, so there is
# nothing to memorise and nothing to look up:
#
#     make DATA_WIDTH=48
#     make BLOCK_MANTISSA_WIDTH=8 DATA_WIDTH=48
#     make lint ARRAY_SIZE=8
#
# `?=` means "default, unless overridden", and a VAR=value pair on the make
# command line overrides it. Note it is `make DATA_WIDTH=48`, NOT
# `make --DATA_WIDTH 48` -- these are make variables, not command-line options,
# so there are no dashes and the `=` is required.
#
# PARAMS below turns each knob into a Verilator -G override, which sets the
# TOP module's parameter without editing any source file. The list is built by
# a foreach over PARAM_LIST, so adding a knob means adding ONE name there.
# =============================================================================

# ARRAY_SIZE -- rows/cols of the PE array. Must be >= 4 and EVEN (see below).
ARRAY_SIZE           ?= 4
# EXP_WIDTH -- shared block exponent, signed.
EXP_WIDTH            ?= 8
# MANTISSA_WIDTH -- log-domain bus into the PE; this is INT+FRAC total.
MANTISSA_WIDTH       ?= 16
# FRAC_WIDTH -- fractional bits of that log. Must be 8 (see below).
FRAC_WIDTH           ?= 8
# DATA_WIDTH -- product width. The accumulator is DATA_WIDTH+1 (guard bit).
DATA_WIDTH           ?= 32
# BLOCK_MANTISSA_WIDTH -- residual log L per element in the INPUT block,
# two's complement. The single biggest accuracy lever.
BLOCK_MANTISSA_WIDTH ?= 6
# OUT_MANTISSA_WIDTH -- mantissa field in the OUTPUT block from format_convertor.
OUT_MANTISSA_WIDTH   ?= 5
# SCALE_FACTOR_WIDTH -- shared per-block scale factor.
SCALE_FACTOR_WIDTH   ?= 4

PARAM_LIST := ARRAY_SIZE EXP_WIDTH MANTISSA_WIDTH FRAC_WIDTH DATA_WIDTH \
              BLOCK_MANTISSA_WIDTH OUT_MANTISSA_WIDTH SCALE_FACTOR_WIDTH

# One-line description per knob, printed by `make help`. Kept beside
# PARAM_LIST so the two cannot drift apart.
DESC_ARRAY_SIZE           := rows/cols of the PE array; the tile is N x N
DESC_EXP_WIDTH            := shared block exponent, signed (per-PE sum is E_row+E_col)
DESC_MANTISSA_WIDTH       := log-domain bus into the PE, INT+FRAC total (8.8 by default)
DESC_FRAC_WIDTH           := fractional bits of that log; INT_WIDTH = MANTISSA_WIDTH-FRAC_WIDTH
DESC_DATA_WIDTH           := product width; the PE accumulator is DATA_WIDTH+1 (guard bit)
DESC_BLOCK_MANTISSA_WIDTH := residual log L per element in the INPUT block, two's complement
DESC_OUT_MANTISSA_WIDTH   := mantissa field per element in the OUTPUT block from format_convertor
DESC_SCALE_FACTOR_WIDTH   := shared per-block scale factor (L is stored as L>>scale)

# -GNAME=value for every knob above. $($(p)) is a double expansion: $(p) is the
# knob's NAME, so $($(p)) is its VALUE.
PARAMS := $(foreach p,$(PARAM_LIST),-G$(p)=$($(p)))

# =============================================================================
# CONSTRAINTS -- enforced by the design, not by this Makefile
#
#   ARRAY_SIZE  must be >= 4 and EVEN.
#       iface sends exponents/mantissas in PAIRS: it reads index [row_idx+1]
#       and advances row_idx by 2. At ARRAY_SIZE=2, row_idx is 1 bit, so
#       `row_idx + 2` does not fit and row_idx+1 indexes past the array.
#
#   FRAC_WIDTH  must be 8.
#       exponential_CORDIC's e[] table is 8 LITERAL constants (the shift-add
#       encodings of e^(2^-k), k=0..7). Any other value fails elaboration,
#       which is correct: more iterations need NEW constants computed, not a
#       resized array.
#
#   ARRAY_SIZE cannot be swept in SIMULATION.
#       tb_TOP_1's tests are explicit 4x4 matrices. `make lint ARRAY_SIZE=8`
#       works; `make ARRAY_SIZE=8` builds but the stimulus is still 4x4.
# =============================================================================

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
DEPS := PE.sv systolic_array.sv iface.sv format_convertor.sv \
        exponential_CORDIC.sv edge_reg_bank.sv

# RTL only (for lint; excludes the testbench).
# PE.sv is deliberately NOT listed -- systolic_array.sv `includes it, so listing
# both would define module PE twice (MODDUP error).
RTL := top_module.sv submodules_FC.sv systolic_array.sv iface.sv \
       format_convertor.sv exponential_CORDIC.sv edge_reg_bank.sv

TOP  := tb_TOP_1
BIN  := obj_dir/V$(TOP)
VCD  := TOP_1_tb.vcd

.PHONY: all run wave lint params sweep clean help

all: run

# ---- build ----
# The knobs are part of the command, not the file list, so make cannot tell
# that obj_dir is stale when only a knob changed. Use `make clean` (or just
# change a source) when switching configurations.
$(BIN): $(SRCS) $(DEPS)
	$(VERILATOR) $(WAIVERS) $(PARAMS) $(EXTRA_FLAGS) --trace --binary $(SRCS)

# ---- build + run ----
run: $(BIN)
	./$(BIN)

# ---- build + run + view waveform ----
wave: run
	$(GTKWAVE) $(VCD) &

# ---- lint only ----
# --top-module is required so -G has one known top to apply to.
lint:
	$(VERILATOR) --lint-only $(WAIVERS) $(PARAMS) $(EXTRA_FLAGS) \
	    --top-module top_module $(RTL)

# ---- show the parameters in force ----
params:
	@echo "parameters for this build:"
	@$(foreach p,$(PARAM_LIST),printf "  %-22s = %-3s %s\n" "$(p)" "$($(p))" "$(DESC_$(p))";)
	@echo ""
	@echo "constraints: ARRAY_SIZE >= 4 and even;  FRAC_WIDTH = 8"

# ---- lint a set of knob combinations ----
sweep:
	@fail=0; \
	for cfg in \
	    "" \
	    "ARRAY_SIZE=8" \
	    "EXP_WIDTH=12" "EXP_WIDTH=6" \
	    "DATA_WIDTH=48" "DATA_WIDTH=24" \
	    "MANTISSA_WIDTH=20" "MANTISSA_WIDTH=12" \
	    "BLOCK_MANTISSA_WIDTH=8" "BLOCK_MANTISSA_WIDTH=4" \
	    "OUT_MANTISSA_WIDTH=7" "OUT_MANTISSA_WIDTH=4" \
	    "SCALE_FACTOR_WIDTH=6" "SCALE_FACTOR_WIDTH=2" \
	    "BLOCK_MANTISSA_WIDTH=8 DATA_WIDTH=48 EXP_WIDTH=12" \
	    "ARRAY_SIZE=8 EXP_WIDTH=12 DATA_WIDTH=48 BLOCK_MANTISSA_WIDTH=8 OUT_MANTISSA_WIDTH=7 SCALE_FACTOR_WIDTH=6" ; do \
	  if $(MAKE) --no-print-directory lint $$cfg >/dev/null 2>&1; then \
	    printf "  PASS  %s\n" "$${cfg:-defaults}"; \
	  else \
	    printf "  FAIL  %s\n" "$${cfg:-defaults}"; fail=1; \
	  fi; \
	done; \
	exit $$fail

# ---- cleanup ----
clean:
	rm -rf obj_dir $(VCD)

# =============================================================================
help:
	@echo "Systolic Array (BFP / LNS) -- make targets"
	@echo ""
	@echo "  make              build and run $(TOP)  (same as 'make run')"
	@echo "  make run          build and run the testbench"
	@echo "  make wave         build, run, then open $(VCD) in gtkwave"
	@echo "  make lint         lint the RTL only, no simulation"
	@echo "  make params       print the parameter values this build would use"
	@echo "  make sweep        lint every knob combination in the sweep list"
	@echo "  make clean        remove obj_dir and $(VCD)"
	@echo "  make help         this message"
	@echo ""
	@echo "PARAMETER KNOBS -- each is named after the RTL parameter it sets."
	@echo "Override with VAR=value on the command line (no dashes):"
	@echo ""
	@$(foreach p,$(PARAM_LIST),printf "  %-22s = %-3s %s\n" "$(p)" "$($(p))" "$(DESC_$(p))";)
	@echo ""
	@echo "  examples:"
	@echo "    make DATA_WIDTH=48"
	@echo "    make BLOCK_MANTISSA_WIDTH=8 DATA_WIDTH=48 EXP_WIDTH=12"
	@echo "    make lint ARRAY_SIZE=8"
	@echo "    make clean && make BLOCK_MANTISSA_WIDTH=8     # knobs are not"
	@echo "                                                  # tracked deps,"
	@echo "                                                  # so clean first"
	@echo ""
	@echo "CONSTRAINTS"
	@echo "  ARRAY_SIZE  must be >= 4 and EVEN   (iface pairs operands)"
	@echo "  FRAC_WIDTH  must be 8               (CORDIC e[] table is 8 consts)"
	@echo "  ARRAY_SIZE  lints at any legal value but cannot be swept in"
	@echo "              simulation: the testbench stimulus is explicit 4x4"