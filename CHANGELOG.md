# Change record

Every change gets an entry here. The "Undo" lines refer to snapshot folders
(the full file set before each step) that are kept outside this repository;
in git, use the commit history instead.

## v0_before_cleanup  (snapshot)
State after: comments rewrite of format_convertor, divide by 1/2/3/4 with
shifts and shift-and-add, found flag removed, exp0/exp1 + exponent_out_1/2 +
max_exp_calculated removed (FC, iface, top, tb), tests 137-140 + divide-path
coverage in tb.
Regression: 140 tests, 125 pass; table and report saved as run4.log /
test_report.txt in the snapshot.

## Planned (all 4 done, see below)
1. Dead wires & params
2. Unused modules (round_2_power, shifter_quantizer)
3. Debug / duplicate ports (dbg_row_ptr/dbg_col_ptr, demand_from_mem, iface_state_out)
4. Exponent pass in iface -> wait-for-column-ready

## v1_dead_wires  (change 1 done)
- systolic_array.sv: removed wire valid_condition + its commented-out assign
- iface.sv: removed genvar i,j; removed params BLOCK_MANTISSA_WIDTH, SCALE_FACTOR_WIDTH
- format_convertor.sv: removed localparam EXP_LSB
- top_module.sv: iface instance no longer passes BLOCK_MANTISSA_WIDTH / SCALE_FACTOR_WIDTH
- exponential_CORDIC.sv: removed params Int_WIDTH, DATA_WIDTH
- PE.sv: CORDIC instance passes only Frac_WIDTH
Check: lint OK, sweep 12/14 (ARRAY_SIZE=8 fails on purpose), 140 rows identical, report identical.
Undo: copy v0_before_cleanup/*.sv back.

## v2_unused_modules  (change 2 done)
- submodules_FC.sv: removed module shifter_quantizer and module round_2_power
  (+ its one-line header comment). Kept shifter, onehot_encoder, rev_num,
  lead_one_detector, log_finder, descaling.
Check: lint OK, sweep 12/14, 140 rows identical, report identical.
Undo: copy v1_dead_wires/submodules_FC.sv back.

## v3_debug_ports  (change 3 done)
- edge_reg_bank.sv: removed outputs dbg_row_ptr / dbg_col_ptr and their assigns; header comment no longer names demand_from_mem
- systolic_array.sv: removed output demand_from_mem; set_i_ready[i] = demand_pe[i][0] directly
- iface.sv: removed output state_out
- top_module.sv: removed output iface_state_out, wires dbg_row_ptr/dbg_col_ptr/demand_from_mem and their connections
- tb_TOP_1.sv: removed wire iface_state_out and its connection
- kept: PE's own demand_from_mem output (it drives demand_pe -> set_i_ready)
Check: lint OK, sweep 12/14, 140 rows identical, report identical.
Undo: copy v2_unused_modules/*.sv back.

## v4_no_exp_pass  (change 4 done)
- iface.sv: SEND_EXPONENTS replaced by WAIT_COLUMN (same code 2'b01): waits for
  col_ready = all valid_column[r][col_idx]; removed outputs valid_exp_out,
  last_exp_sent and regs valid_exp_out_q, last_exp_sent_q; header comment updated
- format_convertor.sv: removed inputs exp_valid, last_exp_sent and state ST_EXP;
  FSM is ST_COLLECT -> ST_ENCODE -> ST_COLLECT; ST_ENCODE clears idx, alog_sum,
  nz_cnt; reset/default state = ST_COLLECT; banner updated
- top_module.sv: removed outputs valid_exp_out, last_exp_sent and their connections
- tb_TOP_1.sv: removed wires valid_exp_out, last_exp_sent and connections
Check: lint OK, sweep 12/14, 140 rows identical, report identical.
Timing: every block_ready 1 clock (10 ns) earlier; blocks still 100 ns apart
(the PEs finish column by column, that is the limit).
Undo: copy v3_debug_ports/*.sv back.

## v5_tb_axi_full  (new testbench only, no RTL change)
- NEW tb_axi_full.sv: all 140 numeric tests through bfp_accel_axi_top (AXI-Lite + AXI-Stream),
  random input gaps / output back-pressure / AXI-Lite delays, AXI protocol monitor, bit-exact
  beat-vs-core check, register checks, per-test TIMING section, 20 robustness tests.
- Result: numeric rows identical to tb_TOP_1 (125/140); 0 AXI errors in the 140 tiles;
  robustness 19/20 (before the fix) -- R13 (ABORT while sending results) FAILS: wrapper drops TVALID and cuts the
  packet without TLAST. Wrapper not changed (waiting for decision).

## v6_abort_fix  (AXI wrapper)
- bfp_accel_axi.sv: `if (abort_cmd)` -> `if (abort_cmd && state != S_SEND)` + comment. ABORT is
  ignored while the output blocks are being sent, so the AXI-Stream packet always completes
  with TLAST (an aborted packet left the DMA waiting forever).
- docs/axi_interface.md: CTRL.ABORT description updated.
- tb_axi_full.sv: R13 now expects: ABORT ignored, all 4 beats delivered, TLAST on beat 4,
  0 violations.
Undo: copy rec/v5_before_abort_fix/ back.

## v7_comments  (comments only, code identical)
- New comments (banner, every variable, maths, limitations, no history) in PE.sv,
  systolic_array.sv, iface.sv, exponential_CORDIC.sv, edge_reg_bank.sv, submodules_FC.sv,
  top_module.sv, bfp_accel_axi.sv, bfp_accel_axi_top.v; tb_TOP_1.sv banner + history notes;
  format_convertor.sv: ARRAY_SIZE <= 4 note. tb_axi_full.sv regenerated from tb_TOP_1.
- Proof: comments stripped, every file's code identical token for token; line endings kept.
- Also: Makefile (make <tb_name> targets, lint_axi, sweep 12/12), PROJECT_CONTEXT.md,
  vivado/create_project.tcl (tb_axi_full as sim top), docs/vivado_guide.md updated.
Undo: copy rec/v6_before_comments/ back.

## v8_xsim_random  (testbench only)
- tb_axi_full.sv: $urandom (not supported this way by Vivado XSim, error XSIM 43-3122) replaced
  by an in-testbench xorshift32 generator (rng_state, seeded from seed). Verilator and XSim now
  use the identical random sequence. Result unchanged: 140 rows identical, 0 AXI errors,
  robustness 20/20.

## v9_github_readme  (files added, no RTL change)
- NEW README.md (GitHub overview: format, architecture, files, how to run, results incl. synthesis).
- NEW bfp_accel_axi_top_clock.xdc: `create_clock -period 10.000 -name clk [get_ports aclk]`
  (the constraint used for the wrapper synthesis: WNS +1.699 ns at 100 MHz).
- NEW .gitignore: Verilator obj_* folders, waveforms, Vivado project/run folders.
Undo: delete these three files.

## v10_doc_fixes  (documentation and reports only, no RTL change)
- reports/axi_full_report.txt, reports/tb_axi_full_terminal.log: re-run with the current tb_axi_full
  (v8 random generator). Results identical; only the random-gap timings differ.
- docs/numeric_range.md: big product / sum overflow now say pe_overflow = 1 (the flag exists);
  driver check 6 (read PE_OVF_LO) replaces the old "open hardware item"; test count wording.
- docs/vivado_guide.md: Vivado 2026.1, block design name bd_accel, block-automation options as seen,
  measured synthesis numbers, steps before the block design (remove out_of_context, disable the
  wrapper xdc), more troubleshooting rows.
- vivado/create_project.tcl: adds bfp_accel_axi_top_clock.xdc to constrs_1.
- CHANGELOG.md: header says the undo snapshots are kept outside the repo.
- PROJECT_CONTEXT.md: synthesis results, block-design progress, paths, new files; listed in .gitignore.
Undo: copy rec/v9_before_docfix/ back.
