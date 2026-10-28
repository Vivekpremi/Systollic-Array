# =============================================================================
# create_project.tcl -- Vivado project for the BFP/LNS accelerator on VEK280
#
# Run from the repository root, in the Vivado Tcl console:
#     cd <path>/Systollic-Array
#     source vivado/create_project.tcl
#
# Creates ./vivado/bfp_vek280 with the RTL added, PE.sv handled as an include,
# the clock constraint, and the testbenches in the simulation set. The block
# design is built after this, in the GUI (see docs/vivado_guide.md).
# =============================================================================
set repo   [file normalize [pwd]]
set projdir [file join $repo vivado bfp_vek280]

create_project bfp_vek280 $projdir -part xcve2802-vsvh1760-2MP-e-S -force

# Board files: use the VEK280 board if this Vivado has it installed
set boards [get_board_parts -quiet *vek280*]
if {[llength $boards] > 0} {
    set_property board_part [lindex [lsort $boards] end] [current_project]
    puts "INFO: board part = [lindex [lsort $boards] end]"
} else {
    puts "WARNING: no VEK280 board files found. Install them from the Vivado Store (Tools > Vivado Store > Boards), then rerun."
}

# ---- design sources ----
set rtl {
    bfp_accel_axi_top.v
    bfp_accel_axi.sv
    top_module.sv
    edge_reg_bank.sv
    systolic_array.sv
    iface.sv
    format_convertor.sv
    submodules_FC.sv
    exponential_CORDIC.sv
}
foreach f $rtl { add_files -norecurse [file join $repo $f] }

# PE.sv is `include'd by systolic_array.sv, so it must NOT be compiled on its
# own (that would define module PE twice). Add it as a Verilog header and put
# the repo root on the include path.
add_files -norecurse [file join $repo PE.sv]
set_property file_type {Verilog Header} [get_files PE.sv]
set_property include_dirs [list $repo] [current_fileset]

set_property top bfp_accel_axi_top [current_fileset]

# ---- constraints ----
# 100 MHz on aclk, for synthesizing the wrapper alone. Disable this file once
# the block design wrapper is the top (the CIPS then constrains the PL clock).
add_files -fileset constrs_1 -norecurse [file join $repo bfp_accel_axi_top_clock.xdc]

# ---- simulation sources ----
# tb_axi_full : 140 tests + 20 robustness tests through the AXI wrapper (the top)
# tb_axi      : 2-tile smoke test
# tb_TOP_1    : the core alone (set it as top to use it)
add_files -fileset sim_1 -norecurse [list [file join $repo tb_axi_full.sv] [file join $repo tb_axi.sv] [file join $repo tb_TOP_1.sv]]
set_property top tb_axi_full [get_filesets sim_1]
set_property include_dirs [list $repo] [get_filesets sim_1]

update_compile_order -fileset sources_1
puts "INFO: project ready at $projdir"
