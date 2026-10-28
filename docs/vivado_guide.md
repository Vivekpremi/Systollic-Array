# Vivado guide: BFP/LNS accelerator on the VEK280

Goal: a Versal block design where the ARM cores (bare-metal) control the
accelerator over AXI-Lite and move blocks through an AXI DMA, then an `.xsa`
for Vitis. Menu names below are from Vivado 2026.1 and can differ slightly
between versions.

**What gets built**

```
 Versal CIPS (ARM A72) ──M_AXI_FPD──> SmartConnect ──> accelerator  s_axi   (control)
                                                   └──> AXI DMA      S_AXI_LITE
 AXI DMA  M_AXIS_MM2S ──> accelerator s_axis    (input blocks)
 accelerator m_axis   ──> AXI DMA  S_AXIS_S2MM  (output blocks)
 AXI DMA  M_AXI_MM2S / M_AXI_S2MM ──> AXI NoC ──> DDR   (buffers)
 pl0_ref_clk (100 MHz) + proc_sys_reset ──> everything in the PL
```

## 1. Project and sources

1. Keep the repository as it is (all RTL, testbenches and `.xdc` files in its root).
2. Start Vivado, open its **Tcl console** (not a Linux terminal) and run:
   ```
   cd <path>/Systollic-Array
   source vivado/create_project.tcl
   ```
   If it warns that no VEK280 board files exist: **Tools > Vivado Store > Boards**,
   install "VEK280", rerun the script.
   Manual alternative: add the RTL files as design sources, the three `tb_*.sv`
   files as **simulation sources only**, `bfp_accel_axi_top_clock.xdc` as a
   constraint; set `PE.sv` file type to **Verilog Header** and the repo root as
   include directory (*Settings > General > Verilog options*).
3. Check **Sources**: `bfp_accel_axi_top` is the top, and `PE.sv` shows as a
   Verilog Header (it is pulled in by `` `include `` in `systolic_array.sv`).
4. **Flow Navigator > Run Simulation > Run Behavioral Simulation**, type
   `run all` in the Tcl console (top: `tb_axi_full`, about 370 µs). Expect the
   same summary as `make tb_axi_full`: 125/140 numeric tests passed, AXI errors
   0, robustness 20 run / 20 passed. The detailed report `axi_full_report.txt`
   is written in the simulation folder.
5. (Optional) Synthesize the wrapper alone: *Synthesis Settings > More Options*
   = `-mode out_of_context`, **Run Synthesis**, **Open Synthesized Design**,
   *Reports > Timing > Report Timing Summary*. Measured: 7,149 LUT, 3,696 FF,
   0 DSP, 0 BRAM, WNS +1.699 ns at 100 MHz (Vivado retiming is needed in
   `format_convertor`; the log shows "Worst Slack before retiming -7.836 ns").

## 2. Block design

Before starting:
- *Synthesis Settings > More Options*: remove `-mode out_of_context`.
- Sources > Constraints: right-click `bfp_accel_axi_top_clock.xdc` > **Disable
  File**. Inside the block design `aclk` is no longer a top-level port, and the
  clock comes from CIPS, which constrains it itself.

1. **IP Integrator > Create Block Design**, name `bd_accel`.
2. **Add IP** (`+`) > *Control, Interfaces & Processing System* (CIPS).
   Click **Run Block Automation**:
   - Design Flow = Full System, Apply Board Preset = Yes, Debug = JTAG
   - PL Clocks = 1, PL Resets = 1
   - Memory Controller Type = LPDDR4 (1 controller)
   This adds `axi_noc_0` with the memory controller (ports `CH0_LPDDR4_0`,
   `CH1_LPDDR4_0`, external clock `lpddr4_clk1`) and one NoC slave port per
   CIPS master (`S00_AXI`..`S05_AXI`).
3. Double-click **CIPS** > *Next* > *PS PMC*:
   - **PL-PS interfaces**: enable **M_AXI_FPD** (PS master into the PL).
   - **Clocking > Output clocks**: PL0 = **100 MHz**.
   - **Interrupts**: enable **IRQ0** (PL to PS). Only needed for interrupts;
     the first bare-metal driver polls.
4. Right-click the canvas > **Add Module** > `bfp_accel_axi_top`. It should show
   interfaces `s_axi`, `s_axis`, `m_axis` and pins `aclk`, `aresetn`, `irq`.
5. **Add IP** > *AXI Direct Memory Access*, double-click it:
   - **Enable Scatter Gather Engine**: off (simple mode)
   - **Width of Buffer Length Register**: 26
   - **Address Width**: 64
   - Read channel and write channel both on; **Stream Data Width = 64**
     (= AXIS_WIDTH, read it from CFG1 if parameters change); memory map width 64
6. Connect the streams by hand (drag between pins):
   - `axi_dma_0/M_AXIS_MM2S` -> `bfp_accel_axi_top_0/s_axis`
   - `bfp_accel_axi_top_0/m_axis` -> `axi_dma_0/S_AXIS_S2MM`
7. **Run Connection Automation**, tick everything:
   - `bfp_accel_axi_top_0/s_axi` and `axi_dma_0/S_AXI_LITE`: master = CIPS `M_AXI_FPD`
   - `axi_dma_0/M_AXI_MM2S` and `M_AXI_S2MM`: slave = `axi_noc_0` (DDR)
   - clocks: `pl0_ref_clk`
   Vivado adds a SmartConnect, a Processor System Reset and the NoC ports.
8. (Interrupts, optional) **Add IP** > *Concat* with 3 inputs:
   `irq`, `axi_dma_0/mm2s_introut`, `axi_dma_0/s2mm_introut` -> CIPS `pl_ps_irq0`.
9. **Address Editor** > *Assign All*. Note the base addresses of
   `bfp_accel_axi_top_0` and `axi_dma_0`; the driver needs them (they also
   end up in `xparameters.h`).
10. **Validate Design** (F6). It must finish without errors.

## 3. Build

1. Sources > right-click `bd_accel` > **Create HDL Wrapper** (let Vivado manage) >
   it becomes the top.
2. **Generate Block Design** (out-of-context per IP is fine).
3. **Run Synthesis**, then **Run Implementation**.
4. Open the implemented design > **Report Timing Summary**: WNS must be >= 0.
   Synthesis alone gave +1.7 ns at 100 MHz; routing usually costs some of that.
   If it is negative, set PL0 to 80 MHz (or 50 MHz) in CIPS and rerun.
5. **Generate Device Image** (Versal's equivalent of a bitstream, a `.pdi`).
6. **File > Export > Export Hardware**, choose **Include device image**, save
   `bd_accel_wrapper.xsa`. That file goes to Vitis.

## 4. Bring back

- the timing summary (WNS, failing paths if any)
- the utilization report (LUT, FF, DSP, BRAM)
- critical warnings from synthesis
- the base addresses from the Address Editor

## If something fails

| Symptom | Likely cause | Fix |
| --- | --- | --- |
| *Add Module* rejects the file | module reference cannot read SystemVerilog below the top on this version | **Tools > Create and Package New IP** > package the project as an IP, then add that IP instead |
| `module PE redeclared` | PE.sv compiled as a normal source | set its file type to *Verilog Header* (the script does this) |
| `cannot open include file PE.sv` | include path missing | *Settings > General > Verilog options > Include directories* = repo root |
| DMA ports not offered to the NoC | connection automation did not create NoC ports | AXI NoC > add 2 slave AXI ports, connect DMA masters, tick MC in its *Connectivity* tab, clock = pl0_ref_clk |
| timing fails | 100 MHz too fast for the core | 80 or 50 MHz PL0 |
| `Command 'get' is not supported` in an `.xdc` | the constraint line got split or edited | the file must be the single line `create_clock -period 10.000 -name clk [get_ports aclk]` |
| `No ports matched 'aclk'` after the block design | `bfp_accel_axi_top_clock.xdc` still enabled | disable that file (the CIPS constrains the PL clock) |
| XSim `[XSIM 43-3122] ... urandom` | an old copy of `tb_axi_full.sv` | use the current one (it has its own random generator) |
