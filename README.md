# BFP / LNS Systolic-Array Matrix-Multiply Accelerator

A 4x4 output-stationary systolic array in SystemVerilog that multiplies
matrices stored in a compact **block log format**. Multiplication is done as
addition of logs, so the design uses **no DSP multipliers**. Target: AMD Versal
**VEK280** (`xcve2802-vsvh1760-2MP-e-S`), Vivado 2026.1.

## Number format

One 48-bit block holds 4 numbers that share an exponent `E` and a `scale`:

```
v = (-1)^sign * 2^( E + L * 2^scale / 256 )        (v = 0 if zero = 1)

[23:0]  L[3..0]  6-bit signed each      [35:32] scale (0..15)
[27:24] zero[3..0]                      [43:36] E, signed (-128..127)
[31:28] sign[3..0]                      [47:44] unused (0)
```

- `E = round(mean of log2|v|)` over the non-zero elements.
- `L` is the residual log in steps of `2^scale / 256` octave.

Input and output blocks use the same format, so a result can be fed straight
back in as an operand. Full range and error analysis: `docs/numeric_range.md`.

## Architecture

```
edge_reg_bank -> systolic_array (4x4 PE) -> iface -> format_convertor
```

| Module | Function |
|---|---|
| `edge_reg_bank` | Stores 4 row blocks (A) and 4 column blocks (B), and feeds one element per product into the array edges (`L << scale`). |
| `PE` | Log add, then 2^f antilog (`exponential_CORDIC`, shift-add), then shift, then linear accumulate. Sticky overflow flag. |
| `systolic_array` | 4x4 grid. Rows flow right, columns flow down. PE[i][j] holds C[i][j]. |
| `iface` | Sends each finished column of results to the encoder, 2 per clock. |
| `format_convertor` | Turns 4 results into one output block: Mitchell log2, mean exponent (divide by 1..4 with shifts and adds), scale search, rounding. |
| `bfp_accel_axi` | AXI wrapper: AXI4-Lite registers, AXI4-Stream in (8 blocks) and out (4 blocks), per-tile core reset, timeout, cycle counter, IRQ. |
| `bfp_accel_axi_top.v` | Verilog shell for use as a Vivado block-design module reference. |

One tile = (4x4) x (4x4). START to DONE takes **109 cycles**. The first output
block comes 78 cycles after START, then one block every 10 cycles.

## Files

| Path | Contents |
|---|---|
| `*.sv`, `*.v` | RTL (`PE.sv` is `include`d by `systolic_array.sv`) |
| `tb_TOP_1.sv` | Core testbench: 140 tests against a double-precision reference |
| `tb_axi.sv` | AXI wrapper smoke test (2 tiles) |
| `tb_axi_full.sv` | All 140 tests through the AXI wrapper, plus 20 robustness tests, an AXI protocol monitor and a per-tile timing report |
| `Makefile` | Verilator flow |
| `top_module_clock.xdc` | 100 MHz clock for `top_module` as top |
| `bfp_accel_axi_top_clock.xdc` | 100 MHz clock for `bfp_accel_axi_top` as top |
| `vivado/create_project.tcl` | Creates the Vivado project |
| `docs/axi_interface.md` | Register map and stream format |
| `docs/vivado_guide.md` | Versal block-design steps |
| `docs/numeric_range.md` | Numeric limits and error derivation |
| `reports/` | Latest simulation reports and logs |
| `CHANGELOG.md` | Change record |

## Simulation (Verilator 5.x)

```
make tb_TOP_1        # core, 140 tests      -> test_report.txt
make tb_axi          # AXI smoke test
make tb_axi_full     # full AXI test        -> axi_full_report.txt
make test_all        # all three
make lint            # core lint
make lint_axi        # wrapper + core lint
make sweep           # lint over 12 parameter sets
make help            # all targets and knobs
```

Waveform: `make tb_axi_full PLUSARGS=+dump`.

### Vivado XSim

Set `PE.sv` file type to **Verilog Header** and add the source folder to
include directories. Add testbenches as simulation sources only. Run with
`run all`.

## Results

**Simulation (default parameters)**
- `tb_TOP_1`: 125 / 140 pass. The 15 failures are blocks with a very wide
  value range (1e-4..1e4), which exceeds what a 6-bit L can hold within 10%.
  This is a format limit, not a logic error.
- `tb_axi_full`: results identical to `tb_TOP_1`; 0 AXI protocol violations,
  0 hangs; robustness 20 / 20.

**Synthesis**: `bfp_accel_axi_top`, out-of-context, xcve2802-2MP, 100 MHz

| LUT | FF | DSP | BRAM | WNS | WHS |
|---|---|---|---|---|---|
| 7,149 (1.4%) | 3,696 (0.35%) | 0 | 0 | +1.699 ns | +0.033 ns |

Timing at 100 MHz depends on Vivado retiming in `format_convertor`. Without
retiming the worst path is about 17.8 ns.

## Limitations

- `ARRAY_SIZE` = 4 only. `FRAC_WIDTH` = 8 only (CORDIC table).
- One tile per core reset; the AXI wrapper does this automatically.
- K > 4 partial sums are added in software.
- Output log uses Mitchell's approximation: up to ~6% low.

## Status

- [x] RTL and verification
- [x] AXI4-Lite / AXI4-Stream wrapper
- [x] Synthesis, timing met at 100 MHz
- [ ] Versal block design (CIPS + NoC/LPDDR4 + AXI DMA): in progress
- [ ] Bare-metal driver
- [ ] Neural-network image-processing demo on the board
