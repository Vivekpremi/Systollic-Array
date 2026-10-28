# AXI interface of the BFP/LNS accelerator (`bfp_accel_axi`)

The wrapper puts `top_module` behind standard AXI interfaces so it can be dropped
into a Vivado block design on the VEK280 and driven by the ARM cores.

| Interface | Type | Purpose |
| --- | --- | --- |
| `s_axi_*` | AXI4-Lite slave, 32-bit | control, status, flags, configuration |
| `s_axis_*` | AXI4-Stream slave | input blocks (from an AXI DMA, MM2S channel) |
| `m_axis_*` | AXI4-Stream master | output blocks (to an AXI DMA, S2MM channel) |
| `irq` | level, active high | tile finished (if enabled) |

One clock (`aclk`) and one active-low reset (`aresetn`) for everything.

## One tile

A tile is one 4x4 by 4x4 multiply (ARRAY_SIZE x ARRAY_SIZE in general):
2 x ARRAY_SIZE input blocks in, ARRAY_SIZE output blocks out.

1. Driver arms the DMA receive channel (S2MM) for ARRAY_SIZE beats.
2. Driver sends the 2 x ARRAY_SIZE input blocks on `s_axis` (DMA MM2S).
   The wrapper stores them; `STATUS.IN_READY` = 1 after the last one.
3. Driver writes `CTRL.START` = 1.
4. The wrapper resets the core (clears every accumulator), loads all
   2 x ARRAY_SIZE blocks in the same clock cycle, and lets the array run.
5. After the core has produced ARRAY_SIZE output blocks, `STATUS.DONE` = 1
   (and `irq` if enabled), and the output blocks go out on `m_axis`.
6. Driver reads the flags (`STATUS`, `PE_OVF_*`, `BLK_EXP_*`) and the
   received output blocks.

The core is reset at the start of every tile because the PE accumulators only
clear on reset.

## Stream format

`AXIS_WIDTH` is derived: the smallest of 32, 64, 128, 256, 512 that holds
BLOCK_WIDTH + 2 bits (64 at the defaults, where a block is 48 bits). One block
per beat, in the low bits. Configure the AXI DMA stream width to match.

**Input (`s_axis`)**: exactly 2 x ARRAY_SIZE beats per tile.

| Beat | Content |
| --- | --- |
| 0 .. ARRAY_SIZE-1 | row block i (row i of A) |
| ARRAY_SIZE .. 2 x ARRAY_SIZE-1 | column block j (column j of B) |

`TDATA[BLOCK_WIDTH-1:0]` = the block, upper bits ignored. `TLAST` on the last
beat. A `TLAST` on any other beat, or a missing one, sets `STATUS.IN_LEN_ERR`
and the input is discarded.

**Output (`m_axis`)**: exactly ARRAY_SIZE beats per tile, beat j = block of column j of C.

| Bits | Content |
| --- | --- |
| `[BLOCK_WIDTH-1:0]` | the output block (same format as an input block) |
| `[BLOCK_WIDTH]` | exp_overflow of this block |
| `[BLOCK_WIDTH+1]` | exp_underflow of this block |
| rest | 0 |

`TLAST` on the last beat. `TREADY` back-pressure is honoured.

## Register map (AXI4-Lite, byte offsets)

| Offset | Name | Access | Bits |
| --- | --- | --- | --- |
| 0x00 | CTRL | W | [0] START: run a tile (ignored while BUSY or without IN_READY). [1] ABORT: reset core and wrapper, drop stored input; ignored while the output blocks are being sent (an AXI-Stream packet cannot be cut short), so the 4 beats always complete with TLAST. Both self-clear. |
| 0x04 | STATUS | R | [0] BUSY, [1] DONE, [2] IN_READY, [3] TIMEOUT, [4] EXP_OVF_ANY, [5] EXP_UNF_ANY, [6] ACC_OVF_ANY, [7] IN_LEN_ERR. DONE/TIMEOUT/flags clear on START. |
| 0x08 | IRQ_EN | RW | [0] interrupt on DONE or TIMEOUT |
| 0x0C | IRQ_STAT | R / W1C | [0] pending; write 1 to clear |
| 0x10 | PE_OVF_LO | R | pe_overflow[31:0] (PE[i][j] at bit i x ARRAY_SIZE + j) |
| 0x14 | PE_OVF_HI | R | pe_overflow[63:32] (ARRAY_SIZE > 5 only) |
| 0x18 | BLK_EXP_OVF | R | bit j = exp_overflow of output block j |
| 0x1C | BLK_EXP_UNF | R | bit j = exp_underflow of output block j |
| 0x20 | CYCLES | R | clock cycles of the last tile, START to DONE |
| 0x24 | TIMEOUT_LIM | RW | cycles before TIMEOUT (default 4096) |
| 0x30 | ID | R | 0xBF5A0001 (design id + version) |
| 0x34 | CFG0 | R | [7:0] ARRAY_SIZE, [15:8] BLOCK_MANTISSA_WIDTH, [23:16] EXP_WIDTH, [31:24] SCALE_FACTOR_WIDTH |
| 0x38 | CFG1 | R | [7:0] DATA_WIDTH, [15:8] MANTISSA_WIDTH, [23:16] FRAC_WIDTH, [31:24] BLOCK_WIDTH |

CFG0/CFG1 let the driver read the build's parameters, so one driver binary
can check it matches the bitstream.

## Vivado note

Vivado's block-design "module reference" needs a Verilog or VHDL top file, so
a thin `bfp_accel_axi_top.v` instantiates the SystemVerilog wrapper with flat
ports. Interface names follow the `s_axi_` / `s_axis_` / `m_axis_` convention,
so Vivado infers the AXI interfaces automatically.
