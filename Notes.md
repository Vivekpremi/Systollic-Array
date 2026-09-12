# BFP–LNS Accelerator: Peripheral Pseudocode

Companion to the RTL in `top_module.sv` / `iface.sv` / `format_convertor.sv`. Each block below is one
module you can implement independently. HW modules are named to slot into `top_module`; SW modules
are named to slot into the host driver library.

---

## HW-1. Controller FSM (`ctrl.sv`) — replaces the `start_op` heuristic

```
module ctrl #(ARRAY_SIZE, EXP_WIDTH, ...)

// Register-mapped inputs (written by driver via AXI4-Lite, see HW-4)
input  cfg_M, cfg_N, cfg_K          // tile dims for THIS tile (<= ARRAY_SIZE)
input  cfg_start                    // pulse: begin this tile
input  cfg_tile_id                  // opaque tag, echoed back on done

// To/from datapath
input  edge_regs_loaded             // from HW-3, row+col edge registers filled
input  valid_out[ARRAY_SIZE][ARRAY_SIZE]  // from systolic_array
input  iface_mantissas_sent_out     // from iface
output array_valid_mask[ARRAY_SIZE] // masks unused rows/cols when M,N < ARRAY_SIZE
output drain_start                  // replaces old start_op, feeds iface.start
output status_done, status_tile_id, status_error

STATE = IDLE

on posedge clk:
  case STATE:
    IDLE:
      if cfg_start:
        latch M, N, K, tile_id
        array_valid_mask <= mask_for(M, N)     // 1s for real rows/cols, 0s for padding
        STATE <= WAIT_LOAD

    WAIT_LOAD:
      // HW-3 fills row/col edge registers; array can start on PE[0][0] as soon as
      // its two operands are valid, before the rest of the tile is loaded.
      if edge_regs_loaded:
        STATE <= COMPUTE

    COMPUTE:
      // completion = corner PE at (M-1, N-1) finished, not the fixed (ARRAY_SIZE-1) corner
      if valid_out[M-1][N-1]:
        STATE <= DRAIN
        drain_start <= 1

    DRAIN:
      drain_start <= 0
      if iface_mantissas_sent_out:            // all N columns of this tile drained
        status_done   <= 1
        status_tile_id <= tile_id
        STATE <= IDLE

  // watchdog: if WAIT_LOAD or COMPUTE exceed a timeout, raise status_error
  // instead of hanging forever (Known Issue: iface has no timeout today)
```

Key behaviors this owns that `top_module` currently fakes:
- Dimension-aware completion (`M-1`, `N-1` instead of a hardcoded corner).
- A clean **tile_id in → tile_id out** handshake so the driver can pipeline multiple tiles.
- A timeout/error path (`status_error`) instead of silent stall.

---

## HW-2. Input-side BFP decode (`bfp_decode.sv`) — wires the existing `descaling` block in

```
module bfp_decode #(BLOCK_MANTISSA_WIDTH=5, SCALE_FACTOR_WIDTH=4, EXP_WIDTH=8)

input  [HEADER_WIDTH-1:0] header      // tile_row, tile_col, out_row, block_exponent(8b), scale_factor(4b)
input  [4:0] packed_mantissa[3:0]     // 4 mantissas per block, per Notes packet format
input  packet_valid

output [MANTISSA_WIDTH-1:0] mantissa_log_domain[3:0]  // expanded, ready for mantissa_row_0/col_0
output [EXP_WIDTH-1:0] exponent_out[3:0]
output decode_valid

unpack header -> block_exponent, scale_factor
for i in 0..3:
    // undo the 5-bit quantization + scale_factor compression (this is NOT an FP conversion,
    // the value stays log-domain fixed point the whole time — see README Concern 1, step "Descaling")
    mantissa_log_domain[i] = descaling(packed_mantissa[i], scale_factor)
    exponent_out[i]        = block_exponent
decode_valid = packet_valid registered one cycle (matches descaling's latency)
```

Sits between HW-3 (edge registers) and `systolic_array.mantissa_row_0`/`mantissa_col_0`. This is the
missing half of the round trip flagged in the README (Known Issue #4).

---

## HW-3. Edge register banks (`edge_loader.sv`)

```
module edge_loader #(ARRAY_SIZE=4)

input  axis_tdata, axis_tvalid, axis_tlast   // from AXI4-Stream ingress (HW-5)
input  axis_row_or_col                       // 1 bit: routes this beat to row-bank or col-bank
output reg [31:0] row_reg[ARRAY_SIZE-1:0]    // one packed BFP block (Notes format) per row
output reg [31:0] col_reg[ARRAY_SIZE-1:0]
output edge_regs_loaded                       // -> ctrl.sv (HW-1)
output axis_tready

count = 0
on posedge clk:
  if axis_tvalid && axis_tready:
    if axis_row_or_col: row_reg[count_row] <= axis_tdata; count_row++
    else:                col_reg[count_col] <= axis_tdata; count_col++

edge_regs_loaded = (count_row == ARRAY_SIZE) && (count_col == ARRAY_SIZE)
// pipe row_reg[i]/col_reg[i] through bfp_decode (HW-2) combinationally / same cycle they land
```

This is what "hides load latency behind compute" from the roadmap: `ctrl.sv` can let `PE[0][0]`
start as soon as `row_reg[0]`/`col_reg[0]` are valid, rather than waiting for all 8 registers.

---

## HW-4. AXI4-Lite control register file (`regfile.sv`)

```
module regfile  // memory-mapped, host-visible

Registers (offsets are illustrative):
  0x00  CTRL        [start(1) | reserved]
  0x04  DIM_M       [8b]
  0x08  DIM_N       [8b]
  0x0C  DIM_K       [8b]
  0x10  TILE_ID     [16b]
  0x14  STATUS      [done(1) | error(1) | busy(1)]
  0x18  STATUS_TILE_ID [16b]   // echoed tile_id of most recently completed tile

on AXI4-Lite write to CTRL.start:
    pulse cfg_start -> ctrl.sv (HW-1)
on AXI4-Lite read of STATUS:
    return {status_error, status_done, busy}
    reading STATUS with done=1 clears status_done (read-to-clear), so driver can poll safely
```

Driver writes DIM_M/N/K + TILE_ID, then CTRL.start; polls (or gets interrupted on) STATUS.done.

---

## HW-5. AXI4-Stream ingress/egress wrapper (`top_shell.sv`)

```
module top_shell  // wraps top_module + ctrl + edge_loader + bfp_decode for FPGA/DMA

// Ingress: host DMA -> operand blocks -> edge_loader -> bfp_decode -> systolic_array
s_axis_tdata[31:0], s_axis_tvalid, s_axis_tlast, s_axis_tready  -> edge_loader (HW-3)

// Egress: systolic_array -> iface -> format_convertor -> AXI-Stream out -> host DMA
m_axis_tdata  = { header(12b), mantissa_out[19:0] }   // pack per Notes format
m_axis_tvalid = block_ready (from format_convertor)
m_axis_tlast  = last block of this tile

// Control plane
s_axi_lite <-> regfile (HW-4)
regfile.cfg_* -> ctrl.sv (HW-1) -> top_module / iface / format_convertor as today
```

This is the FPGA-testable unit: on Xilinx/Intel boards, `s_axis`/`m_axis` connect straight to
AXI-DMA or XDMA IP, and `s_axi_lite` connects to the same DMA IP's control port or a small AXI
interconnect. No further HW changes needed to get host-in/host-out working.

---

## SW-1. Public driver API (`accel.h` / `accel.c`)

```
// what the user actually calls
status accel_matmul(float* A, float* B, float* C, int M, int K, int N):
    plan = tile_plan(M, K, N, ARRAY_SIZE)          // SW-2
    accum = zeros(M, N)                             // host-side accumulator, fp32 or wide fixed

    for tile in plan.tiles:                          // SW-2 decides order (e.g. weight-stationary)
        A_block = fp_to_bfp(A, tile.a_slice)         // SW-3
        B_block = fp_to_bfp(B, tile.b_slice)

        transport_send(A_block, B_block, tile.dims, tile.id)   // SW-5
        result_block = transport_recv_blocking(tile.id)        // SW-5, matches STATUS_TILE_ID

        partial = bfp_to_fp(result_block)            // SW-4
        accum[tile.c_slice] += partial                // SW-6: accumulate across K-tiles

    C[:] = accum[:]
    return OK
```

---

## SW-2. Tiling scheduler (`tiling.c`)

```
struct Tile { a_slice, b_slice, c_slice, dims(M,K,N sub), id }

tile_plan(M, K, N, ARRAY_SIZE):
    tiles = []
    for m0 in range(0, M, ARRAY_SIZE):
      for n0 in range(0, N, ARRAY_SIZE):
        for k0 in range(0, K, ARRAY_SIZE):           // K-tiling drives SW-6 accumulation
            dims = ( min(ARRAY_SIZE, M-m0),
                     min(ARRAY_SIZE, K-k0),
                     min(ARRAY_SIZE, N-n0) )
            tiles.append(Tile(
                a_slice = A[m0:m0+dims.M, k0:k0+dims.K],
                b_slice = B[k0:k0+dims.K, n0:n0+dims.N],
                c_slice = C[m0:m0+dims.M, n0:n0+dims.N],
                dims    = dims,
                id      = next_tile_id()
            ))
    // ordering knob: group by (m0,n0) so all K-subtiles for one output tile run
    // consecutively -> lets SW-6 accumulate into one accum buffer before moving on
    return tiles
```

---

## SW-3. FP32 → BFP encoder (`codec_encode.c`)

```
fp_to_bfp(tensor, slice):                 // one block <= 4 elements at a time, per Notes format
    block = tensor[slice]
    max_exp = max(extract_exponent(x) for x in block)      // shared block exponent
    for x in block:
        aligned      = align_to(x, max_exp)                  // shift mantissa to shared exponent
        log_mantissa = log2_approx(aligned)                  // mirror log_finder's algorithm in SW
                                                              // (lead-one-detect + 8 frac bits)
        quantized[i], scale_factor = quantize_5bit(log_mantissa)
    pack header{tile_row, tile_col, out_row, max_exp, scale_factor} + quantized[0..3]
    return packet   // exact bit layout must match Notes.md / HW-2's unpack
```

Runs once per weight/activation block at load time — cheap relative to array reuse (see README
Concern 1). Can be vectorized (SIMD) since it's plain host code, no HW dependency.

---

## SW-4. BFP → FP32 decoder (`codec_decode.c`)

```
bfp_to_fp(packet):
    header, mantissas = unpack(packet)               // mirrors HW-2's unpack exactly
    for i in 0..3:
        linear_mantissa = 2 ** dequantize_5bit(mantissas[i], header.scale_factor)
        values[i] = linear_mantissa * (2 ** header.block_exponent)
    return values
```

Runs once per output block, at the very last layer only (small — see README's reuse table).
Intermediate layer-to-layer results should call `transport_send` again directly on the BFP
packet without ever routing through this function — decode only at the final host-visible edge.

---

## SW-5. Transport layer (`transport.c`) — talks to HW-5 over DMA

```
transport_send(a_block, b_block, dims, tile_id):
    write DIM_M/DIM_N/DIM_K, TILE_ID to regfile (HW-4) via mmap'd AXI-Lite BAR
    push a_block, b_block onto s_axis ring buffer (DMA descriptor, e.g. Xilinx AXI-DMA / XDMA)
    write CTRL.start = 1

transport_recv_blocking(tile_id):
    poll STATUS (or block on interrupt) until STATUS.done && STATUS_TILE_ID == tile_id
    read result packet off m_axis DMA ring buffer
    return packet

// FPGA bring-up note: use UIO or the vendor's XDMA char device for the mmap'd BAR;
// this same interface abstraction is what gets swapped for a real PCIe kernel driver later —
// SW-1..SW-4 don't need to change.
```

---

## SW-6. Cross-K-tile accumulation (`accumulate.c`)

```
// This is the piece not yet designed anywhere else: partial sums from separate K-subtiles
// of the SAME output tile must be summed on the host before the output tile is finalized.

accumulate(accum_buffer, partial, c_slice):
    accum_buffer[c_slice] += partial     // plain fp32 (or wider fixed-point) add, host-side

// Only re-encode to BFP (SW-3) AFTER all K-subtiles for a given (m0,n0) output tile have been
// summed — never re-encode/re-decode per K-subtile, or you erase the reuse argument from
// README Concern 1 (per-layer round-tripping through FP).
```

---

## Module dependency map

```
                        ┌───────────────────────────┐
 user code ── calls ──► │ SW-1 accel_matmul          │
                        └─────────────┬──────────────┘
                                      │
                 ┌────────────────────┼─────────────────────┐
                 ▼                    ▼                      ▼
          SW-2 tile_plan     SW-3 fp_to_bfp            SW-6 accumulate
                                      │                      ▲
                                      ▼                      │
                              SW-5 transport ───────► SW-4 bfp_to_fp
                                      │                      ▲
                          (DMA / AXI-Lite BAR)               │
                                      ▼                      │
                        ┌───────────────────────────┐        │
                        │ HW-5 top_shell            │        │
                        │  ┌─────────┐  ┌─────────┐ │        │
                        │  │HW-4     │  │HW-3     │ │        │
                        │  │regfile  │─►│edge_ldr │ │        │
                        │  └────┬────┘  └────┬────┘ │        │
                        │       ▼            ▼      │        │
                        │  ┌─────────┐  ┌─────────┐ │        │
                        │  │HW-1 ctrl│◄─┤HW-2 bfp │ │        │
                        │  │  (FSM)  │  │  decode │ │        │
                        │  └────┬────┘  └────┬────┘ │        │
                        │       ▼            ▼      │        │
                        │  systolic_array (existing)│        │
                        │       ▼                   │        │
                        │  iface -> format_convertor│        │
                        │  (existing)         ──────┼────────┘
                        └───────────────────────────┘
```

## Suggested build order

1. HW-1 (`ctrl.sv`) + HW-4 (`regfile.sv`) — smallest, unblocks everything else, replaces the
   fragile `start_op` heuristic immediately even before AXI wrapping exists.
2. HW-2 (`bfp_decode.sv`) wiring in the existing `descaling` module — closes the correctness gap
   (Known Issue #4) and lets you fix `tb_TOP_1.sv` to feed real BFP packets instead of raw ints.
3. HW-3 (`edge_loader.sv`) + HW-5 (`top_shell.sv`) — gets you to an AXI-Stream/AXI-Lite boundary
   you can actually attach DMA IP to on an FPGA.
4. SW-3/SW-4 (codec) — pure host code, can be written and unit-tested against a numpy/C reference
   completely independently of the HW work above.
5. SW-2 (`tiling.c`) + SW-6 (`accumulate.c`) — needs real M/N/K test cases to validate against.
6. SW-5 (`transport.c`) + SW-1 (`accel_matmul`) — wires everything together once 1–5 exist;
   this is your first true end-to-end FPGA test.
