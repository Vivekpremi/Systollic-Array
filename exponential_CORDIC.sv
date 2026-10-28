`timescale 1ns/1ps
//=============================================================================
// exponential_CORDIC.sv  --  e^x for 0 <= x < 1, by shift-and-add
//
// WHAT IT DOES
//   The PE uses it to compute the antilog 2^f = e^(f * ln2).
//   Input  x          : FRAC_WIDTH fraction bits, 0 <= x < 1
//   Output exp_result : e^x with FRAC_WIDTH fraction bits (3 integer bits),
//                       1.0 <= e^x < e. In the PE x = f*ln2 < 0.7, so the
//                       result is 1.0 .. 2.0.
//
// HOW (one bit of x per clock, 8 steps)
//   Write x as a sum of halves: x = b1/2 + b2/4 + ... + b8/256 (its bits).
//   Then e^x = product of e^(1/2^k) for every bit bk that is 1.
//   Start with expx = 1.0. In step k (k = 1..8): if the remaining x is at
//   least 1/2^k, subtract 1/2^k from it and multiply expx by e^(1/2^k).
//   Each multiply is done with shifts and adds:
//       expx * c = expx + (expx >> s1) + (expx >> s2) + (expx >> s3)
//   with c ~ e^(1/2^k) coded in the table e[] (three 3-bit shift amounts,
//   0 = unused):
//       k  e^(1/2^k)   used                      value
//       1  1.6487      1 + 1/2 + 1/8 + 1/32      1.65625
//       2  1.2840      1 + 1/4 + 1/32            1.28125
//       3  1.1331      1 + 1/8 + 1/128           1.13281
//       4  1.0645      1 + 1/16                  1.0625
//       5  1.0317      1 + 1/32                  1.03125
//       6  1.0157      1 + 1/64                  1.01563
//       7  1.0079      1 + 1/128                 1.00781
//       8  1.0039      1 (no change)             1.0
//   (Despite the name, this is a shift-and-add method, not a rotation CORDIC.)
//
// TIMING / HANDSHAKE (FSM: init -> compute x8 -> out -> init)
//   init    : waits for valid_in, takes x
//   compute : 8 clocks, one step each
//   out     : 1 clock, valid_out = 1 and exp_result is valid
//   valid_out comes 9 clocks after the clock that took x. valid_in is only
//   seen in init: a new x while busy is ignored (the PE never sends one).
//   exp_result is 0 whenever valid_out is 0.
//
// LIMITATIONS
//   - Frac_WIDTH must be 8: the table has 8 entries and several constants are
//     written as 8/11-bit literals. Any other value fails elaboration.
//   - Step 8 does nothing (a 3-bit shift field cannot hold a shift of 8), so
//     results are up to 0.39% low when the last bit of x is 1. The other
//     constants are rounded too (see the table), and each shift rounds down.
//   - rst is ACTIVE LOW (the PE connects rst_n to it).
//=============================================================================
module exponential_CORDIC #(
parameter Frac_WIDTH = 8   // fraction bits of x and of the result; must be 8
)

(
    input wire clk,
    input wire rst,                       // active low
    input  wire valid_in,                 // x is valid (taken only when idle)
    input wire [Frac_WIDTH-1:0] x,        // exponent, 0 <= x < 1
    output wire valid_out,                // 1-clock pulse: exp_result is valid
    output wire [Frac_WIDTH + 2 :0] exp_result // e^x, 3 integer + Frac_WIDTH fraction bits
);
// state codes
localparam [1:0] init = 2'b00;
localparam [1:0] compute = 2'b01;
localparam [1:0] out = 2'b10;

// e[k-1] = shift amounts for the multiply by e^(1/2^k): {s1, s2, s3}, 3 bits
// each; a shift of 0 means "not used"
localparam [8:0] e [0:Frac_WIDTH-1] = '{
9'b001_011_101,// k=1: 1 + >>1 + >>3 + >>5
9'b010_101_000,// k=2: 1 + >>2 + >>5
9'b011_111_000,// k=3: 1 + >>3 + >>7
9'b100_000_000,// k=4: 1 + >>4
9'b101_000_000,// k=5: 1 + >>5
9'b110_000_000,// k=6: 1 + >>6
9'b111_000_000,// k=7: 1 + >>7
9'b000_000_000 // k=8: no change (>>8 does not fit 3 bits)

};
reg [1:0]state;

reg [$clog2(Frac_WIDTH)-1:0] iteration_counter; // step number - 1 (0..7)
reg [Frac_WIDTH-1:0] z;                          // the part of x still to be used
reg [Frac_WIDTH-1:0] power_of_two;               // 1/2^k for the current step (0.5, 0.25, ...)
reg signed [Frac_WIDTH+2:0] expx;                // running product, 3.8 fixed point (256 = 1.0)

always@(posedge clk or negedge rst) begin
    if(!rst) begin
        iteration_counter <= 3'b0;
        z <= 8'b0;
        power_of_two <= 8'b10000000;     // 0.5
        expx <= 11'b00100000000;         // 1.0
        state <= init;
    end
    else  begin
        case(state)
            init: begin
                z <= x;
                state <= (valid_in) ? compute : init;
                power_of_two <= 8'b10000000;   // 0.5
                expx <= 11'b00100000000;       // 1.0
                iteration_counter <= 3'b0;
            end
            compute: begin
        if(iteration_counter <= 3'b111) begin
            // this bit of x is 1: use it up and multiply by e^(1/2^k)
            if(power_of_two <= z) begin

                z <= z - power_of_two;
                expx <=   expx +
                          ((e[iteration_counter][8:6] == 3'b000)? 0 : (expx >> e[iteration_counter][8:6])) +
                          ((e[iteration_counter][5:3] == 3'b000)? 0 : (expx >> e[iteration_counter][5:3])) +
                          ((e[iteration_counter][2:0] == 3'b000)? 0 : (expx >> e[iteration_counter][2:0]));
            end
            power_of_two <= power_of_two >> 1;          // next bit: half the weight
            iteration_counter <= iteration_counter + 1;
            state <= (iteration_counter == 3'b111) ? out : compute;   // 8 steps done
        end

            end
            out: begin
                state <= init;     // result is shown for this one clock
            end
            default: state <= init;
        endcase
    end

end

assign exp_result = (valid_out) ? expx : 0;   // 0 except in the out clock
assign valid_out = (state == out);

endmodule

