`timescale 1ns/1ps

module exponential_CORDIC #(
parameter Int_WIDTH = 9, // Integer width
parameter Frac_WIDTH = 8,  // Fractional width 
parameter DATA_WIDTH = 17 // Total width
)

(
    input wire clk,
    input wire rst,
    input  wire valid_in,
    input wire [Frac_WIDTH-1:0] x,
    output wire valid_out,
    output wire [Frac_WIDTH + 2 :0] exp_result // Output exponential result (fixed-point representation)
);
parameter init = 2'b00;
parameter compute = 2'b01;
parameter out = 2'b10;

localparam [8:0] e [0:7] = '{
9'b001_011_101,//>>0 +>>1 +>>3 +>>5
9'b010_101_000,//>>0 +>>2 +>>5     
9'b011_111_000,//>>0 +>>3 +>>7     
9'b100_000_000,//>>0 +>>4          
9'b101_000_000,//>>0 +>>5          
9'b110_000_000,//>>0 +>>6          
9'b111_000_000,//>>0 +>>7          
9'b000_000_000 //>>0 only, no-op   

}; // Precomputed constants for CORDIC iterations
reg [1:0]state;
//assign initial values
//reg [18:0] e[0:7];

reg [2:0] iteration_counter; // Counter for iterations
reg [Frac_WIDTH-1:0] z; // fraction(x)
reg [Frac_WIDTH-1:0] power_of_two; // 2^-iteration_counter
reg signed [Frac_WIDTH+2:0] expx;
always@(posedge clk or negedge rst) begin
    if(!rst) begin
        iteration_counter <= 3'b0;
        z <= 8'b0;
        power_of_two <= 8'b10000000; // Initialize power_of_two to 0.5 in fixed-point representation
        expx <= 11'b00100000000; // Initialize expx to 1 in fixed-point representation 4.8
        state <= init; // Start in the initialization state
    end
    else  begin
        // CORDIC algorithm for computing exponential
        case(state)
            init: begin
                z <= x; // Initialize z to input x
                state <= (valid_in) ? compute : init; // Move to compute state if input is valid
                power_of_two <= 8'b10000000;
                expx <= 11'b00100000000;
                iteration_counter <= 3'b0;
// Format per chunk: [S | kkkkk]
// S = 0 → add, 1 → subtract
// k = shift amount (0–31)
// 4 chunks per constant exept for first → 19 bits
//e[0] <= 19'b1_000010_100110_101000; // e^-1/2   = +1,+2,-6,-8
//e[1] <= 19'b1_000010_000101_101001; // e^-1/4   = +1,+2,+5,-9
//e[2] <= 19'b1_000010_000011_000111; // e^-1/8   = +1,+2,+3,+7
//e[3] <= 19'b1_000010_000011_000100; // e^-1/16  = +1,+2,+3,+4
//e[4] <= 19'b0_100101_001010_000000; // e^-1/32  = +0,-5,+10  
//e[5] <= 19'b0_100110_001100_000000; // e^-1/64  = +0,-6,+12  
//e[6] <= 19'b0_100111_001110_000000; // e^-1/128 = +0,-7,+14  
//e[7] <= 19'b0_101000_010000_000000; // e^-1/256 = +0,-8,+16  
                /////////////each consecutive set of  bits contsains the shift amount////////////////////
            end
            compute: begin
        if(iteration_counter <= 7) begin
            if(power_of_two <= z) begin
                
                z <= z - power_of_two; // Update z
                expx <=   expx + 
                          ((e[iteration_counter][8:6] == 3'b000)? 0 : (expx >> e[iteration_counter][8:6])) +
                          ((e[iteration_counter][5:3] == 3'b000)? 0 : (expx >> e[iteration_counter][5:3])) +
                          ((e[iteration_counter][2:0] == 3'b000)? 0 : (expx >> e[iteration_counter][2:0])); // Update expx
            end
            power_of_two <= power_of_two >> 1; // Update power_of_two for the next iteration
            iteration_counter <= iteration_counter + 1; // Increment iteration counter '
            state <= (iteration_counter == 7) ? out : compute; // Move to output state after last iteration
        end
    
            end
            out: begin
                 // Output the final exponential result
                state <= init; // Indicate that the result is valid
            end
            default: state <= init; // Default case to reset state
        endcase
        // Implementation details would go here
    end
  
end

assign exp_result = (valid_out) ? expx : 0; // Assign the computed exponential result to the output
assign valid_out = (state == out); // Output is valid when in the 'out' state

endmodule

