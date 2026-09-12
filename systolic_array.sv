`include "PE.sv"
module systolic_array #(
    parameter DATA_WIDTH = 32,
    parameter EXP_WIDTH = 8,
    parameter MANTISSA_WIDTH = 16,
    parameter ARRAY_SIZE = 4
)
(
    input wire clk,
    input wire rst_n,
    input wire [MANTISSA_WIDTH-1:0] mantissa_col_0 [ARRAY_SIZE-1:0],
    input wire [MANTISSA_WIDTH-1:0] mantissa_row_0 [ARRAY_SIZE-1:0],
    input wire [EXP_WIDTH-1:0] exponent_col_0 [ARRAY_SIZE-1:0],
    input wire [EXP_WIDTH-1:0] exponent_row_0 [ARRAY_SIZE-1:0],
    input wire valid_in_row_0[ARRAY_SIZE-1:0],
    input wire valid_in_col_0[ARRAY_SIZE-1:0],
    input wire last_in_row_0[ARRAY_SIZE-1:0],
    input wire last_in_col_0[ARRAY_SIZE-1:0],

    output wire [DATA_WIDTH-1:0] Res_out [ARRAY_SIZE-1:0][ARRAY_SIZE-1:0],
    output wire [7:0] exp_out [ARRAY_SIZE-1:0][ARRAY_SIZE-1:0],
    output wire [ARRAY_SIZE-1:0] valid_out[ARRAY_SIZE-1:0],
    output wire set_i_ready[ARRAY_SIZE-1:0],
    output wire demand_from_mem[ARRAY_SIZE-1:0]
);
//16 X 16 systolic array using genvar
genvar i, j;
    wire [MANTISSA_WIDTH-1:0] mantissa_col_i_j [ARRAY_SIZE:0][ARRAY_SIZE:0];
    wire [MANTISSA_WIDTH-1:0] mantissa_row_i_j [ARRAY_SIZE:0][ARRAY_SIZE:0];
    wire [MANTISSA_WIDTH-1:0] mantissa_col_i_j_out [ARRAY_SIZE:0][ARRAY_SIZE:0];
    wire [MANTISSA_WIDTH-1:0] mantissa_row_i_j_out [ARRAY_SIZE:0][ARRAY_SIZE:0];
    wire [EXP_WIDTH-1:0] exponent_col_j [ARRAY_SIZE:0][ARRAY_SIZE:0];
    wire [EXP_WIDTH-1:0] exponent_row_i [ARRAY_SIZE:0][ARRAY_SIZE:0];
    wire [EXP_WIDTH-1:0] exponent_col_j_out [ARRAY_SIZE:0][ARRAY_SIZE:0];
    wire [EXP_WIDTH-1:0] exponent_row_i_out [ARRAY_SIZE:0][ARRAY_SIZE:0];
  
    wire valid_in_row[ARRAY_SIZE-1:0][ARRAY_SIZE-1:0];
    wire valid_in_col[ARRAY_SIZE-1:0][ARRAY_SIZE-1:0];
    wire valid_out_row[ARRAY_SIZE-1:0][ARRAY_SIZE-1:0];
    wire valid_out_col[ARRAY_SIZE-1:0][ARRAY_SIZE-1:0];
    wire valid_condition[ARRAY_SIZE-1:0][ARRAY_SIZE-1:0];

    reg last_hori[ARRAY_SIZE-1:0];
    reg last_vert[ARRAY_SIZE-1:0];

    wire last_out[ARRAY_SIZE-1:0][ARRAY_SIZE-1:0];
    wire last_row_in[ARRAY_SIZE-1:0][ARRAY_SIZE-1:0];
    wire last_col_in[ARRAY_SIZE-1:0][ARRAY_SIZE-1:0];

generate    
    for(i = 0; i < ARRAY_SIZE; i = i + 1) begin : row_loop
        for(j = 0; j < ARRAY_SIZE; j = j + 1) begin : col_loop

PE PE_i_j (
    .clk(clk),
    .rst_n(rst_n),
    .valid_in(valid_in_row[i][j] && valid_in_col[i][j]), // Valid when both top and left neighbors are valid
    .mantissa_col_i_j(mantissa_col_i_j[i][j]),
    .exponent_col_j(exponent_col_j[i][j]),
    .mantissa_row_i_j(mantissa_row_i_j[i][j]),
    .exponent_row_i(exponent_row_i[i][j]),
    .last_row_in(last_row_in[i][j]),
    .last_col_in(last_col_in[i][j]),
    .mantissa_row_i_j_out(mantissa_row_i_j_out[i][j]),
    .mantissa_col_i_j_out(mantissa_col_i_j_out[i][j]),
    .exponent_col_j_out(exponent_col_j_out[i][j]),
    .exponent_row_i_out(exponent_row_i_out[i][j]),
    .Res_out(Res_out[i][j]),
    .valid_out_row_i(valid_out_row[i][j]),
    .valid_out_col_j(valid_out_col[i][j]),
    .last_out(last_out[i][j]),
    .demand_from_mem(demand_from_mem[i]) ,
    .exp_out(exp_out[i][j])
);
assign mantissa_col_i_j[i+1][j] = mantissa_col_i_j_out[i][j];
assign exponent_col_j[i+1][j] = exponent_col_j_out[i][j];
assign mantissa_row_i_j[i][j+1] = mantissa_row_i_j_out[i][j];
assign exponent_row_i[i][j+1] = exponent_row_i_out[i][j];


//assign valid_condition[i][j] = valid_out_row[i][j] && valid_out_col[i][j] && last_hori[i] && last_vert[j]; // Condition for output to be valid
assign valid_out[i][j] = last_out[i][j]; // Output is valid when both row and column outputs are valid
assign valid_in_row[i][j] = (j == 0) ? valid_in_row_0[i] : valid_out_row[i][j-1]; // First column takes input from valid_in_row_0, others take from left neighbor
assign valid_in_col[i][j] = (i == 0) ? valid_in_col_0[j] : valid_out_col[i-1][j]; // First row takes input from valid_in_col_0, others take from top neighbor

assign last_row_in[i][j] = (j == 0) ? ((last_in_row_0[i])? 1 : last_hori[i]) : last_out[i][j-1]; // First column takes last signal from last_in_row_0, others take from left neighbor
assign last_col_in[i][j] = (i == 0) ? ((last_in_col_0[j])? 1 : last_vert[j]) : last_out[i-1][j]; // First row takes last signal from last_in_col_0, others take from top neighbor
        end


always @(posedge clk or negedge rst_n) begin
    if(!rst_n) begin
        last_hori[i] <= 0;
        last_vert[i] <= 0;
    end else begin
        if(last_in_row_0[i]) last_hori[i] <= 1;
        else last_hori[i] <= last_hori[i]; // Hold the last signal until the end of the row
        
        if(last_in_col_0[i]) last_vert[i] <= 1;
        else last_vert[i] <= last_vert[i]; // Hold the last signal until the end of the column
    end
end

assign set_i_ready[i] = demand_from_mem[i]; // Set ready when both row and column inputs are valid

assign valid_in_row[i][0] = valid_in_row_0[i]; 
assign valid_in_col[0][i] = valid_in_col_0[i]; 


assign mantissa_col_i_j[0][i] = (valid_in_col[0][i])? mantissa_col_0[i] : 0;
assign exponent_col_j[0][i] = (valid_in_col[0][i])? exponent_col_0[i] : 0;
assign mantissa_row_i_j[i][0] = (valid_in_row[i][0])? mantissa_row_0[i] : 0;
assign exponent_row_i[i][0] = (valid_in_row[i][0])? exponent_row_0[i] : 0;

    end

endgenerate



endmodule

