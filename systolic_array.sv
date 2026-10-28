// NOTE: PE.sv is included here, so it must NOT also be compiled on its own
// (Vivado: set PE.sv to file type "Verilog Header"; Verilator: don't list it).
`include "PE.sv"
//=============================================================================
// systolic_array.sv  --  ARRAY_SIZE x ARRAY_SIZE grid of PEs (output-stationary)
//
// WHAT IT DOES
//   C = A x B for one tile. PE[i][j] computes C[i][j] = sum_k A[i][k]*B[k][j]
//   and keeps it in its own accumulator (output-stationary).
//     row data    : enters at PE[i][0] (left edge), moves RIGHT one PE per product
//     column data : enters at PE[0][j] (top edge), moves DOWN one PE per product
//   Each PE passes on a copy of its operands when it starts a product, and
//   raises valid_out_row / valid_out_col when the product is done; those
//   become the valid_in of its right / lower neighbour. So PE[i][j] starts
//   (i + j) product-times after PE[0][0]: a diagonal wave (about 10 clocks
//   per product).
//
// EDGES
//   The operands come from edge_reg_bank. Outside valid they are forced to 0.
//   set_i_ready[i] = "PE[i][0] finished a product": it tells the bank to show
//   the next element of row lane i and column lane i.
//
// END OF THE TILE
//   last_* marks the final element of a stream. A PE whose last product had
//   both last_row_in and last_col_in raises last_out, which is passed on like
//   the data. valid_out[i][j] = last_out of PE[i][j] = its Res_out is final.
//   last_hori[i] / last_vert[j] remember that the bank already showed the last
//   element: the bank moves past it (its last flag drops) before PE[i][0] or
//   PE[0][j] has finished that product.
//
// OUTPUTS
//   Res_out[i][j] / exp_out[i][j] : value = Res_out / 2^FRAC_WIDTH * 2^exp_out
//   valid_out[i][j]               : PE[i][j] has its final result (stays 1)
//   pe_overflow[i*ARRAY_SIZE+j]   : PE[i][j] overflow flag (sticky)
//
// NOTES
//   - Wire arrays are [ARRAY_SIZE:0] (one more than the PEs) so the last row
//     and last column of PEs have somewhere to send their outputs; those extra
//     entries are not used.
//   - Interior PE[i][j] needs valid from the left AND from the top in the same
//     clock; that holds because PE[i][j-1] and PE[i-1][j] are on the same
//     diagonal of the wave and finish together.
//   - Only PE[i][0] paces the bank. Using an AND of a whole row would deadlock
//     (the PEs of a row finish at different times, never all together).
//=============================================================================
module systolic_array #(
    parameter DATA_WIDTH = 32,      // PE product width (accumulator DATA_WIDTH+1)
    parameter EXP_WIDTH = 8,        // bits of a block exponent
    parameter MANTISSA_WIDTH = 16,  // bits of a PE log operand
    parameter FRAC_WIDTH = 8,       // fraction bits of the logs
    parameter ARRAY_SIZE = 4        // PEs per side
)
(
    input wire clk,
    input wire rst_n,
    // ---- from edge_reg_bank: current element of each lane ----
    input wire signed [MANTISSA_WIDTH-1:0] mantissa_col_0 [ARRAY_SIZE-1:0],   // column j operand (log)
    input wire signed [MANTISSA_WIDTH-1:0] mantissa_row_0 [ARRAY_SIZE-1:0],   // row i operand (log)
    input wire signed [EXP_WIDTH-1:0] exponent_col_0 [ARRAY_SIZE-1:0],
    input wire signed [EXP_WIDTH-1:0] exponent_row_0 [ARRAY_SIZE-1:0],
    input wire sign_col_0 [ARRAY_SIZE-1:0],
    input wire sign_row_0 [ARRAY_SIZE-1:0],
    input wire zero_col_0 [ARRAY_SIZE-1:0],
    input wire zero_row_0 [ARRAY_SIZE-1:0],
    input wire valid_in_row_0[ARRAY_SIZE-1:0],
    input wire valid_in_col_0[ARRAY_SIZE-1:0],
    input wire last_in_row_0[ARRAY_SIZE-1:0],
    input wire last_in_col_0[ARRAY_SIZE-1:0],

    output wire signed [DATA_WIDTH:0] Res_out [ARRAY_SIZE-1:0][ARRAY_SIZE-1:0],
    output wire signed [EXP_WIDTH:0] exp_out [ARRAY_SIZE-1:0][ARRAY_SIZE-1:0],
    output wire [ARRAY_SIZE-1:0] valid_out[ARRAY_SIZE-1:0],   // valid_out[i][j]: Res_out[i][j] is final
    output wire set_i_ready[ARRAY_SIZE-1:0],                  // to the bank: next element for lanes i
    output wire [ARRAY_SIZE*ARRAY_SIZE-1:0] pe_overflow       // PE[i][j] at bit i*ARRAY_SIZE+j
);
genvar i, j;
    // ---- operand meshes: [i][j] = input of PE[i][j]; *_out = what PE[i][j] passes on ----
    wire signed [MANTISSA_WIDTH-1:0] mantissa_col_i_j [ARRAY_SIZE:0][ARRAY_SIZE:0];
    wire signed [MANTISSA_WIDTH-1:0] mantissa_row_i_j [ARRAY_SIZE:0][ARRAY_SIZE:0];
    wire signed [MANTISSA_WIDTH-1:0] mantissa_col_i_j_out [ARRAY_SIZE:0][ARRAY_SIZE:0];
    wire signed [MANTISSA_WIDTH-1:0] mantissa_row_i_j_out [ARRAY_SIZE:0][ARRAY_SIZE:0];
    wire signed [EXP_WIDTH-1:0] exponent_col_j [ARRAY_SIZE:0][ARRAY_SIZE:0];
    wire signed [EXP_WIDTH-1:0] exponent_row_i [ARRAY_SIZE:0][ARRAY_SIZE:0];
    wire signed [EXP_WIDTH-1:0] exponent_col_j_out [ARRAY_SIZE:0][ARRAY_SIZE:0];
    wire signed [EXP_WIDTH-1:0] exponent_row_i_out [ARRAY_SIZE:0][ARRAY_SIZE:0];
    wire sign_col_i_j     [ARRAY_SIZE:0][ARRAY_SIZE:0];
    wire sign_row_i_j     [ARRAY_SIZE:0][ARRAY_SIZE:0];
    wire sign_col_i_j_out [ARRAY_SIZE:0][ARRAY_SIZE:0];
    wire sign_row_i_j_out [ARRAY_SIZE:0][ARRAY_SIZE:0];
    wire zero_col_i_j     [ARRAY_SIZE:0][ARRAY_SIZE:0];
    wire zero_row_i_j     [ARRAY_SIZE:0][ARRAY_SIZE:0];
    wire zero_col_i_j_out [ARRAY_SIZE:0][ARRAY_SIZE:0];
    wire zero_row_i_j_out [ARRAY_SIZE:0][ARRAY_SIZE:0];

    // ---- valid handshake between neighbours ----
    //   valid_in_row/col[i][j]  : valid arriving at PE[i][j] from the left / top
    //   valid_out_row/col[i][j] : PE[i][j] finished a product (to the right / below)
    wire valid_in_row[ARRAY_SIZE-1:0][ARRAY_SIZE-1:0];
    wire valid_in_col[ARRAY_SIZE-1:0][ARRAY_SIZE-1:0];
    wire valid_out_row[ARRAY_SIZE-1:0][ARRAY_SIZE-1:0];
    wire valid_out_col[ARRAY_SIZE-1:0][ARRAY_SIZE-1:0];
    // demand_pe[i][j]: PE[i][j] finished a product. Only column 0 is used
    // (set_i_ready); one driver per net.
    wire [ARRAY_SIZE-1:0] demand_pe [ARRAY_SIZE-1:0];

    // last_hori[i] / last_vert[j]: the bank has shown the last element of row
    // lane i / column lane j (held until reset)
    reg last_hori[ARRAY_SIZE-1:0];
    reg last_vert[ARRAY_SIZE-1:0];

    wire last_out[ARRAY_SIZE-1:0][ARRAY_SIZE-1:0];      // PE[i][j] has its final result
    wire last_row_in[ARRAY_SIZE-1:0][ARRAY_SIZE-1:0];   // last flag arriving from the left
    wire last_col_in[ARRAY_SIZE-1:0][ARRAY_SIZE-1:0];   // last flag arriving from the top

generate
    for(i = 0; i < ARRAY_SIZE; i = i + 1) begin : row_loop
        for(j = 0; j < ARRAY_SIZE; j = j + 1) begin : col_loop

PE #(.EXP_WIDTH(EXP_WIDTH), .MANTISSA_WIDTH(MANTISSA_WIDTH),
     .FRAC_WIDTH(FRAC_WIDTH), .DATA_WIDTH(DATA_WIDTH)) PE_i_j (
    .clk(clk),
    .rst_n(rst_n),
    .valid_in(valid_in_row[i][j] && valid_in_col[i][j]), // operands from the left AND the top
    .mantissa_col_i_j(mantissa_col_i_j[i][j]),
    .exponent_col_j(exponent_col_j[i][j]),
    .mantissa_row_i_j(mantissa_row_i_j[i][j]),
    .exponent_row_i(exponent_row_i[i][j]),
    .sign_col_i_j(sign_col_i_j[i][j]),
    .sign_row_i_j(sign_row_i_j[i][j]),
    .zero_col_i_j(zero_col_i_j[i][j]),
    .zero_row_i_j(zero_row_i_j[i][j]),
    .last_row_in(last_row_in[i][j]),
    .last_col_in(last_col_in[i][j]),
    .mantissa_row_i_j_out(mantissa_row_i_j_out[i][j]),
    .mantissa_col_i_j_out(mantissa_col_i_j_out[i][j]),
    .exponent_col_j_out(exponent_col_j_out[i][j]),
    .exponent_row_i_out(exponent_row_i_out[i][j]),
    .sign_col_i_j_out(sign_col_i_j_out[i][j]),
    .sign_row_i_j_out(sign_row_i_j_out[i][j]),
    .zero_col_i_j_out(zero_col_i_j_out[i][j]),
    .zero_row_i_j_out(zero_row_i_j_out[i][j]),
    .Res_out(Res_out[i][j]),
    .valid_out_row_i(valid_out_row[i][j]),
    .valid_out_col_j(valid_out_col[i][j]),
    .last_out(last_out[i][j]),
    .demand_from_mem(demand_pe[i][j]) ,
    .overflow_out(pe_overflow[i*ARRAY_SIZE+j]),
    .exp_out(exp_out[i][j])
);
// pass the operands on: column data down, row data right
assign mantissa_col_i_j[i+1][j] = mantissa_col_i_j_out[i][j];
assign exponent_col_j[i+1][j] = exponent_col_j_out[i][j];
assign mantissa_row_i_j[i][j+1] = mantissa_row_i_j_out[i][j];
assign exponent_row_i[i][j+1] = exponent_row_i_out[i][j];
assign sign_col_i_j[i+1][j] = sign_col_i_j_out[i][j];
assign sign_row_i_j[i][j+1] = sign_row_i_j_out[i][j];
assign zero_col_i_j[i+1][j] = zero_col_i_j_out[i][j];
assign zero_row_i_j[i][j+1] = zero_row_i_j_out[i][j];


assign valid_out[i][j] = last_out[i][j];   // final result ready
// valid into PE[i][j]: from the bank on the edges, from the neighbour inside
assign valid_in_row[i][j] = (j == 0) ? valid_in_row_0[i] : valid_out_row[i][j-1];
assign valid_in_col[i][j] = (i == 0) ? valid_in_col_0[j] : valid_out_col[i-1][j];

// last into PE[i][j]: on the edges the bank's flag (or that it was already
// shown), inside the neighbour's last_out
assign last_row_in[i][j] = (j == 0) ? ((last_in_row_0[i])? 1 : last_hori[i]) : last_out[i][j-1];
assign last_col_in[i][j] = (i == 0) ? ((last_in_col_0[j])? 1 : last_vert[j]) : last_out[i-1][j];
        end


// remember that lane i showed its last element (row and column lane)
always @(posedge clk or negedge rst_n) begin
    if(!rst_n) begin
        last_hori[i] <= 0;
        last_vert[i] <= 0;
    end else begin
        if(last_in_row_0[i]) last_hori[i] <= 1;
        else last_hori[i] <= last_hori[i];

        if(last_in_col_0[i]) last_vert[i] <= 1;
        else last_vert[i] <= last_vert[i];
    end
end

assign set_i_ready[i] = demand_pe[i][0];   // entry PE [i][0] paces row i and column i of the bank

// (same as the j == 0 / i == 0 cases above; repeated, same value)
assign valid_in_row[i][0] = valid_in_row_0[i];
assign valid_in_col[0][i] = valid_in_col_0[i];


// edge operands: the bank's values while valid, 0 otherwise
assign sign_col_i_j[0][i]     = (valid_in_col[0][i])? sign_col_0[i] : 1'b0;
assign sign_row_i_j[i][0]     = (valid_in_row[i][0])? sign_row_0[i] : 1'b0;
assign zero_col_i_j[0][i]     = (valid_in_col[0][i])? zero_col_0[i] : 1'b0;
assign zero_row_i_j[i][0]     = (valid_in_row[i][0])? zero_row_0[i] : 1'b0;
assign mantissa_col_i_j[0][i] = (valid_in_col[0][i])? mantissa_col_0[i] : 0;
assign exponent_col_j[0][i] = (valid_in_col[0][i])? exponent_col_0[i] : 0;
assign mantissa_row_i_j[i][0] = (valid_in_row[i][0])? mantissa_row_0[i] : 0;
assign exponent_row_i[i][0] = (valid_in_row[i][0])? exponent_row_0[i] : 0;

    end

endgenerate



endmodule

