`timescale 1ns/1ps
//=============================================================================
// iface.sv  --  array results -> format_convertor, one column at a time
//
// WHAT IT DOES
//   For each column j of C (j = 0, 1, 2, 3 in order):
//     1. waits for `start` from top_module,
//     2. waits until every PE of column j has its final result,
//     3. sends the column's results to format_convertor, TWO per clock:
//        (row 0, row 1), then (row 2, row 3), ...
//   Each result is the pair (Res_out, exp_out) of one PE; its value is
//   Res_out / 2^FRAC_WIDTH * 2^exp_out.
//
// FSM:  WAIT_FOR_START -> WAIT_COLUMN -> SEND_MANTISSAS -> WAIT_FOR_START
//   WAIT_FOR_START : wait for start
//   WAIT_COLUMN    : wait for col_ready (all valid_column[r][col_idx] = 1)
//   SEND_MANTISSAS : one pair per clock; mantissas_sent_out marks the last
//                    pair; then col_idx moves to the next column
//
// TIMING OF THE OUTPUTS
//   valid_man_out = 1 while a pair is on mantissa_out_1/2 and exp_m_out_1/2.
//   mantissas_sent_out = 1 together with the LAST pair of the column.
//
// LIMITATIONS
//   - ARRAY_SIZE must be EVEN and >= 4: results go in pairs (it reads
//     [row_idx+1] and steps row_idx by 2). At ARRAY_SIZE = 2, row_idx is
//     1 bit and row_idx + 2 does not fit.
//   - Columns are sent once each, 0 to ARRAY_SIZE-1; a new tile needs a reset.
//   - SEND_MANTISSAS does not check valid again: WAIT_COLUMN already did, and
//     valid_out stays high once set.
//=============================================================================
module iface#(
    parameter DATA_WIDTH = 32,                       // Res_out has DATA_WIDTH+1 bits
    parameter ARRAY_SIZE = 4,                        // rows = results per column (even, >= 4)
    parameter LOG2_ARRAY_SIZE = $clog2(ARRAY_SIZE),  // bits of a row / column index
    parameter EXP_WIDTH = 8                          // exp_out has EXP_WIDTH+1 bits
)(
    input wire clk,
    input wire rst_n,
    input wire start,                                // top_module: go on with the next column
    input wire [ARRAY_SIZE-1:0] valid_column [ARRAY_SIZE-1:0],              // valid_column[r][c]: PE[r][c] final
    input wire signed [DATA_WIDTH:0] mantissa[ARRAY_SIZE-1:0][ARRAY_SIZE-1:0], // Res_out of every PE
    input wire signed [EXP_WIDTH:0] exponent[ARRAY_SIZE-1:0][ARRAY_SIZE-1:0],  // exp_out of every PE

    output wire signed [DATA_WIDTH:0] mantissa_out_1,mantissa_out_2,   // Res_out of rows row_idx, row_idx+1
    output wire signed [EXP_WIDTH:0]  exp_m_out_1, exp_m_out_2,        // their exp_out
    output wire valid_man_out,                                         // a pair is on the outputs
    output wire mantissas_sent_out                                     // this pair is the column's last
);

reg signed [DATA_WIDTH:0] mantissa_out_1_q,mantissa_out_2_q;
reg signed [EXP_WIDTH:0]  exp_m_out_1_q, exp_m_out_2_q;
reg valid_man_out_q;

reg [1:0] state;
reg [$clog2(ARRAY_SIZE)-1:0] col_idx;   // column now being handled (0..ARRAY_SIZE-1)
reg [$clog2(ARRAY_SIZE)-1:0] row_idx;   // first row of the next pair to send
reg last_mantissa_sent;                 // the pair just put out is the last of the column
localparam [1:0] WAIT_FOR_START = 2'b00,
                 WAIT_COLUMN    = 2'b01,
                 SEND_MANTISSAS = 2'b10;


// col_ready = every PE in the current column has its final result
// (valid_column stays high once set, until reset)
reg col_ready;
integer r;
always @(*) begin
    col_ready = 1'b1;
    for (r = 0; r < ARRAY_SIZE; r = r + 1)
        if (!valid_column[r][col_idx]) col_ready = 1'b0;
end

always @(posedge clk or negedge rst_n) begin
    if(!rst_n) begin
        state <= WAIT_FOR_START;
        mantissa_out_1_q <= 0;
        mantissa_out_2_q <= 0;
        exp_m_out_1_q <= 0;
        exp_m_out_2_q <= 0;
        valid_man_out_q <= 0;
        col_idx <= 0;
        row_idx <= 0;
        last_mantissa_sent <= 0;
    end
    else begin
        case(state)
            WAIT_FOR_START: begin
                if(start) begin
                    state <= WAIT_COLUMN;
                end
                else state <= WAIT_FOR_START;

            end

            WAIT_COLUMN: begin
                if(col_ready) begin
                    state <= SEND_MANTISSAS;
                    row_idx <= 0;
                end
            end

            // one pair per clock; when the last pair is out (mantissas_sent_out),
            // clear the outputs and move to the next column
            SEND_MANTISSAS: begin
                    mantissa_out_1_q <= mantissa[row_idx][col_idx];
                    mantissa_out_2_q <= mantissa[row_idx + 1][col_idx];
                    exp_m_out_1_q <= exponent[row_idx][col_idx];
                    exp_m_out_2_q <= exponent[row_idx + 1][col_idx];
                    valid_man_out_q <= 1;
                    row_idx <= row_idx + 2;
                    last_mantissa_sent <= (row_idx == (LOG2_ARRAY_SIZE)'(ARRAY_SIZE - 2));   // this is the last pair
                    if(mantissas_sent_out) begin
                        col_idx <= col_idx + 1;
                        row_idx <= 0;
                        state <= WAIT_FOR_START;
                        valid_man_out_q <= 0;
                        mantissa_out_1_q <= 0;
                        mantissa_out_2_q <= 0;

                    end
                end
            default : state <= WAIT_FOR_START;

        endcase
    end
end

assign exp_m_out_1 = exp_m_out_1_q;
assign exp_m_out_2 = exp_m_out_2_q;
assign mantissa_out_1 = mantissa_out_1_q;
assign mantissa_out_2 = mantissa_out_2_q;
assign valid_man_out = valid_man_out_q;
assign mantissas_sent_out = last_mantissa_sent;
endmodule

