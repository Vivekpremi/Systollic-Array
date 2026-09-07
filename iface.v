`timescale 1ns/1ps

module iface(
    input wire clk,
    input wire rst_n,
    input wire start,
    input wire [15:0] valid_column [15:0],
    input wire [31:0] mantissa[15:0][15:0],
    input wire [7:0] exponent[15:0][15:0],
    input wire max_exp_calculated,

    output wire valid_exp_out,      
    output wire [7:0] exp_out_1,
    output wire [7:0] exp_out_2,      
    output wire [31:0] mantissa_out_1,mantissa_out_2,
    output wire [7:0]  exp_m_out_1, exp_m_out_2,
    output wire valid_man_out,
    output wire [1:0] state_out,
    output wire last_exp_sent,
    output wire mantissas_sent_out
);

genvar i,j;

reg  valid_exp_out_q;
reg [7:0] exp_out_1_q, exp_out_2_q;
reg [31:0] mantissa_out_1_q,mantissa_out_2_q;
reg [7:0]  exp_m_out_1_q, exp_m_out_2_q;
reg valid_man_out_q;

//FSM to control the flow of data
// first when you recieve start, you start sending exponents when they get ready one by one,
// when max_exp is calculated you recieve a signal for that and then you start sending mantissas 2 at a time with a valid
//you keep sending until you have sent all the mantissas, then you wait for the next start signal

// so stages are 
// WAIT_FOR_START
// SEND_EXPONENTS until max_exp_calculated is high
// SEND_MANTISSAS until all mantissas are sent

reg [1:0] state;
reg [3:0] col_idx; // To keep track of which column's mantissas are being sent
reg [3:0] row_idx; // To keep track of which row's mantissas are being sent
reg last_exp_sent_q;
reg last_mantissa_sent;
parameter WAIT_FOR_START = 2'b00,
          SEND_EXPONENTS = 2'b01,
          SEND_MANTISSAS = 2'b10;
          

always @(posedge clk or negedge rst_n) begin
    if(!rst_n) begin
        state <= WAIT_FOR_START;
        valid_exp_out_q <= 0;
        exp_out_1_q <= 0;
        exp_out_2_q <= 0;
        mantissa_out_1_q <= 0;
        mantissa_out_2_q <= 0;
        exp_m_out_1_q <= 0;
        exp_m_out_2_q <= 0;
        valid_man_out_q <= 0;
        col_idx <= 0;
        row_idx <= 0;
        last_exp_sent_q <= 0;
        last_mantissa_sent <= 0;
    end
    else begin
        case(state)
            WAIT_FOR_START: begin
                if(start) begin // If any start signal is high
                    state <= SEND_EXPONENTS;
                end
                else state <= WAIT_FOR_START;

            end

            SEND_EXPONENTS: begin
                if(max_exp_calculated) begin
                    state <= SEND_MANTISSAS;
                    row_idx <= 0; // Reset row index for mantissa sending
                    exp_out_1_q <= 0; // Clear exponent output
                    exp_out_2_q <= 0;
                    valid_exp_out_q <= 0; // Clear exponent valid signal
                    last_exp_sent_q <= 0; // Clear last exponent sent signal
                end else begin
                    if(valid_column[row_idx][col_idx] && valid_column[row_idx + 1][col_idx]) begin // If any valid signal is high
                        valid_exp_out_q <= 1; // Valid signal for exponent output
                        exp_out_1_q <= exponent[row_idx][col_idx]; // Send exponents one by one (for simplicity, sending the first one here)
                        exp_out_2_q <= exponent[row_idx + 1][col_idx]; // Send the second exponent
                        row_idx <= row_idx + 2; // Move to the next row
                        last_exp_sent_q <= (row_idx == 14) ; // Set last_exp_sent when we've sent the last exponent
                    end
                    else begin
                        valid_exp_out_q <= 0; // No valid exponent to send
                        exp_out_1_q <= 0;
                        exp_out_2_q <= 0;
                        last_exp_sent_q <= 0;
                    end
                end
            end

            SEND_MANTISSAS: begin
                    mantissa_out_1_q <= mantissa[row_idx][col_idx];
                    mantissa_out_2_q <= mantissa[row_idx + 1][col_idx];
                    exp_m_out_1_q <= exponent[row_idx][col_idx];
                    exp_m_out_2_q <= exponent[row_idx + 1][col_idx];
                    valid_man_out_q <= 1;
                    row_idx <= row_idx + 2; // Move to the next two rows
                    last_mantissa_sent <= (row_idx == 14); // Set last_mantissa_sent when we've sent the last pair of mantissas
                    if(mantissas_sent_out) begin // If we've sent all mantissas for the current column
                        col_idx <= col_idx + 1; // Move to the next column
                        row_idx <= 0; // Reset row index for the new column
                        state <= WAIT_FOR_START; // Go back to waiting for the next start signal
                        valid_man_out_q <= 0; // Clear mantissa valid signal
                        mantissa_out_1_q <= 0; // Clear mantissa output
                        mantissa_out_2_q <= 0;
                        
                    end
                end
            default : state <= WAIT_FOR_START;
                
        endcase
    end
end

assign valid_exp_out = valid_exp_out_q;
assign exp_out_1 = exp_out_1_q;
assign exp_out_2 = exp_out_2_q;
assign exp_m_out_1 = exp_m_out_1_q;
assign exp_m_out_2 = exp_m_out_2_q;
assign mantissa_out_1 = mantissa_out_1_q;
assign mantissa_out_2 = mantissa_out_2_q;
assign valid_man_out = valid_man_out_q;
assign state_out = state;
assign last_exp_sent = last_exp_sent_q;
assign mantissas_sent_out = last_mantissa_sent;
endmodule
