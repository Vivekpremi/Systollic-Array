// HARI OM

`timescale 1ns/1ps

module format_convertor #(
    parameter ARRAY_SIZE = 4,
    parameter BLOCK_MANTISSA_WIDTH = 5,
    parameter EXP_WIDTH = 8,
    parameter SCALE_FACTOR_WIDTH = 4
)(
    input clk,
    input rst_n,
    input [31:0] mant0,
    input [31:0] mant1,
    input [EXP_WIDTH-1:0] exp_m_1,
    input [EXP_WIDTH-1:0] exp_m_2,
    input mantissa_valid,
    input all_mantissas_sent,
    input [EXP_WIDTH-1:0] exp0,
    input [EXP_WIDTH-1:0] exp1,
    input exp_valid,
    input last_exp_sent,

    
    output [EXP_WIDTH-1:0] max_exp,
    output [SCALE_FACTOR_WIDTH-1:0] scale_factor,
    output [ARRAY_SIZE * BLOCK_MANTISSA_WIDTH - 1:0] mantissa_out,
    output block_ready,
    output reg max_exp_calculated
);

reg state;
// calc exp regs 
reg  [EXP_WIDTH-1:0] max_exp_int;
reg [$clog2(ARRAY_SIZE)-1:0] idx;
// find scale factor regs
reg [31:0] max_mant_int;
reg scale_factor_ready;

parameter  CALC_MAX_EXP = 1'b0, FIND_SCALE_FACTOR = 1'b1;

always @(posedge clk or negedge rst_n) begin
    if (!rst_n) begin
        state <= CALC_MAX_EXP;
        max_exp_int <= 0;
        max_mant_int <= 0;
        idx <= 0;
        scale_factor_ready <= 0;
        max_exp_calculated <= 0;
    end
    else begin
        case (state)
            CALC_MAX_EXP: begin
                    if(exp_valid) begin
                        if (last_exp_sent) begin
                        state <= FIND_SCALE_FACTOR;
                        max_exp_calculated <= 1;
                    end
                    
                        max_exp_int <= (exp0 > exp1) ? (exp0 > max_exp_int ? exp0 : max_exp_int) : 
                                       (exp1 > max_exp_int ? exp1 : max_exp_int);
                    

                    end
                    else begin
                        max_exp_int <= max_exp_int;
                        state <= CALC_MAX_EXP;
                        max_exp_calculated <= 0;
                    end
                    scale_factor_ready <= 0; // Reset scale factor ready when calculating max exp
             
            end
            FIND_SCALE_FACTOR: begin
                max_exp_calculated <= 0;
                if(mantissa_valid) begin
                
                if (all_mantissas_sent) begin
                    state <= CALC_MAX_EXP;
                    max_exp_int <= 0; // Reset max exp for next block
                    scale_factor_ready <= 1;
                    idx <= 0; // Reset index for mantissa block
                end
                else idx <= idx + 1;
                
                    max_mant_int <= (log_mant0 > log_mant1) ? (log_mant0 > max_mant_int) ? log_mant0 : max_mant_int :
                                    (log_mant1 > max_mant_int) ? log_mant1 : max_mant_int;

                    mant_block[idx*2]     <= log_mant0;
                    mant_block[idx*2 + 1] <= log_mant1;

                    

                end
                else begin
                    max_mant_int <= max_mant_int;
                    state <= FIND_SCALE_FACTOR;
                    
                end
            end
        endcase
    end
end

reg [31:0] mant_block [15:0];

// mantissa shifting
wire [31:0] shifted_mant0;
wire [31:0] shifted_mant1;
wire [31:0] log_mant0;
wire [31:0] log_mant1;

wire [EXP_WIDTH-1:0] shift_exp0;
wire [EXP_WIDTH-1:0] shift_exp1;

assign shift_exp0 = max_exp - exp_m_1;
assign shift_exp1 = max_exp - exp_m_2;

shifter sh0(
    .mant(mant0),
    .exp(shift_exp0),
    .left(1'b0),
    .out(shifted_mant0)
);

shifter sh1(
    .mant(mant1),
    .exp(shift_exp1),
    .left(1'b0),
    .out(shifted_mant1)
);

log_finder lgf0(
    .shifted_mant(shifted_mant0),
    .log_mant(log_mant0)
);

log_finder lgf1(
    .shifted_mant(shifted_mant1),
    .log_mant(log_mant1)
);


//shifter and quantizer for whole block of mantissas
wire [31:0] scale_int = max_mant_int / 15;
wire [31:0] scale_pow2;
round_2_power r2p_1(
    .inp_num(scale_int),
    .out_num(scale_pow2)
);

wire [31:0] scale_factor_unb;
encoder_32_5 enc1(                      // Storing Exponent 
    .inp(scale_pow2),
    .outp(scale_factor_unb)
);

assign scale_factor = scale_factor_unb[SCALE_FACTOR_WIDTH-1:0];

genvar i;
generate
    for (i = 0; i < ARRAY_SIZE; i = i + 1) begin
        shifter_quantizer shq_i(
            .mant(mant_block[i]),
            .scale_factor(scale_factor),
            .out(mantissa_out[BLOCK_MANTISSA_WIDTH*i+:BLOCK_MANTISSA_WIDTH])
        );
    end
endgenerate

assign block_ready = scale_factor_ready;
assign max_exp = max_exp_int;
////////////////////////////////////

endmodule

