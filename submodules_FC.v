// HARI OM

// Shifter module used for all shifting operations

`timescale 1ns/1ps

module shifter (

input [31:0] mant,
input [7:0] exp,
input left,
output [31:0] out

);

assign out = left ? (mant << exp) : (mant >> exp);

endmodule

module shifter_quantizer (

input [31:0] mant,
input [4:0] scale_factor,
output [4:0] out

);

wire [31:0] scaled_mant;
assign scaled_mant = (mant >> scale_factor);

assign out = {1'b0,(scaled_mant > 15)? 4'b1111: scaled_mant[3:0]};

endmodule


// 32:5 encoder
module encoder_32_5 (

input [31:0] inp,
output [31:0] outp

);
assign outp[31:5] = 0;
assign outp[4]=(inp[16] | inp[17] | inp[18] | inp[19] |inp[20] | inp[21] | inp[22] | inp[23]|inp[24] | inp[25] | inp[26] | inp[27]|inp[28] | inp[29] | inp[30] | inp[31] );
assign outp[3]=(inp[8] | inp[9] | inp[10] | inp[11] |inp[12] | inp[13] | inp[14] | inp[15]|inp[24] | inp[25] | inp[26] | inp[27]|inp[28] | inp[29] | inp[30] | inp[31]   );
assign outp[2]=(inp[4] | inp[5] | inp[6] | inp[7] |inp[12] | inp[13] | inp[14] | inp[15]|inp[20] | inp[21] | inp[22] | inp[23]|inp[28] | inp[29] | inp[30] | inp[31]     );
assign outp[1]=(inp[2] | inp[3] | inp[6] | inp[7] |inp[10] | inp[11] | inp[14] | inp[15]|inp[18] | inp[19] | inp[22] | inp[23]|inp[26] | inp[27] | inp[30] | inp[31]     );
assign outp[0]=(inp[1] | inp[3] | inp[5] | inp[7] |inp[9] | inp[11] | inp[13] | inp[15]|inp[17] | inp[19] | inp[21] | inp[23]|inp[25] | inp[27] | inp[29] | inp[31]      );

endmodule

// Reverse a Number (used in leading one detector)
module rev_num(

    input [31:0] org,
    output [31:0] rev
);

    genvar i;
    generate
        for (i = 0; i < 32; i = i + 1) begin
            assign rev[31-i] = org [i];
        end
    endgenerate
endmodule

// Leading One Detector
module lead_one_detector(

    input [31:0] seq,
    output [31:0] one_hot
);
    wire [31:0] seq_rev;
    wire [31:0] one_hot_rev;
    rev_num rev_block (
        .org(seq),
        .rev(seq_rev)
    );

    assign one_hot_rev = seq_rev & (~seq_rev + 1);

    rev_num rev_block_1 (
        .org(one_hot_rev),
        .rev(one_hot) 
    );
    
endmodule

// Log module
module log_finder (

input  [31:0] shifted_mant,
output [31:0] log_mant

);

wire [31:0] lead_one_hot;
wire [31:0] exp_mant;
wire [31:0] raw_mant;  

lead_one_detector lod_0(
    .seq(shifted_mant),
    .one_hot(lead_one_hot)
);

encoder_32_5 enc1(
    .inp(lead_one_hot),
    .outp(exp_mant)
);

wire [7:0] shift_exp = 31 - exp_mant[7:0];
shifter sh1(
    .mant(shifted_mant),
    .exp(shift_exp),
    .left(1'b1),
    .out(raw_mant)
);

assign log_mant = {24'b000000000000000000000000,raw_mant[30:23]} + exp_mant;

endmodule


// Module for rounding to nearest 2 power
module round_2_power (
        input [31:0] inp_num,
        output [31:0] out_num
    );

    wire [31:0] lead_one_hot;
    lead_one_detector lod_1(
        .seq(inp_num),
        .one_hot(lead_one_hot)
    );

    assign out_num = (((lead_one_hot >> 1) & inp_num) != 0) 
                 ? (lead_one_hot << 1) 
                 : lead_one_hot;
    
endmodule


module descaling (
    input [4:0] mant,
    input [3:0] inp_scale_factor,
    output [15:0] inp_pe
);

assign inp_pe = {11'b00000000000,mant} << inp_scale_factor;
   
endmodule

