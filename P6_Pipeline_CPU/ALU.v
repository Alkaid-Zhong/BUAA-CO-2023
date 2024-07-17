`timescale 1ns / 1ps
//////////////////////////////////////////////////////////////////////////////////
// Company: 
// Engineer: 
// 
// Create Date:    13:07:33 11/01/2023 
// Design Name: 
// Module Name:    ALU 
// Project Name: 
// Target Devices: 
// Tool versions: 
// Description: 
//
// Dependencies: 
//
// Revision: 
// Revision 0.01 - File Created
// Additional Comments: 
//
//////////////////////////////////////////////////////////////////////////////////

`define     ALU_ADD     4'b0000
`define     ALU_SUB     4'b0001
`define     ALU_OR      4'b0010
`define     ALU_AND     4'b0011
`define     ALU_SLT     4'b0100
`define     ALU_SLTU    4'b0101
`define     ALU_BNE     4'b0110
`define     ALU_NOP     4'b1111


module ALU(
    input [31:0] inA,
    input [31:0] inB,
    input [3:0] ALUctrl,
    input upperLoad,
    output zero,
    output [31:0] result
    );

    reg [31:0] temp;
    always @(*) begin
        if(ALUctrl == `ALU_ADD)
            temp = inA + inB;
        else if(ALUctrl == `ALU_SUB)
            temp = inA - inB;
        else if(ALUctrl == `ALU_OR)
            temp = inA | inB;
        else if(ALUctrl == `ALU_AND)
            temp = inA & inB;
        else if(ALUctrl == `ALU_SLT) begin
            if($signed(inA) < $signed(inB))
                temp = 32'h0000_0001;
            else
                temp = 32'h0000_0000;
        end
        else if(ALUctrl == `ALU_SLTU) begin
            if(inA < inB)
                temp = 32'h0000_0001;
            else 
                temp = 32'h0000_0000;
        end
        else if(ALUctrl == `ALU_BNE)
            temp = (inA == inB) ? 32'h0000_0001 : 32'h0000_0000;
        else
            temp = 32'h0000_0000;
    end

    assign result = upperLoad ? (temp << 16) : temp;

    assign zero = result == 32'h0000_0000;

endmodule
