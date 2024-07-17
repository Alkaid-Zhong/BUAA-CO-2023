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

`define ALU_ADD 3'b000
`define ALU_SUB 3'b001
`define ALU_OR 3'b010


module ALU(
    input [31:0] inA,
    input [31:0] inB,
    input [2:0] ALUctrl,
    input upperLoad,
    output zero,
    output [31:0] result
    );

    reg [31:0] temp;
    always @(*) begin
        if(ALUctrl == `ALU_ADD)
            temp <= inA + inB;
        else if(ALUctrl == `ALU_SUB)
            temp <= inA - inB;
        else if(ALUctrl == `ALU_OR)
            temp <= inA | inB;
        else
            temp <= 32'h0000_0000;
    end

    assign result = upperLoad ? (temp << 16) : temp;

    assign zero = result == 32'h0000_0000;

endmodule
