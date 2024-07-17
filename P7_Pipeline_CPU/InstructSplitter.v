`timescale 1ns / 1ps
//////////////////////////////////////////////////////////////////////////////////
// Company: 
// Engineer: 
// 
// Create Date:    23:07:37 12/05/2023 
// Design Name: 
// Module Name:    InstructSplitter 
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
module InstructSplitter(
    input [31:0] instruct,
    output [5:0] op,
    output [5:0] func,
    output [4:0] rs,
    output [4:0] rt,
    output [4:0] rd,
    output [15:0] imm16,
    output [25:0] index26
    );

    assign op = instruct[31:26];
    assign func = instruct[5:0];
    assign rs = instruct[25:21];
    assign rt = instruct[20:16];
    assign rd = instruct[15:11];
    assign imm16 = instruct[15:0];
    assign index26 = instruct[25:0];


endmodule
