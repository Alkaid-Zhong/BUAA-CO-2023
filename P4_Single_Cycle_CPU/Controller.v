`timescale 1ns / 1ps
//////////////////////////////////////////////////////////////////////////////////
// Company: 
// Engineer: 
// 
// Create Date:    13:27:28 11/01/2023 
// Design Name: 
// Module Name:    Controller 
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

//ALUctrl
`define ALU_ADD 3'b000
`define ALU_SUB 3'b001
`define ALU_OR 3'b010
`define ALU_NOP 3'b111

//op
`define special 6'b000000
`define ori 6'b001101
`define lw 6'b100011
`define sw 6'b101011
`define beq 6'b000100
`define lui 6'b001111
`define jal 6'b000011

//func
`define add 6'b100000
`define sub 6'b100010
`define jr 6'b001000


module Controller(
    input [31:0] instruct,
    output RegW2rd,
    output ALUuseImm,
    output Mem2Reg,
    output RegWrite,
    output MemWrite,
    output Branch,
    output immSignExt,
    output reg [2:0] ALUctrl,
    output saveHigh,
    output jIndex,
    output link,
    output jr
    );

    wire [5:0] op;
    wire [5:0] func;

    assign op = instruct[31:26];
    assign func = instruct[5:0];

    // RegW2rd
    assign RegW2rd = op == `special;

    //ALUuseImm
    assign ALUuseImm = (op == `ori ||
                        op == `lw ||
                        op == `sw ||
                        op == `lui);

    //Mem2Reg
    assign Mem2Reg = (op == `lw);

    //RegWrite
    assign RegWrite = (op == `special ||
                        op == `ori ||
                        op == `lw ||
                        op == `lui ||
                        op == `jal);

    //MemWrite
    assign MemWrite = (op == `sw);

    //Branch
    assign Branch = (op == `beq);

    //immSignExt
    assign immSignExt = (op == `lw ||
                        op == `sw);

    //ALUctrl
    always @(*) begin
        if(op == `special) begin
            if(func == `add)
                ALUctrl <= `ALU_ADD;
            else if(func == `sub)
                ALUctrl <= `ALU_SUB;
            else
                ALUctrl <= `ALU_NOP;
        end
        else begin
            if(op == `ori ||
                op == `lui)
                ALUctrl <= `ALU_OR;
            else if(op == `lw ||
                    op == `sw)
                ALUctrl <= `ALU_ADD;
            else if(op == `beq)
                ALUctrl <= `ALU_SUB;
            else
                ALUctrl <= `ALU_NOP;
        end
    end

    //saveHigh
    assign saveHigh = (op == `lui);

    //jIndex
    assign jIndex = (op == `jal);

    //link
    assign link = (op == `jal);

    //jr
    assign jr = (op == `special && func == `jr);

endmodule
