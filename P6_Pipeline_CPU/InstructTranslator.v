`timescale 1ns / 1ps
//////////////////////////////////////////////////////////////////////////////////
// Company: 
// Engineer: 
// 
// Create Date:    20:10:16 11/01/2023 
// Design Name: 
// Module Name:    InstructTranslator 
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

module InstructTranslator(
    input [31:0] instruct,
    input [31:0] PC,
    input clk
    );

    wire [5:0] op;
    wire [5:0] func;

    assign op = instruct[31:26];
    assign func = instruct[5:0];
    //instruct signal
    wire [4:0] rs;
    wire [4:0] rt;
    wire [4:0] rd;
    wire [15:0] imm16;
    wire [25:0] index26;
    wire [31:0] index32;
    reg [31:0] imm16_signExt;
    reg [31:0] imm16_zeroExt;
    //init
    assign rs = instruct[25:21];
    assign rt = instruct[20:16];
    assign rd = instruct[15:11];
    assign imm16 = instruct[15:0];
    assign index26 = instruct[25:0];
    assign index32 = {PC[31:28], index26, 2'b00};

    always @(*) begin
        
        imm16_zeroExt = {{16{1'b0}}, imm16};
        if(imm16[15] == 0)
            imm16_signExt = {{16{1'b0}}, imm16};
        else
            imm16_signExt = {{16{1'b1}}, imm16};

    end

    always @(posedge clk) begin


        if(op == `special) begin
            if(func == `add)
                $display("db: %d: %h: @%h: add $%d, $%d, $%d", $time, instruct, PC, rd, rs, rt);
            else if(func == `sub)
                $display("db: %d: %h: @%h: sub $%d, $%d, $%d", $time, instruct, PC, rd, rs, rt);
            else if(func == `jr)
                $display("db: %d: %h: @%h: jr $%d", $time, instruct, PC, rs);
            else if(func == 6'b000000)
                $display("db: %d: %h: @%h: nop", $time, instruct, PC);
            
        end
        else begin
            if(op == `ori)
                $display("db: %d: %h: @%h: ori $%d, $%d, %h", $time, instruct, PC, rt, rs, imm16_zeroExt);
            else if (op == `lw)
                $display("db: %d: %h: @%h: lw $%d, %h($%d)", $time, instruct, PC, rt, imm16_signExt, rs);
            else if (op == `sw)
                $display("db: %d: %h: @%h: sw $%d, %h($%d)", $time, instruct, PC, rt, imm16_signExt, rs);
            else if (op == `beq)
                $display("db: %d: %h: @%h: beq $%d, $%d, %h -> %h", $time, instruct, PC, rs, rt, imm16_signExt, PC + 4 + (imm16_signExt << 2));
            else if (op == `lui)
                $display("db: %d: %h: @%h: lui $%d, %h", $time, instruct, PC, rt, imm16_zeroExt);
            else if (op == `jal)
                $display("db: %d: %h: @%h: jal %h -> %h", $time, instruct, PC, index26, index32);
        end
    end

endmodule
