`timescale 1ns / 1ps
//////////////////////////////////////////////////////////////////////////////////
// Company: 
// Engineer: 
// 
// Create Date:    09:59:45 11/01/2023 
// Design Name: 
// Module Name:    mips 
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
module mips(
    input clk,
    input reset
    );

    //instruct signal
    wire [31:0] instruct;
    wire [4:0] rs;
    wire [4:0] rt;
    wire [4:0] rd;
    wire [15:0] imm16;
    wire [25:0] index26;
    wire [31:0] index32;

    //PC signal
    wire [31:0] PC;
    reg [31:0] PC_next;

    //GRF signal
    reg [4:0] RegAddrW;
    reg [31:0] RegDataW;
    wire [31:0] RegData1;
    wire [31:0] RegData2;

    //ALU signal
    wire ALUzero;
    wire [31:0] ALUresult;
    reg [31:0] inB;
    reg [31:0] imm16_signExt;
    reg [31:0] imm16_zeroExt;

    //DM signal
    wire [31:0] MemData;
    
    //control signal
    wire RegW2rd;
    wire ALUuseImm;
    wire Mem2Reg;
    wire RegWrite;
    wire MemWrite;
    wire Branch;
    wire immSignExt;
    wire [2:0] ALUctrl;
    wire saveHigh;
    wire jIndex;
    wire link;
    wire jr;

    //init
    assign rs = instruct[25:21];
    assign rt = instruct[20:16];
    assign rd = instruct[15:11];
    assign imm16 = instruct[15:0];
    assign index26 = instruct[25:0];

    //IFU
    IFU IFU(
        .PC_next(PC_next),
        .clk(clk),
        .reset(reset),
        .instruct(instruct),
        .PC(PC)
    );
    
    //PC_next
    assign index32 = {PC[31:28], index26, 2'b00};
    always @(*) begin
        if(Branch && ALUzero)
            PC_next <= PC + 4 + (imm16_signExt << 2);
        else if (jIndex)
            PC_next <= index32;
        else if (jr)
            PC_next <= RegData1;
        else
            PC_next <= PC + 4;
    end

    //GRF
    GRF GRF(
        .AddrR1(rs),
        .AddrR2(rt),
        .AddrW(RegAddrW),
        .DataW(RegDataW),
        .WPC(PC),
        .WEn(RegWrite),
        .clk(clk),
        .reset(reset),
        .DataR1(RegData1),
        .DataR2(RegData2)
    );
    always @(*) begin
        if(link)
            RegAddrW <= 5'b11111;
        else if(RegW2rd)
            RegAddrW <= rd;
        else
            RegAddrW <= rt;
    end
    always @(*) begin
        if(Mem2Reg)
            RegDataW <= MemData;
        else if(link)
            RegDataW <= PC + 4;
        else
            RegDataW <= ALUresult;
    end

    //ALU
    always @(*) begin
        imm16_zeroExt <= {{16{1'b0}}, imm16};
        if(imm16[15] == 0)
            imm16_signExt <= {{16{1'b0}}, imm16};
        else
            imm16_signExt <= {{16{1'b1}}, imm16};
    end
    ALU ALU(
        .inA(RegData1),
        .inB(inB),
        .ALUctrl(ALUctrl),
        .saveHigh(saveHigh),
        .zero(ALUzero),
        .result(ALUresult)
    );
    always @(*) begin
        if(ALUuseImm) begin
            if(immSignExt)
                inB <= imm16_signExt;
            else
                inB <= imm16_zeroExt;
        end
        else begin
            inB <= RegData2;
        end
    end

    //Controller
    Controller Controller(
        .instruct(instruct),
        .RegW2rd(RegW2rd),
        .ALUuseImm(ALUuseImm),
        .Mem2Reg(Mem2Reg),
        .RegWrite(RegWrite),
        .MemWrite(MemWrite),
        .Branch(Branch),
        .immSignExt(immSignExt),
        .ALUctrl(ALUctrl),
        .saveHigh(saveHigh),
        .jIndex(jIndex),
        .link(link),
        .jr(jr)
    );

    //DM
    DM DM(
        .Address(ALUresult),
        .Data(RegData2),
        .MemWrite(MemWrite),
        .clk(clk),
        .reset(reset),
        .WPC(PC),
        .MemData(MemData)
    );

    //debug
    /*InstructTranslator DB(
        .instruct(instruct),
        .PC(PC),
        .clk(clk)
    );*/


endmodule
