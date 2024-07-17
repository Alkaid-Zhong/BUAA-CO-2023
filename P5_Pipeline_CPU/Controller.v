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

//-------------------ALU_ALUctrl-------------------
`define ALU_ADD 3'b000
`define ALU_SUB 3'b001
`define ALU_OR 3'b010
`define ALU_NOP 3'b111

//-------------------op-------------------
// cal_r
`define special 6'b000000
// cal_i
`define ori 6'b001101
`define lui 6'b001111
// load
`define lw 6'b100011
// store
`define sw 6'b101011
// branch
`define beq 6'b000100
// jump
`define jal 6'b000011

//-------------------func-------------------
`define nop 6'b000000
`define add 6'b100000
`define sub 6'b100010
`define jr 6'b001000


module Controller(
    input [31:0] instruct,

    output PC_branch,
    output PC_jIndex,
    output PC_jr,

    output Reg_W2rd,
    output Reg_WriteEn,
    output Reg_Link31,
    output Reg_WriteMemData,

    output ALU_inB_UseImm,
    output ALU_immSignExt,
    output reg [2:0] ALU_ALUctrl,
    output ALU_upperLoad,

    output Mem_WriteEn,

    output [2:0] T_new,
    output reg [4:0] Reg_Addr_W,
    output [2:0] T_use_rs,
    output [2:0] T_use_rt
    );

    wire [5:0] op;
    wire [5:0] func;

    assign op = instruct[31:26];
    assign func = instruct[5:0];

    wire cal_r;
    wire cal_i;
    wire load;
    wire store;

    assign cal_r = (op == `special && ( func == `add ||
                                        func == `sub ));
    assign cal_i = (op == `ori ||
                    op == `lui );
    assign load = (op == `lw);
    assign store = (op == `sw);

// ------------------D--------------------
    //PC_branch
    assign PC_branch = (op == `beq);

    //PC_jIndex
    assign PC_jIndex = (op == `jal);

    //PC_jr
    assign PC_jr = (op == `special && func == `jr);


// ------------------E--------------------
    //ALU_inB_UseImm
    assign ALU_inB_UseImm = (op == `ori ||
                             op == `lw ||
                             op == `sw ||
                             op == `lui);

    //ALU_immSignExt
    assign ALU_immSignExt = (op == `lw ||
                             op == `sw);

    //ALU_ALUctrl
    always @(*) begin
        if(op == `special) begin
            if(func == `add)
                ALU_ALUctrl <= `ALU_ADD;
            else if(func == `sub)
                ALU_ALUctrl <= `ALU_SUB;
            else
                ALU_ALUctrl <= `ALU_NOP;
        end
        else begin
            if (op == `ori ||
                op == `lui)
                ALU_ALUctrl <= `ALU_OR;
            else if(op == `lw ||
                    op == `sw)
                ALU_ALUctrl <= `ALU_ADD;
            else if(op == `beq)
                ALU_ALUctrl <= `ALU_SUB;
            else
                ALU_ALUctrl <= `ALU_NOP;
        end
    end

    //ALU_upperLoad
    assign ALU_upperLoad = (op == `lui);

// ------------------M--------------------

    //Mem_WriteEn
    assign Mem_WriteEn = (op == `sw);

// ------------------W--------------------
    // Reg_W2rd
    assign Reg_W2rd = ((op == `special && func == `add) || 
                       (op == `special && func == `sub) ||
                       (op == `special && func == `jr));

    //Reg_WriteMemData
    assign Reg_WriteMemData = (op == `lw);

    //Reg_Link31
    assign Reg_Link31 = (op == `jal);

    // Reg_Addr_W MUX
    wire [4:0] rt;
    wire [4:0] rd;
    assign rt = instruct[20:16];
    assign rd = instruct[15:11];
    always @(*) begin
        if(Reg_Link31)
            Reg_Addr_W <= 5'b11111;
        else if(Reg_W2rd)
            Reg_Addr_W <= rd;
        else
            Reg_Addr_W <= rt;
    end

    //Reg_WriteEn
    assign Reg_WriteEn = (cal_r ||
                          cal_i ||
                          load ||
                          Reg_Link31) && (Reg_Addr_W != 5'b00000);

// ----------Forward & Stall--------------

    assign T_new = (cal_r || cal_i) ? 3'h2 :
                   (load) ? 3'h3 : 
                   (Reg_Link31) ? 3'h2:
                   3'h0;
    assign T_use_rs = (PC_branch || PC_jr) ? 3'h0 :
                      (cal_r || cal_i || load || store) ? 3'h1:
                      3'h5;
    assign T_use_rt = PC_branch ? 3'h0 :
                      cal_r ? 3'h1 :
                      store ? 3'h2 :
                      3'h5;

endmodule
