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
`define     ALU_ADD     4'b0000
`define     ALU_SUB     4'b0001
`define     ALU_OR      4'b0010
`define     ALU_AND     4'b0011
`define     ALU_SLT     4'b0100
`define     ALU_SLTU    4'b0101
`define     ALU_BNE     4'b0110
`define     ALU_NOP     4'b1111

//-----------------MULT_DIV_ctrl-----------------
`define MULT_DIV_mult    2'b00
`define MULT_DIV_multu   2'b01
`define MULT_DIV_div     2'b10
`define MULT_DIV_divu    2'b11

//-------------------op-------------------
// cal_r
`define     special     6'b000000
// cal_i
`define     ori         6'b001101
`define     addi        6'b001000
`define     andi        6'b001100
`define     lui         6'b001111
// load
`define     lb          6'b100000
`define     lh          6'b100001
`define     lw          6'b100011
// store
`define     sb          6'b101000
`define     sh          6'b101001
`define     sw          6'b101011
// branch
`define     beq         6'b000100
`define     bne         6'b000101
// jump
`define     jal         6'b000011

//-------------------func-------------------
`define     nop         6'b000000
`define     add         6'b100000
`define     sub         6'b100010
`define     AND         6'b100100
`define     OR          6'b100101
`define     slt         6'b101010
`define     sltu        6'b101011

`define     mult        6'b011000
`define     multu       6'b011001
`define     div         6'b011010
`define     divu        6'b011011
`define     mfhi        6'b010000
`define     mflo        6'b010010
`define     mthi        6'b010001
`define     mtlo        6'b010011

`define     jr          6'b001000


module Controller(
    input [31:0]        instruct,

    output              PC_branch,
    output              PC_jIndex,
    output              PC_jr,

    output              Reg_W2rd,
    output              Reg_WriteEn,
    output              Reg_Link31,
    output              Reg_WriteMemData,

    output              Link_PC8,

    output              ALU_inB_UseImm,
    output              ALU_immSignExt,
    output reg [3:0]    ALU_ALUctrl,
    output              ALU_upperLoad,

    output              MULT_DIV,
    output              MULT_DIV_start,
    output reg [1:0]    MULT_DIV_ctrl,
    output              MULT_DIV_mfhi,
    output              MULT_DIV_mflo,
    output              MULT_DIV_mthi,
    output              MULT_DIV_mtlo,

    output [3:0]        Mem_ByteWriteEn,
    output [31:0]       Mem_ByteRead,

    output [2:0]        T_new,
    output reg [4:0]    Reg_Addr_W,
    output [2:0]        T_use_rs,
    output [2:0]        T_use_rt
    );

    wire [5:0] op;
    wire [5:0] func;

    assign op = instruct[31:26];
    assign func = instruct[5:0];

    wire cal_r;
    wire cal_i;
    wire load;
    wire store;
    wire mult_div;

    assign cal_r = (op == `special && ( func == `add ||
                                        func == `sub ||
                                        func == `OR  ||
                                        func == `AND ||
                                        func == `slt ||
                                        func == `sltu));
    assign cal_i = (op == `ori  ||
                    op == `addi ||
                    op == `andi ||
                    op == `lui );
    assign load =  (op == `lw || op == `lb || op == `lh);
    assign store = (op == `sw || op == `sb || op == `sh);
    
    assign mult_div = (op == `special && ( func == `mult || 
                                           func == `multu || 
                                           func == `div ||
                                           func == `divu ||
                                           func == `mfhi ||
                                           func == `mflo ||
                                           func == `mthi ||
                                           func == `mtlo
                                         ));

// ------------------D--------------------
    //PC_branch
    assign PC_branch = (op == `beq || op == `bne);

    //PC_jIndex
    assign PC_jIndex = (op == `jal);

    //PC_jr
    assign PC_jr = (op == `special && func == `jr);


// ------------------E--------------------

    // Link_PC8
    assign Link_PC8 = op == `jal;

    //ALU_inB_UseImm
    assign ALU_inB_UseImm = (cal_i || load || store );

    //ALU_immSignExt
    assign ALU_immSignExt = (op == `addi ||
                             load || store);

    //ALU_ALUctrl
    always @(*) begin
        if(op == `special) begin
            if(func == `add)
                ALU_ALUctrl <= `ALU_ADD;
            else if(func == `sub)
                ALU_ALUctrl <= `ALU_SUB;
            else if(func == `OR)
                ALU_ALUctrl <= `ALU_OR;
            else if(func == `AND)
                ALU_ALUctrl <= `ALU_AND;
            else if(func == `slt)
                ALU_ALUctrl <= `ALU_SLT;
            else if(func == `sltu)
                ALU_ALUctrl <= `ALU_SLTU;
            else
                ALU_ALUctrl <= `ALU_NOP;
        end
        else begin
            if     (op == `ori ||
                    op == `lui  )
                ALU_ALUctrl <= `ALU_OR;
            else if(op == `andi)
                ALU_ALUctrl <= `ALU_AND;
            else if(op == `addi ||
                    load ||
                    store )
                ALU_ALUctrl <= `ALU_ADD;
            else if(op == `beq)
                ALU_ALUctrl <= `ALU_SUB;
            else if(op == `bne)
                ALU_ALUctrl <= `ALU_BNE;
            else
                ALU_ALUctrl <= `ALU_NOP;
        end
    end

    //ALU_upperLoad
    assign ALU_upperLoad = (op == `lui);

    // MULT_DIV
    assign MULT_DIV = mult_div;

    // MULT_DIV_start
    assign MULT_DIV_start = (op == `special && ( func == `mult || 
                                                 func == `multu || 
                                                 func == `div ||
                                                 func == `divu
                                                ));
    
    // MULT_DIV_ctrl
    always @(*) begin
        if(func == `mult)
            MULT_DIV_ctrl <= `MULT_DIV_mult;
        else if(func == `multu)
            MULT_DIV_ctrl <= `MULT_DIV_multu;
        else if(func == `div)
            MULT_DIV_ctrl <= `MULT_DIV_div;
        else if(func == `divu)
            MULT_DIV_ctrl <= `MULT_DIV_divu;
        else
            MULT_DIV_ctrl <= 2'b00;
    end

    assign MULT_DIV_mfhi = (op == `special && func == `mfhi);
    assign MULT_DIV_mflo = (op == `special && func == `mflo);

    assign MULT_DIV_mthi = (op == `special && func == `mthi);
    assign MULT_DIV_mtlo = (op == `special && func == `mtlo);

// ------------------M--------------------

    //Mem_ByteWriteEn
    assign Mem_ByteWriteEn = (op == `sw ? 4'b1111 :
                              op == `sh ? 4'b0011 :
                              op == `sb ? 4'b0001 :
                                          4'b0000);
    
    //Mem_ByteRead
    assign Mem_ByteRead = (op == `lw ? 32'hffff_ffff :
                           op == `lh ? 32'h0000_ffff :
                           op == `lb ? 32'h0000_00ff :
                                       32'h0000_0000);

// ------------------W--------------------
    // Reg_W2rd
    assign Reg_W2rd = (cal_r || 
                       (op == `special && func == `jr) ||
                       MULT_DIV_mfhi ||
                       MULT_DIV_mflo );

    //Reg_WriteMemData
    assign Reg_WriteMemData = (load);

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
                          Reg_Link31 ||
                          MULT_DIV_mfhi ||
                          MULT_DIV_mflo) && (Reg_Addr_W != 5'b00000);

// ----------Forward & Stall--------------

    assign T_new = (cal_r || cal_i) ? 3'h2 :
                   (load) ? 3'h3 : 
                   (Reg_Link31 || MULT_DIV_mfhi || MULT_DIV_mflo) ? 3'h2:
                   3'h0;
    assign T_use_rs = (PC_branch || PC_jr) ? 3'h0 :
                      (cal_r || cal_i || load || store || MULT_DIV_start || MULT_DIV_mthi || MULT_DIV_mtlo) ? 3'h1:
                      3'h5;
    assign T_use_rt = PC_branch ? 3'h0 :
                      (cal_r || MULT_DIV_start) ? 3'h1 :
                      store ? 3'h2 :
                      3'h5;

endmodule
