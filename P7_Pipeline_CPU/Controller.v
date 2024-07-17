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
// COP0
`define     cop0        6'b010000


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

`define     eret        6'b011000
`define     syscall     6'b001100


module Controller(
    input [31:0]        instruct,

    output              PC_branch,
    output              PC_jIndex,
    output              PC_jr,

    output              Reg_W2rd,
    output              Reg_WriteEn,
    output              Reg_W2_31,
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
    output [2:0]        T_use_rt,

    output              mfc0,
    output              mtc0,
    output              eret,
    output              syscall,
    output              RI,
    output              add_addi,
    output              sub,
    output              load,
    output              store
    );

    wire [5:0] op;
    wire [5:0] func;
    wire [4:0] rs;
    wire [4:0] rt;
    wire [4:0] rd;

    InstructSplitter Ctrl_splitter(
        .instruct   (instruct),

        .op         (op),
        .func       (func),
        .rs         (rs),
        .rt         (rt),
        .rd         (rd)
    );

    wire nop  = op == `special && func == `nop;
    // cal_r
    wire add  = op == `special && func == `add;
  assign sub  = op == `special && func == `sub;
    wire And  = op == `special && func == `AND;
    wire Or   = op == `special && func == `OR;
    wire slt  = op == `special && func == `slt;
    wire sltu = op == `special && func == `sltu;
    // cal_i;
    wire addi = op == `addi;
    wire andi = op == `andi;
    wire ori  = op == `ori;
    wire lui  = op == `lui;
    // load
    wire lb   = op == `lb;
    wire lh   = op == `lh;
    wire lw   = op == `lw;
    // store
    wire sb   = op == `sb;
    wire sh   = op == `sh;
    wire sw   = op == `sw;
    // mult div
    wire mult = op == `special && func == `mult;
    wire multu= op == `special && func == `multu;
    wire div  = op == `special && func == `div;
    wire divu = op == `special && func == `divu;
    wire mfhi = op == `special && func == `mfhi;
    wire mflo = op == `special && func == `mflo;
    wire mthi = op == `special && func == `mthi;
    wire mtlo = op == `special && func == `mtlo;
    // b
    wire beq  = op == `beq;
    wire bne  = op == `bne;
    // j
    wire jal  = op == `jal;
    wire jr   = op == `special && func == `jr;
    // CP0
  assign mfc0 = op == `cop0 && (rs == 5'b00000);
  assign mtc0 = op == `cop0 && (rs == 5'b00100);
  assign eret = op == `cop0 && func == `eret;
    // syscall
  assign syscall = op == `special && func == `syscall;

    wire cal_r;
    wire cal_i;
    // wire load;
    // wire store;
    wire mult_div;

    assign cal_r = add | sub | Or | And  | slt | sltu;
    assign cal_i = ori | addi | andi | lui;
    assign load =  lb | lh | lw;
    assign store = sb | sh | sw;
    assign mult_div = mult | multu | div | divu | mfhi | mflo | mthi | mtlo;

// RI
    assign RI = !(
        nop | add | sub | And | Or | slt | sltu | lui |
        addi | andi | ori |
        lb | lh | lw | sb | sh | sw |
        mult | multu | div | divu | mfhi | mflo | mthi | mtlo |
        beq | bne | jal | jr |
        mfc0 | mtc0 | eret | syscall
    );
// add/sub/l/s
    assign add_addi = add | addi;
    assign load_store = load | store;

// ------------------D--------------------
    //PC_branch
    assign PC_branch = beq | bne;

    //PC_jIndex
    assign PC_jIndex = jal;

    //PC_jr
    assign PC_jr = jr;


// ------------------E--------------------

    // Link_PC8
    assign Link_PC8 = jal;

    //ALU_inB_UseImm
    assign ALU_inB_UseImm = (cal_i | load | store );

    //ALU_immSignExt
    assign ALU_immSignExt = addi | load | store;

    //ALU_ALUctrl
    always @(*) begin
        if(add | addi | load | store)
            ALU_ALUctrl = `ALU_ADD;
        else if(sub | beq)
            ALU_ALUctrl = `ALU_SUB;
        else if(Or | ori | lui)
            ALU_ALUctrl = `ALU_OR;
        else if(And | andi)
            ALU_ALUctrl = `ALU_AND;
        else if(slt)
            ALU_ALUctrl = `ALU_SLT;
        else if(sltu)
            ALU_ALUctrl = `ALU_SLTU;
        else if(bne)
            ALU_ALUctrl = `ALU_BNE;
        else
            ALU_ALUctrl = `ALU_NOP;
    end

    //ALU_upperLoad
    assign ALU_upperLoad = lui;

    // MULT_DIV
    assign MULT_DIV = mult_div;

    // MULT_DIV_start
    assign MULT_DIV_start = mult | multu | div | divu;
    
    // MULT_DIV_ctrl
    always @(*) begin
        if(mult)
            MULT_DIV_ctrl = `MULT_DIV_mult;
        else if(multu)
            MULT_DIV_ctrl = `MULT_DIV_multu;
        else if(div)
            MULT_DIV_ctrl = `MULT_DIV_div;
        else if(divu)
            MULT_DIV_ctrl = `MULT_DIV_divu;
        else
            MULT_DIV_ctrl = 2'b00;
    end

    assign MULT_DIV_mfhi = mfhi;
    assign MULT_DIV_mflo = mflo;

    assign MULT_DIV_mthi = mthi;
    assign MULT_DIV_mtlo = mtlo;

// ------------------M--------------------

    //Mem_ByteWriteEn
    assign Mem_ByteWriteEn = (sw ? 4'b1111 :
                              sh ? 4'b0011 :
                              sb ? 4'b0001 :
                                   4'b0000);
    
    //Mem_ByteRead
    assign Mem_ByteRead = (lw ? 32'hffff_ffff :
                           lh ? 32'h0000_ffff :
                           lb ? 32'h0000_00ff :
                                32'h0000_0000);

// ------------------W--------------------
    // Reg_W2rd
    assign Reg_W2rd = cal_r | MULT_DIV_mfhi | MULT_DIV_mflo;

    //Reg_WriteMemData
    assign Reg_WriteMemData = load;

    //Reg_W2_31
    assign Reg_W2_31 = jal;

    // Reg_Addr_W MUX
    always @(*) begin
        if(Reg_W2_31)
            Reg_Addr_W = 5'b11111;
        else if(Reg_W2rd)
            Reg_Addr_W = rd;
        else
            Reg_Addr_W = rt;
    end

    //Reg_WriteEn
    assign Reg_WriteEn = (
        cal_r |
        cal_i |
        load |
        Reg_W2_31 |
        MULT_DIV_mfhi |
        MULT_DIV_mflo |
        mfc0
    ) && (Reg_Addr_W != 5'b00000);

// ----------Forward & Stall--------------

    assign T_new = (cal_r | cal_i) ? 3'h2 :
                   (load | mfc0) ? 3'h3 : 
                   (Link_PC8 | MULT_DIV_mfhi | MULT_DIV_mflo) ? 3'h2:
                   3'h0;
    assign T_use_rs = (PC_branch | PC_jr) ? 3'h0 :
                      (cal_r | cal_i | load | store | MULT_DIV_start | MULT_DIV_mthi | MULT_DIV_mtlo) ? 3'h1:
                      3'h5;
    assign T_use_rt = PC_branch ? 3'h0 :
                      (cal_r | MULT_DIV_start) ? 3'h1 :
                      (store | mtc0) ? 3'h2 :
                      3'h5;

endmodule
