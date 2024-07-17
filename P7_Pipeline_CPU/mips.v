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

// ExcCode
`define Exc_Int     5'd0
`define Exc_AdEL    5'd4
`define Exc_AdES    5'd5
`define Exc_Syscall 5'd8
`define Exc_RI      5'd10
`define Exc_Ov      5'd12

module mips(
    input clk,                    // 时钟信号
    input reset,                  // 同步复位信号
    input interrupt,              // 外部中断信号
    output [31:0] macroscopic_pc, // 宏观 PC

    output [31:0] i_inst_addr,    // IM 读取地址（取指 PC）
    input  [31:0] i_inst_rdata,   // IM 读取数据

    output [31:0] m_data_addr,    // DM 读写地址
    input  [31:0] m_data_rdata,   // DM 读取数据
    output [31:0] m_data_wdata,   // DM 待写入数据
    output [3 :0] m_data_byteen,  // DM 字节使能信号

    output [31:0] m_int_addr,     // 中断发生器待写入地址
    output [3 :0] m_int_byteen,   // 中断发生器字节使能信号

    output [31:0] m_inst_addr,    // M 级 PC

    output w_grf_we,              // GRF 写使能信号
    output [4 :0] w_grf_addr,     // GRF 待写入寄存器编号
    output [31:0] w_grf_wdata,    // GRF 待写入数据

    output [31:0] w_inst_addr     // W 级 PC
);

// global
    // stall signal
    reg         stall;

    // flush
    reg         flush;

    // interrupt
    wire [5:0]  HWInt_final;

    // req
    wire        req;

    // FWD & stall signal
    wire [4:0]  FWD_E_Reg_Addr;
    wire [31:0] FWD_E_Reg_Data;
    wire [2:0]  FWD_E_T_new;
    wire        FWD_E_Reg_WriteEn;

    wire [4:0]  FWD_M_Reg_Addr;
    wire [31:0] FWD_M_Reg_Data;
    wire [2:0]  FWD_M_T_new;
    wire        FWD_M_Reg_WriteEn;

    wire [4:0]  FWD_W_Reg_Addr;
    wire [31:0] FWD_W_Reg_Data;
    wire [2:0]  FWD_W_T_new;
    wire        FWD_W_Reg_WriteEn;

    wire [2:0]  D_T_new;
    wire [2:0]  D_T_use_rs;
    wire [2:0]  D_T_use_rt;

    wire        D_MULT_DIV;

// F
    wire [31:0] F_instruct;
    wire [31:0] F_PC;

    reg  [31:0] F_NPC;

// D
    wire [31:0] D_instruct;
    wire [31:0] D_PC;
    wire [4:0]  D_rs;
    wire [4:0]  D_rt;
    wire [15:0] D_imm16;
    wire [25:0] D_index26;

    wire [31:0] D_index32;
    reg  [31:0] D_imm16_signExt;

    // control signal
    wire        D_ctrl_PC_branch;
    wire        D_ctrl_PC_jIndex;
    wire        D_ctrl_PC_jr;
    wire [3:0]  D_ctrl_branch_ALU_ctrl;
    
    // D Register Read Signal
    wire [31:0] D_Reg_Data_R_rs;
    wire [31:0] D_Reg_Data_R_rt;

    // branch signal
    reg  [31:0] D_Actual_Data_rs;
    reg  [31:0] D_Actual_Data_rt;

    wire        D_branch_ALU_zero;
    wire [31:0] D_branch_ALU_result;

    // FWD
    wire [31:0] D_Reg_Data_R_rs_final;
    wire [31:0] D_Reg_Data_R_rt_final;

// E
    wire [31:0] E_PC;
    wire [31:0] E_instruct;
    wire [4:0]  E_rs;
    wire [4:0]  E_rt;
    wire [4:0]  E_rd;
    wire [15:0] E_imm16;
    reg  [31:0] E_imm16_signExt;
    reg  [31:0] E_imm16_zeroExt;

    // ID_EX
    wire [31:0] E_Reg_Data_rs;
    wire [31:0] E_Reg_Data_rt;

    // E ctrl
    wire        E_ctrl_ALU_inB_UseImm;
    wire        E_ctrl_ALU_immSignExt;
    wire [3:0]  E_ctrl_ALU_ALUctrl;
    wire        E_ctrl_ALU_upperLoad;
    
    wire        E_ctrl_linkPC8;

    wire        E_ctrl_MULT_DIV_start;
    wire [1:0]  E_ctrl_MULT_DIV_ctrl;
    wire        E_ctrl_MULT_DIV_mfhi;
    wire        E_ctrl_MULT_DIV_mflo;
    wire        E_ctrl_MULT_DIV_mthi;
    wire        E_ctrl_MULT_DIV_mtlo;

    // ALU
    reg  [31:0] E_ALU_inA;
    reg  [31:0] E_ALU_inB;
    wire [31:0] E_ALU_result;
    wire        E_ALU_zero;

    // E mult div
    wire        E_MULT_DIV_busy;
    wire [31:0] E_MULT_DIV_HI;
    wire [31:0] E_MULT_DIV_LO;

    // E data
    reg  [31:0] E_ALU_result_final;

    // FWD
    wire [31:0] E_Reg_Data_rs_final;
    wire [31:0] E_Reg_Data_rt_final;

// M
    wire [31:0] M_PC;
    wire [31:0] M_instruct;
    wire [4:0]  M_rs;
    wire [4:0]  M_rt;
    wire [4:0]  M_rd;

    // EX_MEM
    wire [31:0] M_ALU_result;
    wire [31:0] M_Reg_Data_rs;
    wire [31:0] M_Reg_Data_rt;

    // M ctrl
    wire [3:0]  M_ctrl_Mem_ByteWriteEn;
    wire [31:0] M_ctrl_Mem_ByteRead;

    // Mem Write
    reg  [31:0] M_Reg_Data_rt_final;
    reg  [31:0] M_Mem_Data;

    // Mem Read
    wire [31:0] M_Mem_Read;

    // Bridge
    wire [31:0] Timer0_data_r;
    wire [31:0] Timer1_data_r;

    wire [31:0] M_Data_R_final; // MemRead

    wire        Timer0_WriteEn;
    wire        Timer1_WriteEn;
    wire        Timer0_IRQ;
    wire        Timer1_IRQ;

// CP0
    wire        D_eret;
    wire        E_mtc0;
    wire        M_eret;
    wire        M_mtc0;
    wire        M_mfc0;
    wire [31:0] CP0_out;
    wire [31:0] CP0_EPC;
    wire [31:0] W_CP0_out;

// W
    wire [31:0] W_PC;
    wire [31:0] W_instruct;
    wire [4:0]  W_rt;
    wire [4:0]  W_rd;

    // MEM_WB
    wire [31:0] W_ALU_result;
    wire [31:0] W_Mem_Data;

    // W ctrl
    wire        W_ctrl_Reg_W2rd;
    wire        W_ctrl_Reg_WriteMemData;
    wire        W_ctrl_Reg_W2_31;
    wire        W_ctrl_LinkPC8;
    wire        W_ctrl_Reg_WriteEn;
    wire        W_ctrl_mfc0;

    // Write Back Data
    reg  [4:0]  W_Reg_Addr_W_final;
    reg  [31:0] W_Reg_Data_W_final;

// ExcCode
    reg  [4:0]  F_ExcCode_final;

    wire [4:0]  D_ExcCode;
    reg  [4:0]  D_ExcCode_final;
    wire        D_RI;
    wire        D_Syscall;

    wire [4:0]  E_ExcCode;
    reg  [4:0]  E_ExcCode_final;
    wire        E_add_addi;
    wire        E_sub;
    wire        E_load;
    wire        E_store;
    wire [32:0] E_inA_ex;
    wire [32:0] E_inB_ex;
    wire [32:0] E_add_ex;
    wire [32:0] E_sub_ex;
    wire        E_Ov;

    wire [4:0]  M_ExcCode;
    reg  [4:0]  M_ExcCode_final;
    wire        M_lb;
    wire        M_lh;
    wire        M_lw;
    wire        M_sb;
    wire        M_sh;
    wire        M_sw;

// BD
    reg         F_BD;
    wire        D_BD;
    wire        E_BD;
    wire        M_BD;

// ---------------------F-----------------------

    // PC
    PC PC(
        .PC_next    (F_NPC),

        .clk        (clk),
        .reset      (reset),
        .req        (req),
        .En         (~stall),

        .PC         (F_PC)
    );

    assign i_inst_addr = F_PC;
    assign F_instruct = i_inst_rdata;
    
    always @(*) begin
        flush = (~stall) & D_eret;
    end

    // ExcCode - AdEL(PC)
    always @(*) begin
        if     (F_PC[1:0] != 2'b00)
            F_ExcCode_final = `Exc_AdEL;
        else if(!(32'h0000_3000 <= F_PC && F_PC <= 32'h0000_6ffc))
            F_ExcCode_final = `Exc_AdEL;
        else
            F_ExcCode_final = 5'h0;
    end

    // BD
    always @(*) begin
        if(D_ctrl_PC_branch || D_ctrl_PC_jIndex || D_ctrl_PC_jr)
            F_BD = 1'b1;
        else
            F_BD = 1'b0;
    end
    
// -------------------IF/ID---------------------

    PipeLineRegister IF_ID(
        .PC             (F_PC),
        .Instruct       (F_instruct),

        .ExcCode        (F_ExcCode_final),
        .BD             (F_BD),

        .clk            (clk),
        .reset          (reset),
        .ReqClr         (req),
        .clr            (flush),
        .en             (~stall),

        .PC_n           (D_PC),
        .Instruct_n     (D_instruct),

        .ExcCode_n      (D_ExcCode),
        .BD_n           (D_BD)
    );

// ---------------------D-----------------------

    Controller D_ctrl(
        .instruct       (D_instruct),

        .T_new          (D_T_new),
        .T_use_rs       (D_T_use_rs),
        .T_use_rt       (D_T_use_rt),

        .PC_branch      (D_ctrl_PC_branch),
        .PC_jIndex      (D_ctrl_PC_jIndex),
        .PC_jr          (D_ctrl_PC_jr),
        .ALU_ALUctrl    (D_ctrl_branch_ALU_ctrl),

        .MULT_DIV       (D_MULT_DIV),

        .eret           (D_eret),
        .RI             (D_RI),
        .syscall        (D_Syscall)
    );


    // ExcCode - RI / Syscall
    always @(*) begin
        if(D_RI)
            D_ExcCode_final = `Exc_RI;
        else if(D_Syscall)
            D_ExcCode_final = `Exc_Syscall;
        else
            D_ExcCode_final = D_ExcCode;
    end

    InstructSplitter D_splitter_GRF(
        .instruct       (D_instruct),

        .rs             (D_rs),
        .rt             (D_rt)
    );
    
    // stall
    always @(*) begin
        if      (D_eret && ((E_mtc0 && E_rd == 5'd14) || (M_mtc0 && M_rd == 5'd14)))
            stall = 1'b1;
        else if (D_rs == FWD_E_Reg_Addr && D_T_use_rs < FWD_E_T_new && FWD_E_Reg_WriteEn)
            stall = 1'b1;
        else if(D_rt == FWD_E_Reg_Addr && D_T_use_rt < FWD_E_T_new && FWD_E_Reg_WriteEn)
            stall = 1'b1;
        else if(D_rs == FWD_M_Reg_Addr && D_T_use_rs < FWD_M_T_new && FWD_M_Reg_WriteEn)
            stall = 1'b1;
        else if(D_rt == FWD_M_Reg_Addr && D_T_use_rt < FWD_M_T_new && FWD_M_Reg_WriteEn)
            stall = 1'b1;
        else if(D_rs == FWD_W_Reg_Addr && D_T_use_rs < FWD_W_T_new && FWD_W_Reg_WriteEn)
            stall = 1'b1;
        else if(D_rt == FWD_W_Reg_Addr && D_T_use_rt < FWD_W_T_new && FWD_W_Reg_WriteEn)
            stall = 1'b1;
        else if(D_MULT_DIV && (E_ctrl_MULT_DIV_start || E_MULT_DIV_busy))
            stall = 1'b1;
        else
            stall = 1'b0;
    end

    // GRF
    GRF GRF(
        .AddrR1     (D_rs),
        .AddrR2     (D_rt),

        .AddrW      (W_Reg_Addr_W_final),//
        .DataW      (W_Reg_Data_W_final),//
        .WPC        (W_PC),//
        .WEn        (W_ctrl_Reg_WriteEn),//

        .clk        (clk),
        .reset      (reset),

        .DataR1     (D_Reg_Data_R_rs),
        .DataR2     (D_Reg_Data_R_rt)
    );

    // FWD MUX
    always @(*) begin
        if     (D_rs == FWD_M_Reg_Addr && FWD_M_T_new == 2'h0 && FWD_M_Reg_WriteEn)
            D_Actual_Data_rs = FWD_M_Reg_Data;
        else if(D_rs == FWD_W_Reg_Addr && FWD_W_T_new == 2'h0 && FWD_W_Reg_WriteEn)
            D_Actual_Data_rs = FWD_W_Reg_Data;
        else
            D_Actual_Data_rs = D_Reg_Data_R_rs;
    end
    always @(*) begin
        if     (D_rt == FWD_M_Reg_Addr && FWD_M_T_new == 2'h0 && FWD_M_Reg_WriteEn)
            D_Actual_Data_rt = FWD_M_Reg_Data;
        else if(D_rt == FWD_W_Reg_Addr && FWD_W_T_new == 2'h0 && FWD_W_Reg_WriteEn)
            D_Actual_Data_rt = FWD_W_Reg_Data;
        else
            D_Actual_Data_rt = D_Reg_Data_R_rt;
    end


    // ALU branch calculate
    ALU ALU_branch(
        .inA        (D_Actual_Data_rs),
        .inB        (D_Actual_Data_rt),
        .ALUctrl    (D_ctrl_branch_ALU_ctrl),
        .upperLoad  (1'b0),
        .zero       (D_branch_ALU_zero),
        .result     (D_branch_ALU_result)
    );
    
    // PC_next MUX

    InstructSplitter D_splitter_PC(
        .instruct   (D_instruct),

        .imm16      (D_imm16),
        .index26    (D_index26)
    );
    assign D_index32 = {D_PC[31:28], D_index26, 2'b00};
    always @(*) begin
        if(D_imm16[15] == 0)
            D_imm16_signExt = {{16{1'b0}}, D_imm16};
        else
            D_imm16_signExt = {{16{1'b1}}, D_imm16};
    end

    always @(*) begin
        if(D_eret)
            F_NPC = CP0_EPC;
        else if(req)
            F_NPC = 32'h0000_4180;
        else if(D_ctrl_PC_branch && D_branch_ALU_zero)
            F_NPC = D_PC + 32'h0000_0004 + (D_imm16_signExt << 2);
        else if (D_ctrl_PC_jIndex)
            F_NPC = D_index32;
        else if (D_ctrl_PC_jr)
            F_NPC = D_Actual_Data_rs;
        else
            F_NPC = F_PC + 32'h0000_0004;
    end

// -------------------ID/EX---------------------

    // for instruct sw save right data of register
    /*
    | D | E | M | W |
     sw  nop nop ori
         sw  nop nop
          |
          rt will change but can't be forwarded
    */
    ReceiveFWD FWD_ID_EX_rs(
        .Reg_Addr           (D_rs),
        .Reg_Data_now       (D_Reg_Data_R_rs),

        .FWD_M_Reg_Addr     (FWD_M_Reg_Addr),
        .FWD_M_Reg_Data     (FWD_M_Reg_Data),
        .FWD_M_T_new        (FWD_M_T_new),
        .FWD_M_Reg_WriteEn  (FWD_M_Reg_WriteEn),

        .FWD_W_Reg_Addr     (FWD_W_Reg_Addr),
        .FWD_W_Reg_Data     (FWD_W_Reg_Data),
        .FWD_W_T_new        (FWD_W_T_new),
        .FWD_W_Reg_WriteEn  (FWD_W_Reg_WriteEn),

        .Reg_Data_new       (D_Reg_Data_R_rs_final)
    );
    ReceiveFWD FWD_ID_EX_rt(
        .Reg_Addr           (D_rt),
        .Reg_Data_now       (D_Reg_Data_R_rt),

        .FWD_M_Reg_Addr     (FWD_M_Reg_Addr),
        .FWD_M_Reg_Data     (FWD_M_Reg_Data),
        .FWD_M_T_new        (FWD_M_T_new),
        .FWD_M_Reg_WriteEn  (FWD_M_Reg_WriteEn),

        .FWD_W_Reg_Addr     (FWD_W_Reg_Addr),
        .FWD_W_Reg_Data     (FWD_W_Reg_Data),
        .FWD_W_T_new        (FWD_W_T_new),
        .FWD_W_Reg_WriteEn  (FWD_W_Reg_WriteEn),

        .Reg_Data_new       (D_Reg_Data_R_rt_final)
    );

    PipeLineRegister ID_EX(
        .PC             (D_PC),
        .Instruct       (D_instruct),
        .Reg_Data_rs    (D_Reg_Data_R_rs_final),
        .Reg_Data_rt    (D_Reg_Data_R_rt_final),
        
        .T_new          (D_T_new),

        .ExcCode        (D_ExcCode_final == `Exc_AdEL ? 32'h0 : D_ExcCode_final),
        .BD             (D_BD),
        
        .clk            (clk),
        .reset          (reset),
        .ReqClr         (req), //
        .clr            (stall),
        .en             (1'b1),
        
        .PC_n           (E_PC),
        .Instruct_n     (E_instruct),
        .Reg_Data_rs_n  (E_Reg_Data_rs),
        .Reg_Data_rt_n  (E_Reg_Data_rt),
        
        .FWD_T_new      (FWD_E_T_new),

        .ExcCode_n      (E_ExcCode),
        .BD_n           (E_BD)
    );

    Controller E_FWD(
        .instruct       (E_instruct),

        .Reg_Addr_W     (FWD_E_Reg_Addr),
        .Reg_WriteEn    (FWD_E_Reg_WriteEn)
    );


// ---------------------E-----------------------


    Controller E_ctrl_ALU(
        .instruct       (E_instruct),
        
        .ALU_inB_UseImm (E_ctrl_ALU_inB_UseImm),
        .ALU_immSignExt (E_ctrl_ALU_immSignExt),
        .ALU_ALUctrl    (E_ctrl_ALU_ALUctrl),
        .ALU_upperLoad  (E_ctrl_ALU_upperLoad),

        .MULT_DIV_start (E_ctrl_MULT_DIV_start),
        .MULT_DIV_ctrl  (E_ctrl_MULT_DIV_ctrl),
        .MULT_DIV_mfhi  (E_ctrl_MULT_DIV_mfhi),
        .MULT_DIV_mflo  (E_ctrl_MULT_DIV_mflo),
        .MULT_DIV_mthi  (E_ctrl_MULT_DIV_mthi),
        .MULT_DIV_mtlo  (E_ctrl_MULT_DIV_mtlo),

        .Link_PC8       (E_ctrl_linkPC8)
    );

    InstructSplitter E_splitter(
        .instruct   (E_instruct),

        .rs         (E_rs),
        .rt         (E_rt),
        .rd         (E_rd),
        .imm16      (E_imm16)
    );

    // BitExtend
    always @(*) begin
        E_imm16_zeroExt = {{16{1'b0}}, E_imm16};
        if(E_imm16[15] == 0)
            E_imm16_signExt = {{16{1'b0}}, E_imm16};
        else
            E_imm16_signExt = {{16{1'b1}}, E_imm16};
    end

    // ALU_in MUX
    always @(*) begin
        if    (E_rs == FWD_M_Reg_Addr && FWD_M_T_new == 2'h0 && FWD_M_Reg_WriteEn)
            E_ALU_inA = FWD_M_Reg_Data;
        else if(E_rs == FWD_W_Reg_Addr && FWD_W_T_new == 2'h0 && FWD_W_Reg_WriteEn)
            E_ALU_inA = FWD_W_Reg_Data;
        else
            E_ALU_inA = E_Reg_Data_rs;
    end
    always @(*) begin
        if(E_ctrl_ALU_inB_UseImm) begin
            if(E_ctrl_ALU_immSignExt)
                E_ALU_inB = E_imm16_signExt;
            else
                E_ALU_inB = E_imm16_zeroExt;
        end
        else if(E_rt == FWD_M_Reg_Addr && FWD_M_T_new == 2'h0 && FWD_M_Reg_WriteEn)
            E_ALU_inB = FWD_M_Reg_Data;
        else if(E_rt == FWD_W_Reg_Addr && FWD_W_T_new == 2'h0 && FWD_W_Reg_WriteEn)
            E_ALU_inB = FWD_W_Reg_Data;
        else begin
            E_ALU_inB = E_Reg_Data_rt;
        end
    end

    ALU ALU(
        .inA        (E_ALU_inA),
        .inB        (E_ALU_inB),
        .ALUctrl    (E_ctrl_ALU_ALUctrl),
        .upperLoad  (E_ctrl_ALU_upperLoad),
        .zero       (E_ALU_zero),
        .result     (E_ALU_result)
    );

    // ExcCode - Ov(add/sub) AdEL(load) AdES(store)
    Controller E_Exc_Ov(
        .instruct       (E_instruct),

        .add_addi       (E_add_addi),
        .sub            (E_sub),
        .load           (E_load),
        .store          (E_store),

        .mtc0           (E_mtc0)
    );
    assign E_inA_ex = {E_ALU_inA[31], E_ALU_inA};
    assign E_inB_ex = {E_ALU_inB[31], E_ALU_inB};
    assign E_add_ex = E_inA_ex + E_inB_ex;
    assign E_sub_ex = E_inA_ex - E_inB_ex;
    assign E_Ov = (E_add_addi && (E_add_ex[32] != E_add_ex[31])) ||
                  (E_sub && (E_sub_ex[32] != E_sub_ex[31]));
    always @(*) begin
        if(E_Ov) begin
            if(E_add_addi || E_sub)
                E_ExcCode_final = `Exc_Ov;
            else if(E_load)
                E_ExcCode_final = `Exc_AdEL;
            else
                E_ExcCode_final = `Exc_AdES;
        end
        else
            E_ExcCode_final = E_ExcCode;
    end

    // MULT_DIV
    MULT_DIV MULT_DIV(
        .inA            (E_ALU_inA),
        .inB            (E_ALU_inB),
        .start          (E_ctrl_MULT_DIV_start),
        .mult_div_ctrl  (E_ctrl_MULT_DIV_ctrl),
        .mthi           (E_ctrl_MULT_DIV_mthi),
        .mtlo           (E_ctrl_MULT_DIV_mtlo),
        .dataW          (E_ALU_inA),
        .reset          (reset),
        .clk            (clk),
        .req            (req),

        .busy           (E_MULT_DIV_busy),
        .HI             (E_MULT_DIV_HI),
        .LO             (E_MULT_DIV_LO)

    );

    // choose ALU / MULT_DIV / PC
    always @(*) begin
        if(E_ctrl_linkPC8) begin
            E_ALU_result_final = E_PC + 32'h0000_0008;
        end
        else if(E_ctrl_MULT_DIV_mfhi) begin
            E_ALU_result_final = E_MULT_DIV_HI;
        end
        else if(E_ctrl_MULT_DIV_mflo) begin
            E_ALU_result_final = E_MULT_DIV_LO;
        end
        else begin
            E_ALU_result_final = E_ALU_result;
        end
    end

// -------------------EX/MEM--------------------

    // for instruct sw save right data of register
    /*
    | D | E | M | W |
        sw   nop  ori
             sw   nop
             |
             rt will change but can't be forwarded
    */
    ReceiveFWD FWD_EX_MEM_rt(
        .Reg_Addr           (E_rt),
        .Reg_Data_now       (E_Reg_Data_rt),

        .FWD_M_Reg_Addr     (FWD_M_Reg_Addr),
        .FWD_M_Reg_Data     (FWD_M_Reg_Data),
        .FWD_M_T_new        (FWD_M_T_new),
        .FWD_M_Reg_WriteEn  (FWD_M_Reg_WriteEn),

        .FWD_W_Reg_Addr     (FWD_W_Reg_Addr),
        .FWD_W_Reg_Data     (FWD_W_Reg_Data),
        .FWD_W_T_new        (FWD_W_T_new),
        .FWD_W_Reg_WriteEn  (FWD_W_Reg_WriteEn),
        
        .Reg_Data_new       (E_Reg_Data_rt_final)
    );

    PipeLineRegister EX_MEM(
        .PC             (E_PC),
        .Instruct       (E_instruct),
        .ALU_result     (E_ALU_result_final),
        .Reg_Data_rt    (E_Reg_Data_rt_final),

        .T_new          (FWD_E_T_new),

        .ExcCode        (E_ExcCode_final),
        .BD             (E_BD),

        .clk            (clk),
        .reset          (reset),
        .ReqClr         (req),
        .clr            (1'b0),
        .en             (1'b1),

        .PC_n           (M_PC),
        .Instruct_n     (M_instruct),
        .ALU_result_n   (M_ALU_result),
        .Reg_Data_rt_n  (M_Reg_Data_rt),

        .FWD_T_new      (FWD_M_T_new),

        .ExcCode_n      (M_ExcCode),
        .BD_n           (M_BD)
    );

    // M FWD
    Controller M_FWD(
        .instruct       (M_instruct),

        .Reg_WriteEn    (FWD_M_Reg_WriteEn),
        .Reg_Addr_W     (FWD_M_Reg_Addr)
    );
    assign FWD_M_Reg_Data = M_ALU_result;

// ---------------------M-----------------------


    Controller M_ctrl(
        .instruct       (M_instruct),

        .Mem_ByteWriteEn(M_ctrl_Mem_ByteWriteEn),
        .Mem_ByteRead   (M_ctrl_Mem_ByteRead)
    );

    InstructSplitter M_splitter(
        .instruct   (M_instruct),
        
        .rs         (M_rs),
        .rt         (M_rt),
        .rd         (M_rd)
    );

    // FWD MUX
    always @(*) begin
        if(M_rt == FWD_W_Reg_Addr && FWD_W_T_new == 2'h0 && FWD_W_Reg_WriteEn)
            M_Reg_Data_rt_final = FWD_W_Reg_Data;
        else
            M_Reg_Data_rt_final = M_Reg_Data_rt;
    end

    // Mem Read
    assign M_Mem_Read = (m_data_rdata >> {M_ALU_result[1:0], 3'b000}) & M_ctrl_Mem_ByteRead;
    
    // BitExtend
    always @(*) begin
        if(M_ctrl_Mem_ByteRead == 32'hffff_ffff)
            M_Mem_Data = M_Mem_Read;
        else if(M_ctrl_Mem_ByteRead == 32'h0000_ffff) begin
            if(M_Mem_Read[15] == 1'b0)
                M_Mem_Data = M_Mem_Read;
            else
                M_Mem_Data = {16'hffff, M_Mem_Read[15:0]};
        end
        else if(M_ctrl_Mem_ByteRead == 32'h0000_00ff) begin
            if(M_Mem_Read[7] == 1'b0)
                M_Mem_Data = M_Mem_Read;
            else
                M_Mem_Data = {24'hff_ffff, M_Mem_Read[7:0]};
        end
        else begin
            M_Mem_Data = 32'h0000_0000;
        end
    end

    // write
    assign m_data_addr = M_ALU_result;
    assign m_data_wdata = M_Reg_Data_rt_final << {M_ALU_result[1:0], 3'b000};
    // assign m_data_byteen = M_ctrl_Mem_ByteWriteEn << M_ALU_result[1:0];
    assign m_inst_addr = M_PC;

    // Bridge
    Bridge M_Bridge(
        .addr           (m_data_addr),
        .data_w         (m_data_wdata),

        .byteen         ((M_ExcCode_final == 5'b00000 && !req) ? (M_ctrl_Mem_ByteWriteEn << M_ALU_result[1:0]) : 4'b0000),

        .DM_data_r      (M_Mem_Data),
        .Timer0_data_r  (Timer0_data_r),
        .Timer1_data_r  (Timer1_data_r),

        .data_r_final   (M_Data_R_final),

        .DM_byteen      (m_data_byteen),
        .Timer0_WriteEn (Timer0_WriteEn),
        .Timer1_WriteEn (Timer1_WriteEn)
    );

    // TC
    TC TC0(
        .clk        (clk),
        .reset      (reset),
        .Addr       (m_data_addr[31:2]),
        .WE         (Timer0_WriteEn),
        .Din        (m_data_wdata),

        .Dout       (Timer0_data_r),
        .IRQ        (Timer0_IRQ)
    );
    TC TC1(
        .clk        (clk),
        .reset      (reset),
        .Addr       (m_data_addr[31:2]),
        .WE         (Timer1_WriteEn),
        .Din        (m_data_wdata),

        .Dout       (Timer1_data_r),
        .IRQ        (Timer1_IRQ)
    );
    // interrupt
    assign HWInt_final = {3'b000, interrupt, Timer1_IRQ, Timer0_IRQ};
    assign m_int_addr = m_data_addr;
    assign m_int_byteen = M_ctrl_Mem_ByteWriteEn;// << M_ALU_result[1:0];

    // ExcCode - AdEL / AdES
    assign macroscopic_pc = M_PC;

    assign M_lb = M_ctrl_Mem_ByteRead == 32'h0000_00ff;
    assign M_lh = M_ctrl_Mem_ByteRead == 32'h0000_ffff;
    assign M_lw = M_ctrl_Mem_ByteRead == 32'hffff_ffff;
    assign M_sb = M_ctrl_Mem_ByteWriteEn == 4'b0001;
    assign M_sh = M_ctrl_Mem_ByteWriteEn == 4'b0011;
    assign M_sw = M_ctrl_Mem_ByteWriteEn == 4'b1111;
    always @(*) begin
        // if      (|HWInt_final)
        //     M_ExcCode_final = `Exc_Int;
        if     (M_lw && (m_data_addr[1:0] != 2'b00))
            M_ExcCode_final = `Exc_AdEL;
        else if(M_lh && (m_data_addr[0] != 1'b0))
            M_ExcCode_final = `Exc_AdEL;
        else if((M_lb || M_lh) && ((32'h0000_7f00 <= m_data_addr && m_data_addr <= 32'h0000_7f0b) ||
                                   (32'h0000_7f10 <= m_data_addr && m_data_addr <= 32'h0000_7f1b) ) )
            M_ExcCode_final = `Exc_AdEL;
        else if((M_lb || M_lh || M_lw) && !((32'h0000_0000 <= m_data_addr && m_data_addr <= 32'h0000_2fff) ||
                                            (32'h0000_7f00 <= m_data_addr && m_data_addr <= 32'h0000_7f0b) ||
                                            (32'h0000_7f10 <= m_data_addr && m_data_addr <= 32'h0000_7f1b) ||
                                            (32'h0000_7f20 <= m_data_addr && m_data_addr <= 32'h0000_7f23) ) )
            M_ExcCode_final = `Exc_AdEL;
        else if(M_sw && (m_data_addr[1:0] != 2'b00))
            M_ExcCode_final = `Exc_AdES;
        else if(M_sh && (m_data_addr[0] != 1'b0))
            M_ExcCode_final = `Exc_AdES;
        else if((M_sb || M_sh) && ((32'h0000_7f00 <= m_data_addr && m_data_addr <= 32'h0000_7f0b) ||
                                   (32'h0000_7f10 <= m_data_addr && m_data_addr <= 32'h0000_7f1b) ) )
            M_ExcCode_final = `Exc_AdES;
        else if(M_sw && (m_data_addr == 32'h0000_7f08 || m_data_addr == 32'h0000_7f18))
            M_ExcCode_final = `Exc_AdES;
        else if((M_sb || M_sh || M_sw) && !((32'h0000_0000 <= m_data_addr && m_data_addr <= 32'h0000_2fff) ||
                                            (32'h0000_7f00 <= m_data_addr && m_data_addr <= 32'h0000_7f0b) ||
                                            (32'h0000_7f10 <= m_data_addr && m_data_addr <= 32'h0000_7f1b) ||
                                            (32'h0000_7f20 <= m_data_addr && m_data_addr <= 32'h0000_7f23) ) )
            M_ExcCode_final = `Exc_AdES;
        else
            M_ExcCode_final = M_ExcCode;
    end

    // CP0 Ctrl
    Controller CP0_ctrl(
        .instruct   (M_instruct),
        
        .eret       (M_eret),
        .mfc0       (M_mfc0),
        .mtc0       (M_mtc0)
    );

    // CP0
    CP0 M_CP0(
        .clk        (clk),
        .reset      (reset),
        .en         (M_mtc0),
        .CP0_Addr   (M_rd),
        .CP0_in     (M_Reg_Data_rt_final),
        .VPC        (M_PC),
        .BD_in      (M_BD),
        .ExcCode_in (M_ExcCode_final),
        .HWInt      (HWInt_final),
        .EXL_clr    (M_eret),

        .CP0_out    (CP0_out),
        .EPC_out    (CP0_EPC),
        .Req        (req)
    );

// -------------------MEM/WB---------------------

    PipeLineRegister MEM_WB(
        .PC             (M_PC),
        .Instruct       (M_instruct),
        .ALU_result     (M_ALU_result),
        .Mem_Data_read  (M_Data_R_final),

        .T_new          (FWD_M_T_new),
        .CP0_out        (CP0_out),

        .clk            (clk),
        .reset          (reset),
        .ReqClr         (req),
        .clr            (1'b0),
        .en             (1'b1),

        .PC_n           (W_PC),
        .Instruct_n     (W_instruct),
        .ALU_result_n   (W_ALU_result),
        .Mem_Data_read_n(W_Mem_Data),

        .FWD_T_new      (FWD_W_T_new),
        .CP0_out_n      (W_CP0_out)
    );


// ---------------------W-----------------------

    Controller W_ctrl(
        .instruct           (W_instruct),
        
        .Reg_W2rd           (W_ctrl_Reg_W2rd),
        .Reg_WriteEn        (W_ctrl_Reg_WriteEn),
        .Reg_W2_31          (W_ctrl_Reg_W2_31),
        .Reg_WriteMemData   (W_ctrl_Reg_WriteMemData),

        .Link_PC8           (W_ctrl_LinkPC8),

        .mfc0               (W_ctrl_mfc0)
    );

    InstructSplitter W_splitter(
        .instruct   (W_instruct),

        .rt         (W_rt),
        .rd         (W_rd)
    );

    // Reg_Addr_W MUX
    always @(*) begin
        if(W_ctrl_Reg_W2_31)
            W_Reg_Addr_W_final = 5'b11111;
        else if(W_ctrl_Reg_W2rd)
            W_Reg_Addr_W_final = W_rd;
        else
            W_Reg_Addr_W_final = W_rt;
    end
    // Reg_Data_W MUX
    always @(*) begin
        if(W_ctrl_Reg_WriteMemData)
            W_Reg_Data_W_final = W_Mem_Data;
        else if(W_ctrl_LinkPC8)
            W_Reg_Data_W_final = W_PC + 32'h0000_0008;
        else if(W_ctrl_mfc0)
            W_Reg_Data_W_final = W_CP0_out;
        else
            W_Reg_Data_W_final = W_ALU_result;
    end

    // W FWD Data
    assign FWD_W_Reg_Addr = W_Reg_Addr_W_final;
    assign FWD_W_Reg_Data = W_Reg_Data_W_final;
    assign FWD_W_Reg_WriteEn = W_ctrl_Reg_WriteEn;

    // tb
    assign w_grf_we = W_ctrl_Reg_WriteEn;
    assign w_grf_addr = W_Reg_Addr_W_final;
    assign w_grf_wdata = W_Reg_Data_W_final;
    assign w_inst_addr = W_PC;

endmodule
