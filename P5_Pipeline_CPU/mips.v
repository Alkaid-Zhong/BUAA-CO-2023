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


// ---------------------F-----------------------

    // stall signal
    reg         stall;

    wire [31:0] F_instruct;
    wire [31:0] F_PC;
    reg  [31:0] F_IFU_NPC;

    //IFU
    IFU IFU(
        .PC_next    (F_IFU_NPC),
        .clk        (clk),
        .reset      (reset),
        .En         (~stall),

        .instruct   (F_instruct),
        .PC         (F_PC)
    );
    
// -------------------IF/ID---------------------

    wire [31:0] D_instruct;
    wire [31:0] D_PC;

    IF_ID IF_ID_Reg(
        .F_PC       (F_PC),
        .F_instruct (F_instruct),
        .En         (~stall),
        .clk        (clk),
        .reset      (reset),
        
        .PC_D       (D_PC),
        .instruct_D (D_instruct)
    );

// ---------------------D-----------------------

    // FWD & stall signal
    wire [4:0]  FWD_E_Reg_Addr;
    wire [31:0] FWD_E_Reg_Data;
    wire [2:0]  FWD_E_T_new;
    wire        FWD_E_Reg_W;

    wire [4:0]  FWD_M_Reg_Addr;
    wire [31:0] FWD_M_Reg_Data;
    wire [2:0]  FWD_M_T_new;
    wire        FWD_M_Reg_W;

    wire [4:0]  FWD_W_Reg_Addr;
    wire [31:0] FWD_W_Reg_Data;
    wire [2:0]  FWD_W_T_new;
    wire        FWD_W_Reg_W;

    wire [2:0]  D_T_new;
    wire [2:0]  D_T_use_rs;
    wire [2:0]  D_T_use_rt;

    // control signal
    wire        D_ctrl_PC_branch;
    wire        D_ctrl_PC_jIndex;
    wire        D_ctrl_PC_jr;
    wire [2:0]  D_ctrl_branch_ALU_ctrl;

    Controller D_ctrl(
        .instruct       (D_instruct),

        .T_new          (D_T_new),
        .T_use_rs       (D_T_use_rs),
        .T_use_rt       (D_T_use_rt),

        .PC_branch      (D_ctrl_PC_branch),
        .PC_jIndex      (D_ctrl_PC_jIndex),
        .PC_jr          (D_ctrl_PC_jr),
        .ALU_ALUctrl    (D_ctrl_branch_ALU_ctrl)
    );

    //debug
    /*InstructTranslator DB(
        .instruct(D_instruct),
        .PC(D_PC),
        .clk(clk)
    );
    //*/

    // D Register Read Signal
    wire [4:0]  D_rs;
    wire [4:0]  D_rt;
    wire [31:0] D_Reg_Data_R_rs;
    wire [31:0] D_Reg_Data_R_rt;

    assign D_rs = D_instruct[25:21];
    assign D_rt = D_instruct[20:16];
    
    // stall
    always @(*) begin
        if     (D_rs == FWD_E_Reg_Addr && D_T_use_rs < FWD_E_T_new && FWD_E_Reg_W)
            stall <= 1'b1;
        else if(D_rt == FWD_E_Reg_Addr && D_T_use_rt < FWD_E_T_new && FWD_E_Reg_W)
            stall <= 1'b1;
        else if(D_rs == FWD_M_Reg_Addr && D_T_use_rs < FWD_M_T_new && FWD_M_Reg_W)
            stall <= 1'b1;
        else if(D_rt == FWD_M_Reg_Addr && D_T_use_rt < FWD_M_T_new && FWD_M_Reg_W)
            stall <= 1'b1;
        else if(D_rs == FWD_W_Reg_Addr && D_T_use_rs < FWD_W_T_new && FWD_W_Reg_W)
            stall <= 1'b1;
        else if(D_rt == FWD_W_Reg_Addr && D_T_use_rt < FWD_W_T_new && FWD_W_Reg_W)
            stall <= 1'b1;
        else
            stall <= 1'b0;
    end

    // Write Back Data
    wire [31:0] W_PC;
    reg  [4:0]  W_Reg_Addr_W;
    reg  [31:0] W_Reg_Data_W;
    wire        W_ctrl_Reg_WriteEn;

    // GRF
    GRF GRF(
        .AddrR1     (D_rs),
        .AddrR2     (D_rt),

        .AddrW      (W_Reg_Addr_W),//
        .DataW      (W_Reg_Data_W),//
        .WPC        (W_PC),//
        .WEn        (W_ctrl_Reg_WriteEn),//

        .clk        (clk),
        .reset      (reset),

        .DataR1     (D_Reg_Data_R_rs),
        .DataR2     (D_Reg_Data_R_rt)
    );

    // branch signal
    wire        D_branch_ALU_zero;
    wire [31:0] D_branch_ALU_result;

    reg  [31:0] D_Actual_Data_rs;
    reg  [31:0] D_Actual_Data_rt;

    // FWD MUX
    always @(*) begin
        if     (D_rs == FWD_M_Reg_Addr && FWD_M_T_new == 2'h0 && FWD_M_Reg_W)
            D_Actual_Data_rs <= FWD_M_Reg_Data;
        else if(D_rs == FWD_W_Reg_Addr && FWD_W_T_new == 2'h0 && FWD_W_Reg_W)
            D_Actual_Data_rs <= FWD_W_Reg_Data;
        else
            D_Actual_Data_rs <= D_Reg_Data_R_rs;
    end
    always @(*) begin
        if     (D_rt == FWD_M_Reg_Addr && FWD_M_T_new == 2'h0 && FWD_M_Reg_W)
            D_Actual_Data_rt <= FWD_M_Reg_Data;
        else if(D_rt == FWD_W_Reg_Addr && FWD_W_T_new == 2'h0 && FWD_W_Reg_W)
            D_Actual_Data_rt <= FWD_W_Reg_Data;
        else
            D_Actual_Data_rt <= D_Reg_Data_R_rt;
    end

    // ALU branch calculate
    ALU ALU_branch(
        .inA        (D_Actual_Data_rs),
        .inB        (D_Actual_Data_rt),
        .ALUctrl    (D_ctrl_branch_ALU_ctrl),
        .zero       (D_branch_ALU_zero),
        .result     (D_branch_ALU_result)
    );
    
    // PC_next MUX
    
    wire [15:0] D_imm16;
    wire [25:0] D_index26;
    wire [31:0] D_index32;
    reg  [31:0] D_imm16_signExt;

    assign D_imm16 = D_instruct[15:0];
    assign D_index26 = D_instruct[25:0];
    assign D_index32 = {D_PC[31:28], D_index26, 2'b00};
    always @(*) begin
        if(D_imm16[15] == 0)
            D_imm16_signExt <= {{16{1'b0}}, D_imm16};
        else
            D_imm16_signExt <= {{16{1'b1}}, D_imm16};
    end

    always @(*) begin
        if(D_ctrl_PC_branch && D_branch_ALU_zero)
            F_IFU_NPC <= D_PC + 32'h0000_0004 + (D_imm16_signExt << 2);
        else if (D_ctrl_PC_jIndex)
            F_IFU_NPC <= D_index32;
        else if (D_ctrl_PC_jr)
            F_IFU_NPC <= D_Actual_Data_rs;
        else
            F_IFU_NPC <= F_PC + 32'h0000_0004;
    end

// -------------------ID/EX---------------------

    wire [31:0] E_PC;
    wire [31:0] E_instruct;
    wire [31:0] E_Reg_Data_rs;
    wire [31:0] E_Reg_Data_rt;
    
    wire [4:0]  D_Reg_Addr_W;
    wire        D_Reg_W;

    Controller Reg_Addr_MUX(
        .instruct       (D_instruct),

        .Reg_WriteEn    (D_Reg_W),
        .Reg_Addr_W     (D_Reg_Addr_W)
    );

    wire [31:0]         D_Reg_Data_R_rs_final;
    wire [31:0]         D_Reg_Data_R_rt_final;
    
    // for instruct sw save right data of register
    /*
    | D | E | M | W |
     sw  nop nop ori
         sw  nop nop
          |
          rt will change but can't be forwarded
    */
    ReceiveFWD FWD_ID_EX(
        .rs             (D_rs),
        .now_rs_Data    (D_Reg_Data_R_rs),
        .rt             (D_rt),
        .now_rt_Data    (D_Reg_Data_R_rt),

        .FWD_M_Reg_Addr (FWD_M_Reg_Addr),
        .FWD_M_Reg_Data (FWD_M_Reg_Data),
        .FWD_M_T_new    (FWD_M_T_new),
        .FWD_M_Reg_W    (FWD_M_Reg_W),

        .FWD_W_Reg_Addr (FWD_W_Reg_Addr),
        .FWD_W_Reg_Data (FWD_W_Reg_Data),
        .FWD_W_T_new    (FWD_W_T_new),
        .FWD_W_Reg_W    (FWD_W_Reg_W),

        .new_rs_Data    (D_Reg_Data_R_rs_final),
        .new_rt_Data    (D_Reg_Data_R_rt_final)
    );

    ID_EX ID_EX_Reg(
        .D_PC           (D_PC),
        .D_instruct     (D_instruct),
        .D_Data_rs      (D_Reg_Data_R_rs_final),
        .D_Data_rt      (D_Reg_Data_R_rt_final),
        
        .T_new          (D_T_new),
        

        .En             (1'b1),
        .clk            (clk),
        .reset          (reset | stall),
        
        .PC_E           (E_PC),
        .instruct_E     (E_instruct),
        .Reg_Data_1_E   (E_Reg_Data_rs),
        .Reg_Data_2_E   (E_Reg_Data_rt),
        
        .FWD_T_new      (FWD_E_T_new)
    );

    Controller E_FWD(
        .instruct       (E_instruct),

        .Reg_Addr_W     (FWD_E_Reg_Addr),
        .Reg_WriteEn    (FWD_E_Reg_W)
    );


// ---------------------E-----------------------

    wire        E_ctrl_ALU_inB_UseImm;
    wire        E_ctrl_ALU_immSignExt;
    wire [2:0]  E_ctrl_ALU_ALUctrl;
    wire        E_ctrl_ALU_upperLoad;
    
    wire        E_ctrl_link31;

    Controller E_ctrl(
        .instruct       (E_instruct),
        
        .ALU_inB_UseImm (E_ctrl_ALU_inB_UseImm),
        .ALU_immSignExt (E_ctrl_ALU_immSignExt),
        .ALU_ALUctrl    (E_ctrl_ALU_ALUctrl),
        .ALU_upperLoad  (E_ctrl_ALU_upperLoad),

        .Reg_Link31     (E_ctrl_link31)
    );

    wire [15:0] E_imm16;
    reg  [31:0] E_imm16_signExt;
    reg  [31:0] E_imm16_zeroExt;

    assign E_imm16 = E_instruct[15:0];

    wire [31:0] E_ALU_result;
    wire        E_ALU_zero;
    reg  [31:0] E_ALU_inA;
    reg  [31:0] E_ALU_inB;

    // BitExtend
    always @(*) begin
        E_imm16_zeroExt <= {{16{1'b0}}, E_imm16};
        if(E_imm16[15] == 0)
            E_imm16_signExt <= {{16{1'b0}}, E_imm16};
        else
            E_imm16_signExt <= {{16{1'b1}}, E_imm16};
    end

    
    wire [4:0] E_rs;
    wire [4:0] E_rt;
    assign E_rs = E_instruct[25:21];
    assign E_rt = E_instruct[20:16];

    // ALU_in MUX
    always @(*) begin
        if    (E_rs == FWD_M_Reg_Addr && FWD_M_T_new == 2'h0 && FWD_M_Reg_W)
            E_ALU_inA <= FWD_M_Reg_Data;
        else if(E_rs == FWD_W_Reg_Addr && FWD_W_T_new == 2'h0 && FWD_W_Reg_W)
            E_ALU_inA <= FWD_W_Reg_Data;
        else
            E_ALU_inA <= E_Reg_Data_rs;
    end
    always @(*) begin
        if(E_ctrl_ALU_inB_UseImm) begin
            if(E_ctrl_ALU_immSignExt)
                E_ALU_inB <= E_imm16_signExt;
            else
                E_ALU_inB <= E_imm16_zeroExt;
        end
        else if(E_rt == FWD_M_Reg_Addr && FWD_M_T_new == 2'h0 && FWD_M_Reg_W)
            E_ALU_inB <= FWD_M_Reg_Data;
        else if(E_rt == FWD_W_Reg_Addr && FWD_W_T_new == 2'h0 && FWD_W_Reg_W)
            E_ALU_inB <= FWD_W_Reg_Data;
        else begin
            E_ALU_inB <= E_Reg_Data_rt;
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

    reg  [31:0] E_ALU_result_final;

    always @(*) begin
        if(E_ctrl_link31) begin
            E_ALU_result_final <= E_PC + 32'h0000_0008;
        end
        else begin
            E_ALU_result_final <= E_ALU_result;
        end
    end

// -------------------EX/MEM--------------------

    wire [31:0] M_PC;
    wire [31:0] M_instruct;
    wire [31:0] M_ALU_result;
    wire [31:0] M_Reg_Data_rt;

    wire [31:0] E_Reg_Data_rt_final;

    // for instruct sw save right data of register
    /*
    | D | E | M | W |
        sw   nop  ori
             sw   nop
             |
             rt will change but can't be forwarded
    */
    ReceiveFWD FWD_EX_MEM(
        .rt             (E_rt),
        .now_rt_Data    (E_Reg_Data_rt),

        .FWD_E_Reg_W    (1'b0),

        .FWD_M_Reg_Addr (FWD_M_Reg_Addr),
        .FWD_M_Reg_Data (FWD_M_Reg_Data),
        .FWD_M_T_new    (FWD_M_T_new),
        .FWD_M_Reg_W    (FWD_M_Reg_W),

        .FWD_W_Reg_Addr (FWD_W_Reg_Addr),
        .FWD_W_Reg_Data (FWD_W_Reg_Data),
        .FWD_W_T_new    (FWD_W_T_new),
        .FWD_W_Reg_W    (FWD_W_Reg_W),

        .new_rt_Data    (E_Reg_Data_rt_final)
    );

    EX_MEM EX_MEM_Reg(
        .E_PC           (E_PC),
        .E_instruct     (E_instruct),
        .E_ALU_result   (E_ALU_result_final),
        .E_Reg_Data_rt  (E_Reg_Data_rt_final),

        .T_new          (FWD_E_T_new),

        .En             (1'b1),
        .clk            (clk),
        .reset          (reset),

        .PC_M           (M_PC),
        .instruct_M     (M_instruct),
        .ALU_result_M   (M_ALU_result),
        .Reg_Data_rt_M  (M_Reg_Data_rt),

        .FWD_T_new      (FWD_M_T_new)
    );

    // M FWD
    Controller M_FWD(
        .instruct       (M_instruct),

        .Reg_WriteEn    (FWD_M_Reg_W),
        .Reg_Addr_W     (FWD_M_Reg_Addr)
    );
    assign FWD_M_Reg_Data = M_ALU_result;

// ---------------------M-----------------------

    wire        M_ctrl_Mem_WriteEn;

    Controller M_ctrl(
        .instruct       (M_instruct),

        .Mem_WriteEn    (M_ctrl_Mem_WriteEn)
    );

    reg  [31:0] M_Mem_Data_W;
    wire [31:0] M_Mem_Data;

    wire [4:0]  M_rt;
    assign M_rt = M_instruct[20:16];

    // FWD MUX
    always @(*) begin
        if(M_rt == FWD_W_Reg_Addr && FWD_W_T_new == 2'h0 && FWD_W_Reg_W)
            M_Mem_Data_W <= FWD_W_Reg_Data;
        else
            M_Mem_Data_W <= M_Reg_Data_rt;
    end

    //DM
    DM DM(
        .Address    (M_ALU_result),
        .Data       (M_Mem_Data_W),
        .MemWrite   (M_ctrl_Mem_WriteEn),

        .clk        (clk),
        .reset      (reset),
        .WPC        (M_PC),

        .MemData    (M_Mem_Data)
    );
    

// -------------------MEM/WB---------------------

    wire [31:0] W_instruct;
    wire [31:0] W_ALU_result;
    wire [31:0] W_Mem_Data;

    MEM_WB MEM_WB_Reg(
        .M_PC           (M_PC),
        .M_instruct     (M_instruct),
        .M_ALU_result   (M_ALU_result),
        .M_Mem_Data     (M_Mem_Data),

        .T_new          (FWD_M_T_new),

        .En             (1'b1),
        .clk            (clk),
        .reset          (reset),

        .PC_W           (W_PC),
        .instruct_W     (W_instruct),
        .ALU_result_W   (W_ALU_result),
        .Mem_Data_W     (W_Mem_Data),

        .FWD_T_new      (FWD_W_T_new)
    );


// ---------------------W-----------------------

    
    // Write Back Data (in D)
    // wire [31:0] W_PC;
    // reg  [4:0]  W_Reg_Addr_W;
    // reg  [31:0] W_Reg_Data_W;
    // wire        W_ctrl_Reg_WriteEn;

    wire        W_ctrl_Reg_W2rd;
    wire        W_ctrl_Reg_WriteMemData;
    wire        W_ctrl_Reg_Link31;

    Controller W_ctrl(
        .instruct           (W_instruct),
        
        .Reg_W2rd           (W_ctrl_Reg_W2rd),
        .Reg_WriteEn        (W_ctrl_Reg_WriteEn),
        .Reg_Link31         (W_ctrl_Reg_Link31),
        .Reg_WriteMemData   (W_ctrl_Reg_WriteMemData)
    );

    wire [4:0]  W_rt;
    wire [4:0]  W_rd;
    
    assign W_rt = W_instruct[20:16];
    assign W_rd = W_instruct[15:11];

    // Reg_Addr_W MUX
    always @(*) begin
        if(W_ctrl_Reg_Link31)
            W_Reg_Addr_W <= 5'b11111;
        else if(W_ctrl_Reg_W2rd)
            W_Reg_Addr_W <= W_rd;
        else
            W_Reg_Addr_W <= W_rt;
    end
    // Reg_Data_W MUX
    always @(*) begin
        if(W_ctrl_Reg_WriteMemData)
            W_Reg_Data_W <= W_Mem_Data;
        else if(W_ctrl_Reg_Link31)
            W_Reg_Data_W <= W_PC + 32'h0000_0008;
        else
            W_Reg_Data_W <= W_ALU_result;
    end

    // W FWD Data
    assign FWD_W_Reg_Addr = W_Reg_Addr_W;
    assign FWD_W_Reg_Data = W_Reg_Data_W;
    assign FWD_W_Reg_W = W_ctrl_Reg_WriteEn;

endmodule
