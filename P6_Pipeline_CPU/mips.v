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
    input           clk,
    input           reset,
    input [31:0]    i_inst_rdata,
    input [31:0]    m_data_rdata,

    output [31:0]   i_inst_addr,
    output [31:0]   m_data_addr,
    output [31:0]   m_data_wdata,
    output [3 :0]   m_data_byteen,
    output [31:0]   m_inst_addr,
    output          w_grf_we,
    output [4 :0]   w_grf_addr,
    output [31:0]   w_grf_wdata,
    output [31:0]   w_inst_addr
);

// ---------------------F-----------------------

    // stall signal
    reg         stall;

    wire [31:0] F_instruct;
    wire [31:0] F_PC;
    reg  [31:0] F_NPC;
    
    // PC
    PC PC(
        .PC_next    (F_NPC),
        .clk        (clk),
        .reset      (reset),
        .En         (~stall),

        .PC         (F_PC)
    );

    assign i_inst_addr = F_PC;
    assign F_instruct = i_inst_rdata;
    
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
    wire        D_MULT_DIV;

    wire        E_ctrl_MULT_DIV_start;
    wire        E_MULT_DIV_busy;

    // control signal
    wire        D_ctrl_PC_branch;
    wire        D_ctrl_PC_jIndex;
    wire        D_ctrl_PC_jr;
    wire [3:0]  D_ctrl_branch_ALU_ctrl;

    Controller D_ctrl(
        .instruct       (D_instruct),

        .T_new          (D_T_new),
        .T_use_rs       (D_T_use_rs),
        .T_use_rt       (D_T_use_rt),

        .PC_branch      (D_ctrl_PC_branch),
        .PC_jIndex      (D_ctrl_PC_jIndex),
        .PC_jr          (D_ctrl_PC_jr),
        .ALU_ALUctrl    (D_ctrl_branch_ALU_ctrl),

        .MULT_DIV       (D_MULT_DIV)
    );

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
        else if(D_MULT_DIV && (E_ctrl_MULT_DIV_start || E_MULT_DIV_busy))
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
        .upperLoad  (1'b0),
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
            F_NPC <= D_PC + 32'h0000_0004 + (D_imm16_signExt << 2);
        else if (D_ctrl_PC_jIndex)
            F_NPC <= D_index32;
        else if (D_ctrl_PC_jr)
            F_NPC <= D_Actual_Data_rs;
        else
            F_NPC <= F_PC + 32'h0000_0004;
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
        
        .FWD_E_Reg_Addr (5'b00000),
        .FWD_E_Reg_Data (32'h0000_0000),
        .FWD_E_T_new    (3'h0),
        .FWD_E_Reg_W    (1'b0),

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
    wire [3:0]  E_ctrl_ALU_ALUctrl;
    wire        E_ctrl_ALU_upperLoad;
    
    wire        E_ctrl_linkPC8;

    wire [1:0]  E_ctrl_MULT_DIV_ctrl;
    wire        E_ctrl_MULT_DIV_mfhi;
    wire        E_ctrl_MULT_DIV_mflo;
    wire        E_ctrl_MULT_DIV_mthi;
    wire        E_ctrl_MULT_DIV_mtlo;

    Controller E_ctrl(
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

    wire [31:0]     E_MULT_DIV_HI;
    wire [31:0]     E_MULT_DIV_LO;

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

        .busy           (E_MULT_DIV_busy),
        .HI             (E_MULT_DIV_HI),
        .LO             (E_MULT_DIV_LO)

    );

    reg  [31:0] E_ALU_result_final;

    always @(*) begin
        if(E_ctrl_linkPC8) begin
            E_ALU_result_final <= E_PC + 32'h0000_0008;
        end
        else if(E_ctrl_MULT_DIV_mfhi) begin
            E_ALU_result_final <= E_MULT_DIV_HI;
        end
        else if(E_ctrl_MULT_DIV_mflo) begin
            E_ALU_result_final <= E_MULT_DIV_LO;
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
        .rs             (5'b00000),
        .now_rs_Data    (32'h0000_0000),
        .rt             (E_rt),
        .now_rt_Data    (E_Reg_Data_rt),

        .FWD_E_Reg_Addr (5'b00000),
        .FWD_E_Reg_Data (32'h0000_0000),
        .FWD_E_T_new    (3'h0),
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

    wire [3:0]  M_ctrl_Mem_ByteWriteEn;
    wire [31:0] M_ctrl_Mem_ByteRead;

    Controller M_ctrl(
        .instruct       (M_instruct),

        .Mem_ByteWriteEn(M_ctrl_Mem_ByteWriteEn),
        .Mem_ByteRead   (M_ctrl_Mem_ByteRead)
    );

    reg  [31:0] M_Mem_Data_W;
    reg  [31:0] M_Mem_Data;

    wire [4:0]  M_rt;
    assign M_rt = M_instruct[20:16];

    // FWD MUX
    always @(*) begin
        if(M_rt == FWD_W_Reg_Addr && FWD_W_T_new == 2'h0 && FWD_W_Reg_W)
            M_Mem_Data_W <= FWD_W_Reg_Data;
        else
            M_Mem_Data_W <= M_Reg_Data_rt;
    end
    
    // tb
    // read
    wire [31:0] M_Mem_Read;
    assign M_Mem_Read = (m_data_rdata >> {M_ALU_result[1:0], 3'b000}) & M_ctrl_Mem_ByteRead;
    
    
    // BitExtend
    always @(*) begin
        if(M_ctrl_Mem_ByteRead == 32'hffff_ffff)
            M_Mem_Data <= M_Mem_Read;
        else if(M_ctrl_Mem_ByteRead == 32'h0000_ffff) begin
            if(M_Mem_Read[15] == 1'b0)
                M_Mem_Data <= M_Mem_Read;
            else
                M_Mem_Data <= {16'hffff, M_Mem_Read[15:0]};
        end
        else if(M_ctrl_Mem_ByteRead == 32'h0000_00ff) begin
            if(M_Mem_Read[7] == 1'b0)
                M_Mem_Data <= M_Mem_Read;
            else
                M_Mem_Data <= {24'hff_ffff, M_Mem_Read[7:0]};
        end
    end

    // write
    assign m_data_addr = M_ALU_result;
    assign m_data_wdata = M_Mem_Data_W << {M_ALU_result[1:0], 3'b000};
    assign m_data_byteen = M_ctrl_Mem_ByteWriteEn << M_ALU_result[1:0];
    assign m_inst_addr = M_PC;


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
    wire        W_ctrl_LinkPC8;

    Controller W_ctrl(
        .instruct           (W_instruct),
        
        .Reg_W2rd           (W_ctrl_Reg_W2rd),
        .Reg_WriteEn        (W_ctrl_Reg_WriteEn),
        .Reg_Link31         (W_ctrl_Reg_Link31),
        .Reg_WriteMemData   (W_ctrl_Reg_WriteMemData),
        .Link_PC8           (W_ctrl_LinkPC8)
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
        else if(W_ctrl_LinkPC8)
            W_Reg_Data_W <= W_PC + 32'h0000_0008;
        else
            W_Reg_Data_W <= W_ALU_result;
    end

    // W FWD Data
    assign FWD_W_Reg_Addr = W_Reg_Addr_W;
    assign FWD_W_Reg_Data = W_Reg_Data_W;
    assign FWD_W_Reg_W = W_ctrl_Reg_WriteEn;

    // tb
    assign w_grf_we = W_ctrl_Reg_WriteEn;
    assign w_grf_addr = W_Reg_Addr_W;
    assign w_grf_wdata = W_Reg_Data_W;
    assign w_inst_addr = W_PC;

endmodule
