`timescale 1ns / 1ps
//////////////////////////////////////////////////////////////////////////////////
// Company: 
// Engineer: 
// 
// Create Date:    22:27:44 12/05/2023 
// Design Name: 
// Module Name:    PipeLineRegister 
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
module PipeLineRegister(
    input [31:0]        PC,
    input [31:0]        Instruct,
    input [31:0]        Reg_Data_rs,
    input [31:0]        Reg_Data_rt,
    input [31:0]        ALU_result,
    input [31:0]        Mem_Data_read,
    
    input [2:0]         T_new,

    input [4:0]         ExcCode,
    input               BD,
    input [31:0]        CP0_out,

    input               clk,
    input               en,
    input               reset,
    input               clr,
    input               ReqClr,

    output reg [31:0]   PC_n,
    output reg [31:0]   Instruct_n,
    output reg [31:0]   Reg_Data_rs_n,
    output reg [31:0]   Reg_Data_rt_n,
    output reg [31:0]   ALU_result_n,
    output reg [31:0]   Mem_Data_read_n,

    output reg [2:0]    FWD_T_new,

    output reg [4:0]    ExcCode_n,
    output reg          BD_n,
    output reg [31:0]   CP0_out_n
    );

    always @(posedge clk) begin
        if(reset) begin
            PC_n <= 32'h0000_3000;
            Instruct_n <= 32'h0000_0000;
            Reg_Data_rs_n <= 32'h0000_0000;
            Reg_Data_rt_n <= 32'h0000_0000;
            ALU_result_n <= 32'h0000_0000;
            Mem_Data_read_n <= 32'h0000_0000;
            FWD_T_new <= 3'h0;
            ExcCode_n <= 5'h0;
            BD_n <= 1'b0;
            CP0_out_n <= 32'h0;
        end
        else if(ReqClr) begin
            PC_n <= 32'h0000_4180;
            Instruct_n <= 32'h0000_0000;
            Reg_Data_rs_n <= 32'h0000_0000;
            Reg_Data_rt_n <= 32'h0000_0000;
            ALU_result_n <= 32'h0000_0000;
            Mem_Data_read_n <= 32'h0000_0000;
            FWD_T_new <= 3'h0;
            ExcCode_n <= 5'h0;
            BD_n <= 1'b0;
            CP0_out_n <= 32'h0;
        end
        else if(clr) begin
            PC_n <= PC;
            Instruct_n <= 32'h0000_0000;
            Reg_Data_rs_n <= 32'h0000_0000;
            Reg_Data_rt_n <= 32'h0000_0000;
            ALU_result_n <= 32'h0000_0000;
            Mem_Data_read_n <= 32'h0000_0000;
            FWD_T_new <= 3'h0;
            ExcCode_n <= 5'h0;
            BD_n <= BD;
            CP0_out_n <= 32'h0;
        end
        else begin
            if(en) begin
                PC_n <= PC;
                Instruct_n <= Instruct;
                Reg_Data_rs_n <= Reg_Data_rs;
                Reg_Data_rt_n <= Reg_Data_rt;
                ALU_result_n <= ALU_result;
                Mem_Data_read_n <= Mem_Data_read;
                if(T_new != 3'h0)
                    FWD_T_new <= T_new - 3'h1;
                else
                    FWD_T_new <= T_new;
                ExcCode_n <= ExcCode;
                BD_n <= BD;
                CP0_out_n <= CP0_out;
            end
            else begin
                PC_n <= PC_n;
                Instruct_n <= Instruct_n;
                Reg_Data_rs_n <= Reg_Data_rs_n;
                Reg_Data_rt_n <= Reg_Data_rt_n;
                ALU_result_n <= ALU_result_n;
                Mem_Data_read_n <= Mem_Data_read_n;
                FWD_T_new <= FWD_T_new;
                ExcCode_n <= ExcCode_n;
                BD_n <= BD_n;
                CP0_out_n <= CP0_out_n;
            end
        end
    end

endmodule
