`timescale 1ns / 1ps
//////////////////////////////////////////////////////////////////////////////////
// Company: 
// Engineer: 
// 
// Create Date:    08:20:07 11/09/2023 
// Design Name: 
// Module Name:    EX_MEM 
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
module EX_MEM(
    input [31:0] E_PC,
    input [31:0] E_instruct,
    input [31:0] E_ALU_result,
    input [31:0] E_Reg_Data_rt,
    
    input [2:0]  T_new,
    
    input En,
    input clk,
    input reset,
    
    output reg [31:0] PC_M,
    output reg [31:0] instruct_M,
    output reg [31:0] ALU_result_M,
    output reg [31:0] Reg_Data_rt_M,
    
    output reg [2:0]  FWD_T_new
    );

    always @(posedge clk) begin
        if(reset) begin
            PC_M <= 32'h0000_3000;
            instruct_M <= 32'h0000_0000;
            ALU_result_M <= 32'h0000_0000;
            Reg_Data_rt_M <= 32'h0000_0000;

            FWD_T_new <= 3'h0;
        end
        else begin
            if(En) begin
                PC_M <= E_PC;
                instruct_M <= E_instruct;
                ALU_result_M <= E_ALU_result;
                Reg_Data_rt_M <= E_Reg_Data_rt;
                
                if(T_new != 3'h0)
                    FWD_T_new <= T_new - 3'h1;
                else
                    FWD_T_new <= T_new;
            end
            else begin
                PC_M <= PC_M;
                instruct_M <= instruct_M;
                ALU_result_M <= ALU_result_M;
                Reg_Data_rt_M <= Reg_Data_rt_M;
                
                FWD_T_new <= FWD_T_new;
            end
        end
    end


endmodule
