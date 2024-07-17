`timescale 1ns / 1ps
//////////////////////////////////////////////////////////////////////////////////
// Company: 
// Engineer: 
// 
// Create Date:    08:35:25 11/09/2023 
// Design Name: 
// Module Name:    MEM_WB 
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
module MEM_WB(
    input [31:0] M_PC,
    input [31:0] M_instruct,
    input [31:0] M_ALU_result,
    input [31:0] M_Mem_Data,
    
    input [2:0]  T_new,

    input En,
    input clk,
    input reset,

    output reg [31:0] PC_W,
    output reg [31:0] instruct_W,
    output reg [31:0] ALU_result_W,
    output reg [31:0] Mem_Data_W,
    
    output reg [2:0]  FWD_T_new
    );

    always @(posedge clk) begin
        if(reset) begin
            PC_W <= 32'h0000_3000;
            instruct_W <= 32'h0000_0000;
            ALU_result_W <= 32'h0000_0000;
            Mem_Data_W <= 32'h0000_0000;
            
            FWD_T_new <= 3'h0;
        end
        else begin
            if(En) begin
                PC_W <= M_PC;
                instruct_W <= M_instruct;
                ALU_result_W <= M_ALU_result;
                Mem_Data_W <= M_Mem_Data;
                
                if(T_new != 3'h0)
                    FWD_T_new <= T_new - 3'h1;
                else
                    FWD_T_new <= T_new;
            end
            else begin
                PC_W <= PC_W;
                instruct_W <= instruct_W;
                ALU_result_W <= ALU_result_W;
                Mem_Data_W <= M_Mem_Data;
                
                FWD_T_new <= FWD_T_new;
            end
        end
    end


endmodule
