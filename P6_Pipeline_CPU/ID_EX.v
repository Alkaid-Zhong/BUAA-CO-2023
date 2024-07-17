`timescale 1ns / 1ps
//////////////////////////////////////////////////////////////////////////////////
// Company: 
// Engineer: 
// 
// Create Date:    17:48:50 11/08/2023 
// Design Name: 
// Module Name:    ID_EX 
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
module ID_EX(
    input [31:0] D_PC,
    input [31:0] D_instruct,
    input [31:0] D_Data_rs,
    input [31:0] D_Data_rt,
    
    input [2:0]  T_new,
    
    input En,
    input clk,
    input reset,
    
    output reg [31:0] PC_E,
    output reg [31:0] instruct_E,
    output reg [31:0] Reg_Data_1_E,
    output reg [31:0] Reg_Data_2_E,
    
    output reg [2:0]  FWD_T_new
    );

    always @(posedge clk) begin
        if(reset) begin
            PC_E <= 32'h0000_3000;
            instruct_E <= 32'h0000_0000;
            Reg_Data_1_E <= 32'h0000_0000;
            Reg_Data_1_E <= 32'h0000_0000;
            
            FWD_T_new <= 3'h0;
        end
        else begin
            if(En) begin
                PC_E <= D_PC;
                instruct_E <= D_instruct;
                Reg_Data_1_E <= D_Data_rs;
                Reg_Data_2_E <= D_Data_rt;
                
                if(T_new != 3'h0)
                    FWD_T_new <= T_new - 3'h1;
                else
                    FWD_T_new <= T_new;
            end
            else begin
                PC_E <= PC_E;
                instruct_E <= instruct_E;
                Reg_Data_1_E <= Reg_Data_1_E;
                Reg_Data_2_E <= Reg_Data_2_E;
                
                FWD_T_new <= FWD_T_new;
            end
        end
    end

endmodule
