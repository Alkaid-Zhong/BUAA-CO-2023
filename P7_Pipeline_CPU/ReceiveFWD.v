`timescale 1ns / 1ps
//////////////////////////////////////////////////////////////////////////////////
// Company: 
// Engineer: 
// 
// Create Date:    17:34:32 11/11/2023 
// Design Name: 
// Module Name:    ReceiveFWD 
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
module ReceiveFWD(
    input [4:0]         Reg_Addr,
    input [31:0]        Reg_Data_now,

    input [4:0]         FWD_M_Reg_Addr,
    input [31:0]        FWD_M_Reg_Data,
    input [2:0]         FWD_M_T_new,
    input               FWD_M_Reg_WriteEn,

    input [4:0]         FWD_W_Reg_Addr,
    input [31:0]        FWD_W_Reg_Data,
    input [2:0]         FWD_W_T_new,
    input               FWD_W_Reg_WriteEn,

    output reg [31:0]   Reg_Data_new
    );

    always @(*) begin

        if     (Reg_Addr == FWD_M_Reg_Addr && FWD_M_Reg_WriteEn && FWD_M_T_new == 3'b000) begin
            Reg_Data_new <= FWD_M_Reg_Data;
        end
        else if(Reg_Addr == FWD_W_Reg_Addr && FWD_W_Reg_WriteEn && FWD_W_T_new == 3'b000) begin
            Reg_Data_new <= FWD_W_Reg_Data;
        end
        else begin
            Reg_Data_new <= Reg_Data_now;
        end
    end

endmodule
