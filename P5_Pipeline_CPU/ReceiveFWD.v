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
    input [4:0]     rs,
    input [31:0]    now_rs_Data,
    input [4:0]     rt,
    input [31:0]    now_rt_Data,

    input [4:0]     FWD_E_Reg_Addr,
    input [31:0]    FWD_E_Reg_Data,
    input [2:0]     FWD_E_T_new,
    input           FWD_E_Reg_W,

    input [4:0]     FWD_M_Reg_Addr,
    input [31:0]    FWD_M_Reg_Data,
    input [2:0]     FWD_M_T_new,
    input           FWD_M_Reg_W,

    input [4:0]     FWD_W_Reg_Addr,
    input [31:0]    FWD_W_Reg_Data,
    input [2:0]     FWD_W_T_new,
    input           FWD_W_Reg_W,

    output reg [31:0] new_rs_Data,
    output reg [31:0] new_rt_Data
    );

    always @(*) begin

        if(rs == FWD_E_Reg_Addr && FWD_E_Reg_W && FWD_E_T_new == 3'b000) begin
            new_rs_Data <= FWD_E_Reg_Data;
        end
        else if(rs == FWD_M_Reg_Addr && FWD_M_Reg_W && FWD_M_T_new == 3'b000) begin
            new_rs_Data <= FWD_M_Reg_Data;
        end
        else if(rs == FWD_W_Reg_Addr && FWD_W_Reg_W && FWD_W_T_new == 3'b000) begin
            new_rs_Data <= FWD_W_Reg_Data;
        end
        else begin
            new_rs_Data <= now_rs_Data;
        end

        if(rt == FWD_E_Reg_Addr && FWD_E_Reg_W && FWD_E_T_new == 3'b000) begin
            new_rt_Data <= FWD_E_Reg_Data;
        end
        else if(rt == FWD_M_Reg_Addr && FWD_M_Reg_W && FWD_M_T_new == 3'b000) begin
            new_rt_Data <= FWD_M_Reg_Data;
        end
        else if(rt == FWD_W_Reg_Addr && FWD_W_Reg_W && FWD_W_T_new == 3'b000) begin
            new_rt_Data <= FWD_W_Reg_Data;
        end
        else begin
            new_rt_Data <= now_rt_Data;
        end
    end

endmodule
