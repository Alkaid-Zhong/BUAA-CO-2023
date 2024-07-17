`timescale 1ns / 1ps
//////////////////////////////////////////////////////////////////////////////////
// Company: 
// Engineer: 
// 
// Create Date:    22:46:59 12/06/2023 
// Design Name: 
// Module Name:    Bridge 
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
module Bridge(
    input [31:0]    addr,
    input [31:0]    data_w,

    input [3:0]     byteen,

    input [31:0]    DM_data_r,
    input [31:0]    Timer0_data_r,
    input [31:0]    Timer1_data_r,

    output [31:0]   addr_final,
    output [31:0]   data_w_final,

    output [31:0]   data_r_final,
    
    output [3:0]    DM_byteen,
    output [0:0]    Timer0_WriteEn,
    output [0:0]    Timer1_WriteEn
    );

    assign addr_final = addr;
    assign data_w_final = data_w;
    assign data_r_final = (32'h0000_0000 <= addr && addr <= 32'h0000_2fff) ? DM_data_r :
                          (32'h0000_7f00 <= addr && addr <= 32'h0000_7f0b) ? Timer0_data_r :
                          (32'h0000_7f10 <= addr && addr <= 32'h0000_7f1b) ? Timer1_data_r :
                          32'h0000_0000;
    assign DM_byteen = (32'h0000_0000 <= addr && addr <= 32'h0000_2fff) ? byteen : 4'b0000;
    assign Timer0_WriteEn = (32'h0000_7f00 <= addr && addr <= 32'h0000_7f0b) ? (&byteen) : 1'b0;
    assign Timer1_WriteEn = (32'h0000_7f10 <= addr && addr <= 32'h0000_7f1b) ? (&byteen) : 1'b0;
endmodule
