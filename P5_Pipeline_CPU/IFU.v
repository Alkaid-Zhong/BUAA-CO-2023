`timescale 1ns / 1ps
//////////////////////////////////////////////////////////////////////////////////
// Company: 
// Engineer: 
// 
// Create Date:    10:20:36 11/01/2023 
// Design Name: 
// Module Name:    DM 
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

module IFU(
    input [31:0] PC_next,
    input clk,
    input reset,
    input En,
    output [31:0] instruct,
    output reg [31:0] PC
    );

    always @(posedge clk) begin
        if (reset) begin
            PC <= 32'h0000_3000;
        end
        else begin
            if(En)
                PC <= PC_next;
            else
                PC <= PC;
        end
    end

    IM IM(
        .PC(PC),
        .instruct(instruct)
    );


endmodule
