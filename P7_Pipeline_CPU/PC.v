`timescale 1ns / 1ps
//////////////////////////////////////////////////////////////////////////////////
// Company: 
// Engineer: 
// 
// Create Date:    18:31:57 11/22/2023 
// Design Name: 
// Module Name:    PC 
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
module PC(
    input [31:0] PC_next,
    input clk,
    input reset,
    input req,
    input En,
    output reg [31:0] PC
    );

    always @(posedge clk) begin
        if (reset) begin
            PC <= 32'h0000_3000;
        end
        else if(req) begin
            PC <= 32'h0000_4180;
        end
        else begin
            if(En)
                PC <= PC_next;
            else
                PC <= PC;
        end
    end

endmodule
