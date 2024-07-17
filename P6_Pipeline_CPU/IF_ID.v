`timescale 1ns / 1ps
//////////////////////////////////////////////////////////////////////////////////
// Company: 
// Engineer: 
// 
// Create Date:    17:12:37 11/08/2023 
// Design Name: 
// Module Name:    IF_ID 
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
module IF_ID(
    input [31:0] F_PC,
    input [31:0] F_instruct,
    input En,
    input clk,
    input reset,
    
    output reg [31:0] PC_D,
    output reg [31:0] instruct_D
    );

    always @(posedge clk) begin
        if(reset) begin
            PC_D <= 32'h0000_3000;
            instruct_D <= 32'h0000_0000;
        end
        else begin
            if(En) begin
                PC_D <= F_PC;
                instruct_D <= F_instruct;
            end
            else begin
                PC_D <= PC_D;
                instruct_D <= instruct_D;
            end
        end
    end

endmodule
