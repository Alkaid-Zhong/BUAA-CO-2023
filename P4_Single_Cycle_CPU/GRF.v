`timescale 1ns / 1ps
//////////////////////////////////////////////////////////////////////////////////
// Company: 
// Engineer: 
// 
// Create Date:    12:04:48 11/01/2023 
// Design Name: 
// Module Name:    GRF 
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

module GRF(
    input [4:0] AddrR1,
    input [4:0] AddrR2,
    input [4:0] AddrW,
    input [31:0] DataW,
    input [31:0] WPC,
    input WEn,
    input clk,
    input reset,
    output [31:0] DataR1,
    output [31:0] DataR2
    );

    reg [31:0] register [0:31];

    integer i;
    always @(posedge clk) begin
        if(reset) begin
            for(i = 0; i < 32; i = i + 1) begin
                register[i] <= 32'h0000_0000;
            end
        end
        else begin
            if(WEn) begin
                $display("@%h: $%d <= %h", WPC, AddrW, DataW);
                register[5'b00000] <= 32'h0000_0000;
                register[AddrW] <= DataW;
            end
        end
    end

    assign DataR1 = register[AddrR1];
    assign DataR2 = register[AddrR2];


endmodule
