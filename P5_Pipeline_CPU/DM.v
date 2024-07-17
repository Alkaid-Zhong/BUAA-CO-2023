`timescale 1ns / 1ps
//////////////////////////////////////////////////////////////////////////////////
// Company: 
// Engineer: 
// 
// Create Date:    14:15:09 11/01/2023 
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

module DM(
    input [31:0] Address,
    input [31:0] Data,
    input MemWrite,
    input clk,
    input reset,
    input [31:0] WPC,
    output [31:0] MemData
    );

    reg [31:0] mem [0:3071];

    wire [11:0] Addr;
    assign Addr = Address[13:2];
    
    integer i;
    always @(posedge clk) begin
        if(reset) begin
            for(i = 0; i < 3072; i = i + 1)
                mem[i] <= 32'h0000_0000;
        end
        else begin
            if(MemWrite) begin
                $display("%d@%h: *%h <= %h", $time, WPC, Address, Data);
                mem[Addr] <= Data;
            end
            else begin
            end
        end
    end

    assign MemData = mem[Addr];


endmodule
