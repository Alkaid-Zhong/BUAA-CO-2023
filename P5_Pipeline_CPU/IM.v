`timescale 1ns / 1ps
//////////////////////////////////////////////////////////////////////////////////
// Company: 
// Engineer: 
// 
// Create Date:    10:28:25 11/01/2023 
// Design Name: 
// Module Name:    IM 
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
module IM(
    input [31:0] PC,
    output [31:0] instruct
    );

    reg [31:0] im [0:4095];

    //integer i;
    initial begin
        $readmemh("code.txt", im, 0);
        /*for(i = 0; i < 2048; i = i + 1) begin
            if(im[i][0] == 0 || im[i][0] == 1)
                $display("%04d: %08x", i, im[i]);
        end*/
    end
    wire [31:0] addr;
    assign addr = {{19{1'b0}}, PC[14:2]} - 32'h0000_0C00;
    assign instruct = im[addr];


endmodule
