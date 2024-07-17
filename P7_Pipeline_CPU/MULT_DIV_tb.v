`timescale 1ns / 1ps

////////////////////////////////////////////////////////////////////////////////
// Company: 
// Engineer:
//
// Create Date:   11:17:18 11/22/2023
// Design Name:   MULT_DIV
// Module Name:   C:/Users/zhong/Desktop/verilog/P6_Pipeline_CPU/MULT_DIV_tb.v
// Project Name:  P6_Pipeline_CPU
// Target Device:  
// Tool versions:  
// Description: 
//
// Verilog Test Fixture created by ISE for module: MULT_DIV
//
// Dependencies:
// 
// Revision:
// Revision 0.01 - File Created
// Additional Comments:
// 
////////////////////////////////////////////////////////////////////////////////

`define MULT_DIV_mult    2'b00
`define MULT_DIV_multu   2'b01
`define MULT_DIV_div     2'b10
`define MULT_DIV_divu    2'b11

module MULT_DIV_tb;

	// Inputs
	reg [31:0] inA;
	reg [31:0] inB;
	reg start;
	reg [1:0] mult_div_ctrl;
	reg mthi;
	reg mtlo;
	reg [31:0] dataW;
	reg reset;
	reg clk;

	// Outputs
	wire busy;
	wire [31:0] HI;
	wire [31:0] LO;

	// Instantiate the Unit Under Test (UUT)
	MULT_DIV uut (
		.inA(inA), 
		.inB(inB), 
		.start(start), 
		.mult_div_ctrl(mult_div_ctrl), 
		.mthi(mthi), 
		.mtlo(mtlo), 
		.dataW(dataW),
		.reset(reset), 
		.clk(clk), 
		.busy(busy), 
		.HI(HI), 
		.LO(LO)
	);

	always #1 clk = ~clk;

	initial begin
		// Initialize Inputs
		inA = 0;
		inB = 0;
		start = 0;
		mult_div_ctrl = 0;
		mthi = 0;
		mtlo = 0;
		reset = 1;
		clk = 0;
		#2
		reset = 0;
		#1;
		start = 1;
		inA = -10;
		inB = 23;
		mult_div_ctrl = `MULT_DIV_mult;
		#2;
		start = 0;
		#20;
		start = 1;
		mult_div_ctrl = `MULT_DIV_multu;
		#2;
		start = 0;
		#20;
		inA = -32;
		inB = -5;
		start = 1;
		mult_div_ctrl = `MULT_DIV_div;
		#2;
		start = 0;
		#30;
		mult_div_ctrl = `MULT_DIV_divu;
		start = 1;
		#2;
		start = 0;
		#30;
		mthi = 1;
		dataW = 32'habcdef00;
		#2;
		mthi = 0;
		mtlo = 1;
		#2;
		mtlo = 0;
        
		// Add stimulus here

	end
      
endmodule

