`timescale 1ns / 1ps

////////////////////////////////////////////////////////////////////////////////
// Company: 
// Engineer:
//
// Create Date:   10:41:16 11/01/2023
// Design Name:   IFU
// Module Name:   C:/Users/zhong/Desktop/verilog/P4_Single_Cycle_CPU/IFU_tb.v
// Project Name:  P4_Single_Cycle_CPU
// Target Device:  
// Tool versions:  
// Description: 
//
// Verilog Test Fixture created by ISE for module: IFU
//
// Dependencies:
// 
// Revision:
// Revision 0.01 - File Created
// Additional Comments:
// 
////////////////////////////////////////////////////////////////////////////////

module IFU_tb;

	// Inputs
	reg [31:0] PC_next;
	reg clk;
	reg reset;

	// Outputs
	wire [31:0] instruct;
	wire [31:0] PC;

	// Instantiate the Unit Under Test (UUT)
	IFU uut (
		.PC_next(PC_next), 
		.clk(clk), 
		.reset(reset), 
		.instruct(instruct), 
		.PC(PC)
	);

	always #1 clk = ~clk;
	always #2 PC_next <= PC_next + 4;

	initial begin
		// Initialize Inputs
		clk = 0;
		reset = 1;
		PC_next = 32'h0000_3000;

		// Wait 100 ns for global reset to finish
		#2;
		reset = 0;
         
		// Add stimulus here

	end
      
endmodule

