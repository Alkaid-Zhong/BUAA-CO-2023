`timescale 1ns / 1ps

////////////////////////////////////////////////////////////////////////////////
// Company: 
// Engineer:
//
// Create Date:   17:08:40 11/01/2023
// Design Name:   mips
// Module Name:   C:/Users/zhong/Desktop/verilog/P4_Single_Cycle_CPU/mips_tb.v
// Project Name:  P4_Single_Cycle_CPU
// Target Device:  
// Tool versions:  
// Description: 
//
// Verilog Test Fixture created by ISE for module: mips
//
// Dependencies:
// 
// Revision:
// Revision 0.01 - File Created
// Additional Comments:
// 
////////////////////////////////////////////////////////////////////////////////

module mips_tb;

	// Inputs
	reg clk;
	reg reset;

	// Instantiate the Unit Under Test (UUT)
	mips uut (
		.clk(clk), 
		.reset(reset)
	);

	always #1 clk = ~clk;

	initial begin
		// Initialize Inputs
		clk = 1;
		reset = 1;

		// Wait 100 ns for global reset to finish
		#4;
		reset = 0;
        
		// Add stimulus here

	end
      
endmodule

