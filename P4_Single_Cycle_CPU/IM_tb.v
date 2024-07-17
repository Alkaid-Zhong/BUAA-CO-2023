`timescale 1ns / 1ps

////////////////////////////////////////////////////////////////////////////////
// Company: 
// Engineer:
//
// Create Date:   11:11:33 11/01/2023
// Design Name:   IM
// Module Name:   C:/Users/zhong/Desktop/verilog/P4_Single_Cycle_CPU/IM_tb.v
// Project Name:  P4_Single_Cycle_CPU
// Target Device:  
// Tool versions:  
// Description: 
//
// Verilog Test Fixture created by ISE for module: IM
//
// Dependencies:
// 
// Revision:
// Revision 0.01 - File Created
// Additional Comments:
// 
////////////////////////////////////////////////////////////////////////////////

module IM_tb;

	// Inputs
	reg [31:0] PC;

	// Outputs
	wire [31:0] instruct;

	// Instantiate the Unit Under Test (UUT)
	IM uut (
		.PC(PC), 
		.instruct(instruct)
	);

	always #2 PC <= PC + 4;
	initial begin
		// Initialize Inputs
		PC = 32'h0000_3000;

		// Wait 100 ns for global reset to finish

        
		// Add stimulus here

	end
      
endmodule

