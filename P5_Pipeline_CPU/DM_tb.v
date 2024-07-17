`timescale 1ns / 1ps

////////////////////////////////////////////////////////////////////////////////
// Company: 
// Engineer:
//
// Create Date:   14:28:57 11/01/2023
// Design Name:   DM
// Module Name:   C:/Users/zhong/Desktop/verilog/P4_Single_Cycle_CPU/DM_tb.v
// Project Name:  P4_Single_Cycle_CPU
// Target Device:  
// Tool versions:  
// Description: 
//
// Verilog Test Fixture created by ISE for module: DM
//
// Dependencies:
// 
// Revision:
// Revision 0.01 - File Created
// Additional Comments:
// 
////////////////////////////////////////////////////////////////////////////////

module DM_tb;

	// Inputs
	reg [31:0] Address;
	reg [31:0] Data;
	reg MemWrite;
	reg clk;
	reg reset;

	// Outputs
	wire [31:0] MemData;

	// Instantiate the Unit Under Test (UUT)
	DM uut (
		.Address(Address), 
		.Data(Data), 
		.MemWrite(MemWrite), 
		.clk(clk), 
		.reset(reset), 
		.MemData(MemData)
	);

	always #1 clk = ~clk;
	always #2 begin
		Address <= Address + 4;
		Data <= Data + 1;
	end

	initial begin
		// Initialize Inputs
		Address = 0;
		Data = 1;
		MemWrite = 1;
		clk = 0;
		reset = 1;

		// Wait 100 ns for global reset to finish
		#2;
		reset = 0;

        
		// Add stimulus here

	end
      
endmodule

