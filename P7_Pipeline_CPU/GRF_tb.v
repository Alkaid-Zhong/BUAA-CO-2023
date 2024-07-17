`timescale 1ns / 1ps

////////////////////////////////////////////////////////////////////////////////
// Company: 
// Engineer:
//
// Create Date:   12:09:04 11/01/2023
// Design Name:   GRF
// Module Name:   C:/Users/zhong/Desktop/verilog/P4_Single_Cycle_CPU/GRF_tb.v
// Project Name:  P4_Single_Cycle_CPU
// Target Device:  
// Tool versions:  
// Description: 
//
// Verilog Test Fixture created by ISE for module: GRF
//
// Dependencies:
// 
// Revision:
// Revision 0.01 - File Created
// Additional Comments:
// 
////////////////////////////////////////////////////////////////////////////////

module GRF_tb;

	// Inputs
	reg [4:0] AddrR1;
	reg [4:0] AddrR2;
	reg [4:0] AddrW;
	reg [31:0] WData;
	reg WEn;
	reg clk;
	reg reset;

	// Outputs
	wire [31:0] RData1;
	wire [31:0] RData2;

	// Instantiate the Unit Under Test (UUT)
	GRF uut (
		.AddrR1(AddrR1), 
		.AddrR2(AddrR2), 
		.AddrW(AddrW), 
		.WData(WData), 
		.WEn(WEn), 
		.clk(clk), 
		.reset(reset), 
		.RData1(RData1), 
		.RData2(RData2)
	);

	always #1 clk = ~clk;

	initial begin
		// Initialize Inputs
		AddrR1 = 0;
		AddrR2 = 0;
		AddrW = 0;
		WData = 0;
		WEn = 1;
		clk = 0;
		reset = 1;

		// Wait 100 ns for global reset to finish
		#2;
		reset = 0;
		AddrR1 = 1;
		AddrR2 = 2;
		AddrW = 1;
		WData = 15;
		#2;
		AddrW = 2;
		WData = 22;
        
		// Add stimulus here

	end
      
endmodule

