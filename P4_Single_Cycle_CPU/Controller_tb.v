`timescale 1ns / 1ps

////////////////////////////////////////////////////////////////////////////////
// Company: 
// Engineer:
//
// Create Date:   15:21:18 11/01/2023
// Design Name:   Controller
// Module Name:   C:/Users/zhong/Desktop/verilog/P4_Single_Cycle_CPU/Controller_tb.v
// Project Name:  P4_Single_Cycle_CPU
// Target Device:  
// Tool versions:  
// Description: 
//
// Verilog Test Fixture created by ISE for module: Controller
//
// Dependencies:
// 
// Revision:
// Revision 0.01 - File Created
// Additional Comments:
// 
////////////////////////////////////////////////////////////////////////////////

module Controller_tb;

	// Inputs
	reg [31:0] instruct;

	// Outputs
	wire RegSrc;
	wire ALUuseImm;
	wire Mem2Reg;
	wire RegWrite;
	wire MemWrite;
	wire Branch;
	wire immSignExt;
	wire [2:0] ALUctrl;
	wire saveHigh;
	wire jump;

	// Instantiate the Unit Under Test (UUT)
	Controller uut (
		.instruct(instruct), 
		.RegSrc(RegSrc), 
		.ALUuseImm(ALUuseImm), 
		.Mem2Reg(Mem2Reg), 
		.RegWrite(RegWrite), 
		.MemWrite(MemWrite), 
		.Branch(Branch), 
		.immSignExt(immSignExt), 
		.ALUctrl(ALUctrl), 
		.saveHigh(saveHigh), 
		.jump(jump)
	);

	initial begin
		// Initialize Inputs
		instruct = 0;

		// Wait 100 ns for global reset to finish
		#2;
		instruct = 32'h00000000;
		#2;
		instruct = 32'h1c96822;
		#2;
		instruct = 32'h1a84022;
		#2;
		instruct = 32'h3c02237a;
		#2;
		instruct = 32'h1406820;
        
		// Add stimulus here

	end
      
endmodule

