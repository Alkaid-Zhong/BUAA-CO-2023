`timescale 1ns / 1ps

////////////////////////////////////////////////////////////////////////////////
// Company: 
// Engineer:
//
// Create Date:   15:20:30 11/01/2023
// Design Name:   ALU
// Module Name:   C:/Users/zhong/Desktop/verilog/P4_Single_Cycle_CPU/ALU_tb.v
// Project Name:  P4_Single_Cycle_CPU
// Target Device:  
// Tool versions:  
// Description: 
//
// Verilog Test Fixture created by ISE for module: ALU
//
// Dependencies:
// 
// Revision:
// Revision 0.01 - File Created
// Additional Comments:
// 
////////////////////////////////////////////////////////////////////////////////
`define ALU_ADD 3'b000
`define ALU_SUB 3'b001
`define ALU_OR 3'b010
module ALU_tb;

	// Inputs
	reg [31:0] inA;
	reg [31:0] inB;
	reg [2:0] ALUctrl;
    reg saveHigh;

	// Outputs
	wire zero;
	wire [31:0] result;

	// Instantiate the Unit Under Test (UUT)
	ALU uut (
		.inA(inA), 
		.inB(inB), 
		.ALUctrl(ALUctrl), 
		.zero(zero), 
		.result(result),
		.saveHigh(saveHigh)
	);

	initial begin
		// Initialize Inputs
		inA = 0;
		inB = 0;
		ALUctrl = 0;
		saveHigh = 0;

		// Wait 100 ns for global reset to finish
		#2;
		inA = 0;
		inB = 16'b1111_1111_1111_1111;
		saveHigh = 1;
		ALUctrl = `ALU_OR;
		#2;
		saveHigh = 0;
		#2;
		inA = 123;
		inB = 234;
		ALUctrl = `ALU_ADD;
		#2;
		ALUctrl = `ALU_SUB;
        
		// Add stimulus here

	end
      
endmodule

