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

`define     ALU_ADD     4'b0000
`define     ALU_SUB     4'b0001
`define     ALU_OR      4'b0010
`define     ALU_AND     4'b0011
`define     ALU_SLT     4'b0100
`define     ALU_SLTU    4'b0101
`define     ALU_BNE     4'b0110
`define     ALU_NOP     4'b1111

module ALU_tb;

	// Inputs
	reg [31:0] inA;
	reg [31:0] inB;
	reg [3:0] ALUctrl;
    reg upperLoad;

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
		.upperLoad(upperLoad)
	);

	initial begin
		// Initialize Inputs
		inA = 0;
		inB = 0;
		ALUctrl = 0;
		upperLoad = 0;

		// Wait 100 ns for global reset to finish
		#2;
		inA = 32'hffff_0000;
		inB = 32'h7fff_ffff;
		ALUctrl = `ALU_SLT;
		#2;
		ALUctrl = `ALU_SLTU;
		#2;
		ALUctrl = `ALU_AND;
		#2;
		ALUctrl = `ALU_BNE;
		#2;
		inA = 32'h0000_0001;
		inB = 32'h0000_0001;
        
		// Add stimulus here

	end
      
endmodule

