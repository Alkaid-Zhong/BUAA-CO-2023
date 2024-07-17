`timescale 1ns / 1ps
//////////////////////////////////////////////////////////////////////////////////
// Company: 
// Engineer: 
// 
// Create Date:    17:17:44 12/06/2023 
// Design Name: 
// Module Name:    CP0 
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

// `define Exc_Int     5'd0
// `define Exc_AdEL    5'd4
// `define Exc_AdES    5'd5
// `define Exc_Syscall 5'd8
// `define Exc_RI      5'd10
// `define Exc_Ov      5'd12

`define SR          CP0_Reg[12]
`define Cause       CP0_Reg[13]
`define EPC         CP0_Reg[14]

// SR R/W
`define IM          15:10           // Interrupt Mask
`define EXL         1               // Exception Level
`define IE          0               // Interrupt Enable

// Cause R
`define BD          31              // Branch Delay
`define IP          15:10           // Interrupt Pending
`define ExcCode     6:2             // ExcCode

module CP0(
    input           clk,
    input           reset,
    input           en,
    input [4:0]     CP0_Addr,
    input [31:0]    CP0_in,
    input [31:0]    VPC,
    input           BD_in,
    input [4:0]     ExcCode_in,
    input [5:0]     HWInt,          // HardWare Interrupt
    input           EXL_clr,

    output [31:0]   CP0_out,
    output [31:0]   EPC_out,
    output          Req
    );

    reg [31:0]  CP0_Reg[0:31];
    wire        Req_interrupt;
    wire        Req_Exception;

    integer i;
    always @(posedge clk) begin
        if(reset) begin
            for(i = 0; i < 32; i = i + 1) begin
                CP0_Reg[i] <= 32'h0000_0000;
            end
        end
        else begin
            `Cause[`IP] <= HWInt;
            // $display("Cause: %h", `Cause);
            if(Req) begin
                `SR[`EXL] <= 1'b1;
                `EPC <= BD_in ? (VPC - 32'h4) : VPC;
                `Cause[`ExcCode] <= Req_interrupt ? 5'b00000 : ExcCode_in;
                `Cause[`BD] <= BD_in;
            end
            else if(en) begin
                if(CP0_Addr == 5'd12) begin
                    `SR[`IM] <= CP0_in[`IM];
                    `SR[`EXL] <= CP0_in[`EXL];
                    `SR[`IE] <= CP0_in[`IE];
                    // $display("%d@%h: $%d <= %h (CP0)", $time, VPC, CP0_Addr, CP0_in);
                end
                else if(CP0_Addr == 5'd14) begin
                    `EPC <= CP0_in;
                    // $display("%d@%h: $%d <= %h (CP0)", $time, VPC, CP0_Addr, CP0_in);
                end
            end
            else if(EXL_clr) begin
                `SR[`EXL] <= 1'b0;
            end
            else begin
                // none
            end
        end
    end

    assign CP0_out = CP0_Reg[CP0_Addr];
    assign EPC_out = `EPC;

    assign Req_interrupt = ((|(HWInt & `SR[`IM])) & `SR[`IE] & (!`SR[`EXL]));
    assign Req_Exception = (ExcCode_in != 5'b00000) & (!`SR[`EXL]);

    assign Req = Req_interrupt | Req_Exception;

endmodule
