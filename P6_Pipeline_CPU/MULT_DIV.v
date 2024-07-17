`timescale 1ns / 1ps
//////////////////////////////////////////////////////////////////////////////////
// Company: 
// Engineer: 
// 
// Create Date:    10:59:20 11/22/2023 
// Design Name: 
// Module Name:    MULT_DIV 
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

`define MULT_DIV_mult    2'b00
`define MULT_DIV_multu   2'b01
`define MULT_DIV_div     2'b10
`define MULT_DIV_divu    2'b11

module MULT_DIV(
    input [31:0]        inA,
    input [31:0]        inB,
    input               start,
    input [1:0]         mult_div_ctrl,
    input               mthi,
    input               mtlo,
    input [31:0]        dataW,
    input               reset,
    input               clk,

    output              busy,
    output reg [31:0]   HI,
    output reg [31:0]   LO
    );

    reg [3:0]  timer;

    reg [31:0] HI_temp;
    reg [31:0] LO_temp;

    always @(posedge clk) begin
        if(reset) begin
            timer <= 4'h0;
            HI <= 32'h0;
            LO <= 32'h0;
            HI_temp <= 32'h0;
            LO_temp <= 32'h0;
        end
        else if(start) begin
            if(mult_div_ctrl == `MULT_DIV_mult) begin
                timer <= 4'h5;
                {HI_temp, LO_temp} <= $signed(inA) * $signed(inB);
            end
            else if(mult_div_ctrl == `MULT_DIV_multu) begin
                timer <= 4'h5;
                {HI_temp, LO_temp} <= inA * inB;
            end
            else if(mult_div_ctrl == `MULT_DIV_div) begin
                timer <= 4'ha;
                LO_temp <= $signed(inA) / $signed(inB);
                HI_temp <= $signed(inA) % $signed(inB);
            end
            else if(mult_div_ctrl == `MULT_DIV_divu) begin
                timer <= 4'ha;
                LO_temp <= inA / inB;
                HI_temp <= inA % inB;
            end
            else begin
                timer <= 4'h0;
                HI_temp <= HI;
                LO_temp <= LO;
            end
        end
        else if(mthi) begin
            HI <= dataW;
        end
        else if(mtlo) begin
            LO <= dataW;
        end
        else begin
            if(timer > 4'h1) begin
                timer <= timer - 4'h1;
            end
            else if(timer == 4'h1) begin
                timer <= timer - 4'h1;
                HI <= HI_temp;
                LO <= LO_temp;
            end
            else begin
                timer <= timer;
                HI <= HI;
                LO <= LO;
                HI_temp <= HI_temp;
                LO_temp <= LO_temp;
            end
        end
    end

    assign busy = timer != 4'h0;

endmodule
