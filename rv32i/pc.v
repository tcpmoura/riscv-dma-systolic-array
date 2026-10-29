`timescale 1ns/1ps
module pc #(parameter RESET_PC=32'h00000000)(
 input wire clk,rst_n,pc_enable,input wire [31:0] pc_next,output reg [31:0] pc_out);
 always @(posedge clk or negedge rst_n)
   if (!rst_n) pc_out <= RESET_PC;
   else if (pc_enable) pc_out <= pc_next;
endmodule
