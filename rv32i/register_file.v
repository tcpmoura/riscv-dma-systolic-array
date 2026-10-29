`timescale 1ns/1ps
module register_file(
 input wire clk,rst_n,stall,reg_write,
 input wire [4:0] rs1,rs2,rd,input wire [31:0] write_data,
 output wire [31:0] read_data1,read_data2);
 reg [31:0] registers[0:31]; integer i;
 assign read_data1 = (rs1==0)?32'b0:registers[rs1];
 assign read_data2 = (rs2==0)?32'b0:registers[rs2];
 always @(posedge clk or negedge rst_n)
   if (!rst_n) for(i=0;i<32;i=i+1) registers[i] <= 32'b0;
   else if (reg_write && !stall && rd!=0) registers[rd] <= write_data;
endmodule
