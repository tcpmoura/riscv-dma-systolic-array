`timescale 1ns/1ps
module alu(input wire [31:0] a,b,input wire [3:0] alu_control,
 output reg [31:0] result,output wire zero);
 always @(*) begin
  result=32'b0;
  case(alu_control)
    4'd0:result=a+b;
    4'd1:result=a-b;
    4'd2:result=a&b;
    4'd3:result=a|b;
    4'd4:result=a^b;
    4'd5:result=($signed(a)<$signed(b))?32'd1:32'd0;
    4'd6:result=(a<b)?32'd1:32'd0;
    4'd7:result=a<<b[4:0];
    4'd8:result=a>>b[4:0];
    4'd9:result=$signed(a)>>>b[4:0];
    default:result=32'b0;
  endcase
 end
 assign zero=(result==0);
endmodule
