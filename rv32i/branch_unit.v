`timescale 1ns/1ps
module branch_unit(input wire [31:0] a,b,input wire [2:0] funct3,input wire branch,
 output reg branch_taken);
 always @(*) begin
 branch_taken=1'b0;
 if(branch) case(funct3)
  3'b000:branch_taken=(a==b);
  3'b001:branch_taken=(a!=b);
  3'b100:branch_taken=($signed(a)<$signed(b));
  3'b101:branch_taken=($signed(a)>=$signed(b));
  3'b110:branch_taken=(a<b);
  3'b111:branch_taken=(a>=b);
  default:branch_taken=1'b0;
 endcase
 end
endmodule
