`timescale 1ns/1ps
module instruction_memory #(parameter MEM_DEPTH=256,parameter MEM_FILE="program.hex")(
 input wire [31:0] pc_addr,output reg [31:0] instruction,output wire instr_valid);
 reg [31:0] memory[0:MEM_DEPTH-1]; integer i;
 wire [29:0] word_addr=pc_addr[31:2];
 assign instr_valid=(pc_addr[1:0]==2'b00)&&(word_addr<MEM_DEPTH);
 initial begin
   for(i=0;i<MEM_DEPTH;i=i+1) memory[i]=32'h00000013;
   $readmemh(MEM_FILE,memory);
 end
 always @(*) begin
  instruction=32'h00000013;
  if(instr_valid) instruction=memory[word_addr];
 end
endmodule
