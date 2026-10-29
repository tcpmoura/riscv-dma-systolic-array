`timescale 1ns/1ps
module rv32i_soc_tb;
 reg clk=1'b0, rst_n=1'b0;
 wire valid,ready,wr,mem_error,retire,halted,trap;
 wire [31:0] addr,wdata,rdata,pc_debug;
 wire [3:0] wstrb;
 integer cycles,retired,stalled,errors;
 always #5 clk=~clk;
 rv32i_core #(.IMEM_FILE("program.hex")) cpu (
  .clk(clk),.rst_n(rst_n),.dmem_valid(valid),.dmem_ready(ready),
  .dmem_write(wr),.dmem_addr(addr),.dmem_wdata(wdata),.dmem_wstrb(wstrb),
  .dmem_rdata(rdata),.dmem_error(mem_error),.debug_pc(pc_debug),
  .halted(halted),.trap(trap),.retire(retire));
 data_memory #(.MEM_DEPTH(1024),.LATENCY(3)) ram (
  .clk(clk),.rst_n(rst_n),.req_valid(valid),.req_ready(ready),
  .req_write(wr),.req_addr(addr),.req_wdata(wdata),.req_wstrb(wstrb),
  .resp_rdata(rdata),.resp_error(mem_error));
 task expect_reg;
  input integer number;
  input [31:0] expected;
  begin
   if(cpu.rf_inst.registers[number] !== expected) begin
    $display("[FAIL] x%0d expected=%h got=%h",number,expected,cpu.rf_inst.registers[number]);
    errors=errors+1;
   end else $display("[PASS] x%0d=%h",number,expected);
  end
 endtask
 initial begin
  cycles=0;retired=0;stalled=0;errors=0;
  repeat(3) @(negedge clk);
  rst_n=1'b1;
  wait(halted||trap||cycles>=200);
  @(negedge clk);
  if(trap || !halted) begin
   $display("[FAIL] cpu trap=%b halted=%b cycles=%0d PC=%h",trap,halted,cycles,pc_debug);
   errors=errors+1;
  end
  expect_reg(1,10);expect_reg(2,20);expect_reg(3,30);expect_reg(4,34);
  expect_reg(5,30);expect_reg(6,0);expect_reg(7,32'hffffffff);
  expect_reg(8,32'hffffffff);expect_reg(9,255);expect_reg(10,32'h34);
  expect_reg(11,0);
  if(ram.memory[0] !== 30) begin $display("[FAIL] RAM[0]");errors=errors+1;end
  else $display("[PASS] RAM[0] = 30");
  if(ram.memory[1][7:0] !== 8'hff) begin $display("[FAIL] RAM[4] byte");errors=errors+1;end
  else $display("[PASS] RAM[4] byte = FF");
  if(stalled<3) begin $display("[FAIL] stall nao ocorreu");errors=errors+1;end
  else $display("[PASS] stall cycles=%0d",stalled);
  $display("Total cycles=%0d retired=%0d errors=%0d",cycles,retired,errors);
  if(errors==0) $display("RV32I INTEGRATION TEST: PASS");
  else $display("RV32I INTEGRATION TEST: FAIL");
  $finish;
 end
 always @(posedge clk) begin
  if(rst_n) begin
   cycles=cycles+1;
   if(retire) retired=retired+1;
   if(valid&&!ready) stalled=stalled+1;
  end
 end
endmodule
