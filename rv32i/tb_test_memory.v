`timescale 1ns/1ps
module tb_test_memory;
 reg clk=0, rst_n=0;
 wire valid,ready,wr,mem_error,retire,halted,trap;
 wire [31:0] addr,wdata,rdata,pc_debug;
 wire [3:0] wstrb;
 integer cycles=0, retired=0, stalled=0, errors=0;
 reg hold_pending=0;
 reg [31:0] saved_pc, saved_addr, saved_wdata;
 reg [3:0] saved_wstrb;
 reg saved_write;
 always #5 clk=~clk;
 rv32i_core #(.IMEM_FILE("test_memory.hex")) cpu (
  .clk(clk),.rst_n(rst_n),.dmem_valid(valid),.dmem_ready(ready),
  .dmem_write(wr),.dmem_addr(addr),.dmem_wdata(wdata),.dmem_wstrb(wstrb),
  .dmem_rdata(rdata),.dmem_error(mem_error),.debug_pc(pc_debug),
  .halted(halted),.trap(trap),.retire(retire));
 data_memory #(.MEM_DEPTH(1024),.LATENCY(3)) ram (
  .clk(clk),.rst_n(rst_n),.req_valid(valid),.req_ready(ready),
  .req_write(wr),.req_addr(addr),.req_wdata(wdata),.req_wstrb(wstrb),
  .resp_rdata(rdata),.resp_error(mem_error));
 task check_reg;
  input integer index;
  input [31:0] expected;
  begin
   if(cpu.rf_inst.registers[index] !== expected) begin
    $display("[FAIL] x%0d expected=%h got=%h",index,expected,cpu.rf_inst.registers[index]);errors=errors+1;
   end else $display("[PASS] x%0d=%h",index,expected);
  end
 endtask
 // Valida estabilidade do barramento e do PC durante stall
 always @(posedge clk) begin
  if(!rst_n) hold_pending <= 0;
  else begin
   if(hold_pending && (!valid || pc_debug !== saved_pc ||
       addr !== saved_addr || wdata !== saved_wdata ||
       wstrb !== saved_wstrb || wr !== saved_write)) begin
     $display("[FAIL] Requisicao/PC mudou durante stall");errors=errors+1;
   end
   hold_pending <= valid && !ready;
   if(valid && !ready) begin
     saved_pc <= pc_debug; saved_addr <= addr;
     saved_wdata <= wdata; saved_wstrb <= wstrb; saved_write <= wr;
   end
  end
 end
 always @(posedge clk) if (rst_n && !halted && !trap) begin
  cycles=cycles+1;
  if(retire) retired=retired+1;
  if(valid&&!ready) stalled=stalled+1;
 end
 initial begin
  repeat(3) @(negedge clk);
  rst_n=1;
  wait(halted||trap||cycles>=250);
  @(negedge clk);
  if (!halted || trap) begin $display("[FAIL] halted=%b trap=%b",halted,trap);errors=errors+1;end
  if(cycles>=250) begin $display("[FAIL] TIMEOUT");errors=errors+1;end
  check_reg(1,32'h80000000);
  check_reg(2,32'h0000007f);
  check_reg(3,32'h80000000);
  check_reg(4,32'h00000000);
  check_reg(5,32'hffffff80);
  check_reg(6,32'h00000080);
  check_reg(7,32'h00000000);
  check_reg(8,32'hffff8000);
  check_reg(9,32'h00008000);
  check_reg(10,32'h80007f00);
  check_reg(11,32'hffffffff);
  check_reg(12,32'hffff7f00);
  check_reg(13,32'h00000000);
  check_reg(14,32'h0000007f);
  check_reg(15,32'hffffffff);
  if (ram.memory[0] !== 32'hffff7f00) begin $display("[FAIL] RAM[0]");errors=errors+1;end else $display("[PASS] RAM[0]");
  if (stalled==0) begin $display("[FAIL] stall nao ocorreu");errors=errors+1;end
  $display("CASE test_memory: cycles=%0d retired=%0d stalled=%0d errors=%0d",cycles,retired,stalled,errors);
  if(errors==0) $display("TEST test_memory: PASS"); else $display("TEST test_memory: FAIL");
  $finish;
 end
endmodule
