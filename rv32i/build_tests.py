from pathlib import Path
P=Path(__file__).resolve().parent

def I(imm,rs1,f3,rd,op=0x13):return ((imm&4095)<<20)|(rs1<<15)|(f3<<12)|(rd<<7)|op
def R(f7,rs2,rs1,f3,rd):return(f7<<25)|(rs2<<20)|(rs1<<15)|(f3<<12)|(rd<<7)|0x33
def S(imm,rs2,rs1,f3):
 v=imm&4095;return ((v>>5)<<25)|(rs2<<20)|(rs1<<15)|(f3<<12)|((v&31)<<7)|0x23
def B(d,rs2,rs1,f3):
 assert d%2==0 and -4096<=d<=4094
 v=d&8191;return ((v>>12&1)<<31)|((v>>5&63)<<25)|(rs2<<20)|(rs1<<15)|(f3<<12)|((v>>1&15)<<8)|((v>>11&1)<<7)|0x63
def J(d,rd):
 assert d%2==0 and -1048576<=d<=1048574
 v=d&0x1fffff;return ((v>>20&1)<<31)|((v>>1&1023)<<21)|((v>>11&1)<<20)|((v>>12&255)<<12)|(rd<<7)|0x6f
def U(imm,rd,op):return (imm&0xfffff000)|(rd<<7)|op
class Program:
 def __init__(self,name):self.name=name;self.lines=[];self.labels={}
 def label(self,name):self.labels[name]=len(self.lines)
 def emit(self,word,asm):self.lines.append((word,asm))
 def imm(self,rd,rs1,v,f3=0,op=0x13,n='addi'):self.emit(I(v,rs1,f3,rd,op),f'{n} x{rd},x{rs1},{v}')
 def reg(self,rd,rs1,rs2,f3,f7=0,n='op'):self.emit(R(f7,rs2,rs1,f3,rd),f'{n} x{rd},x{rs1},x{rs2}')
 def store(self,rs2,rs1,imm,f3,n='sw'):self.emit(S(imm,rs2,rs1,f3),f'{n} x{rs2},{imm}(x{rs1})')
 def branch(self,rs1,rs2,f3,target,n):self.emit(('branch',rs1,rs2,f3,target),f'{n} x{rs1},x{rs2},{target}')
 def jal(self,rd,target):self.emit(('jal',rd,target),f'jal x{rd},{target}')
 def jalr(self,rd,rs1,imm):self.emit(I(imm,rs1,0,rd,0x67),f'jalr x{rd},{imm}(x{rs1})')
 def lui(self,rd,imm):self.emit(U(imm,rd,0x37),f'lui x{rd},0x{imm>>12:x}')
 def auipc(self,rd,imm):self.emit(U(imm,rd,0x17),f'auipc x{rd},0x{imm>>12:x}')
 def ebreak(self):self.emit(0x00100073,'ebreak')
 def raw(self,v,name):self.emit(v,name)
 def dump(self):
  out=[];asm=[]
  for idx,(code,s) in enumerate(self.lines):
   if isinstance(code,tuple):
    if code[0]=='branch':_,a,b,f,target=code;code=B((self.labels[target]-idx)*4,b,a,f)
    else:_,rd,target=code;code=J((self.labels[target]-idx)*4,rd)
   out.append(f'{code:08X}');asm.append(f'{idx*4:04x}: {s}  # {code:08X}')
  (P/f'{self.name}.hex').write_text('\n'.join(out)+'\n')
  labels_by_index={idx:[] for idx in range(len(self.lines))}
  for label,idx in self.labels.items():labels_by_index.setdefault(idx,[]).append(label)
  assembly=['.text','.globl _start','_start:']
  for idx,(word,mnemonic) in enumerate(self.lines):
   for label in labels_by_index.get(idx,[]):assembly.append(label+':')
   assembly.append('    '+(mnemonic if mnemonic!='trap stimulus' else f'.word 0x{out[idx]}'))
  (P/f'{self.name}.s').write_text('\n'.join(assembly)+'\n')

alu=Program('test_alu')
alu.imm(1,0,10);alu.imm(2,0,-3)
alu.reg(3,1,2,0,n='add');alu.reg(4,1,2,0,0x20,'sub')
alu.reg(5,1,2,7,n='and');alu.reg(6,1,2,6,n='or');alu.reg(7,1,2,4,n='xor')
alu.reg(8,2,1,2,n='slt');alu.reg(9,2,1,3,n='sltu')
alu.imm(10,2,-2,2,n='slti');alu.imm(11,2,1,3,n='sltiu')
alu.imm(12,1,6,7,n='andi');alu.imm(13,1,5,6,n='ori');alu.imm(14,1,15,4,n='xori')
alu.imm(15,1,2,1,n='slli');alu.imm(16,2,1,5,n='srli');alu.imm(17,2,(0x20<<5)|1,5,n='srai')
alu.imm(18,0,3);alu.reg(19,1,18,1,n='sll');alu.reg(20,2,18,5,n='srl');alu.reg(21,2,18,5,0x20,'sra')
alu.lui(22,0x12345000);alu.auipc(23,0x1000);alu.ebreak();alu.dump()
alu_checks={1:10,2:0xfffffffd,3:7,4:13,5:8,6:0xffffffff,7:0xfffffff7,8:1,9:0,10:1,11:0,12:2,13:15,14:5,15:40,16:0x7ffffffe,17:0xfffffffe,18:3,19:80,20:0x1fffffff,21:0xffffffff,22:0x12345000,23:0x1058}
# auipc at idx 21 -> address 0x54
branch=Program('test_branch')
branch.imm(1,0,4);branch.imm(2,0,4);branch.imm(3,0,-1);branch.imm(4,0,1);branch.imm(20,0,0)
# Each branch is expected taken; skipped marker is an increment
for f,n,a,b in [(0,'beq',1,2),(1,'bne',1,4),(4,'blt',3,4),(5,'bge',4,3),(6,'bltu',4,3),(7,'bgeu',3,4)]:
 branch.branch(a,b,f,f'skip_{n}',n);branch.imm(20,20,1);branch.label(f'skip_{n}')
# not taken for every encoding: increment x21 once after conditional
branch.imm(21,0,0)
for f,n,a,b in [(0,'beq',1,4),(1,'bne',1,2),(4,'blt',4,3),(5,'bge',3,4),(6,'bltu',3,4),(7,'bgeu',4,3)]:
 branch.branch(a,b,f,f'no_{n}',n);branch.imm(21,21,1);branch.label(f'no_{n}')
# backward branch loop 3 times
branch.imm(22,0,0);branch.imm(23,0,3);branch.label('loop');branch.imm(22,22,1);branch.branch(22,23,4,'loop','blt');branch.ebreak();branch.dump()
branch_checks={1:4,2:4,3:0xffffffff,4:1,20:0,21:6,22:3,23:3}
mem=Program('test_memory')
mem.lui(1,0x80000000);mem.imm(2,0,127);mem.store(1,0,0,2,'sw');mem.imm(3,0,0,2,0x03,'lw');mem.imm(4,0,0,0,0x03,'lb');mem.imm(5,0,3,0,0x03,'lb');mem.imm(6,0,3,4,0x03,'lbu')
mem.imm(7,0,0,1,0x03,'lh');mem.imm(8,0,2,1,0x03,'lh');mem.imm(9,0,2,5,0x03,'lhu')
mem.store(2,0,1,0,'sb');mem.imm(10,0,0,2,0x03,'lw')
mem.imm(11,0,-1);mem.store(11,0,2,1,'sh');mem.imm(12,0,0,2,0x03,'lw')
mem.imm(13,0,0,4,0x03,'lbu');mem.imm(14,0,1,4,0x03,'lbu');mem.imm(15,0,2,0,0x03,'lb')
mem.ebreak();mem.dump()
mem_checks={1:0x80000000,2:127,3:0x80000000,4:0,5:0xffffff80,6:128,7:0,8:0xffff8000,9:0x8000,10:0x80007f00,11:0xffffffff,12:0xffff7f00,13:0,14:127,15:0xffffffff}
# Corrections: word initially 0x80000000 bytes [00,00,00,80], after SB x2 offset1 -> 0x80007f00; SH -1 offset2 -> 0xffff7f00
jump=Program('test_jump')
jump.jal(5,'after_jal');jump.imm(6,0,99);jump.label('after_jal');jump.auipc(1,0);jump.imm(1,1,17);jump.jalr(7,1,0);jump.imm(8,0,88);jump.label('target');jump.imm(9,0,42);jump.ebreak();jump.dump()
# auipc idx2 @8, addi x1+17 =25; jalr target masked 24 => idx6
jump_checks={5:4,6:0,1:25,7:20,8:0,9:42}
exceptions=[]
for name,words in [('test_ecall',[0x00000073]),('test_illegal',[0xffffffff]),('test_misaligned',[I(1,0,2,1,0x03)]),('test_oob',[U(0x00001000,1,0x37),I(0,1,2,2,0x03)])]:
 p=Program(name)
 for w in words:p.raw(w,'trap stimulus')
 p.dump();exceptions.append(name)

# Generates Verilog 2001 testbench per testcase to avoid reliance on elaboration overrides.
def tb(name,expected,latency=3,expect_trap=False,mem_words=None):
 mem_words=mem_words or {}
 checks='\n'.join(f'  check_reg({n},32\'h{v:08x});' for n,v in expected.items())
 checks+='\n'+ '\n'.join(f'  if (ram.memory[{idx}] !== 32\'h{v:08x}) begin $display("[FAIL] RAM[{idx}]");errors=errors+1;end else $display("[PASS] RAM[{idx}]");' for idx,v in mem_words.items())
 if expect_trap:termination='if (!trap || halted) begin $display("[FAIL] trap=%b halted=%b",trap,halted);errors=errors+1;end'
 else:termination='if (!halted || trap) begin $display("[FAIL] halted=%b trap=%b",halted,trap);errors=errors+1;end'
 stallcheck='if (stalled==0) begin $display("[FAIL] stall nao ocorreu");errors=errors+1;end' if mem_words else ''
 code=f'''`timescale 1ns/1ps
module tb_{name};
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
 rv32i_core #(.IMEM_FILE("{name}.hex")) cpu (
  .clk(clk),.rst_n(rst_n),.dmem_valid(valid),.dmem_ready(ready),
  .dmem_write(wr),.dmem_addr(addr),.dmem_wdata(wdata),.dmem_wstrb(wstrb),
  .dmem_rdata(rdata),.dmem_error(mem_error),.debug_pc(pc_debug),
  .halted(halted),.trap(trap),.retire(retire));
 data_memory #(.MEM_DEPTH(1024),.LATENCY({latency})) ram (
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
  {termination}
  if(cycles>=250) begin $display("[FAIL] TIMEOUT");errors=errors+1;end
{checks}
  {stallcheck}
  $display("CASE {name}: cycles=%0d retired=%0d stalled=%0d errors=%0d",cycles,retired,stalled,errors);
  if(errors==0) $display("TEST {name}: PASS"); else $display("TEST {name}: FAIL");
  $finish;
 end
endmodule
'''
 (P/f'tb_{name}.v').write_text(code)

tb('test_alu',alu_checks,latency=1)
tb('test_branch',branch_checks,latency=1)
tb('test_memory',mem_checks,latency=3,mem_words={0:0xffff7f00})
tb('test_jump',jump_checks,latency=1)
for n in exceptions:tb(n,{},latency=3,expect_trap=True)
print('Gerados',len(list(P.glob('test_*.hex'))),'programas e',len(list(P.glob('tb_*.v'))),'testbenches')
