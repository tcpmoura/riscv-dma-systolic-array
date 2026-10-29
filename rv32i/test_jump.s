.text
.globl _start
_start:
    jal x5,after_jal
    addi x6,x0,99
after_jal:
    auipc x1,0x0
    addi x1,x1,17
    jalr x7,0(x1)
    addi x8,x0,88
target:
    addi x9,x0,42
    ebreak
