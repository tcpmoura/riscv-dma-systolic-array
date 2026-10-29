.text
.globl _start
_start:
    lui x1,0x80000
    addi x2,x0,127
    sw x1,0(x0)
    lw x3,x0,0
    lb x4,x0,0
    lb x5,x0,3
    lbu x6,x0,3
    lh x7,x0,0
    lh x8,x0,2
    lhu x9,x0,2
    sb x2,1(x0)
    lw x10,x0,0
    addi x11,x0,-1
    sh x11,2(x0)
    lw x12,x0,0
    lbu x13,x0,0
    lbu x14,x0,1
    lb x15,x0,2
    ebreak
