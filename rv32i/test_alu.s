.text
.globl _start
_start:
    addi x1,x0,10
    addi x2,x0,-3
    add x3,x1,x2
    sub x4,x1,x2
    and x5,x1,x2
    or x6,x1,x2
    xor x7,x1,x2
    slt x8,x2,x1
    sltu x9,x2,x1
    slti x10,x2,-2
    sltiu x11,x2,1
    andi x12,x1,6
    ori x13,x1,5
    xori x14,x1,15
    slli x15,x1,2
    srli x16,x2,1
    srai x17,x2,1025
    addi x18,x0,3
    sll x19,x1,x18
    srl x20,x2,x18
    sra x21,x2,x18
    lui x22,0x12345
    auipc x23,0x1
    ebreak
