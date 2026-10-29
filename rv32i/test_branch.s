.text
.globl _start
_start:
    addi x1,x0,4
    addi x2,x0,4
    addi x3,x0,-1
    addi x4,x0,1
    addi x20,x0,0
    beq x1,x2,skip_beq
    addi x20,x20,1
skip_beq:
    bne x1,x4,skip_bne
    addi x20,x20,1
skip_bne:
    blt x3,x4,skip_blt
    addi x20,x20,1
skip_blt:
    bge x4,x3,skip_bge
    addi x20,x20,1
skip_bge:
    bltu x4,x3,skip_bltu
    addi x20,x20,1
skip_bltu:
    bgeu x3,x4,skip_bgeu
    addi x20,x20,1
skip_bgeu:
    addi x21,x0,0
    beq x1,x4,no_beq
    addi x21,x21,1
no_beq:
    bne x1,x2,no_bne
    addi x21,x21,1
no_bne:
    blt x4,x3,no_blt
    addi x21,x21,1
no_blt:
    bge x3,x4,no_bge
    addi x21,x21,1
no_bge:
    bltu x3,x4,no_bltu
    addi x21,x21,1
no_bltu:
    bgeu x4,x3,no_bgeu
    addi x21,x21,1
no_bgeu:
    addi x22,x0,0
    addi x23,x0,3
loop:
    addi x22,x22,1
    blt x22,x23,loop
    ebreak
