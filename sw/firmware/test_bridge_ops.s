# =============================================================================
# Projeto     : RISC-V RV32I com Controlador DMA e Coprocessador Matricial
# Etapa       : Entrega Parcial IV — Integração Top-Level e Firmware
# Arquivo     : test_bridge_ops.s (gerado por sw/firmware/build_firmware.py)
# Professores : Ana Julia Fernandes de Oliveira Barros
#               Igor Dantas dos Santos Miranda
#               João Alberto Castelo Branco Oliveira
# Grupo 4     : João Vitor Carvalho Lima
#               Matheus Artur Macedo Bomfim
#               Nathalia Miranda
#               Tatiana Caetano Praxedes de Moura
# Descrição   : Programa dirigido que exercita cpu_axi4_bridge com:
#               - 3 SW consecutivos no mesmo endereço (0x0100), incluindo
#                 dois com o mesmo valor para checar contagem 1:1 de AW/W/B;
#               - SB em offsets +1 e +3 (0x0104) verificando WSTRB e preservação;
#               - SH em offsets +0 e +2 (0x0108) verificando WSTRB e preservação;
#               - LB, LBU, LH, LHU e LW gravando resultados em 0x0110..0x0124.
# =============================================================================

.text
.globl _start
_start:
    addi  x10, x0, 256            # 0x0000: 10000513 | x10 = 0x0100 (base de testes na area privada CPU)
    lui   x1, 0x11223             # 0x0004: 112230B7 | x1 = 0x1122_3344
    addi  x1, x1, 836             # 0x0008: 34408093 | completa 0x11223344
    lui   x2, 0x89ABD             # 0x000C: 89ABD137 | x2 = 0x89AB_CDEF
    addi  x2, x2, -529            # 0x0010: DEF10113 | completa 0x89ABCDEF
    sw    x1, 0(x10)              # 0x0014: 00152023 | store #1 em 0x0100 = 0x1122_3344
    sw    x2, 0(x10)              # 0x0018: 00252023 | store #2 consecutivo em 0x0100 = 0x89AB_CDEF
    sw    x2, 0(x10)              # 0x001C: 00252023 | store #3 consecutivo (mesmo dado) em 0x0100
    lui   x3, 0xAA55B             # 0x0020: AA55B1B7 | x3 = 0xAA55_AA55
    addi  x3, x3, -1451           # 0x0024: A5518193 | completa 0xAA55AA55
    sw    x3, 4(x10)              # 0x0028: 00352223 | 0x0104 = 0xAA55_AA55
    addi  x4, x0, 18              # 0x002C: 01200213 | x4 = 0x12
    sb    x4, 5(x10)              # 0x0030: 004502A3 | SB offset +1 (WSTRB=0010) -> 0xAA55_1255
    addi  x5, x0, -16             # 0x0034: FF000293 | x5 = 0xFFFFFFF0 (byte baixo = 0xF0)
    sb    x5, 7(x10)              # 0x0038: 005503A3 | SB offset +3 (WSTRB=1000) -> 0xF055_1255
    sw    x1, 8(x10)              # 0x003C: 00152423 | 0x0108 = 0x1122_3344
    lui   x6, 0x00008             # 0x0040: 00008337 | x6 = 0x0000_80FE
    addi  x6, x6, 254             # 0x0044: 0FE30313 | completa 0x000080FE
    sh    x6, 8(x10)              # 0x0048: 00651423 | SH offset +0 (WSTRB=0011) -> 0x1122_80FE
    lui   x7, 0x0000D             # 0x004C: 0000D3B7 | x7 = 0x0000_CAFE
    addi  x7, x7, -1282           # 0x0050: AFE38393 | completa 0x0000CAFE
    sh    x7, 10(x10)             # 0x0054: 00751523 | SH offset +2 (WSTRB=1100) -> 0xCAFE_80FE
    lb    x11, 8(x10)             # 0x0058: 00850583 | LB  +0 (0xFE)   -> 0xFFFF_FFFE
    lbu   x12, 8(x10)             # 0x005C: 00854603 | LBU +0 (0xFE)   -> 0x0000_00FE
    lb    x13, 9(x10)             # 0x0060: 00950683 | LB  +1 (0x80)   -> 0xFFFF_FF80
    lh    x14, 8(x10)             # 0x0064: 00851703 | LH  +0 (0x80FE) -> 0xFFFF_80FE
    lhu   x15, 10(x10)            # 0x0068: 00A55783 | LHU +2 (0xCAFE) -> 0x0000_CAFE
    lw    x16, 8(x10)             # 0x006C: 00852803 | LW  +0          -> 0xCAFE_80FE
    sw    x11, 16(x10)            # 0x0070: 00B52823 | 0x0110 = resultado LB  +0
    sw    x12, 20(x10)            # 0x0074: 00C52A23 | 0x0114 = resultado LBU +0
    sw    x13, 24(x10)            # 0x0078: 00D52C23 | 0x0118 = resultado LB  +1
    sw    x14, 28(x10)            # 0x007C: 00E52E23 | 0x011C = resultado LH  +0
    sw    x15, 32(x10)            # 0x0080: 02F52023 | 0x0120 = resultado LHU +2
    sw    x16, 36(x10)            # 0x0084: 03052223 | 0x0124 = resultado LW  +0
    ebreak                        # 0x0088: 00100073 | encerra teste dirigido da ponte com halted=1 e trap=0
