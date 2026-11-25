# =============================================================================
# Projeto     : RISC-V RV32I com Controlador DMA e Coprocessador Matricial
# Etapa       : Entrega Parcial IV — Integração Top-Level e Firmware
# Arquivo     : test_decerr_write.s (gerado por sw/firmware/build_firmware.py)
# Professores : Ana Julia Fernandes de Oliveira Barros
#               Igor Dantas dos Santos Miranda
#               João Alberto Castelo Branco Oliveira
# Grupo 4     : João Vitor Carvalho Lima
#               Matheus Artur Macedo Bomfim
#               Nathalia Miranda
#               Tatiana Caetano Praxedes de Moura
# Descrição   : Teste negativo de escrita fora de faixa (0x5000_0000):
#               provoca BRESP=DECERR, propagação cpu_error=1 e trap=1.
# =============================================================================

.text
.globl _start
_start:
    lui   x1, 0x50000             # 0x0000: 500000B7 | x1 = 0x5000_0000 (fora de DMEM e MMIO -> DECERR)
    sw    x1, 0(x1)               # 0x0004: 0010A023 | escrita ilegal -> deve ativar trap=1 antes do EBREAK
    ebreak                        # 0x0008: 00100073 | nao deve ser alcancado em caso de trap correto
