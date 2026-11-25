# =============================================================================
# Projeto     : RISC-V RV32I com Controlador DMA e Coprocessador Matricial
# Etapa       : Entrega Parcial IV — Integração Top-Level e Firmware
# Arquivo     : matmul_dma.s (gerado por sw/firmware/build_firmware.py)
# Professores : Ana Julia Fernandes de Oliveira Barros
#               Igor Dantas dos Santos Miranda
#               João Alberto Castelo Branco Oliveira
# Grupo 4     : João Vitor Carvalho Lima
#               Matheus Artur Macedo Bomfim
#               Nathalia Miranda
#               Tatiana Caetano Praxedes de Moura
# Descrição   : Firmware RV32I que valida o descritor (0x0000..0x001C),
#               protege a área reservada do firmware (0x0000..0x01FF),
#               limita DESC_CPU_WORK (0..64) e DESC_CTRL ({0, 1, 3}),
#               configura o DMA via MMIO, executa polling limitado em
#               DMA_STATUS (com trabalho opcional concorrente em 0x0100..0x01FC),
#               limpa flags por W1C, grava telemetria em 0x0020..0x003C e
#               encerra via EBREAK. A CPU não movimenta matrizes A, B ou C.
# =============================================================================

.text
.globl _start
_start:
    lui   x1, 0x40000             # 0x0000: 400000B7 | x1 = 0x4000_0000 (base MMIO do DMA)
    addi  x2, x0, 0               # 0x0004: 00000113 | x2 = 0x0000_0000 (base do descritor na DMEM)
    addi  x13, x0, 0              # 0x0008: 00000693 | x13 = poll_iters = 0
    addi  x14, x0, 0              # 0x000C: 00000713 | x14 = work_done = 0
    addi  x18, x0, 0              # 0x0010: 00000913 | x18 = PERF_TOTAL_CYCLES = 0
    addi  x19, x0, 0              # 0x0014: 00000993 | x19 = PERF_COMP_CYCLES = 0
    addi  x20, x0, 0              # 0x0018: 00000A13 | x20 = PERF_MEM_READS = 0
    lw    x21, 44(x1)             # 0x001C: 02C0AA83 | x21 = HW_ARRAY_SIZE
    lw    x4, 0(x2)               # 0x0020: 00012203 | x4 = DESC_ADDR_A
    lw    x5, 4(x2)               # 0x0024: 00412283 | x5 = DESC_ADDR_B
    lw    x6, 8(x2)               # 0x0028: 00812303 | x6 = DESC_ADDR_C
    addi  x7, x0, 512             # 0x002C: 20000393 | x7 = 0x0200 (fim da area reservada do firmware)
    bltu  x4, x7, fw_err_reserved  # 0x0030: 0E726063 | rejeita se ADDR_A < 0x0200
    bltu  x5, x7, fw_err_reserved  # 0x0034: 0C72EE63 | rejeita se ADDR_B < 0x0200
    bltu  x6, x7, fw_err_reserved  # 0x0038: 0C736C63 | rejeita se ADDR_C < 0x0200
    lw    x12, 28(x2)             # 0x003C: 01C12603 | x12 = DESC_CPU_WORK
    addi  x7, x0, 64              # 0x0040: 04000393 | x7 = 64 (maximo de palavras em 0x0100..0x01FC)
    bltu  x7, x12, fw_err_work    # 0x0044: 0CC3EC63 | rejeita se DESC_CPU_WORK > 64 (unsigned)
    lw    x11, 24(x2)             # 0x0048: 01812583 | x11 = DESC_CTRL
    bne   x11, x0, check_ctrl_1   # 0x004C: 00059663 | se diferente de 0, valida explicitamente
    addi  x11, x0, 3              # 0x0050: 00300593 | DESC_CTRL=0 -> padrao 0x3 (START | IRQ_EN)
    jal   x0, ctrl_ok             # 0x0054: 0180006F | comando valido
check_ctrl_1:
    addi  x7, x0, 1               # 0x0058: 00100393 | x7 = 1 (START sem IRQ_EN)
    beq   x11, x7, ctrl_ok        # 0x005C: 00758863 | aceita DESC_CTRL = 1
    addi  x7, x0, 3               # 0x0060: 00300393 | x7 = 3 (START com IRQ_EN)
    beq   x11, x7, ctrl_ok        # 0x0064: 00758463 | aceita DESC_CTRL = 3
    jal   x0, fw_err_ctrl         # 0x0068: 0C00006F | rejeita qualquer outro DESC_CTRL (ex.: 2 ou SOFT_RST)
ctrl_ok:
    addi  x3, x0, 6               # 0x006C: 00600193 | mascara W1C: DONE(bit1) | ERR(bit2)
    sw    x3, 4(x1)               # 0x0070: 0030A223 | DMA_STATUS = 0x6 (limpa flags antigos)
    sw    x4, 8(x1)               # 0x0074: 0040A423 | escreve DMA_ADDR_A
    sw    x5, 12(x1)              # 0x0078: 0050A623 | escreve DMA_ADDR_B
    sw    x6, 16(x1)              # 0x007C: 0060A823 | escreve DMA_ADDR_C
    lw    x3, 12(x2)              # 0x0080: 00C12183 | le DESC_DIM_M
    sw    x3, 20(x1)              # 0x0084: 0030AA23 | escreve DMA_DIM_M
    lw    x3, 16(x2)              # 0x0088: 01012183 | le DESC_DIM_K
    sw    x3, 24(x1)              # 0x008C: 0030AC23 | escreve DMA_DIM_K
    lw    x3, 20(x2)              # 0x0090: 01412183 | le DESC_DIM_N
    sw    x3, 28(x1)              # 0x0094: 0030AE23 | escreve DMA_DIM_N
    addi  x15, x0, 256            # 0x0098: 10000793 | x15 = ponteiro da area privada CPU (0x0100..0x01FC)
    lui   x17, 0x0000A            # 0x009C: 0000A8B7 | x17 = base do padrao de trabalho da CPU
    addi  x17, x17, 1280          # 0x00A0: 50088893 | completa 0x0000A500
    lui   x23, 0x00031            # 0x00A4: 00031BB7 | x23 = limite maximo de iteracoes de polling
    addi  x23, x23, -704          # 0x00A8: D40B8B93 | completa 0x00030D40
    sw    x11, 0(x1)              # 0x00AC: 00B0A023 | DMA_CTRL = comando validado (dispara DMA)
poll_loop:
    addi  x13, x13, 1             # 0x00B0: 00168693 | poll_iters++
    bgeu  x14, x12, skip_cpu_work  # 0x00B4: 00C77A63 | pula se work_done >= DESC_CPU_WORK
    add   x3, x17, x14            # 0x00B8: 00E881B3 | x3 = 0x0000A500 + work_done
    sw    x3, 0(x15)              # 0x00BC: 0037A023 | CPU escreve em 0x0100..0x01FC (concorrente ao DMA)
    addi  x15, x15, 4             # 0x00C0: 00478793 | avanca ponteiro da area privada (+4B)
    addi  x14, x14, 1             # 0x00C4: 00170713 | work_done++
skip_cpu_work:
    lw    x10, 4(x1)              # 0x00C8: 0040A503 | x10 = DMA_STATUS
    andi  x16, x10, 6             # 0x00CC: 00657813 | x16 = status & (DONE | ERR)
    bne   x16, x0, poll_done      # 0x00D0: 00081663 | sai assim que DONE=1 ou ERR=1
    bgeu  x13, x23, fw_err_timeout  # 0x00D4: 0776F063 | aborta polling se exceder MAX_POLL_ITERS
    jal   x0, poll_loop           # 0x00D8: FD9FF06F | continua polling
poll_done:
    lw    x18, 32(x1)             # 0x00DC: 0200A903 | x18 = PERF_TOTAL_CYCLES
    lw    x19, 36(x1)             # 0x00E0: 0240A983 | x19 = PERF_COMP_CYCLES
    lw    x20, 40(x1)             # 0x00E4: 0280AA03 | x20 = PERF_MEM_READS
    lw    x21, 44(x1)             # 0x00E8: 02C0AA83 | x21 = HW_ARRAY_SIZE
    sw    x16, 4(x1)              # 0x00EC: 0100A223 | DMA_STATUS W1C (limpa DONE/ERR e baixa irq_done)
    andi  x3, x10, 4              # 0x00F0: 00457193 | testa bit ERR (bit 2)
    bne   x3, x0, set_dma_err     # 0x00F4: 00019863 | desvia se DMA sinalizou ERR=1
    lui   x22, 0x444F5            # 0x00F8: 444F5B37 | OUT_MAGIC = 'DONE' (0x444F4E45)
    addi  x22, x22, -443          # 0x00FC: E45B0B13 | completa 0x444F4E45
    jal   x0, write_telemetry     # 0x0100: 0440006F | grava telemetria de sucesso
set_dma_err:
    lui   x22, 0x45525            # 0x0104: 45525B37 | OUT_MAGIC = 'ERR!' (0x45525221)
    addi  x22, x22, 545           # 0x0108: 221B0B13 | completa 0x45525221
    jal   x0, write_telemetry     # 0x010C: 0380006F | grava telemetria de erro do DMA
fw_err_reserved:
    lui   x10, 0x80000            # 0x0110: 80000537 | causa: endereco na area reservada 0x0000..0x01FF
    addi  x10, x10, 1             # 0x0114: 00150513 | completa 0x80000001
    jal   x0, set_fw_err          # 0x0118: 0240006F | sinaliza FWER sem disparar DMA
fw_err_work:
    lui   x10, 0x80000            # 0x011C: 80000537 | causa: DESC_CPU_WORK > 64
    addi  x10, x10, 2             # 0x0120: 00250513 | completa 0x80000002
    jal   x0, set_fw_err          # 0x0124: 0180006F | sinaliza FWER sem disparar DMA
fw_err_ctrl:
    lui   x10, 0x80000            # 0x0128: 80000537 | causa: DESC_CTRL invalido (aceitos: 0, 1, 3)
    addi  x10, x10, 3             # 0x012C: 00350513 | completa 0x80000003
    jal   x0, set_fw_err          # 0x0130: 00C0006F | sinaliza FWER sem disparar DMA
fw_err_timeout:
    lui   x10, 0x80000            # 0x0134: 80000537 | causa: limite interno de polling excedido
    addi  x10, x10, 4             # 0x0138: 00450513 | completa 0x80000004
set_fw_err:
    lui   x22, 0x46574            # 0x013C: 46574B37 | OUT_MAGIC = 'FWER' (0x46574552)
    addi  x22, x22, 1362          # 0x0140: 552B0B13 | completa 0x46574552
write_telemetry:
    sw    x22, 32(x2)             # 0x0144: 03612023 | OUT_MAGIC
    sw    x10, 36(x2)             # 0x0148: 02A12223 | OUT_DMA_STATUS (ou codigo FWER)
    sw    x18, 40(x2)             # 0x014C: 03212423 | OUT_TOTAL_CYCLES
    sw    x19, 44(x2)             # 0x0150: 03312623 | OUT_COMP_CYCLES
    sw    x20, 48(x2)             # 0x0154: 03412823 | OUT_MEM_READS
    sw    x21, 52(x2)             # 0x0158: 03512A23 | OUT_HW_ARRAY_SIZE
    sw    x13, 56(x2)             # 0x015C: 02D12C23 | OUT_POLL_ITERS
    sw    x14, 60(x2)             # 0x0160: 02E12E23 | OUT_CPU_WORK_DONE
    ebreak                        # 0x0164: 00100073 | encerra execucao do firmware com halted=1 e trap=0
