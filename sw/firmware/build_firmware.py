#!/usr/bin/env python3
# -*- coding: utf-8 -*-
# =============================================================================
# Projeto     : RISC-V RV32I com Controlador DMA e Coprocessador Matricial
# Etapa       : Entrega Parcial IV — Integração Top-Level e Firmware
# Arquivo     : build_firmware.py
# Professores : Ana Julia Fernandes de Oliveira Barros
#               Igor Dantas dos Santos Miranda
#               João Alberto Castelo Branco Oliveira
# Grupo 4     : João Vitor Carvalho Lima
#               Matheus Artur Macedo Bomfim
#               Nathalia Miranda
#               Tatiana Caetano Praxedes de Moura
# Descrição   : Fonte editável e montador determinístico em Python para os
#               firmwares RV32I do SoC. Gera simultaneamente o código Assembly
#               (.s), o binário hexadecimal (.hex preenchido com NOP até 256
#               palavras para $readmemh na IMEM) e o binário bruto (.bin).
#
# Contrato de Memória e Validação do Firmware Principal (matmul_dma):
#   - Região reservada ao firmware na DMEM: 0x0000_0000 .. 0x0000_01FF (512 B):
#       * 0x0000..0x001F : Bloco descritor de entrada
#       * 0x0020..0x003F : Bloco de telemetria de saída
#       * 0x0100..0x01FC : Área de trabalho concorrente da CPU (64 palavras)
#   - Matrizes A, B e C devem iniciar em endereço >= 0x0000_0200 (unsigned).
#   - DESC_CPU_WORK deve estar em 0..64 (máximo de escritas durante o polling).
#   - DESC_CTRL aceito: 0 (padrão 0x3), 1 (START) ou 3 (START | IRQ_EN).
#   - Limite interno de polling: 200.000 iterações (impede travamento infinito).
#   - Códigos OUT_MAGIC (0x0020):
#       * 0x444F_4E45 ("DONE") : DMA concluído com sucesso (DMA_STATUS=0x2)
#       * 0x4552_5221 ("ERR!") : Erro retornado pelo DMA (DMA_STATUS=0x4)
#       * 0x4657_4552 ("FWER") : Erro de validação/timeout do firmware
#         (OUT_DMA_STATUS = 0x8000_0001 área reservada; 0x8000_0002 CPU_WORK>64;
#          0x8000_0003 CTRL inválido; 0x8000_0004 limite de polling atingido)
#
# Uso (a partir da raiz do projeto):
#   python -B sw/firmware/build_firmware.py          # Gera todos os artefatos
#   python -B sw/firmware/build_firmware.py --check  # Valida sincronia exata
# =============================================================================

import argparse
from pathlib import Path
import struct
import sys

if hasattr(sys.stdout, "reconfigure"):
    sys.stdout.reconfigure(encoding="utf-8")

SCRIPT_DIR = Path(__file__).resolve().parent
IMEM_DEPTH_DEFAULT = 256
RV32I_NOP = 0x00000013

# Constantes do contrato do firmware matmul_dma
FW_RESERVED_END = 0x00000200
MAX_CPU_WORK    = 64
MAX_POLL_ITERS  = 200000

MAGIC_DONE      = 0x444F4E45  # "DONE"
MAGIC_DMA_ERR   = 0x45525221  # "ERR!"
MAGIC_FW_ERR    = 0x46574552  # "FWER"

FW_CAUSE_RESERVED = 0x80000001
FW_CAUSE_CPU_WORK = 0x80000002
FW_CAUSE_CTRL     = 0x80000003
FW_CAUSE_TIMEOUT  = 0x80000004


# =============================================================================
# Codificadores RV32I (100% compatíveis com rv32i/control_unit.v)
# =============================================================================

def enc_r(funct7: int, rs2: int, rs1: int, funct3: int, rd: int, opcode: int = 0x33) -> int:
    return ((funct7 & 0x7F) << 25) | ((rs2 & 0x1F) << 20) | ((rs1 & 0x1F) << 15) | ((funct3 & 0x7) << 12) | ((rd & 0x1F) << 7) | (opcode & 0x7F)


def enc_i(imm: int, rs1: int, funct3: int, rd: int, opcode: int = 0x13) -> int:
    if not (-2048 <= imm <= 2047):
        raise ValueError(f"Imediato I fora da faixa 12-bit signed: {imm}")
    return ((imm & 0xFFF) << 20) | ((rs1 & 0x1F) << 15) | ((funct3 & 0x7) << 12) | ((rd & 0x1F) << 7) | (opcode & 0x7F)


def enc_s(imm: int, rs2: int, rs1: int, funct3: int, opcode: int = 0x23) -> int:
    if not (-2048 <= imm <= 2047):
        raise ValueError(f"Imediato S fora da faixa 12-bit signed: {imm}")
    v = imm & 0xFFF
    return (((v >> 5) & 0x7F) << 25) | ((rs2 & 0x1F) << 20) | ((rs1 & 0x1F) << 15) | ((funct3 & 0x7) << 12) | ((v & 0x1F) << 7) | (opcode & 0x7F)


def enc_b(offset: int, rs2: int, rs1: int, funct3: int, opcode: int = 0x63) -> int:
    if (offset % 2) != 0 or not (-4096 <= offset <= 4094):
        raise ValueError(f"Offset de branch fora da faixa: {offset}")
    v = offset & 0x1FFF
    return (
        (((v >> 12) & 0x1) << 31)
        | (((v >> 5) & 0x3F) << 25)
        | ((rs2 & 0x1F) << 20)
        | ((rs1 & 0x1F) << 15)
        | ((funct3 & 0x7) << 12)
        | (((v >> 1) & 0xF) << 8)
        | (((v >> 11) & 0x1) << 7)
        | (opcode & 0x7F)
    )


def enc_u(imm_u: int, rd: int, opcode: int) -> int:
    return (imm_u & 0xFFFFF000) | ((rd & 0x1F) << 7) | (opcode & 0x7F)


def enc_j(offset: int, rd: int, opcode: int = 0x6F) -> int:
    if (offset % 2) != 0 or not (-1048576 <= offset <= 1048574):
        raise ValueError(f"Offset JAL fora da faixa: {offset}")
    v = offset & 0x1FFFFF
    return (
        (((v >> 20) & 0x1) << 31)
        | (((v >> 1) & 0x3FF) << 21)
        | (((v >> 11) & 0x1) << 20)
        | (((v >> 12) & 0xFF) << 12)
        | ((rd & 0x1F) << 7)
        | (opcode & 0x7F)
    )


class Rv32iAsm:
    """Montador simbólico RV32I com resolução de rótulos e validação."""

    def __init__(self, name: str, header_comments: list[str]):
        self.name = name
        self.header_comments = header_comments
        self.items: list[tuple] = []
        self.labels: dict[str, int] = {}

    def label(self, name: str) -> None:
        if name in self.labels:
            raise ValueError(f"Rótulo duplicado: {name}")
        self.labels[name] = len(self.items)

    def _add(self, enc, mnemonic: str, comment: str = "") -> None:
        self.items.append((enc, mnemonic, comment))

    def addi(self, rd: int, rs1: int, imm: int, c: str = "") -> None:
        self._add(enc_i(imm, rs1, 0, rd, 0x13), f"addi  x{rd}, x{rs1}, {imm}", c)

    def andi(self, rd: int, rs1: int, imm: int, c: str = "") -> None:
        self._add(enc_i(imm, rs1, 7, rd, 0x13), f"andi  x{rd}, x{rs1}, {imm}", c)

    def add(self, rd: int, rs1: int, rs2: int, c: str = "") -> None:
        self._add(enc_r(0x00, rs2, rs1, 0, rd), f"add   x{rd}, x{rs1}, x{rs2}", c)

    def lui(self, rd: int, imm_upper20: int, c: str = "") -> None:
        self._add(enc_u(imm_upper20 << 12, rd, 0x37), f"lui   x{rd}, 0x{imm_upper20:05X}", c)

    def li32(self, rd: int, val32: int, c: str = "") -> None:
        """Carrega constante de 32 bits usando LUI + ADDI com ajuste de sinal em imm[11]."""
        v = val32 & 0xFFFFFFFF
        lo = v & 0xFFF
        if lo >= 0x800:
            lo -= 0x1000
            hi = ((v >> 12) + 1) & 0xFFFFF
        else:
            hi = (v >> 12) & 0xFFFFF
        self.lui(rd, hi, c)
        if lo != 0:
            self.addi(rd, rd, lo, f"completa 0x{v:08X}")

    def lb(self, rd: int, offset: int, rs1: int, c: str = "") -> None:
        self._add(enc_i(offset, rs1, 0, rd, 0x03), f"lb    x{rd}, {offset}(x{rs1})", c)

    def lh(self, rd: int, offset: int, rs1: int, c: str = "") -> None:
        self._add(enc_i(offset, rs1, 1, rd, 0x03), f"lh    x{rd}, {offset}(x{rs1})", c)

    def lw(self, rd: int, offset: int, rs1: int, c: str = "") -> None:
        self._add(enc_i(offset, rs1, 2, rd, 0x03), f"lw    x{rd}, {offset}(x{rs1})", c)

    def lbu(self, rd: int, offset: int, rs1: int, c: str = "") -> None:
        self._add(enc_i(offset, rs1, 4, rd, 0x03), f"lbu   x{rd}, {offset}(x{rs1})", c)

    def lhu(self, rd: int, offset: int, rs1: int, c: str = "") -> None:
        self._add(enc_i(offset, rs1, 5, rd, 0x03), f"lhu   x{rd}, {offset}(x{rs1})", c)

    def sb(self, rs2: int, offset: int, rs1: int, c: str = "") -> None:
        self._add(enc_s(offset, rs2, rs1, 0, 0x23), f"sb    x{rs2}, {offset}(x{rs1})", c)

    def sh(self, rs2: int, offset: int, rs1: int, c: str = "") -> None:
        self._add(enc_s(offset, rs2, rs1, 1, 0x23), f"sh    x{rs2}, {offset}(x{rs1})", c)

    def sw(self, rs2: int, offset: int, rs1: int, c: str = "") -> None:
        self._add(enc_s(offset, rs2, rs1, 2, 0x23), f"sw    x{rs2}, {offset}(x{rs1})", c)

    def beq(self, rs1: int, rs2: int, target: str, c: str = "") -> None:
        self._add(("branch", rs1, rs2, 0, target), f"beq   x{rs1}, x{rs2}, {target}", c)

    def bne(self, rs1: int, rs2: int, target: str, c: str = "") -> None:
        self._add(("branch", rs1, rs2, 1, target), f"bne   x{rs1}, x{rs2}, {target}", c)

    def bltu(self, rs1: int, rs2: int, target: str, c: str = "") -> None:
        self._add(("branch", rs1, rs2, 6, target), f"bltu  x{rs1}, x{rs2}, {target}", c)

    def bgeu(self, rs1: int, rs2: int, target: str, c: str = "") -> None:
        self._add(("branch", rs1, rs2, 7, target), f"bgeu  x{rs1}, x{rs2}, {target}", c)

    def jal(self, rd: int, target: str, c: str = "") -> None:
        self._add(("jal", rd, target), f"jal   x{rd}, {target}", c)

    def ebreak(self, c: str = "encerra simulacao com halted=1") -> None:
        self._add(0x00100073, "ebreak", c)

    def assemble(self) -> tuple[list[int], str]:
        words: list[int] = []
        labels_at: dict[int, list[str]] = {}
        for lbl, idx in self.labels.items():
            labels_at.setdefault(idx, []).append(lbl)

        asm_lines = list(self.header_comments)
        asm_lines.append("")
        asm_lines.append(".text")
        asm_lines.append(".globl _start")
        asm_lines.append("_start:")

        for idx, (enc, mnemonic, comment) in enumerate(self.items):
            for lbl in labels_at.get(idx, []):
                asm_lines.append(f"{lbl}:")
            if isinstance(enc, tuple):
                if enc[0] == "branch":
                    _, rs1, rs2, f3, target = enc
                    word = enc_b((self.labels[target] - idx) * 4, rs2, rs1, f3)
                elif enc[0] == "jal":
                    _, rd, target = enc
                    word = enc_j((self.labels[target] - idx) * 4, rd)
                else:
                    raise RuntimeError(f"Instrução simbólica desconhecida: {enc}")
            else:
                word = enc & 0xFFFFFFFF
            words.append(word)
            pc = idx * 4
            cmt = f"  # 0x{pc:04X}: {word:08X}" + (f" | {comment}" if comment else "")
            asm_lines.append(f"    {mnemonic:<28s}{cmt}")

        return words, "\n".join(asm_lines) + "\n"


# =============================================================================
# 1. Firmware Principal: matmul_dma (Valida contrato, configura DMA e monitora)
# =============================================================================
def build_matmul_dma(name: str = "matmul_dma", max_poll_iters: int = MAX_POLL_ITERS) -> Rv32iAsm:
    extra_desc = (
        f"#               (variante dirigida com MAX_POLL_ITERS = {max_poll_iters} para testar FW_ERR_TIMEOUT)."
        if name != "matmul_dma"
        else "#               encerra via EBREAK. A CPU não movimenta matrizes A, B ou C."
    )
    hdr = [
        "# =============================================================================",
        "# Projeto     : RISC-V RV32I com Controlador DMA e Coprocessador Matricial",
        "# Etapa       : Entrega Parcial IV — Integração Top-Level e Firmware",
        f"# Arquivo     : {name}.s (gerado por sw/firmware/build_firmware.py)",
        "# Professores : Ana Julia Fernandes de Oliveira Barros",
        "#               Igor Dantas dos Santos Miranda",
        "#               João Alberto Castelo Branco Oliveira",
        "# Grupo 4     : João Vitor Carvalho Lima",
        "#               Matheus Artur Macedo Bomfim",
        "#               Nathalia Miranda",
        "#               Tatiana Caetano Praxedes de Moura",
        "# Descrição   : Firmware RV32I que valida o descritor (0x0000..0x001C),",
        "#               protege a área reservada do firmware (0x0000..0x01FF),",
        "#               limita DESC_CPU_WORK (0..64) e DESC_CTRL ({0, 1, 3}),",
        "#               configura o DMA via MMIO, executa polling limitado em",
        "#               DMA_STATUS (com trabalho opcional concorrente em 0x0100..0x01FC),",
        "#               limpa flags por W1C, grava telemetria em 0x0020..0x003C e",
        extra_desc,
        "# =============================================================================",
    ]
    p = Rv32iAsm(name, hdr)

    # Mapa de registradores RV32I utilizados:
    #   x1  = 0x4000_0000 (base MMIO do controlador DMA)
    #   x2  = 0x0000_0000 (base do descritor e telemetria na DMEM)
    #   x3  = temporário geral
    #   x4  = DESC_ADDR_A
    #   x5  = DESC_ADDR_B
    #   x6  = DESC_ADDR_C
    #   x7  = constante auxiliar de comparação (0x200 / 64 / 1 / 3)
    #   x10 = status final (DMA_STATUS ou código de erro do firmware 0x8000_000x)
    #   x11 = comando DMA_CTRL validado (1 ou 3)
    #   x12 = DESC_CPU_WORK validado (0..64)
    #   x13 = contador de iterações de polling (OUT_POLL_ITERS)
    #   x14 = contador de escritas concorrentes concluídas (OUT_CPU_WORK_DONE)
    #   x15 = ponteiro da área privada de trabalho (0x0100..0x01FC)
    #   x16 = máscara de conclusão (DMA_STATUS & 0x6 -> DONE ou ERR)
    #   x17 = padrão base escrito na área privada (0x0000_A500)
    #   x18..x21 = PERF_TOTAL_CYCLES, PERF_COMP_CYCLES, PERF_MEM_READS, HW_ARRAY_SIZE
    #   x22 = assinatura OUT_MAGIC ("DONE", "ERR!" ou "FWER")
    #   x23 = limite máximo de iterações de polling (MAX_POLL_ITERS = 200.000)

    p.lui(1, 0x40000, "x1 = 0x4000_0000 (base MMIO do DMA)")
    p.addi(2, 0, 0, "x2 = 0x0000_0000 (base do descritor na DMEM)")

    # Zera registradores de telemetria para o caso de rejeição precoce pelo firmware
    p.addi(13, 0, 0, "x13 = poll_iters = 0")
    p.addi(14, 0, 0, "x14 = work_done = 0")
    p.addi(18, 0, 0, "x18 = PERF_TOTAL_CYCLES = 0")
    p.addi(19, 0, 0, "x19 = PERF_COMP_CYCLES = 0")
    p.addi(20, 0, 0, "x20 = PERF_MEM_READS = 0")
    p.lw(21, 0x2C, 1, "x21 = HW_ARRAY_SIZE")

    # -------------------------------------------------------------------------
    # Validação 1: A, B e C não podem intersectar a área reservada 0x0000..0x01FF
    # -------------------------------------------------------------------------
    p.lw(4, 0x00, 2, "x4 = DESC_ADDR_A")
    p.lw(5, 0x04, 2, "x5 = DESC_ADDR_B")
    p.lw(6, 0x08, 2, "x6 = DESC_ADDR_C")
    p.addi(7, 0, FW_RESERVED_END, "x7 = 0x0200 (fim da area reservada do firmware)")
    p.bltu(4, 7, "fw_err_reserved", "rejeita se ADDR_A < 0x0200")
    p.bltu(5, 7, "fw_err_reserved", "rejeita se ADDR_B < 0x0200")
    p.bltu(6, 7, "fw_err_reserved", "rejeita se ADDR_C < 0x0200")

    # -------------------------------------------------------------------------
    # Validação 2: DESC_CPU_WORK deve estar em 0..64 (caber em 0x0100..0x01FC)
    # -------------------------------------------------------------------------
    p.lw(12, 0x1C, 2, "x12 = DESC_CPU_WORK")
    p.addi(7, 0, MAX_CPU_WORK, "x7 = 64 (maximo de palavras em 0x0100..0x01FC)")
    p.bltu(7, 12, "fw_err_work", "rejeita se DESC_CPU_WORK > 64 (unsigned)")

    # -------------------------------------------------------------------------
    # Validação 3: DESC_CTRL deve ser 0 (padrao 0x3), 1 (START) ou 3 (START|IRQ)
    # -------------------------------------------------------------------------
    p.lw(11, 0x18, 2, "x11 = DESC_CTRL")
    p.bne(11, 0, "check_ctrl_1", "se diferente de 0, valida explicitamente")
    p.addi(11, 0, 3, "DESC_CTRL=0 -> padrao 0x3 (START | IRQ_EN)")
    p.jal(0, "ctrl_ok", "comando valido")

    p.label("check_ctrl_1")
    p.addi(7, 0, 1, "x7 = 1 (START sem IRQ_EN)")
    p.beq(11, 7, "ctrl_ok", "aceita DESC_CTRL = 1")
    p.addi(7, 0, 3, "x7 = 3 (START com IRQ_EN)")
    p.beq(11, 7, "ctrl_ok", "aceita DESC_CTRL = 3")
    p.jal(0, "fw_err_ctrl", "rejeita qualquer outro DESC_CTRL (ex.: 2 ou SOFT_RST)")

    # -------------------------------------------------------------------------
    # Configuração MMIO do DMA e disparo autônomo
    # -------------------------------------------------------------------------
    p.label("ctrl_ok")
    p.addi(3, 0, 6, "mascara W1C: DONE(bit1) | ERR(bit2)")
    p.sw(3, 0x04, 1, "DMA_STATUS = 0x6 (limpa flags antigos)")

    p.sw(4, 0x08, 1, "escreve DMA_ADDR_A")
    p.sw(5, 0x0C, 1, "escreve DMA_ADDR_B")
    p.sw(6, 0x10, 1, "escreve DMA_ADDR_C")
    p.lw(3, 0x0C, 2, "le DESC_DIM_M")
    p.sw(3, 0x14, 1, "escreve DMA_DIM_M")
    p.lw(3, 0x10, 2, "le DESC_DIM_K")
    p.sw(3, 0x18, 1, "escreve DMA_DIM_K")
    p.lw(3, 0x14, 2, "le DESC_DIM_N")
    p.sw(3, 0x1C, 1, "escreve DMA_DIM_N")

    p.addi(15, 0, 0x100, "x15 = ponteiro da area privada CPU (0x0100..0x01FC)")
    p.li32(17, 0x0000A500, "x17 = base do padrao de trabalho da CPU")
    p.li32(23, max_poll_iters, "x23 = limite maximo de iteracoes de polling")

    # Dispara o DMA autônomo (a CPU não movimenta elementos de A, B ou C)
    p.sw(11, 0x00, 1, "DMA_CTRL = comando validado (dispara DMA)")

    # -------------------------------------------------------------------------
    # Laço de polling em DMA_STATUS com proteção de limite máximo
    # -------------------------------------------------------------------------
    p.label("poll_loop")
    p.addi(13, 13, 1, "poll_iters++")
    p.bgeu(14, 12, "skip_cpu_work", "pula se work_done >= DESC_CPU_WORK")
    p.add(3, 17, 14, "x3 = 0x0000A500 + work_done")
    p.sw(3, 0, 15, "CPU escreve em 0x0100..0x01FC (concorrente ao DMA)")
    p.addi(15, 15, 4, "avanca ponteiro da area privada (+4B)")
    p.addi(14, 14, 1, "work_done++")
    p.label("skip_cpu_work")
    p.lw(10, 0x04, 1, "x10 = DMA_STATUS")
    p.andi(16, 10, 6, "x16 = status & (DONE | ERR)")
    p.bne(16, 0, "poll_done", "sai assim que DONE=1 ou ERR=1")
    p.bgeu(13, 23, "fw_err_timeout", "aborta polling se exceder MAX_POLL_ITERS")
    p.jal(0, "poll_loop", "continua polling")

    # -------------------------------------------------------------------------
    # Coleta de telemetria e limpeza W1C após resposta do DMA
    # -------------------------------------------------------------------------
    p.label("poll_done")
    p.lw(18, 0x20, 1, "x18 = PERF_TOTAL_CYCLES")
    p.lw(19, 0x24, 1, "x19 = PERF_COMP_CYCLES")
    p.lw(20, 0x28, 1, "x20 = PERF_MEM_READS")
    p.lw(21, 0x2C, 1, "x21 = HW_ARRAY_SIZE")
    p.sw(16, 0x04, 1, "DMA_STATUS W1C (limpa DONE/ERR e baixa irq_done)")

    p.andi(3, 10, 4, "testa bit ERR (bit 2)")
    p.bne(3, 0, "set_dma_err", "desvia se DMA sinalizou ERR=1")
    p.li32(22, MAGIC_DONE, "OUT_MAGIC = 'DONE' (0x444F4E45)")
    p.jal(0, "write_telemetry", "grava telemetria de sucesso")

    p.label("set_dma_err")
    p.li32(22, MAGIC_DMA_ERR, "OUT_MAGIC = 'ERR!' (0x45525221)")
    p.jal(0, "write_telemetry", "grava telemetria de erro do DMA")

    # -------------------------------------------------------------------------
    # Tratadores de erro de validação/timeout detectados pelo próprio firmware
    # -------------------------------------------------------------------------
    p.label("fw_err_reserved")
    p.li32(10, FW_CAUSE_RESERVED, "causa: endereco na area reservada 0x0000..0x01FF")
    p.jal(0, "set_fw_err", "sinaliza FWER sem disparar DMA")

    p.label("fw_err_work")
    p.li32(10, FW_CAUSE_CPU_WORK, "causa: DESC_CPU_WORK > 64")
    p.jal(0, "set_fw_err", "sinaliza FWER sem disparar DMA")

    p.label("fw_err_ctrl")
    p.li32(10, FW_CAUSE_CTRL, "causa: DESC_CTRL invalido (aceitos: 0, 1, 3)")
    p.jal(0, "set_fw_err", "sinaliza FWER sem disparar DMA")

    p.label("fw_err_timeout")
    p.li32(10, FW_CAUSE_TIMEOUT, "causa: limite interno de polling excedido")

    p.label("set_fw_err")
    p.li32(22, MAGIC_FW_ERR, "OUT_MAGIC = 'FWER' (0x46574552)")

    # -------------------------------------------------------------------------
    # Gravação do bloco de telemetria (0x0020..0x003C) e encerramento limpo
    # -------------------------------------------------------------------------
    p.label("write_telemetry")
    p.sw(22, 0x20, 2, "OUT_MAGIC")
    p.sw(10, 0x24, 2, "OUT_DMA_STATUS (ou codigo FWER)")
    p.sw(18, 0x28, 2, "OUT_TOTAL_CYCLES")
    p.sw(19, 0x2C, 2, "OUT_COMP_CYCLES")
    p.sw(20, 0x30, 2, "OUT_MEM_READS")
    p.sw(21, 0x34, 2, "OUT_HW_ARRAY_SIZE")
    p.sw(13, 0x38, 2, "OUT_POLL_ITERS")
    p.sw(14, 0x3C, 2, "OUT_CPU_WORK_DONE")

    p.ebreak("encerra execucao do firmware com halted=1 e trap=0")
    return p


# =============================================================================
# 2. Firmware Auxiliar: DECERR em leitura -> cpu_error=1 -> trap=1
# =============================================================================
def build_test_decerr_trap() -> Rv32iAsm:
    hdr = [
        "# =============================================================================",
        "# Projeto     : RISC-V RV32I com Controlador DMA e Coprocessador Matricial",
        "# Etapa       : Entrega Parcial IV — Integração Top-Level e Firmware",
        "# Arquivo     : test_decerr_trap.s (gerado por sw/firmware/build_firmware.py)",
        "# Professores : Ana Julia Fernandes de Oliveira Barros",
        "#               Igor Dantas dos Santos Miranda",
        "#               João Alberto Castelo Branco Oliveira",
        "# Grupo 4     : João Vitor Carvalho Lima",
        "#               Matheus Artur Macedo Bomfim",
        "#               Nathalia Miranda",
        "#               Tatiana Caetano Praxedes de Moura",
        "# Descrição   : Teste negativo de leitura fora de faixa (0x5000_0000):",
        "#               provoca RRESP=DECERR, propagação cpu_error=1 e trap=1.",
        "# =============================================================================",
    ]
    p = Rv32iAsm("test_decerr_trap", hdr)
    p.lui(1, 0x50000, "x1 = 0x5000_0000 (fora de DMEM e MMIO -> DECERR)")
    p.lw(2, 0, 1, "leitura ilegal -> deve ativar trap=1 antes do EBREAK")
    p.ebreak("nao deve ser alcancado em caso de trap correto")
    return p


# =============================================================================
# 3. Firmware Auxiliar: DECERR em escrita -> cpu_error=1 -> trap=1
# =============================================================================
def build_test_decerr_write() -> Rv32iAsm:
    hdr = [
        "# =============================================================================",
        "# Projeto     : RISC-V RV32I com Controlador DMA e Coprocessador Matricial",
        "# Etapa       : Entrega Parcial IV — Integração Top-Level e Firmware",
        "# Arquivo     : test_decerr_write.s (gerado por sw/firmware/build_firmware.py)",
        "# Professores : Ana Julia Fernandes de Oliveira Barros",
        "#               Igor Dantas dos Santos Miranda",
        "#               João Alberto Castelo Branco Oliveira",
        "# Grupo 4     : João Vitor Carvalho Lima",
        "#               Matheus Artur Macedo Bomfim",
        "#               Nathalia Miranda",
        "#               Tatiana Caetano Praxedes de Moura",
        "# Descrição   : Teste negativo de escrita fora de faixa (0x5000_0000):",
        "#               provoca BRESP=DECERR, propagação cpu_error=1 e trap=1.",
        "# =============================================================================",
    ]
    p = Rv32iAsm("test_decerr_write", hdr)
    p.lui(1, 0x50000, "x1 = 0x5000_0000 (fora de DMEM e MMIO -> DECERR)")
    p.sw(1, 0, 1, "escrita ilegal -> deve ativar trap=1 antes do EBREAK")
    p.ebreak("nao deve ser alcancado em caso de trap correto")
    return p


# =============================================================================
# 4. Firmware Auxiliar: Exercício dirigido da ponte cpu_axi4_bridge
#    Testa stores consecutivos no mesmo endereço, SB, SH, LB, LBU, LH, LHU e LW
# =============================================================================
def build_test_bridge_ops() -> Rv32iAsm:
    hdr = [
        "# =============================================================================",
        "# Projeto     : RISC-V RV32I com Controlador DMA e Coprocessador Matricial",
        "# Etapa       : Entrega Parcial IV — Integração Top-Level e Firmware",
        "# Arquivo     : test_bridge_ops.s (gerado por sw/firmware/build_firmware.py)",
        "# Professores : Ana Julia Fernandes de Oliveira Barros",
        "#               Igor Dantas dos Santos Miranda",
        "#               João Alberto Castelo Branco Oliveira",
        "# Grupo 4     : João Vitor Carvalho Lima",
        "#               Matheus Artur Macedo Bomfim",
        "#               Nathalia Miranda",
        "#               Tatiana Caetano Praxedes de Moura",
        "# Descrição   : Programa dirigido que exercita cpu_axi4_bridge com:",
        "#               - 3 SW consecutivos no mesmo endereço (0x0100), incluindo",
        "#                 dois com o mesmo valor para checar contagem 1:1 de AW/W/B;",
        "#               - SB em offsets +1 e +3 (0x0104) verificando WSTRB e preservação;",
        "#               - SH em offsets +0 e +2 (0x0108) verificando WSTRB e preservação;",
        "#               - LB, LBU, LH, LHU e LW gravando resultados em 0x0110..0x0124.",
        "# =============================================================================",
    ]
    p = Rv32iAsm("test_bridge_ops", hdr)
    p.addi(10, 0, 0x100, "x10 = 0x0100 (base de testes na area privada CPU)")

    # 1) Três SW consecutivos no mesmo endereço 0x0100 (2 com o mesmo dado)
    p.li32(1, 0x11223344, "x1 = 0x1122_3344")
    p.li32(2, 0x89ABCDEF, "x2 = 0x89AB_CDEF")
    p.sw(1, 0x00, 10, "store #1 em 0x0100 = 0x1122_3344")
    p.sw(2, 0x00, 10, "store #2 consecutivo em 0x0100 = 0x89AB_CDEF")
    p.sw(2, 0x00, 10, "store #3 consecutivo (mesmo dado) em 0x0100")

    # 2) SB em offsets +1 e +3 preservando os demais bytes de 0x0104
    p.li32(3, 0xAA55AA55, "x3 = 0xAA55_AA55")
    p.sw(3, 0x04, 10, "0x0104 = 0xAA55_AA55")
    p.addi(4, 0, 0x12, "x4 = 0x12")
    p.sb(4, 0x05, 10, "SB offset +1 (WSTRB=0010) -> 0xAA55_1255")
    p.addi(5, 0, -16, "x5 = 0xFFFFFFF0 (byte baixo = 0xF0)")
    p.sb(5, 0x07, 10, "SB offset +3 (WSTRB=1000) -> 0xF055_1255")

    # 3) SH em offsets +0 e +2 preservando a outra meia-palavra de 0x0108
    p.sw(1, 0x08, 10, "0x0108 = 0x1122_3344")
    p.li32(6, 0x000080FE, "x6 = 0x0000_80FE")
    p.sh(6, 0x08, 10, "SH offset +0 (WSTRB=0011) -> 0x1122_80FE")
    p.li32(7, 0x0000CAFE, "x7 = 0x0000_CAFE")
    p.sh(7, 0x0A, 10, "SH offset +2 (WSTRB=1100) -> 0xCAFE_80FE")

    # 4) Leituras LB, LBU, LH, LHU e LW de 0x0108 (0xCAFE_80FE) gravadas em 0x0110..0x0124
    p.lb(11, 0x08, 10, "LB  +0 (0xFE)   -> 0xFFFF_FFFE")
    p.lbu(12, 0x08, 10, "LBU +0 (0xFE)   -> 0x0000_00FE")
    p.lb(13, 0x09, 10, "LB  +1 (0x80)   -> 0xFFFF_FF80")
    p.lh(14, 0x08, 10, "LH  +0 (0x80FE) -> 0xFFFF_80FE")
    p.lhu(15, 0x0A, 10, "LHU +2 (0xCAFE) -> 0x0000_CAFE")
    p.lw(16, 0x08, 10, "LW  +0          -> 0xCAFE_80FE")

    p.sw(11, 0x10, 10, "0x0110 = resultado LB  +0")
    p.sw(12, 0x14, 10, "0x0114 = resultado LBU +0")
    p.sw(13, 0x18, 10, "0x0118 = resultado LB  +1")
    p.sw(14, 0x1C, 10, "0x011C = resultado LH  +0")
    p.sw(15, 0x20, 10, "0x0120 = resultado LHU +2")
    p.sw(16, 0x24, 10, "0x0124 = resultado LW  +0")

    p.ebreak("encerra teste dirigido da ponte com halted=1 e trap=0")
    return p


def emit_or_check(prog: Rv32iAsm, out_dir: Path, check_only: bool) -> None:
    words, asm_text = prog.assemble()
    if len(words) > IMEM_DEPTH_DEFAULT:
        raise ValueError(f"Programa '{prog.name}' excede IMEM_DEPTH={IMEM_DEPTH_DEFAULT}: {len(words)} palavras")
    # Preenche o .hex ate IMEM_DEPTH com NOP (addi x0, x0, 0) para $readmemh sem avisos
    padded_words = words + [RV32I_NOP] * (IMEM_DEPTH_DEFAULT - len(words))
    hex_text = "\n".join(f"{w:08X}" for w in padded_words) + "\n"
    bin_bytes = b"".join(struct.pack("<I", w) for w in words)

    paths = [
        (out_dir / f"{prog.name}.s", asm_text.encode("utf-8")),
        (out_dir / f"{prog.name}.hex", hex_text.encode("ascii")),
        (out_dir / f"{prog.name}.bin", bin_bytes),
    ]

    if check_only:
        for path, expected in paths:
            if not path.is_file():
                raise FileNotFoundError(f"Artefato de firmware ausente: {path}")
            actual = path.read_bytes()
            if path.suffix in (".s", ".hex"):
                actual = actual.replace(b"\r\n", b"\n")
            if actual != expected:
                raise AssertionError(f"Artefato desatualizado ou divergente: {path.name}")
    else:
        out_dir.mkdir(parents=True, exist_ok=True)
        for path, data in paths:
            path.write_bytes(data)

    mode = "VERIFICADO" if check_only else "GERADO"
    print(f"[PASS] Firmware '{prog.name}' {mode}: {len(words)} instrucoes ({len(bin_bytes)} bytes) -> {prog.name}.{{s,hex,bin}}")


def build_test_fw_timeout() -> Rv32iAsm:
    return build_matmul_dma(name="test_fw_timeout", max_poll_iters=12)


def main() -> int:
    parser = argparse.ArgumentParser(description="Gera ou valida os artefatos de firmware RV32I (.s, .hex, .bin).")
    parser.add_argument("--out-dir", type=Path, default=SCRIPT_DIR, help="Diretório de saída (padrão: sw/firmware/)")
    parser.add_argument("--check", action="store_true", help="Não sobrescreve; apenas valida sincronia dos arquivos")
    args = parser.parse_args()

    for builder in (
        build_matmul_dma,
        build_test_fw_timeout,
        build_test_decerr_trap,
        build_test_decerr_write,
        build_test_bridge_ops,
    ):
        emit_or_check(builder(), args.out_dir, args.check)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
