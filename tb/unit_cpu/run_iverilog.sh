#!/usr/bin/env bash
# =============================================================================
# Projeto     : RISC-V RV32I com Controlador DMA e Coprocessador Matricial
# Etapa       : Entrega Parcial IV — Integração Top-Level e Firmware
# Script      : run_iverilog.sh
# Professores : Ana Julia Fernandes de Oliveira Barros
#               Igor Dantas dos Santos Miranda
#               João Alberto Castelo Branco Oliveira
# Grupo 4     : João Vitor Carvalho Lima
#               Matheus Artur Macedo Bomfim
#               Nathalia Miranda
#               Tatiana Caetano Praxedes de Moura
# Descrição   : Compila e executa a suíte integrada do SoC completo (RV32I +
#               Ponte AXI4 + DMA + DMEM + Acelerador + Firmware) no Icarus
#               Verilog (-g2012), salvando o log em tb/unit_cpu/sim_iverilog.log.
# Uso (a partir da raiz do repositório):
#   bash tb/unit_cpu/run_iverilog.sh
# =============================================================================
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
ROOT_DIR="$(cd "${SCRIPT_DIR}/../.." && pwd)"
cd "${ROOT_DIR}"

SIM_TIMEOUT="${SIM_TIMEOUT:-600}"
OUT_VVP="tb_riscv_dma_soc.vvp"
LOG_FILE="tb/unit_cpu/sim_iverilog.log"

trap 'rm -f "${OUT_VVP}"' EXIT

iverilog -g2012 -o "${OUT_VVP}" \
    rv32i/adder.v \
    rv32i/alu.v \
    rv32i/branch_unit.v \
    rv32i/control_unit.v \
    rv32i/immediate_generator.v \
    rv32i/instruction_memory.v \
    rv32i/load_store_unit.v \
    rv32i/mux2.v \
    rv32i/mux4.v \
    rv32i/pc.v \
    rv32i/register_file.v \
    rv32i/rv32i_core.v \
    rtl/accelerator/mac_pe.v \
    rtl/accelerator/systolic_array.v \
    rtl/accelerator/systolic_accelerator.v \
    rtl/memory/axi4_dmem.v \
    rtl/bus/axi4_interconnect.v \
    rtl/dma/dma_controller.v \
    rtl/top/dma_mem_acc_subsys.v \
    rtl/cpu/cpu_axi4_bridge.v \
    rtl/top/riscv_dma_soc_top.v \
    tb/unit_cpu/tb_riscv_dma_soc.sv

if command -v timeout >/dev/null 2>&1; then
    timeout "${SIM_TIMEOUT}" vvp "${OUT_VVP}" 2>&1 | tee "${LOG_FILE}"
else
    vvp "${OUT_VVP}" 2>&1 | tee "${LOG_FILE}"
fi

grep -q "ALL TESTS PASSED!" "${LOG_FILE}"
grep -q "Erros: 0" "${LOG_FILE}"
if grep -q '\[FAIL\]' "${LOG_FILE}"; then
    echo "[ERRO] Falha encontrada no log: ${LOG_FILE}" >&2
    exit 1
fi
