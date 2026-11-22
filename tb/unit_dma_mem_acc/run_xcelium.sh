#!/usr/bin/env bash
# =============================================================================
# Projeto     : RISC-V RV32I com Controlador DMA e Coprocessador Matricial
# Etapa       : Entrega Parcial III — DMA, Barramento AXI4 e Memória Compartilhada
# Script      : run_xcelium.sh
# Professores : Ana Julia Fernandes de Oliveira Barros
#               Igor Dantas dos Santos Miranda
#               João Alberto Castelo Branco Oliveira
# Grupo 4     : João Vitor Carvalho Lima
#               Matheus Artur Macedo Bomfim
#               Nathalia Miranda
#               Tatiana Caetano Praxedes de Moura
# Descrição   : Compila e executa a suíte integrada da Entrega Parcial III no
#               Cadence Xcelium (xrun -64bit -sv), salvando o log completo em
#               tb/unit_dma_mem_acc/sim_xcelium.log e validando a aprovação.
# Uso (a partir da raiz do repositório):
#   bash tb/unit_dma_mem_acc/run_xcelium.sh
# =============================================================================
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
ROOT_DIR="$(cd "${SCRIPT_DIR}/../.." && pwd)"
cd "${ROOT_DIR}"

SIM_TIMEOUT="${SIM_TIMEOUT:-600}"
LOG_FILE="tb/unit_dma_mem_acc/sim_xcelium.log"

XRUN_ARGS=(
    -64bit -sv -clean
    rtl/accelerator/mac_pe.v
    rtl/accelerator/systolic_array.v
    rtl/accelerator/systolic_accelerator.v
    rtl/memory/axi4_dmem.v
    rtl/bus/axi4_interconnect.v
    rtl/dma/dma_controller.v
    rtl/top/dma_mem_acc_subsys.v
    tb/unit_dma_mem_acc/tb_dma_mem_acc.sv
    -top tb_dma_mem_acc
    -access +rwc
    -l "${LOG_FILE}"
)

if command -v timeout >/dev/null 2>&1; then
    timeout "${SIM_TIMEOUT}" xrun "${XRUN_ARGS[@]}"
else
    xrun "${XRUN_ARGS[@]}"
fi

grep -q "ALL TESTS PASSED!" "${LOG_FILE}"
grep -q "Erros: 0" "${LOG_FILE}"
if grep -q '\[FAIL\]' "${LOG_FILE}"; then
    echo "[ERRO] Falha encontrada no log: ${LOG_FILE}" >&2
    exit 1
fi
