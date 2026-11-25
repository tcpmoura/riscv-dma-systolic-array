#!/usr/bin/env bash
# =============================================================================
# Projeto     : RISC-V RV32I com Controlador DMA e Coprocessador Matricial
# Etapa       : Entrega Parcial IV — Integração Top-Level e Firmware
# Script      : run_regressao.sh
# Professores : Ana Julia Fernandes de Oliveira Barros
#               Igor Dantas dos Santos Miranda
#               João Alberto Castelo Branco Oliveira
# Grupo 4     : João Vitor Carvalho Lima
#               Matheus Artur Macedo Bomfim
#               Nathalia Miranda
#               Tatiana Caetano Praxedes de Moura
# Descrição   : Regressão única reproduzível do projeto (Entregas I a IV-i/ii).
#               Executa em sequência:
#                 1) Testes unitários do Golden Model oficial (sw/golden_model/)
#                 2) Conferência dos vetores Python 32x32 variados (seed=2026)
#                 3) Conferência do firmware RV32I (.s, .hex e .bin)
#                 4) Testes unitários do Acelerador (Entrega Parcial II)
#                 5) Testes integrados DMA + Barramento AXI4 + DMEM + Acelerador
#                 6) Testes integrados do SoC completo (RV32I + Firmware + DMA)
#
# Uso (a partir da raiz do projeto):
#   bash run_regressao.sh iverilog    # (padrão)
#   bash run_regressao.sh xcelium
# =============================================================================
set -euo pipefail

ROOT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
cd "${ROOT_DIR}"

SIM="${1:-iverilog}"
SIM_TIMEOUT="${SIM_TIMEOUT:-600}"
LOG_DIR="logs_regressao"
mkdir -p "${LOG_DIR}"

if [[ "${SIM}" != "iverilog" && "${SIM}" != "xcelium" ]]; then
    echo "[ERRO] Simulador inválido: '${SIM}'. Use 'iverilog' ou 'xcelium'." >&2
    exit 2
fi

if command -v python3 >/dev/null 2>&1; then
    PYTHON_BIN="python3"
elif command -v python >/dev/null 2>&1; then
    PYTHON_BIN="python"
else
    echo "[ERRO] Python 3 não encontrado no PATH." >&2
    exit 1
fi

run_with_timeout() {
    if command -v timeout >/dev/null 2>&1; then
        timeout "${SIM_TIMEOUT}" "$@"
    else
        "$@"
    fi
}

assert_no_fail_in_log() {
    local log_file="$1"
    if grep -q '\[FAIL\]' "${log_file}"; then
        echo "[ERRO] Falha encontrada no log: ${log_file}" >&2
        exit 1
    fi
}

echo "====================================================================="
echo "  REGRESSAO UNIFICADA — RISCV + DMA + SYSTOLIC ARRAY (sim: ${SIM})"
echo "====================================================================="

# -----------------------------------------------------------------------------
# Etapa 1: Testes unitários do Golden Model oficial (sem contagem fixa)
# -----------------------------------------------------------------------------
echo ""
echo "[1/6] Executando testes do Golden Model oficial..."
GM_LOG="${LOG_DIR}/golden_model_python.log"
"${PYTHON_BIN}" -B sw/golden_model/test_golden_model.py 2>&1 | tee "${GM_LOG}"
grep -q "TODOS OS TESTES PASSARAM" "${GM_LOG}"
assert_no_fail_in_log "${GM_LOG}"

# -----------------------------------------------------------------------------
# Etapa 2: Conferência de reprodutibilidade dos vetores 32x32 variados
# -----------------------------------------------------------------------------
echo ""
echo "[2/6] Verificando vetores Python 32x32 variados (seed=2026)..."
VEC_LOG="${LOG_DIR}/vetores_32x32_check.log"
"${PYTHON_BIN}" -B sw/golden_model/generate_32x32_vectors.py --check 2>&1 | tee "${VEC_LOG}"
grep -q "\[PASS\]" "${VEC_LOG}"
assert_no_fail_in_log "${VEC_LOG}"

# -----------------------------------------------------------------------------
# Etapa 3: Conferência do firmware RV32I (.s, .hex e .bin)
# -----------------------------------------------------------------------------
echo ""
echo "[3/6] Verificando firmware RV32I (sw/firmware/)..."
FW_LOG="${LOG_DIR}/firmware_check.log"
"${PYTHON_BIN}" -B sw/firmware/build_firmware.py --check 2>&1 | tee "${FW_LOG}"
grep -q "\[PASS\]" "${FW_LOG}"
assert_no_fail_in_log "${FW_LOG}"

# -----------------------------------------------------------------------------
# Etapa 4: Testes unitários do Acelerador Sistólico (Entrega Parcial II)
# -----------------------------------------------------------------------------
echo ""
echo "[4/6] Executando testes unitários do Acelerador (${SIM})..."
ACC_LOG="${LOG_DIR}/accelerator_${SIM}.log"

if [[ "${SIM}" == "iverilog" ]]; then
    ACC_VVP="${LOG_DIR}/tb_systolic_accelerator.vvp"
    trap 'rm -f "${ACC_VVP}" "tb_dma_mem_acc.vvp" "tb_riscv_dma_soc.vvp" "tb_systolic_accelerator.vcd"' EXIT
    iverilog -g2012 -o "${ACC_VVP}" \
        rtl/accelerator/mac_pe.v \
        rtl/accelerator/systolic_array.v \
        rtl/accelerator/systolic_accelerator.v \
        tb/unit_accelerator/tb_systolic_accelerator.sv
    run_with_timeout vvp "${ACC_VVP}" 2>&1 | tee "${ACC_LOG}"
    cp "${ACC_LOG}" tb/unit_accelerator/sim_iverilog.log
else
    trap 'rm -f "tb_systolic_accelerator.vcd"' EXIT
    run_with_timeout xrun -64bit -sv -clean \
        rtl/accelerator/mac_pe.v \
        rtl/accelerator/systolic_array.v \
        rtl/accelerator/systolic_accelerator.v \
        tb/unit_accelerator/tb_systolic_accelerator.sv \
        -top tb_systolic_accelerator \
        -access +rwc \
        -l "${ACC_LOG}"
    cp "${ACC_LOG}" tb/unit_accelerator/sim_xcelium.log
fi

grep -q "SUBTESTES PASSARAM.*- PASS!" "${ACC_LOG}"
assert_no_fail_in_log "${ACC_LOG}"

# -----------------------------------------------------------------------------
# Etapa 5: Testes integrados DMA + Barramento AXI4 + DMEM + Acelerador
# -----------------------------------------------------------------------------
echo ""
echo "[5/6] Executando testes integrados DMA + Memória + Acelerador (${SIM})..."
DMA_LOG="${LOG_DIR}/dma_mem_acc_${SIM}.log"

if [[ "${SIM}" == "iverilog" ]]; then
    SIM_TIMEOUT="${SIM_TIMEOUT}" bash tb/unit_dma_mem_acc/run_iverilog.sh
    cp tb/unit_dma_mem_acc/sim_iverilog.log "${DMA_LOG}"
else
    SIM_TIMEOUT="${SIM_TIMEOUT}" bash tb/unit_dma_mem_acc/run_xcelium.sh
    cp tb/unit_dma_mem_acc/sim_xcelium.log "${DMA_LOG}"
fi

grep -q "ALL TESTS PASSED!" "${DMA_LOG}"
grep -q "Erros: 0" "${DMA_LOG}"
assert_no_fail_in_log "${DMA_LOG}"

# -----------------------------------------------------------------------------
# Etapa 6: Testes integrados do SoC completo (RV32I + Firmware + Ponte + DMA)
# -----------------------------------------------------------------------------
echo ""
echo "[6/6] Executando testes integrados do SoC completo (${SIM})..."
SOC_LOG="${LOG_DIR}/soc_riscv_dma_${SIM}.log"

if [[ "${SIM}" == "iverilog" ]]; then
    SIM_TIMEOUT="${SIM_TIMEOUT}" bash tb/unit_cpu/run_iverilog.sh
    cp tb/unit_cpu/sim_iverilog.log "${SOC_LOG}"
else
    SIM_TIMEOUT="${SIM_TIMEOUT}" bash tb/unit_cpu/run_xcelium.sh
    cp tb/unit_cpu/sim_xcelium.log "${SOC_LOG}"
fi

grep -q "ALL TESTS PASSED!" "${SOC_LOG}"
grep -q "Erros: 0" "${SOC_LOG}"
assert_no_fail_in_log "${SOC_LOG}"

echo ""
echo "====================================================================="
echo "  REGRESSAO CONCLUIDA COM SUCESSO (6/6 etapas PASS | simulador: ${SIM})"
echo "  Logs salvos em: ${LOG_DIR}/"
echo "====================================================================="
