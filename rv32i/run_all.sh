#!/usr/bin/env bash
set -uo pipefail
cd "$(dirname "$0")"
modules=(adder.v mux2.v mux4.v pc.v register_file.v alu.v immediate_generator.v control_unit.v branch_unit.v instruction_memory.v load_store_unit.v data_memory.v rv32i_core.v)
failed=0
for name in test_alu test_branch test_memory test_jump test_ecall test_illegal test_misaligned test_oob; do
  echo "============ ${name} ============"
  if xrun -64bit -v2001 -clean "${modules[@]}" "tb_${name}.v" -top "tb_${name}" -access +rwc > "${name}.log" 2>&1; then
    grep -E '\[FAIL\]|TEST .*: (PASS|FAIL)|CASE .*:' "${name}.log" || true
    if ! grep -q "TEST ${name}: PASS" "${name}.log" || grep -q '\[FAIL\]' "${name}.log"; then failed=$((failed+1)); fi
  else
    echo "Falha de compilacao/elaboracao: ver ${name}.log"
    tail -n 12 "${name}.log"
    failed=$((failed+1))
  fi
done
if [ "$failed" -eq 0 ];then echo "ALL VERIFICATION TESTS: PASS";else echo "VERIFICATION FAILURES: $failed";fi
exit "$failed"
