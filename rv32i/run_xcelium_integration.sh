#!/usr/bin/env bash
set -euo pipefail
cd "$(dirname "$0")"
xrun -64bit -v2001 -clean adder.v mux2.v mux4.v pc.v register_file.v alu.v \
 immediate_generator.v control_unit.v branch_unit.v instruction_memory.v \
 load_store_unit.v data_memory.v rv32i_core.v rv32i_soc_tb.v \
 -top rv32i_soc_tb -access +rwc
