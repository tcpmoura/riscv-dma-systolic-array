# RV32I — datapath modularizado

As somas e seleções do núcleo foram transformadas em instâncias de `adder`, `mux2` e `mux4`.
Nenhuma porta externa de `rv32i_core` foi alterada. Os nomes internos `pc4`, `pc_target`, `pc_next`, `alu_a`, `alu_b`, `writeback` foram preservados.

**Execução:** `bash run_all.sh` (8 testes) ou `bash run_xcelium_integration.sh` (teste integrado).

**Novos módulos:**
- `add_pc4_inst`: PC + 4
- `add_branch_inst`: PC + imediato (Branch/JAL)
- `add_jalr_inst`: rs1 + imediato (JALR)
- `mux_jalr_target_inst`: seleciona alvo de branch/jump ou JALR
- `mux_pc_inst`: seleciona sequencial PC+4 ou alvo
- `mux_alu_a_inst`: rs1/PC/zero
- `mux_alu_b_inst`: rs2/imediato
- `mux_wb_inst`: resultado ALU/load/PC+4

Não há modificação do protocolo `valid/ready`, do Stall, do controle de conclusão ou das exceções. A síntese poderá otimizar e combinar fisicamente os blocos lógicos.

A simulação deve ser executada novamente no Xcelium; não é validada pelo material produzido aqui.
