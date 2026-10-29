# Verificação funcional ampliada — RV32I com Stall

Suite de **8 programas independentes**, para executar com o **Xcelium 24.09**. Cada programa possui arquivo `.s` (arquivos Assembly), `.hex` (palavras de instrução) e `tb_*.v` (testbench autochecking). Os nove módulos RTL do núcleo e a memória estão incluídos.

## Execução

Na pasta extraída, execute:

```bash
bash run_all.sh
```

O script executa separadamente oito compilações usando `-clean`, cria um `.log` por caso e informa `ALL VERIFICATION TESTS: PASS` somente se todos os resumos exibirem PASS. Para execução isolada, por exemplo:

```bash
xrun -64bit -v2001 -clean pc.v register_file.v alu.v immediate_generator.v control_unit.v branch_unit.v instruction_memory.v load_store_unit.v data_memory.v rv32i_core.v tb_test_memory.v -top tb_test_memory -access +rwc
```

Use diretório de trabalho com os `.hex` na raiz. Os arquivos `test_*.s` contêm código Assembly com rótulos e correspondem às instruções codificadas nos arquivos hexadecimais.

## Cobertura dirigida

| Caso | O que valida | Critério |
|---|---|---|
| `test_alu` | R/I ALU, signed/unsigned, deslocamentos, LUI, AUIPC | Registradores esperados |
| `test_branch` | BEQ/BNE/BLT/BGE/BLTU/BGEU tomados e não tomados; desvio para trás | Registradores/contadores |
| `test_memory` | LW/LB/LBU/LH/LHU e SW/SB/SH com bytes em posições diversas, Stall com latência 3 | Registradores e RAM |
| `test_jump` | JAL, JALR, endereço de retorno, limpeza do bit 0 do alvo | Registradores |
| `test_ecall` | Instrução ECALL | trap |
| `test_illegal` | Opcode inválido | trap |
| `test_misaligned` | LW desalinhado | trap |
| `test_oob` | LW além da RAM | trap |

**Atenção:** os cenários ECALL, alinhamento e acesso fora da RAM validam a política didática atual de `trap` do projeto, não a arquitetura completa de exceções/CSRs do RISC-V. Os testes não demonstram cobertura exaustiva, formal ou UVM. O status `PASS` é esperado e precisa ser confirmado no ambiente Xcelium do usuário.

## Sinais recomendados para SimVision

`clk`, `rst_n`, `pc_debug`, `cpu.instruction`, `cpu.alu_result`, `cpu.imm`, `cpu.rs1_data`, `cpu.rs2_data`, `cpu.rf_write`, `cpu.writeback`, `valid`, `ready`, `wr`, `addr`, `wdata`, `wstrb`, `rdata`, `retire`, `halted`, `trap`.

Monitorar PC constante durante `valid=1, ready=0`, seguido de uma só conclusão e de uma só escrita. Para cobertura plena RV32I faltam expansões como FENCE e casos de limites e falhas adicionais.
