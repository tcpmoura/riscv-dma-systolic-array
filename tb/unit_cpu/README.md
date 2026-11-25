# Verificação Integrada do SoC Completo: RV32I + Ponte AXI4 + DMA + DMEM + Acelerador

Este diretório contém o testbench autoverificável em SystemVerilog ([`tb_riscv_dma_soc.sv`](tb_riscv_dma_soc.sv)) que valida o Top-Level completo [`rtl/top/riscv_dma_soc_top.v`](../../rtl/top/riscv_dma_soc_top.v) com o processador **RISC-V RV32I real** executando os firmwares gerados por [`sw/firmware/build_firmware.py`](../../sw/firmware/build_firmware.py).

## Como Executar (a partir da raiz do projeto)

```bash
# Icarus Verilog (-g2012):
bash tb/unit_cpu/run_iverilog.sh

# Cadence Xcelium (xrun -64bit -sv):
bash tb/unit_cpu/run_xcelium.sh
```

## Modelo Operacional da Simulação

- Antes de liberar `rst_n`, o testbench inicializa o descritor (`0x0000..0x001C`) e as matrizes de entrada `A` e `B` na `DMEM`.
- Após a liberação do reset, a CPU executa o firmware autonomamente, configura os registradores MMIO do DMA, dispara a operação e acompanha a conclusão por **polling** em `DMA_STATUS`.
- O sinal `irq_done` é monitorado na porta externa do Top-Level `riscv_dma_soc_top` (não há rotina de interrupção na CPU `rv32i_core`).

## Monitores Contínuos de Protocolo e Contrato

Ativos durante toda a execução dos subtestes:

1. **Conformidade AXI4 da ponte (`cpu_axi4_bridge` / `M0`):** ausência de `X/Z`, `LEN=0`, `SIZE=3'b010`, `WLAST=1`, estabilidade de `AW`/`W`/`AR` durante `READY=0`, e `cpu_error=0` fora do ciclo `cpu_ready=1`.
2. **Estabilidade da interface da CPU:** enquanto `cpu_valid=1 && cpu_ready=0`, verifica que `debug_pc`, `cpu_write`, `cpu_addr`, `cpu_wdata` e `cpu_wstrb` permanecem estáveis e que `retire=0`.
3. **Correspondência estrita 1:1:** cada escrita concluída na CPU corresponde a exatamente 1 `AW`, 1 `W` e 1 `B` em `M0`; cada leitura concluída na CPU corresponde a exatamente 1 `AR` e 1 `R`.
4. **Guarda da área reservada e autonomia do DMA:** nenhuma escrita da CPU na DMEM ocorre fora de `0x0000..0x01FF`, e a CPU realiza **zero** leituras ou escritas nas regiões das matrizes `A`, `B` e `C`.
5. **Concorrência real com `DMA_STATUS.BUSY = 1`:** nos testes com `DESC_CPU_WORK > 0`, exige ao menos uma escrita da CPU em `0x0100..0x01FC` aceita enquanto o DMA está ativamente ocupado (`status_busy == 1`).
6. **Integridade do DMA (`M1`):** zero violações de fronteira 4 KB, preservação integral de `A` e `B`, escrita única em cada elemento válido de `C` e zero escritas fora de `C`.

## Cobertura (`27/27 subtestes`)

O testbench instancia o SoC completo em `ARRAY_SIZE = 2`, `4` e `8` (`DMEM_LATENCY = 1`), `4×4` com `DMEM_LATENCY = 3`, o teste dirigido de timeout (`test_fw_timeout.hex`) e três cenários dirigidos da ponte (`test_decerr_trap.hex`, `test_decerr_write.hex` e `test_bridge_ops.hex`):

| Subteste | Cenário Verificado (via Firmware RV32I real) | Critério de Aprovação (`PASS`) |
| :--- | :--- | :--- |
| **Teste 1** *(2×2, 4×4, 8×8)* | Multiplicação `T×T`, `IRQ_EN`, `PERF_*` e `W1C` | `halted=1`, `trap=0`, `OUT_MAGIC="DONE"`, `irq_seen=1`, `irq_done=0` no fim, `0` acessos da CPU às matrizes e `C` exata. |
| **Teste 2** *(2×2, 4×4, 8×8)* | Vetores `.hex` do Golden Model (`4×4` e `8×8`) | Comparação bit a bit contra `mat_4x4_C_expected.hex` e `mat_8x8x8_C_expected.hex`, com `OUT_COMP_CYCLES` e `OUT_MEM_READS` exatos. |
| **Teste 3** *(2×2, 4×4, 8×8)* | Retangular `5×7 × 7×6` + concorrência real CPU × DMA | CPU grava até 16 palavras em `0x0100..0x01FC` durante o *polling* com comprovação de escrita enquanto `DMA_STATUS.BUSY=1`; `C`, área privada e sentinelas íntegras. |
| **Teste 4** *(2×2, 4×4, 8×8)* | Vetores Python `32×32` variados (`seed=2026`, `1.024` resultados) | `1.024/1.024` resultados contra `mat_32x32_varied_C_expected.hex`, escrita única em `C`, zero escritas fora de `C`, `A/B` intactas e `WORK_DURING_BUSY >= 1`. |
| **Teste 5** *(2×2, 4×4, 8×8)* | Configuração inválida (`M=33`) | DMA sinaliza `ERR=1`, firmware limpa via W1C, grava `OUT_MAGIC="ERR!"`, `OUT_DMA_STATUS=0x4`, zero tráfego `M1` e encerra sem `trap`. |
| **Teste 6** *(2×2, 4×4, 8×8)* | Bases customizadas cruzando 4 KB (`0x0FFC/0x1FFC/0x2FFC`) | Execução via firmware com quebra de rajada 4 KB sem violações (`m1_4kb_violations=0`). |
| **Teste FW-1** *(4×4)* | Proteção da área reservada `0x0000..0x01FF` | `A=0x0000`, `C=0x0020` (`1×1`, `2×3`) e `B=0x0100` rejeitados com `OUT_MAGIC="FWER"` e `0x80000001` (zero tráfego `M1`); fronteira `0x0200` executa e entrega `C=6`. |
| **Teste FW-2** *(4×4)* | Limite de `DESC_CPU_WORK` (`0`, `64` e `65`) | `0` não toca `0x0100`; `64` (matriz `16×16`) grava `0x0100..0x01FC` mantendo sentinela `0x0200` intacta; `65` rejeitado com `"FWER"` / `0x80000002` sem tocar `0x0200`. |
| **Teste FW-3** *(4×4)* | Validação de `DESC_CTRL` (`0`, `1`, `2` e `4`) | `0` usa `0x3` (`irq_seen=1`); `1` executa sem IRQ (`irq_seen=0`); `2` (sem `START`) e `4` (`SOFT_RST`) rejeitados imediatamente com `"FWER"` / `0x80000003`. |
| **Teste FW-4** *(4×4)* | Timeout de *polling* (`test_fw_timeout.hex`, `MAX_POLL_ITERS=12`) e recuperação pós-reset | `8×8` atinge limite (`OUT_MAGIC="FWER"`, `0x80000004`, `OUT_POLL_ITERS=12`, `halted=1`, `DMA_BUSY=1`); após `rst_n=0`, DMA volta a ocioso e nova execução `1×1` nas mesmas bases conclui com `"DONE"` (`C=6`). |
| **Teste L3-1** *(4×4, L=3)* | `DMEM_LATENCY = 3` (`7×9 × 9×5`, quebra 4 KB e concorrência) | Ponte `cpu_axi4_bridge` e CPU toleram *wait states* da memória mantendo conformidade AXI4 estrita e `WORK_DURING_BUSY >= 1`. |
| **Teste RESET-1** *(4×4, L=3)* | Reset assíncrono durante transação pendente na ponte | `rst_n=0` com `cpu_valid=1 && cpu_ready=0` zera imediatamente as saídas da ponte e retorna a `ST_IDLE`; liberação do reset conclui multiplicação `4×4` limpa. |
| **Teste TRAP-1** *(4×4)* | Leitura ilegal da CPU (`0x5000_0000`, `test_decerr_trap.hex`) | `RRESP=DECERR` → `cpu_error=1` no ciclo `cpu_ready=1` → `trap=1`, `halted=0`, `debug_pc=0x00000004`. |
| **Teste TRAP-2** *(4×4)* | Escrita ilegal da CPU (`0x5000_0000`, `test_decerr_write.hex`) | `BRESP=DECERR` → `cpu_error=1` no ciclo `cpu_ready=1` → `trap=1`, `halted=0`, `debug_pc=0x00000004`. |
| **Teste BRIDGE-1** *(4×4)* | Operações dirigidas na ponte (`test_bridge_ops.hex`) | 3 `SW` consecutivos em `0x0100` (1:1 `AW`/`W`/`B`), `SB`/`SH` com `WSTRB` (`0010`, `1000`, `0011`, `1100`) preservando bytes vizinhos e `LB`/`LBU`/`LH`/`LHU`/`LW` exatos. |
