# Verificação Unitária: DMA + Barramento AXI4 + DMEM + Acelerador (Entrega Parcial III)

Este diretório contém o ambiente de verificação unitária em SystemVerilog ([`tb_dma_mem_acc.sv`](tb_dma_mem_acc.sv)) para o subsistema integrado da **Entrega Parcial III** (sem CPU), exercitando em conjunto:

1. [`rtl/bus/axi4_interconnect.v`](../../rtl/bus/axi4_interconnect.v) — Matriz AXI4 ($2\text{ Mestres} \times 2\text{ Escravos} + \text{DECERR}$) com transações serializadas (1 transação ativa por vez) e prioridade fixa não preemptiva para o DMA.
2. [`rtl/memory/axi4_dmem.v`](../../rtl/memory/axi4_dmem.v) — Memória de Dados Compartilhada (16 KB, `4096` palavras de 32 bits) com interface escrava AXI4 para o subconjunto do projeto (palavras alinhadas de 32 bits, `SIZE=3'b010`, rajadas `INCR`, máscara `wstrb`, `LATENCY >= 1`, bloqueio de escrita quando `AW` é inválido e resposta `SLVERR` em erro de faixa/alinhamento ou `WLAST` inconsistente). O bloco `initial` que zera a RAM está isolado em `synthesis translate_off/on` (modelo exclusivo de simulação).
3. [`rtl/dma/dma_controller.v`](../../rtl/dma/dma_controller.v) — Controlador DMA autônomo com escravo MMIO (`0x4000_0000`–`0x4000_002F`), mestre AXI4 com quebra automática em fronteira de 4 KB, validação prévia de configuração, controle de *tiling* e *zero-padding* de bordas.
4. [`rtl/accelerator/`](../../rtl/accelerator/) — Coprocessador matricial sistólico da Entrega II (`mac_pe.v`, `systolic_array.v`, `systolic_accelerator.v`).
5. [`rtl/top/dma_mem_acc_subsys.v`](../../rtl/top/dma_mem_acc_subsys.v) — Wrapper de integração do subsistema para `ARRAY_SIZE` = 2, 4 e 8.

---

## Como Executar a Simulação (a partir da raiz do projeto)

### 1. Regressão única reproduzível (recomendado — 4 etapas)
Executa em sequência: (1) Golden Model Python (`12/12`), (2) conferência dos vetores `32×32` variados (`seed=2026`), (3) acelerador unitário (`30/30`) e (4) subsistema DMA + memória + acelerador (`44/44`):

```bash
# Icarus Verilog (-g2012):
bash run_regressao.sh iverilog

# Cadence Xcelium (xrun -64bit -sv):
bash run_regressao.sh xcelium
```

### 2. Execução direta apenas desta suíte (`tb_dma_mem_acc.sv`)
```bash
# Icarus Verilog 12.0:
bash tb/unit_dma_mem_acc/run_iverilog.sh
# Log salvo em: tb/unit_dma_mem_acc/sim_iverilog.log (44/44 PASS)

# Cadence Xcelium 24.09-s013:
bash tb/unit_dma_mem_acc/run_xcelium.sh
# Log salvo em: tb/unit_dma_mem_acc/sim_xcelium.log (44/44 PASS, 0 errors, 0 warnings)
```

*(O log anterior à consolidação foi preservado como referência histórica em [`historico_sim_xcelium_pre_consolidacao_38_38.log`](historico_sim_xcelium_pre_consolidacao_38_38.log).)*

---

## Cobertura dos Testes (`44/44 PASS`)

O testbench instancia quatro cenários físicos por meio de `subsys_bfm_harness` (`2x2`, `4x4`, `8x8` com `DMEM_LATENCY=1` — 14 subtestes cada — e `4x4` com `DMEM_LATENCY=3` — 2 subtestes), atuando como a CPU na porta Mestre 0 AXI4 (`m0_axi_*`). O ambiente combina monitores contínuos de protocolo e transferências (estabilidade de `VALID`/dados sob `READY=0`, fronteira de 4 KB em `AR`/`AW`, distinção entre `SLVERR` e `DECERR`, contagem de escritas `W` por endereço e ausência de atividade `M1` em comandos rejeitados) com verificações de integridade ao final das operações (preservação integral de $A$/$B$, escrita única por elemento válido de $C$, zero escritas fora da região de $C$ e comparação do resultado final):

| Subteste | Cenário Verificado | Critério de Aprovação (`PASS`) |
| :--- | :--- | :--- |
| **Teste 1** | Multiplicação $T \times T$, `IRQ_EN`, `PERF_*` e `W1C` | `HW_ARRAY_SIZE == T`, `PERF_COMP_CYCLES == 3T-2`, `PERF_MEM_READS == 2T^2`, `irq_done=1` e limpeza W1C em `DMA_STATUS.DONE`. |
| **Teste 2** | Caso unitário $1\times 1$ (`-128 * -128`) com bits `[31:8]` sujos | `C[0][0] == +16384`, `PERF_MEM_READS == 2` (sem leituras de *padding*). |
| **Teste 3** | Matrizes retangulares e não múltiplas (`3x5*5x2` e `5x7*7x6`) | *Zero-padding* automático pelo DMA e preservação de palavras sentinela adjacentes na DMEM (`0x2FFC`, `0x3018`). |
| **Teste 4** | Vetores `.hex` do Golden Model (`4x4` e `8x8`) | Comparação bit a bit contra `mat_*_C_expected.hex` e ciclos de cálculo (`256 / 80 / 22`). |
| **Teste 5** | Estresse máximo $32\times 32 \times 32\times 32$ com `-128` | Todos os `1024` elementos de $C$ iguais a `+524288` e `PERF_MEM_READS` idêntico ao Golden Model. |
| **Teste 6** | Bases customizadas (`0x0FF4..` e `0x0FFC..`) cruzando fronteiras de 4 KB | Verificação direta em cada handshake `AR`/`AW` (`addr[11:0] + 4*(LEN+1) <= 4096`) e contagem exata das rajadas divididas nas bordas `0x1000`, `0x2000` e `0x3000`. |
| **Teste 7** | Decodificação `DECERR` (`2'b11`) e erros de configuração (`ERR=1`) | Rejeição limpa (`ERR=1`, `DONE=0`, `BUSY=0`, `IRQ=0`, sem tráfego `M1` e com sentinelas preservadas em `0x3000`, `0x3FF8` e `0x3FFC`) para `M=0`, desalinhamento `A`, sobreposição `AB` e estouro `C > 16 KB`; `DECERR` sem tocar escravos. |
| **Teste 8** | Proteção em `BUSY=1` + Concorrência AXI4 (`M0` vs `M1`) | Escritas em `ADDR_C`/`DIM_M` durante `BUSY=1` ignoradas; acessos simultâneos de `M0` na DMEM concluem sem corromper $C$. |
| **Teste 9** | Abortamento seguro via `DMA_CTRL.SOFT_RST` em operação ativa | Na interconexão serializada, a escrita MMIO de `SOFT_RST` chega entre transações AXI4; verifica retorno a `BUSY=0`, limpeza de `DONE`, `ERR`, `irq_done` e contadores `PERF_*` e recuperação funcional imediata. |
| **Teste 10** | Reset assíncrono (`rst_n=0`) em execução ativa | Limpeza imediata dos controladores, verificação de `STATUS=0`, `PERF_*=0`, restauração de `ADDR_A=0x1000` e recuperação funcional completa. |
| **Teste 11** | Retenção de `RREADY=0` e `BREADY=0` + esperas em `AW`/`W`/`AR` | Manutenção estrita de `VALID` e dados/respostas com `RREADY=0`/`BREADY=0` (em `S0`, `S1` e `DECERR`) e esperas multi-ciclo (`ARREADY=0`, `AWREADY=0`, `WREADY=0`) impostas ao DMA. |
| **Teste 12** | Alteração controlada de endereços internos pós-validação (`SLVERR` vs `DECERR`) | Modifica `reg_addr_a`/`reg_addr_c` após a validação inicial para provocar respostas reais da DMEM (`SLVERR=2'b10`) e da interconexão (`DECERR=2'b11`) em `RRESP` e `BRESP`, distinguindo individualmente cada código, verificando `ERR=1`, `DONE=0`, `irq_done=0` e recuperação posterior. |
| **Teste 13** | Vetores Python $32\times 32$ variados (`seed=2026`, `1024` resultados) | Carrega `mat_32x32_varied_{A,B,C_expected}.hex` (positivos, negativos e extremos `[-128..127]`); valida `1024/1024` palavras de $C$ contra os vetores Python, escrita única por elemento (`W_BEATS=1024`), zero escritas fora de $C$, preservação integral de $A$/$B$, sentinelas externas (`0x0000`, `0x0FF8`, `0x0FFC`), `IRQ_EN`/`W1C`; `PERF_COMP_CYCLES` e `PERF_MEM_READS` comparados com o Golden Model e `PERF_TOTAL_CYCLES` medido e registrado (`2x2/4x4/8x8`). |
| **Teste 14** | Limites exatos da DMEM (`0x3FFC`) e rejeições de fronteira | Casos positivos na última palavra válida `0x3FFC` (`1x1` com sentinelas `0x1FF8`/`0x2FF8`/`0x3FF8` e `2x2` terminando exatamente em `0x4000` com `A=0x2FF8` cruzando `0x3000`); 9 rejeições (`M/K/N=33`, desalinhamento `B`/`C`, estouro `+4B` em `0x3FFC` para `A`/`B`/`C`, sobreposição de borda `BC`) comprovando `ERR=1`, `DONE=0`, `BUSY=0`, sem tráfego `M1` e com sentinelas preservadas (`0x3000`, `0x3FF8`, `0x3FFC`). |
| **Teste L3-1** | `DMEM_LATENCY = 3` (4×4, $7\times 9 \times 9\times 5$, quebra 4 KB verificada) | Tolerância completa a *wait states* da memória com verificação de fronteira 4 KB, escrita única por elemento de $C$, `IRQ_EN` e `W1C`. |
| **Teste L3-2** | `DMEM_LATENCY = 3` com retenção de `RREADY`/`BREADY` e provocação controlada de `SLVERR` | Manutenção de `VALID`/dados sob latência 3 + retenção de `READY=0`, distinção exata de `SLVERR` (`2'b10`) pelo DMA e recuperação completa. |

---

## Desempenho Medido no Cenário Python `32×32` Variado (Teste 13)

| Malha (`ARRAY_SIZE`) | `PERF_TOTAL_CYCLES` | `PERF_COMP_CYCLES` (Golden) | `PERF_MEM_READS` (Golden) | `W_BEATS` (escrita única em $C$) | Razão de ciclos vs `2×2` |
| :---: | :---: | :---: | :---: | :---: | :---: |
| **2×2** | `151.552` | `16.384` | `32.768` | `1.024` | `1,00×` |
| **4×4** | `46.656` | `5.120` | `16.384` | `1.024` | `3,25×` |
| **8×8** | `16.864` | `1.408` | `8.192` | `1.024` | `8,99×` |

*Comparação baseada na contagem de ciclos, sob as mesmas condições de simulação e supondo a mesma frequência de clock. Não representa comparação de frequência máxima ou desempenho após síntese.*

*(Nota metodológica: nos testes `32×32`, a carga inicial de $A$/$B$ e o pré-preenchimento de $C$ utilizam acesso hierárquico `fast_load_matrices_dmem` apenas para preparação da memória; toda a movimentação realizada pelo DMA — leitura de $A$/$B$, processamento no acelerador e escrita de $C$ — ocorre integralmente pelas interfaces AXI4 sob monitoramento contínuo.)*

## Integridade e Escopo da Entrega Parcial III

- **Integridade de entradas e saídas:** antes de cada multiplicação verificada por `verify_c_in_dmem`, o testbench salva o snapshot das palavras completas de $A$/$B$ (incluindo bits `[31:8]`) e confere sua preservação integral ao final, em conjunto com a contagem de exatamente 1 escrita por elemento válido de $C$ e 0 escritas fora da região de $C$.
- **Escopo consolidado:** a Entrega Parcial III possui a arquitetura AXI4 e incorpora vetores `32×32` variados (`seed=2026`, `1.024` resultados) e cenários de verificação de limites adaptados ao contrato global de 16 KB da DMEM.
