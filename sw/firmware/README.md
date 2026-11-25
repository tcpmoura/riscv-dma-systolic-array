# Firmware RISC-V RV32I — Configuração e Controle do DMA (Entrega Parcial IV)

Este diretório contém o gerador reprodutível em Python, os códigos-fonte em Assembly RISC-V (`RV32I`) e os binários gerados (`.hex` para `$readmemh` na `IMEM`, preenchidos com `NOP` `00000013` até 256 palavras, e `.bin` em formato *little-endian*) executados pelo processador `rv32i_core` no Top-Level [`rtl/top/riscv_dma_soc_top.v`](../../rtl/top/riscv_dma_soc_top.v).

> **Fonte editável:** o arquivo [`build_firmware.py`](build_firmware.py) é a fonte única que gera simultaneamente `.s`, `.hex` e `.bin`. Editar apenas um arquivo `.s` **não** atualiza os binários `.hex` e `.bin`; qualquer alteração no firmware deve ser feita em `build_firmware.py`, seguida da sua execução e conferência com `--check`.

## Arquivos

| Arquivo | Função |
| :--- | :--- |
| [`build_firmware.py`](build_firmware.py) | Montador determinístico RV32I em Python (fonte editável que gera e valida `.s`, `.hex` e `.bin` sem depender de toolchain externo). |
| [`matmul_dma.s`](matmul_dma.s) / [`.hex`](matmul_dma.hex) / [`.bin`](matmul_dma.bin) | Firmware principal (**90 instruções / 360 bytes**): valida o descritor, protege a área reservada `0x0000..0x01FF`, limita `DESC_CPU_WORK` (`0..64`) e `DESC_CTRL` (`{0, 1, 3}`), configura e dispara o DMA, executa *polling* limitado (`MAX_POLL_ITERS = 200.000`, com trabalho concorrente opcional), limpa flags por W1C, grava telemetria e encerra via `EBREAK`. |
| [`test_fw_timeout.s`](test_fw_timeout.s) / [`.hex`](test_fw_timeout.hex) / [`.bin`](test_fw_timeout.bin) | Variante dirigida do mesmo fluxo de `matmul_dma` com `MAX_POLL_ITERS = 12` (**90 instruções / 360 bytes**) para verificar `FW_ERR_TIMEOUT` (`0x80000004`) e a recuperação após reset global. |
| [`test_decerr_trap.s`](test_decerr_trap.s) / [`.hex`](test_decerr_trap.hex) / [`.bin`](test_decerr_trap.bin) | Programa dirigido negativo (**3 instruções / 12 bytes**): leitura em endereço não mapeado (`0x5000_0000`) → `RRESP=DECERR` → `cpu_error=1` → `trap=1`. |
| [`test_decerr_write.s`](test_decerr_write.s) / [`.hex`](test_decerr_write.hex) / [`.bin`](test_decerr_write.bin) | Programa dirigido negativo (**3 instruções / 12 bytes**): escrita em endereço não mapeado (`0x5000_0000`) → `BRESP=DECERR` → `cpu_error=1` → `trap=1`. |
| [`test_bridge_ops.s`](test_bridge_ops.s) / [`.hex`](test_bridge_ops.hex) / [`.bin`](test_bridge_ops.bin) | Programa dirigido da ponte `cpu_axi4_bridge` (**35 instruções / 140 bytes**): 3 `SW` consecutivos no mesmo endereço (`0x0100`, dois com dado idêntico), `SB`/`SH` com verificação de `WSTRB` e preservação de bytes vizinhos (`0x0104`, `0x0108`), e leituras `LB`/`LBU`/`LH`/`LHU`/`LW` gravadas em `0x0110..0x0124`. |

## Como Gerar ou Validar os Binários (a partir da raiz do projeto)

```bash
# Regenerar .s, .hex e .bin de todos os programas:
python -B sw/firmware/build_firmware.py

# Conferir sincronia sem sobrescrever arquivos:
python -B sw/firmware/build_firmware.py --check
```

## Contrato de Memória e Caixa de Correio na DMEM

Para permitir que o mesmo binário `matmul_dma.hex` execute qualquer dimensão válida (`1 <= M, K, N <= 32`) e bases configuráveis na DMEM (inclusive no futuro ambiente UVM), o sistema reserva os primeiros **512 bytes** da DMEM exclusivamente para uso do firmware:

| Faixa de Endereços na DMEM | Uso Exclusivo do Firmware |
| :---: | :--- |
| `0x0000_0000 .. 0x0000_001F` | Descritor de entrada (8 palavras, preenchido pelo testbench/ambiente antes da liberação de `rst_n`). |
| `0x0000_0020 .. 0x0000_003F` | Telemetria de saída (8 palavras, gravada pela CPU antes de `EBREAK`). |
| `0x0000_0100 .. 0x0000_01FC` | Área privada de trabalho concorrente da CPU (64 palavras de 32 bits). |
| `0x0000_0200 .. 0x0000_3FFC` | Área livre para as matrizes `A`, `B` e `C` (padrões `0x1000`, `0x2000`, `0x3000`). |

> **Regra de sistema vs. DMA genérico:** a reserva de `0x0000_0000..0x0000_01FF` é um contrato do firmware `matmul_dma` (o controlador DMA isolado da Entrega III continua genérico). Antes de configurar ou disparar o DMA, o firmware valida que `DESC_ADDR_A`, `DESC_ADDR_B` e `DESC_ADDR_C >= 0x0000_0200` (comparação sem sinal). Combinada à validação de limites da DMEM pelo hardware do DMA (`base + tamanho <= 0x4000`), nenhuma matriz pode intersectar a área reservada.

### Descritor de Entrada (`0x0000_0000 .. 0x0000_001C`)

O testbench (ou futuro ambiente UVM) inicializa o descritor e as matrizes `A`/`B` na DMEM enquanto `rst_n = 0`, antes de liberar a execução da CPU:

| Endereço | Campo | Regra e Descrição |
| :---: | :--- | :--- |
| `0x0000_0000` | `DESC_ADDR_A` | Endereço-base da matriz `A` (`>= 0x0200`, alinhado em 4 bytes; padrão `0x1000`). |
| `0x0000_0004` | `DESC_ADDR_B` | Endereço-base da matriz `B` (`>= 0x0200`, alinhado em 4 bytes; padrão `0x2000`). |
| `0x0000_0008` | `DESC_ADDR_C` | Endereço-base da matriz `C` (`>= 0x0200`, alinhado em 4 bytes; padrão `0x3000`). |
| `0x0000_000C` | `DESC_DIM_M` | Número de linhas de `A` e `C` (`1..32`, validado pelo DMA). |
| `0x0000_0010` | `DESC_DIM_K` | Dimensão interna (`1..32`, validado pelo DMA). |
| `0x0000_0014` | `DESC_DIM_N` | Número de colunas de `B` e `C` (`1..32`, validado pelo DMA). |
| `0x0000_0018` | `DESC_CTRL` | Comando de disparo aceito pelo firmware: `0` (mapeado para padrão `0x3 = START \| IRQ_EN`), `1` (`START` sem `IRQ_EN`) ou `3` (`START \| IRQ_EN`). Qualquer outro valor (ex.: `2` sem `START` ou `4` `SOFT_RST`) é rejeitado antes do disparo. |
| `0x0000_001C` | `DESC_CPU_WORK` | Número **máximo** de escritas concorrentes que a CPU realiza na área privada `0x0100..0x01FC` durante o *polling* (`0..64`). Caso o DMA conclua antes, o firmware encerra o *polling* com menos escritas do que o máximo solicitado (`OUT_CPU_WORK_DONE <= DESC_CPU_WORK`). Valores `> 64` são rejeitados antes do disparo. |

### Telemetria de Saída (`0x0000_0020 .. 0x0000_003C`) e Política de Timeout

O firmware acompanha a conclusão do DMA via **polling** em `DMA_STATUS` com um limite de iterações em software (`MAX_POLL_ITERS = 200.000`). O sinal `irq_done` do DMA é exposto na porta externa do Top-Level `riscv_dma_soc_top`, sem rotina de atendimento de interrupção no núcleo `rv32i_core`. Antes de encerrar via `EBREAK`, a CPU grava:

| Endereço | Campo | Descrição |
| :---: | :--- | :--- |
| `0x0000_0020` | `OUT_MAGIC` | `0x444F4E45` (`"DONE"`): DMA concluiu sem erro;<br>`0x45525221` (`"ERR!"`): hardware do DMA sinalizou `ERR=1`;<br>`0x46574552` (`"FWER"`): erro detectado pelo firmware (rejeição prévia ou timeout de *polling*). |
| `0x0000_0024` | `OUT_DMA_STATUS` | Valor lido de `DMA_STATUS` em `"DONE"`/`"ERR!"`, ou código de causa em `"FWER"`:<br>**Rejeições prévias (antes de `START`, sem disparar o DMA):**<br>• `0x80000001`: `DESC_ADDR_A`, `B` ou `C` na área reservada (`< 0x0200`);<br>• `0x80000002`: `DESC_CPU_WORK > 64`;<br>• `0x80000003`: `DESC_CTRL` fora de `{0, 1, 3}`.<br>**Timeout posterior ao `START` (durante o *polling*):**<br>• `0x80000004`: limite de iterações de *polling* (`200.000`) excedido. |
| `0x0000_0028` | `OUT_TOTAL_CYCLES` | Valor lido de `PERF_TOTAL_CYCLES` (`0x4000_0020`; `0` em `"FWER"`). |
| `0x0000_002C` | `OUT_COMP_CYCLES` | Valor lido de `PERF_COMP_CYCLES` (`0x4000_0024`; `0` em `"FWER"`). |
| `0x0000_0030` | `OUT_MEM_READS` | Valor lido de `PERF_MEM_READS` (`0x4000_0028`; `0` em `"FWER"`). |
| `0x0000_0034` | `OUT_HW_ARRAY_SIZE` | Valor lido de `HW_ARRAY_SIZE` (`0x4000_002C`: `2`, `4` ou `8`). |
| `0x0000_0038` | `OUT_POLL_ITERS` | Iterações do laço de *polling* executadas pela CPU (`0` nas rejeições prévias `0x80000001..03`; igual a `MAX_POLL_ITERS` no timeout `0x80000004`). |
| `0x0000_003C` | `OUT_CPU_WORK_DONE` | Quantidade efetiva de escritas concorrentes concluídas pela CPU em `0x0100..0x01FC`. |

> **Política de timeout (`0x80000004`):** trata-se de um limite de iterações de *polling* em software (não de um *watchdog* independente de hardware — caso a CPU fique bloqueada aguardando resposta AXI4, o contador de software também deixa de avançar). Ao atingir o limite após o `START`, o firmware registra `OUT_MAGIC = "FWER"` e `OUT_DMA_STATUS = 0x80000004` e encerra a CPU via `EBREAK`, **sem emitir `SOFT_RST`**. Portanto, o DMA pode permanecer ativo (`DMA_STATUS.BUSY = 1`), sendo obrigatório aplicar **reset global (`rst_n = 0`)** antes de reutilizar as regiões das matrizes ou iniciar uma nova execução. Cancelamento automático via `SOFT_RST` permanece como melhoria futura fora do escopo dos itens i e ii.

## Garantia de Autonomia do DMA

A CPU realiza **exclusivamente** acessos MMIO (`0x4000_0000..0x4000_002C`) e acessos à sua região reservada (`0x0000_0000..0x0000_01FF`). Toda a leitura de `A` e `B`, o *tiling* na malha sistólica e a escrita dos resultados em `C` são executados integralmente pelo controlador DMA sem intervenção da CPU.
