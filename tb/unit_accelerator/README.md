# Verificação Unitária do Acelerador Matricial (Entrega Parcial II)

Este diretório contém o testbench unitário em SystemVerilog ([`tb_systolic_accelerator.sv`](tb_systolic_accelerator.sv)) e os logs de simulação validados no **Cadence Xcelium 24.09** ([`sim_xcelium.log`](sim_xcelium.log)) e no **Icarus Verilog 12.0** ([`sim_iverilog.log`](sim_iverilog.log)), verificando os três módulos RTL do acelerador localizados em [`rtl/accelerator/`](../../rtl/accelerator/):

1. [`rtl/accelerator/mac_pe.v`](../../rtl/accelerator/mac_pe.v) — Unidade MAC (*Processing Element* `Output-Stationary`, entradas `int8` com sinal, acumulador `int32` com sinal). Esquema RTL em [`Datapath_MAC_PE.png`](../../docs/Datapath_MAC_PE.png) ([PDF](../../docs/Datapath_MAC_PE.pdf)).
2. [`rtl/accelerator/systolic_array.v`](../../rtl/accelerator/systolic_array.v) — Malha sistólica bidimensional parametrizável (`ARRAY_SIZE` $\times$ `ARRAY_SIZE`).
3. [`rtl/accelerator/systolic_accelerator.v`](../../rtl/accelerator/systolic_accelerator.v) — Top-Level do acelerador (Buffers locais $A$ e $B$ de $T \times T$ bytes `int8`, gerador de *skew* em hardware nas bordas esquerda/superior, controle temporal de $3T - 2$ ciclos e mux combinacional de leitura de $C$). Esquema RTL em [`Datapath_Systolic_Accelerator.png`](../../docs/Datapath_Systolic_Accelerator.png) ([PDF](../../docs/Datapath_Systolic_Accelerator.pdf)).

---

## Como Executar a Simulação (a partir da raiz do projeto)

Os resultados incluídos nesta entrega foram executados e validados tanto no **Cadence Xcelium 24.09 (`xrun` / SimVision)** (log completo com `0 errors, 0 warnings` em [`sim_xcelium.log`](sim_xcelium.log) e captura real em [`../../docs/SimVision_Waveform_Teste1_2x2.png`](../../docs/SimVision_Waveform_Teste1_2x2.png)) quanto no **Icarus Verilog 12.0 (`-g2012`)** (log em [`sim_iverilog.log`](sim_iverilog.log)). Em caso de qualquer erro de verificação ou *timeout*, o testbench encerra via `$fatal(1, ...)` retornando código de saída diferente de zero (`1`).

### 1. Icarus Verilog (`iverilog -g2012`)
```powershell
iverilog -g2012 -o tb_acc.vvp rtl/accelerator/mac_pe.v rtl/accelerator/systolic_array.v rtl/accelerator/systolic_accelerator.v tb/unit_accelerator/tb_systolic_accelerator.sv
vvp tb_acc.vvp
```

### 2. Cadence Xcelium (`xrun`)
```bash
xrun -64bit -sv -clean \
    rtl/accelerator/mac_pe.v \
    rtl/accelerator/systolic_array.v \
    rtl/accelerator/systolic_accelerator.v \
    tb/unit_accelerator/tb_systolic_accelerator.sv \
    -top tb_systolic_accelerator \
    -access +rwc
```

---

## Cobertura dos Testes Unitários (`30/30 PASS`)

O testbench instancia as três configurações físicas (**`ARRAY_SIZE = 2`**, **`ARRAY_SIZE = 4`** e **`ARRAY_SIZE = 8`**) por meio do módulo parametrizável `accel_bfm_harness` (que emula as operações do DMA e valida estritamente se todos os estímulos pertencem à faixa `int8` de `-128` a `+127`) e executa **sequencialmente** a bateria de **10 subtestes** em cada instância (totalizando **30 subtestes**):

| Subteste | O que é verificado | Ciclos de malha ($2\times2$ / $4\times4$ / $8\times8$) | Critério de Aprovação (`PASS`) |
| :--- | :--- | :---: | :--- |
| **Teste 1** | Execução de 1 Tile único $T \times T$ e contrato temporal | `4` / `10` / `22` | `acc_busy=1` por exatos $3T-2$ ciclos, pulso de 1 ciclo em `acc_done` e matriz $C$ exata |
| **Teste 2** | Extremos `int8` com sinal (`-128`, `+127`, `-1`, `0`) | `32` / `10` / `22` | Extensão de sinal correta e acumulação `int32` sem overflow ($C_{0,0} = +65536$) |
| **Teste 3** | Acumulação parcial ao longo de $K$ sem `acc_clear` e limpeza explícita | `12` / `30` / `66` | 3 execuções seguidas preservam e somam os acumuladores; pulso `acc_clear` zera todos os PEs |
| **Teste 4** | Multiplicação $4\times4$ e $8\times8$ com *Tiling* usando os `.hex` do Golden Model | `256` / `80` / `22` (no $8\times8$) | Comparação exata contra `mat_4x4_C_expected.hex` e `mat_8x8x8_C_expected.hex` |
| **Teste 5a** | Matriz retangular $3\times5 \times 5\times2$ com *Zero-Padding* | `24` / `20` / `22` | Resultado $C$ exato e ciclos de malha idênticos ao Teste 9 do Golden Model |
| **Teste 5b** | Matriz não-múltipla $5\times7 \times 7\times6$ com *Zero-Padding* | `144` / `80` / `22` | Resultado $C$ exato nas 3 malhas com estímulos estritamente em `[-128..127]` |
| **Teste 6** | Proteção de buffers ($A$ e $B$) e comandos (`acc_start`, `acc_clear`) em `acc_busy=1` | `8` / `20` / `44` (2 runs) | Tenta sobrescrever posições de $A$ e $B$ ainda não consumidas pelo *skew*, pulsa `acc_start`/`acc_clear` durante `acc_busy=1` e reexecuta sem recarregar buffers |
| **Teste 7** | Reset assíncrono (`rst_n=0`) durante processamento ativo (`acc_busy=1`) | `4` / `10` / `22` (pós-reset) | Cancela imediatamente `acc_busy`/`acc_done`, verifica simultaneamente `acc_flat === '0'` em todos os PEs antes da borda de subida, comprova limpeza independente de $A$ e $B$ e recupera operação |
| **Teste 8a** | Casos pequenos e assimétricos ($1\times1$, $1\times7 \times 7\times1$ e $6\times3 \times 3\times7$) | `96` / `40` / `22` (no $6\times3\times7$) | Resultado exato para matriz unitária, produto interno de vetores e matrizes retangulares |
| **Teste 8b** | Limite máximo do projeto ($32\times32 \times 32\times32$ com todos os operandos `-128`) | `16384` / `5120` / `1408` | Todos os $1024$ elementos de $C$ iguais a $+524.288$ sem overflow |

---

## Sinais Recomendados para Inspeção de Formas de Onda (SimVision / GTKWave)

Para cada instância (`u_harness_2x2.u_dut`, `u_harness_4x4.u_dut`, `u_harness_8x8.u_dut`):
- **Clock e Reset:** `clk`, `rst_n`
- **Controle e Status DMA $\leftrightarrow$ Acelerador:** `acc_clear`, `acc_start`, `acc_busy`, `acc_done`, `cycle_cnt`
- **Escrita nos Buffers Locais:** `buf_a_we`, `buf_a_row`, `buf_a_col`, `buf_a_wdata`, `buf_b_we`, `buf_b_row`, `buf_b_col`, `buf_b_wdata`
- **Injeção com Skew na Borda da Malha:** `a_edge_flat`, `b_edge_flat`
- **Leitura Combinacional dos Acumuladores:** `buf_c_row`, `buf_c_col`, `buf_c_rdata`

Captura real da simulação no **Cadence SimVision** (`20,000 ps – 200,000 ps`, Teste 1 — malha $2\times2$):
- [SimVision_Waveform_Teste1_2x2.pdf](../../docs/SimVision_Waveform_Teste1_2x2.pdf)
- [SimVision_Waveform_Teste1_2x2.png](../../docs/SimVision_Waveform_Teste1_2x2.png)

Diagrama temporal ilustrativo (alinhado aos instantes do VCD):
- [Waveform_Acelerador_Entrega_2.pdf](../../docs/Waveform_Acelerador_Entrega_2.pdf)
- [Waveform_Acelerador_Entrega_2.png](../../docs/Waveform_Acelerador_Entrega_2.png)

