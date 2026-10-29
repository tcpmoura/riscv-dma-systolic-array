# Modelo de referência e simulação da arquitetura

`matmul_reference_alg(A, B)` é a referência funcional principal para conferir C
no futuro RTL/UVM. Não depende de tiling, skew ou tamanho da malha. A soma usa
precisão ilimitada e a saída é convertida para int32.

`matmul_tiled_golden(A, B, array_size)` simula a arquitetura proposta, usando
`systolic_tile_cycle_accurate` para cada par de blocos. Os caminhos compartilham
validação e conversões; os testes também usam respostas fixas independentes.

## Regras aprovadas

- A é M x K e B é K x N, com M, K e N inteiros entre 1 e 32.
- Matrizes são listas/tuplas retangulares não vazias de inteiros Python.
- A/B aceitam apenas valores entre -128 e 127. Por exemplo, 255 é inválido;
  para representar -1, fornecer -1. Floats, strings e bool são rejeitados.
- Formato ou valor inválido gera ValueError; tipo inválido gera TypeError.
- O modelo completo aceita ARRAY_SIZE igual a 2, 4 ou 8. A função unitária
  de tile permite outros tamanhos positivos para testes didáticos (1 e 3).
- Tiles são quadrados do mesmo tamanho e também exigem int8 válido.
- acc_init, quando fornecido ao teste unitário, deve ter o tamanho do tile
  e valores int32. Permite testar overflow artificialmente.
- O resultado C usa int32 sem saturação. Com K <= 32 e entradas válidas,
  |C[i][j]| <= 524288, portanto não ocorre overflow na multiplicação normal.
- to_int8 e to_int32 continuam disponíveis como conversões modulares para
  testes de representação. Não substituem a validação das matrizes.

## Configuração de memória

`validate_memory_layout(M, K, N, base_a=0x1000, base_b=0x2000, base_c=0x3000)`
verifica separadamente as dimensões e regiões da DMEM de 16 KB. Bases devem ser
inteiras, não negativas, alinhadas em 4 bytes; as três regiões devem caber na
DMEM e não podem se sobrepor. Regiões adjacentes são válidas.

Os intervalos são [base_A, base_A+4*M*K), [base_B, base_B+4*K*N) e
[base_C, base_C+4*M*N). As bases padrão acomodam matrizes 32x32.
A função não simula MMIO nem sinais de status. No futuro RTL, uma configuração
inválida deve resultar em ERR=1 e BUSY=0, sem iniciar transferências.
As funções de multiplicação não recebem endereços; portanto, seu chamador deve
invocar esse validador quando estiver verificando uma configuração do SoC.

## Limites das métricas

Os ciclos representam o cálculo ideal da malha (3*T-2 por produto de tiles),
sem carga, controle, limpeza, arbitragem, espera de memória ou protocolo AXI4.
Não há previsão de PERF_TOTAL_CYCLES, frequência, área ou energia.

Leituras contam palavras válidas de A/B na ordem de tiling modelada, com zeros
gerados localmente e um elemento por palavra. Escritas contam uma palavra por
elemento de C, sem resultados parciais na DMEM. Só coincidirão com o RTL se
este seguir essas regras. A referência escalar 2*M*N*K assume ausência de reuso;
não mede o processador existente.

## Execução na raiz do projeto

```powershell
python -B .\sw\golden_model\test_golden_model.py
python -B .\sw\golden_model\golden_model.py -m 8 -k 8 -n 8
```

São 11 grupos de testes, incluindo resultados fixos, limites 1..32, int8 estrito,
overflow unitário, métricas, memória e conteúdo hexadecimal. Não usar -O, pois
desativa assertions. A exportação é testada em pasta temporária, preservando
os vetores do projeto.

Para gerar vetores explicitamente:

```powershell
python -B .\sw\golden_model\golden_model.py -m 8 -k 8 -n 8 --export-dir sw/golden_model/test_vectors
```

Exportação em row-major, uma palavra de 32 bits por elemento. A/B ocupam o byte
inferior (-1 vira 000000FF); C ocupa 32 bits. A função valida dimensões e faixas
int8/int32, mas não recalcula a matriz C fornecida pelo chamador. A CLI usa a
referência matemática direta para gerar C.
