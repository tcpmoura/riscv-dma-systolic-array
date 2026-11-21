#!/usr/bin/env python3
# -*- coding: utf-8 -*-
# =============================================================================
# Projeto     : RISC-V RV32I com Controlador DMA e Coprocessador Matricial
# Etapa       : Entrega Parcial I — Planejamento, Microarquitetura e Golden Model
# Arquivo     : golden_model.py
# Professores : Ana Julia Fernandes de Oliveira Barros
#               Igor Dantas dos Santos Miranda
#               João Alberto Castelo Branco Oliveira
# Grupo 4     : João Vitor Carvalho Lima
#               Matheus Artur Macedo Bomfim
#               Nathalia Miranda
#               Tatiana Caetano Praxedes de Moura
# Descrição   : Modelo de referência (Golden Model) em Python para o
#               coprocessador Systolic Array parametrizável (2x2, 4x4, 8x8)
#               e estratégia de particionamento em blocos (tiling) do DMA.
#               Fornece referência funcional bit-exata (int8 -> int32),
#               validação de layout de memória, métricas ideais da malha e
#               exportação de vetores .hex para os testbenches RTL e UVM.
# =============================================================================
"""
Modelo de Referência (Golden Model) — Coprocessador Systolic Array + DMA
Projeto Final: RISC-V RV32I + Controlador DMA + Systolic Array Parametrizável

Características implementadas (alinhadas com a especificação de hardware):
1. Formato numérico:
   - Entradas (Matrizes A e B): int8 com sinal (-128 a +127)
   - Acumulador dos PEs e Saída (Matriz C): int32 com sinal (-2^31 a +2^31 - 1)
2. Fluxo da Malha Sistólica:
   - Arquitetura Output-Stationary (C_ij permanece acumulando no PE(i,j))
   - Tamanho parametrizável por ARRAY_SIZE (ex.: 2x2, 4x4, 8x8)
   - Atraso escalonado (skew) na borda: linha i de A atrasada em i ciclos,
     coluna j de B atrasada em j ciclos -> PE(i,j) opera no ciclo t = i + j + k
3. Particionamento em Blocos (Tiling) e Bordas:
   - Controlado por laços de blocos (ti, tj, tk) equivalentes à FSM do DMA
   - Suporte a matrizes maiores que ARRAY_SIZE (tiling)
   - Suporte a matrizes menores ou com dimensões não-múltiplas de ARRAY_SIZE
     via preenchimento automático com zeros (zero-padding) nos buffers locais
4. Referência e métricas:
   - matmul_reference_alg é a referência funcional principal para RTL/UVM.
   - O modelo sistólico estuda a arquitetura proposta, sem temporização AXI4,
     arbitragem, espera de memória ou ciclos de controle do DMA.
   - Ciclos ideais da malha e contagem de palavras válidas, sem leituras extras
     nas bordas, com um elemento por palavra. Não prevê PERF_TOTAL_CYCLES,
     frequência, área ou energia. Não mede o desempenho da CPU real.
"""

from dataclasses import dataclass
from pathlib import Path
from typing import List, Tuple, Dict, Optional
import argparse
import random
import sys

# Constantes arquiteturais alinhadas à especificação de hardware
MAX_DIM = 32               # Dimensão máxima suportada para M, K e N (1..32)
ARRAY_SIZES = (2, 4, 8)    # Tamanhos físicos T suportados pela malha sistólica (T x T)
DMEM_BYTES = 0x4000        # Capacidade total da DMEM compartilhada (16 KB = 16384 bytes)

if hasattr(sys.stdout, "reconfigure"):
    sys.stdout.reconfigure(encoding="utf-8")


# =============================================================================
# 1. Funções de Aritmética de Ponto Fixo em Complemento de 2 (int8 / int32)
# =============================================================================

def to_int8(val: int) -> int:
    """Converte um inteiro para int8 com sinal em complemento de 2 (-128 a +127)."""
    val &= 0xFF
    return val - 0x100 if (val & 0x80) else val


def to_int32(val: int) -> int:
    """Converte um inteiro para int32 com sinal em complemento de 2 (-2^31 a 2^31-1)."""
    val &= 0xFFFFFFFF
    return val - 0x100000000 if (val & 0x80000000) else val


def to_hex32_word(val: int, is_int8_input: bool = False) -> str:
    """
    Formata um elemento para armazenamento Word-Aligned (32 bits) na DMEM:
    - Para A e B (int8): armazena o byte nos bits [7:0] (ex.: 0x000000FF para -1).
    - Para C (int32): armazena a palavra completa de 32 bits em complemento de 2.
    """
    if is_int8_input:
        return f"{(val & 0xFF):08X}"
    return f"{(val & 0xFFFFFFFF):08X}"


# =============================================================================
# 2. Validadores e Modelo Algébrico Direto (Referência Matemática de Software)
# =============================================================================

def validate_matrix(matrix, name: str) -> Tuple[int, int]:
    """Valida estrutura retangular e tipos inteiros (sem bool); retorna (linhas, colunas)."""
    if not isinstance(matrix, (list, tuple)):
        raise TypeError(f"{name} deve ser lista ou tupla de linhas.")
    if not matrix:
        raise ValueError(f"{name} nao pode ser vazia.")
    width = None  # Largura esperada de todas as linhas (extraída da primeira linha)
    for row in matrix:
        if not isinstance(row, (list, tuple)):
            raise TypeError(f"Cada linha de {name} deve ser lista ou tupla.")
        if not row:
            raise ValueError(f"{name} nao pode ter linhas vazias.")
        if width is None:
            width = len(row)
        if len(row) != width:
            raise ValueError(f"{name} deve ser retangular.")
        if any(type(value) is not int for value in row):
            raise TypeError(f"{name} deve conter inteiros, sem bool.")
    return len(matrix), width


def validate_positive_integer(value, name: str) -> None:
    """Garante que o parâmetro seja inteiro estrito (>= 1), rejeitando booleanos."""
    if type(value) is not int:
        raise TypeError(f"{name} deve ser inteiro, sem bool.")
    if value < 1:
        raise ValueError(f"{name} deve ser maior que zero.")


def validate_range(matrix, name, minimum, maximum):
    """Garante que todos os elementos da matriz estejam dentro do intervalo [minimum, maximum]."""
    for i, row in enumerate(matrix):
        for j, value in enumerate(row):
            if not minimum <= value <= maximum:
                raise ValueError(f"{name}[{i}][{j}] deve estar entre {minimum} e {maximum}.")


def validate_dimension(value, name):
    """Valida uma dimensão matricial contra os limites do projeto (1 <= dim <= MAX_DIM)."""
    validate_positive_integer(value, name)
    if value > MAX_DIM:
        raise ValueError(f"{name} deve estar entre 1 e {MAX_DIM}.")


def validate_memory_layout(M, K, N, base_a=0x1000, base_b=0x2000, base_c=0x3000):
    """Valida alinhamento, limites da DMEM (16 KB) e não-sobreposição das regiões A, B e C.

    Não simula registradores MMIO nem DMA; reflete exatamente as regras de validação
    de configuração adotadas no hardware para os intervalos [base, fim).
    """
    for name, value in (("M", M), ("K", K), ("N", N)):
        validate_dimension(value, name)
    intervals = []  # Lista de tuplas (base, fim_exclusivo, nome) para checar sobreposição
    for name, base, count in (("A", base_a, M*K), ("B", base_b, K*N), ("C", base_c, M*N)):
        if type(base) is not int:
            raise TypeError(f"Base de {name} deve ser inteira, sem bool.")
        end = base + count * 4  # Endereço final exclusivo (cada elemento ocupa 1 palavra de 4 bytes)
        if base < 0 or base % 4 or end > DMEM_BYTES:
            raise ValueError(f"Regiao de {name} desalinhada ou fora da DMEM.")
        intervals.append((base, end, name))
    intervals.sort()
    for previous, current in zip(intervals, intervals[1:]):
        if current[0] < previous[1]:
            raise ValueError(f"Regioes de {previous[2]} e {current[2]} se sobrepoem.")


def validate_operands(A, B) -> Tuple[int, int, int]:
    """Valida compatibilidade (MxK * KxN), limites (1..32) e faixa int8 (-128..+127) de A e B."""
    M, K = validate_matrix(A, "A")
    rows_b, N = validate_matrix(B, "B")
    if rows_b != K:
        raise ValueError(f"Dimensoes incompativeis: A={M}x{K}, B={rows_b}x{N}.")
    for name, value in (("M", M), ("K", K), ("N", N)):
        validate_dimension(value, name)
    validate_range(A, "A", -128, 127)
    validate_range(B, "B", -128, 127)
    return M, K, N


def matmul_reference_alg(A: List[List[int]], B: List[List[int]]) -> List[List[int]]:
    """Referência funcional principal (C = A x B), independente de tiling e skew.

    Soma em precisão ilimitada e converte o resultado final para int32 modular,
    equivalente à acumulação módulo 2**32 a cada MAC sem saturação.
    """
    M, K, N = validate_operands(A, B)

    C = [[0 for _ in range(N)] for _ in range(M)]  # Matriz de saída M x N (int32)
    for i in range(M):
        for j in range(N):
            acc = 0  # Acumulador do produto interno da posição C[i][j]
            for k in range(K):
                prod = to_int8(A[i][k]) * to_int8(B[k][j])
                acc += prod
            C[i][j] = to_int32(acc)
    return C


# =============================================================================
# 3. Modelo Ciclo a Ciclo de 1 Tile na Malha Sistólica (Output-Stationary)
# =============================================================================

def systolic_tile_cycle_accurate(
    tile_A: List[List[int]],
    tile_B: List[List[int]],
    acc_init: Optional[List[List[int]]] = None,
    record_trace: bool = False
) -> Tuple[List[List[int]], int, List[Dict]]:
    """
    Simula ciclo a ciclo a execução de um único bloco (tile) T x T na malha sistólica
    Output-Stationary com skew automático nas entradas:
      - Linha i de tile_A entra na borda esquerda com atraso de i ciclos (skew = i).
      - Coluna j de tile_B entra na borda superior com atraso de j ciclos (skew = j).
      - O elemento PE(i, j) realiza MAC do índice k exatamente no ciclo t = i + j + k.
      - Número total de ciclos para processar o tile T x T: (3*T - 2) ciclos.
    """
    T, cols_a = validate_matrix(tile_A, "tile_A")
    rows_b, cols_b = validate_matrix(tile_B, "tile_B")
    if cols_a != T or (rows_b, cols_b) != (T, T):
        raise ValueError("Os tiles devem ser quadrados do mesmo tamanho.")
    if acc_init is not None and validate_matrix(acc_init, "acc_init") != (T, T):
        raise ValueError("acc_init deve ter as mesmas dimensoes dos tiles.")

    validate_range(tile_A, "tile_A", -128, 127)
    validate_range(tile_B, "tile_B", -128, 127)
    if acc_init is not None:
        validate_range(acc_init, "acc_init", -(2**31), 2**31 - 1)

    # Acumuladores estacionários int32 de cada PE(i, j) (preservam somas parciais de tk anterior)
    acc = [
        [to_int32(acc_init[i][j]) if acc_init else 0 for j in range(T)]
        for i in range(T)
    ]
    # Registradores de passagem horizontal (a_reg) e vertical (b_reg) entre PEs vizinhos
    a_reg = [[0 for _ in range(T)] for _ in range(T)]
    b_reg = [[0 for _ in range(T)] for _ in range(T)]

    total_cycles = 3 * T - 2  # Duração fixa da onda sistólica para um tile T x T
    trace = []                # Histórico opcional por ciclo (PEs ativos e snapshot de acc)

    for t in range(total_cycles):
        # 1. Determina quais valores entram nas bordas esquerda (A) e superior (B) no ciclo t:
        #    na linha i, k_a = t - i entra se 0 <= k_a < T; caso contrário entra 0 (bolha de skew).
        a_in = [[0 for _ in range(T)] for _ in range(T)]  # Entradas combinacionais A de cada PE no ciclo t
        b_in = [[0 for _ in range(T)] for _ in range(T)]  # Entradas combinacionais B de cada PE no ciclo t

        for i in range(T):
            k_a = t - i
            a_in[i][0] = to_int8(tile_A[i][k_a]) if (0 <= k_a < T) else 0
            for j in range(1, T):
                a_in[i][j] = a_reg[i][j - 1]

        for j in range(T):
            k_b = t - j
            b_in[0][j] = to_int8(tile_B[k_b][j]) if (0 <= k_b < T) else 0
            for i in range(1, T):
                b_in[i][j] = b_reg[i - 1][j]

        # 2. Atualização síncrona de todos os T x T PEs em paralelo (borda de subida do clock)
        active_pes = 0  # Quantidade de PEs processando um par válido (a_ik, b_kj) neste ciclo
        for i in range(T):
            for j in range(T):
                # Verifica se o par (a_ik, b_kj) válido está cruzando o PE(i,j) neste ciclo t = i + j + k
                k = t - i - j
                if 0 <= k < T:
                    active_pes += 1
                prod = a_in[i][j] * b_in[i][j]
                acc[i][j] = to_int32(acc[i][j] + prod)
                a_reg[i][j] = a_in[i][j]
                b_reg[i][j] = b_in[i][j]

        if record_trace:
            trace.append({
                "cycle": t,
                "active_pes": active_pes,
                "acc_snapshot": [row[:] for row in acc]
            })

    return acc, total_cycles, trace


# =============================================================================
# 4. Simulação Arquitetural com Tiling (sem temporização DMA/AXI4)
# =============================================================================

@dataclass
class GoldenSimulationResult:
    """Agrupa a matriz resultante C e as métricas ideais da estratégia de tiling modelada."""
    C: List[List[int]]                  # Matriz de saída C (M x N, elementos int32)
    M: int                              # Linhas de A e C
    K: int                              # Dimensão interna compartilhada (colunas de A / linhas de B)
    N: int                              # Colunas de B e C
    array_size: int                     # Dimensão física T da malha simulada (2, 4 ou 8)
    num_tile_rows: int                  # Número de blocos em M: ceil(M / T)
    num_tile_cols: int                  # Número de blocos em N: ceil(N / T)
    num_tile_k: int                     # Número de sub-blocos em K: ceil(K / T)
    total_tile_ops: int                 # Total de execuções da malha (num_tile_rows * num_tile_cols * num_tile_k)
    perf_comp_cycles: int               # Ciclos ideais de cálculo na malha: total_tile_ops * (3*T - 2)
    perf_mem_reads: int                 # Total de palavras válidas lidas da DMEM (sem leituras de padding)
    perf_mem_writes: int                # Total de palavras válidas escritas em C (M * N)
    scalar_mem_reads: int               # Leituras em abordagem escalar sem reuso (2 * M * N * K)
    memory_read_reduction_factor: float # Fator de redução de leituras na RAM (scalar_mem_reads / perf_mem_reads)


def extract_tile_with_zero_padding(
    matrix: List[List[int]],
    row_start: int,
    col_start: int,
    array_size: int,
    max_rows: int,
    max_cols: int
) -> Tuple[List[List[int]], int]:
    """
    Simula a leitura de um sub-bloco (tile) T x T pelo DMA para o Buffer Local:
    - Se (r < max_rows) e (c < max_cols), lê o elemento real da memória principal.
    - Caso contrário (borda de matriz menor ou não-múltipla de ARRAY_SIZE),
      preenche o buffer local com 0 (zero-padding).
    Retorna o tile T x T e o número de leituras reais na memória.
    """
    tile = [[0 for _ in range(array_size)] for _ in range(array_size)]  # Buffer local T x T
    valid_reads = 0                                                     # Palavras reais lidas da DMEM
    for i in range(array_size):
        r = row_start + i
        for j in range(array_size):
            c = col_start + j
            if r < max_rows and c < max_cols:
                tile[i][j] = to_int8(matrix[r][c])
                valid_reads += 1
            else:
                tile[i][j] = 0
    return tile, valid_reads


def matmul_tiled_golden(
    A: List[List[int]],
    B: List[List[int]],
    array_size: int
) -> GoldenSimulationResult:
    """Simula cálculo e ordem dos blocos (ti, tj, tk), sem temporização DMA/memória/AXI4.

    Os contadores pressupõem zero-padding local e uma única escrita por elemento de C.
    Dimensões e operandos são validados aqui; endereços são verificados separadamente
    por validate_memory_layout, pois esta função recebe matrizes, não endereços.
    """
    validate_positive_integer(array_size, "ARRAY_SIZE")
    if array_size not in ARRAY_SIZES:
        raise ValueError("ARRAY_SIZE deve ser 2, 4 ou 8.")
    M, K, N = validate_operands(A, B)

    T = array_size
    # Número de blocos em cada dimensão: ceil(M/T), ceil(N/T), ceil(K/T)
    num_tile_rows = (M + T - 1) // T
    num_tile_cols = (N + T - 1) // T
    num_tile_k    = (K + T - 1) // T

    C = [[0 for _ in range(N)] for _ in range(M)]  # Matriz de saída reconstruída bloco a bloco

    perf_comp_cycles = 0  # Acumulador de ciclos de computação da malha (equivalente a PERF_COMP_CYCLES)
    perf_mem_reads   = 0  # Acumulador de leituras válidas na DMEM (equivalente a PERF_MEM_READS)
    perf_mem_writes  = 0  # Acumulador de escritas válidas de C na DMEM
    total_tile_ops   = 0  # Total de disparos de acc_start na malha sistólica

    # Laços externos de Tiling controlados pela FSM do DMA: (ti, tj, tk)
    for ti in range(num_tile_rows):
        row_base = ti * T  # Linha global inicial do bloco C_{ti, tj}
        for tj in range(num_tile_cols):
            col_base = tj * T  # Coluna global inicial do bloco C_{ti, tj}

            # Início de um novo bloco C_{ti, tj}: DMA aciona pulso acc_clear = 1
            acc_tile = [[0 for _ in range(T)] for _ in range(T)]

            # Laço interno de acumulação parcial ao longo da dimensão comum K
            for tk in range(num_tile_k):
                k_base = tk * T  # Deslocamento global ao longo de K

                # 1. DMA lê o bloco A_{ti, tk} e carrega no Buffer Local A
                tile_A, reads_A = extract_tile_with_zero_padding(
                    A, row_base, k_base, T, M, K
                )
                # 2. DMA lê o bloco B_{tk, tj} e carrega no Buffer Local B
                tile_B, reads_B = extract_tile_with_zero_padding(
                    B, k_base, col_base, T, K, N
                )
                perf_mem_reads += (reads_A + reads_B)

                # 3. DMA aciona acc_start = 1 -> Malha Sistólica acumula o produto do bloco
                acc_tile, cycles_tile, _ = systolic_tile_cycle_accurate(
                    tile_A, tile_B, acc_init=acc_tile, record_trace=False
                )
                perf_comp_cycles += cycles_tile
                total_tile_ops += 1

            # 4. Fim do somatório em tk: DMA lê os acumuladores e escreve C_{ti, tj} na memória
            for i in range(T):
                r = row_base + i
                for j in range(T):
                    c = col_base + j
                    if r < M and c < N:
                        C[r][c] = acc_tile[i][j]
                        perf_mem_writes += 1

    scalar_reads = 2 * M * N * K  # Referência escalar sem reuso de dados em buffers locais
    reduction = (scalar_reads / perf_mem_reads) if perf_mem_reads > 0 else 1.0

    return GoldenSimulationResult(
        C=C,
        M=M,
        K=K,
        N=N,
        array_size=T,
        num_tile_rows=num_tile_rows,
        num_tile_cols=num_tile_cols,
        num_tile_k=num_tile_k,
        total_tile_ops=total_tile_ops,
        perf_comp_cycles=perf_comp_cycles,
        perf_mem_reads=perf_mem_reads,
        perf_mem_writes=perf_mem_writes,
        scalar_mem_reads=scalar_reads,
        memory_read_reduction_factor=reduction
    )


# =============================================================================
# 5. Exportação de Arquivos Hexadecimais para Simulação RTL / UVM ($readmemh)
# =============================================================================

def export_test_vectors_hex(
    output_dir: Path,
    A: List[List[int]],
    B: List[List[int]],
    C: List[List[int]],
    prefix: str = "vec"
) -> Dict[str, Path]:
    """
    Exporta as matrizes A, B e C em arquivos .hex alinhados em palavras de 32 bits
    (Row-Major), prontos para inicialização da memória de dados e conferência no UVM.
    """
    M, _, N = validate_operands(A, B)
    if validate_matrix(C, "C") != (M, N):
        raise ValueError("C deve ter M linhas e N colunas.")
    validate_range(C, "C", -(2**31), 2**31 - 1)
    output_dir.mkdir(parents=True, exist_ok=True)
    files = {}  # Dicionário de retorno mapeando "A", "B" e "C" aos caminhos criados

    path_a = output_dir / f"{prefix}_A.hex"
    path_b = output_dir / f"{prefix}_B.hex"
    path_c = output_dir / f"{prefix}_C_expected.hex"

    with path_a.open("w", encoding="utf-8") as fa:
        for row in A:
            for val in row:
                fa.write(to_hex32_word(val, is_int8_input=True) + "\n")
    files["A"] = path_a

    with path_b.open("w", encoding="utf-8") as fb:
        for row in B:
            for val in row:
                fb.write(to_hex32_word(val, is_int8_input=True) + "\n")
    files["B"] = path_b

    with path_c.open("w", encoding="utf-8") as fc:
        for row in C:
            for val in row:
                fc.write(to_hex32_word(val, is_int8_input=False) + "\n")
    files["C"] = path_c

    return files


# =============================================================================
# 6. Interface de Linha de Comando (Demonstração e Comparativo 2x2, 4x4, 8x8)
# =============================================================================

def generate_random_matrix(rows: int, cols: int, seed: int) -> List[List[int]]:
    """Gera matriz determinística (rows x cols) com valores int8 (-128..+127) a partir de seed."""
    validate_dimension(rows, "rows")
    validate_dimension(cols, "cols")
    rng = random.Random(seed)
    return [[rng.randint(-128, 127) for _ in range(cols)] for _ in range(rows)]


def print_matrix(name: str, mat: List[List[int]], max_display: int = 8) -> None:
    """Imprime uma matriz formatada no terminal, truncando a exibição acima de max_display."""
    R = len(mat)                     # Quantidade de linhas da matriz
    C = len(mat[0]) if R > 0 else 0  # Quantidade de colunas da matriz
    print(f"\nMatriz {name} ({R}x{C}):")
    for i in range(min(R, max_display)):
        row_str = " ".join(f"{mat[i][j]:6d}" for j in range(min(C, max_display)))
        if C > max_display:
            row_str += " ..."
        print("  [" + row_str + " ]")
    if R > max_display:
        print("  ...")


def main():
    """Executa comparação CLI entre malhas 2x2, 4x4 e 8x8 e exporta vetores .hex opcionais."""
    parser = argparse.ArgumentParser(
        description="Golden Model — Multiplicação Matricial por Blocos (Tiling) em Systolic Array"
    )
    parser.add_argument("-m", type=int, default=8, help="Linhas de A e C (M)")
    parser.add_argument("-k", type=int, default=8, help="Colunas de A e linhas de B (K)")
    parser.add_argument("-n", type=int, default=8, help="Colunas de B e C (N)")
    parser.add_argument("--seed", type=int, default=42, help="Semente aleatória")
    parser.add_argument("--export-dir", type=str, default="", help="Diretório opcional para exportar .hex")
    args = parser.parse_args()
    if not all(1 <= d <= MAX_DIM for d in (args.m, args.k, args.n)):
        parser.error("M, K e N devem estar entre 1 e 32.")
    print("Metricas ideais: excluem temporizacao AXI4/DMA e latencia total do SoC.")

    A = generate_random_matrix(args.m, args.k, args.seed)
    B = generate_random_matrix(args.k, args.n, args.seed + 1)

    print("=" * 78)
    print(f"GOLDEN MODEL — MULTIPLICAÇÃO MATRICIAL COM TILING (M={args.m}, K={args.k}, N={args.n})")
    print("Formato: Entradas A, B em int8 [-128..127] | Acumulador e Saída C em int32")
    print("=" * 78)

    if args.m <= 8 and args.k <= 8 and args.n <= 8:
        print_matrix("A (int8)", A)
        print_matrix("B (int8)", B)

    C_ref = matmul_reference_alg(A, B)
    if args.m <= 8 and args.n <= 8:
        print_matrix("C Esperada = A x B (int32)", C_ref)

    print("\n" + "-" * 78)
    print("COMPARATIVO ENTRE CONFIGURAÇÕES FÍSICAS DO SYSTOLIC ARRAY (2x2, 4x4, 8x8):")
    print("-" * 78)
    header = (
        f"{'ARRAY_SIZE':^12} | {'Blocos (ti,tj,tk)':^18} | {'Tile Ops':^10} | "
        f"{'Ciclos Malha':^14} | {'Leituras RAM':^14} | {'Economia vs CPU':^15} | {'Check':^7}"
    )
    print(header)
    print("-" * 78)

    for size in (2, 4, 8):
        res = matmul_tiled_golden(A, B, array_size=size)
        match = (res.C == C_ref)
        status = "PASS" if match else "FAIL"
        grid_str = f"{res.num_tile_rows}x{res.num_tile_cols}x{res.num_tile_k}"
        print(
            f"{f'{size}x{size}':^12} | {grid_str:^18} | {res.total_tile_ops:^10d} | "
            f"{res.perf_comp_cycles:^14d} | {res.perf_mem_reads:^14d} | "
            f"{res.memory_read_reduction_factor:^14.2f}x | {status:^7}"
        )
        if not match:
            raise RuntimeError( f"Erro: resultado de {size}x{size} divergiu da referência!")

    print("-" * 78)
    print(f"Leituras sem reuso (processador escalar tradicional, 2*M*N*K): {2 * args.m * args.n * args.k}")

    if args.export_dir:
        out_path = Path(args.export_dir)
        files = export_test_vectors_hex(out_path, A, B, C_ref, prefix=f"mat_{args.m}x{args.k}x{args.n}")
        print(f"\nVetores .hex exportados em: {out_path.resolve()}")
        for k_name, p_file in files.items():
            print(f"  - {k_name}: {p_file.name}")


if __name__ == "__main__":
    main()
