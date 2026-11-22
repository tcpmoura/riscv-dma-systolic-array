#!/usr/bin/env python3
# -*- coding: utf-8 -*-
# =============================================================================
# Projeto     : RISC-V RV32I com Controlador DMA e Coprocessador Matricial
# Etapa       : Entrega Parcial III — DMA, Barramento AXI4 e Memória Compartilhada
# Arquivo     : generate_32x32_vectors.py
# Professores : Ana Julia Fernandes de Oliveira Barros
#               Igor Dantas dos Santos Miranda
#               João Alberto Castelo Branco Oliveira
# Grupo 4     : João Vitor Carvalho Lima
#               Matheus Artur Macedo Bomfim
#               Nathalia Miranda
#               Tatiana Caetano Praxedes de Moura
# Descrição   : Gera e verifica os vetores .hex 32x32x32 com dados int8 variados
#               (positivos, negativos e zeros, seed=2026) usando o Golden Model
#               oficial (golden_model.py).
#
# Origem e adaptação do cenário:
#   - Algoritmo de geração (random.Random(2026), A[32x32] seguida de B[32x32] em
#     [-128, 127], row-major) originado na contribuição de Matheus Artur Macedo
#     Bomfim (Entrega Parcial III — Rev3).
#   - Conversão explícita para o formato oficial da base AXI4: cada elemento int8
#     de A e B é armazenado no byte inferior [7:0] de uma palavra de 32 bits
#     (8 dígitos hex, ex.: 000000BC para -68), preservando integralmente o valor
#     em complemento de dois de 8 bits; cada elemento int32 de C é armazenado em
#     32 bits (8 dígitos hex em complemento de dois).
#   - Validação cruzada: matriz C gerada e conferida pelo golden_model.py oficial
#     (referência algébrica e simulação por tiles para ARRAY_SIZE = 2, 4 e 8).
# =============================================================================

import argparse
from pathlib import Path
import random
import sys
from typing import List, Tuple

if hasattr(sys.stdout, "reconfigure"):
    sys.stdout.reconfigure(encoding="utf-8")

# Garante importação local do Golden Model oficial independentemente do cwd
SCRIPT_DIR = Path(__file__).resolve().parent
sys.path.insert(0, str(SCRIPT_DIR))

from golden_model import (
    export_test_vectors_hex,
    matmul_reference_alg,
    matmul_tiled_golden,
    to_hex32_word,
)

DEFAULT_DIM = 32
DEFAULT_SEED = 2026
DEFAULT_PREFIX = "mat_32x32_varied"
DEFAULT_OUT_DIR = SCRIPT_DIR / "test_vectors"


def generate_varied_32x32_matrices(
    m: int = DEFAULT_DIM,
    k: int = DEFAULT_DIM,
    n: int = DEFAULT_DIM,
    seed: int = DEFAULT_SEED,
) -> Tuple[List[List[int]], List[List[int]]]:
    """
    Gera A[m x k] e B[k x n] com um único PRNG random.Random(seed) sequencial
    (primeiro A row-major, depois B row-major, valores int8 em [-128, 127]),
    preservando exatamente a sequência determinística de seed=2026.
    """
    rng = random.Random(seed)
    A = [[rng.randint(-128, 127) for _ in range(k)] for _ in range(m)]
    B = [[rng.randint(-128, 127) for _ in range(n)] for _ in range(k)]
    return A, B


def decode_word_low_byte_int8(hex_word: str) -> int:
    """Decodifica o byte inferior [7:0] de uma palavra hex de 32 bits como int8 com sinal."""
    word = int(hex_word.strip(), 16)
    byte_val = word & 0xFF
    return byte_val - 256 if byte_val >= 128 else byte_val


def verify_exported_vectors(
    out_dir: Path,
    A: List[List[int]],
    B: List[List[int]],
    C_ref: List[List[int]],
    prefix: str = DEFAULT_PREFIX,
) -> None:
    """Confere linha a linha os .hex exportados e a preservação do sinal int8/int32."""
    expected_a = [to_hex32_word(val, is_int8_input=True) for row in A for val in row]
    expected_b = [to_hex32_word(val, is_int8_input=True) for row in B for val in row]
    expected_c = [to_hex32_word(val, is_int8_input=False) for row in C_ref for val in row]

    files = {
        "A": (out_dir / f"{prefix}_A.hex", expected_a, [v for r in A for v in r]),
        "B": (out_dir / f"{prefix}_B.hex", expected_b, [v for r in B for v in r]),
        "C": (out_dir / f"{prefix}_C_expected.hex", expected_c, [v for r in C_ref for v in r]),
    }

    for label, (path, exp_lines, raw_vals) in files.items():
        if not path.is_file():
            raise FileNotFoundError(f"Arquivo de vetor ausente: {path}")
        got_lines = [line.strip().upper() for line in path.read_text(encoding="utf-8").splitlines() if line.strip()]
        if len(got_lines) != len(exp_lines):
            raise AssertionError(
                f"{path.name}: quantidade de linhas={len(got_lines)} != esperado={len(exp_lines)}"
            )
        for idx, (got, exp) in enumerate(zip(got_lines, exp_lines)):
            if got != exp:
                raise AssertionError(
                    f"{path.name}: divergência no índice {idx}: obtido={got}, esperado={exp}"
                )
            if label in ("A", "B"):
                if len(got) != 8 or not got.startswith("000000"):
                    raise AssertionError(f"{path.name}[{idx}]: formato inválido ({got}), esperado 000000XX")
                if decode_word_low_byte_int8(got) != raw_vals[idx]:
                    raise AssertionError(f"{path.name}[{idx}]: sinal int8 não preservado!")


def main() -> int:
    parser = argparse.ArgumentParser(
        description="Gera ou valida os vetores .hex 32x32 variados (seed=2026) no padrão 32-bit."
    )
    parser.add_argument("--out-dir", type=Path, default=DEFAULT_OUT_DIR, help="Diretório de saída dos .hex")
    parser.add_argument("--prefix", type=str, default=DEFAULT_PREFIX, help="Prefixo dos arquivos .hex")
    parser.add_argument("--seed", type=int, default=DEFAULT_SEED, help="Semente PRNG (padrão: 2026)")
    parser.add_argument(
        "--check",
        action="store_true",
        help="Não sobrescreve arquivos; apenas valida reprodutibilidade e identidade dos .hex existentes.",
    )
    args = parser.parse_args()

    A, B = generate_varied_32x32_matrices(DEFAULT_DIM, DEFAULT_DIM, DEFAULT_DIM, seed=args.seed)
    C_ref = matmul_reference_alg(A, B)

    # Validação no modelo ciclo-a-ciclo por tiles nas 3 malhas suportadas (2x2, 4x4 e 8x8)
    for array_size in (2, 4, 8):
        res = matmul_tiled_golden(A, B, array_size=array_size)
        if res.C != C_ref:
            raise AssertionError(f"Divergência no Golden Model oficial para ARRAY_SIZE={array_size}")

    if not args.check:
        export_test_vectors_hex(args.out_dir, A, B, C_ref, prefix=args.prefix)

    verify_exported_vectors(args.out_dir, A, B, C_ref, prefix=args.prefix)

    mode = "VERIFICADOS" if args.check else "GERADOS E VERIFICADOS"
    print(
        f"[PASS] Vetores 32x32 variados (seed={args.seed}) {mode} em {args.out_dir}: "
        f"{args.prefix}_A.hex, {args.prefix}_B.hex, {args.prefix}_C_expected.hex "
        f"(1024 palavras cada; Golden Model 2x2/4x4/8x8 OK)"
    )
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
