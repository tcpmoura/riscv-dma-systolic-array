#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Suíte de Testes Automatizados do Modelo de Referência (Golden Model)
Valida:
1. Aritmética int8 com sinal e acumulação int32 nos limites extremos (-128, +127).
2. Execução ciclo a ciclo de 1 Tile (Output-Stationary + Skew: t = i + j + k).
3. Particionamento em blocos (Tiling) para matrizes maiores que ARRAY_SIZE (2x2, 4x4, 8x8).
4. Matrizes menores que ARRAY_SIZE e dimensões retangulares/não-múltiplas (Zero-Padding).
5. Exportação de vetores .hex para uso nos testbenches Verilog/SystemVerilog e UVM.
"""

from pathlib import Path
import sys
from tempfile import TemporaryDirectory
from copy import deepcopy

if hasattr(sys.stdout, "reconfigure"):
    sys.stdout.reconfigure(encoding="utf-8")

# Garante importação local
sys.path.insert(0, str(Path(__file__).resolve().parent))

from golden_model import (
    validate_memory_layout,
    to_int8,
    to_int32,
    matmul_reference_alg,
    systolic_tile_cycle_accurate,
    matmul_tiled_golden,
    generate_random_matrix,
    export_test_vectors_hex,
)


def test_1_cycle_accurate_skew_wave():
    """
    Valida que um bloco 3x3 leva exatamente 3*3 - 2 = 7 ciclos e que a onda diagonal
    ativa 1, 3, 6, 7, 6, 3, 1 PEs por ciclo (exatamente como no Slide 8 da aula!).
    """
    A = [
        [1, 2, 3],
        [4, 5, 6],
        [7, 8, 9],
    ]
    B = [
        [9, 8, 7],
        [6, 5, 4],
        [3, 2, 1],
    ]
    C_expected = matmul_reference_alg(A, B)
    C_systolic, cycles, trace = systolic_tile_cycle_accurate(A, B, record_trace=True)

    assert cycles == 7, f"Esperado 7 ciclos para 3x3, obteve {cycles}"
    active_wave = [step["active_pes"] for step in trace]
    assert active_wave == [1, 3, 6, 7, 6, 3, 1], f"Onda de PEs ativos incorreta: {active_wave}"
    assert C_systolic == C_expected, f"Resultado 3x3 incorreto: {C_systolic} != {C_expected}"
    print("[PASS] Teste 1: Execução ciclo a ciclo + Onda diagonal de Skew (1,3,6,7,6,3,1 PEs ativos em 7 ciclos)")


def test_2_signed_int8_extremes():
    """
    Valida operandos negativos e extremos de int8 (-128 e +127) acumulando em int32 sem overflow.
    """
    A = [
        [-128, -128, -128, -128],
        [ 127, -128,  127, -128],
        [  -1,   -1,   -1,   -1],
        [   0,  100, -100,   50],
    ]
    B = [
        [-128,  127,   -1,    2],
        [-128, -128,    1,   -2],
        [-128,  127,   -1,    2],
        [-128, -128,    1,   -2],
    ]
    C_ref = matmul_reference_alg(A, B)
    for array_size in (2, 4, 8):
        res = matmul_tiled_golden(A, B, array_size=array_size)
        assert res.C == C_ref, f"Falha nos extremos int8 para malha {array_size}x{array_size}"
    # (-128)*(-128)*4 = 16384 * 4 = +65536 (que caberia apenas em >16 bits signed, provando a necessidade do int32!)
    assert C_ref[0][0] == 65536, f"Esperado +65536 no acumulador int32, obteve {C_ref[0][0]}"
    print("[PASS] Teste 2: Limites extremos int8 (-128, +127) com acumulador int32 (ex.: 4*(-128)*(-128) = +65536)")


def test_3_tiling_multiple_sizes():
    """
    Valida matrizes quadradas maiores que a malha (8x8 e 16x16) rodando em 2x2, 4x4 e 8x8.
    """
    for dim in (2, 4, 8, 16):
        A = generate_random_matrix(dim, dim, seed=100 + dim)
        B = generate_random_matrix(dim, dim, seed=200 + dim)
        C_ref = matmul_reference_alg(A, B)
        for array_size in (2, 4, 8):
            res = matmul_tiled_golden(A, B, array_size=array_size)
            assert res.C == C_ref, f"Falha em matriz {dim}x{dim} na malha {array_size}x{array_size}"
    print("[PASS] Teste 3: Tiling exato de matrizes 2x2, 4x4, 8x8 e 16x16 nas malhas 2x2, 4x4 e 8x8")


def test_4_non_multiple_and_rectangular_zero_padding():
    """
    Valida matrizes menores que a malha (ex.: 3x3 em 4x4/8x8) e retangulares não-múltiplas
    (ex.: 5x7 * 7x6 e 9x5 * 5x11) usando preenchimento automático com zeros (zero-padding).
    """
    cases = [
        (1, 1, 1),
        (3, 3, 3),   # Menor que 4x4 e 8x8
        (3, 4, 2),   # Retangular pequena (igual ao exemplo da aula!)
        (5, 7, 6),   # Não-múltipla de 2, 4 e 8
        (9, 5, 11),  # Maior em M e N, menor em K para 8x8
    ]
    for (M, K, N) in cases:
        A = generate_random_matrix(M, K, seed=M * 10 + K)
        B = generate_random_matrix(K, N, seed=K * 10 + N)
        C_ref = matmul_reference_alg(A, B)
        for array_size in (2, 4, 8):
            res = matmul_tiled_golden(A, B, array_size=array_size)
            assert res.C == C_ref, (
                f"Falha no caso retangular/não-múltiplo ({M}x{K})*({K}x{N}) em malha {array_size}x{array_size}"
            )
    print("[PASS] Teste 4: Matrizes menores que a malha e retangulares/não-múltiplas com Zero-Padding")


def test_5_export_hex_vectors():
    """Compara cada palavra, sinal e ordem row-major com valores fixos."""
    A = [[-128, -1, 127], [0, 2, -3]]
    B = [[1, -1], [2, 3], [-4, 5]]
    C = [[-638, 760], [16, -9]]
    expected = {
        "A": ["00000080", "000000FF", "0000007F", "00000000", "00000002", "000000FD"],
        "B": ["00000001", "000000FF", "00000002", "00000003", "000000FC", "00000005"],
        "C": ["FFFFFD82", "000002F8", "00000010", "FFFFFFF7"],
    }
    with TemporaryDirectory(prefix="golden_test_") as directory:
        files = export_test_vectors_hex(Path(directory), A, B, C)
        for key, path in files.items():
            assert path.read_text(encoding="utf-8").splitlines() == expected[key], key
    print("[PASS] Teste 5: Conteudo completo dos HEX, sem alterar vetores do projeto")


def test_6_known_answers_and_input_preservation():
    # Valores esperados fixos, sem chamar a referencia para produzi-los.
    cases = [
        ([[1, 2], [3, 4]], [[5, 6], [7, 8]], [[19, 22], [43, 50]]),
        ([[-128, -1, 127], [0, 2, -3]], [[1, -1], [2, 3], [-4, 5]],
         [[-638, 760], [16, -9]]),
        ([[7, -8], [0, 127]], [[1, 0], [0, 1]], [[7, -8], [0, 127]]),
        ([[0, 0]], [[-128], [127]], [[0]]),
    ]
    for A, B, expected in cases:
        original = deepcopy((A, B))
        assert matmul_reference_alg(A, B) == expected
        for size in (2, 4, 8):
            assert matmul_tiled_golden(A, B, size).C == expected
        assert (A, B) == original
    print("[PASS] Teste 6: Respostas fixas, identidade, zero e preservacao das entradas")


def expect_error(kind, function, *args):
    try:
        function(*args)
    except kind as error:
        assert str(error)
    else:
        raise AssertionError(f"Esperado {kind.__name__} em {function.__name__}")


def test_7_invalid_inputs():
    bad_cases = [
        ([], [[1]], ValueError), ([[1]], [], ValueError),
        ([[]], [[1]], ValueError), ([[1]], [[]], ValueError),
        ([[1], [2, 3]], [[1]], ValueError),
        ([[1, 2]], [[1], [2, 3]], ValueError),
        ([[1, 2]], [[1]], ValueError),
        ([[1.5]], [[1]], TypeError), ([[1]], [[True]], TypeError),
        ([["1"]], [[1]], TypeError), (None, [[1]], TypeError),
        ([1], [[1]], TypeError),
    ]
    for A, B, kind in bad_cases:
        expect_error(kind, matmul_reference_alg, A, B)
        expect_error(kind, matmul_tiled_golden, A, B, 2)
    for size in (0, -1, 1, 3, 16, 2.5, True, "2"):
        kind = ValueError if type(size) is int else TypeError
        expect_error(kind, matmul_tiled_golden, [[1]], [[1]], size)
    expect_error(ValueError, systolic_tile_cycle_accurate, [[1, 2]], [[1]])
    expect_error(ValueError, systolic_tile_cycle_accurate, [[1]], [[1]], [[0, 0]])
    expect_error(ValueError, generate_random_matrix, 0, 2, 42)
    with TemporaryDirectory() as directory:
        target = Path(directory) / "invalid"
        expect_error(ValueError, export_test_vectors_hex, target, [[1]], [[1]], [[1, 2]])
        assert not target.exists(), "Exportacao invalida criou arquivos"
    print("[PASS] Teste 7: Entradas invalidas rejeitadas com mensagens claras")


def test_8_modular_arithmetic_and_accumulation():
    assert to_int8(128) == -128 and to_int8(255) == -1
    assert to_int8(-129) == 127 and to_int8(256) == 0
    assert to_int32(2147483648) == -2147483648
    assert to_int32(-2147483649) == 2147483647
    initial = [[2147483647]]
    result, _, _ = systolic_tile_cycle_accurate([[1]], [[1]], initial)
    assert result == [[-2147483648]] and initial == [[2147483647]]
    result, _, _ = systolic_tile_cycle_accurate([[-1]], [[1]], [[-2147483648]])
    assert result == [[2147483647]]
    print("[PASS] Teste 8: Conversao modular e overflow int32 nos dois sentidos")


def test_9_ideal_metrics():
    # M=3, K=5, N=2: conta palavras validas, sem leituras do zero-padding.
    A, B = [[1] * 5 for _ in range(3)], [[1] * 2 for _ in range(5)]
    for size, cycles, reads in [(2, 24, 35), (4, 20, 25), (8, 22, 25)]:
        result = matmul_tiled_golden(A, B, size)
        assert result.C == [[5, 5]] * 3
        assert result.perf_comp_cycles == cycles
        assert result.perf_mem_reads == reads
        assert result.perf_mem_writes == 6
        assert result.scalar_mem_reads == 60
    print("[PASS] Teste 9: Ciclos ideais e palavras validas nas bordas")


def test_10_project_limits():
    A = [[-128] * 32 for _ in range(32)]
    B = [[-128] * 32 for _ in range(32)]
    expected = [[524288] * 32 for _ in range(32)]
    assert matmul_reference_alg(A, B) == expected
    for size in (2, 4, 8):
        assert matmul_tiled_golden(A, B, size).C == expected
    for A, B in [([[1]] * 33, [[1]]), ([[1] * 33], [[1]] * 33), ([[1]], [[1] * 33])]:
        expect_error(ValueError, matmul_reference_alg, A, B)
        expect_error(ValueError, matmul_tiled_golden, A, B, 2)
    for value in (-129, 128, 255):
        for A, B in [([[value]], [[1]]), ([[1]], [[value]])]:
            expect_error(ValueError, matmul_reference_alg, A, B)
            expect_error(ValueError, matmul_tiled_golden, A, B, 2)
            expect_error(ValueError, systolic_tile_cycle_accurate, A, B)
            with TemporaryDirectory() as directory:
                target = Path(directory) / "invalid"
                expect_error(ValueError, export_test_vectors_hex, target, A, B, [[0]])
                assert not target.exists()
    with TemporaryDirectory() as directory:
        expect_error(ValueError, export_test_vectors_hex, Path(directory), [[1]], [[1]], [[2**31]])
    expect_error(ValueError, generate_random_matrix, 33, 1, 0)
    print("[PASS] Teste 10: Limites 1..32, int8 estrito e pior caso sem overflow")


def test_11_memory_layout():
    validate_memory_layout(32, 32, 32)
    validate_memory_layout(1, 1, 1, 0, 4, 0x3FFC)
    validate_memory_layout(2, 2, 2, 32, 0, 16)  # Regioes adjacentes, fora de ordem.
    for bases in [(1, 8, 16), (-4, 8, 16), (0x4000, 8, 16),
                  (0, 0, 16), (0, 16, 16), (0, 16, 0)]:
        expect_error(ValueError, validate_memory_layout, 1, 1, 1, *bases)
    expect_error(ValueError, validate_memory_layout, 2, 2, 2, 0, 8, 32)
    expect_error(ValueError, validate_memory_layout, 2, 2, 2, 0, 16, 0x3FFC)
    expect_error(ValueError, validate_memory_layout, 33, 1, 1)
    expect_error(TypeError, validate_memory_layout, 1, 1, 1, True, 4, 8)
    print("[PASS] Teste 11: Alinhamento, limites da DMEM e sobreposicao")


def main():
    if not __debug__:
        raise RuntimeError("Execute sem -O: os testes dependem de assertions.")
    tests = [value for name, value in sorted(globals().items()) if name.startswith("test_") and callable(value)]
    tests.sort(key=lambda test: int(test.__name__.split("_")[1]))
    for test in tests:
        test()
    print(f"TODOS OS TESTES PASSARAM ({len(tests)}/{len(tests)} PASS)")


if __name__ == "__main__":
    main()
