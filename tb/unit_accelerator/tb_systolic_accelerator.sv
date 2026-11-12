// =============================================================================
// Testbench: tb_systolic_accelerator.sv
// Descrição: Testbench Unitário Autoverificável em SystemVerilog para o
//            Acelerador Matricial (Systolic Array Output-Stationary).
//
// Escopo de Verificação (Entrega Parcial II — Seção 8.b):
//   - Instancia as três configurações físicas do projeto: ARRAY_SIZE = 2 (2x2),
//     ARRAY_SIZE = 4 (4x4) e ARRAY_SIZE = 8 (8x8), executando sequencialmente
//     a bateria completa de testes em cada instância.
//   - Emula o comportamento do controlador de DMA (tasks BFM para carga de
//     buffers locais com zero-padding e validação estrita da faixa int8,
//     controle de acc_clear / acc_start, monitoramento ciclo a ciclo de
//     acc_busy / acc_done com timeout e leitura combinacional de C).
//   - Executa 10 subtestes em cada tamanho de malha (30 subtestes no total):
//       1. Cálculo de 1 Tile T x T e contrato temporal (3*T - 2 ciclos).
//       2. Extremos aritméticos int8 (-128, +127, -1, 0) e acumulador int32.
//       3. Acumulação parcial ao longo de K (sem acc_clear) e limpeza explícita.
//       4. Multiplicação 4x4 e 8x8 com Tiling usando os arquivos .hex do
//          Golden Model (mat_4x4_*.hex e mat_8x8x8_*.hex).
//       5a. Matriz retangular (3x5) * (5x2) com Zero-Padding e contagem de ciclos.
//       5b. Matriz não-múltipla (5x7) * (7x6) com Zero-Padding.
//       6. Proteção de buffers (A e B) e comandos (acc_clear, acc_start) durante
//          acc_busy = 1 (sobre posições ainda não consumidas + reexecução sem
//          recarga), prioridade de acc_clear e retenção de C.
//       7. Reset assíncrono (rst_n = 0) durante processamento ativo (acc_busy = 1),
//          verificando cancelamento imediato, limpeza de buffers/PEs e recuperação.
//       8a. Casos pequenos e assimétricos (1x1, 1x7 * 7x1 e 6x3 * 3x7).
//       8b. Caso limite máximo do projeto: 32x32 * 32x32 com todos os operandos
//           iguais a -128 (C[i][j] = +524288 em todos os 1024 elementos).
// =============================================================================
`timescale 1ns / 1ps

// -----------------------------------------------------------------------------
// Módulo Harness Parametrizável: encapsula 1 instância do DUT (T x T) e as
// tasks BFM que emulam o controlador de DMA para qualquer ARRAY_SIZE.
// -----------------------------------------------------------------------------
module accel_bfm_harness #(
    parameter int ARRAY_SIZE = 4,
    parameter int IDX_W      = $clog2(ARRAY_SIZE)
) (
    input logic clk
);

    logic                  rst_n;
    logic                  acc_clear;
    logic                  acc_start;
    logic                  acc_busy;
    logic                  acc_done;

    logic                  buf_a_we;
    logic [IDX_W-1:0]      buf_a_row;
    logic [IDX_W-1:0]      buf_a_col;
    logic signed [7:0]     buf_a_wdata;

    logic                  buf_b_we;
    logic [IDX_W-1:0]      buf_b_row;
    logic [IDX_W-1:0]      buf_b_col;
    logic signed [7:0]     buf_b_wdata;

    logic [IDX_W-1:0]      buf_c_row;
    logic [IDX_W-1:0]      buf_c_col;
    logic signed [31:0]    buf_c_rdata;

    // Matrizes de trabalho no BFM (até 32x32 conforme especificação)
    int                    mat_A [0:31][0:31];
    int                    mat_B [0:31][0:31];
    int                    mat_C_got [0:31][0:31];
    int                    mat_C_exp [0:31][0:31];

    int                    total_comp_cycles;
    int                    errors;
    int                    subtests_passed;
    int                    subtests_total;

    // Instância do DUT (Top-Level do Acelerador)
    systolic_accelerator #(
        .ARRAY_SIZE (ARRAY_SIZE),
        .IDX_W      (IDX_W)
    ) u_dut (
        .clk         (clk),
        .rst_n       (rst_n),
        .acc_clear   (acc_clear),
        .acc_start   (acc_start),
        .acc_busy    (acc_busy),
        .acc_done    (acc_done),
        .buf_a_we    (buf_a_we),
        .buf_a_row   (buf_a_row),
        .buf_a_col   (buf_a_col),
        .buf_a_wdata (buf_a_wdata),
        .buf_b_we    (buf_b_we),
        .buf_b_row   (buf_b_row),
        .buf_b_col   (buf_b_col),
        .buf_b_wdata (buf_b_wdata),
        .buf_c_row   (buf_c_row),
        .buf_c_col   (buf_c_col),
        .buf_c_rdata (buf_c_rdata)
    );

    // -------------------------------------------------------------------------
    // Validação estrita de faixa int8 [-128..127] e relatório por subteste
    // -------------------------------------------------------------------------

    task automatic bfm_check_int8(input int val, input string ctx);
        if (val < -128 || val > 127) begin
            $display("[FAIL][%0dx%0d] %s: valor %0d fora da faixa estrita int8 [-128..127]!",
                     ARRAY_SIZE, ARRAY_SIZE, ctx, val);
            errors = errors + 1;
        end
    endtask

    task automatic report_subtest(input int err_before, input string label);
        subtests_total = subtests_total + 1;
        if (errors == err_before) begin
            subtests_passed = subtests_passed + 1;
            $display("[PASS][%0dx%0d] %s", ARRAY_SIZE, ARRAY_SIZE, label);
        end else begin
            $display("[FAIL][%0dx%0d] %s (%0d erros neste subteste)",
                     ARRAY_SIZE, ARRAY_SIZE, label, (errors - err_before));
        end
    endtask

    // -------------------------------------------------------------------------
    // Tasks BFM (Emulação do Controlador de DMA)
    // -------------------------------------------------------------------------

    task automatic bfm_reset();
        rst_n       = 1'b0;
        acc_clear   = 1'b0;
        acc_start   = 1'b0;
        buf_a_we    = 1'b0;
        buf_a_row   = '0;
        buf_a_col   = '0;
        buf_a_wdata = 8'sd0;
        buf_b_we    = 1'b0;
        buf_b_row   = '0;
        buf_b_col   = '0;
        buf_b_wdata = 8'sd0;
        buf_c_row   = '0;
        buf_c_col   = '0;
        @(posedge clk);
        @(posedge clk);
        #1;
        if (acc_busy !== 1'b0 || acc_done !== 1'b0) begin
            $display("[FAIL][%0dx%0d] Reset nao zerou acc_busy/acc_done!", ARRAY_SIZE, ARRAY_SIZE);
            errors = errors + 1;
        end
        rst_n = 1'b1;
        @(posedge clk);
        #1;
    endtask

    task automatic bfm_pulse_clear();
        @(negedge clk);
        acc_clear = 1'b1;
        acc_start = 1'b0;
        @(posedge clk);
        #1;
        @(negedge clk);
        acc_clear = 1'b0;
    endtask

    task automatic bfm_write_elem_a(input int r, input int c, input int val);
        bfm_check_int8(val, "bfm_write_elem_a");
        @(negedge clk);
        buf_a_we    = 1'b1;
        buf_a_row   = r[IDX_W-1:0];
        buf_a_col   = c[IDX_W-1:0];
        buf_a_wdata = val[7:0];
        @(posedge clk);
        #1;
        @(negedge clk);
        buf_a_we    = 1'b0;
    endtask

    task automatic bfm_write_elem_b(input int r, input int c, input int val);
        bfm_check_int8(val, "bfm_write_elem_b");
        @(negedge clk);
        buf_b_we    = 1'b1;
        buf_b_row   = r[IDX_W-1:0];
        buf_b_col   = c[IDX_W-1:0];
        buf_b_wdata = val[7:0];
        @(posedge clk);
        #1;
        @(negedge clk);
        buf_b_we    = 1'b0;
    endtask

    // Carrega simultaneamente um par de tiles A_{ti,tk} e B_{tk,tj} com Zero-Padding
    task automatic bfm_load_tiles_padded(
        input int row_base,
        input int k_base,
        input int col_base,
        input int M,
        input int K,
        input int N
    );
        int i, j, r_a, c_a, r_b, c_b;
        int val_a, val_b;
        for (i = 0; i < ARRAY_SIZE; i = i + 1) begin
            for (j = 0; j < ARRAY_SIZE; j = j + 1) begin
                r_a   = row_base + i;
                c_a   = k_base   + j;
                val_a = (r_a < M && c_a < K) ? mat_A[r_a][c_a] : 0;
                bfm_check_int8(val_a, "bfm_load_tiles_padded(A)");

                r_b   = k_base   + i;
                c_b   = col_base + j;
                val_b = (r_b < K && c_b < N) ? mat_B[r_b][c_b] : 0;
                bfm_check_int8(val_b, "bfm_load_tiles_padded(B)");

                @(negedge clk);
                buf_a_we    = 1'b1;
                buf_a_row   = i[IDX_W-1:0];
                buf_a_col   = j[IDX_W-1:0];
                buf_a_wdata = val_a[7:0];

                buf_b_we    = 1'b1;
                buf_b_row   = i[IDX_W-1:0];
                buf_b_col   = j[IDX_W-1:0];
                buf_b_wdata = val_b[7:0];

                @(posedge clk);
                #1;
            end
        end
        @(negedge clk);
        buf_a_we = 1'b0;
        buf_b_we = 1'b0;
    endtask

    // Pulsa acc_start, monitora acc_busy ciclo a ciclo e verifica (3*T - 2) ciclos e pulso acc_done
    task automatic bfm_run_tile_and_verify_timing(output int busy_cycles_measured);
        int expected_cycles;
        expected_cycles      = (3 * ARRAY_SIZE) - 2;
        busy_cycles_measured = 0;

        @(negedge clk);
        acc_start = 1'b1;
        acc_clear = 1'b0;
        @(posedge clk); // Borda em que acc_start é aceito -> acc_busy deve subir para 1
        #1;
        if (acc_busy !== 1'b1 || acc_done !== 1'b0) begin
            $display("[FAIL][%0dx%0d] Apos acc_start: esperado acc_busy=1, acc_done=0; obteve busy=%b, done=%b",
                     ARRAY_SIZE, ARRAY_SIZE, acc_busy, acc_done);
            errors = errors + 1;
        end
        @(negedge clk);
        acc_start = 1'b0;

        // Conta quantas bordas de subida ocorrem com acc_busy == 1 (com limite contra travamento)
        while (acc_busy === 1'b1 && busy_cycles_measured <= (expected_cycles + 5)) begin
            busy_cycles_measured = busy_cycles_measured + 1;
            if (acc_done !== 1'b0) begin
                $display("[FAIL][%0dx%0d] acc_done subiu prematuramente durante acc_busy=1!", ARRAY_SIZE, ARRAY_SIZE);
                errors = errors + 1;
            end
            @(posedge clk);
            #1;
            if (busy_cycles_measured > (expected_cycles + 5)) begin
                $display("[FAIL][%0dx%0d] Timeout aguardando fim de acc_busy!", ARRAY_SIZE, ARRAY_SIZE);
                errors = errors + 1;
            end
        end

        // Na borda em que acc_busy cai para 0, acc_done deve estar em 1 por exatamente 1 ciclo
        if (busy_cycles_measured != expected_cycles) begin
            $display("[FAIL][%0dx%0d] Contagem de ciclos de acc_busy incorreta: esperado %0d, obteve %0d",
                     ARRAY_SIZE, ARRAY_SIZE, expected_cycles, busy_cycles_measured);
            errors = errors + 1;
        end
        if (acc_done !== 1'b1) begin
            $display("[FAIL][%0dx%0d] Esperado pulso acc_done=1 ao fim de acc_busy, obteve %b",
                     ARRAY_SIZE, ARRAY_SIZE, acc_done);
            errors = errors + 1;
        end

        // Verifica que no ciclo seguinte acc_done volta para 0
        @(posedge clk);
        #1;
        if (acc_done !== 1'b0 || acc_busy !== 1'b0) begin
            $display("[FAIL][%0dx%0d] acc_done nao limpou apos 1 ciclo (done=%b, busy=%b)!",
                     ARRAY_SIZE, ARRAY_SIZE, acc_done, acc_busy);
            errors = errors + 1;
        end
    endtask

    // Lê combinacionalmente um elemento PE(r, c)
    task automatic bfm_read_elem_c(input int r, input int c, output int val);
        @(negedge clk);
        buf_c_row = r[IDX_W-1:0];
        buf_c_col = c[IDX_W-1:0];
        #1;
        val = buf_c_rdata;
    endtask

    // Executa multiplicação completa C (MxN) = A (MxK) * B (KxN) com Tiling (ti, tj, tk)
    task automatic bfm_matmul_tiled(input int M, input int K, input int N);
        int num_tile_rows, num_tile_cols, num_tile_k;
        int ti, tj, tk, i, j, r, c;
        int cyc, val_c;

        num_tile_rows     = (M + ARRAY_SIZE - 1) / ARRAY_SIZE;
        num_tile_cols     = (N + ARRAY_SIZE - 1) / ARRAY_SIZE;
        num_tile_k        = (K + ARRAY_SIZE - 1) / ARRAY_SIZE;
        total_comp_cycles = 0;

        for (ti = 0; ti < num_tile_rows; ti = ti + 1) begin
            for (tj = 0; tj < num_tile_cols; tj = tj + 1) begin
                // Início do bloco C_{ti, tj}: tk = 0 -> pulsa acc_clear
                bfm_pulse_clear();

                // Acumula produtos ao longo da dimensão comum K
                for (tk = 0; tk < num_tile_k; tk = tk + 1) begin
                    bfm_load_tiles_padded(
                        ti * ARRAY_SIZE,
                        tk * ARRAY_SIZE,
                        tj * ARRAY_SIZE,
                        M, K, N
                    );
                    bfm_run_tile_and_verify_timing(cyc);
                    total_comp_cycles = total_comp_cycles + cyc;
                end

                // Ao concluir o somatório em tk, lê os acumuladores válidos do bloco C_{ti, tj}
                for (i = 0; i < ARRAY_SIZE; i = i + 1) begin
                    r = ti * ARRAY_SIZE + i;
                    for (j = 0; j < ARRAY_SIZE; j = j + 1) begin
                        c = tj * ARRAY_SIZE + j;
                        if (r < M && c < N) begin
                            bfm_read_elem_c(i, j, val_c);
                            mat_C_got[r][c] = val_c;
                        end
                    end
                end
            end
        end
    endtask

    // Calcula referência algébrica direta sem truncar mat_A/mat_B e compara com mat_C_got
    task automatic check_against_reference(
        input int    M,
        input int    K,
        input int    N,
        input int    err_before,
        input string test_label
    );
        int i, j, k, acc_ref;
        string full_label;

        for (i = 0; i < M; i = i + 1) begin
            for (k = 0; k < K; k = k + 1) begin
                bfm_check_int8(mat_A[i][k], "check_against_reference(A)");
            end
        end
        for (k = 0; k < K; k = k + 1) begin
            for (j = 0; j < N; j = j + 1) begin
                bfm_check_int8(mat_B[k][j], "check_against_reference(B)");
            end
        end

        for (i = 0; i < M; i = i + 1) begin
            for (j = 0; j < N; j = j + 1) begin
                acc_ref = 0;
                for (k = 0; k < K; k = k + 1) begin
                    acc_ref = acc_ref + (mat_A[i][k] * mat_B[k][j]);
                end
                mat_C_exp[i][j] = acc_ref;
                if (mat_C_got[i][j] !== acc_ref) begin
                    $display("[FAIL][%0dx%0d] %s: C[%0d][%0d] obtido=%0d, esperado=%0d",
                             ARRAY_SIZE, ARRAY_SIZE, test_label, i, j, mat_C_got[i][j], acc_ref);
                    errors = errors + 1;
                end
            end
        end

        full_label = $sformatf("%s (M=%0d, K=%0d, N=%0d | %0d ciclos de malha)",
                               test_label, M, K, N, total_comp_cycles);
        report_subtest(err_before, full_label);
    endtask

    // -------------------------------------------------------------------------
    // Suíte Completa de Testes Unitários para a instância (ARRAY_SIZE x ARRAY_SIZE)
    // -------------------------------------------------------------------------
    task automatic run_all_tests();
        int i, j, val_c, cyc, err_before;
        int expected_cycles, wait_cycles;
        reg [31:0] hex_a_4x4 [0:15];
        reg [31:0] hex_b_4x4 [0:15];
        reg [31:0] hex_c_4x4 [0:15];
        reg [31:0] hex_a_8x8 [0:63];
        reg [31:0] hex_b_8x8 [0:63];
        reg [31:0] hex_c_8x8 [0:63];

        errors          = 0;
        subtests_passed = 0;
        subtests_total  = 0;
        bfm_reset();

        // =====================================================================
        // TESTE 1: 1 Tile Único (T x T) + Temporização (3*T - 2 ciclos)
        //          Operandos estritamente dentro de [-128..127] (de -95 a +94)
        // =====================================================================
        err_before = errors;
        for (i = 0; i < ARRAY_SIZE; i = i + 1) begin
            for (j = 0; j < ARRAY_SIZE; j = j + 1) begin
                mat_A[i][j] = ((i * ARRAY_SIZE + j) * 3) - 95;
                mat_B[i][j] = ((i + 1) * (j + 2)) - 20;
            end
        end
        bfm_matmul_tiled(ARRAY_SIZE, ARRAY_SIZE, ARRAY_SIZE);
        check_against_reference(ARRAY_SIZE, ARRAY_SIZE, ARRAY_SIZE, err_before,
                                "Teste 1: Tile Unico TxT e Temporizacao (3T-2 ciclos)");

        // =====================================================================
        // TESTE 2: Limites Extremos int8 (-128, +127, -1, 0) e Acumulador int32
        // =====================================================================
        err_before = errors;
        for (i = 0; i < 4; i = i + 1) begin
            for (j = 0; j < 4; j = j + 1) begin
                if (i == 0)      mat_A[i][j] = -128;
                else if (i == 1) mat_A[i][j] = (j % 2 == 0) ? 127 : -128;
                else if (i == 2) mat_A[i][j] = -1;
                else             mat_A[i][j] = (j * 50) - 75;

                if (j == 0)      mat_B[i][j] = -128;
                else if (j == 1) mat_B[i][j] = (i % 2 == 0) ? 127 : -128;
                else if (j == 2) mat_B[i][j] = (i % 2 == 0) ? -1 : 1;
                else             mat_B[i][j] = (i % 2 == 0) ? 2 : -2;
            end
        end
        bfm_matmul_tiled(4, 4, 4);
        if (mat_C_got[0][0] !== 65536) begin
            $display("[FAIL][%0dx%0d] Teste 2: Esperado C[0][0] = +65536, obteve %0d",
                     ARRAY_SIZE, ARRAY_SIZE, mat_C_got[0][0]);
            errors = errors + 1;
        end
        check_against_reference(4, 4, 4, err_before,
                                "Teste 2: Extremos int8 (-128, +127) -> C[0][0]=+65536 em int32");

        // =====================================================================
        // TESTE 3: Acumulação Múltipla sem acc_clear vs Limpeza com acc_clear
        // =====================================================================
        err_before = errors;
        bfm_pulse_clear();
        for (i = 0; i < ARRAY_SIZE; i = i + 1) begin
            for (j = 0; j < ARRAY_SIZE; j = j + 1) begin
                bfm_write_elem_a(i, j, (i == j) ? 2 : 0); // 2 * Identidade
                bfm_write_elem_b(i, j, (i * 10) + j + 1); // Max em 8x8 = 78 <= 127
            end
        end
        // Executa 3 vezes seguidas SEM acc_clear: deve acumular 3 * (2 * B) = 6 * B
        bfm_run_tile_and_verify_timing(cyc);
        bfm_run_tile_and_verify_timing(cyc);
        bfm_run_tile_and_verify_timing(cyc);
        for (i = 0; i < ARRAY_SIZE; i = i + 1) begin
            for (j = 0; j < ARRAY_SIZE; j = j + 1) begin
                bfm_read_elem_c(i, j, val_c);
                if (val_c !== 6 * ((i * 10) + j + 1)) begin
                    $display("[FAIL][%0dx%0d] Teste 3: Acumulacao sem clear falhou em (%0d,%0d): %0d != %0d",
                             ARRAY_SIZE, ARRAY_SIZE, i, j, val_c, 6 * ((i * 10) + j + 1));
                    errors = errors + 1;
                end
            end
        end
        // Agora pulsa acc_clear e verifica que todos os acumuladores zeraram
        bfm_pulse_clear();
        for (i = 0; i < ARRAY_SIZE; i = i + 1) begin
            for (j = 0; j < ARRAY_SIZE; j = j + 1) begin
                bfm_read_elem_c(i, j, val_c);
                if (val_c !== 0) begin
                    $display("[FAIL][%0dx%0d] Teste 3: acc_clear nao zerou PE(%0d,%0d): %0d",
                             ARRAY_SIZE, ARRAY_SIZE, i, j, val_c);
                    errors = errors + 1;
                end
            end
        end
        report_subtest(err_before,
                       "Teste 3: Acumulacao de 3 tiles consecutivos sem clear e zeramento por acc_clear");

        // =====================================================================
        // TESTE 4: Vetores .hex do Golden Model (4x4 e 8x8)
        // =====================================================================
        err_before = errors;
        $readmemh("sw/golden_model/test_vectors/mat_4x4_A.hex", hex_a_4x4);
        $readmemh("sw/golden_model/test_vectors/mat_4x4_B.hex", hex_b_4x4);
        $readmemh("sw/golden_model/test_vectors/mat_4x4_C_expected.hex", hex_c_4x4);
        for (i = 0; i < 4; i = i + 1) begin
            for (j = 0; j < 4; j = j + 1) begin
                mat_A[i][j] = $signed(hex_a_4x4[i*4 + j][7:0]);
                mat_B[i][j] = $signed(hex_b_4x4[i*4 + j][7:0]);
            end
        end
        bfm_matmul_tiled(4, 4, 4);
        for (i = 0; i < 4; i = i + 1) begin
            for (j = 0; j < 4; j = j + 1) begin
                if (mat_C_got[i][j] !== $signed(hex_c_4x4[i*4 + j])) begin
                    $display("[FAIL][%0dx%0d] Teste 4a (HEX 4x4): C[%0d][%0d]=%0d != HEX=%0d",
                             ARRAY_SIZE, ARRAY_SIZE, i, j, mat_C_got[i][j], $signed(hex_c_4x4[i*4 + j]));
                    errors = errors + 1;
                end
            end
        end

        $readmemh("sw/golden_model/test_vectors/mat_8x8x8_A.hex", hex_a_8x8);
        $readmemh("sw/golden_model/test_vectors/mat_8x8x8_B.hex", hex_b_8x8);
        $readmemh("sw/golden_model/test_vectors/mat_8x8x8_C_expected.hex", hex_c_8x8);
        for (i = 0; i < 8; i = i + 1) begin
            for (j = 0; j < 8; j = j + 1) begin
                mat_A[i][j] = $signed(hex_a_8x8[i*8 + j][7:0]);
                mat_B[i][j] = $signed(hex_b_8x8[i*8 + j][7:0]);
            end
        end
        bfm_matmul_tiled(8, 8, 8);
        for (i = 0; i < 8; i = i + 1) begin
            for (j = 0; j < 8; j = j + 1) begin
                if (mat_C_got[i][j] !== $signed(hex_c_8x8[i*8 + j])) begin
                    $display("[FAIL][%0dx%0d] Teste 4b (HEX 8x8): C[%0d][%0d]=%0d != HEX=%0d",
                             ARRAY_SIZE, ARRAY_SIZE, i, j, mat_C_got[i][j], $signed(hex_c_8x8[i*8 + j]));
                    errors = errors + 1;
                end
            end
        end
        if ((ARRAY_SIZE == 2 && total_comp_cycles != 256) ||
            (ARRAY_SIZE == 4 && total_comp_cycles != 80)  ||
            (ARRAY_SIZE == 8 && total_comp_cycles != 22)) begin
            $display("[FAIL][%0dx%0d] Teste 4b: Ciclos de malha 8x8 (%0d) divergiram da tabela do README!",
                     ARRAY_SIZE, ARRAY_SIZE, total_comp_cycles);
            errors = errors + 1;
        end
        report_subtest(err_before,
                       $sformatf("Teste 4: Vetores Golden Model .hex (4x4 e 8x8 | %0d ciclos de malha no 8x8)",
                                 total_comp_cycles));

        // =====================================================================
        // TESTE 5a e 5b: Matrizes Retangulares e Não-Múltiplas com Zero-Padding
        //                Operandos estritamente em [-128..127]
        // =====================================================================
        err_before = errors;
        for (i = 0; i < 3; i = i + 1)
            for (j = 0; j < 5; j = j + 1)
                mat_A[i][j] = 1;
        for (i = 0; i < 5; i = i + 1)
            for (j = 0; j < 2; j = j + 1)
                mat_B[i][j] = 1;
        bfm_matmul_tiled(3, 5, 2);
        if ((ARRAY_SIZE == 2 && total_comp_cycles != 24) ||
            (ARRAY_SIZE == 4 && total_comp_cycles != 20) ||
            (ARRAY_SIZE == 8 && total_comp_cycles != 22)) begin
            $display("[FAIL][%0dx%0d] Teste 5a: Ciclos em 3x5*5x2 (%0d) divergiram do Golden Model!",
                     ARRAY_SIZE, ARRAY_SIZE, total_comp_cycles);
            errors = errors + 1;
        end
        check_against_reference(3, 5, 2, err_before,
                                "Teste 5a: Matriz Retangular 3x5 * 5x2 com Zero-Padding");

        err_before = errors;
        for (i = 0; i < 5; i = i + 1)
            for (j = 0; j < 7; j = j + 1)
                mat_A[i][j] = ((i * 7 + j) * 4) - 70; // Faixa [-70 .. +66]
        for (i = 0; i < 7; i = i + 1)
            for (j = 0; j < 6; j = j + 1)
                mat_B[i][j] = ((i * 6 + j) * 3) - 60; // Faixa [-60 .. +63]
        bfm_matmul_tiled(5, 7, 6);
        check_against_reference(5, 7, 6, err_before,
                                "Teste 5b: Matriz Nao-Multipla 5x7 * 7x6 com Zero-Padding");

        // =====================================================================
        // TESTE 6: Proteção rigorosa durante acc_busy = 1 (sobre posições de A
        //          e B ainda não consumidas + acc_start + acc_clear + reexecução
        //          sem recarregar buffers), prioridade de acc_clear e retenção
        // =====================================================================
        err_before      = errors;
        expected_cycles = (3 * ARRAY_SIZE) - 2;
        bfm_pulse_clear();
        for (i = 0; i < ARRAY_SIZE; i = i + 1) begin
            for (j = 0; j < ARRAY_SIZE; j = j + 1) begin
                bfm_write_elem_a(i, j, 3);
                bfm_write_elem_b(i, j, 4);
            end
        end

        // 6.1. Prioridade de acc_clear sobre acc_start quando acionados no mesmo ciclo
        @(negedge clk);
        acc_clear = 1'b1;
        acc_start = 1'b1;
        @(posedge clk);
        #1;
        if (acc_busy !== 1'b0) begin
            $display("[FAIL][%0dx%0d] Teste 6: acc_start iniciou mesmo com acc_clear=1 simultaneo!",
                     ARRAY_SIZE, ARRAY_SIZE);
            errors = errors + 1;
        end

        // 6.2. Inicia a execução real do tile
        @(negedge clk);
        acc_clear = 1'b0;
        acc_start = 1'b1;
        @(posedge clk); // Borda de aceitação do acc_start -> acc_busy sobe para 1 (cycle_cnt = 0)
        #1;
        if (acc_busy !== 1'b1) begin
            $display("[FAIL][%0dx%0d] Teste 6: acc_busy nao subiu apos acc_start!", ARRAY_SIZE, ARRAY_SIZE);
            errors = errors + 1;
        end

        // Ciclo ativo 1 (cycle_cnt = 0): Tenta sobrescrever A[0][T-1] e B[T-1][0],
        // que SÓ serão lidos pelo skew em cycle_cnt = T-1 >= 1, além de pulsar acc_clear!
        @(negedge clk);
        acc_start   = 1'b0;
        acc_clear   = 1'b1;
        buf_a_we    = 1'b1;
        buf_a_row   = '0;
        buf_a_col   = (ARRAY_SIZE - 1);
        buf_a_wdata = 8'sd99;
        buf_b_we    = 1'b1;
        buf_b_row   = (ARRAY_SIZE - 1);
        buf_b_col   = '0;
        buf_b_wdata = -8'sd99;
        @(posedge clk); // Fim do ciclo ativo 1 (cycle_cnt avança para 1)
        #1;

        // Ciclo ativo 2 (cycle_cnt = 1): Tenta sobrescrever A[T-1][T-1] e B[T-1][T-1],
        // que SÓ serão lidos em cycle_cnt = 2*T - 2 >= 2, tenta sobrescrever A[0][0]/B[0][0]
        // e pulsa acc_start = 1 durante acc_busy = 1 para tentar reiniciar o contador!
        @(negedge clk);
        acc_clear   = 1'b0;
        acc_start   = 1'b1;
        buf_a_we    = 1'b1;
        buf_a_row   = (ARRAY_SIZE - 1);
        buf_a_col   = (ARRAY_SIZE - 1);
        buf_a_wdata = 8'sd100;
        buf_b_we    = 1'b1;
        buf_b_row   = (ARRAY_SIZE - 1);
        buf_b_col   = (ARRAY_SIZE - 1);
        buf_b_wdata = -8'sd100;
        @(posedge clk); // Fim do ciclo ativo 2 (cycle_cnt avança para 2)
        #1;

        // Ciclo ativo 3 (cycle_cnt = 2): Tenta sobrescrever A[0][0] e B[0][0]
        @(negedge clk);
        acc_start   = 1'b0;
        buf_a_we    = 1'b1;
        buf_a_row   = '0;
        buf_a_col   = '0;
        buf_a_wdata = 8'sd77;
        buf_b_we    = 1'b1;
        buf_b_row   = '0;
        buf_b_col   = '0;
        buf_b_wdata = -8'sd77;
        @(posedge clk); // Fim do ciclo ativo 3
        #1;

        @(negedge clk);
        buf_a_we = 1'b0;
        buf_b_we = 1'b0;

        // Já consumimos 3 bordas de subida com acc_busy = 1; conta as restantes com limite de timeout
        wait_cycles = 3;
        while (acc_busy === 1'b1 && wait_cycles <= (expected_cycles + 5)) begin
            wait_cycles = wait_cycles + 1;
            @(posedge clk);
            #1;
            if (wait_cycles > (expected_cycles + 5)) begin
                $display("[FAIL][%0dx%0d] Teste 6: Timeout aguardando fim de acc_busy!",
                         ARRAY_SIZE, ARRAY_SIZE);
                errors = errors + 1;
            end
        end

        // Verifica que acc_start durante acc_busy não reiniciou nem alterou a duração (3*T - 2)
        if (wait_cycles != expected_cycles || acc_done !== 1'b1) begin
            $display("[FAIL][%0dx%0d] Teste 6: Duracao alterada por comandos durante busy: ciclos=%0d (esp=%0d), done=%b",
                     ARRAY_SIZE, ARRAY_SIZE, wait_cycles, expected_cycles, acc_done);
            errors = errors + 1;
        end

        // Aguarda 3 ciclos ocioso para provar retenção de C e confere resultado da 1ª execução
        repeat (3) @(posedge clk);
        for (i = 0; i < ARRAY_SIZE; i = i + 1) begin
            for (j = 0; j < ARRAY_SIZE; j = j + 1) begin
                bfm_read_elem_c(i, j, val_c);
                if (val_c !== (ARRAY_SIZE * 12)) begin
                    $display("[FAIL][%0dx%0d] Teste 6 (1a exec): Corrupcao durante acc_busy em (%0d,%0d): %0d != %0d",
                             ARRAY_SIZE, ARRAY_SIZE, i, j, val_c, ARRAY_SIZE * 12);
                    errors = errors + 1;
                end
            end
        end

        // 6.3. Reexecuta um novo cálculo SEM recarregar buf_a e buf_b:
        // Se qualquer escrita indevida (incluindo A[0][0] ou B[0][0]) tivesse sido gravada,
        // esta segunda execução falharia!
        bfm_run_tile_and_verify_timing(cyc);
        for (i = 0; i < ARRAY_SIZE; i = i + 1) begin
            for (j = 0; j < ARRAY_SIZE; j = j + 1) begin
                bfm_read_elem_c(i, j, val_c);
                if (val_c !== (2 * ARRAY_SIZE * 12)) begin
                    $display("[FAIL][%0dx%0d] Teste 6 (2a exec sem recarga): Buffer foi alterado durante busy em (%0d,%0d): %0d != %0d",
                             ARRAY_SIZE, ARRAY_SIZE, i, j, val_c, 2 * ARRAY_SIZE * 12);
                    errors = errors + 1;
                end
            end
        end
        report_subtest(err_before,
                       "Teste 6: Protecao de A/B/start/clear em acc_busy=1 (com reexecucao sem recarga) e retencao de C");

        // =====================================================================
        // TESTE 7: Reset Assíncrono (rst_n = 0) Durante Processamento Ativo
        //          - Verifica limpeza assíncrona dos acumuladores ANTES da
        //            próxima borda de subida do clock;
        //          - Verifica independentemente a limpeza de buf_a (carregando
        //            apenas buf_b) e a limpeza de buf_b (carregando apenas buf_a);
        //          - Verifica recuperação completa em nova operação.
        // =====================================================================
        err_before = errors;
        bfm_pulse_clear();
        for (i = 0; i < ARRAY_SIZE; i = i + 1) begin
            for (j = 0; j < ARRAY_SIZE; j = j + 1) begin
                bfm_write_elem_a(i, j, 10);
                bfm_write_elem_b(i, j, 10);
            end
        end
        // Inicia processamento e avança 2 ciclos dentro de acc_busy = 1
        @(negedge clk);
        acc_start = 1'b1;
        @(posedge clk);
        #1;
        @(negedge clk);
        acc_start = 1'b0;
        @(posedge clk); // PE(0,0) já acumulou 10*10 = 100
        #1;
        @(posedge clk); // PE(0,0) já acumulou 200; PE(0,1) e PE(1,0) acumularam 100
        #1;
        buf_c_row = '0;
        buf_c_col = '0;
        #1;
        if (acc_busy !== 1'b1 || buf_c_rdata === 32'sd0) begin
            $display("[FAIL][%0dx%0d] Teste 7: Acelerador deveria estar ocupado e com soma parcial nao-nula antes do reset!",
                     ARRAY_SIZE, ARRAY_SIZE);
            errors = errors + 1;
        end

        // 7.1. Aciona reset assíncrono no meio da execução e verifica IMEDIATAMENTE
        //      (ANTES de qualquer borda de subida do clock) que busy, done e todos
        //      os acumuladores zeraram assincronamente. A verificação de todos os
        //      PEs é simultânea: compara o vetor completo acc_flat com zero após
        //      um único atraso de 1 ns (a próxima borda de subida ocorre ~6 ns depois).
        rst_n = 1'b0;
        #1;
        if (acc_busy !== 1'b0 || acc_done !== 1'b0) begin
            $display("[FAIL][%0dx%0d] Teste 7: Reset assincrono nao derrubou acc_busy/acc_done imediatamente!",
                     ARRAY_SIZE, ARRAY_SIZE);
            errors = errors + 1;
        end
        if (u_dut.acc_flat !== '0) begin
            $display("[FAIL][%0dx%0d] Teste 7: acc_flat nao zerou assincronamente antes da borda de clock: %h",
                     ARRAY_SIZE, ARRAY_SIZE, u_dut.acc_flat);
            errors = errors + 1;
            // Diagnóstico: identifica individualmente os PEs que falharam
            // (o reset continua ativo, portanto os valores ficam congelados).
            for (i = 0; i < ARRAY_SIZE; i = i + 1) begin
                for (j = 0; j < ARRAY_SIZE; j = j + 1) begin
                    buf_c_row = i[IDX_W-1:0];
                    buf_c_col = j[IDX_W-1:0];
                    #1;
                    if (buf_c_rdata !== 32'sd0) begin
                        $display("[FAIL][%0dx%0d] Teste 7:   PE(%0d,%0d) = %0d",
                                 ARRAY_SIZE, ARRAY_SIZE, i, j, buf_c_rdata);
                    end
                end
            end
        end

        // Libera o reset
        @(posedge clk);
        #1;
        rst_n = 1'b1;
        @(posedge clk);
        #1;

        // 7.2. Comprova que buf_a foi zerado pelo reset (carregando SOMENTE buf_b com 1s)
        for (i = 0; i < ARRAY_SIZE; i = i + 1) begin
            for (j = 0; j < ARRAY_SIZE; j = j + 1) begin
                bfm_write_elem_b(i, j, 1);
            end
        end
        bfm_run_tile_and_verify_timing(cyc);
        for (i = 0; i < ARRAY_SIZE; i = i + 1) begin
            for (j = 0; j < ARRAY_SIZE; j = j + 1) begin
                bfm_read_elem_c(i, j, val_c);
                if (val_c !== 0) begin
                    $display("[FAIL][%0dx%0d] Teste 7: Buffer A nao foi zerado pelo reset (C[%0d][%0d]=%0d com B=1)!",
                             ARRAY_SIZE, ARRAY_SIZE, i, j, val_c);
                    errors = errors + 1;
                end
            end
        end

        // 7.3. Preenche ambos os buffers novamente, aplica reset e comprova que buf_b
        //      foi zerado pelo reset (carregando SOMENTE buf_a com 1s)
        for (i = 0; i < ARRAY_SIZE; i = i + 1) begin
            for (j = 0; j < ARRAY_SIZE; j = j + 1) begin
                bfm_write_elem_a(i, j, 10);
                bfm_write_elem_b(i, j, 10);
            end
        end
        rst_n = 1'b0;
        #1;
        @(posedge clk);
        #1;
        rst_n = 1'b1;
        @(posedge clk);
        #1;
        for (i = 0; i < ARRAY_SIZE; i = i + 1) begin
            for (j = 0; j < ARRAY_SIZE; j = j + 1) begin
                bfm_write_elem_a(i, j, 1);
            end
        end
        bfm_run_tile_and_verify_timing(cyc);
        for (i = 0; i < ARRAY_SIZE; i = i + 1) begin
            for (j = 0; j < ARRAY_SIZE; j = j + 1) begin
                bfm_read_elem_c(i, j, val_c);
                if (val_c !== 0) begin
                    $display("[FAIL][%0dx%0d] Teste 7: Buffer B nao foi zerado pelo reset (C[%0d][%0d]=%0d com A=1)!",
                             ARRAY_SIZE, ARRAY_SIZE, i, j, val_c);
                    errors = errors + 1;
                end
            end
        end

        // 7.4. Executa uma nova operação após o reset para comprovar recuperação completa
        for (i = 0; i < ARRAY_SIZE; i = i + 1) begin
            for (j = 0; j < ARRAY_SIZE; j = j + 1) begin
                mat_A[i][j] = (i + 1) * 2;
                mat_B[i][j] = (j + 1) * 3;
            end
        end
        bfm_matmul_tiled(ARRAY_SIZE, ARRAY_SIZE, ARRAY_SIZE);
        check_against_reference(ARRAY_SIZE, ARRAY_SIZE, ARRAY_SIZE, err_before,
                                "Teste 7: Reset assincrono durante acc_busy=1, limpeza independente de A/B/PEs e recuperacao");

        // =====================================================================
        // TESTE 8a: Casos Pequenos e Assimétricos (1x1, 1x7 * 7x1 e 6x3 * 3x7)
        // =====================================================================
        err_before  = errors;
        mat_A[0][0] = -128;
        mat_B[0][0] = 127;
        bfm_matmul_tiled(1, 1, 1);
        if (mat_C_got[0][0] !== -16256) begin
            $display("[FAIL][%0dx%0d] Teste 8a (1x1): C[0][0]=%0d != -16256",
                     ARRAY_SIZE, ARRAY_SIZE, mat_C_got[0][0]);
            errors = errors + 1;
        end
        for (j = 0; j < 7; j = j + 1) begin
            mat_A[0][j] = (j % 2 == 0) ? -15 * (j + 1) : 15 * (j + 1); // [-105 .. +90]
            mat_B[j][0] = (j * 10) - 30;                               // [-30 .. +30]
        end
        bfm_matmul_tiled(1, 7, 1);
        if (mat_C_got[0][0] !== (-15*(-30) + 30*(-20) - 45*(-10) + 60*0 - 75*10 + 90*20 - 105*30)) begin
            $display("[FAIL][%0dx%0d] Teste 8a (1x7*7x1): C[0][0]=%0d incorreto",
                     ARRAY_SIZE, ARRAY_SIZE, mat_C_got[0][0]);
            errors = errors + 1;
        end
        for (i = 0; i < 6; i = i + 1)
            for (j = 0; j < 3; j = j + 1)
                mat_A[i][j] = ((i * 3 + j) * 7) - 60; // [-60 .. +59]
        for (i = 0; i < 3; i = i + 1)
            for (j = 0; j < 7; j = j + 1)
                mat_B[i][j] = ((i * 7 + j) * 5) - 50; // [-50 .. +50]
        bfm_matmul_tiled(6, 3, 7);
        check_against_reference(6, 3, 7, err_before,
                                "Teste 8a: Casos pequenos e assimetricos (1x1, 1x7*7x1 e 6x3*3x7)");

        // =====================================================================
        // TESTE 8b: Caso Limite Máximo do Projeto (32x32 * 32x32 com -128)
        //           Verifica que todos os 1024 elementos resultam em +524288
        // =====================================================================
        err_before = errors;
        for (i = 0; i < 32; i = i + 1) begin
            for (j = 0; j < 32; j = j + 1) begin
                mat_A[i][j] = -128;
                mat_B[i][j] = -128;
            end
        end
        bfm_matmul_tiled(32, 32, 32);
        for (i = 0; i < 32; i = i + 1) begin
            for (j = 0; j < 32; j = j + 1) begin
                if (mat_C_got[i][j] !== 524288) begin
                    $display("[FAIL][%0dx%0d] Teste 8b (32x32 -128): C[%0d][%0d]=%0d != 524288",
                             ARRAY_SIZE, ARRAY_SIZE, i, j, mat_C_got[i][j]);
                    errors = errors + 1;
                end
            end
        end
        check_against_reference(32, 32, 32, err_before,
                                "Teste 8b: Limite maximo 32x32 com -128 (C[i][j] = +524288)");
    endtask

endmodule


// -----------------------------------------------------------------------------
// Módulo Top-Level do Testbench Unitário (Executa sequencialmente 2x2, 4x4 e 8x8)
// -----------------------------------------------------------------------------
module tb_systolic_accelerator;

    logic clk;
    initial clk = 1'b0;
    always #5 clk = ~clk; // Clock de 100 MHz (período = 10 ns)

    // Instâncias para os 3 tamanhos de malha exigidos no projeto: 2x2, 4x4 e 8x8
    accel_bfm_harness #(.ARRAY_SIZE(2)) u_harness_2x2 (.clk(clk));
    accel_bfm_harness #(.ARRAY_SIZE(4)) u_harness_4x4 (.clk(clk));
    accel_bfm_harness #(.ARRAY_SIZE(8)) u_harness_8x8 (.clk(clk));

    int total_errors;
    int total_passed;
    int total_subtests;

    // Watchdog global contra travamento infinito da simulação
    initial begin
        #500_000_000;
        $fatal(1, "[TIMEOUT GLOBAL] A simulacao excedeu o tempo maximo permitido!");
    end

    initial begin
        $dumpfile("tb_systolic_accelerator.vcd");
        $dumpvars(0, tb_systolic_accelerator);

        $display("==============================================================================");
        $display(" INICIANDO VERIFICACAO UNITARIA DO ACELERADOR SISTOLICO (2x2, 4x4 e 8x8)");
        $display("==============================================================================");

        $display("\n--- [1/3] Bateria de Testes na Malha 2x2 (ARRAY_SIZE = 2, 4 ciclos/tile) ---");
        u_harness_2x2.run_all_tests();

        $display("\n--- [2/3] Bateria de Testes na Malha 4x4 (ARRAY_SIZE = 4, 10 ciclos/tile) ---");
        u_harness_4x4.run_all_tests();

        $display("\n--- [3/3] Bateria de Testes na Malha 8x8 (ARRAY_SIZE = 8, 22 ciclos/tile) ---");
        u_harness_8x8.run_all_tests();

        total_errors   = u_harness_2x2.errors          + u_harness_4x4.errors          + u_harness_8x8.errors;
        total_passed   = u_harness_2x2.subtests_passed + u_harness_4x4.subtests_passed + u_harness_8x8.subtests_passed;
        total_subtests = u_harness_2x2.subtests_total  + u_harness_4x4.subtests_total  + u_harness_8x8.subtests_total;

        $display("\n==============================================================================");
        if (total_errors == 0 && total_passed == total_subtests) begin
            $display(" RESULTADO FINAL: TODOS OS %0d/%0d SUBTESTES PASSARAM (2x2, 4x4 e 8x8) - PASS!",
                     total_passed, total_subtests);
            $display("==============================================================================");
            $finish;
        end else begin
            $display(" RESULTADO FINAL: FALHA ENCONTRADA (%0d/%0d aprovados, %0d erros) - FAIL!",
                     total_passed, total_subtests, total_errors);
            $display("==============================================================================");
            $fatal(1, "Simulacao encerrada com codigo de erro devido a falhas na verificacao.");
        end
    end

endmodule
