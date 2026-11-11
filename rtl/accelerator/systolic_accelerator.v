// =============================================================================
// Módulo: systolic_accelerator.v
// Descrição: Top-Level do Acelerador Matricial (Buffers Locais A/B + Gerador
//            de Skew em Hardware + Malha Sistólica Output-Stationary + Controle).
//
// Contrato da Interface (conforme Seção 5 da Especificação):
//   - Quando ocioso (acc_busy == 0):
//       * buf_a_we / buf_b_we gravam 1 elemento int8 por ciclo na borda de
//         subida de clk nos Buffers Locais A e B (T x T elementos).
//       * acc_clear = 1 limpa todos os acumuladores acc[i][j] da malha.
//       * acc_start = 1 (com acc_clear = 0) inicia o cálculo do tile:
//         acc_busy sobe para 1, os registradores de passagem entre PEs são
//         limpos (preservando acc[i][j]) e a primeira atualização MAC ocorre
//         na borda de subida seguinte.
//   - Durante a execução (acc_busy == 1):
//       * Comandos externos (acc_start, acc_clear, buf_a_we, buf_b_we) são
//         ignorados para proteger os buffers e a acumulação em andamento.
//       * A lógica de skew injeta a linha i de A com atraso de i ciclos
//         (k_a = t - i) e a coluna j de B com atraso de j ciclos (k_b = t - j),
//         inserindo 0 fora da janela válida (0 <= k < T).
//       * Após exatas (3*T - 2) atualizações MAC, acc_busy cai para 0 e
//         acc_done sobe para 1 por exatamente 1 ciclo de clock.
//   - Leitura de C (buf_c_rdata):
//       * Combinacional, indexada por buf_c_row e buf_c_col.
// =============================================================================
`timescale 1ns / 1ps

module systolic_accelerator #(
    parameter integer ARRAY_SIZE = 4,
    parameter integer IDX_W      = $clog2(ARRAY_SIZE)
) (
    input  wire                  clk,
    input  wire                  rst_n,

    // Sinais de Controle e Status (DMA <-> Acelerador)
    input  wire                  acc_clear,
    input  wire                  acc_start,
    output reg                   acc_busy,
    output reg                   acc_done,

    // Porta de Escrita do Buffer Local A (DMA -> Acelerador)
    input  wire                  buf_a_we,
    input  wire [IDX_W-1:0]      buf_a_row,
    input  wire [IDX_W-1:0]      buf_a_col,
    input  wire signed [7:0]     buf_a_wdata,

    // Porta de Escrita do Buffer Local B (DMA -> Acelerador)
    input  wire                  buf_b_we,
    input  wire [IDX_W-1:0]      buf_b_row,
    input  wire [IDX_W-1:0]      buf_b_col,
    input  wire signed [7:0]     buf_b_wdata,

    // Porta de Leitura Combinacional dos Acumuladores C (DMA <-> Acelerador)
    input  wire [IDX_W-1:0]      buf_c_row,
    input  wire [IDX_W-1:0]      buf_c_col,
    output wire signed [31:0]    buf_c_rdata
);

    // Número total de ciclos de cálculo de 1 tile T x T com skew: 3*T - 2
    localparam integer TOTAL_CYCLES = (3 * ARRAY_SIZE) - 2;
    localparam integer CNT_W        = $clog2(TOTAL_CYCLES + 1);

    // Buffers locais A e B (T x T bytes int8)
    reg signed [7:0] buf_a [0:ARRAY_SIZE-1][0:ARRAY_SIZE-1];
    reg signed [7:0] buf_b [0:ARRAY_SIZE-1][0:ARRAY_SIZE-1];

    // Contador de ciclos da onda sistólica: t = 0 .. (TOTAL_CYCLES - 1)
    reg [CNT_W-1:0] cycle_cnt;

    // Sinais de controle interno para a malha de PEs
    wire pe_clear_acc;
    wire pe_clear_pipe;
    wire pe_en;

    // Com acc_busy == 0, acc_clear tem prioridade sobre acc_start (pulsos separados)
    assign pe_clear_acc  = (!acc_busy) && acc_clear;
    assign pe_clear_pipe = (!acc_busy) && (!acc_clear) && acc_start;
    assign pe_en         = acc_busy;

    // -------------------------------------------------------------------------
    // Escrita Síncrona nos Buffers Locais A e B (somente quando acc_busy == 0)
    // -------------------------------------------------------------------------
    integer r_idx, c_idx;
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            for (r_idx = 0; r_idx < ARRAY_SIZE; r_idx = r_idx + 1) begin
                for (c_idx = 0; c_idx < ARRAY_SIZE; c_idx = c_idx + 1) begin
                    buf_a[r_idx][c_idx] <= 8'sd0;
                    buf_b[r_idx][c_idx] <= 8'sd0;
                end
            end
        end else if (!acc_busy) begin
            if (buf_a_we) begin
                buf_a[buf_a_row][buf_a_col] <= buf_a_wdata;
            end
            if (buf_b_we) begin
                buf_b[buf_b_row][buf_b_col] <= buf_b_wdata;
            end
        end
    end

    // -------------------------------------------------------------------------
    // Máquina de Controle Temporal (acc_busy, acc_done e cycle_cnt)
    // -------------------------------------------------------------------------
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            acc_busy  <= 1'b0;
            acc_done  <= 1'b0;
            cycle_cnt <= {CNT_W{1'b0}};
        end else begin
            // Por padrão, acc_done é um pulso de apenas 1 ciclo
            acc_done <= 1'b0;

            if (!acc_busy) begin
                if (!acc_clear && acc_start) begin
                    acc_busy  <= 1'b1;
                    cycle_cnt <= {CNT_W{1'b0}};
                end
            end else begin
                // Malha ativa: conta os 3*T - 2 ciclos (0 até TOTAL_CYCLES - 1)
                if (cycle_cnt == (TOTAL_CYCLES - 1)) begin
                    acc_busy  <= 1'b0;
                    acc_done  <= 1'b1;
                    cycle_cnt <= {CNT_W{1'b0}};
                end else begin
                    cycle_cnt <= cycle_cnt + 1'b1;
                end
            end
        end
    end

    // -------------------------------------------------------------------------
    // Lógica de Skew em Hardware nas Bordas Esquerda (A) e Superior (B)
    //   - Linha i de A: k_a = t - i. Se 0 <= k_a < T, injeta buf_a[i][k_a]; senão 0.
    //   - Coluna j de B: k_b = t - j. Se 0 <= k_b < T, injeta buf_b[k_b][j]; senão 0.
    // -------------------------------------------------------------------------
    reg  [ARRAY_SIZE*8-1:0] a_edge_flat;
    reg  [ARRAY_SIZE*8-1:0] b_edge_flat;
    integer i_skew, j_skew;
    integer k_a, k_b;

    always @(*) begin
        a_edge_flat = {(ARRAY_SIZE*8){1'b0}};
        b_edge_flat = {(ARRAY_SIZE*8){1'b0}};
        k_a         = 0;
        k_b         = 0;

        if (acc_busy) begin
            for (i_skew = 0; i_skew < ARRAY_SIZE; i_skew = i_skew + 1) begin
                k_a = $signed({1'b0, cycle_cnt}) - i_skew;
                if ((k_a >= 0) && (k_a < ARRAY_SIZE)) begin
                    a_edge_flat[i_skew*8 +: 8] = buf_a[i_skew][k_a];
                end else begin
                    a_edge_flat[i_skew*8 +: 8] = 8'sd0;
                end
            end

            for (j_skew = 0; j_skew < ARRAY_SIZE; j_skew = j_skew + 1) begin
                k_b = $signed({1'b0, cycle_cnt}) - j_skew;
                if ((k_b >= 0) && (k_b < ARRAY_SIZE)) begin
                    b_edge_flat[j_skew*8 +: 8] = buf_b[k_b][j_skew];
                end else begin
                    b_edge_flat[j_skew*8 +: 8] = 8'sd0;
                end
            end
        end
    end

    // -------------------------------------------------------------------------
    // Instância da Malha Sistólica Parametrizável (ARRAY_SIZE x ARRAY_SIZE)
    // -------------------------------------------------------------------------
    wire [ARRAY_SIZE*ARRAY_SIZE*32-1:0] acc_flat;

    systolic_array #(
        .ARRAY_SIZE (ARRAY_SIZE)
    ) u_systolic_array (
        .clk         (clk),
        .rst_n       (rst_n),
        .clear_acc   (pe_clear_acc),
        .clear_pipe  (pe_clear_pipe),
        .en          (pe_en),
        .a_edge_flat (a_edge_flat),
        .b_edge_flat (b_edge_flat),
        .acc_flat    (acc_flat)
    );

    // -------------------------------------------------------------------------
    // Leitura Combinacional de C selecionada por (buf_c_row, buf_c_col)
    // -------------------------------------------------------------------------
    wire signed [31:0] acc_matrix [0:ARRAY_SIZE-1][0:ARRAY_SIZE-1];

    genvar gr, gc;
    generate
        for (gr = 0; gr < ARRAY_SIZE; gr = gr + 1) begin : gen_unpack_r
            for (gc = 0; gc < ARRAY_SIZE; gc = gc + 1) begin : gen_unpack_c
                assign acc_matrix[gr][gc] = acc_flat[(gr*ARRAY_SIZE + gc)*32 +: 32];
            end
        end
    endgenerate

    assign buf_c_rdata = acc_matrix[buf_c_row][buf_c_col];

endmodule
