// =============================================================================
// Projeto     : RISC-V RV32I com Controlador DMA e Coprocessador Matricial
// Etapa       : Entrega Parcial II — RTL do Acelerador (Systolic Array) e Testes Unitários
// Módulo      : systolic_accelerator.v
// Professores : Ana Julia Fernandes de Oliveira Barros
//               Igor Dantas dos Santos Miranda
//               João Alberto Castelo Branco Oliveira
// Grupo 4     : João Vitor Carvalho Lima
//               Matheus Artur Macedo Bomfim
//               Nathalia Miranda
//               Tatiana Caetano Praxedes de Moura
// Descrição   : Top-Level do Acelerador Matricial (Buffers Locais A e B +
//               Gerador de Skew Combinacional + Malha Sistólica Output-Stationary
//               + Máquina de Controle Temporal).
//               Interface dedicada de 15 sinais com o controlador DMA:
//               - Ocioso (acc_busy=0): permite escrita em buf_a/buf_b, limpeza
//                 acc_clear e disparo acc_start (preservando acumuladores C).
//               - Ativo (acc_busy=1): ignora comandos externos para proteger o
//                 cálculo, injeta skew (k_a = t-i, k_b = t-j) por exatos 3T-2
//                 ciclos e pulsa acc_done por 1 ciclo ao concluir o tile.
//               - Leitura de C (buf_c_rdata): combinacional por (buf_c_row, col).
// =============================================================================
`timescale 1ns / 1ps

module systolic_accelerator #(
    parameter integer ARRAY_SIZE = 4,                  // Dimensão física T da malha (2, 4 ou 8)
    parameter integer IDX_W      = $clog2(ARRAY_SIZE)  // Largura de índice local dos buffers (0..T-1)
) (
    input  wire                  clk,                  // Clock principal do sistema
    input  wire                  rst_n,                // Reset assíncrono ativo em nível baixo

    // Sinais de Controle e Status (DMA <-> Acelerador)
    input  wire                  acc_clear,            // Pulso de 1 ciclo para zerar os acumuladores C
    input  wire                  acc_start,            // Pulso de 1 ciclo para iniciar a onda sistólica
    output reg                   acc_busy,             // Alto durante os 3T-2 ciclos de execução do tile
    output reg                   acc_done,             // Pulso de 1 ciclo sinalizando conclusão do tile

    // Porta de Escrita do Buffer Local A (DMA -> Acelerador, ativa quando acc_busy == 0)
    input  wire                  buf_a_we,             // Habilita gravação no Buffer Local A
    input  wire [IDX_W-1:0]      buf_a_row,            // Linha destino em buf_a (0..T-1)
    input  wire [IDX_W-1:0]      buf_a_col,            // Coluna destino em buf_a (0..T-1)
    input  wire signed [7:0]     buf_a_wdata,          // Elemento int8 gravado em buf_a

    // Porta de Escrita do Buffer Local B (DMA -> Acelerador, ativa quando acc_busy == 0)
    input  wire                  buf_b_we,             // Habilita gravação no Buffer Local B
    input  wire [IDX_W-1:0]      buf_b_row,            // Linha destino em buf_b (0..T-1)
    input  wire [IDX_W-1:0]      buf_b_col,            // Coluna destino em buf_b (0..T-1)
    input  wire signed [7:0]     buf_b_wdata,          // Elemento int8 gravado em buf_b

    // Porta de Leitura Combinacional dos Acumuladores C (DMA <-> Acelerador)
    input  wire [IDX_W-1:0]      buf_c_row,            // Linha selecionada na malha (0..T-1)
    input  wire [IDX_W-1:0]      buf_c_col,            // Coluna selecionada na malha (0..T-1)
    output wire signed [31:0]    buf_c_rdata           // Valor acumulado int32 do PE(buf_c_row, buf_c_col)
);

    // Constantes temporais da onda sistólica para um tile T x T
    localparam integer TOTAL_CYCLES = (3 * ARRAY_SIZE) - 2; // Ciclos de execução por tile (4, 10 ou 22)
    localparam integer CNT_W        = $clog2(TOTAL_CYCLES + 1); // Largura do contador de ciclos

    // Buffers locais de entrada A e B (T x T registradores int8 cada)
    reg signed [7:0] buf_a [0:ARRAY_SIZE-1][0:ARRAY_SIZE-1]; // Sub-bloco atual de A carregado pelo DMA
    reg signed [7:0] buf_b [0:ARRAY_SIZE-1][0:ARRAY_SIZE-1]; // Sub-bloco atual de B carregado pelo DMA

    // Contador de ciclos da onda sistólica: t = 0 .. (TOTAL_CYCLES - 1)
    reg [CNT_W-1:0] cycle_cnt; // Índice temporal t usado pelo gerador de skew

    // Sinais internos de controle roteados para a malha systolic_array
    wire pe_clear_acc;  // Limpa acumuladores e pipeline (somente se ocioso e acc_clear=1)
    wire pe_clear_pipe; // Limpa apenas pipeline no disparo de acc_start (preserva C)
    wire pe_en;         // Habilita MAC e avanço sistólico enquanto acc_busy=1

    // Com acc_busy == 0, acc_clear tem prioridade sobre acc_start (proteção contra comandos simultâneos)
    assign pe_clear_acc  = (!acc_busy) && acc_clear;
    assign pe_clear_pipe = (!acc_busy) && (!acc_clear) && acc_start;
    assign pe_en         = acc_busy;

    // -------------------------------------------------------------------------
    // Escrita Síncrona nos Buffers Locais A e B (bloqueada quando acc_busy == 1)
    // -------------------------------------------------------------------------
    integer r_idx, c_idx; // Índices de laço para zeramento dos buffers no reset assíncrono
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
    reg  [ARRAY_SIZE*8-1:0] a_edge_flat; // Entradas int8 escalonadas da borda esquerda (A)
    reg  [ARRAY_SIZE*8-1:0] b_edge_flat; // Entradas int8 escalonadas da borda superior (B)
    integer i_skew, j_skew;              // Índices de linha (A) e coluna (B) no gerador de skew
    integer k_a, k_b;                    // Índices deslocados no tempo (t - i e t - j)

    // Bloco combinacional de skew: seleciona o elemento válido ou injeta 0 (bolha)
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
    wire [ARRAY_SIZE*ARRAY_SIZE*32-1:0] acc_flat; // Vetor linearizado com todos os acumuladores da malha

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
    wire signed [31:0] acc_matrix [0:ARRAY_SIZE-1][0:ARRAY_SIZE-1]; // Visão 2D dos acumuladores para indexação direta

    genvar gr, gc; // Índices de desempacotamento 2D (linha gr, coluna gc)
    generate
        for (gr = 0; gr < ARRAY_SIZE; gr = gr + 1) begin : gen_unpack_r
            for (gc = 0; gc < ARRAY_SIZE; gc = gc + 1) begin : gen_unpack_c
                assign acc_matrix[gr][gc] = acc_flat[(gr*ARRAY_SIZE + gc)*32 +: 32];
            end
        end
    endgenerate

    assign buf_c_rdata = acc_matrix[buf_c_row][buf_c_col];

endmodule
