// =============================================================================
// Projeto     : RISC-V RV32I com Controlador DMA e Coprocessador Matricial
// Etapa       : Entrega Parcial II — RTL do Acelerador (Systolic Array) e Testes Unitários
// Módulo      : systolic_array.v
// Professores : Ana Julia Fernandes de Oliveira Barros
//               Igor Dantas dos Santos Miranda
//               João Alberto Castelo Branco Oliveira
// Grupo 4     : João Vitor Carvalho Lima
//               Matheus Artur Macedo Bomfim
//               Nathalia Miranda
//               Tatiana Caetano Praxedes de Moura
// Descrição   : Malha Sistólica Bidimensional Parametrizável (T x T, T = 2/4/8)
//               composta por elementos mac_pe em arquitetura Output-Stationary.
//               Os elementos de A entram pela borda esquerda (a_edge_flat) e
//               avançam horizontalmente; os de B entram pela borda superior
//               (b_edge_flat) e avançam verticalmente; os acumuladores int32
//               permanecem estacionários em cada PE(i,j) e são expostos em
//               acc_flat para leitura combinacional.
// =============================================================================
`timescale 1ns / 1ps

module systolic_array #(
    parameter integer ARRAY_SIZE = 4           // Dimensão física T da malha quadrada (2, 4 ou 8)
) (
    input  wire                                clk,         // Clock principal do sistema
    input  wire                                rst_n,       // Reset assíncrono ativo em nível baixo
    input  wire                                clear_acc,   // Zera acumuladores e pipeline de todos os PEs
    input  wire                                clear_pipe,  // Zera apenas pipeline de todos os PEs (mantém C)
    input  wire                                en,          // Habilita execução simultânea dos T x T PEs
    input  wire [ARRAY_SIZE*8-1:0]             a_edge_flat, // Vetor linearizado com T entradas int8 da borda esquerda (A)
    input  wire [ARRAY_SIZE*8-1:0]             b_edge_flat, // Vetor linearizado com T entradas int8 da borda superior (B)
    output wire [ARRAY_SIZE*ARRAY_SIZE*32-1:0] acc_flat     // Vetor linearizado (row-major) com os T*T acumuladores int32
);

    // Matrizes de fios internos que interligam cada PE(gi, gj) aos seus vizinhos
    wire signed [7:0]  pe_a_in  [0:ARRAY_SIZE-1][0:ARRAY_SIZE-1]; // Entrada A de cada PE(gi, gj)
    wire signed [7:0]  pe_b_in  [0:ARRAY_SIZE-1][0:ARRAY_SIZE-1]; // Entrada B de cada PE(gi, gj)
    wire signed [7:0]  pe_a_out [0:ARRAY_SIZE-1][0:ARRAY_SIZE-1]; // Saída registrada A repassada à direita
    wire signed [7:0]  pe_b_out [0:ARRAY_SIZE-1][0:ARRAY_SIZE-1]; // Saída registrada B repassada abaixo
    wire signed [31:0] pe_acc   [0:ARRAY_SIZE-1][0:ARRAY_SIZE-1]; // Acumulador int32 de cada PE(gi, gj)

    // Geração estrutural parametrizável da grade T x T de unidades MAC
    genvar gi, gj; // Índices de geração de linha (gi) e coluna (gj) da malha
    generate
        for (gi = 0; gi < ARRAY_SIZE; gi = gi + 1) begin : gen_row
            for (gj = 0; gj < ARRAY_SIZE; gj = gj + 1) begin : gen_col

                // Conexão horizontal de A: borda esquerda (gj == 0) ou PE vizinho à esquerda (gj - 1)
                if (gj == 0) begin : gen_a_border
                    assign pe_a_in[gi][gj] = a_edge_flat[gi*8 +: 8];
                end else begin : gen_a_internal
                    assign pe_a_in[gi][gj] = pe_a_out[gi][gj-1];
                end

                // Conexão vertical de B: borda superior (gi == 0) ou PE vizinho acima (gi - 1)
                if (gi == 0) begin : gen_b_border
                    assign pe_b_in[gi][gj] = b_edge_flat[gj*8 +: 8];
                end else begin : gen_b_internal
                    assign pe_b_in[gi][gj] = pe_b_out[gi-1][gj];
                end

                // Instância do Elemento de Processamento PE(gi, gj)
                mac_pe u_mac_pe (
                    .clk        (clk),
                    .rst_n      (rst_n),
                    .clear_acc  (clear_acc),
                    .clear_pipe (clear_pipe),
                    .en         (en),
                    .a_in       (pe_a_in[gi][gj]),
                    .b_in       (pe_b_in[gi][gj]),
                    .a_out      (pe_a_out[gi][gj]),
                    .b_out      (pe_b_out[gi][gj]),
                    .acc_out    (pe_acc[gi][gj])
                );

                // Exporta o acumulador int32 do PE(gi, gj) para o vetor linearizado
                assign acc_flat[(gi*ARRAY_SIZE + gj)*32 +: 32] = pe_acc[gi][gj];

            end
        end
    endgenerate

endmodule
