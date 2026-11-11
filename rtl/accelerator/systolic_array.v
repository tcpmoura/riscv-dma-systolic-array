// =============================================================================
// Módulo: systolic_array.v
// Descrição: Malha Sistólica Bidimensional Parametrizável (ARRAY_SIZE x ARRAY_SIZE)
//            composta por unidades MAC (mac_pe) em arquitetura Output-Stationary.
//
// Parâmetros:
//   - ARRAY_SIZE: Dimensão física T da malha quadrada (ex.: 2, 4 ou 8).
//
// Interconexão:
//   - Os elementos de A entram pela borda esquerda (a_edge_flat) e avançam
//     horizontalmente (coluna j -> j+1) a cada ciclo ativo (en = 1).
//   - Os elementos de B entram pela borda superior (b_edge_flat) e avançam
//     verticalmente (linha i -> i+1) a cada ciclo ativo (en = 1).
//   - Os acumuladores C[i][j] (int32) permanecem estacionários em cada PE(i,j)
//     e são expostos no barramento acc_flat para leitura combinacional.
// =============================================================================
`timescale 1ns / 1ps

module systolic_array #(
    parameter integer ARRAY_SIZE = 4
) (
    input  wire                                clk,
    input  wire                                rst_n,
    input  wire                                clear_acc,
    input  wire                                clear_pipe,
    input  wire                                en,
    input  wire [ARRAY_SIZE*8-1:0]             a_edge_flat,
    input  wire [ARRAY_SIZE*8-1:0]             b_edge_flat,
    output wire [ARRAY_SIZE*ARRAY_SIZE*32-1:0] acc_flat
);

    // Fios internos de interconexão entre os PEs da malha T x T
    wire signed [7:0]  pe_a_in  [0:ARRAY_SIZE-1][0:ARRAY_SIZE-1];
    wire signed [7:0]  pe_b_in  [0:ARRAY_SIZE-1][0:ARRAY_SIZE-1];
    wire signed [7:0]  pe_a_out [0:ARRAY_SIZE-1][0:ARRAY_SIZE-1];
    wire signed [7:0]  pe_b_out [0:ARRAY_SIZE-1][0:ARRAY_SIZE-1];
    wire signed [31:0] pe_acc   [0:ARRAY_SIZE-1][0:ARRAY_SIZE-1];

    genvar gi, gj;
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
