// =============================================================================
// Módulo: mac_pe.v
// Descrição: Elemento de Processamento (PE / Unidade MAC) para Malha Sistólica
//            com fluxo de dados Output-Stationary.
//
// Operação:
//   - Entradas A (esquerda) e B (topo) em formato int8 com sinal (signed [7:0]).
//   - Produto A * B em int16 com sinal (signed [15:0]), acumulado em int32
//     com sinal (signed [31:0]) sem saturação (aritmética modular 2^32).
//   - Quando habilitado (en = 1), atualiza o acumulador acc_out e propaga
//     a_in -> a_out (para a direita) e b_in -> b_out (para baixo) na borda
//     de subida do clock.
//   - clear_acc zera o acumulador e os registradores de propagação.
//   - clear_pipe zera apenas os registradores de propagação (a_out, b_out),
//     preservando acc_out para acumulação de múltiplos blocos ao longo de K.
// =============================================================================
`timescale 1ns / 1ps

module mac_pe (
    input  wire               clk,
    input  wire               rst_n,
    input  wire               clear_acc,
    input  wire               clear_pipe,
    input  wire               en,
    input  wire signed [7:0]  a_in,
    input  wire signed [7:0]  b_in,
    output reg  signed [7:0]  a_out,
    output reg  signed [7:0]  b_out,
    output reg  signed [31:0] acc_out
);

    // Multiplicação int8 x int8 -> int16 com extensão de sinal exata
    wire signed [15:0] prod;
    assign prod = a_in * b_in;

    // Extensão de sinal de 16 bits para 32 bits para soma no acumulador int32
    wire signed [31:0] prod_ext;
    assign prod_ext = {{16{prod[15]}}, prod};

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            a_out   <= 8'sd0;
            b_out   <= 8'sd0;
            acc_out <= 32'sd0;
        end else if (clear_acc) begin
            a_out   <= 8'sd0;
            b_out   <= 8'sd0;
            acc_out <= 32'sd0;
        end else if (clear_pipe) begin
            a_out   <= 8'sd0;
            b_out   <= 8'sd0;
        end else if (en) begin
            a_out   <= a_in;
            b_out   <= b_in;
            acc_out <= acc_out + prod_ext;
        end
    end

endmodule
