// =============================================================================
// Projeto     : RISC-V RV32I com Controlador DMA e Coprocessador Matricial
// Etapa       : Entrega Parcial II — RTL do Acelerador (Systolic Array) e Testes Unitários
// Módulo      : mac_pe.v
// Professores : Igor Dantas dos Santos Miranda
//               João Alberto Castelo Branco Oliveira
// Grupo 4     : João Vitor Carvalho Lima
//               Matheus Artur Macedo Bomfim
//               Nathalia Miranda
//               Tatiana Caetano Praxedes de Moura
// Descrição   : Elemento de Processamento (PE / Unidade MAC) para Malha Sistólica
//               com fluxo de dados Output-Stationary.
//               Multiplica entradas int8 com sinal (a_in * b_in -> int16),
//               estende o sinal para int32 e acumula em acc_out (módulo 2^32,
//               sem saturação), propagando a_in -> a_out (direita) e
//               b_in -> b_out (baixo) a cada ciclo ativo (en = 1).
//               Suporta clear_acc (zera acumulador e pipeline) e clear_pipe
//               (zera apenas a_out/b_out, preservando acc_out ao longo de K).
// =============================================================================
`timescale 1ns / 1ps

module mac_pe (
    input  wire               clk,        // Clock principal do sistema
    input  wire               rst_n,      // Reset assíncrono ativo em nível baixo
    input  wire               clear_acc,  // Zera acc_out, a_out e b_out (novo bloco C[ti][tj])
    input  wire               clear_pipe, // Zera apenas a_out e b_out (preserva acc_out em tk > 0)
    input  wire               en,         // Habilita operação MAC e propagação sistólica
    input  wire signed [7:0]  a_in,       // Operando A (int8) vindo da borda esquerda ou PE à esquerda
    input  wire signed [7:0]  b_in,       // Operando B (int8) vindo da borda superior ou PE acima
    output reg  signed [7:0]  a_out,      // Operando A registrado e repassado ao PE à direita
    output reg  signed [7:0]  b_out,      // Operando B registrado e repassado ao PE abaixo
    output reg  signed [31:0] acc_out     // Acumulador estacionário int32 da posição C[i][j]
);

    // Multiplicação combinacional int8 x int8 -> int16 com sinal exato
    wire signed [15:0] prod;     // Produto parcial de 16 bits (faixa -16256 a +16384)
    assign prod = a_in * b_in;

    // Extensão de sinal de 16 bits para 32 bits para acumulação em int32
    wire signed [31:0] prod_ext; // Produto com sinal estendido para 32 bits
    assign prod_ext = {{16{prod[15]}}, prod};

    // Bloco sequencial do PE: aplica reset assíncrono, limpezas síncronas
    // (clear_acc / clear_pipe) ou realiza MAC + propagação quando en = 1.
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
