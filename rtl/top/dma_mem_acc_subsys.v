// =============================================================================
// Projeto     : RISC-V RV32I com Controlador DMA e Coprocessador Matricial
// Etapa       : Entrega Parcial III — DMA, Barramento AXI4 e Memória Compartilhada
// Módulo      : dma_mem_acc_subsys.v
// Professores : Ana Julia Fernandes de Oliveira Barros
//               Igor Dantas dos Santos Miranda
//               João Alberto Castelo Branco Oliveira
// Grupo 4     : João Vitor Carvalho Lima
//               Matheus Artur Macedo Bomfim
//               Nathalia Miranda
//               Tatiana Caetano Praxedes de Moura
// Descrição   : Módulo Top-Level de integração da Entrega Parcial III (sem CPU).
//               Interliga os quatro subsistemas principais:
//                 1. axi4_interconnect    (árbitro M1>M0 + decodificador + DECERR)
//                 2. axi4_dmem            (memória compartilhada de 16 KB, S0)
//                 3. dma_controller       (escravo MMIO S1 + mestre AXI4 M1)
//                 4. systolic_accelerator (coprocessador matricial da Entrega II)
//               Expõe a porta Mestre 0 AXI4 (m0_axi_*) e irq_done para o
//               testbench BFM na Entrega III e para a CPU RISC-V na Entrega IV.
// =============================================================================
`timescale 1ns / 1ps

module dma_mem_acc_subsys #(
    parameter integer ARRAY_SIZE   = 4,    // Dimensão T da malha sistólica (2, 4 ou 8)
    parameter integer DMEM_WORDS   = 4096, // Palavras de 32 bits na DMEM (4096 = 16 KB)
    parameter integer DMEM_LATENCY = 1     // Latência em ciclos da DMEM (>= 1)
) (
    input  wire        clk,                // Clock principal do sistema
    input  wire        rst_n,              // Reset assíncrono ativo em nível baixo

    // Sinal de interrupção de conclusão do DMA direcionado à CPU / testbench
    output wire        irq_done,

    // Porta Mestre 0 AXI4 (canais AW, W, B, AR, R — usada pelo BFM ou futura ponte CPU)
    input  wire [31:0] m0_axi_awaddr,
    input  wire [7:0]  m0_axi_awlen,
    input  wire [2:0]  m0_axi_awsize,
    input  wire [1:0]  m0_axi_awburst,
    input  wire        m0_axi_awvalid,
    output wire        m0_axi_awready,

    input  wire [31:0] m0_axi_wdata,
    input  wire [3:0]  m0_axi_wstrb,
    input  wire        m0_axi_wlast,
    input  wire        m0_axi_wvalid,
    output wire        m0_axi_wready,

    output wire [1:0]  m0_axi_bresp,
    output wire        m0_axi_bvalid,
    input  wire        m0_axi_bready,

    input  wire [31:0] m0_axi_araddr,
    input  wire [7:0]  m0_axi_arlen,
    input  wire [2:0]  m0_axi_arsize,
    input  wire [1:0]  m0_axi_arburst,
    input  wire        m0_axi_arvalid,
    output wire        m0_axi_arready,

    output wire [31:0] m0_axi_rdata,
    output wire [1:0]  m0_axi_rresp,
    output wire        m0_axi_rlast,
    output wire        m0_axi_rvalid,
    input  wire        m0_axi_rready
);

    localparam integer IDX_W = $clog2(ARRAY_SIZE); // Largura de índice local dos buffers (0..T-1)

    // Fios internos do barramento AXI4 — Mestre 1 (Controlador DMA -> Interconnect)
    wire [31:0] m1_axi_awaddr;
    wire [7:0]  m1_axi_awlen;
    wire [2:0]  m1_axi_awsize;
    wire [1:0]  m1_axi_awburst;
    wire        m1_axi_awvalid;
    wire        m1_axi_awready;

    wire [31:0] m1_axi_wdata;
    wire [3:0]  m1_axi_wstrb;
    wire        m1_axi_wlast;
    wire        m1_axi_wvalid;
    wire        m1_axi_wready;

    wire [1:0]  m1_axi_bresp;
    wire        m1_axi_bvalid;
    wire        m1_axi_bready;

    wire [31:0] m1_axi_araddr;
    wire [7:0]  m1_axi_arlen;
    wire [2:0]  m1_axi_arsize;
    wire [1:0]  m1_axi_arburst;
    wire        m1_axi_arvalid;
    wire        m1_axi_arready;

    wire [31:0] m1_axi_rdata;
    wire [1:0]  m1_axi_rresp;
    wire        m1_axi_rlast;
    wire        m1_axi_rvalid;
    wire        m1_axi_rready;

    // Fios internos do barramento AXI4 — Escravo 0 (Interconnect -> DMEM Compartilhada)
    wire [31:0] s0_axi_awaddr;
    wire [7:0]  s0_axi_awlen;
    wire [2:0]  s0_axi_awsize;
    wire [1:0]  s0_axi_awburst;
    wire        s0_axi_awvalid;
    wire        s0_axi_awready;

    wire [31:0] s0_axi_wdata;
    wire [3:0]  s0_axi_wstrb;
    wire        s0_axi_wlast;
    wire        s0_axi_wvalid;
    wire        s0_axi_wready;

    wire [1:0]  s0_axi_bresp;
    wire        s0_axi_bvalid;
    wire        s0_axi_bready;

    wire [31:0] s0_axi_araddr;
    wire [7:0]  s0_axi_arlen;
    wire [2:0]  s0_axi_arsize;
    wire [1:0]  s0_axi_arburst;
    wire        s0_axi_arvalid;
    wire        s0_axi_arready;

    wire [31:0] s0_axi_rdata;
    wire [1:0]  s0_axi_rresp;
    wire        s0_axi_rlast;
    wire        s0_axi_rvalid;
    wire        s0_axi_rready;

    // Fios internos do barramento AXI4 — Escravo 1 (Interconnect -> Registradores MMIO do DMA)
    wire [31:0] s1_axi_awaddr;
    wire [7:0]  s1_axi_awlen;
    wire [2:0]  s1_axi_awsize;
    wire [1:0]  s1_axi_awburst;
    wire        s1_axi_awvalid;
    wire        s1_axi_awready;

    wire [31:0] s1_axi_wdata;
    wire [3:0]  s1_axi_wstrb;
    wire        s1_axi_wlast;
    wire        s1_axi_wvalid;
    wire        s1_axi_wready;

    wire [1:0]  s1_axi_bresp;
    wire        s1_axi_bvalid;
    wire        s1_axi_bready;

    wire [31:0] s1_axi_araddr;
    wire [7:0]  s1_axi_arlen;
    wire [2:0]  s1_axi_arsize;
    wire [1:0]  s1_axi_arburst;
    wire        s1_axi_arvalid;
    wire        s1_axi_arready;

    wire [31:0] s1_axi_rdata;
    wire [1:0]  s1_axi_rresp;
    wire        s1_axi_rlast;
    wire        s1_axi_rvalid;
    wire        s1_axi_rready;

    // Interface dedicada DMA <-> Acelerador Sistólico (15 sinais oficiais da Entrega II)
    wire               acc_clear;   // Pulso de 1 ciclo para zerar os acumuladores dos PEs
    wire               acc_start;   // Pulso de 1 ciclo para iniciar a onda sistólica (preserva C)
    wire               acc_busy;    // Indica que a malha sistólica está executando um bloco
    wire               acc_done;    // Pulso de 1 ciclo sinalizando fim dos 3T-2 ciclos do bloco

    wire               buf_a_we;    // Habilita escrita de um elemento int8 no Buffer A
    wire [IDX_W-1:0]   buf_a_row;   // Linha destino no Buffer A (0..T-1)
    wire [IDX_W-1:0]   buf_a_col;   // Coluna destino no Buffer A (0..T-1)
    wire signed [7:0]  buf_a_wdata; // Dado int8 (bits [7:0] da palavra) gravado no Buffer A

    wire               buf_b_we;    // Habilita escrita de um elemento int8 no Buffer B
    wire [IDX_W-1:0]   buf_b_row;   // Linha destino no Buffer B (0..T-1)
    wire [IDX_W-1:0]   buf_b_col;   // Coluna destino no Buffer B (0..T-1)
    wire signed [7:0]  buf_b_wdata; // Dado int8 (bits [7:0] da palavra) gravado no Buffer B

    wire [IDX_W-1:0]   buf_c_row;   // Linha selecionada para leitura combinacional de C (0..T-1)
    wire [IDX_W-1:0]   buf_c_col;   // Coluna selecionada para leitura combinacional de C (0..T-1)
    wire signed [31:0] buf_c_rdata; // Resultado acumulado int32 lido diretamente da malha

    // -------------------------------------------------------------------------
    // Instância 1: Matriz de Interconexão AXI4 (Árbitro + Decodificador)
    // -------------------------------------------------------------------------
    axi4_interconnect u_interconnect (
        .clk            (clk),
        .rst_n          (rst_n),

        .m0_axi_awaddr  (m0_axi_awaddr),
        .m0_axi_awlen   (m0_axi_awlen),
        .m0_axi_awsize  (m0_axi_awsize),
        .m0_axi_awburst (m0_axi_awburst),
        .m0_axi_awvalid (m0_axi_awvalid),
        .m0_axi_awready (m0_axi_awready),
        .m0_axi_wdata   (m0_axi_wdata),
        .m0_axi_wstrb   (m0_axi_wstrb),
        .m0_axi_wlast   (m0_axi_wlast),
        .m0_axi_wvalid  (m0_axi_wvalid),
        .m0_axi_wready  (m0_axi_wready),
        .m0_axi_bresp   (m0_axi_bresp),
        .m0_axi_bvalid  (m0_axi_bvalid),
        .m0_axi_bready  (m0_axi_bready),
        .m0_axi_araddr  (m0_axi_araddr),
        .m0_axi_arlen   (m0_axi_arlen),
        .m0_axi_arsize  (m0_axi_arsize),
        .m0_axi_arburst (m0_axi_arburst),
        .m0_axi_arvalid (m0_axi_arvalid),
        .m0_axi_arready (m0_axi_arready),
        .m0_axi_rdata   (m0_axi_rdata),
        .m0_axi_rresp   (m0_axi_rresp),
        .m0_axi_rlast   (m0_axi_rlast),
        .m0_axi_rvalid  (m0_axi_rvalid),
        .m0_axi_rready  (m0_axi_rready),

        .m1_axi_awaddr  (m1_axi_awaddr),
        .m1_axi_awlen   (m1_axi_awlen),
        .m1_axi_awsize  (m1_axi_awsize),
        .m1_axi_awburst (m1_axi_awburst),
        .m1_axi_awvalid (m1_axi_awvalid),
        .m1_axi_awready (m1_axi_awready),
        .m1_axi_wdata   (m1_axi_wdata),
        .m1_axi_wstrb   (m1_axi_wstrb),
        .m1_axi_wlast   (m1_axi_wlast),
        .m1_axi_wvalid  (m1_axi_wvalid),
        .m1_axi_wready  (m1_axi_wready),
        .m1_axi_bresp   (m1_axi_bresp),
        .m1_axi_bvalid  (m1_axi_bvalid),
        .m1_axi_bready  (m1_axi_bready),
        .m1_axi_araddr  (m1_axi_araddr),
        .m1_axi_arlen   (m1_axi_arlen),
        .m1_axi_arsize  (m1_axi_arsize),
        .m1_axi_arburst (m1_axi_arburst),
        .m1_axi_arvalid (m1_axi_arvalid),
        .m1_axi_arready (m1_axi_arready),
        .m1_axi_rdata   (m1_axi_rdata),
        .m1_axi_rresp   (m1_axi_rresp),
        .m1_axi_rlast   (m1_axi_rlast),
        .m1_axi_rvalid  (m1_axi_rvalid),
        .m1_axi_rready  (m1_axi_rready),

        .s0_axi_awaddr  (s0_axi_awaddr),
        .s0_axi_awlen   (s0_axi_awlen),
        .s0_axi_awsize  (s0_axi_awsize),
        .s0_axi_awburst (s0_axi_awburst),
        .s0_axi_awvalid (s0_axi_awvalid),
        .s0_axi_awready (s0_axi_awready),
        .s0_axi_wdata   (s0_axi_wdata),
        .s0_axi_wstrb   (s0_axi_wstrb),
        .s0_axi_wlast   (s0_axi_wlast),
        .s0_axi_wvalid  (s0_axi_wvalid),
        .s0_axi_wready  (s0_axi_wready),
        .s0_axi_bresp   (s0_axi_bresp),
        .s0_axi_bvalid  (s0_axi_bvalid),
        .s0_axi_bready  (s0_axi_bready),
        .s0_axi_araddr  (s0_axi_araddr),
        .s0_axi_arlen   (s0_axi_arlen),
        .s0_axi_arsize  (s0_axi_arsize),
        .s0_axi_arburst (s0_axi_arburst),
        .s0_axi_arvalid (s0_axi_arvalid),
        .s0_axi_arready (s0_axi_arready),
        .s0_axi_rdata   (s0_axi_rdata),
        .s0_axi_rresp   (s0_axi_rresp),
        .s0_axi_rlast   (s0_axi_rlast),
        .s0_axi_rvalid  (s0_axi_rvalid),
        .s0_axi_rready  (s0_axi_rready),

        .s1_axi_awaddr  (s1_axi_awaddr),
        .s1_axi_awlen   (s1_axi_awlen),
        .s1_axi_awsize  (s1_axi_awsize),
        .s1_axi_awburst (s1_axi_awburst),
        .s1_axi_awvalid (s1_axi_awvalid),
        .s1_axi_awready (s1_axi_awready),
        .s1_axi_wdata   (s1_axi_wdata),
        .s1_axi_wstrb   (s1_axi_wstrb),
        .s1_axi_wlast   (s1_axi_wlast),
        .s1_axi_wvalid  (s1_axi_wvalid),
        .s1_axi_wready  (s1_axi_wready),
        .s1_axi_bresp   (s1_axi_bresp),
        .s1_axi_bvalid  (s1_axi_bvalid),
        .s1_axi_bready  (s1_axi_bready),
        .s1_axi_araddr  (s1_axi_araddr),
        .s1_axi_arlen   (s1_axi_arlen),
        .s1_axi_arsize  (s1_axi_arsize),
        .s1_axi_arburst (s1_axi_arburst),
        .s1_axi_arvalid (s1_axi_arvalid),
        .s1_axi_arready (s1_axi_arready),
        .s1_axi_rdata   (s1_axi_rdata),
        .s1_axi_rresp   (s1_axi_rresp),
        .s1_axi_rlast   (s1_axi_rlast),
        .s1_axi_rvalid  (s1_axi_rvalid),
        .s1_axi_rready  (s1_axi_rready)
    );

    // -------------------------------------------------------------------------
    // Instância 2: Memória de Dados Compartilhada AXI4 (DMEM — 16 KB)
    // -------------------------------------------------------------------------
    axi4_dmem #(
        .MEM_WORDS (DMEM_WORDS),
        .LATENCY   (DMEM_LATENCY)
    ) u_dmem (
        .clk           (clk),
        .rst_n         (rst_n),
        .s_axi_awaddr  (s0_axi_awaddr),
        .s_axi_awlen   (s0_axi_awlen),
        .s_axi_awsize  (s0_axi_awsize),
        .s_axi_awburst (s0_axi_awburst),
        .s_axi_awvalid (s0_axi_awvalid),
        .s_axi_awready (s0_axi_awready),
        .s_axi_wdata   (s0_axi_wdata),
        .s_axi_wstrb   (s0_axi_wstrb),
        .s_axi_wlast   (s0_axi_wlast),
        .s_axi_wvalid  (s0_axi_wvalid),
        .s_axi_wready  (s0_axi_wready),
        .s_axi_bresp   (s0_axi_bresp),
        .s_axi_bvalid  (s0_axi_bvalid),
        .s_axi_bready  (s0_axi_bready),
        .s_axi_araddr  (s0_axi_araddr),
        .s_axi_arlen   (s0_axi_arlen),
        .s_axi_arsize  (s0_axi_arsize),
        .s_axi_arburst (s0_axi_arburst),
        .s_axi_arvalid (s0_axi_arvalid),
        .s_axi_arready (s0_axi_arready),
        .s_axi_rdata   (s0_axi_rdata),
        .s_axi_rresp   (s0_axi_rresp),
        .s_axi_rlast   (s0_axi_rlast),
        .s_axi_rvalid  (s0_axi_rvalid),
        .s_axi_rready  (s0_axi_rready)
    );

    // -------------------------------------------------------------------------
    // Instância 3: Controlador DMA Autônomo
    // -------------------------------------------------------------------------
    dma_controller #(
        .ARRAY_SIZE (ARRAY_SIZE)
    ) u_dma (
        .clk           (clk),
        .rst_n         (rst_n),
        .irq_done      (irq_done),

        .s_axi_awaddr  (s1_axi_awaddr),
        .s_axi_awlen   (s1_axi_awlen),
        .s_axi_awsize  (s1_axi_awsize),
        .s_axi_awburst (s1_axi_awburst),
        .s_axi_awvalid (s1_axi_awvalid),
        .s_axi_awready (s1_axi_awready),
        .s_axi_wdata   (s1_axi_wdata),
        .s_axi_wstrb   (s1_axi_wstrb),
        .s_axi_wlast   (s1_axi_wlast),
        .s_axi_wvalid  (s1_axi_wvalid),
        .s_axi_wready  (s1_axi_wready),
        .s_axi_bresp   (s1_axi_bresp),
        .s_axi_bvalid  (s1_axi_bvalid),
        .s_axi_bready  (s1_axi_bready),
        .s_axi_araddr  (s1_axi_araddr),
        .s_axi_arlen   (s1_axi_arlen),
        .s_axi_arsize  (s1_axi_arsize),
        .s_axi_arburst (s1_axi_arburst),
        .s_axi_arvalid (s1_axi_arvalid),
        .s_axi_arready (s1_axi_arready),
        .s_axi_rdata   (s1_axi_rdata),
        .s_axi_rresp   (s1_axi_rresp),
        .s_axi_rlast   (s1_axi_rlast),
        .s_axi_rvalid  (s1_axi_rvalid),
        .s_axi_rready  (s1_axi_rready),

        .m_axi_awaddr  (m1_axi_awaddr),
        .m_axi_awlen   (m1_axi_awlen),
        .m_axi_awsize  (m1_axi_awsize),
        .m_axi_awburst (m1_axi_awburst),
        .m_axi_awvalid (m1_axi_awvalid),
        .m_axi_awready (m1_axi_awready),
        .m_axi_wdata   (m1_axi_wdata),
        .m_axi_wstrb   (m1_axi_wstrb),
        .m_axi_wlast   (m1_axi_wlast),
        .m_axi_wvalid  (m1_axi_wvalid),
        .m_axi_wready  (m1_axi_wready),
        .m_axi_bresp   (m1_axi_bresp),
        .m_axi_bvalid  (m1_axi_bvalid),
        .m_axi_bready  (m1_axi_bready),
        .m_axi_araddr  (m1_axi_araddr),
        .m_axi_arlen   (m1_axi_arlen),
        .m_axi_arsize  (m1_axi_arsize),
        .m_axi_arburst (m1_axi_arburst),
        .m_axi_arvalid (m1_axi_arvalid),
        .m_axi_arready (m1_axi_arready),
        .m_axi_rdata   (m1_axi_rdata),
        .m_axi_rresp   (m1_axi_rresp),
        .m_axi_rlast   (m1_axi_rlast),
        .m_axi_rvalid  (m1_axi_rvalid),
        .m_axi_rready  (m1_axi_rready),

        .acc_clear     (acc_clear),
        .acc_start     (acc_start),
        .acc_busy      (acc_busy),
        .acc_done      (acc_done),
        .buf_a_we      (buf_a_we),
        .buf_a_row     (buf_a_row),
        .buf_a_col     (buf_a_col),
        .buf_a_wdata   (buf_a_wdata),
        .buf_b_we      (buf_b_we),
        .buf_b_row     (buf_b_row),
        .buf_b_col     (buf_b_col),
        .buf_b_wdata   (buf_b_wdata),
        .buf_c_row     (buf_c_row),
        .buf_c_col     (buf_c_col),
        .buf_c_rdata   (buf_c_rdata)
    );

    // -------------------------------------------------------------------------
    // Instância 4: Acelerador Matricial Sistólico (Entrega II)
    // -------------------------------------------------------------------------
    systolic_accelerator #(
        .ARRAY_SIZE (ARRAY_SIZE)
    ) u_accelerator (
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

endmodule
