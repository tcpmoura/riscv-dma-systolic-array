// =============================================================================
// Projeto     : RISC-V RV32I com Controlador DMA e Coprocessador Matricial
// Etapa       : Entrega Parcial IV — Integração Top-Level e Firmware
// Módulo      : riscv_dma_soc_top.v
// Professores : Ana Julia Fernandes de Oliveira Barros
//               Igor Dantas dos Santos Miranda
//               João Alberto Castelo Branco Oliveira
// Grupo 4     : João Vitor Carvalho Lima
//               Matheus Artur Macedo Bomfim
//               Nathalia Miranda
//               Tatiana Caetano Praxedes de Moura
// Descrição   : Módulo Top-Level completo do SoC (arquitetura Harvard),
//               instanciando e interligando todos os subsistemas do projeto:
//                 1. rv32i_core         (CPU RISC-V RV32I + IMEM interna)
//                 2. cpu_axi4_bridge    (Ponte valid/ready -> Mestre 0 AXI4)
//                 3. dma_mem_acc_subsys (Interconnect AXI4, DMEM 16 KB,
//                                        Controlador DMA e Systolic Array)
//
// Parametrização:
//   - ARRAY_SIZE   : Dimensão física T da malha sistólica (2, 4 ou 8).
//   - DMEM_WORDS   : Profundidade da memória compartilhada (padrão 4096 = 16 KB).
//   - DMEM_LATENCY : Latência em ciclos da DMEM (>= 1).
//   - IMEM_DEPTH   : Profundidade da memória de instruções em palavras de 32b.
//   - IMEM_FILE    : Arquivo .hex do firmware carregado na IMEM da CPU.
// =============================================================================
`timescale 1ns / 1ps

module riscv_dma_soc_top #(
    parameter integer ARRAY_SIZE   = 4,
    parameter integer DMEM_WORDS   = 4096,
    parameter integer DMEM_LATENCY = 1,
    parameter integer IMEM_DEPTH   = 256,
    parameter         IMEM_FILE    = "sw/firmware/matmul_dma.hex"
) (
    input  wire        clk,       // Clock principal do sistema (borda de subida)
    input  wire        rst_n,     // Reset global assíncrono ativo em nível baixo
    output wire [31:0] debug_pc,  // Endereço atual do Program Counter (PC) da CPU
    output wire        halted,    // 1 quando o firmware encerra normalmente via EBREAK
    output wire        trap,      // 1 em exceção da CPU ou erro AXI4 (SLVERR/DECERR)
    output wire        retire,    // Pulso de 1 ciclo ao concluir cada instrução da CPU
    output wire        irq_done   // Interrupção de conclusão emitida pelo controlador DMA
);

    // -------------------------------------------------------------------------
    // Fios internos: Interface nativa de dados da CPU (rv32i_core <-> ponte)
    // -------------------------------------------------------------------------
    wire        cpu_dmem_valid;
    wire        cpu_dmem_ready;
    wire        cpu_dmem_write;
    wire [31:0] cpu_dmem_addr;
    wire [31:0] cpu_dmem_wdata;
    wire [3:0]  cpu_dmem_wstrb;
    wire [31:0] cpu_dmem_rdata;
    wire        cpu_dmem_error;

    // -------------------------------------------------------------------------
    // Fios internos: Porta Mestre 0 AXI4 (cpu_axi4_bridge <-> dma_mem_acc_subsys)
    // -------------------------------------------------------------------------
    wire [31:0] m0_axi_awaddr;
    wire [7:0]  m0_axi_awlen;
    wire [2:0]  m0_axi_awsize;
    wire [1:0]  m0_axi_awburst;
    wire        m0_axi_awvalid;
    wire        m0_axi_awready;

    wire [31:0] m0_axi_wdata;
    wire [3:0]  m0_axi_wstrb;
    wire        m0_axi_wlast;
    wire        m0_axi_wvalid;
    wire        m0_axi_wready;

    wire [1:0]  m0_axi_bresp;
    wire        m0_axi_bvalid;
    wire        m0_axi_bready;

    wire [31:0] m0_axi_araddr;
    wire [7:0]  m0_axi_arlen;
    wire [2:0]  m0_axi_arsize;
    wire [1:0]  m0_axi_arburst;
    wire        m0_axi_arvalid;
    wire        m0_axi_arready;

    wire [31:0] m0_axi_rdata;
    wire [1:0]  m0_axi_rresp;
    wire        m0_axi_rlast;
    wire        m0_axi_rvalid;
    wire        m0_axi_rready;

    // -------------------------------------------------------------------------
    // 1. Processador RISC-V RV32I (Arquitetura Harvard com IMEM interna)
    // -------------------------------------------------------------------------
    rv32i_core #(
        .IMEM_DEPTH (IMEM_DEPTH),
        .IMEM_FILE  (IMEM_FILE)
    ) u_cpu (
        .clk        (clk),
        .rst_n      (rst_n),
        .dmem_valid (cpu_dmem_valid),
        .dmem_ready (cpu_dmem_ready),
        .dmem_write (cpu_dmem_write),
        .dmem_addr  (cpu_dmem_addr),
        .dmem_wdata (cpu_dmem_wdata),
        .dmem_wstrb (cpu_dmem_wstrb),
        .dmem_rdata (cpu_dmem_rdata),
        .dmem_error (cpu_dmem_error),
        .debug_pc   (debug_pc),
        .halted     (halted),
        .trap       (trap),
        .retire     (retire)
    );

    // -------------------------------------------------------------------------
    // 2. Ponte Síncrona CPU -> Mestre 0 AXI4
    // -------------------------------------------------------------------------
    cpu_axi4_bridge u_cpu_bridge (
        .clk           (clk),
        .rst_n         (rst_n),
        .cpu_valid     (cpu_dmem_valid),
        .cpu_ready     (cpu_dmem_ready),
        .cpu_write     (cpu_dmem_write),
        .cpu_addr      (cpu_dmem_addr),
        .cpu_wdata     (cpu_dmem_wdata),
        .cpu_wstrb     (cpu_dmem_wstrb),
        .cpu_rdata     (cpu_dmem_rdata),
        .cpu_error     (cpu_dmem_error),
        .m_axi_awaddr  (m0_axi_awaddr),
        .m_axi_awlen   (m0_axi_awlen),
        .m_axi_awsize  (m0_axi_awsize),
        .m_axi_awburst (m0_axi_awburst),
        .m_axi_awvalid (m0_axi_awvalid),
        .m_axi_awready (m0_axi_awready),
        .m_axi_wdata   (m0_axi_wdata),
        .m_axi_wstrb   (m0_axi_wstrb),
        .m_axi_wlast   (m0_axi_wlast),
        .m_axi_wvalid  (m0_axi_wvalid),
        .m_axi_wready  (m0_axi_wready),
        .m_axi_bresp   (m0_axi_bresp),
        .m_axi_bvalid  (m0_axi_bvalid),
        .m_axi_bready  (m0_axi_bready),
        .m_axi_araddr  (m0_axi_araddr),
        .m_axi_arlen   (m0_axi_arlen),
        .m_axi_arsize  (m0_axi_arsize),
        .m_axi_arburst (m0_axi_arburst),
        .m_axi_arvalid (m0_axi_arvalid),
        .m_axi_arready (m0_axi_arready),
        .m_axi_rdata   (m0_axi_rdata),
        .m_axi_rresp   (m0_axi_rresp),
        .m_axi_rlast   (m0_axi_rlast),
        .m_axi_rvalid  (m0_axi_rvalid),
        .m_axi_rready  (m0_axi_rready)
    );

    // -------------------------------------------------------------------------
    // 3. Subsistema Integrado: Interconnect AXI4 + DMEM 16 KB + DMA + Acelerador
    // -------------------------------------------------------------------------
    dma_mem_acc_subsys #(
        .ARRAY_SIZE   (ARRAY_SIZE),
        .DMEM_WORDS   (DMEM_WORDS),
        .DMEM_LATENCY (DMEM_LATENCY)
    ) u_subsys (
        .clk            (clk),
        .rst_n          (rst_n),
        .irq_done       (irq_done),
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
        .m0_axi_rready  (m0_axi_rready)
    );

endmodule
