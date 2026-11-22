// =============================================================================
// Projeto     : RISC-V RV32I com Controlador DMA e Coprocessador Matricial
// Etapa       : Entrega Parcial III — DMA, Barramento AXI4 e Memória Compartilhada
// Módulo      : tb_dma_mem_acc.sv
// Professores : Ana Julia Fernandes de Oliveira Barros
//               Igor Dantas dos Santos Miranda
//               João Alberto Castelo Branco Oliveira
// Grupo 4     : João Vitor Carvalho Lima
//               Matheus Artur Macedo Bomfim
//               Nathalia Miranda
//               Tatiana Caetano Praxedes de Moura
// Descrição   : Testbench Unitário Autoverificável em SystemVerilog para o
//               subsistema integrado DMA + Barramento AXI4 + DMEM + Acelerador
//               Sistólico (integração sem a CPU, controlada via BFM na porta M0).
//
// Escopo de Verificação:
//   - Instancia 4 configurações físicas via subsys_bfm_harness:
//       * ARRAY_SIZE = 2, DMEM_LATENCY = 1 (bateria principal)
//       * ARRAY_SIZE = 4, DMEM_LATENCY = 1 (bateria principal)
//       * ARRAY_SIZE = 8, DMEM_LATENCY = 1 (bateria principal)
//       * ARRAY_SIZE = 4, DMEM_LATENCY = 3 (2 subtestes focados em wait states)
//   - Monitora continuamente a conformidade AXI4 (estabilidade de VALID/dados
//     durante READY=0, ausência de X/Z, fronteira 4 KB em AR/AW, escrita única
//     por elemento válido de C e ausência de escritas fora de C).
// =============================================================================
`timescale 1ns / 1ps

// -----------------------------------------------------------------------------
// Harness parametrizável: encapsula um DUT (dma_mem_acc_subsys), o monitor
// contínuo AXI4, as tarefas BFM da CPU (M0) e a suíte de testes unitários.
// -----------------------------------------------------------------------------
module subsys_bfm_harness #(
    parameter int ARRAY_SIZE   = 4, // Dimensão T da malha sistólica testada (2, 4 ou 8)
    parameter int DMEM_LATENCY = 1  // Latência em ciclos configurada na DMEM (>= 1)
) (
    input logic clk                 // Clock compartilhado fornecido pelo top-level do TB
);

    // Endereços MMIO oficiais dos 12 registradores do DMA (escravo S1)
    localparam logic [31:0] REG_DMA_CTRL          = 32'h4000_0000; // Controle (START/IRQ_EN/SOFT_RST)
    localparam logic [31:0] REG_DMA_STATUS        = 32'h4000_0004; // Status (BUSY/DONE/ERR, W1C)
    localparam logic [31:0] REG_DMA_ADDR_A        = 32'h4000_0008; // Endereço base da matriz A
    localparam logic [31:0] REG_DMA_ADDR_B        = 32'h4000_000C; // Endereço base da matriz B
    localparam logic [31:0] REG_DMA_ADDR_C        = 32'h4000_0010; // Endereço base da matriz C
    localparam logic [31:0] REG_DMA_DIM_M         = 32'h4000_0014; // Dimensão M (linhas de A/C)
    localparam logic [31:0] REG_DMA_DIM_K         = 32'h4000_0018; // Dimensão K (colunas A / linhas B)
    localparam logic [31:0] REG_DMA_DIM_N         = 32'h4000_001C; // Dimensão N (colunas de B/C)
    localparam logic [31:0] REG_PERF_TOTAL_CYCLES = 32'h4000_0020; // Ciclos totais START -> DONE
    localparam logic [31:0] REG_PERF_COMP_CYCLES  = 32'h4000_0024; // Ciclos com acc_busy = 1
    localparam logic [31:0] REG_PERF_MEM_READS    = 32'h4000_0028; // Palavras lidas da DMEM
    localparam logic [31:0] REG_HW_ARRAY_SIZE     = 32'h4000_002C; // Dimensão física T da malha

    // Sinais de controle global e interrupção conectados ao DUT
    logic        rst_n;    // Reset assíncrono ativo em nível baixo dirigido pelo TB
    logic        irq_done; // Interrupção de conclusão observada na saída do DUT

    // Sinais AXI4 da porta Mestre 0 (dirigidos pelas tarefas BFM simulando a CPU)
    logic [31:0] m0_axi_awaddr;
    logic [7:0]  m0_axi_awlen;
    logic [2:0]  m0_axi_awsize;
    logic [1:0]  m0_axi_awburst;
    logic        m0_axi_awvalid;
    logic        m0_axi_awready;

    logic [31:0] m0_axi_wdata;
    logic [3:0]  m0_axi_wstrb;
    logic        m0_axi_wlast;
    logic        m0_axi_wvalid;
    logic        m0_axi_wready;

    logic [1:0]  m0_axi_bresp;
    logic        m0_axi_bvalid;
    logic        m0_axi_bready;

    logic [31:0] m0_axi_araddr;
    logic [7:0]  m0_axi_arlen;
    logic [2:0]  m0_axi_arsize;
    logic [1:0]  m0_axi_arburst;
    logic        m0_axi_arvalid;
    logic        m0_axi_arready;

    logic [31:0] m0_axi_rdata;
    logic [1:0]  m0_axi_rresp;
    logic        m0_axi_rlast;
    logic        m0_axi_rvalid;
    logic        m0_axi_rready;

    // Instância sob teste (DUT): subsistema integrado da Entrega Parcial III
    dma_mem_acc_subsys #(
        .ARRAY_SIZE   (ARRAY_SIZE),
        .DMEM_WORDS   (4096),
        .DMEM_LATENCY (DMEM_LATENCY)
    ) u_dut (
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

    // Contadores de placar desta instância do harness
    int errors;          // Total de erros acumulados nesta instância
    int subtests_passed; // Quantidade de subtestes aprovados (PASS)
    int subtests_total;  // Quantidade total de subtestes executados

    // Inicializa os sinais da porta M0 em estado seguro no instante t = 0
    initial begin
        rst_n          = 1'b0;
        m0_axi_awaddr  = 32'd0;
        m0_axi_awlen   = 8'd0;
        m0_axi_awsize  = 3'b010;
        m0_axi_awburst = 2'b01;
        m0_axi_awvalid = 1'b0;
        m0_axi_wdata   = 32'd0;
        m0_axi_wstrb   = 4'b0000;
        m0_axi_wlast   = 1'b0;
        m0_axi_wvalid  = 1'b0;
        m0_axi_bready  = 1'b0;
        m0_axi_araddr  = 32'd0;
        m0_axi_arlen   = 8'd0;
        m0_axi_arsize  = 3'b010;
        m0_axi_arburst = 2'b01;
        m0_axi_arvalid = 1'b0;
        m0_axi_rready  = 1'b0;
    end

    // -------------------------------------------------------------------------
    // Monitor Contínuo de Conformidade AXI4 (M0, M1, S0 e S1)
    // Garante ausência de X/Z em sinais de controle, estabilidade durante stall,
    // imutabilidade de VALID antes do handshake READY e respeito estrito à
    // fronteira de 4 KB (addr[11:0] + 4*(LEN+1) <= 4096) em AR e AW.
    // -------------------------------------------------------------------------
    // Registros do ciclo anterior para verificar estabilidade quando VALID && !READY
    logic        prev_m1_awvalid, prev_m1_awready; // Estado anterior do canal AW do DMA (M1)
    logic [31:0] prev_m1_awaddr;
    logic [7:0]  prev_m1_awlen;

    logic        prev_m1_wvalid, prev_m1_wready;   // Estado anterior do canal W do DMA (M1)
    logic [31:0] prev_m1_wdata;
    logic        prev_m1_wlast;

    logic        prev_m1_arvalid, prev_m1_arready; // Estado anterior do canal AR do DMA (M1)
    logic [31:0] prev_m1_araddr;
    logic [7:0]  prev_m1_arlen;

    logic        prev_s0_rvalid, prev_s0_rready;   // Estado anterior do canal R da DMEM (S0)
    logic [31:0] prev_s0_rdata;
    logic [1:0]  prev_s0_rresp;
    logic        prev_s0_rlast;

    logic        prev_s0_bvalid, prev_s0_bready;   // Estado anterior do canal B da DMEM (S0)
    logic [1:0]  prev_s0_bresp;

    logic        prev_s1_rvalid, prev_s1_rready;   // Estado anterior do canal R do MMIO (S1)
    logic [31:0] prev_s1_rdata;
    logic [1:0]  prev_s1_rresp;
    logic        prev_s1_rlast;

    logic        prev_s1_bvalid, prev_s1_bready;   // Estado anterior do canal B do MMIO (S1)
    logic [1:0]  prev_s1_bresp;

    logic        prev_m0_rvalid, prev_m0_rready;   // Estado anterior do canal R entregue a M0
    logic [31:0] prev_m0_rdata;
    logic [1:0]  prev_m0_rresp;
    logic        prev_m0_rlast;

    logic        prev_m0_bvalid, prev_m0_bready;   // Estado anterior do canal B entregue a M0
    logic [1:0]  prev_m0_bresp;

    // Contadores de observabilidade de fronteira 4 KB e rajadas do DMA (M1)
    int          m1_4kb_violations;   // Violações detectadas (addr[11:0] + 4*(LEN+1) > 4096)
    int          m1_ar_burst_cnt;     // Total de handshakes AR concluídos por M1
    int          m1_ar_4kb_end_cnt;   // Rajadas AR que terminaram exatamente no limite 4096
    int          m1_ar_4kb_start_cnt; // Rajadas AR que iniciaram exatamente em offset 0x000
    int          m1_aw_burst_cnt;     // Total de handshakes AW concluídos por M1
    int          m1_aw_4kb_end_cnt;   // Rajadas AW que terminaram exatamente no limite 4096
    int          m1_aw_4kb_start_cnt; // Rajadas AW que iniciaram exatamente em offset 0x000
    int          m1_any_valid_cycles; // Ciclos com qualquer VALID (AR/AW/W) ativo em M1

    // Contadores de ciclos de espera (VALID=1 && READY=0) para comprovar cobertura de stalls
    int          m1_ar_wait_cycles;   // Ciclos de espera sofridos por M1 em AR
    int          m1_aw_wait_cycles;   // Ciclos de espera sofridos por M1 em AW
    int          m1_w_wait_cycles;    // Ciclos de espera sofridos por M1 em W
    int          s0_r_wait_cycles;    // Ciclos de retenção de RREADY=0 na DMEM (S0)
    int          s0_b_wait_cycles;    // Ciclos de retenção de BREADY=0 na DMEM (S0)
    int          s1_r_wait_cycles;    // Ciclos de retenção de RREADY=0 no MMIO (S1)
    int          s1_b_wait_cycles;    // Ciclos de retenção de BREADY=0 no MMIO (S1)

    // Observabilidade de respostas de erro AXI4 recebidas diretamente pelo DMA (distinguindo SLVERR de DECERR)
    int          m1_rresp_slverr_cnt;     // Handshakes R de M1 com RRESP == SLVERR (2'b10)
    int          m1_rresp_decerr_cnt;     // Handshakes R de M1 com RRESP == DECERR (2'b11)
    int          m1_bresp_slverr_cnt;     // Handshakes B de M1 com BRESP == SLVERR (2'b10)
    int          m1_bresp_decerr_cnt;     // Handshakes B de M1 com BRESP == DECERR (2'b11)
    bit          expect_injected_axi_err; // Libera checagem de alinhamento durante Teste 12 (provocação SLVERR)

    // Rastreador de escrita única por elemento válido de C no canal W do DMA (M1)
    logic [31:0] m1_wr_track_addr;        // Endereço corrente rastreado para cada beat W do DMA
    int          m1_w_beats_total;        // Total de palavras escritas por M1 na operação atual
    int          m1_wr_oob_cnt;           // Escritas de M1 fora da faixa da DMEM (0..16 KB)
    int          m1_wr_hit_cnt [0:4095];  // Contagem de escritas de M1 por palavra da DMEM (0..4095)

    // Snapshot das palavras completas de A/B antes do START (somente testbench).
    logic [31:0] input_a_before [0:1023];
    logic [31:0] input_b_before [0:1023];
    int input_a_word, input_b_word, input_a_count, input_b_count;
    bit input_snapshot_valid = 1'b0;

    task automatic snapshot_inputs(input int M, input int K, input int N,
                                   input logic [31:0] base_a, input logic [31:0] base_b);
        int idx;
        begin
            input_a_word = base_a >> 2;
            input_b_word = base_b >> 2;
            input_a_count = M * K;
            input_b_count = K * N;
            for (idx = 0; idx < input_a_count; idx = idx + 1)
                input_a_before[idx] = u_dut.u_dmem.memory[input_a_word + idx];
            for (idx = 0; idx < input_b_count; idx = idx + 1)
                input_b_before[idx] = u_dut.u_dmem.memory[input_b_word + idx];
            input_snapshot_valid = 1'b1;
        end
    endtask

    task automatic verify_inputs_preserved(input string ctx);
        int idx;
        begin
            if (!input_snapshot_valid) begin
                $display("[FAIL] %s: snapshot de A/B ausente", ctx);
                errors = errors + 1;
            end else begin
                for (idx = 0; idx < input_a_count; idx = idx + 1)
                    if (u_dut.u_dmem.memory[input_a_word + idx] !== input_a_before[idx]) begin
                        $display("[FAIL] %s: A alterada na palavra %0d", ctx, idx);
                        errors = errors + 1;
                    end
                for (idx = 0; idx < input_b_count; idx = idx + 1)
                    if (u_dut.u_dmem.memory[input_b_word + idx] !== input_b_before[idx]) begin
                        $display("[FAIL] %s: B alterada na palavra %0d", ctx, idx);
                        errors = errors + 1;
                    end
            end
            input_snapshot_valid = 1'b0;
        end
    endtask

    // Zera o rastreador de escrita unica antes de cada multiplicacao.
    task automatic clear_m1_write_tracker();
        int idx_w;
        begin
            m1_wr_track_addr = 32'd0;
            m1_w_beats_total = 0;
            m1_wr_oob_cnt    = 0;
            for (idx_w = 0; idx_w < 4096; idx_w = idx_w + 1) begin
                m1_wr_hit_cnt[idx_w] = 0;
            end
        end
    endtask

    // Zera os contadores do monitor antes de cada cenário direcionado (Testes 6, 11 e 12)
    task automatic clear_axi_monitor_counters();
        begin
            m1_4kb_violations   = 0;
            m1_ar_burst_cnt     = 0;
            m1_ar_4kb_end_cnt   = 0;
            m1_ar_4kb_start_cnt = 0;
            m1_aw_burst_cnt     = 0;
            m1_aw_4kb_end_cnt   = 0;
            m1_aw_4kb_start_cnt = 0;
            m1_any_valid_cycles = 0;
            m1_ar_wait_cycles   = 0;
            m1_aw_wait_cycles   = 0;
            m1_w_wait_cycles    = 0;
            s0_r_wait_cycles    = 0;
            s0_b_wait_cycles    = 0;
            s1_r_wait_cycles    = 0;
            s1_b_wait_cycles    = 0;
            m1_rresp_slverr_cnt = 0;
            m1_rresp_decerr_cnt = 0;
            m1_bresp_slverr_cnt = 0;
            m1_bresp_decerr_cnt = 0;
        end
    endtask

    always @(posedge clk) begin
        if (rst_n !== 1'b1) begin
            prev_m1_awvalid <= 1'b0; prev_m1_awready <= 1'b0;
            prev_m1_wvalid  <= 1'b0; prev_m1_wready  <= 1'b0;
            prev_m1_arvalid <= 1'b0; prev_m1_arready <= 1'b0;
            prev_s0_rvalid  <= 1'b0; prev_s0_rready  <= 1'b0;
            prev_s0_bvalid  <= 1'b0; prev_s0_bready  <= 1'b0;
            prev_s1_rvalid  <= 1'b0; prev_s1_rready  <= 1'b0;
            prev_s1_bvalid  <= 1'b0; prev_s1_bready  <= 1'b0;
            prev_m0_rvalid  <= 1'b0; prev_m0_rready  <= 1'b0;
            prev_m0_bvalid  <= 1'b0; prev_m0_bready  <= 1'b0;
        end else begin
            // Checagem de estado indefinido (X/Z) usando reducao XOR (compativel com Icarus e Xcelium)
            if (^{u_dut.m1_axi_awvalid, u_dut.m1_axi_wvalid, u_dut.m1_axi_arvalid,
                  u_dut.s0_axi_bvalid, u_dut.s0_axi_rvalid,
                  u_dut.s1_axi_bvalid, u_dut.s1_axi_rvalid} === 1'bx) begin
                $display("[FAIL][%0dx%0d L%0d] AXI4 Protocol: sinal VALID com estado X/Z! m1_aw=%b m1_w=%b m1_ar=%b s0_b=%b s0_r=%b",
                         ARRAY_SIZE, ARRAY_SIZE, DMEM_LATENCY,
                         u_dut.m1_axi_awvalid, u_dut.m1_axi_wvalid, u_dut.m1_axi_arvalid,
                         u_dut.s0_axi_bvalid, u_dut.s0_axi_rvalid);
                errors = errors + 1;
            end

            // Contagem de atividade e ciclos de espera (VALID=1 && READY=0)
            if (u_dut.m1_axi_arvalid || u_dut.m1_axi_awvalid || u_dut.m1_axi_wvalid)
                m1_any_valid_cycles = m1_any_valid_cycles + 1;
            if (u_dut.m1_axi_arvalid && !u_dut.m1_axi_arready) m1_ar_wait_cycles = m1_ar_wait_cycles + 1;
            if (u_dut.m1_axi_awvalid && !u_dut.m1_axi_awready) m1_aw_wait_cycles = m1_aw_wait_cycles + 1;
            if (u_dut.m1_axi_wvalid  && !u_dut.m1_axi_wready)  m1_w_wait_cycles  = m1_w_wait_cycles  + 1;
            if (u_dut.s0_axi_rvalid  && !u_dut.s0_axi_rready)  s0_r_wait_cycles  = s0_r_wait_cycles  + 1;
            if (u_dut.s0_axi_bvalid  && !u_dut.s0_axi_bready)  s0_b_wait_cycles  = s0_b_wait_cycles  + 1;
            if (u_dut.s1_axi_rvalid  && !u_dut.s1_axi_rready)  s1_r_wait_cycles  = s1_r_wait_cycles  + 1;
            if (u_dut.s1_axi_bvalid  && !u_dut.s1_axi_bready)  s1_b_wait_cycles  = s1_b_wait_cycles  + 1;

            // Monitoramento de respostas de erro recebidas pelo mestre DMA (distinguindo SLVERR de DECERR)
            if (u_dut.m1_axi_rvalid && u_dut.m1_axi_rready) begin
                if (u_dut.m1_axi_rresp == 2'b10) m1_rresp_slverr_cnt = m1_rresp_slverr_cnt + 1;
                if (u_dut.m1_axi_rresp == 2'b11) m1_rresp_decerr_cnt = m1_rresp_decerr_cnt + 1;
            end
            if (u_dut.m1_axi_bvalid && u_dut.m1_axi_bready) begin
                if (u_dut.m1_axi_bresp == 2'b10) m1_bresp_slverr_cnt = m1_bresp_slverr_cnt + 1;
                if (u_dut.m1_axi_bresp == 2'b11) m1_bresp_decerr_cnt = m1_bresp_decerr_cnt + 1;
            end

            // Rastreamento de escrita única por elemento de C no canal AW/W do mestre DMA (M1)
            if (u_dut.m1_axi_awvalid && u_dut.m1_axi_awready) begin
                m1_wr_track_addr = u_dut.m1_axi_awaddr;
            end
            if (u_dut.m1_axi_wvalid && u_dut.m1_axi_wready) begin
                m1_w_beats_total = m1_w_beats_total + 1;
                if (m1_wr_track_addr[31:14] == 18'd0) begin
                    m1_wr_hit_cnt[m1_wr_track_addr[13:2]] = m1_wr_hit_cnt[m1_wr_track_addr[13:2]] + 1;
                end else begin
                    m1_wr_oob_cnt = m1_wr_oob_cnt + 1;
                end
                m1_wr_track_addr = m1_wr_track_addr + 32'd4;
            end

            // Verificação estrita da fronteira de 4 KB e parâmetros em cada handshake M1 AR
            if (u_dut.m1_axi_arvalid && u_dut.m1_axi_arready) begin
                int unsigned ar_end_4k;
                ar_end_4k = int'(u_dut.m1_axi_araddr[11:0]) + ((int'(u_dut.m1_axi_arlen) + 1) * 4);
                m1_ar_burst_cnt = m1_ar_burst_cnt + 1;
                if (ar_end_4k == 4096) m1_ar_4kb_end_cnt = m1_ar_4kb_end_cnt + 1;
                if (u_dut.m1_axi_araddr[11:0] == 12'h000) m1_ar_4kb_start_cnt = m1_ar_4kb_start_cnt + 1;

                if (ar_end_4k > 4096) begin
                    $display("[FAIL][%0dx%0d L%0d] AXI4 4KB Violation em M1 AR: addr=0x%08h len=%0d (end_offset=%0d > 4096)",
                             ARRAY_SIZE, ARRAY_SIZE, DMEM_LATENCY, u_dut.m1_axi_araddr, u_dut.m1_axi_arlen, ar_end_4k);
                    m1_4kb_violations = m1_4kb_violations + 1;
                    errors = errors + 1;
                end
                if (!expect_injected_axi_err && (u_dut.m1_axi_araddr[1:0] != 2'b00)) begin
                    $display("[FAIL][%0dx%0d L%0d] AXI4 Alignment Violation em M1 AR: addr=0x%08h",
                             ARRAY_SIZE, ARRAY_SIZE, DMEM_LATENCY, u_dut.m1_axi_araddr);
                    errors = errors + 1;
                end
                if (u_dut.m1_axi_arsize != 3'b010 || u_dut.m1_axi_arburst != 2'b01) begin
                    $display("[FAIL][%0dx%0d L%0d] AXI4 ARSIZE/ARBURST invalido em M1 AR!",
                             ARRAY_SIZE, ARRAY_SIZE, DMEM_LATENCY);
                    errors = errors + 1;
                end
            end

            // Verificação estrita da fronteira de 4 KB e parâmetros em cada handshake M1 AW
            if (u_dut.m1_axi_awvalid && u_dut.m1_axi_awready) begin
                int unsigned aw_end_4k;
                aw_end_4k = int'(u_dut.m1_axi_awaddr[11:0]) + ((int'(u_dut.m1_axi_awlen) + 1) * 4);
                m1_aw_burst_cnt = m1_aw_burst_cnt + 1;
                if (aw_end_4k == 4096) m1_aw_4kb_end_cnt = m1_aw_4kb_end_cnt + 1;
                if (u_dut.m1_axi_awaddr[11:0] == 12'h000) m1_aw_4kb_start_cnt = m1_aw_4kb_start_cnt + 1;

                if (aw_end_4k > 4096) begin
                    $display("[FAIL][%0dx%0d L%0d] AXI4 4KB Violation em M1 AW: addr=0x%08h len=%0d (end_offset=%0d > 4096)",
                             ARRAY_SIZE, ARRAY_SIZE, DMEM_LATENCY, u_dut.m1_axi_awaddr, u_dut.m1_axi_awlen, aw_end_4k);
                    m1_4kb_violations = m1_4kb_violations + 1;
                    errors = errors + 1;
                end
                if (!expect_injected_axi_err && (u_dut.m1_axi_awaddr[1:0] != 2'b00)) begin
                    $display("[FAIL][%0dx%0d L%0d] AXI4 Alignment Violation em M1 AW: addr=0x%08h",
                             ARRAY_SIZE, ARRAY_SIZE, DMEM_LATENCY, u_dut.m1_axi_awaddr);
                    errors = errors + 1;
                end
                if (u_dut.m1_axi_awsize != 3'b010 || u_dut.m1_axi_awburst != 2'b01) begin
                    $display("[FAIL][%0dx%0d L%0d] AXI4 AWSIZE/AWBURST invalido em M1 AW!",
                             ARRAY_SIZE, ARRAY_SIZE, DMEM_LATENCY);
                    errors = errors + 1;
                end
            end

            // Estabilidade de AW quando AWVALID && !AWREADY
            if (prev_m1_awvalid && !prev_m1_awready) begin
                if (!u_dut.m1_axi_awvalid ||
                    (u_dut.m1_axi_awaddr !== prev_m1_awaddr) ||
                    (u_dut.m1_axi_awlen  !== prev_m1_awlen)) begin
                    $display("[FAIL][%0dx%0d L%0d] AXI4 Violation: M1 AW mudou antes de AWREADY!",
                             ARRAY_SIZE, ARRAY_SIZE, DMEM_LATENCY);
                    errors = errors + 1;
                end
            end

            // Estabilidade de W quando WVALID && !WREADY
            if (prev_m1_wvalid && !prev_m1_wready) begin
                if (!u_dut.m1_axi_wvalid ||
                    (u_dut.m1_axi_wdata !== prev_m1_wdata) ||
                    (u_dut.m1_axi_wlast !== prev_m1_wlast)) begin
                    $display("[FAIL][%0dx%0d L%0d] AXI4 Violation: M1 WDATA/WLAST mudou antes de WREADY! prev=%h cur=%h",
                             ARRAY_SIZE, ARRAY_SIZE, DMEM_LATENCY, prev_m1_wdata, u_dut.m1_axi_wdata);
                    errors = errors + 1;
                end
            end

            // Estabilidade de AR quando ARVALID && !ARREADY
            if (prev_m1_arvalid && !prev_m1_arready) begin
                if (!u_dut.m1_axi_arvalid ||
                    (u_dut.m1_axi_araddr !== prev_m1_araddr) ||
                    (u_dut.m1_axi_arlen  !== prev_m1_arlen)) begin
                    $display("[FAIL][%0dx%0d L%0d] AXI4 Violation: M1 AR mudou antes de ARREADY!",
                             ARRAY_SIZE, ARRAY_SIZE, DMEM_LATENCY);
                    errors = errors + 1;
                end
            end

            // Estabilidade de R em S0 quando RVALID && !RREADY
            if (prev_s0_rvalid && !prev_s0_rready) begin
                if (!u_dut.s0_axi_rvalid ||
                    (u_dut.s0_axi_rdata !== prev_s0_rdata) ||
                    (u_dut.s0_axi_rresp !== prev_s0_rresp) ||
                    (u_dut.s0_axi_rlast !== prev_s0_rlast)) begin
                    $display("[FAIL][%0dx%0d L%0d] AXI4 Violation: S0 R mudou antes de RREADY!",
                             ARRAY_SIZE, ARRAY_SIZE, DMEM_LATENCY);
                    errors = errors + 1;
                end
            end

            // Estabilidade de B em S0 quando BVALID && !BREADY
            if (prev_s0_bvalid && !prev_s0_bready) begin
                if (!u_dut.s0_axi_bvalid ||
                    (u_dut.s0_axi_bresp !== prev_s0_bresp)) begin
                    $display("[FAIL][%0dx%0d L%0d] AXI4 Violation: S0 B mudou antes de BREADY!",
                             ARRAY_SIZE, ARRAY_SIZE, DMEM_LATENCY);
                    errors = errors + 1;
                end
            end

            // Estabilidade de R em S1 quando RVALID && !RREADY
            if (prev_s1_rvalid && !prev_s1_rready) begin
                if (!u_dut.s1_axi_rvalid ||
                    (u_dut.s1_axi_rdata !== prev_s1_rdata) ||
                    (u_dut.s1_axi_rresp !== prev_s1_rresp) ||
                    (u_dut.s1_axi_rlast !== prev_s1_rlast)) begin
                    $display("[FAIL][%0dx%0d L%0d] AXI4 Violation: S1 R mudou antes de RREADY!",
                             ARRAY_SIZE, ARRAY_SIZE, DMEM_LATENCY);
                    errors = errors + 1;
                end
            end

            // Estabilidade de B em S1 quando BVALID && !BREADY
            if (prev_s1_bvalid && !prev_s1_bready) begin
                if (!u_dut.s1_axi_bvalid ||
                    (u_dut.s1_axi_bresp !== prev_s1_bresp)) begin
                    $display("[FAIL][%0dx%0d L%0d] AXI4 Violation: S1 B mudou antes de BREADY!",
                             ARRAY_SIZE, ARRAY_SIZE, DMEM_LATENCY);
                    errors = errors + 1;
                end
            end

            // Estabilidade de R e B entregues a M0 quando RVALID && !RREADY / BVALID && !BREADY
            if (prev_m0_rvalid && !prev_m0_rready) begin
                if (!m0_axi_rvalid ||
                    (m0_axi_rdata !== prev_m0_rdata) ||
                    (m0_axi_rresp !== prev_m0_rresp) ||
                    (m0_axi_rlast !== prev_m0_rlast)) begin
                    $display("[FAIL][%0dx%0d L%0d] AXI4 Violation: M0 R mudou antes de RREADY!",
                             ARRAY_SIZE, ARRAY_SIZE, DMEM_LATENCY);
                    errors = errors + 1;
                end
            end
            if (prev_m0_bvalid && !prev_m0_bready) begin
                if (!m0_axi_bvalid ||
                    (m0_axi_bresp !== prev_m0_bresp)) begin
                    $display("[FAIL][%0dx%0d L%0d] AXI4 Violation: M0 B mudou antes de BREADY!",
                             ARRAY_SIZE, ARRAY_SIZE, DMEM_LATENCY);
                    errors = errors + 1;
                end
            end

            prev_m1_awvalid <= u_dut.m1_axi_awvalid;
            prev_m1_awready <= u_dut.m1_axi_awready;
            prev_m1_awaddr  <= u_dut.m1_axi_awaddr;
            prev_m1_awlen   <= u_dut.m1_axi_awlen;

            prev_m1_wvalid  <= u_dut.m1_axi_wvalid;
            prev_m1_wready  <= u_dut.m1_axi_wready;
            prev_m1_wdata   <= u_dut.m1_axi_wdata;
            prev_m1_wlast   <= u_dut.m1_axi_wlast;

            prev_m1_arvalid <= u_dut.m1_axi_arvalid;
            prev_m1_arready <= u_dut.m1_axi_arready;
            prev_m1_araddr  <= u_dut.m1_axi_araddr;
            prev_m1_arlen   <= u_dut.m1_axi_arlen;

            prev_s0_rvalid  <= u_dut.s0_axi_rvalid;
            prev_s0_rready  <= u_dut.s0_axi_rready;
            prev_s0_rdata   <= u_dut.s0_axi_rdata;
            prev_s0_rresp   <= u_dut.s0_axi_rresp;
            prev_s0_rlast   <= u_dut.s0_axi_rlast;

            prev_s0_bvalid  <= u_dut.s0_axi_bvalid;
            prev_s0_bready  <= u_dut.s0_axi_bready;
            prev_s0_bresp   <= u_dut.s0_axi_bresp;

            prev_s1_rvalid  <= u_dut.s1_axi_rvalid;
            prev_s1_rready  <= u_dut.s1_axi_rready;
            prev_s1_rdata   <= u_dut.s1_axi_rdata;
            prev_s1_rresp   <= u_dut.s1_axi_rresp;
            prev_s1_rlast   <= u_dut.s1_axi_rlast;

            prev_s1_bvalid  <= u_dut.s1_axi_bvalid;
            prev_s1_bready  <= u_dut.s1_axi_bready;
            prev_s1_bresp   <= u_dut.s1_axi_bresp;

            prev_m0_rvalid  <= m0_axi_rvalid;
            prev_m0_rready  <= m0_axi_rready;
            prev_m0_rdata   <= m0_axi_rdata;
            prev_m0_rresp   <= m0_axi_rresp;
            prev_m0_rlast   <= m0_axi_rlast;

            prev_m0_bvalid  <= m0_axi_bvalid;
            prev_m0_bready  <= m0_axi_bready;
            prev_m0_bresp   <= m0_axi_bresp;
        end
    end

    // -------------------------------------------------------------------------
    // Buffers de Trabalho para Estímulos e Referência Matemática
    // -------------------------------------------------------------------------
    int signed mat_A [0:31][0:31];     // Matriz de entrada A gerada pelo TB (até 32x32)
    int signed mat_B [0:31][0:31];     // Matriz de entrada B gerada pelo TB (até 32x32)
    int signed mat_C_ref [0:31][0:31]; // Matriz esperada C = A x B calculada como referência

    logic [31:0] hex_a_4x4 [0:15];     // Vetor .hex 4x4 da matriz A (Golden Model)
    logic [31:0] hex_b_4x4 [0:15];     // Vetor .hex 4x4 da matriz B (Golden Model)
    logic [31:0] hex_c_4x4 [0:15];     // Vetor .hex 4x4 esperado de C (Golden Model)

    logic [31:0] hex_a_8x8 [0:63];       // Vetor .hex 8x8 da matriz A (Golden Model)
    logic [31:0] hex_b_8x8 [0:63];       // Vetor .hex 8x8 da matriz B (Golden Model)
    logic [31:0] hex_c_8x8 [0:63];       // Vetor .hex 8x8 esperado de C (Golden Model)

    logic [31:0] hex_a_32x32 [0:1023];   // Vetor .hex 32x32 variado da matriz A (Python seed=2026)
    logic [31:0] hex_b_32x32 [0:1023];   // Vetor .hex 32x32 variado da matriz B (Python seed=2026)
    logic [31:0] hex_c_32x32 [0:1023];   // Vetor .hex 32x32 esperado de C (Python seed=2026)

    // Contabiliza e imprime o resultado [PASS]/[FAIL] de cada subteste
    task automatic report_subtest(input int err_before, input string label);
        begin
            subtests_total = subtests_total + 1;
            if (errors == err_before) begin
                subtests_passed = subtests_passed + 1;
                $display("[PASS][%0dx%0d L%0d] %s", ARRAY_SIZE, ARRAY_SIZE, DMEM_LATENCY, label);
            end else begin
                $display("[FAIL][%0dx%0d L%0d] %s (erros: %0d)",
                         ARRAY_SIZE, ARRAY_SIZE, DMEM_LATENCY, label, errors - err_before);
            end
        end
    endtask

    // -------------------------------------------------------------------------
    // Tarefas BFM da CPU (Mestre 0 AXI4)
    // -------------------------------------------------------------------------
    task automatic bfm_reset();
        begin
            rst_n          = 1'b0;
            m0_axi_awaddr  = 32'd0;
            m0_axi_awlen   = 8'd0;
            m0_axi_awsize  = 3'b010;
            m0_axi_awburst = 2'b01;
            m0_axi_awvalid = 1'b0;
            m0_axi_wdata   = 32'd0;
            m0_axi_wstrb   = 4'b0000;
            m0_axi_wlast   = 1'b0;
            m0_axi_wvalid  = 1'b0;
            m0_axi_bready  = 1'b0;
            m0_axi_araddr  = 32'd0;
            m0_axi_arlen   = 8'd0;
            m0_axi_arsize  = 3'b010;
            m0_axi_arburst = 2'b01;
            m0_axi_arvalid = 1'b0;
            m0_axi_rready  = 1'b0;
            repeat (3) @(posedge clk);
            #1;
            rst_n = 1'b1;
            @(posedge clk);
        end
    endtask

    task automatic axi_write_word_resp(
        input  logic [31:0] addr,
        input  logic [31:0] data,
        input  logic [3:0]  strb,
        input  logic [1:0]  expected_resp
    );
        int timeout;
        logic aw_ok, w_ok;
        bit aborted;
        begin
            @(negedge clk);
            m0_axi_awaddr  = addr;
            m0_axi_awlen   = 8'd0;
            m0_axi_awsize  = 3'b010;
            m0_axi_awburst = 2'b01;
            m0_axi_awvalid = 1'b1;
            m0_axi_wdata   = data;
            m0_axi_wstrb   = strb;
            m0_axi_wlast   = 1'b1;
            m0_axi_wvalid  = 1'b1;
            m0_axi_bready  = 1'b1;

            aw_ok   = 1'b0;
            w_ok    = 1'b0;
            aborted = 1'b0;
            timeout = 0;

            while (!(aw_ok && w_ok) && !aborted) begin
                @(posedge clk);
                if (m0_axi_awvalid && m0_axi_awready) aw_ok = 1'b1;
                if (m0_axi_wvalid  && m0_axi_wready)  w_ok  = 1'b1;
                #1;
                if (aw_ok) m0_axi_awvalid = 1'b0;
                if (w_ok)  m0_axi_wvalid  = 1'b0;
                timeout = timeout + 1;
                if (timeout > 200) begin
                    $display("[FAIL][%0dx%0d L%0d] Timeout AW/W em addr=0x%08h",
                             ARRAY_SIZE, ARRAY_SIZE, DMEM_LATENCY, addr);
                    errors = errors + 1;
                    m0_axi_awvalid = 1'b0; m0_axi_wvalid = 1'b0; m0_axi_bready = 1'b0;
                    aborted = 1'b1;
                end
            end

            timeout = 0;
            begin
                logic b_ok;
                b_ok = 1'b0;
                while (!b_ok && !aborted) begin
                    @(posedge clk);
                    if (m0_axi_bvalid && m0_axi_bready) begin
                        b_ok = 1'b1;
                        if (m0_axi_bresp !== expected_resp) begin
                            $display("[FAIL][%0dx%0d L%0d] BRESP inesperado em 0x%08h: obtido=%b esperado=%b",
                                     ARRAY_SIZE, ARRAY_SIZE, DMEM_LATENCY, addr, m0_axi_bresp, expected_resp);
                            errors = errors + 1;
                        end
                    end
                    #1;
                    if (b_ok) m0_axi_bready = 1'b0;
                    timeout = timeout + 1;
                    if (timeout > 200) begin
                        $display("[FAIL][%0dx%0d L%0d] Timeout BVALID em addr=0x%08h",
                                 ARRAY_SIZE, ARRAY_SIZE, DMEM_LATENCY, addr);
                        errors = errors + 1;
                        m0_axi_bready = 1'b0;
                        aborted = 1'b1;
                    end
                end
            end
        end
    endtask

    task automatic axi_write_word(input logic [31:0] addr, input logic [31:0] data);
        begin
            axi_write_word_resp(addr, data, 4'b1111, 2'b00);
        end
    endtask

    task automatic axi_read_word_resp(
        input  logic [31:0] addr,
        output logic [31:0] data,
        input  logic [1:0]  expected_resp
    );
        int timeout;
        bit aborted;
        logic ar_ok, r_ok;
        begin
            @(negedge clk);
            m0_axi_araddr  = addr;
            m0_axi_arlen   = 8'd0;
            m0_axi_arsize  = 3'b010;
            m0_axi_arburst = 2'b01;
            m0_axi_arvalid = 1'b1;
            m0_axi_rready  = 1'b1;
            data = 32'd0;
            aborted = 1'b0;

            ar_ok = 1'b0;
            timeout = 0;
            while (!ar_ok && !aborted) begin
                @(posedge clk);
                if (m0_axi_arvalid && m0_axi_arready) ar_ok = 1'b1;
                #1;
                if (ar_ok) m0_axi_arvalid = 1'b0;
                timeout = timeout + 1;
                if (timeout > 200) begin
                    $display("[FAIL][%0dx%0d L%0d] Timeout ARREADY em addr=0x%08h",
                             ARRAY_SIZE, ARRAY_SIZE, DMEM_LATENCY, addr);
                    errors = errors + 1;
                    m0_axi_arvalid = 1'b0; m0_axi_rready = 1'b0;
                    aborted = 1'b1;
                end
            end

            r_ok = 1'b0;
            timeout = 0;
            while (!r_ok && !aborted) begin
                @(posedge clk);
                if (m0_axi_rvalid && m0_axi_rready) begin
                    r_ok = 1'b1;
                    data = m0_axi_rdata;
                    if (m0_axi_rresp !== expected_resp || m0_axi_rlast !== 1'b1) begin
                        $display("[FAIL][%0dx%0d L%0d] RRESP/RLAST inesperado em 0x%08h: rresp=%b (esp %b), rlast=%b",
                                 ARRAY_SIZE, ARRAY_SIZE, DMEM_LATENCY, addr, m0_axi_rresp, expected_resp, m0_axi_rlast);
                        errors = errors + 1;
                    end
                end
                #1;
                if (r_ok) m0_axi_rready = 1'b0;
                timeout = timeout + 1;
                if (timeout > 200) begin
                    $display("[FAIL][%0dx%0d L%0d] Timeout RVALID em addr=0x%08h",
                             ARRAY_SIZE, ARRAY_SIZE, DMEM_LATENCY, addr);
                    errors = errors + 1;
                    m0_axi_rready = 1'b0;
                    aborted = 1'b1;
                end
            end
        end
    endtask

    task automatic axi_read_word(input logic [31:0] addr, output logic [31:0] data);
        begin
            axi_read_word_resp(addr, data, 2'b00);
        end
    endtask

    // Transação em rajada explícita para testar estouro de fronteira por M0
    task automatic axi_read_burst_expect_resp(
        input logic [31:0] addr,
        input logic [7:0]  len,
        input logic [1:0]  expected_resp
    );
        int timeout, beat;
        bit aborted;
        logic ar_ok, beat_ok;
        begin
            @(negedge clk);
            m0_axi_araddr  = addr;
            m0_axi_arlen   = len;
            m0_axi_arsize  = 3'b010;
            m0_axi_arburst = 2'b01;
            m0_axi_arvalid = 1'b1;
            m0_axi_rready  = 1'b1;
            aborted = 1'b0;

            ar_ok = 1'b0;
            timeout = 0;
            while (!ar_ok && !aborted) begin
                @(posedge clk);
                if (m0_axi_arvalid && m0_axi_arready) ar_ok = 1'b1;
                #1;
                if (ar_ok) m0_axi_arvalid = 1'b0;
                timeout = timeout + 1;
                if (timeout > 200) begin
                    errors = errors + 1; m0_axi_arvalid = 1'b0; m0_axi_rready = 1'b0; aborted = 1'b1;
                end
            end

            for (beat = 0; beat <= len && !aborted; beat = beat + 1) begin
                beat_ok = 1'b0;
                timeout = 0;
                while (!beat_ok && !aborted) begin
                    @(posedge clk);
                    if (m0_axi_rvalid && m0_axi_rready) begin
                        beat_ok = 1'b1;
                        if (m0_axi_rresp !== expected_resp) begin
                            $display("[FAIL][%0dx%0d L%0d] Burst DECERR beat %0d: rresp=%b (esp %b)",
                                     ARRAY_SIZE, ARRAY_SIZE, DMEM_LATENCY, beat, m0_axi_rresp, expected_resp);
                            errors = errors + 1;
                        end
                        if (m0_axi_rlast !== (beat == len)) begin
                            $display("[FAIL][%0dx%0d L%0d] Burst RLAST erro no beat %0d: rlast=%b",
                                     ARRAY_SIZE, ARRAY_SIZE, DMEM_LATENCY, beat, m0_axi_rlast);
                            errors = errors + 1;
                        end
                    end
                    #1;
                    if (beat_ok && (beat == len)) m0_axi_rready = 1'b0;
                    timeout = timeout + 1;
                    if (timeout > 200) begin
                        errors = errors + 1; m0_axi_rready = 1'b0; aborted = 1'b1;
                    end
                end
            end
        end
    endtask

    // Escrita em rajada (ou palavra única) com esperas explícitas em WVALID (após AW e entre beats)
    // e retenção explícita de BREADY=0 após BVALID=1, verificando estabilidade de BVALID/BRESP.
    task automatic axi_write_burst_with_stalls(
        input logic [31:0] addr,
        input logic [7:0]  len,
        input logic [31:0] base_data,
        input int          w_gap_cycles,
        input int          bready_hold_cycles,
        input logic [1:0]  expected_resp
    );
        int timeout, beat, hold_k;
        bit aborted;
        logic aw_ok, beat_ok, bvalid_seen;
        logic [1:0] snap_bresp;
        begin
            @(negedge clk);
            m0_axi_awaddr  = addr;
            m0_axi_awlen   = len;
            m0_axi_awsize  = 3'b010;
            m0_axi_awburst = 2'b01;
            m0_axi_awvalid = 1'b1;
            m0_axi_wvalid  = 1'b0; // Atraso intencional de W em relação a AW
            m0_axi_bready  = 1'b0; // Retém BREADY=0 para testar estabilidade de BVALID
            aborted        = 1'b0;

            aw_ok   = 1'b0;
            timeout = 0;
            while (!aw_ok && !aborted) begin
                @(posedge clk);
                if (m0_axi_awvalid && m0_axi_awready) aw_ok = 1'b1;
                #1;
                if (aw_ok) m0_axi_awvalid = 1'b0;
                timeout = timeout + 1;
                if (timeout > 200) begin
                    errors = errors + 1; m0_axi_awvalid = 1'b0; aborted = 1'b1;
                end
            end

            for (beat = 0; beat <= len && !aborted; beat = beat + 1) begin
                // Espera intencional no canal de dados W (WVALID=0 enquanto escravo aguarda)
                repeat (w_gap_cycles) @(posedge clk);
                #1;
                m0_axi_wdata  = base_data + beat;
                m0_axi_wstrb  = 4'b1111;
                m0_axi_wlast  = (beat == len);
                m0_axi_wvalid = 1'b1;

                beat_ok = 1'b0;
                timeout = 0;
                while (!beat_ok && !aborted) begin
                    @(posedge clk);
                    if (m0_axi_wvalid && m0_axi_wready) beat_ok = 1'b1;
                    #1;
                    if (beat_ok) begin
                        m0_axi_wvalid = 1'b0;
                        m0_axi_wlast  = 1'b0;
                    end
                    timeout = timeout + 1;
                    if (timeout > 200) begin
                        errors = errors + 1; m0_axi_wvalid = 1'b0; m0_axi_wlast = 1'b0; aborted = 1'b1;
                    end
                end
            end

            // Aguarda BVALID subir com BREADY=0
            bvalid_seen = 1'b0;
            timeout     = 0;
            while (!bvalid_seen && !aborted) begin
                @(posedge clk);
                if (m0_axi_bvalid) begin
                    bvalid_seen = 1'b1;
                    snap_bresp  = m0_axi_bresp;
                end
                timeout = timeout + 1;
                if (timeout > 200) begin
                    errors = errors + 1; aborted = 1'b1;
                end
            end

            // Mantém BREADY=0 por bready_hold_cycles ciclos e confere estabilidade de BVALID/BRESP
            for (hold_k = 0; hold_k < bready_hold_cycles && !aborted; hold_k = hold_k + 1) begin
                @(posedge clk);
                if (m0_axi_bvalid !== 1'b1 || m0_axi_bresp !== snap_bresp) begin
                    $display("[FAIL][%0dx%0d L%0d] BVALID/BRESP instavel com BREADY=0 em 0x%08h!",
                             ARRAY_SIZE, ARRAY_SIZE, DMEM_LATENCY, addr);
                    errors = errors + 1;
                end
            end

            // Libera BREADY=1 para concluir o handshake do canal B
            #1;
            m0_axi_bready = 1'b1;
            @(posedge clk);
            if (!aborted && (!m0_axi_bvalid || m0_axi_bresp !== expected_resp)) begin
                $display("[FAIL][%0dx%0d L%0d] Handshake B falhou apos retencao de BREADY em 0x%08h: resp=%b",
                         ARRAY_SIZE, ARRAY_SIZE, DMEM_LATENCY, addr, m0_axi_bresp);
                errors = errors + 1;
            end
            #1;
            m0_axi_bready = 1'b0;
        end
    endtask

    // Leitura em rajada (ou palavra única) com retenção explícita de RREADY=0 em cada beat
    // após RVALID=1, conferindo manutenção de RVALID, RDATA, RRESP e RLAST.
    task automatic axi_read_burst_with_rready_stalls(
        input logic [31:0] addr,
        input logic [7:0]  len,
        input logic [31:0] expected_base_data,
        input bit          check_data,
        input int          rready_hold_cycles,
        input logic [1:0]  expected_resp
    );
        int timeout, beat, hold_k;
        bit aborted;
        logic ar_ok, rvalid_seen;
        logic [31:0] snap_rdata;
        logic [1:0]  snap_rresp;
        logic        snap_rlast;
        begin
            @(negedge clk);
            m0_axi_araddr  = addr;
            m0_axi_arlen   = len;
            m0_axi_arsize  = 3'b010;
            m0_axi_arburst = 2'b01;
            m0_axi_arvalid = 1'b1;
            m0_axi_rready  = 1'b0; // Retém RREADY=0
            aborted        = 1'b0;

            ar_ok   = 1'b0;
            timeout = 0;
            while (!ar_ok && !aborted) begin
                @(posedge clk);
                if (m0_axi_arvalid && m0_axi_arready) ar_ok = 1'b1;
                #1;
                if (ar_ok) m0_axi_arvalid = 1'b0;
                timeout = timeout + 1;
                if (timeout > 200) begin
                    errors = errors + 1; m0_axi_arvalid = 1'b0; aborted = 1'b1;
                end
            end

            for (beat = 0; beat <= len && !aborted; beat = beat + 1) begin
                rvalid_seen = 1'b0;
                timeout     = 0;
                while (!rvalid_seen && !aborted) begin
                    @(posedge clk);
                    if (m0_axi_rvalid) begin
                        rvalid_seen = 1'b1;
                        snap_rdata  = m0_axi_rdata;
                        snap_rresp  = m0_axi_rresp;
                        snap_rlast  = m0_axi_rlast;
                    end
                    timeout = timeout + 1;
                    if (timeout > 200) begin
                        errors = errors + 1; aborted = 1'b1;
                    end
                end

                // Mantém RREADY=0 por rready_hold_cycles ciclos e verifica imutabilidade
                for (hold_k = 0; hold_k < rready_hold_cycles && !aborted; hold_k = hold_k + 1) begin
                    @(posedge clk);
                    if (m0_axi_rvalid !== 1'b1 ||
                        m0_axi_rdata  !== snap_rdata ||
                        m0_axi_rresp  !== snap_rresp ||
                        m0_axi_rlast  !== snap_rlast) begin
                        $display("[FAIL][%0dx%0d L%0d] RVALID/RDATA/RRESP/RLAST mudou durante RREADY=0 no beat %0d!",
                                 ARRAY_SIZE, ARRAY_SIZE, DMEM_LATENCY, beat);
                        errors = errors + 1;
                    end
                end

                // Pulsa RREADY=1 por 1 ciclo para consumir o beat atual
                #1;
                m0_axi_rready = 1'b1;
                @(posedge clk);
                if (!aborted) begin
                    if (!m0_axi_rvalid || m0_axi_rresp !== expected_resp || m0_axi_rlast !== (beat == len)) begin
                        $display("[FAIL][%0dx%0d L%0d] Handshake R invalido apos stall no beat %0d!",
                                 ARRAY_SIZE, ARRAY_SIZE, DMEM_LATENCY, beat);
                        errors = errors + 1;
                    end
                    if (check_data && (m0_axi_rdata !== (expected_base_data + beat))) begin
                        $display("[FAIL][%0dx%0d L%0d] RDATA incorreto apos stall no beat %0d: obtido=%h esp=%h",
                                 ARRAY_SIZE, ARRAY_SIZE, DMEM_LATENCY, beat, m0_axi_rdata, expected_base_data + beat);
                        errors = errors + 1;
                    end
                end
                #1;
                m0_axi_rready = 1'b0;
            end
        end
    endtask

    // -------------------------------------------------------------------------
    // Helpers de Referência, Carga e Conferência de Matrizes
    // -------------------------------------------------------------------------
    task automatic compute_reference(input int M, input int K, input int N);
        int i, j, k;
        int signed acc;
        begin
            for (i = 0; i < M; i = i + 1) begin
                for (j = 0; j < N; j = j + 1) begin
                    acc = 0;
                    for (k = 0; k < K; k = k + 1) begin
                        acc = acc + (mat_A[i][k] * mat_B[k][j]);
                    end
                    mat_C_ref[i][j] = acc;
                end
            end
        end
    endtask

    task automatic write_matrices_to_dmem(
        input int M, input int K, input int N,
        input logic [31:0] base_a, input logic [31:0] base_b,
        input bit pollute_upper_bits
    );
        int i, j;
        logic [31:0] word_val;
        logic [7:0]  byte_val;
        begin
            for (i = 0; i < M; i = i + 1) begin
                for (j = 0; j < K; j = j + 1) begin
                    byte_val = mat_A[i][j][7:0];
                    word_val = pollute_upper_bits ? {24'hA55A_FF, byte_val} : {24'd0, byte_val};
                    axi_write_word(base_a + ((i * K + j) * 4), word_val);
                end
            end
            for (i = 0; i < K; i = i + 1) begin
                for (j = 0; j < N; j = j + 1) begin
                    byte_val = mat_B[i][j][7:0];
                    word_val = pollute_upper_bits ? {24'hDEAD_BE, byte_val} : {24'd0, byte_val};
                    axi_write_word(base_b + ((i * N + j) * 4), word_val);
                end
            end
        end
    endtask

    // Carga direta na memória física para casos grandes como 32x32 (economiza milhões de ciclos de TB)
    task automatic fast_load_matrices_dmem(
        input int M, input int K, input int N,
        input logic [31:0] base_a, input logic [31:0] base_b
    );
        int i, j;
        begin
            for (i = 0; i < M; i = i + 1) begin
                for (j = 0; j < K; j = j + 1) begin
                    u_dut.u_dmem.memory[(base_a >> 2) + (i * K + j)] = {24'd0, mat_A[i][j][7:0]};
                end
            end
            for (i = 0; i < K; i = i + 1) begin
                for (j = 0; j < N; j = j + 1) begin
                    u_dut.u_dmem.memory[(base_b >> 2) + (i * N + j)] = {24'd0, mat_B[i][j][7:0]};
                end
            end
        end
    endtask

    task automatic config_and_run_dma(
        input int M, input int K, input int N,
        input logic [31:0] base_a, input logic [31:0] base_b, input logic [31:0] base_c,
        input bit irq_en
    );
        logic [31:0] st;
        int polls;
        bit timed_out;
        begin
            clear_m1_write_tracker();
            snapshot_inputs(M, K, N, base_a, base_b);
            axi_write_word(REG_DMA_ADDR_A, base_a);
            axi_write_word(REG_DMA_ADDR_B, base_b);
            axi_write_word(REG_DMA_ADDR_C, base_c);
            axi_write_word(REG_DMA_DIM_M, M);
            axi_write_word(REG_DMA_DIM_K, K);
            axi_write_word(REG_DMA_DIM_N, N);
            axi_write_word(REG_DMA_CTRL, irq_en ? 32'h0000_0003 : 32'h0000_0001);

            polls = 0;
            st = 32'd0;
            timed_out = 1'b0;
            do begin
                repeat (4) @(posedge clk);
                axi_read_word(REG_DMA_STATUS, st);
                polls = polls + 1;
                if (polls > 25000) begin
                    $display("[FAIL][%0dx%0d L%0d] Timeout aguardando DMA_STATUS.DONE!",
                             ARRAY_SIZE, ARRAY_SIZE, DMEM_LATENCY);
                    errors = errors + 1;
                    timed_out = 1'b1;
                end
            end while ((st[0] == 1'b1) && !timed_out); // Enquanto BUSY == 1

            if (!timed_out && (st[2] !== 1'b0 || st[1] !== 1'b1)) begin
                $display("[FAIL][%0dx%0d L%0d] Status final inesperado: 0x%08h",
                         ARRAY_SIZE, ARRAY_SIZE, DMEM_LATENCY, st);
                errors = errors + 1;
            end
        end
    endtask

    task automatic verify_c_in_dmem(
        input int M, input int N,
        input logic [31:0] base_c,
        input string ctx,
        input bit fast_check
    );
        int i, j, word_idx, idx_w, c_start_word, c_end_word;
        logic [31:0] got;
        int signed got_s;
        bit mismatch_found;
        begin
            verify_inputs_preserved(ctx);
            c_start_word = base_c >> 2;
            c_end_word   = c_start_word + (M * N);
            // Garante que o DMA gravou exatamente M * N palavras no total e nenhuma fora da regiao de C
            if (m1_w_beats_total !== (M * N)) begin
                $display("[FAIL][%0dx%0d L%0d] %s: Total de escritas W do DMA=%0d != M*N=%0d",
                         ARRAY_SIZE, ARRAY_SIZE, DMEM_LATENCY, ctx, m1_w_beats_total, M * N);
                errors = errors + 1;
            end
            if (m1_wr_oob_cnt !== 0) begin
                $display("[FAIL][%0dx%0d L%0d] %s: Detectadas %0d escritas W do DMA fora da DMEM!",
                         ARRAY_SIZE, ARRAY_SIZE, DMEM_LATENCY, ctx, m1_wr_oob_cnt);
                errors = errors + 1;
            end
            for (idx_w = 0; idx_w < 4096; idx_w = idx_w + 1) begin
                if ((idx_w < c_start_word || idx_w >= c_end_word) && m1_wr_hit_cnt[idx_w] !== 0) begin
                    $display("[FAIL][%0dx%0d L%0d] %s: Escrita indevida do DMA fora de C no endereco 0x%04h (%0d vezes)",
                             ARRAY_SIZE, ARRAY_SIZE, DMEM_LATENCY, ctx, idx_w << 2, m1_wr_hit_cnt[idx_w]);
                    errors = errors + 1;
                end
            end
            mismatch_found = 1'b0;
            for (i = 0; i < M && !mismatch_found; i = i + 1) begin
                for (j = 0; j < N && !mismatch_found; j = j + 1) begin
                    word_idx = (base_c >> 2) + (i * N + j);
                    if (m1_wr_hit_cnt[word_idx] !== 1) begin
                        $display("[FAIL][%0dx%0d L%0d] %s: C[%0d][%0d] recebeu %0d escritas do DMA (esperado exatamente 1)",
                                 ARRAY_SIZE, ARRAY_SIZE, DMEM_LATENCY, ctx, i, j, m1_wr_hit_cnt[word_idx]);
                        errors = errors + 1;
                        mismatch_found = 1'b1;
                    end
                    if (fast_check) begin
                        got = u_dut.u_dmem.memory[word_idx];
                    end else begin
                        axi_read_word(base_c + ((i * N + j) * 4), got);
                    end
                    got_s = $signed(got);
                    if (got_s !== mat_C_ref[i][j]) begin
                        $display("[FAIL][%0dx%0d L%0d] %s: C[%0d][%0d] obtido=%0d (0x%08h) != esperado=%0d",
                                 ARRAY_SIZE, ARRAY_SIZE, DMEM_LATENCY, ctx, i, j, got_s, got, mat_C_ref[i][j]);
                        errors = errors + 1;
                        mismatch_found = 1'b1;
                    end
                end
            end
        end
    endtask

    // Valida rejeicao limpa de configuracao invalida do DMA:
    // ERR=1, DONE=0, BUSY=0, IRQ=0, zero trafego M1 (AR/AW/W) e memoria preservada.
    task automatic expect_rejected_dma_cfg(
        input string tag,
        input int M, input int K, input int N,
        input logic [31:0] base_a, input logic [31:0] base_b, input logic [31:0] base_c
    );
        logic [31:0] st, rd_s0, rd_s1, rd_s2;
        begin
            // Grava sentinelas em pontos criticos da DMEM (inclui ultima palavra 0x3FFC)
            axi_write_word(32'h0000_3000, 32'hCAFE_1111);
            axi_write_word(32'h0000_3FF8, 32'hCAFE_2222);
            axi_write_word(32'h0000_3FFC, 32'hCAFE_3333);

            clear_axi_monitor_counters();
            clear_m1_write_tracker();
            axi_write_word(REG_DMA_ADDR_A, base_a);
            axi_write_word(REG_DMA_ADDR_B, base_b);
            axi_write_word(REG_DMA_ADDR_C, base_c);
            axi_write_word(REG_DMA_DIM_M, M);
            axi_write_word(REG_DMA_DIM_K, K);
            axi_write_word(REG_DMA_DIM_N, N);
            axi_write_word(REG_DMA_CTRL, 32'h0000_0003); // START=1, IRQ_EN=1
            repeat (6) @(posedge clk);

            axi_read_word(REG_DMA_STATUS, st);
            if (st[2:0] !== 3'b100 || irq_done !== 1'b0) begin
                $display("[FAIL][%0dx%0d L%0d] %s: status=%b irq=%b (esperado ERR=1 DONE=0 BUSY=0 IRQ=0)",
                         ARRAY_SIZE, ARRAY_SIZE, DMEM_LATENCY, tag, st[2:0], irq_done);
                errors = errors + 1;
            end
            if (m1_ar_burst_cnt != 0 || m1_aw_burst_cnt != 0 ||
                m1_w_beats_total != 0 || m1_any_valid_cycles != 0) begin
                $display("[FAIL][%0dx%0d L%0d] %s: DMA emitiu trafego M1 indevido (ar=%0d aw=%0d w=%0d valid_cyc=%0d)",
                         ARRAY_SIZE, ARRAY_SIZE, DMEM_LATENCY, tag,
                         m1_ar_burst_cnt, m1_aw_burst_cnt, m1_w_beats_total, m1_any_valid_cycles);
                errors = errors + 1;
            end
            axi_read_word(32'h0000_3000, rd_s0);
            axi_read_word(32'h0000_3FF8, rd_s1);
            axi_read_word(32'h0000_3FFC, rd_s2);
            if (rd_s0 !== 32'hCAFE_1111 || rd_s1 !== 32'hCAFE_2222 || rd_s2 !== 32'hCAFE_3333) begin
                $display("[FAIL][%0dx%0d L%0d] %s: DMEM alterada apos comando rejeitado! (0x%08h, 0x%08h, 0x%08h)",
                         ARRAY_SIZE, ARRAY_SIZE, DMEM_LATENCY, tag, rd_s0, rd_s1, rd_s2);
                errors = errors + 1;
            end
            axi_write_word(REG_DMA_STATUS, 32'h0000_0004); // limpa ERR via W1C
            axi_read_word(REG_DMA_STATUS, st);
            if (st[2:0] !== 3'b000) begin
                $display("[FAIL][%0dx%0d L%0d] %s: W1C nao limpou ERR (status=%b)",
                         ARRAY_SIZE, ARRAY_SIZE, DMEM_LATENCY, tag, st[2:0]);
                errors = errors + 1;
            end
        end
    endtask

    // -------------------------------------------------------------------------
    // Suíte Completa de Testes
    // -------------------------------------------------------------------------
    task automatic run_all_tests();
        int i, j, err_before;
        int expected_comp_cyc, expected_reads;
        int ti_cnt, tj_cnt, tk_cnt;
        logic [31:0] rd_val, rd_perf_tot, rd_perf_cmp, rd_perf_rds, rd_hw_sz;
        logic [31:0] sentinel_before, sentinel_after;
        begin
            errors                  = 0;
            subtests_passed         = 0;
            subtests_total          = 0;
            expect_injected_axi_err = 1'b0;
            clear_axi_monitor_counters();

            bfm_reset();

            // =================================================================
            // TESTE 1: Base T x T, IRQ_EN, W1C e Contadores de Desempenho
            // =================================================================
            err_before = errors;
            axi_read_word(REG_HW_ARRAY_SIZE, rd_hw_sz);
            if (rd_hw_sz !== ARRAY_SIZE) begin
                $display("[FAIL][%0dx%0d L%0d] Teste 1: HW_ARRAY_SIZE=%0d != %0d",
                         ARRAY_SIZE, ARRAY_SIZE, DMEM_LATENCY, rd_hw_sz, ARRAY_SIZE);
                errors = errors + 1;
            end
            for (i = 0; i < ARRAY_SIZE; i = i + 1) begin
                for (j = 0; j < ARRAY_SIZE; j = j + 1) begin
                    mat_A[i][j] = (((i * ARRAY_SIZE + j) * 3) % 200) - 100;
                    mat_B[i][j] = (i == j) ? 2 : -1;
                end
            end
            compute_reference(ARRAY_SIZE, ARRAY_SIZE, ARRAY_SIZE);
            write_matrices_to_dmem(ARRAY_SIZE, ARRAY_SIZE, ARRAY_SIZE, 32'h1000, 32'h2000, 1'b0);
            config_and_run_dma(ARRAY_SIZE, ARRAY_SIZE, ARRAY_SIZE, 32'h1000, 32'h2000, 32'h3000, 1'b1);

            if (irq_done !== 1'b1) begin
                $display("[FAIL][%0dx%0d L%0d] Teste 1: irq_done deveria estar em 1 com IRQ_EN=1!",
                         ARRAY_SIZE, ARRAY_SIZE, DMEM_LATENCY);
                errors = errors + 1;
            end
            axi_read_word(REG_PERF_COMP_CYCLES, rd_perf_cmp);
            axi_read_word(REG_PERF_MEM_READS,   rd_perf_rds);
            axi_read_word(REG_PERF_TOTAL_CYCLES, rd_perf_tot);

            if (rd_perf_cmp !== (3 * ARRAY_SIZE - 2)) begin
                $display("[FAIL][%0dx%0d L%0d] Teste 1: PERF_COMP_CYCLES=%0d != %0d",
                         ARRAY_SIZE, ARRAY_SIZE, DMEM_LATENCY, rd_perf_cmp, 3*ARRAY_SIZE-2);
                errors = errors + 1;
            end
            if (rd_perf_rds !== (2 * ARRAY_SIZE * ARRAY_SIZE)) begin
                $display("[FAIL][%0dx%0d L%0d] Teste 1: PERF_MEM_READS=%0d != %0d",
                         ARRAY_SIZE, ARRAY_SIZE, DMEM_LATENCY, rd_perf_rds, 2*ARRAY_SIZE*ARRAY_SIZE);
                errors = errors + 1;
            end
            if (rd_perf_tot <= rd_perf_cmp) begin
                $display("[FAIL][%0dx%0d L%0d] Teste 1: PERF_TOTAL_CYCLES (%0d) <= COMP (%0d)",
                         ARRAY_SIZE, ARRAY_SIZE, DMEM_LATENCY, rd_perf_tot, rd_perf_cmp);
                errors = errors + 1;
            end
            verify_c_in_dmem(ARRAY_SIZE, ARRAY_SIZE, 32'h3000, "Teste 1", 1'b0);

            // Testa limpeza W1C de DONE e queda automática de irq_done
            axi_write_word(REG_DMA_STATUS, 32'h0000_0002);
            axi_read_word(REG_DMA_STATUS, rd_val);
            if (rd_val[1] !== 1'b0 || irq_done !== 1'b0) begin
                $display("[FAIL][%0dx%0d L%0d] Teste 1: W1C falhou ao limpar DONE/irq_done!",
                         ARRAY_SIZE, ARRAY_SIZE, DMEM_LATENCY);
                errors = errors + 1;
            end
            report_subtest(err_before, "Teste 1: Tile TxT, IRQ_EN, contadores PERF_* e limpeza W1C");

            // =================================================================
            // TESTE 2: Matriz Unitaria 1x1 com Extremos Negativos (-128 * -128)
            // =================================================================
            err_before = errors;
            mat_A[0][0] = -128;
            mat_B[0][0] = -128;
            compute_reference(1, 1, 1);
            write_matrices_to_dmem(1, 1, 1, 32'h1000, 32'h2000, 1'b1);
            config_and_run_dma(1, 1, 1, 32'h1000, 32'h2000, 32'h3000, 1'b0);
            axi_read_word(REG_PERF_MEM_READS, rd_perf_rds);
            if (rd_perf_rds !== 2) begin
                $display("[FAIL][%0dx%0d L%0d] Teste 2: 1x1 deve ler exatamente 2 palavras, leu %0d",
                         ARRAY_SIZE, ARRAY_SIZE, DMEM_LATENCY, rd_perf_rds);
                errors = errors + 1;
            end
            verify_c_in_dmem(1, 1, 32'h3000, "Teste 2 (1x1)", 1'b0);
            report_subtest(err_before, "Teste 2: Matriz 1x1 (-128 * -128 = +16384), bits [31:8] sujos, 2 leituras");

            // =================================================================
            // TESTE 3: Matrizes Retangulares e Nao-Multiplas (3x5*5x2 e 5x7*7x6)
            //          + Verificacao de Zero-Padding e Nao-Corrupcao Adjacente
            // =================================================================
            err_before = errors;
            // Grava palavras sentinela logo antes e logo depois da região de C (0x3000 .. 0x3014)
            axi_write_word(32'h2FFC, 32'hCAFE_BABE);
            axi_write_word(32'h3018, 32'hBEEF_F00D); // 3*2 = 6 palavras = 24 bytes (0x3000..0x3017)

            for (i = 0; i < 3; i = i + 1)
                for (j = 0; j < 5; j = j + 1)
                    mat_A[i][j] = (i * 7) - (j * 5) + 3;
            for (i = 0; i < 5; i = i + 1)
                for (j = 0; j < 2; j = j + 1)
                    mat_B[i][j] = (j == 0) ? (10 - i * 4) : (-20 + i * 9);
            compute_reference(3, 5, 2);
            write_matrices_to_dmem(3, 5, 2, 32'h1000, 32'h2000, 1'b1);
            config_and_run_dma(3, 5, 2, 32'h1000, 32'h2000, 32'h3000, 1'b0);
            verify_c_in_dmem(3, 2, 32'h3000, "Teste 3a (3x5*5x2)", 1'b0);

            axi_read_word(32'h2FFC, sentinel_before);
            axi_read_word(32'h3018, sentinel_after);
            if (sentinel_before !== 32'hCAFE_BABE || sentinel_after !== 32'hBEEF_F00D) begin
                $display("[FAIL][%0dx%0d L%0d] Teste 3a: Corrupcao adjacente a C! before=%h after=%h",
                         ARRAY_SIZE, ARRAY_SIZE, DMEM_LATENCY, sentinel_before, sentinel_after);
                errors = errors + 1;
            end
            axi_read_word(REG_PERF_MEM_READS, rd_perf_rds);
            expected_reads = ((2 + ARRAY_SIZE - 1) / ARRAY_SIZE) * (3 * 5) +
                             ((3 + ARRAY_SIZE - 1) / ARRAY_SIZE) * (5 * 2);
            if (rd_perf_rds !== expected_reads) begin
                $display("[FAIL][%0dx%0d L%0d] Teste 3a: PERF_MEM_READS=%0d != %0d",
                         ARRAY_SIZE, ARRAY_SIZE, DMEM_LATENCY, rd_perf_rds, expected_reads);
                errors = errors + 1;
            end

            // Caso 3b: 5x7 * 7x6
            for (i = 0; i < 5; i = i + 1)
                for (j = 0; j < 7; j = j + 1)
                    mat_A[i][j] = ((i * 7 + j) % 21) - 10;
            for (i = 0; i < 7; i = i + 1)
                for (j = 0; j < 6; j = j + 1)
                    mat_B[i][j] = ((i * 6 + j) % 19) - 9;
            compute_reference(5, 7, 6);
            write_matrices_to_dmem(5, 7, 6, 32'h1000, 32'h2000, 1'b0);
            config_and_run_dma(5, 7, 6, 32'h1000, 32'h2000, 32'h3000, 1'b0);
            verify_c_in_dmem(5, 6, 32'h3000, "Teste 3b (5x7*7x6)", 1'b0);
            report_subtest(err_before, "Teste 3: Matrizes retangulares (3x5*5x2 e 5x7*7x6), zero-padding e sentinelas");

            // =================================================================
            // TESTE 4: Tiling Completo contra Vetores .hex do Golden Model
            // =================================================================
            err_before = errors;
            $readmemh("sw/golden_model/test_vectors/mat_4x4_A.hex", hex_a_4x4);
            $readmemh("sw/golden_model/test_vectors/mat_4x4_B.hex", hex_b_4x4);
            $readmemh("sw/golden_model/test_vectors/mat_4x4_C_expected.hex", hex_c_4x4);
            for (i = 0; i < 4; i = i + 1)
                for (j = 0; j < 4; j = j + 1) begin
                    mat_A[i][j] = $signed(hex_a_4x4[i*4 + j][7:0]);
                    mat_B[i][j] = $signed(hex_b_4x4[i*4 + j][7:0]);
                    mat_C_ref[i][j] = $signed(hex_c_4x4[i*4 + j]);
                end
            write_matrices_to_dmem(4, 4, 4, 32'h1000, 32'h2000, 1'b0);
            config_and_run_dma(4, 4, 4, 32'h1000, 32'h2000, 32'h3000, 1'b0);
            verify_c_in_dmem(4, 4, 32'h3000, "Teste 4a (HEX 4x4)", 1'b0);

            $readmemh("sw/golden_model/test_vectors/mat_8x8x8_A.hex", hex_a_8x8);
            $readmemh("sw/golden_model/test_vectors/mat_8x8x8_B.hex", hex_b_8x8);
            $readmemh("sw/golden_model/test_vectors/mat_8x8x8_C_expected.hex", hex_c_8x8);
            for (i = 0; i < 8; i = i + 1)
                for (j = 0; j < 8; j = j + 1) begin
                    mat_A[i][j] = $signed(hex_a_8x8[i*8 + j][7:0]);
                    mat_B[i][j] = $signed(hex_b_8x8[i*8 + j][7:0]);
                    mat_C_ref[i][j] = $signed(hex_c_8x8[i*8 + j]);
                end
            write_matrices_to_dmem(8, 8, 8, 32'h1000, 32'h2000, 1'b0);
            config_and_run_dma(8, 8, 8, 32'h1000, 32'h2000, 32'h3000, 1'b0);
            verify_c_in_dmem(8, 8, 32'h3000, "Teste 4b (HEX 8x8)", 1'b0);

            axi_read_word(REG_PERF_COMP_CYCLES, rd_perf_cmp);
            if ((ARRAY_SIZE == 2 && rd_perf_cmp != 256) ||
                (ARRAY_SIZE == 4 && rd_perf_cmp != 80)  ||
                (ARRAY_SIZE == 8 && rd_perf_cmp != 22)) begin
                $display("[FAIL][%0dx%0d L%0d] Teste 4b: PERF_COMP_CYCLES=%0d diverge do Golden Model!",
                         ARRAY_SIZE, ARRAY_SIZE, DMEM_LATENCY, rd_perf_cmp);
                errors = errors + 1;
            end
            report_subtest(err_before, "Teste 4: Vetores .hex Golden Model (4x4 e 8x8) com PERF_COMP_CYCLES exato");

            // =================================================================
            // TESTE 5: Caso Limite Maximo 32x32 * 32x32 (todos -128 -> +524288)
            // =================================================================
            err_before = errors;
            for (i = 0; i < 32; i = i + 1) begin
                for (j = 0; j < 32; j = j + 1) begin
                    mat_A[i][j]     = -128;
                    mat_B[i][j]     = -128;
                    mat_C_ref[i][j] = 524288;
                end
            end
            fast_load_matrices_dmem(32, 32, 32, 32'h1000, 32'h2000);
            config_and_run_dma(32, 32, 32, 32'h1000, 32'h2000, 32'h3000, 1'b0);
            verify_c_in_dmem(32, 32, 32'h3000, "Teste 5 (32x32)", 1'b1);
            axi_read_word(REG_PERF_MEM_READS, rd_perf_rds);
            expected_reads = ((32 + ARRAY_SIZE - 1) / ARRAY_SIZE) * 2048;
            if (rd_perf_rds !== expected_reads) begin
                $display("[FAIL][%0dx%0d L%0d] Teste 5: PERF_MEM_READS=%0d != %0d",
                         ARRAY_SIZE, ARRAY_SIZE, DMEM_LATENCY, rd_perf_rds, expected_reads);
                errors = errors + 1;
            end
            report_subtest(err_before, "Teste 5: Estresse maximo 32x32 * 32x32 com -128 -> C=+524288 e PERF_MEM_READS exato");

            // =================================================================
            // TESTE 6: Bases Customizadas + Quebra em Fronteira AXI4 de 4 KB
            // Verifica nos handshakes AR e AW: addr[11:0] + 4*(LEN+1) <= 4096
            // e comprova a ocorrência real das divisões de rajada na fronteira.
            // =================================================================
            err_before = errors;
            for (i = 0; i < 5; i = i + 1) begin
                for (j = 0; j < 5; j = j + 1) begin
                    mat_A[i][j] = (i * 5 + j) - 12;
                    mat_B[i][j] = (j * 5 + i) - 10;
                end
            end
            compute_reference(5, 5, 5);

            // 6a: Bases (0x0FF4, 0x1FF8, 0x2FF0) cruzando 0x1000, 0x2000 e 0x3000
            clear_axi_monitor_counters();
            write_matrices_to_dmem(5, 5, 5, 32'h0FF4, 32'h1FF8, 1'b0);
            config_and_run_dma(5, 5, 5, 32'h0FF4, 32'h1FF8, 32'h2FF0, 1'b0);
            verify_c_in_dmem(5, 5, 32'h2FF0, "Teste 6a (4KB split 0x0FF4/1FF8/2FF0)", 1'b0);
            if (m1_4kb_violations != 0) begin
                $display("[FAIL][%0dx%0d L%0d] Teste 6a: Detectadas %0d violacoes de fronteira 4 KB!",
                         ARRAY_SIZE, ARRAY_SIZE, DMEM_LATENCY, m1_4kb_violations);
                errors = errors + 1;
            end

            // 6b: Bases (0x0FFC, 0x1FFC, 0x2FFC) que forçam quebra intra-linha
            // em A, B e C para TODOS os tamanhos de malha (2x2, 4x4 e 8x8)
            clear_axi_monitor_counters();
            write_matrices_to_dmem(5, 5, 5, 32'h0FFC, 32'h1FFC, 1'b0);
            config_and_run_dma(5, 5, 5, 32'h0FFC, 32'h1FFC, 32'h2FFC, 1'b0);
            verify_c_in_dmem(5, 5, 32'h2FFC, "Teste 6b (4KB split 0x0FFC/1FFC/2FFC)", 1'b0);
            begin
                int expected_aw_bursts;
                expected_aw_bursts = (5 * ((5 + ARRAY_SIZE - 1) / ARRAY_SIZE)) + 1;
                if (m1_4kb_violations != 0 ||
                    m1_ar_4kb_end_cnt < 2  || m1_ar_4kb_start_cnt < 2 ||
                    m1_aw_4kb_end_cnt < 1  || m1_aw_4kb_start_cnt < 1 ||
                    m1_aw_burst_cnt != expected_aw_bursts) begin
                    $display("[FAIL][%0dx%0d L%0d] Teste 6b: Falha na divisao 4 KB! viol=%0d ar_end=%0d ar_start=%0d aw_end=%0d aw_start=%0d aw_bursts=%0d (esp %0d)",
                             ARRAY_SIZE, ARRAY_SIZE, DMEM_LATENCY,
                             m1_4kb_violations, m1_ar_4kb_end_cnt, m1_ar_4kb_start_cnt,
                             m1_aw_4kb_end_cnt, m1_aw_4kb_start_cnt, m1_aw_burst_cnt, expected_aw_bursts);
                    errors = errors + 1;
                end
            end
            report_subtest(err_before, "Teste 6: Bases customizadas (0x0FF4.. e 0x0FFC..) com verificacao estrita de fronteira 4 KB em AR/AW");

            // =================================================================
            // TESTE 7: Decodificacao DECERR + Configuracoes Invalidas (ERR=1)
            // =================================================================
            err_before = errors;
            // 7a: Endereços fora da DMEM e MMIO -> DECERR (2'b11)
            axi_read_word_resp(32'h0000_4000, rd_val, 2'b11);
            axi_write_word_resp(32'h0001_0000, 32'h1234_5678, 4'b1111, 2'b11);
            axi_read_word_resp(32'h4000_0030, rd_val, 2'b11);
            // Rajada iniciando em 0x3FFC com LEN=1 (estoura 0x3FFF) -> DECERR sem tocar DMEM
            axi_read_burst_expect_resp(32'h0000_3FFC, 8'd1, 2'b11);

            // 7b: Configurações inválidas do DMA -> ERR=1, DONE=0, BUSY=0, zero tráfego M1 e memória intacta
            expect_rejected_dma_cfg("Teste 7b (M=0)",             0, 4, 4, 32'h1000, 32'h2000, 32'h3000);
            expect_rejected_dma_cfg("Teste 7b (Desalinhado A)",   4, 4, 4, 32'h1002, 32'h2000, 32'h3000);
            expect_rejected_dma_cfg("Teste 7b (Sobreposicao AB)", 4, 4, 4, 32'h1000, 32'h1020, 32'h3000);
            expect_rejected_dma_cfg("Teste 7b (Estouro C>16KB)",  4, 4, 4, 32'h1000, 32'h2000, 32'h3FE0);
            report_subtest(err_before, "Teste 7: DECERR de faixa/rajada e erros de config (dim=0, desalinhamento, sobreposicao, fim>16KB)");

            // =================================================================
            // TESTE 8: Protecao em BUSY=1 + Concorrencia M0(CPU) x M1(DMA)
            // =================================================================
            err_before = errors;
            for (i = 0; i < 8; i = i + 1) begin
                for (j = 0; j < 8; j = j + 1) begin
                    mat_A[i][j] = ((i + j) % 15) - 7;
                    mat_B[i][j] = ((i * 2 - j) % 13) - 6;
                end
            end
            compute_reference(8, 8, 8);
            fast_load_matrices_dmem(8, 8, 8, 32'h1000, 32'h2000);

            clear_m1_write_tracker();
            snapshot_inputs(8, 8, 8, 32'h1000, 32'h2000);
            axi_write_word(REG_DMA_ADDR_A, 32'h1000);
            axi_write_word(REG_DMA_ADDR_B, 32'h2000);
            axi_write_word(REG_DMA_ADDR_C, 32'h3000);
            axi_write_word(REG_DMA_DIM_M, 8);
            axi_write_word(REG_DMA_DIM_K, 8);
            axi_write_word(REG_DMA_DIM_N, 8);
            axi_write_word(REG_DMA_CTRL, 32'h0000_0001);

            // Disputa ativa no barramento enquanto o DMA está em execução
            for (i = 0; i < 8; i = i + 1) begin
                // Tenta alterar configuração durante BUSY=1 (deve ser ignorado)
                axi_write_word(REG_DMA_ADDR_C, 32'h3800);
                axi_write_word(REG_DMA_DIM_M, 2);
                // CPU acessa sua área geral na DMEM (0x0000_0100 + i*4) em concorrência com DMA
                axi_write_word(32'h0000_0100 + (i * 4), 32'h5500_0000 + i);
                axi_read_word(32'h0000_0100 + (i * 4), rd_val);
                if (rd_val !== (32'h5500_0000 + i)) begin
                    $display("[FAIL][%0dx%0d L%0d] Teste 8: Leitura/escrita M0 na DMEM falhou sob concorrencia!",
                             ARRAY_SIZE, ARRAY_SIZE, DMEM_LATENCY);
                    errors = errors + 1;
                end
            end

            // Aguarda conclusão do DMA
            begin
                int polls = 0;
                logic [31:0] st;
                do begin
                    repeat (4) @(posedge clk);
                    axi_read_word(REG_DMA_STATUS, st);
                    polls = polls + 1;
                end while (st[0] == 1'b1 && polls < 20000);
            end
            // Verifica se DMA_ADDR_C continuou em 0x3000 e matriz C está perfeita em 0x3000
            axi_read_word(REG_DMA_ADDR_C, rd_val);
            if (rd_val !== 32'h3000) begin
                $display("[FAIL][%0dx%0d L%0d] Teste 8: ADDR_C foi sobrescrito durante BUSY=1: 0x%08h",
                         ARRAY_SIZE, ARRAY_SIZE, DMEM_LATENCY, rd_val);
                errors = errors + 1;
            end
            verify_c_in_dmem(8, 8, 32'h3000, "Teste 8 (Concorrencia)", 1'b1);
            report_subtest(err_before, "Teste 8: Protecao de config em BUSY=1 e concorrencia AXI4 M0(CPU) vs M1(DMA)");

            // =================================================================
            // TESTE 9: Abortamento Seguro via DMA_CTRL.SOFT_RST em Operacao Ativa
            // Nota: como axi4_interconnect serializa 1 transacao por vez, a escrita
            // MMIO de SOFT_RST chega entre transacoes AXI4, comprovando abortamento
            // durante operacao DMA, limpeza de BUSY/DONE/ERR/PERF_* e recuperacao.
            // =================================================================
            err_before = errors;
            axi_write_word(REG_DMA_ADDR_A, 32'h1000);
            axi_write_word(REG_DMA_ADDR_B, 32'h2000);
            axi_write_word(REG_DMA_ADDR_C, 32'h3000);
            axi_write_word(REG_DMA_DIM_M, 16);
            axi_write_word(REG_DMA_DIM_K, 16);
            axi_write_word(REG_DMA_DIM_N, 16);
            axi_write_word(REG_DMA_CTRL, 32'h0000_0001);

            repeat (15) @(posedge clk);
            // Aciona SOFT_RST (bit 2) durante a operação ativa do DMA
            axi_write_word(REG_DMA_CTRL, 32'h0000_0004);
            begin
                int polls = 0;
                logic [31:0] st;
                do begin
                    @(posedge clk);
                    axi_read_word(REG_DMA_STATUS, st);
                    polls = polls + 1;
                end while (st[0] == 1'b1 && polls < 500);
                // Verifica retorno a IDLE com BUSY=0, DONE=0, ERR=0, irq_done=0 e contadores zerados
                if (st[2:0] !== 3'b000 || irq_done !== 1'b0) begin
                    $display("[FAIL][%0dx%0d L%0d] Teste 9: Status/IRQ nao zerados apos SOFT_RST! st=%b irq=%b",
                             ARRAY_SIZE, ARRAY_SIZE, DMEM_LATENCY, st[2:0], irq_done);
                    errors = errors + 1;
                end
            end
            axi_read_word(REG_PERF_TOTAL_CYCLES, rd_perf_tot);
            axi_read_word(REG_PERF_COMP_CYCLES,  rd_perf_cmp);
            axi_read_word(REG_PERF_MEM_READS,    rd_perf_rds);
            if (rd_perf_tot !== 32'd0 || rd_perf_cmp !== 32'd0 || rd_perf_rds !== 32'd0) begin
                $display("[FAIL][%0dx%0d L%0d] Teste 9: Contadores PERF_* nao zerados apos SOFT_RST! tot=%0d cmp=%0d rds=%0d",
                         ARRAY_SIZE, ARRAY_SIZE, DMEM_LATENCY, rd_perf_tot, rd_perf_cmp, rd_perf_rds);
                errors = errors + 1;
            end

            // Dispara multiplicação normal logo após o SOFT_RST para provar recuperação limpa
            for (i = 0; i < 4; i = i + 1)
                for (j = 0; j < 4; j = j + 1) begin
                    mat_A[i][j] = i + 1;
                    mat_B[i][j] = j + 1;
                end
            compute_reference(4, 4, 4);
            write_matrices_to_dmem(4, 4, 4, 32'h1000, 32'h2000, 1'b0);
            config_and_run_dma(4, 4, 4, 32'h1000, 32'h2000, 32'h3000, 1'b0);
            verify_c_in_dmem(4, 4, 32'h3000, "Teste 9 (Pos SOFT_RST)", 1'b0);
            report_subtest(err_before, "Teste 9: Abortamento seguro por SOFT_RST em operacao ativa, limpeza de status/contadores e recuperacao");

            // =================================================================
            // TESTE 10: Reset Assincrono Geral (rst_n=0) em Execucao + Recuperacao
            // =================================================================
            err_before = errors;
            axi_write_word(REG_DMA_DIM_M, 8);
            axi_write_word(REG_DMA_DIM_K, 8);
            axi_write_word(REG_DMA_DIM_N, 8);
            axi_write_word(REG_DMA_CTRL, 32'h0000_0001);
            repeat (10) @(posedge clk);
            rst_n = 1'b0;
            #1;
            if (u_dut.u_dma.status_busy !== 1'b0 || irq_done !== 1'b0 ||
                u_dut.u_accelerator.acc_busy !== 1'b0) begin
                $display("[FAIL][%0dx%0d L%0d] Teste 10: Reset geral nao zerou sinais internos!",
                         ARRAY_SIZE, ARRAY_SIZE, DMEM_LATENCY);
                errors = errors + 1;
            end
            bfm_reset();
            // Verifica que ADDR_A voltou para 0x1000 e que STATUS (BUSY/DONE/ERR) e PERF_* zeraram
            axi_read_word(REG_DMA_ADDR_A, rd_val);
            if (rd_val !== 32'h0000_1000) errors = errors + 1;
            axi_read_word(REG_DMA_STATUS, rd_val);
            if (rd_val[2:0] !== 3'b000) errors = errors + 1;
            axi_read_word(REG_PERF_TOTAL_CYCLES, rd_perf_tot);
            axi_read_word(REG_PERF_COMP_CYCLES,  rd_perf_cmp);
            axi_read_word(REG_PERF_MEM_READS,    rd_perf_rds);
            if (rd_perf_tot !== 32'd0 || rd_perf_cmp !== 32'd0 || rd_perf_rds !== 32'd0) errors = errors + 1;
            // Recuperação completa pós-reset
            write_matrices_to_dmem(4, 4, 4, 32'h1000, 32'h2000, 1'b0);
            config_and_run_dma(4, 4, 4, 32'h1000, 32'h2000, 32'h3000, 1'b0);
            verify_c_in_dmem(4, 4, 32'h3000, "Teste 10 (Pos Reset)", 1'b0);
            report_subtest(err_before, "Teste 10: Reset geral assincrono em execucao ativa, limpeza de status/contadores e recuperacao");

            // =================================================================
            // TESTE 11: Retencao de RREADY/BREADY e Esperas em AW/W/AR
            // =================================================================
            err_before = errors;
            clear_axi_monitor_counters();

            // 11a: Retencao de BREADY=0, RREADY=0 e atraso em WVALID na DMEM (S0), MMIO (S1) e DECERR
            // Escrita e leitura em rajada (4 beats) na DMEM com w_gap=2 e retencao=3 ciclos
            axi_write_burst_with_stalls(32'h0000_0200, 8'd3, 32'hCAFE_1000, 2, 3, 2'b00);
            axi_read_burst_with_rready_stalls(32'h0000_0200, 8'd3, 32'hCAFE_1000, 1'b1, 3, 2'b00);

            // Escrita e leitura no MMIO (S1) com atraso em W e retencao de BREADY/RREADY por 3 ciclos
            axi_write_burst_with_stalls(REG_DMA_ADDR_A, 8'd0, 32'h0000_1000, 2, 3, 2'b00);
            axi_read_burst_with_rready_stalls(REG_DMA_ADDR_A, 8'd0, 32'h0000_1000, 1'b1, 3, 2'b00);

            // Escrita e leitura em DECERR com retencao de BREADY/RREADY por 2 ciclos
            axi_write_burst_with_stalls(32'h0000_5000, 8'd0, 32'hBAD0_0000, 1, 2, 2'b11);
            axi_read_burst_with_rready_stalls(32'h0000_5000, 8'd1, 32'h0000_0000, 1'b0, 2, 2'b11);

            if (s0_b_wait_cycles < 3 || s0_r_wait_cycles < 12 ||
                s1_b_wait_cycles < 3 || s1_r_wait_cycles < 3) begin
                $display("[FAIL][%0dx%0d L%0d] Teste 11a: Contagem de stalls insuficiente! s0_b=%0d s0_r=%0d s1_b=%0d s1_r=%0d",
                         ARRAY_SIZE, ARRAY_SIZE, DMEM_LATENCY,
                         s0_b_wait_cycles, s0_r_wait_cycles, s1_b_wait_cycles, s1_r_wait_cycles);
                errors = errors + 1;
            end

            // 11b: Esperas multi-ciclo nos canais de endereco e dados (ARREADY=0, AWREADY=0, WREADY=0)
            // impostas ao mestre DMA (M1) durante leitura (ST_RD_BURST) e escrita (ST_WR_BURST)
            for (i = 0; i < 4; i = i + 1)
                for (j = 0; j < 4; j = j + 1) begin
                    mat_A[i][j] = (i * 3) - j + 2;
                    mat_B[i][j] = (j * 2) + i - 3;
                end
            compute_reference(4, 4, 4);
            write_matrices_to_dmem(4, 4, 4, 32'h1000, 32'h2000, 1'b0);

            clear_axi_monitor_counters();
            clear_m1_write_tracker();
            snapshot_inputs(4, 4, 4, 32'h1000, 32'h2000);
            axi_write_word(REG_DMA_ADDR_A, 32'h1000);
            axi_write_word(REG_DMA_ADDR_B, 32'h2000);
            axi_write_word(REG_DMA_ADDR_C, 32'h3000);
            axi_write_word(REG_DMA_DIM_M, 4);
            axi_write_word(REG_DMA_DIM_K, 4);
            axi_write_word(REG_DMA_DIM_N, 4);
            axi_write_word(REG_DMA_CTRL, 32'h0000_0001);

            // Enquanto o DMA carrega os blocos (AR) e depois grava C (AW/W), M0 executa
            // transacoes com esperas em WVALID e BREADY para reter ARREADY/AWREADY/WREADY de M1
            axi_write_burst_with_stalls(32'h0000_0220, 8'd1, 32'h1111_0000, 3, 3, 2'b00);
            begin
                int wait_wr;
                wait_wr = 0;
                while (u_dut.u_dma.dma_state != 4'd9 && u_dut.u_dma.dma_state != 4'd10 && wait_wr < 400) begin
                    @(posedge clk);
                    wait_wr = wait_wr + 1;
                end
            end
            // Ocupa o barramento logo antes de ST_WR_SETUP/ST_WR_BURST para impor varios ciclos de AWREADY=0 e WREADY=0 a M1
            axi_write_burst_with_stalls(32'h0000_0230, 8'd1, 32'h2222_0000, 3, 3, 2'b00);

            begin
                int polls = 0;
                logic [31:0] st;
                do begin
                    repeat (4) @(posedge clk);
                    axi_read_word(REG_DMA_STATUS, st);
                    polls = polls + 1;
                end while (st[0] == 1'b1 && polls < 10000);
            end
            verify_c_in_dmem(4, 4, 32'h3000, "Teste 11b (DMA sob wait-states)", 1'b0);
            if (m1_ar_wait_cycles < 3 || m1_aw_wait_cycles < 2 || m1_w_wait_cycles < 3) begin
                $display("[FAIL][%0dx%0d L%0d] Teste 11b: DMA nao sofreu esperas esperadas! ar_w=%0d aw_w=%0d w_w=%0d",
                         ARRAY_SIZE, ARRAY_SIZE, DMEM_LATENCY,
                         m1_ar_wait_cycles, m1_aw_wait_cycles, m1_w_wait_cycles);
                errors = errors + 1;
            end
            report_subtest(err_before, "Teste 11: Retencao de RREADY/BREADY (estabilidade VALID/dados) e esperas em AW/W/AR");

            // =================================================================
            // TESTE 12: Alteracao Controlada de Enderecos Internos Pos-Validacao
            // para provocar respostas SLVERR (2'b10) e DECERR (2'b11) da DMEM e
            // da interconexao, conferindo distincao do codigo recebido, ERR=1,
            // ausencia de DONE e recuperacao posterior do DMA.
            // =================================================================
            err_before = errors;
            expect_injected_axi_err = 1'b1;

            // 12a: RRESP = DECERR (2'b11) provocado pela interconexao durante leitura em rajada
            clear_axi_monitor_counters();
            axi_write_word(REG_DMA_ADDR_A, 32'h1000);
            axi_write_word(REG_DMA_ADDR_B, 32'h2000);
            axi_write_word(REG_DMA_ADDR_C, 32'h3000);
            axi_write_word(REG_DMA_DIM_M, ARRAY_SIZE);
            axi_write_word(REG_DMA_DIM_K, ARRAY_SIZE);
            axi_write_word(REG_DMA_DIM_N, ARRAY_SIZE);
            axi_write_word(REG_DMA_CTRL, 32'h0000_0003); // START=1, IRQ_EN=1
            // No estado ST_ACC_CLEAR (apos validacao inicial), altera reg_addr_a para fora de 16 KB
            u_dut.u_dma.reg_addr_a = 32'h0000_5000;
            repeat (35) @(posedge clk);
            axi_read_word(REG_DMA_STATUS, rd_val);
            if (rd_val !== 32'h0000_0004 || irq_done !== 1'b0 ||
                m1_rresp_decerr_cnt == 0 || m1_rresp_slverr_cnt != 0) begin
                $display("[FAIL][%0dx%0d L%0d] Teste 12a (RRESP DECERR): status=%b irq=%b dec=%0d slv=%0d",
                         ARRAY_SIZE, ARRAY_SIZE, DMEM_LATENCY, rd_val[2:0], irq_done,
                         m1_rresp_decerr_cnt, m1_rresp_slverr_cnt);
                errors = errors + 1;
            end
            axi_write_word(REG_DMA_STATUS, 32'h0000_0004); // limpa ERR via W1C

            // 12b: RRESP = SLVERR (2'b10) provocado pela DMEM (endereco desalinhado dentro de 16 KB)
            clear_axi_monitor_counters();
            axi_write_word(REG_DMA_ADDR_A, 32'h1000);
            axi_write_word(REG_DMA_CTRL, 32'h0000_0003);
            u_dut.u_dma.reg_addr_a = 32'h0000_1002; // desalinhado dentro da DMEM -> SLVERR
            repeat (35) @(posedge clk);
            axi_read_word(REG_DMA_STATUS, rd_val);
            if (rd_val !== 32'h0000_0004 || irq_done !== 1'b0 ||
                m1_rresp_slverr_cnt == 0 || m1_rresp_decerr_cnt != 0) begin
                $display("[FAIL][%0dx%0d L%0d] Teste 12b (RRESP SLVERR): status=%b irq=%b slv=%0d dec=%0d",
                         ARRAY_SIZE, ARRAY_SIZE, DMEM_LATENCY, rd_val[2:0], irq_done,
                         m1_rresp_slverr_cnt, m1_rresp_decerr_cnt);
                errors = errors + 1;
            end
            axi_write_word(REG_DMA_STATUS, 32'h0000_0004); // limpa ERR via W1C

            // 12c: BRESP = DECERR (2'b11) provocado pela interconexao durante escrita de C
            clear_axi_monitor_counters();
            axi_write_word(REG_DMA_ADDR_A, 32'h1000);
            axi_write_word(REG_DMA_ADDR_B, 32'h2000);
            axi_write_word(REG_DMA_ADDR_C, 32'h3000);
            axi_write_word(REG_DMA_CTRL, 32'h0000_0003);
            begin
                int w_acc;
                w_acc = 0;
                while (u_dut.u_dma.dma_state != 4'd8 && u_dut.u_dma.dma_state != 4'd9 && w_acc < 500) begin
                    @(posedge clk);
                    w_acc = w_acc + 1;
                end
            end
            u_dut.u_dma.reg_addr_c = 32'h0000_5000; // fora de 16 KB -> BRESP DECERR
            repeat (45) @(posedge clk);
            axi_read_word(REG_DMA_STATUS, rd_val);
            if (rd_val !== 32'h0000_0004 || irq_done !== 1'b0 ||
                m1_bresp_decerr_cnt == 0 || m1_bresp_slverr_cnt != 0) begin
                $display("[FAIL][%0dx%0d L%0d] Teste 12c (BRESP DECERR): status=%b irq=%b dec=%0d slv=%0d",
                         ARRAY_SIZE, ARRAY_SIZE, DMEM_LATENCY, rd_val[2:0], irq_done,
                         m1_bresp_decerr_cnt, m1_bresp_slverr_cnt);
                errors = errors + 1;
            end
            axi_write_word(REG_DMA_STATUS, 32'h0000_0004); // limpa ERR via W1C

            // 12d: BRESP = SLVERR (2'b10) provocado pela DMEM + recuperacao posterior
            clear_axi_monitor_counters();
            axi_write_word(REG_DMA_ADDR_C, 32'h3000);
            axi_write_word(REG_DMA_CTRL, 32'h0000_0003);
            begin
                int w_acc;
                w_acc = 0;
                while (u_dut.u_dma.dma_state != 4'd8 && u_dut.u_dma.dma_state != 4'd9 && w_acc < 500) begin
                    @(posedge clk);
                    w_acc = w_acc + 1;
                end
            end
            u_dut.u_dma.reg_addr_c = 32'h0000_3002; // desalinhado dentro da DMEM -> BRESP SLVERR
            repeat (45) @(posedge clk);
            axi_read_word(REG_DMA_STATUS, rd_val);
            if (rd_val !== 32'h0000_0004 || irq_done !== 1'b0 ||
                m1_bresp_slverr_cnt == 0 || m1_bresp_decerr_cnt != 0) begin
                $display("[FAIL][%0dx%0d L%0d] Teste 12d (BRESP SLVERR): status=%b irq=%b slv=%0d dec=%0d",
                         ARRAY_SIZE, ARRAY_SIZE, DMEM_LATENCY, rd_val[2:0], irq_done,
                         m1_bresp_slverr_cnt, m1_bresp_decerr_cnt);
                errors = errors + 1;
            end
            axi_write_word(REG_DMA_STATUS, 32'h0000_0004); // limpa ERR via W1C
            expect_injected_axi_err = 1'b0;

            // Recuperacao completa pos-erro AXI4
            write_matrices_to_dmem(4, 4, 4, 32'h1000, 32'h2000, 1'b0);
            config_and_run_dma(4, 4, 4, 32'h1000, 32'h2000, 32'h3000, 1'b1);
            if (irq_done !== 1'b1) errors = errors + 1;
            verify_c_in_dmem(4, 4, 32'h3000, "Teste 12 (Recuperacao pos RRESP/BRESP ERR)", 1'b0);
            axi_write_word(REG_DMA_STATUS, 32'h0000_0002); // limpa DONE via W1C
            report_subtest(err_before, "Teste 12: Alteracao controlada de enderecos pos-validacao (SLVERR vs DECERR em RRESP/BRESP) e recuperacao");

            // =================================================================
            // TESTE 13: Vetores Python 32x32 Variados (seed=2026, 1024 resultados)
            // Cenario originado na contribuicao de Matheus (Rev3), convertido
            // para palavras de 32 bits no formato oficial da base AXI4 e validado
            // contra o Golden Model oficial em ARRAY_SIZE = 2, 4 e 8.
            // Nota: a carga inicial de A/B e o pre-preenchimento de C usam acesso
            // hierarquico (fast_load) apenas para preparar a memoria; toda a
            // leitura de A/B, execucao no acelerador e escrita de C pelo DMA
            // ocorrem integralmente pelas interfaces AXI4 sob monitoramento ativo.
            // =================================================================
            err_before = errors;
            $readmemh("sw/golden_model/test_vectors/mat_32x32_varied_A.hex", hex_a_32x32);
            $readmemh("sw/golden_model/test_vectors/mat_32x32_varied_B.hex", hex_b_32x32);
            $readmemh("sw/golden_model/test_vectors/mat_32x32_varied_C_expected.hex", hex_c_32x32);

            for (i = 0; i < 32; i = i + 1) begin
                for (j = 0; j < 32; j = j + 1) begin
                    if ((^hex_a_32x32[i*32 + j]) === 1'bx ||
                        (^hex_b_32x32[i*32 + j]) === 1'bx ||
                        (^hex_c_32x32[i*32 + j]) === 1'bx) begin
                        $display("[FAIL][%0dx%0d L%0d] Teste 13: Vetor Python 32x32 contem X/Z em [%0d][%0d]!",
                                 ARRAY_SIZE, ARRAY_SIZE, DMEM_LATENCY, i, j);
                        errors = errors + 1;
                    end
                    mat_A[i][j]     = $signed(hex_a_32x32[i*32 + j][7:0]);
                    mat_B[i][j]     = $signed(hex_b_32x32[i*32 + j][7:0]);
                    mat_C_ref[i][j] = $signed(hex_c_32x32[i*32 + j]);
                end
            end

            // Sentinelas fora da regiao de C e pre-preenchimento de C para detectar
            // palavras nao sobrescritas ou escritas espurias fora de [0x3000..0x3FFC].
            u_dut.u_dmem.memory[32'h0000 >> 2] = 32'hC0DE_CAFE;
            u_dut.u_dmem.memory[32'h0FF8 >> 2] = 32'h5A5A_1234;
            u_dut.u_dmem.memory[32'h0FFC >> 2] = 32'hBEEF_F00D;
            for (i = 0; i < 1024; i = i + 1) begin
                u_dut.u_dmem.memory[(32'h3000 >> 2) + i] = 32'hA5A5_A5A5;
            end

            fast_load_matrices_dmem(32, 32, 32, 32'h1000, 32'h2000);
            clear_axi_monitor_counters();
            config_and_run_dma(32, 32, 32, 32'h1000, 32'h2000, 32'h3000, 1'b1);

            if (irq_done !== 1'b1) begin
                $display("[FAIL][%0dx%0d L%0d] Teste 13: irq_done deveria estar em 1 com IRQ_EN=1!",
                         ARRAY_SIZE, ARRAY_SIZE, DMEM_LATENCY);
                errors = errors + 1;
            end

            // Confere os 1024 resultados contra hex_c_32x32 (Python), escrita unica
            // em cada elemento de C, zero escritas fora de C e preservacao de A/B.
            verify_c_in_dmem(32, 32, 32'h3000, "Teste 13 (Python 32x32 variado)", 1'b1);

            if (u_dut.u_dmem.memory[32'h0000 >> 2] !== 32'hC0DE_CAFE ||
                u_dut.u_dmem.memory[32'h0FF8 >> 2] !== 32'h5A5A_1234 ||
                u_dut.u_dmem.memory[32'h0FFC >> 2] !== 32'hBEEF_F00D) begin
                $display("[FAIL][%0dx%0d L%0d] Teste 13: Sentinela fora de C foi corrompida!",
                         ARRAY_SIZE, ARRAY_SIZE, DMEM_LATENCY);
                errors = errors + 1;
            end
            if (m1_4kb_violations != 0 ||
                m1_rresp_slverr_cnt != 0 || m1_rresp_decerr_cnt != 0 ||
                m1_bresp_slverr_cnt != 0 || m1_bresp_decerr_cnt != 0) begin
                $display("[FAIL][%0dx%0d L%0d] Teste 13: Anomalia no monitor AXI4 (4kb=%0d r_slv=%0d r_dec=%0d b_slv=%0d b_dec=%0d)",
                         ARRAY_SIZE, ARRAY_SIZE, DMEM_LATENCY, m1_4kb_violations,
                         m1_rresp_slverr_cnt, m1_rresp_decerr_cnt,
                         m1_bresp_slverr_cnt, m1_bresp_decerr_cnt);
                errors = errors + 1;
            end

            axi_read_word(REG_PERF_TOTAL_CYCLES, rd_perf_tot);
            axi_read_word(REG_PERF_COMP_CYCLES,  rd_perf_cmp);
            axi_read_word(REG_PERF_MEM_READS,    rd_perf_rds);
            expected_comp_cyc = ((32 + ARRAY_SIZE - 1) / ARRAY_SIZE) *
                                ((32 + ARRAY_SIZE - 1) / ARRAY_SIZE) *
                                ((32 + ARRAY_SIZE - 1) / ARRAY_SIZE) *
                                (3 * ARRAY_SIZE - 2);
            expected_reads    = ((32 + ARRAY_SIZE - 1) / ARRAY_SIZE) * 2048;
            if (rd_perf_cmp !== expected_comp_cyc) begin
                $display("[FAIL][%0dx%0d L%0d] Teste 13: PERF_COMP_CYCLES=%0d != %0d",
                         ARRAY_SIZE, ARRAY_SIZE, DMEM_LATENCY, rd_perf_cmp, expected_comp_cyc);
                errors = errors + 1;
            end
            if (rd_perf_rds !== expected_reads) begin
                $display("[FAIL][%0dx%0d L%0d] Teste 13: PERF_MEM_READS=%0d != %0d",
                         ARRAY_SIZE, ARRAY_SIZE, DMEM_LATENCY, rd_perf_rds, expected_reads);
                errors = errors + 1;
            end
            $display("[INFO][%0dx%0d L%0d] Teste 13 (32x32 variado seed=2026): TOTAL_CYCLES=%0d COMP_CYCLES=%0d MEM_READS=%0d W_BEATS=%0d",
                     ARRAY_SIZE, ARRAY_SIZE, DMEM_LATENCY, rd_perf_tot, rd_perf_cmp, rd_perf_rds, m1_w_beats_total);
            axi_write_word(REG_DMA_STATUS, 32'h0000_0002); // limpa DONE via W1C
            if (irq_done !== 1'b0) begin
                $display("[FAIL][%0dx%0d L%0d] Teste 13: irq_done nao baixou apos W1C de DONE!",
                         ARRAY_SIZE, ARRAY_SIZE, DMEM_LATENCY);
                errors = errors + 1;
            end
            report_subtest(err_before, "Teste 13: Vetores Python 32x32 variados (seed=2026, 1024 resultados) com escrita unica em C e preservacao de A/B");

            // =================================================================
            // TESTE 14: Limites Exatos da DMEM (ultima palavra 0x3FFC com
            // sentinelas) e Rejeicoes de Fronteira (dim>32, estouro +4B em
            // 0x3FFC, desalinhamento B/C, sobreposicao de borda B/C).
            // Adaptado dos cenarios de limites de Matheus ao contrato AXI4
            // global de 16 KB (sem impor particoes fixas em 0x2000/0x3000).
            // =================================================================
            err_before = errors;

            // 14a: Caso positivo 1x1 ocupando exatamente a ultima palavra valida
            // da DMEM (C=0x3FFC) com sentinelas adjacentes protegidas.
            mat_A[0][0] = -37;
            mat_B[0][0] = 19;
            compute_reference(1, 1, 1);
            axi_write_word(32'h0000_1FF8, 32'hC0DE_CAFE);
            axi_write_word(32'h0000_2FF8, 32'hC0DE_CAFE);
            axi_write_word(32'h0000_3FF8, 32'hC0DE_CAFE);
            write_matrices_to_dmem(1, 1, 1, 32'h0000_1FFC, 32'h0000_2FFC, 1'b0);
            config_and_run_dma(1, 1, 1, 32'h0000_1FFC, 32'h0000_2FFC, 32'h0000_3FFC, 1'b0);
            verify_c_in_dmem(1, 1, 32'h0000_3FFC, "Teste 14a (1x1 ultima palavra 0x3FFC)", 1'b0);
            axi_read_word(32'h0000_1FF8, rd_val);
            if (rd_val !== 32'hC0DE_CAFE) begin
                $display("[FAIL][%0dx%0d L%0d] Teste 14a: Sentinela 0x1FF8 alterada: 0x%08h", ARRAY_SIZE, ARRAY_SIZE, DMEM_LATENCY, rd_val);
                errors = errors + 1;
            end
            axi_read_word(32'h0000_2FF8, rd_val);
            if (rd_val !== 32'hC0DE_CAFE) begin
                $display("[FAIL][%0dx%0d L%0d] Teste 14a: Sentinela 0x2FF8 alterada: 0x%08h", ARRAY_SIZE, ARRAY_SIZE, DMEM_LATENCY, rd_val);
                errors = errors + 1;
            end
            axi_read_word(32'h0000_3FF8, rd_val);
            if (rd_val !== 32'hC0DE_CAFE) begin
                $display("[FAIL][%0dx%0d L%0d] Teste 14a: Sentinela 0x3FF8 alterada: 0x%08h", ARRAY_SIZE, ARRAY_SIZE, DMEM_LATENCY, rd_val);
                errors = errors + 1;
            end
            axi_write_word(REG_DMA_STATUS, 32'h0000_0002); // limpa DONE

            // 14b: Caso positivo 2x2 terminando exatamente em 0x4000 (C=0x3FF0..0x3FFC)
            // e com bases livres deslocadas (A=0x2FF8 cruza 0x3000; B=0x3FC0..0x3FCF)
            for (i = 0; i < 2; i = i + 1)
                for (j = 0; j < 2; j = j + 1) begin
                    mat_A[i][j] = (i * 2 + j) * 7 - 10;
                    mat_B[i][j] = 11 - (i * 3 + j * 4);
                end
            compute_reference(2, 2, 2);
            axi_write_word(32'h0000_3FEC, 32'hDEAD_BEEF);
            clear_axi_monitor_counters();
            write_matrices_to_dmem(2, 2, 2, 32'h0000_2FF8, 32'h0000_3FC0, 1'b0);
            config_and_run_dma(2, 2, 2, 32'h0000_2FF8, 32'h0000_3FC0, 32'h0000_3FF0, 1'b0);
            verify_c_in_dmem(2, 2, 32'h0000_3FF0, "Teste 14b (2x2 fim exato 0x4000 + A cruza 0x3000)", 1'b0);
            axi_read_word(32'h0000_3FEC, rd_val);
            if (rd_val !== 32'hDEAD_BEEF) begin
                $display("[FAIL][%0dx%0d L%0d] Teste 14b: Sentinela 0x3FEC alterada: 0x%08h", ARRAY_SIZE, ARRAY_SIZE, DMEM_LATENCY, rd_val);
                errors = errors + 1;
            end
            if (m1_4kb_violations != 0) begin
                $display("[FAIL][%0dx%0d L%0d] Teste 14b: Violacao 4KB=%0d", ARRAY_SIZE, ARRAY_SIZE, DMEM_LATENCY, m1_4kb_violations);
                errors = errors + 1;
            end
            axi_write_word(REG_DMA_STATUS, 32'h0000_0002); // limpa DONE

            // 14c: Rejeicoes de limite rigorosas (ERR=1, DONE=0, BUSY=0, 0 trafego M1, DMEM preservada)
            expect_rejected_dma_cfg("Teste 14c (M=33 > 32)",            33,  4,  4, 32'h1000, 32'h2000, 32'h3000);
            expect_rejected_dma_cfg("Teste 14c (K=33 > 32)",             4, 33,  4, 32'h1000, 32'h2000, 32'h3000);
            expect_rejected_dma_cfg("Teste 14c (N=33 > 32)",             4,  4, 33, 32'h1000, 32'h2000, 32'h3000);
            expect_rejected_dma_cfg("Teste 14c (Desalinhado B)",         4,  4,  4, 32'h1000, 32'h2001, 32'h3000);
            expect_rejected_dma_cfg("Teste 14c (Desalinhado C)",         4,  4,  4, 32'h1000, 32'h2000, 32'h3003);
            expect_rejected_dma_cfg("Teste 14c (A estoura 0x4000 +4B)",  1,  2,  1, 32'h3FFC, 32'h2000, 32'h3000);
            expect_rejected_dma_cfg("Teste 14c (B estoura 0x4000 +4B)",  1,  2,  1, 32'h1000, 32'h3FFC, 32'h3000);
            expect_rejected_dma_cfg("Teste 14c (C estoura 0x4000 +4B)",  2,  1,  1, 32'h1000, 32'h2000, 32'h3FFC);
            expect_rejected_dma_cfg("Teste 14c (Sobreposicao borda BC)", 4,  4,  4, 32'h1000, 32'h2000, 32'h203C);

            report_subtest(err_before, "Teste 14: Limites exatos da DMEM (ultima palavra 0x3FFC) e rejeicoes (dim>32, +4B>16KB) sem trafego M1");
        end
    endtask

    // Bateria focada em wait-states e protocolo para a instância LATENCY=3
    task automatic run_latency3_tests();
        int i, j, err_before;
        logic [31:0] rd_val;
        begin
            errors                  = 0;
            subtests_passed         = 0;
            subtests_total          = 0;
            expect_injected_axi_err = 1'b0;
            clear_axi_monitor_counters();
            clear_m1_write_tracker();
            bfm_reset();

            err_before = errors;
            for (i = 0; i < 7; i = i + 1)
                for (j = 0; j < 9; j = j + 1)
                    mat_A[i][j] = ((i * 5 + j * 3) % 31) - 15;
            for (i = 0; i < 9; i = i + 1)
                for (j = 0; j < 5; j = j + 1)
                    mat_B[i][j] = ((i * 2 - j * 7) % 25) - 12;

            compute_reference(7, 9, 5);
            clear_axi_monitor_counters();
            write_matrices_to_dmem(7, 9, 5, 32'h0FF4, 32'h1FF8, 1'b0);
            config_and_run_dma(7, 9, 5, 32'h0FF4, 32'h1FF8, 32'h2FF0, 1'b1);
            verify_c_in_dmem(7, 5, 32'h2FF0, "Lat3 (7x9*9x5 com 4KB split)", 1'b0);
            if (m1_4kb_violations != 0 || m1_ar_4kb_end_cnt == 0 || m1_aw_4kb_end_cnt == 0) begin
                $display("[FAIL][4x4 L3] Violacao ou ausencia de split 4KB em LATENCY=3!");
                errors = errors + 1;
            end
            axi_write_word(REG_DMA_STATUS, 32'h0000_0002); // limpa DONE via W1C
            axi_read_word(REG_DMA_STATUS, rd_val);
            if (rd_val[1] !== 1'b0) errors = errors + 1;
            report_subtest(err_before, "Teste L3-1: DMEM LATENCY=3, 7x9*9x5 retangular com quebra 4KB verificada e IRQ/W1C");

            // L3-2: Retencao de RREADY/BREADY + Provocacao controlada de SLVERR sob DMEM_LATENCY=3
            err_before = errors;
            clear_axi_monitor_counters();
            axi_write_burst_with_stalls(32'h0000_0300, 8'd3, 32'hBEEF_3000, 2, 3, 2'b00);
            axi_read_burst_with_rready_stalls(32'h0000_0300, 8'd3, 32'hBEEF_3000, 1'b1, 3, 2'b00);
            if (s0_b_wait_cycles < 3 || s0_r_wait_cycles < 12) errors = errors + 1;

            expect_injected_axi_err = 1'b1;
            clear_axi_monitor_counters();
            axi_write_word(REG_DMA_ADDR_A, 32'h1000);
            axi_write_word(REG_DMA_ADDR_B, 32'h2000);
            axi_write_word(REG_DMA_ADDR_C, 32'h3000);
            axi_write_word(REG_DMA_DIM_M, 4);
            axi_write_word(REG_DMA_DIM_K, 4);
            axi_write_word(REG_DMA_DIM_N, 4);
            axi_write_word(REG_DMA_CTRL, 32'h0000_0003);
            u_dut.u_dma.reg_addr_a = 32'h0000_1002; // desalinhado dentro da DMEM -> SLVERR com LATENCY=3
            repeat (60) @(posedge clk);
            axi_read_word(REG_DMA_STATUS, rd_val);
            if (rd_val !== 32'h0000_0004 || irq_done !== 1'b0 ||
                m1_rresp_slverr_cnt == 0 || m1_rresp_decerr_cnt != 0) errors = errors + 1;
            axi_write_word(REG_DMA_STATUS, 32'h0000_0004);
            expect_injected_axi_err = 1'b0;

            for (i = 0; i < 4; i = i + 1)
                for (j = 0; j < 4; j = j + 1) begin
                    mat_A[i][j] = i + j + 1;
                    mat_B[i][j] = i - j + 2;
                end
            compute_reference(4, 4, 4);
            write_matrices_to_dmem(4, 4, 4, 32'h1000, 32'h2000, 1'b0);
            config_and_run_dma(4, 4, 4, 32'h1000, 32'h2000, 32'h3000, 1'b0);
            verify_c_in_dmem(4, 4, 32'h3000, "Lat3 (Recuperacao pos SLVERR)", 1'b0);
            report_subtest(err_before, "Teste L3-2: DMEM LATENCY=3 com retencao RREADY/BREADY, provocacao controlada de SLVERR e recuperacao");
        end
    endtask

endmodule


// -----------------------------------------------------------------------------
// Módulo Top-Level do Testbench: gera o clock de 100 MHz, instancia os quatro
// harnesses (2x2, 4x4, 8x8 e 4x4 LATENCY=3) e consolida o placar final.
// -----------------------------------------------------------------------------
module tb_dma_mem_acc;
    logic clk; // Clock principal de 100 MHz (período de 10 ns)

    initial begin
        clk = 1'b0;
        forever #5 clk = ~clk; // Clock de 100 MHz (período 10 ns)
    end

    // Instâncias independentes para validar parametrização de ARRAY_SIZE e DMEM_LATENCY
    subsys_bfm_harness #(.ARRAY_SIZE(2), .DMEM_LATENCY(1)) u_harness_2x2  (.clk(clk));
    subsys_bfm_harness #(.ARRAY_SIZE(4), .DMEM_LATENCY(1)) u_harness_4x4  (.clk(clk));
    subsys_bfm_harness #(.ARRAY_SIZE(8), .DMEM_LATENCY(1)) u_harness_8x8  (.clk(clk));
    subsys_bfm_harness #(.ARRAY_SIZE(4), .DMEM_LATENCY(3)) u_harness_lat3 (.clk(clk));

    int total_errors; // Soma total de erros das 4 instâncias
    int total_passed; // Soma total de subtestes aprovados
    int total_run;    // Soma total de subtestes executados

    initial begin
        `ifdef DUMP_VCD
            $dumpfile("tb_dma_mem_acc.vcd");
            $dumpvars(0, tb_dma_mem_acc);
        `endif

        $display("=====================================================================");
        $display("  INICIO DA VERIFICACAO UNITARIA: DMA + BARRAMENTO AXI4 + DMEM + ACC ");
        $display("=====================================================================");

        $display("\n--- Executando bateria na configuracao ARRAY_SIZE = 2 (2x2) ---");
        u_harness_2x2.run_all_tests();

        $display("\n--- Executando bateria na configuracao ARRAY_SIZE = 4 (4x4) ---");
        u_harness_4x4.run_all_tests();

        $display("\n--- Executando bateria na configuracao ARRAY_SIZE = 8 (8x8) ---");
        u_harness_8x8.run_all_tests();

        $display("\n--- Executando bateria com DMEM_LATENCY = 3 (4x4) ---");
        u_harness_lat3.run_latency3_tests();

        total_errors = u_harness_2x2.errors + u_harness_4x4.errors +
                       u_harness_8x8.errors + u_harness_lat3.errors;
        total_passed = u_harness_2x2.subtests_passed + u_harness_4x4.subtests_passed +
                       u_harness_8x8.subtests_passed + u_harness_lat3.subtests_passed;
        total_run    = u_harness_2x2.subtests_total + u_harness_4x4.subtests_total +
                       u_harness_8x8.subtests_total + u_harness_lat3.subtests_total;

        $display("\n=====================================================================");
        $display("  RESUMO FINAL ENTREGA III: %0d / %0d subtestes PASS | Erros: %0d",
                 total_passed, total_run, total_errors);
        $display("=====================================================================");

        if (total_errors != 0 || total_passed != total_run) begin
            $fatal(1, "FALHA NA VERIFICACAO DO SUBSISTEMA DMA+MEM+ACC!");
        end

        $display("ALL TESTS PASSED!");
        $finish;
    end

endmodule
