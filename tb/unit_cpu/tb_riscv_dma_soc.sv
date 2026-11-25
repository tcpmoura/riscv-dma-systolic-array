// =============================================================================
// Projeto     : RISC-V RV32I com Controlador DMA e Coprocessador Matricial
// Etapa       : Entrega Parcial IV — Integração Top-Level e Firmware
// Módulo      : tb_riscv_dma_soc.sv
// Professores : Ana Julia Fernandes de Oliveira Barros
//               Igor Dantas dos Santos Miranda
//               João Alberto Castelo Branco Oliveira
// Grupo 4     : João Vitor Carvalho Lima
//               Matheus Artur Macedo Bomfim
//               Nathalia Miranda
//               Tatiana Caetano Praxedes de Moura
// Descrição   : Testbench autoverificável de integração completa do SoC
//               (riscv_dma_soc_top: CPU RV32I real executando firmware na IMEM
//               + cpu_axi4_bridge + axi4_interconnect + axi4_dmem +
//               dma_controller + systolic_accelerator).
//
// Cobertura (27 subtestes autoverificáveis):
//   - ARRAY_SIZE = 2, 4 e 8 com DMEM_LATENCY = 1 (6 subtestes cada = 18):
//       1) Tile TxT, IRQ_EN, telemetria PERF_* e limpeza W1C via firmware
//       2) Vetores .hex do Golden Model (4x4 e 8x8) executados pela CPU real
//       3) Retangular 5x7*7x6 com concorrência real CPU (0x0100..0x01FC) vs DMA
//       4) Vetores Python 32x32 variados (seed=2026, 1024 resultados)
//       5) Configuração inválida (M=33) tratada pelo firmware (OUT_MAGIC="ERR!")
//       6) Bases customizadas (0x0FFC/0x1FFC/0x2FFC) com quebra 4 KB via firmware
//   - Contratos do firmware em ARRAY_SIZE = 4 (4 subtestes):
//       FW-1) Proteção da área reservada 0x0000..0x01FF (A=0x0000, C=0x0020,
//             B=0x0100 rejeitados com "FWER"/0x80000001; fronteira 0x0200 aceita)
//       FW-2) Limite de DESC_CPU_WORK (0, 64 com sentinela 0x0200 intacta e 65
//             rejeitado com "FWER"/0x80000002)
//       FW-3) Validação de DESC_CTRL (0->padrão 0x3, 1 sem IRQ, 2 e 4 rejeitados
//             com "FWER"/0x80000003 sem polling infinito)
//       FW-4) Timeout de polling (test_fw_timeout.hex, MAX_POLL_ITERS=12):
//             "FWER"/0x80000004 com DMA_BUSY=1 e recuperação completa pós-reset
//   - ARRAY_SIZE = 4 com DMEM_LATENCY = 3 (2 subtestes):
//       L3-1)    Retangular 7x9*9x5 com concorrência CPU vs DMA sob LATENCY=3
//       RESET-1) Reset assíncrono durante transação pendente na ponte e retomada limpa
//   - Testes dirigidos da ponte cpu_axi4_bridge (3 subtestes):
//       TRAP-1)   test_decerr_trap.hex  -> RRESP=DECERR -> cpu_error=1 -> trap=1
//       TRAP-2)   test_decerr_write.hex -> BRESP=DECERR -> cpu_error=1 -> trap=1
//       BRIDGE-1) test_bridge_ops.hex   -> SW consecutivos (1:1 AW/W/B), SB/SH
//                 com WSTRB e preservação de bytes, e LB/LBU/LH/LHU/LW
//
// Invariantes monitorados continuamente:
//   - Conformidade AXI4 na ponte cpu_axi4_bridge (estabilidade sob READY=0,
//     LEN=0, WLAST=1, ausência de X/Z e cpu_error=0 fora de cpu_ready=1).
//   - Estabilidade da interface da CPU (debug_pc, cpu_write, cpu_addr,
//     cpu_wdata, cpu_wstrb) e retire=0 enquanto cpu_valid=1 && cpu_ready=0.
//   - Correspondência estrita 1:1 entre transações concluídas na CPU e
//     handshakes AXI4 em M0 (AW/W/B para escritas; AR/R para leituras).
//   - Restrição de escritas da CPU na DMEM à área reservada 0x0000..0x01FF e
//     prova de sobreposição temporal real (escrita CPU em 0x0100..0x01FC
//     durante DMA_STATUS.BUSY = 1).
//   - Prova de autonomia do DMA: zero acessos da CPU (M0) às regiões das
//     matrizes A/B/C durante toda a execução do firmware.
//   - Escrita única por elemento válido de C, zero escritas fora de C e
//     preservação integral de todas as palavras de A/B.
// =============================================================================
`timescale 1ns / 1ps

module soc_test_harness #(
    parameter int ARRAY_SIZE   = 4,
    parameter int DMEM_LATENCY = 1,
    parameter     IMEM_FILE    = "sw/firmware/matmul_dma.hex"
) (
    input logic clk
);
    // Endereços da caixa de correio / telemetria na DMEM (palavras de 32 bits)
    localparam int W_DESC_ADDR_A     = 32'h0000 >> 2;
    localparam int W_DESC_ADDR_B     = 32'h0004 >> 2;
    localparam int W_DESC_ADDR_C     = 32'h0008 >> 2;
    localparam int W_DESC_DIM_M      = 32'h000C >> 2;
    localparam int W_DESC_DIM_K      = 32'h0010 >> 2;
    localparam int W_DESC_DIM_N      = 32'h0014 >> 2;
    localparam int W_DESC_CTRL       = 32'h0018 >> 2;
    localparam int W_DESC_CPU_WORK   = 32'h001C >> 2;

    localparam int W_OUT_MAGIC       = 32'h0020 >> 2;
    localparam int W_OUT_DMA_STATUS  = 32'h0024 >> 2;
    localparam int W_OUT_TOTAL_CYC   = 32'h0028 >> 2;
    localparam int W_OUT_COMP_CYC    = 32'h002C >> 2;
    localparam int W_OUT_MEM_READS   = 32'h0030 >> 2;
    localparam int W_OUT_HW_SIZE     = 32'h0034 >> 2;
    localparam int W_OUT_POLL_ITERS  = 32'h0038 >> 2;
    localparam int W_OUT_WORK_DONE   = 32'h003C >> 2;

    localparam logic [31:0] MAGIC_DONE      = 32'h444F_4E45; // "DONE"
    localparam logic [31:0] MAGIC_ERR       = 32'h4552_5221; // "ERR!"
    localparam logic [31:0] MAGIC_FWER      = 32'h4657_4552; // "FWER"
    localparam logic [31:0] FW_ERR_RESERVED = 32'h8000_0001; // A/B/C em 0x0000..0x01FF
    localparam logic [31:0] FW_ERR_WORK     = 32'h8000_0002; // DESC_CPU_WORK > 64
    localparam logic [31:0] FW_ERR_CTRL     = 32'h8000_0003; // DESC_CTRL fora de {0, 1, 3}
    localparam logic [31:0] FW_ERR_TIMEOUT  = 32'h8000_0004; // Limite de polling excedido

    logic        rst_n;
    logic [31:0] debug_pc;
    logic        halted;
    logic        trap;
    logic        retire;
    logic        irq_done;

    // Instância completa do SoC (CPU RV32I + Ponte AXI4 + Interconnect + DMEM + DMA + Acc)
    riscv_dma_soc_top #(
        .ARRAY_SIZE   (ARRAY_SIZE),
        .DMEM_WORDS   (4096),
        .DMEM_LATENCY (DMEM_LATENCY),
        .IMEM_DEPTH   (256),
        .IMEM_FILE    (IMEM_FILE)
    ) u_soc (
        .clk      (clk),
        .rst_n    (rst_n),
        .debug_pc (debug_pc),
        .halted   (halted),
        .trap     (trap),
        .retire   (retire),
        .irq_done (irq_done)
    );

    int errors;
    int subtests_passed;
    int subtests_total;

    // Buffers de referência e vetores .hex
    int signed mat_A     [0:31][0:31];
    int signed mat_B     [0:31][0:31];
    int signed mat_C_ref [0:31][0:31];

    logic [31:0] hex_a_4x4   [0:15];
    logic [31:0] hex_b_4x4   [0:15];
    logic [31:0] hex_c_4x4   [0:15];

    logic [31:0] hex_a_8x8   [0:63];
    logic [31:0] hex_b_8x8   [0:63];
    logic [31:0] hex_c_8x8   [0:63];

    logic [31:0] hex_a_32x32 [0:1023];
    logic [31:0] hex_b_32x32 [0:1023];
    logic [31:0] hex_c_32x32 [0:1023];

    // Snapshot de integridade das matrizes de entrada A/B
    logic [31:0] snap_a [0:1023];
    logic [31:0] snap_b [0:1023];
    int snap_a_word, snap_b_word, snap_a_cnt, snap_b_cnt;

    // Monitores contínuos da interface CPU, da ponte CPU (M0) e do DMA (M1)
    logic        prev_cpu_wait;
    logic [31:0] prev_cpu_pc;
    logic        prev_cpu_write;
    logic [31:0] prev_cpu_addr;
    logic [31:0] prev_cpu_wdata;
    logic [3:0]  prev_cpu_wstrb;

    logic        prev_m0_awvalid, prev_m0_awready;
    logic [31:0] prev_m0_awaddr;
    logic [7:0]  prev_m0_awlen;
    logic        prev_m0_wvalid, prev_m0_wready, prev_m0_wlast;
    logic [31:0] prev_m0_wdata;
    logic [3:0]  prev_m0_wstrb;
    logic        prev_m0_arvalid, prev_m0_arready;
    logic [31:0] prev_m0_araddr;
    logic [7:0]  prev_m0_arlen;

    bit          mon_active;
    bit          irq_seen;
    int          m0_wait_cycles;
    int          m1_4kb_violations;
    int          cpu_matrix_region_accesses;
    int          cpu_dmem_wr_outside_res_cnt;
    int          cpu_work_while_dma_busy;
    int          cpu_wr_done_cnt;
    int          cpu_rd_done_cnt;
    int          m0_aw_hs_cnt;
    int          m0_w_hs_cnt;
    int          m0_b_hs_cnt;
    int          m0_ar_hs_cnt;
    int          m0_r_hs_cnt;
    int          m0_b_decerr_cnt;
    int          m0_r_decerr_cnt;
    int          m0_aw_0100_cnt;
    int          m0_wstrb_0010_cnt;
    int          m0_wstrb_1000_cnt;
    int          m0_wstrb_0011_cnt;
    int          m0_wstrb_1100_cnt;
    int          m0_wstrb_1111_cnt;
    int          m1_ar_hs_cnt;
    int          m1_aw_hs_cnt;
    logic [31:0] mon_base_a, mon_end_a;
    logic [31:0] mon_base_b, mon_end_b;
    logic [31:0] mon_base_c, mon_end_c;

    // Rastreador de escrita única por elemento de C no canal W do DMA (M1)
    logic [31:0] m1_wr_track_addr;
    int          m1_w_beats_total;
    int          m1_wr_oob_cnt;
    int          m1_wr_hit_cnt [0:4095];

    initial begin
        rst_n                       = 1'b0;
        mon_active                  = 1'b0;
        irq_seen                    = 1'b0;
        errors                      = 0;
        subtests_passed             = 0;
        subtests_total              = 0;
        m0_wait_cycles              = 0;
        m1_4kb_violations           = 0;
        cpu_matrix_region_accesses  = 0;
        cpu_dmem_wr_outside_res_cnt = 0;
        cpu_work_while_dma_busy     = 0;
        cpu_wr_done_cnt             = 0;
        cpu_rd_done_cnt             = 0;
        m0_aw_hs_cnt                = 0;
        m0_w_hs_cnt                 = 0;
        m0_b_hs_cnt                 = 0;
        m0_ar_hs_cnt                = 0;
        m0_r_hs_cnt                 = 0;
        m0_b_decerr_cnt             = 0;
        m0_r_decerr_cnt             = 0;
        m0_aw_0100_cnt              = 0;
        m0_wstrb_0010_cnt           = 0;
        m0_wstrb_1000_cnt           = 0;
        m0_wstrb_0011_cnt           = 0;
        m0_wstrb_1100_cnt           = 0;
        m0_wstrb_1111_cnt           = 0;
        m1_ar_hs_cnt                = 0;
        m1_aw_hs_cnt                = 0;
    end

    // -------------------------------------------------------------------------
    // Monitor Contínuo de Protocolo CPU/AXI4 (M0/Ponte e M1/DMA) e Contratos
    // -------------------------------------------------------------------------
    always @(posedge clk) begin
        if (rst_n !== 1'b1 || !mon_active) begin
            prev_cpu_wait   <= 1'b0;
            prev_m0_awvalid <= 1'b0; prev_m0_awready <= 1'b0;
            prev_m0_wvalid  <= 1'b0; prev_m0_wready  <= 1'b0;
            prev_m0_arvalid <= 1'b0; prev_m0_arready <= 1'b0;
        end else begin
            // 1. Checagem de X/Z nos sinais de controle da ponte CPU <-> AXI4
            if (^{u_soc.m0_axi_awvalid, u_soc.m0_axi_wvalid, u_soc.m0_axi_arvalid,
                  u_soc.m0_axi_bvalid, u_soc.m0_axi_rvalid,
                  u_soc.cpu_dmem_ready, u_soc.cpu_dmem_error} === 1'bx) begin
                $display("[FAIL][SoC %0dx%0d L%0d] Sinal de controle com X/Z na ponte CPU!",
                         ARRAY_SIZE, ARRAY_SIZE, DMEM_LATENCY);
                errors = errors + 1;
            end

            // 2. Contrato rv32i_core: cpu_error deve ser 0 sempre que cpu_ready == 0
            if (u_soc.cpu_dmem_ready === 1'b0 && u_soc.cpu_dmem_error !== 1'b0) begin
                $display("[FAIL][SoC %0dx%0d L%0d] cpu_error=1 fora do ciclo cpu_ready=1!",
                         ARRAY_SIZE, ARRAY_SIZE, DMEM_LATENCY);
                errors = errors + 1;
            end

            // 2b. Estabilidade da CPU e retire=0 enquanto cpu_valid=1 && cpu_ready=0
            if (prev_cpu_wait) begin
                if (u_soc.cpu_dmem_valid !== 1'b1 ||
                    debug_pc             !== prev_cpu_pc    ||
                    u_soc.cpu_dmem_write !== prev_cpu_write ||
                    u_soc.cpu_dmem_addr  !== prev_cpu_addr  ||
                    u_soc.cpu_dmem_wdata !== prev_cpu_wdata ||
                    u_soc.cpu_dmem_wstrb !== prev_cpu_wstrb) begin
                    $display("[FAIL][SoC %0dx%0d L%0d] Instabilidade da interface CPU durante stall (PC=0x%08h prev=0x%08h)!",
                             ARRAY_SIZE, ARRAY_SIZE, DMEM_LATENCY, debug_pc, prev_cpu_pc);
                    errors = errors + 1;
                end
            end
            if (u_soc.cpu_dmem_valid && !u_soc.cpu_dmem_ready) begin
                if (retire !== 1'b0) begin
                    $display("[FAIL][SoC %0dx%0d L%0d] retire=1 enquanto CPU aguardava memoria (PC=0x%08h)!",
                             ARRAY_SIZE, ARRAY_SIZE, DMEM_LATENCY, debug_pc);
                    errors = errors + 1;
                end
                prev_cpu_wait  <= 1'b1;
                prev_cpu_pc    <= debug_pc;
                prev_cpu_write <= u_soc.cpu_dmem_write;
                prev_cpu_addr  <= u_soc.cpu_dmem_addr;
                prev_cpu_wdata <= u_soc.cpu_dmem_wdata;
                prev_cpu_wstrb <= u_soc.cpu_dmem_wstrb;
            end else begin
                prev_cpu_wait  <= 1'b0;
            end

            // 2c. Contagem 1:1 de transações concluídas na CPU vs handshakes AXI4 (M0)
            if (u_soc.cpu_dmem_valid && u_soc.cpu_dmem_ready) begin
                if (u_soc.cpu_dmem_write) cpu_wr_done_cnt = cpu_wr_done_cnt + 1;
                else                      cpu_rd_done_cnt = cpu_rd_done_cnt + 1;
            end
            if (u_soc.m0_axi_awvalid && u_soc.m0_axi_awready) begin
                m0_aw_hs_cnt = m0_aw_hs_cnt + 1;
                if (u_soc.m0_axi_awaddr == 32'h0000_0100) m0_aw_0100_cnt = m0_aw_0100_cnt + 1;
            end
            if (u_soc.m0_axi_wvalid  && u_soc.m0_axi_wready) begin
                m0_w_hs_cnt = m0_w_hs_cnt + 1;
                case (u_soc.m0_axi_wstrb)
                    4'b0010: m0_wstrb_0010_cnt = m0_wstrb_0010_cnt + 1;
                    4'b1000: m0_wstrb_1000_cnt = m0_wstrb_1000_cnt + 1;
                    4'b0011: m0_wstrb_0011_cnt = m0_wstrb_0011_cnt + 1;
                    4'b1100: m0_wstrb_1100_cnt = m0_wstrb_1100_cnt + 1;
                    4'b1111: m0_wstrb_1111_cnt = m0_wstrb_1111_cnt + 1;
                    default: ;
                endcase
            end
            if (u_soc.m0_axi_bvalid  && u_soc.m0_axi_bready) begin
                m0_b_hs_cnt = m0_b_hs_cnt + 1;
                if (u_soc.m0_axi_bresp === 2'b11) m0_b_decerr_cnt = m0_b_decerr_cnt + 1;
            end
            if (u_soc.m0_axi_arvalid && u_soc.m0_axi_arready) m0_ar_hs_cnt = m0_ar_hs_cnt + 1;
            if (u_soc.m0_axi_rvalid  && u_soc.m0_axi_rready) begin
                m0_r_hs_cnt = m0_r_hs_cnt + 1;
                if (u_soc.m0_axi_rresp === 2'b11) m0_r_decerr_cnt = m0_r_decerr_cnt + 1;
            end

            // 3. Regras AXI4 da ponte (LEN=0, WLAST=1 e estabilidade durante READY=0)
            if (u_soc.m0_axi_awvalid && (u_soc.m0_axi_awlen !== 8'd0 || u_soc.m0_axi_awsize !== 3'b010)) begin
                $display("[FAIL][SoC %0dx%0d L%0d] Ponte M0 AWLEN/AWSIZE invalidos!",
                         ARRAY_SIZE, ARRAY_SIZE, DMEM_LATENCY);
                errors = errors + 1;
            end
            if (u_soc.m0_axi_wvalid && u_soc.m0_axi_wlast !== 1'b1) begin
                $display("[FAIL][SoC %0dx%0d L%0d] Ponte M0 WLAST!=1 em beat unico!",
                         ARRAY_SIZE, ARRAY_SIZE, DMEM_LATENCY);
                errors = errors + 1;
            end
            if (u_soc.m0_axi_arvalid && (u_soc.m0_axi_arlen !== 8'd0 || u_soc.m0_axi_arsize !== 3'b010)) begin
                $display("[FAIL][SoC %0dx%0d L%0d] Ponte M0 ARLEN/ARSIZE invalidos!",
                         ARRAY_SIZE, ARRAY_SIZE, DMEM_LATENCY);
                errors = errors + 1;
            end

            if (prev_m0_awvalid && !prev_m0_awready) begin
                if (!u_soc.m0_axi_awvalid || u_soc.m0_axi_awaddr !== prev_m0_awaddr || u_soc.m0_axi_awlen !== prev_m0_awlen) begin
                    $display("[FAIL][SoC %0dx%0d L%0d] Violacao AXI4 M0 AW sob AWREADY=0!",
                             ARRAY_SIZE, ARRAY_SIZE, DMEM_LATENCY);
                    errors = errors + 1;
                end
            end
            if (prev_m0_wvalid && !prev_m0_wready) begin
                if (!u_soc.m0_axi_wvalid || u_soc.m0_axi_wdata !== prev_m0_wdata ||
                    u_soc.m0_axi_wstrb !== prev_m0_wstrb || u_soc.m0_axi_wlast !== prev_m0_wlast) begin
                    $display("[FAIL][SoC %0dx%0d L%0d] Violacao AXI4 M0 W sob WREADY=0!",
                             ARRAY_SIZE, ARRAY_SIZE, DMEM_LATENCY);
                    errors = errors + 1;
                end
            end
            if (prev_m0_arvalid && !prev_m0_arready) begin
                if (!u_soc.m0_axi_arvalid || u_soc.m0_axi_araddr !== prev_m0_araddr || u_soc.m0_axi_arlen !== prev_m0_arlen) begin
                    $display("[FAIL][SoC %0dx%0d L%0d] Violacao AXI4 M0 AR sob ARREADY=0!",
                             ARRAY_SIZE, ARRAY_SIZE, DMEM_LATENCY);
                    errors = errors + 1;
                end
            end

            // Contagem de ciclos de espera da CPU disputando barramento
            if ((u_soc.m0_axi_awvalid && !u_soc.m0_axi_awready) ||
                (u_soc.m0_axi_wvalid  && !u_soc.m0_axi_wready)  ||
                (u_soc.m0_axi_arvalid && !u_soc.m0_axi_arready)) begin
                m0_wait_cycles = m0_wait_cycles + 1;
            end

            // 4. Autonomia do DMA, restrição da área reservada e concorrência real com BUSY=1
            if (u_soc.m0_axi_awvalid && u_soc.m0_axi_awready) begin
                if ((u_soc.m0_axi_awaddr >= mon_base_a && u_soc.m0_axi_awaddr < mon_end_a) ||
                    (u_soc.m0_axi_awaddr >= mon_base_b && u_soc.m0_axi_awaddr < mon_end_b) ||
                    (u_soc.m0_axi_awaddr >= mon_base_c && u_soc.m0_axi_awaddr < mon_end_c)) begin
                    cpu_matrix_region_accesses = cpu_matrix_region_accesses + 1;
                end
                // Toda escrita da CPU na DMEM (não-MMIO) deve permanecer em 0x0000..0x01FF
                if (u_soc.m0_axi_awaddr[31:14] == 18'd0 && u_soc.m0_axi_awaddr >= 32'h0000_0200) begin
                    cpu_dmem_wr_outside_res_cnt = cpu_dmem_wr_outside_res_cnt + 1;
                end
                // Conta escritas de trabalho da CPU (0x0100..0x01FC) aceitas enquanto DMA_STATUS.BUSY == 1
                if (u_soc.m0_axi_awaddr >= 32'h0000_0100 && u_soc.m0_axi_awaddr < 32'h0000_0200 &&
                    u_soc.u_subsys.u_dma.status_busy === 1'b1) begin
                    cpu_work_while_dma_busy = cpu_work_while_dma_busy + 1;
                end
            end
            if (u_soc.m0_axi_arvalid && u_soc.m0_axi_arready) begin
                if ((u_soc.m0_axi_araddr >= mon_base_a && u_soc.m0_axi_araddr < mon_end_a) ||
                    (u_soc.m0_axi_araddr >= mon_base_b && u_soc.m0_axi_araddr < mon_end_b) ||
                    (u_soc.m0_axi_araddr >= mon_base_c && u_soc.m0_axi_araddr < mon_end_c)) begin
                    cpu_matrix_region_accesses = cpu_matrix_region_accesses + 1;
                end
            end

            // 5. Fronteira 4 KB e rastreador de escrita única do DMA (M1)
            if (u_soc.u_subsys.m1_axi_arvalid && u_soc.u_subsys.m1_axi_arready) begin
                int unsigned ar_end_4k;
                m1_ar_hs_cnt = m1_ar_hs_cnt + 1;
                ar_end_4k = int'(u_soc.u_subsys.m1_axi_araddr[11:0]) + ((int'(u_soc.u_subsys.m1_axi_arlen) + 1) * 4);
                if (ar_end_4k > 4096) m1_4kb_violations = m1_4kb_violations + 1;
            end
            if (u_soc.u_subsys.m1_axi_awvalid && u_soc.u_subsys.m1_axi_awready) begin
                int unsigned aw_end_4k;
                m1_aw_hs_cnt = m1_aw_hs_cnt + 1;
                aw_end_4k = int'(u_soc.u_subsys.m1_axi_awaddr[11:0]) + ((int'(u_soc.u_subsys.m1_axi_awlen) + 1) * 4);
                if (aw_end_4k > 4096) m1_4kb_violations = m1_4kb_violations + 1;
                m1_wr_track_addr = u_soc.u_subsys.m1_axi_awaddr;
            end
            if (u_soc.u_subsys.m1_axi_wvalid && u_soc.u_subsys.m1_axi_wready) begin
                m1_w_beats_total = m1_w_beats_total + 1;
                if (m1_wr_track_addr[31:14] == 18'd0) begin
                    m1_wr_hit_cnt[m1_wr_track_addr[13:2]] = m1_wr_hit_cnt[m1_wr_track_addr[13:2]] + 1;
                end else begin
                    m1_wr_oob_cnt = m1_wr_oob_cnt + 1;
                end
                m1_wr_track_addr = m1_wr_track_addr + 32'd4;
            end

            if (irq_done === 1'b1) irq_seen = 1'b1;

            // Atualiza estado anterior para checagem no próximo ciclo
            prev_m0_awvalid <= u_soc.m0_axi_awvalid;
            prev_m0_awready <= u_soc.m0_axi_awready;
            prev_m0_awaddr  <= u_soc.m0_axi_awaddr;
            prev_m0_awlen   <= u_soc.m0_axi_awlen;
            prev_m0_wvalid  <= u_soc.m0_axi_wvalid;
            prev_m0_wready  <= u_soc.m0_axi_wready;
            prev_m0_wdata   <= u_soc.m0_axi_wdata;
            prev_m0_wstrb   <= u_soc.m0_axi_wstrb;
            prev_m0_wlast   <= u_soc.m0_axi_wlast;
            prev_m0_arvalid <= u_soc.m0_axi_arvalid;
            prev_m0_arready <= u_soc.m0_axi_arready;
            prev_m0_araddr  <= u_soc.m0_axi_araddr;
            prev_m0_arlen   <= u_soc.m0_axi_arlen;
        end
    end

    // -------------------------------------------------------------------------
    // Tarefas Auxiliares
    // -------------------------------------------------------------------------
    task automatic report_subtest(input int err_before, input string label);
        begin
            subtests_total = subtests_total + 1;
            if (errors == err_before) begin
                subtests_passed = subtests_passed + 1;
                $display("[PASS][SoC %0dx%0d L%0d] %s", ARRAY_SIZE, ARRAY_SIZE, DMEM_LATENCY, label);
            end else begin
                $display("[FAIL][SoC %0dx%0d L%0d] %s (erros: %0d)",
                         ARRAY_SIZE, ARRAY_SIZE, DMEM_LATENCY, label, errors - err_before);
            end
        end
    endtask

    task automatic compute_reference(input int M, input int K, input int N);
        int i, j, k;
        int signed acc;
        begin
            for (i = 0; i < M; i = i + 1)
                for (j = 0; j < N; j = j + 1) begin
                    acc = 0;
                    for (k = 0; k < K; k = k + 1)
                        acc = acc + (mat_A[i][k] * mat_B[k][j]);
                    mat_C_ref[i][j] = acc;
                end
        end
    endtask

    task automatic clear_trackers(
        input logic [31:0] base_a, input logic [31:0] bytes_a,
        input logic [31:0] base_b, input logic [31:0] bytes_b,
        input logic [31:0] base_c, input logic [31:0] bytes_c
    );
        int idx_w;
        begin
            irq_seen                    = 1'b0;
            m0_wait_cycles              = 0;
            m1_4kb_violations           = 0;
            cpu_matrix_region_accesses  = 0;
            cpu_dmem_wr_outside_res_cnt = 0;
            cpu_work_while_dma_busy     = 0;
            cpu_wr_done_cnt             = 0;
            cpu_rd_done_cnt             = 0;
            m0_aw_hs_cnt                = 0;
            m0_w_hs_cnt                 = 0;
            m0_b_hs_cnt                 = 0;
            m0_ar_hs_cnt                = 0;
            m0_r_hs_cnt                 = 0;
            m0_b_decerr_cnt             = 0;
            m0_r_decerr_cnt             = 0;
            m0_aw_0100_cnt              = 0;
            m0_wstrb_0010_cnt           = 0;
            m0_wstrb_1000_cnt           = 0;
            m0_wstrb_0011_cnt           = 0;
            m0_wstrb_1100_cnt           = 0;
            m0_wstrb_1111_cnt           = 0;
            m1_ar_hs_cnt                = 0;
            m1_aw_hs_cnt                = 0;
            mon_base_a                  = base_a;
            mon_end_a                   = base_a + bytes_a;
            mon_base_b                  = base_b;
            mon_end_b                   = base_b + bytes_b;
            mon_base_c                  = base_c;
            mon_end_c                   = base_c + bytes_c;
            m1_wr_track_addr            = 32'd0;
            m1_w_beats_total            = 0;
            m1_wr_oob_cnt               = 0;
            for (idx_w = 0; idx_w < 4096; idx_w = idx_w + 1)
                m1_wr_hit_cnt[idx_w] = 0;
        end
    endtask

    // Confere correspondência estrita 1:1 entre transações da CPU e handshakes AXI4 M0
    task automatic verify_m0_1to1(input string ctx);
        begin
            if (cpu_wr_done_cnt !== m0_aw_hs_cnt ||
                cpu_wr_done_cnt !== m0_w_hs_cnt  ||
                cpu_wr_done_cnt !== m0_b_hs_cnt) begin
                $display("[FAIL][SoC %0dx%0d L%0d] %s: 1:1 escrita quebrado (cpu_wr=%0d AW=%0d W=%0d B=%0d)",
                         ARRAY_SIZE, ARRAY_SIZE, DMEM_LATENCY, ctx,
                         cpu_wr_done_cnt, m0_aw_hs_cnt, m0_w_hs_cnt, m0_b_hs_cnt);
                errors = errors + 1;
            end
            if (cpu_rd_done_cnt !== m0_ar_hs_cnt ||
                cpu_rd_done_cnt !== m0_r_hs_cnt) begin
                $display("[FAIL][SoC %0dx%0d L%0d] %s: 1:1 leitura quebrado (cpu_rd=%0d AR=%0d R=%0d)",
                         ARRAY_SIZE, ARRAY_SIZE, DMEM_LATENCY, ctx,
                         cpu_rd_done_cnt, m0_ar_hs_cnt, m0_r_hs_cnt);
                errors = errors + 1;
            end
        end
    endtask

    // Prepara DMEM (descritor + matrizes A/B + pré-preenchimento de C) com CPU em reset
    task automatic prepare_and_run_soc(
        input int M, input int K, input int N,
        input logic [31:0] base_a, input logic [31:0] base_b, input logic [31:0] base_c,
        input logic [31:0] ctrl_cmd,
        input int cpu_work_iters,
        input bit load_ab_and_clear_c,
        input int max_cycles
    );
        int i, j, cyc;
        begin
            mon_active = 1'b0;
            rst_n      = 1'b0;
            repeat (3) @(posedge clk);

            // Grava bloco descritor na área geral da DMEM (0x0000..0x001C)
            u_soc.u_subsys.u_dmem.memory[W_DESC_ADDR_A]   = base_a;
            u_soc.u_subsys.u_dmem.memory[W_DESC_ADDR_B]   = base_b;
            u_soc.u_subsys.u_dmem.memory[W_DESC_ADDR_C]   = base_c;
            u_soc.u_subsys.u_dmem.memory[W_DESC_DIM_M]    = M;
            u_soc.u_subsys.u_dmem.memory[W_DESC_DIM_K]    = K;
            u_soc.u_subsys.u_dmem.memory[W_DESC_DIM_N]    = N;
            u_soc.u_subsys.u_dmem.memory[W_DESC_CTRL]     = ctrl_cmd;
            u_soc.u_subsys.u_dmem.memory[W_DESC_CPU_WORK] = cpu_work_iters;

            // Zera telemetria de saída antes da execução
            for (i = W_OUT_MAGIC; i <= W_OUT_WORK_DONE; i = i + 1)
                u_soc.u_subsys.u_dmem.memory[i] = 32'd0;

            if (load_ab_and_clear_c) begin
                snap_a_word = base_a >> 2;
                snap_b_word = base_b >> 2;
                snap_a_cnt  = M * K;
                snap_b_cnt  = K * N;
                for (i = 0; i < M; i = i + 1)
                    for (j = 0; j < K; j = j + 1) begin
                        u_soc.u_subsys.u_dmem.memory[snap_a_word + i*K + j] = {24'd0, mat_A[i][j][7:0]};
                        snap_a[i*K + j] = {24'd0, mat_A[i][j][7:0]};
                    end
                for (i = 0; i < K; i = i + 1)
                    for (j = 0; j < N; j = j + 1) begin
                        u_soc.u_subsys.u_dmem.memory[snap_b_word + i*N + j] = {24'd0, mat_B[i][j][7:0]};
                        snap_b[i*N + j] = {24'd0, mat_B[i][j][7:0]};
                    end
                for (i = 0; i < M * N; i = i + 1)
                    u_soc.u_subsys.u_dmem.memory[(base_c >> 2) + i] = 32'hA5A5_A5A5;

                clear_trackers(base_a, M*K*4, base_b, K*N*4, base_c, M*N*4);
            end else begin
                clear_trackers(32'h1000, 32'h1000, 32'h2000, 32'h1000, 32'h3000, 32'h1000);
            end

            // Libera o reset da CPU RV32I para executar o firmware autonomamente
            @(negedge clk);
            mon_active = 1'b1;
            rst_n      = 1'b1;

            cyc = 0;
            while (!halted && !trap && cyc < max_cycles) begin
                @(posedge clk);
                cyc = cyc + 1;
            end
            @(posedge clk);
            mon_active = 1'b0;

            if (cyc >= max_cycles) begin
                $display("[FAIL][SoC %0dx%0d L%0d] Timeout (%0d ciclos) aguardando halted/trap! PC=0x%08h",
                         ARRAY_SIZE, ARRAY_SIZE, DMEM_LATENCY, max_cycles, debug_pc);
                errors = errors + 1;
            end
        end
    endtask

    task automatic verify_normal_execution(
        input int M, input int K, input int N,
        input logic [31:0] base_c,
        input int exp_cpu_work,
        input bit exp_irq,
        input string ctx
    );
        int i, j, idx_w, c_start, c_end;
        int exp_comp, exp_reads;
        logic [31:0] got_magic, got_status, got_tot, got_cmp, got_rds, got_hwsz, got_work;
        int signed got_s;
        bit mismatch;
        begin
            got_magic  = u_soc.u_subsys.u_dmem.memory[W_OUT_MAGIC];
            got_status = u_soc.u_subsys.u_dmem.memory[W_OUT_DMA_STATUS];
            got_tot    = u_soc.u_subsys.u_dmem.memory[W_OUT_TOTAL_CYC];
            got_cmp    = u_soc.u_subsys.u_dmem.memory[W_OUT_COMP_CYC];
            got_rds    = u_soc.u_subsys.u_dmem.memory[W_OUT_MEM_READS];
            got_hwsz   = u_soc.u_subsys.u_dmem.memory[W_OUT_HW_SIZE];
            got_work   = u_soc.u_subsys.u_dmem.memory[W_OUT_WORK_DONE];

            verify_m0_1to1(ctx);

            if (!halted || trap) begin
                $display("[FAIL][SoC %0dx%0d L%0d] %s: halted=%b trap=%b (esperado halted=1 trap=0)",
                         ARRAY_SIZE, ARRAY_SIZE, DMEM_LATENCY, ctx, halted, trap);
                errors = errors + 1;
            end
            if (got_magic !== MAGIC_DONE || got_status !== 32'h0000_0002) begin
                $display("[FAIL][SoC %0dx%0d L%0d] %s: OUT_MAGIC=0x%08h OUT_STATUS=0x%08h",
                         ARRAY_SIZE, ARRAY_SIZE, DMEM_LATENCY, ctx, got_magic, got_status);
                errors = errors + 1;
            end
            if (irq_seen !== exp_irq || irq_done !== 1'b0) begin
                $display("[FAIL][SoC %0dx%0d L%0d] %s: irq_seen=%b(esp %b) irq_done_final=%b",
                         ARRAY_SIZE, ARRAY_SIZE, DMEM_LATENCY, ctx, irq_seen, exp_irq, irq_done);
                errors = errors + 1;
            end
            if (got_hwsz !== ARRAY_SIZE) begin
                $display("[FAIL][SoC %0dx%0d L%0d] %s: HW_ARRAY_SIZE=%0d != %0d",
                         ARRAY_SIZE, ARRAY_SIZE, DMEM_LATENCY, ctx, got_hwsz, ARRAY_SIZE);
                errors = errors + 1;
            end
            if (cpu_matrix_region_accesses !== 0) begin
                $display("[FAIL][SoC %0dx%0d L%0d] %s: CPU acessou regioes de matriz (%0d vezes)!",
                         ARRAY_SIZE, ARRAY_SIZE, DMEM_LATENCY, ctx, cpu_matrix_region_accesses);
                errors = errors + 1;
            end
            if (cpu_dmem_wr_outside_res_cnt !== 0) begin
                $display("[FAIL][SoC %0dx%0d L%0d] %s: CPU escreveu fora da area reservada 0x0000..0x01FF (%0d vezes)!",
                         ARRAY_SIZE, ARRAY_SIZE, DMEM_LATENCY, ctx, cpu_dmem_wr_outside_res_cnt);
                errors = errors + 1;
            end
            if (m1_4kb_violations !== 0 || m1_wr_oob_cnt !== 0) begin
                $display("[FAIL][SoC %0dx%0d L%0d] %s: 4KB_viol=%0d oob_wr=%0d",
                         ARRAY_SIZE, ARRAY_SIZE, DMEM_LATENCY, ctx, m1_4kb_violations, m1_wr_oob_cnt);
                errors = errors + 1;
            end

            // Conferência exata dos contadores do Golden Model
            exp_comp = ((M + ARRAY_SIZE - 1) / ARRAY_SIZE) *
                       ((N + ARRAY_SIZE - 1) / ARRAY_SIZE) *
                       ((K + ARRAY_SIZE - 1) / ARRAY_SIZE) *
                       (3 * ARRAY_SIZE - 2);
            exp_reads = ((N + ARRAY_SIZE - 1) / ARRAY_SIZE) * (M * K) +
                        ((M + ARRAY_SIZE - 1) / ARRAY_SIZE) * (K * N);
            if (got_cmp !== exp_comp || got_rds !== exp_reads || got_tot == 0) begin
                $display("[FAIL][SoC %0dx%0d L%0d] %s: PERF tot=%0d cmp=%0d(esp %0d) rds=%0d(esp %0d)",
                         ARRAY_SIZE, ARRAY_SIZE, DMEM_LATENCY, ctx, got_tot, got_cmp, exp_comp, got_rds, exp_reads);
                errors = errors + 1;
            end

            // Preservação integral das matrizes de entrada A e B
            for (i = 0; i < snap_a_cnt; i = i + 1)
                if (u_soc.u_subsys.u_dmem.memory[snap_a_word + i] !== snap_a[i]) begin
                    $display("[FAIL][SoC %0dx%0d L%0d] %s: A alterada no indice %0d!",
                             ARRAY_SIZE, ARRAY_SIZE, DMEM_LATENCY, ctx, i);
                    errors = errors + 1;
                end
            for (i = 0; i < snap_b_cnt; i = i + 1)
                if (u_soc.u_subsys.u_dmem.memory[snap_b_word + i] !== snap_b[i]) begin
                    $display("[FAIL][SoC %0dx%0d L%0d] %s: B alterada no indice %0d!",
                             ARRAY_SIZE, ARRAY_SIZE, DMEM_LATENCY, ctx, i);
                    errors = errors + 1;
                end

            // Escrita única em C, zero escritas fora de C e comparação bit a bit
            c_start = base_c >> 2;
            c_end   = c_start + (M * N);
            if (m1_w_beats_total !== (M * N)) begin
                $display("[FAIL][SoC %0dx%0d L%0d] %s: m1_w_beats_total=%0d != %0d",
                         ARRAY_SIZE, ARRAY_SIZE, DMEM_LATENCY, ctx, m1_w_beats_total, M * N);
                errors = errors + 1;
            end
            for (idx_w = 0; idx_w < 4096; idx_w = idx_w + 1)
                if ((idx_w < c_start || idx_w >= c_end) && m1_wr_hit_cnt[idx_w] !== 0) begin
                    $display("[FAIL][SoC %0dx%0d L%0d] %s: Escrita DMA fora de C em 0x%04h!",
                             ARRAY_SIZE, ARRAY_SIZE, DMEM_LATENCY, ctx, idx_w << 2);
                    errors = errors + 1;
                end

            mismatch = 1'b0;
            for (i = 0; i < M && !mismatch; i = i + 1)
                for (j = 0; j < N && !mismatch; j = j + 1) begin
                    idx_w = c_start + (i * N + j);
                    if (m1_wr_hit_cnt[idx_w] !== 1) begin
                        $display("[FAIL][SoC %0dx%0d L%0d] %s: C[%0d][%0d] recebeu %0d escritas!",
                                 ARRAY_SIZE, ARRAY_SIZE, DMEM_LATENCY, ctx, i, j, m1_wr_hit_cnt[idx_w]);
                        errors = errors + 1;
                        mismatch = 1'b1;
                    end
                    got_s = $signed(u_soc.u_subsys.u_dmem.memory[idx_w]);
                    if (got_s !== mat_C_ref[i][j]) begin
                        $display("[FAIL][SoC %0dx%0d L%0d] %s: C[%0d][%0d]=%0d != esp %0d",
                                 ARRAY_SIZE, ARRAY_SIZE, DMEM_LATENCY, ctx, i, j, got_s, mat_C_ref[i][j]);
                        errors = errors + 1;
                        mismatch = 1'b1;
                    end
                end

            // Verifica trabalho concorrente da CPU na área privada 0x0100..0x01FC
            if (exp_cpu_work > 0) begin
                if (got_work == 0 || got_work > exp_cpu_work) begin
                    $display("[FAIL][SoC %0dx%0d L%0d] %s: OUT_WORK_DONE=%0d (esp 1..%0d)",
                             ARRAY_SIZE, ARRAY_SIZE, DMEM_LATENCY, ctx, got_work, exp_cpu_work);
                    errors = errors + 1;
                end
                if (cpu_work_while_dma_busy < 1) begin
                    $display("[FAIL][SoC %0dx%0d L%0d] %s: Sem prova de concorrencia real (cpu_work_while_dma_busy=%0d)",
                             ARRAY_SIZE, ARRAY_SIZE, DMEM_LATENCY, ctx, cpu_work_while_dma_busy);
                    errors = errors + 1;
                end
                for (i = 0; i < got_work; i = i + 1)
                    if (u_soc.u_subsys.u_dmem.memory[(32'h0100 >> 2) + i] !== (32'h0000_A500 + i)) begin
                        $display("[FAIL][SoC %0dx%0d L%0d] %s: Area privada CPU 0x%04h corrompida!",
                                 ARRAY_SIZE, ARRAY_SIZE, DMEM_LATENCY, ctx, 32'h0100 + i*4);
                        errors = errors + 1;
                    end
            end else if (got_work !== 32'd0) begin
                $display("[FAIL][SoC %0dx%0d L%0d] %s: OUT_WORK_DONE=%0d (esp 0)",
                         ARRAY_SIZE, ARRAY_SIZE, DMEM_LATENCY, ctx, got_work);
                errors = errors + 1;
            end
        end
    endtask

    // -------------------------------------------------------------------------
    // Suíte Principal do SoC (6 subtestes por malha física 2x2, 4x4 e 8x8)
    // -------------------------------------------------------------------------
    task automatic run_soc_suite();
        int i, j, err_before;
        logic [31:0] sent_before, sent_after;
        begin
            errors          = 0;
            subtests_passed = 0;
            subtests_total  = 0;

            // TESTE 1: Tile TxT via firmware + IRQ + telemetria PERF_* + W1C
            err_before = errors;
            for (i = 0; i < ARRAY_SIZE; i = i + 1)
                for (j = 0; j < ARRAY_SIZE; j = j + 1) begin
                    mat_A[i][j] = (((i * ARRAY_SIZE + j) * 5) % 200) - 100;
                    mat_B[i][j] = (i == j) ? 3 : -2;
                end
            compute_reference(ARRAY_SIZE, ARRAY_SIZE, ARRAY_SIZE);
            prepare_and_run_soc(ARRAY_SIZE, ARRAY_SIZE, ARRAY_SIZE,
                                32'h1000, 32'h2000, 32'h3000, 32'h3, 0, 1'b1, 20000);
            verify_normal_execution(ARRAY_SIZE, ARRAY_SIZE, ARRAY_SIZE, 32'h3000, 0, 1'b1, "Teste 1 (TxT)");
            report_subtest(err_before, "Teste 1: Firmware RV32I configura DMA (TxT), IRQ, PERF_* e W1C sem CPU mover matrizes");

            // TESTE 2: Vetores .hex Golden Model (4x4 e 8x8) pela CPU real
            err_before = errors;
            $readmemh("sw/golden_model/test_vectors/mat_4x4_A.hex", hex_a_4x4);
            $readmemh("sw/golden_model/test_vectors/mat_4x4_B.hex", hex_b_4x4);
            $readmemh("sw/golden_model/test_vectors/mat_4x4_C_expected.hex", hex_c_4x4);
            for (i = 0; i < 4; i = i + 1)
                for (j = 0; j < 4; j = j + 1) begin
                    mat_A[i][j]     = $signed(hex_a_4x4[i*4 + j][7:0]);
                    mat_B[i][j]     = $signed(hex_b_4x4[i*4 + j][7:0]);
                    mat_C_ref[i][j] = $signed(hex_c_4x4[i*4 + j]);
                end
            prepare_and_run_soc(4, 4, 4, 32'h1000, 32'h2000, 32'h3000, 32'h3, 0, 1'b1, 30000);
            verify_normal_execution(4, 4, 4, 32'h3000, 0, 1'b1, "Teste 2a (HEX 4x4)");

            $readmemh("sw/golden_model/test_vectors/mat_8x8x8_A.hex", hex_a_8x8);
            $readmemh("sw/golden_model/test_vectors/mat_8x8x8_B.hex", hex_b_8x8);
            $readmemh("sw/golden_model/test_vectors/mat_8x8x8_C_expected.hex", hex_c_8x8);
            for (i = 0; i < 8; i = i + 1)
                for (j = 0; j < 8; j = j + 1) begin
                    mat_A[i][j]     = $signed(hex_a_8x8[i*8 + j][7:0]);
                    mat_B[i][j]     = $signed(hex_b_8x8[i*8 + j][7:0]);
                    mat_C_ref[i][j] = $signed(hex_c_8x8[i*8 + j]);
                end
            prepare_and_run_soc(8, 8, 8, 32'h1000, 32'h2000, 32'h3000, 32'h3, 0, 1'b1, 60000);
            verify_normal_execution(8, 8, 8, 32'h3000, 0, 1'b1, "Teste 2b (HEX 8x8)");
            report_subtest(err_before, "Teste 2: Vetores .hex Golden Model (4x4 e 8x8) executados pelo firmware RV32I");

            // TESTE 3: Retangular 5x7*7x6 + Concorrência Real CPU (0x0100..0x01FC) vs DMA
            err_before = errors;
            for (i = 0; i < 5; i = i + 1)
                for (j = 0; j < 7; j = j + 1)
                    mat_A[i][j] = ((i * 7 + j) % 23) - 11;
            for (i = 0; i < 7; i = i + 1)
                for (j = 0; j < 6; j = j + 1)
                    mat_B[i][j] = ((i * 6 + j) % 19) - 9;
            compute_reference(5, 7, 6);
            u_soc.u_subsys.u_dmem.memory[32'h2FFC >> 2] = 32'hDEAD_CAFE;
            u_soc.u_subsys.u_dmem.memory[32'h3078 >> 2] = 32'hBEEF_1234;
            prepare_and_run_soc(5, 7, 6, 32'h1000, 32'h2000, 32'h3000, 32'h3, 16, 1'b1, 80000);
            verify_normal_execution(5, 7, 6, 32'h3000, 16, 1'b1, "Teste 3 (5x7*7x6 + CPU work)");
            sent_before = u_soc.u_subsys.u_dmem.memory[32'h2FFC >> 2];
            sent_after  = u_soc.u_subsys.u_dmem.memory[32'h3078 >> 2];
            if (sent_before !== 32'hDEAD_CAFE || sent_after !== 32'hBEEF_1234) begin
                $display("[FAIL][SoC %0dx%0d L%0d] Teste 3: Sentinela corrompida (0x%08h, 0x%08h)",
                         ARRAY_SIZE, ARRAY_SIZE, DMEM_LATENCY, sent_before, sent_after);
                errors = errors + 1;
            end
            report_subtest(err_before, "Teste 3: Matriz retangular 5x7*7x6 com concorrencia real CPU (0x0100..0x01FC, BUSY=1) vs DMA");

            // TESTE 4: Vetores Python 32x32 variados (seed=2026, 1024 resultados) pela CPU real
            err_before = errors;
            $readmemh("sw/golden_model/test_vectors/mat_32x32_varied_A.hex", hex_a_32x32);
            $readmemh("sw/golden_model/test_vectors/mat_32x32_varied_B.hex", hex_b_32x32);
            $readmemh("sw/golden_model/test_vectors/mat_32x32_varied_C_expected.hex", hex_c_32x32);
            for (i = 0; i < 32; i = i + 1)
                for (j = 0; j < 32; j = j + 1) begin
                    mat_A[i][j]     = $signed(hex_a_32x32[i*32 + j][7:0]);
                    mat_B[i][j]     = $signed(hex_b_32x32[i*32 + j][7:0]);
                    mat_C_ref[i][j] = $signed(hex_c_32x32[i*32 + j]);
                end
            prepare_and_run_soc(32, 32, 32, 32'h1000, 32'h2000, 32'h3000, 32'h3, 8, 1'b1, 300000);
            verify_normal_execution(32, 32, 32, 32'h3000, 8, 1'b1, "Teste 4 (Python 32x32 variado)");
            $display("[INFO][SoC %0dx%0d L%0d] Teste 4 (32x32 variado via CPU RV32I): DMA_TOTAL_CYCLES=%0d COMP=%0d READS=%0d POLL_ITERS=%0d M0_WAIT=%0d WORK_DURING_BUSY=%0d",
                     ARRAY_SIZE, ARRAY_SIZE, DMEM_LATENCY,
                     u_soc.u_subsys.u_dmem.memory[W_OUT_TOTAL_CYC],
                     u_soc.u_subsys.u_dmem.memory[W_OUT_COMP_CYC],
                     u_soc.u_subsys.u_dmem.memory[W_OUT_MEM_READS],
                     u_soc.u_subsys.u_dmem.memory[W_OUT_POLL_ITERS],
                     m0_wait_cycles, cpu_work_while_dma_busy);
            report_subtest(err_before, "Teste 4: Vetores Python 32x32 variados (seed=2026, 1024 resultados) comandados pela CPU RV32I");

            // TESTE 5: Configuração inválida (M=33) -> DMA ERR=1 -> firmware grava OUT_MAGIC="ERR!"
            err_before = errors;
            u_soc.u_subsys.u_dmem.memory[32'h3000 >> 2] = 32'hC0DE_5555;
            prepare_and_run_soc(33, 4, 4, 32'h1000, 32'h2000, 32'h3000, 32'h3, 0, 1'b0, 10000);
            verify_m0_1to1("Teste 5 (ERR)");
            if (!halted || trap ||
                u_soc.u_subsys.u_dmem.memory[W_OUT_MAGIC]      !== MAGIC_ERR ||
                u_soc.u_subsys.u_dmem.memory[W_OUT_DMA_STATUS] !== 32'h0000_0004 ||
                m1_w_beats_total !== 0 || m1_ar_hs_cnt !== 0 || m1_aw_hs_cnt !== 0 ||
                cpu_dmem_wr_outside_res_cnt !== 0 ||
                u_soc.u_subsys.u_dmem.memory[32'h3000 >> 2]    !== 32'hC0DE_5555 ||
                irq_done !== 1'b0) begin
                $display("[FAIL][SoC %0dx%0d L%0d] Teste 5 (ERR): halted=%b trap=%b magic=0x%08h st=0x%08h w=%0d",
                         ARRAY_SIZE, ARRAY_SIZE, DMEM_LATENCY, halted, trap,
                         u_soc.u_subsys.u_dmem.memory[W_OUT_MAGIC],
                         u_soc.u_subsys.u_dmem.memory[W_OUT_DMA_STATUS], m1_w_beats_total);
                errors = errors + 1;
            end
            report_subtest(err_before, "Teste 5: Configuracao invalida (M=33) rejeitada pelo DMA e tratada pelo firmware (OUT_MAGIC=ERR!)");

            // TESTE 6: Bases customizadas cruzando fronteiras de 4 KB (0x0FFC/0x1FFC/0x2FFC) via firmware
            err_before = errors;
            for (i = 0; i < 5; i = i + 1)
                for (j = 0; j < 5; j = j + 1) begin
                    mat_A[i][j] = (i * 5 + j) - 12;
                    mat_B[i][j] = (j * 5 + i) - 10;
                end
            compute_reference(5, 5, 5);
            prepare_and_run_soc(5, 5, 5, 32'h0FFC, 32'h1FFC, 32'h2FFC, 32'h3, 4, 1'b1, 50000);
            verify_normal_execution(5, 5, 5, 32'h2FFC, 4, 1'b1, "Teste 6 (Bases 4KB via firmware)");
            report_subtest(err_before, "Teste 6: Bases customizadas (0x0FFC/0x1FFC/0x2FFC) cruzando 4 KB via firmware RV32I");
        end
    endtask

    // -------------------------------------------------------------------------
    // Suíte de Contratos do Firmware (Área Reservada, DESC_CPU_WORK, DESC_CTRL)
    // -------------------------------------------------------------------------
    task automatic verify_fw_rejection(input logic [31:0] exp_cause, input string ctx);
        logic [31:0] got_magic, got_status, got_polls, got_work;
        begin
            got_magic  = u_soc.u_subsys.u_dmem.memory[W_OUT_MAGIC];
            got_status = u_soc.u_subsys.u_dmem.memory[W_OUT_DMA_STATUS];
            got_polls  = u_soc.u_subsys.u_dmem.memory[W_OUT_POLL_ITERS];
            got_work   = u_soc.u_subsys.u_dmem.memory[W_OUT_WORK_DONE];

            verify_m0_1to1(ctx);
            if (!halted || trap ||
                got_magic !== MAGIC_FWER || got_status !== exp_cause ||
                got_polls !== 32'd0 || got_work !== 32'd0 ||
                m1_ar_hs_cnt !== 0 || m1_aw_hs_cnt !== 0 || m1_w_beats_total !== 0 ||
                irq_seen !== 1'b0 || irq_done !== 1'b0 ||
                cpu_dmem_wr_outside_res_cnt !== 0) begin
                $display("[FAIL][SoC %0dx%0d L%0d] %s: halted=%b trap=%b magic=0x%08h st=0x%08h(esp 0x%08h) m1_ar=%0d m1_aw=%0d out_res=%0d",
                         ARRAY_SIZE, ARRAY_SIZE, DMEM_LATENCY, ctx, halted, trap,
                         got_magic, got_status, exp_cause, m1_ar_hs_cnt, m1_aw_hs_cnt,
                         cpu_dmem_wr_outside_res_cnt);
                errors = errors + 1;
            end
        end
    endtask

    task automatic run_fw_contract_suite();
        int i, j, err_before;
        begin
            // TESTE FW-1: Proteção da área reservada 0x0000..0x01FF (3 sobreposições + fronteira 0x0200)
            err_before = errors;

            // 1a) A = 0x0000 (sobreposição com descritor)
            prepare_and_run_soc(1, 1, 1, 32'h0000, 32'h1000, 32'h2000, 32'h3, 0, 1'b0, 10000);
            verify_fw_rejection(FW_ERR_RESERVED, "FW-1a (A=0x0000 descritor)");

            // 1b) C = 0x0020 (sobreposição com telemetria: 1x1, A=2, B=3 -> rejeitado antes do DMA)
            u_soc.u_subsys.u_dmem.memory[32'h1000 >> 2] = 32'd2;
            u_soc.u_subsys.u_dmem.memory[32'h1004 >> 2] = 32'd3;
            prepare_and_run_soc(1, 1, 1, 32'h1000, 32'h1004, 32'h0020, 32'h3, 0, 1'b0, 10000);
            verify_fw_rejection(FW_ERR_RESERVED, "FW-1b (C=0x0020 telemetria)");
            if (u_soc.u_subsys.u_dmem.memory[32'h1000 >> 2] !== 32'd2 ||
                u_soc.u_subsys.u_dmem.memory[32'h1004 >> 2] !== 32'd3) begin
                $display("[FAIL][SoC %0dx%0d L%0d] FW-1b: Entradas A/B alteradas!", ARRAY_SIZE, ARRAY_SIZE, DMEM_LATENCY);
                errors = errors + 1;
            end

            // 1c) B = 0x0100 (sobreposição com área de trabalho da CPU)
            u_soc.u_subsys.u_dmem.memory[32'h0100 >> 2] = 32'hCAFE_1234;
            prepare_and_run_soc(1, 1, 1, 32'h1000, 32'h0100, 32'h2000, 32'h3, 8, 1'b0, 10000);
            verify_fw_rejection(FW_ERR_RESERVED, "FW-1c (B=0x0100 area privada)");
            if (u_soc.u_subsys.u_dmem.memory[32'h0100 >> 2] !== 32'hCAFE_1234) begin
                $display("[FAIL][SoC %0dx%0d L%0d] FW-1c: Sentinela em 0x0100 alterada!", ARRAY_SIZE, ARRAY_SIZE, DMEM_LATENCY);
                errors = errors + 1;
            end

            // 1d) Fronteira válida exata: A = 0x0200, B = 0x0210, C = 0x0220 (1x1, 2*3=6)
            mat_A[0][0] = 2;
            mat_B[0][0] = 3;
            compute_reference(1, 1, 1);
            prepare_and_run_soc(1, 1, 1, 32'h0200, 32'h0210, 32'h0220, 32'h3, 0, 1'b1, 20000);
            verify_normal_execution(1, 1, 1, 32'h0220, 0, 1'b1, "FW-1d (fronteira valida 0x0200)");
            report_subtest(err_before, "Teste FW-1: Protecao da area reservada 0x0000..0x01FF (A=0x0000, C=0x0020, B=0x0100 -> FWER; 0x0200 -> DONE)");

            // TESTE FW-2: Limite de DESC_CPU_WORK (0, 64 e 65)
            err_before = errors;

            // 2a) DESC_CPU_WORK = 0
            for (i = 0; i < 4; i = i + 1)
                for (j = 0; j < 4; j = j + 1) begin
                    mat_A[i][j] = i + j + 1;
                    mat_B[i][j] = (i == j) ? 2 : -1;
                end
            compute_reference(4, 4, 4);
            u_soc.u_subsys.u_dmem.memory[32'h0100 >> 2] = 32'h1122_3344;
            u_soc.u_subsys.u_dmem.memory[32'h0200 >> 2] = 32'h5566_7788;
            prepare_and_run_soc(4, 4, 4, 32'h1000, 32'h2000, 32'h3000, 32'h3, 0, 1'b1, 30000);
            verify_normal_execution(4, 4, 4, 32'h3000, 0, 1'b1, "FW-2a (DESC_CPU_WORK=0)");
            if (u_soc.u_subsys.u_dmem.memory[32'h0100 >> 2] !== 32'h1122_3344 ||
                u_soc.u_subsys.u_dmem.memory[32'h0200 >> 2] !== 32'h5566_7788) begin
                $display("[FAIL][SoC %0dx%0d L%0d] FW-2a: Sentinelas 0x0100/0x0200 corrompidas!", ARRAY_SIZE, ARRAY_SIZE, DMEM_LATENCY);
                errors = errors + 1;
            end

            // 2b) DESC_CPU_WORK = 64 (matriz 16x16 para preencher todas as 64 palavras 0x0100..0x01FC sem tocar 0x0200)
            for (i = 0; i < 16; i = i + 1)
                for (j = 0; j < 16; j = j + 1) begin
                    mat_A[i][j] = ((i * 16 + j) % 15) - 7;
                    mat_B[i][j] = ((j * 16 + i) % 13) - 6;
                end
            compute_reference(16, 16, 16);
            u_soc.u_subsys.u_dmem.memory[32'h0200 >> 2] = 32'h99AA_BBCC;
            prepare_and_run_soc(16, 16, 16, 32'h1000, 32'h2000, 32'h3000, 32'h3, 64, 1'b1, 150000);
            verify_normal_execution(16, 16, 16, 32'h3000, 64, 1'b1, "FW-2b (DESC_CPU_WORK=64)");
            if (u_soc.u_subsys.u_dmem.memory[W_OUT_WORK_DONE] !== 32'd64 ||
                u_soc.u_subsys.u_dmem.memory[32'h01FC >> 2] !== 32'h0000_A53F ||
                u_soc.u_subsys.u_dmem.memory[32'h0200 >> 2] !== 32'h99AA_BBCC) begin
                $display("[FAIL][SoC %0dx%0d L%0d] FW-2b: work=%0d [0x01FC]=0x%08h [0x0200]=0x%08h",
                         ARRAY_SIZE, ARRAY_SIZE, DMEM_LATENCY,
                         u_soc.u_subsys.u_dmem.memory[W_OUT_WORK_DONE],
                         u_soc.u_subsys.u_dmem.memory[32'h01FC >> 2],
                         u_soc.u_subsys.u_dmem.memory[32'h0200 >> 2]);
                errors = errors + 1;
            end

            // 2c) DESC_CPU_WORK = 65 (rejeitado antes do DMA, sentinela 0x0200 intacta)
            u_soc.u_subsys.u_dmem.memory[32'h0200 >> 2] = 32'hDEAD_BEEF;
            prepare_and_run_soc(4, 4, 4, 32'h1000, 32'h2000, 32'h3000, 32'h3, 65, 1'b0, 10000);
            verify_fw_rejection(FW_ERR_WORK, "FW-2c (DESC_CPU_WORK=65)");
            if (u_soc.u_subsys.u_dmem.memory[32'h0200 >> 2] !== 32'hDEAD_BEEF) begin
                $display("[FAIL][SoC %0dx%0d L%0d] FW-2c: Sentinela em 0x0200 sobrescrita (0x%08h)!",
                         ARRAY_SIZE, ARRAY_SIZE, DMEM_LATENCY, u_soc.u_subsys.u_dmem.memory[32'h0200 >> 2]);
                errors = errors + 1;
            end
            report_subtest(err_before, "Teste FW-2: Limite de DESC_CPU_WORK (0 e 64 com 0x0200 intacto; 65 rejeitado com FWER/0x80000002)");

            // TESTE FW-3: Validação de DESC_CTRL (0->0x3, 1 sem IRQ, 2 e 4 rejeitados)
            err_before = errors;
            for (i = 0; i < 4; i = i + 1)
                for (j = 0; j < 4; j = j + 1) begin
                    mat_A[i][j] = (i * 4 + j) - 6;
                    mat_B[i][j] = (j * 4 - i) + 2;
                end
            compute_reference(4, 4, 4);

            // 3a) DESC_CTRL = 0 -> padrão 0x3 (START | IRQ_EN)
            prepare_and_run_soc(4, 4, 4, 32'h1000, 32'h2000, 32'h3000, 32'h0, 0, 1'b1, 30000);
            verify_normal_execution(4, 4, 4, 32'h3000, 0, 1'b1, "FW-3a (DESC_CTRL=0 -> padrao 0x3)");

            // 3b) DESC_CTRL = 1 -> START sem IRQ_EN (irq_seen deve ser 0)
            prepare_and_run_soc(4, 4, 4, 32'h1000, 32'h2000, 32'h3000, 32'h1, 0, 1'b1, 30000);
            verify_normal_execution(4, 4, 4, 32'h3000, 0, 1'b0, "FW-3b (DESC_CTRL=1 sem IRQ)");

            // 3c) DESC_CTRL = 2 -> apenas IRQ_EN sem START (rejeitado sem travar polling)
            prepare_and_run_soc(4, 4, 4, 32'h1000, 32'h2000, 32'h3000, 32'h2, 0, 1'b0, 10000);
            verify_fw_rejection(FW_ERR_CTRL, "FW-3c (DESC_CTRL=2 sem START)");

            // 3d) DESC_CTRL = 4 -> SOFT_RST (rejeitado antes do DMA)
            prepare_and_run_soc(4, 4, 4, 32'h1000, 32'h2000, 32'h3000, 32'h4, 0, 1'b0, 10000);
            verify_fw_rejection(FW_ERR_CTRL, "FW-3d (DESC_CTRL=4 SOFT_RST)");

            report_subtest(err_before, "Teste FW-3: Validacao de DESC_CTRL (0->0x3, 1 sem IRQ, 2 e 4 rejeitados com FWER/0x80000003)");
        end
    endtask

    // Bateria sob DMEM_LATENCY = 3
    task automatic run_lat3_test();
        int i, j, err_before;
        begin
            errors          = 0;
            subtests_passed = 0;
            subtests_total  = 0;
            err_before      = errors;

            for (i = 0; i < 7; i = i + 1)
                for (j = 0; j < 9; j = j + 1)
                    mat_A[i][j] = ((i * 9 + j) % 27) - 13;
            for (i = 0; i < 9; i = i + 1)
                for (j = 0; j < 5; j = j + 1)
                    mat_B[i][j] = ((i * 5 + j) % 25) - 12;
            compute_reference(7, 9, 5);

            prepare_and_run_soc(7, 9, 5, 32'h0FF8, 32'h1FF4, 32'h2FF0, 32'h3, 8, 1'b1, 100000);
            verify_normal_execution(7, 9, 5, 32'h2FF0, 8, 1'b1, "Teste L3-1 (7x9*9x5 LATENCY=3)");
            report_subtest(err_before, "Teste L3-1: SoC completo com DMEM_LATENCY=3, 7x9*9x5, quebra 4 KB e concorrencia CPU/DMA");
        end
    endtask

    // Teste negativo de erro AXI4 em leitura (RRESP=DECERR -> cpu_error=1 -> trap=1)
    task automatic run_decerr_trap_test();
        int err_before;
        begin
            errors          = 0;
            subtests_passed = 0;
            subtests_total  = 0;
            err_before      = errors;

            prepare_and_run_soc(4, 4, 4, 32'h1000, 32'h2000, 32'h3000, 32'h3, 0, 1'b0, 2000);
            verify_m0_1to1("Teste TRAP-1 (DECERR leitura)");
            if (!trap || halted || debug_pc !== 32'h0000_0004 || m0_r_decerr_cnt !== 1) begin
                $display("[FAIL][SoC %0dx%0d L%0d] Teste TRAP-1: trap=%b halted=%b pc=0x%08h r_decerr=%0d",
                         ARRAY_SIZE, ARRAY_SIZE, DMEM_LATENCY, trap, halted, debug_pc, m0_r_decerr_cnt);
                errors = errors + 1;
            end
            report_subtest(err_before, "Teste TRAP-1: Leitura ilegal da CPU (0x5000_0000) gera RRESP=DECERR -> cpu_error=1 -> trap=1");
        end
    endtask

    // Teste negativo de erro AXI4 em escrita (BRESP=DECERR -> cpu_error=1 -> trap=1)
    task automatic run_decerr_write_test();
        int err_before;
        begin
            errors          = 0;
            subtests_passed = 0;
            subtests_total  = 0;
            err_before      = errors;

            prepare_and_run_soc(4, 4, 4, 32'h1000, 32'h2000, 32'h3000, 32'h3, 0, 1'b0, 2000);
            verify_m0_1to1("Teste TRAP-2 (DECERR escrita)");
            if (!trap || halted || debug_pc !== 32'h0000_0004 ||
                m0_b_decerr_cnt !== 1 || cpu_wr_done_cnt !== 1 || cpu_rd_done_cnt !== 0) begin
                $display("[FAIL][SoC %0dx%0d L%0d] Teste TRAP-2: trap=%b halted=%b pc=0x%08h b_decerr=%0d wr=%0d rd=%0d",
                         ARRAY_SIZE, ARRAY_SIZE, DMEM_LATENCY, trap, halted, debug_pc,
                         m0_b_decerr_cnt, cpu_wr_done_cnt, cpu_rd_done_cnt);
                errors = errors + 1;
            end
            report_subtest(err_before, "Teste TRAP-2: Escrita ilegal da CPU (0x5000_0000) gera BRESP=DECERR -> cpu_error=1 -> trap=1");
        end
    endtask

    // Teste dirigido da ponte: SW consecutivos (1:1 AW/W/B), SB/SH/WSTRB e LB/LBU/LH/LHU/LW
    task automatic run_bridge_ops_test();
        int err_before;
        logic [31:0] v_0100, v_0104, v_0108;
        logic [31:0] v_lb0, v_lbu0, v_lb1, v_lh0, v_lhu2, v_lw0;
        begin
            errors          = 0;
            subtests_passed = 0;
            subtests_total  = 0;
            err_before      = errors;

            // Pre-preenche região de trabalho para garantir ausência de resíduos
            u_soc.u_subsys.u_dmem.memory[32'h0100 >> 2] = 32'h0000_0000;
            u_soc.u_subsys.u_dmem.memory[32'h0104 >> 2] = 32'h0000_0000;
            u_soc.u_subsys.u_dmem.memory[32'h0108 >> 2] = 32'h0000_0000;
            u_soc.u_subsys.u_dmem.memory[32'h0110 >> 2] = 32'hDEAD_0001;
            u_soc.u_subsys.u_dmem.memory[32'h0114 >> 2] = 32'hDEAD_0002;
            u_soc.u_subsys.u_dmem.memory[32'h0118 >> 2] = 32'hDEAD_0003;
            u_soc.u_subsys.u_dmem.memory[32'h011C >> 2] = 32'hDEAD_0004;
            u_soc.u_subsys.u_dmem.memory[32'h0120 >> 2] = 32'hDEAD_0005;
            u_soc.u_subsys.u_dmem.memory[32'h0124 >> 2] = 32'hDEAD_0006;

            prepare_and_run_soc(4, 4, 4, 32'h1000, 32'h2000, 32'h3000, 32'h3, 0, 1'b0, 4000);
            verify_m0_1to1("Teste BRIDGE-1");

            v_0100 = u_soc.u_subsys.u_dmem.memory[32'h0100 >> 2];
            v_0104 = u_soc.u_subsys.u_dmem.memory[32'h0104 >> 2];
            v_0108 = u_soc.u_subsys.u_dmem.memory[32'h0108 >> 2];
            v_lb0  = u_soc.u_subsys.u_dmem.memory[32'h0110 >> 2];
            v_lbu0 = u_soc.u_subsys.u_dmem.memory[32'h0114 >> 2];
            v_lb1  = u_soc.u_subsys.u_dmem.memory[32'h0118 >> 2];
            v_lh0  = u_soc.u_subsys.u_dmem.memory[32'h011C >> 2];
            v_lhu2 = u_soc.u_subsys.u_dmem.memory[32'h0120 >> 2];
            v_lw0  = u_soc.u_subsys.u_dmem.memory[32'h0124 >> 2];

            if (!halted || trap) begin
                $display("[FAIL][SoC %0dx%0d L%0d] BRIDGE-1: halted=%b trap=%b",
                         ARRAY_SIZE, ARRAY_SIZE, DMEM_LATENCY, halted, trap);
                errors = errors + 1;
            end
            // 15 escritas (3 consecutivas em 0x0100) e 6 leituras (LB/LBU/LB/LH/LHU/LW)
            if (cpu_wr_done_cnt !== 15 || cpu_rd_done_cnt !== 6 || m0_aw_0100_cnt !== 3 ||
                m0_wstrb_0010_cnt !== 1 || m0_wstrb_1000_cnt !== 1 ||
                m0_wstrb_0011_cnt !== 1 || m0_wstrb_1100_cnt !== 1 || m0_wstrb_1111_cnt !== 11 ||
                cpu_dmem_wr_outside_res_cnt !== 0) begin
                $display("[FAIL][SoC %0dx%0d L%0d] BRIDGE-1 contagens: wr=%0d rd=%0d aw0100=%0d strb=(%0d,%0d,%0d,%0d,%0d)",
                         ARRAY_SIZE, ARRAY_SIZE, DMEM_LATENCY, cpu_wr_done_cnt, cpu_rd_done_cnt, m0_aw_0100_cnt,
                         m0_wstrb_0010_cnt, m0_wstrb_1000_cnt, m0_wstrb_0011_cnt, m0_wstrb_1100_cnt, m0_wstrb_1111_cnt);
                errors = errors + 1;
            end
            if (v_0100 !== 32'h89AB_CDEF || v_0104 !== 32'hF055_1255 || v_0108 !== 32'hCAFE_80FE) begin
                $display("[FAIL][SoC %0dx%0d L%0d] BRIDGE-1 memoria: [0x0100]=0x%08h [0x0104]=0x%08h [0x0108]=0x%08h",
                         ARRAY_SIZE, ARRAY_SIZE, DMEM_LATENCY, v_0100, v_0104, v_0108);
                errors = errors + 1;
            end
            if (v_lb0  !== 32'hFFFF_FFFE || v_lbu0 !== 32'h0000_00FE || v_lb1  !== 32'hFFFF_FF80 ||
                v_lh0  !== 32'hFFFF_80FE || v_lhu2 !== 32'h0000_CAFE || v_lw0  !== 32'hCAFE_80FE) begin
                $display("[FAIL][SoC %0dx%0d L%0d] BRIDGE-1 loads: lb0=0x%08h lbu0=0x%08h lb1=0x%08h lh0=0x%08h lhu2=0x%08h lw0=0x%08h",
                         ARRAY_SIZE, ARRAY_SIZE, DMEM_LATENCY, v_lb0, v_lbu0, v_lb1, v_lh0, v_lhu2, v_lw0);
                errors = errors + 1;
            end
            report_subtest(err_before, "Teste BRIDGE-1: 3 SW consecutivos (1:1 AW/W/B), SB/SH com WSTRB/preservacao e LB/LBU/LH/LHU/LW");
        end
    endtask

    // Reset assíncrono no meio de transação pendente na ponte (LATENCY=3) + recuperação limpa
    task automatic run_reset_mid_tx_test();
        int i, j, cyc, err_before;
        bit saw_pending;
        begin
            err_before  = errors;
            saw_pending = 1'b0;

            for (i = 0; i < 4; i = i + 1)
                for (j = 0; j < 4; j = j + 1) begin
                    mat_A[i][j] = (i * 4 + j) - 7;
                    mat_B[i][j] = (j * 4 - i) + 3;
                end
            compute_reference(4, 4, 4);

            // Prepara DMEM e libera reset para entrar em uma transação pendente na ponte
            mon_active = 1'b0;
            rst_n      = 1'b0;
            repeat (3) @(posedge clk);

            u_soc.u_subsys.u_dmem.memory[W_DESC_ADDR_A]   = 32'h1000;
            u_soc.u_subsys.u_dmem.memory[W_DESC_ADDR_B]   = 32'h2000;
            u_soc.u_subsys.u_dmem.memory[W_DESC_ADDR_C]   = 32'h3000;
            u_soc.u_subsys.u_dmem.memory[W_DESC_DIM_M]    = 4;
            u_soc.u_subsys.u_dmem.memory[W_DESC_DIM_K]    = 4;
            u_soc.u_subsys.u_dmem.memory[W_DESC_DIM_N]    = 4;
            u_soc.u_subsys.u_dmem.memory[W_DESC_CTRL]     = 32'h3;
            u_soc.u_subsys.u_dmem.memory[W_DESC_CPU_WORK] = 4;
            for (i = W_OUT_MAGIC; i <= W_OUT_WORK_DONE; i = i + 1)
                u_soc.u_subsys.u_dmem.memory[i] = 32'd0;

            snap_a_word = 32'h1000 >> 2;
            snap_b_word = 32'h2000 >> 2;
            snap_a_cnt  = 16;
            snap_b_cnt  = 16;
            for (i = 0; i < 4; i = i + 1)
                for (j = 0; j < 4; j = j + 1) begin
                    u_soc.u_subsys.u_dmem.memory[snap_a_word + i*4 + j] = {24'd0, mat_A[i][j][7:0]};
                    snap_a[i*4 + j] = {24'd0, mat_A[i][j][7:0]};
                    u_soc.u_subsys.u_dmem.memory[snap_b_word + i*4 + j] = {24'd0, mat_B[i][j][7:0]};
                    snap_b[i*4 + j] = {24'd0, mat_B[i][j][7:0]};
                end
            for (i = 0; i < 16; i = i + 1)
                u_soc.u_subsys.u_dmem.memory[(32'h3000 >> 2) + i] = 32'hA5A5_A5A5;

            clear_trackers(32'h1000, 64, 32'h2000, 64, 32'h3000, 64);

            @(negedge clk);
            mon_active = 1'b1;
            rst_n      = 1'b1;

            // Aguarda transação pendente ativa dentro da ponte cpu_axi4_bridge (ST_WR_WAIT ou ST_RD_WAIT)
            cyc = 0;
            while (!saw_pending && cyc < 500) begin
                @(posedge clk);
                cyc = cyc + 1;
                if (u_soc.cpu_dmem_valid === 1'b1 && u_soc.cpu_dmem_ready === 1'b0 &&
                    (u_soc.u_cpu_bridge.state === 2'd1 || u_soc.u_cpu_bridge.state === 2'd2)) begin
                    saw_pending = 1'b1;
                end
            end

            if (!saw_pending) begin
                $display("[FAIL][SoC %0dx%0d L%0d] Teste RESET-1: nao observou transacao pendente na ponte!",
                         ARRAY_SIZE, ARRAY_SIZE, DMEM_LATENCY);
                errors = errors + 1;
            end

            // Injeta reset assíncrono durante a transação pendente e valida limpeza imediata da ponte
            @(negedge clk);
            mon_active = 1'b0;
            rst_n      = 1'b0;
            #1;
            if (u_soc.m0_axi_awvalid !== 1'b0 || u_soc.m0_axi_wvalid  !== 1'b0 ||
                u_soc.m0_axi_arvalid !== 1'b0 || u_soc.cpu_dmem_ready !== 1'b0 ||
                u_soc.cpu_dmem_error !== 1'b0 || u_soc.u_cpu_bridge.state !== 2'd0 ||
                halted !== 1'b0 || trap !== 1'b0) begin
                $display("[FAIL][SoC %0dx%0d L%0d] Teste RESET-1: ponte nao limpou saidas em rst_n=0 (aw=%b w=%b ar=%b rdy=%b err=%b st=%0d)",
                         ARRAY_SIZE, ARRAY_SIZE, DMEM_LATENCY, u_soc.m0_axi_awvalid, u_soc.m0_axi_wvalid,
                         u_soc.m0_axi_arvalid, u_soc.cpu_dmem_ready, u_soc.cpu_dmem_error, u_soc.u_cpu_bridge.state);
                errors = errors + 1;
            end

            repeat (2) @(posedge clk);
            for (i = W_OUT_MAGIC; i <= W_OUT_WORK_DONE; i = i + 1)
                u_soc.u_subsys.u_dmem.memory[i] = 32'd0;
            clear_trackers(32'h1000, 64, 32'h2000, 64, 32'h3000, 64);

            // Libera reset e valida execução completa limpa após o abortamento
            @(negedge clk);
            mon_active = 1'b1;
            rst_n      = 1'b1;

            cyc = 0;
            while (!halted && !trap && cyc < 40000) begin
                @(posedge clk);
                cyc = cyc + 1;
            end
            @(posedge clk);
            mon_active = 1'b0;

            if (cyc >= 40000) begin
                $display("[FAIL][SoC %0dx%0d L%0d] Teste RESET-1: timeout apos liberacao do reset!",
                         ARRAY_SIZE, ARRAY_SIZE, DMEM_LATENCY);
                errors = errors + 1;
            end
            verify_normal_execution(4, 4, 4, 32'h3000, 4, 1'b1, "Teste RESET-1 (pos-reset)");
            report_subtest(err_before, "Teste RESET-1: Reset assincrono durante transacao pendente na ponte limpa FSM e retoma execucao limpa");
        end
    endtask

    // Teste dirigido do caminho de timeout de polling (FW_ERR_TIMEOUT) + recuperação pós-reset
    task automatic run_fw_timeout_test();
        int i, j, err_before;
        logic [31:0] got_magic, got_status, got_polls;
        begin
            errors          = 0;
            subtests_passed = 0;
            subtests_total  = 0;
            err_before      = errors;

            // Fase 1: 8x8 em malha 4x4 (8 tiles) excede MAX_POLL_ITERS=12 em test_fw_timeout.hex
            for (i = 0; i < 8; i = i + 1)
                for (j = 0; j < 8; j = j + 1) begin
                    mat_A[i][j] = ((i * 8 + j) % 11) - 5;
                    mat_B[i][j] = ((j * 8 + i) % 9) - 4;
                end
            compute_reference(8, 8, 8);
            prepare_and_run_soc(8, 8, 8, 32'h1000, 32'h2000, 32'h3000, 32'h3, 0, 1'b1, 10000);

            got_magic  = u_soc.u_subsys.u_dmem.memory[W_OUT_MAGIC];
            got_status = u_soc.u_subsys.u_dmem.memory[W_OUT_DMA_STATUS];
            got_polls  = u_soc.u_subsys.u_dmem.memory[W_OUT_POLL_ITERS];

            verify_m0_1to1("Teste FW-4 (timeout)");
            if (!halted || trap ||
                got_magic !== MAGIC_FWER || got_status !== FW_ERR_TIMEOUT ||
                got_polls !== 32'd12 ||
                u_soc.u_subsys.u_dma.status_busy !== 1'b1 ||
                cpu_dmem_wr_outside_res_cnt !== 0) begin
                $display("[FAIL][SoC %0dx%0d L%0d] Teste FW-4 (timeout): halted=%b trap=%b magic=0x%08h st=0x%08h polls=%0d busy=%b",
                         ARRAY_SIZE, ARRAY_SIZE, DMEM_LATENCY, halted, trap,
                         got_magic, got_status, got_polls, u_soc.u_subsys.u_dma.status_busy);
                errors = errors + 1;
            end

            // Fase 2: Reset global (política documentada) limpa DMA ativo e permite nova execução limpa
            mon_active = 1'b0;
            rst_n      = 1'b0;
            repeat (3) @(posedge clk);
            if (u_soc.u_subsys.u_dma.status_busy !== 1'b0 ||
                u_soc.u_subsys.m1_axi_arvalid    !== 1'b0 ||
                u_soc.u_subsys.m1_axi_awvalid    !== 1'b0 ||
                u_soc.u_subsys.m1_axi_wvalid     !== 1'b0 ||
                u_soc.m0_axi_awvalid             !== 1'b0 ||
                u_soc.m0_axi_wvalid              !== 1'b0 ||
                u_soc.m0_axi_arvalid             !== 1'b0 ||
                halted !== 1'b0 || trap !== 1'b0 || irq_done !== 1'b0) begin
                $display("[FAIL][SoC %0dx%0d L%0d] Teste FW-4 (reset): sistema nao ficou ocioso apos rst_n=0 (busy=%b halted=%b irq=%b)",
                         ARRAY_SIZE, ARRAY_SIZE, DMEM_LATENCY, u_soc.u_subsys.u_dma.status_busy, halted, irq_done);
                errors = errors + 1;
            end

            // Reutiliza as mesmas regiões A/B/C (0x1000/0x2000/0x3000) em uma 1x1 (2*3=6) concluída antes do limite
            mat_A[0][0] = 2;
            mat_B[0][0] = 3;
            compute_reference(1, 1, 1);
            prepare_and_run_soc(1, 1, 1, 32'h1000, 32'h2000, 32'h3000, 32'h3, 0, 1'b1, 10000);
            verify_normal_execution(1, 1, 1, 32'h3000, 0, 1'b1, "Teste FW-4 (recuperacao pos-reset)");
            report_subtest(err_before, "Teste FW-4: Timeout de polling (FWER/0x80000004 com DMA_BUSY=1) e recuperacao limpa apos reset global");
        end
    endtask

endmodule


module tb_riscv_dma_soc;
    logic clk;

    initial begin
        clk = 1'b0;
        forever #5 clk = ~clk; // Clock de 100 MHz (período de 10 ns)
    end

    // Instâncias independentes do SoC completo (2x2, 4x4, 8x8, LATENCY=3, timeout e ponte)
    soc_test_harness #(.ARRAY_SIZE(2), .DMEM_LATENCY(1), .IMEM_FILE("sw/firmware/matmul_dma.hex"))        u_soc_2x2     (.clk(clk));
    soc_test_harness #(.ARRAY_SIZE(4), .DMEM_LATENCY(1), .IMEM_FILE("sw/firmware/matmul_dma.hex"))        u_soc_4x4     (.clk(clk));
    soc_test_harness #(.ARRAY_SIZE(8), .DMEM_LATENCY(1), .IMEM_FILE("sw/firmware/matmul_dma.hex"))        u_soc_8x8     (.clk(clk));
    soc_test_harness #(.ARRAY_SIZE(4), .DMEM_LATENCY(3), .IMEM_FILE("sw/firmware/matmul_dma.hex"))        u_soc_lat3    (.clk(clk));
    soc_test_harness #(.ARRAY_SIZE(4), .DMEM_LATENCY(1), .IMEM_FILE("sw/firmware/test_fw_timeout.hex"))   u_soc_timeout (.clk(clk));
    soc_test_harness #(.ARRAY_SIZE(4), .DMEM_LATENCY(1), .IMEM_FILE("sw/firmware/test_decerr_trap.hex"))  u_soc_trap    (.clk(clk));
    soc_test_harness #(.ARRAY_SIZE(4), .DMEM_LATENCY(1), .IMEM_FILE("sw/firmware/test_decerr_write.hex")) u_soc_trap_wr (.clk(clk));
    soc_test_harness #(.ARRAY_SIZE(4), .DMEM_LATENCY(1), .IMEM_FILE("sw/firmware/test_bridge_ops.hex"))   u_soc_bridge  (.clk(clk));

    int total_errors;
    int total_passed;
    int total_run;

    initial begin
        `ifdef DUMP_VCD
            $dumpfile("tb_riscv_dma_soc.vcd");
            $dumpvars(0, tb_riscv_dma_soc);
        `endif

        $display("=====================================================================");
        $display("  INICIO DA VERIFICACAO INTEGRADA DO SOC: RV32I + DMA + AXI4 + ACC   ");
        $display("=====================================================================");

        $display("\n--- Executando firmware RV32I no SoC ARRAY_SIZE = 2 (2x2) ---");
        u_soc_2x2.run_soc_suite();

        $display("\n--- Executando firmware RV32I no SoC ARRAY_SIZE = 4 (4x4) ---");
        u_soc_4x4.run_soc_suite();

        $display("\n--- Executando suite de contratos do firmware (4x4) ---");
        u_soc_4x4.run_fw_contract_suite();
        u_soc_timeout.run_fw_timeout_test();

        $display("\n--- Executando firmware RV32I no SoC ARRAY_SIZE = 8 (8x8) ---");
        u_soc_8x8.run_soc_suite();

        $display("\n--- Executando firmware RV32I no SoC DMEM_LATENCY = 3 (4x4) ---");
        u_soc_lat3.run_lat3_test();
        u_soc_lat3.run_reset_mid_tx_test();

        $display("\n--- Executando testes dirigidos da ponte cpu_axi4_bridge ---");
        u_soc_trap.run_decerr_trap_test();
        u_soc_trap_wr.run_decerr_write_test();
        u_soc_bridge.run_bridge_ops_test();

        total_errors = u_soc_2x2.errors  + u_soc_4x4.errors     + u_soc_8x8.errors +
                       u_soc_lat3.errors + u_soc_timeout.errors + u_soc_trap.errors +
                       u_soc_trap_wr.errors + u_soc_bridge.errors;
        total_passed = u_soc_2x2.subtests_passed     + u_soc_4x4.subtests_passed  +
                       u_soc_8x8.subtests_passed     + u_soc_lat3.subtests_passed +
                       u_soc_timeout.subtests_passed + u_soc_trap.subtests_passed +
                       u_soc_trap_wr.subtests_passed + u_soc_bridge.subtests_passed;
        total_run    = u_soc_2x2.subtests_total      + u_soc_4x4.subtests_total   +
                       u_soc_8x8.subtests_total      + u_soc_lat3.subtests_total  +
                       u_soc_timeout.subtests_total  + u_soc_trap.subtests_total  +
                       u_soc_trap_wr.subtests_total  + u_soc_bridge.subtests_total;

        $display("\n=====================================================================");
        $display("  RESUMO FINAL SOC (ENTREGA IV): %0d / %0d subtestes PASS | Erros: %0d",
                 total_passed, total_run, total_errors);
        $display("=====================================================================");

        if (total_errors != 0 || total_passed != total_run) begin
            $fatal(1, "FALHA NA VERIFICACAO INTEGRADA DO SOC!");
        end

        $display("ALL TESTS PASSED!");
        $finish;
    end

endmodule
