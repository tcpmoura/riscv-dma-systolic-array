// =============================================================================
// Projeto     : RISC-V RV32I com Controlador DMA e Coprocessador Matricial
// Etapa       : Entrega Parcial III — DMA, Barramento AXI4 e Memória Compartilhada
// Módulo      : dma_controller.v
// Professores : Ana Julia Fernandes de Oliveira Barros
//               Igor Dantas dos Santos Miranda
//               João Alberto Castelo Branco Oliveira
// Grupo 4     : João Vitor Carvalho Lima
//               Matheus Artur Macedo Bomfim
//               Nathalia Miranda
//               Tatiana Caetano Praxedes de Moura
// Descrição   : Controlador de DMA Autônomo que gerencia toda a operação
//               C = A x B sem intervenção da CPU após o disparo de START.
//               - Escravo AXI4 MMIO (0x4000_0000..0x4000_002C): configuração,
//                 status R/W1C, proteção de escrita em BUSY=1 e contadores PERF.
//               - Mestre AXI4 (32b, INCR): lê A e B e grava C na DMEM com
//                 quebra automática em fronteiras de 4 KB.
//               - Controle de Tiling (ti, tj, tk), zero-padding local nos
//                 buffers do acelerador, interrupção irq_done e abortamento
//                 seguro por SOFT_RST ou erro AXI4 (RRESP/BRESP != OKAY).
//
// Mapa de Registradores MMIO (Offsets relativos a 0x4000_0000):
//   0x00 DMA_CTRL          (R/W)   [0]=START(W1S), [1]=IRQ_EN, [2]=SOFT_RST(W1S)
//   0x04 DMA_STATUS        (R/W1C) [0]=BUSY(RO), [1]=DONE(R/W1C), [2]=ERR(R/W1C)
//   0x08 DMA_ADDR_A        (R/W)   Base da matriz A na DMEM (Reset: 0x0000_1000)
//   0x0C DMA_ADDR_B        (R/W)   Base da matriz B na DMEM (Reset: 0x0000_2000)
//   0x10 DMA_ADDR_C        (R/W)   Base da matriz C na DMEM (Reset: 0x0000_3000)
//   0x14 DMA_DIM_M         (R/W)   Linhas de A e C (1..32, Reset: 4)
//   0x18 DMA_DIM_K         (R/W)   Colunas de A / Linhas de B (1..32, Reset: 4)
//   0x1C DMA_DIM_N         (R/W)   Colunas de B e C (1..32, Reset: 4)
//   0x20 PERF_TOTAL_CYCLES (RO)    Ciclos totais ativos entre START e DONE
//   0x24 PERF_COMP_CYCLES  (RO)    Ciclos acumulados com acc_busy = 1
//   0x28 PERF_MEM_READS    (RO)    Total de palavras lidas da DMEM pelo DMA
//   0x2C HW_ARRAY_SIZE     (RO)    Dimensão física T da malha (2, 4 ou 8)
// =============================================================================
`timescale 1ns / 1ps

module dma_controller #(
    parameter integer ARRAY_SIZE = 4,                  // Dimensão T da malha sistólica (2, 4 ou 8)
    parameter integer IDX_W      = $clog2(ARRAY_SIZE)  // Bits de endereçamento local dos buffers (0..T-1)
) (
    input  wire        clk,                            // Clock principal do sistema
    input  wire        rst_n,                          // Reset assíncrono ativo em nível baixo

    // Sinal de interrupção para a CPU (ativo quando IRQ_EN=1 e DONE=1)
    output wire        irq_done,

    // =========================================================================
    // Interface Escravo AXI4 (Configuração MMIO pela CPU / Testbench)
    // =========================================================================
    input  wire [31:0] s_axi_awaddr,
    input  wire [7:0]  s_axi_awlen,
    input  wire [2:0]  s_axi_awsize,
    input  wire [1:0]  s_axi_awburst,
    input  wire        s_axi_awvalid,
    output wire        s_axi_awready,

    input  wire [31:0] s_axi_wdata,
    input  wire [3:0]  s_axi_wstrb,
    input  wire        s_axi_wlast,
    input  wire        s_axi_wvalid,
    output wire        s_axi_wready,

    output reg  [1:0]  s_axi_bresp,
    output reg         s_axi_bvalid,
    input  wire        s_axi_bready,

    input  wire [31:0] s_axi_araddr,
    input  wire [7:0]  s_axi_arlen,
    input  wire [2:0]  s_axi_arsize,
    input  wire [1:0]  s_axi_arburst,
    input  wire        s_axi_arvalid,
    output wire        s_axi_arready,

    output reg  [31:0] s_axi_rdata,
    output reg  [1:0]  s_axi_rresp,
    output reg         s_axi_rlast,
    output reg         s_axi_rvalid,
    input  wire        s_axi_rready,

    // =========================================================================
    // Interface Mestre AXI4 (Acesso à DMEM Compartilhada)
    // =========================================================================
    output reg  [31:0] m_axi_awaddr,
    output reg  [7:0]  m_axi_awlen,
    output wire [2:0]  m_axi_awsize,
    output wire [1:0]  m_axi_awburst,
    output reg         m_axi_awvalid,
    input  wire        m_axi_awready,

    output wire [31:0] m_axi_wdata,
    output wire [3:0]  m_axi_wstrb,
    output wire        m_axi_wlast,
    output reg         m_axi_wvalid,
    input  wire        m_axi_wready,

    input  wire [1:0]  m_axi_bresp,
    input  wire        m_axi_bvalid,
    output reg         m_axi_bready,

    output reg  [31:0] m_axi_araddr,
    output reg  [7:0]  m_axi_arlen,
    output wire [2:0]  m_axi_arsize,
    output wire [1:0]  m_axi_arburst,
    output reg         m_axi_arvalid,
    input  wire        m_axi_arready,

    input  wire [31:0] m_axi_rdata,
    input  wire [1:0]  m_axi_rresp,
    input  wire        m_axi_rlast,
    input  wire        m_axi_rvalid,
    output reg         m_axi_rready,

    // =========================================================================
    // Interface DMA <-> Acelerador Sistólico (15 Sinais Oficiais)
    // =========================================================================
    output reg                   acc_clear,
    output reg                   acc_start,
    input  wire                  acc_busy,
    input  wire                  acc_done,

    output reg                   buf_a_we,
    output reg  [IDX_W-1:0]      buf_a_row,
    output reg  [IDX_W-1:0]      buf_a_col,
    output reg  signed [7:0]     buf_a_wdata,

    output reg                   buf_b_we,
    output reg  [IDX_W-1:0]      buf_b_row,
    output reg  [IDX_W-1:0]      buf_b_col,
    output reg  signed [7:0]     buf_b_wdata,

    output reg  [IDX_W-1:0]      buf_c_row,
    output reg  [IDX_W-1:0]      buf_c_col,
    input  wire signed [31:0]    buf_c_rdata
);

    // Códigos de resposta AXI4 usados no MMIO e verificados no mestre DMA
    localparam [1:0] AXI_RESP_OKAY   = 2'b00; // Transação bem-sucedida
    localparam [1:0] AXI_RESP_SLVERR = 2'b10; // Erro de acesso no escravo

    // Dimensão T da malha em larguras compatíveis com os comparadores internos
    localparam [5:0]  T_U6  = ARRAY_SIZE; // Valor T em 6 bits (usado nos índices 0..32)
    localparam [31:0] T_U32 = ARRAY_SIZE; // Valor T em 32 bits (exposto em HW_ARRAY_SIZE)

    // Constantes fixas do Mestre AXI4 do DMA (palavras de 32 bits em rajada INCR)
    assign m_axi_awsize  = 3'b010;  // 4 bytes por beat (32 bits) na escrita
    assign m_axi_awburst = 2'b01;   // Tipo INCR (endereços incrementais) na escrita
    assign m_axi_arsize  = 3'b010;  // 4 bytes por beat (32 bits) na leitura
    assign m_axi_arburst = 2'b01;   // Tipo INCR (endereços incrementais) na leitura
    assign m_axi_wstrb   = 4'b1111; // Escrita de palavra completa de 32 bits em C

    // -------------------------------------------------------------------------
    // Registradores MMIO de Configuração, Status e Desempenho
    // -------------------------------------------------------------------------
    reg        reg_irq_en;        // Habilita geração de interrupção irq_done (DMA_CTRL[1])
    reg        status_done;       // Flag de conclusão normal da matriz C (DMA_STATUS[1], W1C)
    reg        status_err;        // Flag de erro de config ou de resposta AXI4 (DMA_STATUS[2], W1C)

    reg [31:0] reg_addr_a;        // Endereço base da matriz A na DMEM (padrão 0x1000)
    reg [31:0] reg_addr_b;        // Endereço base da matriz B na DMEM (padrão 0x2000)
    reg [31:0] reg_addr_c;        // Endereço base da matriz C na DMEM (padrão 0x3000)
    reg [31:0] reg_dim_m;         // Dimensão M (número de linhas de A e C, faixa 1..32)
    reg [31:0] reg_dim_k;         // Dimensão K (colunas de A / linhas de B, faixa 1..32)
    reg [31:0] reg_dim_n;         // Dimensão N (número de colunas de B e C, faixa 1..32)

    reg [31:0] perf_total_cycles; // Contador de ciclos totais ativos (START até DONE)
    reg [31:0] perf_comp_cycles;  // Contador de ciclos em que o acelerador esteve ativo (acc_busy=1)
    reg [31:0] perf_mem_reads;    // Contador de palavras efetivamente lidas da DMEM (sem padding)

    // Pulsos internos de 1 ciclo gerados por escritas da CPU nos registradores MMIO
    reg        mmio_start_pulse;    // Dispara início da operação (DMA_CTRL[0])
    reg        mmio_soft_rst_pulse; // Requisita abortamento seguro (DMA_CTRL[2])
    reg        mmio_w1c_done_pulse; // Limpa status_done por escrita de 1 (DMA_STATUS[1])
    reg        mmio_w1c_err_pulse;  // Limpa status_err por escrita de 1 (DMA_STATUS[2])

    // -------------------------------------------------------------------------
    // Estados da FSM Principal do Controlador DMA
    // -------------------------------------------------------------------------
    localparam [3:0] ST_IDLE          = 4'd0;  // Ocioso aguardando START (BUSY=0)
    localparam [3:0] ST_ACC_CLEAR     = 4'd1;  // Zera acumuladores do acelerador para novo bloco C[ti][tj]
    localparam [3:0] ST_TILE_INIT     = 4'd2;  // Inicia carregamento do sub-bloco tk (começa por A)
    localparam [3:0] ST_LOAD_ROW_PREP = 4'd3;  // Calcula endereço e colunas válidas de uma linha de A ou B
    localparam [3:0] ST_RD_SETUP      = 4'd4;  // Ajusta comprimento da rajada AR respeitando fronteira de 4 KB
    localparam [3:0] ST_RD_BURST      = 4'd5;  // Recebe beats no canal R e grava bits [7:0] em buf_a/buf_b
    localparam [3:0] ST_PAD_ZERO      = 4'd6;  // Preenche posições excedentes (borda) com 0 sem ler DMEM
    localparam [3:0] ST_NEXT_LOAD_ROW = 4'd7;  // Avança linha do tile ou alterna de Buffer A para Buffer B
    localparam [3:0] ST_ACC_START     = 4'd8;  // Emite pulso acc_start para disparar a onda sistólica
    localparam [3:0] ST_WAIT_ACC      = 4'd9;  // Aguarda acc_done (3T-2 ciclos) e decide próximo tk ou escrita
    localparam [3:0] ST_WR_ROW_PREP   = 4'd10; // Calcula endereço row-major de uma linha válida de C
    localparam [3:0] ST_WR_SETUP      = 4'd11; // Configura rajada AW/W respeitando fronteira de 4 KB
    localparam [3:0] ST_WR_BURST      = 4'd12; // Envia palavras de buf_c_rdata no canal W até WLAST
    localparam [3:0] ST_WR_RESP       = 4'd13; // Aguarda handshake BVALID&BREADY e avança para próximo bloco
    localparam [3:0] ST_ABORT_CLEAN   = 4'd14; // Limpa acelerador e encerra após SOFT_RST ou erro AXI4

    reg [3:0] dma_state;                   // Estado atual da FSM principal do DMA
    wire      status_busy = (dma_state != ST_IDLE); // Indica operação em andamento (DMA_STATUS[0])

    assign irq_done = reg_irq_en & status_done; // Interrupção combinacional habilitada por IRQ_EN

    // -------------------------------------------------------------------------
    // Validador Combinacional de Configuração (executado no disparo de START)
    // Garante dimensões 1..32, alinhamento em 4 bytes, permanência dentro dos
    // 16 KB da DMEM e ausência de sobreposição entre as matrizes A, B e C.
    // -------------------------------------------------------------------------
    wire dim_m_ok = (reg_dim_m >= 32'd1) && (reg_dim_m <= 32'd32); // M dentro de 1..32
    wire dim_k_ok = (reg_dim_k >= 32'd1) && (reg_dim_k <= 32'd32); // K dentro de 1..32
    wire dim_n_ok = (reg_dim_n >= 32'd1) && (reg_dim_n <= 32'd32); // N dentro de 1..32
    wire dims_ok  = dim_m_ok && dim_k_ok && dim_n_ok;              // Todas as dimensões válidas

    wire align_ok = (reg_addr_a[1:0] == 2'b00) &&
                    (reg_addr_b[1:0] == 2'b00) &&
                    (reg_addr_c[1:0] == 2'b00);                    // Três bases alinhadas em 4 bytes

    // Tamanhos em bytes de cada matriz (máximo 32 x 32 x 4 = 4096 bytes)
    wire [13:0] bytes_a = dims_ok ? ({8'd0, reg_dim_m[5:0]} * {8'd0, reg_dim_k[5:0]} * 14'd4) : 14'd0; // Tamanho de A em bytes
    wire [13:0] bytes_b = dims_ok ? ({8'd0, reg_dim_k[5:0]} * {8'd0, reg_dim_n[5:0]} * 14'd4) : 14'd0; // Tamanho de B em bytes
    wire [13:0] bytes_c = dims_ok ? ({8'd0, reg_dim_m[5:0]} * {8'd0, reg_dim_n[5:0]} * 14'd4) : 14'd0; // Tamanho de C em bytes

    // Endereços finais exclusivos [base, end) em 33 bits para evitar overflow
    wire [32:0] end_a = {1'b0, reg_addr_a} + {19'd0, bytes_a}; // Fim da região de A
    wire [32:0] end_b = {1'b0, reg_addr_b} + {19'd0, bytes_b}; // Fim da região de B
    wire [32:0] end_c = {1'b0, reg_addr_c} + {19'd0, bytes_c}; // Fim da região de C

    wire range_ok = (reg_addr_a[31:14] == 18'd0) && (end_a <= 33'h0_0000_4000) &&
                    (reg_addr_b[31:14] == 18'd0) && (end_b <= 33'h0_0000_4000) &&
                    (reg_addr_c[31:14] == 18'd0) && (end_c <= 33'h0_0000_4000); // Todas dentro de 16 KB

    wire ov_ab = ({1'b0, reg_addr_a} < end_b) && ({1'b0, reg_addr_b} < end_a); // Sobreposição A vs B
    wire ov_ac = ({1'b0, reg_addr_a} < end_c) && ({1'b0, reg_addr_c} < end_a); // Sobreposição A vs C
    wire ov_bc = ({1'b0, reg_addr_b} < end_c) && ({1'b0, reg_addr_c} < end_b); // Sobreposição B vs C
    wire no_overlap = !(ov_ab || ov_ac || ov_bc);                               // Sem sobreposição entre matrizes

    wire cfg_valid = dims_ok && align_ok && range_ok && no_overlap; // Resultado consolidado da validação

    // -------------------------------------------------------------------------
    // Interface Escrava AXI4 (MMIO): FSM de Escrita (AW -> W -> B)
    // -------------------------------------------------------------------------
    localparam [1:0] MMIO_WR_IDLE = 2'd0; // Aguarda comando AW no escravo MMIO
    localparam [1:0] MMIO_WR_DATA = 2'd1; // Recebe palavra W e atualiza registrador MMIO
    localparam [1:0] MMIO_WR_RESP = 2'd2; // Mantém BVALID ativo até BREADY

    reg [1:0]  mmio_wr_state; // Estado atual da FSM de escrita MMIO
    reg [5:0]  mmio_wr_addr;  // Offset de 6 bits (0x00..0x2C) registrado em AW
    reg        mmio_wr_err;   // Indica erro de protocolo na escrita MMIO (retorna SLVERR)

    wire mmio_aw_bad = (s_axi_awaddr[1:0] != 2'b00) ||
                       (s_axi_awsize != 3'b010)     ||
                       (s_axi_awlen != 8'd0)        ||
                       (s_axi_awaddr[5:0] > 6'h2C); // Checa acesso alinhado de palavra única em 0x00..0x2C

    assign s_axi_awready = (mmio_wr_state == MMIO_WR_IDLE); // Pronto para receber AW
    assign s_axi_wready  = (mmio_wr_state == MMIO_WR_DATA); // Pronto para receber W

    // Função auxiliar que atualiza seletivamente apenas os bytes habilitados por wstrb
    function [31:0] apply_wstrb;
        input [31:0] old_val; // Valor atual armazenado no registrador
        input [31:0] new_val; // Novo valor vindo do barramento WDATA
        input [3:0]  strb;    // Máscara de habilitação de escrita por byte
        begin
            apply_wstrb[7:0]   = strb[0] ? new_val[7:0]   : old_val[7:0];
            apply_wstrb[15:8]  = strb[1] ? new_val[15:8]  : old_val[15:8];
            apply_wstrb[23:16] = strb[2] ? new_val[23:16] : old_val[23:16];
            apply_wstrb[31:24] = strb[3] ? new_val[31:24] : old_val[31:24];
        end
    endfunction

    // Bloco sequencial de escrita MMIO: decodifica acessos da CPU aos registradores,
    // ignora alterações de configuração durante BUSY=1 e gera pulsos de controle/W1C.
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            mmio_wr_state       <= MMIO_WR_IDLE;
            mmio_wr_addr        <= 6'd0;
            mmio_wr_err         <= 1'b0;
            s_axi_bvalid        <= 1'b0;
            s_axi_bresp         <= AXI_RESP_OKAY;
            reg_irq_en          <= 1'b0;
            reg_addr_a          <= 32'h0000_1000;
            reg_addr_b          <= 32'h0000_2000;
            reg_addr_c          <= 32'h0000_3000;
            reg_dim_m           <= 32'd4;
            reg_dim_k           <= 32'd4;
            reg_dim_n           <= 32'd4;
            mmio_start_pulse    <= 1'b0;
            mmio_soft_rst_pulse <= 1'b0;
            mmio_w1c_done_pulse <= 1'b0;
            mmio_w1c_err_pulse  <= 1'b0;
        end else begin
            // Pulsos de evento único duram apenas 1 ciclo
            mmio_start_pulse    <= 1'b0;
            mmio_soft_rst_pulse <= 1'b0;
            mmio_w1c_done_pulse <= 1'b0;
            mmio_w1c_err_pulse  <= 1'b0;

            case (mmio_wr_state)
                MMIO_WR_IDLE: begin
                    s_axi_bvalid <= 1'b0;
                    if (s_axi_awvalid) begin
                        mmio_wr_addr  <= s_axi_awaddr[5:0];
                        mmio_wr_err   <= mmio_aw_bad;
                        mmio_wr_state <= MMIO_WR_DATA;
                    end
                end

                MMIO_WR_DATA: begin
                    if (s_axi_wvalid) begin
                        if (!mmio_wr_err && s_axi_wlast) begin
                            case (mmio_wr_addr)
                                6'h00: begin // DMA_CTRL
                                    if (s_axi_wstrb[0]) begin
                                        reg_irq_en <= s_axi_wdata[1];
                                        if (s_axi_wdata[2]) begin
                                            mmio_soft_rst_pulse <= 1'b1;
                                        end else if (s_axi_wdata[0] && !status_busy) begin
                                            mmio_start_pulse <= 1'b1;
                                        end
                                    end
                                end
                                6'h04: begin // DMA_STATUS (W1C em DONE e ERR)
                                    if (s_axi_wstrb[0]) begin
                                        if (s_axi_wdata[1]) mmio_w1c_done_pulse <= 1'b1;
                                        if (s_axi_wdata[2]) mmio_w1c_err_pulse  <= 1'b1;
                                    end
                                end
                                6'h08: if (!status_busy) reg_addr_a <= apply_wstrb(reg_addr_a, s_axi_wdata, s_axi_wstrb);
                                6'h0C: if (!status_busy) reg_addr_b <= apply_wstrb(reg_addr_b, s_axi_wdata, s_axi_wstrb);
                                6'h10: if (!status_busy) reg_addr_c <= apply_wstrb(reg_addr_c, s_axi_wdata, s_axi_wstrb);
                                6'h14: if (!status_busy) reg_dim_m  <= apply_wstrb(reg_dim_m,  s_axi_wdata, s_axi_wstrb);
                                6'h18: if (!status_busy) reg_dim_k  <= apply_wstrb(reg_dim_k,  s_axi_wdata, s_axi_wstrb);
                                6'h1C: if (!status_busy) reg_dim_n  <= apply_wstrb(reg_dim_n,  s_axi_wdata, s_axi_wstrb);
                                default: ; // 0x20..0x2C são RO
                            endcase
                        end

                        if (s_axi_wlast) begin
                            s_axi_bvalid  <= 1'b1;
                            s_axi_bresp   <= mmio_wr_err ? AXI_RESP_SLVERR : AXI_RESP_OKAY;
                            mmio_wr_state <= MMIO_WR_RESP;
                        end else begin
                            mmio_wr_err   <= 1'b1;
                        end
                    end
                end

                MMIO_WR_RESP: begin
                    if (s_axi_bvalid && s_axi_bready) begin
                        s_axi_bvalid  <= 1'b0;
                        mmio_wr_state <= MMIO_WR_IDLE;
                    end
                end

                default: mmio_wr_state <= MMIO_WR_IDLE;
            endcase
        end
    end

    // -------------------------------------------------------------------------
    // Interface Escrava AXI4 (MMIO): FSM de Leitura (AR -> R)
    // -------------------------------------------------------------------------
    localparam MMIO_RD_IDLE = 1'b0; // Aguarda comando AR no escravo MMIO
    localparam MMIO_RD_DATA = 1'b1; // Mantém RVALID/RDATA/RLAST até RREADY

    reg        mmio_rd_state;       // Estado atual da FSM de leitura MMIO
    wire       mmio_ar_bad = (s_axi_araddr[1:0] != 2'b00) ||
                             (s_axi_arsize != 3'b010)     ||
                             (s_axi_arlen != 8'd0)        ||
                             (s_axi_araddr[5:0] > 6'h2C); // Checa acesso alinhado de palavra única em 0x00..0x2C

    assign s_axi_arready = (mmio_rd_state == MMIO_RD_IDLE); // Aceita AR quando ocioso

    // Bloco sequencial de leitura MMIO: devolve os registradores de controle,
    // status, configuração, contadores de desempenho e tamanho físico da malha.
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            mmio_rd_state <= MMIO_RD_IDLE;
            s_axi_rvalid  <= 1'b0;
            s_axi_rdata   <= 32'd0;
            s_axi_rresp   <= AXI_RESP_OKAY;
            s_axi_rlast   <= 1'b0;
        end else begin
            case (mmio_rd_state)
                MMIO_RD_IDLE: begin
                    s_axi_rvalid <= 1'b0;
                    s_axi_rlast  <= 1'b0;
                    if (s_axi_arvalid) begin
                        s_axi_rvalid <= 1'b1;
                        s_axi_rlast  <= 1'b1;
                        if (mmio_ar_bad) begin
                            s_axi_rdata <= 32'd0;
                            s_axi_rresp <= AXI_RESP_SLVERR;
                        end else begin
                            s_axi_rresp <= AXI_RESP_OKAY;
                            case (s_axi_araddr[5:0])
                                6'h00: s_axi_rdata <= {29'd0, 1'b0, reg_irq_en, 1'b0};
                                6'h04: s_axi_rdata <= {29'd0, status_err, status_done, status_busy};
                                6'h08: s_axi_rdata <= reg_addr_a;
                                6'h0C: s_axi_rdata <= reg_addr_b;
                                6'h10: s_axi_rdata <= reg_addr_c;
                                6'h14: s_axi_rdata <= reg_dim_m;
                                6'h18: s_axi_rdata <= reg_dim_k;
                                6'h1C: s_axi_rdata <= reg_dim_n;
                                6'h20: s_axi_rdata <= perf_total_cycles;
                                6'h24: s_axi_rdata <= perf_comp_cycles;
                                6'h28: s_axi_rdata <= perf_mem_reads;
                                6'h2C: s_axi_rdata <= T_U32;
                                default: s_axi_rdata <= 32'd0;
                            endcase
                        end
                        mmio_rd_state <= MMIO_RD_DATA;
                    end
                end

                MMIO_RD_DATA: begin
                    if (s_axi_rvalid && s_axi_rready) begin
                        s_axi_rvalid  <= 1'b0;
                        s_axi_rlast   <= 1'b0;
                        mmio_rd_state <= MMIO_RD_IDLE;
                    end
                end

                default: mmio_rd_state <= MMIO_RD_IDLE;
            endcase
        end
    end

    // -------------------------------------------------------------------------
    // Registradores Internos da FSM de Tiling e Transferência AXI4
    // -------------------------------------------------------------------------
    reg [5:0]  row_base;          // Linha base global do bloco atual de C (ti * T)
    reg [5:0]  col_base;          // Coluna base global do bloco atual de C (tj * T)
    reg [5:0]  k_base;            // Índice base global ao longo da dimensão K (tk * T)
    reg        buf_sel;           // Seleção do buffer alvo: 0 = Buffer A, 1 = Buffer B
    reg [5:0]  loc_r;             // Linha local dentro do bloco T x T (0..T-1)
    reg [5:0]  loc_c;             // Coluna local dentro do bloco T x T (0..T-1)

    reg [5:0]  burst_rem_cols;    // Colunas válidas restantes a transferir na linha atual
    reg [31:0] burst_cur_addr;    // Endereço corrente em bytes na DMEM para a próxima rajada
    reg [4:0]  burst_beats_total; // Total de beats da rajada AXI4 em voo (1..T)
    reg [4:0]  burst_beat_cnt;    // Contador de beats já transferidos na rajada em voo

    reg        soft_rst_pending;  // Memoriza SOFT_RST até a rajada/onda atual encerrar de forma limpa
    reg        axi_err_seen;      // Memoriza resposta AXI4 != OKAY recebida em RRESP ou BRESP

    // Conexão combinacional direta dos acumuladores de C ao canal W do AXI4
    assign m_axi_wdata = buf_c_rdata;                                    // Dado int32 lido diretamente do acelerador
    assign m_axi_wlast = (burst_beat_cnt == (burst_beats_total - 5'd1)); // Sinaliza o último beat da rajada W

    // Auxiliares combinacionais de dimensões restantes e proteção de fronteira 4 KB
    wire [5:0] cfg_m = reg_dim_m[5:0]; // Dimensão M em 6 bits (1..32)
    wire [5:0] cfg_k = reg_dim_k[5:0]; // Dimensão K em 6 bits (1..32)
    wire [5:0] cfg_n = reg_dim_n[5:0]; // Dimensão N em 6 bits (1..32)

    wire [5:0] rem_m = cfg_m - row_base; // Linhas restantes de A/C a partir de row_base
    wire [5:0] rem_k = cfg_k - k_base;   // Elementos restantes em K a partir de k_base
    wire [5:0] rem_n = cfg_n - col_base; // Colunas restantes de B/C a partir de col_base

    wire [5:0] valid_c_rows = (rem_m >= T_U6) ? T_U6 : rem_m; // Linhas válidas de C no bloco atual (<= T)
    wire [5:0] valid_c_cols = (rem_n >= T_U6) ? T_U6 : rem_n; // Colunas válidas de C no bloco atual (<= T)

    // Limita o tamanho da próxima rajada INCR para jamais cruzar fronteira AXI4 de 4 KB (1024 palavras)
    wire [10:0] words_to_4k      = 11'd1024 - {1'b0, burst_cur_addr[11:2]}; // Palavras até o fim da página 4 KB
    wire [10:0] rem_cols_ext     = {5'd0, burst_rem_cols};                  // Colunas restantes em 11 bits
    wire [4:0]  next_burst_words = (rem_cols_ext <= words_to_4k) ? burst_rem_cols[4:0] : words_to_4k[4:0]; // Beats da rajada

    // Variáveis combinacionais auxiliares para cálculo de endereço row-major de cada linha do tile
    reg [5:0]  prep_global_r;   // Índice de linha global (0..31) da matriz A, B ou C
    reg [5:0]  prep_valid_cols; // Quantidade de colunas reais (sem padding) na linha atual
    reg [31:0] prep_row_offset; // Deslocamento em palavras (row * largura + col_base)

    // Bloco combinacional de preparação de endereço: calcula índice global e deslocamento
    // row-major para leitura de A/B (ST_LOAD_ROW_PREP) ou escrita de C (ST_WR_ROW_PREP).
    always @(*) begin
        prep_global_r   = 6'd0;
        prep_valid_cols = 6'd0;
        prep_row_offset = 32'd0;
        if (dma_state == ST_LOAD_ROW_PREP) begin
            if (buf_sel == 1'b0) begin
                prep_global_r   = row_base + loc_r;
                prep_valid_cols = (rem_k >= T_U6) ? T_U6 : rem_k;
                prep_row_offset = ({26'd0, prep_global_r} * {26'd0, cfg_k}) + {26'd0, k_base};
            end else begin
                prep_global_r   = k_base + loc_r;
                prep_valid_cols = (rem_n >= T_U6) ? T_U6 : rem_n;
                prep_row_offset = ({26'd0, prep_global_r} * {26'd0, cfg_n}) + {26'd0, col_base};
            end
        end else if (dma_state == ST_WR_ROW_PREP) begin
            prep_global_r   = row_base + loc_r;
            prep_valid_cols = valid_c_cols;
            prep_row_offset = ({26'd0, prep_global_r} * {26'd0, cfg_n}) + {26'd0, col_base};
        end
    end

    // -------------------------------------------------------------------------
    // FSM Principal do Controlador DMA
    // Coordena autonomamente: validação inicial -> limpeza dos acumuladores ->
    // leitura de A e B com zero-padding -> execução da malha sistólica ->
    // acumulação ao longo de K -> gravação dos blocos de C -> status_done/IRQ.
    // -------------------------------------------------------------------------
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            dma_state         <= ST_IDLE;
            status_done       <= 1'b0;
            status_err        <= 1'b0;
            perf_total_cycles <= 32'd0;
            perf_comp_cycles  <= 32'd0;
            perf_mem_reads    <= 32'd0;

            row_base          <= 6'd0;
            col_base          <= 6'd0;
            k_base            <= 6'd0;
            buf_sel           <= 1'b0;
            loc_r             <= 6'd0;
            loc_c             <= 6'd0;

            burst_rem_cols    <= 6'd0;
            burst_cur_addr    <= 32'd0;
            burst_beats_total <= 5'd0;
            burst_beat_cnt    <= 5'd0;
            soft_rst_pending  <= 1'b0;
            axi_err_seen      <= 1'b0;

            m_axi_awaddr      <= 32'd0;
            m_axi_awlen       <= 8'd0;
            m_axi_awvalid     <= 1'b0;
            m_axi_wvalid      <= 1'b0;
            m_axi_bready      <= 1'b0;
            m_axi_araddr      <= 32'd0;
            m_axi_arlen       <= 8'd0;
            m_axi_arvalid     <= 1'b0;
            m_axi_rready      <= 1'b0;

            acc_clear         <= 1'b0;
            acc_start         <= 1'b0;
            buf_a_we          <= 1'b0;
            buf_a_row         <= {IDX_W{1'b0}};
            buf_a_col         <= {IDX_W{1'b0}};
            buf_a_wdata       <= 8'sd0;
            buf_b_we          <= 1'b0;
            buf_b_row         <= {IDX_W{1'b0}};
            buf_b_col         <= {IDX_W{1'b0}};
            buf_b_wdata       <= 8'sd0;
            buf_c_row         <= {IDX_W{1'b0}};
            buf_c_col         <= {IDX_W{1'b0}};
        end else begin
            // Sinais de pulso único retornam a zero por padrão
            acc_clear <= 1'b0;
            acc_start <= 1'b0;
            buf_a_we  <= 1'b0;
            buf_b_we  <= 1'b0;

            // Tratamento de W1C em DMA_STATUS (quando não colide com set)
            if (mmio_w1c_done_pulse) status_done <= 1'b0;
            if (mmio_w1c_err_pulse)  status_err  <= 1'b0;

            // Memoriza requisição de SOFT_RST durante operação em voo
            if (mmio_soft_rst_pulse) begin
                if (dma_state == ST_IDLE) begin
                    status_done       <= 1'b0;
                    status_err        <= 1'b0;
                    perf_total_cycles <= 32'd0;
                    perf_comp_cycles  <= 32'd0;
                    perf_mem_reads    <= 32'd0;
                    acc_clear         <= 1'b1;
                end else begin
                    soft_rst_pending  <= 1'b1;
                end
            end

            // Contadores de desempenho durante operação ativa
            if (dma_state != ST_IDLE) begin
                perf_total_cycles <= perf_total_cycles + 32'd1;
                if (acc_busy) begin
                    perf_comp_cycles <= perf_comp_cycles + 32'd1;
                end
            end

            case (dma_state)
                ST_IDLE: begin
                    m_axi_awvalid    <= 1'b0;
                    m_axi_wvalid     <= 1'b0;
                    m_axi_bready     <= 1'b0;
                    m_axi_arvalid    <= 1'b0;
                    m_axi_rready     <= 1'b0;
                    soft_rst_pending <= 1'b0;
                    axi_err_seen     <= 1'b0;

                    if (!mmio_soft_rst_pulse && mmio_start_pulse) begin
                        status_done <= 1'b0;
                        status_err  <= 1'b0;
                        if (!cfg_valid) begin
                            status_err <= 1'b1;
                        end else begin
                            perf_total_cycles <= 32'd0;
                            perf_comp_cycles  <= 32'd0;
                            perf_mem_reads    <= 32'd0;
                            row_base          <= 6'd0;
                            col_base          <= 6'd0;
                            k_base            <= 6'd0;
                            dma_state         <= ST_ACC_CLEAR;
                        end
                    end
                end

                ST_ACC_CLEAR: begin
                    acc_clear <= 1'b1;
                    k_base    <= 6'd0;
                    dma_state <= ST_TILE_INIT;
                end

                ST_TILE_INIT: begin
                    if (soft_rst_pending || axi_err_seen) begin
                        dma_state <= ST_ABORT_CLEAN;
                    end else begin
                        buf_sel   <= 1'b0;
                        loc_r     <= 6'd0;
                        loc_c     <= 6'd0;
                        dma_state <= ST_LOAD_ROW_PREP;
                    end
                end

                ST_LOAD_ROW_PREP: begin
                    if (soft_rst_pending || axi_err_seen) begin
                        dma_state <= ST_ABORT_CLEAN;
                    end else if (((buf_sel == 1'b0) && (prep_global_r >= cfg_m)) ||
                                 ((buf_sel == 1'b1) && (prep_global_r >= cfg_k))) begin
                        // Linha inteira fora dos limites reais -> preenche buffer local com zeros
                        loc_c          <= 6'd0;
                        burst_rem_cols <= 6'd0;
                        dma_state      <= ST_PAD_ZERO;
                    end else begin
                        loc_c          <= 6'd0;
                        burst_rem_cols <= prep_valid_cols;
                        burst_cur_addr <= ((buf_sel == 1'b0) ? reg_addr_a : reg_addr_b) + (prep_row_offset << 2);
                        dma_state      <= ST_RD_SETUP;
                    end
                end

                ST_RD_SETUP: begin
                    if (soft_rst_pending || axi_err_seen) begin
                        dma_state <= ST_ABORT_CLEAN;
                    end else if (burst_rem_cols == 6'd0) begin
                        if (loc_c < T_U6) begin
                            dma_state <= ST_PAD_ZERO;
                        end else begin
                            dma_state <= ST_NEXT_LOAD_ROW;
                        end
                    end else begin
                        burst_beats_total <= next_burst_words;
                        burst_beat_cnt    <= 5'd0;
                        m_axi_araddr      <= burst_cur_addr;
                        m_axi_arlen       <= {3'd0, next_burst_words} - 8'd1;
                        m_axi_arvalid     <= 1'b1;
                        m_axi_rready      <= 1'b1;
                        dma_state         <= ST_RD_BURST;
                    end
                end

                ST_RD_BURST: begin
                    if (m_axi_arvalid && m_axi_arready) begin
                        m_axi_arvalid <= 1'b0;
                    end

                    if (m_axi_rvalid && m_axi_rready) begin
                        if (buf_sel == 1'b0) begin
                            buf_a_we    <= 1'b1;
                            buf_a_row   <= loc_r[IDX_W-1:0];
                            buf_a_col   <= loc_c[IDX_W-1:0];
                            buf_a_wdata <= m_axi_rdata[7:0];
                        end else begin
                            buf_b_we    <= 1'b1;
                            buf_b_row   <= loc_r[IDX_W-1:0];
                            buf_b_col   <= loc_c[IDX_W-1:0];
                            buf_b_wdata <= m_axi_rdata[7:0];
                        end

                        perf_mem_reads <= perf_mem_reads + 32'd1;
                        if (m_axi_rresp != AXI_RESP_OKAY) begin
                            axi_err_seen <= 1'b1;
                        end

                        loc_c          <= loc_c + 6'd1;
                        burst_beat_cnt <= burst_beat_cnt + 5'd1;
                        burst_rem_cols <= burst_rem_cols - 6'd1;
                        burst_cur_addr <= burst_cur_addr + 32'd4;

                        if (m_axi_rlast) begin
                            m_axi_rready <= 1'b0;
                            if (soft_rst_pending || axi_err_seen || (m_axi_rresp != AXI_RESP_OKAY)) begin
                                dma_state <= ST_ABORT_CLEAN;
                            end else if (burst_rem_cols > 6'd1) begin
                                // Mais palavras na linha devido à quebra de fronteira 4 KB
                                dma_state <= ST_RD_SETUP;
                            end else if ((loc_c + 6'd1) < T_U6) begin
                                dma_state <= ST_PAD_ZERO;
                            end else begin
                                dma_state <= ST_NEXT_LOAD_ROW;
                            end
                        end
                    end
                end

                ST_PAD_ZERO: begin
                    if (buf_sel == 1'b0) begin
                        buf_a_we    <= 1'b1;
                        buf_a_row   <= loc_r[IDX_W-1:0];
                        buf_a_col   <= loc_c[IDX_W-1:0];
                        buf_a_wdata <= 8'sd0;
                    end else begin
                        buf_b_we    <= 1'b1;
                        buf_b_row   <= loc_r[IDX_W-1:0];
                        buf_b_col   <= loc_c[IDX_W-1:0];
                        buf_b_wdata <= 8'sd0;
                    end

                    if (loc_c == (T_U6 - 6'd1)) begin
                        dma_state <= ST_NEXT_LOAD_ROW;
                    end else begin
                        loc_c <= loc_c + 6'd1;
                    end
                end

                ST_NEXT_LOAD_ROW: begin
                    if (loc_r < (T_U6 - 6'd1)) begin
                        loc_r     <= loc_r + 6'd1;
                        dma_state <= ST_LOAD_ROW_PREP;
                    end else if (buf_sel == 1'b0) begin
                        buf_sel   <= 1'b1;
                        loc_r     <= 6'd0;
                        dma_state <= ST_LOAD_ROW_PREP;
                    end else begin
                        dma_state <= ST_ACC_START;
                    end
                end

                ST_ACC_START: begin
                    if (soft_rst_pending) begin
                        dma_state <= ST_ABORT_CLEAN;
                    end else begin
                        acc_start <= 1'b1;
                        dma_state <= ST_WAIT_ACC;
                    end
                end

                ST_WAIT_ACC: begin
                    if (acc_done) begin
                        if (soft_rst_pending) begin
                            dma_state <= ST_ABORT_CLEAN;
                        end else if ((k_base + T_U6) < cfg_k) begin
                            // Mais blocos ao longo de K: mantém acumuladores intactos!
                            k_base    <= k_base + T_U6;
                            dma_state <= ST_TILE_INIT;
                        end else begin
                            // Todas as parcelas tk acumuladas -> grava C[ti][tj] na DMEM
                            loc_r     <= 6'd0;
                            dma_state <= ST_WR_ROW_PREP;
                        end
                    end
                end

                ST_WR_ROW_PREP: begin
                    if (soft_rst_pending || axi_err_seen) begin
                        dma_state <= ST_ABORT_CLEAN;
                    end else begin
                        loc_c          <= 6'd0;
                        burst_rem_cols <= valid_c_cols;
                        burst_cur_addr <= reg_addr_c + (prep_row_offset << 2);
                        dma_state      <= ST_WR_SETUP;
                    end
                end

                ST_WR_SETUP: begin
                    // Posiciona endereço combinacional do acumulador antes de subir WVALID
                    buf_c_row         <= loc_r[IDX_W-1:0];
                    buf_c_col         <= loc_c[IDX_W-1:0];
                    burst_beats_total <= next_burst_words;
                    burst_beat_cnt    <= 5'd0;

                    m_axi_awaddr      <= burst_cur_addr;
                    m_axi_awlen       <= {3'd0, next_burst_words} - 8'd1;
                    m_axi_awvalid     <= 1'b1;
                    m_axi_wvalid      <= 1'b1;
                    dma_state         <= ST_WR_BURST;
                end

                ST_WR_BURST: begin
                    if (m_axi_awvalid && m_axi_awready) begin
                        m_axi_awvalid <= 1'b0;
                    end

                    if (m_axi_wvalid && m_axi_wready) begin
                        loc_c          <= loc_c + 6'd1;
                        buf_c_col      <= loc_c[IDX_W-1:0] + {{(IDX_W-1){1'b0}}, 1'b1};
                        burst_beat_cnt <= burst_beat_cnt + 5'd1;
                        burst_rem_cols <= burst_rem_cols - 6'd1;
                        burst_cur_addr <= burst_cur_addr + 32'd4;

                        if ((burst_beat_cnt + 5'd1) == burst_beats_total) begin
                            m_axi_wvalid <= 1'b0;
                            m_axi_bready <= 1'b1;
                            dma_state    <= ST_WR_RESP;
                        end
                    end
                end

                ST_WR_RESP: begin
                    if (m_axi_bvalid && m_axi_bready) begin
                        m_axi_bready <= 1'b0;
                        if (soft_rst_pending || axi_err_seen || (m_axi_bresp != AXI_RESP_OKAY)) begin
                            if (m_axi_bresp != AXI_RESP_OKAY) axi_err_seen <= 1'b1;
                            dma_state <= ST_ABORT_CLEAN;
                        end else if (burst_rem_cols > 6'd0) begin
                            // Continua mesma linha após fronteira de 4 KB
                            dma_state <= ST_WR_SETUP;
                        end else if ((loc_r + 6'd1) < valid_c_rows) begin
                            loc_r     <= loc_r + 6'd1;
                            dma_state <= ST_WR_ROW_PREP;
                        end else begin
                            // Bloco C[ti][tj] inteiramente gravado na DMEM!
                            if ((col_base + T_U6) < cfg_n) begin
                                col_base  <= col_base + T_U6;
                                dma_state <= ST_ACC_CLEAR;
                            end else if ((row_base + T_U6) < cfg_m) begin
                                col_base  <= 6'd0;
                                row_base  <= row_base + T_U6;
                                dma_state <= ST_ACC_CLEAR;
                            end else begin
                                // Matriz C inteira concluída!
                                status_done <= 1'b1;
                                dma_state   <= ST_IDLE;
                            end
                        end
                    end
                end

                ST_ABORT_CLEAN: begin
                    acc_clear        <= 1'b1;
                    soft_rst_pending <= 1'b0;
                    if (axi_err_seen && !soft_rst_pending) begin
                        status_err <= 1'b1;
                    end else begin
                        status_done       <= 1'b0;
                        status_err        <= 1'b0;
                        perf_total_cycles <= 32'd0;
                        perf_comp_cycles  <= 32'd0;
                        perf_mem_reads    <= 32'd0;
                    end
                    axi_err_seen <= 1'b0;
                    dma_state    <= ST_IDLE;
                end

                default: dma_state <= ST_IDLE;
            endcase
        end
    end

endmodule
