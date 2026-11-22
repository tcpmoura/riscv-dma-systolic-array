// =============================================================================
// Projeto     : RISC-V RV32I com Controlador DMA e Coprocessador Matricial
// Etapa       : Entrega Parcial III — DMA, Barramento AXI4 e Memória Compartilhada
// Módulo      : axi4_interconnect.v
// Professores : Ana Julia Fernandes de Oliveira Barros
//               Igor Dantas dos Santos Miranda
//               João Alberto Castelo Branco Oliveira
// Grupo 4     : João Vitor Carvalho Lima
//               Matheus Artur Macedo Bomfim
//               Nathalia Miranda
//               Tatiana Caetano Praxedes de Moura
// Descrição   : Matriz de Interconexão AXI4 (2 Mestres x 2 Escravos + DECERR)
//               com transações serializadas (1 transação ativa por vez).
//               - Mestres : M0 = CPU / Testbench BFM (menor prioridade)
//                           M1 = Controlador DMA (prioridade fixa sobre M0)
//               - Escravos: S0 = DMEM compartilhada (0x0000_0000..0x0000_3FFF)
//                           S1 = MMIO do DMA        (0x4000_0000..0x4000_002F)
//               - Arbitragem não preemptiva: nunca interrompe uma transação em
//                 curso; aguarda o handshake final (BVALID&BREADY ou
//                 RVALID&RREADY&RLAST). Endereços fora de faixa ou rajadas que
//                 ultrapassem o limite recebem DECERR (2'b11) interno.
// =============================================================================
`timescale 1ns / 1ps

module axi4_interconnect (
    input  wire        clk,             // Clock principal do sistema
    input  wire        rst_n,           // Reset assíncrono ativo em nível baixo

    // =========================================================================
    // Mestre 0 (CPU / Testbench BFM)
    // =========================================================================
    input  wire [31:0] m0_axi_awaddr,
    input  wire [7:0]  m0_axi_awlen,
    input  wire [2:0]  m0_axi_awsize,
    input  wire [1:0]  m0_axi_awburst,
    input  wire        m0_axi_awvalid,
    output reg         m0_axi_awready,

    input  wire [31:0] m0_axi_wdata,
    input  wire [3:0]  m0_axi_wstrb,
    input  wire        m0_axi_wlast,
    input  wire        m0_axi_wvalid,
    output reg         m0_axi_wready,

    output reg  [1:0]  m0_axi_bresp,
    output reg         m0_axi_bvalid,
    input  wire        m0_axi_bready,

    input  wire [31:0] m0_axi_araddr,
    input  wire [7:0]  m0_axi_arlen,
    input  wire [2:0]  m0_axi_arsize,
    input  wire [1:0]  m0_axi_arburst,
    input  wire        m0_axi_arvalid,
    output reg         m0_axi_arready,

    output reg  [31:0] m0_axi_rdata,
    output reg  [1:0]  m0_axi_rresp,
    output reg         m0_axi_rlast,
    output reg         m0_axi_rvalid,
    input  wire        m0_axi_rready,

    // =========================================================================
    // Mestre 1 (Controlador DMA — Prioridade Fixa)
    // =========================================================================
    input  wire [31:0] m1_axi_awaddr,
    input  wire [7:0]  m1_axi_awlen,
    input  wire [2:0]  m1_axi_awsize,
    input  wire [1:0]  m1_axi_awburst,
    input  wire        m1_axi_awvalid,
    output reg         m1_axi_awready,

    input  wire [31:0] m1_axi_wdata,
    input  wire [3:0]  m1_axi_wstrb,
    input  wire        m1_axi_wlast,
    input  wire        m1_axi_wvalid,
    output reg         m1_axi_wready,

    output reg  [1:0]  m1_axi_bresp,
    output reg         m1_axi_bvalid,
    input  wire        m1_axi_bready,

    input  wire [31:0] m1_axi_araddr,
    input  wire [7:0]  m1_axi_arlen,
    input  wire [2:0]  m1_axi_arsize,
    input  wire [1:0]  m1_axi_arburst,
    input  wire        m1_axi_arvalid,
    output reg         m1_axi_arready,

    output reg  [31:0] m1_axi_rdata,
    output reg  [1:0]  m1_axi_rresp,
    output reg         m1_axi_rlast,
    output reg         m1_axi_rvalid,
    input  wire        m1_axi_rready,

    // =========================================================================
    // Escravo 0 (DMEM Compartilhada: 0x0000_0000 .. 0x0000_3FFF)
    // =========================================================================
    output reg  [31:0] s0_axi_awaddr,
    output reg  [7:0]  s0_axi_awlen,
    output reg  [2:0]  s0_axi_awsize,
    output reg  [1:0]  s0_axi_awburst,
    output reg         s0_axi_awvalid,
    input  wire        s0_axi_awready,

    output reg  [31:0] s0_axi_wdata,
    output reg  [3:0]  s0_axi_wstrb,
    output reg         s0_axi_wlast,
    output reg         s0_axi_wvalid,
    input  wire        s0_axi_wready,

    input  wire [1:0]  s0_axi_bresp,
    input  wire        s0_axi_bvalid,
    output reg         s0_axi_bready,

    output reg  [31:0] s0_axi_araddr,
    output reg  [7:0]  s0_axi_arlen,
    output reg  [2:0]  s0_axi_arsize,
    output reg  [1:0]  s0_axi_arburst,
    output reg         s0_axi_arvalid,
    input  wire        s0_axi_arready,

    input  wire [31:0] s0_axi_rdata,
    input  wire [1:0]  s0_axi_rresp,
    input  wire        s0_axi_rlast,
    input  wire        s0_axi_rvalid,
    output reg         s0_axi_rready,

    // =========================================================================
    // Escravo 1 (MMIO do DMA: 0x4000_0000 .. 0x4000_002F)
    // =========================================================================
    output reg  [31:0] s1_axi_awaddr,
    output reg  [7:0]  s1_axi_awlen,
    output reg  [2:0]  s1_axi_awsize,
    output reg  [1:0]  s1_axi_awburst,
    output reg         s1_axi_awvalid,
    input  wire        s1_axi_awready,

    output reg  [31:0] s1_axi_wdata,
    output reg  [3:0]  s1_axi_wstrb,
    output reg         s1_axi_wlast,
    output reg         s1_axi_wvalid,
    input  wire        s1_axi_wready,

    input  wire [1:0]  s1_axi_bresp,
    input  wire        s1_axi_bvalid,
    output reg         s1_axi_bready,

    output reg  [31:0] s1_axi_araddr,
    output reg  [7:0]  s1_axi_arlen,
    output reg  [2:0]  s1_axi_arsize,
    output reg  [1:0]  s1_axi_arburst,
    output reg         s1_axi_arvalid,
    input  wire        s1_axi_arready,

    input  wire [31:0] s1_axi_rdata,
    input  wire [1:0]  s1_axi_rresp,
    input  wire        s1_axi_rlast,
    input  wire        s1_axi_rvalid,
    output reg         s1_axi_rready
);

    // Estados da FSM de arbitragem serializada (1 transação ativa no barramento)
    localparam [2:0] ARB_IDLE   = 3'd0; // Barramento livre: avalia requisições (prioridade M1 > M0)
    localparam [2:0] ARB_WR     = 3'd1; // Encaminha transação de escrita válida (AW/W -> B) para S0/S1
    localparam [2:0] ARB_RD     = 3'd2; // Encaminha transação de leitura válida (AR -> R) para S0/S1
    localparam [2:0] ARB_DEC_WR = 3'd3; // Respondedor interno DECERR para escrita fora de faixa
    localparam [2:0] ARB_DEC_RD = 3'd4; // Respondedor interno DECERR para leitura fora de faixa

    // Identificadores internos dos escravos decodificados
    localparam [1:0] SLV_DMEM   = 2'd0; // Escravo 0: Memória de dados compartilhada (16 KB)
    localparam [1:0] SLV_MMIO   = 2'd1; // Escravo 1: Registradores MMIO do DMA
    localparam [1:0] SLV_DECERR = 2'd2; // Endereço inválido ou rajada ultrapassando a faixa

    localparam [1:0] AXI_RESP_DECERR = 2'b11; // Código de erro de decodificação AXI4

    // Registradores internos de estado e concessão de barramento
    reg [2:0] state;        // Estado atual do árbitro/roteador
    reg       grant_m;      // Mestre concedido: 0 = M0 (CPU/TB), 1 = M1 (DMA)
    reg [1:0] grant_s;      // Escravo concedido: SLV_DMEM, SLV_MMIO ou SLV_DECERR
    reg       aw_done;      // Indica que o handshake AW já ocorreu na escrita atual
    reg       w_done;       // Indica que o último beat W (WLAST) já foi transferido
    reg       ar_done;      // Indica que o handshake AR já ocorreu na leitura atual
    reg [7:0] dec_len;      // Comprimento ARLEN registrado para gerar todos os beats DECERR
    reg [7:0] dec_cnt;      // Contador de beats já respondidos em ARB_DEC_RD

    // -------------------------------------------------------------------------
    // Função de decodificação completa de faixas de endereço
    // Calcula o último byte da rajada (addr + 4*len + 3) em 33 bits para
    // garantir que nenhum beat ultrapasse o limite do escravo selecionado.
    // -------------------------------------------------------------------------
    function [1:0] decode_slave;
        input [31:0] addr;  // Endereço inicial da transação (AWADDR ou ARADDR)
        input [7:0]  len;   // Comprimento AXI4 da rajada (AWLEN ou ARLEN)
        reg   [32:0] end_byte; // Último byte acessado pela rajada (33b evita overflow)
        begin
            end_byte = {1'b0, addr} + ({23'd0, len, 2'b00}) + 33'd3;
            // DMEM: [0x0000_0000 .. 0x0000_3FFF]
            if ((addr[31:14] == 18'd0) && (end_byte <= 33'h0_0000_3FFF)) begin
                decode_slave = SLV_DMEM;
            // MMIO DMA: [0x4000_0000 .. 0x4000_002F] (apenas palavra única)
            end else if ((addr[31:6] == 26'h1000000) && (addr[5:0] <= 6'h2F) &&
                         (end_byte <= 33'h0_4000_002F) && (len == 8'd0)) begin
                decode_slave = SLV_MMIO;
            end else begin
                decode_slave = SLV_DECERR;
            end
        end
    endfunction

    // Sinais multiplexados do mestre ativo (baseado apenas em grant_m registrado)
    wire        act_m_awvalid = grant_m ? m1_axi_awvalid : m0_axi_awvalid; // AWVALID do mestre concedido
    wire [31:0] act_m_awaddr  = grant_m ? m1_axi_awaddr  : m0_axi_awaddr;  // AWADDR do mestre concedido
    wire [7:0]  act_m_awlen   = grant_m ? m1_axi_awlen   : m0_axi_awlen;   // AWLEN do mestre concedido
    wire [2:0]  act_m_awsize  = grant_m ? m1_axi_awsize  : m0_axi_awsize;  // AWSIZE do mestre concedido
    wire [1:0]  act_m_awburst = grant_m ? m1_axi_awburst : m0_axi_awburst; // AWBURST do mestre concedido

    wire        act_m_wvalid  = grant_m ? m1_axi_wvalid  : m0_axi_wvalid;  // WVALID do mestre concedido
    wire [31:0] act_m_wdata   = grant_m ? m1_axi_wdata   : m0_axi_wdata;   // WDATA do mestre concedido
    wire [3:0]  act_m_wstrb   = grant_m ? m1_axi_wstrb   : m0_axi_wstrb;   // WSTRB do mestre concedido
    wire        act_m_wlast   = grant_m ? m1_axi_wlast   : m0_axi_wlast;   // WLAST do mestre concedido

    wire        act_m_bready  = grant_m ? m1_axi_bready  : m0_axi_bready;  // BREADY do mestre concedido

    wire        act_m_arvalid = grant_m ? m1_axi_arvalid : m0_axi_arvalid; // ARVALID do mestre concedido
    wire [31:0] act_m_araddr  = grant_m ? m1_axi_araddr  : m0_axi_araddr;  // ARADDR do mestre concedido
    wire [7:0]  act_m_arlen   = grant_m ? m1_axi_arlen   : m0_axi_arlen;   // ARLEN do mestre concedido
    wire [2:0]  act_m_arsize  = grant_m ? m1_axi_arsize  : m0_axi_arsize;  // ARSIZE do mestre concedido
    wire [1:0]  act_m_arburst = grant_m ? m1_axi_arburst : m0_axi_arburst; // ARBURST do mestre concedido

    wire        act_m_rready  = grant_m ? m1_axi_rready  : m0_axi_rready;  // RREADY do mestre concedido

    // Sinais multiplexados de resposta do escravo ativo (S0 = DMEM ou S1 = MMIO)
    wire        act_s_awready = (grant_s == SLV_DMEM) ? s0_axi_awready : s1_axi_awready; // AWREADY do escravo ativo
    wire        act_s_wready  = (grant_s == SLV_DMEM) ? s0_axi_wready  : s1_axi_wready;  // WREADY do escravo ativo
    wire        act_s_bvalid  = (grant_s == SLV_DMEM) ? s0_axi_bvalid  : s1_axi_bvalid;  // BVALID do escravo ativo
    wire [1:0]  act_s_bresp   = (grant_s == SLV_DMEM) ? s0_axi_bresp   : s1_axi_bresp;   // BRESP do escravo ativo

    wire        act_s_arready = (grant_s == SLV_DMEM) ? s0_axi_arready : s1_axi_arready; // ARREADY do escravo ativo
    wire        act_s_rvalid  = (grant_s == SLV_DMEM) ? s0_axi_rvalid  : s1_axi_rvalid;  // RVALID do escravo ativo
    wire [31:0] act_s_rdata   = (grant_s == SLV_DMEM) ? s0_axi_rdata   : s1_axi_rdata;   // RDATA do escravo ativo
    wire [1:0]  act_s_rresp   = (grant_s == SLV_DMEM) ? s0_axi_rresp   : s1_axi_rresp;   // RRESP do escravo ativo
    wire        act_s_rlast   = (grant_s == SLV_DMEM) ? s0_axi_rlast   : s1_axi_rlast;   // RLAST do escravo ativo

    // -------------------------------------------------------------------------
    // FSM Principal de Arbitragem e Controle de Transação Única
    // Concede o barramento a M1 (DMA) ou M0 (CPU/TB) em ARB_IDLE e mantém a
    // conexão travada até o handshake de término (BVALID&BREADY ou RLAST&RREADY).
    // -------------------------------------------------------------------------
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            state    <= ARB_IDLE;
            grant_m  <= 1'b0;
            grant_s  <= SLV_DMEM;
            aw_done  <= 1'b0;
            w_done   <= 1'b0;
            ar_done  <= 1'b0;
            dec_len  <= 8'd0;
            dec_cnt  <= 8'd0;
        end else begin
            case (state)
                ARB_IDLE: begin
                    aw_done <= 1'b0;
                    w_done  <= 1'b0;
                    ar_done <= 1'b0;
                    dec_cnt <= 8'd0;

                    // Prioridade fixa: M1 (DMA) tem preferência sobre M0 (CPU/TB)
                    if (m1_axi_awvalid) begin
                        grant_m <= 1'b1;
                        grant_s <= decode_slave(m1_axi_awaddr, m1_axi_awlen);
                        state   <= (decode_slave(m1_axi_awaddr, m1_axi_awlen) == SLV_DECERR) ? ARB_DEC_WR : ARB_WR;
                    end else if (m1_axi_arvalid) begin
                        grant_m <= 1'b1;
                        grant_s <= decode_slave(m1_axi_araddr, m1_axi_arlen);
                        dec_len <= m1_axi_arlen;
                        state   <= (decode_slave(m1_axi_araddr, m1_axi_arlen) == SLV_DECERR) ? ARB_DEC_RD : ARB_RD;
                    end else if (m0_axi_awvalid) begin
                        grant_m <= 1'b0;
                        grant_s <= decode_slave(m0_axi_awaddr, m0_axi_awlen);
                        state   <= (decode_slave(m0_axi_awaddr, m0_axi_awlen) == SLV_DECERR) ? ARB_DEC_WR : ARB_WR;
                    end else if (m0_axi_arvalid) begin
                        grant_m <= 1'b0;
                        grant_s <= decode_slave(m0_axi_araddr, m0_axi_arlen);
                        dec_len <= m0_axi_arlen;
                        state   <= (decode_slave(m0_axi_araddr, m0_axi_arlen) == SLV_DECERR) ? ARB_DEC_RD : ARB_RD;
                    end
                end

                ARB_WR: begin
                    if (!aw_done && act_m_awvalid && act_s_awready) begin
                        aw_done <= 1'b1;
                    end
                    if (!w_done && act_m_wvalid && act_s_wready && act_m_wlast) begin
                        w_done <= 1'b1;
                    end
                    if (act_s_bvalid && act_m_bready) begin
                        state <= ARB_IDLE;
                    end
                end

                ARB_RD: begin
                    if (!ar_done && act_m_arvalid && act_s_arready) begin
                        ar_done <= 1'b1;
                    end
                    if (act_s_rvalid && act_m_rready && act_s_rlast) begin
                        state <= ARB_IDLE;
                    end
                end

                ARB_DEC_WR: begin
                    if (!aw_done && act_m_awvalid) begin
                        aw_done <= 1'b1;
                    end
                    if (aw_done && !w_done && act_m_wvalid && act_m_wlast) begin
                        w_done <= 1'b1;
                    end
                    if (aw_done && w_done && act_m_bready) begin
                        state <= ARB_IDLE;
                    end
                end

                ARB_DEC_RD: begin
                    if (!ar_done && act_m_arvalid) begin
                        ar_done <= 1'b1;
                        dec_cnt <= 8'd0;
                    end else if (ar_done && act_m_rready) begin
                        if (dec_cnt == dec_len) begin
                            state <= ARB_IDLE;
                        end else begin
                            dec_cnt <= dec_cnt + 8'd1;
                        end
                    end
                end

                default: state <= ARB_IDLE;
            endcase
        end
    end

    // -------------------------------------------------------------------------
    // Multiplexação Combinacional Livre de Laços entre Mestre e Escravo Concedidos
    // -------------------------------------------------------------------------
    always @(*) begin
        // Valores padrão seguros para todos os mestres
        m0_axi_awready = 1'b0; m0_axi_wready = 1'b0;
        m0_axi_bresp   = 2'b00; m0_axi_bvalid = 1'b0;
        m0_axi_arready = 1'b0; m0_axi_rdata  = 32'd0;
        m0_axi_rresp   = 2'b00; m0_axi_rlast  = 1'b0; m0_axi_rvalid = 1'b0;

        m1_axi_awready = 1'b0; m1_axi_wready = 1'b0;
        m1_axi_bresp   = 2'b00; m1_axi_bvalid = 1'b0;
        m1_axi_arready = 1'b0; m1_axi_rdata  = 32'd0;
        m1_axi_rresp   = 2'b00; m1_axi_rlast  = 1'b0; m1_axi_rvalid = 1'b0;

        // Valores padrão seguros para todos os escravos
        s0_axi_awaddr  = 32'd0; s0_axi_awlen = 8'd0; s0_axi_awsize = 3'b010; s0_axi_awburst = 2'b01; s0_axi_awvalid = 1'b0;
        s0_axi_wdata   = 32'd0; s0_axi_wstrb = 4'd0; s0_axi_wlast  = 1'b0;   s0_axi_wvalid  = 1'b0;
        s0_axi_bready  = 1'b0;
        s0_axi_araddr  = 32'd0; s0_axi_arlen = 8'd0; s0_axi_arsize = 3'b010; s0_axi_arburst = 2'b01; s0_axi_arvalid = 1'b0;
        s0_axi_rready  = 1'b0;

        s1_axi_awaddr  = 32'd0; s1_axi_awlen = 8'd0; s1_axi_awsize = 3'b010; s1_axi_awburst = 2'b01; s1_axi_awvalid = 1'b0;
        s1_axi_wdata   = 32'd0; s1_axi_wstrb = 4'd0; s1_axi_wlast  = 1'b0;   s1_axi_wvalid  = 1'b0;
        s1_axi_bready  = 1'b0;
        s1_axi_araddr  = 32'd0; s1_axi_arlen = 8'd0; s1_axi_arsize = 3'b010; s1_axi_arburst = 2'b01; s1_axi_arvalid = 1'b0;
        s1_axi_rready  = 1'b0;

        case (state)
            ARB_WR: begin
                if (grant_s == SLV_DMEM) begin
                    s0_axi_awaddr  = act_m_awaddr;
                    s0_axi_awlen   = act_m_awlen;
                    s0_axi_awsize  = act_m_awsize;
                    s0_axi_awburst = act_m_awburst;
                    s0_axi_awvalid = (!aw_done) ? act_m_awvalid : 1'b0;

                    s0_axi_wdata   = act_m_wdata;
                    s0_axi_wstrb   = act_m_wstrb;
                    s0_axi_wlast   = act_m_wlast;
                    s0_axi_wvalid  = (!w_done) ? act_m_wvalid : 1'b0;

                    s0_axi_bready  = act_m_bready;
                end else if (grant_s == SLV_MMIO) begin
                    s1_axi_awaddr  = act_m_awaddr;
                    s1_axi_awlen   = act_m_awlen;
                    s1_axi_awsize  = act_m_awsize;
                    s1_axi_awburst = act_m_awburst;
                    s1_axi_awvalid = (!aw_done) ? act_m_awvalid : 1'b0;

                    s1_axi_wdata   = act_m_wdata;
                    s1_axi_wstrb   = act_m_wstrb;
                    s1_axi_wlast   = act_m_wlast;
                    s1_axi_wvalid  = (!w_done) ? act_m_wvalid : 1'b0;

                    s1_axi_bready  = act_m_bready;
                end

                if (grant_m == 1'b0) begin
                    m0_axi_awready = (!aw_done) ? act_s_awready : 1'b0;
                    m0_axi_wready  = (!w_done)  ? act_s_wready  : 1'b0;
                    m0_axi_bvalid  = act_s_bvalid;
                    m0_axi_bresp   = act_s_bresp;
                end else begin
                    m1_axi_awready = (!aw_done) ? act_s_awready : 1'b0;
                    m1_axi_wready  = (!w_done)  ? act_s_wready  : 1'b0;
                    m1_axi_bvalid  = act_s_bvalid;
                    m1_axi_bresp   = act_s_bresp;
                end
            end

            ARB_RD: begin
                if (grant_s == SLV_DMEM) begin
                    s0_axi_araddr  = act_m_araddr;
                    s0_axi_arlen   = act_m_arlen;
                    s0_axi_arsize  = act_m_arsize;
                    s0_axi_arburst = act_m_arburst;
                    s0_axi_arvalid = (!ar_done) ? act_m_arvalid : 1'b0;
                    s0_axi_rready  = act_m_rready;
                end else if (grant_s == SLV_MMIO) begin
                    s1_axi_araddr  = act_m_araddr;
                    s1_axi_arlen   = act_m_arlen;
                    s1_axi_arsize  = act_m_arsize;
                    s1_axi_arburst = act_m_arburst;
                    s1_axi_arvalid = (!ar_done) ? act_m_arvalid : 1'b0;
                    s1_axi_rready  = act_m_rready;
                end

                if (grant_m == 1'b0) begin
                    m0_axi_arready = (!ar_done) ? act_s_arready : 1'b0;
                    m0_axi_rvalid  = act_s_rvalid;
                    m0_axi_rdata   = act_s_rdata;
                    m0_axi_rresp   = act_s_rresp;
                    m0_axi_rlast   = act_s_rlast;
                end else begin
                    m1_axi_arready = (!ar_done) ? act_s_arready : 1'b0;
                    m1_axi_rvalid  = act_s_rvalid;
                    m1_axi_rdata   = act_s_rdata;
                    m1_axi_rresp   = act_s_rresp;
                    m1_axi_rlast   = act_s_rlast;
                end
            end

            ARB_DEC_WR: begin
                if (grant_m == 1'b0) begin
                    m0_axi_awready = (!aw_done);
                    m0_axi_wready  = (aw_done && !w_done);
                    m0_axi_bvalid  = (aw_done && w_done);
                    m0_axi_bresp   = AXI_RESP_DECERR;
                end else begin
                    m1_axi_awready = (!aw_done);
                    m1_axi_wready  = (aw_done && !w_done);
                    m1_axi_bvalid  = (aw_done && w_done);
                    m1_axi_bresp   = AXI_RESP_DECERR;
                end
            end

            ARB_DEC_RD: begin
                if (grant_m == 1'b0) begin
                    m0_axi_arready = (!ar_done);
                    m0_axi_rvalid  = ar_done;
                    m0_axi_rdata   = 32'hDEAD_DEC0;
                    m0_axi_rresp   = AXI_RESP_DECERR;
                    m0_axi_rlast   = ar_done && (dec_cnt == dec_len);
                end else begin
                    m1_axi_arready = (!ar_done);
                    m1_axi_rvalid  = ar_done;
                    m1_axi_rdata   = 32'hDEAD_DEC0;
                    m1_axi_rresp   = AXI_RESP_DECERR;
                    m1_axi_rlast   = ar_done && (dec_cnt == dec_len);
                end
            end

            default: ;
        endcase
    end

endmodule
