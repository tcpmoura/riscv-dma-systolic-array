// =============================================================================
// Projeto     : RISC-V RV32I com Controlador DMA e Coprocessador Matricial
// Etapa       : Entrega Parcial IV — Integração Top-Level e Firmware
// Módulo      : cpu_axi4_bridge.v
// Professores : Ana Julia Fernandes de Oliveira Barros
//               Igor Dantas dos Santos Miranda
//               João Alberto Castelo Branco Oliveira
// Grupo 4     : João Vitor Carvalho Lima
//               Matheus Artur Macedo Bomfim
//               Nathalia Miranda
//               Tatiana Caetano Praxedes de Moura
// Descrição   : Ponte síncrona sintetizável (Verilog) que converte a interface
//               de memória de dados valid/ready do processador rv32i_core para
//               a porta Mestre 0 AXI4 (m0_axi_*) de dma_mem_acc_subsys.
//
// Características de Protocolo e Temporização:
//   - Atende exatamente uma requisição da CPU por vez (LEN=0, 1 beat de 32b,
//     SIZE=3'b010, BURST=2'b01 INCR, WLAST=1'b1).
//   - Captura endereço, dado e wstrb em registradores ao aceitar cpu_valid,
//     mantendo AW/W/AR imutáveis durante READY=0 (conformidade AXI4 estrita).
//   - Entrega cpu_ready = 1 por exatamente 1 ciclo no estado ST_RESP junto com
//     cpu_rdata e cpu_error (BRESP/RRESP != OKAY), mantendo cpu_error = 0 fora
//     do ciclo de conclusão para não bloquear o sinal complete da CPU.
// =============================================================================
`timescale 1ns / 1ps

module cpu_axi4_bridge (
    input  wire        clk,             // Clock principal do sistema (borda de subida)
    input  wire        rst_n,           // Reset global assíncrono ativo em nível baixo

    // =========================================================================
    // Interface de Dados da CPU (rv32i_core: valid/ready de palavra única)
    // =========================================================================
    input  wire        cpu_valid,       // CPU solicita leitura (LW/LH/LB) ou escrita (SW/SH/SB)
    output wire        cpu_ready,       // Pulso de 1 ciclo confirmando conclusão da requisição
    input  wire        cpu_write,       // 1 = escrita (Store), 0 = leitura (Load)
    input  wire [31:0] cpu_addr,        // Endereço alinhado em palavra de 32 bits
    input  wire [31:0] cpu_wdata,       // Dado de escrita posicionado nos bytes pela LSU
    input  wire [3:0]  cpu_wstrb,       // Máscara de escrita por byte (SW/SH/SB)
    output wire [31:0] cpu_rdata,       // Palavra de 32 bits lida da DMEM ou MMIO
    output wire        cpu_error,       // 1 apenas no ciclo cpu_ready se BRESP/RRESP != OKAY

    // =========================================================================
    // Porta Mestre 0 AXI4 (conectada a dma_mem_acc_subsys.m0_axi_*)
    // =========================================================================
    output wire [31:0] m_axi_awaddr,    // Endereço da escrita AXI4
    output wire [7:0]  m_axi_awlen,     // Comprimento fixo 0 (1 beat por transação)
    output wire [2:0]  m_axi_awsize,    // Tamanho fixo 3'b010 (4 bytes = 32 bits)
    output wire [1:0]  m_axi_awburst,   // Tipo fixo 2'b01 (INCR)
    output reg         m_axi_awvalid,   // Requisição de endereço de escrita válida
    input  wire        m_axi_awready,   // Aceitação do canal AW pelo barramento

    output wire [31:0] m_axi_wdata,     // Dado de escrita AXI4 (32 bits)
    output wire [3:0]  m_axi_wstrb,     // Máscara de bytes ativos na escrita
    output wire        m_axi_wlast,     // Sempre 1 (transação de beat único)
    output reg         m_axi_wvalid,    // Dado de escrita válido
    input  wire        m_axi_wready,    // Aceitação do canal W pelo barramento

    input  wire [1:0]  m_axi_bresp,     // Resposta de escrita (00=OKAY, 10=SLVERR, 11=DECERR)
    input  wire        m_axi_bvalid,    // Resposta B disponível no barramento
    output wire        m_axi_bready,    // Ponte pronta para aceitar resposta B

    output wire [31:0] m_axi_araddr,    // Endereço da leitura AXI4
    output wire [7:0]  m_axi_arlen,     // Comprimento fixo 0 (1 beat por transação)
    output wire [2:0]  m_axi_arsize,    // Tamanho fixo 3'b010 (4 bytes = 32 bits)
    output wire [1:0]  m_axi_arburst,   // Tipo fixo 2'b01 (INCR)
    output reg         m_axi_arvalid,   // Requisição de endereço de leitura válida
    input  wire        m_axi_arready,   // Aceitação do canal AR pelo barramento

    input  wire [31:0] m_axi_rdata,     // Palavra de 32 bits retornada pelo escravo
    input  wire [1:0]  m_axi_rresp,     // Resposta de leitura (00=OKAY, 10=SLVERR, 11=DECERR)
    input  wire        m_axi_rlast,     // Último beat da leitura (esperado 1 para LEN=0)
    input  wire        m_axi_rvalid,    // Dado de leitura disponível no barramento
    output wire        m_axi_rready     // Ponte pronta para aceitar resposta R
);

    // -------------------------------------------------------------------------
    // Codificação dos Estados da FSM da Ponte CPU -> AXI4
    // -------------------------------------------------------------------------
    localparam [1:0] ST_IDLE    = 2'd0; // Aguarda nova requisição da CPU (cpu_valid = 1)
    localparam [1:0] ST_WR_WAIT = 2'd1; // Executa escrita AXI4 (AW + W -> aguarda BVALID)
    localparam [1:0] ST_RD_WAIT = 2'd2; // Executa leitura AXI4 (AR -> aguarda RVALID)
    localparam [1:0] ST_RESP    = 2'd3; // Entrega cpu_ready=1 por 1 ciclo exato à CPU

    localparam [1:0] AXI_RESP_OKAY = 2'b00; // Código AXI4 de transação bem-sucedida

    // Registradores de estado e captura estável da transação em andamento
    reg [1:0]  state;        // Estado corrente da FSM
    reg [31:0] lat_addr;     // Endereço capturado de cpu_addr (estável durante stall AXI4)
    reg [31:0] lat_wdata;    // Dado capturado de cpu_wdata (estável durante stall AXI4)
    reg [3:0]  lat_wstrb;    // Máscara capturada de cpu_wstrb (estável durante stall AXI4)
    reg [31:0] resp_rdata;   // Dado de leitura registrado para entrega no ciclo ST_RESP
    reg        resp_error;   // Flag de erro registrada (1 apenas durante ST_RESP)

    // -------------------------------------------------------------------------
    // Constantes e Sinais Contínuos AXI4 (sempre beat único de 32 bits)
    // -------------------------------------------------------------------------
    assign m_axi_awaddr  = lat_addr;
    assign m_axi_awlen   = 8'd0;
    assign m_axi_awsize  = 3'b010;
    assign m_axi_awburst = 2'b01;

    assign m_axi_wdata   = lat_wdata;
    assign m_axi_wstrb   = lat_wstrb;
    assign m_axi_wlast   = 1'b1;

    assign m_axi_araddr  = lat_addr;
    assign m_axi_arlen   = 8'd0;
    assign m_axi_arsize  = 3'b010;
    assign m_axi_arburst = 2'b01;

    // Aceita resposta B apenas durante escrita ativa e R apenas durante leitura ativa
    assign m_axi_bready  = (state == ST_WR_WAIT);
    assign m_axi_rready  = (state == ST_RD_WAIT);

    // Resposta à CPU restrita ao ciclo único de ST_RESP (cpu_error=0 nos demais estados)
    assign cpu_ready     = (state == ST_RESP);
    assign cpu_rdata     = resp_rdata;
    assign cpu_error     = (state == ST_RESP) ? resp_error : 1'b0;

    // -------------------------------------------------------------------------
    // Máquina de Estados Síncrona Principal
    // -------------------------------------------------------------------------
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            state         <= ST_IDLE;
            lat_addr      <= 32'd0;
            lat_wdata     <= 32'd0;
            lat_wstrb     <= 4'd0;
            resp_rdata    <= 32'd0;
            resp_error    <= 1'b0;
            m_axi_awvalid <= 1'b0;
            m_axi_wvalid  <= 1'b0;
            m_axi_arvalid <= 1'b0;
        end else begin
            case (state)
                ST_IDLE: begin
                    resp_error <= 1'b0;
                    if (cpu_valid) begin
                        // Trava todos os atributos da requisição antes de expor ao AXI4
                        lat_addr  <= cpu_addr;
                        lat_wdata <= cpu_wdata;
                        lat_wstrb <= cpu_wstrb;
                        if (cpu_write) begin
                            m_axi_awvalid <= 1'b1;
                            m_axi_wvalid  <= 1'b1;
                            state         <= ST_WR_WAIT;
                        end else begin
                            m_axi_arvalid <= 1'b1;
                            state         <= ST_RD_WAIT;
                        end
                    end
                end

                ST_WR_WAIT: begin
                    // Baixa AWVALID e WVALID individualmente assim que cada canal aceita
                    if (m_axi_awvalid && m_axi_awready) begin
                        m_axi_awvalid <= 1'b0;
                    end
                    if (m_axi_wvalid && m_axi_wready) begin
                        m_axi_wvalid <= 1'b0;
                    end
                    // Conclui a escrita somente quando a resposta B é recebida
                    if (m_axi_bvalid && m_axi_bready) begin
                        m_axi_awvalid <= 1'b0;
                        m_axi_wvalid  <= 1'b0;
                        resp_rdata    <= 32'd0;
                        resp_error    <= (m_axi_bresp != AXI_RESP_OKAY);
                        state         <= ST_RESP;
                    end
                end

                ST_RD_WAIT: begin
                    // Baixa ARVALID assim que o endereço de leitura é aceito
                    if (m_axi_arvalid && m_axi_arready) begin
                        m_axi_arvalid <= 1'b0;
                    end
                    // Conclui a leitura quando o dado único R é recebido
                    if (m_axi_rvalid && m_axi_rready) begin
                        m_axi_arvalid <= 1'b0;
                        resp_rdata    <= m_axi_rdata;
                        resp_error    <= (m_axi_rresp != AXI_RESP_OKAY) || !m_axi_rlast;
                        state         <= ST_RESP;
                    end
                end

                ST_RESP: begin
                    // Pulso de 1 ciclo em cpu_ready consumido nesta borda; retorna limpo a IDLE
                    resp_error <= 1'b0;
                    state      <= ST_IDLE;
                end

                default: begin
                    state         <= ST_IDLE;
                    m_axi_awvalid <= 1'b0;
                    m_axi_wvalid  <= 1'b0;
                    m_axi_arvalid <= 1'b0;
                    resp_error    <= 1'b0;
                end
            endcase
        end
    end

endmodule
