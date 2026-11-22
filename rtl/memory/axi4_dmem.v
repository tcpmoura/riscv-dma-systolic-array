// =============================================================================
// Projeto     : RISC-V RV32I com Controlador DMA e Coprocessador Matricial
// Etapa       : Entrega Parcial III — DMA, Barramento AXI4 e Memória Compartilhada
// Módulo      : axi4_dmem.v
// Professores : Ana Julia Fernandes de Oliveira Barros
//               Igor Dantas dos Santos Miranda
//               João Alberto Castelo Branco Oliveira
// Grupo 4     : João Vitor Carvalho Lima
//               Matheus Artur Macedo Bomfim
//               Nathalia Miranda
//               Tatiana Caetano Praxedes de Moura
// Descrição   : Memória de Dados Compartilhada (DMEM — 16 KB padrão = 4096 x 32b)
//               com interface escrava AXI4 para o subconjunto adotado no projeto
//               (palavras alinhadas de 32 bits, SIZE=3'b010, rajadas INCR,
//               máscara wstrb por byte e latência parametrizável >= 1).
//               Armazena dados gerais da CPU e as matrizes A, B e C (faixa
//               0x0000_0000 a 0x0000_3FFF). Bloqueia escritas quando o comando
//               AW é inválido e sinaliza SLVERR em erro de protocolo/WLAST.
// =============================================================================
`timescale 1ns / 1ps

module axi4_dmem #(
    parameter integer MEM_WORDS = 4096, // Quantidade de palavras de 32 bits (4096 = 16 KB)
    parameter integer LATENCY   = 1     // Ciclos de resposta da memória (>= 1; > 1 insere wait states)
) (
    input  wire        clk,             // Clock principal do sistema
    input  wire        rst_n,           // Reset assíncrono ativo em nível baixo

    // Canal AW (Write Address): recebe endereço inicial e parâmetros da rajada de escrita
    input  wire [31:0] s_axi_awaddr,
    input  wire [7:0]  s_axi_awlen,
    input  wire [2:0]  s_axi_awsize,
    input  wire [1:0]  s_axi_awburst,
    input  wire        s_axi_awvalid,
    output wire        s_axi_awready,

    // Canal W (Write Data): recebe palavras de 32 bits, máscara por byte e flag de último beat
    input  wire [31:0] s_axi_wdata,
    input  wire [3:0]  s_axi_wstrb,
    input  wire        s_axi_wlast,
    input  wire        s_axi_wvalid,
    output wire        s_axi_wready,

    // Canal B (Write Response): devolve status OKAY ou SLVERR ao final da escrita
    output reg  [1:0]  s_axi_bresp,
    output reg         s_axi_bvalid,
    input  wire        s_axi_bready,

    // Canal AR (Read Address): recebe endereço inicial e parâmetros da rajada de leitura
    input  wire [31:0] s_axi_araddr,
    input  wire [7:0]  s_axi_arlen,
    input  wire [2:0]  s_axi_arsize,
    input  wire [1:0]  s_axi_arburst,
    input  wire        s_axi_arvalid,
    output wire        s_axi_arready,

    // Canal R (Read Data): devolve as palavras lidas, status e indicador do último beat
    output reg  [31:0] s_axi_rdata,
    output reg  [1:0]  s_axi_rresp,
    output reg         s_axi_rlast,
    output reg         s_axi_rvalid,
    input  wire        s_axi_rready
);

    localparam integer ADDR_W = $clog2(MEM_WORDS); // Largura do índice de palavra (12 bits p/ 4096 palavras)

    // Códigos de resposta AXI4 suportados pelo escravo
    localparam [1:0] AXI_RESP_OKAY   = 2'b00; // Transação concluída sem erros
    localparam [1:0] AXI_RESP_SLVERR = 2'b10; // Erro no escravo (alinhamento, faixa, tipo ou WLAST)

    // Estados da FSM de escrita (AW -> W -> WAIT -> B)
    localparam [1:0] WR_IDLE = 2'd0; // Aguarda requisição de endereço de escrita (AW)
    localparam [1:0] WR_DATA = 2'd1; // Recebe os beats de dados no canal W até WLAST
    localparam [1:0] WR_WAIT = 2'd2; // Emula ciclos de latência (LATENCY > 1) antes de BVALID
    localparam [1:0] WR_RESP = 2'd3; // Mantém BVALID ativo até o handshake com BREADY

    // Estados da FSM de leitura (AR -> WAIT -> R)
    localparam [1:0] RD_IDLE = 2'd0; // Aguarda requisição de endereço de leitura (AR)
    localparam [1:0] RD_WAIT = 2'd1; // Emula ciclos de latência (LATENCY > 1) antes do 1º beat
    localparam [1:0] RD_DATA = 2'd2; // Entrega os beats no canal R até RLAST com RREADY

    // Banco físico de memória RAM (4096 palavras x 32 bits = 16 KB)
    reg [31:0] memory [0:MEM_WORDS-1];

    // Inicialização exclusiva do modelo de simulação (não sintetizável em ASIC:
    // em silício físico, a implementação e inicialização da SRAM pertencem ao fluxo de síntese)
    // synthesis translate_off
    integer init_i; // Índice auxiliar para zerar a memória no início da simulação
    initial begin
        for (init_i = 0; init_i < MEM_WORDS; init_i = init_i + 1) begin
            memory[init_i] = 32'h0000_0000;
        end
    end
    // synthesis translate_on

    // -------------------------------------------------------------------------
    // Validação combinacional de requisições AW e AR
    // Verifica alinhamento em 4 bytes, tamanho de 32 bits (SIZE=010), rajada
    // INCR (BURST=01) e se o primeiro e o último beat cabem dentro de MEM_WORDS.
    // -------------------------------------------------------------------------
    wire [30:0] aw_word_full = {1'b0, s_axi_awaddr[31:2]};   // Índice inicial da palavra em AW (31b evita overflow)
    wire [30:0] aw_end_word  = aw_word_full + {23'd0, s_axi_awlen}; // Índice da última palavra da rajada AW
    wire        aw_req_bad   = (s_axi_awaddr[1:0] != 2'b00) ||
                               (s_axi_awsize != 3'b010)     ||
                               (s_axi_awburst != 2'b01)     ||
                               (aw_word_full >= MEM_WORDS)  ||
                               (aw_end_word >= MEM_WORDS);   // Flag indicando comando AW fora do contrato

    wire [30:0] ar_word_full = {1'b0, s_axi_araddr[31:2]};   // Índice inicial da palavra em AR
    wire [30:0] ar_end_word  = ar_word_full + {23'd0, s_axi_arlen}; // Índice da última palavra da rajada AR
    wire        ar_req_bad   = (s_axi_araddr[1:0] != 2'b00) ||
                               (s_axi_arsize != 3'b010)     ||
                               (s_axi_arburst != 2'b01)     ||
                               (ar_word_full >= MEM_WORDS)  ||
                               (ar_end_word >= MEM_WORDS);   // Flag indicando comando AR fora do contrato

    // -------------------------------------------------------------------------
    // Registradores internos e controle da FSM de Escrita (AW -> W -> B)
    // -------------------------------------------------------------------------
    reg [1:0]        wr_state;    // Estado atual da máquina de escrita
    reg [ADDR_W-1:0] wr_word_idx; // Ponteiro de palavra atual sendo escrita na RAM
    reg [7:0]        wr_len;      // Comprimento registrado da rajada (AWLEN = beats - 1)
    reg [7:0]        wr_cnt;      // Contador de beats já recebidos no canal W
    reg              wr_err;      // Flag de erro acumulada (bloqueia escrita se AW inválido)
    reg [15:0]       wr_wait_cnt; // Contador decrescente para emular latência de escrita

    assign s_axi_awready = (wr_state == WR_IDLE); // Aceita novo AW apenas quando ocioso
    assign s_axi_wready  = (wr_state == WR_DATA); // Aceita beats W após registrar o AW

    // Bloco sequencial de escrita: registra o endereço AW, aplica wstrb por byte
    // a cada beat válido de W, verifica consistência de WLAST e devolve BRESP.
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            wr_state     <= WR_IDLE;
            wr_word_idx  <= {ADDR_W{1'b0}};
            wr_len       <= 8'd0;
            wr_cnt       <= 8'd0;
            wr_err       <= 1'b0;
            wr_wait_cnt  <= 16'd0;
            s_axi_bvalid <= 1'b0;
            s_axi_bresp  <= AXI_RESP_OKAY;
        end else begin
            case (wr_state)
                WR_IDLE: begin
                    s_axi_bvalid <= 1'b0;
                    if (s_axi_awvalid) begin
                        wr_word_idx <= s_axi_awaddr[2 +: ADDR_W];
                        wr_len      <= s_axi_awlen;
                        wr_cnt      <= 8'd0;
                        wr_err      <= aw_req_bad;
                        wr_state    <= WR_DATA;
                    end
                end

                WR_DATA: begin
                    if (s_axi_wvalid) begin
                        if (!wr_err) begin
                            if (s_axi_wstrb[0]) memory[wr_word_idx][7:0]   <= s_axi_wdata[7:0];
                            if (s_axi_wstrb[1]) memory[wr_word_idx][15:8]  <= s_axi_wdata[15:8];
                            if (s_axi_wstrb[2]) memory[wr_word_idx][23:16] <= s_axi_wdata[23:16];
                            if (s_axi_wstrb[3]) memory[wr_word_idx][31:24] <= s_axi_wdata[31:24];
                        end

                        // Verifica consistência de WLAST com AWLEN
                        if (s_axi_wlast && (wr_cnt != wr_len)) begin
                            wr_err <= 1'b1;
                        end else if (!s_axi_wlast && (wr_cnt == wr_len)) begin
                            wr_err <= 1'b1;
                        end

                        wr_word_idx <= wr_word_idx + {{(ADDR_W-1){1'b0}}, 1'b1};
                        wr_cnt      <= wr_cnt + 8'd1;

                        if (s_axi_wlast) begin
                            if (LATENCY <= 1) begin
                                s_axi_bvalid <= 1'b1;
                                s_axi_bresp  <= (wr_err || (wr_cnt != wr_len)) ? AXI_RESP_SLVERR : AXI_RESP_OKAY;
                                wr_state     <= WR_RESP;
                            end else begin
                                wr_wait_cnt  <= LATENCY[15:0] - 16'd1;
                                wr_state     <= WR_WAIT;
                            end
                        end
                    end
                end

                WR_WAIT: begin
                    if (wr_wait_cnt <= 16'd1) begin
                        s_axi_bvalid <= 1'b1;
                        s_axi_bresp  <= wr_err ? AXI_RESP_SLVERR : AXI_RESP_OKAY;
                        wr_state     <= WR_RESP;
                    end else begin
                        wr_wait_cnt <= wr_wait_cnt - 16'd1;
                    end
                end

                WR_RESP: begin
                    if (s_axi_bvalid && s_axi_bready) begin
                        s_axi_bvalid <= 1'b0;
                        wr_state     <= WR_IDLE;
                    end
                end

                default: wr_state <= WR_IDLE;
            endcase
        end
    end

    // -------------------------------------------------------------------------
    // Registradores internos e controle da FSM de Leitura (AR -> R)
    // -------------------------------------------------------------------------
    reg [1:0]        rd_state;    // Estado atual da máquina de leitura
    reg [ADDR_W-1:0] rd_word_idx; // Ponteiro de palavra atual sendo lida da RAM
    reg [7:0]        rd_len;      // Comprimento registrado da rajada (ARLEN = beats - 1)
    reg [7:0]        rd_cnt;      // Contador de beats já entregues no canal R
    reg              rd_err;      // Flag de erro (retorna 0 + SLVERR se AR inválido)
    reg [15:0]       rd_wait_cnt; // Contador decrescente para emular latência de leitura

    assign s_axi_arready = (rd_state == RD_IDLE); // Aceita novo AR apenas quando ocioso

    // Bloco sequencial de leitura: captura o endereço AR, aguarda LATENCY ciclos
    // e entrega sequencialmente cada palavra no canal R respeitando RREADY.
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            rd_state     <= RD_IDLE;
            rd_word_idx  <= {ADDR_W{1'b0}};
            rd_len       <= 8'd0;
            rd_cnt       <= 8'd0;
            rd_err       <= 1'b0;
            rd_wait_cnt  <= 16'd0;
            s_axi_rvalid <= 1'b0;
            s_axi_rdata  <= 32'd0;
            s_axi_rresp  <= AXI_RESP_OKAY;
            s_axi_rlast  <= 1'b0;
        end else begin
            case (rd_state)
                RD_IDLE: begin
                    s_axi_rvalid <= 1'b0;
                    s_axi_rlast  <= 1'b0;
                    if (s_axi_arvalid) begin
                        rd_word_idx <= s_axi_araddr[2 +: ADDR_W];
                        rd_len      <= s_axi_arlen;
                        rd_cnt      <= 8'd0;
                        rd_err      <= ar_req_bad;

                        if (LATENCY <= 1) begin
                            s_axi_rvalid <= 1'b1;
                            s_axi_rdata  <= ar_req_bad ? 32'd0 : memory[s_axi_araddr[2 +: ADDR_W]];
                            s_axi_rresp  <= ar_req_bad ? AXI_RESP_SLVERR : AXI_RESP_OKAY;
                            s_axi_rlast  <= (s_axi_arlen == 8'd0);
                            rd_state     <= RD_DATA;
                        end else begin
                            rd_wait_cnt  <= LATENCY[15:0] - 16'd1;
                            rd_state     <= RD_WAIT;
                        end
                    end
                end

                RD_WAIT: begin
                    if (rd_wait_cnt <= 16'd1) begin
                        s_axi_rvalid <= 1'b1;
                        s_axi_rdata  <= rd_err ? 32'd0 : memory[rd_word_idx];
                        s_axi_rresp  <= rd_err ? AXI_RESP_SLVERR : AXI_RESP_OKAY;
                        s_axi_rlast  <= (rd_cnt == rd_len);
                        rd_state     <= RD_DATA;
                    end else begin
                        rd_wait_cnt <= rd_wait_cnt - 16'd1;
                    end
                end

                RD_DATA: begin
                    if (s_axi_rvalid && s_axi_rready) begin
                        if (rd_cnt == rd_len) begin
                            s_axi_rvalid <= 1'b0;
                            s_axi_rlast  <= 1'b0;
                            rd_state     <= RD_IDLE;
                        end else begin
                            rd_cnt      <= rd_cnt + 8'd1;
                            rd_word_idx <= rd_word_idx + {{(ADDR_W-1){1'b0}}, 1'b1};
                            s_axi_rdata <= rd_err ? 32'd0 : memory[rd_word_idx + {{(ADDR_W-1){1'b0}}, 1'b1}];
                            s_axi_rresp <= rd_err ? AXI_RESP_SLVERR : AXI_RESP_OKAY;
                            s_axi_rlast <= ((rd_cnt + 8'd1) == rd_len);
                        end
                    end
                end

                default: rd_state <= RD_IDLE;
            endcase
        end
    end

endmodule
