`timescale 1ns/1ps

// Memoria de dados RV32I: uma requisicao por vez.
// Enderecos e dados sao palavras de 32 bits; wstrb seleciona bytes.
// Protocolo: req_valid mantido ate req_ready=1; transferencia ocorre
// na borda de subida com req_valid && req_ready.
// LATENCY=1: resposta disponivel durante o ciclo seguinte a captura.
module data_memory #(
    parameter MEM_DEPTH = 1024,
    parameter LATENCY   = 1
)(
    input  wire        clk,
    input  wire        rst_n,
    input  wire        req_valid,
    output wire        req_ready,
    input  wire        req_write,
    input  wire [31:0] req_addr,
    input  wire [31:0] req_wdata,
    input  wire [3:0]  req_wstrb,
    output reg  [31:0] resp_rdata,
    output wire        resp_error
);
    reg [31:0] memory [0:MEM_DEPTH-1];
    reg        busy;
    reg        response_valid;
    reg        pending_write;
    reg [31:0] pending_addr;
    reg [31:0] pending_wdata;
    reg [3:0]  pending_wstrb;
    reg [31:0] pending_rdata;
    reg        pending_error;
    integer    count;
    integer    i;

    wire [29:0] word_index = req_addr[31:2];
    wire addr_bad = (req_addr[1:0] != 2'b00) || (word_index >= MEM_DEPTH);
    wire [29:0] pending_index = pending_addr[31:2];

    // Resposta ready pertence somente a uma transacao pendente.
    assign req_ready  = response_valid;
    assign resp_error = response_valid && pending_error;

    initial begin
        if (LATENCY < 1) begin
            $display("ERROR: LATENCY must be >= 1");
            $finish;
        end
        for (i = 0; i < MEM_DEPTH; i = i + 1)
            memory[i] = 32'b0; // somente inicializacao de simulacao
    end

    // pending_rdata e a amostra de leitura na captura da requisicao.
    // Evita acesso assincrono aos dados durante o stall.
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            busy           <= 1'b0;
            response_valid <= 1'b0;
            pending_write  <= 1'b0;
            pending_addr   <= 32'b0;
            pending_wdata  <= 32'b0;
            pending_wstrb  <= 4'b0;
            pending_rdata  <= 32'b0;
            pending_error  <= 1'b0;
            resp_rdata     <= 32'b0;
            count          <= 0;
        end else if (response_valid) begin
            // A CPU deve manter valid ate esta borda (handshake).
            if (req_valid) begin
                if (pending_write && !pending_error) begin
                    if (pending_wstrb[0]) memory[pending_index][7:0]   <= pending_wdata[7:0];
                    if (pending_wstrb[1]) memory[pending_index][15:8]  <= pending_wdata[15:8];
                    if (pending_wstrb[2]) memory[pending_index][23:16] <= pending_wdata[23:16];
                    if (pending_wstrb[3]) memory[pending_index][31:24] <= pending_wdata[31:24];
                end
                response_valid <= 1'b0;
                busy <= 1'b0;
            end
        end else if (busy) begin
            if (count == 1) begin
                response_valid <= 1'b1;
                count <= 0;
                resp_rdata <= pending_rdata;
            end else begin
                count <= count - 1;
            end
        end else if (req_valid) begin
            busy          <= 1'b1;
            pending_write <= req_write;
            pending_addr  <= req_addr;
            pending_wdata <= req_wdata;
            pending_wstrb <= req_wstrb;
            pending_error <= addr_bad;
            pending_rdata <= addr_bad ? 32'b0 : memory[word_index];
            count         <= LATENCY;
        end
    end
endmodule
