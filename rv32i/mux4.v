`timescale 1ns/1ps
module mux4 (
    input wire [31:0] a,
    input wire [31:0] b,
    input wire [31:0] c,
    input wire [31:0] d,
    input wire [1:0] sel,
    output reg [31:0] result
);
    always @(*) begin
        case (sel)
            2'b00: result = a;
            2'b01: result = b;
            2'b10: result = c;
            2'b11: result = d;
            default: result = a;
        endcase
    end
endmodule
