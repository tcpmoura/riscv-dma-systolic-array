`timescale 1ns/1ps
module mux2 (
    input wire [31:0] a,
    input wire [31:0] b,
    input wire sel,
    output wire [31:0] result
);
    assign result = sel ? b : a;
endmodule
