`timescale 1ns/1ps
module adder (
    input wire [31:0] a,
    input wire [31:0] b,
    output wire [31:0] result
);
    assign result = a + b;
endmodule
