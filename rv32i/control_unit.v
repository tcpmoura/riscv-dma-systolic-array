`timescale 1ns/1ps

module control_unit (
    input  wire [31:0] instruction,
    output reg         reg_write,
    output reg  [1:0]  alu_a_sel,    // 00=rs1, 01=PC, 10=zero
    output reg         alu_src,      // 0=rs2, 1=immediate
    output reg  [3:0]  alu_control,  // matches alu.v
    output reg         mem_read,
    output reg         mem_write,
    output reg  [1:0]  result_src,   // 00=ALU, 01=memory, 10=PC+4
    output reg  [1:0]  mem_size,     // 00=byte, 01=half, 10=word
    output reg         mem_unsigned, // for LB/LH versus LBU/LHU
    output reg         branch,
    output reg         jump,
    output reg         jalr,
    output reg         illegal_instr,
    output reg         ecall,
    output reg         ebreak,
    output reg         fence
);
    wire [6:0] opcode = instruction[6:0];
    wire [2:0] funct3 = instruction[14:12];
    wire [6:0] funct7 = instruction[31:25];

    // ALU control encoding used by alu.v:
    // ADD=0 SUB=1 AND=2 OR=3 XOR=4 SLT=5 SLTU=6 SLL=7 SRL=8 SRA=9
    always @(*) begin
        reg_write     = 1'b0;
        alu_a_sel     = 2'b00;
        alu_src       = 1'b0;
        alu_control   = 4'd0;
        mem_read      = 1'b0;
        mem_write     = 1'b0;
        result_src    = 2'b00;
        mem_size      = 2'b10;
        mem_unsigned  = 1'b0;
        branch        = 1'b0;
        jump          = 1'b0;
        jalr          = 1'b0;
        illegal_instr = 1'b1;
        ecall         = 1'b0;
        ebreak        = 1'b0;
        fence         = 1'b0;

        case (opcode)
            7'b0110011: begin // OP: register/register
                case (funct3)
                    3'b000: if (funct7 == 7'b0000000 || funct7 == 7'b0100000) begin
                        alu_control = (funct7 == 7'b0100000) ? 4'd1 : 4'd0;
                        illegal_instr = 1'b0;
                    end
                    3'b001: if (funct7 == 7'b0000000) begin alu_control=4'd7; illegal_instr=0; end
                    3'b010: if (funct7 == 7'b0000000) begin alu_control=4'd5; illegal_instr=0; end
                    3'b011: if (funct7 == 7'b0000000) begin alu_control=4'd6; illegal_instr=0; end
                    3'b100: if (funct7 == 7'b0000000) begin alu_control=4'd4; illegal_instr=0; end
                    3'b101: if (funct7 == 7'b0000000 || funct7 == 7'b0100000) begin
                        alu_control = (funct7 == 7'b0100000) ? 4'd9 : 4'd8;
                        illegal_instr=0;
                    end
                    3'b110: if (funct7 == 7'b0000000) begin alu_control=4'd3; illegal_instr=0; end
                    3'b111: if (funct7 == 7'b0000000) begin alu_control=4'd2; illegal_instr=0; end
                endcase
                if (!illegal_instr) reg_write=1'b1;
            end
            7'b0010011: begin // OP-IMM
                alu_src = 1'b1;
                case (funct3)
                    3'b000: begin alu_control=4'd0; illegal_instr=0; end // ADDI
                    3'b010: begin alu_control=4'd5; illegal_instr=0; end // SLTI
                    3'b011: begin alu_control=4'd6; illegal_instr=0; end // SLTIU
                    3'b100: begin alu_control=4'd4; illegal_instr=0; end // XORI
                    3'b110: begin alu_control=4'd3; illegal_instr=0; end // ORI
                    3'b111: begin alu_control=4'd2; illegal_instr=0; end // ANDI
                    3'b001: if (funct7 == 7'b0000000) begin alu_control=4'd7; illegal_instr=0; end // SLLI
                    3'b101: if (funct7 == 7'b0000000 || funct7 == 7'b0100000) begin
                        alu_control=(funct7 == 7'b0100000) ? 4'd9 : 4'd8;
                        illegal_instr=0;
                    end
                endcase
                if (!illegal_instr) reg_write=1'b1;
            end
            7'b0000011: begin // LOAD
                alu_src=1'b1;
                case (funct3)
                    3'b000: begin mem_size=0; illegal_instr=0; end // LB
                    3'b001: begin mem_size=1; illegal_instr=0; end // LH
                    3'b010: begin mem_size=2; illegal_instr=0; end // LW
                    3'b100: begin mem_size=0; mem_unsigned=1; illegal_instr=0; end // LBU
                    3'b101: begin mem_size=1; mem_unsigned=1; illegal_instr=0; end // LHU
                endcase
                if (!illegal_instr) begin reg_write=1; mem_read=1; result_src=2'b01; end
            end
            7'b0100011: begin // STORE
                alu_src=1'b1;
                case (funct3)
                    3'b000: begin mem_size=0; illegal_instr=0; end // SB
                    3'b001: begin mem_size=1; illegal_instr=0; end // SH
                    3'b010: begin mem_size=2; illegal_instr=0; end // SW
                endcase
                if (!illegal_instr) mem_write=1;
            end
            7'b1100011: begin // BRANCH (comparison handled by branch_unit)
                case (funct3)
                    3'b000, 3'b001, 3'b100, 3'b101, 3'b110, 3'b111:
                        begin branch=1'b1; illegal_instr=0; end
                endcase
            end
            7'b1101111: begin // JAL
                jump=1; reg_write=1; result_src=2'b10; illegal_instr=0;
            end
            7'b1100111: begin // JALR (funct3 must be 000)
                if (funct3 == 3'b000) begin
                    jump=1; jalr=1; reg_write=1; result_src=2'b10;
                    alu_src=1; illegal_instr=0;
                end
            end
            7'b0110111: begin // LUI: zero + U-immediate
                alu_a_sel=2'b10; alu_src=1; reg_write=1; illegal_instr=0;
            end
            7'b0010111: begin // AUIPC: PC + U-immediate
                alu_a_sel=2'b01; alu_src=1; reg_write=1; illegal_instr=0;
            end
            7'b0001111: begin // FENCE only (not FENCE.I / Zifencei)
                if (funct3 == 3'b000) begin fence=1; illegal_instr=0; end
            end
            7'b1110011: begin // Only ECALL / EBREAK; CSR requires Zicsr
                if (instruction == 32'h00000073) begin ecall=1; illegal_instr=0; end
                else if (instruction == 32'h00100073) begin ebreak=1; illegal_instr=0; end
            end
            default: begin end
        endcase
    end
endmodule
