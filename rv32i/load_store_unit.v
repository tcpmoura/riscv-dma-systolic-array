`timescale 1ns/1ps

module load_store_unit (
    input  wire [31:0] addr,
    input  wire [2:0]  funct3,
    input  wire        mem_read,
    input  wire        mem_write,
    input  wire [31:0] rs2_data,
    input  wire [31:0] mem_rdata,
    output wire [31:0] mem_addr,
    output reg  [31:0] mem_wdata,
    output reg  [3:0]  mem_wstrb,
    output reg  [31:0] load_data,
    output reg         mem_req,
    output reg         misaligned,
    output reg         invalid_access
);
    reg [31:0] shifted_data;
    reg [15:0] half_data;
    reg [7:0]  byte_data;

    assign mem_addr = {addr[31:2], 2'b00};

    always @(*) begin
        mem_wdata = 32'b0;
        mem_wstrb = 4'b0000;
        load_data = 32'b0;
        mem_req = 1'b0;
        misaligned = 1'b0;
        invalid_access = 1'b0;
        shifted_data = mem_rdata >> {addr[1:0], 3'b000};
        byte_data = shifted_data[7:0];
        half_data = shifted_data[15:0];

        if (mem_read && mem_write) begin
            invalid_access = 1'b1;
        end else if (mem_read) begin
            case (funct3)
                3'b000: load_data = {{24{byte_data[7]}}, byte_data}; // LB
                3'b100: load_data = {24'b0, byte_data};             // LBU
                3'b001: begin                                      // LH
                    if (addr[0]) misaligned = 1'b1;
                    else load_data = {{16{half_data[15]}}, half_data};
                end
                3'b101: begin                                      // LHU
                    if (addr[0]) misaligned = 1'b1;
                    else load_data = {16'b0, half_data};
                end
                3'b010: begin                                      // LW
                    if (addr[1:0] != 2'b00) misaligned = 1'b1;
                    else load_data = mem_rdata;
                end
                default: invalid_access = 1'b1;
            endcase
        end else if (mem_write) begin
            case (funct3)
                3'b000: begin                                      // SB
                    mem_wstrb = 4'b0001 << addr[1:0];
                    mem_wdata = {24'b0, rs2_data[7:0]} << {addr[1:0], 3'b000};
                end
                3'b001: begin                                      // SH
                    if (addr[0]) misaligned = 1'b1;
                    else begin
                        mem_wstrb = 4'b0011 << addr[1:0];
                        mem_wdata = {16'b0, rs2_data[15:0]} << {addr[1:0], 3'b000};
                    end
                end
                3'b010: begin                                      // SW
                    if (addr[1:0] != 2'b00) misaligned = 1'b1;
                    else begin
                        mem_wstrb = 4'b1111;
                        mem_wdata = rs2_data;
                    end
                end
                default: invalid_access = 1'b1;
            endcase
        end

        if ((mem_read || mem_write) && !misaligned && !invalid_access)
            mem_req = 1'b1;
        else begin
            mem_wstrb = 4'b0000;
            mem_wdata = 32'b0;
            load_data = 32'b0;
        end
    end
endmodule
