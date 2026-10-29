`timescale 1ns/1ps
module rv32i_core #(
    parameter IMEM_DEPTH = 256,
    parameter IMEM_FILE = "program.hex"
)(
    input wire clk, input wire rst_n,
    output wire dmem_valid, input wire dmem_ready,
    output wire dmem_write, output wire [31:0] dmem_addr,
    output wire [31:0] dmem_wdata, output wire [3:0] dmem_wstrb,
    input wire [31:0] dmem_rdata, input wire dmem_error,
    output wire [31:0] debug_pc,
    output reg halted, output reg trap,
    output wire retire
);
    wire [31:0] pc_current, instruction, imm;
    wire instr_valid;
    wire [4:0] rs1 = instruction[19:15];
    wire [4:0] rs2 = instruction[24:20];
    wire [4:0] rd  = instruction[11:7];
    wire [2:0] funct3 = instruction[14:12];
    wire [31:0] rs1_data,rs2_data,alu_a,alu_b,alu_result;
    wire alu_zero;
    wire reg_write,alu_src,mem_read,mem_write,mem_unsigned;
    wire branch,jump,jalr,illegal_instr,ecall,ebreak,fence;
    wire [1:0] alu_a_sel,result_src,mem_size;
    wire [3:0] alu_control;
    wire branch_taken;
    wire [31:0] lsu_addr,lsu_wdata,load_data;
    wire [3:0] lsu_wstrb;
    wire lsu_req,misaligned,invalid_access;
    wire instr_exception = !instr_valid || illegal_instr || ecall || ebreak;
    wire mem_exception = (mem_read || mem_write) && (misaligned || invalid_access);
    wire needs_mem = mem_read || mem_write;
    wire memory_complete = dmem_valid && dmem_ready;
    wire fault = instr_exception || mem_exception;
    wire complete = !halted && !trap && !fault && (!needs_mem || memory_complete) && !dmem_error;
    wire fail_now = !halted && !trap && (fault || (memory_complete && dmem_error));
    // Somadores de PC, branch e JALR (com mascaramento do bit 0).
    wire [31:0] pc4, pc_branch, jalr_sum, jalr_target;
    wire [31:0] pc_target, pc_next;
    wire take_branch;
    wire [31:0] writeback;
    wire rf_write = complete && reg_write;

    adder add_pc4_inst(.a(pc_current),.b(32'd4),.result(pc4));
    adder add_branch_inst(.a(pc_current),.b(imm),.result(pc_branch));
    adder add_jalr_inst(.a(rs1_data),.b(imm),.result(jalr_sum));
    assign jalr_target = {jalr_sum[31:1],1'b0};

    mux2 mux_jalr_target_inst(.a(pc_branch),.b(jalr_target),
                              .sel(jalr),.result(pc_target));
    assign take_branch = branch && branch_taken;
    mux2 mux_pc_inst(.a(pc4),.b(pc_target),
                     .sel(jump || take_branch),.result(pc_next));

    // Operando A: rs1 / PC / zero / rs1 (reservado).
    mux4 mux_alu_a_inst(.a(rs1_data),.b(pc_current),.c(32'b0),
                        .d(rs1_data),.sel(alu_a_sel),.result(alu_a));
    mux2 mux_alu_b_inst(.a(rs2_data),.b(imm),
                        .sel(alu_src),.result(alu_b));

    // Write-back: ALU / Load / PC+4 / ALU (reservado).
    mux4 mux_wb_inst(.a(alu_result),.b(load_data),.c(pc4),
                     .d(alu_result),.sel(result_src),.result(writeback));

    assign debug_pc = pc_current;
    assign retire = complete;
    // Valid so fica alto durante uma instrucao de memoria valida.
    assign dmem_valid = !halted && !trap && !fault && lsu_req;
    assign dmem_write = mem_write;
    assign dmem_addr = lsu_addr;
    assign dmem_wdata = lsu_wdata;
    assign dmem_wstrb = lsu_wstrb;

    pc pc_inst(.clk(clk),.rst_n(rst_n),.pc_enable(complete),
               .pc_next(pc_next),.pc_out(pc_current));
    instruction_memory #(.MEM_DEPTH(IMEM_DEPTH),.MEM_FILE(IMEM_FILE))
      imem_inst(.pc_addr(pc_current),.instruction(instruction),.instr_valid(instr_valid));
    register_file rf_inst(.clk(clk),.rst_n(rst_n),.stall(!complete),
                          .reg_write(rf_write),.rs1(rs1),.rs2(rs2),
                          .rd(rd),.write_data(writeback),
                          .read_data1(rs1_data),.read_data2(rs2_data));
    immediate_generator imm_inst(.instruction(instruction),.immediate(imm));
    control_unit ctrl_inst(.instruction(instruction),.reg_write(reg_write),
         .alu_a_sel(alu_a_sel),.alu_src(alu_src),.alu_control(alu_control),
         .mem_read(mem_read),.mem_write(mem_write),.result_src(result_src),
         .mem_size(mem_size),.mem_unsigned(mem_unsigned),.branch(branch),
         .jump(jump),.jalr(jalr),.illegal_instr(illegal_instr),
         .ecall(ecall),.ebreak(ebreak),.fence(fence));
    alu alu_inst(.a(alu_a),.b(alu_b),.alu_control(alu_control),
                 .result(alu_result),.zero(alu_zero));
    branch_unit branch_inst(.a(rs1_data),.b(rs2_data),.funct3(funct3),
                            .branch(branch),.branch_taken(branch_taken));
    load_store_unit lsu_inst(.addr(alu_result),.funct3(funct3),
      .mem_read(mem_read),.mem_write(mem_write),.rs2_data(rs2_data),
      .mem_rdata(dmem_rdata),.mem_addr(lsu_addr),.mem_wdata(lsu_wdata),
      .mem_wstrb(lsu_wstrb),.load_data(load_data),.mem_req(lsu_req),
      .misaligned(misaligned),.invalid_access(invalid_access));

    // Política didática: EBREAK encerra; ECALL/ilegal/erro geram trap.
    // Ainda nao existem mtvec/mepc/mcause nem interrupcoes.
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin halted <= 1'b0; trap <= 1'b0; end
        else if (!halted && !trap) begin
            if (ebreak && instr_valid) halted <= 1'b1;
            else if (fail_now) trap <= 1'b1;
        end
    end
endmodule
