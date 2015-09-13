/* ****************************************************************************
  This Source Code Form is subject to the terms of the
  Open Hardware Description License, v. 1.0. If a copy
  of the OHDL was not distributed with this file, You
  can obtain one at http://juliusbaxter.net/ohdl/ohdl.txt

  Description: mor1kx decode unit for MAROCCHINO pipeline

  Derived from mor1kx_decode & mor1kx_decode_execute_cappuccino

  Outputs:
   - ALU operation
   - indication of other type of op - LSU/SPR
   - immediates
   - register file addresses
   - exception decodes:  illegal, system call

  Copyright (C) 2012 Julius Baxter <juliusbaxter@gmail.com>
  Copyright (C) 2013 Stefan Kristiansson <stefan.kristiansson@saunalahti.fi>
  Copyright (C) 2015 Andrey Bacherov <avbacherov@opencores.org>

***************************************************************************** */

`include "mor1kx-defines.v"

module mor1kx_decode_marocchino
#(
  parameter OPTION_OPERAND_WIDTH = 32,
  parameter OPTION_RESET_PC      = {{(OPTION_OPERAND_WIDTH-13){1'b0}},
                                    `OR1K_RESET_VECTOR,8'd0},
  parameter OPTION_RF_ADDR_WIDTH =  5,

  parameter FEATURE_SYSCALL      = "ENABLED",
  parameter FEATURE_TRAP         = "ENABLED",

  parameter FEATURE_PSYNC        = "NONE",
  parameter FEATURE_CSYNC        = "NONE",

  parameter FEATURE_FPU          = "NONE" // ENABLED|NONE
)
(
  input                                 clk,
  input                                 rst,

  // pipeline control signal in
  input                                 padv_decode_i,
  input                                 pipeline_flush_i,

  // INSN
  input          [`OR1K_INSN_WIDTH-1:0] dcod_insn_i,
  input                                 dcod_op_branch_i,
  input                                 dcod_delay_slot_i,
  input                                 dcod_insn_valid_i,

  // PC
  input      [OPTION_OPERAND_WIDTH-1:0] pc_decode_i,
  output reg [OPTION_OPERAND_WIDTH-1:0] pc_exec_o,

  // IMM
  output reg [OPTION_OPERAND_WIDTH-1:0] exec_immediate_o,
  output reg                            exec_immediate_sel_o,

  // GPR addresses
  output     [OPTION_RF_ADDR_WIDTH-1:0] dcod_rfa_adr_o, // to RF
  output     [OPTION_RF_ADDR_WIDTH-1:0] dcod_rfb_adr_o, // to RF
  output reg [OPTION_RF_ADDR_WIDTH-1:0] exec_rfd_adr_o,
  output reg                            exec_rf_wb_o,

  // flag & branches
  output                                dcod_op_bf_o, // to BRANCH PREDICTION
  output                                dcod_op_bnf_o, // to BRANCH PREDICTION
  output                          [9:0] dcod_immjbr_upper_o, // to BRANCH PREDICTION : Upper 10 bits of immediate for jumps and branches
  output                                dcod_take_branch_o, // to FETCH
  output reg                            exec_op_setflag_o,
  output reg                            exec_op_brcond_o,
  output reg                            exec_op_branch_o,
  output reg                            exec_delay_slot_o,
  output reg                            exec_op_jal_o,
  output reg [OPTION_OPERAND_WIDTH-1:0] exec_jal_result_o,
  input      [OPTION_OPERAND_WIDTH-1:0] dcod_rfb_i,
  output                                dcod_branch_o,
  output     [OPTION_OPERAND_WIDTH-1:0] dcod_branch_target_o,
  // Branch prediction signals
  input                                 predicted_flag_i,
  output reg                            exec_predicted_flag_o,
  // The target pc that should be used in case of branch misprediction
  output reg [OPTION_OPERAND_WIDTH-1:0] exec_mispredict_target_o,

  // LSU related
  output reg                            exec_op_lsu_load_o,
  output reg                            exec_op_lsu_store_o,
  output reg                            exec_op_lsu_atomic_o,
  output reg                      [1:0] exec_lsu_length_o,
  output reg                            exec_lsu_zext_o,
  input                                 take_op_lsu_i, // LSU->DECODE feedback (drop LSU related commands)

  // Sync operations
  output reg                            exec_op_msync_o,

  // Adder related
  output reg                            exec_op_add_o,
  output reg                            exec_adder_do_sub_o,
  output reg                            exec_adder_do_carry_o,
  // Various 1-clock related
  output reg                            exec_op_shift_o,
  output reg                            exec_op_ffl1_o,
  output reg                            exec_op_movhi_o,
  output reg                            exec_op_cmov_o,
  // Multiplier related
  output reg                            exec_op_mul_o,
  input                                 take_op_mul_i,
  // Divider related
  output reg                            exec_op_div_o,
  output reg                            exec_op_div_signed_o,
  output reg                            exec_op_div_unsigned_o,
  input                                 take_op_div_i,
  // FPU related
  output reg    [`OR1K_FPUOP_WIDTH-1:0] exec_op_fp32_arith_o,
  input                                 take_op_fp32_arith_i, // FP32->DECODE feedback (drop FP32 arithmetic related command)
  output reg    [`OR1K_FPUOP_WIDTH-1:0] exec_op_fp32_cmp_o,

  // ALU related opc
  output reg  [`OR1K_ALU_OPC_WIDTH-1:0] exec_opc_alu_o,
  output reg  [`OR1K_ALU_OPC_WIDTH-1:0] exec_opc_alu_secondary_o,

  // MTSPR / MFSPR
  output reg                            exec_op_mfspr_o,
  output reg                            exec_op_mtspr_o,
  input                                 take_op_mXspr_i, // CTRL->DECODE feedback (drop M(F|T)SPR command

  // 1-clock instruction flag to force EXECUTE valid
  output reg                            exec_insn_1clk_o,

  // Hazards resolving
  output                                dcod_bubble_o, // to CTRL
  output reg                            exec_bubble_o,

  // Exception flags
  //   income FETCH exception flags
  input                                 dcod_except_ibus_err_i,
  input                                 dcod_except_itlb_miss_i,
  input                                 dcod_except_ipagefault_i,
  //   outcome FETCH exception flags
  output reg                            exec_except_ibus_err_o,
  output reg                            exec_except_itlb_miss_o,
  output reg                            exec_except_ipagefault_o,
  output reg                            exec_except_illegal_o,
  output reg                            exec_except_ibus_align_o,
  //   outcome DECODE exception flags
  output reg                            exec_except_syscall_o,
  output reg                            exec_except_trap_o,
  //   enable exceptions processing
  output reg                            exec_excepts_en_o,

  // RFE
  output reg                            exec_op_rfe_o
);

  wire [OPTION_OPERAND_WIDTH-1:0] imm_sext;
  wire                            imm_sext_sel;
  wire [OPTION_OPERAND_WIDTH-1:0] imm_zext;
  wire                            imm_zext_sel;
  wire [OPTION_OPERAND_WIDTH-1:0] imm_high;
  wire                            imm_high_sel;
  wire [OPTION_OPERAND_WIDTH-1:0] dcod_immediate;
  wire                            dcod_immediate_sel;

  wire dcod_op_jal; // l.jal | l.jalr : jump and link
  wire dcod_op_jr; // l.jr  | l.jalr : jump to register
  wire dcod_op_jb_imm; // l.j  | l.jal  | l.bnf | l.bf : jumps or contitional branches to immediate

  // Insn opcode
  wire [`OR1K_OPCODE_WIDTH-1:0]  opc_insn = dcod_insn_i[`OR1K_OPCODE_SELECT];
  wire [`OR1K_ALU_OPC_WIDTH-1:0] opc_alu  = dcod_insn_i[`OR1K_ALU_OPC_SELECT];

  wire dcod_op_alu = (opc_insn == `OR1K_OPCODE_ALU);

  // load opcodes are 6'b10_0000 to 6'b10_0110, 0 to 6, so check for 7 and up
  wire dcod_op_lsu_load = ((dcod_insn_i[31:30] == 2'b10) &
                           ~(&dcod_insn_i[28:26]) & ~dcod_insn_i[29]) |
                          (opc_insn == `OR1K_OPCODE_LWA);

  // Detect when instruction is store
  wire dcod_op_lsu_store = (opc_insn == `OR1K_OPCODE_SW) | (opc_insn == `OR1K_OPCODE_SB) |
                           (opc_insn == `OR1K_OPCODE_SH) | (opc_insn == `OR1K_OPCODE_SWA);

  wire dcod_op_lsu_atomic = (opc_insn == `OR1K_OPCODE_LWA) | (opc_insn == `OR1K_OPCODE_SWA);

  // Decode length of load/store operation
  reg [1:0] dcod_lsu_length;
  always @(*)
    case (opc_insn)
      // byte
      `OR1K_OPCODE_SB,
      `OR1K_OPCODE_LBZ,
      `OR1K_OPCODE_LBS: dcod_lsu_length = 2'b00;
      // half word
      `OR1K_OPCODE_SH,
      `OR1K_OPCODE_LHZ,
      `OR1K_OPCODE_LHS: dcod_lsu_length = 2'b01;
      // word
      `OR1K_OPCODE_SW,
      `OR1K_OPCODE_SWA,
      `OR1K_OPCODE_LWZ,
      `OR1K_OPCODE_LWS,
      `OR1K_OPCODE_LWA: dcod_lsu_length = 2'b10;
      // default
      default:          dcod_lsu_length = 2'b10;
  endcase

  wire dcod_lsu_zext = opc_insn[0];

  wire dcod_op_msync = (opc_insn == `OR1K_OPCODE_SYSTRAPSYNC) &
                       (dcod_insn_i[`OR1K_SYSTRAPSYNC_OPC_SELECT] ==
                        `OR1K_SYSTRAPSYNC_OPC_MSYNC);

  wire dcod_op_mtspr = (opc_insn == `OR1K_OPCODE_MTSPR);
  wire dcod_op_mfspr = (opc_insn == `OR1K_OPCODE_MFSPR);

  // Detect when setflag instruction
  wire dcod_op_setflag = (opc_insn == `OR1K_OPCODE_SF) |
                         (opc_insn == `OR1K_OPCODE_SFIMM);


  // --- adder ---
  wire dcod_op_add = (dcod_op_alu &
                      ((opc_alu == `OR1K_ALU_OPC_ADDC) |
                       (opc_alu == `OR1K_ALU_OPC_ADD) |
                       (opc_alu == `OR1K_ALU_OPC_SUB))) |
                     (opc_insn == `OR1K_OPCODE_ADDIC) |
                     (opc_insn == `OR1K_OPCODE_ADDI);
  // Adder control logic
  // Subtract when comparing to check if equal
  wire dcod_adder_do_sub = (dcod_op_alu & (opc_alu == `OR1K_ALU_OPC_SUB)) |
                           dcod_op_setflag;
  // Generate carry-in select
  wire dcod_adder_do_carry = (dcod_op_alu & (opc_alu == `OR1K_ALU_OPC_ADDC)) |
                             (opc_insn == `OR1K_OPCODE_ADDIC);


  // --- multiplier ---
  wire dcod_op_mul_signed = (dcod_op_alu & (opc_alu == `OR1K_ALU_OPC_MUL)) |
                            (opc_insn == `OR1K_OPCODE_MULI);

  wire dcod_op_mul_unsigned = dcod_op_alu & (opc_alu == `OR1K_ALU_OPC_MULU);

  wire dcod_op_mul = dcod_op_mul_signed | dcod_op_mul_unsigned;


  // --- divider ---
  wire dcod_op_div_signed = dcod_op_alu & (opc_alu == `OR1K_ALU_OPC_DIV);

  wire dcod_op_div_unsigned = dcod_op_alu & (opc_alu == `OR1K_ALU_OPC_DIVU);

  wire dcod_op_div = dcod_op_div_signed | dcod_op_div_unsigned;


  // --- shifter / ffl1 / movhi / cmov ---
  wire dcod_op_shift = (dcod_op_alu & (opc_alu  == `OR1K_ALU_OPC_SHRT)) |
                       (opc_insn == `OR1K_OPCODE_SHRTI);

  wire dcod_op_ffl1  = dcod_op_alu & (opc_alu  == `OR1K_ALU_OPC_FFL1);

  wire dcod_op_movhi = (opc_insn == `OR1K_OPCODE_MOVHI);

  wire dcod_op_cmov  = (dcod_op_alu & (opc_alu == `OR1K_ALU_OPC_CMOV));


  // Which instructions cause writeback?
  wire dcod_rf_wb =
    (opc_insn == `OR1K_OPCODE_JAL) | (opc_insn == `OR1K_OPCODE_JALR) |
    (opc_insn == `OR1K_OPCODE_MOVHI) | (opc_insn == `OR1K_OPCODE_LWA) |
    // All '10????' opcodes excliding l.sfxxi
    ((dcod_insn_i[31:30] == 2'b10) & ~(opc_insn == `OR1K_OPCODE_SFIMM)) |
    // All '11????' opcodes excluding: l.sfxx, l.mtspr and lf.sfxx
    ((dcod_insn_i[31:30] == 2'b11) &
     ~((opc_insn == `OR1K_OPCODE_SF) | dcod_op_mtspr | dcod_op_lsu_store) &
     ~((FEATURE_FPU != "NONE") & (opc_insn == `OR1K_OPCODE_FPU) & dcod_insn_i[3]));


  // Register file addresses
  assign dcod_rfa_adr_o = dcod_insn_i[`OR1K_RA_SELECT];
  assign dcod_rfb_adr_o = dcod_insn_i[`OR1K_RB_SELECT];

  wire [OPTION_RF_ADDR_WIDTH-1:0] dcod_rfd_adr;
  assign dcod_rfd_adr = dcod_op_jal ? 9 : dcod_insn_i[`OR1K_RD_SELECT];


  // Upper 10 bits for jump/branch instructions
  assign dcod_immjbr_upper_o = dcod_insn_i[25:16];

  // Immediate in l.mtspr is broken up, reassemble
  wire [`OR1K_IMM_WIDTH-1:0] dcod_imm16;
  assign dcod_imm16 = (dcod_op_mtspr | dcod_op_lsu_store) ?
                        {dcod_insn_i[25:21],dcod_insn_i[10:0]} :
                        dcod_insn_i[`OR1K_IMM_SELECT];

  assign imm_sext     = {{16{dcod_imm16[15]}}, dcod_imm16[15:0]};
  assign imm_sext_sel = ((opc_insn[5:4] == 2'b10) &
                          ~(opc_insn == `OR1K_OPCODE_ORI) &
                          ~(opc_insn == `OR1K_OPCODE_ANDI)) |
                        (opc_insn == `OR1K_OPCODE_SWA) |
                        (opc_insn == `OR1K_OPCODE_LWA) |
                        (opc_insn == `OR1K_OPCODE_SW) |
                        (opc_insn == `OR1K_OPCODE_SH) |
                        (opc_insn == `OR1K_OPCODE_SB);

  assign imm_zext     = {{16{1'b0}}, dcod_imm16[15:0]};
  assign imm_zext_sel = ((opc_insn[5:4] == 2'b10) &
                          ((opc_insn == `OR1K_OPCODE_ORI) |
                           (opc_insn == `OR1K_OPCODE_ANDI))) |
                        (opc_insn == `OR1K_OPCODE_MTSPR);

  assign imm_high     = {dcod_imm16, 16'd0};
  assign imm_high_sel = dcod_op_movhi;

  assign dcod_immediate     = imm_sext_sel ? imm_sext :
                              imm_zext_sel ? imm_zext : imm_high;
  assign dcod_immediate_sel = imm_sext_sel | imm_zext_sel | imm_high_sel;


  // ALU opcode
  wire [`OR1K_ALU_OPC_WIDTH-1:0] dcod_opc_alu =
    (opc_insn == `OR1K_OPCODE_ORI)  ? `OR1K_ALU_OPC_OR  :
    (opc_insn == `OR1K_OPCODE_ANDI) ? `OR1K_ALU_OPC_AND :
    (opc_insn == `OR1K_OPCODE_XORI) ? `OR1K_ALU_OPC_XOR :
      ({`OR1K_ALU_OPC_WIDTH{dcod_op_alu}} & opc_alu);

  wire [`OR1K_ALU_OPC_WIDTH-1:0] dcod_opc_alu_secondary;
  assign dcod_opc_alu_secondary = dcod_op_setflag ?
                                    dcod_insn_i[`OR1K_COMP_OPC_SELECT]:
                                    {1'b0,dcod_insn_i[`OR1K_ALU_OPC_SECONDARY_SELECT]};


  wire dcod_op_rfe = (opc_insn == `OR1K_OPCODE_RFE);

  wire dcod_except_syscall = (FEATURE_SYSCALL != "NONE") &
                               (opc_insn == `OR1K_OPCODE_SYSTRAPSYNC) &
                               (dcod_insn_i[`OR1K_SYSTRAPSYNC_OPC_SELECT] ==
                                `OR1K_SYSTRAPSYNC_OPC_SYSCALL);

  wire dcod_except_trap = (FEATURE_TRAP != "NONE") &
                            (opc_insn == `OR1K_OPCODE_SYSTRAPSYNC) &
                            (dcod_insn_i[`OR1K_SYSTRAPSYNC_OPC_SELECT] ==
                             `OR1K_SYSTRAPSYNC_OPC_TRAP);

  // Illegal instruction decode
  reg illegal_insn_r;
  // Instruction executed during 1 clock
  reg insn_one_clock_r;
  // ---
  always @* begin
    case (opc_insn)
      `OR1K_OPCODE_J,
      `OR1K_OPCODE_JAL,
      `OR1K_OPCODE_JR,
      `OR1K_OPCODE_JALR,
      `OR1K_OPCODE_BNF,
      `OR1K_OPCODE_BF,
      `OR1K_OPCODE_MOVHI,
      `OR1K_OPCODE_RFE,
      `OR1K_OPCODE_ADDI,
      `OR1K_OPCODE_ADDIC,
      `OR1K_OPCODE_ANDI,
      `OR1K_OPCODE_ORI,
      `OR1K_OPCODE_XORI,
      `OR1K_OPCODE_SF,
      `OR1K_OPCODE_SFIMM,
      `OR1K_OPCODE_NOP:
         begin
          illegal_insn_r   = 1'b0;
          insn_one_clock_r = 1'b1;
         end

      `OR1K_OPCODE_MFSPR,
      `OR1K_OPCODE_MTSPR,
      `OR1K_OPCODE_LWZ,
      `OR1K_OPCODE_LWS,
      `OR1K_OPCODE_LBZ,
      `OR1K_OPCODE_LBS,
      `OR1K_OPCODE_LHZ,
      `OR1K_OPCODE_LHS,
      `OR1K_OPCODE_SW,
      `OR1K_OPCODE_SB,
      `OR1K_OPCODE_SH,
      `OR1K_OPCODE_SWA,
      `OR1K_OPCODE_LWA:
         begin
          illegal_insn_r   = 1'b0;
          insn_one_clock_r = 1'b0;
         end

      `OR1K_OPCODE_LD,
      `OR1K_OPCODE_SD:
         begin
          illegal_insn_r   = !(OPTION_OPERAND_WIDTH==64);
          insn_one_clock_r = !(OPTION_OPERAND_WIDTH==64); // pass illegal insruction exception through EXECUTE
         end

      `OR1K_OPCODE_CUST1,
      `OR1K_OPCODE_CUST2,
      `OR1K_OPCODE_CUST3,
      `OR1K_OPCODE_CUST4,
      `OR1K_OPCODE_CUST5,
      `OR1K_OPCODE_CUST6,
      `OR1K_OPCODE_CUST7,
      `OR1K_OPCODE_CUST8:
         begin
          illegal_insn_r   = 1'b1;
          insn_one_clock_r = 1'b1; // pass illegal insruction exception through EXECUTE
         end

      `OR1K_OPCODE_FPU:
         begin
          illegal_insn_r = (FEATURE_FPU == "NONE");
          case (dcod_insn_i[`OR1K_FPUOP_SELECT])
            `OR1K_FPCOP_SFEQ,
            `OR1K_FPCOP_SFNE,
            `OR1K_FPCOP_SFGT,
            `OR1K_FPCOP_SFGE,
            `OR1K_FPCOP_SFLT,
            `OR1K_FPCOP_SFLE: insn_one_clock_r = 1'b1;
            default:          insn_one_clock_r = (FEATURE_FPU == "NONE"); // pass illegal insruction exception through EXECUTE
          endcase
         end

      //`OR1K_OPCODE_MACRC, // Same to l.movhi - check!
      `OR1K_OPCODE_MACI,
      `OR1K_OPCODE_MAC:
         begin
          illegal_insn_r   = 1'b1;
          insn_one_clock_r = 1'b1; // pass illegal insruction exception through EXECUTE
         end

      `OR1K_OPCODE_MULI:
         begin
          illegal_insn_r   = 1'b1;
          insn_one_clock_r = 1'b0;
         end

      `OR1K_OPCODE_SHRTI:
         begin
          case (dcod_insn_i[`OR1K_ALU_OPC_SECONDARY_SELECT])
            `OR1K_ALU_OPC_SECONDARY_SHRT_SLL,
            `OR1K_ALU_OPC_SECONDARY_SHRT_SRL,
            `OR1K_ALU_OPC_SECONDARY_SHRT_SRA,
            `OR1K_ALU_OPC_SECONDARY_SHRT_ROR: illegal_insn_r = 1'b0;
            default:                          illegal_insn_r = 1'b1;
          endcase
          insn_one_clock_r = 1'b1;
         end

      `OR1K_OPCODE_ALU:
        case (dcod_insn_i[`OR1K_ALU_OPC_SELECT])
          `OR1K_ALU_OPC_ADD,
          `OR1K_ALU_OPC_ADDC,
          `OR1K_ALU_OPC_SUB,
          `OR1K_ALU_OPC_OR,
          `OR1K_ALU_OPC_XOR,
          `OR1K_ALU_OPC_AND,
          `OR1K_ALU_OPC_CMOV,
          `OR1K_ALU_OPC_FFL1:
            begin
              illegal_insn_r   = 1'b0;
              insn_one_clock_r = 1'b1;
            end
          `OR1K_ALU_OPC_DIV,
          `OR1K_ALU_OPC_DIVU,
          `OR1K_ALU_OPC_MUL,
          `OR1K_ALU_OPC_MULU:
            begin
              illegal_insn_r   = 1'b0;
              insn_one_clock_r = 1'b0;
            end
          `OR1K_ALU_OPC_EXTBH,
          `OR1K_ALU_OPC_EXTW:
            begin
              illegal_insn_r   = 1'b1;
              insn_one_clock_r = 1'b1;
            end
          `OR1K_ALU_OPC_SHRT:
            begin
              case (dcod_insn_i[`OR1K_ALU_OPC_SECONDARY_SELECT])
                `OR1K_ALU_OPC_SECONDARY_SHRT_SLL,
                `OR1K_ALU_OPC_SECONDARY_SHRT_SRL,
                `OR1K_ALU_OPC_SECONDARY_SHRT_SRA,
                `OR1K_ALU_OPC_SECONDARY_SHRT_ROR: illegal_insn_r = 1'b0;
                default:                          illegal_insn_r = 1'b1;
              endcase // case (dcod_insn_i[`OR1K_ALU_OPC_SECONDARY_SELECT])
              insn_one_clock_r = 1'b1;
            end
          default:
            begin
              illegal_insn_r   = 1'b1;
              insn_one_clock_r = 1'b1; // pass illegal insruction exception through EXECUTE
            end
        endcase // alu_opc

      `OR1K_OPCODE_SYSTRAPSYNC: begin
        if (((dcod_insn_i[`OR1K_SYSTRAPSYNC_OPC_SELECT] == `OR1K_SYSTRAPSYNC_OPC_SYSCALL) &
             (FEATURE_SYSCALL=="ENABLED")) |
            ((dcod_insn_i[`OR1K_SYSTRAPSYNC_OPC_SELECT] == `OR1K_SYSTRAPSYNC_OPC_TRAP) &
             (FEATURE_TRAP=="ENABLED")) |
            ((dcod_insn_i[`OR1K_SYSTRAPSYNC_OPC_SELECT] == `OR1K_SYSTRAPSYNC_OPC_PSYNC) &
             (FEATURE_PSYNC!="NONE")) |
            ((dcod_insn_i[`OR1K_SYSTRAPSYNC_OPC_SELECT] == `OR1K_SYSTRAPSYNC_OPC_CSYNC) &
             (FEATURE_CSYNC!="NONE"))) begin
          illegal_insn_r   = 1'b0;
          insn_one_clock_r = 1'b1; // MAROCCHINO_TODO: perhaps it is incorrect for PSYNC & CSYNC
        end
        else if (dcod_insn_i[`OR1K_SYSTRAPSYNC_OPC_SELECT] == `OR1K_SYSTRAPSYNC_OPC_MSYNC) begin
          illegal_insn_r   = 1'b0;
          insn_one_clock_r = 1'b0;
        end
        else begin
          illegal_insn_r   = 1'b1;
          insn_one_clock_r = 1'b1;
        end
      end // case sys-trap-sync

      default:
        begin
          illegal_insn_r   = 1'b1;
          insn_one_clock_r = 1'b1; // pass illegal insruction exception through EXECUTE
        end
    endcase // case (dcod_insn_i[`OR1K_OPCODE_SELECT])
  end // always


  wire dcod_except_illegal = illegal_insn_r;


  // Calculate the link register result
  wire [OPTION_OPERAND_WIDTH-1:0] next_pc_after_branch_insn =
    (pc_decode_i + 8); // (FEATURE_DELAY_SLOT == "ENABLED")
  // latch it
  always @(posedge clk)
    if (padv_decode_i)
      exec_jal_result_o <= next_pc_after_branch_insn; // for GPR[9]


  // Branch detection

  // jumps to register
  assign dcod_op_jr = (opc_insn == `OR1K_OPCODE_JR) |
                      (opc_insn == `OR1K_OPCODE_JALR);
  // jumps with link
  assign dcod_op_jal  = (opc_insn == `OR1K_OPCODE_JALR) |
                        (opc_insn == `OR1K_OPCODE_JAL);

  // conditional branches
  assign dcod_op_bf_o     = (opc_insn == `OR1K_OPCODE_BF)  & (~pipeline_flush_i);
  assign dcod_op_bnf_o    = (opc_insn == `OR1K_OPCODE_BNF) & (~pipeline_flush_i);
  // combined conditional branches
  wire dcod_op_brcond     = dcod_op_bf_o | dcod_op_bnf_o;

  // jumps or contitional branches to immediate
  assign dcod_op_jb_imm = (opc_insn < `OR1K_OPCODE_NOP); // l.j  | l.jal  | l.bnf | l.bf

  wire branch_to_imm = dcod_op_jb_imm &
                       // l.j/l.jal  or  l.bf/bnf and flag is right
                       (~(|opc_insn[2:1]) | (opc_insn[2] == predicted_flag_i));

  wire [OPTION_OPERAND_WIDTH-1:0] branch_to_imm_target =
    pc_decode_i +
    {{4{dcod_immjbr_upper_o[9]}},dcod_immjbr_upper_o,dcod_imm16,2'b00};

  wire branch_to_reg = dcod_op_jr & (~(exec_rf_wb_o & (dcod_rfb_adr_o == exec_rfd_adr_o)));

  assign dcod_branch_target_o = branch_to_imm ? branch_to_imm_target : dcod_rfb_i;

  // exception on wrong branch target
  assign dcod_branch_o          = branch_to_imm | branch_to_reg;
  wire   dcod_except_ibus_align = dcod_branch_o & (|dcod_branch_target_o[1:0]);


  wire [OPTION_OPERAND_WIDTH-1:0] dcod_mispredict_target =
    ((dcod_op_bf_o & (~predicted_flag_i)) |
     (dcod_op_bnf_o & predicted_flag_i)) ? branch_to_imm_target :
                                           next_pc_after_branch_insn;

  // Forward branch prediction signals to execute stage
  always @(posedge clk) begin
    if (padv_decode_i & dcod_op_brcond) begin
      exec_mispredict_target_o <= dcod_mispredict_target;
      exec_predicted_flag_o    <= predicted_flag_i;
    end
  end

  // take branch flag for FETCH
  assign dcod_take_branch_o = branch_to_imm | dcod_op_jr;



  // FPU-32 arithmetic part
  wire [(`OR1K_FPUOP_WIDTH-1):0] dcod_op_fp32_arith =
    {(FEATURE_FPU != "NONE") & (opc_insn == `OR1K_OPCODE_FPU) & ~dcod_insn_i[3],
      dcod_insn_i[`OR1K_FPUOP_WIDTH-2:0]};

  // FPU-32 comparison part
  wire [(`OR1K_FPUOP_WIDTH-1):0] dcod_op_fp32_cmp =
    {(FEATURE_FPU != "NONE") & (opc_insn == `OR1K_OPCODE_FPU) & dcod_insn_i[3],
      dcod_insn_i[`OR1K_FPUOP_WIDTH-2:0]};



  // Detect the situation where there is a jump to register in decode
  // stage and an instruction in execute stage that will write to that
  // register.
  //
  // decode bubble is also used in "ctrl" module to block FETCH advance.
  //
  // A bubble is also inserted when an rfe instruction is in decode stage,
  // the main purpose of this is to stall fetch while the rfe is propagating
  // up to ctrl stage.

  // due to registry hazard for "jr"
  wire jr_bubble = dcod_op_jr & (exec_rf_wb_o & (dcod_rfb_adr_o == exec_rfd_adr_o));

  // all bubbles
  assign dcod_bubble_o = jr_bubble | dcod_op_rfe;
  wire   padv_bubble   = dcod_bubble_o & padv_decode_i;

  // to prevent changing PC latched in EXEC->CTRL
  always @(posedge clk `OR_ASYNC_RST) begin
    if (rst)
      exec_bubble_o <= 1'b0;
    else if (pipeline_flush_i)
      exec_bubble_o <= 1'b0;
    else if (padv_decode_i)
      exec_bubble_o <= dcod_bubble_o;
  end // @ clock



  // single clock "OP" controls to execute stage
  always @(posedge clk `OR_ASYNC_RST) begin
    if (rst) begin
      // flag and branches
      exec_op_setflag_o        <= 1'b0;
      exec_op_jal_o            <= 1'b0;
      exec_op_brcond_o         <= 1'b0;
      exec_op_branch_o         <= 1'b0;
      exec_delay_slot_o        <= 1'b0;
      // ALU common
      exec_opc_alu_o           <= {`OR1K_ALU_OPC_WIDTH{1'b0}};
      exec_opc_alu_secondary_o <= {`OR1K_ALU_OPC_WIDTH{1'b0}};
      // Particular EXEC units related
      exec_op_add_o            <= 1'b0;
      exec_adder_do_sub_o      <= 1'b0;
      exec_adder_do_carry_o    <= 1'b0;
      exec_op_shift_o          <= 1'b0;
      exec_op_ffl1_o           <= 1'b0;
      exec_op_movhi_o          <= 1'b0;
      exec_op_cmov_o           <= 1'b0;
      // FPU comparison
      exec_op_fp32_cmp_o       <= {`OR1K_FPUOP_WIDTH{1'b0}};
    end
    else if (pipeline_flush_i | padv_bubble) begin
      // bubble already masked by padv-decode forces clearing exception flags
      // flag and branches
      exec_op_setflag_o        <= 1'b0;
      exec_op_jal_o            <= 1'b0;
      exec_op_brcond_o         <= 1'b0;
      exec_op_branch_o         <= 1'b0;
      exec_delay_slot_o        <= 1'b0;
      // ALU common
      exec_opc_alu_o           <= {`OR1K_ALU_OPC_WIDTH{1'b0}};
      exec_opc_alu_secondary_o <= {`OR1K_ALU_OPC_WIDTH{1'b0}};
      // Particular EXEC units related
      exec_op_add_o            <= 1'b0;
      exec_adder_do_sub_o      <= 1'b0;
      exec_adder_do_carry_o    <= 1'b0;
      exec_op_shift_o          <= 1'b0;
      exec_op_ffl1_o           <= 1'b0;
      exec_op_movhi_o          <= 1'b0;
      exec_op_cmov_o           <= 1'b0;
      // FPU comparison
      exec_op_fp32_cmp_o       <= {`OR1K_FPUOP_WIDTH{1'b0}};
    end
    else if (padv_decode_i) begin
      // flag and branches
      exec_op_setflag_o        <= dcod_op_setflag;
      exec_op_jal_o            <= dcod_op_jal;
      exec_op_brcond_o         <= dcod_op_brcond;
      exec_op_branch_o         <= dcod_op_branch_i;
      exec_delay_slot_o        <= dcod_delay_slot_i;
      // ALU common
      exec_opc_alu_o           <= dcod_opc_alu;
      exec_opc_alu_secondary_o <= dcod_opc_alu_secondary;
      // Particular EXEC units related
      exec_op_add_o            <= dcod_op_add;
      exec_adder_do_sub_o      <= dcod_adder_do_sub;
      exec_adder_do_carry_o    <= dcod_adder_do_carry;
      exec_op_shift_o          <= dcod_op_shift;
      exec_op_ffl1_o           <= dcod_op_ffl1;
      exec_op_movhi_o          <= dcod_op_movhi;
      exec_op_cmov_o           <= dcod_op_cmov;
      // FPU comparison
      exec_op_fp32_cmp_o       <= dcod_op_fp32_cmp;
    end
  end // @clock


  // multi-clock/pipelined "OP" controls with auto deasssert to execute stage
  always @(posedge clk `OR_ASYNC_RST) begin
    if (rst) begin
      // LSU related
      exec_op_lsu_load_o       <= 1'b0;
      exec_op_lsu_store_o      <= 1'b0;
      exec_op_lsu_atomic_o     <= 1'b0;
      // Sync operations
      exec_op_msync_o          <= 1'b0;
      // Particular EXEC units related
      exec_op_mul_o            <= 1'b0;
      exec_op_div_o            <= 1'b0;
      exec_op_div_signed_o     <= 1'b0;
      exec_op_div_unsigned_o   <= 1'b0;
      // MTSPR / MFSPR
      exec_op_mfspr_o          <= 1'b0;
      exec_op_mtspr_o          <= 1'b0;
      // FPU arithmetic
      exec_op_fp32_arith_o     <= {`OR1K_FPUOP_WIDTH{1'b0}};
    end
    else if (pipeline_flush_i | padv_bubble) begin
      // bubble already masked by padv-decode forces clearing exception flags
      // LSU related
      exec_op_lsu_load_o       <= 1'b0;
      exec_op_lsu_store_o      <= 1'b0;
      exec_op_lsu_atomic_o     <= 1'b0;
      // Sync operations
      exec_op_msync_o          <= 1'b0;
      // Multiplier related
      exec_op_mul_o            <= 1'b0;
      // Divider related
      exec_op_div_o            <= 1'b0;
      exec_op_div_signed_o     <= 1'b0;
      exec_op_div_unsigned_o   <= 1'b0;
      // MTSPR / MFSPR
      exec_op_mfspr_o          <= 1'b0;
      exec_op_mtspr_o          <= 1'b0;
      // FPU arithmetic
      exec_op_fp32_arith_o     <= {`OR1K_FPUOP_WIDTH{1'b0}};
    end
    else if (padv_decode_i) begin
      // LSU related
      exec_op_lsu_load_o       <= dcod_op_lsu_load;
      exec_op_lsu_store_o      <= dcod_op_lsu_store;
      exec_op_lsu_atomic_o     <= dcod_op_lsu_atomic;
      // Sync operations
      exec_op_msync_o          <= dcod_op_msync;
      // Multiplier related
      exec_op_mul_o            <= dcod_op_mul;
      // Divider related
      exec_op_div_o            <= dcod_op_div;
      exec_op_div_signed_o     <= dcod_op_div_signed;
      exec_op_div_unsigned_o   <= dcod_op_div_unsigned;
      // MTSPR / MFSPR
      exec_op_mfspr_o          <= dcod_op_mfspr;
      exec_op_mtspr_o          <= dcod_op_mtspr;
      // FPU arithmetic
      exec_op_fp32_arith_o     <= dcod_op_fp32_arith;
    end
    else begin // MAROCCHINO_TODO: if (exec_insn_taken_i)
      // LSU related
      if (take_op_lsu_i) begin
        exec_op_lsu_load_o   <= 1'b0;
        exec_op_lsu_store_o  <= 1'b0;
        exec_op_lsu_atomic_o <= 1'b0;
      end
      // Sync operations
      exec_op_msync_o <= 1'b0;
      // Multiplier related
      if (take_op_mul_i) begin
        exec_op_mul_o <= 1'b0;
      end
      // Divider related
      if (take_op_div_i) begin
        exec_op_div_o          <= 1'b0;
        exec_op_div_signed_o   <= 1'b0;
        exec_op_div_unsigned_o <= 1'b0;
      end
      // MTSPR / MFSPR
      if (take_op_mXspr_i) begin
        exec_op_mfspr_o <= 1'b0;
        exec_op_mtspr_o <= 1'b0;
      end
      // FPU arithmetic
      if (take_op_fp32_arith_i) begin
        exec_op_fp32_arith_o <= {`OR1K_FPUOP_WIDTH{1'b0}};
      end
    end
  end // @clock

  //   lsu additional parameters
  always @(posedge clk) begin
    if (padv_decode_i) begin
      exec_lsu_length_o <= dcod_lsu_length;
      exec_lsu_zext_o   <= dcod_lsu_zext;
    end
  end // @clock


  // rfe is a special case, instead of pushing the pipeline full
  // of nops on a decode bubble, we push it full of rfes.
  // The reason for this is that we need the rfe to reach control
  // stage so it will cause the branch.
  // It will clear itself by the pipeline_flush_i that the rfe
  // will generate.
  always @(posedge clk `OR_ASYNC_RST) begin
    if (rst) begin
      exec_op_rfe_o <= 1'b0;
    end
    else if (pipeline_flush_i) begin
      exec_op_rfe_o <= 1'b0;
    end
    else if (padv_decode_i) begin
      exec_op_rfe_o <= dcod_op_rfe;
    end
  end // @clock


  // IMM
  always @(posedge clk) begin
    if (padv_decode_i) begin
      exec_immediate_o     <= dcod_immediate;
      exec_immediate_sel_o <= dcod_immediate_sel;
    end
  end // @clock


  //--------------------//
  // Exceptions related //
  //--------------------//
  always @(posedge clk `OR_ASYNC_RST) begin
    if (rst) begin
      // FETCH/I-BUS related exceptions
      exec_except_ibus_align_o <= 1'b0;
      exec_except_ibus_err_o   <= 1'b0;
      exec_except_itlb_miss_o  <= 1'b0;
      exec_except_ipagefault_o <= 1'b0;
      // DECODE related exceptions
      exec_except_illegal_o    <= 1'b0;
      exec_except_syscall_o    <= 1'b0;
      exec_except_trap_o       <= 1'b0;
      // enable exceptions processing
      // (no instruction for restart)
      exec_excepts_en_o        <= 1'b0;
    end
    else if (pipeline_flush_i | padv_bubble) begin
      // bubble already masked by padv-decode forces clearing exception flags
      // FETCH/I-BUS related exceptions
      exec_except_ibus_align_o <= 1'b0;
      exec_except_ibus_err_o   <= 1'b0;
      exec_except_itlb_miss_o  <= 1'b0;
      exec_except_ipagefault_o <= 1'b0;
      // DECODE related exceptions
      exec_except_illegal_o    <= 1'b0;
      exec_except_syscall_o    <= 1'b0;
      exec_except_trap_o       <= 1'b0;
      // enable exceptions processing
      // (no instruction for restart)
      exec_excepts_en_o        <= 1'b0;
    end
    else if (padv_decode_i) begin
      // FETCH/I-BUS related exceptions
      exec_except_ibus_align_o <= dcod_except_ibus_align;
      exec_except_ibus_err_o   <= dcod_except_ibus_err_i;
      exec_except_itlb_miss_o  <= dcod_except_itlb_miss_i;
      exec_except_ipagefault_o <= dcod_except_ipagefault_i;
      // DECODE related exceptions
      exec_except_illegal_o    <= dcod_except_illegal;
      exec_except_syscall_o    <= dcod_except_syscall;
      exec_except_trap_o       <= dcod_except_trap;
      // enable exceptions processing
      // some instructions couldn't be restarted
      exec_excepts_en_o        <= dcod_insn_valid_i & 
                                  (~dcod_bubble_o) & 
                                  (~dcod_op_mtspr) & 
                                  (~dcod_op_rfe);
    end
  end // @ clock


  //-------------------------------------------------//
  // 1-clock instruction flag to force EXECUTE valid //
  //-------------------------------------------------//
  // !!! invalid instruction isn't included here
  // !!! because it is already included in insn-one-clock-r
  wire dcod_excepts = dcod_except_ibus_align  | dcod_except_ibus_err_i   |
                      dcod_except_itlb_miss_i | dcod_except_ipagefault_i |
                      dcod_except_syscall     | dcod_except_trap;
  // ---
  always @(posedge clk `OR_ASYNC_RST) begin
    if (rst)
      exec_insn_1clk_o <= 1'b1;
    else if (pipeline_flush_i | padv_bubble)
      exec_insn_1clk_o <= 1'b1;
    else if (padv_decode_i)
      exec_insn_1clk_o <= insn_one_clock_r | dcod_excepts;
  end // @ clock


  //------------//
  // WB related //
  //------------//
  // --- destination address & PC ---
  always @(posedge clk) begin
    if (padv_decode_i & 
        (~pipeline_flush_i) & (~dcod_bubble_o)) begin
      exec_rfd_adr_o <= dcod_rfd_adr;
      pc_exec_o      <= pc_decode_i;
    end
  end // @clock
  // --- flag that WB is required ---
  // bubble already masked by padv-decode forces clearing exception flags
  always @(posedge clk `OR_ASYNC_RST) begin
    if (rst)
      exec_rf_wb_o <= 1'b0;
    else if (pipeline_flush_i | padv_bubble)
      exec_rf_wb_o <= 1'b0;
    else if (padv_decode_i)
      exec_rf_wb_o <= dcod_rf_wb;
  end // @clock

endmodule // mor1kx_decode_marocchino
