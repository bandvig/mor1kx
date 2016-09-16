////////////////////////////////////////////////////////////////////////
//                                                                    //
//  mor1kx_cpu_marocchino                                             //
//                                                                    //
//  Description: MAROCCHINO pipeline CPU module                       //
//               Derived from mor1kx_cpu_cappuccino                   //
//                                                                    //
////////////////////////////////////////////////////////////////////////
//                                                                    //
//   Copyright (C) 2012 Julius Baxter                                 //
//                      juliusbaxter@gmail.com                        //
//                                                                    //
//   Copyright (C) 2015 Andrey Bacherov                               //
//                      avbacherov@opencores.org                      //
//                                                                    //
//      This Source Code Form is subject to the terms of the          //
//      Open Hardware Description License, v. 1.0. If a copy          //
//      of the OHDL was not distributed with this file, You           //
//      can obtain one at http://juliusbaxter.net/ohdl/ohdl.txt       //
//                                                                    //
////////////////////////////////////////////////////////////////////////

`include "mor1kx-defines.v"

module mor1kx_cpu_marocchino
#(
  parameter OPTION_OPERAND_WIDTH = 32,
  // data cache
  parameter OPTION_DCACHE_BLOCK_WIDTH   = 5,
  parameter OPTION_DCACHE_SET_WIDTH     = 9,
  parameter OPTION_DCACHE_WAYS          = 2,
  parameter OPTION_DCACHE_LIMIT_WIDTH   = 32,
  parameter OPTION_DCACHE_SNOOP         = "NONE",
  parameter OPTION_DCACHE_CLEAR_ON_INIT = 0,
  // data mmu
  parameter FEATURE_DMMU_HW_TLB_RELOAD  = "NONE",
  parameter OPTION_DMMU_SET_WIDTH       = 6,
  parameter OPTION_DMMU_WAYS            = 1,
  parameter OPTION_DMMU_CLEAR_ON_INIT   = 0,
  // instruction cache
  parameter OPTION_ICACHE_BLOCK_WIDTH   = 5,
  parameter OPTION_ICACHE_SET_WIDTH     = 9,
  parameter OPTION_ICACHE_WAYS          = 2,
  parameter OPTION_ICACHE_LIMIT_WIDTH   = 32,
  parameter OPTION_ICACHE_CLEAR_ON_INIT = 0,
  // instruction mmu
  parameter FEATURE_IMMU_HW_TLB_RELOAD  = "NONE",
  parameter OPTION_IMMU_SET_WIDTH       = 6,
  parameter OPTION_IMMU_WAYS            = 1,
  parameter OPTION_IMMU_CLEAR_ON_INIT   = 0,

  parameter FEATURE_DEBUGUNIT    = "NONE",
  parameter FEATURE_PERFCOUNTERS = "NONE",

  parameter OPTION_PIC_TRIGGER   = "LEVEL",
  parameter OPTION_PIC_NMI_WIDTH = 0,

  parameter OPTION_RF_CLEAR_ON_INIT  = 0,
  parameter OPTION_RF_ADDR_WIDTH     = 5,

  parameter OPTION_RESET_PC = {{(OPTION_OPERAND_WIDTH-13){1'b0}},
                              `OR1K_RESET_VECTOR,8'd0},

  parameter FEATURE_PSYNC = "NONE",
  parameter FEATURE_CSYNC = "NONE",

  parameter FEATURE_FPU    = "NONE", // ENABLED|NONE: pipeline marocchino

  parameter OPTION_STORE_BUFFER_DEPTH_WIDTH   = 4, // 16 taps
  parameter OPTION_STORE_BUFFER_CLEAR_ON_INIT = 0,

  parameter FEATURE_MULTICORE      = "NONE",

  parameter FEATURE_TRACEPORT_EXEC = "NONE"
)
(
  input                             clk,
  input                             rst,

  // Instruction bus
  input                             ibus_err_i,
  input                             ibus_ack_i,
  input      [`OR1K_INSN_WIDTH-1:0] ibus_dat_i,
  output [OPTION_OPERAND_WIDTH-1:0] ibus_adr_o,
  output                            ibus_req_o,
  output                            ibus_burst_o,

  // Data bus
  input                             dbus_err_i,
  input                             dbus_ack_i,
  input  [OPTION_OPERAND_WIDTH-1:0] dbus_dat_i,
  output [OPTION_OPERAND_WIDTH-1:0] dbus_adr_o,
  output [OPTION_OPERAND_WIDTH-1:0] dbus_dat_o,
  output                            dbus_req_o,
  output                      [3:0] dbus_bsel_o,
  output                            dbus_we_o,
  output                            dbus_burst_o,

  // Interrupts
  input                      [31:0] irq_i,

  // Debug interface
  input                      [15:0] du_addr_i,
  input                             du_stb_i,
  input  [OPTION_OPERAND_WIDTH-1:0] du_dat_i,
  input                             du_we_i,
  output [OPTION_OPERAND_WIDTH-1:0] du_dat_o,
  output                            du_ack_o,
  // Stall control from debug interface
  input                             du_stall_i,
  output                            du_stall_o,

  output reg                        traceport_exec_valid_o,
  output reg                 [31:0] traceport_exec_pc_o,
  output reg [`OR1K_INSN_WIDTH-1:0] traceport_exec_insn_o,
  output [OPTION_OPERAND_WIDTH-1:0] traceport_exec_wbdata_o,
  output [OPTION_RF_ADDR_WIDTH-1:0] traceport_exec_wbreg_o,
  output                            traceport_exec_wben_o,

  // SPR accesses to external units (cache, mmu, etc.)
  output [15:0]                     spr_bus_addr_o,
  output                            spr_bus_we_o,
  output                            spr_bus_stb_o,
  output [OPTION_OPERAND_WIDTH-1:0] spr_bus_dat_o,

  input  [OPTION_OPERAND_WIDTH-1:0] multicore_coreid_i,
  input  [OPTION_OPERAND_WIDTH-1:0] multicore_numcores_i,

  input                      [31:0] snoop_adr_i,
  input                             snoop_en_i
);

  wire     [`OR1K_INSN_WIDTH-1:0] dcod_insn;
  wire                            dcod_insn_valid;

  wire [OPTION_OPERAND_WIDTH-1:0] pc_decode;
  wire [OPTION_OPERAND_WIDTH-1:0] pc_exec;
  wire [OPTION_OPERAND_WIDTH-1:0] pc_wb;

  wire                            wb_atomic_flag_set;
  wire                            wb_atomic_flag_clear;

  wire                            wb_int_flag_set;
  wire                            wb_int_flag_clear;

  wire                            ctrl_flag;
  wire                            ctrl_carry;

  wire                            dcod_flag_wb; // instruction writes comparison flag
  wire                            dcod_carry_wb; // instruction writes carry flag

  wire                            dcod_flag_req;  // instructions require comparison flag
  wire                            dcod_carry_req; // instructions require carry flag

  wire                            dcod_op_mfspr; // to OMAN & CTRL (not latched)
  wire                            dcod_op_mtspr; // to OMAN & CTRL (not latched)


  wire [OPTION_OPERAND_WIDTH-1:0] wb_alu_1clk_result;
  wire [OPTION_OPERAND_WIDTH-1:0] wb_div_result;
  wire [OPTION_OPERAND_WIDTH-1:0] wb_mul_result;
  wire [OPTION_OPERAND_WIDTH-1:0] wb_fp32_arith_res;
  wire [OPTION_OPERAND_WIDTH-1:0] wb_lsu_result;
  wire [OPTION_OPERAND_WIDTH-1:0] wb_mfspr_dat;
  wire [OPTION_OPERAND_WIDTH-1:0] wb_result; // WB result combiner


  wire                            dcod_valid;
  wire                            exec_valid;
  wire                            lsu_valid;   // result ready or exceptions


  wire [OPTION_OPERAND_WIDTH-1:0] dcod_rfa;
  wire [OPTION_OPERAND_WIDTH-1:0] dcod_rfb;
  wire [OPTION_RF_ADDR_WIDTH-1:0] dcod_rfa_adr;
  wire [OPTION_RF_ADDR_WIDTH-1:0] dcod_rfb_adr;
  wire                            dcod_rfa_req;
  wire                            dcod_rfb_req;
  wire [OPTION_OPERAND_WIDTH-1:0] dcod_immediate;
  wire                            dcod_immediate_sel;


  wire [OPTION_RF_ADDR_WIDTH-1:0] dcod_rfd_adr;
  wire                            dcod_rf_wb;
  wire [OPTION_RF_ADDR_WIDTH-1:0] wb_rfd_adr;
  wire                            wb_rf_wb;

  // from OMAN (EXECUTE-to-DECODE hazards)
  wire                            exe2dec_hazard_a;
  wire                            exe2dec_hazard_b;

  wire                            dcod_op_jr;
  wire                            stall_fetch;

  wire                            dcod_delay_slot;
  wire                            wb_delay_slot;


  // branching
  //  ## detect jump/branch to indicate "delay slot" for next fetched instruction
  wire                            dcod_jump_or_branch;
  //  ## support IBUS error handling in CTRL
  wire                            exec_jump_or_branch;
  //  ## do branch (pedicted or unconditional)
  wire                            dcod_do_branch;
  wire [OPTION_OPERAND_WIDTH-1:0] dcod_do_branch_target;

  // stall conditional fetching till flag computation completion (see OMAN for details)
  wire                            dcod_op_brcond;  // l.bf or l.bnf



  wire      [`OR1K_IMM_WIDTH-1:0] dcod_imm16;
  wire                            dcod_op_lsu_load;
  wire                            dcod_op_lsu_store;
  wire                            dcod_op_lsu_atomic;
  wire                      [1:0] dcod_lsu_length;
  wire                            dcod_lsu_zext;
  wire                            dcod_op_msync;
  wire                            lsu_busy;
  wire                            grant_wb_to_lsu;


  // Instruction which passes EXECUTION through
  wire                            dcod_op_pass_exec;


  wire                            dcod_op_1clk;
  wire                            op_1clk_busy;
  wire  [`OR1K_ALU_OPC_WIDTH-1:0] dcod_opc_alu_secondary;

  wire                            dcod_op_add;
  wire                            dcod_adder_do_sub;
  wire                            dcod_adder_do_carry;

  wire                            dcod_op_jal;
  wire [OPTION_OPERAND_WIDTH-1:0] dcod_jal_result;

  wire                            dcod_op_shift;
  wire                            dcod_op_ffl1;
  wire                            dcod_op_movhi;
  wire                            dcod_op_cmov;

  wire  [`OR1K_ALU_OPC_WIDTH-1:0] dcod_opc_logic;

  wire                            dcod_op_setflag;

  wire                            grant_wb_to_1clk;

  // Divider
  wire                            dcod_op_div;
  wire                            dcod_op_div_signed;
  wire                            dcod_op_div_unsigned;
  wire                            div_busy;
  wire                            div_valid;
  wire                            grant_wb_to_div;


  // Pipelined multiplier
  wire                            dcod_op_mul;
  wire                            mul_busy;
  wire                            mul_valid;
  wire                            grant_wb_to_mul;

  // FPU-32 arithmetic part
  wire                              dcod_op_fp32_arith;
  wire                        [2:0] dcod_opc_fp32_arith;
  wire                              fp32_arith_busy; // idicates that arihmetic units are busy
  wire                              fp32_arith_valid;
  wire                              grant_wb_to_fp32_arith;
  wire  [`OR1K_FPCSR_ALLF_SIZE-1:0] wb_fp32_arith_fpcsr;    // only flags
  wire                              wb_fp32_arith_wb_fpcsr; // update FPCSR
  wire                              wb_except_fp32_arith;   // generate FPx exception by FPx flags

  // FPU-32 comparison part
  wire                            dcod_op_fp32_cmp;
  wire                      [2:0] dcod_opc_fp32_cmp;
  wire                            wb_fp32_flag_set;
  wire                            wb_fp32_flag_clear;
  wire                            wb_fp32_cmp_inv;
  wire                            wb_fp32_cmp_inf;
  wire                            wb_fp32_cmp_wb_fpcsr;
  wire                            wb_except_fp32_cmp;

  // Forwarding comparision flag
  wire                            exec_op_1clk_cmp; // integer or fp32
  wire                            exec_flag_set;    // integer or fp32 comparison result


  wire [OPTION_OPERAND_WIDTH-1:0] sbuf_eear;
  wire [OPTION_OPERAND_WIDTH-1:0] sbuf_epcr;
  wire                            sbuf_err;


  // SPR access buses (Unit -> CTRL part)
  //   GPR
  wire                            spr_bus_ack_gpr;
  wire [OPTION_OPERAND_WIDTH-1:0] spr_bus_dat_gpr;
  //   Data MMU
  wire [OPTION_OPERAND_WIDTH-1:0] spr_bus_dat_dmmu;
  wire                            spr_bus_ack_dmmu;
  //   Data Cache
  wire [OPTION_OPERAND_WIDTH-1:0] spr_bus_dat_dc;
  wire                            spr_bus_ack_dc;
  //   Insn MMU
  wire [OPTION_OPERAND_WIDTH-1:0] spr_bus_dat_immu;
  wire                            spr_bus_ack_immu;
  //   Insn Cache
  wire [OPTION_OPERAND_WIDTH-1:0] spr_bus_dat_ic;
  wire                            spr_bus_ack_ic;


  // pipeline controls from CTRL to units
  wire padv_fetch;
  wire padv_decode;
  wire padv_wb;
  wire pipeline_flush;

  // enable modules and other control signals from CTRL
  wire                            ic_enable;
  wire                            immu_enable;
  wire                            dc_enable;
  wire                            dmmu_enable;
  wire                            supervisor_mode;

  // FPU related controls
  wire                             except_fpu_enable;
  wire [`OR1K_FPCSR_ALLF_SIZE-1:0] ctrl_fpu_mask_flags;
  wire   [`OR1K_FPCSR_RM_SIZE-1:0] ctrl_fpu_round_mode;


  // Exceptions: reported by IFETCH
  //  # connections IFETCH->OMAN
  wire fetch_except_ibus_err;
  wire fetch_except_ipagefault;
  wire fetch_except_itlb_miss;
  wire fetch_except_ibus_align;
  //  # connections OMAN(WB-latches)->CTRL
  wire wb_except_ibus_err;
  wire wb_except_ipagefault;
  wire wb_except_itlb_miss;
  wire wb_except_ibus_align;

  // Exceptions: reported from DECODE to OMAN
  wire dcod_except_illegal;
  wire dcod_except_syscall;
  wire dcod_except_trap;
  // Enable l.trap exception
  wire du_trap_enable;
  // Exceptions: latched by WB latches for processing in CONTROL-unit
  wire wb_except_illegal;
  wire wb_except_syscall;
  wire wb_except_trap;

  //  # combined IFETCH/DECODE exceptions flag
  wire wb_fd_an_except;

  //  # overflow exception
  wire except_overflow_enable;
  wire wb_except_overflow_div;
  wire wb_except_overflow_1clk;

  // Exceptions: reported by LSU
  //  # particular LSU exception flags
  wire                            wb_except_dbus_err;
  wire                            wb_except_dpagefault;
  wire                            wb_except_dtlb_miss;
  wire                            wb_except_dbus_align;
  wire [OPTION_OPERAND_WIDTH-1:0] wb_lsu_except_addr;
  //  # combined LSU exceptions flag
  wire                            wb_an_except_lsu;


  // External Interrupts Collection
  //  # from "Tick Timer"
  wire        tt_rdy;
  wire        tt_interrupt_enable;
  //  # from "Programmble Interrupt Controller"
  wire [31:0] spr_picsr;
  wire        pic_interrupt_enable;
  //  # flag to enabel/disable exterlal interrupts processing
  //    depending on the fact is instructions restartable or not
  wire        exec_interrupts_en;
  //  # WB latches
  reg         wb_tt_interrupt_r;
  reg         wb_pic_interrupt_r;
  reg         wb_an_interrupt_r; // from PIC or TT


  // Exeptions process:
  wire dcod_op_rfe;
  wire wb_op_rfe;
  wire ctrl_branch_exception;
  wire [OPTION_OPERAND_WIDTH-1:0] ctrl_branch_except_pc;
  //   exeptions process: fetch->ctrl
  wire fetch_ecxeption_taken;


  // FETCH none latched outputs
  wire                            fetch_rf_adr_valid; // fetch->rf
  wire [OPTION_RF_ADDR_WIDTH-1:0] fetch_rfa_adr;      // fetch->rf
  wire [OPTION_RF_ADDR_WIDTH-1:0] fetch_rfb_adr;      // fetch->rf


  mor1kx_fetch_marocchino
  #(
    .OPTION_OPERAND_WIDTH             (OPTION_OPERAND_WIDTH), // FETCH
    .OPTION_RESET_PC                  (OPTION_RESET_PC), // FETCH
    // ICACHE configuration
    .OPTION_ICACHE_BLOCK_WIDTH        (OPTION_ICACHE_BLOCK_WIDTH), // FETCH
    .OPTION_ICACHE_SET_WIDTH          (OPTION_ICACHE_SET_WIDTH), // FETCH
    .OPTION_ICACHE_WAYS               (OPTION_ICACHE_WAYS), // FETCH
    .OPTION_ICACHE_LIMIT_WIDTH        (OPTION_ICACHE_LIMIT_WIDTH), // FETCH
    .OPTION_ICACHE_CLEAR_ON_INIT      (OPTION_ICACHE_CLEAR_ON_INIT), // FETCH
    // IMMU configuration
    .FEATURE_IMMU_HW_TLB_RELOAD       (FEATURE_IMMU_HW_TLB_RELOAD), // FETCH
    .OPTION_IMMU_SET_WIDTH            (OPTION_IMMU_SET_WIDTH), // FETCH
    .OPTION_IMMU_WAYS                 (OPTION_IMMU_WAYS), // FETCH
    .OPTION_IMMU_CLEAR_ON_INIT        (OPTION_IMMU_CLEAR_ON_INIT) // FETCH
  )
  u_fetch
  (
    // clocks & resets
    .clk                              (clk),
    .rst                              (rst),

    // pipeline control
    .padv_fetch_i                     (padv_fetch), // FETCH
    .stall_fetch_i                    (stall_fetch), // FETCH
    .pipeline_flush_i                 (pipeline_flush), // FETCH

    // configuration
    .ic_enable_i                      (ic_enable), // FETCH
    .immu_enable_i                    (immu_enable), // FETCH
    .supervisor_mode_i                (supervisor_mode), // FETCH

    // SPR interface
    //  input
    .spr_bus_addr_i                   (spr_bus_addr_o), // FETCH
    .spr_bus_we_i                     (spr_bus_we_o), // FETCH
    .spr_bus_stb_i                    (spr_bus_stb_o), // FETCH
    .spr_bus_dat_i                    (spr_bus_dat_o), // FETCH
    //  output from cache
    .spr_bus_dat_ic_o                 (spr_bus_dat_ic), // FETCH
    .spr_bus_ack_ic_o                 (spr_bus_ack_ic), // FETCH
    //  output from immu
    .spr_bus_dat_immu_o               (spr_bus_dat_immu), // FETCH
    .spr_bus_ack_immu_o               (spr_bus_ack_immu), // FETCH

    // interface to ibus
    .ibus_err_i                       (ibus_err_i), // FETCH
    .ibus_ack_i                       (ibus_ack_i), // FETCH
    .ibus_dat_i                       (ibus_dat_i[`OR1K_INSN_WIDTH-1:0]), // FETCH
    .ibus_req_o                       (ibus_req_o), // FETCH
    .ibus_adr_o                       (ibus_adr_o), // FETCH
    .ibus_burst_o                     (ibus_burst_o), // FETCH

    // branch/jump control transfer
    //  ## detect jump/branch to indicate "delay slot" for next fetched instruction
    .dcod_jump_or_branch_i            (dcod_jump_or_branch), // FETCH
    //  ## do branch (pedicted or unconditional)
    .dcod_do_branch_i                 (dcod_do_branch), // FETCH
    .dcod_do_branch_target_i          (dcod_do_branch_target), // FETCH

    // DU/exception/rfe control transfer
    .ctrl_branch_exception_i          (ctrl_branch_exception), // FETCH
    .ctrl_branch_except_pc_i          (ctrl_branch_except_pc), // FETCH

    //   To RF
    .fetch_rfa_adr_o                  (fetch_rfa_adr), // FETCH (not latched, to RF)
    .fetch_rfb_adr_o                  (fetch_rfb_adr), // FETCH (not latched, to RF)
    .fetch_rf_adr_valid_o             (fetch_rf_adr_valid), // FETCH (bus-access-done & padv-fetch)

    //   To DECODE
    .pc_decode_o                      (pc_decode), // FETCH
    .dcod_insn_o                      (dcod_insn), // FETCH
    .dcod_delay_slot_o                (dcod_delay_slot), // FETCH
    .dcod_insn_valid_o                (dcod_insn_valid), // FETCH

    //   Exceptions
    .fetch_except_ibus_err_o          (fetch_except_ibus_err), // FETCH
    .fetch_except_itlb_miss_o         (fetch_except_itlb_miss), // FETCH
    .fetch_except_ipagefault_o        (fetch_except_ipagefault), // FETCH
    .fetch_exception_taken_o          (fetch_ecxeption_taken) // FETCH
  );



  mor1kx_decode_marocchino
  #(
    .OPTION_OPERAND_WIDTH             (OPTION_OPERAND_WIDTH), // DECODE & DECODE->EXE
    .OPTION_RESET_PC                  (OPTION_RESET_PC), // DECODE & DECODE->EXE
    .OPTION_RF_ADDR_WIDTH             (OPTION_RF_ADDR_WIDTH), // DECODE & DECODE->EXE
    .FEATURE_PSYNC                    (FEATURE_PSYNC), // DECODE & DECODE->EXE
    .FEATURE_CSYNC                    (FEATURE_CSYNC), // DECODE & DECODE->EXE
    .FEATURE_FPU                      (FEATURE_FPU) // DECODE & DECODE->EXE
  )
  u_decode
  (
    // INSN
    .dcod_insn_i                      (dcod_insn), // DECODE & DECODE->EXE
    // Data dependancy detection
    .dcod_op_jr_o                     (dcod_op_jr), // DECODE & DECODE->EXE
    .exe2dec_hazard_b_i               (exe2dec_hazard_b), // DECODE & DECODE->EXE
    // PC
    .pc_decode_i                      (pc_decode), // DECODE & DECODE->EXE
    // IMM
    .dcod_immediate_o                 (dcod_immediate), // DECODE & DECODE->EXE
    .dcod_immediate_sel_o             (dcod_immediate_sel), // DECODE & DECODE->EXE
    // various instruction attributes
    .dcod_rfa_req_o                   (dcod_rfa_req), // DECODE & DECODE->EXE
    .dcod_rfa_adr_o                   (dcod_rfa_adr), // DECODE & DECODE->EXE
    .dcod_rfb_req_o                   (dcod_rfb_req), // DECODE & DECODE->EXE
    .dcod_rfb_adr_o                   (dcod_rfb_adr), // DECODE & DECODE->EXE
    .dcod_rf_wb_o                     (dcod_rf_wb), // DECODE & DECODE->EXE
    .dcod_rfd_adr_o                   (dcod_rfd_adr), // DECODE & DECODE->EXE
    .dcod_flag_wb_o                   (dcod_flag_wb), // DECODE & DECODE->EXE
    .dcod_carry_wb_o                  (dcod_carry_wb), // DECODE & DECODE->EXE
    .dcod_flag_req_o                  (dcod_flag_req), // DECODE & DECODE->EXE
    .dcod_carry_req_o                 (dcod_carry_req), // DECODE & DECODE->EXE
    // flag & branches
    .dcod_jump_or_branch_o            (dcod_jump_or_branch), // DECODE & DECODE->EXE
    // Forwarding comparision flag
    .exec_op_1clk_cmp_i               (exec_op_1clk_cmp), // DECODE & DECODE->EXE
    .exec_flag_set_i                  (exec_flag_set), // DECODE & DECODE->EXE
    .ctrl_flag_i                      (ctrl_flag), // DECODE & DECODE->EXE
    // Do jump/branch and jump/branch target for FETCH
    .dcod_rfb_i                       (dcod_rfb), // DECODE & DECODE->EXE
    .dcod_do_branch_o                 (dcod_do_branch), // DECODE & DECODE->EXE
    .dcod_do_branch_target_o          (dcod_do_branch_target), // DECODE & DECODE->EXE
    // stall conditional fetching till flag computation completion (see OMAN for details)
    .dcod_op_brcond_o                 (dcod_op_brcond), // DECODE & DECODE->EXE
    // LSU related
    .dcod_imm16_o                     (dcod_imm16), // DECODE & DECODE->EXE
    .dcod_op_lsu_load_o               (dcod_op_lsu_load), // DECODE & DECODE->EXE
    .dcod_op_lsu_store_o              (dcod_op_lsu_store), // DECODE & DECODE->EXE
    .dcod_op_lsu_atomic_o             (dcod_op_lsu_atomic), // DECODE & DECODE->EXE
    .dcod_lsu_length_o                (dcod_lsu_length), // DECODE & DECODE->EXE
    .dcod_lsu_zext_o                  (dcod_lsu_zext), // DECODE & DECODE->EXE
    .dcod_op_msync_o                  (dcod_op_msync), // DECODE & DECODE->EXE
    // Instruction which passes EXECUTION through
    .dcod_op_pass_exec_o              (dcod_op_pass_exec), // DECODE & DECODE->EXE
    // 1-clock instruction
    .dcod_op_1clk_o                   (dcod_op_1clk), // DECODE & DECODE->EXE
    // ALU related opc
    .dcod_opc_alu_secondary_o         (dcod_opc_alu_secondary), // DECODE & DECODE->EXE
    // Adder related
    .dcod_op_add_o                    (dcod_op_add), // DECODE & DECODE->EXE
    .dcod_adder_do_sub_o              (dcod_adder_do_sub), // DECODE & DECODE->EXE
    .dcod_adder_do_carry_o            (dcod_adder_do_carry), // DECODE & DECODE->EXE
    // Various 1-clock related
    .dcod_op_shift_o                  (dcod_op_shift), // DECODE & DECODE->EXE
    .dcod_op_ffl1_o                   (dcod_op_ffl1), // DECODE & DECODE->EXE
    .dcod_op_movhi_o                  (dcod_op_movhi), // DECODE & DECODE->EXE
    .dcod_op_cmov_o                   (dcod_op_cmov), // DECODE & DECODE->EXE
    // Logic
    .dcod_opc_logic_o                 (dcod_opc_logic), // DECODE & DECODE->EXE
    // Jump & Link
    .dcod_op_jal_o                    (dcod_op_jal), // DECODE & DECODE->EXE
    .dcod_jal_result_o                (dcod_jal_result), // DECODE & DECODE->EXE
    // Set flag related
    .dcod_op_setflag_o                (dcod_op_setflag), // DECODE & DECODE->EXE
    .dcod_op_fp32_cmp_o               (dcod_op_fp32_cmp), // DECODE & DECODE->EXE
    .dcod_opc_fp32_cmp_o              (dcod_opc_fp32_cmp), // DECODE & DECODE->EXE
    // Multiplier related
    .dcod_op_mul_o                    (dcod_op_mul), // DECODE & DECODE->EXE
    // Divider related
    .dcod_op_div_o                    (dcod_op_div), // DECODE & DECODE->EXE
    .dcod_op_div_signed_o             (dcod_op_div_signed), // DECODE & DECODE->EXE
    .dcod_op_div_unsigned_o           (dcod_op_div_unsigned), // DECODE & DECODE->EXE
    // FPU arithmmetic related
    .dcod_op_fp32_arith_o             (dcod_op_fp32_arith), // DECODE & DECODE->EXE
    .dcod_opc_fp32_arith_o            (dcod_opc_fp32_arith), // DECODE & DECODE->EXE
    // MTSPR / MFSPR
    .dcod_op_mfspr_o                  (dcod_op_mfspr), // DECODE & DECODE->EXE
    .dcod_op_mtspr_o                  (dcod_op_mtspr), // DECODE & DECODE->EXE
    // Exception flags
    //  ## enable l.trap exception
    .du_trap_enable_i                 (du_trap_enable), // DECODE & DECODE->EXE
    //  ## outcome exception flags
    .fetch_except_ibus_align_o        (fetch_except_ibus_align), // DECODE & DECODE->EXE
    .dcod_except_illegal_o            (dcod_except_illegal), // DECODE & DECODE->EXE
    .dcod_except_syscall_o            (dcod_except_syscall), // DECODE & DECODE->EXE
    .dcod_except_trap_o               (dcod_except_trap), // DECODE & DECODE->EXE
    // RFE proc
    .dcod_op_rfe_o                    (dcod_op_rfe) // DECODE & DECODE->EXE
  );


  //-------------------//
  // 32-bit multiplier //
  //-------------------//

  mor1kx_multiplier_marocchino
  #(
    .OPTION_OPERAND_WIDTH            (OPTION_OPERAND_WIDTH) // MUL
  )
  u_multiplier
  (
    // clocks & resets
    .clk                              (clk), // MUL
    .rst                              (rst), // MUL
    // pipeline controls
    .pipeline_flush_i                 (pipeline_flush), // MUL
    .padv_decode_i                    (padv_decode), // MUL
    .padv_wb_i                        (padv_wb), // MUL
    .grant_wb_to_mul_i                (grant_wb_to_mul), // MUL
    // input data
    //   ## from DECODE
    .dcod_rfa_i                       (dcod_rfa), // MUL
    .dcod_rfb_i                       (dcod_rfb), // MUL
    //   ## forwarding from WB
    .exe2dec_hazard_a_i               (exe2dec_hazard_a), // MUL
    .exe2dec_hazard_b_i               (exe2dec_hazard_b), // MUL
    .wb_result_i                      (wb_result), // MUL
    //  other inputs/outputs
    .dcod_op_mul_i                    (dcod_op_mul), // MUL
    .mul_busy_o                       (mul_busy), // MUL
    .mul_valid_o                      (mul_valid), // MUL
    .wb_mul_result_o                  (wb_mul_result) // MUL
  );


  //----------------//
  // 32-bit divider //
  //----------------//

  //  # update carry flag by division
  wire wb_div_carry_set;
  wire wb_div_carry_clear;

  //  # update overflow flag by division
  wire wb_div_overflow_set;
  wire wb_div_overflow_clear;

  // divisor instance
  mor1kx_divider_marocchino
  #(
    .OPTION_OPERAND_WIDTH             (OPTION_OPERAND_WIDTH) // DIV
  )
  u_divider
  (
    // clocks & resets
    .clk                              (clk), // DIV
    .rst                              (rst), // DIV
    // pipeline controls
    .pipeline_flush_i                 (pipeline_flush), // DIV
    .padv_decode_i                    (padv_decode), // DIV
    .padv_wb_i                        (padv_wb), // DIV
    .grant_wb_to_div_i                (grant_wb_to_div), // DIV
    // input data
    //   ## from DECODE
    .dcod_rfa_i                       (dcod_rfa), // DIV
    .dcod_rfb_i                       (dcod_rfb), // DIV
    //   ## forwarding from WB
    .exe2dec_hazard_a_i               (exe2dec_hazard_a), // DIV
    .exe2dec_hazard_b_i               (exe2dec_hazard_b), // DIV
    .wb_result_i                      (wb_result), // DIV
    // division command
    .dcod_op_div_i                    (dcod_op_div), // DIV
    .dcod_op_div_signed_i             (dcod_op_div_signed), // DIV
    .dcod_op_div_unsigned_i           (dcod_op_div_unsigned), // DIV
    // division engine state
    .div_busy_o                       (div_busy), // DIV
    .div_valid_o                      (div_valid), // DIV
    // write back
    //  # update carry flag by division
    .wb_div_carry_set_o               (wb_div_carry_set), // DIV
    .wb_div_carry_clear_o             (wb_div_carry_clear), // DIV
    //  # update overflow flag by division
    .wb_div_overflow_set_o            (wb_div_overflow_set), // DIV
    .wb_div_overflow_clear_o          (wb_div_overflow_clear), // DIV
    //  # generate overflow exception by division
    .except_overflow_enable_i         (except_overflow_enable), // DIV
    .wb_except_overflow_div_o         (wb_except_overflow_div), // DIV
    //  # division result
    .wb_div_result_o                  (wb_div_result) // DIV
  );


  //-----------------------------------------------//
  // 1-clock operations including FP-32 comparison //
  //-----------------------------------------------//

  //  # update carry flag by 1clk-operation
  wire wb_1clk_carry_set;
  wire wb_1clk_carry_clear;

  //  # update overflow flag by 1clk-operation
  wire wb_1clk_overflow_set;
  wire wb_1clk_overflow_clear;

  // 1clk instance
  mor1kx_exec_1clk_marocchino
  #(
    .OPTION_OPERAND_WIDTH             (OPTION_OPERAND_WIDTH), // 1CLK
    .OPTION_RF_ADDR_WIDTH             (OPTION_RF_ADDR_WIDTH), // 1CLK
    .FEATURE_FPU                      (FEATURE_FPU) // 1CLK
  )
  u_exec_1clk
  (
    // clocks & resets
    .clk                              (clk),
    .rst                              (rst),

    // pipeline controls
    .pipeline_flush_i                 (pipeline_flush), // 1CLK
    .padv_decode_i                    (padv_decode), // 1CLK
    .padv_wb_i                        (padv_wb), // 1CLK
    .grant_wb_to_1clk_i               (grant_wb_to_1clk), // 1CLK

    // input data
    //   from DECODE
    .dcod_rfa_i                       (dcod_rfa), // 1CLK
    .dcod_rfb_i                       (dcod_rfb), // 1CLK
    //   forwarding from WB
    .exe2dec_hazard_a_i               (exe2dec_hazard_a), // 1CLK
    .exe2dec_hazard_b_i               (exe2dec_hazard_b), // 1CLK
    .wb_result_i                      (wb_result), // 1CLK

    // 1-clock instruction auxiliaries
    .dcod_op_1clk_i                   (dcod_op_1clk), // 1CLK
    .op_1clk_busy_o                   (op_1clk_busy), // 1CLK
    .dcod_opc_alu_secondary_i         (dcod_opc_alu_secondary), // 1CLK
    .carry_i                          (ctrl_carry), // 1CLK
    .flag_i                           (ctrl_flag), // 1CLK

    // adder
    .dcod_op_add_i                    (dcod_op_add), // 1CLK
    .dcod_adder_do_sub_i              (dcod_adder_do_sub), // 1CLK
    .dcod_adder_do_carry_i            (dcod_adder_do_carry), // 1CLK
    // shift, ffl1, movhi, cmov
    .dcod_op_shift_i                  (dcod_op_shift), // 1CLK
    .dcod_op_ffl1_i                   (dcod_op_ffl1), // 1CLK
    .dcod_op_movhi_i                  (dcod_op_movhi), // 1CLK
    .dcod_op_cmov_i                   (dcod_op_cmov), // 1CLK
    // logic
    .dcod_opc_logic_i                 (dcod_opc_logic), // 1CLK
    // jump & link
    .dcod_op_jal_i                    (dcod_op_jal), // 1CLK
    .dcod_jal_result_i                (dcod_jal_result), // 1CLK
    // WB-latched 1-clock arithmetic result
    .wb_alu_1clk_result_o             (wb_alu_1clk_result), // 1CLK
    //  # update carry flag by 1clk-operation
    .wb_1clk_carry_set_o              (wb_1clk_carry_set), // 1CLK
    .wb_1clk_carry_clear_o            (wb_1clk_carry_clear), // 1CLK
    //  # update overflow flag by 1clk-operation
    .wb_1clk_overflow_set_o           (wb_1clk_overflow_set), // 1CLK
    .wb_1clk_overflow_clear_o         (wb_1clk_overflow_clear), // 1CLK
    //  # generate overflow exception by 1clk-operation
    .except_overflow_enable_i         (except_overflow_enable), // 1CLK
    .wb_except_overflow_1clk_o        (wb_except_overflow_1clk), // 1CLK

    // integer comparison flag
    .dcod_op_setflag_i                (dcod_op_setflag), // 1CLK
    // WB: integer comparison result
    .wb_int_flag_set_o                (wb_int_flag_set), // 1CLK
    .wb_int_flag_clear_o              (wb_int_flag_clear), // 1CLK

    // FP32 comparison flag
    .dcod_op_fp32_cmp_i               (dcod_op_fp32_cmp), // 1CLK
    .dcod_opc_fp32_cmp_i              (dcod_opc_fp32_cmp), // 1CLK
    .except_fpu_enable_i              (except_fpu_enable), // 1CLK
    .ctrl_fpu_mask_flags_inv_i        (ctrl_fpu_mask_flags[`OR1K_FPCSR_IVF - `OR1K_FPCSR_OVF]), // 1CLK
    .ctrl_fpu_mask_flags_inf_i        (ctrl_fpu_mask_flags[`OR1K_FPCSR_INF - `OR1K_FPCSR_OVF]), // 1CLK
    // WB: FP32 comparison results
    .wb_fp32_flag_set_o               (wb_fp32_flag_set), // 1CLK
    .wb_fp32_flag_clear_o             (wb_fp32_flag_clear), // 1CLK
    .wb_fp32_cmp_inv_o                (wb_fp32_cmp_inv), // 1CLK
    .wb_fp32_cmp_inf_o                (wb_fp32_cmp_inf), // 1CLK
    .wb_fp32_cmp_wb_fpcsr_o           (wb_fp32_cmp_wb_fpcsr), // 1CLK
    .wb_except_fp32_cmp_o             (wb_except_fp32_cmp), // 1CLK

    // Forwarding comparision flag result for conditional branch take/not
    .exec_op_1clk_cmp_o               (exec_op_1clk_cmp), // 1CLK
    .exec_flag_set_o                  (exec_flag_set) // 1CLK
  );


  //---------------------------//
  // FPU-32 arithmetic related //
  //---------------------------//
  generate
  /* verilator lint_off WIDTH */
  if (FEATURE_FPU != "NONE") begin :  alu_fp32_arith_ena
  /* verilator lint_on WIDTH */
    // fp32 arithmetic instance
    pfpu32_top_marocchino  u_pfpu32
    (
      // clock & reset
      .clk                      (clk), // FPU32_ARITH
      .rst                      (rst), // FPU32_ARITH

      // pipeline control inputs
      .flush_i                  (pipeline_flush), // FPU32_ARITH
      .padv_decode_i            (padv_decode), // FPU32_ARITH
      .padv_wb_i                (padv_wb), // FPU32_ARITH
      .grant_wb_to_fp32_arith_i (grant_wb_to_fp32_arith), // FPU32_ARITH

      // pipeline control outputs
      .fp32_arith_busy_o        (fp32_arith_busy), // FPU32_ARITH
      .fp32_arith_valid_o       (fp32_arith_valid), // FPU32_ARITH

      // Configuration
      .round_mode_i             (ctrl_fpu_round_mode), // FPU32_ARITH
      .except_fpu_enable_i      (except_fpu_enable), // FPU32_ARITH
      .ctrl_fpu_mask_flags_i    (ctrl_fpu_mask_flags), // FPU32_ARITH

      // Operands and commands
      .dcod_op_fp32_arith_i     (dcod_op_fp32_arith), // FPU32_ARITH
      .dcod_opc_fp32_arith_i    (dcod_opc_fp32_arith), // FPU32_ARITH
      //   from DECODE
      .dcod_rfa_i               (dcod_rfa), // FPU32_ARITH
      .dcod_rfb_i               (dcod_rfb), // FPU32_ARITH
      //   forwarding from WB
      .exe2dec_hazard_a_i       (exe2dec_hazard_a), // FPU32_ARITH
      .exe2dec_hazard_b_i       (exe2dec_hazard_b), // FPU32_ARITH
      .wb_result_i              (wb_result), // FPU32_ARITH

      // FPU-32 arithmetic part
      .wb_fp32_arith_res_o      (wb_fp32_arith_res), // FPU32_ARITH
      .wb_fp32_arith_fpcsr_o    (wb_fp32_arith_fpcsr), // FPU32_ARITH
      .wb_fp32_arith_wb_fpcsr_o (wb_fp32_arith_wb_fpcsr), // FPU32_ARITH
      .wb_except_fp32_arith_o   (wb_except_fp32_arith) // FPU32_ARITH
    );
  end
  else begin :  alu_fp32_arith_none
    assign fp32_arith_busy        = 1'b0;
    assign fp32_arith_valid       = 1'b0;
    assign wb_fp32_arith_res      = {OPTION_OPERAND_WIDTH{1'b0}};
    assign wb_fp32_arith_fpcsr    = {`OR1K_FPCSR_ALLF_SIZE{1'b0}};
    assign wb_fp32_arith_wb_fpcsr = 1'b0;
    assign wb_except_fp32_arith   = 1'b0;
  end // fpu_ena/fpu_none
  endgenerate // FPU arithmetic related


  //--------------//
  // LSU instance //
  //--------------//

  mor1kx_lsu_marocchino
  #(
    .OPTION_OPERAND_WIDTH               (OPTION_OPERAND_WIDTH),
    .OPTION_DCACHE_BLOCK_WIDTH          (OPTION_DCACHE_BLOCK_WIDTH),
    .OPTION_DCACHE_SET_WIDTH            (OPTION_DCACHE_SET_WIDTH),
    .OPTION_DCACHE_WAYS                 (OPTION_DCACHE_WAYS),
    .OPTION_DCACHE_LIMIT_WIDTH          (OPTION_DCACHE_LIMIT_WIDTH),
    .OPTION_DCACHE_SNOOP                (OPTION_DCACHE_SNOOP),
    .OPTION_DCACHE_CLEAR_ON_INIT        (OPTION_DCACHE_CLEAR_ON_INIT),
    .FEATURE_DMMU_HW_TLB_RELOAD         (FEATURE_DMMU_HW_TLB_RELOAD),
    .OPTION_DMMU_SET_WIDTH              (OPTION_DMMU_SET_WIDTH),
    .OPTION_DMMU_WAYS                   (OPTION_DMMU_WAYS),
    .OPTION_DMMU_CLEAR_ON_INIT          (OPTION_DMMU_CLEAR_ON_INIT),
    .OPTION_STORE_BUFFER_DEPTH_WIDTH    (OPTION_STORE_BUFFER_DEPTH_WIDTH),
    .OPTION_STORE_BUFFER_CLEAR_ON_INIT  (OPTION_STORE_BUFFER_CLEAR_ON_INIT)
  )
  u_lsu
  (
    // clocks & resets
    .clk                              (clk),
    .rst                              (rst),
    // Pipeline controls
    .pipeline_flush_i                 (pipeline_flush), // LSU
    .padv_decode_i                    (padv_decode), // LSU
    .padv_wb_i                        (padv_wb), // LSU
    .grant_wb_to_lsu_i                (grant_wb_to_lsu), // LSU
    // configuration
    .dc_enable_i                      (dc_enable), // LSU
    .dmmu_enable_i                    (dmmu_enable), // LSU
    .supervisor_mode_i                (supervisor_mode), // LSU
    // Input from DECODE (not latched)
    .dcod_delay_slot_i                (dcod_delay_slot), // LSU (for store buffer EPCR computation)
    .pc_decode_i                      (pc_decode), // LSU (for store buffer EPCR computation)
    .dcod_imm16_i                     (dcod_imm16), // LSU
    .dcod_rfa_i                       (dcod_rfa), // LSU
    .dcod_rfb_i                       (dcod_rfb), // LSU
    .dcod_op_lsu_load_i               (dcod_op_lsu_load), // LSU
    .dcod_op_lsu_store_i              (dcod_op_lsu_store), // LSU
    .dcod_op_lsu_atomic_i             (dcod_op_lsu_atomic), // LSU
    .dcod_lsu_length_i                (dcod_lsu_length), // LSU
    .dcod_lsu_zext_i                  (dcod_lsu_zext), // LSU
    .dcod_op_msync_i                  (dcod_op_msync), // LSU
    //   forwarding from WB
    .exe2dec_hazard_a_i               (exe2dec_hazard_a), // LSU
    .exe2dec_hazard_b_i               (exe2dec_hazard_b), // LSU
    .wb_result_i                      (wb_result), // LSU
    // inter-module interface
    .spr_bus_addr_i                   (spr_bus_addr_o), // LSU
    .spr_bus_we_i                     (spr_bus_we_o), // LSU
    .spr_bus_stb_i                    (spr_bus_stb_o), // LSU
    .spr_bus_dat_i                    (spr_bus_dat_o), // LSU
    .spr_bus_dat_dc_o                 (spr_bus_dat_dc), // LSU
    .spr_bus_ack_dc_o                 (spr_bus_ack_dc), // LSU
    .spr_bus_dat_dmmu_o               (spr_bus_dat_dmmu), // LSU
    .spr_bus_ack_dmmu_o               (spr_bus_ack_dmmu), // LSU
    // DBUS bridge interface
    .dbus_err_i                       (dbus_err_i), // LSU
    .dbus_ack_i                       (dbus_ack_i), // LSU
    .dbus_dat_i                       (dbus_dat_i[OPTION_OPERAND_WIDTH-1:0]), // LSU
    .dbus_adr_o                       (dbus_adr_o), // LSU
    .dbus_req_o                       (dbus_req_o), // LSU
    .dbus_dat_o                       (dbus_dat_o), // LSU
    .dbus_bsel_o                      (dbus_bsel_o[3:0]), // LSU
    .dbus_we_o                        (dbus_we_o), // LSU
    .dbus_burst_o                     (dbus_burst_o), // LSU
    // Cache sync for multi-core environment
    .snoop_adr_i                      (snoop_adr_i[31:0]), // LSU
    .snoop_en_i                       (snoop_en_i), // LSU
    // Exceprions & errors
    .sbuf_eear_o                      (sbuf_eear), // LSU
    .sbuf_epcr_o                      (sbuf_epcr), // LSU
    .sbuf_err_o                       (sbuf_err), // LSU
    // Outputs
    .lsu_busy_o                       (lsu_busy), // LSU
    .lsu_valid_o                      (lsu_valid), // LSU: result ready or exceptions
    .wb_lsu_result_o                  (wb_lsu_result), // LSU

    //  # particular LSU exception flags
    .wb_except_dbus_err_o             (wb_except_dbus_err), // LSU
    .wb_except_dpagefault_o           (wb_except_dpagefault), // LSU
    .wb_except_dtlb_miss_o            (wb_except_dtlb_miss), // LSU
    .wb_except_dbus_align_o           (wb_except_dbus_align), // LSU
    .wb_lsu_except_addr_o             (wb_lsu_except_addr), // LSU
    //  # combined LSU exceptions flag
    .wb_an_except_lsu_o               (wb_an_except_lsu), // LSU

    .wb_atomic_flag_set_o             (wb_atomic_flag_set), // LSU
    .wb_atomic_flag_clear_o           (wb_atomic_flag_clear) // LSU
  );


  //-----------//
  // WB:result //
  //-----------//

  assign wb_result =  wb_alu_1clk_result | wb_div_result     |
                      wb_mul_result      | wb_fp32_arith_res |
                      wb_lsu_result      | wb_mfspr_dat;


  //------------------------------------//
  // WB: External Interrupts Collection //
  //------------------------------------//
  wire exec_tt_interrupt  = tt_rdy       & tt_interrupt_enable  & exec_interrupts_en; // from "Tick Timer"
  wire exec_pic_interrupt = (|spr_picsr) & pic_interrupt_enable & exec_interrupts_en; // from "Programmble Interrupt Controller"
  // --- wb-latches ---
  always @(posedge clk `OR_ASYNC_RST) begin
    if (rst) begin
      wb_tt_interrupt_r   <= 1'b0;
      wb_pic_interrupt_r  <= 1'b0;
      wb_an_interrupt_r   <= 1'b0;
    end
    else if (pipeline_flush) begin  // WB: External Interrupts Collection
      wb_tt_interrupt_r   <= 1'b0;
      wb_pic_interrupt_r  <= 1'b0;
      wb_an_interrupt_r   <= 1'b0;
    end
    else if (padv_wb) begin         // WB: External Interrupts Collection
      wb_tt_interrupt_r   <= exec_tt_interrupt;
      wb_pic_interrupt_r  <= exec_pic_interrupt;
      wb_an_interrupt_r   <= (exec_tt_interrupt | exec_pic_interrupt);
    end
  end // @clock


  //-------------------------------//
  // Registers File (GPR) instance //
  //-------------------------------//

  mor1kx_rf_marocchino
  #(
    .OPTION_OPERAND_WIDTH     (OPTION_OPERAND_WIDTH), // RF
    .OPTION_RF_CLEAR_ON_INIT  (OPTION_RF_CLEAR_ON_INIT), // RF
    .OPTION_RF_ADDR_WIDTH     (OPTION_RF_ADDR_WIDTH), // RF
    .FEATURE_DEBUGUNIT        (FEATURE_DEBUGUNIT) // RF
  )
  u_rf
  (
    // clocks & resets
    .clk                              (clk),
    .rst                              (rst),
    // pipeline control signals
    .pipeline_flush_i                 (pipeline_flush), // RF
    // SPR bus
    .spr_bus_addr_i                   (spr_bus_addr_o), // RF
    .spr_bus_stb_i                    (spr_bus_stb_o), // RF
    .spr_bus_we_i                     (spr_bus_we_o), // RF
    .spr_bus_dat_i                    (spr_bus_dat_o), // RF
    .spr_bus_ack_gpr_o                (spr_bus_ack_gpr), // RF
    .spr_bus_dat_gpr_o                (spr_bus_dat_gpr), // RF
    // from FETCH
    .fetch_rf_adr_valid_i             (fetch_rf_adr_valid), // RF
    .fetch_rfa_adr_i                  (fetch_rfa_adr), // RF
    .fetch_rfb_adr_i                  (fetch_rfb_adr), // RF
    // from DECODE
    .dcod_rfa_req_i                   (dcod_rfa_req), // RF
    .dcod_rfa_adr_i                   (dcod_rfa_adr), // RF
    .dcod_rfb_req_i                   (dcod_rfb_req), // RF
    .dcod_rfb_adr_i                   (dcod_rfb_adr), // RF
    .dcod_immediate_i                 (dcod_immediate), // RF
    .dcod_immediate_sel_i             (dcod_immediate_sel), // RF
    // from WB
    .wb_rf_wb_i                       (wb_rf_wb), // RF
    .wb_rfd_adr_i                     (wb_rfd_adr), // RF
    .wb_result_i                      (wb_result), // RF
    // Outputs
    .dcod_rfa_o                       (dcod_rfa), // RF
    .dcod_rfb_o                       (dcod_rfb) // RF
  );


  mor1kx_oman_marocchino
  #(
    .OPTION_OPERAND_WIDTH (OPTION_OPERAND_WIDTH),
    .OPTION_RF_ADDR_WIDTH (OPTION_RF_ADDR_WIDTH)
  )
  u_oman
  (
    // clock & reset
    .clk                        (clk),
    .rst                        (rst),

    // pipeline control
    .padv_decode_i              (padv_decode), // OMAN
    .padv_wb_i                  (padv_wb), // OMAN
    .pipeline_flush_i           (pipeline_flush), // OMAN

    // DECODE non-latched flags to indicate next required unit
    // (The information is stored in order control buffer)
    .dcod_op_pass_exec_i        (dcod_op_pass_exec), // OMAN
    .dcod_jump_or_branch_i      (dcod_jump_or_branch), // OMAN
    .dcod_op_1clk_i             (dcod_op_1clk), // OMAN
    .dcod_op_div_i              (dcod_op_div), // OMAN
    .dcod_op_mul_i              (dcod_op_mul), // OMAN
    .dcod_op_fp32_arith_i       (dcod_op_fp32_arith), // OMAN
    .dcod_op_ls_i               (dcod_op_lsu_load | dcod_op_lsu_store), // OMAN
    .dcod_op_lsu_atomic_i       (dcod_op_lsu_atomic), // OMAN
    .dcod_op_rfe_i              (dcod_op_rfe), // OMAN

    // DECODE non-latched additional information related instruction
    //  part #1: iformation stored in order control buffer
    .dcod_delay_slot_i          (dcod_delay_slot), // OMAN
    .dcod_flag_wb_i             (dcod_flag_wb), // OMAN
    .dcod_carry_wb_i            (dcod_carry_wb), // OMAN
    .dcod_rf_wb_i               (dcod_rf_wb), // OMAN
    .dcod_rfd_adr_i             (dcod_rfd_adr), // OMAN
    .pc_decode_i                (pc_decode), // OMAN
    //  part #2: information required for data dependancy detection
    .dcod_rfa_req_i             (dcod_rfa_req), // OMAN
    .dcod_rfa_adr_i             (dcod_rfa_adr), // OMAN
    .dcod_rfb_req_i             (dcod_rfb_req), // OMAN
    .dcod_rfb_adr_i             (dcod_rfb_adr), // OMAN
    .dcod_flag_req_i            (dcod_flag_req), // OMAN
    .dcod_carry_req_i           (dcod_carry_req), // OMAN
    .dcod_op_jr_i               (dcod_op_jr), // OMAN
    .dcod_op_brcond_i           (dcod_op_brcond), // OMAN
    //  part #3: information required for create enable for
    //           for external (timer/ethernet/uart/etc) interrupts
    .dcod_op_lsu_store_i        (dcod_op_lsu_store), // OMAN
    .dcod_op_mtspr_i            (dcod_op_mtspr), // OMAN
    .dcod_op_msync_i            (dcod_op_msync), // OMAN
    //  part #4: for MF(T)SPR processing
    .dcod_op_mfspr_i            (dcod_op_mfspr), // OMAN

    // collect busy flags from exwcution module
    .op_1clk_busy_i             (op_1clk_busy), // OMAN
    .mul_busy_i                 (mul_busy), // OMAN
    .div_busy_i                 (div_busy), // OMAN
    .fp32_arith_busy_i          (fp32_arith_busy), // OMAN
    .lsu_busy_i                 (lsu_busy), // OMAN

    // collect valid flags from execution modules
    .div_valid_i                (div_valid), // OMAN
    .mul_valid_i                (mul_valid), // OMAN
    .fp32_arith_valid_i         (fp32_arith_valid), // OMAN
    .lsu_valid_i                (lsu_valid), // OMAN: result ready or exceptions

    // FETCH & DECODE exceptions
    .fetch_except_ibus_err_i    (fetch_except_ibus_err), // OMAN
    .fetch_except_ipagefault_i  (fetch_except_ipagefault), // OMAN
    .fetch_except_itlb_miss_i   (fetch_except_itlb_miss), // OMAN
    .fetch_except_ibus_align_i  (fetch_except_ibus_align), // OMAN
    .dcod_except_illegal_i      (dcod_except_illegal), // OMAN
    .dcod_except_syscall_i      (dcod_except_syscall), // OMAN
    .dcod_except_trap_i         (dcod_except_trap), // OMAN

    // EXECUTE-to-DECODE hazards
    .stall_fetch_o              (stall_fetch), // OMAN
    .exe2dec_hazard_a_o         (exe2dec_hazard_a), // OMAN
    .exe2dec_hazard_b_o         (exe2dec_hazard_b), // OMAN

    // DECODE result could be processed by EXECUTE
    .dcod_valid_o               (dcod_valid), // OMAN

    // EXECUTE completed (desired unit is ready)
    .exec_valid_o               (exec_valid), // OMAN

    // control WB latches of execution modules
    .grant_wb_to_1clk_o         (grant_wb_to_1clk), // OMAN
    .grant_wb_to_div_o          (grant_wb_to_div), // OMAN
    .grant_wb_to_mul_o          (grant_wb_to_mul), // OMAN
    .grant_wb_to_fp32_arith_o   (grant_wb_to_fp32_arith), // OMAN
    .grant_wb_to_lsu_o          (grant_wb_to_lsu), // OMAN

    // Support IBUS error handling in CTRL
    .exec_jump_or_branch_o      (exec_jump_or_branch), // OMAN
    .pc_exec_o                  (pc_exec), // OMAN

    //   Flag to enabel/disable exterlal interrupts processing
    // depending on the fact is instructions restartable or not
    .exec_interrupts_en_o       (exec_interrupts_en), // OMAN

    // WB outputs
    //  ## instruction related information
    .pc_wb_o                    (pc_wb), // OMAN
    .wb_delay_slot_o            (wb_delay_slot), // OMAN
    .wb_rfd_adr_o               (wb_rfd_adr), // OMAN
    .wb_rf_wb_o                 (wb_rf_wb), // OMAN
    //  ## RFE processing
    .wb_op_rfe_o                (wb_op_rfe), // OMAN
    //  ## IFETCH exceptions
    .wb_except_ibus_err_o       (wb_except_ibus_err), // OMAN
    .wb_except_ipagefault_o     (wb_except_ipagefault), // OMAN
    .wb_except_itlb_miss_o      (wb_except_itlb_miss), // OMAN
    .wb_except_ibus_align_o     (wb_except_ibus_align), // OMAN
    //  ## DECODE exceptions
    .wb_except_illegal_o        (wb_except_illegal), // OMAN
    .wb_except_syscall_o        (wb_except_syscall), // OMAN
    .wb_except_trap_o           (wb_except_trap), // OMAN
    //  ## combined DECODE/IFETCH exceptions
    .wb_fd_an_except_o          (wb_fd_an_except) // OMAN
  );


`ifndef SYNTHESIS
// synthesis translate_off
/* Debug signals required for the debug monitor
   endtask */
// synthesis translate_on
`endif

  //-------//
  // TIMER //
  //-------//
  //  # connection wires
  wire [31:0] spr_bus_dat_tt;
  wire        spr_bus_ack_tt;
  //  # timer instance
  mor1kx_ticktimer_oneself u_ticktimer
  (
    // clock and reset
    .clk                (clk), // TIMER
    .rst                (rst), // TIMER
    // ready flag
    .tt_rdy_o           (tt_rdy), // TIMER
    // SPR interface
    .spr_bus_addr_i     (spr_bus_addr_o), // TIMER
    .spr_bus_we_i       (spr_bus_we_o), // TIMER
    .spr_bus_stb_i      (spr_bus_stb_o), // TIMER
    .spr_bus_dat_i      (spr_bus_dat_o), // TIMER
    .spr_bus_dat_tt_o   (spr_bus_dat_tt), // TIMER
    .spr_bus_ack_tt_o   (spr_bus_ack_tt) // TIMER
  );


  //-----//
  // PIC //
  //-----//
  //  # connection wires
  wire [31:0] spr_bus_dat_pic;
  wire        spr_bus_ack_pic;
  //  # timer instance
  mor1kx_pic_oneself
  #(
    .OPTION_PIC_TRIGGER   (OPTION_PIC_TRIGGER), // PIC
    .OPTION_PIC_NMI_WIDTH (OPTION_PIC_NMI_WIDTH) // PIC
  )
  u_pic
  (
    // clock and reset
    .clk                (clk), // PIC
    .rst                (rst), // PIC
    // input interrupt lines
    .irq_i              (irq_i), // PIC
    // output interrupt lines
    .spr_picsr_o        (spr_picsr), // PIC
    // SPR BUS
    //  # inputs
    .spr_bus_addr_i     (spr_bus_addr_o), // PIC
    .spr_bus_we_i       (spr_bus_we_o), // PIC
    .spr_bus_stb_i      (spr_bus_stb_o), // PIC
    .spr_bus_dat_i      (spr_bus_dat_o), // PIC
    //  # outputs
    .spr_bus_dat_pic_o  (spr_bus_dat_pic), // PIC
    .spr_bus_ack_pic_o  (spr_bus_ack_pic) // PIC
  );


  //------//
  // CTRL //
  //------//
  mor1kx_ctrl_marocchino
  #(
    .OPTION_OPERAND_WIDTH       (OPTION_OPERAND_WIDTH), // CTRL
    .OPTION_RESET_PC            (OPTION_RESET_PC), // CTRL
    .OPTION_PIC_TRIGGER         (OPTION_PIC_TRIGGER), // CTRL
    .OPTION_DCACHE_BLOCK_WIDTH  (OPTION_DCACHE_BLOCK_WIDTH), // CTRL
    .OPTION_DCACHE_SET_WIDTH    (OPTION_DCACHE_SET_WIDTH), // CTRL
    .OPTION_DCACHE_WAYS         (OPTION_DCACHE_WAYS), // CTRL
    .OPTION_DMMU_SET_WIDTH      (OPTION_DMMU_SET_WIDTH), // CTRL
    .OPTION_DMMU_WAYS           (OPTION_DMMU_WAYS), // CTRL
    .OPTION_ICACHE_BLOCK_WIDTH  (OPTION_ICACHE_BLOCK_WIDTH), // CTRL
    .OPTION_ICACHE_SET_WIDTH    (OPTION_ICACHE_SET_WIDTH), // CTRL
    .OPTION_ICACHE_WAYS         (OPTION_ICACHE_WAYS), // CTRL
    .OPTION_IMMU_SET_WIDTH      (OPTION_IMMU_SET_WIDTH), // CTRL
    .OPTION_IMMU_WAYS           (OPTION_IMMU_WAYS), // CTRL
    .FEATURE_DEBUGUNIT          (FEATURE_DEBUGUNIT), // CTRL
    .FEATURE_PERFCOUNTERS       (FEATURE_PERFCOUNTERS), // CTRL
    .FEATURE_MAC                ("NONE"), // CTRL
    .FEATURE_FPU                (FEATURE_FPU), // CTRL
    .FEATURE_MULTICORE          (FEATURE_MULTICORE) // CTRL
  )
  u_ctrl
  (
    // clocks & resets
    .clk (clk),
    .rst (rst),

    // Inputs / Outputs for pipeline control signals
    .dcod_insn_valid_i                (dcod_insn_valid), // CTRL
    .stall_fetch_i                    (stall_fetch), // CTRL
    .dcod_valid_i                     (dcod_valid), // CTRL
    .exec_valid_i                     (exec_valid), // CTRL
    .pipeline_flush_o                 (pipeline_flush), // CTRL
    .padv_fetch_o                     (padv_fetch), // CTRL
    .padv_decode_o                    (padv_decode), // CTRL
    .padv_wb_o                        (padv_wb), // CTRL

    // MF(T)SPR coomand processing
    //  ## iput data & command from DECODE
    .dcod_rfa_i                       (dcod_rfa), // CTRL: part of addr for MT(F)SPR
    .dcod_imm16_i                     (dcod_imm16), // CTRL: part of addr for MT(F)SPR
    .dcod_rfb_i                       (dcod_rfb), // CTRL: data for MTSPR
    .dcod_op_mfspr_i                  (dcod_op_mfspr), // CTRL
    .dcod_op_mtspr_i                  (dcod_op_mtspr), // CTRL
    //  ## result to WB_MUX
    .wb_mfspr_dat_o                   (wb_mfspr_dat), // CTRL: for WB_MUX

    // Track branch address for exception processing support
    .dcod_do_branch_i                 (dcod_do_branch), // CTRL
    .dcod_do_branch_target_i          (dcod_do_branch_target), // CTRL
    //.dcod_jump_or_branch_i            (dcod_jump_or_branch), // CTRL
    // Support IBUS error handling in CTRL
    .exec_jump_or_branch_i            (exec_jump_or_branch), // CTRL
    .pc_exec_i                        (pc_exec), // CTRL

    // Debug System accesses CPU SPRs through DU
    .du_addr_i                        (du_addr_i), // CTRL
    .du_stb_i                         (du_stb_i), // CTRL
    .du_dat_i                         (du_dat_i), // CTRL
    .du_we_i                          (du_we_i), // CTRL
    .du_dat_o                         (du_dat_o), // CTRL
    .du_ack_o                         (du_ack_o), // CTRL
    // Stall control from debug interface
    .du_stall_i                       (du_stall_i), // CTRL
    .du_stall_o                       (du_stall_o), // CTRL
    // Enable l.trap exception
    .du_trap_enable_o                 (du_trap_enable), // CTRL

    // SPR accesses to external units (cache, mmu, etc.)
    .spr_bus_addr_o                   (spr_bus_addr_o), // CTRL
    .spr_bus_we_o                     (spr_bus_we_o), // CTRL
    .spr_bus_stb_o                    (spr_bus_stb_o), // CTRL
    .spr_bus_dat_o                    (spr_bus_dat_o), // CTRL
    .spr_bus_dat_dc_i                 (spr_bus_dat_dc), // CTRL
    .spr_bus_ack_dc_i                 (spr_bus_ack_dc), // CTRL
    .spr_bus_dat_ic_i                 (spr_bus_dat_ic), // CTRL
    .spr_bus_ack_ic_i                 (spr_bus_ack_ic), // CTRL
    .spr_bus_dat_dmmu_i               (spr_bus_dat_dmmu), // CTRL
    .spr_bus_ack_dmmu_i               (spr_bus_ack_dmmu), // CTRL
    .spr_bus_dat_immu_i               (spr_bus_dat_immu), // CTRL
    .spr_bus_ack_immu_i               (spr_bus_ack_immu), // CTRL
    .spr_bus_dat_mac_i                ({OPTION_OPERAND_WIDTH{1'b0}}), // CTRL
    .spr_bus_ack_mac_i                (1'b0), // CTRL
    .spr_bus_dat_pmu_i                ({OPTION_OPERAND_WIDTH{1'b0}}), // CTRL
    .spr_bus_ack_pmu_i                (1'b0), // CTRL
    .spr_bus_dat_pcu_i                ({OPTION_OPERAND_WIDTH{1'b0}}), // CTRL
    .spr_bus_ack_pcu_i                (1'b0), // CTRL
    .spr_bus_dat_fpu_i                ({OPTION_OPERAND_WIDTH{1'b0}}), // CTRL
    .spr_bus_ack_fpu_i                (1'b0), // CTRL
    .spr_bus_dat_tt_i                 (spr_bus_dat_tt), // CTRL
    .spr_bus_ack_tt_i                 (spr_bus_ack_tt), // CTRL
    .spr_bus_dat_pic_i                (spr_bus_dat_pic), // CTRL
    .spr_bus_ack_pic_i                (spr_bus_ack_pic), // CTRL
    .spr_bus_dat_gpr_i                (spr_bus_dat_gpr), // CTRL
    .spr_bus_ack_gpr_i                (spr_bus_ack_gpr), // CTRL

    // WB: External Interrupt Collection
    .tt_interrupt_enable_o            (tt_interrupt_enable), // CTRL
    .pic_interrupt_enable_o           (pic_interrupt_enable), // CTRL
    .wb_tt_interrupt_i                (wb_tt_interrupt_r), // CTRL
    .wb_pic_interrupt_i               (wb_pic_interrupt_r), // CTRL
    .wb_an_interrupt_i                (wb_an_interrupt_r), // CTRL

    // WB: programm counter
    .pc_wb_i                          (pc_wb), // CTRL

    // WB: flag
    .wb_int_flag_set_i                (wb_int_flag_set), // CTRL
    .wb_int_flag_clear_i              (wb_int_flag_clear), // CTRL
    .wb_fp32_flag_set_i               (wb_fp32_flag_set), // CTRL
    .wb_fp32_flag_clear_i             (wb_fp32_flag_clear), // CTRL
    .wb_atomic_flag_set_i             (wb_atomic_flag_set), // CTRL
    .wb_atomic_flag_clear_i           (wb_atomic_flag_clear), // CTRL

    // WB: carry
    .wb_div_carry_set_i               (wb_div_carry_set), // CTRL
    .wb_div_carry_clear_i             (wb_div_carry_clear), // CTRL
    .wb_1clk_carry_set_i              (wb_1clk_carry_set), // CTRL
    .wb_1clk_carry_clear_i            (wb_1clk_carry_clear), // CTRL

    // WB: overflow
    .wb_div_overflow_set_i            (wb_div_overflow_set), // CTRL
    .wb_div_overflow_clear_i          (wb_div_overflow_clear), // CTRL
    .wb_1clk_overflow_set_i           (wb_1clk_overflow_set), // CTRL
    .wb_1clk_overflow_clear_i         (wb_1clk_overflow_clear), // CTRL

    //  # FPX32 related flags
    //    ## arithmetic part
    .wb_fp32_arith_fpcsr_i            (wb_fp32_arith_fpcsr), // CTRL
    .wb_fp32_arith_wb_fpcsr_i         (wb_fp32_arith_wb_fpcsr), // CTRL
    .wb_except_fp32_arith_i           (wb_except_fp32_arith), // CTRL
    //    ## comparison part
    .wb_fp32_cmp_inv_i                (wb_fp32_cmp_inv), // CTRL
    .wb_fp32_cmp_inf_i                (wb_fp32_cmp_inf), // CTRL
    .wb_fp32_cmp_wb_fpcsr_i           (wb_fp32_cmp_wb_fpcsr), // CTRL
    .wb_except_fp32_cmp_i             (wb_except_fp32_cmp), // CTRL

    //  # Excepion processing auxiliaries
    .sbuf_eear_i                      (sbuf_eear), // CTRL
    .sbuf_epcr_i                      (sbuf_epcr), // CTRL
    .sbuf_err_i                       (sbuf_err), // CTRL
    .wb_delay_slot_i                  (wb_delay_slot), // CTRL

    //  # particular IFETCH exception flags
    .wb_except_ibus_err_i             (wb_except_ibus_err), // CTRL
    .wb_except_itlb_miss_i            (wb_except_itlb_miss), // CTRL
    .wb_except_ipagefault_i           (wb_except_ipagefault), // CTRL
    .wb_except_ibus_align_i           (wb_except_ibus_align), // CTRL
    .wb_lsu_except_addr_i             (wb_lsu_except_addr), // CTRL
    //  # particular DECODE exception flags
    .wb_except_illegal_i              (wb_except_illegal), // CTRL
    .wb_except_syscall_i              (wb_except_syscall), // CTRL
    .wb_except_trap_i                 (wb_except_trap), // CTRL
    //  # combined DECODE/IFETCH exceptions flag
    .wb_fd_an_except_i                (wb_fd_an_except), // CTRL

    //  # particular LSU exception flags
    .wb_except_dbus_err_i             (wb_except_dbus_err), // CTRL
    .wb_except_dtlb_miss_i            (wb_except_dtlb_miss), // CTRL
    .wb_except_dpagefault_i           (wb_except_dpagefault), // CTRL
    .wb_except_dbus_align_i           (wb_except_dbus_align), // CTRL
    //  # combined LSU exceptions flag
    .wb_an_except_lsu_i               (wb_an_except_lsu), // CTRL

    //  # overflow exception processing
    .except_overflow_enable_o         (except_overflow_enable), // CTRL
    .wb_except_overflow_div_i         (wb_except_overflow_div), // CTRL
    .wb_except_overflow_1clk_i        (wb_except_overflow_1clk), // CTRL

    //  # Branch to exception/rfe processing address
    .ctrl_branch_exception_o          (ctrl_branch_exception), // CTRL
    .ctrl_branch_except_pc_o          (ctrl_branch_except_pc), // CTRL
    .fetch_exception_taken_i          (fetch_ecxeption_taken), // CTRL
    //  # l.rfe
    .wb_op_rfe_i                      (wb_op_rfe), // CTRL

    // Multicore related
    .multicore_coreid_i               (multicore_coreid_i), // CTRL
    .multicore_numcores_i             (multicore_numcores_i), // CTRL

    // Flag & Carry
    .ctrl_flag_o                      (ctrl_flag), // CTRL
    .ctrl_carry_o                     (ctrl_carry), // CTRL

    // Enable modules
    .ic_enable_o                      (ic_enable), // CTRL
    .immu_enable_o                    (immu_enable), // CTRL
    .dc_enable_o                      (dc_enable), // CTRL
    .dmmu_enable_o                    (dmmu_enable), // CTRL
    .supervisor_mode_o                (supervisor_mode), // CTRL

    // FPU rounding mode
    .except_fpu_enable_o              (except_fpu_enable), // CTRL
    .ctrl_fpu_mask_flags_o            (ctrl_fpu_mask_flags), // CTRL
    .ctrl_fpu_round_mode_o            (ctrl_fpu_round_mode) // CTRL
  );

/*
   reg [`OR1K_INSN_WIDTH-1:0] traceport_stage_dcod_insn;
   reg [`OR1K_INSN_WIDTH-1:0] traceport_stage_exec_insn;

   reg            traceport_waitexec;

   always @(posedge clk) begin
      if (FEATURE_TRACEPORT_EXEC != "NONE") begin
   if (rst) begin
      traceport_waitexec <= 0;
   end else begin
      if (padv_decode) begin
         traceport_stage_dcod_insn <= dcod_insn;
      end

      if (padv_execute) begin
         traceport_stage_exec_insn <= traceport_stage_dcod_insn;
      end

      if (ctrl_new_input) begin
         traceport_exec_insn_o <= traceport_stage_exec_insn;
      end

      traceport_exec_pc_o <= pc_ctrl;
      if (!traceport_waitexec) begin
         if (ctrl_new_input & !ctrl_bubble) begin
      if (exec_valid) begin
         traceport_exec_valid_o <= 1'b1;
      end else begin
         traceport_exec_valid_o <= 1'b0;
         traceport_waitexec <= 1'b1;
      end
         end else begin
      traceport_exec_valid_o <= 1'b0;
         end
      end else begin
         if (exec_valid) begin
      traceport_exec_valid_o <= 1'b1;
      traceport_waitexec <= 1'b0;
         end else begin
      traceport_exec_valid_o <= 1'b0;
         end
      end // else: !if(!traceport_waitexec)
   end // else: !if(rst)
      end else begin // if (FEATURE_TRACEPORT_EXEC != "NONE")
   traceport_stage_dcod_insn <= {`OR1K_INSN_WIDTH{1'b0}};
   traceport_stage_exec_insn <= {`OR1K_INSN_WIDTH{1'b0}};
   traceport_exec_insn_o <= {`OR1K_INSN_WIDTH{1'b0}};
   traceport_exec_pc_o <= 32'h0;
   traceport_exec_valid_o <= 1'b0;
      end
   end

   generate
      if (FEATURE_TRACEPORT_EXEC != "NONE") begin
   assign traceport_exec_wbreg_o = wb_rfd_adr;
   assign traceport_exec_wben_o = wb_rf_wb;
   assign traceport_exec_wbdata_o = wb_result;
      end else begin
   assign traceport_exec_wbreg_o = {OPTION_RF_ADDR_WIDTH{1'b0}};
   assign traceport_exec_wben_o = 1'b0;
   assign traceport_exec_wbdata_o = {OPTION_OPERAND_WIDTH{1'b0}};
      end
   endgenerate
*/
endmodule // mor1kx_cpu_marocchino
