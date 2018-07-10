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
//   Copyright (C) 2015 - 2018 Andrey Bacherov                        //
//                             avbacherov@opencores.org               //
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
  // write buffer
  parameter OPTION_STORE_BUFFER_DEPTH_WIDTH   = 4, // 16 taps
  parameter OPTION_STORE_BUFFER_CLEAR_ON_INIT = 0,
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
  // interrupt controller
  parameter OPTION_PIC_TRIGGER          = "LEVEL",
  parameter OPTION_PIC_NMI_WIDTH        = 0,
  // debug unit, performance counters, trace
  parameter FEATURE_DEBUGUNIT           = "NONE",
  parameter FEATURE_PERFCOUNTERS        = "NONE",
  parameter FEATURE_TRACEPORT_EXEC      = "NONE",
  // m-core
  parameter FEATURE_MULTICORE           = "NONE",
  parameter OPTION_RF_NUM_SHADOW_GPR    = 0,      // for multicore mostly
  // Redister File
  parameter OPTION_RF_CLEAR_ON_INIT     = 0,
  parameter OPTION_RF_ADDR_WIDTH        = 5,
  // starting PC
  parameter OPTION_RESET_PC             = {{(OPTION_OPERAND_WIDTH-13){1'b0}},
                                           `OR1K_RESET_VECTOR,8'd0},
  // arithmetic modules
  parameter FEATURE_DIVIDER             = "SERIAL",
  // special instructions
  parameter FEATURE_PSYNC               = "NONE",
  parameter FEATURE_CSYNC               = "NONE"
)
(
  // Wishbone clock and reset
  input                             wb_clk,
  input                             wb_rst,

  // CPU clock and reset
  input                             cpu_clk,
  input                             cpu_rst,
  // For lwa/swa
  output                            pipeline_flush_o,

  // Instruction bus
  input                             ibus_err_i,
  input                             ibus_ack_i,
  input      [`OR1K_INSN_WIDTH-1:0] ibus_dat_i,
  input                             ibus_burst_last_i,
  output [OPTION_OPERAND_WIDTH-1:0] ibus_adr_o,
  output                            ibus_req_o,
  output                            ibus_burst_o,

  // Data bus
  input                             dbus_err_i,
  input                             dbus_ack_i,
  input  [OPTION_OPERAND_WIDTH-1:0] dbus_dat_i,
  input                             dbus_burst_last_i,
  output [OPTION_OPERAND_WIDTH-1:0] dbus_adr_o,
  output [OPTION_OPERAND_WIDTH-1:0] dbus_dat_o,
  output                            dbus_req_o,
  output                      [3:0] dbus_bsel_o,
  output                            dbus_lwa_cmd_o, // atomic load
  output                            dbus_stna_cmd_o, // none-atomic store
  output                            dbus_swa_cmd_o, // atomic store
  output                            dbus_burst_o,
  // Other connections for lwa/swa support
  input                             dbus_atomic_flg_i,

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

  // SPR accesses to external units (cache, mmu, etc.)
  output [15:0]                     spr_bus_addr_o,
  output                            spr_bus_we_o,
  output                            spr_bus_stb_o,
  output [OPTION_OPERAND_WIDTH-1:0] spr_bus_dat_o,

  // trace report
  output reg                        traceport_exec_valid_o,
  output reg                 [31:0] traceport_exec_pc_o,
  output reg [`OR1K_INSN_WIDTH-1:0] traceport_exec_insn_o,
  output [OPTION_OPERAND_WIDTH-1:0] traceport_exec_wbdata_o,
  output [OPTION_RF_ADDR_WIDTH-1:0] traceport_exec_wbreg_o,
  output                            traceport_exec_wben_o,

  // multi-core
  input  [OPTION_OPERAND_WIDTH-1:0] multicore_coreid_i,
  input  [OPTION_OPERAND_WIDTH-1:0] multicore_numcores_i,
  input                      [31:0] snoop_adr_i,
  input                             snoop_en_i
);

  localparam DEST_EXTADR_WIDTH  = 3; // log2(Order Control Buffer depth)

  // branch predictor parameters
  localparam GSHARE_BITS_NUM      = 12;


  // Instruction PC
  wire [OPTION_OPERAND_WIDTH-1:0] pc_fetch;
  wire [OPTION_OPERAND_WIDTH-1:0] pc_decode;
  wire [OPTION_OPERAND_WIDTH-1:0] pc_wb;


  // IFETCH outputs for RF reading and DECODE
  //  # instruction word valid flag
  wire                            fetch_valid;
  //  # instruction is in delay slot
  wire                            fetch_delay_slot;
  //  # instruction word itsef
  wire     [`OR1K_INSN_WIDTH-1:0] fetch_insn;
  //  # operand addresses
  wire [OPTION_RF_ADDR_WIDTH-1:0] fetch_rfa1_adr;
  wire [OPTION_RF_ADDR_WIDTH-1:0] fetch_rfb1_adr;
  wire [OPTION_RF_ADDR_WIDTH-1:0] fetch_rfa2_adr;
  wire [OPTION_RF_ADDR_WIDTH-1:0] fetch_rfb2_adr;
  //  # copy #1 of operand addresses
  wire [OPTION_RF_ADDR_WIDTH-1:0] fetch_rfa1_adr_rf;
  wire [OPTION_RF_ADDR_WIDTH-1:0] fetch_rfb1_adr_rf;
  wire [OPTION_RF_ADDR_WIDTH-1:0] fetch_rfa2_adr_rf;
  wire [OPTION_RF_ADDR_WIDTH-1:0] fetch_rfb2_adr_rf;
  //  # destiny addresses
  wire [OPTION_RF_ADDR_WIDTH-1:0] fetch_rfd1_adr;
  wire [OPTION_RF_ADDR_WIDTH-1:0] fetch_rfd2_adr;


  // for RAT
  //  # allocated as D1
  wire                            ratin_rfd1_wb;
  wire [OPTION_RF_ADDR_WIDTH-1:0] ratin_rfd1_adr;
  //  # allocated as D2
  wire                            ratin_rfd2_wb;
  wire [OPTION_RF_ADDR_WIDTH-1:0] ratin_rfd2_adr;
  //  # operands requestes
  wire                            ratin_rfa1_req;
  wire                            ratin_rfb1_req;
  wire                            ratin_rfa2_req;
  wire                            ratin_rfb2_req;


  wire                            dcod_empty;


  wire                            wb_atomic_flag_set;
  wire                            wb_atomic_flag_clear;

  wire                            wb_int_flag_set;
  wire                            wb_int_flag_clear;

  wire                            ctrl_flag;
  wire                            ctrl_flag_sr;
  wire                            ctrl_carry;

  wire                            dcod_flag_wb; // instruction writes comparison flag
  wire                            dcod_carry_wb; // instruction writes carry flag

  wire                            dcod_op_mtspr;
  wire                            dcod_op_mXspr; // (l.mfspr | l.mtspr)


  // Write-back outputs per execution unit
  // !!! Copies are usefull mostly for FPGA implementation to simplify routing
  // !!! Don't acivate "Remove duplicate registers" option in
  // !!! MAROCCHINO_TODO: <determine optimal settings>
  //  # from 1-clock execution units
  wire [OPTION_OPERAND_WIDTH-1:0] wb_alu_1clk_result;
  wire [OPTION_OPERAND_WIDTH-1:0] wb_alu_1clk_result_cp1; // copy #1
  wire [OPTION_OPERAND_WIDTH-1:0] wb_alu_1clk_result_cp2; // copy #2
  wire [OPTION_OPERAND_WIDTH-1:0] wb_alu_1clk_result_cp3; // copy #3
  //  # from integer division execution unit
  wire [OPTION_OPERAND_WIDTH-1:0] wb_div_result;
  wire [OPTION_OPERAND_WIDTH-1:0] wb_div_result_cp1; // copy #1
  wire [OPTION_OPERAND_WIDTH-1:0] wb_div_result_cp2; // copy #2
  wire [OPTION_OPERAND_WIDTH-1:0] wb_div_result_cp3; // copy #3
  //  # from integer multiplier execution unit
  wire [OPTION_OPERAND_WIDTH-1:0] wb_mul_result;
  wire [OPTION_OPERAND_WIDTH-1:0] wb_mul_result_cp1; // copy #1
  wire [OPTION_OPERAND_WIDTH-1:0] wb_mul_result_cp2; // copy #2
  wire [OPTION_OPERAND_WIDTH-1:0] wb_mul_result_cp3; // copy #3
  //  # from FP32 execution unit
  wire [OPTION_OPERAND_WIDTH-1:0] wb_fpxx_arith_res_hi;
  wire [OPTION_OPERAND_WIDTH-1:0] wb_fpxx_arith_res_hi_cp1; // copy #1
  wire [OPTION_OPERAND_WIDTH-1:0] wb_fpxx_arith_res_hi_cp2; // copy #2
  wire [OPTION_OPERAND_WIDTH-1:0] wb_fpxx_arith_res_hi_cp3; // copy #3
  //  # from FP64 execution unit
  wire [OPTION_OPERAND_WIDTH-1:0] wb_fpxx_arith_res_lo;
  wire [OPTION_OPERAND_WIDTH-1:0] wb_fpxx_arith_res_lo_cp1; // copy #1
  wire [OPTION_OPERAND_WIDTH-1:0] wb_fpxx_arith_res_lo_cp2; // copy #2
  wire [OPTION_OPERAND_WIDTH-1:0] wb_fpxx_arith_res_lo_cp3; // copy #3
  //  # from LSU execution unit
  wire [OPTION_OPERAND_WIDTH-1:0] wb_lsu_result;
  wire [OPTION_OPERAND_WIDTH-1:0] wb_lsu_result_cp1; // copy #1
  wire [OPTION_OPERAND_WIDTH-1:0] wb_lsu_result_cp2; // copy #2
  wire [OPTION_OPERAND_WIDTH-1:0] wb_lsu_result_cp3; // copy #3
  //  # from CTRL execution unit
  wire [OPTION_OPERAND_WIDTH-1:0] wb_mfspr_result;
  wire [OPTION_OPERAND_WIDTH-1:0] wb_mfspr_result_cp1; // copy #1
  wire [OPTION_OPERAND_WIDTH-1:0] wb_mfspr_result_cp2; // copy #2
  wire [OPTION_OPERAND_WIDTH-1:0] wb_mfspr_result_cp3; // copy #3
  // Combined write-back outputs
  //  # regular result
  wire [OPTION_OPERAND_WIDTH-1:0] wb_result1;     // WB result combiner
  wire [OPTION_OPERAND_WIDTH-1:0] wb_result1_cp1; // copy #1
  wire [OPTION_OPERAND_WIDTH-1:0] wb_result1_cp2; // copy #2
  wire [OPTION_OPERAND_WIDTH-1:0] wb_result1_cp3; // copy #3
  //  # extention for FPU3264
  wire [OPTION_OPERAND_WIDTH-1:0] wb_result2;     // WB result combiner for FPU64
  wire [OPTION_OPERAND_WIDTH-1:0] wb_result2_cp1; // copy #1
  wire [OPTION_OPERAND_WIDTH-1:0] wb_result2_cp2; // copy #2
  wire [OPTION_OPERAND_WIDTH-1:0] wb_result2_cp3; // copy #3


  wire                            dcod_free;
  wire                            dcod_valid;
  wire                            exec_valid;
  wire                            lsu_valid;   // result ready or exceptions


  // 1-clock "WB to DECODE operand forwarding" flags
  //  # relative operand A1
  wire                            dcod_wb2dec_d1a1_fwd;
  wire                            dcod_wb2dec_d2a1_fwd;
  //  # relative operand B1
  wire                            dcod_wb2dec_d1b1_fwd;
  wire                            dcod_wb2dec_d2b1_fwd;
  //  # relative operand A2
  wire                            dcod_wb2dec_d1a2_fwd;
  wire                            dcod_wb2dec_d2a2_fwd;
  //  # relative operand B2
  wire                            dcod_wb2dec_d1b2_fwd;
  wire                            dcod_wb2dec_d2b2_fwd;


  wire [OPTION_OPERAND_WIDTH-1:0] dcod_rfa1;
  wire [OPTION_OPERAND_WIDTH-1:0] dcod_rfb1;
  wire [OPTION_OPERAND_WIDTH-1:0] dcod_immediate;
  wire                            dcod_immediate_sel;
  // Special case for l.jr/l.jalr
  wire [OPTION_OPERAND_WIDTH-1:0] dcod_rfb1_jr;
  // for FPU64:
  wire [OPTION_OPERAND_WIDTH-1:0] dcod_rfa2;
  wire [OPTION_OPERAND_WIDTH-1:0] dcod_rfb2;


  wire [OPTION_RF_ADDR_WIDTH-1:0] dcod_rfd1_adr;
  wire                            dcod_rfd1_wb;
  // for FPU64:
  wire [OPTION_RF_ADDR_WIDTH-1:0] dcod_rfd2_adr;
  wire                            dcod_rfd2_wb;


  // OMAN-to-DECODE hazards
  //  # relative operand A1
  wire                            omn2dec_hazard_d1a1;
  wire                            omn2dec_hazard_d2a1;
  wire    [DEST_EXTADR_WIDTH-1:0] omn2dec_extadr_dxa1;
  //  # relative operand B1
  wire                            omn2dec_hazard_d1b1;
  wire                            omn2dec_hazard_d2b1;
  wire    [DEST_EXTADR_WIDTH-1:0] omn2dec_extadr_dxb1;
  //  # relative operand A2
  wire                            omn2dec_hazard_d1a2;
  wire                            omn2dec_hazard_d2a2;
  wire    [DEST_EXTADR_WIDTH-1:0] omn2dec_extadr_dxa2;
  //  # relative operand B2
  wire                            omn2dec_hazard_d1b2;
  wire                            omn2dec_hazard_d2b2;
  wire    [DEST_EXTADR_WIDTH-1:0] omn2dec_extadr_dxb2;
  // Hazard could be resolving
  //  ## FLAG or CARRY
  wire                            wb_flag_wb;
  wire                            wb_carry_wb;
  //  ## A or B operand
  wire                            wb_rfd1_odd;
  //  ## for hazards resolution in RSRVS
  wire    [DEST_EXTADR_WIDTH-1:0] wb_extadr;

  // Special WB-controls for RF
  wire [OPTION_RF_ADDR_WIDTH-1:0] wb_rf_even_addr;
  wire                            wb_rf_even_wb;
  wire [OPTION_RF_ADDR_WIDTH-1:0] wb_rf_odd_addr;
  wire                            wb_rf_odd_wb;


  // Logic to support Jump / Branch taking
  //  # from FETCH
  //    ## jump/branch variants
  wire                            fetch_op_jimm;
  wire                            fetch_op_jr;
  wire                            fetch_op_bf;
  wire                            fetch_op_bnf;
  //    ## combined jump/branch flag
  wire                            fetch_op_jb;
  //    ## pc-relative target
  wire [OPTION_OPERAND_WIDTH-1:0] fetch_to_imm_target;
  //  ## l.jr / l.jalr  gathering target
  wire                            jr_gathering_target;
  //  ## support IBUS error handling in CTRL
  wire                            wb_jump_or_branch;
  //  ## do branch (pedicted or unconditional)
  wire                            do_branch;
  wire [OPTION_OPERAND_WIDTH-1:0] do_branch_target;
  //  ## branch prediction support
  wire [OPTION_OPERAND_WIDTH-1:0] after_ds_target;
  wire                            predict_miss;
  wire                      [1:0] bc_cnt_value;  // current value of saturation counter
  wire      [GSHARE_BITS_NUM-1:0] bc_cnt_radr;   // saturation counter ID
  wire                            bc_cnt_we;     // update saturation counter
  wire                      [1:0] bc_cnt_wdat;   // new saturation counter value
  wire      [GSHARE_BITS_NUM-1:0] bc_cnt_wadr;   // saturation counter id
  wire                            bc_hist_taken; // conditional branch really taken
  //  ## support NPC handling in CTRL
  wire                            wb_do_branch;
  wire [OPTION_OPERAND_WIDTH-1:0] wb_do_branch_target;


  // Delay slot
  wire                            dcod_delay_slot;
  wire                            wb_delay_slot;


  wire      [`OR1K_IMM_WIDTH-1:0] dcod_imm16;
  wire                            dcod_op_lsu_load;
  wire                            dcod_op_lsu_store;
  wire                            dcod_op_lsu_atomic;
  wire                      [1:0] dcod_lsu_length;
  wire                            dcod_lsu_zext;
  wire                            dcod_op_msync;
  wire                            dcod_op_lsu_any;
  wire [OPTION_OPERAND_WIDTH-1:0] dcod_sbuf_epcr; // EPCR for STORE_BUFFER exception
  wire                            lsu_free;
  wire                            grant_wb_to_lsu;


  // Instructions which push EXECUTION without extra conditions
  wire                            dcod_op_push_exec;
  // Instructions which push WRITE-BACK without extra conditions
  wire                            dcod_op_push_wb;


  // Reservation station for 1-clock execution units
  wire                            dcod_op_1clk;
  wire                            op_1clk_free;
  wire  [`OR1K_ALU_OPC_WIDTH-1:0] dcod_opc_alu_secondary;

  wire                            dcod_op_add;
  wire                            dcod_adder_do_sub;
  wire                            dcod_adder_do_carry;

  wire                            dcod_op_jal;

  wire                            dcod_op_shift;
  wire                            dcod_op_ffl1;
  wire                            dcod_op_movhi;
  wire                            dcod_op_cmov;

  wire                            dcod_op_logic;
  wire  [`OR1K_ALU_OPC_WIDTH-1:0] dcod_opc_logic;

  wire                            dcod_op_setflag;

  wire                            grant_wb_to_1clk;
  wire                            taking_1clk_op;


  // Divider
  wire                            dcod_op_div;
  wire                            dcod_op_div_signed;
  wire                            dcod_op_div_unsigned;
  wire                            div_valid;
  wire                            grant_wb_to_div;
  // Pipelined multiplier
  wire                            dcod_op_mul;
  wire                            mul_valid;
  wire                            grant_wb_to_mul;
  // Reservation station for integer MUL/DIV
  wire                            dcod_op_muldiv;
  wire                            muldiv_free;


  // FPU3264 arithmetic part
  wire                              dcod_op_fpxx_arith; // to OMAN and FPU3264_ARITH
  wire                              dcod_op_fp64_arith; // to OMAN and FPU3264_ARITH
  wire                              dcod_op_fpxx_add; // to FPU3264_ARITH
  wire                              dcod_op_fpxx_sub; // to FPU3264_ARITH
  wire                              dcod_op_fpxx_mul; // to FPU3264_ARITH
  wire                              dcod_op_fpxx_div; // to FPU3264_ARITH
  wire                              dcod_op_fpxx_i2f; // to FPU3264_ARITH
  wire                              dcod_op_fpxx_f2i; // to FPU3264_ARITH
  wire                              fpxx_arith_valid;
  wire                              grant_wb_to_fpxx_arith;
  wire                              exec_except_fpxx_arith;
  wire  [`OR1K_FPCSR_ALLF_SIZE-1:0] wb_fpxx_arith_fpcsr;    // only flags
  wire                              wb_fpxx_arith_wb_fpcsr; // update FPCSR
  wire                              wb_except_fpxx_arith;   // generate FPx exception by FPx flags
  // FPU3264 comparison part
  wire                              dcod_op_fpxx_cmp;
  wire                        [2:0] dcod_opc_fpxx_cmp;
  wire                              exec_op_fpxx_cmp;
  wire                        [2:0] exec_opc_fpxx_cmp;
  wire                              fpxx_cmp_valid;
  wire                              grant_wb_to_fpxx_cmp;
  wire                              exec_except_fpxx_cmp;
  wire                              wb_fpxx_flag_set;
  wire                              wb_fpxx_flag_clear;
  wire                              wb_fpxx_cmp_inv;
  wire                              wb_fpxx_cmp_inf;
  wire                              wb_fpxx_cmp_wb_fpcsr;
  wire                              wb_except_fpxx_cmp;
  // FPU3264 reservationstation controls
  wire                              dcod_op_fpxx_any;
  wire                              exec_op_fpxx_any;
  wire                              fpxx_free;
  wire                              fpxx_taking_op;


  wire [OPTION_OPERAND_WIDTH-1:0] sbuf_eear;
  wire [OPTION_OPERAND_WIDTH-1:0] sbuf_epcr;
  wire                            sbuf_err;


  // CTRL -> Unit on Wishbone clock (TT & PIC)
  wire                            spr_bus_toggle;

  // SPR access buses (Unit -> CTRL part)
  //   GPR[0]
  wire                            spr_bus_ack_gpr0;
  wire [OPTION_OPERAND_WIDTH-1:0] spr_bus_dat_gpr0;
  //   GPR [S]hadow
  wire                            spr_bus_ack_gprS;
  wire [OPTION_OPERAND_WIDTH-1:0] spr_bus_dat_gprS;
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
  wire padv_dcod;
  wire padv_exec;
  wire padv_wb;
  wire pipeline_flush;

  // For lwa/swa
  assign pipeline_flush_o = pipeline_flush;

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
  wire fetch_an_except;
  //  # pre-WB IFETCH exceptions (OMAN output)
  wire exec_except_ibus_err;
  wire exec_except_ipagefault;
  wire exec_except_itlb_miss;
  wire exec_except_ibus_align;
  //  # WB-latches for IFETCH exceptions (OMAN->CTRL)
  reg  wb_except_ibus_err_r;
  reg  wb_except_ipagefault_r;
  reg  wb_except_itlb_miss_r;
  reg  wb_except_ibus_align_r;

  // Exceptions: reported from DECODE to OMAN
  wire dcod_except_illegal;
  wire dcod_except_syscall;
  wire dcod_except_trap;
  // Enable l.trap exception
  wire du_trap_enable;
  // Exceptions: pre-WB DECODE exceptions (OMAN output)
  wire exec_except_illegal;
  wire exec_except_syscall;
  wire exec_except_trap;
  // Exceptions: latched by WB latches for processing in CONTROL-unit
  reg  wb_except_illegal_r;
  reg  wb_except_syscall_r;
  reg  wb_except_trap_r;

  //  # overflow exception
  wire except_overflow_enable;
  //    ## from division
  wire exec_except_overflow_div;
  wire wb_except_overflow_div;
  //    ## from 1-CLOCK
  wire exec_except_overflow_1clk;
  wire wb_except_overflow_1clk;

  // Exceptions: reported by LSU
  //  # particular LSU exception flags
  wire                            wb_except_dbus_err;
  wire                            wb_except_dpagefault;
  wire                            wb_except_dtlb_miss;
  wire                            wb_except_dbus_align;
  wire [OPTION_OPERAND_WIDTH-1:0] wb_lsu_except_addr;
  //  # combined LSU exceptions flag
  wire                            exec_an_except_lsu;


  // External Interrupts Collection
  //  # from "Tick Timer"
  wire        tt_rdy;
  wire        tt_interrupt_enable;
  //  # from "Programmble Interrupt Controller"
  wire        pic_rdy; // an interrupt
  wire        pic_interrupt_enable;
  //  # flag to enabel/disable exterlal interrupts processing
  //    depending on the fact is instructions restartable or not
  wire        exec_interrupts_en;
  //  # WB latches
  reg         wb_tt_interrupt_r;
  reg         wb_pic_interrupt_r;


  // Exeptions process:
  wire dcod_op_rfe;
  wire exec_op_rfe;
  reg  wb_op_rfe_r;
  wire ctrl_branch_exception;
  wire [OPTION_OPERAND_WIDTH-1:0] ctrl_branch_except_pc;
  //   exeptions process: fetch->ctrl
  wire fetch_ecxeption_taken;


  // Combined exception/interrupt flag
  wire exec_an_except;
  reg  wb_an_except_r;


  //----------------------------//
  // Instruction FETCH instance //
  //----------------------------//

  mor1kx_fetch_marocchino
  #(
    .OPTION_OPERAND_WIDTH             (OPTION_OPERAND_WIDTH), // FETCH
    .OPTION_RF_ADDR_WIDTH             (OPTION_RF_ADDR_WIDTH), // FETCH
    // branch predictor parameters
    .GSHARE_BITS_NUM                  (GSHARE_BITS_NUM), // FETCH
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
    .cpu_clk                          (cpu_clk), // FETCH
    .cpu_rst                          (cpu_rst), // FETCH

    // pipeline control
    .padv_fetch_i                     (padv_fetch), // FETCH
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
    .ibus_burst_last_i                (ibus_burst_last_i), // FETCH
    .ibus_req_o                       (ibus_req_o), // FETCH
    .ibus_adr_o                       (ibus_adr_o), // FETCH
    .ibus_burst_o                     (ibus_burst_o), // FETCH

    // Jump/Branch processing
    //  # jump/branch variants
    .fetch_op_jimm_o                  (fetch_op_jimm), // FETCH
    .fetch_op_jr_o                    (fetch_op_jr), // FETCH
    .fetch_op_bf_o                    (fetch_op_bf), // FETCH
    .fetch_op_bnf_o                   (fetch_op_bnf), // FETCH
    //  # combined jump/branch flag
    .fetch_op_jb_o                    (fetch_op_jb), // FETCH
    //  # "to immediate driven target"
    .fetch_to_imm_target_o            (fetch_to_imm_target), // FETCH
    //  # do branch (pedicted or unconditional)
    .do_branch_i                      (do_branch), // FETCH
    .do_branch_target_i               (do_branch_target), // FETCH
    .jr_gathering_target_i            (jr_gathering_target), // FETCH
    //  # branch prediction support
    .after_ds_target_o                (after_ds_target), // FETCH
    .predict_miss_i                   (predict_miss), // FETCH
    .bc_cnt_value_o                   (bc_cnt_value), // FETCH
    .bc_cnt_radr_o                    (bc_cnt_radr), // FETCH
    .bc_cnt_we_i                      (bc_cnt_we), // FETCH
    .bc_cnt_wdat_i                    (bc_cnt_wdat), // FETCH
    .bc_cnt_wadr_i                    (bc_cnt_wadr), // FETCH
    .bc_hist_taken_i                  (bc_hist_taken), // FETCH

    // DU/exception/rfe control transfer
    .ctrl_branch_exception_i          (ctrl_branch_exception), // FETCH
    .ctrl_branch_except_pc_i          (ctrl_branch_except_pc), // FETCH

    // to RF read
    //  # instruction word valid flag
    .fetch_valid_o                    (fetch_valid), // FETCH
    //  # instruction is in delay slot
    .fetch_delay_slot_o               (fetch_delay_slot), // FETCH
    //  # instruction word itsef
    .fetch_insn_o                     (fetch_insn), // FETCH
    //  # operand addresses
    .fetch_rfa1_adr_o                 (fetch_rfa1_adr), // FETCH
    .fetch_rfb1_adr_o                 (fetch_rfb1_adr), // FETCH
    .fetch_rfa2_adr_o                 (fetch_rfa2_adr), // FETCH
    .fetch_rfb2_adr_o                 (fetch_rfb2_adr), // FETCH
    //  # copy #1 of operand addresses
    .fetch_rfa1_adr_rf_o              (fetch_rfa1_adr_rf), // FETCH
    .fetch_rfb1_adr_rf_o              (fetch_rfb1_adr_rf), // FETCH
    .fetch_rfa2_adr_rf_o              (fetch_rfa2_adr_rf), // FETCH
    .fetch_rfb2_adr_rf_o              (fetch_rfb2_adr_rf), // FETCH
    //  # destiny addresses
    .fetch_rfd1_adr_o                 (fetch_rfd1_adr), // FETCH
    .fetch_rfd2_adr_o                 (fetch_rfd2_adr), // FETCH

    //  Exceptions
    .fetch_except_ibus_err_o          (fetch_except_ibus_err), // FETCH
    .fetch_except_itlb_miss_o         (fetch_except_itlb_miss), // FETCH
    .fetch_except_ipagefault_o        (fetch_except_ipagefault), // FETCH
    .fetch_an_except_o                (fetch_an_except), // FETCH
    .fetch_exception_taken_o          (fetch_ecxeption_taken), // FETCH

    // Instruction PC
    .pc_fetch_o                       (pc_fetch) // FETCH
  );


  //-------------------------------//
  // Registers File (GPR) instance //
  //-------------------------------//

  mor1kx_rf_marocchino
  #(
    .OPTION_OPERAND_WIDTH           (OPTION_OPERAND_WIDTH), // RF
    .OPTION_RF_CLEAR_ON_INIT        (OPTION_RF_CLEAR_ON_INIT), // RF
    .OPTION_RF_ADDR_WIDTH           (OPTION_RF_ADDR_WIDTH), // RF
    .FEATURE_DEBUGUNIT              (FEATURE_DEBUGUNIT), // RF
    .OPTION_RF_NUM_SHADOW_GPR       (OPTION_RF_NUM_SHADOW_GPR) // RF
  )
  u_rf
  (
    // clocks & resets
    .cpu_clk                          (cpu_clk), // RF
    .cpu_rst                          (cpu_rst), // RF
    // pipeline control signals
    .pipeline_flush_i                 (pipeline_flush), // RF
    .padv_dcod_i                      (padv_dcod), // RF
    // SPR bus
    .spr_bus_addr_i                   (spr_bus_addr_o), // RF
    .spr_bus_stb_i                    (spr_bus_stb_o), // RF
    .spr_bus_we_i                     (spr_bus_we_o), // RF
    .spr_bus_dat_i                    (spr_bus_dat_o), // RF
    .spr_bus_ack_gpr0_o               (spr_bus_ack_gpr0), // RF
    .spr_bus_dat_gpr0_o               (spr_bus_dat_gpr0), // RF
    .spr_bus_ack_gprS_o               (spr_bus_ack_gprS), // RF
    .spr_bus_dat_gprS_o               (spr_bus_dat_gprS), // RF
    // from FETCH
    .fetch_rfa1_adr_i                 (fetch_rfa1_adr_rf), // RF
    .fetch_rfb1_adr_i                 (fetch_rfb1_adr_rf), // RF
    // for FPU64
    .fetch_rfa2_adr_i                 (fetch_rfa2_adr_rf), // RF
    .fetch_rfb2_adr_i                 (fetch_rfb2_adr_rf), // RF
    // from DECODE
    .dcod_immediate_i                 (dcod_immediate), // RF
    .dcod_immediate_sel_i             (dcod_immediate_sel), // RF
    // Special WB-controls for RF
    .wb_rf_even_addr_i                (wb_rf_even_addr), // RF
    .wb_rf_even_wb_i                  (wb_rf_even_wb), // RF
    .wb_rf_odd_addr_i                 (wb_rf_odd_addr), // RF
    .wb_rf_odd_wb_i                   (wb_rf_odd_wb), // RF
    // from WB
    .wb_rfd1_odd_i                    (wb_rfd1_odd), // RF
    .wb_result1_i                     (wb_result1), // RF
    // for FPU64
    .wb_result2_i                     (wb_result2), // RF
    // 1-clock "WB to DECODE operand forwarding" flags
    //  # relative operand A1
    .dcod_wb2dec_d1a1_fwd_i           (dcod_wb2dec_d1a1_fwd), // RF
    .dcod_wb2dec_d2a1_fwd_i           (dcod_wb2dec_d2a1_fwd), // RF
    //  # relative operand B1
    .dcod_wb2dec_d1b1_fwd_i           (dcod_wb2dec_d1b1_fwd), // RF
    .dcod_wb2dec_d2b1_fwd_i           (dcod_wb2dec_d2b1_fwd), // RF
    //  # relative operand A2
    .dcod_wb2dec_d1a2_fwd_i           (dcod_wb2dec_d1a2_fwd), // RF
    .dcod_wb2dec_d2a2_fwd_i           (dcod_wb2dec_d2a2_fwd), // RF
    //  # relative operand B2
    .dcod_wb2dec_d1b2_fwd_i           (dcod_wb2dec_d1b2_fwd), // RF
    .dcod_wb2dec_d2b2_fwd_i           (dcod_wb2dec_d2b2_fwd), // RF
    // Operands
    .dcod_rfa1_o                      (dcod_rfa1), // RF
    .dcod_rfb1_o                      (dcod_rfb1), // RF
    .dcod_rfa2_o                      (dcod_rfa2), // RF
    .dcod_rfb2_o                      (dcod_rfb2), // RF
    // we use adder for l.jl/l.jalr to compute return address: (pc+8)
    .dcod_op_jal_i                    (dcod_op_jal), // RF
    .pc_decode_i                      (pc_decode), // RF
    // Special case for l.jr/l.jalr
    .dcod_rfb1_jr_o                   (dcod_rfb1_jr) // RF
  );


  //--------//
  // DECODE //
  //--------//

  mor1kx_decode_marocchino
  #(
    .OPTION_OPERAND_WIDTH             (OPTION_OPERAND_WIDTH), // DECODE
    .OPTION_RF_ADDR_WIDTH             (OPTION_RF_ADDR_WIDTH), // DECODE
    .FEATURE_PSYNC                    (FEATURE_PSYNC), // DECODE
    .FEATURE_CSYNC                    (FEATURE_CSYNC) // DECODE
  )
  u_decode
  (
    // clocks ans reset
    .cpu_clk                          (cpu_clk), // DECODE
    .cpu_rst                          (cpu_rst), // DECODE
    // pipeline controls
    .padv_dcod_i                      (padv_dcod), // DECODE
    .padv_exec_i                      (padv_exec), // DECODE
    .pipeline_flush_i                 (pipeline_flush), // DECODE
    // from IFETCH
    //  # instruction word valid flag
    .fetch_valid_i                    (fetch_valid), // DECODE
    //  # an exception
    .fetch_an_except_i                (fetch_an_except), // DECODE
    //  # instruction is in delay slot
    .fetch_delay_slot_i               (fetch_delay_slot), // DECODE
    //  # instruction word itsef
    .fetch_insn_i                     (fetch_insn), // DECODE
    //  # destiny addresses
    .fetch_rfd1_adr_i                 (fetch_rfd1_adr), // DECODE
    .fetch_rfd2_adr_i                 (fetch_rfd2_adr), // DECODE
    // for RAT
    //  # allocated as D1
    .ratin_rfd1_wb_o                  (ratin_rfd1_wb), // DECODE
    .ratin_rfd1_adr_o                 (ratin_rfd1_adr), // DECODE
    //  # allocated as D2
    .ratin_rfd2_wb_o                  (ratin_rfd2_wb), // DECODE
    .ratin_rfd2_adr_o                 (ratin_rfd2_adr), // DECODE
    //  # requested operands
    .ratin_rfa1_req_o                 (ratin_rfa1_req), // DECODE
    .ratin_rfb1_req_o                 (ratin_rfb1_req), // DECODE
    .ratin_rfa2_req_o                 (ratin_rfa2_req), // DECODE
    .ratin_rfb2_req_o                 (ratin_rfb2_req), // DECODE
    // latched instruction word and it's attributes
    .dcod_empty_o                     (dcod_empty), // DECODE
    .dcod_delay_slot_o                (dcod_delay_slot), // DECODE
    // destiny D1
    .dcod_rfd1_adr_o                  (dcod_rfd1_adr), // DECODE
    .dcod_rfd1_wb_o                   (dcod_rfd1_wb), // DECODE
    // destiny D2 (for FPU64)
    .dcod_rfd2_adr_o                  (dcod_rfd2_adr), // DECODE
    .dcod_rfd2_wb_o                   (dcod_rfd2_wb), // DECODE
    // instruction PC
    .pc_fetch_i                       (pc_fetch), // DECODE
    .pc_decode_o                      (pc_decode), // DECODE
    // IMM
    .dcod_immediate_o                 (dcod_immediate), // DECODE
    .dcod_immediate_sel_o             (dcod_immediate_sel), // DECODE
    // various instruction attributes
    .dcod_flag_wb_o                   (dcod_flag_wb), // DECODE
    .dcod_carry_wb_o                  (dcod_carry_wb), // DECODE
    // LSU related
    .dcod_imm16_o                     (dcod_imm16), // DECODE
    .dcod_op_lsu_load_o               (dcod_op_lsu_load), // DECODE
    .dcod_op_lsu_store_o              (dcod_op_lsu_store), // DECODE
    .dcod_op_lsu_atomic_o             (dcod_op_lsu_atomic), // DECODE
    .dcod_lsu_length_o                (dcod_lsu_length), // DECODE
    .dcod_lsu_zext_o                  (dcod_lsu_zext), // DECODE
    .dcod_op_msync_o                  (dcod_op_msync), // DECODE
    .dcod_op_lsu_any_o                (dcod_op_lsu_any), // DECODE
    // EPCR for store buffer. delay-slot ? (pc-4) : pc
    .dcod_sbuf_epcr_o                 (dcod_sbuf_epcr), // DECODE
    // Instructions which push EXECUTION without extra conditions
    .dcod_op_push_exec_o              (dcod_op_push_exec), // DECODE
    // Instructions which push WRITE-BACK without extra conditions
    .dcod_op_push_wb_o                (dcod_op_push_wb), // DECODE
    // 1-clock instruction
    .dcod_op_1clk_o                   (dcod_op_1clk), // DECODE
    // ALU related opc
    .dcod_opc_alu_secondary_o         (dcod_opc_alu_secondary), // DECODE
    // Adder related
    .dcod_op_add_o                    (dcod_op_add), // DECODE
    .dcod_adder_do_sub_o              (dcod_adder_do_sub), // DECODE
    .dcod_adder_do_carry_o            (dcod_adder_do_carry), // DECODE
    // Various 1-clock related
    .dcod_op_shift_o                  (dcod_op_shift), // DECODE
    .dcod_op_ffl1_o                   (dcod_op_ffl1), // DECODE
    .dcod_op_movhi_o                  (dcod_op_movhi), // DECODE
    .dcod_op_cmov_o                   (dcod_op_cmov), // DECODE
    // Logic
    .dcod_op_logic_o                  (dcod_op_logic), // DECODE
    .dcod_opc_logic_o                 (dcod_opc_logic), // DECODE
    // Jump & Link
    .dcod_op_jal_o                    (dcod_op_jal), // DECODE
    // Set flag related
    .dcod_op_setflag_o                (dcod_op_setflag), // DECODE
    // Multiplier related
    .dcod_op_mul_o                    (dcod_op_mul), // DECODE
    // Divider related
    .dcod_op_div_o                    (dcod_op_div), // DECODE
    .dcod_op_div_signed_o             (dcod_op_div_signed), // DECODE
    .dcod_op_div_unsigned_o           (dcod_op_div_unsigned), // DECODE
    // Combined for MULDIV_RSRVS
    .dcod_op_muldiv_o                 (dcod_op_muldiv), // DECODE
    // FPU-64 arithmetic part
    .dcod_op_fpxx_arith_o             (dcod_op_fpxx_arith), // DECODE
    .dcod_op_fp64_arith_o             (dcod_op_fp64_arith), // DECODE
    .dcod_op_fpxx_add_o               (dcod_op_fpxx_add), // DECODE
    .dcod_op_fpxx_sub_o               (dcod_op_fpxx_sub), // DECODE
    .dcod_op_fpxx_mul_o               (dcod_op_fpxx_mul), // DECODE
    .dcod_op_fpxx_div_o               (dcod_op_fpxx_div), // DECODE
    .dcod_op_fpxx_i2f_o               (dcod_op_fpxx_i2f), // DECODE
    .dcod_op_fpxx_f2i_o               (dcod_op_fpxx_f2i), // DECODE
    // FPU-64 comparison part
    .dcod_op_fpxx_cmp_o               (dcod_op_fpxx_cmp), // DECODE
    .dcod_opc_fpxx_cmp_o              (dcod_opc_fpxx_cmp), // DECODE
    // Combined for FPU_RSRVS
    .dcod_op_fpxx_any_o               (dcod_op_fpxx_any), // DECODE
    // MTSPR / MFSPR
    .dcod_op_mtspr_o                  (dcod_op_mtspr), // DECODE
    .dcod_op_mXspr_o                  (dcod_op_mXspr), // DECODE
    // Exception flags
    //  ## enable l.trap exception
    .du_trap_enable_i                 (du_trap_enable), // DECODE
    //  ## outcome exception flags
    .dcod_except_illegal_o            (dcod_except_illegal), // DECODE
    .dcod_except_syscall_o            (dcod_except_syscall), // DECODE
    .dcod_except_trap_o               (dcod_except_trap), // DECODE
    // RFE proc
    .dcod_op_rfe_o                    (dcod_op_rfe) // DECODE
  );


  //-------------------//
  // [O]rder [MAN]ager //
  //-------------------//

  mor1kx_oman_marocchino
  #(
    .OPTION_OPERAND_WIDTH       (OPTION_OPERAND_WIDTH), // OMAN
    .OPTION_RF_ADDR_WIDTH       (OPTION_RF_ADDR_WIDTH), // OMAN
    .DEST_EXTADR_WIDTH          (DEST_EXTADR_WIDTH), // OMAN
    // branch predictor parameters
    .GSHARE_BITS_NUM            (GSHARE_BITS_NUM) // OMAN
  )
  u_oman
  (
    // clock & reset
    .cpu_clk                    (cpu_clk), // OMAN
    .cpu_rst                    (cpu_rst), // OMAN

    // pipeline control
    .padv_dcod_i                (padv_dcod), // OMAN
    .padv_exec_i                (padv_exec), // OMAN
    .padv_wb_i                  (padv_wb), // OMAN
    .pipeline_flush_i           (pipeline_flush), // OMAN

    // fetched instruction is valid
    .fetch_valid_i              (fetch_valid), // OMAN
    .fetch_delay_slot_i         (fetch_delay_slot), // OMAN

    // for RAT
    //  # allocated as D1
    .ratin_rfd1_wb_i            (ratin_rfd1_wb), // OMAN
    .ratin_rfd1_adr_i           (ratin_rfd1_adr), // OMAN
    //  # allocated as D2
    .ratin_rfd2_wb_i            (ratin_rfd2_wb), // OMAN
    .ratin_rfd2_adr_i           (ratin_rfd2_adr), // OMAN
    //  # requested operands
    // operand A1
    .ratin_rfa1_req_i           (ratin_rfa1_req), // OMAN
    .fetch_rfa1_adr_i           (fetch_rfa1_adr), // OMAN
    // operand B1
    .ratin_rfb1_req_i           (ratin_rfb1_req), // OMAN
    .fetch_rfb1_adr_i           (fetch_rfb1_adr), // OMAN
    // operand A2 (for FPU64)
    .ratin_rfa2_req_i           (ratin_rfa2_req), // OMAN
    .fetch_rfa2_adr_i           (fetch_rfa2_adr), // OMAN
    // operand B2 (for FPU64)
    .ratin_rfb2_req_i           (ratin_rfb2_req), // OMAN
    .fetch_rfb2_adr_i           (fetch_rfb2_adr), // OMAN

    // DECODE non-latched flags to indicate next required unit
    // (The information is stored in order control buffer)
    .dcod_op_push_exec_i        (dcod_op_push_exec), // OMAN
    .dcod_op_push_wb_i          (dcod_op_push_wb), // OMAN
    .fetch_op_jb_i              (fetch_op_jb), // OMAN
    .dcod_op_1clk_i             (dcod_op_1clk), // OMAN
    .dcod_op_div_i              (dcod_op_div), // OMAN
    .dcod_op_mul_i              (dcod_op_mul), // OMAN
    .dcod_op_fpxx_arith_i       (dcod_op_fpxx_arith), // OMAN
    .dcod_op_ls_i               (dcod_op_lsu_load | dcod_op_lsu_store), // OMAN
    .dcod_op_rfe_i              (dcod_op_rfe), // OMAN
    // for FPU64
    .dcod_op_fpxx_cmp_i         (dcod_op_fpxx_cmp), // OMAN

    // DECODE non-latched additional information related instruction
    //  part #1: iformation stored in order control buffer
    .pc_decode_i                (pc_decode), // OMAN
    .dcod_rfd1_adr_i            (dcod_rfd1_adr), // OMAN
    .dcod_rfd1_wb_i             (dcod_rfd1_wb), // OMAN
    .dcod_carry_wb_i            (dcod_carry_wb), // OMAN
    .dcod_flag_wb_i             (dcod_flag_wb), // OMAN
    .dcod_delay_slot_i          (dcod_delay_slot), // OMAN
    .dcod_rfd2_adr_i            (dcod_rfd2_adr), // OMAN for FPU64
    .dcod_rfd2_wb_i             (dcod_rfd2_wb), // OMAN for FPU64
    //  part #2: information required for create enable for
    //           for external (timer/ethernet/uart/etc) interrupts
    .dcod_op_lsu_store_i        (dcod_op_lsu_store), // OMAN
    .dcod_op_msync_i            (dcod_op_msync), // OMAN
    .dcod_op_mXspr_i            (dcod_op_mXspr), // OMAN

    // for unit hazard detection
    .op_1clk_free_i             (op_1clk_free), // OMAN
    .dcod_op_muldiv_i           (dcod_op_muldiv), // OMAN
    .muldiv_free_i              (muldiv_free), // OMAN
    .dcod_op_fpxx_any_i         (dcod_op_fpxx_any), // OMAN
    .fpxx_free_i                (fpxx_free), // OMAN
    .dcod_op_lsu_any_i          (dcod_op_lsu_any), // OMAN (load | store | l.msync)
    .lsu_free_i                 (lsu_free), // OMAN

    // collect valid flags from execution modules
    .div_valid_i                (div_valid), // OMAN
    .mul_valid_i                (mul_valid), // OMAN
    .fpxx_arith_valid_i         (fpxx_arith_valid), // OMAN
    .fpxx_cmp_valid_i           (fpxx_cmp_valid), // OMAN
    .lsu_valid_i                (lsu_valid), // OMAN: result ready or exceptions

    // FETCH & DECODE exceptions
    .fetch_except_ibus_err_i    (fetch_except_ibus_err), // OMAN
    .fetch_except_ipagefault_i  (fetch_except_ipagefault), // OMAN
    .fetch_except_itlb_miss_i   (fetch_except_itlb_miss), // OMAN
    .dcod_except_illegal_i      (dcod_except_illegal), // OMAN
    .dcod_except_syscall_i      (dcod_except_syscall), // OMAN
    .dcod_except_trap_i         (dcod_except_trap), // OMAN

    // 1-clock "WB to DECODE operand forwarding" flags
    //  # relative operand A1
    .dcod_wb2dec_d1a1_fwd_o     (dcod_wb2dec_d1a1_fwd), // OMAN
    .dcod_wb2dec_d2a1_fwd_o     (dcod_wb2dec_d2a1_fwd), // OMAN
    //  # relative operand B1
    .dcod_wb2dec_d1b1_fwd_o     (dcod_wb2dec_d1b1_fwd), // OMAN
    .dcod_wb2dec_d2b1_fwd_o     (dcod_wb2dec_d2b1_fwd), // OMAN
    //  # relative operand A2
    .dcod_wb2dec_d1a2_fwd_o     (dcod_wb2dec_d1a2_fwd), // OMAN
    .dcod_wb2dec_d2a2_fwd_o     (dcod_wb2dec_d2a2_fwd), // OMAN
    //  # relative operand B2
    .dcod_wb2dec_d1b2_fwd_o     (dcod_wb2dec_d1b2_fwd), // OMAN
    .dcod_wb2dec_d2b2_fwd_o     (dcod_wb2dec_d2b2_fwd), // OMAN

    // OMAN-to-DECODE hazards
    //  # relative operand A1
    .omn2dec_hazard_d1a1_o      (omn2dec_hazard_d1a1), // OMAN
    .omn2dec_hazard_d2a1_o      (omn2dec_hazard_d2a1), // OMAN
    .omn2dec_extadr_dxa1_o      (omn2dec_extadr_dxa1), // OMAN
    //  # relative operand B1
    .omn2dec_hazard_d1b1_o      (omn2dec_hazard_d1b1), // OMAN
    .omn2dec_hazard_d2b1_o      (omn2dec_hazard_d2b1), // OMAN
    .omn2dec_extadr_dxb1_o      (omn2dec_extadr_dxb1), // OMAN
    //  # relative operand A2
    .omn2dec_hazard_d1a2_o      (omn2dec_hazard_d1a2), // OMAN
    .omn2dec_hazard_d2a2_o      (omn2dec_hazard_d2a2), // OMAN
    .omn2dec_extadr_dxa2_o      (omn2dec_extadr_dxa2), // OMAN
    //  # relative operand B2
    .omn2dec_hazard_d1b2_o      (omn2dec_hazard_d1b2), // OMAN
    .omn2dec_hazard_d2b2_o      (omn2dec_hazard_d2b2), // OMAN
    .omn2dec_extadr_dxb2_o      (omn2dec_extadr_dxb2), // OMAN

    // DECODE result could be processed by EXECUTE
    .dcod_free_o                (dcod_free), // OMAN
    .dcod_valid_o               (dcod_valid), // OMAN

    // EXECUTE completed (desired unit is ready)
    .exec_valid_o               (exec_valid), // OMAN

    // control WB latches of execution modules
    .grant_wb_to_1clk_o         (grant_wb_to_1clk), // OMAN
    .grant_wb_to_div_o          (grant_wb_to_div), // OMAN
    .grant_wb_to_mul_o          (grant_wb_to_mul), // OMAN
    .grant_wb_to_fpxx_arith_o   (grant_wb_to_fpxx_arith), // OMAN
    .grant_wb_to_lsu_o          (grant_wb_to_lsu), // OMAN
    // for FPU64
    .grant_wb_to_fpxx_cmp_o     (grant_wb_to_fpxx_cmp), // OMAN

    // Logic to support Jump / Branch taking
    //    ## jump/branch variants
    .fetch_op_jimm_i            (fetch_op_jimm), // OMAN
    .fetch_op_jr_i              (fetch_op_jr), // OMAN
    .fetch_op_bf_i              (fetch_op_bf), // OMAN
    .fetch_op_bnf_i             (fetch_op_bnf), // OMAN
    //    ## "to immediate driven target"
    .fetch_to_imm_target_i      (fetch_to_imm_target), // OMAN
    // register target
    .dcod_rfb1_jr_i             (dcod_rfb1_jr), // OMAN
    .wb_result1_i               (wb_result1), // OMAN
    // comparision flag for l.bf / l.bnf
    .ctrl_flag_sr_i             (ctrl_flag_sr), // OMAN
    // jump/branch signals to IFETCH
    .do_branch_o                (do_branch), // OMAN
    .do_branch_target_o         (do_branch_target), // OMAN
    .jr_gathering_target_o      (jr_gathering_target), // OMAN
    //  # branch prediction support
    .after_ds_target_i          (after_ds_target), // OMAN
    .predict_miss_o             (predict_miss), // OMAN
    .bc_cnt_value_i             (bc_cnt_value), // OMAN
    .bc_cnt_radr_i              (bc_cnt_radr), // OMAN
    .bc_cnt_we_o                (bc_cnt_we), // OMAN
    .bc_cnt_wdat_o              (bc_cnt_wdat), // OMAN
    .bc_cnt_wadr_o              (bc_cnt_wadr), // OMAN
    .bc_hist_taken_o            (bc_hist_taken), // OMAN
    // Support IBUS error handling in CTRL
    .wb_jump_or_branch_o        (wb_jump_or_branch), // OMAN
    .wb_do_branch_o             (wb_do_branch), // OMAN
    .wb_do_branch_target_o      (wb_do_branch_target), // OMAN

    //   Flag to enabel/disable exterlal interrupts processing
    // depending on the fact is instructions restartable or not
    .exec_interrupts_en_o       (exec_interrupts_en), // OMAN

    // pre-WB l.rfe
    .exec_op_rfe_o              (exec_op_rfe), // OMAN
    // pre-WB output exceptions: IFETCH
    .exec_except_ibus_err_o     (exec_except_ibus_err), // OMAN
    .exec_except_ipagefault_o   (exec_except_ipagefault), // OMAN
    .exec_except_itlb_miss_o    (exec_except_itlb_miss), // OMAN
    .exec_except_ibus_align_o   (exec_except_ibus_align), // OMAN
    // pre-WB output exceptions: DECODE
    .exec_except_illegal_o      (exec_except_illegal), // OMAN
    .exec_except_syscall_o      (exec_except_syscall), // OMAN
    .exec_except_trap_o         (exec_except_trap), // OMAN

    // WB outputs
    //  ## special WB-controls for RF
    .wb_rf_even_addr_o          (wb_rf_even_addr), // OMAN
    .wb_rf_even_wb_o            (wb_rf_even_wb), // OMAN
    .wb_rf_odd_addr_o           (wb_rf_odd_addr), // OMAN
    .wb_rf_odd_wb_o             (wb_rf_odd_wb), // OMAN
    //  ## instruction related information
    .pc_wb_o                    (pc_wb), // OMAN
    .wb_delay_slot_o            (wb_delay_slot), // OMAN
    .wb_rfd1_odd_o              (wb_rfd1_odd), // OMAN
    .wb_flag_wb_o               (wb_flag_wb), // OMAN
    .wb_carry_wb_o              (wb_carry_wb), // OMAN
    // for hazards resolution in RSRVS
    .wb_extadr_o                (wb_extadr) // OMAN
  );


  //--------------------//
  // 1-clock operations //
  //--------------------//

  // single clock operations controls
  //  # commands
  wire                           exec_op_ffl1;
  wire                           exec_op_add;
  wire                           exec_op_shift;
  wire                           exec_op_movhi;
  wire                           exec_op_cmov;
  wire                           exec_op_logic;
  wire                           exec_op_setflag;
  // all of earlier components:
  localparam ONE_CLK_OP_WIDTH = 7;

  //  # attributes
  wire                           exec_adder_do_sub;
  wire                           exec_adder_do_carry;
  wire [`OR1K_ALU_OPC_WIDTH-1:0] exec_opc_alu_secondary;
  wire [`OR1K_ALU_OPC_WIDTH-1:0] exec_opc_logic;
  // attributes include all of earlier components:
  localparam ONE_CLK_OPC_WIDTH = 2 + (2 * `OR1K_ALU_OPC_WIDTH);

  // input operands A and B with forwarding from WB
  wire [OPTION_OPERAND_WIDTH-1:0] exec_1clk_a1;
  wire [OPTION_OPERAND_WIDTH-1:0] exec_1clk_b1;

  //  # update carry flag by 1clk-operation
  wire wb_1clk_carry_set;
  wire wb_1clk_carry_clear;

  //  # update overflow flag by 1clk-operation
  wire wb_1clk_overflow_set;
  wire wb_1clk_overflow_clear;

  // **** reservation station for 1-clk ****
  mor1kx_rsrvs_marocchino // 1CLK_RSVRS
  #(
    .OPTION_OPERAND_WIDTH         (OPTION_OPERAND_WIDTH), // 1CLK_RSVRS
    .OP_WIDTH                     (ONE_CLK_OP_WIDTH), // 1CLK_RSVRS
    .OPC_WIDTH                    (ONE_CLK_OPC_WIDTH), // 1CLK_RSVRS
    .DEST_EXTADR_WIDTH            (DEST_EXTADR_WIDTH), // 1CLK_RSVRS
    // Reservation station is used for 1-clock execution module.
    // As 1-clock pushed if only it is granted by write-back access
    // all input operandes already forwarder. So we don't use
    // exec_op_o and we remove exra logic for it.
    .RSRVS_1CLK                   (1), // 1CLK_RSVRS
    // Reservation station is used for LSU
    .RSRVS_LSU                    (0), // 1CLK_RSVRS
    // Reservation station is used for integer MUL/DIV.
    .RSRVS_MULDIV                 (0), // 1CLK_RSVRS
    // Reservation station is used for FPU3264.
    // Extra logic for the A2 and B2 related hazards is generated.
    .RSRVS_FPU                    (0), // 1CLK_RSVRS
    // Packed operands for various reservation stations:
    //  # LSU :   {   x,    x, rfb1, rfa1}
    //  # 1CLK:   {   x,    x, rfb1, rfa1}
    //  # MULDIV: {   x,    x, rfb1, rfa1}
    //  # FPU:    {rfb2, rfa2, rfb1, rfa1}
    .DCOD_RFXX_WIDTH              (2 * OPTION_OPERAND_WIDTH), // 1CLK_RSRVS
    // OMAN-to-DECODE hazard flags layout for various reservation stations:
    //  # LSU :   {   x,    x,     x,    x,  d2b1, d1b1,  d2a1, d1a1 }
    //  # 1CLK:   {   x,    x,     x,    x,  d2b1, d1b1,  d2a1, d1a1 }
    //  # MULDIV: {   x,    x,     x,    x,  d2b1, d1b1,  d2a1, d1a1 }
    //  # FPU:    {d2b2, d1b2,  d2a2, d1a2,  d2b1, d1b1,  d2a1, d1a1 }
    .OMN2DEC_HAZARDS_FLAGS_WIDTH  (4), // 1CLK_RSVRS
    // OMAN-to-DECODE hazard id layout for various reservation stations:
    //  # LSU :   {   x,    x, dxb1, dxa1 }
    //  # 1CLK:   {   x,    x, dxb1, dxa1 }
    //  # MULDIV: {   x,    x, dxb1, dxa1 }
    //  # FPU:    {dxb2, dxa2, dxb1, dxa1 }
    .OMN2DEC_HAZARDS_ADDRS_WIDTH  (2 * DEST_EXTADR_WIDTH) // 1CLK_RSVRS
  )
  u_1clk_rsrvs
  (
    // clocks and resets
    .cpu_clk                    (cpu_clk), // 1CLK_RSVRS
    .cpu_rst                    (cpu_rst), // 1CLK_RSVRS
    // pipeline control signals in
    .pipeline_flush_i           (pipeline_flush), // 1CLK_RSVRS
    .padv_exec_i                (padv_exec), // 1CLK_RSVRS
    .taking_op_i                (taking_1clk_op), // 1CLK_RSVRS
    // input data from DECODE
    .dcod_rfxx_i                ({dcod_rfb1, dcod_rfa1}), // 1CLK_RSVRS
    // OMAN-to-DECODE hazards
    //  # hazards flags
    .omn2dec_hazards_flags_i    ({omn2dec_hazard_d2b1, omn2dec_hazard_d1b1, // 1CLK_RSVRS
                                  omn2dec_hazard_d2a1, omn2dec_hazard_d1a1}), // 1CLK_RSVRS
    //  # hasards addresses
    .omn2dec_hazards_addrs_i    ({omn2dec_extadr_dxb1, omn2dec_extadr_dxa1}), // 1CLK_RSVRS
    // Hazard could be resolving
    //  ## write-back attributes
    .wb_extadr_i                (wb_extadr), // 1CLK_RSVRS
    //  ## forwarding results
    .wb_result1_i               (wb_result1_cp1), // 1CLK_RSVRS
    .wb_result2_i               (wb_result2_cp1), // 1CLK_RSVRS
    // command and its additional attributes
    .dcod_op_any_i              (dcod_op_1clk), // 1CLK_RSVRS
    .dcod_op_i                  ({dcod_op_ffl1, dcod_op_add, dcod_op_shift, dcod_op_movhi, // 1CLK_RSVRS
                                  dcod_op_cmov, dcod_op_logic, dcod_op_setflag}), // 1CLK_RSVRS
    .dcod_opc_i                 ({dcod_adder_do_sub, dcod_adder_do_carry, // 1CLK_RSVRS
                                  dcod_opc_alu_secondary, dcod_opc_logic}), // 1CLK_RSVRS
    // outputs
    //   command and its additional attributes
    .exec_op_any_o              (), // 1CLK_RSVRS
    .exec_op_o                  ({exec_op_ffl1, exec_op_add, exec_op_shift, exec_op_movhi, // 1CLK_RSVRS
                                  exec_op_cmov, exec_op_logic, exec_op_setflag}), // 1CLK_RSVRS
    .exec_opc_o                 ({exec_adder_do_sub, exec_adder_do_carry, // 1CLK_RSVRS
                                  exec_opc_alu_secondary, exec_opc_logic}), // 1CLK_RSVRS
    //   operands
    .exec_rfa1_o                (exec_1clk_a1), // 1CLK_RSVRS
    .exec_rfb1_o                (exec_1clk_b1), // 1CLK_RSVRS
    //  ## for FPU64
    .exec_rfa2_o                (), // 1CLK_RSVRS
    .exec_rfb2_o                (), // 1CLK_RSVRS
    //   unit-is-busy flag
    .unit_free_o                (op_1clk_free) // 1CLK_RSVRS
  );


  // **** 1clk ****
  mor1kx_exec_1clk_marocchino
  #(
    .OPTION_OPERAND_WIDTH             (OPTION_OPERAND_WIDTH) // 1CLK_EXEC
  )
  u_exec_1clk
  (
    // clocks & resets
    .cpu_clk                          (cpu_clk), // 1CLK_EXEC
    .cpu_rst                          (cpu_rst), // 1CLK_EXEC

    // pipeline controls
    .pipeline_flush_i                 (pipeline_flush), // 1CLK_EXEC
    .padv_wb_i                        (padv_wb), // 1CLK_EXEC
    .grant_wb_to_1clk_i               (grant_wb_to_1clk), // 1CLK_EXEC
    .taking_1clk_op_o                 (taking_1clk_op), // 1CLK_EXEC

    // input operands A and B with forwarding from WB
    .exec_1clk_a1_i                   (exec_1clk_a1), // 1CLK_EXEC
    .exec_1clk_b1_i                   (exec_1clk_b1), // 1CLK_EXEC

    // 1-clock instruction auxiliaries
    .exec_opc_alu_secondary_i         (exec_opc_alu_secondary), // 1CLK_EXEC
    .carry_i                          (ctrl_carry), // 1CLK_EXEC
    .flag_i                           (ctrl_flag), // 1CLK_EXEC

    // adder
    .exec_op_add_i                    (exec_op_add), // 1CLK_EXEC
    .exec_adder_do_sub_i              (exec_adder_do_sub), // 1CLK_EXEC
    .exec_adder_do_carry_i            (exec_adder_do_carry), // 1CLK_EXEC
    // shift, ffl1, movhi, cmov
    .exec_op_shift_i                  (exec_op_shift), // 1CLK_EXEC
    .exec_op_ffl1_i                   (exec_op_ffl1), // 1CLK_EXEC
    .exec_op_movhi_i                  (exec_op_movhi), // 1CLK_EXEC
    .exec_op_cmov_i                   (exec_op_cmov), // 1CLK_EXEC
    // logic
    .exec_op_logic_i                  (exec_op_logic), // 1CLK_EXEC
    .exec_opc_logic_i                 (exec_opc_logic), // 1CLK_EXEC
    // WB-latched 1-clock arithmetic result
    .wb_alu_1clk_result_o             (wb_alu_1clk_result), // 1CLK_EXEC
    .wb_alu_1clk_result_cp1_o         (wb_alu_1clk_result_cp1), // 1CLK_EXEC
    .wb_alu_1clk_result_cp2_o         (wb_alu_1clk_result_cp2), // 1CLK_EXEC
    .wb_alu_1clk_result_cp3_o         (wb_alu_1clk_result_cp3), // 1CLK_EXEC
    //  # update carry flag by 1clk-operation
    .wb_1clk_carry_set_o              (wb_1clk_carry_set), // 1CLK_EXEC
    .wb_1clk_carry_clear_o            (wb_1clk_carry_clear), // 1CLK_EXEC
    //  # update overflow flag by 1clk-operation
    .wb_1clk_overflow_set_o           (wb_1clk_overflow_set), // 1CLK_EXEC
    .wb_1clk_overflow_clear_o         (wb_1clk_overflow_clear), // 1CLK_EXEC
    //  # generate overflow exception by 1clk-operation
    .except_overflow_enable_i         (except_overflow_enable), // 1CLK_EXEC
    .exec_except_overflow_1clk_o      (exec_except_overflow_1clk), // 1CLK_EXEC
    .wb_except_overflow_1clk_o        (wb_except_overflow_1clk), // 1CLK_EXEC

    // integer comparison flag
    .exec_op_setflag_i                (exec_op_setflag), // 1CLK_EXEC
    // WB: integer comparison result
    .wb_int_flag_set_o                (wb_int_flag_set), // 1CLK_EXEC
    .wb_int_flag_clear_o              (wb_int_flag_clear) // 1CLK_EXEC
  );


  //---------------------------------------------//
  // Reservation station for integer MUL/DIV     //
  //   # 32-bits integer multiplier              //
  //   # 32-bits integer divider                 //
  //---------------------------------------------//
  // any kind of multi-clock operation
  wire exec_op_muldiv;
  // run integer multiplier
  wire exec_op_mul;
  // run divider
  wire exec_op_div;
  wire exec_op_div_signed;
  wire exec_op_div_unsigned;

  // OP layout integer MUL/DIV reservation station
  localparam MULDIV_OP_WIDTH = 1;

  // OPC layout for multi-clocks reservation station
  //  # int multiplier:                                      1
  //  # int divider + (signed / unsigned division):          3
  //  # ------------------------------------------------------
  //  # overall:                                             4
  localparam MULDIV_OPC_WIDTH = 4;

  // MUL/DIV input operands
  wire [(OPTION_OPERAND_WIDTH-1):0] exec_muldiv_a1;
  wire [(OPTION_OPERAND_WIDTH-1):0] exec_muldiv_b1;

  //  # MCLK is tacking operands
  wire imul_taking_op, idiv_taking_op;
  wire muldiv_taking_op = imul_taking_op | idiv_taking_op;

  // **** Integer MUL/DIV reservation station instance ****
  mor1kx_rsrvs_marocchino // MULDIV_RSRVS
  #(
    .OPTION_OPERAND_WIDTH         (OPTION_OPERAND_WIDTH), // MULDIV_RSRVS
    .OP_WIDTH                     (MULDIV_OP_WIDTH), // MULDIV_RSRVS
    .OPC_WIDTH                    (MULDIV_OPC_WIDTH), // MULDIV_RSRVS
    .DEST_EXTADR_WIDTH            (DEST_EXTADR_WIDTH), // MULDIV_RSRVS
    // Reservation station is used for 1-clock execution module.
    // As 1-clock pushed if only it is granted by write-back access
    // all input operandes already forwarder. So we don't use
    // exec_op_o and we remove exra logic for it.
    .RSRVS_1CLK                   (0), // MULDIV_RSRVS
    // Reservation station is used for LSU
    .RSRVS_LSU                    (0), // MULDIV_RSRVS
    // Reservation station is used for integer MUL/DIV.
    .RSRVS_MULDIV                 (1), // MULDIV_RSRVS
    // Reservation station is used for FPU3264.
    // Extra logic for the A2 and B2 related hazards is generated.
    .RSRVS_FPU                    (0), // MULDIV_RSRVS
    // Packed operands for various reservation stations:
    //  # LSU :   {   x,    x, rfb1, rfa1}
    //  # 1CLK:   {   x,    x, rfb1, rfa1}
    //  # MULDIV: {   x,    x, rfb1, rfa1}
    //  # FPU:    {rfb2, rfa2, rfb1, rfa1}
    .DCOD_RFXX_WIDTH              (2 * OPTION_OPERAND_WIDTH), // MULDIV_RSRVS
    // OMAN-to-DECODE hazard flags layout for various reservation stations:
    //  # LSU :   {   x,    x,     x,    x,  d2b1, d1b1,  d2a1, d1a1 }
    //  # 1CLK:   {   x,    x,     x,    x,  d2b1, d1b1,  d2a1, d1a1 }
    //  # MULDIV: {   x,    x,     x,    x,  d2b1, d1b1,  d2a1, d1a1 }
    //  # FPU:    {d2b2, d1b2,  d2a2, d1a2,  d2b1, d1b1,  d2a1, d1a1 }
    .OMN2DEC_HAZARDS_FLAGS_WIDTH  (4), // MULDIV_RSRVS
    // OMAN-to-DECODE hazard id layout for various reservation stations:
    //  # LSU :   {   x,    x, dxb1, dxa1 }
    //  # 1CLK:   {   x,    x, dxb1, dxa1 }
    //  # MULDIV: {   x,    x, dxb1, dxa1 }
    //  # FPU:    {dxb2, dxa2, dxb1, dxa1 }
    .OMN2DEC_HAZARDS_ADDRS_WIDTH  (2 * DEST_EXTADR_WIDTH) // MULDIV_RSRVS
  )
  u_muldiv_rsrvs
  (
    // clocks and resets
    .cpu_clk                    (cpu_clk), // MULDIV_RSRVS
    .cpu_rst                    (cpu_rst), // MULDIV_RSRVS
    // pipeline control signals in
    .pipeline_flush_i           (pipeline_flush), // MULDIV_RSRVS
    .padv_exec_i                (padv_exec), // MULDIV_RSRVS
    .taking_op_i                (muldiv_taking_op), // MULDIV_RSRVS
    // input data from DECODE
    .dcod_rfxx_i                ({dcod_rfb1, dcod_rfa1}), // MULDIV_RSRVS
    // OMAN-to-DECODE hazards
    //  # hazards flags
    .omn2dec_hazards_flags_i    ({omn2dec_hazard_d2b1, omn2dec_hazard_d1b1, // MULDIV_RSRVS
                                  omn2dec_hazard_d2a1, omn2dec_hazard_d1a1}), // MULDIV_RSRVS
    //  # hasards addresses
    .omn2dec_hazards_addrs_i    ({omn2dec_extadr_dxb1, omn2dec_extadr_dxa1}), // MULDIV_RSRVS
    // Hazard could be resolving
    //  ## write-back attributes
    .wb_extadr_i                (wb_extadr), // MULDIV_RSRVS
    //  ## forwarding results
    .wb_result1_i               (wb_result1_cp2), // MULDIV_RSRVS
    .wb_result2_i               (wb_result2_cp2), // MULDIV_RSRVS
    // command and its additional attributes
    .dcod_op_any_i              (dcod_op_muldiv), // MULDIV_RSRVS
    .dcod_op_i                  (dcod_op_muldiv), // MULDIV_RSRVS
    .dcod_opc_i                 ({dcod_op_mul, // MULDIV_RSRVS
                                  dcod_op_div, dcod_op_div_signed, dcod_op_div_unsigned}),  // MULDIV_RSRVS
    // outputs
    //   command and its additional attributes
    .exec_op_any_o              (), // MULDIV_RSRVS
    .exec_op_o                  (exec_op_muldiv), // MULDIV_RSRVS
    .exec_opc_o                 ({exec_op_mul,  // MULDIV_RSRVS
                                  exec_op_div, exec_op_div_signed, exec_op_div_unsigned}),  // MULDIV_RSRVS
    //   operands
    .exec_rfa1_o                (exec_muldiv_a1), // MULDIV_RSRVS
    .exec_rfb1_o                (exec_muldiv_b1), // MULDIV_RSRVS
    .exec_rfa2_o                (), // MULDIV_RSRVS
    .exec_rfb2_o                (), // MULDIV_RSRVS
    //   unit-is-busy flag
    .unit_free_o                (muldiv_free) // MULDIV_RSRVS
  );


  //-------------------//
  // 32-bit multiplier //
  //-------------------//

  mor1kx_multiplier_marocchino
  #(
    .OPTION_OPERAND_WIDTH             (OPTION_OPERAND_WIDTH) // MUL
  )
  u_multiplier
  (
    // clocks & resets
    .cpu_clk                          (cpu_clk), // MUL
    .cpu_rst                          (cpu_rst), // MUL
    // pipeline controls
    .pipeline_flush_i                 (pipeline_flush), // MUL
    .padv_wb_i                        (padv_wb), // MUL
    .grant_wb_to_mul_i                (grant_wb_to_mul), // MUL
    // input operands from reservation station
    .exec_mul_a1_i                    (exec_muldiv_a1), // MUL
    .exec_mul_b1_i                    (exec_muldiv_b1), // MUL
    //  other inputs/outputs
    .exec_op_muldiv_i                 (exec_op_muldiv), // MUL
    .exec_op_mul_i                    (exec_op_mul), // MUL
    .imul_taking_op_o                 (imul_taking_op), // MUL
    .mul_valid_o                      (mul_valid), // MUL
    .wb_mul_result_o                  (wb_mul_result), // MUL
    .wb_mul_result_cp1_o              (wb_mul_result_cp1), // MUL
    .wb_mul_result_cp2_o              (wb_mul_result_cp2), // MUL
    .wb_mul_result_cp3_o              (wb_mul_result_cp3) // MUL
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

  // **** integer divider ****
  mor1kx_divider_marocchino
  #(
    .OPTION_OPERAND_WIDTH             (OPTION_OPERAND_WIDTH), // DIV
    .FEATURE_DIVIDER                  (FEATURE_DIVIDER) // DIV
  )
  u_divider
  (
    // clocks & resets
    .cpu_clk                          (cpu_clk), // DIV
    .cpu_rst                          (cpu_rst), // DIV
    // pipeline controls
    .pipeline_flush_i                 (pipeline_flush), // DIV
    .padv_wb_i                        (padv_wb), // DIV
    .grant_wb_to_div_i                (grant_wb_to_div), // DIV
    // input data from reservation station
    .exec_div_a1_i                    (exec_muldiv_a1), // DIV
    .exec_div_b1_i                    (exec_muldiv_b1), // DIV
    // division command
    .exec_op_muldiv_i                 (exec_op_muldiv), // DIV
    .exec_op_div_i                    (exec_op_div), // DIV
    .exec_op_div_signed_i             (exec_op_div_signed), // DIV
    .exec_op_div_unsigned_i           (exec_op_div_unsigned), // DIV
    // division engine state
    .idiv_taking_op_o                 (idiv_taking_op), // DIV
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
    .exec_except_overflow_div_o       (exec_except_overflow_div), // DIV
    .wb_except_overflow_div_o         (wb_except_overflow_div), // DIV
    //  # division result
    .wb_div_result_o                  (wb_div_result), // DIV
    .wb_div_result_cp1_o              (wb_div_result_cp1), // DIV
    .wb_div_result_cp2_o              (wb_div_result_cp2), // DIV
    .wb_div_result_cp3_o              (wb_div_result_cp3) // DIV
  );


  //---------------------------------------------//
  // Reservation station for FPU3264             //
  //   # 32/64-bits FP arithmetic                //
  //   # 64-bits FP comparison                   //
  //---------------------------------------------//
  // run fp3264 arithmetic
  wire exec_op_fp64_arith, exec_op_fpxx_add, exec_op_fpxx_sub, exec_op_fpxx_mul,
                           exec_op_fpxx_div, exec_op_fpxx_i2f, exec_op_fpxx_f2i;
  // run fp64 comparison
  //(declared earlier)

  // OP layout for FPU reservation station: (fpxx_arith OR fpxx_cmp)
  localparam FPU_OP_WIDTH = 1;

  // OPC layout for multi-clocks reservation station
  //  # fp3264 arithmetic type (add,sub,mul,div,i2f,f2i):    6
  //  # double precision bit:                                1
  //  # run fp64 comparison:                                 1
  //  # fp64 comparison variant:                             3
  //  # ------------------------------------------------------
  //  # overall:                                            11
  localparam FPU_OPC_WIDTH = 11;

  // FPU input operands
  wire [(OPTION_OPERAND_WIDTH-1):0] exec_fpxx_a1;
  wire [(OPTION_OPERAND_WIDTH-1):0] exec_fpxx_b1;
  wire [(OPTION_OPERAND_WIDTH-1):0] exec_fpxx_a2;
  wire [(OPTION_OPERAND_WIDTH-1):0] exec_fpxx_b2;

  // **** FPU3264 reservation station instance ****
  mor1kx_rsrvs_marocchino // FPU_RSRVS
  #(
    .OPTION_OPERAND_WIDTH         (OPTION_OPERAND_WIDTH), // FPU_RSRVS
    .OP_WIDTH                     (FPU_OP_WIDTH), // FPU_RSRVS
    .OPC_WIDTH                    (FPU_OPC_WIDTH), // FPU_RSRVS
    .DEST_EXTADR_WIDTH            (DEST_EXTADR_WIDTH), // FPU_RSRVS
    // Reservation station is used for 1-clock execution module.
    // As 1-clock pushed if only it is granted by write-back access
    // all input operandes already forwarder. So we don't use
    // exec_op_o and we remove exra logic for it.
    .RSRVS_1CLK                   (0), // FPU_RSRVS
    // Reservation station is used for LSU
    .RSRVS_LSU                    (0), // FPU_RSRVS
    // Reservation station is used for integer MUL/DIV.
    .RSRVS_MULDIV                 (0), // FPU_RSRVS
    // Reservation station is used for FPU3264.
    // Extra logic for the A2 and B2 related hazards is generated.
    .RSRVS_FPU                    (1), // FPU_RSRVS
    // Packed operands for various reservation stations:
    //  # LSU :   {   x,    x, rfb1, rfa1}
    //  # 1CLK:   {   x,    x, rfb1, rfa1}
    //  # MULDIV: {   x,    x, rfb1, rfa1}
    //  # FPU:    {rfb2, rfa2, rfb1, rfa1}
    .DCOD_RFXX_WIDTH              (4 * OPTION_OPERAND_WIDTH), // FPU_RSRVS
    // OMAN-to-DECODE hazard flags layout for various reservation stations:
    //  # LSU :   {   x,    x,     x,    x,  d2b1, d1b1,  d2a1, d1a1 }
    //  # 1CLK:   {   x,    x,     x,    x,  d2b1, d1b1,  d2a1, d1a1 }
    //  # MULDIV: {   x,    x,     x,    x,  d2b1, d1b1,  d2a1, d1a1 }
    //  # FPU:    {d2b2, d1b2,  d2a2, d1a2,  d2b1, d1b1,  d2a1, d1a1 }
    .OMN2DEC_HAZARDS_FLAGS_WIDTH  (8), // FPU_RSRVS
    // OMAN-to-DECODE hazard id layout for various reservation stations:
    //  # LSU :   {   x,    x, dxb1, dxa1 }
    //  # 1CLK:   {   x,    x, dxb1, dxa1 }
    //  # MULDIV: {   x,    x, dxb1, dxa1 }
    //  # FPU:    {dxb2, dxa2, dxb1, dxa1 }
    .OMN2DEC_HAZARDS_ADDRS_WIDTH  (4 * DEST_EXTADR_WIDTH) // FPU_RSRVS
  )
  u_fpxx_rsrvs
  (
    // clocks and resets
    .cpu_clk                    (cpu_clk), // FPU_RSRVS
    .cpu_rst                    (cpu_rst), // FPU_RSRVS
    // pipeline control signals in
    .pipeline_flush_i           (pipeline_flush), // FPU_RSRVS
    .padv_exec_i                (padv_exec), // FPU_RSRVS
    .taking_op_i                (fpxx_taking_op), // FPU_RSRVS
    // input data from DECODE
    .dcod_rfxx_i                ({dcod_rfb2, dcod_rfa2, dcod_rfb1, dcod_rfa1}), // FPU_RSRVS
    // OMAN-to-DECODE hazards
    //  # hazards flags
    .omn2dec_hazards_flags_i    ({omn2dec_hazard_d2b2, omn2dec_hazard_d1b2,  // FPU_RSRVS
                                  omn2dec_hazard_d2a2, omn2dec_hazard_d1a2, // FPU_RSRVS
                                  omn2dec_hazard_d2b1, omn2dec_hazard_d1b1, // FPU_RSRVS
                                  omn2dec_hazard_d2a1, omn2dec_hazard_d1a1}), // FPU_RSRVS
    //  # hasards addresses
    .omn2dec_hazards_addrs_i    ({omn2dec_extadr_dxb2, omn2dec_extadr_dxa2, // FPU_RSRVS
                                  omn2dec_extadr_dxb1, omn2dec_extadr_dxa1}), // FPU_RSRVS
    // Hazard could be resolving
    //  ## write-back attributes
    .wb_extadr_i                (wb_extadr), // FPU_RSRVS
    //  ## forwarding results
    .wb_result1_i               (wb_result1_cp2), // FPU_RSRVS
    .wb_result2_i               (wb_result2_cp2), // FPU_RSRVS
    // command and its additional attributes
    .dcod_op_any_i              (dcod_op_fpxx_any), // FPU_RSRVS
    .dcod_op_i                  (dcod_op_fpxx_any), // FPU_RSRVS
    .dcod_opc_i                 ({dcod_op_fpxx_add,   dcod_op_fpxx_sub, dcod_op_fpxx_mul, // FPU_RSRVS
                                  dcod_op_fpxx_div,   dcod_op_fpxx_i2f, dcod_op_fpxx_f2i, // FPU_RSRVS
                                  dcod_op_fp64_arith, dcod_op_fpxx_cmp, dcod_opc_fpxx_cmp}), // FPU_RSRVS
    // outputs
    //   command and its additional attributes
    .exec_op_any_o              (), // FPU_RSRVS
    .exec_op_o                  (exec_op_fpxx_any), // FPU_RSRVS
    .exec_opc_o                 ({exec_op_fpxx_add,   exec_op_fpxx_sub, exec_op_fpxx_mul, // FPU_RSRVS
                                  exec_op_fpxx_div,   exec_op_fpxx_i2f, exec_op_fpxx_f2i, // FPU_RSRVS
                                  exec_op_fp64_arith, exec_op_fpxx_cmp, exec_opc_fpxx_cmp}),  // FPU_RSRVS
    //   operands
    .exec_rfa1_o                (exec_fpxx_a1), // FPU_RSRVS
    .exec_rfb1_o                (exec_fpxx_b1), // FPU_RSRVS
    .exec_rfa2_o                (exec_fpxx_a2), // FPU_RSRVS
    .exec_rfb2_o                (exec_fpxx_b2), // FPU_RSRVS
    //   unit-is-busy flag
    .unit_free_o                (fpxx_free) // FPU_RSRVS
  );


  //---------//
  // FPU3264 //
  //---------//
  pfpu_top_marocchino  u_pfpu3264
  (
    // clock & reset
    .cpu_clk                    (cpu_clk), // FPU3264
    .cpu_rst                    (cpu_rst), // FPU3264

    // pipeline control
    .pipeline_flush_i           (pipeline_flush), // FPU3264
    .padv_wb_i                  (padv_wb), // FPU3264
    .grant_wb_to_fpxx_arith_i   (grant_wb_to_fpxx_arith), // FPU3264
    .grant_wb_to_fpxx_cmp_i     (grant_wb_to_fpxx_cmp), // FPU3264

    // pipeline control outputs
    .fpxx_taking_op_o           (fpxx_taking_op), // FPU3264
    .fpxx_arith_valid_o         (fpxx_arith_valid), // FPU3264
    .fpxx_cmp_valid_o           (fpxx_cmp_valid), // FPU3264

    // Configuration
    .fpu_round_mode_i           (ctrl_fpu_round_mode), // FPU3264
    .except_fpu_enable_i        (except_fpu_enable), // FPU3264
    .fpu_mask_flags_i           (ctrl_fpu_mask_flags), // FPU3264

    // From multi-clock reservation station
    .exec_op_fpxx_any_i         (exec_op_fpxx_any), // FPU3264

    // Commands for arithmetic part
    .exec_op_fp64_arith_i       (exec_op_fp64_arith), // FPU3264
    .exec_op_fpxx_add_i         (exec_op_fpxx_add), // FPU3264
    .exec_op_fpxx_sub_i         (exec_op_fpxx_sub), // FPU3264
    .exec_op_fpxx_mul_i         (exec_op_fpxx_mul), // FPU3264
    .exec_op_fpxx_div_i         (exec_op_fpxx_div), // FPU3264
    .exec_op_fpxx_i2f_i         (exec_op_fpxx_i2f), // FPU3264
    .exec_op_fpxx_f2i_i         (exec_op_fpxx_f2i), // FPU3264

    // Commands for comparison part
    .exec_op_fpxx_cmp_i         (exec_op_fpxx_cmp), // FPU3264
    .exec_opc_fpxx_cmp_i        (exec_opc_fpxx_cmp), // FPU3264

    // Operands from reservation station
    .exec_fpxx_a1_i             (exec_fpxx_a1), // FPU3264
    .exec_fpxx_b1_i             (exec_fpxx_b1), // FPU3264
    .exec_fpxx_a2_i             (exec_fpxx_a2), // FPU3264
    .exec_fpxx_b2_i             (exec_fpxx_b2), // FPU3264

    // Pre-WB outputs
    .exec_except_fpxx_arith_o   (exec_except_fpxx_arith), // FPU3264
    .exec_except_fpxx_cmp_o     (exec_except_fpxx_cmp), // FPU3264

    // FPU2364 arithmetic part
    .wb_fpxx_arith_res_hi_o     (wb_fpxx_arith_res_hi), // FPU3264
    .wb_fpxx_arith_res_hi_cp1_o (wb_fpxx_arith_res_hi_cp1), // FPU3264
    .wb_fpxx_arith_res_hi_cp2_o (wb_fpxx_arith_res_hi_cp2), // FPU3264
    .wb_fpxx_arith_res_hi_cp3_o (wb_fpxx_arith_res_hi_cp3), // FPU3264
    .wb_fpxx_arith_res_lo_o     (wb_fpxx_arith_res_lo), // FPU3264
    .wb_fpxx_arith_res_lo_cp1_o (wb_fpxx_arith_res_lo_cp1), // FPU3264
    .wb_fpxx_arith_res_lo_cp2_o (wb_fpxx_arith_res_lo_cp2), // FPU3264
    .wb_fpxx_arith_res_lo_cp3_o (wb_fpxx_arith_res_lo_cp3), // FPU3264
    .wb_fpxx_arith_fpcsr_o      (wb_fpxx_arith_fpcsr), // FPU3264
    .wb_fpxx_arith_wb_fpcsr_o   (wb_fpxx_arith_wb_fpcsr), // FPU3264
    .wb_except_fpxx_arith_o     (wb_except_fpxx_arith), // FPU3264

    // FPU-64 comparison part
    .wb_fpxx_flag_set_o         (wb_fpxx_flag_set), // FPU3264
    .wb_fpxx_flag_clear_o       (wb_fpxx_flag_clear), // FPU3264
    .wb_fpxx_cmp_inv_o          (wb_fpxx_cmp_inv), // FPU3264
    .wb_fpxx_cmp_inf_o          (wb_fpxx_cmp_inf), // FPU3264
    .wb_fpxx_cmp_wb_fpcsr_o     (wb_fpxx_cmp_wb_fpcsr), // FPU3264
    .wb_except_fpxx_cmp_o       (wb_except_fpxx_cmp) // FPU3264
  );


  //--------------//
  // LSU instance //
  //--------------//

  // LSU -> RSRVS feedback
  wire lsu_taking_op;

  // RSRVS -> LSU connections
  //  # combined load/store/msync
  wire                            exec_op_lsu_any;
  //  # particular commands and their attributes
  wire                            exec_op_lsu_load;
  wire                            exec_op_lsu_store;
  wire                            exec_op_msync;
  wire                            exec_op_lsu_atomic;
  wire                      [1:0] exec_lsu_length;
  wire                            exec_lsu_zext;
  //  # immediate offset for address computation
  wire      [`OR1K_IMM_WIDTH-1:0] exec_lsu_imm16;
  //  # PC for store buffer EPCR computation
  wire [OPTION_OPERAND_WIDTH-1:0] exec_sbuf_epcr;
  //  # operands after frorwarding from WB
  wire [OPTION_OPERAND_WIDTH-1:0] exec_lsu_a1;
  wire [OPTION_OPERAND_WIDTH-1:0] exec_lsu_b1;

  // **** reservation station for LSU ****

  // load/store commands:
  localparam LSU_OP_WIDTH = 2;

  // l.msync and commands attributes:
  //  ## l.msync                                 1
  //  ## atomic operation:                       1
  //  ## length:                                 2
  //  ## zero extension:                         1
  //  ## immediate width:                       16
  //  ## EPCR for STORE_BUFFER exception:       32
  localparam LSU_OPC_WIDTH = 5 + `OR1K_IMM_WIDTH + OPTION_OPERAND_WIDTH;

  // reservation station instance
  mor1kx_rsrvs_marocchino // LSU_RSRVS
  #(
    .OPTION_OPERAND_WIDTH         (OPTION_OPERAND_WIDTH), // LSU_RSRVS
    .OP_WIDTH                     (LSU_OP_WIDTH), // LSU_RSRVS
    .OPC_WIDTH                    (LSU_OPC_WIDTH), // LSU_RSRVS
    .DEST_EXTADR_WIDTH            (DEST_EXTADR_WIDTH), // LSU_RSRVS
    // Reservation station is used for 1-clock execution module.
    // As 1-clock pushed if only it is granted by write-back access
    // all input operandes already forwarder. So we don't use
    // exec_op_o and we remove exra logic for it.
    .RSRVS_1CLK                   (0), // LSU_RSRVS
    // Reservation station is used for LSU
    .RSRVS_LSU                    (1), // LSU_RSRVS
    // Reservation station is used for integer MUL/DIV.
    .RSRVS_MULDIV                 (0), // LSU_RSRVS
    // Reservation station is used for FPU3264.
    // Extra logic for the A2 and B2 related hazards is generated.
    .RSRVS_FPU                    (0), // LSU_RSRVS
    // Packed operands for various reservation stations:
    //  # LSU :   {   x,    x, rfb1, rfa1}
    //  # 1CLK:   {   x,    x, rfb1, rfa1}
    //  # MULDIV: {   x,    x, rfb1, rfa1}
    //  # FPU:    {rfb2, rfa2, rfb1, rfa1}
    .DCOD_RFXX_WIDTH              (2 * OPTION_OPERAND_WIDTH), // LSU_RSRVS
    // OMAN-to-DECODE hazard flags layout for various reservation stations:
    //  # LSU :   {   x,    x,     x,    x,  d2b1, d1b1,  d2a1, d1a1 }
    //  # 1CLK:   {   x,    x,     x,    x,  d2b1, d1b1,  d2a1, d1a1 }
    //  # MULDIV: {   x,    x,     x,    x,  d2b1, d1b1,  d2a1, d1a1 }
    //  # FPU:    {d2b2, d1b2,  d2a2, d1a2,  d2b1, d1b1,  d2a1, d1a1 }
    .OMN2DEC_HAZARDS_FLAGS_WIDTH  (4), // LSU_RSRVS
    // OMAN-to-DECODE hazard id layout for various reservation stations:
    //  # LSU :   {   x,    x, dxb1, dxa1 }
    //  # 1CLK:   {   x,    x, dxb1, dxa1 }
    //  # MULDIV: {   x,    x, dxb1, dxa1 }
    //  # FPU:    {dxb2, dxa2, dxb1, dxa1 }
    .OMN2DEC_HAZARDS_ADDRS_WIDTH  (2 * DEST_EXTADR_WIDTH) // LSU_RSRVS
  )
  u_lsu_rsrvs
  (
    // clocks and resets
    .cpu_clk                    (cpu_clk), // LSU_RSVRS
    .cpu_rst                    (cpu_rst), // LSU_RSVRS
    // pipeline control signals in
    .pipeline_flush_i           (pipeline_flush), // LSU_RSVRS
    .padv_exec_i                (padv_exec), // LSU_RSVRS
    .taking_op_i                (lsu_taking_op), // LSU_RSVRS
    // input data from DECODE
    .dcod_rfxx_i                ({dcod_rfb1, dcod_rfa1}), // LSU_RSVRS
    // OMAN-to-DECODE hazards
    //  # hazards flags
    .omn2dec_hazards_flags_i    ({omn2dec_hazard_d2b1, omn2dec_hazard_d1b1, // LSU_RSVRS
                                  omn2dec_hazard_d2a1, omn2dec_hazard_d1a1}), // LSU_RSVRS
    //  # hasards addresses
    .omn2dec_hazards_addrs_i    ({omn2dec_extadr_dxb1, omn2dec_extadr_dxa1}), // LSU_RSVRS
    // Hazard could be resolving
    //  ## write-back attributes
    .wb_extadr_i                (wb_extadr), // LSU_RSVRS
    //  ## forwarding results
    .wb_result1_i               (wb_result1_cp3), // LSU_RSVRS
    .wb_result2_i               (wb_result2_cp3), // LSU_RSVRS
    // command and its additional attributes
    .dcod_op_any_i              (dcod_op_lsu_any),  // LSU_RSVRS
    .dcod_op_i                  ({dcod_op_lsu_load, dcod_op_lsu_store}), // LSU_RSVRS
    .dcod_opc_i                 ({dcod_op_msync,    dcod_op_lsu_atomic, // LSU_RSVRS
                                  dcod_lsu_length,  dcod_lsu_zext, // LSU_RSVRS
                                  dcod_imm16,       dcod_sbuf_epcr}), // LSU_RSVRS
    // outputs
    //   command and its additional attributes
    .exec_op_any_o              (exec_op_lsu_any), // LSU_RSVRS
    .exec_op_o                  ({exec_op_lsu_load, exec_op_lsu_store}), // LSU_RSVRS
    .exec_opc_o                 ({exec_op_msync,    exec_op_lsu_atomic, // LSU_RSVRS
                                  exec_lsu_length,  exec_lsu_zext, // LSU_RSVRS
                                  exec_lsu_imm16,   exec_sbuf_epcr}), // LSU_RSVRS
    //   operands
    .exec_rfa1_o                (exec_lsu_a1), // LSU_RSVRS
    .exec_rfb1_o                (exec_lsu_b1), // LSU_RSVRS
    //  ## for FPU64
    .exec_rfa2_o                (), // LSU_RSVRS
    .exec_rfb2_o                (), // LSU_RSVRS
    //   unit-is-busy flag
    .unit_free_o                (lsu_free) // LSU_RSVRS
  );


  // **** LSU instance ****
  mor1kx_lsu_marocchino
  #(
    .OPTION_OPERAND_WIDTH               (OPTION_OPERAND_WIDTH), // LSU
    .OPTION_DCACHE_BLOCK_WIDTH          (OPTION_DCACHE_BLOCK_WIDTH), // LSU
    .OPTION_DCACHE_SET_WIDTH            (OPTION_DCACHE_SET_WIDTH), // LSU
    .OPTION_DCACHE_WAYS                 (OPTION_DCACHE_WAYS), // LSU
    .OPTION_DCACHE_LIMIT_WIDTH          (OPTION_DCACHE_LIMIT_WIDTH), // LSU
    .OPTION_DCACHE_SNOOP                (OPTION_DCACHE_SNOOP), // LSU
    .OPTION_DCACHE_CLEAR_ON_INIT        (OPTION_DCACHE_CLEAR_ON_INIT), // LSU
    .FEATURE_DMMU_HW_TLB_RELOAD         (FEATURE_DMMU_HW_TLB_RELOAD), // LSU
    .OPTION_DMMU_SET_WIDTH              (OPTION_DMMU_SET_WIDTH), // LSU
    .OPTION_DMMU_WAYS                   (OPTION_DMMU_WAYS), // LSU
    .OPTION_DMMU_CLEAR_ON_INIT          (OPTION_DMMU_CLEAR_ON_INIT), // LSU
    .OPTION_STORE_BUFFER_DEPTH_WIDTH    (OPTION_STORE_BUFFER_DEPTH_WIDTH), // LSU
    .OPTION_STORE_BUFFER_CLEAR_ON_INIT  (OPTION_STORE_BUFFER_CLEAR_ON_INIT) // LSU
  )
  u_lsu
  (
    // clocks & resets
    .cpu_clk                          (cpu_clk), // LSU
    .cpu_rst                          (cpu_rst), // LSU
    // Pipeline controls
    .pipeline_flush_i                 (pipeline_flush), // LSU
    .padv_wb_i                        (padv_wb), // LSU
    .grant_wb_to_lsu_i                (grant_wb_to_lsu), // LSU
    // configuration
    .dc_enable_i                      (dc_enable), // LSU
    .dmmu_enable_i                    (dmmu_enable), // LSU
    .supervisor_mode_i                (supervisor_mode), // LSU
    // Input from RSRVR
    .exec_op_lsu_any_i                (exec_op_lsu_any), //LSU
    .exec_op_lsu_load_i               (exec_op_lsu_load), // LSU
    .exec_op_lsu_store_i              (exec_op_lsu_store), // LSU
    .exec_op_msync_i                  (exec_op_msync), // LSU
    .exec_op_lsu_atomic_i             (exec_op_lsu_atomic), // LSU
    .exec_lsu_length_i                (exec_lsu_length), // LSU
    .exec_lsu_zext_i                  (exec_lsu_zext), // LSU
    .exec_lsu_imm16_i                 (exec_lsu_imm16), // LSU
    .exec_sbuf_epcr_i                 (exec_sbuf_epcr), // LSU (for store buffer EPCR computation)
    .exec_lsu_a1_i                    (exec_lsu_a1), // LSU
    .exec_lsu_b1_i                    (exec_lsu_b1), // LSU
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
    .dbus_dat_i                       (dbus_dat_i), // LSU
    .dbus_burst_last_i                (dbus_burst_last_i), // LSU
    .dbus_adr_o                       (dbus_adr_o), // LSU
    .dbus_req_o                       (dbus_req_o), // LSU
    .dbus_dat_o                       (dbus_dat_o), // LSU
    .dbus_bsel_o                      (dbus_bsel_o[3:0]), // LSU
    .dbus_lwa_cmd_o                   (dbus_lwa_cmd_o), // LSU: atomic load
    .dbus_stna_cmd_o                  (dbus_stna_cmd_o), // LSU: none-atomic store
    .dbus_swa_cmd_o                   (dbus_swa_cmd_o), // LSU: atomic store
    .dbus_burst_o                     (dbus_burst_o), // LSU
    // Other connections for lwa/swa support
    .dbus_atomic_flg_i                (dbus_atomic_flg_i), // LSU
    // Cache sync for multi-core environment
    .snoop_adr_i                      (snoop_adr_i[31:0]), // LSU
    .snoop_en_i                       (snoop_en_i), // LSU
    // Pipe control output flags
    .lsu_taking_op_o                  (lsu_taking_op), // LSU
    .lsu_valid_o                      (lsu_valid), // LSU: result ready or exceptions
    // Imprecise exception (with appropriate PC) came via the store buffer
    .sbuf_eear_o                      (sbuf_eear), // LSU
    .sbuf_epcr_o                      (sbuf_epcr), // LSU
    .sbuf_err_o                       (sbuf_err), // LSU
    //  Pre-WriteBack "an exception" flag
    .exec_an_except_lsu_o             (exec_an_except_lsu), // LSU
    // WriteBack load  result
    .wb_lsu_result_o                  (wb_lsu_result), // LSU
    .wb_lsu_result_cp1_o              (wb_lsu_result_cp1), // LSU
    .wb_lsu_result_cp2_o              (wb_lsu_result_cp2), // LSU
    .wb_lsu_result_cp3_o              (wb_lsu_result_cp3), // LSU
    // Atomic operation flag set/clear logic
    .wb_atomic_flag_set_o             (wb_atomic_flag_set), // LSU
    .wb_atomic_flag_clear_o           (wb_atomic_flag_clear), // LSU
    // Exceptions & errors
    .wb_except_dbus_err_o             (wb_except_dbus_err), // LSU
    .wb_except_dpagefault_o           (wb_except_dpagefault), // LSU
    .wb_except_dtlb_miss_o            (wb_except_dtlb_miss), // LSU
    .wb_except_dbus_align_o           (wb_except_dbus_align), // LSU
    .wb_lsu_except_addr_o             (wb_lsu_except_addr) // LSU:
  );


  //-----------//
  // WB:result //
  //-----------//
  // --- regular ---
  assign wb_result1 = wb_alu_1clk_result   |
                      wb_div_result        | wb_mul_result |
                      wb_fpxx_arith_res_hi |
                      wb_lsu_result        | wb_mfspr_result;
  // copy #1 (to simplify feedback routing)
  assign wb_result1_cp1 = wb_alu_1clk_result_cp1   |
                          wb_div_result_cp1        | wb_mul_result_cp1 |
                          wb_fpxx_arith_res_hi_cp1 |
                          wb_lsu_result_cp1        | wb_mfspr_result_cp1;
  // copy #2 (to simplify feedback routing)
  assign wb_result1_cp2 = wb_alu_1clk_result_cp2   |
                          wb_div_result_cp2        | wb_mul_result_cp2 |
                          wb_fpxx_arith_res_hi_cp2 |
                          wb_lsu_result_cp2        | wb_mfspr_result_cp2;
  // copy #3 (to simplify feedback routing)
  assign wb_result1_cp3 = wb_alu_1clk_result_cp3   |
                          wb_div_result_cp3        | wb_mul_result_cp3 |
                          wb_fpxx_arith_res_hi_cp3 |
                          wb_lsu_result_cp3        | wb_mfspr_result_cp3;
  // --- FPU64 extention ---
  assign wb_result2 = wb_fpxx_arith_res_lo;
  // copy #1 (to simplify feedback routing)
  assign wb_result2_cp1 = wb_fpxx_arith_res_lo_cp1;
  // copy #2 (to simplify feedback routing)
  assign wb_result2_cp2 = wb_fpxx_arith_res_lo_cp2;
  // copy #3 (to simplify feedback routing)
  assign wb_result2_cp3 = wb_fpxx_arith_res_lo_cp3;

  //------------------------------------//
  // WB: External Interrupts Collection //
  //------------------------------------//
  wire exec_tt_interrupt  = tt_rdy  & tt_interrupt_enable  & exec_interrupts_en; // from "Tick Timer"
  wire exec_pic_interrupt = pic_rdy & pic_interrupt_enable & exec_interrupts_en; // from "Programmble Interrupt Controller"
  // --- wb-latches ---
  always @(posedge cpu_clk) begin
    if (cpu_rst | pipeline_flush) begin  // WB: External Interrupts Collection
      wb_tt_interrupt_r   <= 1'b0;
      wb_pic_interrupt_r  <= 1'b0;
    end
    else if (padv_wb) begin  // WB: External Interrupts Collection
      wb_tt_interrupt_r   <= exec_tt_interrupt;
      wb_pic_interrupt_r  <= exec_pic_interrupt;
    end
  end // @clock


  //--------------------------------//
  // RFE & IFETCH/DECODE EXCEPTIONS //
  //--------------------------------//

  always @(posedge cpu_clk) begin
    if (cpu_rst | pipeline_flush) begin
      // RFE
      wb_op_rfe_r            <= 1'b0;
      // FETCH/DECODE exceptions
      wb_except_ibus_err_r   <= 1'b0;
      wb_except_ipagefault_r <= 1'b0;
      wb_except_itlb_miss_r  <= 1'b0;
      wb_except_ibus_align_r <= 1'b0;
      // DECODE exceptions
      wb_except_illegal_r    <= 1'b0;
      wb_except_syscall_r    <= 1'b0;
      wb_except_trap_r       <= 1'b0;
    end
    else if (padv_wb) begin
      // RFE
      wb_op_rfe_r            <= exec_op_rfe;
      // IFETCH exceptions
      wb_except_ibus_err_r   <= exec_except_ibus_err;
      wb_except_ipagefault_r <= exec_except_ipagefault;
      wb_except_itlb_miss_r  <= exec_except_itlb_miss;
      wb_except_ibus_align_r <= exec_except_ibus_align;
      // DECODE exceptions
      wb_except_illegal_r    <= exec_except_illegal;
      wb_except_syscall_r    <= exec_except_syscall;
      wb_except_trap_r       <= exec_except_trap;
    end
  end // @clock

  //---------------------------------------//
  // WB: Combined exception/interrupt flag //
  //---------------------------------------//
  assign exec_an_except = exec_except_ibus_err     | exec_except_ipagefault    |  // EXEC-AN-EXCEPT
                          exec_except_itlb_miss    | exec_except_ibus_align    |  // EXEC-AN-EXCEPT
                          exec_except_illegal      | exec_except_syscall       |  // EXEC-AN-EXCEPT
                          exec_except_trap         |                              // EXEC-AN-EXCEPT
                          exec_except_overflow_div | exec_except_overflow_1clk |  // EXEC-AN-EXCEPT
                          exec_except_fpxx_cmp     | exec_except_fpxx_arith    |  // EXEC-AN-EXCEPT
                          exec_an_except_lsu       | sbuf_err                  |  // EXEC-AN-EXCEPT
                          exec_tt_interrupt        | exec_pic_interrupt;          // EXEC-AN-EXCEPT
  // --- wb-latch ---
  always @(posedge cpu_clk) begin
    if (cpu_rst | pipeline_flush) // WB: combined exception/interrupt flag
      wb_an_except_r <= 1'b0;
    else if (padv_wb) // WB: combined exception/interrupt flag
      wb_an_except_r <= exec_an_except;
  end // @clock


  //-------//
  // TIMER //
  //-------//
  //  # connection wires
  wire [31:0] spr_bus_dat_tt;
  wire        spr_bus_ack_tt;
  //  # timer instance
  mor1kx_ticktimer_marocchino
  u_ticktimer
  (
    // Wishbone clock and reset
    .wb_clk             (wb_clk), // TIMER
    .wb_rst             (wb_rst), // TIMER
    // CPU clock and reset
    .cpu_clk            (cpu_clk), // TIMER
    .cpu_rst            (cpu_rst), // TIMER
    // ready flag
    .tt_rdy_o           (tt_rdy), // TIMER
    // SPR interface
    .spr_bus_addr_i     (spr_bus_addr_o), // TIMER
    .spr_bus_we_i       (spr_bus_we_o), // TIMER
    .spr_bus_stb_i      (spr_bus_stb_o), // TIMER
    .spr_bus_toggle_i   (spr_bus_toggle), // TIMER
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
  mor1kx_pic_marocchino
  #(
    .OPTION_PIC_TRIGGER   (OPTION_PIC_TRIGGER), // PIC
    .OPTION_PIC_NMI_WIDTH (OPTION_PIC_NMI_WIDTH) // PIC
  )
  u_pic
  (
    // Wishbone clock and reset
    .wb_clk             (wb_clk), // PIC
    .wb_rst             (wb_rst), // PIC
    // CPU clock and reset
    .cpu_clk            (cpu_clk), // PIC
    .cpu_rst            (cpu_rst), // PIC
    // input interrupt lines
    .irq_i              (irq_i), // PIC
    // output interrupt lines
    .pic_rdy_o          (pic_rdy), // PIC
    // SPR BUS
    //  # inputs
    .spr_bus_addr_i     (spr_bus_addr_o), // PIC
    .spr_bus_we_i       (spr_bus_we_o), // PIC
    .spr_bus_stb_i      (spr_bus_stb_o), // PIC
    .spr_bus_toggle_i   (spr_bus_toggle), // PIC
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
    .FEATURE_MULTICORE          (FEATURE_MULTICORE), // CTRL
    .OPTION_RF_NUM_SHADOW_GPR   (OPTION_RF_NUM_SHADOW_GPR) // CTRL
  )
  u_ctrl
  (
    // clocks & resets
    .cpu_clk                          (cpu_clk), // CTRL
    .cpu_rst                          (cpu_rst), // CTRL

    // Inputs / Outputs for pipeline control signals
    .fetch_valid_i                    (fetch_valid), // CTRL
    .dcod_empty_i                     (dcod_empty), // CTRL
    .dcod_free_i                      (dcod_free), // CTRL
    .dcod_valid_i                     (dcod_valid), // CTRL
    .exec_valid_i                     (exec_valid), // CTRL
    .pipeline_flush_o                 (pipeline_flush), // CTRL
    .padv_fetch_o                     (padv_fetch), // CTRL
    .padv_dcod_o                      (padv_dcod), // CTRL
    .padv_exec_o                      (padv_exec), // CTRL
    .padv_wb_o                        (padv_wb), // CTRL

    // MF(T)SPR coomand processing
    //  ## iput data & command from DECODE
    .dcod_rfa1_i                      (dcod_rfa1[`OR1K_IMM_WIDTH-1:0]), // CTRL: base of addr for MT(F)SPR
    .dcod_imm16_i                     (dcod_imm16), // CTRL: offset for addr for MT(F)SPR
    .dcod_rfb1_i                      (dcod_rfb1), // CTRL: data for MTSPR
    .dcod_op_mtspr_i                  (dcod_op_mtspr), // CTRL
    .dcod_op_mXspr_i                  (dcod_op_mXspr), // CTRL
    //  ## result to WB_MUX
    .wb_mfspr_result_o                (wb_mfspr_result), // CTRL: for WB_MUX
    .wb_mfspr_result_cp1_o            (wb_mfspr_result_cp1), // CTRL: for WB_MUX
    .wb_mfspr_result_cp2_o            (wb_mfspr_result_cp2), // CTRL: for WB_MUX
    .wb_mfspr_result_cp3_o            (wb_mfspr_result_cp3), // CTRL: for WB_MUX

    // Support IBUS error handling in CTRL
    .wb_jump_or_branch_i              (wb_jump_or_branch), // CTRL
    .wb_do_branch_i                   (wb_do_branch), // CTRL
    .wb_do_branch_target_i            (wb_do_branch_target), // CTRL

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
    .spr_bus_toggle_o                 (spr_bus_toggle), // CTRL
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
    .spr_bus_dat_gpr0_i               (spr_bus_dat_gpr0), // CTRL
    .spr_bus_ack_gpr0_i               (spr_bus_ack_gpr0), // CTRL
    .spr_bus_dat_gprS_i               (spr_bus_dat_gprS), // CTRL
    .spr_bus_ack_gprS_i               (spr_bus_ack_gprS), // CTRL

    // WB: External Interrupt Collection
    .tt_interrupt_enable_o            (tt_interrupt_enable), // CTRL
    .pic_interrupt_enable_o           (pic_interrupt_enable), // CTRL
    .wb_tt_interrupt_i                (wb_tt_interrupt_r), // CTRL
    .wb_pic_interrupt_i               (wb_pic_interrupt_r), // CTRL

    // WB: programm counter
    .pc_wb_i                          (pc_wb), // CTRL

    // WB: flag
    .wb_int_flag_set_i                (wb_int_flag_set), // CTRL
    .wb_int_flag_clear_i              (wb_int_flag_clear), // CTRL
    .wb_fpxx_flag_set_i               (wb_fpxx_flag_set), // CTRL
    .wb_fpxx_flag_clear_i             (wb_fpxx_flag_clear), // CTRL
    .wb_atomic_flag_set_i             (wb_atomic_flag_set), // CTRL
    .wb_atomic_flag_clear_i           (wb_atomic_flag_clear), // CTRL
    .wb_flag_wb_i                     (wb_flag_wb), // CTRL

    // WB: carry
    .wb_div_carry_set_i               (wb_div_carry_set), // CTRL
    .wb_div_carry_clear_i             (wb_div_carry_clear), // CTRL
    .wb_1clk_carry_set_i              (wb_1clk_carry_set), // CTRL
    .wb_1clk_carry_clear_i            (wb_1clk_carry_clear), // CTRL
    .wb_carry_wb_i                    (wb_carry_wb), // CTRL

    // WB: overflow
    .wb_div_overflow_set_i            (wb_div_overflow_set), // CTRL
    .wb_div_overflow_clear_i          (wb_div_overflow_clear), // CTRL
    .wb_1clk_overflow_set_i           (wb_1clk_overflow_set), // CTRL
    .wb_1clk_overflow_clear_i         (wb_1clk_overflow_clear), // CTRL

    //  # FPX3264 arithmetic part
    .wb_fpxx_arith_fpcsr_i            (wb_fpxx_arith_fpcsr), // CTRL
    .wb_fpxx_arith_wb_fpcsr_i         (wb_fpxx_arith_wb_fpcsr), // CTRL
    .wb_except_fpxx_arith_i           (wb_except_fpxx_arith), // CTRL
    //  # FPX64 comparison part
    .wb_fpxx_cmp_inv_i                (wb_fpxx_cmp_inv), // CTRL
    .wb_fpxx_cmp_inf_i                (wb_fpxx_cmp_inf), // CTRL
    .wb_fpxx_cmp_wb_fpcsr_i           (wb_fpxx_cmp_wb_fpcsr), // CTRL
    .wb_except_fpxx_cmp_i             (wb_except_fpxx_cmp), // CTRL

    //  # Excepion processing auxiliaries
    .sbuf_eear_i                      (sbuf_eear), // CTRL
    .sbuf_epcr_i                      (sbuf_epcr), // CTRL
    .sbuf_err_i                       (sbuf_err), // CTRL
    .wb_delay_slot_i                  (wb_delay_slot), // CTRL

    //  # combined exceptions/interrupt flag
    .exec_an_except_i                 (exec_an_except), // CTRL
    .wb_an_except_i                   (wb_an_except_r), // CTRL

    //  # particular IFETCH exception flags
    .wb_except_ibus_err_i             (wb_except_ibus_err_r), // CTRL
    .wb_except_itlb_miss_i            (wb_except_itlb_miss_r), // CTRL
    .wb_except_ipagefault_i           (wb_except_ipagefault_r), // CTRL
    .wb_except_ibus_align_i           (wb_except_ibus_align_r), // CTRL

    //  # particular DECODE exception flags
    .wb_except_illegal_i              (wb_except_illegal_r), // CTRL
    .wb_except_syscall_i              (wb_except_syscall_r), // CTRL
    .wb_except_trap_i                 (wb_except_trap_r), // CTRL

    //  # particular LSU exception flags
    .wb_except_dbus_err_i             (wb_except_dbus_err), // CTRL
    .wb_except_dtlb_miss_i            (wb_except_dtlb_miss), // CTRL
    .wb_except_dpagefault_i           (wb_except_dpagefault), // CTRL
    .wb_except_dbus_align_i           (wb_except_dbus_align), // CTRL
    .wb_lsu_except_addr_i             (wb_lsu_except_addr), // CTRL

    //  # overflow exception processing
    .except_overflow_enable_o         (except_overflow_enable), // CTRL
    .wb_except_overflow_div_i         (wb_except_overflow_div), // CTRL
    .wb_except_overflow_1clk_i        (wb_except_overflow_1clk), // CTRL

    //  # Branch to exception/rfe processing address
    .ctrl_branch_exception_o          (ctrl_branch_exception), // CTRL
    .ctrl_branch_except_pc_o          (ctrl_branch_except_pc), // CTRL
    .fetch_exception_taken_i          (fetch_ecxeption_taken), // CTRL
    //  # l.rfe
    .exec_op_rfe_i                    (exec_op_rfe), // CTRL
    .wb_op_rfe_i                      (wb_op_rfe_r), // CTRL

    // Multicore related
    .multicore_coreid_i               (multicore_coreid_i), // CTRL
    .multicore_numcores_i             (multicore_numcores_i), // CTRL

    // Flag & Carry
    .ctrl_flag_o                      (ctrl_flag), // CTRL
    .ctrl_flag_sr_o                   (ctrl_flag_sr), // CTRL
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

   always @(posedge cpu_clk) begin
      if (FEATURE_TRACEPORT_EXEC != "NONE") begin
   if (cpu_rst) begin
      traceport_waitexec <= 0;
   end else begin
      if (padv_exec) begin
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
   end // else: !if(cpu_rst)
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
   assign traceport_exec_wbreg_o = ?;
   assign traceport_exec_wben_o = ?;
   assign traceport_exec_wbdata_o = wb_result1;
      end else begin
   assign traceport_exec_wbreg_o = {OPTION_RF_ADDR_WIDTH{1'b0}};
   assign traceport_exec_wben_o = 1'b0;
   assign traceport_exec_wbdata_o = {OPTION_OPERAND_WIDTH{1'b0}};
      end
   endgenerate
*/
endmodule // mor1kx_cpu_marocchino
