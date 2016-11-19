////////////////////////////////////////////////////////////////////////
//                                                                    //
//  mor1kx_lsu_marocchino                                             //
//                                                                    //
//  Description: Data bus interface for MAROCCHINO pipeline           //
//               Dbus interface request signal out synchronous        //
//               32-bit specific                                      //
//                                                                    //
//               Derived from mor1kx_lsu_cappuccino                   //
//                                                                    //
////////////////////////////////////////////////////////////////////////
//                                                                    //
//   Copyright (C) 2012 Julius Baxter                                 //
//                      juliusbaxter@gmail.com                        //
//                                                                    //
//   Copyright (C) 2013 Stefan Kristiansson                           //
//                      stefan.kristiansson@saunalahti.fi             //
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

module mor1kx_lsu_marocchino
#(
  parameter OPTION_OPERAND_WIDTH        = 32,
  parameter DEST_REG_ADDR_WIDTH         =  8, // OPTION_RF_ADDR_WIDTH + log2(Re-Ordering buffer width)
  // data cache
  parameter OPTION_DCACHE_BLOCK_WIDTH   = 5,
  parameter OPTION_DCACHE_SET_WIDTH     = 9,
  parameter OPTION_DCACHE_WAYS          = 2,
  parameter OPTION_DCACHE_LIMIT_WIDTH   = 32,
  parameter OPTION_DCACHE_SNOOP         = "NONE",
  parameter OPTION_DCACHE_CLEAR_ON_INIT = 0,
  // mmu cache
  parameter FEATURE_DMMU_HW_TLB_RELOAD = "NONE",
  parameter OPTION_DMMU_SET_WIDTH      = 6,
  parameter OPTION_DMMU_WAYS           = 1,
  parameter OPTION_DMMU_CLEAR_ON_INIT  = 0,
  // store buffer
  parameter OPTION_STORE_BUFFER_DEPTH_WIDTH   = 4, // 16 taps
  parameter OPTION_STORE_BUFFER_CLEAR_ON_INIT = 0
)
(
  // clocks & resets
  input                                 clk,
  input                                 rst,
  // Pipeline controls
  input                                 pipeline_flush_i,
  input                                 padv_decode_i,
  input                                 padv_wb_i,
  input                                 grant_wb_to_lsu_i,
  // configuration
  input                                 dc_enable_i,
  input                                 dmmu_enable_i,
  input                                 supervisor_mode_i,
  // Input from DECODE (not latched)
  input                                 dcod_delay_slot_i, // for store buffer EPCR computation
  input      [OPTION_OPERAND_WIDTH-1:0] pc_decode_i,       // for store buffer EPCR computation
  input           [`OR1K_IMM_WIDTH-1:0] dcod_imm16_i, // immediate offset for address computation
  input      [OPTION_OPERAND_WIDTH-1:0] dcod_rfa_i,   // operand "A" (part of address)
  input      [OPTION_OPERAND_WIDTH-1:0] dcod_rfb_i,   // operand "B" (value to store)
  input                                 dcod_op_lsu_load_i,
  input                                 dcod_op_lsu_store_i,
  input                                 dcod_op_lsu_atomic_i,
  input                           [1:0] dcod_lsu_length_i,
  input                                 dcod_lsu_zext_i,
  input                                 dcod_op_msync_i,
  // OMAN-to-DECODE hazards
  //  combined flag
  input                                 omn2dec_hazards_i,
  //  by operands
  input                                 busy_hazard_a_i,
  input       [DEST_REG_ADDR_WIDTH-1:0] busy_hazard_a_adr_i,
  input                                 busy_hazard_b_i,
  input       [DEST_REG_ADDR_WIDTH-1:0] busy_hazard_b_adr_i,
  // EXEC-to-DECODE hazards
  //  combined flag
  input                                 exe2dec_hazards_i,
  //  by operands
  input                                 exe2dec_hazard_a_i,
  input                                 exe2dec_hazard_b_i,
  // Data for hazards resolving
  //  hazard could be passed from DECODE to EXECUTE
  input                                 exec_rf_wb_i,
  input       [DEST_REG_ADDR_WIDTH-1:0] exec_rfd_adr_i,
  //  hazard could be resolving
  input                                 wb_rf_wb_i,
  input       [DEST_REG_ADDR_WIDTH-1:0] wb_rfd_adr_i,
  input      [OPTION_OPERAND_WIDTH-1:0] wb_result_i,
  // SPR interface
  input                          [15:0] spr_bus_addr_i,
  input                                 spr_bus_we_i,
  input                                 spr_bus_stb_i,
  input      [OPTION_OPERAND_WIDTH-1:0] spr_bus_dat_i,
  output     [OPTION_OPERAND_WIDTH-1:0] spr_bus_dat_dc_o,
  output                                spr_bus_ack_dc_o,
  output     [OPTION_OPERAND_WIDTH-1:0] spr_bus_dat_dmmu_o,
  output                                spr_bus_ack_dmmu_o,
  // interface to data bus
  output reg [OPTION_OPERAND_WIDTH-1:0] dbus_adr_o,
  output reg                            dbus_req_o,
  output reg [OPTION_OPERAND_WIDTH-1:0] dbus_dat_o,
  output reg                      [3:0] dbus_bsel_o,
  output                                dbus_we_o,
  output                                dbus_burst_o,
  input                                 dbus_err_i,
  input                                 dbus_ack_i,
  input      [OPTION_OPERAND_WIDTH-1:0] dbus_dat_i,
  // Cache sync for multi-core environment
  input                          [31:0] snoop_adr_i,
  input                                 snoop_en_i,
  // Exceprions & errors
  //  # Indicator of dbus exception came via the store buffer
  //    and appropriate PC
  output reg [OPTION_OPERAND_WIDTH-1:0] sbuf_eear_o,
  output reg [OPTION_OPERAND_WIDTH-1:0] sbuf_epcr_o,
  output reg                            sbuf_err_o,
  // output flags and load result
  output                                lsu_busy_o,
  output                                lsu_valid_o, // result ready or exceptions
  output reg [OPTION_OPERAND_WIDTH-1:0] wb_lsu_result_o,
  // exception output
  //  # particular LSU exception flags
  output reg                            wb_except_dbus_err_o,
  output reg                            wb_except_dpagefault_o,
  output reg                            wb_except_dtlb_miss_o,
  output reg                            wb_except_dbus_align_o,
  output reg [OPTION_OPERAND_WIDTH-1:0] wb_lsu_except_addr_o,
  //  # combined LSU exceptions flag
  output reg                            wb_an_except_lsu_o,
  // Atomic operation flag set/clear logic
  output reg                            wb_atomic_flag_set_o,
  output reg                            wb_atomic_flag_clear_o
);

  // Input registers for LSU address calculation stage
  //  # registers for command from DECODE
  wire                            lsu_load_w;
  wire                            lsu_store_w;
  wire                            lsu_atomic_w;
  wire                      [1:0] lsu_length_w;
  wire                            lsu_zext_w;
  //  # registers for operands from DECODE
  wire      [`OR1K_IMM_WIDTH-1:0] lsu_imm16_w; // immediate offset for address computation
  //  # registers for store buffer EPCR computation
  wire [OPTION_OPERAND_WIDTH-1:0] pc_decode_w;
  //  # operands after frorwarding from WB
  wire [OPTION_OPERAND_WIDTH-1:0] lsu_a;
  wire [OPTION_OPERAND_WIDTH-1:0] lsu_b;


  // Input register for DBUS/DCACHE access stage
  //  # load/store
  reg                            cmd_load_r;  // either atomic or not
  reg                            cmd_lwa_r;   // exactly load linked
  reg                            cmd_store_r; // not (!!!) atomic
  reg                            cmd_swa_r;   // atomic only
  reg                      [1:0] cmd_length;
  reg                            cmd_zext;
  reg [OPTION_OPERAND_WIDTH-1:0] cmd_rfb;  // register file B in (store operand)
  //  # registers for store buffer EPCR computation
  reg [OPTION_OPERAND_WIDTH-1:0] cmd_epcr;
  //  # latched virtual address
  reg [OPTION_OPERAND_WIDTH-1:0] virt_addr_cmd;
  //  # l.msync
  reg                            cmd_msync_r;


  // DBUS FSM
  //  # DBUS FSM states
  localparam [4:0] DBUS_IDLE      = 5'b00001, // eq. to DCACHE
                   DMEM_REQ       = 5'b10000,
                   DBUS_READ      = 5'b00010, // eq. to DCACHE
                   DBUS_WRITE     = 5'b00100, // eq. to DCACHE
                   DBUS_DC_REFILL = 5'b01000; // eq. to DCACHE
  //  # DBUS FSM state indicator
  reg        [4:0] dbus_state;
  //  # DBUS FSM other registers & wires
  reg                               dbus_we;
  reg                               dbus_atomic;
  wire                              dbus_swa_discard; // reservation is lost, execute empty read
  wire                              dbus_swa_success; // l.swa is successfull
  wire                              dbus_swa_ack;     // complete DBUS trunsaction with l.swa
  reg                               sbuf_odata;       // not written data on buffer's output


  // DMMU
  wire                              dmmu_cache_inhibit;
  wire   [OPTION_OPERAND_WIDTH-1:0] phys_addr_cmd;
  /* HW reload TLB related (MAROCCHINO_TODO : not implemented yet)
  wire                              tlb_reload_req;
  wire                              tlb_reload_busy;
  wire [OPTION_OPERAND_WIDTH-1:0]   tlb_reload_addr;
  wire                              tlb_reload_pagefault;
  reg                               tlb_reload_ack;
  reg [OPTION_OPERAND_WIDTH-1:0]    tlb_reload_data;
  wire                              tlb_reload_pagefault_clear;
  reg                               tlb_reload_done; */


  // DCACHE
  wire                              dc_ack;
  wire   [OPTION_OPERAND_WIDTH-1:0] dc_dat;
  wire                              dc_access;
  wire                              dc_refill_req;
  reg                               dc_refill_allowed; // combinatorial
  wire   [OPTION_OPERAND_WIDTH-1:0] next_refill_adr;
  wire                              dc_refill_last;


  // Store buffer
  wire                              sbuf_write; // without exceptions and pipeline-flush
  wire                              sbuf_we;    // with exceptions and pipeline-flush
  // ---
  wire                              sbuf_rdwr_empty;  // read and write simultaneously if buffer is empty
  wire                              sbuf_re;          // with exceptions and pipeline-flush
  // ---
  wire                              sbuf_full;
  wire                              sbuf_empty;
  wire   [OPTION_OPERAND_WIDTH-1:0] sbuf_epcr;
  wire   [OPTION_OPERAND_WIDTH-1:0] sbuf_virt_addr;
  wire   [OPTION_OPERAND_WIDTH-1:0] sbuf_phys_addr;
  wire   [OPTION_OPERAND_WIDTH-1:0] sbuf_dat;
  wire [OPTION_OPERAND_WIDTH/8-1:0] sbuf_bsel;

  // Atomic operations
  reg    [OPTION_OPERAND_WIDTH-1:0] atomic_addr;
  reg                               atomic_reserve;

  // Snoop (for multicore SoC)
  wire                              snoop_event;
  wire                              snoop_hit;


  // Exceptions detected on DCACHE/DBUS access stage
  // # exceptions related to address computation and conversion
  wire                              except_align;
  wire                              except_dtlb_miss;
  wire                              except_dpagefault;
  // # combination of all previous
  wire                              lsu_excepts_addr;

  // Instant DBUS acceess error
  wire                              dbus_err_instant;
  // # combination of all exceptions
  wire                              excepts_any;  // combined of previous

  // Exceptions latched for WB
  reg                               except_dbus_err_r;
  reg                               except_align_r;
  reg                               except_dtlb_miss_r;
  reg                               except_dpagefault_r;
  reg                               excepts_any_r;  // any of previous


  // load/store data
  wire   [OPTION_OPERAND_WIDTH-1:0] lsu_ldat; // formated load data
  wire   [OPTION_OPERAND_WIDTH-1:0] lsu_sdat; // formated store data

  // LSU pipe controls
  //  # report on command execution
  wire                              lsu_ack_load;
  wire                              lsu_ack_store;
  wire                              lsu_ack_swa;
  reg                               lsu_ack_load_pending;
  reg                               lsu_ack_store_pending;
  //  # busy of various stages
  wire                              lsu_busy_rsrvs; // resrevation station is full
  wire                              lsu_busy_load;  // waiting load completions
  wire                              lsu_busy_store; // waiting store (either conditional or not) completion
  wire                              lsu_busy_mem;   // DCACHE/DBUS access stage busy
  wire                              lsu_busy_wb;    // Result is waiting WB access

  // Flushing logic provides continuous clean up output
  // from pipeline-flush till transaction (read/re-fill) completion
  reg                               flush_r;
  wire                              flush_by_ctrl;


  // short names for local use
  localparam LSUOOW = OPTION_OPERAND_WIDTH;


  //-------------------//
  // LSU pipe controls //
  //-------------------//

  // Exceptions detected on DCACHE/DBUS access stage
  //  # exceptions related to address computation and conversion
  assign lsu_excepts_addr = except_align | except_dtlb_miss | except_dpagefault;
  //  # all exceptions
  assign excepts_any  = lsu_excepts_addr | dbus_err_instant;


  // Signal to take new LSU command (less priority than flushing or exceptions)
  // Pay attention:
  //   The signal couldn't be raised if lsu-excepts-wb is asserted because
  //   (a) lsu-load-r and lsu-store-r already cleaned by lsu-excepts-any
  //   (b) LSU reports busy by asserting lsu-busy-o, so no new command could arrive
  wire lsu_takes_load  = lsu_load_w  & ~(lsu_busy_mem | lsu_busy_wb); // for DCACHE mostly
  wire lsu_takes_store = lsu_store_w & ~(lsu_busy_mem | lsu_busy_wb); // for DCACHE mostly
  wire lsu_takes_ls    = lsu_takes_load | lsu_takes_store;


  // report on command execution
  //  # load completion / waiting
  assign lsu_ack_load   = cmd_load_r &   (((dbus_state == DBUS_READ) & dbus_ack_i) | dc_ack);
  assign lsu_busy_load  = cmd_load_r & ~((((dbus_state == DBUS_READ) & dbus_ack_i) | dc_ack) & padv_wb_i & grant_wb_to_lsu_i);
  //  # store completion
  assign lsu_ack_store  = sbuf_write; // it already includes cmd_store_r
  assign lsu_ack_swa    = cmd_swa_r & dbus_swa_ack;
  assign lsu_busy_store = (cmd_store_r & ~(sbuf_write & padv_wb_i & grant_wb_to_lsu_i)) |   // waiting store completion
                          (cmd_swa_r   & ~(dbus_swa_ack & padv_wb_i & grant_wb_to_lsu_i));  // waiting store completion


  // output assignement (1-clk ahead for WB-latching)
  assign lsu_valid_o = lsu_ack_load_pending  | lsu_ack_load  |               // LSU result ready or exceptions
                       lsu_ack_store_pending | lsu_ack_store | lsu_ack_swa | // LSU result ready or exceptions
                       excepts_any_r;                                        // LSU result ready or exceptions


  // LSU is busy
  //  # DCACHE/DBUS access stage busy
  assign lsu_busy_mem = lsu_busy_load  | (dbus_state == DBUS_DC_REFILL) |  // DCACHE/DBUS access stage busy
                        lsu_busy_store |                                   // DCACHE/DBUS access stage busy
                        cmd_msync_r    | snoop_hit | flush_r;              // DCACHE/DBUS access stage busy
  //  # Result is waiting WB access
  assign lsu_busy_wb = (lsu_ack_load_pending | lsu_ack_store_pending) & ~(padv_wb_i & grant_wb_to_lsu_i);
  //  # BUSY reported to execution [O]rder [MAN]ager, OMAN
  assign lsu_busy_o = lsu_busy_rsrvs | excepts_any_r; // overall busy


  // Flushing from pipeline-flush-i till DBUS transaction completion
  //  # conditions to assert flush-r
  wire assert_flush_r = (((dbus_state == DBUS_READ) | (dbus_state == DBUS_WRITE)) & ~dbus_ack_i) | // assert flush-r
                        (dbus_state == DBUS_DC_REFILL);                                            // assert flush-r
  //  # conditions to de-assert flush-r
  wire deassert_flush_r = (((dbus_state == DBUS_READ) | (dbus_state == DBUS_WRITE)) & dbus_ack_i) | // de-assert flush-r
                          (dbus_state == DBUS_IDLE);                                                // de-assert flush-r
  // ---
  always @(posedge clk `OR_ASYNC_RST) begin
    if (rst)
      flush_r  <= 1'b0; // on reset
    else if (flush_r & deassert_flush_r)
      flush_r  <= 1'b0; // on de-assert
    else if (pipeline_flush_i & assert_flush_r)
      flush_r  <= 1'b1; // on assert
  end // @clock
  // ---
  assign flush_by_ctrl = pipeline_flush_i | flush_r;


  //---------------------------//
  // Address computation stage //
  //---------------------------//

  //   To clean up registers on the stage we use only pipeline-flush, but not flush-r.
  // It makes possible to get next to flush instruction and start execution just after
  // completion DBUS transaction which has been flushed by pipeline flush.

  // EPCR for store buffer
  //  ## delay-slot ? (pc-4) : pc
  wire [LSUOOW-1:0] pc_decode_ds = pc_decode_i - {{(LSUOOW-3){1'b0}},dcod_delay_slot_i,2'b00};

  // attributes include:
  //  ## separate load, store and atomic flags: averall 3
  //  ## length: 2
  //  ## zero extension: 1
  //  ## immediate width: 16
  //  ## PC address width: 32
  localparam LSU_ATTR_WIDTH = 6 + `OR1K_IMM_WIDTH + OPTION_OPERAND_WIDTH;

  // reservation station instance
  mor1kx_rsrvs_marocchino
  #(
    .OPTION_OPERAND_WIDTH     (OPTION_OPERAND_WIDTH), // LSU_RSVRS
    .USE_OPC                  (1), // LSU_RSVRS
    .OPC_WIDTH                (LSU_ATTR_WIDTH), // LSU_RSVRS
    .DEST_REG_ADDR_WIDTH      (DEST_REG_ADDR_WIDTH), // LSU_RSVRS
    .FEATURE_FPU              ("NONE"), // LSU_RSVRS
    .USE_RSVRS_FLAG_CARRY     (0), // LSU_RSVRS
    .DEST_FLAG_ADDR_WIDTH     (1) // LSU_RSVRS
  )
  u_lsu_rsrvs
  (
    // clocks and resets
    .clk                      (clk),
    .rst                      (rst),
    // pipeline control signals in
    .pipeline_flush_i         (excepts_any | pipeline_flush_i), // LSU_RSVRS
    .padv_decode_i            (padv_decode_i), // LSU_RSVRS
    .taking_op_i              (lsu_takes_ls), // LSU_RSVRS
    // input data from DECODE
    .dcod_rfa_i               (dcod_rfa_i), // LSU_RSVRS
    .dcod_rfb_i               (dcod_rfb_i), // LSU_RSVRS
    // for FPU64
    .dcod_rfa2_i              ({OPTION_OPERAND_WIDTH{1'b0}}), // LSU_RSVRS
    .dcod_rfb2_i              ({OPTION_OPERAND_WIDTH{1'b0}}), // LSU_RSVRS
    // OMAN-to-DECODE hazards
    //  combined flag
    .omn2dec_hazards_i        (omn2dec_hazards_i), // LSU_RSVRS
    //  by FLAG and CARRY
    .busy_hazard_f_i          (1'b0), // LSU_RSVRS
    .busy_hazard_f_adr_i      (1'b0), // LSU_RSVRS
    .busy_hazard_c_i          (1'b0), // LSU_RSVRS
    .busy_hazard_c_adr_i      (1'b0), // LSU_RSVRS
    //  by operands
    .busy_hazard_a_i          (busy_hazard_a_i), // LSU_RSVRS
    .busy_hazard_a_adr_i      (busy_hazard_a_adr_i), // LSU_RSVRS
    .busy_hazard_b_i          (busy_hazard_b_i), // LSU_RSVRS
    .busy_hazard_b_adr_i      (busy_hazard_b_adr_i), // LSU_RSVRS
    // for FPU64
    .busy_hazard_a2_i         (1'b0), // LSU_RSVRS
    .busy_hazard_a2_adr_i     ({DEST_REG_ADDR_WIDTH{1'b0}}), // LSU_RSVRS
    .busy_hazard_b2_i         (1'b0), // LSU_RSVRS
    .busy_hazard_b2_adr_i     ({DEST_REG_ADDR_WIDTH{1'b0}}), // LSU_RSVRS
    // EXEC-to-DECODE hazards
    //  combined flag
    .exe2dec_hazards_i        (exe2dec_hazards_i), // LSU_RSVRS
    //  by operands
    .exe2dec_hazard_a_i       (exe2dec_hazard_a_i), // LSU_RSVRS
    .exe2dec_hazard_b_i       (exe2dec_hazard_b_i), // LSU_RSVRS
    // for FPU64
    .exe2dec_hazard_a2_i      (1'b0), // LSU_RSVRS
    .exe2dec_hazard_b2_i      (1'b0), // LSU_RSVRS
    // Hazard could be passed from DECODE to EXECUTE
    //  ## FLAG or CARRY
    .exec_flag_wb_i           (1'b0), // LSU_RSVRS
    .exec_carry_wb_i          (1'b0), // LSU_RSVRS
    .exec_flag_carry_adr_i    (1'b0), // LSU_RSVRS
    //  ## A or B operand
    .exec_rf_wb_i             (exec_rf_wb_i), // LSU_RSVRS
    .exec_rfd_adr_i           (exec_rfd_adr_i), // LSU_RSVRS
    //  ## for FPU64
    .exec_rf_wb2_i            (1'b0), // LSU_RSVRS
    .exec_rfd2_adr_i          ({DEST_REG_ADDR_WIDTH{1'b0}}), // LSU_RSVRS
    //  ## passing only with writting back
    .padv_wb_i                (padv_wb_i), // LSU_RSVRS
    // Hazard could be resolving
    //  ## FLAG or CARRY
    .wb_flag_wb_i             (1'b0), // LSU_RSVRS
    .wb_carry_wb_i            (1'b0), // LSU_RSVRS
    .wb_flag_carry_adr_i      (1'b0), // LSU_RSVRS
    //  ## A or B operand
    .wb_rf_wb_i               (wb_rf_wb_i), // LSU_RSVRS
    .wb_rfd_adr_i             (wb_rfd_adr_i), // LSU_RSVRS
    .wb_result_i              (wb_result_i), // LSU_RSVRS
    //  ## for FPU64
    .wb_rf_wb2_i              (1'b0), // LSU_RSVRS
    .wb_rfd2_adr_i            ({DEST_REG_ADDR_WIDTH{1'b0}}), // LSU_RSVRS
    .wb_result2_i             ({LSUOOW{1'b0}}), // LSU_RSVRS
    // command and its additional attributes
    .dcod_op_i                (dcod_op_lsu_load_i | dcod_op_lsu_store_i), // LSU_RSVRS
    .dcod_opc_i               ({dcod_op_lsu_load_i,dcod_op_lsu_store_i,dcod_op_lsu_atomic_i, // LSU_RSVRS
                                dcod_lsu_length_i,dcod_lsu_zext_i,dcod_imm16_i,pc_decode_ds}), // LSU_RSVRS
    // outputs
    //   command attributes from busy stage
    .busy_opc_o               (), // LSU_RSVRS
    //   command and its additional attributes
    .exec_op_o                (), // LSU_RSVRS
    .exec_opc_o               ({lsu_load_w,lsu_store_w,lsu_atomic_w, // LSU_RSVRS
                                lsu_length_w,lsu_zext_w,lsu_imm16_w,pc_decode_w}), // LSU_RSVRS
    //   operands
    .exec_rfa_o               (lsu_a), // LSU_RSVRS
    .exec_rfb_o               (lsu_b), // LSU_RSVRS
    //  ## for FPU64
    .exec_rfa2_o              (), // LSU_RSVRS
    .exec_rfb2_o              (), // LSU_RSVRS
    //   unit-is-busy flag
    .unit_busy_o              (lsu_busy_rsrvs) // LSU_RSVRS
  );

  // compute address
  wire [LSUOOW-1:0] virt_addr = lsu_a + {{(LSUOOW-16){lsu_imm16_w[15]}},lsu_imm16_w};


  //---------------------------//
  // DBUS/DCACHE acceess stage //
  //---------------------------//

  // latches for load (either atomic or not) commands
  always @(posedge clk `OR_ASYNC_RST) begin
    if (rst) begin
      // load
      cmd_load_r  <= 1'b0;
      cmd_lwa_r   <= 1'b0;
      // store
      cmd_store_r <= 1'b0;
      cmd_swa_r   <= 1'b0;
      // store buffer EPCR
      cmd_epcr    <= {LSUOOW{1'b0}};
    end
    else if (excepts_any | pipeline_flush_i) begin
      // load
      cmd_load_r  <= 1'b0;
      cmd_lwa_r   <= 1'b0;
      cmd_store_r <= 1'b0;
      // store
      cmd_store_r <= 1'b0;
      cmd_swa_r   <= 1'b0;
      // store buffer EPCR
      cmd_epcr    <= {LSUOOW{1'b0}};
    end
    else if (lsu_takes_ls) begin
      // load
      cmd_load_r  <= lsu_load_w;
      cmd_lwa_r   <= lsu_load_w  &  lsu_atomic_w;
      // store
      cmd_store_r <= lsu_store_w & ~lsu_atomic_w;
      cmd_swa_r   <= lsu_store_w &  lsu_atomic_w;
      // store buffer EPCR
      cmd_epcr    <= pc_decode_w;
    end
    else begin
      // load
      if (lsu_ack_load) begin
        cmd_load_r  <= 1'b0;
        cmd_lwa_r   <= 1'b0;
      end
      // classic store
      if (lsu_ack_store) begin
        cmd_store_r <= 1'b0;
        cmd_epcr    <= {LSUOOW{1'b0}};
      end
      // store conditional
      if (lsu_ack_swa)
        cmd_swa_r   <= 1'b0;
    end
  end // @clock

  // latch additional parameters of a command
  //       and calculated virtual adderss
  always @(posedge clk `OR_ASYNC_RST) begin
    if (rst) begin
      // additional parameters of a command
      cmd_length    <= 2'd0;
      cmd_zext      <= 1'b0;
      cmd_rfb       <= {LSUOOW{1'b0}};
      // calculated virtual adderss
      virt_addr_cmd <= {LSUOOW{1'b0}};
    end
    else if (excepts_any | pipeline_flush_i) begin
      // additional parameters of a command
      cmd_length    <= 2'd0;
      cmd_zext      <= 1'b0;
      cmd_rfb       <= {LSUOOW{1'b0}};
      // calculated virtual adderss
      virt_addr_cmd <= virt_addr_cmd;
    end
    else if (lsu_takes_ls) begin
      // additional parameters of a command
      cmd_length    <= lsu_length_w;
      cmd_zext      <= lsu_zext_w;
      cmd_rfb       <= lsu_b;
      // calculated virtual adderss
      virt_addr_cmd <= virt_addr;
    end
  end // @clock

  // l.msync cause LSU busy
  wire cmd_msync_deassert = cmd_msync_r & ((dbus_state == DBUS_IDLE) & sbuf_empty | dbus_err_instant); // deassert busy by l.msync
  // ---
  always @(posedge clk `OR_ASYNC_RST) begin
    if (rst)
      cmd_msync_r <= 1'b0;
    else if (padv_decode_i & dcod_op_msync_i)
      cmd_msync_r <= 1'b1;
    else if (cmd_msync_deassert)
      cmd_msync_r <= 1'b0;
  end // @clock



  //----------------//
  // LSU exceptions //
  //----------------//

  // --- bus error ---
  assign dbus_err_instant = dbus_req_o & dbus_err_i;

  // --- bus error during bus access from store buffer ---
  //  ## pay attention that l.swa is executed around of
  //     store buffer, so we don't take into accaunt
  //     atomic store here.
  always @(posedge clk `OR_ASYNC_RST) begin
    if (rst) begin
      sbuf_err_o  <= 1'b0;
      sbuf_eear_o <= {LSUOOW{1'b0}};
      sbuf_epcr_o <= {LSUOOW{1'b0}};
    end
    else if (flush_by_ctrl) begin // prevent store buffer DBUS error report
      sbuf_err_o  <= 1'b0;
      sbuf_eear_o <= {LSUOOW{1'b0}};
      sbuf_epcr_o <= {LSUOOW{1'b0}};
    end
    else if (dbus_err_instant & dbus_we & ~dbus_atomic) begin
      sbuf_err_o  <= 1'b1;
      sbuf_eear_o <= sbuf_virt_addr;
      sbuf_epcr_o <= sbuf_epcr;
    end
  end // @ clock

  // --- align error detection ---
  wire align_err_word  = |virt_addr_cmd[1:0];
  wire align_err_short = virt_addr_cmd[0];

  assign except_align = (cmd_load_r | cmd_store_r | cmd_swa_r) &      // Align Exception: detection enabled
                        (((cmd_length == 2'b10) & align_err_word) |   // Align Exception: wrong word align
                         ((cmd_length == 2'b01) & align_err_short));  // Align Exception: wrong short align

  // --- intermediate latching ---
  reg [LSUOOW-1:0] lsu_except_addr_r;
  // ---
  always @(posedge clk `OR_ASYNC_RST) begin
    if (rst) begin
      // exception flags
      except_dbus_err_r   <= 1'b0;
      except_align_r      <= 1'b0;
      except_dtlb_miss_r  <= 1'b0;
      except_dpagefault_r <= 1'b0;
      // any of them
      excepts_any_r       <= 1'b0;
      // exception virtual address
      lsu_except_addr_r   <= {LSUOOW{1'b0}};
    end
    else if ((padv_wb_i & grant_wb_to_lsu_i) | flush_by_ctrl) begin  // drop local exception flags
      // exception flags
      except_dbus_err_r   <= 1'b0;
      except_align_r      <= 1'b0;
      except_dtlb_miss_r  <= 1'b0;
      except_dpagefault_r <= 1'b0;
      // any of them
      excepts_any_r       <= 1'b0;
      // exception virtual address
      lsu_except_addr_r   <= lsu_except_addr_r;
    end
    else begin
      // exception flags
      if (dbus_err_instant)
        except_dbus_err_r   <= 1'b1;
      if (except_align)
        except_align_r      <= 1'b1;
      if (except_dtlb_miss)
        except_dtlb_miss_r  <= 1'b1;
      if (except_dpagefault)
        except_dpagefault_r <= 1'b1;
      // exception virtual address and any exception
      if (excepts_any & ~excepts_any_r) begin // latch exception virtual address
        excepts_any_r       <= 1'b1;
        lsu_except_addr_r   <= virt_addr_cmd;
      end
    end
  end // @clock

  // WB latches for LSU EXCEPTIONS
  always @(posedge clk `OR_ASYNC_RST) begin
    if (rst) begin
      //  # particular LSU exception flags
      wb_except_dbus_err_o   <= 1'b0;
      wb_except_dpagefault_o <= 1'b0;
      wb_except_dtlb_miss_o  <= 1'b0;
      wb_except_dbus_align_o <= 1'b0;
      wb_lsu_except_addr_o   <= {LSUOOW{1'b0}};
      //  # combined LSU exceptions flag
      wb_an_except_lsu_o     <= 1'b0;
    end
    else if (flush_by_ctrl) begin  // drop WB-reported exceprions
      //  # particular LSU exception flags
      wb_except_dbus_err_o   <= 1'b0;
      wb_except_dpagefault_o <= 1'b0;
      wb_except_dtlb_miss_o  <= 1'b0;
      wb_except_dbus_align_o <= 1'b0;
      wb_lsu_except_addr_o   <= {LSUOOW{1'b0}};
      //  # combined LSU exceptions flag
      wb_an_except_lsu_o     <= 1'b0;
    end
    else if (padv_wb_i & grant_wb_to_lsu_i) begin
      //  # particular LSU exception flags
      wb_except_dbus_err_o   <= except_dbus_err_r   | dbus_err_instant;
      wb_except_dpagefault_o <= except_dpagefault_r | except_dpagefault;
      wb_except_dtlb_miss_o  <= except_dtlb_miss_r  | except_dtlb_miss;
      wb_except_dbus_align_o <= except_align_r      | except_align;
      wb_lsu_except_addr_o   <= excepts_any_r ? lsu_except_addr_r : virt_addr_cmd;
      //  # combined LSU exceptions flag
      wb_an_except_lsu_o     <= excepts_any | excepts_any_r;
    end
  end // @clock



  //-----------------//
  // LSU load output //
  //-----------------//

  assign lsu_ldat = dc_access ? dc_dat : dbus_dat_i;

  // Select part of bus for load
  reg [LSUOOW-1:0] dbus_dat_aligned;
  always @(*) begin
    case(virt_addr_cmd[1:0])
      2'b00: dbus_dat_aligned = lsu_ldat;
      2'b01: dbus_dat_aligned = {lsu_ldat[23:0],8'd0};
      2'b10: dbus_dat_aligned = {lsu_ldat[15:0],16'd0};
      2'b11: dbus_dat_aligned = {lsu_ldat[7:0],24'd0};
    endcase
  end

  // Do appropriate extension for load
  reg [LSUOOW-1:0] dbus_dat_extended;
  always @(*) begin
    case({cmd_zext, cmd_length})
      3'b100:  dbus_dat_extended = {24'd0,dbus_dat_aligned[31:24]}; // lbz
      3'b101:  dbus_dat_extended = {16'd0,dbus_dat_aligned[31:16]}; // lhz
      3'b000:  dbus_dat_extended = {{24{dbus_dat_aligned[31]}},
                                    dbus_dat_aligned[31:24]}; // lbs
      3'b001:  dbus_dat_extended = {{16{dbus_dat_aligned[31]}},
                                    dbus_dat_aligned[31:16]}; // lhs
      default: dbus_dat_extended = dbus_dat_aligned;
    endcase
  end

  //------------------------------//
  // LSU temporary output storage //
  //------------------------------//

  reg [LSUOOW-1:0] lsu_result_r;

  // pending 'load ready' flag for WB_MUX
  always @(posedge clk `OR_ASYNC_RST) begin
    if (rst) begin
      lsu_ack_load_pending  <= 1'b0;
      lsu_ack_store_pending <= 1'b0;
      lsu_result_r          <= {LSUOOW{1'b0}};
    end
    else if ((padv_wb_i & grant_wb_to_lsu_i) |        // prevent LSU's pending ACKs
             excepts_any | flush_by_ctrl) begin   // prevent LSU's pending ACKs
      lsu_ack_load_pending  <= 1'b0;
      lsu_ack_store_pending <= 1'b0;
      lsu_result_r          <= lsu_result_r;
    end
    else begin
      // pending 'load ack'
      if (~lsu_ack_load_pending)
        lsu_ack_load_pending  <= lsu_ack_load;
      // pending 'store ack'
      if (~lsu_ack_store_pending)
        lsu_ack_store_pending <= lsu_ack_store | lsu_ack_swa;
      // pending result
      if (lsu_ack_load)
        lsu_result_r <= dbus_dat_extended;
    end
  end // @clock

  //------------------------//
  // LSU write-back latches //
  //------------------------//

  always @(posedge clk `OR_ASYNC_RST) begin
    if (rst)
      wb_lsu_result_o <= {LSUOOW{1'b0}};
    else if (padv_wb_i) begin
      if (grant_wb_to_lsu_i) begin
        if (excepts_any | excepts_any_r)
          wb_lsu_result_o <= {LSUOOW{1'b0}};
        else
          wb_lsu_result_o <= lsu_ack_load         ? dbus_dat_extended :
                             lsu_ack_load_pending ? lsu_result_r      :
                                                    {LSUOOW{1'b0}};
      end
      else
        wb_lsu_result_o <= {LSUOOW{1'b0}};
    end
  end // @clock


  //----------------------------//
  // LSU formatted store output //
  //----------------------------//

  // Big endian bus mapping
  reg [3:0] dbus_bsel;
  always @(*) begin
    case (cmd_length)
      2'b00: // byte access
        case(virt_addr_cmd[1:0])
          2'b00: dbus_bsel = 4'b1000;
          2'b01: dbus_bsel = 4'b0100;
          2'b10: dbus_bsel = 4'b0010;
          2'b11: dbus_bsel = 4'b0001;
        endcase
      2'b01: // halfword access
        case(virt_addr_cmd[1])
          1'b0: dbus_bsel = 4'b1100;
          1'b1: dbus_bsel = 4'b0011;
        endcase
      2'b10,
      2'b11: dbus_bsel = 4'b1111;
    endcase
  end

  // Data bus mapping for store
  assign lsu_sdat =
    (cmd_length == 2'b00) ? {cmd_rfb[7:0],cmd_rfb[7:0],cmd_rfb[7:0],cmd_rfb[7:0]} : // byte access
    (cmd_length == 2'b01) ? {cmd_rfb[15:0],cmd_rfb[15:0]} : // halfword access
                            cmd_rfb; // word access



  //----------//
  // DBUS FSM //
  //----------//

  assign dbus_burst_o = (dbus_state == DBUS_DC_REFILL) & ~dc_refill_last;

  // Slightly subtle, but if there is an atomic store coming out from the
  // store buffer, and the link has been broken while it was waiting there,
  // the bus access is still performed as a (discarded) read.
  assign dbus_we_o = dbus_we & ~dbus_swa_discard;


  // re-filll-allowed corresponds to refill-request position in DBUS FSM
  // !!! exceptions and flushing are already taken into accaunt in DCACHE,
  //     so we don't use them here
  always @(*) begin
    dc_refill_allowed = 1'b0;
    case (dbus_state)
      DMEM_REQ: begin
        if (sbuf_odata | cmd_store_r | cmd_swa_r) // re-fill not allowed
          dc_refill_allowed = 1'b0;
        else if (dc_refill_req) // it automatically means (l.load & dc-access)
          dc_refill_allowed = 1'b1;
      end
      default:;
    endcase
  end // always

  // state machine
  always @(posedge clk `OR_ASYNC_RST) begin
    if (rst) begin
      // DBUS controls
      dbus_req_o  <= 1'b0;            // DBUS reset
      dbus_we     <= 1'b0;            // DBUS reset
      dbus_bsel_o <= 4'hf;            // DBUS reset
      dbus_adr_o  <= {LSUOOW{1'b0}};  // DBUS reset
      dbus_dat_o  <= {LSUOOW{1'b0}};  // DBUS reset
      dbus_atomic <= 1'b0;            // DBUS reset
      sbuf_odata  <= 1'b0;            // DBUS reset
      // DBUS FSM state
      dbus_state  <= DBUS_IDLE;       // DBUS reset
    end
    else if (dbus_err_instant) begin // in DBUS FSM: highest priority
      // DBUS controls
      dbus_req_o  <= 1'b0;            // DBUS error
      dbus_we     <= 1'b0;            // DBUS error
      dbus_bsel_o <= 4'hf;            // DBUS error
      dbus_adr_o  <= {LSUOOW{1'b0}};  // DBUS error
      dbus_dat_o  <= {LSUOOW{1'b0}};  // DBUS error
      dbus_atomic <= 1'b0;            // DBUS error
      sbuf_odata  <= 1'b0;            // DBUS error; MAROCCHINO_TODO: force buffer empty by DBUS error ?
      // DBUS FSM state
      dbus_state  <= DBUS_IDLE;       // DBUS error
    end
    else begin
      // process
      case (dbus_state)
        DBUS_IDLE: begin
          if (excepts_any_r | spr_bus_stb_i| pipeline_flush_i) // DBUS FSM keep idling
            dbus_state <= DBUS_IDLE;
          else if (lsu_takes_ls | sbuf_odata) // idle -> dmem req
            dbus_state <= DMEM_REQ;
        end

        DMEM_REQ: begin
          if (lsu_excepts_addr | pipeline_flush_i) begin // dmem req
            dbus_state  <= DBUS_IDLE; // dmem req -> exceptions or pipe flush
          end
          else if (sbuf_odata | cmd_store_r | cmd_swa_r) begin
            if (~snoop_hit) begin
              // DBUS controls
              //  # no buffered data for write == store buffer is empty
              dbus_req_o  <= 1'b1;
              dbus_we     <= 1'b1;
              dbus_bsel_o <= sbuf_odata ? sbuf_bsel      : dbus_bsel;
              dbus_adr_o  <= sbuf_odata ? sbuf_phys_addr : phys_addr_cmd;
              dbus_dat_o  <= sbuf_odata ? sbuf_dat       : lsu_sdat;
              dbus_atomic <= sbuf_odata ? 1'b0           : cmd_swa_r; // we execute store conditional around store buffer
              sbuf_odata  <= 1'b0; // DBUS: dmem-req - > write
              // DBUS FSM state
              dbus_state  <= DBUS_WRITE;
            end
          end
          else if (dc_refill_req) begin // it automatically means (l.load & dc-access)
            dbus_req_o  <= 1'b1;
            dbus_adr_o  <= phys_addr_cmd;
            dbus_state  <= DBUS_DC_REFILL;
          end
          else if (cmd_load_r & ~dc_access) begin
            dbus_req_o  <= 1'b1;
            dbus_adr_o  <= phys_addr_cmd;
            dbus_bsel_o <= dbus_bsel;
            dbus_state  <= DBUS_READ;
          end
          else if (~lsu_takes_ls) begin // no new memory request
            dbus_state <= DBUS_IDLE;    // no new memory request
          end
        end // idle

        DBUS_DC_REFILL: begin
          if (snoop_hit) begin
            dbus_req_o  <= 1'b0;                                  // DC-REFILL: snoop-hit
            dbus_adr_o  <= {LSUOOW{1'b0}};                        // DC-REFILL: snoop-hit
            dbus_state  <= flush_by_ctrl ? DBUS_IDLE : DMEM_REQ;  // DC-REFILL: snoop-hit
          end
          else if (dbus_ack_i) begin
            dbus_adr_o <= next_refill_adr;    // DC-REFILL: DBUS-ack
            if (dc_refill_last) begin
              dbus_req_o  <= 1'b0;            // DC-REFILL: DBUS-last-ack
              dbus_adr_o  <= {LSUOOW{1'b0}};  // DC-REFILL: DBUS-last-ack
              dbus_state  <= DBUS_IDLE;       // DC-REFILL: DBUS-last-ack
            end
          end
        end // dc-refill

        DBUS_READ: begin
          if (dbus_ack_i) begin
            dbus_req_o  <= 1'b0;                                 // DBUS: read complete
            dbus_bsel_o <= 4'hf;                                 // DBUS: read complete
            dbus_adr_o  <= {LSUOOW{1'b0}};                       // DBUS: read complete
            dbus_state  <= flush_by_ctrl ? DBUS_IDLE : DMEM_REQ; // DBUS: read complete
          end
        end // read

        DBUS_WRITE: begin
           if (dbus_ack_i) begin
            // DBUS controls
            dbus_req_o  <= 1'b0;           // DBUS: write complete
            dbus_we     <= 1'b0;           // DBUS: write complete
            dbus_bsel_o <= 4'hf;           // DBUS: write complete
            dbus_adr_o  <= {LSUOOW{1'b0}}; // DBUS: write complete
            dbus_dat_o  <= {LSUOOW{1'b0}}; // DBUS: write complete
            dbus_atomic <= 1'b0;
            // pending data for write (see also sbuf_re)
            if ((~sbuf_empty) | (sbuf_rdwr_empty & ~(lsu_excepts_addr | pipeline_flush_i))) // DBUS: write complete
              sbuf_odata  <= 1'b1;  // DBUS: write complete
            // DBUS FSM next state
            if (lsu_excepts_addr | flush_by_ctrl)
              dbus_state <= DBUS_IDLE; // DBUS: write complete, exceptions or flushing
            else
              dbus_state <= DMEM_REQ;  // DBUS: write complete, no exceptions, no flushing
          end
        end // write-state

        default: begin
          // DBUS controls
          dbus_req_o  <= 1'b0;            // DBUS deault
          dbus_we     <= 1'b0;            // DBUS deault
          dbus_bsel_o <= 4'hf;            // DBUS deault
          dbus_adr_o  <= {LSUOOW{1'b0}};  // DBUS deault
          dbus_dat_o  <= {LSUOOW{1'b0}};  // DBUS deault
          dbus_atomic <= 1'b0;            // DBUS deault
          sbuf_odata  <= 1'b0;            // DBUS deault
          // DBUS FSM state
          dbus_state  <= DBUS_IDLE;       // DBUS deault
        end
      endcase
    end
  end // @ clock state machine



  //-----------------------//
  // Store buffer instance //
  //-----------------------//

  // store buffer write controls
  assign sbuf_write = cmd_store_r & ~sbuf_full & ~snoop_hit;  // SBUFF write
  // include exceptions and pipe flushing
  assign sbuf_we = sbuf_write & ~(excepts_any | pipeline_flush_i);


  //
  // store buffer read controls
  //
  // auxiliary: read and write simultaneously if buffer is empty
  assign sbuf_rdwr_empty = cmd_store_r & sbuf_empty & ~snoop_hit;
  //
  //  # in DMEM-REQ state we perform reading only if buffer is empty and no pending data on buffer's output
  //    as a result the buffer keep empty status. At the same time we use data from command latches
  //    (not from buffer's output) for initialize DBUS transaction for the case while store-buffer-write
  //    is used as ACK to report that store is executed
  //  # in WRITE at the end of transactions read next value:
  //     - from empty buffer (keep empty flag, but set sbuf-odata flag, see DBUS state machine)
  //     - from non-empty buffer regardless of exceptions or flushing
  //
  // MAROCCHINO_TODO: force buffer empty by DBUS error ?
  //
  //               in DMEM-REQ state we take into accaunt appropriate exceptions
  assign sbuf_re = ((dbus_state == DMEM_REQ) & sbuf_rdwr_empty & ~sbuf_odata & ~(lsu_excepts_addr | pipeline_flush_i)) | // SBUFF read, dmem-req
  //               in WRITE state if write request overlaps DBUS ACK and empty buffer state
                   ((dbus_state == DBUS_WRITE) & dbus_ack_i & sbuf_rdwr_empty & ~(lsu_excepts_addr | pipeline_flush_i)) | // SBUFF read, dbus-write
  //               in WRITE state we read non-empty buffer to prepare next write
                   ((dbus_state == DBUS_WRITE) & dbus_ack_i & ~sbuf_empty); // SBUFF read, dbus-write


  // store buffer module
  mor1kx_store_buffer_marocchino
  #(
    .DEPTH_WIDTH          (OPTION_STORE_BUFFER_DEPTH_WIDTH),
    .OPTION_OPERAND_WIDTH (OPTION_OPERAND_WIDTH),
    .CLEAR_ON_INIT        (OPTION_STORE_BUFFER_CLEAR_ON_INIT)
  )
  u_store_buffer
  (
    .clk          (clk),
    .rst          (rst),
    // entry port
    .sbuf_epcr_i  (cmd_epcr), // STORE_BUFFER
    .virt_addr_i  (virt_addr_cmd), // STORE_BUFFER
    .phys_addr_i  (phys_addr_cmd), // STORE_BUFFER
    .dat_i        (lsu_sdat), // STORE_BUFFER
    .bsel_i       (dbus_bsel), // STORE_BUFFER
    .write_i      (sbuf_we), // STORE_BUFFER
    // output port
    .sbuf_epcr_o  (sbuf_epcr), // STORE_BUFFER
    .virt_addr_o  (sbuf_virt_addr), // STORE_BUFFER
    .phys_addr_o  (sbuf_phys_addr), // STORE_BUFFER
    .dat_o        (sbuf_dat), // STORE_BUFFER
    .bsel_o       (sbuf_bsel), // STORE_BUFFER
    .read_i       (sbuf_re), // STORE_BUFFER
    // status flags
    .full_o       (sbuf_full), // STORE_BUFFER
    .empty_o      (sbuf_empty) // STORE_BUFFER
  );


  // We have to mask out our own snooped bus access
  assign snoop_event = (snoop_en_i & ~((snoop_adr_i == dbus_adr_o) & dbus_ack_i)) &
                       (OPTION_DCACHE_SNOOP != "NONE");


  //-------------------------//
  // Atomic operations logic //
  //-------------------------//
  assign dbus_swa_discard = dbus_atomic & ~atomic_reserve;
  // ---
  assign dbus_swa_ack     = dbus_atomic & dbus_ack_i;
  // ---
  assign dbus_swa_success = dbus_swa_ack &  atomic_reserve; // for WB & DCACHE
  wire   dbus_swa_fail    = dbus_swa_ack & ~atomic_reserve; // for WB

  always @(posedge clk `OR_ASYNC_RST) begin
    if (rst) begin
      atomic_reserve <= 1'b0;
      atomic_addr    <= {LSUOOW{1'b0}};
    end
    else if (excepts_any | pipeline_flush_i |                 // drop atomic reserve
             dbus_swa_ack |                                       // drop atomic reserve
             (cmd_store_r & (phys_addr_cmd == atomic_addr)) |     // drop atomic reserve
             (snoop_event & (snoop_adr_i == atomic_addr))) begin  // drop atomic reserve
      atomic_reserve <= 1'b0;
      atomic_addr    <= {LSUOOW{1'b0}};
    end
    else if (cmd_lwa_r) begin
      if (snoop_event & (snoop_adr_i == phys_addr_cmd)) begin
        atomic_reserve <= 1'b0;
        atomic_addr    <= {LSUOOW{1'b0}};
      end
      else if (lsu_ack_load) begin
        atomic_reserve <= 1'b1;
        atomic_addr    <= phys_addr_cmd;
      end
    end
  end // @clock

  reg atomic_flag_set;
  reg atomic_flag_clear;

  always @(posedge clk `OR_ASYNC_RST) begin
    if (rst) begin
      atomic_flag_set   <= 1'b0;
      atomic_flag_clear <= 1'b0;
    end
    else if ((padv_wb_i & grant_wb_to_lsu_i) |      // prevent set/clear atomic flag pending
             excepts_any | flush_by_ctrl) begin // prevent set/clear atomic flag pending
      atomic_flag_set   <= 1'b0;
      atomic_flag_clear <= 1'b0;
    end
    else if (dbus_swa_ack) begin
      atomic_flag_set   <=  atomic_reserve;
      atomic_flag_clear <= ~atomic_reserve;
    end
  end // @clock

  // atomic flags for WB_MUX
  always @(posedge clk `OR_ASYNC_RST) begin
    if (rst) begin
      wb_atomic_flag_set_o   <= 1'b0;
      wb_atomic_flag_clear_o <= 1'b0;
    end
    else if (flush_by_ctrl) begin // drop WB atomic flags
      wb_atomic_flag_set_o   <= 1'b0;
      wb_atomic_flag_clear_o <= 1'b0;
    end
    else if (padv_wb_i) begin
      if (grant_wb_to_lsu_i & ~(excepts_any | excepts_any_r)) begin // conditions for WB atomic flags
        wb_atomic_flag_set_o   <= dbus_swa_success | atomic_flag_set;
        wb_atomic_flag_clear_o <= dbus_swa_fail    | atomic_flag_clear;
      end
      else begin
        wb_atomic_flag_set_o   <= 1'b0;
        wb_atomic_flag_clear_o <= 1'b0;
      end
    end
  end // @clock



  //--------------------//
  // SPR access support //
  //--------------------//
  //   For MAROCCHINO SPR access means that pipeline is stalled till ACK.
  // So, no padv-*. We only delay SPR access command till DBUS transaction
  // completion.
  wire spr_bus_lsu_stb = spr_bus_stb_i & (dbus_state == DBUS_IDLE); // SPR access



  //-------------------//
  // Instance of cache //
  //-------------------//

  wire dc_store_allowed = lsu_ack_store | dbus_swa_success;

  mor1kx_dcache_marocchino
  #(
    .OPTION_OPERAND_WIDTH         (OPTION_OPERAND_WIDTH),
    .OPTION_DCACHE_BLOCK_WIDTH    (OPTION_DCACHE_BLOCK_WIDTH),
    .OPTION_DCACHE_SET_WIDTH      (OPTION_DCACHE_SET_WIDTH),
    .OPTION_DCACHE_WAYS           (OPTION_DCACHE_WAYS),
    .OPTION_DCACHE_LIMIT_WIDTH    (OPTION_DCACHE_LIMIT_WIDTH),
    .OPTION_DCACHE_SNOOP          (OPTION_DCACHE_SNOOP),
    .OPTION_DCACHE_CLEAR_ON_INIT  (OPTION_DCACHE_CLEAR_ON_INIT)
  )
  u_dcache
  (
    // clock & reset
    .clk                        (clk), // DCACHE
    .rst                        (rst), // DCACHE
    // pipe controls
    .lsu_takes_load_i           (lsu_takes_load), // DCACHE
    .lsu_takes_store_i          (lsu_takes_store), // DCACHE
    .pipeline_flush_i           (pipeline_flush_i), // DCACHE
    // configuration
    .enable_i                   (dc_enable_i), // DCACHE
    // exceptions
    .lsu_excepts_any_i          (excepts_any), // DCACHE
    .dbus_err_instant_i         (dbus_err_instant), // DCACHE
    // Regular operation
    //  # addresses and "DCHACHE inhibit" flag
    .virt_addr_i                (virt_addr), // DCACHE
    .virt_addr_cmd_i            (virt_addr_cmd), // DCACHE
    .phys_addr_cmd_i            (phys_addr_cmd), // DCACHE
    .dmmu_cache_inhibit_i       (dmmu_cache_inhibit), // DCACHE
    //  # DCACHE regular answer
    .dc_access_o                (dc_access), // DCACHE
    .dc_ack_o                   (dc_ack), // DCACHE
    .dc_dat_o                   (dc_dat), // DCACHE
    //  # STORE format / store data / do|cancel storing
    .dbus_bsel_i                (dbus_bsel), // DCACHE
    .dbus_sdat_i                (lsu_sdat), // DCACHE
    .dc_store_allowed_i         (dc_store_allowed), // DCACHE
    .dbus_swa_discard_i         (dbus_swa_discard), // DCACHE
    // re-fill
    .dc_refill_req_o            (dc_refill_req), // DCACHE
    .dc_refill_allowed_i        (dc_refill_allowed), // DCACHE
    .next_refill_adr_o          (next_refill_adr), // DCACHE
    .refill_last_o              (dc_refill_last), // DCACHE
    .dbus_dat_i                 (dbus_dat_i), // DCACHE
    .dbus_ack_i                 (dbus_ack_i), // DCACHE
    // SNOOP
    .snoop_adr_i                (snoop_adr_i[31:0]), // DCACHE
    .snoop_event_i              (snoop_event), // DCACHE
    .snoop_hit_o                (snoop_hit), // DCACHE
    // SPR interface
    .spr_bus_addr_i             (spr_bus_addr_i[15:0]), // DCACHE
    .spr_bus_we_i               (spr_bus_we_i), // DCACHE
    .spr_bus_stb_i              (spr_bus_lsu_stb), // DCACHE
    .spr_bus_dat_i              (spr_bus_dat_i), // DCACHE
    .spr_bus_dat_o              (spr_bus_dat_dc_o), // DCACHE
    .spr_bus_ack_o              (spr_bus_ack_dc_o) // DCACHE
  );



  //------------------//
  // Instance of DMMU //
  //------------------//

  mor1kx_dmmu_marocchino
  #(
    .FEATURE_DMMU_HW_TLB_RELOAD (FEATURE_DMMU_HW_TLB_RELOAD),
    .OPTION_OPERAND_WIDTH       (OPTION_OPERAND_WIDTH),
    .OPTION_DMMU_SET_WIDTH      (OPTION_DMMU_SET_WIDTH),
    .OPTION_DMMU_WAYS           (OPTION_DMMU_WAYS),
    .OPTION_DMMU_CLEAR_ON_INIT  (OPTION_DMMU_CLEAR_ON_INIT)
  )
  u_dmmu
  (
    // clocks and resets
    .clk                              (clk), // DMMU
    .rst                              (rst), // DMMU
    // pipe controls
    .lsu_takes_ls_i                   (lsu_takes_ls), // DMMU
    .pipeline_flush_i                 (pipeline_flush_i), // DMMU
    // exceptions
    .lsu_excepts_any_i                (excepts_any), // DMMU
    // configuration and commands
    .enable_i                         (dmmu_enable_i), // DMMU
    .supervisor_mode_i                (supervisor_mode_i), // DMMU
    .lsu_store_i                      (lsu_store_w), // DMMU
    .lsu_load_i                       (lsu_load_w), // DMMU
    // address translation
    .virt_addr_i                      (virt_addr), // DMMU
    .virt_addr_cmd_i                  (virt_addr_cmd), // DMMU
    .phys_addr_cmd_o                  (phys_addr_cmd), // DMMU
    // translation flags
    .cache_inhibit_o                  (dmmu_cache_inhibit), // DMMU
    .tlb_miss_o                       (except_dtlb_miss), // DMMU
    .pagefault_o                      (except_dpagefault), // DMMU
    // HW TLB reload face.  MAROCCHINO_TODO: not implemented
    .tlb_reload_ack_i                 (1'b0), // DMMU
    .tlb_reload_data_i                ({LSUOOW{1'b0}}), // DMMU
    .tlb_reload_pagefault_clear_i     (1'b0), // DMMU
    .tlb_reload_req_o                 (), // DMMU
    .tlb_reload_busy_o                (), // DMMU
    .tlb_reload_addr_o                (), // DMMU
    .tlb_reload_pagefault_o           (), // DMMU
    // SPR bus
    .spr_bus_addr_i                   (spr_bus_addr_i[15:0]), // DMMU
    .spr_bus_we_i                     (spr_bus_we_i), // DMMU
    .spr_bus_stb_i                    (spr_bus_lsu_stb), // DMMU
    .spr_bus_dat_i                    (spr_bus_dat_i), // DMMU
    .spr_bus_dat_o                    (spr_bus_dat_dmmu_o), // DMMU
    .spr_bus_ack_o                    (spr_bus_ack_dmmu_o) // DMMU
  );

endmodule // mor1kx_lsu_marocchino
