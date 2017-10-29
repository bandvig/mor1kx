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
//   Copyright (C) 2015-2017 Andrey Bacherov                          //
//                           avbacherov@opencores.org                 //
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
  input                                 cpu_clk,
  input                                 cpu_rst,
  // Pipeline controls
  input                                 pipeline_flush_i,
  input                                 padv_wb_i,
  input                                 grant_wb_to_lsu_i,
  // configuration
  input                                 dc_enable_i,
  input                                 dmmu_enable_i,
  input                                 supervisor_mode_i,
  // Input from RSRVS
  input                                 exec_op_lsu_any_i,
  input                                 exec_op_lsu_load_i,
  input                                 exec_op_lsu_store_i,
  input                                 exec_op_msync_i,
  input                                 exec_op_lsu_atomic_i,
  input                           [1:0] exec_lsu_length_i,
  input                                 exec_lsu_zext_i,
  input           [`OR1K_IMM_WIDTH-1:0] exec_lsu_imm16_i, // immediate offset for address computation
  input      [OPTION_OPERAND_WIDTH-1:0] exec_sbuf_epcr_i,       // for store buffer EPCR computation
  input      [OPTION_OPERAND_WIDTH-1:0] exec_lsu_a1_i,   // operand "A" (part of address)
  input      [OPTION_OPERAND_WIDTH-1:0] exec_lsu_b1_i,   // operand "B" (value to store)
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
  input                                 dbus_burst_last_i,
  // Cache sync for multi-core environment
  input                          [31:0] snoop_adr_i,
  input                                 snoop_en_i,
  // Pipe control output flags
  output                                lsu_taking_op_o,
  output                                lsu_valid_o, // result ready or exceptions
  // Imprecise exception (with appropriate PC) came via the store buffer
  output reg [OPTION_OPERAND_WIDTH-1:0] sbuf_eear_o,
  output reg [OPTION_OPERAND_WIDTH-1:0] sbuf_epcr_o,
  output reg                            sbuf_err_o,
  //  Pre-WriteBack "an exception" flag
  output                                exec_an_except_lsu_o,
  // WriteBack load  result
  output     [OPTION_OPERAND_WIDTH-1:0] wb_lsu_result_o,
  output reg [OPTION_OPERAND_WIDTH-1:0] wb_lsu_result_cp1_o,
  output reg [OPTION_OPERAND_WIDTH-1:0] wb_lsu_result_cp2_o,
  output reg [OPTION_OPERAND_WIDTH-1:0] wb_lsu_result_cp3_o,
  // Atomic operation flag set/clear logic
  output                                wb_atomic_flag_set_o,
  output                                wb_atomic_flag_clear_o,
  // Exceptions & errors
  output                                wb_except_dbus_err_o,
  output                                wb_except_dpagefault_o,
  output                                wb_except_dtlb_miss_o,
  output                                wb_except_dbus_align_o,
  output     [OPTION_OPERAND_WIDTH-1:0] wb_lsu_except_addr_o
);


  // short names for local use
  localparam LSUOOW = OPTION_OPERAND_WIDTH;



  // input registers for DMMU stage
  reg               s1o_op_lsu_load;
  reg               s1o_op_lsu_store;   // any kind: atomic or regular
  reg               s1o_op_lsu_atomic;
  reg               s1o_op_msync;
  reg               s1o_op_lsu_ls;
  reg  [LSUOOW-1:0] s1o_virt_addr;
  reg  [LSUOOW-1:0] s1o_lsu_b1;
  reg  [LSUOOW-1:0] s1o_sbuf_epcr;       // for store buffer EPCR computation
  reg         [1:0] s1o_lsu_length;
  reg               s1o_lsu_zext;
  /* HW reload TLB related (MAROCCHINO_TODO : not implemented yet)
  wire              tlb_reload_req;
  wire              tlb_reload_busy;
  wire [LSUOOW-1:0] tlb_reload_addr;
  wire              tlb_reload_pagefault;
  reg               tlb_reload_ack;
  reg  [LSUOOW-1:0] tlb_reload_data;
  wire              tlb_reload_pagefault_clear;
  reg               tlb_reload_done; */


  // Register na wires for stage #2: DBUS access / DCACHE check

  //  # any kind of command has been taken
  reg               s2o_op_lsu_store;

  //  # load command
  reg               s2o_lwa;   // exactly atomic load
  reg         [1:0] s2o_length;
  reg               s2o_zext;
  //  # load processing
  //    ## cachable area
  wire              s2t_dc_ack_read;
  reg               s2o_dc_ack_read;
  wire [LSUOOW-1:0] s2t_dc_dat;
  reg  [LSUOOW-1:0] s2o_dc_dat;
  wire              s2t_dc_refill_req;
  reg               s2o_dc_refill_req;
  wire              dc_refill_allowed;
  wire              dc_refill_first;
  //    ## none cachable area
  wire              s2t_dbus_read_req;
  reg               s2o_dbus_read_req;
  wire              dbus_load_ack;
  wire              dbus_load_err;
  reg  [LSUOOW-1:0] s2o_dbus_dat;
  //    ## combined load ACK :MAROCCHINO_TODO: makes sense for l.lwa only now
  wire              s3t_load_ack;

  //  # store
  reg               s2o_store; // not (!!!) atomic
  reg               s2o_swa;   // atomic only
  //  # auxiliaries
  //  # DBUS "bsel" and formatted data to store
  reg         [3:0] s2o_bsel;
  reg  [LSUOOW-1:0] s2o_sdat;     // register file B in (store operand)
  wire              dbus_swa_ack; // complete DBUS trunsaction with l.swa
  wire              dbus_swa_err; // DBUS error during l.swa
  //    ## combined store ACK
  wire              s3t_store_ack;

  //  # combined DBUS-load/SBUFF-store ACK
  reg               s3o_ls_ack;

  //    ## registers for store buffer EPCR computation
  reg  [LSUOOW-1:0] s2o_epcr;

  //  # latched virtual and physical addresses
  reg  [LSUOOW-1:0] s2o_virt_addr;
  reg  [LSUOOW-1:0] s2o_phys_addr;

  wire              lsu_s2_rdy;   // operation complete or an exception
  reg               lsu_wb_miss;  // pending registers are busy


  // DBUS FSM
  //  # DBUS FSM states
  localparam [8:0] DBUS_IDLE      = 9'b000000001, /*  0 */
                   DMEM_REQ       = 9'b000000010, /*  1 */
                   DBUS_READ      = 9'b000000100, /*  2 */
                   DBUS_TO_REFILL = 9'b000001000, /*  3 */
                   DBUS_DC_REFILL = 9'b000010000, /*  4 */
                   DBUS_DC_REREAD = 9'b000100000, /*  5 */
                   DBUS_SBUF_READ = 9'b001000000, /*  6 */
                   DBUS_INI_WRITE = 9'b010000000, /*  7 */
                   DBUS_WRITE     = 9'b100000000; /*  8 */
  //  # DBUS FSM state indicator
  reg        [8:0] dbus_state;
  //  # particular states
  wire             dbus_idle_state    = dbus_state[0];
  wire             dmem_req_state     = dbus_state[1];
  wire             dbus_read_state    = dbus_state[2];
  assign           dc_refill_allowed  = dbus_state[3];
  wire             dc_refill_state    = dbus_state[4];
  wire             dc_reread_state    = dbus_state[5];
  wire             sbuf_read_state    = dbus_state[6];
//wire             sbuf_init_write    = dbus_state[7];
  wire             dbus_write_state   = dbus_state[8];
  //  # DBUS FSM other registers & wires
  reg              dbus_we;
  reg              dbus_atomic;
  wire             dbus_swa_discard; // reservation is lost, execute empty read



  // Store buffer
  wire              sbuf_write; // without exceptions and pipeline-flush
  wire              sbuf_we;    // with exceptions and pipeline-flush
  // ---
  wire              sbuf_full;
  wire              sbuf_empty;
  wire [LSUOOW-1:0] sbuf_epcr;
  wire [LSUOOW-1:0] sbuf_virt_addr;
  wire [LSUOOW-1:0] sbuf_phys_addr;
  wire [LSUOOW-1:0] sbuf_dat;
  wire        [3:0] sbuf_bsel;

  // Atomic operations
  reg               atomic_reserve;
  reg  [LSUOOW-1:0] atomic_addr;
  // initial values for simulations
 `ifndef SYNTHESIS
  // synthesis translate_off
  initial begin
    atomic_addr = {LSUOOW{1'b0}};
  end
  // synthesis translate_on
 `endif // !syhth

  // Snoop (for multicore SoC)
  wire              snoop_event;
  wire              snoop_hit;


  // Exceptions detected on DCACHE/DBUS access stage
  //  # latched address convertion exceptions
  reg               s2o_tlb_miss;
  reg               s2o_pagefault;
  reg               s2o_align;
  // # combination of align/tlb/pagefault
  wire              s2t_excepts_addr;
  reg               s2o_excepts_addr;
  // # DBUS error not related to STORE_BUFFER
  wire              s3t_dbus_err_nsbuf;
  reg               s2o_dbus_err_nsbuf;
  // # combined of address + DBUS error not related to STORE_BUFFER
  reg               s2o_excepts_any;

  // Flushing logic provides continuous clean up output
  // from pipeline-flush till transaction (read/re-fill) completion
  reg               flush_r;
  wire              flush_by_ctrl;


  //*******************//
  // LSU pipe controls //
  //*******************//


  // LSU pipe controls
  //  ## per stage busy signals
  wire   lsu_s2_busy = s2o_dc_refill_req | s2o_dbus_read_req | // stage #2 is busy
                       s2o_op_lsu_store                      | // stage #2 is busy
                       (lsu_s2_rdy & lsu_wb_miss)            | // stage #2 is busy
                       s2o_excepts_any;                        // stage #2 is busy, MAROCCHINO_TODO: "& (~snoop_hit)" -??
  //  ---
  wire   lsu_s1_busy = s2o_dc_refill_req |                            // stage #1 is busy
                       (s2o_dbus_read_req        & s1o_op_lsu_load) | // stage #1 is busy
                       (lsu_s2_rdy & lsu_wb_miss & s1o_op_lsu_load) | // stage #1 is busy
                       s1o_op_lsu_store  | s1o_op_msync             | // stage #1 is busy
                       flush_r;                                       // stage #1 is busy
  //  ## per stage advance signals
  wire   lsu_s1_adv  = exec_op_lsu_any_i & (~lsu_s1_busy);
  wire   lsu_s2_adv  = s1o_op_lsu_ls     & (~lsu_s2_busy);
  wire   lsu_s3_adv  = lsu_s2_rdy        & (~lsu_wb_miss);
  //  ## to LSU_RSRVS
  assign lsu_taking_op_o = lsu_s1_adv;


  // DBUS read completion
  assign dbus_load_ack = (dbus_read_state | dc_refill_first) & dbus_ack_i;
  assign dbus_load_err = (dbus_read_state | dc_refill_state) & dbus_err_i;

  // Combined DCACHE/DBUS read ACK
  assign s3t_load_ack  = s2o_dc_ack_read | dbus_load_ack; // used for l.lwa processing only

  // Combined store ACK
  assign s3t_store_ack    = sbuf_write |  dbus_swa_ack;
  wire   dc_store_allowed = sbuf_write | (dbus_swa_ack & atomic_reserve);

  // DBUS error not related to STORE_BUFFER
  assign s3t_dbus_err_nsbuf = dbus_load_err | dbus_swa_err;


  //-----------------------------------------------------------------//
  // Flushing from pipeline-flush-i till DBUS transaction completion //
  //-----------------------------------------------------------------//

  always @(posedge cpu_clk) begin
    if (cpu_rst)
      flush_r <= 1'b0; // on reset
    else if (flush_r & dbus_idle_state)
      flush_r <= 1'b0; // on de-assert
    else if (~flush_r)
      flush_r <= pipeline_flush_i;
  end // @clock
  // ---
  assign flush_by_ctrl = pipeline_flush_i | flush_r;


  //--------------------//
  // SPR access support //
  //--------------------//

  //   For MAROCCHINO SPR access means that pipeline is stalled till ACK.
  // So, no padv-*. We only delay SPR access command till DBUS transaction
  // completion.

  wire spr_bus_stb_lsu = spr_bus_stb_i & dbus_idle_state; // SPR access


  //--------------------------//
  // Load/Store input command //
  //--------------------------//
  wire exec_op_lsu_ls = exec_op_lsu_load_i | exec_op_lsu_store_i;


  /*********************************/
  /* Stage #1: address computation */
  /*********************************/


  // compute virtual address
  wire [LSUOOW-1:0] s1t_virt_addr = exec_lsu_a1_i + {{(LSUOOW-16){exec_lsu_imm16_i[15]}},exec_lsu_imm16_i};


  // load and "new command is in stage #1" flag
  always @(posedge cpu_clk) begin
    if (cpu_rst | pipeline_flush_i) begin
      s1o_op_lsu_load   <= 1'b0;
      s1o_op_lsu_atomic <= 1'b0;
      s1o_op_lsu_ls     <= 1'b0;
    end
    else if (lsu_s1_adv) begin // rise "new load command is in stage #1" flag
      s1o_op_lsu_load   <= exec_op_lsu_load_i;
      s1o_op_lsu_atomic <= exec_op_lsu_atomic_i;
      s1o_op_lsu_ls     <= exec_op_lsu_ls;
    end
    else if (lsu_s2_adv) begin   // drop "new load command is in stage #1" flag
      s1o_op_lsu_load   <= 1'b0;
      s1o_op_lsu_atomic <= 1'b0;
      s1o_op_lsu_ls     <= 1'b0;
    end
  end // @cpu-clk

  // store command
  always @(posedge cpu_clk) begin
    if (cpu_rst | pipeline_flush_i)
      s1o_op_lsu_store  <= 1'b0;
    else if (lsu_s1_adv) // rise "new store command is in stage #1" flag
      s1o_op_lsu_store  <= exec_op_lsu_store_i;
    else if (s3t_store_ack) // drop "new store command is in stage #1" flag
      s1o_op_lsu_store  <= 1'b0;
  end // @cpu-clk

  // l.msync is express (and 1-clock length)
  always @(posedge cpu_clk) begin
    if (cpu_rst)                           // ! no flushing for l.msync
      s1o_op_msync <= 1'b0;
    else if (lsu_s1_adv)                   // latch l.msync from LSU-RSRVS
      s1o_op_msync <= exec_op_msync_i;
    else if (dbus_idle_state & sbuf_empty) // de-assert msync
      s1o_op_msync <= 1'b0;                // de-assert msync
  end // @cpu-clk

  // load/store attributes
  always @(posedge cpu_clk) begin
    if (lsu_s1_adv) begin // latch data from LSU-RSRVS without l.msync
      s1o_virt_addr  <= s1t_virt_addr;
      s1o_lsu_b1     <= exec_lsu_b1_i;
      s1o_sbuf_epcr  <= exec_sbuf_epcr_i;
      s1o_lsu_length <= exec_lsu_length_i;
      s1o_lsu_zext   <= exec_lsu_zext_i;
    end
  end // @cpu-clk


  // --- Big endian bus mapping ---
  reg [3:0] s2t_bsel;
  // ---
  always @(s1o_lsu_length or s1o_virt_addr[1:0]) begin
    // synthesis parallel_case full_case
    case (s1o_lsu_length)
      2'b00: // byte access
      begin
        // synthesis parallel_case full_case
        case( s1o_virt_addr[1:0])
          2'b00: s2t_bsel = 4'b1000;
          2'b01: s2t_bsel = 4'b0100;
          2'b10: s2t_bsel = 4'b0010;
          2'b11: s2t_bsel = 4'b0001;
        endcase
      end
      2'b01: // halfword access
      begin
        // synthesis parallel_case full_case
        case(s1o_virt_addr[1])
          1'b0: s2t_bsel = 4'b1100;
          1'b1: s2t_bsel = 4'b0011;
        endcase
      end
      2'b10,
      2'b11: s2t_bsel = 4'b1111;
    endcase
  end

  // --- Data bus mapping for store ---
  wire [LSUOOW-1:0] s2t_sdat =
    (s1o_lsu_length == 2'b00) ? {s1o_lsu_b1[7:0],s1o_lsu_b1[7:0],s1o_lsu_b1[7:0],s1o_lsu_b1[7:0]} : // byte access
    (s1o_lsu_length == 2'b01) ? {s1o_lsu_b1[15:0],s1o_lsu_b1[15:0]} : // halfword access
                                s1o_lsu_b1; // word access

  // --- align error detection ---
  wire s2t_align = s1o_op_lsu_ls &                                        // Align Exception: detection enabled
                   (((s1o_lsu_length == 2'b10) & (|s1o_virt_addr[1:0])) | // Align Exception: wrong word align
                    ((s1o_lsu_length == 2'b01) & s1o_virt_addr[0]));      // Align Exception: wrong short align


  // address conversion result
  wire [LSUOOW-1:0] s2t_phys_addr;
  wire              s2t_cache_inhibit;
  wire              s2t_tlb_miss;
  wire              s2t_pagefault;

  // Enable DMUU for load/store only
  wire dmmu_enable = dmmu_enable_i & exec_op_lsu_ls;

  //------------------//
  // Instance of DMMU //
  //------------------//

  mor1kx_dmmu_marocchino
  #(
    .FEATURE_DMMU_HW_TLB_RELOAD       (FEATURE_DMMU_HW_TLB_RELOAD), // DMMU
    .OPTION_OPERAND_WIDTH             (OPTION_OPERAND_WIDTH), // DMMU
    .OPTION_DMMU_SET_WIDTH            (OPTION_DMMU_SET_WIDTH), // DMMU
    .OPTION_DMMU_WAYS                 (OPTION_DMMU_WAYS), // DMMU
    .OPTION_DMMU_CLEAR_ON_INIT        (OPTION_DMMU_CLEAR_ON_INIT) // DMMU
  )
  u_dmmu
  (
    // clocks and resets
    .cpu_clk                          (cpu_clk), // DMMU
    .cpu_rst                          (cpu_rst), // DMMU
    // pipe controls
    .lsu_s1_adv_i                     (lsu_s1_adv), // DMMU
    .pipeline_flush_i                 (pipeline_flush_i), // DMMU
    // configuration and commands
    .enable_i                         (dmmu_enable), // DMMU
    .supervisor_mode_i                (supervisor_mode_i), // DMMU
    .s1o_op_lsu_store_i               (s1o_op_lsu_store), // DMMU
    .s1o_op_lsu_load_i                (s1o_op_lsu_load), // DMMU
    // address translation
    .virt_addr_idx_i                  (s1t_virt_addr), // DMMU
    .virt_addr_tag_i                  (s1o_virt_addr), // DMMU
    .phys_addr_o                      (s2t_phys_addr), // DMMU
    // translation flags
    .cache_inhibit_o                  (s2t_cache_inhibit), // DMMU
    .tlb_miss_o                       (s2t_tlb_miss), // DMMU
    .pagefault_o                      (s2t_pagefault), // DMMU
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
    .spr_bus_stb_i                    (spr_bus_stb_lsu), // DMMU
    .spr_bus_dat_i                    (spr_bus_dat_i), // DMMU
    .spr_bus_dat_o                    (spr_bus_dat_dmmu_o), // DMMU
    .spr_bus_ack_o                    (spr_bus_ack_dmmu_o) // DMMU
  );

  // Combination of align/tlb/pagefault
  assign s2t_excepts_addr = s2t_tlb_miss | s2t_pagefault | s2t_align;


  //-------------------//
  // Instance of cache //
  //-------------------//

  mor1kx_dcache_marocchino
  #(
    .OPTION_OPERAND_WIDTH         (OPTION_OPERAND_WIDTH), // DCACHE
    .OPTION_DCACHE_BLOCK_WIDTH    (OPTION_DCACHE_BLOCK_WIDTH), // DCACHE
    .OPTION_DCACHE_SET_WIDTH      (OPTION_DCACHE_SET_WIDTH), // DCACHE
    .OPTION_DCACHE_WAYS           (OPTION_DCACHE_WAYS), // DCACHE
    .OPTION_DCACHE_LIMIT_WIDTH    (OPTION_DCACHE_LIMIT_WIDTH), // DCACHE
    .OPTION_DCACHE_SNOOP          (OPTION_DCACHE_SNOOP), // DCACHE
    .OPTION_DCACHE_CLEAR_ON_INIT  (OPTION_DCACHE_CLEAR_ON_INIT) // DCACHE
  )
  u_dcache
  (
    // clock & reset
    .cpu_clk                    (cpu_clk), // DCACHE
    .cpu_rst                    (cpu_rst), // DCACHE
    // pipe controls
    .lsu_s1_adv_i               (lsu_s1_adv), // DACHE
    .lsu_s2_adv_i               (lsu_s2_adv), // DACHE
    .flush_by_ctrl_i            (flush_by_ctrl), // DCACHE
    // configuration
    .dc_enable_i                (dc_enable_i), // DCACHE
    // exceptions
    .dmmu_excepts_addr_i        (s2o_excepts_addr), // DCACHE
    .dbus_err_i                 (dbus_err_i), // DCACHE
    // Regular operation
    //  # addresses and "DCHACHE inhibit" flag
    .virt_addr_idx_i            (s1t_virt_addr), // DCACHE
    .virt_addr_s1o_i            (s1o_virt_addr), // DCACHE
    .phys_addr_s2t_i            (s2t_phys_addr), // DCACHE
    .dmmu_cache_inhibit_i       (s2t_cache_inhibit), // DCACHE
    //  # DCACHE regular answer
    .s1o_op_lsu_load_i          (s1o_op_lsu_load), // DCACHE
    .dc_ack_read_o              (s2t_dc_ack_read), // DCACHE
    .dc_dat_o                   (s2t_dc_dat), // DCACHE
    //  # STORE format / store data / do|cancel storing
    .s1o_op_lsu_store_i         (s1o_op_lsu_store), // DCACHE
    .dbus_bsel_i                (s2o_bsel), // DCACHE
    .dbus_sdat_i                (s2o_sdat), // DCACHE
    .dc_dat_s2o_i               (s2o_dc_dat), // DCACHE
    .s3t_store_ack_i            (s3t_store_ack), // DCACHE
    .dc_store_allowed_i         (dc_store_allowed), // DCACHE
    // re-fill
    .dc_refill_req_o            (s2t_dc_refill_req), // DCACHE
    .dc_refill_allowed_i        (dc_refill_allowed), // DCACHE
    .refill_first_o             (dc_refill_first), // DCACHE
    .phys_addr_s2o_i            (s2o_phys_addr), // DCACHE
    .dbus_dat_i                 (dbus_dat_i), // DCACHE
    .dbus_ack_i                 (dbus_ack_i), // DCACHE
    .dbus_burst_last_i          (dbus_burst_last_i), // DCACHE
    // DBUS read request
    .dbus_read_req_o            (s2t_dbus_read_req), // DCACHE
    // SNOOP
    .snoop_adr_i                (snoop_adr_i[31:0]), // DCACHE
    .snoop_event_i              (snoop_event), // DCACHE
    .snoop_hit_o                (snoop_hit), // DCACHE
    // SPR interface
    .spr_bus_addr_i             (spr_bus_addr_i[15:0]), // DCACHE
    .spr_bus_we_i               (spr_bus_we_i), // DCACHE
    .spr_bus_stb_i              (spr_bus_stb_lsu), // DCACHE
    .spr_bus_dat_i              (spr_bus_dat_i), // DCACHE
    .spr_bus_dat_o              (spr_bus_dat_dc_o), // DCACHE
    .spr_bus_ack_o              (spr_bus_ack_dc_o) // DCACHE
  );


  /*****************************************/
  /* Stage #2: DBUS acceess / DCACHE check */
  /*****************************************/

  //----------------------------//
  // STORE_BUFF error exception //
  //----------------------------//

  // --- bus error during bus access from store buffer ---
  //  ## pay attention that l.swa is executed around of
  //     store buffer, so we don't take it into accaunt here.
  wire sbuf_err = dbus_write_state & (~dbus_atomic) & dbus_err_i ; // to force empty STORE_BUFFER
  // ---
  always @(posedge cpu_clk) begin
    if (cpu_rst | flush_by_ctrl) // reset store buffer DBUS error
      sbuf_err_o <= 1'b0;
    else if (sbuf_err)          // rise store buffer DBUS error
      sbuf_err_o <= 1'b1;
  end // @ clock

  //----------//
  // DBUS FSM //
  //----------//

  assign dbus_burst_o = dc_refill_state;

  // Slightly subtle, but if there is an atomic store coming out from the
  // store buffer, and the link has been broken while it was waiting there,
  // the bus access is still performed as a (discarded) read.
  // MAROCCHINO_TODO: not correct for CDC
  assign dbus_we_o = dbus_we & ~dbus_swa_discard;

  // DBUS state machine: switching
  always @(posedge cpu_clk) begin
    if (cpu_rst) begin
      dbus_req_o <= 1'b0;      // DBUS_FSM reset
      dbus_state <= DBUS_IDLE; // DBUS_FSM reset
    end
    else begin
      // synthesis parallel_case full_case
      case (dbus_state)
        DBUS_IDLE: begin
          if (flush_by_ctrl)              // DBUS_FSM: keep idling
            dbus_state <= DBUS_IDLE;      // DBUS_FSM: keep idling
          else if (~sbuf_empty)           // DBUS_FSM: idle -> dbus-sbuf-read
            dbus_state <= DBUS_SBUF_READ; // DBUS_FSM: idle -> dbus-sbuf-read
          else if (lsu_s2_adv)            // DBUS_FSM: idle -> dmem req
            dbus_state <= DMEM_REQ;       // DBUS_FSM: idle -> dmem req
        end // idle

        DMEM_REQ: begin
          if (flush_by_ctrl) begin          // dmem req
            dbus_state <= DBUS_IDLE;        // dmem req: pipe flush
          end
          else if (~sbuf_empty) begin       // dmem req
            dbus_state <= DBUS_SBUF_READ;   // dmem req -> dbus-sbuf-read
          end
          else if (s2o_excepts_addr) begin  // dmem req
            dbus_state <= DBUS_IDLE;        // dmem req: address conversion an exception
          end
          else if (s2o_swa) begin
            if (~snoop_hit) begin
              dbus_req_o <= ~dbus_req_o;   // dmem req -> write for l.swa
              dbus_state <= DBUS_WRITE;    // dmem req -> write for l.swa
            end
          end
          else if (s2o_dc_refill_req) begin // dmem-req
            dbus_state <= DBUS_TO_REFILL;   // dmem-req -> to-re-fill
          end
          else if (s2o_dbus_read_req) begin // dmem-req
            dbus_req_o <= ~dbus_req_o;      // dmem-req -> dbus-read
            dbus_state <= DBUS_READ;        // dmem-req -> dbus-read
          end
          else if (~lsu_s2_adv) begin       // dmem-req: no new memory request MAROCCHINO_TODO: redundancy ??
            dbus_state <= DBUS_IDLE;        // dmem-req: no new memory request
          end
        end // dmem-req

        DBUS_READ: begin
          if (dbus_err_i | dbus_ack_i)      // dbus-read
            dbus_state <= DBUS_IDLE;        // dbus-read: complete
        end // dbus-read

        DBUS_TO_REFILL: begin
          if (flush_by_ctrl)             // to-re-fill
            dbus_state <= DBUS_IDLE;     // to-re-fill cancelled by flush
          else begin
            dbus_req_o <= ~dbus_req_o;     // to-re-fill -> dcache-re-fill
            dbus_state <= DBUS_DC_REFILL;  // to-re-fill -> dcache-re-fill
          end
        end // to-re-fill

        DBUS_DC_REFILL: begin
          if (dbus_err_i)                                       // dcache-re-fill
            dbus_state <= DBUS_IDLE;                            // dcache-re-fill: DBUS error
          else if (snoop_hit)                                   // dcache-re-fill
            dbus_state <= flush_by_ctrl ? DBUS_IDLE : DMEM_REQ; // dcache-re-fill: snoop-hit
          else if (dbus_ack_i & dbus_burst_last_i)              // dcache-re-fill
            dbus_state <= DBUS_DC_REREAD;                       // dcache-re-fill: last-ack
        end // dc-refill

        DBUS_DC_REREAD: begin       // dc-re-read
          dbus_state <= DBUS_IDLE;  // dc-re-read
        end // dc-re-read

        DBUS_SBUF_READ: begin
          dbus_state <= DBUS_INI_WRITE;  // dbus-sbuf-read
        end // dbus-sbuf-red

        DBUS_INI_WRITE: begin
          dbus_req_o <= ~dbus_req_o;     // dbus-ini-write -> write
          dbus_state <= DBUS_WRITE;      // dbus-ini-write -> write : from buffer output
        end // dbus-ini-write

        DBUS_WRITE: begin
          if (dbus_err_i) begin         // dbus-write
            dbus_state <= DBUS_IDLE;    // dbus-write: DBUS error
          end
          else if (dbus_ack_i) begin
            dbus_state <= sbuf_empty ? DMEM_REQ : DBUS_SBUF_READ; // DBUS: write complete
          end
        end // dbus-write

        default:;
      endcase
    end
  end // @ clock: DBUS_FSM

  // DBUS state machine: control signals
  always @(posedge cpu_clk) begin
    // synthesis parallel_case full_case
    case (dbus_state)
      DMEM_REQ: begin
        if (flush_by_ctrl | (~sbuf_empty) | s2o_excepts_addr) begin // dmem-req
        end
        else if (s2o_swa) begin
          if (~snoop_hit) begin
            dbus_we     <= 1'b1;          // dmem req -> write for l.swa
            dbus_bsel_o <= s2o_bsel;      // dmem req -> write for l.swa
            dbus_adr_o  <= s2o_phys_addr; // dmem req -> write for l.swa
            dbus_dat_o  <= s2o_sdat;      // dmem req -> write for l.swa
            dbus_atomic <= 1'b1;          // dmem req -> write for l.swa
          end
        end
        else if (s2o_dbus_read_req) begin // dmem-req
          dbus_we     <= 1'b0;            // dmem-req -> dbus-read
          dbus_bsel_o <= s2o_bsel;        // dmem-req -> dbus-read
          dbus_adr_o  <= s2o_phys_addr;   // dmem-req -> dbus-read
          dbus_atomic <= 1'b0;            // dmem-req -> dbus-read
        end
      end // dmem-req

      DBUS_TO_REFILL: begin
        if (~flush_by_ctrl) begin         // to-re-fill
          dbus_we     <= 1'b0;            // to-re-fill -> dcache-re-fill
          dbus_bsel_o <= 4'b1111;         // to-re-fill -> dcache-re-fill
          dbus_adr_o  <= s2o_phys_addr;   // to-re-fill -> dcache-re-fill
          dbus_atomic <= 1'b0;            // to-re-fill -> dcache-re-fill
        end
      end // to-re-fill

      DBUS_INI_WRITE: begin
        // DBUS controls
        dbus_we     <= 1'b1;            // dbus-ini-write -> write
        dbus_bsel_o <= sbuf_bsel;       // dbus-ini-write -> write
        dbus_adr_o  <= sbuf_phys_addr;  // dbus-ini-write -> write
        dbus_dat_o  <= sbuf_dat;        // dbus-ini-write -> write
        dbus_atomic <= 1'b0;            // dbus-ini-write -> write : l.swa goes around buffer
        // Update data for potential DBUS error on write
        //  -- they make sense only if sbuf-err is raised
        sbuf_eear_o <= sbuf_virt_addr;  // dbus-ini-write -> write : from buffer output
        sbuf_epcr_o <= sbuf_epcr;       // dbus-ini-write -> write : from buffer output
      end // dbus-ini-write

      DBUS_WRITE: begin
        if (dbus_err_i | dbus_ack_i)    // dbus-write
          dbus_atomic <= 1'b0;          // dbus-write: ACK/ERR
      end // dbus-write

      default:;
    endcase
  end // @ clock: DBUS_FSM



  //-----------------------//
  // Store buffer instance //
  //-----------------------//

  // store buffer write controls
  assign sbuf_write = s2o_store & (~sbuf_full) & (~snoop_hit) & (~lsu_wb_miss) & grant_wb_to_lsu_i;  // SBUFF write
  // include exceptions and pipe flushing
  assign sbuf_we    = sbuf_write & (~s2o_excepts_addr) & (~flush_by_ctrl);


  // store buffer module
  mor1kx_store_buffer_marocchino
  #(
    .DEPTH_WIDTH          (OPTION_STORE_BUFFER_DEPTH_WIDTH), // STORE_BUFFER
    .OPTION_OPERAND_WIDTH (OPTION_OPERAND_WIDTH), // STORE_BUFFER
    .CLEAR_ON_INIT        (OPTION_STORE_BUFFER_CLEAR_ON_INIT) // STORE_BUFFER
  )
  u_store_buffer
  (
    .cpu_clk              (cpu_clk), // STORE_BUFFER
    .cpu_rst              (cpu_rst), // STORE_BUFFER
    // DBUS error during write data from store buffer (force empty)
    .sbuf_err_i           (sbuf_err), // STORE_BUFFER
    // entry port
    .sbuf_epcr_i          (s2o_epcr), // STORE_BUFFER
    .virt_addr_i          (s2o_virt_addr), // STORE_BUFFER
    .phys_addr_i          (s2o_phys_addr), // STORE_BUFFER
    .dat_i                (s2o_sdat), // STORE_BUFFER
    .bsel_i               (s2o_bsel), // STORE_BUFFER
    .write_i              (sbuf_we), // STORE_BUFFER
    // output port
    .sbuf_epcr_o          (sbuf_epcr), // STORE_BUFFER
    .virt_addr_o          (sbuf_virt_addr), // STORE_BUFFER
    .phys_addr_o          (sbuf_phys_addr), // STORE_BUFFER
    .dat_o                (sbuf_dat), // STORE_BUFFER
    .bsel_o               (sbuf_bsel), // STORE_BUFFER
    .read_i               (sbuf_read_state), // STORE_BUFFER
    // status flags
    .full_o               (sbuf_full), // STORE_BUFFER
    .empty_o              (sbuf_empty) // STORE_BUFFER
  );


  // We have to mask out our own snooped bus access
  assign snoop_event = (snoop_en_i & ~((snoop_adr_i == dbus_adr_o) & dbus_ack_i)) &
                       (OPTION_DCACHE_SNOOP != "NONE");


  //-------------------------//
  // Atomic operations logic //
  //-------------------------//
  // MAROCCHINO_TODO: not correct for CDC
  assign dbus_swa_discard = dbus_atomic & ~atomic_reserve;
  // ---
  assign dbus_swa_ack     = dbus_atomic & dbus_ack_i;
  assign dbus_swa_err     = dbus_atomic & dbus_err_i;
  // ---
  always @(posedge cpu_clk) begin
    if (cpu_rst | flush_by_ctrl) // drop atomic reserve
      atomic_reserve <= 1'b0;
    else if (s2o_excepts_any |                                    // drop atomic reserve
             dbus_swa_ack |                                       // drop atomic reserve
             (s2o_store & (s2o_phys_addr == atomic_addr)) |       // drop atomic reserve
             (snoop_event & (snoop_adr_i == atomic_addr))) begin  // drop atomic reserve
      atomic_reserve <= 1'b0;
    end
    else if (s2o_lwa) begin
      if (snoop_event & (snoop_adr_i == s2o_phys_addr))
        atomic_reserve <= 1'b0;
      else if (s3t_load_ack) // set atomic-reserve
        atomic_reserve <= 1'b1;
    end
  end // @clock
  // ---
  always @(posedge cpu_clk) begin
    if (s2o_lwa) begin
      if (snoop_event & (snoop_adr_i == s2o_phys_addr)) begin
      end
      else if (s3t_load_ack) begin // set atomic-address
        atomic_addr <= s2o_phys_addr;
      end
    end
  end // @clock
  // ---
  reg s2o_atomic_flag_set;
  reg s2o_atomic_flag_clear;
  // ---
  always @(posedge cpu_clk) begin
    if (cpu_rst | flush_by_ctrl) begin // reset/flush s2o- atomic set/clear flags
      s2o_atomic_flag_set   <= 1'b0;
      s2o_atomic_flag_clear <= 1'b0;
    end
    else if (dbus_swa_ack) begin          // determine  s2o- atomic set/clear flags
      s2o_atomic_flag_set   <=   atomic_reserve;
      s2o_atomic_flag_clear <= (~atomic_reserve);
    end
    else if (lsu_s3_adv) begin            // drop  s2o- atomic set/clear flags
      s2o_atomic_flag_set   <= 1'b0;
      s2o_atomic_flag_clear <= 1'b0;
    end
  end // @clock



  //-------------------------//
  // S2O_* control registers //
  //-------------------------//

  // latches for none atomic store command
  always @(posedge cpu_clk) begin
    if (cpu_rst | flush_by_ctrl) begin
      s2o_op_lsu_store <= 1'b0;
      s2o_store        <= 1'b0;
      s2o_swa          <= 1'b0;
    end
    else if (lsu_s2_adv) begin // latch store command in stage #2
      s2o_op_lsu_store <= s1o_op_lsu_store;
      s2o_store        <= s1o_op_lsu_store & (~s1o_op_lsu_atomic);
      s2o_swa          <= s1o_op_lsu_store &   s1o_op_lsu_atomic;
    end
    else if (s3t_store_ack) begin // claer store command in stage #2
      s2o_op_lsu_store <= 1'b0;
      s2o_store        <= 1'b0;
      s2o_swa          <= 1'b0;
    end
  end // @clock

  // latches for store data
  always @(posedge cpu_clk) begin
    if (lsu_s2_adv) begin
      s2o_epcr <= s1o_sbuf_epcr;
      s2o_bsel <= s2t_bsel;
      s2o_sdat <= s2t_sdat;
    end
  end // @clock

  // latches for load (either atomic or not) commands
  always @(posedge cpu_clk) begin
    if (cpu_rst | flush_by_ctrl) begin
      s2o_lwa  <= 1'b0;
    end
    else if (lsu_s2_adv) begin // latch atomic load command
      s2o_lwa  <= s1o_op_lsu_load & s1o_op_lsu_atomic;
    end
    else if (s3t_load_ack) begin // clean atomic load command
      s2o_lwa  <= 1'b0;
    end
  end // @clock

  // latch additional parameters of a command
  //       and calculated virtual adderss
  always @(posedge cpu_clk) begin
    if (lsu_s2_adv) begin
      // additional parameters of a command
      s2o_length        <= s1o_lsu_length;
      s2o_zext          <= s1o_lsu_zext;
      // virtual and physical addersses
      s2o_virt_addr     <= s1o_virt_addr;
      s2o_phys_addr     <= s2t_phys_addr;
    end
  end // @clock


  // --- DCACHE re-fill request ---
  wire deassert_s2o_dc_refill_req = (dbus_idle_state   & flush_by_ctrl)    | // de-assert re-fill request
                                    (dmem_req_state    & flush_by_ctrl)    | // de-assert re-fill request
                                    (dmem_req_state    & s2o_excepts_addr) | // de-assert re-fill request
                                    (dc_refill_allowed & flush_by_ctrl)    | // de-assert re-fill request
                                    (dc_refill_state   & dbus_err_i)       | // de-assert re-fill request
                                    dc_reread_state;                         // de-assert re-fill request
  // ---
  always @(posedge cpu_clk) begin
    if (cpu_rst)
      s2o_dc_refill_req <= 1'b0;              // reset / flush
    else if (lsu_s2_adv)
      s2o_dc_refill_req <= s2t_dc_refill_req;
    else if (deassert_s2o_dc_refill_req)
      s2o_dc_refill_req <= 1'b0;              // re-fill done or canceled
  end // @ clock
  // --- DCACHE ack ---
  always @(posedge cpu_clk) begin
    if (cpu_rst | flush_by_ctrl)
      s2o_dc_ack_read <= 1'b0;  // reset / flush
    else if (lsu_s2_adv)        // rise dc-ack-read
      s2o_dc_ack_read <= s2t_dc_ack_read;
    else if (lsu_s3_adv)        // drop dc-ack-read
      s2o_dc_ack_read <= 1'b0;  // WriteBack/Pending is taking result
  end // @ clock
  // --- DCACHE data ---
  always @(posedge cpu_clk) begin
    if (lsu_s2_adv)             // latch DCACHE data
      s2o_dc_dat <= s2t_dc_dat;
  end // @ clock


  // --- DBUS read request ---
  wire deassert_s2o_dbus_read_req = (dbus_idle_state & flush_by_ctrl)    | // de-assert dbus read request
                                    (dmem_req_state  & flush_by_ctrl)    | // de-assert dbus read request
                                    (dmem_req_state  & s2o_excepts_addr) | // de-assert dbus read request
                                    (dbus_read_state & (dbus_ack_i | dbus_err_i)); // de-assert dbus read request
  // ---
  always @(posedge cpu_clk) begin
    if (cpu_rst)
      s2o_dbus_read_req <= 1'b0;          // reset / flush
    else if (lsu_s2_adv)
      s2o_dbus_read_req <= s2t_dbus_read_req;
    else if (deassert_s2o_dbus_read_req)
      s2o_dbus_read_req <= 1'b0;          // dbus read done or canceled
  end // @ clock
  // --- combined DBUS-load/SBUFF-store ACK ---
  always @(posedge cpu_clk) begin
    if (cpu_rst | flush_by_ctrl)
      s3o_ls_ack <= 1'b0;                     // reset / flush
    else if (dbus_load_ack | s3t_store_ack)   // rise combined DBUS-load/SBUFF-store ACK
      s3o_ls_ack <= 1'b1;                     // dbus-load-ack OR s3t-store-ack
    else if (lsu_s3_adv)                      // drop combined DBUS-load/SBUFF-store ACK
      s3o_ls_ack <= 1'b0;                     // WriteBack/Pending is taking result
  end // @ clock
  // --- DBUS load data ---
  always @(posedge cpu_clk) begin
    if (dbus_load_ack)            // latch DBUS read data
      s2o_dbus_dat <= dbus_dat_i;
  end // @ clock


  // --- latches for exceptions ---
  always @(posedge cpu_clk) begin
    if (cpu_rst | flush_by_ctrl) begin
      // address conversion exceptions
      s2o_tlb_miss       <= 1'b0; // reset / flush
      s2o_pagefault      <= 1'b0; // reset / flush
      s2o_align          <= 1'b0; // reset / flush
      // "instant" (not related to STORE_BUFFER) DBUS error
      s2o_dbus_err_nsbuf <= 1'b0; // reset / flush
      // combined exceptions
      s2o_excepts_addr   <= 1'b0; // reset / flush
      s2o_excepts_any    <= 1'b0; // reset / flush
    end
    else if (lsu_s2_adv) begin                  // latch address conversion exceptions
      // address conversion exceptions
      s2o_tlb_miss       <= s2t_tlb_miss;
      s2o_pagefault      <= s2t_pagefault;
      s2o_align          <= s2t_align;
      // combined exceptions
      s2o_excepts_addr   <= s2t_excepts_addr;
      s2o_excepts_any    <= s2t_excepts_addr;
    end
    else if (dbus_err_i) begin                  // rise s2o_* DBUS error
      s2o_dbus_err_nsbuf <= s3t_dbus_err_nsbuf; // at a DBUS error
      s2o_excepts_any    <= 1'b1;               // at a DBUS error
    end
  end // @clock


  // --- "operation complete" and "LSU valid" ---
  assign lsu_s2_rdy  = s2o_dc_ack_read | s3o_ls_ack | s2o_excepts_any;
  // --- "operation complete" and "LSU valid" ---
  assign lsu_valid_o = lsu_s2_rdy | lsu_wb_miss;
  //--- "WriteBack miss" flag ---
  always @(posedge cpu_clk) begin
    if (cpu_rst | flush_by_ctrl)
      lsu_wb_miss <= 1'b0;
    else if (padv_wb_i & grant_wb_to_lsu_i)
      lsu_wb_miss <= 1'b0;
    else if (lsu_s3_adv)    // rise lsu-wb-miss
      lsu_wb_miss <= 1'b1;
  end // @clock


  //-----------------//
  // LSU load output //
  //-----------------//

  wire [LSUOOW-1:0] s3t_ldat = s2o_dc_ack_read ? s2o_dc_dat : s2o_dbus_dat;

  // Select part of bus for load
  reg [LSUOOW-1:0] s3t_ldat_aligned;
  // ---
  always @(s2o_virt_addr[1:0] or s3t_ldat) begin
    // synthesis parallel_case full_case
    case(s2o_virt_addr[1:0])
      2'b00: s3t_ldat_aligned = s3t_ldat;
      2'b01: s3t_ldat_aligned = {s3t_ldat[23:0],8'd0};
      2'b10: s3t_ldat_aligned = {s3t_ldat[15:0],16'd0};
      2'b11: s3t_ldat_aligned = {s3t_ldat[7:0],24'd0};
    endcase
  end

  // Do appropriate extension for load
  reg [LSUOOW-1:0] s3t_ldat_extended;
  // ---
  always @(s2o_zext or s2o_length or s3t_ldat_aligned) begin
    // synthesis parallel_case full_case
    case({s2o_zext, s2o_length})
      3'b100:  s3t_ldat_extended = {24'd0,s3t_ldat_aligned[31:24]}; // lbz
      3'b101:  s3t_ldat_extended = {16'd0,s3t_ldat_aligned[31:16]}; // lhz
      3'b000:  s3t_ldat_extended = {{24{s3t_ldat_aligned[31]}},
                                    s3t_ldat_aligned[31:24]}; // lbs
      3'b001:  s3t_ldat_extended = {{16{s3t_ldat_aligned[31]}},
                                    s3t_ldat_aligned[31:16]}; // lhs
      default: s3t_ldat_extended = s3t_ldat_aligned;
    endcase
  end


  localparam  ODAT_LDAT_LSB          = 0;
  localparam  ODAT_LDAT_MSB          = LSUOOW - 1;
  localparam  ODAT_EXCEPT_ADDR_LSB   = ODAT_LDAT_MSB          + 1;
  localparam  ODAT_EXCEPT_ADDR_MSB   = ODAT_EXCEPT_ADDR_LSB   + LSUOOW - 1;
  localparam  ODAT_ATOMIC_FLAG_SET   = ODAT_EXCEPT_ADDR_MSB   + 1;
  localparam  ODAT_ATOMIC_FLAG_CLR   = ODAT_ATOMIC_FLAG_SET   + 1;
  localparam  ODAT_EXCEPT_DBUS_ERR   = ODAT_ATOMIC_FLAG_CLR   + 1;
  localparam  ODAT_EXCEPT_DPAGEFAULT = ODAT_EXCEPT_DBUS_ERR   + 1;
  localparam  ODAT_EXCEPT_DTLB_MISS  = ODAT_EXCEPT_DPAGEFAULT + 1;
  localparam  ODAT_EXCEPT_DBUS_ALIGN = ODAT_EXCEPT_DTLB_MISS  + 1;
  localparam  ODAT_EXCEPTS_ANY       = ODAT_EXCEPT_DBUS_ALIGN + 1;

  // MSB and data width for 1st and Write-Back taps
  //  -- without "result or except" bit
  localparam  ODAT_MISS_MSB = ODAT_EXCEPTS_ANY;

  // LSU's output data set
  wire [ODAT_MISS_MSB:0] s3t_odat =
    {
      // Any exception
      s2o_excepts_any,        // tap of output data set
      // Particular LSU exception flags
      s2o_align,              // tap of output data set
      s2o_tlb_miss,           // tap of output data set
      s2o_pagefault,          // tap of output data set
      s2o_dbus_err_nsbuf,     // tap of output data set
      // Atomic operation flag set/clear logic
      s2o_atomic_flag_clear,  // tap of output data set
      s2o_atomic_flag_set,    // tap of output data set
      // WB-output assignement
      s2o_virt_addr,          // tap of output data set
      s3t_ldat_extended       // tap of output data set
    };
  // LSU's output data set registered by WriteBack miss
  reg [ODAT_MISS_MSB:0] s3o_odat_miss;
  // ---
  always @(posedge cpu_clk) begin
    if (lsu_s3_adv) // save output data set in "miss" register
      s3o_odat_miss <= s3t_odat;
  end // @clock

  // pre-WB exceprions & errors
  // MAROCCHINO_TODO: need more accurate processing for store buffer bus error
  assign exec_an_except_lsu_o = (lsu_wb_miss ? s3o_odat_miss[ODAT_EXCEPTS_ANY] : s2o_excepts_any) & grant_wb_to_lsu_i;

  // LSU's WB-registered output data set (without "an except flag)
  localparam  ODAT_WB_MSB = ODAT_MISS_MSB - 1;
  localparam  ODAT_WB_DW  = ODAT_WB_MSB   + 1;
  // ---
  reg  [ODAT_WB_MSB:0] wb_odat;
  wire [LSUOOW-1:0] wb_lsu_result_m = lsu_wb_miss ? s3o_odat_miss[ODAT_LDAT_MSB:ODAT_LDAT_LSB] : s3t_odat[ODAT_LDAT_MSB:ODAT_LDAT_LSB];
  // ---
  always @(posedge cpu_clk) begin
    if (padv_wb_i) begin
      if (grant_wb_to_lsu_i) begin
        wb_odat             <= lsu_wb_miss ? s3o_odat_miss[ODAT_WB_MSB:0] : s3t_odat[ODAT_WB_MSB:0];
        wb_lsu_result_cp1_o <= wb_lsu_result_m;
        wb_lsu_result_cp2_o <= wb_lsu_result_m;
        wb_lsu_result_cp3_o <= wb_lsu_result_m;
      end
      else begin
        wb_odat             <= {ODAT_WB_DW{1'b0}};
        wb_lsu_result_cp1_o <= {ODAT_WB_DW{1'b0}};
        wb_lsu_result_cp2_o <= {ODAT_WB_DW{1'b0}};
        wb_lsu_result_cp3_o <= {ODAT_WB_DW{1'b0}};
      end
    end
  end // @clock

  // WB-output assignement
  assign wb_lsu_result_o        = wb_odat[ODAT_LDAT_MSB:ODAT_LDAT_LSB];
  assign wb_lsu_except_addr_o   = wb_odat[ODAT_EXCEPT_ADDR_MSB:ODAT_EXCEPT_ADDR_LSB];
  // Atomic operation flag set/clear logic
  assign wb_atomic_flag_set_o   = wb_odat[ODAT_ATOMIC_FLAG_SET];
  assign wb_atomic_flag_clear_o = wb_odat[ODAT_ATOMIC_FLAG_CLR];
  // Particular LSU exception flags
  assign wb_except_dbus_err_o   = wb_odat[ODAT_EXCEPT_DBUS_ERR];
  assign wb_except_dpagefault_o = wb_odat[ODAT_EXCEPT_DPAGEFAULT];
  assign wb_except_dtlb_miss_o  = wb_odat[ODAT_EXCEPT_DTLB_MISS];
  assign wb_except_dbus_align_o = wb_odat[ODAT_EXCEPT_DBUS_ALIGN];

endmodule // mor1kx_lsu_marocchino
