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
  input      [OPTION_OPERAND_WIDTH-1:0] exec_sbuf_epcr_i,       // for store buffer EPCR computation
  input           [`OR1K_IMM_WIDTH-1:0] exec_lsu_imm16_i, // immediate offset for address computation
  input      [OPTION_OPERAND_WIDTH-1:0] exec_lsu_a1_i,   // operand "A" (part of address)
  input      [OPTION_OPERAND_WIDTH-1:0] exec_lsu_b1_i,   // operand "B" (value to store)
  input                                 exec_op_lsu_load_i,
  input                                 exec_op_lsu_store_i,
  input                                 exec_op_lsu_atomic_i,
  input                           [1:0] exec_lsu_length_i,
  input                                 exec_lsu_zext_i,
  input                                 exec_op_msync_i,
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
  input      [OPTION_OPERAND_WIDTH-1:0] dbus_burst_adr_i,
  input                                 dbus_burst_last_i,
  // Cache sync for multi-core environment
  input                          [31:0] snoop_adr_i,
  input                                 snoop_en_i,
  // Output flags and load result
  output                                lsu_taking_op_o,
  output reg                            lsu_valid_o, // result ready or exceptions
  output     [OPTION_OPERAND_WIDTH-1:0] wb_lsu_result_o,
  output reg                            wb_lsu_valid_miss_o,
  output                                wb_rfd1_wb_lsu_miss_o, // alias for wb_lsu_valid_miss_o
  output                                wb_flag_wb_lsu_miss_o, // alias for wb_lsu_valid_miss_o
  // Exceprions & errors
  //  # pre-WB
  output                                exec_an_except_lsu_o,
  //  # Indicator of dbus exception came via the store buffer
  //    and appropriate PC
  output reg [OPTION_OPERAND_WIDTH-1:0] sbuf_eear_o,
  output reg [OPTION_OPERAND_WIDTH-1:0] sbuf_epcr_o,
  output reg                            sbuf_err_o,
  // exception output
  //  # particular LSU exception flags
  output     [OPTION_OPERAND_WIDTH-1:0] wb_lsu_except_addr_o,
  output                                wb_except_dbus_err_o,
  output                                wb_except_dpagefault_o,
  output                                wb_except_dtlb_miss_o,
  output                                wb_except_dbus_align_o,
  // Atomic operation flag set/clear logic
  output                                wb_atomic_flag_set_o,
  output                                wb_atomic_flag_clear_o
);


  // short names for local use
  localparam LSUOOW = OPTION_OPERAND_WIDTH;


  // input registers for DMMU stage
  reg  [LSUOOW-1:0] s1r_virt_addr;
  reg  [LSUOOW-1:0] s1r_lsu_b1;
  reg  [LSUOOW-1:0] s1r_sbuf_epcr;       // for store buffer EPCR computation
  reg               s1r_op_lsu_load;
  reg               s1r_op_lsu_store;
  reg               s1r_op_lsu_atomic;
  reg         [1:0] s1r_lsu_length;
  reg               s1r_lsu_zext;
  reg               s1r_op_msync;
  reg               s1r_op_lsu_ls;
  /* HW reload TLB related (MAROCCHINO_TODO : not implemented yet)
  wire              tlb_reload_req;
  wire              tlb_reload_busy;
  wire [LSUOOW-1:0] tlb_reload_addr;
  wire              tlb_reload_pagefault;
  reg               tlb_reload_ack;
  reg  [LSUOOW-1:0] tlb_reload_data;
  wire              tlb_reload_pagefault_clear;
  reg               tlb_reload_done; */


  // Input register for DBUS/DCACHE access stage
  //  # load/store
  reg               s2r_load;  // either atomic or not
  reg               s2r_lwa;   // exactly load linked
  reg               s2r_store; // not (!!!) atomic
  reg               s2r_swa;   // atomic only
  reg         [1:0] s2r_length;
  reg               s2r_zext;
  //  # DBUS "bsel" and formatted data to store
  reg         [3:0] s2r_bsel;
  reg  [LSUOOW-1:0] s2r_sdat;  // register file B in (store operand)
  //  # registers for store buffer EPCR computation
  reg  [LSUOOW-1:0] s2r_epcr;
  //  # latched virtual and physical addresses
  reg  [LSUOOW-1:0] s2r_virt_addr;
  reg  [LSUOOW-1:0] s2r_phys_addr;
  //  # latched address convertion attributes
  reg               s2r_cache_inhibit;
  reg               s2r_tlb_miss;
  reg               s2r_pagefault;
  reg               s2r_align;
  //  # l.msync
  reg               s2r_op_msync;


  // DBUS FSM
  //  # DBUS FSM states
  localparam [4:0] DBUS_IDLE      = 5'b00001, // eq. to DCACHE
                   DMEM_REQ       = 5'b10000,
                   DBUS_READ      = 5'b00010, // eq. to DCACHE
                   DBUS_WRITE     = 5'b00100, // eq. to DCACHE
                   DBUS_DC_REFILL = 5'b01000; // eq. to DCACHE
  //  # DBUS FSM state indicator
  reg        [4:0] dbus_state;
  //  # particular states
  wire             dbus_idle_state  = dbus_state[0];
  wire             dmem_req_state   = dbus_state[4];
  wire             dbus_read_state  = dbus_state[1];
  wire             dbus_write_state = dbus_state[2];
  wire             dc_refill_state  = dbus_state[3];
  //  # DBUS FSM other registers & wires
  reg              dbus_we;
  reg              dbus_atomic;
  wire             dbus_swa_discard; // reservation is lost, execute empty read
  wire             dbus_swa_success; // l.swa is successfull
  wire             dbus_swa_ack;     // complete DBUS trunsaction with l.swa
  reg              sbuf_odata;       // not written data on buffer's output


  // DCACHE
  wire              dc_ack;
  wire [LSUOOW-1:0] dc_dat;
  wire              dc_access_read;
  wire              dc_refill_req;
  reg               dc_refill_allowed;
  wire              dc_refill_first;


  // Store buffer
  wire              sbuf_write; // without exceptions and pipeline-flush
  wire              sbuf_we;    // with exceptions and pipeline-flush
  // ---
  wire              sbuf_rdwr_empty;  // read and write simultaneously if buffer is empty
  wire              sbuf_re;          // with exceptions and pipeline-flush
  // ---
  wire              sbuf_full;
  wire              sbuf_empty;
  wire [LSUOOW-1:0] sbuf_epcr;
  wire [LSUOOW-1:0] sbuf_virt_addr;
  wire [LSUOOW-1:0] sbuf_phys_addr;
  wire [LSUOOW-1:0] sbuf_dat;
  wire        [3:0] sbuf_bsel;

  // Atomic operations
  reg  [LSUOOW-1:0] atomic_addr;
  reg               atomic_reserve;

  // Snoop (for multicore SoC)
  wire              snoop_event;
  wire              snoop_hit;


  // Exceptions detected on DCACHE/DBUS access stage
  // # combination of align/tlb/pagefault registered in s2r_*
  wire              s2t_excepts_addr;
  // # combined of address + DBUS instant error
  wire              s2t_excepts_any; // non-registered
  wire              s2t_ack_excepts_any; // to put in output buffer: MAROCCHINO_TODO: reg_it ??
  // # pending for either address or DBUS errors
  reg               s2p_excepts_any; // registered


  // LSU pipe controls
  //  # report on command execution
  wire              s2t_ack_load;
  wire              s2t_ack_store;
  wire              s2t_ack_swa;
  //  # LSU's pipe free signals (LSU is able to take next command)
  wire              s2t_free_load;  // complete load
  wire              s2t_free_store; // complete store
  wire              s2t_free_all;   // also includes l.msync, exceptions and flushing
  //  # All output buffer rooms are reserved
  reg               obuff_closed_r;

  // Flushing logic provides continuous clean up output
  // from pipeline-flush till transaction (read/re-fill) completion
  reg               flush_r;
  wire              flush_by_ctrl;


  //*******************//
  // LSU pipe controls //
  //*******************//

  // advance stage #1 (address conversion)
  assign lsu_taking_op_o = exec_op_lsu_any_i & (~s1r_op_lsu_ls | s2t_free_all);


  // Exceptions detected on DCACHE/DBUS access stage
  //  # exceptions related to address computation and conversion
  assign s2t_excepts_addr = s2r_align | s2r_tlb_miss | s2r_pagefault; // MAROCCHINO_TODO: reg_it ??
  //  # all exceptions
  assign s2t_excepts_any  = s2t_excepts_addr | dbus_err_i;
  //  # pending latch for any LSU exception
  always @(posedge cpu_clk) begin
    if (cpu_rst | flush_by_ctrl)  // drop any LSU exception pending latch
      s2p_excepts_any <= 1'b0;
    else if (~s2p_excepts_any)  // test any LSU exception pending latch
      s2p_excepts_any <= s2t_excepts_any;
  end // @clock
  //  # push output buffer
  assign s2t_ack_excepts_any = s2t_excepts_any & (~s2p_excepts_any);


  // Stage #2 is taking new load/store command (less priority than flushing or exceptions)
  wire s2_taking_load  = s1r_op_lsu_load  & s2t_free_all; // for DCACHE mostly
  wire s2_taking_store = s1r_op_lsu_store & s2t_free_all; // for DCACHE mostly
  wire s2_taking_ls    = s1r_op_lsu_ls    & s2t_free_all;

  // l.load completion / waiting / WB-miss
  wire   dbus_ack_load  = (dbus_read_state | dc_refill_first) & dbus_ack_i;
  // ---
  assign s2t_ack_load   = (s2r_load & dc_ack) | dbus_ack_load;
  // ---
  assign s2t_free_load  = (~s2r_load) | /*MAROCCHINO_TODO: dc_ack |*/ dbus_ack_load;// LSU free of l.load


  // l.store completion / waiting / WB-miss
  assign s2t_ack_store  = sbuf_write; // it already includes s2r_store
  // ---
  assign s2t_ack_swa    = s2r_swa & dbus_swa_ack; // MAROCCHINO_TODO: just dbus-swa-ack ?
  // ---
  assign s2t_free_store = ((~s2r_store) & (~s2r_swa)) | s2t_ack_store | s2t_ack_swa; // LSU free of l.store


  // LSU is able to take next command
  assign s2t_free_all = s2t_free_load & s2t_free_store &   // LSU is free
                        (~dc_refill_state)             &   // LSU is free
                        (~s2r_op_msync)                &   // LSU is free
                        (~snoop_hit)                   &   // LSU is free : MAROCCHINO_TODO: already taken into accaunt by others ?
                        (~obuff_closed_r)              &   // LSU is free
                        (~s2p_excepts_any) & (~flush_r);   // LSU is free


  //-----------------------------------------------------------------//
  // Flushing from pipeline-flush-i till DBUS transaction completion //
  //-----------------------------------------------------------------//

  // initial value of flush-r for simulations
 `ifndef SYNTHESIS
  // synthesis translate_off
  initial flush_r = 1'b0;
  // synthesis translate_on
 `endif // !synth
  // ----
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


  /*********************************/
  /* Stage #0: address computation */
  /*********************************/

  // compute virtual address
  wire [LSUOOW-1:0] s0t_virt_addr = exec_lsu_a1_i + {{(LSUOOW-16){exec_lsu_imm16_i[15]}},exec_lsu_imm16_i};


  /********************************/
  /* Stage #1: address conversion */
  /********************************/

  // l.msync is express (and 1-clock length)
  always @(posedge cpu_clk) begin
    if (cpu_rst) // ! no flushing for l.msync
      s1r_op_msync <= 1'b0;
    else if (lsu_taking_op_o) // latch l.msync from LSU-RSRVS
      s1r_op_msync <= exec_op_msync_i;
    else
      s1r_op_msync <= 1'b0; // already passed on stage #2
  end // @cpu-clk

  // load/store and "new command is in stage #1" flag
  always @(posedge cpu_clk) begin
    if (cpu_rst | pipeline_flush_i) begin
      s1r_op_lsu_load   <= 1'b0;
      s1r_op_lsu_store  <= 1'b0;
      s1r_op_lsu_atomic <= 1'b0;
      s1r_op_lsu_ls     <= 1'b0;
    end
    else if (lsu_taking_op_o) begin // rise "new load/store command is in stage #1" flag
      s1r_op_lsu_load   <= exec_op_lsu_load_i;
      s1r_op_lsu_store  <= exec_op_lsu_store_i;
      s1r_op_lsu_atomic <= exec_op_lsu_atomic_i;
      s1r_op_lsu_ls     <= ~exec_op_msync_i;
    end
    else if (s2_taking_ls) begin   // drop "new load/store command is in stage #1" flag
      s1r_op_lsu_load   <= 1'b0;
      s1r_op_lsu_store  <= 1'b0;
      s1r_op_lsu_atomic <= 1'b0;
      s1r_op_lsu_ls     <= 1'b0;
    end
  end // @cpu-clk

  // load/store attributes
  always @(posedge cpu_clk) begin
    if (cpu_rst | pipeline_flush_i) begin
      s1r_virt_addr     <= {LSUOOW{1'b0}};
      s1r_lsu_b1        <= {LSUOOW{1'b0}};
      s1r_sbuf_epcr     <= {LSUOOW{1'b0}};
      s1r_lsu_length    <= 2'd0;
      s1r_lsu_zext      <= 1'b0;
    end
    else if (lsu_taking_op_o) begin // latch data from LSU-RSRVS without l.msync
      s1r_virt_addr     <= s0t_virt_addr;
      s1r_lsu_b1        <= exec_lsu_b1_i;
      s1r_sbuf_epcr     <= exec_sbuf_epcr_i;
      s1r_lsu_length    <= exec_lsu_length_i;
      s1r_lsu_zext      <= exec_lsu_zext_i;
    end
  end // @cpu-clk

  // address conversion result
  wire [LSUOOW-1:0] s1t_phys_addr;
  wire              s1t_cache_inhibit;
  wire              s1t_tlb_miss;
  wire              s1t_pagefault;

  // Enable DMUU for load/store only
  wire dmmu_enable = dmmu_enable_i & (~exec_op_msync_i);

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
    .padv_dmmu_i                      (lsu_taking_op_o), // DMMU
    .pipeline_flush_i                 (pipeline_flush_i), // DMMU
    // configuration and commands
    .enable_i                         (dmmu_enable), // DMMU
    .supervisor_mode_i                (supervisor_mode_i), // DMMU
    .op_lsu_store_i                   (s1r_op_lsu_store), // DMMU
    .op_lsu_load_i                    (s1r_op_lsu_load), // DMMU
    // address translation
    .virt_addr_idx_i                  (s0t_virt_addr), // DMMU
    .virt_addr_tag_i                  (s1r_virt_addr), // DMMU
    .phys_addr_o                      (s1t_phys_addr), // DMMU
    // translation flags
    .cache_inhibit_o                  (s1t_cache_inhibit), // DMMU
    .tlb_miss_o                       (s1t_tlb_miss), // DMMU
    .pagefault_o                      (s1t_pagefault), // DMMU
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

  // --- Big endian bus mapping ---
  reg [3:0] s1t_bsel;
  // ---
  always @(s1r_lsu_length or s1r_virt_addr[1:0]) begin
    // synthesis parallel_case full_case
    case (s1r_lsu_length)
      2'b00: // byte access
      begin
        // synthesis parallel_case full_case
        case(s1r_virt_addr[1:0])
          2'b00: s1t_bsel = 4'b1000;
          2'b01: s1t_bsel = 4'b0100;
          2'b10: s1t_bsel = 4'b0010;
          2'b11: s1t_bsel = 4'b0001;
        endcase
      end
      2'b01: // halfword access
      begin
        // synthesis parallel_case full_case
        case(s1r_virt_addr[1])
          1'b0: s1t_bsel = 4'b1100;
          1'b1: s1t_bsel = 4'b0011;
        endcase
      end
      2'b10,
      2'b11: s1t_bsel = 4'b1111;
    endcase
  end

  // --- Data bus mapping for store ---
  wire [LSUOOW-1:0] s1t_sdat =
    (s1r_lsu_length == 2'b00) ? {s1r_lsu_b1[7:0],s1r_lsu_b1[7:0],s1r_lsu_b1[7:0],s1r_lsu_b1[7:0]} : // byte access
    (s1r_lsu_length == 2'b01) ? {s1r_lsu_b1[15:0],s1r_lsu_b1[15:0]} : // halfword access
                                s1r_lsu_b1; // word access

  // --- align error detection ---
  wire s1t_align = (s1r_op_lsu_load | s1r_op_lsu_store) &                 // Align Exception: detection enabled
                   (((s1r_lsu_length == 2'b10) & (|s1r_virt_addr[1:0])) | // Align Exception: wrong word align
                    ((s1r_lsu_length == 2'b01) & s1r_virt_addr[0]));      // Align Exception: wrong short align


  /*********************************/
  /* Stage #2: DBUS/DCACHE acceess */
  /*********************************/

  //   To clean up registers on the stage we use only pipeline-flush, but not flush-r.
  // It makes possible to get next to flush instruction and start execution just after
  // completion DBUS transaction which has been flushed by pipeline flush.

  // latches for load (either atomic or not) commands
  always @(posedge cpu_clk) begin
    if (cpu_rst | pipeline_flush_i) begin
      s2r_load <= 1'b0;
      s2r_lwa  <= 1'b0;
    end
    else if (s2_taking_load) begin // latch load command (either atomic or not)
      s2r_load <= 1'b1;
      s2r_lwa  <= s1r_op_lsu_atomic;
    end
    else if (s2t_ack_load) begin
      s2r_load <= 1'b0;
      s2r_lwa  <= 1'b0;
    end
  end // @clock

  // latches for none atomic store command
  always @(posedge cpu_clk) begin
    if (cpu_rst | pipeline_flush_i)
      s2r_store <= 1'b0;
    else if (s2_taking_store) // latch none atomic store command
      s2r_store <= ~s1r_op_lsu_atomic;
    else if (s2t_ack_store)
      s2r_store <= 1'b0;
  end // @clock

  // latches for atomic store command
  always @(posedge cpu_clk) begin
    if (cpu_rst | pipeline_flush_i)
      s2r_swa <= 1'b0;
    else if (s2_taking_store) // latch atomic store command
      s2r_swa <= s1r_op_lsu_atomic;
    else if (s2t_ack_swa)
      s2r_swa <= 1'b0;
  end // @clock

  // latches for store buffer EPCR
  always @(posedge cpu_clk) begin
    if (s2_taking_store) // store buffer EPCR: doesn't matter atomic or not
      s2r_epcr <= s1r_sbuf_epcr;
  end // @clock

  // DBUS "bsel" and formatted data to store
  always @(posedge cpu_clk) begin
    if (s2_taking_ls) begin
      s2r_bsel <= s1t_bsel;
      s2r_sdat <= s1t_sdat;
    end
  end // @clock

  // latch additional parameters of a command
  //       and calculated virtual adderss
  always @(posedge cpu_clk) begin
    if (cpu_rst) begin
      // additional parameters of a command
      s2r_length        <= 2'd0;
      s2r_zext          <= 1'b0;
      // virtual and physical addersses
      s2r_virt_addr     <= {LSUOOW{1'b0}};
      s2r_phys_addr     <= {LSUOOW{1'b0}};
    end
    else if (s2_taking_ls) begin
      // additional parameters of a command
      s2r_length        <= s1r_lsu_length;
      s2r_zext          <= s1r_lsu_zext;
      // virtual and physical addersses
      s2r_virt_addr     <= s1r_virt_addr;
      s2r_phys_addr     <= s1t_phys_addr;
    end
  end // @clock

  // latch address conversion attributes
  always @(posedge cpu_clk) begin
    if (cpu_rst | pipeline_flush_i) begin
      s2r_cache_inhibit <= 1'b0;
      s2r_tlb_miss      <= 1'b0;
      s2r_pagefault     <= 1'b0;
      s2r_align         <= 1'b0;
    end
    else if (s2_taking_ls) begin
      s2r_cache_inhibit <= s1t_cache_inhibit;
      s2r_tlb_miss      <= s1t_tlb_miss;
      s2r_pagefault     <= s1t_pagefault;
      s2r_align         <= s1t_align;
    end
  end // @clock


  // LSU dosen't take next commad till completion all previous ones
  always @(posedge cpu_clk) begin
    if (cpu_rst)
      s2r_op_msync <= 1'b0;
    else if (s1r_op_msync) // assert busy by l.msync
      s2r_op_msync <= 1'b1;
    else if (dbus_idle_state & sbuf_empty) // deassert busy by l.msync
      s2r_op_msync <= 1'b0;
  end // @clock


  //----------------------//
  // DBUS error exception //
  //----------------------//

  // --- bus error during bus access from store buffer ---
  //  ## pay attention that l.swa is executed around of
  //     store buffer, so we don't take into accaunt
  //     atomic store here.
  wire sbuf_err = dbus_err_i & dbus_we & ~dbus_atomic;
  // ---
  always @(posedge cpu_clk) begin
    if (cpu_rst | flush_by_ctrl) // prevent store buffer DBUS error report
      sbuf_err_o  <= 1'b0;
    else if (sbuf_err)          // rise store buffer DBUS error
      sbuf_err_o  <= 1'b1;
  end // @ clock


  //-----------------//
  // LSU load output //
  //-----------------//

  wire [LSUOOW-1:0] s2t_ldat = dbus_ack_load ? dbus_dat_i : dc_dat;

  // Select part of bus for load
  reg [LSUOOW-1:0] s2t_ldat_aligned;
  // ---
  always @(s2r_virt_addr[1:0] or s2t_ldat) begin
    // synthesis parallel_case full_case
    case(s2r_virt_addr[1:0])
      2'b00: s2t_ldat_aligned = s2t_ldat;
      2'b01: s2t_ldat_aligned = {s2t_ldat[23:0],8'd0};
      2'b10: s2t_ldat_aligned = {s2t_ldat[15:0],16'd0};
      2'b11: s2t_ldat_aligned = {s2t_ldat[7:0],24'd0};
    endcase
  end

  // Do appropriate extension for load
  reg [LSUOOW-1:0] s2t_ldat_extended;
  // ---
  always @(s2r_zext or s2r_length or s2t_ldat_aligned) begin
    // synthesis parallel_case full_case
    case({s2r_zext, s2r_length})
      3'b100:  s2t_ldat_extended = {24'd0,s2t_ldat_aligned[31:24]}; // lbz
      3'b101:  s2t_ldat_extended = {16'd0,s2t_ldat_aligned[31:16]}; // lhz
      3'b000:  s2t_ldat_extended = {{24{s2t_ldat_aligned[31]}},
                                    s2t_ldat_aligned[31:24]}; // lbs
      3'b001:  s2t_ldat_extended = {{16{s2t_ldat_aligned[31]}},
                                    s2t_ldat_aligned[31:16]}; // lhs
      default: s2t_ldat_extended = s2t_ldat_aligned;
    endcase
  end



  //----------//
  // DBUS FSM //
  //----------//

  assign dbus_burst_o = dc_refill_state;

  // Slightly subtle, but if there is an atomic store coming out from the
  // store buffer, and the link has been broken while it was waiting there,
  // the bus access is still performed as a (discarded) read.
  assign dbus_we_o = dbus_we & ~dbus_swa_discard;


  // re-filll-allowed corresponds to refill-request position in DBUS FSM
  // !!! exceptions and flushing are already taken into accaunt in DCACHE,
  //     so we don't use them here
  always @(dbus_state or sbuf_odata or s2r_store or s2r_swa or dc_refill_req) begin
    dc_refill_allowed = 1'b0;
    // synthesis parallel_case full_case
    case (dbus_state)
      DMEM_REQ: begin
        if (sbuf_odata | s2r_store | s2r_swa) // re-fill not allowed
          dc_refill_allowed = 1'b0;
        else if (dc_refill_req) // it automatically means (l.load & dc-access)
          dc_refill_allowed = 1'b1;
      end
      default: begin
      end
    endcase
  end // always

  // state machine
  always @(posedge cpu_clk) begin
    if (cpu_rst) begin
      // DBUS controls
      dbus_req_o  <= 1'b0;            // DBUS reset
      dbus_we     <= 1'b0;            // DBUS reset
      dbus_adr_o  <= {LSUOOW{1'b0}};  // DBUS reset : for correct sim. with snoop
      dbus_atomic <= 1'b0;            // DBUS reset
      sbuf_odata  <= 1'b0;            // DBUS reset
      // DBUS FSM state
      dbus_state  <= DBUS_IDLE;       // DBUS reset
    end
    else if (dbus_err_i) begin // in DBUS FSM: highest priority
      // DBUS controls
      dbus_req_o  <= dbus_req_o;      // DBUS error: no toggle
      dbus_we     <= 1'b0;            // DBUS error
      dbus_atomic <= 1'b0;            // DBUS error
      sbuf_odata  <= 1'b0;            // DBUS error
      // DBUS FSM state
      dbus_state  <= DBUS_IDLE;       // DBUS error
    end
    else begin
      // process
      // synthesis parallel_case full_case
      case (dbus_state)
        DBUS_IDLE: begin
          if (pipeline_flush_i) // DBUS FSM keep idling
            dbus_state <= DBUS_IDLE;
          else if (s2_taking_ls) // idle -> dmem req
            dbus_state <= DMEM_REQ;
        end

        DMEM_REQ: begin
          if (sbuf_odata) begin
            // DBUS controls
            //  # buffered data for write == store buffer is not empty
            dbus_req_o  <= ~dbus_req_o;     // dmem req -> write : from buffer output
            dbus_we     <= 1'b1;            // dmem req -> write : from buffer output
            dbus_bsel_o <= sbuf_bsel;       // dmem req -> write : from buffer output
            dbus_adr_o  <= sbuf_phys_addr;  // dmem req -> write : from buffer output
            dbus_dat_o  <= sbuf_dat;        // dmem req -> write : from buffer output
            dbus_atomic <= 1'b0;            // dmem req -> write : from buffer output : l.swa goes around buffer
            sbuf_odata  <= 1'b0;            // dmem req -> write : from buffer output
            //  # update data for potential (!) DBUS error on write
            //  # they make sense only if sbuf-err is raised
            sbuf_eear_o <= sbuf_virt_addr;  // dmem req -> write : from buffer output
            sbuf_epcr_o <= sbuf_epcr;       // dmem req -> write : from buffer output
            // DBUS FSM state
            dbus_state  <= DBUS_WRITE;      // dmem req -> write : from buffer output
          end
          else if (s2t_excepts_addr | pipeline_flush_i) begin // dmem req
            dbus_state  <= DBUS_IDLE; // dmem req -> exceptions or pipe flush
          end
          else if (s2r_store | s2r_swa) begin
            if (~snoop_hit) begin
              // DBUS controls
              //  # no buffered data for write == store buffer is empty
              //    we wrte data in buffer, but it keeps empty in the case
              //    because we also perform implicit instant reading
              dbus_req_o  <= ~dbus_req_o;   // dmem req -> write : 1st write in empty buffer
              dbus_we     <= 1'b1;          // dmem req -> write : 1st write in empty buffer
              dbus_bsel_o <= s2r_bsel;      // dmem req -> write : 1st write in empty buffer
              dbus_adr_o  <= s2r_phys_addr; // dmem req -> write : 1st write in empty buffer
              dbus_dat_o  <= s2r_sdat;      // dmem req -> write : 1st write in empty buffer
              dbus_atomic <= s2r_swa;     // dmem req -> write : l.swa goes around buffer
              sbuf_odata  <= 1'b0;          // dmem-req -> write : 1st write in empty buffer
              // DBUS FSM state
              dbus_state  <= DBUS_WRITE;
            end
          end
          else if (dc_refill_req) begin // it automatically means (l.load & dc-access)
            dbus_req_o  <= ~dbus_req_o;   // dmem-req -> dcache-re-fill
            dbus_adr_o  <= s2r_phys_addr;
            dbus_state  <= DBUS_DC_REFILL;
          end
          else if (s2r_load & ~dc_access_read) begin // not cached or DCACHE is disabled
            dbus_req_o  <= ~dbus_req_o;   // dmem-req -> dbus-read
            dbus_adr_o  <= s2r_phys_addr;
            dbus_bsel_o <= s2r_bsel;
            dbus_state  <= DBUS_READ;
          end
          else if (~s2_taking_ls) begin // no new memory request
            dbus_state <= DBUS_IDLE;    // no new memory request
          end
        end // idle

        DBUS_DC_REFILL: begin
          if (snoop_hit) begin
            dbus_state  <= flush_by_ctrl ? DBUS_IDLE : DMEM_REQ;  // DC-REFILL: snoop-hit
          end
          else if (dbus_ack_i & dbus_burst_last_i) begin
            dbus_state  <= DBUS_IDLE;       // DC-REFILL: DBUS-last-ack
          end
        end // dc-refill

        DBUS_READ: begin
          if (dbus_ack_i)
             dbus_state  <= flush_by_ctrl ? DBUS_IDLE : DMEM_REQ; // DBUS: read complete
        end // read

        DBUS_WRITE: begin
          if (dbus_ack_i) begin
            // DBUS controls
            dbus_we     <= 1'b0;           // DBUS: write complete
            dbus_atomic <= 1'b0;
            // pending data for write (see also sbuf_re)
            if ((~sbuf_empty) | (sbuf_rdwr_empty & ~(s2t_excepts_addr | pipeline_flush_i))) begin // DBUS: write complete
              sbuf_odata <= 1'b1;     // DBUS: current write complete, process next write
              dbus_state <= DMEM_REQ; // DBUS: current write complete, process next write
            end
            else if (s2_taking_ls | s2r_load | s2r_swa)
              dbus_state <= DMEM_REQ;  // DBUS: write complete, no more writes, take new or proc pending command
            else
              dbus_state <= DBUS_IDLE; // DBUS: write complete, no more writes, no new command
          end
        end // write-state

        default: begin
          // DBUS controls
          dbus_req_o  <= dbus_req_o;      // DBUS deault: no toggle
          dbus_we     <= 1'b0;            // DBUS deault
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
  assign sbuf_write = s2r_store & ~sbuf_full & ~snoop_hit;  // SBUFF write
  // include exceptions and pipe flushing
  assign sbuf_we = sbuf_write & ~(s2t_excepts_any | pipeline_flush_i);


  //
  // store buffer read controls
  //
  // auxiliary: read and write simultaneously if buffer is empty
  assign sbuf_rdwr_empty = s2r_store & sbuf_empty & ~snoop_hit;
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
  assign sbuf_re = (dmem_req_state   & sbuf_rdwr_empty & ~sbuf_odata & ~(s2t_excepts_addr | pipeline_flush_i)) | // SBUFF read, dmem-req
  //               in WRITE state if write request overlaps DBUS ACK and empty buffer state
                   (dbus_write_state & dbus_ack_i & sbuf_rdwr_empty & ~(s2t_excepts_addr | pipeline_flush_i))  | // SBUFF read, dbus-write
  //               in WRITE state we read non-empty buffer to prepare next write
                   (dbus_write_state & dbus_ack_i & ~sbuf_empty); // SBUFF read, dbus-write


  // store buffer module
  mor1kx_store_buffer_marocchino
  #(
    .DEPTH_WIDTH          (OPTION_STORE_BUFFER_DEPTH_WIDTH), // STORE_BUFFER
    .OPTION_OPERAND_WIDTH (OPTION_OPERAND_WIDTH), // STORE_BUFFER
    .CLEAR_ON_INIT        (OPTION_STORE_BUFFER_CLEAR_ON_INIT) // STORE_BUFFER
  )
  u_store_buffer
  (
    .cpu_clk      (cpu_clk), // STORE_BUFFER
    .cpu_rst      (cpu_rst), // STORE_BUFFER
    // DBUS error during write data from store buffer (force empty)
    .sbuf_err_i   (sbuf_err), // STORE_BUFFER
    // entry port
    .sbuf_epcr_i  (s2r_epcr), // STORE_BUFFER
    .virt_addr_i  (s2r_virt_addr), // STORE_BUFFER
    .phys_addr_i  (s2r_phys_addr), // STORE_BUFFER
    .dat_i        (s2r_sdat), // STORE_BUFFER
    .bsel_i       (s2r_bsel), // STORE_BUFFER
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
  // // MAROCCHINO_TODO: not correct for CDC
  assign dbus_swa_discard = dbus_atomic & ~atomic_reserve;
  // ---
  assign dbus_swa_ack     = dbus_atomic & dbus_ack_i;
  // ---
  assign dbus_swa_success = dbus_swa_ack &  atomic_reserve; // for WB & DCACHE
  wire   dbus_swa_fail    = dbus_swa_ack & ~atomic_reserve; // for WB
  // ---
  always @(posedge cpu_clk) begin
    if (cpu_rst | pipeline_flush_i) begin // drop atomic reserve
      atomic_reserve <= 1'b0;
      atomic_addr    <= {LSUOOW{1'b0}};
    end
    else if (s2t_excepts_any |                                    // drop atomic reserve
             dbus_swa_ack |                                       // drop atomic reserve
             (s2r_store & (s2r_phys_addr == atomic_addr)) |       // drop atomic reserve
             (snoop_event & (snoop_adr_i == atomic_addr))) begin  // drop atomic reserve
      atomic_reserve <= 1'b0;
      atomic_addr    <= {LSUOOW{1'b0}};
    end
    else if (s2r_lwa) begin
      if (snoop_event & (snoop_adr_i == s2r_phys_addr)) begin
        atomic_reserve <= 1'b0;
        atomic_addr    <= {LSUOOW{1'b0}};
      end
      else if (s2t_ack_load) begin
        atomic_reserve <= 1'b1;
        atomic_addr    <= s2r_phys_addr;
      end
    end
  end // @clock


  //-------------------//
  // Instance of cache //
  //-------------------//

  wire dc_store_allowed = s2t_ack_store | dbus_swa_success;

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
    .dc_taking_load_i           (s2_taking_load), // DCACHE
    .dc_taking_store_i          (s2_taking_store), // DCACHE
    .pipeline_flush_i           (pipeline_flush_i), // DCACHE
    // configuration
    .dc_enable_i                (dc_enable_i), // DCACHE
    // exceptions
    .dmmu_excepts_addr_i        (s2t_excepts_addr), // DCACHE
    .dbus_err_i                 (dbus_err_i), // DCACHE
    // Regular operation
    //  # addresses and "DCHACHE inhibit" flag
    .virt_addr_idx_i            (s1r_virt_addr), // DCACHE
    .virt_addr_cmd_i            (s2r_virt_addr), // DCACHE
    .phys_addr_tag_i            (s2r_phys_addr), // DCACHE
    .dmmu_cache_inhibit_i       (s2r_cache_inhibit), // DCACHE
    //  # DCACHE regular answer
    .dc_access_read_o           (dc_access_read), // DCACHE
    .dc_ack_o                   (dc_ack), // DCACHE
    .dc_dat_o                   (dc_dat), // DCACHE
    //  # STORE format / store data / do|cancel storing
    .dbus_bsel_i                (s2r_bsel), // DCACHE
    .dbus_sdat_i                (s2r_sdat), // DCACHE
    .dc_store_allowed_i         (dc_store_allowed), // DCACHE
    .dbus_swa_discard_i         (dbus_swa_discard), // DCACHE
    // re-fill
    .dc_refill_req_o            (dc_refill_req), // DCACHE
    .dc_refill_allowed_i        (dc_refill_allowed), // DCACHE
    .refill_first_o             (dc_refill_first), // DCACHE
    .dbus_dat_i                 (dbus_dat_i), // DCACHE
    .dbus_ack_i                 (dbus_ack_i), // DCACHE
    .dbus_burst_adr_i           (dbus_burst_adr_i), // DCACHE
    .dbus_burst_last_i          (dbus_burst_last_i), // DCACHE
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


  //----------------------------------------------------------------------------//
  // Oputput triple buffer to prevent padv_wb backward propagation to LSU_RSRVS //
  //----------------------------------------------------------------------------//

  localparam  OBUFF_LDAT_LSB          = 0;
  localparam  OBUFF_LDAT_MSB          = LSUOOW - 1;
  localparam  OBUFF_EXCEPT_ADDR_LSB   = OBUFF_LDAT_MSB          + 1;
  localparam  OBUFF_EXCEPT_ADDR_MSB   = OBUFF_EXCEPT_ADDR_LSB   + LSUOOW - 1;
  localparam  OBUFF_ATOMIC_FLAG_SET   = OBUFF_EXCEPT_ADDR_MSB   + 1;
  localparam  OBUFF_ATOMIC_FLAG_CLR   = OBUFF_ATOMIC_FLAG_SET   + 1;
  localparam  OBUFF_EXCEPT_DBUS_ERR   = OBUFF_ATOMIC_FLAG_CLR   + 1;
  localparam  OBUFF_EXCEPT_DPAGEFAULT = OBUFF_EXCEPT_DBUS_ERR   + 1;
  localparam  OBUFF_EXCEPT_DTLB_MISS  = OBUFF_EXCEPT_DPAGEFAULT + 1;
  localparam  OBUFF_EXCEPT_DBUS_ALIGN = OBUFF_EXCEPT_DTLB_MISS  + 1;
  localparam  OBUFF_EXCEPTS_ANY       = OBUFF_EXCEPT_DBUS_ALIGN + 1;
  localparam  OBUFF_RESULT_OR_EXCEPT  = OBUFF_EXCEPTS_ANY       + 1;

  // MSB and data width for 1st and Write-Back taps
  //  -- without "result or except" bit
  localparam  OBUFF_WB_MSB = OBUFF_EXCEPT_DBUS_ALIGN;
  localparam  OBUFF_WB_DW  = OBUFF_WB_MSB + 1;

  // MSB and data width for 2nd pending tap
  //  -- with "result or except" bit
  localparam  OBUFF_PX_MSB = OBUFF_RESULT_OR_EXCEPT;
  localparam  OBUFF_PX_DW  = OBUFF_PX_MSB + 1;

  // Data to store in Write-Back tap of output buffer
  //  -- without "result or except" bit
  wire [OBUFF_WB_MSB:0] obuff_wb_din =
    {
      // Particular LSU exception flags
      s2r_align,            // Write-Back tap of output buffer
      s2r_tlb_miss,         // Write-Back tap of output buffer
      s2r_pagefault,        // Write-Back tap of output buffer
      dbus_err_i,           // Write-Back tap of output buffer : MAROCCHINO_TODO : complete it
      // Atomic operation flag set/clear logic
      dbus_swa_fail,        // Write-Back tap of output buffer
      dbus_swa_success,     // Write-Back tap of output buffer
      // WB-output assignement
      s2r_virt_addr,        // Write-Back tap of output buffer (exception virtual address)
      s2t_ldat_extended     // Write-Back tap of output buffer
    };

  // Data to store in Write-Back tap of output buffer
  //  -- with "result or except" bit
  wire [OBUFF_PX_MSB:0] obuff_px_din = {1'b1,s2t_excepts_any,obuff_wb_din};

  // output buffer taps
  reg [OBUFF_WB_MSB:0] obuff_wb; // Write-Back tap
  reg [OBUFF_PX_MSB:0] obuff_p1; // Pre-WB tap
  reg [OBUFF_PX_MSB:0] obuff_p2; // Pre-Pre-WB tap

  // pointers
  reg  [1:0] obuff_curr_ptr; // on p1 or p2
  reg  [1:0] obuff_prev_ptr; // on wb or p1

  // LSU is writing result or an exception into output buffer
  wire obuff_we = s2t_ack_load | s2t_ack_store | s2t_ack_swa | s2t_ack_excepts_any;

  // Write-Back is reading LSU's output with taking into accaunt "missed" result
  wire obuff_re = (padv_wb_i & grant_wb_to_lsu_i) | wb_lsu_valid_miss_o;

  // Concatinated control for output buffer (for debug mostly)
  // {(new commnad), (write result to buff), (read by access to WB)}
  wire [2:0] obuff_nc_we_re = {s2_taking_ls, obuff_we, obuff_re};

  // Output buffer state machine
  always @(posedge cpu_clk) begin
    if (cpu_rst | flush_by_ctrl) begin
      // buffer taps
      obuff_wb <= {OBUFF_WB_DW{1'b0}};
      obuff_p1 <= {OBUFF_PX_DW{1'b0}};
      obuff_p2 <= {OBUFF_PX_DW{1'b0}};
      // pointers
      obuff_curr_ptr <= 2'b01;
      obuff_prev_ptr <= 2'b01;
      // all taps are reserved flag
      obuff_closed_r <= 1'b0;
      // speculative valid flag
      lsu_valid_o <= 1'b0; // reset / pipeline flush
      // miss flag(s)
      wb_lsu_valid_miss_o <= 1'b0; // reset / pipeline flush
    end
    else begin
      // synthesis parallel_case full_case
      case (obuff_nc_we_re)
        // read only:
        3'b001: begin
          // push buffer
          obuff_wb <= obuff_p1[OBUFF_WB_MSB:0];
          obuff_p1 <= obuff_p2;
          obuff_p2 <= {OBUFF_PX_DW{1'b0}};
          // "decrement" pointers
          obuff_curr_ptr <= obuff_p2[OBUFF_RESULT_OR_EXCEPT] ? obuff_curr_ptr : 2'b01;
          obuff_prev_ptr <= obuff_p2[OBUFF_RESULT_OR_EXCEPT] ? obuff_prev_ptr : 2'b01;
          // "open" buffer
          obuff_closed_r <= 1'b0;
          // real valid flag
          lsu_valid_o <= obuff_p2[OBUFF_RESULT_OR_EXCEPT]; // outbuff controls: nc = 0, we = 0, re = 1
          // miss flag(s)
          wb_lsu_valid_miss_o <= ~obuff_p1[OBUFF_RESULT_OR_EXCEPT]; // outbuff controls: nc = 0, we = 0, re = 1
        end

        // write only:
        3'b010: begin
          // wb-tap should be meaningful for 1-clock only
          obuff_wb <= {OBUFF_WB_DW{1'b0}};
          // put LSU's result into current tap
          obuff_p1 <= obuff_curr_ptr[0] ? obuff_px_din : obuff_p1;
          obuff_p2 <= obuff_curr_ptr[1] ? obuff_px_din : obuff_p2;
          // "increment" pointers
          obuff_curr_ptr <= 2'b10; // set to p2 by nc = 0, we = 1, re = 0
          obuff_prev_ptr <= 2'b10; // set to p1 by nc = 0, we = 1, re = 0
          // "open"/"close" buffer
          obuff_closed_r <= obuff_curr_ptr[1]; // write to p2 by nc = 0, we = 1, re = 0
          // real valid flag
          lsu_valid_o <= 1'b1;
        end

        // read overlapping write:
        3'b011: begin
          // put LSU's result into previous tap
          obuff_wb <= obuff_prev_ptr[0] ? obuff_wb_din : obuff_p1[OBUFF_WB_MSB:0];
          obuff_p1 <= obuff_prev_ptr[1] ? obuff_px_din : obuff_p2;
          obuff_p2 <= {OBUFF_PX_DW{1'b0}};
          // "open" buffer (without conditions)
          obuff_closed_r <= 1'b0;
          // real valid flag
          lsu_valid_o <= obuff_prev_ptr[1]; // outbuff controls: nc = 0, we = 1, re = 1
          // miss flag(s)
          wb_lsu_valid_miss_o <= 1'b0; // outbuff controls: nc = 0, we = 1, re = 1
        end

        // LSU is taking new command:
        3'b100: begin
          // wb-tap should be meaningful for 1-clock only
          obuff_wb <= {OBUFF_WB_DW{1'b0}};
          // "close" buffer if last tap (p2) has become "reserved"
          obuff_closed_r <= obuff_curr_ptr[1];
          // speculative valid flag
          lsu_valid_o <= 1'b1; // outbuff controls: nc = 1, we = 0, re = 0
        end

        // New command ovelapping read:
        3'b101: begin
          obuff_wb <= obuff_p1[OBUFF_WB_MSB:0];
          obuff_p1 <= obuff_p2;
          obuff_p2 <= {OBUFF_PX_DW{1'b0}};
          // "decrement" pointers
          obuff_curr_ptr <= obuff_p2[OBUFF_RESULT_OR_EXCEPT] ? obuff_curr_ptr : 2'b01;
          obuff_prev_ptr <= obuff_p2[OBUFF_RESULT_OR_EXCEPT] ? obuff_prev_ptr : 2'b01;
          // "close" buffer if last tap (p2) keep "reserved"
          obuff_closed_r <= obuff_p2[OBUFF_RESULT_OR_EXCEPT];
          // speculative valid flag
          lsu_valid_o <= 1'b1; // outbuff controls: nc = 1, we = 0, re = 1
          // miss flag(s)
          wb_lsu_valid_miss_o <= ~obuff_p1[OBUFF_RESULT_OR_EXCEPT]; // outbuff controls: nc = 1, we = 0, re = 1
        end

        // New command overlapping write:
        3'b110: begin
          // wb-tap should be meaningful for 1-clock only
          obuff_wb <= {OBUFF_WB_DW{1'b0}};
          // put LSU's result into current tap
          obuff_p1 <= obuff_curr_ptr[0] ? obuff_px_din : obuff_p1;
          obuff_p2 <= obuff_curr_ptr[1] ? obuff_px_din : obuff_p2;
          // "increment" pointers
          obuff_curr_ptr <= 2'b10; // set to p2 by nc = 1, we = 1, re = 0
          obuff_prev_ptr <= 2'b10; // set to p1 by nc = 1, we = 1, re = 0
          // "open"/"close" buffer
          obuff_closed_r <= obuff_curr_ptr[0]; // write to p2 by nc = 1, we = 1, re = 0
          // speculative valid flag
          lsu_valid_o <= 1'b1; // outbuff controls: nc = 1, we = 1, re = 0
        end

        // All events at the same time
        3'b111: begin
          // put LSU's result into previous tap
          obuff_wb <= obuff_prev_ptr[0] ? obuff_wb_din : obuff_p1[OBUFF_WB_MSB:0];
          obuff_p1 <= obuff_prev_ptr[1] ? obuff_px_din : obuff_p2;
          obuff_p2 <= {OBUFF_PX_DW{1'b0}};
          // "close" buffer if last tap has become "reserved"
          obuff_closed_r <= obuff_curr_ptr[1];
          // speculative valid flag
          lsu_valid_o <= 1'b1; // outbuff controls: nc = 1, we = 1, re = 1
          // miss flag(s)
          wb_lsu_valid_miss_o <= 1'b0; // outbuff controls: nc = 0, we = 1, re = 1
        end

        // no actions
        default: begin
          // wb-tap should be meaningful for 1-clock only
          obuff_wb <= {OBUFF_WB_DW{1'b0}};
        end
      endcase
    end
  end // @cpu-clk


  // pre-WB exceprions & errors
  // MAROCCHINO_TODO: need more accurate processing for store buffer bus error
  assign exec_an_except_lsu_o = (obuff_p1[OBUFF_RESULT_OR_EXCEPT] ? obuff_p1[OBUFF_EXCEPTS_ANY] : s2t_excepts_any) & (grant_wb_to_lsu_i | wb_lsu_valid_miss_o);

  // WB-output assignement
  assign wb_lsu_result_o        = obuff_wb[OBUFF_LDAT_MSB:OBUFF_LDAT_LSB];
  assign wb_lsu_except_addr_o   = obuff_wb[OBUFF_EXCEPT_ADDR_MSB:OBUFF_EXCEPT_ADDR_LSB];
  // Atomic operation flag set/clear logic
  assign wb_atomic_flag_set_o   = obuff_wb[OBUFF_ATOMIC_FLAG_SET];
  assign wb_atomic_flag_clear_o = obuff_wb[OBUFF_ATOMIC_FLAG_CLR];
  // Particular LSU exception flags
  assign wb_except_dbus_err_o   = obuff_wb[OBUFF_EXCEPT_DBUS_ERR];
  assign wb_except_dpagefault_o = obuff_wb[OBUFF_EXCEPT_DPAGEFAULT];
  assign wb_except_dtlb_miss_o  = obuff_wb[OBUFF_EXCEPT_DTLB_MISS];
  assign wb_except_dbus_align_o = obuff_wb[OBUFF_EXCEPT_DBUS_ALIGN];


  //--------------------//
  // Other "miss" flags //
  //--------------------//
  // MAROCCHINO_TODO: snoop_hit and miss relations ??

  // LSU's load ACK miss
  //   - prevents resolving D1 related hazards
  //   - prevents write back to RF
  assign wb_rfd1_wb_lsu_miss_o = wb_lsu_valid_miss_o;

  // LSU's l.swa ACK miss
  //   - prevents write back flag and taking conditional branches
  assign wb_flag_wb_lsu_miss_o = wb_lsu_valid_miss_o;

endmodule // mor1kx_lsu_marocchino
