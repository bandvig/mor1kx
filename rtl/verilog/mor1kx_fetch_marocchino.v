////////////////////////////////////////////////////////////////////////
//                                                                    //
//  mor1kx_fetch_marocchino                                           //
//                                                                    //
//  Description: mor1kx fetch/address stage unit                      //
//               for MAROCCHINO pipeline                              //
//               basically an interface to the ibus/icache subsystem  //
//               that can react to exception and branch signals       //
//                                                                    //
//               refactored version of mor1kx_fetch_cappuccino        //
//                                                                    //
////////////////////////////////////////////////////////////////////////
//                                                                    //
//   Copyright (C) 2012  Julius Baxter                                //
//                       juliusbaxter@gmail.com                       //
//                                                                    //
//                       Stefan Kristiansson                          //
//                       stefan.kristiansson@saunalahti.fi            //
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

module mor1kx_fetch_marocchino
#(
  parameter OPTION_OPERAND_WIDTH        = 32,
  parameter OPTION_RF_ADDR_WIDTH        =  5,
  // branch predictor parameters
  parameter GSHARE_BITS_NUM             = 10,
  // cache configuration
  parameter OPTION_ICACHE_BLOCK_WIDTH   =  5,
  parameter OPTION_ICACHE_SET_WIDTH     =  9,
  parameter OPTION_ICACHE_WAYS          =  2,
  parameter OPTION_ICACHE_LIMIT_WIDTH   = 32,
  parameter OPTION_ICACHE_CLEAR_ON_INIT =  0,
  // mmu configuration
  parameter FEATURE_IMMU_HW_TLB_RELOAD  = "NONE",
  parameter OPTION_IMMU_SET_WIDTH       =  6,
  parameter OPTION_IMMU_WAYS            =  1,
  parameter OPTION_IMMU_CLEAR_ON_INIT   =  0
)
(
  // clock and reset
  input                                 cpu_clk,
  input                                 cpu_rst,

  // pipeline control
  input                                 padv_fetch_i,
  input                                 pipeline_flush_i,

  // configuration
  input                                 ic_enable_i,
  input                                 immu_enable_i,
  input                                 supervisor_mode_i,

  // SPR interface
  //  input
  input [15:0]                          spr_bus_addr_i,
  input                                 spr_bus_we_i,
  input                                 spr_bus_stb_i,
  input      [OPTION_OPERAND_WIDTH-1:0] spr_bus_dat_i,
  //  output from cache
  output     [OPTION_OPERAND_WIDTH-1:0] spr_bus_dat_ic_o,
  output                                spr_bus_ack_ic_o,
  //  output from immu
  output     [OPTION_OPERAND_WIDTH-1:0] spr_bus_dat_immu_o,
  output                                spr_bus_ack_immu_o,

  // interface to ibus
  input                                 ibus_err_i,
  input                                 ibus_ack_i,
  input          [`OR1K_INSN_WIDTH-1:0] ibus_dat_i,
  input                                 ibus_burst_last_i,
  output reg                            ibus_req_o,
  output reg [OPTION_OPERAND_WIDTH-1:0] ibus_adr_o,
  output reg                            ibus_burst_o,

  // Jump/Branch processing
  //  # jump/branch variants
  output                                fetch_op_jimm_o,
  output                                fetch_op_jr_o,
  output                                fetch_op_bf_o,
  output                                fetch_op_bnf_o,
  //  # combined jump/branch flag
  output                                fetch_op_jb_o,
  //  # "to immediate driven target"
  output     [OPTION_OPERAND_WIDTH-1:0] fetch_to_imm_target_o,
  //  # do branch (predicted or unconditional)
  input                                 do_branch_i,
  input      [OPTION_OPERAND_WIDTH-1:0] do_branch_target_i,
  input                                 jr_gathering_target_i,
  //  # branch prediction support
  output     [OPTION_OPERAND_WIDTH-1:0] after_ds_target_o,
  input                                 predict_miss_i,
  output                          [1:0] bc_cnt_value_o,  // current value of saturation counter
  output          [GSHARE_BITS_NUM-1:0] bc_cnt_radr_o,   // saturation counter ID
  input                                 bc_cnt_we_i,     // update saturation counter
  input                           [1:0] bc_cnt_wdat_i,   // new saturation counter value
  input           [GSHARE_BITS_NUM-1:0] bc_cnt_wadr_i,   // saturation counter id
  input                                 bc_hist_taken_i, // conditional branch really taken

  // DU/exception/rfe control transfer
  input                                 ctrl_branch_exception_i,
  input      [OPTION_OPERAND_WIDTH-1:0] ctrl_branch_except_pc_i,

  // to RF read and DECODE
  //  # instruction word valid flag
  output                                fetch_valid_o,
  //  # instruction is in delay slot
  output reg                            fetch_delay_slot_o,
  //  # instruction word itsef
  output         [`OR1K_INSN_WIDTH-1:0] fetch_insn_o,
  //  # operand addresses
  output     [OPTION_RF_ADDR_WIDTH-1:0] fetch_rfa1_adr_o,
  output     [OPTION_RF_ADDR_WIDTH-1:0] fetch_rfb1_adr_o,
  output     [OPTION_RF_ADDR_WIDTH-1:0] fetch_rfa2_adr_o,
  output     [OPTION_RF_ADDR_WIDTH-1:0] fetch_rfb2_adr_o,
  //  # copy #1 of operand addresses
  output     [OPTION_RF_ADDR_WIDTH-1:0] fetch_rfa1_adr_rf_o,
  output     [OPTION_RF_ADDR_WIDTH-1:0] fetch_rfb1_adr_rf_o,
  output     [OPTION_RF_ADDR_WIDTH-1:0] fetch_rfa2_adr_rf_o,
  output     [OPTION_RF_ADDR_WIDTH-1:0] fetch_rfb2_adr_rf_o,
  //  # destinaton addresses
  output     [OPTION_RF_ADDR_WIDTH-1:0] fetch_rfd1_adr_o,
  output     [OPTION_RF_ADDR_WIDTH-1:0] fetch_rfd2_adr_o,

  // Exceptions
  output reg                            fetch_except_ibus_err_o,
  output reg                            fetch_except_itlb_miss_o,
  output reg                            fetch_except_ipagefault_o,
  output reg                            fetch_an_except_o,

  //  Instruction PC
  output     [OPTION_OPERAND_WIDTH-1:0] pc_fetch_o
);

  /*
     Definitions:
       s??r_name - "S"tage number "??", input "r"egisters
       s??t_name - "S"tage number "??", "T"emporary (internally)
  */

  localparam IFOOW = OPTION_OPERAND_WIDTH; // short name


  /**** Stage #1 (IMMU/ICACHE access) registers and wires ****/

  // IMMU's registered input(s)
  reg                 [IFOOW-1:0] s1o_virt_addr; // for comparison in IMMU
  reg                 [IFOOW-1:0] s2o_virt_addr; // i.e. pc_fetch_o
  // IMMU's super-cache statuses
  wire                            s1o_immu_rdy;
  wire                            s1o_immu_upd;
  /* HW reload TLB related (MAROCCHINO_TODO : not implemented yet)
  wire                            tlb_reload_req;
  reg                             tlb_reload_ack;
  wire                [IFOOW-1:0] tlb_reload_addr;
  reg                 [IFOOW-1:0] tlb_reload_data;
  wire                            tlb_reload_pagefault;
  wire                            tlb_reload_busy; */
  // next address latch
  reg                 [IFOOW-1:0] s1r_virt_addr_next;


  /**** Stage #2 (IMMU/ICACHE result latch) registers and wires ****/

  //--- ICACHE related controls and signals ---

  // Not cacheble area -> IBUS access request
  wire                            s2t_ibus_read_req;
  reg                             s2o_ibus_read_req;
  // ICACHE ack
  wire                            s2t_ic_ack;
  // ICACHE data
  wire     [`OR1K_INSN_WIDTH-1:0] s2t_ic_dat;
  reg      [`OR1K_INSN_WIDTH-1:0] s2o_ic_dat;
  reg      [`OR1K_INSN_WIDTH-1:0] s2o_ic_dat_cp; // copy for jump/branch feedback
  reg  [OPTION_RF_ADDR_WIDTH-1:0] s2o_ic_dat_rfa1_adr_rf; // copy for RF access
  reg  [OPTION_RF_ADDR_WIDTH-1:0] s2o_ic_dat_rfb1_adr_rf; // copy for RF access
  // ICACHE requests and performs refill
  wire                            s2t_ic_refill_req;
  reg                             s2o_ic_refill_req;
  wire                            ic_refill_first;
  // ACK from ICACHE or IBUS without exceptions
  reg                             s2o_imem_ack;
  reg                             s2o_imem_ack_cp;  // copy for jump/branch feedback
  reg                             s2o_imem_ack_rf; // copy for RF access

  //--- IBUS access state machine controls ---

  // exceptions
  reg               s2o_immu_an_except; // force IBUS_FSM to move to EXCEPT state
  // ---
  reg               fetch_an_except_cp; // copy for jump/branch feedback
  reg               fetch_an_except_rf; // copy for RF access

  //   IBUS output ready
  // Indicates IBUS ACK for IBUS direct access only
  // (not ACKs for ICACHE refill):
  wire              ibus_ack;
  // IBUS access state machine
  localparam  [3:0] IBUS_IDLE         = 4'b0001,
                    IBUS_READ         = 4'b0010,
                    IBUS_TO_IC_REFILL = 4'b0100,
                    IBUS_IC_REFILL    = 4'b1000;
  // ---
  reg         [3:0] ibus_state;
  // particular states
  wire              ibus_idle_state = ibus_state[0];
  wire              ibus_read_state = ibus_state[1];
  wire              to_refill_state = ibus_state[2];
  wire              ic_refill_state = ibus_state[3];

  // registered IBUS ack and data
  reg                             s2o_ibus_ack;
  reg                             s2o_ibus_ack_cp; // copy for jump/branch feedback
  reg                             s2o_ibus_ack_rf;
  reg      [`OR1K_INSN_WIDTH-1:0] s2o_ibus_dat;
  reg      [`OR1K_INSN_WIDTH-1:0] s2o_ibus_dat_cp; // copy for jump/branch feedback
  reg  [OPTION_RF_ADDR_WIDTH-1:0] s2o_ibus_dat_rfa1_adr_rf;
  reg  [OPTION_RF_ADDR_WIDTH-1:0] s2o_ibus_dat_rfb1_adr_rf;


  /**** SPR BUS transactions support ****/


  //   For MAROCCHINO SPR access means that pipeline is stalled till ACK.
  // So, no padv-*. We only delay SPR access command till IBUS transaction
  // completion.
  wire spr_bus_stb_ifetch = (ibus_idle_state & spr_bus_stb_i);


  /**** IFETCH pipe controls ****/

  // Conditions to stall stage #1
  wire stall_s1 = s1o_immu_upd |                          // stalling stage #1
                  s2o_ic_refill_req | s2o_ibus_read_req | // stalling stage #1
                  fetch_an_except_o;                      // stalling stage #1
  // Advance stage #1
  wire padv_s1  = padv_fetch_i & (~stall_s1); 

  // For advancing both stages simultaneously
  wire s2_en = s1o_immu_rdy &                                // advansing stage #2 is enabled
               (~s2o_ic_refill_req) & (~s2o_ibus_read_req) & // advansing stage #2 is enabled
               (~fetch_an_except_o);                         // advansing stage #2 is enabled
  // Advance stage #2
  wire padv_s2 = padv_fetch_i & s2_en;


  //-----------------------------------------------//
  // Flush logic                                   //
  // - flush registers from pipeline-flush command //
  //   till IBUS transaction completion            //
  //-----------------------------------------------//

  // store flush command till IBUS transactions complete
  reg  flush_r;
  // --- combination of pipeline-flush and flush-r ---
  wire flush_by_ctrl = pipeline_flush_i | flush_r;


  //------------------------------------------------//
  // Mispredict logic                               //
  // - flush registers from prediction miss command //
  //   till IBUS transaction completion             //
  //------------------------------------------------//

  // store prediction miss command till complete IBUS transactions
  reg  predict_miss_r;
  // --- combined flush-by-predict-miss ---
  wire flush_by_predict_miss = predict_miss_i | predict_miss_r;


  /************************************************/
  /* Stage #1: PC update and IMMU / ICACHE access */
  /************************************************/

  // Select the PC for next fetch:
  wire [IFOOW-1:0] virt_addr_mux;
  // ---
  assign virt_addr_mux = do_branch_i ? do_branch_target_i : s1r_virt_addr_next;

  // register next virtual address
  always @(posedge cpu_clk) begin
    if (ctrl_branch_exception_i) begin  // next address to fetch
      s1r_virt_addr_next <= ctrl_branch_except_pc_i;
    end
    else if (padv_s1) begin // next address to fetch
      s1r_virt_addr_next <= virt_addr_mux + 3'd4;
    end
    else if (do_branch_i) begin // next address to fetch
      s1r_virt_addr_next <= do_branch_target_i;
    end
  end // @ clock

  // IMMU match address store register
  always @(posedge cpu_clk) begin
    if (padv_s1)
      s1o_virt_addr <= virt_addr_mux;
  end // @ clock


  //------------------//
  // Instance of IMMU //
  //------------------//

  // IMMU's output wires
  wire [IFOOW-1:0] s2t_phys_addr;
  reg  [IFOOW-1:0] s2o_phys_addr;
  wire             s2t_cache_inhibit;
  wire             s2t_tlb_miss;
  wire             s2t_pagefault;

  // IMMU module
  mor1kx_immu_marocchino
  #(
    .FEATURE_IMMU_HW_TLB_RELOAD     (FEATURE_IMMU_HW_TLB_RELOAD), // IMMU
    .OPTION_OPERAND_WIDTH           (OPTION_OPERAND_WIDTH), // IMMU
    .OPTION_IMMU_SET_WIDTH          (OPTION_IMMU_SET_WIDTH), // IMMU
    .OPTION_IMMU_WAYS               (OPTION_IMMU_WAYS), // IMMU
    .OPTION_IMMU_CLEAR_ON_INIT      (OPTION_IMMU_CLEAR_ON_INIT) // IMMU
  )
  u_immu
  (
    .cpu_clk                        (cpu_clk), // IMMU
    .cpu_rst                        (cpu_rst), // IMMU
    // whole CPU pipe controls
    .pipeline_flush_i               (pipeline_flush_i), // IMMU
    // IFETCH's stages advancing controls
    .predict_miss_i                 (predict_miss_i), // IMMU
    .padv_s1_i                      (padv_s1), // IMMU
    .jr_gathering_target_i          (jr_gathering_target_i), // IMMU
    .s1o_immu_rdy_o                 (s1o_immu_rdy), // IMMU
    .s1o_immu_upd_o                 (s1o_immu_upd), // IMMU
    // configuration
    .immu_enable_i                  (immu_enable_i), // IMMU
    .supervisor_mode_i              (supervisor_mode_i), // IMMU
    // address translation
    .virt_addr_mux_i                (virt_addr_mux), // IMMU
    .virt_addr_s1o_i                (s1o_virt_addr), // IMMU
    .phys_addr_o                    (s2t_phys_addr), // IMMU
    // flags
    .cache_inhibit_o                (s2t_cache_inhibit), // IMMU
    .tlb_miss_o                     (s2t_tlb_miss), // IMMU
    .pagefault_o                    (s2t_pagefault), // IMMU
    // TLB HW reload face. MAROCCHINO_TODO: not implemented
    .tlb_reload_req_o               (), // IMMU
    .tlb_reload_ack_i               (1'b0), // IMMU
    .tlb_reload_addr_o              (), // IMMU
    .tlb_reload_data_i              ({OPTION_OPERAND_WIDTH{1'b0}}), // IMMU
    .tlb_reload_pagefault_o         (), // IMMU
    .tlb_reload_pagefault_clear_i   (1'b0), // IMMU
    .tlb_reload_busy_o              (), // IMMU
    // SPR bus face
    .spr_bus_addr_i                 (spr_bus_addr_i[15:0]), // IMMU
    .spr_bus_we_i                   (spr_bus_we_i), // IMMU
    .spr_bus_stb_i                  (spr_bus_stb_ifetch), // IMMU
    .spr_bus_dat_i                  (spr_bus_dat_i), // IMMU
    .spr_bus_dat_o                  (spr_bus_dat_immu_o), // IMMU
    .spr_bus_ack_o                  (spr_bus_ack_immu_o) // IMMU
  );


  //-------------------//
  // Instance of cache //
  //-------------------//

  // ICACHE module
  mor1kx_icache_marocchino
  #(
    .OPTION_OPERAND_WIDTH         (OPTION_OPERAND_WIDTH), // ICACHE
    .OPTION_ICACHE_BLOCK_WIDTH    (OPTION_ICACHE_BLOCK_WIDTH), // ICACHE
    .OPTION_ICACHE_SET_WIDTH      (OPTION_ICACHE_SET_WIDTH), // ICACHE
    .OPTION_ICACHE_WAYS           (OPTION_ICACHE_WAYS), // ICACHE
    .OPTION_ICACHE_LIMIT_WIDTH    (OPTION_ICACHE_LIMIT_WIDTH), // ICACHE
    .OPTION_ICACHE_CLEAR_ON_INIT  (OPTION_ICACHE_CLEAR_ON_INIT) // ICACHE
  )
  u_icache
  (
    // clock and reset
    .cpu_clk                  (cpu_clk), // ICACHE
    .cpu_rst                  (cpu_rst), // ICACHE
    // pipe controls
    .padv_s1_i                (padv_s1), // ICACHE
    .padv_s2_i                (padv_s2), // ICACHE
    .pipeline_flush_i         (pipeline_flush_i), // ICACHE
    // fetch exceptions
    .ibus_err_i               (ibus_err_i), // ICACHE: cancel re-fill
    // configuration
    .ic_enable_i              (ic_enable_i), // ICACHE
    // regular requests in/out
    .virt_addr_mux_i          (virt_addr_mux), // ICACHE
    .virt_addr_s1o_i          (s1o_virt_addr), // ICACHE
    .virt_addr_s2o_i          (s2o_virt_addr), // ICACHE
    .phys_addr_s2t_i          (s2t_phys_addr), // ICACHE
    .immu_cache_inhibit_i     (s2t_cache_inhibit), // ICACHE
    .ic_ack_o                 (s2t_ic_ack), // ICACHE
    .ic_dat_o                 (s2t_ic_dat), // ICACHE
    // IBUS access request
    .ibus_read_req_o          (s2t_ibus_read_req), // ICACHE
    // re-fill
    .refill_req_o             (s2t_ic_refill_req), // ICACHE
    .to_refill_i              (to_refill_state), // ICACHE
    .ic_refill_first_o        (ic_refill_first), // ICACHE
    .phys_addr_s2o_i          (s2o_phys_addr), // ICACHE
    .ibus_dat_i               (ibus_dat_i), // ICACHE
    .ibus_burst_last_i        (ibus_burst_last_i), // ICACHE
    .ibus_ack_i               (ibus_ack_i), // ICACHE
    // SPR bus
    .spr_bus_addr_i           (spr_bus_addr_i[15:0]), // ICACHE
    .spr_bus_we_i             (spr_bus_we_i), // ICACHE
    .spr_bus_stb_i            (spr_bus_stb_ifetch), // ICACHE
    .spr_bus_dat_i            (spr_bus_dat_i), // ICACHE
    .spr_bus_dat_o            (spr_bus_dat_ic_o), // ICACHE
    .spr_bus_ack_o            (spr_bus_ack_ic_o) // ICACHE
  );


  /****************************************/
  /* Stage #2: ICACHE check / IBUS access */
  /****************************************/


  // IMMU match address store register
  always @(posedge cpu_clk) begin
    if (padv_s2)
      s2o_virt_addr <= s1o_virt_addr;
  end // @ clock

  // registered physical address
  always @(posedge cpu_clk) begin
    if (padv_s2)
      s2o_phys_addr <= s2t_phys_addr;
  end

  //--------------------//
  // IBUS state machine //
  //--------------------//

  // IBUS output ready (no bus error case)
  //  (a) read none-cached area
  //  (b) 1-st data during cache re-fill
  assign ibus_ack = (ibus_read_state | ic_refill_first) & ibus_ack_i;

  // state machine itself
  always @(posedge cpu_clk) begin
    if (cpu_rst) begin
      ibus_req_o    <= 1'b0;       // by reset
      ibus_burst_o  <= 1'b0;       // by reset
      ibus_state    <= IBUS_IDLE;  // by reset
    end
    else begin
      // synthesis parallel_case
      case (ibus_state)
        IBUS_IDLE: begin
          if (spr_bus_stb_i | pipeline_flush_i | predict_miss_i | fetch_an_except_o) // IBUS-IDLE
            ibus_state <= IBUS_IDLE;         // IBUS-IDLE -> flushing / exceptions / etc
          else if (s2o_ic_refill_req) begin  // IBUS-IDLE
            ibus_req_o   <= ~ibus_req_o;       // IBUS-IDLE -> IBUS-TO-IC-REFILL
            ibus_burst_o <= 1'b1;              // IBUS-IDLE -> IBUS-TO-IC-REFILL
            ibus_state   <= IBUS_TO_IC_REFILL; // IBUS-IDLE -> IBUS-TO-IC-REFILL
          end
          else if (s2o_ibus_read_req) begin  // IBUS-IDLE
            ibus_req_o <= ~ibus_req_o;       // IBUS-IDLE -> IBUS read
            ibus_state <= IBUS_READ;         // IBUS-IDLE -> IBUS read
          end
        end

        IBUS_READ: begin
          if (ibus_ack_i | ibus_err_i)  // IBUS read
            ibus_state <= IBUS_IDLE;    // IBUS read complete
        end // read

        IBUS_TO_IC_REFILL: begin
          ibus_state <= IBUS_IC_REFILL; // IBUS-TO-IC-REFILL -> ICACHE re-fill
        end

        IBUS_IC_REFILL: begin
          if (ibus_err_i | (ibus_ack_i & ibus_burst_last_i)) begin // ICACHE re-fill
            ibus_burst_o <= 1'b0;      // IBUS error / last re-fill
            ibus_state   <= IBUS_IDLE; // IBUS error / last re-fill
          end
        end // ic-refill

        default:;
      endcase // case (state)
    end // reset / regular update
  end // @ clock

  // IBUS access machine: read address
  always @(posedge cpu_clk) begin
    if (ibus_idle_state & (s2o_ibus_read_req | s2o_ic_refill_req)) // IBUS-IDLE -> IBUS read / ICACHE re-fill
      ibus_adr_o <= s2o_phys_addr; // IBUS-IDLE -> IBUS read / ICACHE re-fill
  end // @ clock


  // --- flush extender ---
  wire deassert_flush_r = ibus_idle_state                                   | // de-assert flush extender
                          (ibus_read_state & (ibus_ack_i | ibus_err_i))     | // de-assert flush extender
                          (ic_refill_state & ibus_err_i)                    | // de-assert flush extender
                          (ic_refill_state & ibus_ack_i & ibus_burst_last_i); // de-assert flush extender
  // ---
  always @(posedge cpu_clk) begin
    if (cpu_rst)
      flush_r <= 1'b0;          // cpu-rst / pipeline-flush
    else if (deassert_flush_r)
      flush_r <= 1'b0;          // de-assert
    else if (pipeline_flush_i)
      flush_r <= 1'b1;
  end // at clock


  // --- misprediction extender ---
  wire deassert_predict_miss_r = ibus_idle_state                                   | // de-assert misprediction extender
                                 (ibus_read_state & (ibus_ack_i | ibus_err_i))     | // de-assert misprediction extender
                                 (ic_refill_state & ibus_err_i)                    | // de-assert misprediction extender
                                 (ic_refill_state & ibus_ack_i & ibus_burst_last_i); // de-assert misprediction extender
  // ---
  always @(posedge cpu_clk) begin
    if (pipeline_flush_i)
      predict_miss_r <= 1'b0;         // cpu-rst / pipeline-flush
    else if (deassert_predict_miss_r)
      predict_miss_r <= 1'b0;         // de-assert
    else if (predict_miss_i)          // assert predict miss register
      predict_miss_r <= 1'b1;
  end // at clock



  //----------------------------------------------------------//
  // Stage #2 ICAHCE/IBUS result latches and output of IFETCH //
  //----------------------------------------------------------//


  // ACK from ICACHE or IBUS without exceptions
  always @(posedge cpu_clk) begin
    if (flush_by_ctrl | flush_by_predict_miss) begin // drop "s2o-imem-ack"
      s2o_imem_ack    <= 1'b0;         // reset / flush
      s2o_imem_ack_cp <= 1'b0;         // reset / flush
      s2o_imem_ack_rf <= 1'b0;         // reset / flush
    end
    else if (ibus_ack) begin
      s2o_imem_ack    <= 1'b1;         // IBUS ack
      s2o_imem_ack_cp <= 1'b1;         // IBUS ack
      s2o_imem_ack_rf <= 1'b1;         // IBUS ack
    end
    else if (padv_fetch_i) begin
      if (s2_en) begin                 // eq. padv_s2, latch IMEM ACK
        s2o_imem_ack    <= s2t_ic_ack;
        s2o_imem_ack_cp <= s2t_ic_ack;
        s2o_imem_ack_rf <= s2t_ic_ack;
      end
      else begin
        s2o_imem_ack    <= 1'b0;       // no new insn/except at pipe advancing
        s2o_imem_ack_cp <= 1'b0;       // no new insn/except at pipe advancing
        s2o_imem_ack_rf <= 1'b0;       // no new insn/except at pipe advancing
      end
    end
  end // @ clock


  // MMU's exception with fetch request validity flag
  wire s2t_immu_an_except = (s2t_tlb_miss | s2t_pagefault);


  // Exceptions: IBUS error and combined
  always @(posedge cpu_clk) begin
    if (flush_by_ctrl | flush_by_predict_miss) begin // drop "IBUS error" and "fetch-an-except"
      fetch_except_ibus_err_o <= 1'b0;  // reset / flush
      fetch_an_except_o       <= 1'b0;  // reset / flush
      fetch_an_except_cp      <= 1'b0;  // reset / flush
      fetch_an_except_rf      <= 1'b0;  // reset / flush
    end
    else if (ibus_err_i) begin
      fetch_except_ibus_err_o <= 1'b1;  // IBUS error
      fetch_an_except_o       <= 1'b1;  // IBUS error
      fetch_an_except_cp      <= 1'b1;  // IBUS error
      fetch_an_except_rf      <= 1'b1;  // IBUS error
    end
    else if (padv_s2) begin           // latch "an except" and "IBUS err"
      fetch_except_ibus_err_o <= 1'b0;  // s1/s2 advancing
      fetch_an_except_o       <= s2t_immu_an_except; // s1/s2 advancing
      fetch_an_except_cp      <= s2t_immu_an_except; // s1/s2 advancing
      fetch_an_except_rf      <= s2t_immu_an_except; // s1/s2 advancing
    end
  end // @ clock


  // Exceptions: IMMU related
  always @(posedge cpu_clk) begin
    if (flush_by_ctrl | flush_by_predict_miss) begin // drop "IMMU" exceptions
      // separate exceptions
      fetch_except_itlb_miss_o  <= 1'b0;  // reset / flush
      fetch_except_ipagefault_o <= 1'b0;  // reset / flush
      // IMMU combined exception flag
      s2o_immu_an_except        <= 1'b0;  // reset / flush
    end
    else if (padv_s2) begin           // latch IMMU related flags
      // separate exceptions
      fetch_except_itlb_miss_o  <= s2t_tlb_miss; // s1/s2 advancing
      fetch_except_ipagefault_o <= s2t_pagefault; // s1/s2 advancing
      // IMMU combined exception flag
      s2o_immu_an_except        <= s2t_immu_an_except; // s1/s2 advancing
    end
  end // @ clock


  // IFETCH output valid: istruction or exception
  assign fetch_valid_o = s2o_imem_ack | fetch_an_except_o;


  // --- ICACHE re-fill request ---
  wire deassert_s2o_ic_refill_req =
    to_refill_state ? 1'b0 : // de-assert re-fill request
      (ic_refill_state ? (ibus_err_i | (ibus_ack_i & ibus_burst_last_i)) : // de-assert re-fill request
                         (pipeline_flush_i | predict_miss_i | s2o_immu_an_except)); // de-assert re-fill request
  // ---
  always @(posedge cpu_clk) begin
    if (cpu_rst)
      s2o_ic_refill_req <= 1'b0;  // reset / flush
    else if (deassert_s2o_ic_refill_req)
      s2o_ic_refill_req <= 1'b0;  // de-assert
    else if (padv_s2)
      s2o_ic_refill_req <= s2t_ic_refill_req;
  end // @ clock
  // --- ICACHE data ---
  always @(posedge cpu_clk) begin
    if (padv_s2) begin
      s2o_ic_dat             <= s2t_ic_dat;
      s2o_ic_dat_cp          <= s2t_ic_dat;
      s2o_ic_dat_rfa1_adr_rf <= s2t_ic_dat[`OR1K_RA_SELECT];
      s2o_ic_dat_rfb1_adr_rf <= s2t_ic_dat[`OR1K_RB_SELECT];
    end
  end // @ clock


  // --- IBUS read request ---
  wire deassert_s2o_ibus_read_req =
    ibus_read_state ? (ibus_err_i | ibus_ack_i) : // de-assert IBUS request
                      (pipeline_flush_i | predict_miss_i | s2o_immu_an_except); // de-assert IBUS request
  // ---
  always @(posedge cpu_clk) begin
    if (cpu_rst)
      s2o_ibus_read_req <= 1'b0;  // reset / flush
    else if (deassert_s2o_ibus_read_req)
      s2o_ibus_read_req <= 1'b0;  // de-assert
    else if (padv_s2)
      s2o_ibus_read_req <= s2t_ibus_read_req;
  end // @ clock
  // --- IBUS ack ---
  always @(posedge cpu_clk) begin
    if (flush_by_ctrl | flush_by_predict_miss) begin // drop "s2o-ibus-ack"
      s2o_ibus_ack    <= 1'b0; // reset / flush
      s2o_ibus_ack_cp <= 1'b0; // reset / flush
      s2o_ibus_ack_rf <= 1'b0; // reset / flush
    end
    else if (ibus_ack) begin
      s2o_ibus_ack    <= 1'b1; // IBUS ack
      s2o_ibus_ack_cp <= 1'b1; // IBUS ack
      s2o_ibus_ack_rf <= 1'b1; // IBUS ack
    end
    else if (padv_fetch_i) begin
      s2o_ibus_ack    <= 1'b0; // no new insn/except at pipe advancing
      s2o_ibus_ack_cp <= 1'b0; // no new insn/except at pipe advancing
      s2o_ibus_ack_rf <= 1'b0; // no new insn/except at pipe advancing
    end
  end // @ clock
  // --- IBUS data ---
  always @(posedge cpu_clk) begin
    if (ibus_ack) begin
      s2o_ibus_dat             <= ibus_dat_i;
      s2o_ibus_dat_cp          <= ibus_dat_i;
      s2o_ibus_dat_rfa1_adr_rf <= ibus_dat_i[`OR1K_RA_SELECT];
      s2o_ibus_dat_rfb1_adr_rf <= ibus_dat_i[`OR1K_RB_SELECT];
    end
  end // @ clock


  // insrtuction word mux
  wire s3t_miss_or_except = (~s2o_imem_ack) | fetch_an_except_o;
  // ---
  assign fetch_insn_o = s3t_miss_or_except ? {`OR1K_OPCODE_CUST8,26'd0} :
                          (s2o_ibus_ack ? s2o_ibus_dat : s2o_ic_dat);
  // operand addresses
  assign fetch_rfa1_adr_o = fetch_insn_o[`OR1K_RA_SELECT];
  assign fetch_rfb1_adr_o = fetch_insn_o[`OR1K_RB_SELECT];
  assign fetch_rfa2_adr_o = fetch_insn_o[`OR1K_RA_SELECT] + 1'b1;
  assign fetch_rfb2_adr_o = fetch_insn_o[`OR1K_RB_SELECT] + 1'b1;
  // destinaton addresses
  assign fetch_rfd1_adr_o = fetch_insn_o[`OR1K_RD_SELECT];
  assign fetch_rfd2_adr_o = fetch_insn_o[`OR1K_RD_SELECT] + 1'b1;


  // copy of insrtuction word mux for Jump/Branch feedback
  wire [`OR1K_INSN_WIDTH-1:0] fetch_insn_cp;
  // ---
  wire s3t_miss_or_except_cp = (~s2o_imem_ack_cp) | fetch_an_except_cp;
  // ---
  assign fetch_insn_cp = s3t_miss_or_except_cp ? {`OR1K_OPCODE_CUST8,26'd0} :
                           (s2o_ibus_ack_cp ? s2o_ibus_dat_cp : s2o_ic_dat_cp);
  // Jump/Branch processing
  wire [`OR1K_OPCODE_WIDTH-1:0] opc_insn = fetch_insn_cp[`OR1K_OPCODE_SELECT];
  //  # jump/branch variants
  assign fetch_op_jimm_o  = (opc_insn == `OR1K_OPCODE_J)  | (opc_insn == `OR1K_OPCODE_JAL);
  assign fetch_op_jr_o    = (opc_insn == `OR1K_OPCODE_JR) | (opc_insn == `OR1K_OPCODE_JALR);
  assign fetch_op_bf_o    = (opc_insn == `OR1K_OPCODE_BF);
  assign fetch_op_bnf_o   = (opc_insn == `OR1K_OPCODE_BNF);
  //  # combined jump/branch flag
  assign fetch_op_jb_o    = fetch_op_jimm_o | fetch_op_jr_o | fetch_op_bf_o | fetch_op_bnf_o;
  //  # "to immediate driven target"
  assign fetch_to_imm_target_o = pc_fetch_o + {{4{fetch_insn_cp[25]}},fetch_insn_cp[25:0],2'b00};


  // copy #1 of rfxx_adr
  wire s3t_miss_or_except_rf = (~s2o_imem_ack_rf) | fetch_an_except_rf;
  // ---
  assign fetch_rfa1_adr_rf_o = s3t_miss_or_except_rf ? {OPTION_RF_ADDR_WIDTH{1'b0}} :
                                  (s2o_ibus_ack_rf ? s2o_ibus_dat_rfa1_adr_rf : s2o_ic_dat_rfa1_adr_rf);
  assign fetch_rfb1_adr_rf_o = s3t_miss_or_except_rf ? {OPTION_RF_ADDR_WIDTH{1'b0}} :
                                  (s2o_ibus_ack_rf ? s2o_ibus_dat_rfb1_adr_rf : s2o_ic_dat_rfb1_adr_rf);
  assign fetch_rfa2_adr_rf_o = fetch_rfa1_adr_rf_o + 1'b1;
  assign fetch_rfb2_adr_rf_o = fetch_rfb1_adr_rf_o + 1'b1;


  // delay slot flag
  always @(posedge cpu_clk) begin
    if (flush_by_ctrl | flush_by_predict_miss) // drop "fetch-delay-slot"
      fetch_delay_slot_o <= 1'b0; // reset / flush
    else if (padv_fetch_i)
      fetch_delay_slot_o <= fetch_valid_o ? fetch_op_jb_o : fetch_delay_slot_o;
  end // @ clock


  // PC
  assign pc_fetch_o = s2o_virt_addr;

  // Address after delay slot (valid if fetch_op_jb_o)
  assign after_ds_target_o = s2o_virt_addr + 4'd8; // (FEATURE_DSX == "ENABLED")


  //-----------------------------------//
  // Slobal set of saturation counters //
  //-----------------------------------//

  // --- stage #1 read current counter ---

  wire [GSHARE_BITS_NUM-1:0] s1t_bc_cnt_radr;
  reg  [GSHARE_BITS_NUM-1:0] s1o_bc_cnt_radr;

  // Global history buffer
  reg  [GSHARE_BITS_NUM-1:0] bc_hist_taken_r;
  // ---
  always @(posedge cpu_clk) begin
    if (cpu_rst)
      bc_hist_taken_r <= {GSHARE_BITS_NUM{1'b0}};
    else if (bc_cnt_we_i)
      bc_hist_taken_r <= {bc_hist_taken_r[GSHARE_BITS_NUM-2:0], bc_hist_taken_i};
  end // at clock


  // current read address
  assign s1t_bc_cnt_radr = virt_addr_mux[GSHARE_BITS_NUM+1:2] ^ bc_hist_taken_r;
  // ---
  always @(posedge cpu_clk) begin
    if (padv_s1)
      s1o_bc_cnt_radr <= s1t_bc_cnt_radr;
  end // at clock

  // saturatin counter read address
  wire [GSHARE_BITS_NUM-1:0] bc_cnt_radr;
  assign bc_cnt_radr = (padv_s1 ? s1t_bc_cnt_radr : s1o_bc_cnt_radr);

  // Read/Write port (*_rwp_*) write
  // !!! In short loop it is possible simultaneous
  // !!! update and reading the same counter
  wire bc_cnt_rwp_we = bc_cnt_we_i & (bc_cnt_wadr_i == bc_cnt_radr); // BC-CNT-RWP-WE
  wire bc_cnt_rwp_en = padv_s1 | bc_cnt_rwp_we;

  // Write-only port (*_wp_*) enable
  wire bc_cnt_wp_en = bc_cnt_we_i & (bc_cnt_wadr_i != bc_cnt_radr); // BC-CNT-WP-WE

  // saturation counter read result
  wire [1:0] s2t_bc_cnt_value;

  // Saturation counters RAM
  mor1kx_dpram_en_w1st_sclk
  #(
    .ADDR_WIDTH     (GSHARE_BITS_NUM),
    .DATA_WIDTH     (2),
    .CLEAR_ON_INIT  (1)
  )
  u_bc_cnt_ram
  (
    // common clock
    .clk    (cpu_clk),
    // port "a"
    .en_a   (bc_cnt_rwp_en),
    .we_a   (bc_cnt_rwp_we),
    .addr_a (bc_cnt_radr),
    .din_a  (bc_cnt_wdat_i),
    .dout_a (s2t_bc_cnt_value),
    // port "b"
    .en_b   (bc_cnt_wp_en),
    .we_b   (1'b1),
    .addr_b (bc_cnt_wadr_i),
    .din_b  (bc_cnt_wdat_i),
    .dout_b ()
  );


  // --- stage #2 latch current counter ---

  // output saturation counter value
  reg    [1:0] s2o_bc_cnt_value;
  assign       bc_cnt_value_o = s2o_bc_cnt_value;
  // ---
  always @(posedge cpu_clk) begin
    if (padv_s2)
      s2o_bc_cnt_value <= s2t_bc_cnt_value;
  end

  // output saturation counter ID
  reg    [GSHARE_BITS_NUM-1:0] s2o_bc_cnt_radr;
  assign                       bc_cnt_radr_o = s2o_bc_cnt_radr;
  // ---
  always @(posedge cpu_clk) begin
    if (padv_s2)
      s2o_bc_cnt_radr <= s1o_bc_cnt_radr;
  end // at clock

endmodule // mor1kx_fetch_marocchino
