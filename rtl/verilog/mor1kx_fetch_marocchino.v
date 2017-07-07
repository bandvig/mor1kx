////////////////////////////////////////////////////////////////////////
//                                                                    //
//  mor1kx_icache_marocchino                                          //
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
  parameter OPTION_RESET_PC             = {{(OPTION_OPERAND_WIDTH-13){1'b0}},
                                           `OR1K_RESET_VECTOR,8'd0},
  parameter OPTION_RF_ADDR_WIDTH        =  5,
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
  input      [OPTION_OPERAND_WIDTH-1:0] ibus_burst_adr_i,
  input                                 ibus_burst_last_i,
  output reg                            ibus_req_o,
  output reg [OPTION_OPERAND_WIDTH-1:0] ibus_adr_o,
  output                                ibus_burst_o,

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
  //  ## do branch (pedicted or unconditional)
  input                                 do_branch_i,
  input      [OPTION_OPERAND_WIDTH-1:0] do_branch_target_i,
  input                                 fetch_jr_bc_hazard_i,

  // DU/exception/rfe control transfer
  input                                 ctrl_branch_exception_i,
  input      [OPTION_OPERAND_WIDTH-1:0] ctrl_branch_except_pc_i,

  // to RF read and DECODE
  //  # instruction word valid flag
  output reg                            fetch_insn_valid_o,
  //  # instruction is in delay slot
  output reg                            fetch_delay_slot_o,
  //  # instruction word itsef
  output         [`OR1K_INSN_WIDTH-1:0] fetch_insn_o,
  //  # operand addresses
  output     [OPTION_RF_ADDR_WIDTH-1:0] fetch_rfa1_adr_o,
  output     [OPTION_RF_ADDR_WIDTH-1:0] fetch_rfb1_adr_o,
  output     [OPTION_RF_ADDR_WIDTH-1:0] fetch_rfa2_adr_o,
  output     [OPTION_RF_ADDR_WIDTH-1:0] fetch_rfb2_adr_o,
  //  # D2 address
  output     [OPTION_RF_ADDR_WIDTH-1:0] fetch_rfd2_adr_o,

  // Exceptions
  output reg                            fetch_except_ibus_err_o,
  output reg                            fetch_except_itlb_miss_o,
  output reg                            fetch_except_ipagefault_o,
  output reg                            fetch_an_except_o,
  output reg                            fetch_exception_taken_o,

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

  // Stage #1 access is valid (regular address update or valid branching)
  //   To minimize logic for control padv-s1s2 computation we don't
  // stop IFETCH stages #1 and #2 even so jump/branch target
  // is not ready. So we rise "hit" if virtual address is valid.
  reg                             s1o_fetch_req_hit;
  reg                             s2o_fetch_req_hit;
  // IMMU's registered input(s)
  reg                 [IFOOW-1:0] s1o_virt_addr; // for comparison in IMMU
  reg                 [IFOOW-1:0] s2o_virt_addr; // i.e. pc_fetch_o 
  /* HW reload TLB related (MAROCCHINO_TODO : not implemented yet)
  wire                            tlb_reload_req;
  reg                             tlb_reload_ack;
  wire                [IFOOW-1:0] tlb_reload_addr;
  reg                 [IFOOW-1:0] tlb_reload_data;
  wire                            tlb_reload_pagefault;
  wire                            tlb_reload_busy; */


  /**** Stage #2 (IMMU/ICACHE result latch) registers and wires ****/

  //--- ICACHE related controls and signals ---

  // Not cacheble area -> IBUS access request
  wire                            s2t_ibus_read_req;
  reg                             s2o_ibus_read_req;
  // ICACHE ack
  wire                            s2t_ic_ack;
  reg                             s2o_ic_ack;
  // ICACHE data
  wire     [`OR1K_INSN_WIDTH-1:0] s2t_ic_dat;
  reg      [`OR1K_INSN_WIDTH-1:0] s2o_ic_dat;
  // ICACHE requests and performs refill
  wire                            s2t_ic_refill_req;
  reg                             s2o_ic_refill_req;
  wire                            ic_refill_first;

  //--- IBUS access state machine controls ---

  // exceptions
  reg               s2o_immu_an_except; // force IBUS_FSM to move to EXCEPT state
  // miss or exception for multiplexing ICACHE/IBUS data
  reg               s2o_miss_or_except; // allow moving IBUS_FSM for at least one time

  //   IBUS output ready
  // Indicates IBUS ACK for IBUS direct access only
  // (not ACKs for ICACHE refill):
  wire              ibus_ack;
  // IBUS FSM statuses
  wire              ibus_fsm_free;
  // IBUS access state machine
  localparam  [5:0] IBUS_IDLE         = 6'b000001,
                    IBUS_READ         = 6'b000010,
                    IBUS_TO_IC_REFILL = 6'b000100,
                    IBUS_IC_REFILL    = 6'b001000,
                    IBUS_IC_REREAD    = 6'b010000,
                    IBUS_AN_EXCEPT    = 6'b100000;
  // ---
  reg         [5:0] ibus_state;
  // particular states
  wire              ibus_idle_state = ibus_state[0];
  wire              ibus_read_state = ibus_state[1];
  wire              to_refill_state = ibus_state[2];
  wire              ic_refill_state = ibus_state[3];

  // registered IBUS ack and data
  reg                        s2o_ibus_ack;
  reg [`OR1K_INSN_WIDTH-1:0] s2o_ibus_dat;

  /**** SPR BUS transactions support ****/


  //   For MAROCCHINO SPR access means that pipeline is stalled till ACK.
  // So, no padv-*. We only delay SPR access command till IBUS transaction
  // completion.
  wire spr_bus_stb_ifetch = (ibus_idle_state & spr_bus_stb_i);


  /**** IFETCH pipe controls ****/


  // Advance stage #1 and #2 simultaneously
  wire padv_s1s2 = padv_fetch_i & ibus_fsm_free;


  //-----------------------------------------------//
  // Flush logic                                   //
  // - flush registers from pipeline-flush command //
  //   till IBUS transaction completion            //
  //-----------------------------------------------//

  // store flush command till IBUS transactions complete
  reg flush_r;
  // initial value of flush-r for simulations
 `ifndef SYNTHESIS
  // synthesis translate_off
  initial flush_r = 1'b0;
  // synthesis translate_on
 `endif // !synth
  // ---
  always @(posedge cpu_clk) begin
    if (cpu_rst)
      flush_r <= 1'b0;
    else if (ibus_fsm_free)
      flush_r <= 1'b0;
    else if (~flush_r)
      flush_r <= pipeline_flush_i;
  end // @ clock
  // --- combination of pipeline-flush and flush-r ---
  wire flush_by_ctrl = pipeline_flush_i | flush_r;


  /************************************************/
  /* Stage #1: PC update and IMMU / ICACHE access */
  /************************************************/


  // 1-clock fetch-exception-taken
  always @(posedge cpu_clk) begin
    if (cpu_rst | flush_by_ctrl)
      fetch_exception_taken_o <= 1'b0;
    else if (fetch_exception_taken_o)
      fetch_exception_taken_o <= 1'b0;
    else if (padv_s1s2)
      fetch_exception_taken_o <= ctrl_branch_exception_i;
  end // @ clock
  // ---
  wire do_except = ctrl_branch_exception_i & (~fetch_exception_taken_o);


  // store branch flag and target if stage #1 is busy
  reg             do_branch_p;
  reg [IFOOW-1:0] do_branch_target_p;
  // ---
  always @(posedge cpu_clk) begin
    if (cpu_rst | flush_by_ctrl) begin
      do_branch_p <= 1'b0;  // reset / flushing
    end
    else if (padv_s1s2) begin // clean up / keep stored branch
      do_branch_p <= 1'b0; // regular advance stages #1 and #2
    end
    else if (~do_branch_p) begin
      do_branch_p        <= do_branch_i & (~fetch_jr_bc_hazard_i); // if IFETCH's stage #1 stalled
      do_branch_target_p <= do_branch_target_i;                    // if IFETCH's stage #1 stalled
    end
  end // @ clock


  // Select the PC for next fetch:
  wire [IFOOW-1:0] virt_addr_mux;
  //                     use DU/exceptions/rfe provided address
  assign virt_addr_mux = do_except            ? ctrl_branch_except_pc_i :
  //                     regular update of IFETCH
                         do_branch_p          ? do_branch_target_p      :
                         fetch_jr_bc_hazard_i ? s1o_virt_addr           :
                         do_branch_i          ? do_branch_target_i      :
                                                (s1o_virt_addr + 3'd4);   // next PC if no branch


  // IMMU match address store register
  always @(posedge cpu_clk) begin
    if (cpu_rst) begin
      s1o_virt_addr <= OPTION_RESET_PC - 4; // reset: will be restored on 1st advance
      s2o_virt_addr <= {IFOOW{1'b0}};
    end
    else if (padv_s1s2) begin
      s1o_virt_addr <= virt_addr_mux;
      s2o_virt_addr <= s1o_virt_addr;
    end
  end // @ clock


  // new fetch request is valid
  always @(posedge cpu_clk) begin
    if (cpu_rst | flush_by_ctrl)
      s1o_fetch_req_hit <= 1'b0;
    else if (padv_s1s2)
      s1o_fetch_req_hit <= ~fetch_jr_bc_hazard_i;
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
    .OPTION_RESET_PC                (OPTION_RESET_PC), // IMMU
    .OPTION_IMMU_SET_WIDTH          (OPTION_IMMU_SET_WIDTH), // IMMU
    .OPTION_IMMU_WAYS               (OPTION_IMMU_WAYS), // IMMU
    .OPTION_IMMU_CLEAR_ON_INIT      (OPTION_IMMU_CLEAR_ON_INIT) // IMMU
  )
  u_immu
  (
    .cpu_clk                        (cpu_clk), // IMMU
    .cpu_rst                        (cpu_rst), // IMMU
    // controls
    .padv_immu_i                    (padv_s1s2), // IMMU
    .flush_by_ctrl_i                (flush_by_ctrl), // IMMU
    // configuration
    .enable_i                       (immu_enable_i), // IMMU
    .supervisor_mode_i              (supervisor_mode_i), // IMMU
    // address translation
    .virt_addr_mux_i                (virt_addr_mux), // IMMU
    .virt_addr_tag_i                (s1o_virt_addr), // IMMU
    .fetch_req_hit_i                (s1o_fetch_req_hit), // IMMU: enables IMMU's exceptions
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
    .cpu_clk              (cpu_clk), // ICACHE
    .cpu_rst              (cpu_rst), // ICACHE
    // pipe controls
    .padv_s1s2_i          (padv_s1s2), // ICACHE
    .flush_by_ctrl_i      (flush_by_ctrl), // ICACHE
    // fetch exceptions
    .immu_an_except_i     (s2o_immu_an_except), // ICACHE
    .ibus_err_i           (ibus_err_i), // ICACHE: cancel re-fill
    // configuration
    .ic_enable_i          (ic_enable_i), // ICACHE
    // regular requests in/out
    .virt_addr_mux_i      (virt_addr_mux), // ICACHE
    .virt_addr_cmd_i      (s1o_virt_addr), // ICACHE
    .virt_addr_lru_i      (s2o_virt_addr), // ICACHE: for update LRU info
    .phys_addr_tag_i      (s2t_phys_addr), // ICACHE
    .fetch_req_hit_i      (s1o_fetch_req_hit), // ICACHE: enables ICACHE's ACK
    .immu_cache_inhibit_i (s2t_cache_inhibit), // ICACHE
    .ic_ack_o             (s2t_ic_ack), // ICACHE
    .ic_dat_o             (s2t_ic_dat), // ICACHE
    // IBUS access request
    .ibus_read_req_o      (s2t_ibus_read_req), // ICACHE
    // re-fill
    .refill_req_o         (s2t_ic_refill_req), // ICACHE
    .to_refill_i          (to_refill_state), // ICACHE
    .ic_refill_first_o    (ic_refill_first), // ICACHE
    .ibus_dat_i           (ibus_dat_i), // ICACHE
    .ibus_burst_adr_i     (ibus_burst_adr_i), // ICACHE
    .ibus_burst_last_i    (ibus_burst_last_i), // ICACHE
    .ibus_ack_i           (ibus_ack_i), // ICACHE
    // SPR bus
    .spr_bus_addr_i       (spr_bus_addr_i[15:0]), // ICACHE
    .spr_bus_we_i         (spr_bus_we_i), // ICACHE
    .spr_bus_stb_i        (spr_bus_stb_ifetch), // ICACHE
    .spr_bus_dat_i        (spr_bus_dat_i), // ICACHE
    .spr_bus_dat_o        (spr_bus_dat_ic_o), // ICACHE
    .spr_bus_ack_o        (spr_bus_ack_ic_o) // ICACHE
  );


  /****************************************/
  /* Stage #2: ICACHE check / IBUS access */
  /****************************************/


  // registered physical address
  always @(posedge cpu_clk) begin
    if (padv_s1s2)
      s2o_phys_addr <= s2t_phys_addr;
  end

  //--------------------//
  // IBUS state machine //
  //--------------------//

  // IBUS FSM is free to process next request
  //  (1) It follows appropriate FSM condtions, but without taking into account exceptions
  //  (2) If s2o_fetch_req_hit is rized that means either s2o_ic_ack or s2o_ic_refill_req
  //      or s2o_ibus_read_req is rized too
  assign ibus_fsm_free = ibus_idle_state & ((~s2o_fetch_req_hit) | s2o_ic_ack | s2o_ibus_ack);

  // IBUS output ready (no bus error case)
  //  (a) read none-cached area
  //  (b) 1-st data during cache re-fill
  assign ibus_ack = (ibus_read_state | ic_refill_first) & ibus_ack_i;

  // state machine itself
  always @(posedge cpu_clk) begin
    if (cpu_rst) begin
      ibus_req_o <= 1'b0;           // by reset
      ibus_state <= IBUS_IDLE;      // by reset
    end
    else begin
      // synthesis parallel_case full_case
      case (ibus_state)
        IBUS_IDLE: begin
          if (spr_bus_stb_i | flush_by_ctrl)
            ibus_state <= IBUS_IDLE;         // IBUS-IDLE -> keep by flushing
          else if (s2o_immu_an_except)
            ibus_state <= IBUS_AN_EXCEPT;    // IBUS-IDLE -> IBUS-AN-EXCEPT by an IMMU exception
          else if (s2o_ic_refill_req)
            ibus_state <= IBUS_TO_IC_REFILL; // IBUS-IDLE -> IBUS-TO-IC-REFILL
          else if (s2o_ibus_read_req) begin
            ibus_req_o <= ~ibus_req_o;       // IBUS-IDLE -> IBUS read
            ibus_adr_o <= s2o_phys_addr;     // IBUS-IDLE -> IBUS read
            ibus_state <= IBUS_READ;         // IBUS-IDLE -> IBUS read
          end
        end

        IBUS_READ: begin
          if (ibus_err_i)             // IBUS read
            ibus_state <= flush_by_ctrl ? IBUS_IDLE : IBUS_AN_EXCEPT; // IBUS read error
          else if (ibus_ack_i)
            ibus_state <= IBUS_IDLE;  // IBUS read complete
        end // read

        IBUS_TO_IC_REFILL: begin          // to_refill_state
          if (flush_by_ctrl)
            ibus_state <= IBUS_IDLE;      // IBUS-TO-IC-REFILL -> IDLE by flush
          else begin
            ibus_req_o <= ~ibus_req_o;    // IBUS-TO-IC-REFILL -> ICACHE refill
            ibus_adr_o <= s2o_phys_addr;  // IBUS-TO-IC-REFILL -> ICACHE refill
            ibus_state <= IBUS_IC_REFILL; // IBUS-TO-IC-REFILL -> ICACHE refill
          end
        end

        IBUS_IC_REFILL: begin
          if (ibus_err_i)                          // ICACHE refill
            ibus_state <= flush_by_ctrl ? IBUS_IDLE : IBUS_AN_EXCEPT; // IBUS error during ICACHE refill
          else if (ibus_ack_i & ibus_burst_last_i) // ICACHE refill
            ibus_state <= flush_by_ctrl ? IBUS_IDLE : IBUS_IC_REREAD; // last refill
        end // ic-refill

        IBUS_IC_REREAD: begin
          ibus_state <= IBUS_IDLE; // re-read after re-fill
        end

        IBUS_AN_EXCEPT: begin
          if (flush_by_ctrl)
            ibus_state <= IBUS_IDLE; // IBUS ERR -> IDLE by flushing
        end

        default: begin
          ibus_state <= IBUS_IDLE;      // default
        end
      endcase // case (state)
    end // reset / regular update
  end // @ clock

  // And burst mode
  assign ibus_burst_o = ic_refill_state;


  //----------------------------------------------------------//
  // Stage #2 ICAHCE/IBUS result latches and output of IFETCH //
  //----------------------------------------------------------//

  // MMU's exception with fetch request validity flag
  wire s2t_immu_an_except = (s2t_tlb_miss | s2t_pagefault);

  // instruction request is meaningfull
  always @(posedge cpu_clk) begin
    if (cpu_rst | flush_by_ctrl)
      s2o_fetch_req_hit <= 1'b0;    // reset / flush
    else if (padv_fetch_i) begin
      if (ibus_fsm_free)            // eq. padv_s1s2
        s2o_fetch_req_hit <= s1o_fetch_req_hit | s2t_immu_an_except; // block S1/S2 advancing by an IMMU's exception
      else
        s2o_fetch_req_hit <= 1'b0;  // no new insn/except at pipe advancing
    end
  end // @clock

  // (1) output is an instruction or an exception
  // (2) miss or exception for multiplexing instruction word
  always @(posedge cpu_clk) begin
    if (cpu_rst | flush_by_ctrl) begin
      fetch_insn_valid_o <= 1'b0;   // reset / flush
      s2o_miss_or_except <= 1'b1;   // reset / flush
    end
    else if (ibus_ack | ibus_err_i) begin
      fetch_insn_valid_o <= 1'b1;       // IBUS ack or error
      s2o_miss_or_except <= ibus_err_i; // IBUS ack or error
    end
    else if (padv_fetch_i) begin
      if (ibus_fsm_free) begin            // eq. padv_s1s2
        fetch_insn_valid_o <=   s2t_ic_ack  | s2t_immu_an_except;
        s2o_miss_or_except <= (~s2t_ic_ack) | s2t_immu_an_except;
      end
      else begin
        fetch_insn_valid_o <= 1'b0; // no new insn/except at pipe advancing
        s2o_miss_or_except <= 1'b1; // no new insn/except at pipe advancing
      end
    end
  end // @ clock


  // Exceptions
  always @(posedge cpu_clk) begin
    if (cpu_rst | flush_by_ctrl) begin
      // separate exceptions
      fetch_except_ibus_err_o   <= 1'b0;  // reset / flush
      fetch_except_itlb_miss_o  <= 1'b0;  // reset / flush
      fetch_except_ipagefault_o <= 1'b0;  // reset / flush
      // IMMU combined exception flag
      s2o_immu_an_except        <= 1'b0;  // reset / flush
      // Overall combined exception flag
      fetch_an_except_o         <= 1'b0;  // reset / flush
    end
    else if (ibus_err_i) begin
      fetch_except_ibus_err_o   <= 1'b1;  // reset / flush
      fetch_an_except_o         <= 1'b1;  // reset / flush
    end
    else if (padv_s1s2) begin
      // separate exceptions
      fetch_except_itlb_miss_o  <= s2t_tlb_miss;
      fetch_except_ipagefault_o <= s2t_pagefault;
      // IMMU combined exception flag
      s2o_immu_an_except        <= s2t_immu_an_except;
      // Overall combined exception flag
      fetch_an_except_o         <= s2t_immu_an_except;
    end
  end // @ clock


  // --- ICACHE re-fill request ---
  always @(posedge cpu_clk) begin
    if (cpu_rst | flush_by_ctrl)
      s2o_ic_refill_req <= 1'b0;  // reset / flush
    else if (to_refill_state)
      s2o_ic_refill_req <= 1'b0;  // IBUS FSM is going to re-fill
    else if (padv_s1s2)
      s2o_ic_refill_req <= s2t_ic_refill_req & (~s2t_immu_an_except);
  end // @ clock
  // --- ICACHE ack ---
  always @(posedge cpu_clk) begin
    if (cpu_rst | flush_by_ctrl)
      s2o_ic_ack <= 1'b0;  // reset / flush
    else if (padv_fetch_i) begin
      if (ibus_fsm_free)    // eq. padv_s1s2
        s2o_ic_ack <= s2t_ic_ack & (~s2t_immu_an_except);
      else
        s2o_ic_ack <= 1'b0; // no new insn/except at pipe advancing
    end
  end // @ clock  
  // --- ICACHE data ---
  always @(posedge cpu_clk) begin
    if (padv_s1s2)
      s2o_ic_dat <= s2t_ic_dat;
  end // @ clock


  // --- IBUS read request ---
  always @(posedge cpu_clk) begin
    if (cpu_rst | flush_by_ctrl)
      s2o_ibus_read_req <= 1'b0;  // reset / flush
    else if (ibus_read_state)
      s2o_ibus_read_req <= 1'b0;  // IBUS-FSM is reading
    else if (padv_s1s2)
      s2o_ibus_read_req <= s2t_ibus_read_req & (~s2t_immu_an_except);
  end // @ clock
  // --- IBUS ack ---
  always @(posedge cpu_clk) begin
    if (cpu_rst | flush_by_ctrl)
      s2o_ibus_ack <= 1'b0;     // reset / flush
    else if (ibus_ack)
      s2o_ibus_ack <= 1'b1;     // IBUS ack
    else if (padv_fetch_i)
      s2o_ibus_ack <= 1'b0;     // no new insn/except at pipe advancing
  end // @ clock
  // --- IBUS data ---
  always @(posedge cpu_clk) begin
    if (ibus_ack)
      s2o_ibus_dat <= ibus_dat_i;
  end // @ clock


  // insrtuction word mux
  assign fetch_insn_o = s2o_miss_or_except ? {`OR1K_OPCODE_NOP,26'd0} :
                        s2o_ibus_ack       ? s2o_ibus_dat             :
                                             s2o_ic_dat;

  // to RF read
  assign fetch_rfa1_adr_o = fetch_insn_o[`OR1K_RA_SELECT];
  assign fetch_rfb1_adr_o = fetch_insn_o[`OR1K_RB_SELECT];
  assign fetch_rfa2_adr_o = fetch_insn_o[`OR1K_RA_SELECT] + 1'b1;
  assign fetch_rfb2_adr_o = fetch_insn_o[`OR1K_RB_SELECT] + 1'b1;
  // to DECODE
  assign fetch_rfd2_adr_o = fetch_insn_o[`OR1K_RD_SELECT] + 1'b1;

  // Jump/Branch processing
  wire [`OR1K_OPCODE_WIDTH-1:0] opc_insn = fetch_insn_o[`OR1K_OPCODE_SELECT];
  //  # jump/branch variants
  assign fetch_op_jimm_o  = (opc_insn == `OR1K_OPCODE_J)  | (opc_insn == `OR1K_OPCODE_JAL);
  assign fetch_op_jr_o    = (opc_insn == `OR1K_OPCODE_JR) | (opc_insn == `OR1K_OPCODE_JALR);
  assign fetch_op_bf_o    = (opc_insn == `OR1K_OPCODE_BF);
  assign fetch_op_bnf_o   = (opc_insn == `OR1K_OPCODE_BNF);
  //  # combined jump/branch flag
  assign fetch_op_jb_o    = fetch_op_jimm_o | fetch_op_jr_o | fetch_op_bf_o | fetch_op_bnf_o;
  //  # "to immediate driven target"
  assign fetch_to_imm_target_o = pc_fetch_o + {{4{fetch_insn_o[25]}},fetch_insn_o[25:0],2'b00};


  // delay slot flag
  always @(posedge cpu_clk) begin
    if (cpu_rst | flush_by_ctrl)
      fetch_delay_slot_o <= 1'b0; // reset / flush
    else if (padv_fetch_i)
      fetch_delay_slot_o <= fetch_insn_valid_o ? fetch_op_jb_o : fetch_delay_slot_o;
  end // @ clock
  

  // PC
  assign pc_fetch_o = s2o_virt_addr;

endmodule // mor1kx_fetch_marocchino
