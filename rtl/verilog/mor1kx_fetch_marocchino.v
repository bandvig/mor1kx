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

  // branch/jump control transfer
  //  ## detect jump/branch to indicate "delay slot" for next fetched instruction
  input                                 dcod_jump_or_branch_i,
  //  ## do branch (pedicted or unconditional)
  input                                 do_branch_i,
  input      [OPTION_OPERAND_WIDTH-1:0] do_branch_target_i,
  input                                 fetch_jr_bc_hazard_i,

  // DU/exception/rfe control transfer
  input                                 ctrl_branch_exception_i,
  input      [OPTION_OPERAND_WIDTH-1:0] ctrl_branch_except_pc_i,

  // to RF
  output     [OPTION_RF_ADDR_WIDTH-1:0] fetch_rfa1_adr_o,
  output     [OPTION_RF_ADDR_WIDTH-1:0] fetch_rfb1_adr_o,
  // for FPU64
  output     [OPTION_RF_ADDR_WIDTH-1:0] fetch_rfa2_adr_o,
  output     [OPTION_RF_ADDR_WIDTH-1:0] fetch_rfb2_adr_o,

  // to DECODE
  output reg                            dcod_insn_valid_o,
  output reg [OPTION_OPERAND_WIDTH-1:0] pc_decode_o,
  output reg     [`OR1K_INSN_WIDTH-1:0] dcod_insn_o,
  output reg                            dcod_delay_slot_o,
  output     [OPTION_RF_ADDR_WIDTH-1:0] dcod_rfa1_adr_o,
  output     [OPTION_RF_ADDR_WIDTH-1:0] dcod_rfb1_adr_o,
  // for FPU64
  output     [OPTION_RF_ADDR_WIDTH-1:0] dcod_rfa2_adr_o,
  output     [OPTION_RF_ADDR_WIDTH-1:0] dcod_rfb2_adr_o,
  output     [OPTION_RF_ADDR_WIDTH-1:0] insn_rfd2_adr_o,

  // exceptions
  output reg                            fetch_except_ibus_err_o,
  output reg                            fetch_except_itlb_miss_o,
  output reg                            fetch_except_ipagefault_o,
  output reg                            fetch_an_except_o,
  output reg                            fetch_exception_taken_o
);

  /*
     Definitions:
       s??r_name - "S"tage number "??", input "r"egisters
       s??t_name - "S"tage number "??", "T"emporary (internally)
  */

  localparam IFOOW = OPTION_OPERAND_WIDTH; // short name


  /**** Stage #1 (MMU access) registers and wires ****/

  // Stage #1 access is valid (regular address update or valid branching)
  //   To minimize logic for control padv-s1s2 computation we don't
  // stop IFETCH stages #1 and #2 even so jump/branch target
  // is not ready. So we rise "hit" if virtual address is valid.
  reg                             s1r_fetch_req_hit;
  // IMMU's registered input(s)
  reg                 [IFOOW-1:0] s1r_virt_addr; // for comparison in IMMU
  /* HW reload TLB related (MAROCCHINO_TODO : not implemented yet)
  wire                            tlb_reload_req;
  reg                             tlb_reload_ack;
  wire                [IFOOW-1:0] tlb_reload_addr;
  reg                 [IFOOW-1:0] tlb_reload_data;
  wire                            tlb_reload_pagefault;
  wire                            tlb_reload_busy; */


  /**** Stage #2 (ICACHE access) registers and wires ****/

  // Regular update or valid branching
  reg                             s2r_fetch_req_hit;
  // Mappinf delay slot
  reg                             s2r_ds;
  // Registered virtual address
  reg                 [IFOOW-1:0] s2r_virt_addr;
  // IMMU's registered outputs
  reg                 [IFOOW-1:0] s2r_phys_addr;
  reg                             s2r_cache_inhibit;
  reg                             s2r_tlb_miss;
  reg                             s2r_pagefault;


  //--- ICACHE related controls and signals ---

  // Not cacheble area -> IBUS access request
  wire                            ibus_access_req;
  // ICACHE output ready (by read or re-fill) and data
  wire                            ic_ack;
  wire     [`OR1K_INSN_WIDTH-1:0] ic_dat;
  // ICACHE requests and performs refill
  wire                            ic_refill_req;
  wire                            ic_refill_first;


  //--- IBUS access state machine controls ---

  //   IBUS output ready
  // Indicates IBUS ACK for IBUS direct access only
  // (not ACKs for ICACHE refill):
  wire              ibus_ack;
  // IBUS FSM statuses
  wire              ibus_fsm_free;
  // IBUS access state machine
  localparam  [4:0] IBUS_IDLE       = 5'b00001,
                    IMEM_REQ        = 5'b00010,
                    IBUS_READ       = 5'b00100,
                    IBUS_IC_REFILL  = 5'b01000,
                    IBUS_ERR        = 5'b10000;
  // ---
  reg         [4:0] ibus_state;
  // particular states
  wire              ibus_idle_state = ibus_state[0];
  wire              imem_req_state  = ibus_state[1];
  wire              ibus_read_state = ibus_state[2];
  wire              ic_refill_state = ibus_state[3];
  wire              ibus_err_state  = ibus_state[4];


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


  // detector that stage #2 is fetching next instruction:
  //   PC in DECODE and IFETCH differs by 4
  wire s2_fetching_next_insn = s2r_virt_addr[2] ^ pc_decode_o[2];


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


  //---------------------------------------//
  //   Forming stage #1 hit and delay slot //
  // flags in case of jump/branch decoding //
  //---------------------------------------//

  // As stages #1 and #2 advances synchronously, if j/b is in DECODE than
  //  - stage #2 is (a) fetching delay slot or (b) doing ICACHE re-fill
  //  - stage #1 is (a) miss transformation or (b) delay slot transformation
  // tracking delay slot processing

  localparam [2:0] S1_OUT_REGULAR = 3'b001, // "till j/b in DECODE stage"
                   S1_OUT_BUBBLE  = 3'b010,
                   S1_OUT_DS      = 3'b100;
  // ---
  reg [2:0] s1_out_status_r;
  // ---
  always @(posedge cpu_clk) begin
    if (cpu_rst | flush_by_ctrl) begin
      s1_out_status_r <= S1_OUT_REGULAR; // by reset or flush
    end
    else begin
      // synthesis parallel_case full_case
      case (s1_out_status_r)
        // mapping delay slot
        S1_OUT_DS : begin
          if (padv_s1s2)
            s1_out_status_r <= S1_OUT_REGULAR;
        end
        // bubbling till j/b hazards resolving
        S1_OUT_BUBBLE : begin
          if (padv_s1s2) // stage #1 output status: S1-OUT-DS or S1-OUT-BUBBLE
            s1_out_status_r <= fetch_jr_bc_hazard_i ? S1_OUT_BUBBLE : S1_OUT_REGULAR;
        end
        // waiting j/b in DECODE stage
        S1_OUT_REGULAR : begin
          if (padv_s1s2) // stage #1 output status: S1-OUT-REGULAR
            s1_out_status_r <= (dcod_jump_or_branch_i &  s2_fetching_next_insn) ?
                                          (fetch_jr_bc_hazard_i ? S1_OUT_BUBBLE : S1_OUT_REGULAR) :
                                                                                  S1_OUT_REGULAR;
          else
            s1_out_status_r <= (dcod_jump_or_branch_i &  s2_fetching_next_insn) ? S1_OUT_BUBBLE :
                               (dcod_jump_or_branch_i & ~s2_fetching_next_insn) ? S1_OUT_DS     :
                                                                                  S1_OUT_REGULAR;
        end
        // by default
        default :
          s1_out_status_r <= S1_OUT_REGULAR; // by default
      endcase
    end
  end // @cpu-clk
  // --- output is "bubble" ---
  wire s1t_out_bubble = (dcod_jump_or_branch_i &   s2_fetching_next_insn) | s1_out_status_r[1];
  // --- mapping delay slot ---
  wire s1t_out_ds     = (dcod_jump_or_branch_i &  ~s2_fetching_next_insn) | s1_out_status_r[2];


  // Select the PC for next fetch:
  wire [IFOOW-1:0] virt_addr_mux;
  //                     use DU/exceptions/rfe provided address
  assign virt_addr_mux = do_except            ? ctrl_branch_except_pc_i :
  //                     regular update of IFETCH
                         do_branch_p          ? do_branch_target_p      :
                         fetch_jr_bc_hazard_i ? s1r_virt_addr           :
                         do_branch_i          ? do_branch_target_i      :
                         s1t_out_bubble       ? s1r_virt_addr           : // repeat till j/b hazards resolving
                                                (s1r_virt_addr + 3'd4);   // next PC if no branch


  // IMMU match address store register
  always @(posedge cpu_clk) begin
    if (cpu_rst)
      s1r_virt_addr <= OPTION_RESET_PC - 4; // reset: will be restored on 1st advance
    else if (flush_by_ctrl)
      s1r_virt_addr <= {IFOOW{1'b0}};       // flushing
    else if (padv_s1s2)
      s1r_virt_addr <= virt_addr_mux;
  end // @ clock


  // new fetch request is valid
  always @(posedge cpu_clk) begin
    if (cpu_rst | flush_by_ctrl)
      s1r_fetch_req_hit <= 1'b0; // start fetching from "virt-addr-next" (see hierarhy in virt-addr-mux)
    else if (padv_s1s2)
      s1r_fetch_req_hit <= ~fetch_jr_bc_hazard_i;
  end // @ clock


  //------------------//
  // Instance of IMMU //
  //------------------//

  // IMMU's output wires
  wire [IFOOW-1:0] s1t_phys_addr;
  wire             s1t_cache_inhibit;
  wire             s1t_tlb_miss;
  wire             s1t_pagefault;

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
    .virt_addr_i                    (s1r_virt_addr), // IMMU
    .phys_addr_o                    (s1t_phys_addr), // IMMU
    // flags
    .cache_inhibit_o                (s1t_cache_inhibit), // IMMU
    .tlb_miss_o                     (s1t_tlb_miss), // IMMU
    .pagefault_o                    (s1t_pagefault), // IMMU
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


  /****************************************/
  /* Stage #2: ICACHE check / IBUS access */
  /****************************************/


  // - Stage #2 access is valid (regular address update or valid branching)
  // - Registered virtual address
  // - IMMU's registered outputs
  always @ (posedge cpu_clk) begin
    if (cpu_rst | flush_by_ctrl) begin
      s2r_fetch_req_hit <= 1'b0;
      s2r_ds            <= 1'b0;
      s2r_virt_addr     <= {IFOOW{1'b0}};
      s2r_phys_addr     <= {IFOOW{1'b0}};
      s2r_cache_inhibit <= 1'b0;
      s2r_tlb_miss      <= 1'b0;
      s2r_pagefault     <= 1'b0;
    end
    else if (padv_s1s2) begin
      s2r_fetch_req_hit <= s1r_fetch_req_hit & (~s1t_out_bubble);
      s2r_ds            <= s1t_out_ds;
      s2r_virt_addr     <= s1r_virt_addr;
      s2r_phys_addr     <= s1t_phys_addr;
      s2r_cache_inhibit <= s1t_cache_inhibit;
      s2r_tlb_miss      <= s1t_tlb_miss;
      s2r_pagefault     <= s1t_pagefault;
    end
  end // @cpu-clock


  // MMU's exception with fetch request validity flag
  wire s2t_immu_an_except = s2r_fetch_req_hit & (s2r_tlb_miss | s2r_pagefault);


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
    .padv_ic_i            (padv_s1s2), // ICACHE
    .flush_by_ctrl_i      (flush_by_ctrl), // ICACHE
    // fetch exceptions
    .immu_an_except_i     (s2t_immu_an_except), // ICACHE
    .ibus_err_i           (ibus_err_i), // ICACHE: cancel re-fill
    // configuration
    .ic_enable_i          (ic_enable_i), // ICACHE
    // regular requests in/out
    .phys_addr_idx_i      (s1t_phys_addr), // ICACHE
    .phys_addr_tag_i      (s2r_phys_addr), // ICACHE
    .fetch_req_hit_i      (s2r_fetch_req_hit), // ICACHE: anables ICACHE's ACK
    .immu_cache_inhibit_i (s2r_cache_inhibit), // ICACHE
    .ic_ack_o             (ic_ack), // ICACHE
    .ic_dat_o             (ic_dat), // ICACHE
    // IBUS access request
    .ibus_access_req_o    (ibus_access_req), // ICACHE
    // re-fill
    .refill_req_o         (ic_refill_req), // ICACHE
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


  //--------------------//
  // IBUS state machine //
  //--------------------//

  // IBUS FSM status is stop
  // !!! should follows appropriate FSM condition,
  //     but without taking into account exceptions
  assign ibus_fsm_free = ibus_idle_state                          | // IBUS FSM is free
                         (imem_req_state  & (~s2r_fetch_req_hit)) | // IBUS FSM is free: continue fetching by miss
                         (imem_req_state  & ic_ack)               | // IBUS FSM is free
                         (ibus_read_state & ibus_ack_i);            // IBUS FSM is free

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
          if (flush_by_ctrl)
            ibus_state <= IBUS_IDLE; // IBUS-IDLE -> keep by exceptions or flushing
          else if (padv_fetch_i)     // eq. padv_s1s2 (in IBUS-IDLE state of IBUS FSM)
            ibus_state <= IMEM_REQ;  // IBUS-IDLE -> IMEM-REQ
        end

        IMEM_REQ: begin
          if (s2t_immu_an_except | flush_by_ctrl) begin
            ibus_state <= IBUS_IDLE;  // IMEM-REQ -> IBUS-IDLE (exceptions or flushing)
          end
          else if (ic_refill_req) begin
            ibus_req_o <= ~ibus_req_o;      // IMEM-REQ -> ICACHE refill
            ibus_adr_o <= s2r_phys_addr;    // IMEM-REQ -> ICACHE refill
            ibus_state <= IBUS_IC_REFILL;   // IMEM-REQ -> ICACHE refill
          end
          else if (ibus_access_req) begin
            ibus_req_o <= ~ibus_req_o;      // IMEM-REQ -> IBUS read
            ibus_adr_o <= s2r_phys_addr;    // IMEM-REQ -> IBUS read
            ibus_state <= IBUS_READ;        // IMEM-REQ -> IBUS read
          end
          else if (padv_fetch_i) begin // IMEM-REQ (no exceptions, no flushing, no re-fill request, no ibus-access request)
            ibus_state <= (~s2r_fetch_req_hit | ic_ack) ? IMEM_REQ : IBUS_IDLE; // IMEM-REQ: eq. padv_s1s2 (in IMEM-REQ state of IBUS FSM)
          end
          else
            ibus_state <= IBUS_IDLE;
        end

        IBUS_IC_REFILL: begin
          if ((ibus_ack_i & ibus_burst_last_i) | ibus_err_i) // ICACHE refill
            ibus_state <= ibus_err_i ? IBUS_ERR : IBUS_IDLE;  // ICACHE refill -> error / idling
        end // ic-refill

        IBUS_READ: begin
          if (ibus_ack_i | ibus_err_i) begin    // IBUS read
            if (flush_by_ctrl)                  // IBUS READ -> IDLE: also priority in IMMU and ICACHE
              ibus_state <= IBUS_IDLE;          // IBUS READ -> IDLE by flushing
            else if (ibus_err_i)                // IBUS READ -> IBUS ERROR
              ibus_state <= IBUS_ERR;           // IBUS READ -> IBUS ERROR
            else if (padv_fetch_i)              // eq. padv_s1s2 (IBUS read -> IMEM REQUEST)
              ibus_state <= IMEM_REQ;           // IBUS read -> IMEM REQUEST
            else
              ibus_state <= IBUS_IDLE;          // IBUS READ -> IDLE
          end
        end // read

        IBUS_ERR: begin
          if (flush_by_ctrl)
            ibus_state <= IBUS_IDLE; // IBUS ERR -> IDLE by flushing
        end

        default: begin
          ibus_req_o <= ibus_req_o;     // default: no toggle
          ibus_state <= IBUS_IDLE;      // default
        end
      endcase // case (state)
    end // reset / regular update
  end // @ clock

  // And burst mode
  assign ibus_burst_o = ic_refill_state;


  //------------------------------//
  // Pending ICAHCE/IBUS ACK/DATA //
  //------------------------------//

  reg                         imem_ack_p;
  reg  [`OR1K_INSN_WIDTH-1:0] imem_dat_p;

  // Pending ICAHCE/IBUS ACK
  always @(posedge cpu_clk) begin
    if (cpu_rst | flush_by_ctrl)
      imem_ack_p <= 1'b0;
    else if (padv_fetch_i)
      imem_ack_p <= 1'b0;
    else if (ic_ack | ibus_ack)
      imem_ack_p <= 1'b1;
  end // @ clock

  // Pending ICACHE/IBUS DATA
  always @(posedge cpu_clk) begin
    if (ic_ack)
      imem_dat_p <= ic_dat;
    else if (ibus_ack)
      imem_dat_p <= ibus_dat_i;
  end // @ clock


  //---------------------------------//
  // Stage #2 output is "delay slot" //
  //---------------------------------//

  // valid instruction or exceptions
  wire s2t_insn_or_excepts;


  // As stages #1 and #2 advances synchronously, if j/b is in DECODE than
  //  - stage #2 is (a) fetching delay slot or (b) doing ICACHE re-fill
  //  - stage #1 is (a) miss transformation or (b) delay slot transformation
  // tracking delay slot processing
  reg s2p_out_ds;
  // ---
  always @(posedge cpu_clk) begin
    if (cpu_rst | flush_by_ctrl)
      s2p_out_ds <= 1'b0;
    else if (padv_fetch_i) begin // stage #2 tracking delay slot
      if (s2p_out_ds)
        s2p_out_ds <= ~s2t_insn_or_excepts;
      else
        s2p_out_ds <= (dcod_jump_or_branch_i & s2_fetching_next_insn) & ~s2t_insn_or_excepts;
    end
  end // @ clock
  // --- stage #2 is fetching delay slot ---
  wire s2t_out_ds = (dcod_jump_or_branch_i & s2_fetching_next_insn) | s2p_out_ds;


  //-------------------------------------------//
  // Stage #2 output latches (output of FETCH) //
  //-------------------------------------------//

  // not masked combination of ACKs
  wire s2t_ack = ic_ack | ibus_ack | imem_ack_p;

  // combined MMU's and IBUS's exceptions
  wire s2t_an_except = (s2t_immu_an_except | ibus_err_state);

  // valid instruction
  assign s2t_insn_or_excepts = s2t_ack | s2t_an_except;

  // instruction word
  wire [`OR1K_INSN_WIDTH-1:0] s2t_insn_mux;
  // ---
  assign s2t_insn_mux = s2t_an_except ? {`OR1K_OPCODE_NOP,26'd0} :
                        imem_ack_p    ? imem_dat_p               :
                        ibus_ack      ? ibus_dat_i               :
                        ic_ack        ? ic_dat                   :
                                        {`OR1K_OPCODE_NOP,26'd0};

  // to DECODE: delay slot flag
  always @(posedge cpu_clk) begin
    if (cpu_rst | flush_by_ctrl) begin
      dcod_delay_slot_o         <= 1'b0;
      dcod_insn_o               <= {`OR1K_OPCODE_NOP,26'd0};
      dcod_insn_valid_o         <= 1'b0;
      // exceptions
      fetch_except_ibus_err_o   <= 1'b0;
      fetch_except_itlb_miss_o  <= 1'b0;
      fetch_except_ipagefault_o <= 1'b0;
      fetch_an_except_o         <= 1'b0;
      // actual programm counter
      pc_decode_o               <= {IFOOW{1'b0}}; // reset / flush
    end
    else if (padv_fetch_i) begin
      dcod_delay_slot_o         <= (s2r_ds | s2t_out_ds) & s2t_insn_or_excepts;
      dcod_insn_o               <= s2t_insn_mux;
      dcod_insn_valid_o         <= s2t_insn_or_excepts; // valid instruction or exception
      // exceptions
      fetch_except_ibus_err_o   <= ibus_err_state;
      fetch_except_itlb_miss_o  <= s2r_fetch_req_hit & s2r_tlb_miss;
      fetch_except_ipagefault_o <= s2r_fetch_req_hit & s2r_pagefault;
      fetch_an_except_o         <= s2t_an_except;
      // actual programm counter
      pc_decode_o               <= s2t_insn_or_excepts ? s2r_virt_addr : pc_decode_o;
    end
  end // @ clock

  // to RF
  assign fetch_rfa1_adr_o = s2t_insn_mux[`OR1K_RA_SELECT];
  assign fetch_rfb1_adr_o = s2t_insn_mux[`OR1K_RB_SELECT];

  // to DECODE
  assign dcod_rfa1_adr_o = dcod_insn_o[`OR1K_RA_SELECT];
  assign dcod_rfb1_adr_o = dcod_insn_o[`OR1K_RB_SELECT];

  // to FPU64
  assign fetch_rfa2_adr_o = s2t_insn_mux[`OR1K_RA_SELECT] + 1'b1;
  assign fetch_rfb2_adr_o = s2t_insn_mux[`OR1K_RB_SELECT] + 1'b1;
  // ---
  reg [(OPTION_RF_ADDR_WIDTH-1):0] dcod_rfa2_adr_r, dcod_rfb2_adr_r, insn_rfd2_adr_r;
  // ---
  always @(posedge cpu_clk) begin
    if (cpu_rst | flush_by_ctrl) begin
      dcod_rfa2_adr_r <= {{(OPTION_RF_ADDR_WIDTH-1){1'b0}},1'b1};
      dcod_rfb2_adr_r <= {{(OPTION_RF_ADDR_WIDTH-1){1'b0}},1'b1};
      insn_rfd2_adr_r <= {{(OPTION_RF_ADDR_WIDTH-1){1'b0}},1'b1};
    end
    else if (padv_fetch_i) begin
      dcod_rfa2_adr_r <= fetch_rfa2_adr_o;
      dcod_rfb2_adr_r <= fetch_rfb2_adr_o;
      insn_rfd2_adr_r <= s2t_insn_mux[`OR1K_RD_SELECT] + 1'b1;
    end
  end // @ clock
  // ---
  assign dcod_rfa2_adr_o = dcod_rfa2_adr_r;
  assign dcod_rfb2_adr_o = dcod_rfb2_adr_r;
  assign insn_rfd2_adr_o = insn_rfd2_adr_r;

endmodule // mor1kx_fetch_marocchino
