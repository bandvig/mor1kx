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
  input                                 clk,
  input                                 rst,

  // pipeline control
  input                                 padv_fetch_i,
  input                                 clean_fetch_i,
  input                                 stall_fetch_i,
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
  output reg                            ibus_req_o,
  output reg [OPTION_OPERAND_WIDTH-1:0] ibus_adr_o,
  output                                ibus_burst_o,

  // branch/jump control transfer
  input                                 dcod_take_branch_i,
  input      [OPTION_OPERAND_WIDTH-1:0] dcod_branch_target_i,
  input                                 branch_mispredict_i,
  input      [OPTION_OPERAND_WIDTH-1:0] exec_mispredict_target_i,
  output                                mispredict_deassert_o,
  // exception/rfe control transfer
  input                                 ctrl_branch_exception_i,
  input      [OPTION_OPERAND_WIDTH-1:0] ctrl_branch_except_pc_i,
  // debug unit command for control transfer
  input                                 du_restart_i,
  input      [OPTION_OPERAND_WIDTH-1:0] du_restart_pc_i,

  // to RF
  output     [OPTION_RF_ADDR_WIDTH-1:0] fetch_rfa_adr_o,
  output     [OPTION_RF_ADDR_WIDTH-1:0] fetch_rfb_adr_o,
  output                                fetch_rf_adr_valid_o,
  // to DECODE
  output reg [OPTION_OPERAND_WIDTH-1:0] pc_decode_o,
  output reg     [`OR1K_INSN_WIDTH-1:0] dcod_insn_o,
  output reg                            dcod_op_branch_o,
  output reg                            dcod_delay_slot_o,
  output reg                            dcod_insn_valid_o,
  // exceptions
  output reg                            dcod_except_ibus_err_o,
  output reg                            dcod_except_itlb_miss_o,
  output reg                            dcod_except_ipagefault_o,
  output reg                            fetch_exception_taken_o
);

  /*
     Definitions:
       s??o_name - "S"tage number "??", "O"utput
       s??t_name - "S"tage number "??", "T"emporary (internally)
  */

  localparam IFOOW = OPTION_OPERAND_WIDTH; // short name

  /* MMU related controls and signals */

  // IMMU's regular output
  wire                            immu_cache_inhibit;
  // IMMU exceptions (valid for enabled mmu only)
  //  # connections to IMMU module
  wire                            immu_tlb_miss;
  wire                            immu_pagefault;
  //  # masked with flush by branch or mispredict
  wire                            except_itlb_miss;
  wire                            except_ipagefault;
  /* HW reload TLB related (MAROCCHINO_TODO : not implemented yet)
  wire                            tlb_reload_req;
  reg                             tlb_reload_ack;
  wire                [IFOOW-1:0] tlb_reload_addr;
  reg                 [IFOOW-1:0] tlb_reload_data;
  wire                            tlb_reload_pagefault;
  wire                            tlb_reload_busy; */


  /* ICACHE related controls and signals */

  // ICACHE access flag (without taking exceptions into accaunt)
  wire                            ic_access;
  // ICACHE output ready (by read or re-fill) and data
  wire                            ic_ack;
  wire     [`OR1K_INSN_WIDTH-1:0] ic_dat;
  // ICACHE requests and performs refill
  wire                            ic_refill_req;
  reg                             ic_refill_allowed; // combinatorial
  wire                [IFOOW-1:0] next_refill_adr;
  wire                            ic_refill_last;


  /* IBUS access state machine controls */

  //   IBUS output ready
  // Indicates IBUS ACK for IBUS direct access only
  // (not ACKs for ICACHE refill):
  wire                            ibus_ack;
  // IBUS FSM statuses
  wire                            ibus_fsm_free;
  // IBUS access state machine
  localparam                [3:0] IBUS_IDLE       = 4'b0001,
                                  IMEM_REQ        = 4'b0010,
                                  IBUS_READ       = 4'b0100,
                                  IBUS_IC_REFILL  = 4'b1000;
  //
  reg                       [3:0] ibus_state;
  // IBUS error processing
  wire                            ibus_err_instant; // error reported "just now"
  wire                            except_ibus_err;  // masked by stage #2 flushing (see later)


  /* ICACHE/IBUS requests and nswers */

  // The logic is located in Stage #2 section

  //   ACK/DATA stored
  // They passed (if ready) to stage #2 output latches @ next advance
  reg                         ic_ack_stored;
  reg  [`OR1K_INSN_WIDTH-1:0] ic_dat_stored;
  reg                         ibus_ack_stored;
  reg  [`OR1K_INSN_WIDTH-1:0] ibus_dat_stored;


  /* Wires & registers are used across FETCH pipe stages */

  // Flush processing
  wire flush_by_ctrl;       // flush registers from pipeline-flush command till IBUS transaction completion
  wire flush_by_borm_ds_s2; // flush stage #2 by-branch OR by-mispredict and takes into accaunt fetching delay slot flag
  wire flush_by_misp_ds_s3; // flush stage #3 by-mispredict and takes into accaunt delay slot flag on stage #2 output

  // ICACHE/IMMU match address store register
  //   The register operates in the same way
  // as memory blocks in ICACHE/IMMU to provide correct
  // address for comparision on output of ICACHE/MMU memory blocks.
  reg  [IFOOW-1:0] virt_addr_fetch;

  // Physical address (after translation in IMMU)
  wire [IFOOW-1:0] phys_addr_fetch;

  // to s3:
  reg [IFOOW-1:0] s2o_pc; // program counter
  reg             s2o_ds; // delay slot is in stage #3 (on stage #2 output)

  // jump/branch instruction is on stage #2 outputs
  wire s3t_jb;


  /********************/
  /* IFETCH exeptions */
  /********************/

  // IMMU exceptions masked by branch or "mispredicted branch" cases.
  assign except_itlb_miss  = immu_tlb_miss  & ~flush_by_borm_ds_s2;
  assign except_ipagefault = immu_pagefault & ~flush_by_borm_ds_s2;

  // IBUS error during IBUS access
  assign ibus_err_instant = ibus_req_o & ibus_err_i;
  // IBUS error stored for exception processing
  reg  ibus_err_r;
  // ---
  always @(posedge clk `OR_ASYNC_RST) begin
    if (rst)
      ibus_err_r <= 1'b0;
    else if (flush_by_borm_ds_s2 | flush_by_ctrl)
      ibus_err_r <= 1'b0;
    else if (ibus_err_instant)
      ibus_err_r <= 1'b1;
  end // @ clock
  // IBUS error stored and masked by branch or "mispredicted branch" cases.
  assign except_ibus_err = ibus_err_r & ~flush_by_borm_ds_s2;

  // combined MMU's and IBUS's exceptions
  wire fetch_excepts = except_itlb_miss | except_ipagefault | except_ibus_err;


  /************************/
  /* IFETCH pipe controls */
  /************************/

  // Advance stage #1
  wire padv_s1 = padv_fetch_i & ibus_fsm_free;


  /************************************************/
  /* Stage #1: PC update and IMMU / ICACHE access */
  /************************************************/


  //   1-clock flag to indicate that ICACHE/IBUS
  // has started to fetch new instruction
  reg imem_new_fetch_r;
  // ---
  always @(posedge clk `OR_ASYNC_RST) begin
    if (rst)
      imem_new_fetch_r <= 1'b0;
    else if (padv_s1 & ~(fetch_excepts | flush_by_ctrl))
      imem_new_fetch_r <= 1'b1;
    else
      imem_new_fetch_r <= 1'b0;
  end // @ clock


  // take delay slot with next padv-s1
  reg take_ds_r;
  // fetching delay slot
  reg fetching_ds_r;
  // pay attention: if low bits of s1o-virt-addr are equal to
  //                s2o-pc ones it means that delay slot isn't
  //                under processing right now
  always @(posedge clk `OR_ASYNC_RST) begin
    if (rst)
      take_ds_r <= 1'b0;
    else if ((padv_s1 & ~fetch_excepts) | flush_by_ctrl)
      take_ds_r <= 1'b0;
    else if (~take_ds_r)
      take_ds_r <= (s3t_jb & ~imem_new_fetch_r & ~fetching_ds_r);
  end // @ clock
  // combined flag to take delay slot with next padv-s1
  wire take_ds = (s3t_jb & ~imem_new_fetch_r & ~fetching_ds_r) | take_ds_r;

  // ---
  always @(posedge clk `OR_ASYNC_RST) begin
    if (rst)
      fetching_ds_r <= 1'b0;
    else if (flush_by_ctrl)
      fetching_ds_r <= 1'b0;
    else if (padv_s1 & ~fetch_excepts)
      fetching_ds_r <= take_ds;
    else if (~fetching_ds_r)
      fetching_ds_r <= (s3t_jb & imem_new_fetch_r);
  end // @ clock
  // combined fetching delay slot flag
  wire fetching_ds = (s3t_jb & imem_new_fetch_r) | fetching_ds_r;


  // store mispredict flag and target if stage #1 is busy
  reg                            mispredict_stored;
  reg [OPTION_OPERAND_WIDTH-1:0] mispredict_target_stored;
  reg                            mispredict_taken_r;
  // ---
  always @(posedge clk `OR_ASYNC_RST) begin
    if (rst) begin
      mispredict_stored        <= 1'b0;           // reset
      mispredict_target_stored <= {IFOOW{1'b0}};  // reset
    end
    else if ((padv_s1 & ~(fetch_excepts | take_ds)) |   // clean up stored mispredict
             mispredict_taken_r | flush_by_ctrl) begin  // clean up stored mispredict
      mispredict_stored        <= 1'b0;           // mispredict has been taken or flushed by pipe-flushing
      mispredict_target_stored <= {IFOOW{1'b0}};  // mispredict has been taken or flushed by pipe-flushing
    end
    else if (branch_mispredict_i & ~mispredict_stored) begin
      mispredict_stored        <= 1'b1;
      mispredict_target_stored <= exec_mispredict_target_i;
    end
  end // @ clock
  // ---
  always @(posedge clk `OR_ASYNC_RST) begin
    if (rst)
      mispredict_taken_r <= 1'b0;
    else if (mispredict_taken_r | flush_by_ctrl) // branch-mispredict-i flag will be cleaned by taken (see later)
      mispredict_taken_r <= 1'b0;
    else if (padv_s1 & ~(fetch_excepts | take_ds) & branch_mispredict_i)  // for "mispredict has been taken"
      mispredict_taken_r <= 1'b1;
  end // @ clock
  // drop mispredict flag in EXECUTE
  assign mispredict_deassert_o  = mispredict_taken_r | mispredict_stored;
  // flush some registers in stages #2 & #3 if mispredict branch processing
  wire flush_by_mispredict = (branch_mispredict_i & ~mispredict_taken_r) | mispredict_stored;


  // store branch flag and target if stage #1 is busy
  reg                            branch_stored;
  reg [OPTION_OPERAND_WIDTH-1:0] branch_target_stored;
  // ---
  always @(posedge clk `OR_ASYNC_RST) begin
    if (rst) begin
      branch_stored        <= 1'b0;           // reset
      branch_target_stored <= {IFOOW{1'b0}};  // reset
    end
    else if ((padv_s1 & ~(fetch_excepts | take_ds)) | flush_by_ctrl) begin  // for clean up stored branch
      branch_stored        <= 1'b0;           // take stored branch or flush by pipe-flushing
      branch_target_stored <= {IFOOW{1'b0}};  // take stored branch or flush by pipe-flushing
    end
    else if (dcod_take_branch_i & ~stall_fetch_i & ~branch_stored) begin
      branch_stored        <= 1'b1;
      branch_target_stored <= dcod_branch_target_i;
    end
  end // @ clock
  // flush some registers if branch processing
  wire flush_by_branch = dcod_take_branch_i | branch_stored;


  // combined: by-branch OR by-mispredict and takes into accaunt fetching delay slot flag
  assign flush_by_borm_ds_s2 = (flush_by_branch | flush_by_mispredict) & ~fetching_ds;
  // combined: by-mispredict and takes into accaunt delay slot flag on stage #2 output
  assign flush_by_misp_ds_s3 = flush_by_mispredict & ~s2o_ds; 


  // 1-clock fetch-exception-taken
  // The flush-by-ctrl is dropped synchronously with s1-stall
  always @(posedge clk `OR_ASYNC_RST) begin
    if (rst)
      fetch_exception_taken_o <= 1'b0;
    else if (padv_s1 & ~(fetch_excepts | flush_by_ctrl))
      fetch_exception_taken_o <= ctrl_branch_exception_i;
    else
      fetch_exception_taken_o <= 1'b0;
  end // @ clock


  // regular value of next PC
  wire [OPTION_OPERAND_WIDTH-1:0] s1t_pc_next = virt_addr_fetch + 4;

  // Select the PC for next fetch
  wire [OPTION_OPERAND_WIDTH-1:0] virt_addr =
    // Debug (MAROCCHINO_TODO)
    du_restart_i                                         ? du_restart_pc_i :
    // padv-s1 and neither exceptions nor pipeline flush
    (ctrl_branch_exception_i & ~fetch_exception_taken_o) ? ctrl_branch_except_pc_i :
    take_ds                                              ? s1t_pc_next :
    (branch_mispredict_i & ~mispredict_taken_r)          ? exec_mispredict_target_i :
    mispredict_stored                                    ? mispredict_target_stored :
    dcod_take_branch_i                                   ? dcod_branch_target_i :
    branch_stored                                        ? branch_target_stored :
                                                           s1t_pc_next;


  // ICACHE/IMMU match address store register
  always @(posedge clk `OR_ASYNC_RST) begin
    if (rst)
      virt_addr_fetch <= OPTION_RESET_PC - 4; // will be restored on 1st advance
    else if (padv_s1 & ~(fetch_excepts | flush_by_ctrl))
      virt_addr_fetch <= virt_addr;
  end // @ clock


  /****************************************/
  /* Stage #2: ICACHE check / IBUS access */
  /****************************************/


  //----------------------------------------//
  // IBUS/ICACHE <-> FETCH's pipe interface //
  //----------------------------------------//

  // ACKs and DATA stored till nearest advance
  always @(posedge clk `OR_ASYNC_RST) begin
    if (rst) begin
      // ACKs
      ic_ack_stored   <= 1'b0;
      ibus_ack_stored <= 1'b0;
      // DATA
      ic_dat_stored   <= {`OR1K_OPCODE_NOP,26'd0};
      ibus_dat_stored <= {`OR1K_OPCODE_NOP,26'd0};
    end
    else if (padv_fetch_i | fetch_excepts | flush_by_ctrl) begin
      // ACKs
      ic_ack_stored   <= 1'b0;
      ibus_ack_stored <= 1'b0;
      // DATA
      ic_dat_stored   <= {`OR1K_OPCODE_NOP,26'd0};
      ibus_dat_stored <= {`OR1K_OPCODE_NOP,26'd0};
    end
    else if (ic_ack | ibus_ack) begin
      // ACKs
      ic_ack_stored   <= ic_ack;
      ibus_ack_stored <= ibus_ack;
      // DATA
      ic_dat_stored   <= ic_dat;
      ibus_dat_stored <= ibus_dat_i;
    end
  end // @ clock


  //-------------------------//
  // Stage #2 output latches //
  //-------------------------//

  // masked ACKs
  wire s2t_ic_ack_instant   = ic_ack          & ~(flush_by_borm_ds_s2 | fetch_excepts);
  wire s2t_ibus_ack_instant = ibus_ack        & ~(flush_by_borm_ds_s2 | fetch_excepts);
  wire s2t_ic_ack_stored    = ic_ack_stored   & ~(flush_by_borm_ds_s2 | fetch_excepts);
  wire s2t_ibus_ack_stored  = ibus_ack_stored & ~(flush_by_borm_ds_s2 | fetch_excepts);

  // not masked combination of ACKs
  wire s2t_ack_raw = ic_ack | ibus_ack | ic_ack_stored | ibus_ack_stored;

  // to s3: instruction valid flags
  reg s2o_ic_ack_instant, s2o_ibus_ack_instant;
  reg s2o_ic_ack_stored,  s2o_ibus_ack_stored;
  //   To minimize number of multiplexors we
  // latche all instuction sources and their validity flags.
  always @(posedge clk `OR_ASYNC_RST) begin
    if (rst) begin
      // instruction valid flags
      s2o_ic_ack_instant   <= 1'b0;
      s2o_ibus_ack_instant <= 1'b0;
      s2o_ic_ack_stored    <= 1'b0;
      s2o_ibus_ack_stored  <= 1'b0;
      // delay slot flag
      s2o_ds               <= 1'b0;
    end
    else if (flush_by_ctrl) begin
      // instruction valid flags
      s2o_ic_ack_instant   <= 1'b0;
      s2o_ibus_ack_instant <= 1'b0;
      s2o_ic_ack_stored    <= 1'b0;
      s2o_ibus_ack_stored  <= 1'b0;
      // delay slot flag
      s2o_ds               <= 1'b0;
    end
    else if (padv_fetch_i) begin
      // instruction valid flags
      s2o_ic_ack_instant   <= s2t_ic_ack_instant;
      s2o_ibus_ack_instant <= s2t_ibus_ack_instant;
      s2o_ic_ack_stored    <= s2t_ic_ack_stored;
      s2o_ibus_ack_stored  <= s2t_ibus_ack_stored;
      // delay slot flag
      s2o_ds               <= fetching_ds & ((s2t_ack_raw & ~ibus_err_instant) | fetch_excepts);
    end
  end // @ clock

  // to s3: instruction words
  reg [`OR1K_INSN_WIDTH-1:0] s2o_ic_dat_instant;
  reg [`OR1K_INSN_WIDTH-1:0] s2o_ibus_dat_instant;
  reg [`OR1K_INSN_WIDTH-1:0] s2o_ic_dat_stored;
  reg [`OR1K_INSN_WIDTH-1:0] s2o_ibus_dat_stored;
  //   To minimize number of multiplexors we
  // latche all instuction sources and their validity flags.
  always @(posedge clk `OR_ASYNC_RST) begin
    if (padv_fetch_i) begin
      s2o_ic_dat_instant   <= ic_dat;
      s2o_ibus_dat_instant <= ibus_dat_i;
      s2o_ic_dat_stored    <= ic_dat_stored;
      s2o_ibus_dat_stored  <= ibus_dat_stored;
    end
  end // @ clock

  // to s3: exception flags
  reg s2o_ibus_err;
  reg s2o_itlb_miss;
  reg s2o_ipagefault;
  // Exceptions: go to pipe around stall logic
  always @(posedge clk `OR_ASYNC_RST) begin
    if (rst) begin
      s2o_ibus_err   <= 1'b0;
      s2o_itlb_miss  <= 1'b0;
      s2o_ipagefault <= 1'b0;
    end
    else if (flush_by_ctrl) begin
      s2o_ibus_err   <= 1'b0;
      s2o_itlb_miss  <= 1'b0;
      s2o_ipagefault <= 1'b0;
    end
    else if (padv_fetch_i) begin
      s2o_ibus_err   <= except_ibus_err;
      s2o_itlb_miss  <= except_itlb_miss;
      s2o_ipagefault <= except_ipagefault;
    end
  end // @ clock

  // to s3: program counter
  always @(posedge clk `OR_ASYNC_RST) begin
    if (rst)
      s2o_pc <= {IFOOW{1'b0}};
    else if (flush_by_ctrl)
      s2o_pc <= {IFOOW{1'b0}};
    else if (padv_fetch_i) begin
      if ((s2t_ack_raw & ~ibus_err_instant) | fetch_excepts)  // store s2o-pc
        s2o_pc <= virt_addr_fetch;
      else
        s2o_pc <= {IFOOW{1'b0}};
    end
  end // @ clock


  /*****************************************/
  /* Stage #3: delay slot & output latches */
  /*****************************************/

  // stage #3 exceptions with flushing mask
  wire s3t_excepts = (s2o_itlb_miss | s2o_ipagefault | s2o_ibus_err) & ~flush_by_misp_ds_s3;

  // valid instruction
  wire s3t_insn = (s2o_ic_ack_instant | s2o_ibus_ack_instant |
                   s2o_ic_ack_stored  | s2o_ibus_ack_stored) & ~flush_by_misp_ds_s3;

  // select insn
  wire [OPTION_OPERAND_WIDTH-1:0] s3t_insn_mux =
    ~s3t_insn            ? {`OR1K_OPCODE_NOP,26'd0} :
    s2o_ic_ack_instant   ? s2o_ic_dat_instant :
    s2o_ibus_ack_instant ? s2o_ibus_dat_instant :
    s2o_ic_ack_stored    ? s2o_ic_dat_stored :
                           s2o_ibus_dat_stored;

  // detect jump/branch to indicate "delay slot" for next fetched instruction
  assign s3t_jb = ((s3t_insn_mux[`OR1K_OPCODE_SELECT] < `OR1K_OPCODE_NOP) |   // l.j  | l.jal  | l.bnf | l.bf
                   (s3t_insn_mux[`OR1K_OPCODE_SELECT] == `OR1K_OPCODE_JR) |   // l.jr
                   (s3t_insn_mux[`OR1K_OPCODE_SELECT] == `OR1K_OPCODE_JALR)); // l.jalr

  // to DECODE: delay slot flag
  always @(posedge clk `OR_ASYNC_RST) begin
    if (rst) begin
      dcod_op_branch_o  <= 1'b0;
      dcod_delay_slot_o <= 1'b0;
      dcod_insn_o       <= {`OR1K_OPCODE_NOP,26'd0};
      dcod_insn_valid_o <= 1'b0;
      // exceptions
      dcod_except_ibus_err_o   <= 1'b0;
      dcod_except_itlb_miss_o  <= 1'b0;
      dcod_except_ipagefault_o <= 1'b0;
      // actual programm counter
      pc_decode_o <= {IFOOW{1'b0}}; // reset
    end
    else if (flush_by_ctrl | clean_fetch_i) begin
      dcod_op_branch_o  <= 1'b0;
      dcod_delay_slot_o <= 1'b0;
      dcod_insn_o       <= {`OR1K_OPCODE_NOP,26'd0};
      dcod_insn_valid_o <= 1'b0;
      // exceptions
      dcod_except_ibus_err_o   <= 1'b0;
      dcod_except_itlb_miss_o  <= 1'b0;
      dcod_except_ipagefault_o <= 1'b0;
      // actual programm counter
      pc_decode_o <= {IFOOW{1'b0}}; // flush
    end
    else if (padv_fetch_i) begin
      dcod_op_branch_o  <= s3t_jb;
      dcod_delay_slot_o <= s2o_ds;
      dcod_insn_o       <= s3t_insn_mux;
      dcod_insn_valid_o <= s3t_insn | s3t_excepts;
      // exceptions
      dcod_except_ibus_err_o   <= s2o_ibus_err   & ~flush_by_misp_ds_s3;
      dcod_except_itlb_miss_o  <= s2o_itlb_miss  & ~flush_by_misp_ds_s3;
      dcod_except_ipagefault_o <= s2o_ipagefault & ~flush_by_misp_ds_s3;
      // actual programm counter
      if (s3t_insn | s3t_excepts)
        pc_decode_o <= s2o_pc;        // valid instruction or exception
      else
        pc_decode_o <= {IFOOW{1'b0}}; // neither instruction nor exception
    end
  end // @ clock

  // to RF
  assign fetch_rfa_adr_o      = s3t_insn_mux[`OR1K_RA_SELECT];
  assign fetch_rfb_adr_o      = s3t_insn_mux[`OR1K_RB_SELECT];
  assign fetch_rf_adr_valid_o = padv_fetch_i & s3t_insn & ~(flush_by_ctrl | clean_fetch_i);


  /********** End of FETCH pipe. Start other logics. **********/

  //-------------//
  // Flush logic //
  //-------------//

  // store flush command till IBUS transactions complete
  reg flush_r;
  // ----
  always @(posedge clk `OR_ASYNC_RST) begin
    if (rst)
      flush_r <= 1'b0;
    else if (ibus_fsm_free)
      flush_r <= 1'b0;
    else if (~flush_r)
      flush_r <= pipeline_flush_i;
  end // @ clock

  // combination of pipeline-flush and flush-r
  assign flush_by_ctrl = pipeline_flush_i | flush_r;

  //--------------------//
  // IBUS state machine //
  //--------------------//

  // IBUS output ready (no bus error case)
  // !!! should follows appropriate FSM condition,
  assign ibus_ack = (ibus_state == IBUS_READ) & ibus_ack_i;

  // IBUS FSM status is stop
  // !!! should follows appropriate FSM condition,
  //     but without taking into account exceptions
  assign ibus_fsm_free =
    (ibus_state == IBUS_IDLE) |                                   // IBUS FSM is free
    ((ibus_state == IMEM_REQ) & (flush_by_borm_ds_s2 | ic_ack)) | // IBUS FSM is free
    ibus_ack;                                                     // IBUS FSM is free


  // ICACHE re-fill-allowed corresponds to refill-request position in IBUS FSM
  always @(*) begin
    ic_refill_allowed = 1'b0;
    case (ibus_state)
      IMEM_REQ: begin
        if (fetch_excepts | flush_by_ctrl |       // re-fill isn't allowed due to exceptions/flushing
            (padv_fetch_i & flush_by_borm_ds_s2)) // re-fill isn't allowed due to flushing by branch or mispredict (eq. padv_s1)
          ic_refill_allowed = 1'b0;
        else if (ic_refill_req) // automatically means (ic-access & ~ic-ack)
          ic_refill_allowed = 1'b1;
      end
      default:;
    endcase
  end // always


  // state machine itself
  always @(posedge clk `OR_ASYNC_RST) begin
    if (rst) begin
      ibus_req_o <= 1'b0;           // by reset
      ibus_adr_o <= {IFOOW{1'b0}};  // by reset
      ibus_state <= IBUS_IDLE;      // by reset
    end
    else begin
      case (ibus_state)
        IBUS_IDLE: begin
          ibus_req_o <= 1'b0;           // idle defaults
          ibus_adr_o <= {IFOOW{1'b0}};  // idle defaults
          ibus_state <= IBUS_IDLE;      // idle defaults
          // ---
          if (padv_fetch_i & ~flush_by_ctrl) // eq. padv_s1 (in IDLE state of IBUS FSM)
            ibus_state <= IMEM_REQ;
        end
      
        IMEM_REQ: begin
          ibus_req_o <= 1'b0;           // imem req defaults
          ibus_adr_o <= {IFOOW{1'b0}};  // imem req defaults
          ibus_state <= IMEM_REQ;       // imem req defaults
          // ---
          if (fetch_excepts | flush_by_ctrl) begin
            ibus_state <= IBUS_IDLE;
          end
          else if (padv_fetch_i & (flush_by_borm_ds_s2 | ic_ack)) begin // eq. padv_s1 (in IMEM-REQ state of IBUS FSM)
            ibus_state <= IMEM_REQ;
          end
          else if (ic_refill_req) begin
            ibus_req_o <= 1'b1;
            ibus_adr_o <= phys_addr_fetch;
            ibus_state <= IBUS_IC_REFILL;
          end
          else if (~ic_access) begin
            ibus_req_o <= 1'b1;
            ibus_adr_o <= phys_addr_fetch;
            ibus_state <= IBUS_READ;
          end
          else
            ibus_state <= IBUS_IDLE;
        end
  
        IBUS_IC_REFILL: begin
          ibus_req_o <= 1'b1;           // re-fill defaults
          ibus_adr_o <= ibus_adr_o;     // re-fill defaults
          ibus_state <= IBUS_IC_REFILL; // re-fill defaults
          // ---
          if (ibus_ack_i) begin
            ibus_adr_o <= next_refill_adr;
            if (ic_refill_last) begin
              ibus_req_o <= 1'b0;
              ibus_adr_o <= {IFOOW{1'b0}};
              ibus_state <= IBUS_IDLE;
            end
          end
          else if (ibus_err_i) begin
            ibus_req_o <= 1'b0;           // bus error during re-fill
            ibus_adr_o <= {IFOOW{1'b0}};  // bus error during re-fill
            //flush_r    <= 1'b0;           // bus error during re-fill
            ibus_state <= IBUS_IDLE;      // bus error during re-fill
          end
        end // ic-refill
  
        IBUS_READ: begin
          ibus_req_o <= 1'b1;       // read defaults
          ibus_adr_o <= ibus_adr_o; // read defaults
          ibus_state <= IBUS_READ;  // read defaults
          // ---
          if (ibus_ack_i) begin
            ibus_req_o <= 1'b0;
            ibus_adr_o <= {IFOOW{1'b0}};
            if (padv_fetch_i & ~flush_by_ctrl)  // IBUS READ -> IMEM REQUEST (eq. padv_s1)
              ibus_state <= IMEM_REQ;                // IBUS READ -> IMEM REQUEST
            else
              ibus_state <= IBUS_IDLE; // IBUS READ -> IDLE
          end
          else if (ibus_err_i) begin
            ibus_req_o <= 1'b0;           // bus error during read
            ibus_adr_o <= {IFOOW{1'b0}};  // bus error during read
            //flush_r    <= 1'b0;           // bus error during read
            ibus_state <= IBUS_IDLE;           // bus error during read
          end
        end // read
  
        default:;
      endcase // case (state)
    end // reset / regular update
  end // @ clock

  // And burst mode
  assign ibus_burst_o = (ibus_state == IBUS_IC_REFILL) & ~ic_refill_last;


  //---------------//
  // SPR interface //
  //---------------//

  //   For MAROCCHINO SPR access means that pipeline is stalled till ACK.
  // So, no padv-*. We only delay SPR access command till IBUS transaction
  // completion.
  wire spr_bus_ifetch_stb = spr_bus_stb_i & (ibus_state == IBUS_IDLE);



  //-------------------//
  // Instance of cache //
  //-------------------//

  // ICACHE module
  mor1kx_icache_marocchino
  #(
    .OPTION_OPERAND_WIDTH         (OPTION_OPERAND_WIDTH),
    .OPTION_ICACHE_BLOCK_WIDTH    (OPTION_ICACHE_BLOCK_WIDTH),
    .OPTION_ICACHE_SET_WIDTH      (OPTION_ICACHE_SET_WIDTH),
    .OPTION_ICACHE_WAYS           (OPTION_ICACHE_WAYS),
    .OPTION_ICACHE_LIMIT_WIDTH    (OPTION_ICACHE_LIMIT_WIDTH),
    .OPTION_ICACHE_CLEAR_ON_INIT  (OPTION_ICACHE_CLEAR_ON_INIT)
  )
  u_icache
  (
    // clock and reset
    .clk                  (clk),
    .rst                  (rst),
    // pipe controls
    .padv_s1_i            (padv_s1), // ICACHE
    .flush_by_ctrl_i      (flush_by_ctrl), // ICACHE
    // fetch exceptions
    .fetch_excepts_i      (fetch_excepts), // ICACHE
    .ibus_err_i           (ibus_err_i), // ICACHE
    // configuration
    .enable_i             (ic_enable_i), // ICACHE
    // regular requests in/out
    .virt_addr_i          (virt_addr), // ICACHE
    .phys_addr_fetch_i    (phys_addr_fetch), // ICACHE
    .immu_cache_inhibit_i (immu_cache_inhibit), // ICACHE
    .ic_access_o          (ic_access), // ICACHE
    .ic_ack_o             (ic_ack), // ICACHE
    .ic_dat_o             (ic_dat), // ICACHE
    // re-fill
    .refill_req_o         (ic_refill_req), // ICACHE
    .ic_refill_allowed_i  (ic_refill_allowed), // ICACHE
    .next_refill_adr_o    (next_refill_adr), // ICACHE
    .refill_last_o        (ic_refill_last), // ICACHE
    .ibus_dat_i           (ibus_dat_i), // ICACHE
    .ibus_ack_i           (ibus_ack_i), // ICACHE
    // SPR bus
    .spr_bus_addr_i       (spr_bus_addr_i[15:0]), // ICACHE
    .spr_bus_we_i         (spr_bus_we_i), // ICACHE
    .spr_bus_stb_i        (spr_bus_ifetch_stb), // ICACHE
    .spr_bus_dat_i        (spr_bus_dat_i), // ICACHE
    .spr_bus_dat_o        (spr_bus_dat_ic_o), // ICACHE
    .spr_bus_ack_o        (spr_bus_ack_ic_o) // ICACHE
  );


  //------------------//
  // Instance of IMMU //
  //------------------//

  // advance IMMU
  wire immu_adv = padv_s1 & ~(fetch_excepts | flush_by_ctrl);

  // Force switching IMMU off in case of IMMU-generated exceptions
  // We use pipeline-flush-i here because FETCH is anycase stopped by
  // IMMU's exceptions
  wire immu_force_off = (immu_tlb_miss | immu_pagefault) & pipeline_flush_i;

  // IMMU
  mor1kx_immu_marocchino
  #(
    .FEATURE_IMMU_HW_TLB_RELOAD (FEATURE_IMMU_HW_TLB_RELOAD),
    .OPTION_OPERAND_WIDTH       (OPTION_OPERAND_WIDTH),
    .OPTION_RESET_PC            (OPTION_RESET_PC),
    .OPTION_IMMU_SET_WIDTH      (OPTION_IMMU_SET_WIDTH),
    .OPTION_IMMU_WAYS           (OPTION_IMMU_WAYS),
    .OPTION_IMMU_CLEAR_ON_INIT  (OPTION_IMMU_CLEAR_ON_INIT)
  )
  u_immu
  (
    .clk                            (clk),
    .rst                            (rst),
    // controls
    .adv_i                          (immu_adv), // IMMU advance
    .force_off_i                    (immu_force_off), // drop stored "IMMU enable"
    // configuration
    .enable_i                       (immu_enable_i), // IMMU
    .supervisor_mode_i              (supervisor_mode_i), // IMMU
    // address translation
    .virt_addr_i                    (virt_addr), // IMMU
    .virt_addr_fetch_i              (virt_addr_fetch), // IMMU
    .phys_addr_fetch_o              (phys_addr_fetch), // IMMU
    // flags
    .cache_inhibit_o                (immu_cache_inhibit), // IMMU
    .tlb_miss_o                     (immu_tlb_miss), // IMMU
    .pagefault_o                    (immu_pagefault), // IMMU
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
    .spr_bus_stb_i                  (spr_bus_ifetch_stb), // IMMU
    .spr_bus_dat_i                  (spr_bus_dat_i), // IMMU
    .spr_bus_dat_o                  (spr_bus_dat_immu_o), // IMMU
    .spr_bus_ack_o                  (spr_bus_ack_immu_o) // IMMU
  );

endmodule // mor1kx_fetch_marocchino
