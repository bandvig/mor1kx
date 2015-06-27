/* ****************************************************************************
  This Source Code Form is subject to the terms of the
  Open Hardware Description License, v. 1.0. If a copy
  of the OHDL was not distributed with this file, You
  can obtain one at http://juliusbaxter.net/ohdl/ohdl.txt

  Description: mor1kx fetch/address stage unit for MAROCCHINO pipeline

  basically an interface to the ibus/icache subsystem that can react to
  exception and branch signals.

  pipelined version of mor1kx_fetch_cappuccino

  Copyright (C) 2012 Authors

  Author(s): Julius Baxter <juliusbaxter@gmail.com>
             Stefan Kristiansson <stefan.kristiansson@saunalahti.fi>

  Copyright (C) 2015 Authors

  Author(s): Andrey Bacherov <avbacherov@opencores.org>

***************************************************************************** */

`include "mor1kx-defines.v"

module mor1kx_fetch_marocchino
#(
  parameter OPTION_OPERAND_WIDTH       = 32,
  parameter OPTION_RESET_PC            = {{(OPTION_OPERAND_WIDTH-13){1'b0}},
                                          `OR1K_RESET_VECTOR,8'd0},
  parameter OPTION_RF_ADDR_WIDTH       =  5,
  // cache configuration
  parameter OPTION_ICACHE_BLOCK_WIDTH  =  5,
  parameter OPTION_ICACHE_SET_WIDTH    =  9,
  parameter OPTION_ICACHE_WAYS         =  2,
  parameter OPTION_ICACHE_LIMIT_WIDTH  = 32,
  // mmu configuration
  parameter FEATURE_IMMU_HW_TLB_RELOAD = "NONE",
  parameter OPTION_IMMU_SET_WIDTH      = 6,
  parameter OPTION_IMMU_WAYS           = 1
)
(
  // clock and reset
  input                                 clk,
  input                                 rst,

  // pipeline control
  input                                 padv_i,
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
  output                                ibus_req_o,
  output     [OPTION_OPERAND_WIDTH-1:0] ibus_adr_o,
  output                                ibus_burst_o,

  // branch/jump control transfer
  input                                 decode_branch_i,
  input      [OPTION_OPERAND_WIDTH-1:0] decode_branch_target_i,
  input                                 decode_op_brcond_i,
  input                                 branch_mispredict_i,
  input      [OPTION_OPERAND_WIDTH-1:0] execute_mispredict_target_i,
  // exception/rfe control transfer
  input                                 doing_rfe_i,
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
  output reg                            fetch_valid_o,
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

  /* Stage #1: ?? */

   // registers
   reg [OPTION_OPERAND_WIDTH-1:0]     pc_fetch;
   reg [OPTION_OPERAND_WIDTH-1:0]     pc_addr;
   reg                 ctrl_branch_exception_r;

   wire             bus_access_done;
   wire             ctrl_branch_exception_edge;
   wire                stall_fetch_valid;
   wire             addr_valid;
   reg                 flush;
   wire                flushing;

   reg                 nop_ack;

   reg                 imem_err;
   wire             imem_ack;
   wire [`OR1K_INSN_WIDTH-1:0]        imem_dat;

   wire             ic_ack;
   wire [`OR1K_INSN_WIDTH-1:0]        ic_dat;

   wire             ic_req;
   wire             ic_refill_allowed;
   wire             ic_refill;
   wire             ic_refill_req;
   wire             ic_refill_done;
   wire             ic_invalidate;
   wire [OPTION_OPERAND_WIDTH-1:0]    ic_addr;
   wire [OPTION_OPERAND_WIDTH-1:0]    ic_addr_match;

   wire             ic_access;

   reg                 ic_enable_r;
   wire             ic_enabled;

   wire [OPTION_OPERAND_WIDTH-1:0]    immu_phys_addr;
   wire             immu_cache_inhibit;
   wire             pagefault;
   wire             tlb_miss;
   wire             except_itlb_miss;
   wire             except_ipagefault;

   wire             immu_busy;

   wire             tlb_reload_req;
   reg                 tlb_reload_ack;
   wire [OPTION_OPERAND_WIDTH-1:0]    tlb_reload_addr;
   reg [OPTION_OPERAND_WIDTH-1:0]     tlb_reload_data;
   wire             tlb_reload_pagefault;
   wire             tlb_reload_busy;

   reg                 fetching_brcond;
   reg                 fetching_mispredicted_branch;
   wire             mispredict_stall;

   reg                 exception_while_tlb_reload;
   wire             except_ipagefault_clear;

   assign bus_access_done = (imem_ack | imem_err | nop_ack) & !immu_busy &
             !tlb_reload_busy;
   assign ctrl_branch_exception_edge = ctrl_branch_exception_i &
                   !ctrl_branch_exception_r;

   /* used to keep fetch_valid_o high during stall */
   assign stall_fetch_valid = !padv_i & fetch_valid_o;

   assign addr_valid = bus_access_done & padv_i &
             !(except_itlb_miss | except_ipagefault) |
             dcod_except_itlb_miss_o & ctrl_branch_exception_i |
             dcod_except_ipagefault_o & ctrl_branch_exception_i |
             doing_rfe_i;

   assign except_itlb_miss = tlb_miss & immu_enable_i & bus_access_done &
              !mispredict_stall & !doing_rfe_i;
   assign except_ipagefault = pagefault & immu_enable_i & bus_access_done &
               !mispredict_stall & !doing_rfe_i |
               tlb_reload_pagefault;

   assign fetch_rfa_adr_o = imem_dat[`OR1K_RA_SELECT];
   assign fetch_rfb_adr_o = imem_dat[`OR1K_RB_SELECT];
   assign fetch_rf_adr_valid_o = bus_access_done & padv_i;

   // Signal to indicate that the ongoing bus access should be flushed
   always @(posedge clk `OR_ASYNC_RST)
     if (rst)
       flush <= 0;
     else if (bus_access_done & padv_i | du_restart_i)
       flush <= 0;
     else if (pipeline_flush_i)
       flush <= 1;

   // pipeline_flush_i comes on the same edge as branch_except_occur during
   // rfe, but on an edge later when an exception occurs, but we always need
   // to keep on flushing when the branch signal comes in.
   assign flushing = pipeline_flush_i | ctrl_branch_exception_edge | flush;

   // Branch misprediction stall logic
   always @(posedge clk `OR_ASYNC_RST)
     if (rst)
       fetching_brcond <= 0;
     else if (pipeline_flush_i)
       fetching_brcond <= 0;
     else if (decode_op_brcond_i & addr_valid)
       fetching_brcond <= 1;
     else if (bus_access_done & padv_i | du_restart_i)
       fetching_brcond <= 0;

   always @(posedge clk `OR_ASYNC_RST)
     if (rst)
       fetching_mispredicted_branch <= 0;
     else if (pipeline_flush_i)
       fetching_mispredicted_branch <= 0;
     else if (bus_access_done & padv_i | du_restart_i)
       fetching_mispredicted_branch <= 0;
     else if (fetching_brcond & branch_mispredict_i & padv_i)
       fetching_mispredicted_branch <= 1;

   assign mispredict_stall = fetching_mispredicted_branch |
              branch_mispredict_i & fetching_brcond;

   always @(posedge clk `OR_ASYNC_RST)
     if (rst)
       ctrl_branch_exception_r <= 1'b0;
     else
       ctrl_branch_exception_r <= ctrl_branch_exception_i;

   // calculate address stage pc
   always @(*)
      if (rst)
   pc_addr = OPTION_RESET_PC;
      else if (du_restart_i)
   pc_addr = du_restart_pc_i;
      else if (ctrl_branch_exception_i & !fetch_exception_taken_o)
   pc_addr = ctrl_branch_except_pc_i;
      else if (branch_mispredict_i | fetching_mispredicted_branch)
   pc_addr = execute_mispredict_target_i;
      else if (decode_branch_i)
   pc_addr = decode_branch_target_i;
      else
   pc_addr = pc_fetch + 4;

   // Register fetch pc from address stage
   always @(posedge clk `OR_ASYNC_RST)
      if (rst)
   pc_fetch <= OPTION_RESET_PC;
      else if (addr_valid | du_restart_i)
   pc_fetch <= pc_addr;

   // fetch_exception_taken_o generation
   always @(posedge clk `OR_ASYNC_RST)
     if (rst)
       fetch_exception_taken_o <= 1'b0;
     else if (fetch_exception_taken_o)
       fetch_exception_taken_o <= 1'b0;
     else if (ctrl_branch_exception_i & bus_access_done & padv_i)
       fetch_exception_taken_o <= 1'b1;
     else
       fetch_exception_taken_o <= 1'b0;

   // fetch_valid_o generation
   always @(posedge clk `OR_ASYNC_RST)
     if (rst)
       fetch_valid_o <= 1'b0;
     else if (pipeline_flush_i)
       fetch_valid_o <= 1'b0;
     else if (bus_access_done & padv_i & !mispredict_stall & !immu_busy &
         !tlb_reload_busy | stall_fetch_valid)
       fetch_valid_o <= 1'b1;
     else
       fetch_valid_o <= 1'b0;

   // Register instruction coming in
   always @(posedge clk `OR_ASYNC_RST)
     if (rst)
       dcod_insn_o <= {`OR1K_OPCODE_NOP,26'd0};
     else if (imem_err | flushing)
       dcod_insn_o <= {`OR1K_OPCODE_NOP,26'd0};
     else if (bus_access_done & padv_i & !mispredict_stall)
       dcod_insn_o <= imem_dat;

   // Register PC for later stages
   always @(posedge clk)
     if (bus_access_done & padv_i & !mispredict_stall)
       pc_decode_o <= pc_fetch;

   always @(posedge clk `OR_ASYNC_RST)
     if (rst)
       dcod_except_ibus_err_o <= 0;
     else if (du_restart_i)
       dcod_except_ibus_err_o <= 0;
     else if (imem_err)
       dcod_except_ibus_err_o <= 1;
     else if (dcod_except_ibus_err_o & ctrl_branch_exception_i)
       dcod_except_ibus_err_o <= 0;

   always @(posedge clk `OR_ASYNC_RST)
     if (rst)
       dcod_except_itlb_miss_o <= 0;
     else if (du_restart_i)
       dcod_except_itlb_miss_o <= 0;
     else if (tlb_reload_busy)
       dcod_except_itlb_miss_o <= 0;
     else if (except_itlb_miss)
       dcod_except_itlb_miss_o <= 1;
     else if (dcod_except_itlb_miss_o & ctrl_branch_exception_i)
       dcod_except_itlb_miss_o <= 0;

   assign except_ipagefault_clear = dcod_except_ipagefault_o &
                ctrl_branch_exception_i;

   always @(posedge clk `OR_ASYNC_RST)
     if (rst)
       dcod_except_ipagefault_o <= 0;
     else if (du_restart_i)
       dcod_except_ipagefault_o <= 0;
     else if (except_ipagefault)
       dcod_except_ipagefault_o <= 1;
     else if (except_ipagefault_clear)
       dcod_except_ipagefault_o <= 0;

   // Bus access logic
   localparam [2:0]
     IDLE      = 0,
     READ      = 1,
     TLB_RELOAD      = 2,
     IC_REFILL    = 3;

   reg [2:0] state;

   reg [OPTION_OPERAND_WIDTH-1:0] ibus_adr;
   wire [OPTION_OPERAND_WIDTH-1:0] next_ibus_adr;
   reg [`OR1K_INSN_WIDTH-1:0]      ibus_dat;
   reg              ibus_req;
   reg              ibus_ack;

   wire          ibus_access;

   //
   // Under certain circumstances, there is a need to insert an nop
   // into the pipeline in order for it to move forward.
   // Here those conditions are handled and an acknowledged signal
   // is generated.
   //
   always @(posedge clk `OR_ASYNC_RST)
     if (rst)
       nop_ack <= 0;
     else
       nop_ack <= padv_i & !bus_access_done & !(ibus_req & ibus_access) &
        ((immu_enable_i & (tlb_miss | pagefault) &
          !tlb_reload_busy) |
         ctrl_branch_exception_edge & !tlb_reload_busy |
         exception_while_tlb_reload & !tlb_reload_busy |
         tlb_reload_pagefault |
         mispredict_stall);

   assign ibus_access = (!ic_access | tlb_reload_busy | ic_invalidate) &
         !ic_refill |
         (state != IDLE) & (state != IC_REFILL) |
         ibus_ack;
   assign imem_ack = ibus_access ? ibus_ack : ic_ack;
   assign imem_dat = (nop_ack | except_itlb_miss | except_ipagefault) ?
           {`OR1K_OPCODE_NOP,26'd0} :
           ibus_access ? ibus_dat : ic_dat;
   assign ibus_adr_o = ibus_adr;
   assign ibus_req_o = ibus_req;
   assign ibus_burst_o = !ibus_access & ic_refill & !ic_refill_done;

   assign next_ibus_adr = (OPTION_ICACHE_BLOCK_WIDTH == 5) ?
           {ibus_adr[31:5], ibus_adr[4:0] + 5'd4} : // 32 byte
           {ibus_adr[31:4], ibus_adr[3:0] + 4'd4};  // 16 byte

   always @(posedge clk `OR_ASYNC_RST)
     if (rst)
       imem_err <= 0;
     else
       imem_err <= ibus_err_i;

   always @(posedge clk) begin
      ibus_ack <= 0;
      exception_while_tlb_reload <= 0;
      tlb_reload_ack <= 0;

      case (state)
   IDLE: begin
      ibus_req <= 0;
      if (padv_i & ibus_access & !ibus_ack & !imem_err & !nop_ack) begin
         if (tlb_reload_req) begin
       ibus_adr <= tlb_reload_addr;
       ibus_req <= 1;
       state <= TLB_RELOAD;
         end else if (immu_enable_i) begin
       ibus_adr <= immu_phys_addr;
       if (!tlb_miss & !pagefault & !immu_busy) begin
          ibus_req <= 1;
          state <= READ;
       end
         end else if (!ctrl_branch_exception_i | doing_rfe_i) begin
       ibus_adr <= pc_fetch;
       ibus_req <= 1;
       state <= READ;
         end
      end else if (ic_refill_req) begin
         ibus_adr <= ic_addr_match;
         ibus_req <= 1;
         state <= IC_REFILL;
      end
   end

   IC_REFILL: begin
      ibus_req <= 1;
      if (ibus_ack_i) begin
         ibus_adr <= next_ibus_adr;
         if (ic_refill_done) begin
       ibus_req <= 0;
       state <= IDLE;
         end
      end
   end

   READ: begin
      ibus_ack <= ibus_ack_i;
      ibus_dat <= ibus_dat_i;
      if (ibus_ack_i | ibus_err_i) begin
         ibus_req <= 0;
         state <= IDLE;
      end
   end

   TLB_RELOAD: begin
      if (ctrl_branch_exception_i)
        exception_while_tlb_reload <= 1;

      ibus_adr <= tlb_reload_addr;
      tlb_reload_data <= ibus_dat_i;
      tlb_reload_ack <= ibus_ack_i & tlb_reload_req;

      if (!tlb_reload_req)
        state <= IDLE;

      ibus_req <= tlb_reload_req;
      if (ibus_ack_i | tlb_reload_ack)
        ibus_req <= 0;
   end

   default:
     state <= IDLE;
      endcase // case (state)

      if (rst) begin
    ibus_req <= 0;
    state <= IDLE;
      end
   end

   always @(posedge clk `OR_ASYNC_RST)
     if (rst)
       ic_enable_r <= 0;
     else if (ic_enable_i & !ibus_req)
       ic_enable_r <= 1;
     else if (!ic_enable_i & !ic_refill)
       ic_enable_r <= 0;

   assign ic_enabled = ic_enable_i & ic_enable_r;
   assign ic_addr = (addr_valid | du_restart_i) ? pc_addr : pc_fetch;
   assign ic_addr_match = immu_enable_i ? immu_phys_addr : pc_fetch;
   assign ic_refill_allowed = (!((tlb_miss | pagefault) & immu_enable_i) &
               !ctrl_branch_exception_i & !pipeline_flush_i &
               !mispredict_stall | doing_rfe_i) &
               !tlb_reload_busy & !immu_busy;

   assign ic_req = padv_i & !dcod_except_ibus_err_o &
         !dcod_except_itlb_miss_o & !except_itlb_miss &
         !dcod_except_ipagefault_o & !except_ipagefault &
         ic_access & ic_refill_allowed;

  /* MMU related controls and signals */

  // Flag to take into accaunt address translation.
  reg immu_enabled_r; 

  // mmu's regular output
  wire                            immu_busy; // process SPR request
  wire [OPTION_OPERAND_WIDTH-1:0] immu_phys_addr;
  wire                            immu_cache_inhibit;
  wire                            immu_tlb_miss;
  wire                            immu_pagefault;
  // mmu exceptions (valid for enabled mmu only)
  wire                            except_itlb_miss;
  wire                            except_ipagefault;
  wire                            except_immu; // any mmu exception

  // HW reload TLB related (MAROCCHINO_TODO : not implemented yet)
  wire                            tlb_reload_req;
  reg                             tlb_reload_ack;
  wire [OPTION_OPERAND_WIDTH-1:0] tlb_reload_addr;
  reg  [OPTION_OPERAND_WIDTH-1:0] tlb_reload_data;
  wire                            tlb_reload_pagefault;
  wire                            tlb_reload_busy;


  /* ICACHE related controls */
  
  // Flags to access ICACHE first
  reg  ic_enabled_r;
  wire ic_access;
  //   Hack? Work around MMU?
  // Today the thing is actual for DCACHE only.
  // Addresses 0x8??????? are treated as non-cacheble regardless MMU's flag.
  wire ic_check_limit_width;

  // ICACHE <-> IBUS state machine
  wire ic_cpu_req;
  wire ic_refill_allowed;
  wire ic_refill;
  wire ic_refill_req;
  wire ic_refill_done;
  wire ic_invalidate;

  // ICACHE result
  wire                        ic_ack;
  wire [`OR1K_INSN_WIDTH-1:0] ic_dat;


  /* IBUS access state machine controls */

  localparam [2:0]  IDLE       = 0,
                    READ       = 1,
                    TLB_RELOAD = 2,
                    IC_REFILL  = 3;

  reg [2:0] state;

  reg                             ibus_req_r;
  reg                             ibus_err_r;
  reg  [OPTION_OPERAND_WIDTH-1:0] ibus_adr_r;
  wire [OPTION_OPERAND_WIDTH-1:0] next_ibus_adr;

  wire                            ibus_access;


  /* Combined ICACHE/IBUS result */

  // ACK/DATA
  wire                        imem_ack;
  wire [`OR1K_INSN_WIDTH-1:0] imem_dat;


  /* declare wires & registers are common for FETCH pipe */

  // The flag: (a) stalls stages #1 & #2
  //           (b) invalidates output of stage #3
  wire s1s2_stall; 


  /***********************/
  /* Stage #1: PC update */
  /***********************/


  reg [OPTION_OPERAND_WIDTH-1:0] s1o_pc;

  wire [OPTION_OPERAND_WIDTH-1:0] s1t_pc_mux =
    du_restart_i                                           ? du_restart_pc_i :
    (ctrl_branch_exception_i & (~fetch_exception_taken_o)) ? ctrl_branch_except_pc_i :
    (branch_mispredict_i | fetching_mispredicted_branch)   ? execute_mispredict_target_i :
    decode_branch_i                                        ? decode_branch_target_i :
                                                             (s1o_pc + 4);

  always @(posedge clk `OR_ASYNC_RST) begin
    if (rst)
      s1o_pc <= OPTION_RESET_PC;
    else if (padv_i & (~s1s2_stall) ??)
      s1o_pc <= s1t_pc_mux;
  end // @ clock

  // 1-clock flag indicating new valid request on stage #1 output
  reg s1o_new_addr;
  always @(posedge clk `OR_ASYNC_RST) begin
    if (rst)
      s1o_new_addr <= 1'b0;
    else if (padv_i & (~s1s2_stall) & ~flush ??)
      s1o_new_addr <= 1'b1;
    else
      s1o_new_addr <= 1'b0;
  end // @ clock

  


  /**********************************/
  /* Stage #2: IMMU / ICACHE access */
  /**********************************/ 


  // 1-clock flag indicating that ICACHE/MMU have taken new address
  reg s2r_new_addr;
  // ICACHE/IMMU match address
  reg [OPTION_OPERAND_WIDTH-1:0] s2r_virt_addr_match;
  //   The register operates in the same way
  // as memory blocks in  to provide correct
  // address for comparision on output of ICACHE/MMU memory blocks.
  //   It is also play role of virtual address store to use
  // in cases of cache/mmu miss.
  always @(posedge clk `OR_ASYNC_RST) begin
    if (rst) begin
      s2r_virt_addr_match <= OPTION_RESET_PC;
      s2r_new_addr        <= 1'b0;
    end
    else begin
      s2r_virt_addr_match <= s1o_pc;
      s2r_new_addr        <= s1o_new_addr;
    end
  end // @ clock

  // Also update cache/mmu enable/disable controls here.
  always @(posedge clk `OR_ASYNC_RST) begin
    if (rst) begin
      ic_enabled_r   <= 1'b0;
      immu_enabled_r <= 1'b0;
    end
    else if (s1o_new_addr) begin
      ic_enabled_r   <= ic_enable_i;
      immu_enabled_r <= immu_enable_i;
    end
  end // @ clock

  // ICACHE/IMMU input virtual address
  //   If pipe is stalle for some reason we use "bypassed"
  // address which is equal to one already latched in ICACHE/MMU memory blocks
  wire [OPTION_OPERAND_WIDTH-1:0] s2t_virt_addr_mux =
    s1s2_stall ? s2r_virt_addr_match : s1o_pc;

  // Select physical address depending on IMMU enabled/disabled
  wire [OPTION_OPERAND_WIDTH-1:0] s2t_phys_addr_mux =
    immu_enabled_r ? immu_phys_addr : s2r_virt_addr_match;

  // mmu exceptions (valid for enabled IMMU only)
  assign except_itlb_miss  = immu_enabled_r & immu_tlb_miss;
  assign except_ipagefault = immu_enabled_r & immu_pagefault;
  assign except_immu       = except_itlb_miss | except_ipagefault;


  /**************************/
  /* Stage #3: I-BUS access */
  /**************************/


  // to s3: instruction word
  reg [OPTION_OPERAND_WIDTH-1:0] s3o_insn;
  //   To minimize number of multiplexors we
  // don't to NOP instruction here for invalid cases.
  // Instead we drop valid flag for such cases and NOP invalid
  // instruction on next stage.
  always @(posedge clk `OR_ASYNC_RST) begin
    if (rst)
      s3o_insn <= {`OR1K_OPCODE_NOP,26'd0};
    else if ( flush ???)
      s3o_insn <= {`OR1K_OPCODE_NOP,26'd0};
    else if (padv_i)
      s3o_insn <= ??;
  end // @ clock

  // to s3: program counter
  reg [OPTION_OPERAND_WIDTH-1:0] s3o_pc;
  // to s3: exception flags
  reg s3o_ibus_err;
  reg s3o_itlb_miss;
  reg s3o_ipagefault;
  // PC & Exceptions
  always @(posedge clk `OR_ASYNC_RST) begin
    if (rst) begin
      // program counter
      s3o_pc         <= OPTION_RESET_PC;
      // exception flags
      s3o_ibus_err   <= 1'b0;
      s3o_itlb_miss  <= 1'b0;
      s3o_ipagefault <= 1'b0;
    end
    else if ( flush ???) begin
      // program counter
      s3o_pc         <= OPTION_RESET_PC;
      // exception flags
      s3o_ibus_err   <= 1'b0;
      s3o_itlb_miss  <= 1'b0;
      s3o_ipagefault <= 1'b0;
    end
    else if (padv_i) begin
      // program counter
      s3o_pc         <= s2t_virt_addr_mux;
      // exception flags
      s3o_ibus_err   <= ??;
      s3o_itlb_miss  <= ??;
      s3o_ipagefault <= ??;
    end
  end // @ clock

  // to s3: valid flag
  reg s3o_valid;
  always @(posedge clk `OR_ASYNC_RST) begin
    if (rst)
      s3o_valid <= 1'b0;
    else if ( flush ???)
      s3o_valid <= 1'b0;
    else if (padv_i)
      s3o_valid <= ~s1s2_stall ??;
  end // @ clock


  /*****************************************/
  /* Stage #4: delay slot & output latches */
  /*****************************************/


  wire s4t_insn_mux =
    s3o_valid ? s3o_insn : {`OR1K_OPCODE_NOP,26'd0};

  // to DECODE: instruction word
  always @(posedge clk `OR_ASYNC_RST) begin
    if (rst)
      dcod_insn_o <= {`OR1K_OPCODE_NOP,26'd0};
    else if ( flush ???)
      dcod_insn_o <= {`OR1K_OPCODE_NOP,26'd0};
    else if (padv_i)
      dcod_insn_o <= s4t_insn_mux;
  end // @ clock

  // to DECODE: actual program counter
  always @(posedge clk `OR_ASYNC_RST) begin
    if (rst)
      pc_decode_o <= OPTION_RESET_PC;
    else if (padv_i)
      pc_decode_o <= s3o_pc;
  end // @ clock

  // to DECODE: valid flag
  always @(posedge clk `OR_ASYNC_RST) begin
    if (rst)
      fetch_valid_o <= 1'b0;
    else if ( flush ???)
      fetch_valid_o <= 1'b0;
    else if (padv_i)
      fetch_valid_o <= s3o_valid;
  end // @ clock

  // exceptions
  always @(posedge clk `OR_ASYNC_RST) begin
    if (rst) begin
      dcod_except_ibus_err_o   <= 1'b0;
      dcod_except_itlb_miss_o  <= 1'b0;
      dcod_except_ipagefault_o <= 1'b0;
    end
    else if ( flush ???) begin
      dcod_except_ibus_err_o   <= 1'b0;
      dcod_except_itlb_miss_o  <= 1'b0;
      dcod_except_ipagefault_o <= 1'b0;
    end
    else if (padv_i) begin
      dcod_except_ibus_err_o   <= s3o_ibus_err;
      dcod_except_itlb_miss_o  <= s3o_itlb_miss;
      dcod_except_ipagefault_o <= s3o_ipagefault;
    end
  end // @ clock

  // to RF
  assign fetch_rfa_adr_o      = s3o_insn[`OR1K_RA_SELECT];
  assign fetch_rfb_adr_o      = s3o_insn[`OR1K_RB_SELECT];
  assign fetch_rf_adr_valid_o = padv_i & s3o_valid;


  //----------------------------------------//
  // IBUS/ICACHE <-> FETCH's pipe interface //
  //----------------------------------------//

  // auxiliaries
  wire ic_rdy   = ic_access & ic_ack; // by ICACHE read or re-fill
  wire ibus_rdy = (state == READ) & (ibus_ack_i & ~ibus_err_i);

  wire imem_ack_store   = (ic_rdy | ibus_rdy) & ~padv_i;
  wire imem_ack_instant = (ic_rdy | ibus_rdy) &  padv_i;
  
  wire [`OR1K_INSN_WIDTH-1:0] imem_dat_instant =
    ibus_rdy ? ibus_dat_i : ic_dat;
  
  // ACK/DATA stored till nearest advance
  reg                         imem_ack_r;
  reg  [`OR1K_INSN_WIDTH-1:0] imem_dat_r;
  // ---
  always @(posedge clk `OR_ASYNC_RST) begin
    if (rst) begin
      imem_ack_r <= 1'b0;
      imem_dat_r <= {`OR1K_OPCODE_NOP,26'd0};
    end
    else if (flush ?? | padv_i) begin
      imem_ack_r <= 1'b0;
      imem_dat_r <= {`OR1K_OPCODE_NOP,26'd0};
    end
    else if (imem_ack_store) begin
      imem_ack_r <= 1'b1;
      imem_dat_r <= imem_dat_instant;
    end
  end // @ clock

  // to FETCH's pipe
  assign imem_ack = imem_ack_instant | imem_ack_r;
  assign imem_dat = imem_ack_instant ? imem_dat_instant : imem_dat_r;


  //--------------------//
  // IBUS state machine //
  //--------------------//
 
  // MAROCCHINO_TODO:
  //   The ic_check_limit_width usage isn't harmonized with IMMU on/off.
  // On the other hand it isn't problem for FETCH because
  // ic_check_limit_width == 1 thanks to setting of 
  // OPTION_ICACHE_LIMIT_WIDTH == OPTION_OPERAND_WIDTH from instance of mor1kx.

  assign ibus_access =
    (~ic_enabled_r | (ic_enabled_r & ~ic_check_limit_width)) &
    ~except_immu & ~ibus_err_r;

  assign next_ibus_adr = (OPTION_ICACHE_BLOCK_WIDTH == 5) ?
    {ibus_adr_r[31:5], ibus_adr_r[4:0] + 5'd4} : // 32 byte
    {ibus_adr_r[31:4], ibus_adr_r[3:0] + 4'd4};  // 16 byte

  // store IBUS error for exception processing
  always @(posedge clk `OR_ASYNC_RST) begin
    if (rst)
      ibus_err_r <= 1'b0;
    else if (flush ??)
      ibus_err_r <= 1'b0;
    else if (ibus_req_r & ibus_err_i)
      ibus_err_r <= 1'b1;
  end // @ clock

  // store IBUS ack

  // state machine
  always @(posedge clk `OR_ASYNC_RST) begin
    // states
    case (state)
      IDLE: begin
        ibus_req_r <= 1'b0;
        if (ic_access & ic_refill_req) begin
          ibus_req_r <= 1'b1;
          ibus_adr_r <= s2t_phys_addr_mux;
          state      <= IC_REFILL;
        end
        else if (ibus_access & s2r_new_addr) begin
          ibus_req_r <= 1'b1;
          ibus_adr_r <= s2t_phys_addr_mux;
          state      <= READ;
        end
      end // idle

      IC_REFILL: begin
        ibus_req_r <= 1'b1;
        if (ibus_ack_i) begin // MAROCCHINO_TODO: bus error?
          ibus_adr_r <= next_ibus_adr;
          if (ic_refill_done) begin
            ibus_req_r <= 1'b0;
            state      <= IDLE;
          end
        end
      end // ic-refill

      READ: begin
        if (ibus_ack_i | ibus_err_i) begin
          ibus_req_r <= 1'b0;
          state      <= IDLE;
        end
      end // read

      default: state <= IDLE;
    endcase // case (state)

    if (rst) begin
      ibus_req_r <= 1'b0;
      state      <= IDLE;
    end
  end // @ clock

  // to WBUS bridge
  assign ibus_adr_o   = ibus_adr_r;
  assign ibus_req_o   = ibus_req_r;
  assign ibus_burst_o = (state == IC_REFILL) & ~ic_refill_done;


  //-------------------//
  // Instance of cache //
  //-------------------//
generate : icache_gen
  if (OPTION_ICACHE_LIMIT_WIDTH == OPTION_OPERAND_WIDTH)
    assign ic_check_limit_width = 1'b1;
  else if (OPTION_ICACHE_LIMIT_WIDTH < OPTION_OPERAND_WIDTH)
    assign ic_check_limit_width = 
      (s2t_phys_addr_mux[OPTION_OPERAND_WIDTH-1:OPTION_ICACHE_LIMIT_WIDTH] == 0);
  else begin
    initial begin
      $display("ERROR: OPTION_ICACHE_LIMIT_WIDTH > OPTION_OPERAND_WIDTH");
      $finish();
    end
  end
endgenerate

  assign ic_access =
    ic_enabled_r & ic_check_limit_width &
    ~(immu_enabled_r & immu_cache_inhibit) &
    ~except_immu & ~ibus_err_r;

  mor1kx_icache
  #(
    .OPTION_ICACHE_BLOCK_WIDTH(OPTION_ICACHE_BLOCK_WIDTH),
    .OPTION_ICACHE_SET_WIDTH(OPTION_ICACHE_SET_WIDTH),
    .OPTION_ICACHE_WAYS(OPTION_ICACHE_WAYS),
    .OPTION_ICACHE_LIMIT_WIDTH(OPTION_ICACHE_LIMIT_WIDTH)
  )
  mor1kx_icache
  (
    .clk                 (clk),
    .rst                 (rst),
    // Outputs
    .refill_o            (ic_refill), // ICACHE
    .refill_req_o        (ic_refill_req), // ICACHE
    .refill_done_o       (ic_refill_done), // ICACHE
    .invalidate_o        (ic_invalidate), // ICACHE
    .cpu_ack_o           (ic_ack), // ICACHE
    .cpu_dat_o           (ic_dat[OPTION_OPERAND_WIDTH-1:0]), // ICACHE
    // Inputs
    .ic_imem_err_i       (imem_err), // ICACHE
    .ic_access_i         (ic_access), // ICACHE
    .cpu_adr_i           (s2t_virt_addr_mux), // ICACHE (was: ic_addr)
    .cpu_adr_match_i     (s2t_phys_addr_mux), // ICACHE (was: ic_addr_match)
    .cpu_req_i           (ic_cpu_req), // ICACHE (was: ic_req)
    .wradr_i             (ibus_adr), // ICACHE
    .wrdat_i             (ibus_dat_i), // ICACHE
    .we_i                (ibus_ack_i), // ICACHE
    // SPR bus
    .spr_bus_addr_i      (spr_bus_addr_i[15:0]), // ICACHE
    .spr_bus_we_i        (spr_bus_we_i), // ICACHE
    .spr_bus_stb_i       (spr_bus_stb_i), // ICACHE
    .spr_bus_dat_i       (spr_bus_dat_i), // ICACHE
    .spr_bus_dat_o       (spr_bus_dat_ic_o), // ICACHE
    .spr_bus_ack_o       (spr_bus_ack_ic_o) // ICACHE
  );


  //-----------------//
  // Instance of MMU //
  //-----------------//

  wire            immu_spr_bus_stb;
  wire            immu_enable;

  assign immu_spr_bus_stb = spr_bus_stb_i & spr_bus_we_i;

  assign immu_enable = immu_enable_i & !pipeline_flush_i & !mispredict_stall;

  mor1kx_immu
  #(
    .FEATURE_IMMU_HW_TLB_RELOAD(FEATURE_IMMU_HW_TLB_RELOAD),
    .OPTION_OPERAND_WIDTH(OPTION_OPERAND_WIDTH),
    .OPTION_IMMU_SET_WIDTH(OPTION_IMMU_SET_WIDTH),
    .OPTION_IMMU_WAYS(OPTION_IMMU_WAYS)
  )
  mor1kx_immu
  (
    .clk                            (clk),
    .rst                            (rst),
    // Outputs
    .busy_o                         (immu_busy), // IMMU
    .phys_addr_o                    (immu_phys_addr), // IMMU
    .cache_inhibit_o                (immu_cache_inhibit), // IMMU
    .tlb_miss_o                     (immu_tlb_miss), // IMMU (was: tlb_miss)
    .pagefault_o                    (immu_pagefault), // IMMU (was: pagefault)
    .tlb_reload_req_o               (tlb_reload_req), // IMMU
    .tlb_reload_addr_o              (tlb_reload_addr), // IMMU
    .tlb_reload_pagefault_o         (tlb_reload_pagefault), // IMMU
    .tlb_reload_busy_o              (tlb_reload_busy), // IMMU
    // Inputs
    .enable_i                       (immu_enable), // IMMU
    .virt_addr_i                    (s2t_virt_addr_mux), // IMMU (was: ic_addr)
    .virt_addr_match_i              (s2r_virt_addr_match), // IMMU (was: pc_fetch)
    .supervisor_mode_i              (supervisor_mode_i), // IMMU
    .tlb_reload_ack_i               (tlb_reload_ack), // IMMU
    .tlb_reload_data_i              (tlb_reload_data), // IMMU
    .tlb_reload_pagefault_clear_i   (except_ipagefault_clear), // IMMU
    // SPR bus
    .spr_bus_addr_i                 (spr_bus_addr_i[15:0]), // IMMU
    .spr_bus_we_i                   (spr_bus_we_i),
    .spr_bus_stb_i                  (immu_spr_bus_stb), // IMMU
    .spr_bus_dat_i                  (spr_bus_dat_i), // IMMU
    .spr_bus_dat_o                  (spr_bus_dat_immu_o), // IMMU
    .spr_bus_ack_o                  (spr_bus_ack_immu_o) // IMMU
  );

endmodule // mor1kx_fetch_marocchino
