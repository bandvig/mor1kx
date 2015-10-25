/* ****************************************************************************
  This Source Code Form is subject to the terms of the
  Open Hardware Description License, v. 1.0. If a copy
  of the OHDL was not distributed with this file, You
  can obtain one at http://juliusbaxter.net/ohdl/ohdl.txt

  Description: mor1kx [O]rder [MAN]ager unit for MAROCCHINO pipeline
    a) collect various state signals from DECODE and EXECUTE modules
    b) analisys of conflicts
    c) generate valid flags for advance DECODE and WB

  Copyright (C) 2015 Andrey Bacherov <avbacherov@opencores.org>

***************************************************************************** */

`include "mor1kx-defines.v"


module mor1kx_oman_marocchino
#(
  parameter OPTION_OPERAND_WIDTH = 32,
  parameter OPTION_RF_ADDR_WIDTH =  5
)
(
  // clock & reset
  input                                 clk,
  input                                 rst,

  // pipeline control
  input                                 padv_decode_i,
  input                                 padv_wb_i,
  input                                 pipeline_flush_i,

  // DECODE non-latched flags to indicate next required unit
  // (The information is stored in order control buffer)
  input                                 dcod_op_pass_exec_i,
  input                                 dcod_op_1clk_i,
  input                                 dcod_op_div_i,
  input                                 dcod_op_mul_i,
  input                                 dcod_op_fp32_arith_i,
  input                                 dcod_op_ls_i,     // load / store (we need store for pushing LSU exceptions)
  input                                 dcod_op_lsu_atomic_i,
  input                                 dcod_op_rfe_i,    // l.rfe

  // DECODE non-latched additional information related instruction
  //  part #1: iformation stored in order control buffer
  input                                 dcod_delay_slot_i, // instruction is in delay slot
  input                                 dcod_flag_wb_i,    // instruction affects comparison flag
  input                                 dcod_carry_wb_i,   // instruction affects carry flag
  input                                 dcod_rf_wb_i,      // instruction generates WB
  input      [OPTION_RF_ADDR_WIDTH-1:0] dcod_rfd_adr_i,    // WB address
  input      [OPTION_OPERAND_WIDTH-1:0] pc_decode_i,       // instruction virtual address
  //  part #2: information required for data dependancy detection
  input                                 dcod_rfa_req_i,    // instruction requires operand A
  input      [OPTION_RF_ADDR_WIDTH-1:0] dcod_rfa_adr_i,    // source of operand A
  input                                 dcod_rfb_req_i,    // instruction requires operand B
  input      [OPTION_RF_ADDR_WIDTH-1:0] dcod_rfb_adr_i,    // source of operand B
  input                                 dcod_flag_req_i,   // need comparison flag (l.cmov)
  input                                 dcod_carry_req_i,  // need carry flag
  input                                 dcod_op_jr_i,      // l.jr/l.jalr require operand B (potentially hazard)
  //  part #3: information required for create enable for
  //           for external (timer/ethernet/uart/etc) interrupts
  input                                 dcod_op_lsu_store_i,
  input                                 dcod_op_mtspr_i,
  //  part #4: for MF(T)SPR processing
  input                                 dcod_op_mfspr_i,

  // collect busy flags from execution module
  input                                 div_busy_i,
  input                                 fp32_arith_busy_i,
  input                                 lsu_busy_i,

  // collect valid flags from execution modules
  input                                 exec_op_1clk_i,
  input                                 div_valid_i,
  input                                 mul_valid_i,
  input                                 fp32_arith_valid_i,
  input                                 lsu_valid_i,
  input                                 lsu_excepts_i,

  // FETCH & DECODE exceptions
  input                                 dcod_except_ibus_err_i,
  input                                 dcod_except_ipagefault_i,
  input                                 dcod_except_itlb_miss_i,
  input                                 dcod_except_ibus_align_i,
  input                                 dcod_except_illegal_i,
  input                                 dcod_except_syscall_i,
  input                                 dcod_except_trap_i,

  // EXECUTE-to-DECODE hazards
  output                                dcod_bubble_o,
  output                                exe2dec_hazard_a_o,
  output                                exe2dec_hazard_b_o,

  // DECODE result could be processed by EXECUTE
  output                                dcod_valid_o,

  // EXECUTE completed (desired unit is ready)
  output                                exec_valid_o,

  // control WB latches of execution modules
  output                                grant_wb_to_1clk_o,
  output                                grant_wb_to_div_o,
  output                                grant_wb_to_mul_o,
  output                                grant_wb_to_fp32_arith_o,
  output                                grant_wb_to_lsu_o,
  // common flag signaling that WB ir required
  output                                do_rf_wb_o,

  // WB outputs
  //  ## instruction related information
  output reg [OPTION_OPERAND_WIDTH-1:0] pc_wb_o,
  output reg                            wb_delay_slot_o,
  output reg [OPTION_RF_ADDR_WIDTH-1:0] wb_rfd_adr_o,
  output reg                            wb_rf_wb_o,
  //  ## RFE processing
  output reg                            wb_op_rfe_o,
  //  ## output exceptions
  output reg                            wb_except_ibus_err_o,
  output reg                            wb_except_ipagefault_o,
  output reg                            wb_except_itlb_miss_o,
  output reg                            wb_except_ibus_align_o,
  output reg                            wb_except_illegal_o,
  output reg                            wb_except_syscall_o,
  output reg                            wb_except_trap_o,
  output reg                            wb_interrupts_en_o
);

  // [O]rder [C]ontrol [B]uffer [T]ap layout
  //  [0...6] Exceptions generated by FETCH & DECODE.
  //          Pay atttention that LSU exceptions go to WB around order control buffer.
  //  [7] "A FETCH or DECODE exception" flag.
  //      FETCH & DECODE exceptions combined by OR to simplify logic for exception analysis.
  localparam  OCBT_FD_AN_EXCEPT_POS   = 7;
  //  Flag that external interrupt is enabled (instruction is re-startable)
  localparam  OCBT_INTERRUPTS_EN_POS  = OCBT_FD_AN_EXCEPT_POS   + 1;
  //  Unit wise requested/ready
  localparam  OCBT_OP_PASS_EXEC_POS   = OCBT_INTERRUPTS_EN_POS  + 1;
  localparam  OCBT_OP_1CLK_POS        = OCBT_OP_PASS_EXEC_POS   + 1;
  localparam  OCBT_OP_DIV_POS         = OCBT_OP_1CLK_POS        + 1;
  localparam  OCBT_OP_MUL_POS         = OCBT_OP_DIV_POS         + 1;
  localparam  OCBT_OP_FP32_POS        = OCBT_OP_MUL_POS         + 1; // arithmetic part only, FP comparison is 1-clock
  localparam  OCBT_OP_LS_POS          = OCBT_OP_FP32_POS        + 1; // load / store (we need it for pushing LSU exceptions)
  localparam  OCBT_OP_LSU_ATOMIC_POS  = OCBT_OP_LS_POS          + 1;
  localparam  OCBT_OP_RFE_POS         = OCBT_OP_LSU_ATOMIC_POS  + 1; // l.rfe
  //  Instruction is in delay slot
  localparam  OCBT_DELAY_SLOT_POS     = OCBT_OP_RFE_POS         + 1;
  //  Instruction affect comparison flag
  localparam  OCBT_FLAG_WB_POS        = OCBT_DELAY_SLOT_POS     + 1;
  //  Instruction affect carry flag
  localparam  OCBT_CARRY_WB_POS       = OCBT_FLAG_WB_POS        + 1;
  //  Instruction generates WB
  localparam  OCBT_RF_WB_POS          = OCBT_CARRY_WB_POS       + 1;
  localparam  OCBT_RFD_ADR_LSB        = OCBT_RF_WB_POS          + 1;
  localparam  OCBT_RFD_ADR_MSB        = OCBT_RF_WB_POS          + OPTION_RF_ADDR_WIDTH;
  //  Program counter
  localparam  OCBT_PC_LSB             = OCBT_RFD_ADR_MSB        + 1;
  localparam  OCBT_PC_MSB             = OCBT_RFD_ADR_MSB        + OPTION_OPERAND_WIDTH;
  //  value of MSB of order control buffer tap
  localparam  OCBT_MSB                = OCBT_PC_MSB;
  localparam  OCBT_WIDTH              = OCBT_MSB                + 1;


  // Flag that istruction is restrartable.
  // Instructions which are not restartable:
  //     "invalid" (empty FETCH result),
  //     l.mtspr (change internal CPU control registers)
  //     l.rfe (cause return from exception process with serious
  //            changing CPU state).
  //   Note #1 we just not run execution for "invalid" command (CTRL), so
  // such "commands" don't achieve WB where exceptions are processed.
  //   Note #2 l.rfe is a special case. We push pipe full of rfes.
  // The reason for this is that we need the rfe to reach WB stage
  // so it will cause the branch. It will clear itself by the
  // pipeline_flush_i that the rfe will generate.
  //   MAROCCHINO_TODO: think about l.msync and store
  wire interrupts_en = ~dcod_op_mtspr_i & ~dcod_op_rfe_i & ~dcod_op_lsu_store_i;


  // Combine FETCH related exceptions
  wire fetch_an_except = dcod_except_ibus_err_i  | dcod_except_ipagefault_i |
                         dcod_except_itlb_miss_i | dcod_except_ibus_align_i;
  // Combine DECODE related exceptions
  wire dcod_an_except = dcod_except_illegal_i | dcod_except_syscall_i |
                        dcod_except_trap_i;


  // input pack
  wire  [OCBT_MSB:0] ocbi;
  assign ocbi = { // various instruction related information
                  pc_decode_i,       // instruction virtual address
                  dcod_rfd_adr_i,    // WB address
                  dcod_rf_wb_i,      // instruction generates WB
                  dcod_carry_wb_i,   // istruction affects carry flag
                  dcod_flag_wb_i,    // istruction affects comparison flag
                  dcod_delay_slot_i, // istruction is in delay slot
                  // unit that must be granted for WB
                  dcod_op_rfe_i,     // l.rfe
                  dcod_op_lsu_atomic_i,
                  dcod_op_ls_i,      // load / store (we need it for pushing LSU exceptions)
                  dcod_op_fp32_arith_i,
                  dcod_op_mul_i,
                  dcod_op_div_i,
                  dcod_op_1clk_i,
                  dcod_op_pass_exec_i,
                  // Flag that istruction is restartable
                  interrupts_en,
                  // combined FETCH & DECODE exceptions flag
                  (fetch_an_except | dcod_an_except),
                  // FETCH & DECODE exceptions
                  dcod_except_ibus_err_i,
                  dcod_except_ipagefault_i,
                  dcod_except_itlb_miss_i,
                  dcod_except_ibus_align_i,
                  dcod_except_illegal_i,
                  dcod_except_syscall_i,
                  dcod_except_trap_i };

  // for 1-st step only one tap is implemented
  reg [OCBT_MSB:0] ocbo00_r;
  reg              ocb_empty;
  // ---
  always @(posedge clk `OR_ASYNC_RST) begin
    if (rst) begin
      ocbo00_r  <= {OCBT_WIDTH{1'b0}};
      ocb_empty <= 1'b1;
    end
    else if (pipeline_flush_i) begin
      ocbo00_r  <= {OCBT_WIDTH{1'b0}};
      ocb_empty <= 1'b1;
    end
    else if (padv_decode_i) begin
      ocbo00_r  <= ocbi;
      ocb_empty <= 1'b0;
    end
    else if (padv_wb_i) begin
      ocbo00_r  <= {OCBT_WIDTH{1'b0}};
      ocb_empty <= 1'b1;
    end
  end // @ clock

  // "buffer is full" flag
  // MAROCCHINO_TODO: it has different sense for long buffer
  wire ocb_full = ~ocb_empty;

  // Grant WB-access to units
  assign grant_wb_to_1clk_o        = ocbo00_r[OCBT_OP_1CLK_POS];
  assign grant_wb_to_div_o         = ocbo00_r[OCBT_OP_DIV_POS];
  assign grant_wb_to_mul_o         = ocbo00_r[OCBT_OP_MUL_POS];
  assign grant_wb_to_fp32_arith_o  = ocbo00_r[OCBT_OP_FP32_POS];
  assign grant_wb_to_lsu_o         = ocbo00_r[OCBT_OP_LS_POS];
  // common flag signaling that WB is required
  assign do_rf_wb_o                = ocbo00_r[OCBT_RF_WB_POS];

  // EXECUTE-to-DECODE hazards
  //  # WB address and flag
  wire [OPTION_RF_ADDR_WIDTH-1:0] exec_rfd_adr = ocbo00_r[OCBT_RFD_ADR_MSB:OCBT_RFD_ADR_LSB];
  wire                            exec_rf_wb   = ocbo00_r[OCBT_RF_WB_POS];
  //  # Hazard by operand A
  assign exe2dec_hazard_a_o = exec_rf_wb & dcod_rfa_req_i & (exec_rfd_adr == dcod_rfa_adr_i);
  //  # Hazard by operand B
  assign exe2dec_hazard_b_o = exec_rf_wb & dcod_rfb_req_i & (exec_rfd_adr == dcod_rfb_adr_i);


  //   Bubble is just used to block FETCH advance (CTRL).
  //   Detect the situation where there is a jump to register in decode
  // stage and an instruction in execute stage that will write to that
  // register.
  //   A bubble is also inserted when an rfe instruction is in decode stage,
  // the main purpose of this is to stall fetch while the rfe is propagating
  // up to WB stage.
  //   By DECODE exceptions (FETCH exceptions block it in FETCH itself)
  assign dcod_bubble_o = (dcod_op_jr_i & exe2dec_hazard_b_o) | dcod_op_rfe_i | dcod_an_except;


  // auxiliaries
  wire lsu_valid_or_excepts = lsu_valid_i | lsu_excepts_i;


  //   An execute module is ready and granted access to WB
  //   Instructions l.mf(t)spr have got guaranted WB access because
  // no any new instruction is issued into execution till
  // l.mf(t)spr has been completed. Pay attention that we start
  // l.mf(t)spr ecxecution after completion of all peviously
  // issued instructions only.
  //   l.rfe and FETCH/DECODE exceptions are also should
  // push WB latches
  assign exec_valid_o = ocbo00_r[OCBT_OP_1CLK_POS] |
                        (div_valid_i & ocbo00_r[OCBT_OP_DIV_POS]) |
                        (mul_valid_i & ocbo00_r[OCBT_OP_MUL_POS]) |
                        (fp32_arith_valid_i & ocbo00_r[OCBT_OP_FP32_POS]) |
                        (lsu_valid_or_excepts & ocbo00_r[OCBT_OP_LS_POS]) |
                        ocbo00_r[OCBT_OP_PASS_EXEC_POS] | // also includes l.rfe in the sense
                        ocbo00_r[OCBT_FD_AN_EXCEPT_POS];


  // waiting EXECUTE result (only multicycle instructions make sense)
  wire exec_waiting = (~div_valid_i & ocbo00_r[OCBT_OP_DIV_POS]) |
                      (~mul_valid_i & ocbo00_r[OCBT_OP_MUL_POS]) |
                      (~fp32_arith_valid_i & ocbo00_r[OCBT_OP_FP32_POS]) |
                      (~lsu_valid_or_excepts & ocbo00_r[OCBT_OP_LS_POS]);

  // DECODE stall components
  //  stall by unit usage hazard
  //     (unit could be either busy or waiting for WB access)
  wire stall_by_hazard_u =
    (dcod_op_1clk_i & exec_op_1clk_i & ~ocbo00_r[OCBT_OP_1CLK_POS]) |
    (dcod_op_div_i & (div_busy_i | (div_valid_i & ~ocbo00_r[OCBT_OP_DIV_POS]))) |
    (dcod_op_mul_i & mul_valid_i & ~ocbo00_r[OCBT_OP_MUL_POS]) |
    (dcod_op_fp32_arith_i & (fp32_arith_busy_i | (fp32_arith_valid_i & ~ocbo00_r[OCBT_OP_FP32_POS]))) |
    (dcod_op_ls_i & (lsu_busy_i | (lsu_valid_i & ~ocbo00_r[OCBT_OP_LS_POS])));

  //  stall if OCB is full
  wire stall_by_ocb_full = ocb_full & exec_waiting; // MAROCCHINO_TODO: extend to whole buffer

  //  stall by operand A hazard
  //    hazard has occured inside OCB
  wire ocb_hazard_a = 1'b0; // MAROCCHINO_TODO: makes sense if buffer length > 1
  //    combine with DECODE-to-EXECUTE hazard
  wire stall_by_hazard_a = ocb_hazard_a | (exe2dec_hazard_a_o & exec_waiting);

  //  stall by operand B hazard
  //    hazard has occured inside OCB
  wire ocb_hazard_b = 1'b0; // MAROCCHINO_TODO: makes sense if buffer length > 1
  //    combine with DECODE-to-EXECUTE hazard
  wire stall_by_hazard_b = ocb_hazard_b | (exe2dec_hazard_b_o & (exec_waiting | dcod_op_jr_i));

  //  stall by comparison flag hazard
  //    hazard has occured inside OCB
  wire ocb_flag = 1'b0; // MAROCCHINO_TODO: makes sense if buffer length > 1
  //    waiting completion of atomic instruction (others WB-flag instructions are 1-clk)
  wire flag_waiting = ~lsu_valid_or_excepts & ocbo00_r[OCBT_OP_LSU_ATOMIC_POS];
  //    combine with DECODE-to-EXECUTE hazard
  wire stall_by_flag = dcod_flag_req_i & (ocb_flag | flag_waiting);

  //  stall by carry flag hazard
  //    hazard has occured inside OCB
  wire ocb_carry = 1'b0; // MAROCCHINO_TODO: makes sense if buffer length > 1
  //    waiting completion of DIV instruction (others WB-carry instructions are 1-clk)
  wire carry_waiting = ~div_valid_i & ocbo00_r[OCBT_OP_DIV_POS];
  //    combine with DECODE-to-EXECUTE hazard
  wire stall_by_carry = dcod_carry_req_i & (ocb_carry | carry_waiting);

  //  stall by:
  //    a) MF(T)SPR in decode till and OCB become empty, see here
  //    b) till completion MF(T)SPR, see CTRL
  //       this completion generates padv-wb,
  //       in next turn padv-wb cleans up OCB and restores
  //       instructions issue
  wire stall_by_mXspr = (dcod_op_mtspr_i | dcod_op_mfspr_i) & ~ocb_empty;

  // combine stalls to decode-valid flag
  assign dcod_valid_o = ~stall_by_hazard_u & ~stall_by_ocb_full &
                        ~stall_by_hazard_a & ~stall_by_hazard_b &
                        ~stall_by_flag     & ~stall_by_carry    &
                        ~stall_by_mXspr;


  // an internal exception
  wire pipe_an_except = ocbo00_r[OCBT_FD_AN_EXCEPT_POS] | (ocbo00_r[OCBT_OP_LS_POS] & lsu_excepts_i); 

  // WB: delay slot and wb-request
  always @(posedge clk `OR_ASYNC_RST) begin
    if (rst) begin
      wb_rf_wb_o      <= 1'b0;
      wb_delay_slot_o <= 1'b0;
    end
    else if (pipeline_flush_i) begin
      wb_rf_wb_o      <= 1'b0;
      wb_delay_slot_o <= 1'b0;
    end
    else if (padv_wb_i) begin
      wb_rf_wb_o      <= exec_rf_wb & (~pipe_an_except);
      wb_delay_slot_o <= ocbo00_r[OCBT_DELAY_SLOT_POS];
    end
  end // @clock

  // address of destination register & PC
  always @(posedge clk) begin
    if (padv_wb_i & (~pipeline_flush_i)) begin
      wb_rfd_adr_o <= exec_rfd_adr;
      pc_wb_o      <= ocbo00_r[OCBT_PC_MSB:OCBT_PC_LSB];
    end
  end // @clock

  // WB EXCEPTIONS (excluding LSU's) & RFE
  always @(posedge clk `OR_ASYNC_RST) begin
    if (rst) begin
      // RFE
      wb_op_rfe_o            <= 1'b0;
      // Flag that istruction is restrartable
      wb_interrupts_en_o     <= 1'b0;
      // FETCH/DECODE exceptions
      wb_except_ibus_err_o   <= 1'b0;
      wb_except_ipagefault_o <= 1'b0;
      wb_except_itlb_miss_o  <= 1'b0;
      wb_except_ibus_align_o <= 1'b0;
      wb_except_illegal_o    <= 1'b0;
      wb_except_syscall_o    <= 1'b0;
      wb_except_trap_o       <= 1'b0;
    end
    else if (pipeline_flush_i) begin
      // RFE
      wb_op_rfe_o            <= 1'b0;
      // Flag that istruction is restrartable
      wb_interrupts_en_o     <= 1'b0;
      // FETCH/DECODE exceptions
      wb_except_ibus_err_o   <= 1'b0;
      wb_except_ipagefault_o <= 1'b0;
      wb_except_itlb_miss_o  <= 1'b0;
      wb_except_ibus_align_o <= 1'b0;
      wb_except_illegal_o    <= 1'b0;
      wb_except_syscall_o    <= 1'b0;
      wb_except_trap_o       <= 1'b0;
    end
    else if (padv_wb_i) begin
      // RFE
      wb_op_rfe_o            <= ocbo00_r[OCBT_OP_RFE_POS];
      // Flag that istruction is restrartable
      wb_interrupts_en_o     <= ocbo00_r[OCBT_INTERRUPTS_EN_POS];
      // FETCH/DECODE exceptions
      wb_except_ibus_err_o   <= ocbo00_r[6];
      wb_except_ipagefault_o <= ocbo00_r[5];
      wb_except_itlb_miss_o  <= ocbo00_r[4];
      wb_except_ibus_align_o <= ocbo00_r[3];
      wb_except_illegal_o    <= ocbo00_r[2];
      wb_except_syscall_o    <= ocbo00_r[1];
      wb_except_trap_o       <= ocbo00_r[0];
    end
  end // @clock

endmodule // mor1kx_oman_marocchino
