/////////////////////////////////////////////////////////////////////
//                                                                 //
//  mor1kx_oman_marocchino                                         //
//                                                                 //
//  Description: mor1kx [O]rder [MAN]ager unit                     //
//               for MAROCCHINO pipeline                           //
//    a) collect various state signals from DECODE                 //
//       and EXECUTE modules                                       //
//    b) analisys of conflicts                                     //
//    c) generate valid flags for advance DECODE and WB            //
//                                                                 //
/////////////////////////////////////////////////////////////////////
//                                                                 //
//   Copyright (C) 2015 Andrey Bacherov                            //
//                      avbacherov@opencores.org                   //
//                                                                 //
//      This Source Code Form is subject to the terms of the       //
//      Open Hardware Description License, v. 1.0. If a copy       //
//      of the OHDL was not distributed with this file, You        //
//      can obtain one at http://juliusbaxter.net/ohdl/ohdl.txt    //
//                                                                 //
/////////////////////////////////////////////////////////////////////

`include "mor1kx-defines.v"



/////////////////////////////////////////////////////////////////////
//  Single cell of [R]egisters [A]llocation [T]able                //
/////////////////////////////////////////////////////////////////////
module rat_cell
#(
  parameter OPTION_RF_ADDR_WIDTH =  5,
  parameter DEST_EXT_ADDR_WIDTH  =  3,
  parameter GPR_ADDR             =  0
)
(
  // clock & reset
  input                                cpu_clk,
  input                                cpu_rst,

  // pipeline control
  input                                padv_dcod_i,
  input                                padv_wb_i,
  input                                pipeline_flush_i,

  // input allocation information
  //  # allocated as D1
  input                                ratin_rfd1_wb_i,
  input     [OPTION_RF_ADDR_WIDTH-1:0] ratin_rfd1_adr_i,
  //  # allocated as D2
  input                                ratin_rfd2_wb_i,
  input     [OPTION_RF_ADDR_WIDTH-1:0] ratin_rfd2_adr_i,
  //  # allocation id
  input       [DEST_EXT_ADDR_WIDTH-1:0] next_ext_bits_i,

  // input to clear allocation bits
  input       [DEST_EXT_ADDR_WIDTH-1:0] exec_ext_bits_i,

  // output allocation information
  output reg                            rat_rd1_alloc_o, // allocated by D1
  output reg                            rat_rd2_alloc_o, // allocated by D2
  output reg                            rat_rdx_alloc_o, // allocated by D1/D2
  output reg  [DEST_EXT_ADDR_WIDTH-1:0] rat_ext_bits_o   // allocation ID
);

  localparam [OPTION_RF_ADDR_WIDTH-1:0] GPR_ADR = GPR_ADDR;

  // set allocation flags
  wire set_rd1_alloc = ratin_rfd1_wb_i & (ratin_rfd1_adr_i == GPR_ADR);
  wire set_rd2_alloc = ratin_rfd2_wb_i & (ratin_rfd2_adr_i == GPR_ADR);
  wire set_rdx_alloc = (set_rd1_alloc | set_rd2_alloc);

  // condition to keep allocation flags at write-back
  wire keep_alloc_at_wb    = (exec_ext_bits_i != rat_ext_bits_o);
  // next values of allocation flags at write-back
  wire rat_rd1_alloc_at_wb = rat_rd1_alloc_o & keep_alloc_at_wb;
  wire rat_rd2_alloc_at_wb = rat_rd2_alloc_o & keep_alloc_at_wb;
  wire rat_rdx_alloc_at_wb = rat_rdx_alloc_o & keep_alloc_at_wb;

  // allocation flags
  always @(posedge cpu_clk) begin
    if (cpu_rst | pipeline_flush_i) begin
      rat_rd1_alloc_o <= 1'b0;
      rat_rd2_alloc_o <= 1'b0;
      rat_rdx_alloc_o <= 1'b0;
    end
    else begin
      // synthesis parallel_case full_case
      case ({padv_wb_i,padv_dcod_i})
        // FETCH->DECODE
        2'b01: begin
          rat_rd1_alloc_o <= set_rdx_alloc ? set_rd1_alloc : rat_rd1_alloc_o;
          rat_rd2_alloc_o <= set_rdx_alloc ? set_rd2_alloc : rat_rd2_alloc_o;
          rat_rdx_alloc_o <= set_rdx_alloc ? 1'b1          : rat_rdx_alloc_o;
        end
        // EXECUTE->WB
        2'b10: begin
          rat_rd1_alloc_o <= rat_rd1_alloc_at_wb;
          rat_rd2_alloc_o <= rat_rd2_alloc_at_wb;
          rat_rdx_alloc_o <= rat_rdx_alloc_at_wb;
        end
        // overlapping
        2'b11: begin
          rat_rd1_alloc_o <= set_rdx_alloc ? set_rd1_alloc : rat_rd1_alloc_at_wb;
          rat_rd2_alloc_o <= set_rdx_alloc ? set_rd2_alloc : rat_rd2_alloc_at_wb;
          rat_rdx_alloc_o <= set_rdx_alloc ? 1'b1          : rat_rdx_alloc_at_wb;
        end
        // don't change by default
        default:;
      endcase
    end
  end

  // extention bits
  always @(posedge cpu_clk) begin
    if (cpu_rst)
      rat_ext_bits_o <= {DEST_EXT_ADDR_WIDTH{1'b0}};
    else if (padv_dcod_i)
      rat_ext_bits_o <= set_rdx_alloc ? next_ext_bits_i : rat_ext_bits_o;
  end

endmodule // rat_cell



/////////////////////////////////////////////////////////////////////
//  [O]rder [MAN]ager itself                                       //
/////////////////////////////////////////////////////////////////////

module mor1kx_oman_marocchino
#(
  parameter OPTION_OPERAND_WIDTH = 32,
  parameter OPTION_RF_ADDR_WIDTH =  5,
  parameter DEST_EXT_ADDR_WIDTH  =  3,  // log2(Order Control Buffer depth)
  // branch predictor parameters
  parameter GSHARE_BITS_NUM      = 10
)
(
  // clock & reset
  input                                 cpu_clk,
  input                                 cpu_rst,

  // pipeline control
  input                                 padv_dcod_i,
  input                                 padv_exec_i,
  input                                 padv_wb_i,
  input                                 pipeline_flush_i,

  // fetched instruction is valid
  input                                 fetch_valid_i,
  input                                 fetch_delay_slot_i,

  // for RAT
  //  # allocation SR[F]
  input                                 fetch_flag_wb_i, // any instruction which affects comparison flag
  //  # allocated as D1
  input                                 ratin_rfd1_wb_i,
  input      [OPTION_RF_ADDR_WIDTH-1:0] ratin_rfd1_adr_i,
  //  # allocated as D2
  input                                 ratin_rfd2_wb_i,
  input      [OPTION_RF_ADDR_WIDTH-1:0] ratin_rfd2_adr_i,
  //  # requested operands
  // operand A1
  input                                 ratin_rfa1_req_i,
  input      [OPTION_RF_ADDR_WIDTH-1:0] fetch_rfa1_adr_i,
  // operand B1
  input                                 ratin_rfb1_req_i,
  input      [OPTION_RF_ADDR_WIDTH-1:0] fetch_rfb1_adr_i,
  // operand A2 (for FPU64)
  input                                 ratin_rfa2_req_i,
  input      [OPTION_RF_ADDR_WIDTH-1:0] fetch_rfa2_adr_i,
  // operand B2 (for FPU64)
  input                                 ratin_rfb2_req_i,
  input      [OPTION_RF_ADDR_WIDTH-1:0] fetch_rfb2_adr_i,

  // DECODE flags to indicate next required unit
  // (The information is stored in order control buffer)
  input                                 dcod_op_push_exec_i,
  input                                 dcod_op_push_wb_i,
  input                                 fetch_op_jb_i,
  input                                 dcod_op_1clk_i,
  input                                 dcod_op_div_i,
  input                                 dcod_op_mul_i,
  input                                 dcod_op_fpxx_arith_i,
  input                                 dcod_op_ls_i,     // load / store (we need store for pushing LSU exceptions)
  input                                 dcod_op_rfe_i,    // l.rfe
  // for FPU3264
  input                                 dcod_op_fp64_cmp_i,

  // DECODE additional information related instruction
  //  part #1: information stored in order control buffer
  input      [OPTION_OPERAND_WIDTH-1:0] pc_decode_i,            // instruction virtual address
  input      [OPTION_RF_ADDR_WIDTH-1:0] dcod_rfd1_adr_i,        // WB address
  input                                 dcod_rfd1_wb_i,         // instruction generates WB to D1
  input                                 dcod_carry_wb_i,        // instruction affects carry flag
  input                                 dcod_flag_wb_i,         // any instruction which affects comparison flag
  input                                 dcod_delay_slot_i,      // instruction is in delay slot
  input      [OPTION_RF_ADDR_WIDTH-1:0] dcod_rfd2_adr_i,        // WB2 address for FPU64
  input                                 dcod_rfd2_wb_i,         // instruction generates WB to D2
  //  part #2: information required for create enable for
  //           for external (timer/ethernet/uart/etc) interrupts
  input                                 dcod_op_lsu_store_i,
  input                                 dcod_op_msync_i,
  input                                 dcod_op_mXspr_i,

  // for unit hazard detection
  input                                 op_1clk_free_i,
  input                                 dcod_op_muldiv_i,
  input                                 muldiv_free_i,
  input                                 dcod_op_fpxx_any_i,
  input                                 fpxx_free_i,
  input                                 dcod_op_lsu_any_i, // (load | store | l.msync)
  input                                 lsu_free_i,

  // collect valid flags from execution modules
  input                                 div_valid_i,
  input                                 mul_valid_i,
  input                                 fpxx_arith_valid_i,
  input                                 fp64_cmp_valid_i,
  input                                 lsu_valid_i,

  // FETCH & DECODE exceptions
  input                                 fetch_except_ibus_err_i,
  input                                 fetch_except_ipagefault_i,
  input                                 fetch_except_itlb_miss_i,
  input                                 dcod_except_illegal_i,
  input                                 dcod_except_syscall_i,
  input                                 dcod_except_trap_i,

  // 1-clock "WB to DECODE operand forwarding" flags
  //  # relative operand A1
  output reg                            dcod_wb2dec_d1a1_fwd_o,
  output reg                            dcod_wb2dec_d2a1_fwd_o,
  //  # relative operand B1
  output reg                            dcod_wb2dec_d1b1_fwd_o,
  output reg                            dcod_wb2dec_d2b1_fwd_o,
  //  # relative operand A2
  output reg                            dcod_wb2dec_d1a2_fwd_o,
  output reg                            dcod_wb2dec_d2a2_fwd_o,
  //  # relative operand B2
  output reg                            dcod_wb2dec_d1b2_fwd_o,
  output reg                            dcod_wb2dec_d2b2_fwd_o,

  // OMAN-to-DECODE hazards
  //  # relative operand A1
  output reg                            omn2dec_hazard_d1a1_o,
  output reg                            omn2dec_hazard_d2a1_o,
  output reg                            omn2dec_hazard_dxa1_o,
  output reg  [DEST_EXT_ADDR_WIDTH-1:0] omn2dec_hazard_dxa1_adr_o,
  //  # relative operand B1
  output reg                            omn2dec_hazard_d1b1_o,
  output reg                            omn2dec_hazard_d2b1_o,
  output reg                            omn2dec_hazard_dxb1_o,
  output reg  [DEST_EXT_ADDR_WIDTH-1:0] omn2dec_hazard_dxb1_adr_o,
  //  # relative operand A2
  output reg                            omn2dec_hazard_d1a2_o,
  output reg                            omn2dec_hazard_d2a2_o,
  output reg                            omn2dec_hazard_dxa2_o,
  output reg  [DEST_EXT_ADDR_WIDTH-1:0] omn2dec_hazard_dxa2_adr_o,
  //  # relative operand B2
  output reg                            omn2dec_hazard_d1b2_o,
  output reg                            omn2dec_hazard_d2b2_o,
  output reg                            omn2dec_hazard_dxb2_o,
  output reg  [DEST_EXT_ADDR_WIDTH-1:0] omn2dec_hazard_dxb2_adr_o,

  // DECODE result could be processed by EXECUTE
  output                                dcod_free_o,
  output                                dcod_valid_o,

  // EXECUTE completed (desired unit is ready)
  output                                exec_valid_o,

  // control WB latches of execution modules
  output                                grant_wb_to_1clk_o,
  output                                grant_wb_to_div_o,
  output                                grant_wb_to_mul_o,
  output                                grant_wb_to_fpxx_arith_o,
  output                                grant_wb_to_lsu_o,
  // for FPU64
  output                                grant_wb_to_fp64_cmp_o,

  // Logic to support Jump / Branch taking
  //  # from IFETCH
  //    ## jump/branch variants
  input                                 fetch_op_jimm_i,
  input                                 fetch_op_jr_i,
  input                                 fetch_op_bf_i,
  input                                 fetch_op_bnf_i,
  //    ## "to immediate driven target"
  input      [OPTION_OPERAND_WIDTH-1:0] fetch_to_imm_target_i,
  //  # target for l.jr / l.jalr
  input      [OPTION_OPERAND_WIDTH-1:0] dcod_rfb1_jr_i,
  input      [OPTION_OPERAND_WIDTH-1:0] wb_result1_i,
  // comparision flag for l.bf/l.bnf
  input                                 ctrl_flag_sr_i,
  // jump/branch signals to IFETCH
  output                                do_branch_o,
  output     [OPTION_OPERAND_WIDTH-1:0] do_branch_target_o,
  output                                jr_gathering_target_o,
  //  # branch prediction support
  input      [OPTION_OPERAND_WIDTH-1:0] after_ds_target_i,
  output                                predict_miss_o,
  output     [OPTION_OPERAND_WIDTH-1:0] predict_miss_target_o,
  input                           [1:0] bc_cnt_value_i,  // current value of saturation counter
  input           [GSHARE_BITS_NUM-1:0] bc_cnt_radr_i,   // saturation counter ID
  output reg                            bc_cnt_we_o,     // update saturation counter
  output reg                      [1:0] bc_cnt_wdat_o,   // new saturation counter value
  output reg      [GSHARE_BITS_NUM-1:0] bc_cnt_wadr_o,   // saturation counter id
  output reg                            bc_hist_taken_o, // conditional branch really taken
  // Support IBUS error handling in CTRL
  output reg                            wb_jump_or_branch_o,
  output reg                            wb_do_branch_o,
  output reg [OPTION_OPERAND_WIDTH-1:0] wb_do_branch_target_o,


  //   Flag to enabel/disable exterlal interrupts processing
  // depending on the fact is instructions restartable or not
  output                                exec_interrupts_en_o,

  // pre-WB l.rfe
  output                                exec_op_rfe_o,
  // pre-WB output exceptions: IFETCH
  output                                exec_except_ibus_err_o,
  output                                exec_except_ipagefault_o,
  output                                exec_except_itlb_miss_o,
  output                                exec_except_ibus_align_o,
  // pre-WB output exceptions: DECODE
  output                                exec_except_illegal_o,
  output                                exec_except_syscall_o,
  output                                exec_except_trap_o,

  // WB outputs
  //  ## special WB-controls for RF
  output reg [OPTION_RF_ADDR_WIDTH-1:0] wb_rf_even_addr_o,
  output reg                            wb_rf_even_wb_o,
  output reg [OPTION_RF_ADDR_WIDTH-1:0] wb_rf_odd_addr_o,
  output reg                            wb_rf_odd_wb_o,
  //  ## instruction related information
  output reg [OPTION_OPERAND_WIDTH-1:0] pc_wb_o,
  output reg                            wb_delay_slot_o,
  output reg [OPTION_RF_ADDR_WIDTH-1:0] wb_rfd1_adr_o,
  output reg                            wb_rfd1_wb_o,
  output reg                            wb_flag_wb_o,
  output reg                            wb_carry_wb_o,
  // for FPU64
  output reg [OPTION_RF_ADDR_WIDTH-1:0] wb_rfd2_adr_o,
  output reg                            wb_rfd2_wb_o,
  // for hazards resolution in RSRVS
  output reg  [DEST_EXT_ADDR_WIDTH-1:0] wb_ext_bits_o
);

  // [O]rder [C]ontrol [B]uffer [T]ap layout for
  // pipeline [C]ontrol flags.
  // We reset taps of the buffer at CPU reset
  // or pipeline flushing.
  localparam  OCBTC_OP_PUSH_WB_POS     =                            0;
  localparam  OCBTC_JUMP_OR_BRANCH_POS = OCBTC_OP_PUSH_WB_POS     + 1;
  localparam  OCBTC_OP_1CLK_POS        = OCBTC_JUMP_OR_BRANCH_POS + 1;
  localparam  OCBTC_OP_DIV_POS         = OCBTC_OP_1CLK_POS        + 1;
  localparam  OCBTC_OP_MUL_POS         = OCBTC_OP_DIV_POS         + 1;
  localparam  OCBTC_OP_FPXX_ARITH_POS  = OCBTC_OP_MUL_POS         + 1; // arithmetic part only
  localparam  OCBTC_OP_FP64_CMP_POS    = OCBTC_OP_FPXX_ARITH_POS  + 1; // granting write back to fpxx comparison
  localparam  OCBTC_OP_LS_POS          = OCBTC_OP_FP64_CMP_POS    + 1; // load / store
  // we also reset extention bits because zero value is meaningfull
  localparam  OCBTC_EXT_ADR_LSB        = OCBTC_OP_LS_POS          + 1;
  localparam  OCBTC_EXT_ADR_MSB        = OCBTC_OP_LS_POS          + DEST_EXT_ADDR_WIDTH;
  // value of MSB of order control buffer tap
  localparam  OCBTC_MSB                = OCBTC_EXT_ADR_MSB;
  localparam  OCBTC_WIDTH              = OCBTC_MSB                + 1;


  // [O]rder [C]ontrol [B]uffer [T]ap layout for
  // various instruction [A]ttributes
  // We don't reset taps of the buffer at CPU reset
  // or pipeline flushing.
  //  [0...5] Exceptions generated by FETCH & DECODE.
  //    (a) Doesn't include IBUS align violation.
  //        It goes through "jump/branch attributes order control buffer".
  //    (b) LSU exceptions go to WB around any OCB
  //  [6] Flag that external interrupt is enabled (instruction is re-startable)
  localparam  OCBTA_INTERRUPTS_EN_POS  =                           6;
  //  Unit wise requested/ready
  localparam  OCBTA_OP_RFE_POS         = OCBTA_INTERRUPTS_EN_POS + 1;
  //  Instruction is in delay slot
  localparam  OCBTA_DELAY_SLOT_POS     = OCBTA_OP_RFE_POS        + 1;
  //  Instruction writting comparison flag
  localparam  OCBTA_FLAG_WB_POS        = OCBTA_DELAY_SLOT_POS    + 1; // any such instruction
  //  Instruction writting carry flag
  localparam  OCBTA_CARRY_WB_POS       = OCBTA_FLAG_WB_POS       + 1;
  //  Instruction generates WB to D1
  localparam  OCBTA_RFD1_WB_POS        = OCBTA_CARRY_WB_POS      + 1;
  localparam  OCBTA_RFD1_ADR_LSB       = OCBTA_RFD1_WB_POS       + 1;
  localparam  OCBTA_RFD1_ADR_MSB       = OCBTA_RFD1_WB_POS       + OPTION_RF_ADDR_WIDTH;
  //  Instruction generates WB to D2
  localparam  OCBTA_RFD2_WB_POS        = OCBTA_RFD1_ADR_MSB      + 1;
  localparam  OCBTA_RFD2_ADR_LSB       = OCBTA_RFD2_WB_POS       + 1;
  localparam  OCBTA_RFD2_ADR_MSB       = OCBTA_RFD2_WB_POS       + OPTION_RF_ADDR_WIDTH;
  //  Program counter
  localparam  OCBTA_PC_LSB             = OCBTA_RFD2_ADR_MSB      + 1;
  localparam  OCBTA_PC_MSB             = OCBTA_RFD2_ADR_MSB      + OPTION_OPERAND_WIDTH;
  //  value of MSB of order control buffer tap
  localparam  OCBTA_MSB                = OCBTA_PC_MSB;
  localparam  OCBTA_WIDTH              = OCBTA_MSB               + 1;


  // a jump/branch instruction in DECODE
  reg dcod_op_jb_r;
  // ---
  always @(posedge cpu_clk) begin
    if (padv_dcod_i)
      dcod_op_jb_r <= fetch_op_jb_i;
  end // at clock

  // IFETCH exceptions in DECODE
  reg dcod_fetch_except_ibus_err_r;
  reg dcod_fetch_except_ipagefault_r;
  reg dcod_fetch_except_itlb_miss_r;
  // ---
  always @(posedge cpu_clk) begin
    if (cpu_rst | pipeline_flush_i) begin
      dcod_fetch_except_ibus_err_r   <= 1'b0;
      dcod_fetch_except_ipagefault_r <= 1'b0;
      dcod_fetch_except_itlb_miss_r  <= 1'b0;
    end
    else if (padv_dcod_i) begin
      dcod_fetch_except_ibus_err_r   <= fetch_except_ibus_err_i;
      dcod_fetch_except_ipagefault_r <= fetch_except_ipagefault_i;
      dcod_fetch_except_itlb_miss_r  <= fetch_except_itlb_miss_i;
    end
    else if (padv_exec_i) begin
      dcod_fetch_except_ibus_err_r   <= 1'b0;
      dcod_fetch_except_ipagefault_r <= 1'b0;
      dcod_fetch_except_itlb_miss_r  <= 1'b0;
    end
  end // at clock


  // extension to DEST, FLAG or CARRY
  // Zero value is reserved as "not used"
  localparam [DEST_EXT_ADDR_WIDTH-1:0] EXT_BITS_MAX = ((1 << DEST_EXT_ADDR_WIDTH) - 1);
  localparam [DEST_EXT_ADDR_WIDTH-1:0] EXT_BITS_MIN = 1;
  // ---
  reg  [DEST_EXT_ADDR_WIDTH-1:0] dcod_ext_bits_r;
  reg  [DEST_EXT_ADDR_WIDTH-1:0] next_ext_bits_r;
  // ---
  wire [DEST_EXT_ADDR_WIDTH-1:0] next_ext_bits_w;
  assign next_ext_bits_w = (next_ext_bits_r == EXT_BITS_MAX) ? EXT_BITS_MIN : (next_ext_bits_r + 1'b1);
  // ---
  always @(posedge cpu_clk) begin
    if (cpu_rst | pipeline_flush_i) begin
      dcod_ext_bits_r <= {DEST_EXT_ADDR_WIDTH{1'b0}};
      next_ext_bits_r <= EXT_BITS_MIN;
    end
    else if (padv_dcod_i) begin
      dcod_ext_bits_r <= fetch_valid_i ? next_ext_bits_r : dcod_ext_bits_r;
      next_ext_bits_r <= fetch_valid_i ? next_ext_bits_w : next_ext_bits_r;
    end
  end // @clock


  // Compute OCBs depth
  localparam INSN_OCB_NUM_TAPS    = EXT_BITS_MAX - 1; // extention bits "0" is reserved as "not used"
  localparam JB_ATTR_OCB_NUM_TAPS = (INSN_OCB_NUM_TAPS >> 1) + 1;


  //-----------------------//
  // OCB-Controls instance //
  //-----------------------//

  // --- OCB-Controls input ---
  wire  [OCBTC_MSB:0] ocbci;
  assign ocbci = {
                  dcod_ext_bits_r, // OCB-Controls entrance
                  dcod_op_ls_i, // OCB-Controls entrance
                  dcod_op_fp64_cmp_i, // OCB-Controls entrance
                  dcod_op_fpxx_arith_i, // OCB-Controls entrance
                  dcod_op_mul_i, // OCB-Controls entrance
                  dcod_op_div_i, // OCB-Controls entrance
                  dcod_op_1clk_i, // OCB-Controls entrance
                  dcod_op_jb_r, // OCB-Controls entrance
                  dcod_op_push_wb_i // OCB-Controls entrance
                 };

  // --- OCB-Controls input ---
  wire [OCBTC_MSB:0] ocbco;

  // --- OCB-Controls status ---
  wire ocbc_full, ocbc_empty;

  // --- OCB-Controls instance ---
  mor1kx_ocb_marocchino
  #(
    .NUM_TAPS   (INSN_OCB_NUM_TAPS),  // INSN_CTRL_OCB
    .NUM_OUTS   (1),                  // INSN_CTRL_OCB
    .DATA_SIZE  (OCBTC_WIDTH),        // INSN_CTRL_OCB
    .FULL_FLAG  ("ENABLED"),          // INSN_CTRL_OCB
    .EMPTY_FLAG ("ENABLED")           // INSN_CTRL_OCB
  )
  u_ocbc
  (
    // clocks, resets
    .clk              (cpu_clk), // INSN_CTRL_OCB
    .rst              (cpu_rst), // INSN_CTRL_OCB
    // pipe controls
    .pipeline_flush_i (pipeline_flush_i), // INSN_CTRL_OCB
    .write_i          (padv_exec_i), // INSN_CTRL_OCB
    .read_i           (padv_wb_i), // INSN_CTRL_OCB
    // value at reset/flush
    .reset_taps       (cpu_rst | pipeline_flush_i), // INSN_CTRL_OCB
    .default_value_i  ({OCBTC_WIDTH{1'b0}}), // INSN_CTRL_OCB
    // data input
    .ocbi_i           (ocbci), // INSN_CTRL_OCB
    // "OCB is empty" flag
    .empty_o          (ocbc_empty), // INSN_CTRL_OCB
    // "OCB is full" flag
    //   (a) external control logic must stop the "writing without reading"
    //       operation if OCB is full
    //   (b) however, the "writing + reading" is possible
    //       because it just pushes OCB and keeps it full
    .full_o           (ocbc_full), // INSN_CTRL_OCB
    // output layout
    // { out[n-1], out[n-2], ... out[0] } : DECODE (entrance) -> EXECUTE (exit)
    .ocbo_o           (ocbco) // INSN_CTRL_OCB
  );


  //-------------------------//
  // OCB-Attributes instance //
  //-------------------------//

  // Flag that istruction is restrartable.
  // Instructions which are not restartable:
  //     "store" - they are buffered and couldn't be removed from buffer
  //               till execution completion
  //     l.msync - it forces LSU to report "busy" till completion of
  //               all previously issued loads and stores
  //     l.mtspr - change internal CPU control registers
  //     l.mfspr - combined with l.mtspr (_mXspr_) to reduce combinatorial logic
  //     l.rfe   - cause return from exception process with serious
  //               changing CPU state.
  //   Note #1 we just not run execution for "invalid" command (CTRL), so
  // such "commands" don't achieve WB where exceptions are processed.
  //   Note #2 l.rfe is a special case. We push pipe full of rfes.
  // The reason for this is that we need the rfe to reach WB stage
  // so it will cause the branch. It will clear itself by the
  // pipeline_flush_i that the rfe will generate.
  wire interrupts_en = ~(dcod_op_lsu_store_i | dcod_op_msync_i | dcod_op_mXspr_i | dcod_op_rfe_i);

  // --- OCB-Attributes input ---
  wire  [OCBTA_MSB:0] ocbai;
  assign ocbai = {
                  pc_decode_i, // OCB-Attributes entrance
                  dcod_rfd2_adr_i, // OCB-Attributes entrance
                  dcod_rfd2_wb_i, // OCB-Attributes entrance
                  dcod_rfd1_adr_i, // OCB-Attributes entrance
                  dcod_rfd1_wb_i, // OCB-Attributes entrance
                  dcod_carry_wb_i, // OCB-Attributes entrance
                  dcod_flag_wb_i, // OCB-Attributes entrance
                  dcod_delay_slot_i, // OCB-Attributes entrance
                  dcod_op_rfe_i, // OCB-Attributes entrance
                  // Flag that istruction is restartable
                  interrupts_en, // OCB-Attributes entrance
                  // FETCH & DECODE exceptions
                  dcod_fetch_except_ibus_err_r, // OCB-Attributes entrance
                  dcod_fetch_except_ipagefault_r, // OCB-Attributes entrance
                  dcod_fetch_except_itlb_miss_r, // OCB-Attributes entrance
                  dcod_except_illegal_i, // OCB-Attributes entrance
                  dcod_except_syscall_i, // OCB-Attributes entrance
                  dcod_except_trap_i // OCB-Attributes entrance
                 };

  // --- OCB-Attribute output ---
  wire [OCBTA_MSB:0] ocbao;

  // --- OCB-Attribute instance ---
  mor1kx_ocb_marocchino
  #(
    .NUM_TAPS   (INSN_OCB_NUM_TAPS),  // INSN_ATTR_OCB
    .NUM_OUTS   (1),                  // INSN_ATTR_OCB
    .DATA_SIZE  (OCBTA_WIDTH),        // INSN_ATTR_OCB
    .FULL_FLAG  ("NONE"),             // INSN_ATTR_OCB
    .EMPTY_FLAG ("NONE")              // INSN_ATTR_OCB
  )
  u_ocba
  (
    // clocks, resets
    .clk              (cpu_clk), // INSN_ATTR_OCB
    .rst              (cpu_rst), // INSN_ATTR_OCB
    // pipe controls
    .pipeline_flush_i (pipeline_flush_i), // INSN_ATTR_OCB
    .write_i          (padv_exec_i), // INSN_ATTR_OCB
    .read_i           (padv_wb_i), // INSN_ATTR_OCB
    // value at reset/flush
    .reset_taps       (1'b0), // INSN_ATTR_OCB
    .default_value_i  ({OCBTA_WIDTH{1'b0}}), // INSN_ATTR_OCB
    // data input
    .ocbi_i           (ocbai), // INSN_ATTR_OCB
    // "OCB is empty" flag
    .empty_o          (), // INSN_ATTR_OCB
    // "OCB is full" flag
    //   (a) external control logic must stop the "writing without reading"
    //       operation if OCB is full
    //   (b) however, the "writing + reading" is possible
    //       because it just pushes OCB and keeps it full
    .full_o           (), // INSN_ATTR_OCB
    // output layout
    // { out[n-1], out[n-2], ... out[0] } : DECODE (entrance) -> EXECUTE (exit)
    .ocbo_o           (ocbao) // INSN_ATTR_OCB
  );


  // Grant WB-access to units
  assign grant_wb_to_1clk_o        = ocbco[OCBTC_OP_1CLK_POS];
  assign grant_wb_to_div_o         = ocbco[OCBTC_OP_DIV_POS];
  assign grant_wb_to_mul_o         = ocbco[OCBTC_OP_MUL_POS];
  assign grant_wb_to_fpxx_arith_o  = ocbco[OCBTC_OP_FPXX_ARITH_POS];
  assign grant_wb_to_lsu_o         = ocbco[OCBTC_OP_LS_POS];
  assign grant_wb_to_fp64_cmp_o    = ocbco[OCBTC_OP_FP64_CMP_POS];


  //--------------------------//
  // Analysis of data hazards //
  //--------------------------//

  // Shorten aliases
  wire [DEST_EXT_ADDR_WIDTH-1:0] exec_ext_bits = ocbco[OCBTC_EXT_ADR_MSB:OCBTC_EXT_ADR_LSB];

  // We needn't WB-to-DECODE hazards for FLAG and CARRY:
  //  (a) we process FLAG for l.bf/.bnf in separate way
  //  (b) only 1-clock instructions request FLAG/CARRY,
  //      however any case they granted with WB accees after completion FLAG/CARRY update

  // RAT parameters
  localparam  NUM_GPRS = (1 << OPTION_RF_ADDR_WIDTH);

  // RAT outputs
  wire [(NUM_GPRS-1):0] rat_rd1_alloc; // allocated by D1
  wire [(NUM_GPRS-1):0] rat_rd2_alloc; // allocated by D2
  wire [(NUM_GPRS-1):0] rat_rdx_alloc; // allocated by D1/D2
  wire [(DEST_EXT_ADDR_WIDTH-1):0] rat_ext_bits [(NUM_GPRS-1):0]; // allocation ID

  // setup RAT cells
  // !!! Even so GPR[0] must be equal to zero we include it into RAT
  // !!! because it could be allocated by initialization routine.
  generate
  genvar ic;
  for (ic = 0; ic < NUM_GPRS; ic = ic + 1) begin : rat_cell_k
    // RAT cells instansence
    rat_cell
    #(
      .OPTION_RF_ADDR_WIDTH   (OPTION_RF_ADDR_WIDTH), // RAT-CELL
      .DEST_EXT_ADDR_WIDTH    (DEST_EXT_ADDR_WIDTH), // RAT-CELL
      .GPR_ADDR               (ic) // RAT-CELL
    )
    u_rat_cell
    (
      // clock & reset
      .cpu_clk                (cpu_clk), // RAT-CELL
      .cpu_rst                (cpu_rst), // RAT-CELL
      // pipeline control
      .padv_dcod_i            (padv_dcod_i), // RAT-CELL
      .padv_wb_i              (padv_wb_i), // RAT-CELL
      .pipeline_flush_i       (pipeline_flush_i), // RAT-CELL
      // input allocation information
      //  # allocated as D1
      .ratin_rfd1_wb_i        (ratin_rfd1_wb_i), // RAT-CELL
      .ratin_rfd1_adr_i       (ratin_rfd1_adr_i), // RAT-CELL
      //  # allocated as D2
      .ratin_rfd2_wb_i        (ratin_rfd2_wb_i), // RAT-CELL
      .ratin_rfd2_adr_i       (ratin_rfd2_adr_i), // RAT-CELL
      //  # allocation id
      .next_ext_bits_i        (next_ext_bits_r), // RAT-CELL
      // input to clear allocation bits
      .exec_ext_bits_i        (exec_ext_bits), // RAT-CELL
      // output allocation information
      .rat_rd1_alloc_o        (rat_rd1_alloc[ic]), // RAT-CELL
      .rat_rd2_alloc_o        (rat_rd2_alloc[ic]), // RAT-CELL
      .rat_rdx_alloc_o        (rat_rdx_alloc[ic]), // RAT-CELL
      .rat_ext_bits_o         (rat_ext_bits[ic]) // RAT-CELL
    );
  end
  endgenerate

  // allocation identifiers
  wire [(DEST_EXT_ADDR_WIDTH-1):0] hazard_dxa1_adr;
  wire [(DEST_EXT_ADDR_WIDTH-1):0] hazard_dxb1_adr;
  wire [(DEST_EXT_ADDR_WIDTH-1):0] hazard_dxa2_adr;
  wire [(DEST_EXT_ADDR_WIDTH-1):0] hazard_dxb2_adr;

  // set hazard flags at reading RF
  //  # relative operand A1
  wire   hazard_d1a1_set = rat_rd1_alloc[fetch_rfa1_adr_i] & ratin_rfa1_req_i;
  wire   hazard_d2a1_set = rat_rd2_alloc[fetch_rfa1_adr_i] & ratin_rfa1_req_i;
  wire   hazard_dxa1_set = rat_rdx_alloc[fetch_rfa1_adr_i] & ratin_rfa1_req_i;
  assign hazard_dxa1_adr = rat_ext_bits[fetch_rfa1_adr_i];
  //  # relative operand B1
  wire   hazard_d1b1_set = rat_rd1_alloc[fetch_rfb1_adr_i] & ratin_rfb1_req_i;
  wire   hazard_d2b1_set = rat_rd2_alloc[fetch_rfb1_adr_i] & ratin_rfb1_req_i;
  wire   hazard_dxb1_set = rat_rdx_alloc[fetch_rfb1_adr_i] & ratin_rfb1_req_i;
  assign hazard_dxb1_adr = rat_ext_bits[fetch_rfb1_adr_i];
  //  # relative operand A2
  wire   hazard_d1a2_set = rat_rd1_alloc[fetch_rfa2_adr_i] & ratin_rfa2_req_i;
  wire   hazard_d2a2_set = rat_rd2_alloc[fetch_rfa2_adr_i] & ratin_rfa2_req_i;
  wire   hazard_dxa2_set = rat_rdx_alloc[fetch_rfa2_adr_i] & ratin_rfa2_req_i;
  assign hazard_dxa2_adr = rat_ext_bits[fetch_rfa2_adr_i];
  //  # relative operand B2
  wire   hazard_d1b2_set = rat_rd1_alloc[fetch_rfb2_adr_i] & ratin_rfb2_req_i;
  wire   hazard_d2b2_set = rat_rd2_alloc[fetch_rfb2_adr_i] & ratin_rfb2_req_i;
  wire   hazard_dxb2_set = rat_rdx_alloc[fetch_rfb2_adr_i] & ratin_rfb2_req_i;
  assign hazard_dxb2_adr = rat_ext_bits[fetch_rfb2_adr_i];

  // don't set hazard flags if write-back advancing resolved hazard
  // the case takes place if write-back overlapping FETCH->DECODE advance
  wire no_exe2fth_dxa1 = (hazard_dxa1_adr != exec_ext_bits);
  wire no_exe2fth_dxb1 = (hazard_dxb1_adr != exec_ext_bits);
  wire no_exe2fth_dxa2 = (hazard_dxa2_adr != exec_ext_bits);
  wire no_exe2fth_dxb2 = (hazard_dxb2_adr != exec_ext_bits);

  // clear DECODE-visible hazards flags if write-back:
  //  (a) is not overlapping FETCH->DECODE advance and
  //  (b) resolving hazard
  wire no_exe2dcd_dxa1 = (omn2dec_hazard_dxa1_adr_o != exec_ext_bits);
  wire no_exe2dcd_dxb1 = (omn2dec_hazard_dxb1_adr_o != exec_ext_bits);
  wire no_exe2dcd_dxa2 = (omn2dec_hazard_dxa2_adr_o != exec_ext_bits);
  wire no_exe2dcd_dxb2 = (omn2dec_hazard_dxb2_adr_o != exec_ext_bits);

  // OMAN-to-DECODE hazards flags by operands
  always @(posedge cpu_clk) begin
    if (cpu_rst | pipeline_flush_i) begin
      //  # relative operand A1
      omn2dec_hazard_d1a1_o <= 1'b0;
      omn2dec_hazard_d2a1_o <= 1'b0;
      omn2dec_hazard_dxa1_o <= 1'b0;
      //  # relative operand B1
      omn2dec_hazard_d1b1_o <= 1'b0;
      omn2dec_hazard_d2b1_o <= 1'b0;
      omn2dec_hazard_dxb1_o <= 1'b0;
      //  # relative operand A2
      omn2dec_hazard_d1a2_o <= 1'b0;
      omn2dec_hazard_d2a2_o <= 1'b0;
      omn2dec_hazard_dxa2_o <= 1'b0;
      //  # relative operand B2
      omn2dec_hazard_d1b2_o <= 1'b0;
      omn2dec_hazard_d2b2_o <= 1'b0;
      omn2dec_hazard_dxb2_o <= 1'b0;
    end
    else begin
      // synthesis parallel_case full_case
      case({padv_wb_i,padv_dcod_i})
        // only FETCH->DECODE
        2'b01: begin
          //  # relative operand A1
          omn2dec_hazard_d1a1_o <= hazard_d1a1_set;
          omn2dec_hazard_d2a1_o <= hazard_d2a1_set;
          omn2dec_hazard_dxa1_o <= hazard_dxa1_set;
          //  # relative operand B1
          omn2dec_hazard_d1b1_o <= hazard_d1b1_set;
          omn2dec_hazard_d2b1_o <= hazard_d2b1_set;
          omn2dec_hazard_dxb1_o <= hazard_dxb1_set;
          //  # relative operand A2
          omn2dec_hazard_d1a2_o <= hazard_d1a2_set;
          omn2dec_hazard_d2a2_o <= hazard_d2a2_set;
          omn2dec_hazard_dxa2_o <= hazard_dxa2_set;
          //  # relative operand B2
          omn2dec_hazard_d1b2_o <= hazard_d1b2_set;
          omn2dec_hazard_d2b2_o <= hazard_d2b2_set;
          omn2dec_hazard_dxb2_o <= hazard_dxb2_set;
        end
        // only write-back
        2'b10: begin
          //  # relative operand A1
          omn2dec_hazard_d1a1_o <= omn2dec_hazard_d1a1_o & no_exe2dcd_dxa1;
          omn2dec_hazard_d2a1_o <= omn2dec_hazard_d2a1_o & no_exe2dcd_dxa1;
          omn2dec_hazard_dxa1_o <= omn2dec_hazard_dxa1_o & no_exe2dcd_dxa1;
          //  # relative operand B1
          omn2dec_hazard_d1b1_o <= omn2dec_hazard_d1b1_o & no_exe2dcd_dxb1;
          omn2dec_hazard_d2b1_o <= omn2dec_hazard_d2b1_o & no_exe2dcd_dxb1;
          omn2dec_hazard_dxb1_o <= omn2dec_hazard_dxb1_o & no_exe2dcd_dxb1;
          //  # relative operand A2
          omn2dec_hazard_d1a2_o <= omn2dec_hazard_d1a2_o & no_exe2dcd_dxa2;
          omn2dec_hazard_d2a2_o <= omn2dec_hazard_d2a2_o & no_exe2dcd_dxa2;
          omn2dec_hazard_dxa2_o <= omn2dec_hazard_dxa2_o & no_exe2dcd_dxa2;
          //  # relative operand B2
          omn2dec_hazard_d1b2_o <= omn2dec_hazard_d1b2_o & no_exe2dcd_dxb2;
          omn2dec_hazard_d2b2_o <= omn2dec_hazard_d2b2_o & no_exe2dcd_dxb2;
          omn2dec_hazard_dxb2_o <= omn2dec_hazard_dxb2_o & no_exe2dcd_dxb2;
        end
        // write-back overlapping FETCH->DECODE
        2'b11: begin
          //  # relative operand A1
          omn2dec_hazard_d1a1_o <= hazard_d1a1_set & no_exe2fth_dxa1;
          omn2dec_hazard_d2a1_o <= hazard_d2a1_set & no_exe2fth_dxa1;
          omn2dec_hazard_dxa1_o <= hazard_dxa1_set & no_exe2fth_dxa1;
          //  # relative operand B1
          omn2dec_hazard_d1b1_o <= hazard_d1b1_set & no_exe2fth_dxb1;
          omn2dec_hazard_d2b1_o <= hazard_d2b1_set & no_exe2fth_dxb1;
          omn2dec_hazard_dxb1_o <= hazard_dxb1_set & no_exe2fth_dxb1;
          //  # relative operand A2
          omn2dec_hazard_d1a2_o <= hazard_d1a2_set & no_exe2fth_dxa2;
          omn2dec_hazard_d2a2_o <= hazard_d2a2_set & no_exe2fth_dxa2;
          omn2dec_hazard_dxa2_o <= hazard_dxa2_set & no_exe2fth_dxa2;
          //  # relative operand B2
          omn2dec_hazard_d1b2_o <= hazard_d1b2_set & no_exe2fth_dxb2;
          omn2dec_hazard_d2b2_o <= hazard_d2b2_set & no_exe2fth_dxb2;
          omn2dec_hazard_dxb2_o <= hazard_dxb2_set & no_exe2fth_dxb2;
        end
        // no change
        default:;
      endcase
    end
  end // at clock

  // OMAN-to-DECODE hazards ids by operands
  always @(posedge cpu_clk) begin
    if (cpu_rst) begin
      omn2dec_hazard_dxa1_adr_o <= {DEST_EXT_ADDR_WIDTH{1'b0}};
      omn2dec_hazard_dxb1_adr_o <= {DEST_EXT_ADDR_WIDTH{1'b0}};
      omn2dec_hazard_dxa2_adr_o <= {DEST_EXT_ADDR_WIDTH{1'b0}};
      omn2dec_hazard_dxb2_adr_o <= {DEST_EXT_ADDR_WIDTH{1'b0}};
    end
    else if (padv_dcod_i) begin
      omn2dec_hazard_dxa1_adr_o <= hazard_dxa1_adr;
      omn2dec_hazard_dxb1_adr_o <= hazard_dxb1_adr;
      omn2dec_hazard_dxa2_adr_o <= hazard_dxa2_adr;
      omn2dec_hazard_dxb2_adr_o <= hazard_dxb2_adr;
    end
  end


  // 1-clock "WB to DECODE operand forwarding" flags
  //  # when write-back overlapes FETCH->DECODE advance
  wire exe2fth_dxa1 = (hazard_dxa1_adr == exec_ext_bits);
  wire exe2fth_dxb1 = (hazard_dxb1_adr == exec_ext_bits);
  wire exe2fth_dxa2 = (hazard_dxa2_adr == exec_ext_bits);
  wire exe2fth_dxb2 = (hazard_dxb2_adr == exec_ext_bits);
  //  # when write-back only
  wire exe2dcd_dxa1 = (omn2dec_hazard_dxa1_adr_o == exec_ext_bits);
  wire exe2dcd_dxb1 = (omn2dec_hazard_dxb1_adr_o == exec_ext_bits);
  wire exe2dcd_dxa2 = (omn2dec_hazard_dxa2_adr_o == exec_ext_bits);
  wire exe2dcd_dxb2 = (omn2dec_hazard_dxb2_adr_o == exec_ext_bits);
  // ---
  always @(posedge cpu_clk) begin
    if (cpu_rst) begin
      //  # relative operand A1
      dcod_wb2dec_d1a1_fwd_o <= 1'b0;
      dcod_wb2dec_d2a1_fwd_o <= 1'b0;
      //  # relative operand B1
      dcod_wb2dec_d1b1_fwd_o <= 1'b0;
      dcod_wb2dec_d2b1_fwd_o <= 1'b0;
      //  # relative operand A2
      dcod_wb2dec_d1a2_fwd_o <= 1'b0;
      dcod_wb2dec_d2a2_fwd_o <= 1'b0;
      //  # relative operand B2
      dcod_wb2dec_d1b2_fwd_o <= 1'b0;
      dcod_wb2dec_d2b2_fwd_o <= 1'b0;
    end
    else begin
      // synthesis parallel_case full_case
      case ({padv_wb_i,padv_dcod_i})
        // when write-back only
        2'b10: begin
          //  # relative operand A1
          dcod_wb2dec_d1a1_fwd_o <= omn2dec_hazard_d1a1_o & exe2dcd_dxa1;
          dcod_wb2dec_d2a1_fwd_o <= omn2dec_hazard_d2a1_o & exe2dcd_dxa1;
          //  # relative operand B1
          dcod_wb2dec_d1b1_fwd_o <= omn2dec_hazard_d1b1_o & exe2dcd_dxb1;
          dcod_wb2dec_d2b1_fwd_o <= omn2dec_hazard_d2b1_o & exe2dcd_dxb1;
          //  # relative operand A2
          dcod_wb2dec_d1a2_fwd_o <= omn2dec_hazard_d1a2_o & exe2dcd_dxa2;
          dcod_wb2dec_d2a2_fwd_o <= omn2dec_hazard_d2a2_o & exe2dcd_dxa2;
          //  # relative operand B2
          dcod_wb2dec_d1b2_fwd_o <= omn2dec_hazard_d1b2_o & exe2dcd_dxb2;
          dcod_wb2dec_d2b2_fwd_o <= omn2dec_hazard_d2b2_o & exe2dcd_dxb2;
        end
        // when write-back overlapes FETCH->DECODE advance
        2'b11: begin
          //  # relative operand A1
          dcod_wb2dec_d1a1_fwd_o <= hazard_d1a1_set & exe2fth_dxa1;
          dcod_wb2dec_d2a1_fwd_o <= hazard_d2a1_set & exe2fth_dxa1;
          //  # relative operand B1
          dcod_wb2dec_d1b1_fwd_o <= hazard_d1b1_set & exe2fth_dxb1;
          dcod_wb2dec_d2b1_fwd_o <= hazard_d2b1_set & exe2fth_dxb1;
          //  # relative operand A2
          dcod_wb2dec_d1a2_fwd_o <= hazard_d1a2_set & exe2fth_dxa2;
          dcod_wb2dec_d2a2_fwd_o <= hazard_d2a2_set & exe2fth_dxa2;
          //  # relative operand B2
          dcod_wb2dec_d1b2_fwd_o <= hazard_d1b2_set & exe2fth_dxb2;
          dcod_wb2dec_d2b2_fwd_o <= hazard_d2b2_set & exe2fth_dxb2;
        end
        // 1-clock length
        default: begin
          //  # relative operand A1
          dcod_wb2dec_d1a1_fwd_o <= 1'b0;
          dcod_wb2dec_d2a1_fwd_o <= 1'b0;
          //  # relative operand B1
          dcod_wb2dec_d1b1_fwd_o <= 1'b0;
          dcod_wb2dec_d2b1_fwd_o <= 1'b0;
          //  # relative operand A2
          dcod_wb2dec_d1a2_fwd_o <= 1'b0;
          dcod_wb2dec_d2a2_fwd_o <= 1'b0;
          //  # relative operand B2
          dcod_wb2dec_d1b2_fwd_o <= 1'b0;
          dcod_wb2dec_d2b2_fwd_o <= 1'b0;
        end
      endcase
    end
  end // @clock


  //   An execute module is ready and granted access to WB
  //   Instructions l.mf(t)spr have got guaranted WB access because
  // no any new instruction is issued into execution till
  // l.mf(t)spr has been completed. Pay attention that we start
  // l.mf(t)spr ecxecution after completion of all peviously
  // issued instructions only.
  //   l.rfe and FETCH/DECODE exceptions are also should
  // push WB latches
  // ---
  //   Declaration valid flag for jump/branch attributes
  wire   exec_jb_attr_valid;
  // ---
  //   For l.jal/l.jalr we use adder in 1-clk to compure return address,
  // so for the cases we additionally wait "jump/branch attributes".
  assign exec_valid_o =
    (ocbco[OCBTC_OP_1CLK_POS] & ~ocbco[OCBTC_JUMP_OR_BRANCH_POS]) | // EXEC VALID: but wait attributes for l.jal/ljalr
    (exec_jb_attr_valid       &  ocbco[OCBTC_JUMP_OR_BRANCH_POS]) | // EXEC VALID
    (div_valid_i              &  ocbco[OCBTC_OP_DIV_POS])         | // EXEC VALID
    (mul_valid_i              &  ocbco[OCBTC_OP_MUL_POS])         | // EXEC VALID
    (fpxx_arith_valid_i       &  ocbco[OCBTC_OP_FPXX_ARITH_POS])  | // EXEC VALID
    (fp64_cmp_valid_i         &  ocbco[OCBTC_OP_FP64_CMP_POS])    | // EXEC VALID
    (lsu_valid_i              &  ocbco[OCBTC_OP_LS_POS])          | // EXEC VALID
                                 ocbco[OCBTC_OP_PUSH_WB_POS];       // EXEC VALID

  // DECODE valid
  assign dcod_valid_o = (~ocbc_full) &                           // DECODE VALID
                        ((dcod_op_1clk_i     & op_1clk_free_i) | // DECODE VALID
                         (dcod_op_muldiv_i   & muldiv_free_i)  | // DECODE VALID
                         (dcod_op_fpxx_any_i & fpxx_free_i)    | // DECODE VALID
                         (dcod_op_lsu_any_i  & lsu_free_i)     | // DECODE VALID
                         (dcod_op_mXspr_i    & ocbc_empty)     | // DECODE VALID
                         dcod_op_push_exec_i);                   // DECODE VALID



  //---------------------------------------//
  // Logic to support Jump / Branch taking //
  //---------------------------------------//

  // state machine for tracking Jump / Branch related hazards
  localparam [6:0] JB_FSM_CATCHING_JB           = 7'b0000001, // on IFETCH output
                   JB_FSM_GET_B1                = 7'b0000010, // get rB for l.jr/ljalr if no hazard
                   JB_FSM_WAITING_B1            = 7'b0000100, // waiting rB for l.jr/ljalr if hazard
                   JB_FSM_DOING_JR              = 7'b0001000, // execute l.jr/ljalr
                   JB_FSM_PREDICT_CATCHING_DS   = 7'b0010000, // from conditional branch till delay slot
                   JB_FSM_PREDICT_WAITING_FLAG  = 7'b0100000, // from delay slot till periction reslolving
                   JB_FSM_PREDICT_MISS          = 7'b1000000; // restart IFETCH from mispredict target (1-clock)
  // ---
  reg [6:0] jb_fsm_state_r;
  // initial value for simulations
  // we need it because we use mispredict flag ORed
  // with pipeline-flush and cpu-rst in IFETCH
 `ifndef SYNTHESIS
  // synthesis translate_off
  initial jb_fsm_state_r = 7'd0;
  // synthesis translate_on
 `endif // !synth
  // --- particular states ---
  //wire jb_fsm_get_b1_state               = jb_fsm_state_r[1];
  //wire jb_fsm_waiting_b1_state           = jb_fsm_state_r[2];
  wire jb_fsm_doing_jr_state             = jb_fsm_state_r[3];
  wire jb_fsm_predict_catching_ds_state  = jb_fsm_state_r[4];
  wire jb_fsm_predict_waiting_flag_state = jb_fsm_state_r[5];
  wire jb_fsm_predict_miss_state         = jb_fsm_state_r[6];


  // --- detect flag hazard for l.bf/l.bnf ---
  // Opposite to GPR RAT we keep SR[F] allocated flag till
  // completion writting to SR[F]
  reg                            flag_alloc_r;     // SR[F] allocated for write-back
  reg  [DEST_EXT_ADDR_WIDTH-1:0] flag_alloc_ext_r; // SR[F] allocation index
  // ---
  wire keep_flag_alloc_at_wb = flag_alloc_r & (flag_alloc_ext_r != wb_ext_bits_o);
  // ---
  always @(posedge cpu_clk) begin
    if (cpu_rst | pipeline_flush_i)
      flag_alloc_r <= 1'b0;
    else begin
      // synthesis parallel_case full_case
      case ({wb_flag_wb_o,padv_dcod_i})
        // FETCH->DECODE
        2'b01: flag_alloc_r <= fetch_flag_wb_i | flag_alloc_r;
        // WB-only
        2'b10: flag_alloc_r <= keep_flag_alloc_at_wb;
        // overlapping
        2'b11: flag_alloc_r <= fetch_flag_wb_i | keep_flag_alloc_at_wb;
        // don't change by default
        default:;
      endcase
    end
  end // at cpu-clock
  // ---
  always @(posedge cpu_clk) begin
    if (cpu_rst)
      flag_alloc_ext_r <= {DEST_EXT_ADDR_WIDTH{1'b0}};
    else if (padv_dcod_i)
      flag_alloc_ext_r <= fetch_flag_wb_i ? next_ext_bits_r : flag_alloc_ext_r;
  end // at cpu-clock


  // --- various pendings till rB/flag computationcompletion ---
  reg  [DEST_EXT_ADDR_WIDTH-1:0] jb_hazard_ext_p;
  reg [OPTION_OPERAND_WIDTH-1:0] jr_target_p;
  reg                            jr_gathering_target_p;

  // --- prediction related registers ---
  reg                            predict_bc_taken_r;    // 0 if not taken
  reg                            predict_flag_value_r;
  reg                            predict_flag_alloc_r;
  reg [OPTION_OPERAND_WIDTH-1:0] predict_hit_target_r;
  reg [OPTION_OPERAND_WIDTH-1:0] predict_miss_target_r;
  // --- use / do  preticted or instant conditional branch (bc) ---
  wire use_bc_predict    = fetch_op_bf_i | fetch_op_bnf_i;
  wire do_bc_predict_raw = bc_cnt_value_i[1];
  wire do_bc_predict     = use_bc_predict & do_bc_predict_raw;
  // --- wait completion writting to SR[F] ---
  wire keep_predict_flag_alloc = predict_flag_alloc_r & (jb_hazard_ext_p != wb_ext_bits_o);
  // --- compute raw hit/miss for prediction ---
  // --- they are used only inside J/B FSM ---
  wire predict_hit_raw  = (~predict_flag_alloc_r) &  (~(predict_flag_value_r ^ ctrl_flag_sr_i));
  wire predict_miss_raw = (~predict_flag_alloc_r) &    (predict_flag_value_r ^ ctrl_flag_sr_i);
  // --- complete prediction hit ---
  wire predict_hit = (jb_fsm_predict_catching_ds_state | jb_fsm_predict_waiting_flag_state) & predict_hit_raw;

  // Jump / Branch state machine
  always @(posedge cpu_clk) begin
    if (cpu_rst | pipeline_flush_i) begin
      jb_fsm_state_r  <= JB_FSM_CATCHING_JB;
      // for prediction processing
      predict_bc_taken_r   <= 1'b0;
      predict_flag_value_r <= 1'b0;
      predict_flag_alloc_r <= 1'b0;
      // for l.jr/l.jalr procesing
      jr_gathering_target_p <= 1'b0;
      // other attributes
      jb_hazard_ext_p <= {DEST_EXT_ADDR_WIDTH{1'b0}};
    end
    else begin
      // synthesis parallel_case full_case
      case (jb_fsm_state_r)
        // catching j/b on IFETCH output
        JB_FSM_CATCHING_JB: begin
          if (padv_dcod_i) begin
            if (fetch_op_jr_i) begin
              if (rat_rd1_alloc[fetch_rfb1_adr_i]) begin
                jb_fsm_state_r  <= JB_FSM_WAITING_B1;
                jb_hazard_ext_p <= hazard_dxb1_adr;
              end
              else begin // no rB related hazards
                jb_fsm_state_r <= JB_FSM_GET_B1;
              end
              jr_gathering_target_p <= 1'b1;
            end
            else if (use_bc_predict) begin
              jb_fsm_state_r        <= JB_FSM_PREDICT_CATCHING_DS;
              predict_bc_taken_r    <= do_bc_predict_raw;
              predict_flag_value_r  <= (fetch_op_bf_i  &   do_bc_predict_raw) | // PREDICTED FLAG
                                       (fetch_op_bnf_i & (~do_bc_predict_raw)); // PREDICTED FLAG
              predict_flag_alloc_r  <= keep_flag_alloc_at_wb;
              jb_hazard_ext_p       <= flag_alloc_ext_r;
            end
          end
        end

        // gathering target for l.jr/l.jalr
        JB_FSM_GET_B1: begin
          jb_fsm_state_r        <= JB_FSM_DOING_JR;
          jr_gathering_target_p <= 1'b0;
        end
        // waiting target for l.jr/l.jalr
        JB_FSM_WAITING_B1: begin
          if (jb_hazard_ext_p == wb_ext_bits_o) begin
            jb_fsm_state_r        <= JB_FSM_DOING_JR;
            jr_gathering_target_p <= 1'b0;
          end
        end
        // doing l.jr/l.jalr
        JB_FSM_DOING_JR: begin
          jb_fsm_state_r <= JB_FSM_CATCHING_JB;
        end

        // catching delay slot after branch conditional
        JB_FSM_PREDICT_CATCHING_DS: begin
          if (predict_hit_raw) begin
            jb_fsm_state_r <= JB_FSM_CATCHING_JB;
          end
          else if (padv_dcod_i) begin
            if (fetch_valid_i & fetch_delay_slot_i)
              jb_fsm_state_r <= predict_miss_raw ? JB_FSM_PREDICT_MISS : JB_FSM_PREDICT_WAITING_FLAG;
          end
          predict_flag_alloc_r <= keep_predict_flag_alloc;
        end
        // waiting flag computation
        JB_FSM_PREDICT_WAITING_FLAG: begin
          if (predict_hit_raw) begin
            jb_fsm_state_r <= JB_FSM_CATCHING_JB;
          end
          else if (predict_miss_raw) begin
            jb_fsm_state_r <= JB_FSM_PREDICT_MISS;
          end
          predict_flag_alloc_r <= keep_predict_flag_alloc;
        end
        // miss prediction (1-clock)
        JB_FSM_PREDICT_MISS: begin
          jb_fsm_state_r <= JB_FSM_CATCHING_JB;
        end
        // others
        default:;
      endcase
    end
  end // @cpu-clock

  // store target for l.jr/l.jalr
  always @(posedge cpu_clk) begin
    // synthesis parallel_case full_case
    case (jb_fsm_state_r)
      // gathering l.jr/l.jalr target if no B1 hazards
      JB_FSM_GET_B1: begin
        jr_target_p <= dcod_rfb1_jr_i;
      end
      // waiting target for l.jr/l.jalr
      // (continuously as rigth result is latched
      //  automatically at hazard resolution)
      JB_FSM_WAITING_B1: begin
        jr_target_p <= wb_result1_i;
      end
      // others
      default:;
    endcase
  end // @cpu-clock

  // store targets for miss and hit predictions
  //  !!! minimal set of conditions is used for storing
  //  !!! because we use the values with appropiate
  //  !!! flags only
  always @(posedge cpu_clk) begin
    if (padv_dcod_i & fetch_op_jb_i) begin
      predict_hit_target_r  <= do_bc_predict_raw ? fetch_to_imm_target_i : after_ds_target_i;
      predict_miss_target_r <= do_bc_predict_raw ? after_ds_target_i : fetch_to_imm_target_i;
    end
  end // at clock


  // DECODE is "locked" flag.
  //  (1) We set the flag after delay slot has passed
  // into DECODE and till prediction resolving.
  //  (2) For l.jr/l.jalr we use another approach:
  // IFETCH just generates bubbles till rB1 hazard resolving.
  // ---
  reg  dcod_locked_r;
  // ---
  always @(posedge cpu_clk) begin
    if (cpu_rst | pipeline_flush_i)
      dcod_locked_r <= 1'b0;
    else if (dcod_locked_r) begin
      if ((jb_fsm_predict_waiting_flag_state & predict_hit_raw) | // UNLOCK DECODE
          jb_fsm_predict_miss_state)                              // UNLOCK DECODE
        dcod_locked_r <= 1'b0;
    end
    else if (padv_dcod_i)
      dcod_locked_r <= fetch_valid_i & fetch_delay_slot_i &                     // LOCK DECODE
                       (jb_fsm_predict_catching_ds_state & (~predict_hit_raw)); // LOCK DECODE
  end // at clock
  // ---
  assign dcod_free_o = (~dcod_locked_r);



  // --- Feedback to IFETCH --- //


  // "Do branch"
  assign do_branch_o = fetch_op_jimm_i       |  // do jump to immediate
                       do_bc_predict         |  // do branch conditional
                       jb_fsm_doing_jr_state;   // do jump to register (B1)
  // branch target
  assign do_branch_target_o = jb_fsm_doing_jr_state ? jr_target_p :          // branch target selection
                                                      fetch_to_imm_target_i; // branch target selection


  // we execute l.jr/l.jalr after registering target only
  assign jr_gathering_target_o = fetch_op_jr_i | jr_gathering_target_p;

  // branch prediction missed
  assign predict_miss_o        = jb_fsm_predict_miss_state;
  assign predict_miss_target_o = predict_miss_target_r;



  // --- Feedforward to WB --- //


  // JUMP/BRANCH attribute control flags
  localparam  JB_ATTR_C_DO_BRANCH_POS  = 0; // if "do" the "target" makes sence
  localparam  JB_ATTR_C_IBUS_ALIGN_POS = JB_ATTR_C_DO_BRANCH_POS  + 1;
  localparam  JB_ATTR_C_VALID_POS      = JB_ATTR_C_IBUS_ALIGN_POS + 1;
  //---
  localparam  JB_ATTR_C_MSB            = JB_ATTR_C_VALID_POS;
  localparam  JB_ATTR_C_WIDTH          = JB_ATTR_C_MSB + 1;


  // Jump/Branch attributes valid instantly
  wire jb_attr_valid_instant = fetch_op_jimm_i;

  // Jump/Branch valid after miss by various reasons
  // !!! each case is 1-clock length
  wire jb_attr_valid_after_miss = predict_hit | jb_fsm_predict_miss_state | jb_fsm_doing_jr_state;

  // Jump/Branch : do branch flag
  wire jb_attr_do_branch = fetch_op_jimm_i |                                    // JB ATTR DO BRANCH FLAG
                           (predict_hit               ?   predict_bc_taken_r  : // JB ATTR DO BRANCH FLAG
                            jb_fsm_predict_miss_state ? (~predict_bc_taken_r) : // JB ATTR DO BRANCH FLAG
                                                        jb_fsm_doing_jr_state); // JB ATTR DO BRANCH FLAG

  // Jump/Branch target (makes sence only if do branch flag is raized)
  reg [OPTION_OPERAND_WIDTH-1:0] jb_attr_target_r;
  // ---
  always @(predict_hit           or jb_fsm_predict_miss_state or
           jb_fsm_doing_jr_state or
           predict_hit_target_r  or predict_miss_target_r or
           jr_target_p           or fetch_to_imm_target_i) begin
    // synthesis parallel_case full_case
    case ({predict_hit, jb_fsm_predict_miss_state, jb_fsm_doing_jr_state})
      3'b100:  jb_attr_target_r = predict_hit_target_r;
      3'b010:  jb_attr_target_r = predict_miss_target_r;
      3'b001:  jb_attr_target_r = jr_target_p;
      default: jb_attr_target_r = fetch_to_imm_target_i;
    endcase
  end // case

  // Write/Read to Jump/Branch attributes buffers
  wire jb_attr_ocb_write = (padv_dcod_i & jb_attr_valid_instant) | jb_attr_valid_after_miss;
  wire jb_attr_ocb_read  = padv_wb_i & ocbco[OCBTC_JUMP_OR_BRANCH_POS];

  // Jump/Branch control attributes buffer
  //  !!! with clean up at reset or flush

  // --- JB-ATTR-C-OCB ouitput / input ---
  wire [JB_ATTR_C_MSB:0] jb_attr_c_ocbo;
  wire [JB_ATTR_C_MSB:0] jb_attr_c_ocbi;
  // ---
  assign jb_attr_c_ocbi =
    {
      1'b1,     // JB ATTR-C VALID
      (jb_fsm_doing_jr_state & (|jr_target_p[1:0])), // JB ATTR-C IBUS ALIGN
      jb_attr_do_branch // JB ATTR-C DO BRANCH
    };

  // --- JB-ATTR-C-OCB instance ---
  mor1kx_ocb_marocchino
  #(
    .NUM_TAPS   (JB_ATTR_OCB_NUM_TAPS), // JB-ATTR-C-OCB: extention bits "0" is reserved as "not used"
    .NUM_OUTS   (1),                    // JB-ATTR-C-OCB
    .DATA_SIZE  (JB_ATTR_C_WIDTH),      // JB-ATTR-C-OCB
    .FULL_FLAG  ("NONE"),               // JB-ATTR-C-OCB
    .EMPTY_FLAG ("NONE")                // JB-ATTR-C-OCB
  )
  u_jb_attr_c_ocb
  (
    // clocks, resets
    .clk              (cpu_clk), // JB-ATTR-C-OCB
    .rst              (cpu_rst), // JB-ATTR-C-OCB
    // pipe controls
    .pipeline_flush_i (pipeline_flush_i), // JB-ATTR-C-OCB
    .write_i          (jb_attr_ocb_write), // JB-ATTR-C-OCB
    .read_i           (jb_attr_ocb_read), // JB-ATTR-C-OCB
    // value at reset/flush
    .reset_taps       (cpu_rst | pipeline_flush_i), // JB-ATTR-C-OCB
    .default_value_i  ({JB_ATTR_C_WIDTH{1'b0}}), // JB-ATTR-C-OCB
    // data input
    .ocbi_i           (jb_attr_c_ocbi), // JB-ATTR-C-OCB
    // "OCB is empty" flag
    .empty_o          (), // JB-ATTR-C-OCB
    // "OCB is full" flag
    //   (a) external control logic must stop the "writing without reading"
    //       operation if OCB is full
    //   (b) however, the "writing + reading" is possible
    //       because it just pushes OCB and keeps it full
    .full_o           (), // JB-ATTR-C-OCB
    // output layout
    // { out[n-1], out[n-2], ... out[0] } : DECODE (entrance) -> EXECUTE (exit)
    .ocbo_o           (jb_attr_c_ocbo) // JB-ATTR-C-OCB
  );

  // --- JB-ATTR-C-OCB valid unpack ---
  assign exec_jb_attr_valid = jb_attr_c_ocbo[JB_ATTR_C_VALID_POS];


  // Jump/Branch attributes data buffer
  //  !!! without clean up at reset or flush

  localparam JB_ATTR_D_MSB   = OPTION_OPERAND_WIDTH - 1;
  localparam JB_ATTR_D_WIDTH = JB_ATTR_D_MSB        + 1;

  // --- JB-ATTR-D-OCB ouitput / input ---
  wire [JB_ATTR_D_MSB:0] jb_attr_d_ocbo;
  wire [JB_ATTR_D_MSB:0] jb_attr_d_ocbi;
  // --- JB-ATTR-D-OCB instance ---
  assign jb_attr_d_ocbi = jb_attr_target_r;

  // --- JB-ATTR-D-OCB instance ---
  mor1kx_ocb_marocchino
  #(
    .NUM_TAPS   (JB_ATTR_OCB_NUM_TAPS), // JB-ATTR-D-OCB: extention bits "0" is reserved as "not used"
    .NUM_OUTS   (1),                    // JB-ATTR-D-OCB
    .DATA_SIZE  (JB_ATTR_D_WIDTH),      // JB-ATTR-D-OCB
    .FULL_FLAG  ("NONE"),               // JB-ATTR-D-OCB
    .EMPTY_FLAG ("NONE")                // JB-ATTR-D-OCB
  )
  u_jb_attr_d_ocb
  (
    // clocks, resets
    .clk              (cpu_clk), // JB-ATTR-D-OCB
    .rst              (cpu_rst), // JB-ATTR-D-OCB
    // pipe controls
    .pipeline_flush_i (pipeline_flush_i), // JB-ATTR-D-OCB
    .write_i          (jb_attr_ocb_write), // JB-ATTR-D-OCB
    .read_i           (jb_attr_ocb_read), // JB-ATTR-D-OCB
    // value at reset/flush
    .reset_taps       (1'b0), // JB-ATTR-D-OCB
    .default_value_i  ({JB_ATTR_D_WIDTH{1'b0}}), // JB-ATTR-D-OCB
    // data input
    .ocbi_i           (jb_attr_d_ocbi), // JB-ATTR-D-OCB
    // "OCB is empty" flag
    .empty_o          (), // JB-ATTR-D-OCB
    // "OCB is full" flag
    //   (a) external control logic must stop the "writing without reading"
    //       operation if OCB is full
    //   (b) however, the "writing + reading" is possible
    //       because it just pushes OCB and keeps it full
    .full_o           (), // JB-ATTR-D-OCB
    // output layout
    // { out[n-1], out[n-2], ... out[0] } : DECODE (entrance) -> EXECUTE (exit)
    .ocbo_o           (jb_attr_d_ocbo) // JB-ATTR-D-OCB
  );


  //----------------------------------------//
  // Saturation counter as branch predictor //
  //----------------------------------------//
  //  (a) We update counter only after completion
  //      all previous updates of SR[F]
  //  (b) They are 1-clock length to prevent multiple
  //      updates of counter

  // --- pending values for update saturation counters and branch history ---
  reg                    [1:0] bc_cnt_value_up_p;
  reg                    [1:0] bc_cnt_value_dn_p;
  reg  [(GSHARE_BITS_NUM-1):0] bc_cnt_radr_p;
  // --- pre-computed up and down next values of saturation counter ---
  wire [1:0] bc_cnt_value_up = (bc_cnt_value_i == 2'b11) ? bc_cnt_value_i : (bc_cnt_value_i + 1'b1);
  wire [1:0] bc_cnt_value_dn = (bc_cnt_value_i == 2'b00) ? bc_cnt_value_i : (bc_cnt_value_i - 1'b1);
  // --- data for update saturation counters and branch history ---
  always @(posedge cpu_clk) begin
    if (padv_dcod_i & (fetch_op_bf_i | fetch_op_bnf_i)) begin // PENDING FOR SATURATION COUNTERS
      bc_cnt_value_up_p <= bc_cnt_value_up; // PENDING FOR SATURATION COUNTERS
      bc_cnt_value_dn_p <= bc_cnt_value_dn; // PENDING FOR SATURATION COUNTERS
      bc_cnt_radr_p     <= bc_cnt_radr_i;   // PENDING FOR SATURATION COUNTERS
    end
  end

  // --- write to saturation counters and branch history (1-clock) ---
  //     !!! use-bc-instant-p couldn't be overlapped by predict
  //     !!! resolving events (from prev. l.bf/l.bnf) because
  //     !!! DECODE is locked till prediction resolving
  always @(posedge cpu_clk) begin
    if (cpu_rst)
      bc_cnt_we_o <= 1'b0;
    else if (predict_hit | jb_fsm_predict_miss_state)
      bc_cnt_we_o <= 1'b1;
    else
      bc_cnt_we_o <= 1'b0;
  end // at clock
  // --- saturation counter update address ---
  always @(posedge cpu_clk) begin
    if (predict_hit | jb_fsm_predict_miss_state)
      bc_cnt_wadr_o <= bc_cnt_radr_p;
  end // at clock
  // --- data for saturation counters and branch history ---
  always @(posedge cpu_clk) begin
    if (predict_hit) begin
      bc_hist_taken_o <= predict_bc_taken_r;
      bc_cnt_wdat_o   <= predict_bc_taken_r ? bc_cnt_value_up_p : bc_cnt_value_dn_p;
    end
    else if (jb_fsm_predict_miss_state) begin
      bc_hist_taken_o <= (~predict_bc_taken_r);
      bc_cnt_wdat_o   <= (~predict_bc_taken_r) ? bc_cnt_value_up_p : bc_cnt_value_dn_p;
    end
  end // at clock



  //--------------------//
  // Write Back latches //
  //--------------------//

  // WB JUMP or BRANCH attributes
  //  # flags
  always @(posedge cpu_clk) begin
    if (cpu_rst | pipeline_flush_i) begin
      wb_jump_or_branch_o <= 1'b0;
      wb_do_branch_o      <= 1'b0;
    end
    else if (padv_wb_i) begin
      wb_jump_or_branch_o <= ocbco[OCBTC_JUMP_OR_BRANCH_POS];
      wb_do_branch_o      <= jb_attr_c_ocbo[JB_ATTR_C_DO_BRANCH_POS];
    end
    else begin
      wb_jump_or_branch_o <= 1'b0; // 1-clock length
      wb_do_branch_o      <= 1'b0;
    end
  end // @clock
  //  # target
  always @(posedge cpu_clk) begin
    if (padv_wb_i)
      wb_do_branch_target_o <= jb_attr_d_ocbo;
  end // @clock



  //   Flag to enabel/disable exterlal interrupts processing
  // depending on the fact is instructions restartable or not
  assign exec_interrupts_en_o     = ocbao[OCBTA_INTERRUPTS_EN_POS];

  // pre-WB l.rfe
  assign exec_op_rfe_o            = ocbao[OCBTA_OP_RFE_POS];
  // IFETCH exceptions
  assign exec_except_ibus_err_o   = ocbao[5];
  assign exec_except_ipagefault_o = ocbao[4];
  assign exec_except_itlb_miss_o  = ocbao[3];
  assign exec_except_ibus_align_o = jb_attr_c_ocbo[JB_ATTR_C_IBUS_ALIGN_POS];
  // DECODE exceptions
  assign exec_except_illegal_o    = ocbao[2];
  assign exec_except_syscall_o    = ocbao[1];
  assign exec_except_trap_o       = ocbao[0];

  // PC at EXECUTE (moslty for debugging)
  wire [OPTION_OPERAND_WIDTH-1:0] pc_exec = ocbao[OCBTA_PC_MSB:OCBTA_PC_LSB];

  // instuction requests write-back
  wire exec_rfd1_wb = ocbao[OCBTA_RFD1_WB_POS];
  wire exec_rfd2_wb = ocbao[OCBTA_RFD2_WB_POS];

  // destiny addresses
  wire [OPTION_RF_ADDR_WIDTH-1:0] exec_rfd1_adr = ocbao[OCBTA_RFD1_ADR_MSB:OCBTA_RFD1_ADR_LSB];
  wire [OPTION_RF_ADDR_WIDTH-1:0] exec_rfd2_adr = ocbao[OCBTA_RFD2_ADR_MSB:OCBTA_RFD2_ADR_LSB];


  // special WB-controls for RF
  //  if A(B)'s address is odd than A2(B2)=A(B)+1 is even and vise verse
  //  1-clock WB-pulses
  always @(posedge cpu_clk) begin
    if (cpu_rst | pipeline_flush_i) begin
      wb_rf_even_wb_o <= 1'b0;
      wb_rf_odd_wb_o  <= 1'b0;
    end
    else if (padv_wb_i) begin
      wb_rf_even_wb_o <= (exec_rfd1_wb & ~exec_rfd1_adr[0]) | (exec_rfd2_wb & ~exec_rfd2_adr[0]);
      wb_rf_odd_wb_o  <= (exec_rfd1_wb &  exec_rfd1_adr[0]) | (exec_rfd2_wb &  exec_rfd2_adr[0]);
    end
    else begin
      wb_rf_even_wb_o <= 1'b0;
      wb_rf_odd_wb_o  <= 1'b0;
    end
  end // @clock
  //   Even/Odd WB-addresses
  always @(posedge cpu_clk) begin
    if (padv_wb_i) begin
      wb_rf_even_addr_o <= exec_rfd1_adr[0] ? exec_rfd2_adr : exec_rfd1_adr;
      wb_rf_odd_addr_o  <= exec_rfd1_adr[0] ? exec_rfd1_adr : exec_rfd2_adr;
    end
  end // @clock


  // D1 WB-to-RF request (1-clock length)
  always @(posedge cpu_clk) begin
    if (cpu_rst | pipeline_flush_i)
      wb_rfd1_wb_o <= 1'b0;
    else if (padv_wb_i)
      wb_rfd1_wb_o <= exec_rfd1_wb;
    else
      wb_rfd1_wb_o <= 1'b0;
  end // @clock


  // D2 WB-to-RF request (1-clock length)
  always @(posedge cpu_clk) begin
    if (cpu_rst | pipeline_flush_i)
      wb_rfd2_wb_o <= 1'b0;
    else if (padv_wb_i)
      wb_rfd2_wb_o <= exec_rfd2_wb;
    else
      wb_rfd2_wb_o <= 1'b0;
  end // @clock


  // OMAN WB-to-FLAG-request
  always @(posedge cpu_clk) begin
    if (cpu_rst | pipeline_flush_i)
      wb_flag_wb_o <= 1'b0;
    else if (padv_wb_i)
      wb_flag_wb_o <= ocbao[OCBTA_FLAG_WB_POS];
    else
      wb_flag_wb_o <= 1'b0;
  end // @clock


  // WB-to-CARRY-request (1-clock to prevent extra writes)
  always @(posedge cpu_clk) begin
    if (cpu_rst | pipeline_flush_i)
      wb_carry_wb_o <= 1'b0;
    else if (padv_wb_i)
      wb_carry_wb_o <= ocbao[OCBTA_CARRY_WB_POS];
    else
      wb_carry_wb_o <= 1'b0;
  end // @clock


  // WB delay slot
  always @(posedge cpu_clk) begin
    if (cpu_rst | pipeline_flush_i)
      wb_delay_slot_o <= 1'b0;
    else if (padv_wb_i)
      wb_delay_slot_o <= ocbao[OCBTA_DELAY_SLOT_POS];
  end // @clock


  // address of D1
  always @(posedge cpu_clk) begin
    if (padv_wb_i)
      wb_rfd1_adr_o <= exec_rfd1_adr;
  end // @clock


  // address of D2
  always @(posedge cpu_clk) begin
    if (padv_wb_i)
      wb_rfd2_adr_o <= exec_rfd2_adr;
  end // @clock


  // extention bits: valid for 1-clock
  // to reolve hazards in rezervation stations
  always @(posedge cpu_clk) begin
    if (cpu_rst | pipeline_flush_i)
      wb_ext_bits_o <= {DEST_EXT_ADDR_WIDTH{1'b0}};
    else if (padv_wb_i)
      wb_ext_bits_o <= exec_ext_bits;
    else
      wb_ext_bits_o <= {DEST_EXT_ADDR_WIDTH{1'b0}};
  end // @clock


  // PC
  always @(posedge cpu_clk) begin
    if (padv_wb_i)
      pc_wb_o <= pc_exec;
  end // @clock

endmodule // mor1kx_oman_marocchino
