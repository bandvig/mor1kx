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
  parameter DEST_EXT_ADDR_WIDTH  =  3  // log2(Order Control Buffer depth)
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
  output                                fetch_jr_bc_hazard_o, // jump/branch data not ready
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

  // [O]rder [C]ontrol [B]uffer [T]ap layout
  //  [0...5] Exceptions generated by FETCH & DECODE.
  //    (a) Doesn't include IBUS align violation.
  //        It goes through "jump/branch attributes order control buffer".
  //    (b) LSU exceptions go to WB around any OCB
  //  [6] Flag that external interrupt is enabled (instruction is re-startable)
  localparam  OCBT_INTERRUPTS_EN_POS  = 6;
  //  Unit wise requested/ready
  localparam  OCBT_OP_PUSH_WB_POS     = OCBT_INTERRUPTS_EN_POS  + 1;
  localparam  OCBT_JUMP_OR_BRANCH_POS = OCBT_OP_PUSH_WB_POS     + 1;
  localparam  OCBT_OP_1CLK_POS        = OCBT_JUMP_OR_BRANCH_POS + 1;
  localparam  OCBT_OP_DIV_POS         = OCBT_OP_1CLK_POS        + 1;
  localparam  OCBT_OP_MUL_POS         = OCBT_OP_DIV_POS         + 1;
  localparam  OCBT_OP_FPXX_ARITH_POS  = OCBT_OP_MUL_POS         + 1; // arithmetic part only
  localparam  OCBT_RFD2_WB_POS        = OCBT_OP_FPXX_ARITH_POS  + 1;
  localparam  OCBT_OP_FP64_CMP_POS    = OCBT_RFD2_WB_POS        + 1; // source for granting write back to fpxx comparison
  localparam  OCBT_OP_LS_POS          = OCBT_OP_FP64_CMP_POS    + 1; // load / store (we need it for pushing LSU exceptions)
  localparam  OCBT_OP_RFE_POS         = OCBT_OP_LS_POS          + 1;
  //  Instruction is in delay slot
  localparam  OCBT_DELAY_SLOT_POS     = OCBT_OP_RFE_POS         + 1;
  //  Instruction writting comparison flag
  localparam  OCBT_FLAG_WB_POS        = OCBT_DELAY_SLOT_POS     + 1; // any such instruction
  //  Instruction writting carry flag
  localparam  OCBT_CARRY_WB_POS       = OCBT_FLAG_WB_POS        + 1;
  //  Instruction generates WB to D1
  localparam  OCBT_RFD1_WB_POS        = OCBT_CARRY_WB_POS       + 1;
  localparam  OCBT_RFD1_ADR_LSB       = OCBT_RFD1_WB_POS        + 1;
  localparam  OCBT_RFD1_ADR_MSB       = OCBT_RFD1_WB_POS        + OPTION_RF_ADDR_WIDTH;
  //  Instruction generates WB to D2
  localparam  OCBT_RFD2_ADR_LSB       = OCBT_RFD1_ADR_MSB       + 1;
  localparam  OCBT_RFD2_ADR_MSB       = OCBT_RFD1_ADR_MSB       + OPTION_RF_ADDR_WIDTH;
  //  Extention to address of RFD, FLAG and CARRY
  localparam  OCBT_EXT_ADR_LSB        = OCBT_RFD2_ADR_MSB       + 1;
  localparam  OCBT_EXT_ADR_MSB        = OCBT_RFD2_ADR_MSB       + DEST_EXT_ADDR_WIDTH;
  //  Program counter
  localparam  OCBT_PC_LSB             = OCBT_EXT_ADR_MSB        + 1;
  localparam  OCBT_PC_MSB             = OCBT_EXT_ADR_MSB        + OPTION_OPERAND_WIDTH;
  //  value of MSB of order control buffer tap
  localparam  OCBT_MSB                = OCBT_PC_MSB;
  localparam  OCBT_WIDTH              = OCBT_MSB                + 1;


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

  // input pack
  wire  [OCBT_MSB:0] ocbi;
  assign ocbi = { // various instruction related information
                  pc_decode_i,            // instruction virtual address
                  dcod_ext_bits_r,        // extension to RFD, FLAG or CARRY
                  dcod_rfd2_adr_i,        // WB address D2
                  dcod_rfd1_adr_i,        // WB address D1
                  dcod_rfd1_wb_i,         // instruction generates WB
                  dcod_carry_wb_i,        // istruction affects carry flag
                  dcod_flag_wb_i,         // any instruction which affects comparison flag
                  dcod_delay_slot_i,      // istruction is in delay slot
                  // unit that must be granted for WB
                  dcod_op_rfe_i,
                  dcod_op_ls_i,      // load / store (we need it for pushing LSU exceptions)
                  dcod_op_fp64_cmp_i,
                  dcod_rfd2_wb_i, // MAROCCHINO_TODO: move it near to D2 adr
                  dcod_op_fpxx_arith_i,
                  dcod_op_mul_i,
                  dcod_op_div_i,
                  dcod_op_1clk_i,
                  dcod_op_jb_r,
                  dcod_op_push_wb_i, // OMAN entrance
                  // Flag that istruction is restartable
                  interrupts_en,
                  // FETCH & DECODE exceptions
                  dcod_fetch_except_ibus_err_r,
                  dcod_fetch_except_ipagefault_r,
                  dcod_fetch_except_itlb_miss_r,
                  dcod_except_illegal_i,
                  dcod_except_syscall_i,
                  dcod_except_trap_i };


  wire [OCBT_MSB:0] ocbo00;

  wire ocb_full, ocb_empty;


  //-------------------------------//
  // Order Control Buffer instance //
  //-------------------------------//

  mor1kx_ocb_marocchino
  #(
    .NUM_TAPS   (EXT_BITS_MAX-1), // INSN_OCB: extention bits "0" is reserved as "not used"
    .NUM_OUTS   (1),              // INSN_OCB
    .DATA_SIZE  (OCBT_WIDTH)      // INSN_OCB
  )
  u_ocb
  (
    // clocks, resets
    .clk              (cpu_clk), // INSN_OCB
    .rst              (cpu_rst), // INSN_OCB
    // pipe controls
    .pipeline_flush_i (pipeline_flush_i), // INSN_OCB
    .write_i          (padv_exec_i), // INSN_OCB
    .read_i           (padv_wb_i), // INSN_OCB
    // value at reset/flush
    .reset_taps       (cpu_rst | pipeline_flush_i), // INSN_OCB
    .default_value_i  ({OCBT_WIDTH{1'b0}}), // INSN_OCB
    // data input
    .ocbi_i           (ocbi), // INSN_OCB
    // "OCB is empty" flag
    .empty_o          (ocb_empty), // INSN_OCB
    // "OCB is full" flag
    //   (a) external control logic must stop the "writing without reading"
    //       operation if OCB is full
    //   (b) however, the "writing + reading" is possible
    //       because it just pushes OCB and keeps it full
    .full_o           (ocb_full), // INSN_OCB
    // output layout
    // { out[n-1], out[n-2], ... out[0] } : DECODE (entrance) -> EXECUTE (exit)
    .ocbo_o           (ocbo00) // INSN_OCB
  );


  // Grant WB-access to units
  assign grant_wb_to_1clk_o        = ocbo00[OCBT_OP_1CLK_POS];
  assign grant_wb_to_div_o         = ocbo00[OCBT_OP_DIV_POS];
  assign grant_wb_to_mul_o         = ocbo00[OCBT_OP_MUL_POS];
  assign grant_wb_to_fpxx_arith_o  = ocbo00[OCBT_OP_FPXX_ARITH_POS];
  assign grant_wb_to_lsu_o         = ocbo00[OCBT_OP_LS_POS];
  assign grant_wb_to_fp64_cmp_o    = ocbo00[OCBT_OP_FP64_CMP_POS];


  //--------------------------//
  // Analysis of data hazards //
  //--------------------------//

  // Shorten aliases
  wire [DEST_EXT_ADDR_WIDTH-1:0] exec_ext_bits = ocbo00[OCBT_EXT_ADR_MSB:OCBT_EXT_ADR_LSB];

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
  //  # forward declarations
  wire jr_hazard_d1b1_set;
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
          dcod_wb2dec_d1b1_fwd_o <= (hazard_d1b1_set | jr_hazard_d1b1_set) & exe2fth_dxb1;
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
    (ocbo00[OCBT_OP_1CLK_POS] & ~ocbo00[OCBT_JUMP_OR_BRANCH_POS]) | // EXEC VALID: but wait attributes for l.jal/ljalr
    (exec_jb_attr_valid       &  ocbo00[OCBT_JUMP_OR_BRANCH_POS]) | // EXEC VALID
    (div_valid_i              &  ocbo00[OCBT_OP_DIV_POS])         | // EXEC VALID
    (mul_valid_i              &  ocbo00[OCBT_OP_MUL_POS])         | // EXEC VALID
    (fpxx_arith_valid_i       &  ocbo00[OCBT_OP_FPXX_ARITH_POS])  | // EXEC VALID
    (fp64_cmp_valid_i         &  ocbo00[OCBT_OP_FP64_CMP_POS])    | // EXEC VALID
    (lsu_valid_i              &  ocbo00[OCBT_OP_LS_POS])          | // EXEC VALID
                                 ocbo00[OCBT_OP_PUSH_WB_POS];       // EXEC VALID

  // DECODE valid
  assign dcod_valid_o = (~ocb_full) &                            // DECODE VALID
                        ((dcod_op_1clk_i     & op_1clk_free_i) | // DECODE VALID
                         (dcod_op_muldiv_i   & muldiv_free_i)  | // DECODE VALID
                         (dcod_op_fpxx_any_i & fpxx_free_i)    | // DECODE VALID
                         (dcod_op_lsu_any_i  & lsu_free_i)     | // DECODE VALID
                         (dcod_op_mXspr_i    & ocb_empty)      | // DECODE VALID
                         dcod_op_push_exec_i);                   // DECODE VALID


  //---------------------------------------//
  // Logic to support Jump / Branch taking //
  //---------------------------------------//

  // state machine for tracking Jump / Branch related hazards
  localparam [5:0] JB_FSM_CATCHING_JB  = 6'b000001, // on IFETCH output
                   JB_FSM_CHK_HAZARDS  = 6'b000010, // on DECODE
                   JB_FSM_WAITING_B1   = 6'b000100, // waiting rB for l.jr/ljalr
                   JB_FSM_WAITING_FLAG = 6'b001000, // waiting SR[F] for l.bf/l.bnf
                   JB_FSM_DOING_JR     = 6'b010000, // execute l.jr/ljalr
                   JB_FSM_DOING_BC     = 6'b100000; // execute l.bf/l.bnf
  // ---
  reg [5:0] jb_fsm_state_r;
  // --- particular states ---
  wire jb_fsm_waiting_b1_state   = jb_fsm_state_r[2];
  wire jb_fsm_waiting_flag_state = jb_fsm_state_r[3];
  wire jb_fsm_doing_jr_state     = jb_fsm_state_r[4];
  wire jb_fsm_doing_bc_state     = jb_fsm_state_r[5];


  // --- detect b1 hazard for l.jr/l.jalr ---
  assign jr_hazard_d1b1_set = rat_rd1_alloc[fetch_rfb1_adr_i] & fetch_op_jr_i;
  reg    jr_hazard_d1b1_r;
  // ---
  always @(posedge cpu_clk) begin
    if (cpu_rst | pipeline_flush_i)
      jr_hazard_d1b1_r <= 1'b0;
    else begin
      // synthesis parallel_case full_case
      case({padv_wb_i,padv_dcod_i})
        // only FETCH->DECODE
        2'b01:   jr_hazard_d1b1_r <= jr_hazard_d1b1_set;
        // write-back overlapping FETCH->DECODE
        2'b11:   jr_hazard_d1b1_r <= jr_hazard_d1b1_set & no_exe2fth_dxb1;
        // 1-clock length
        default: jr_hazard_d1b1_r <= 1'b0;
      endcase
    end
  end // at clock


  // --- detect flag hazard for l.bf/l.bnf ---
  reg                            flag_alloc_r;     // SR[F] allocated for write-back
  reg  [DEST_EXT_ADDR_WIDTH-1:0] flag_alloc_ext_r; // SR[F] allocation index
  // ---
  wire keep_flag_alloc_at_wb = flag_alloc_r & (flag_alloc_ext_r != exec_ext_bits);
  // ---
  always @(posedge cpu_clk) begin
    if (cpu_rst | pipeline_flush_i)
      flag_alloc_r <= 1'b0;
    else begin
      // synthesis parallel_case full_case
      case ({padv_wb_i,padv_dcod_i})
        // FETCH->DECODE
        2'b01: flag_alloc_r <= fetch_flag_wb_i ? 1'b1 : flag_alloc_r;
        // WB-only
        2'b10: flag_alloc_r <= keep_flag_alloc_at_wb;
        // overlapping
        2'b11: flag_alloc_r <= fetch_flag_wb_i ? 1'b1 : keep_flag_alloc_at_wb;
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


  // --- DECODE stage J/B instruction flags ---
  reg dcod_op_jr_r, dcod_op_bf_r, dcod_op_bnf_r, dcod_op_bc_r;

  // --- various pendings till rB/flag computationcompletion ---
  reg  [DEST_EXT_ADDR_WIDTH-1:0] jb_hazard_ext_p;
  reg [OPTION_OPERAND_WIDTH-1:0] jb_target_p;
  reg                      [1:0] jb_bf_p;
  reg                      [1:0] jb_bnf_p;


  // Jump / Branch state machine
  always @(posedge cpu_clk) begin
    if (cpu_rst | pipeline_flush_i) begin
      jb_fsm_state_r  <= JB_FSM_CATCHING_JB;
      // on DECODE stage
      dcod_op_jr_r    <= 1'b0;
      dcod_op_bf_r    <= 1'b0;
      dcod_op_bnf_r   <= 1'b0;
      dcod_op_bc_r    <= 1'b0;
      // for l.bf/l.bnf execution
      jb_bf_p         <= 2'b00;
      jb_bnf_p        <= 2'b00;
      // other attributes
      jb_hazard_ext_p <= {DEST_EXT_ADDR_WIDTH{1'b0}};
      jb_target_p     <= {OPTION_OPERAND_WIDTH{1'b0}};
    end
    else begin
      // synthesis parallel_case full_case
      case (jb_fsm_state_r)
        // catching j/b on IFETCH output
        JB_FSM_CATCHING_JB : begin
          if (padv_dcod_i) begin
            // store "to immediate" target permanently to simplify logic
            jb_target_p <= fetch_to_imm_target_i;
            // next state ?
            if (fetch_op_jr_i | fetch_op_bf_i | fetch_op_bnf_i) begin
              jb_fsm_state_r  <= JB_FSM_CHK_HAZARDS;
              // on DECODE stage
              dcod_op_jr_r    <= fetch_op_jr_i;
              dcod_op_bf_r    <= fetch_op_bf_i;
              dcod_op_bnf_r   <= fetch_op_bnf_i;
              dcod_op_bc_r    <= fetch_op_bf_i | fetch_op_bnf_i;
            end
          end
        end
        // checking j/b related hazards in DECODE
        JB_FSM_CHK_HAZARDS : begin
          if (dcod_op_jr_r) begin
            // select next state
            if (jr_hazard_d1b1_r) begin
              jb_fsm_state_r  <= JB_FSM_WAITING_B1;
              jb_hazard_ext_p <= omn2dec_hazard_dxb1_adr_o;
            end
            else begin // no rB related hazards
              jb_fsm_state_r <= JB_FSM_DOING_JR;
              jb_target_p    <= dcod_rfb1_jr_i;
            end
          end // l.jr/l.jalr
          else if (dcod_op_bc_r) begin
            // select next state
            if (flag_alloc_r) begin
              jb_fsm_state_r  <= JB_FSM_WAITING_FLAG;
              jb_hazard_ext_p <= flag_alloc_ext_r;
              jb_bf_p         <= {1'b0,dcod_op_bf_r};
              jb_bnf_p        <= {1'b0,dcod_op_bnf_r};
            end
            else if (wb_flag_wb_o) begin
              jb_fsm_state_r <= JB_FSM_DOING_BC;
              jb_bf_p        <= {dcod_op_bf_r,1'b0};
              jb_bnf_p       <= {dcod_op_bnf_r,1'b0};
            end
            else // no SR[F] related hazards
              jb_fsm_state_r <= JB_FSM_CATCHING_JB;
          end // l.bf/l.bnf
          // drop DECODE "ops"
          dcod_op_jr_r    <= 1'b0;
          dcod_op_bf_r    <= 1'b0;
          dcod_op_bnf_r   <= 1'b0;
          dcod_op_bc_r    <= 1'b0;
        end
        // waiting address for jump
        JB_FSM_WAITING_B1 : begin
          if (jb_hazard_ext_p == wb_ext_bits_o) begin
            jb_fsm_state_r <= JB_FSM_DOING_JR;
            jb_target_p    <= wb_result1_i;
          end
        end
        // waiting flag computation
        JB_FSM_WAITING_FLAG : begin
          if (jb_hazard_ext_p == wb_ext_bits_o) begin
            jb_fsm_state_r <= JB_FSM_DOING_BC;
            jb_bf_p        <= {jb_bf_p[0],1'b0};
            jb_bnf_p       <= {jb_bnf_p[0],1'b0};
          end
        end
        // doing j/b
        JB_FSM_DOING_JR,
        JB_FSM_DOING_BC : begin
          jb_fsm_state_r <= JB_FSM_CATCHING_JB;
          jb_bf_p        <= 2'b00;
          jb_bnf_p       <= 2'b00;
        end
        // others
        default:;
      endcase
    end
  end // @cpu-clock


  // do_branch_o is meaningless if fetch_jr_bc_hazard_o is rised
  assign do_branch_o = fetch_op_jimm_i       | // do jump to immediate
                       jb_fsm_doing_jr_state | // do jump to register (B1)
                       ((dcod_op_bf_r  | jb_bf_p[1])  &   ctrl_flag_sr_i)  | // do conditional branch
                       ((dcod_op_bnf_r | jb_bnf_p[1]) & (~ctrl_flag_sr_i));  // do conditional branch
  // branch target
  assign do_branch_target_o = fetch_op_jimm_i ? fetch_to_imm_target_i : // branch target selection
                                                jb_target_p ;           // branch target selection


  // Detect IBUS align violation on OMAN. For JB_ATTR_OCB only.
  wire oman_except_ibus_align = jb_fsm_doing_jr_state & (|jb_target_p[1:0]);


  // Combine IFETCH hazards
  assign fetch_jr_bc_hazard_o = (fetch_op_jr_i | fetch_op_bf_i | fetch_op_bnf_i) |  // detect l.jr/l.jalr/l.bf/l.bnf
                                dcod_op_jr_r   | jb_fsm_waiting_b1_state         |  // waiting rB for l.jr/l.jalr
                                (dcod_op_bc_r & (flag_alloc_r | wb_flag_wb_o))   |  // waiting flag for l.bf/l.bnf
                                jb_fsm_waiting_flag_state;                          // waiting flag for l.bf/l.bnf

  // jump / branch has no any hazarad (valid command)
  wire jr_bc_valid = fetch_op_jimm_i                 | // jump to immediate is valid permanently
                     jb_fsm_doing_jr_state           | // l.jr/l.jalr hazards are resolved
                     (dcod_op_bc_r & (~flag_alloc_r) & (~wb_flag_wb_o)) | // valid flag for l.bf/l.bnf in DECODE
                     jb_fsm_doing_bc_state;                               // l.bf/l.bnf hazards are resolved


  // JUMP/BRANCH attribute flags
  localparam  JB_ATTR_DO_BRANCH_POS         = 0; // if "do" the "target" makes sence
  localparam  JP_ATTR_EXCEPT_IBUS_ALIGN_POS = JB_ATTR_DO_BRANCH_POS         + 1;
  localparam  JB_ATTR_VALID_POS             = JP_ATTR_EXCEPT_IBUS_ALIGN_POS + 1;
  localparam  JB_ATTR_INSN                  = JB_ATTR_VALID_POS             + 1;
  //---
  localparam  JB_ATTR_FLAGS_MSB             = JB_ATTR_INSN;
  localparam  JB_ATTR_FLAGS_WIDTH           = JB_ATTR_FLAGS_MSB + 1;
  //---
  reg  [JB_ATTR_FLAGS_MSB:0]        jb_attr_flags_r;
  reg  [(OPTION_OPERAND_WIDTH-1):0] jb_attr_target_r;
  // --- JB attributes register controls ---
  wire jb_attr_miss  = jb_attr_flags_r[JB_ATTR_INSN] & (~jb_attr_flags_r[JB_ATTR_VALID_POS]);
  wire jb_attr_write = (padv_dcod_i & fetch_op_jb_i) | jb_attr_miss;
  wire jb_attr_read  = padv_wb_i & ocbo00[OCBT_JUMP_OR_BRANCH_POS];
  // ---
  always @(posedge cpu_clk) begin
    if (cpu_rst | pipeline_flush_i)
      jb_attr_flags_r <= {JB_ATTR_FLAGS_WIDTH{1'b0}};
    else if (jb_attr_write) // waiting for resolving j/b hazards
      jb_attr_flags_r <= {1'b1,jr_bc_valid,oman_except_ibus_align,do_branch_o};
    else if (jb_attr_read)
      jb_attr_flags_r <= {JB_ATTR_FLAGS_WIDTH{1'b0}};
  end
  // ---
  always @(posedge cpu_clk) begin
    if (jb_attr_write) // waiting for resolving j/b hazards
      jb_attr_target_r <= do_branch_target_o;
  end
  // --- JB_ATTR_OCB output unpack ---
  assign exec_jb_attr_valid = jb_attr_flags_r[JB_ATTR_VALID_POS];

  // DECODE is "locked" flag.
  // We set the flag after delay slot has passed into EXEC
  // and till pushing Jump/Branch attributes into WB.
  // This prevents possible overriding of Jump/Branch
  // attributes register with possible Jump/Brunch instruction
  // followed by delay slot.
  // ---
  reg  dcod_locked_r;
  // ---
  always @(posedge cpu_clk) begin
    if (cpu_rst | pipeline_flush_i)
      dcod_locked_r <= 1'b0;
    else if (dcod_locked_r) begin
      if (padv_wb_i) // MAROCCHINO_TODO: sync to JB attribute register update?
        dcod_locked_r <= (~ocbo00[OCBT_JUMP_OR_BRANCH_POS]); // Pushing Jumb/Branch to WB unlocks DECODE
    end
    else if (padv_dcod_i)
      dcod_locked_r <= fetch_valid_i & fetch_delay_slot_i &
                       jb_attr_flags_r[JB_ATTR_INSN] & ((~exec_jb_attr_valid) | (~ocbo00[OCBT_JUMP_OR_BRANCH_POS]));
  end // at clock
  // ---
  assign dcod_free_o = (~dcod_locked_r);


  // WB JUMP or BRANCH attributes
  //  # flags
  always @(posedge cpu_clk) begin
    if (cpu_rst | pipeline_flush_i) begin
      wb_jump_or_branch_o <= 1'b0;
      wb_do_branch_o      <= 1'b0;
    end
    else if (padv_wb_i) begin
      wb_jump_or_branch_o <= ocbo00[OCBT_JUMP_OR_BRANCH_POS];
      wb_do_branch_o      <= jb_attr_flags_r[JB_ATTR_DO_BRANCH_POS];
    end
    else begin
      wb_jump_or_branch_o <= 1'b0; // 1-clock length
      wb_do_branch_o      <= 1'b0;
    end
  end // @clock
  //  # target
  always @(posedge cpu_clk) begin
    if (padv_wb_i)
      wb_do_branch_target_o <= jb_attr_target_r;
  end // @clock



  //   Flag to enabel/disable exterlal interrupts processing
  // depending on the fact is instructions restartable or not
  assign exec_interrupts_en_o     = ocbo00[OCBT_INTERRUPTS_EN_POS];

  // pre-WB l.rfe
  assign exec_op_rfe_o            = ocbo00[OCBT_OP_RFE_POS];
  // IFETCH exceptions
  assign exec_except_ibus_err_o   = ocbo00[5];
  assign exec_except_ipagefault_o = ocbo00[4];
  assign exec_except_itlb_miss_o  = ocbo00[3];
  assign exec_except_ibus_align_o = jb_attr_flags_r[JP_ATTR_EXCEPT_IBUS_ALIGN_POS];
  // DECODE exceptions
  assign exec_except_illegal_o    = ocbo00[2];
  assign exec_except_syscall_o    = ocbo00[1];
  assign exec_except_trap_o       = ocbo00[0];

  // PC at EXECUTE (moslty for debugging)
  wire [OPTION_OPERAND_WIDTH-1:0] pc_exec = ocbo00[OCBT_PC_MSB:OCBT_PC_LSB];

  // instuction requests write-back
  wire exec_rfd1_wb = ocbo00[OCBT_RFD1_WB_POS];
  wire exec_rfd2_wb = ocbo00[OCBT_RFD2_WB_POS];

  // destiny addresses
  wire [OPTION_RF_ADDR_WIDTH-1:0] exec_rfd1_adr = ocbo00[OCBT_RFD1_ADR_MSB:OCBT_RFD1_ADR_LSB];
  wire [OPTION_RF_ADDR_WIDTH-1:0] exec_rfd2_adr = ocbo00[OCBT_RFD2_ADR_MSB:OCBT_RFD2_ADR_LSB];


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
      wb_flag_wb_o <= ocbo00[OCBT_FLAG_WB_POS];
    else
      wb_flag_wb_o <= 1'b0;
  end // @clock


  // WB-to-CARRY-request (1-clock to prevent extra writes)
  always @(posedge cpu_clk) begin
    if (cpu_rst | pipeline_flush_i)
      wb_carry_wb_o <= 1'b0;
    else if (padv_wb_i)
      wb_carry_wb_o <= ocbo00[OCBT_CARRY_WB_POS];
    else
      wb_carry_wb_o <= 1'b0;
  end // @clock


  // WB delay slot
  always @(posedge cpu_clk) begin
    if (cpu_rst | pipeline_flush_i)
      wb_delay_slot_o <= 1'b0;
    else if (padv_wb_i)
      wb_delay_slot_o <= ocbo00[OCBT_DELAY_SLOT_POS];
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
