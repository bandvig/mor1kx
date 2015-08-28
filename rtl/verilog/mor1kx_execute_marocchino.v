/* ****************************************************************************
  This Source Code Form is subject to the terms of the
  Open Hardware Description License, v. 1.0. If a copy
  of the OHDL was not distributed with this file, You
  can obtain one at http://juliusbaxter.net/ohdl/ohdl.txt

  Description: mor1kx execute stage for MAROCCHINO pipeline

  Derived from mor1kx_execute_alu and mor1kx_execute_ctrl_cappuccino

  Copyright (C) 2012 Julius Baxter <juliusbaxter@gmail.com>
  Copyright (C) 2012-2014 Stefan Kristiansson <stefan.kristiansson@saunalahti.fi>
  Copyright (C) 2015 Andrey Bacherov <avbacherov@opencores.org>

***************************************************************************** */

`include "mor1kx-defines.v"

module mor1kx_execute_marocchino
#(
  parameter OPTION_OPERAND_WIDTH = 32,
  parameter OPTION_RF_ADDR_WIDTH =  5,

  parameter FEATURE_OVERFLOW   = "NONE",
  parameter FEATURE_CARRY_FLAG = "ENABLED",

  parameter FEATURE_EXT = "NONE",

  parameter FEATURE_FPU = "NONE" // ENABLED|NONE
)
(
  // clocks and resets
  input                                 clk,
  input                                 rst,

  // pipeline control signal in
  input                                 padv_decode_i,
  input                                 padv_wb_i,
  input                                 pipeline_flush_i,// flush pipelined fpu

  // input data
  input      [OPTION_OPERAND_WIDTH-1:0] rfa_i,
  input      [OPTION_OPERAND_WIDTH-1:0] rfb_i,
  input      [OPTION_OPERAND_WIDTH-1:0] immediate_i,
  input                                 immediate_sel_i,

  // opcode for alu
  input       [`OR1K_ALU_OPC_WIDTH-1:0] opc_alu_i,
  input       [`OR1K_ALU_OPC_WIDTH-1:0] opc_alu_secondary_i,

  // adder's inputs
  input                                 op_add_i,
  input                                 adder_do_sub_i,
  input                                 adder_do_carry_i,

  // shift, ffl1, movhi, cmov
  input                                 op_shift_i,
  input                                 op_ffl1_i,
  input                                 op_movhi_i,
  input                                 op_cmov_i,

  // jump & link
  input                                 op_jal_i,
  input      [OPTION_OPERAND_WIDTH-1:0] exec_jal_result_i,

  // output latches for 1-clock operations
  output reg                            wb_alu_1clk_rdy_o,
  output reg [OPTION_OPERAND_WIDTH-1:0] wb_alu_1clk_result_o,

  // multiplier inputs/outputs
  input                                 op_mul_i,
  output reg [OPTION_OPERAND_WIDTH-1:0] wb_mul_result_o,
  output reg                            wb_mul_rdy_o,

  // dividion inputs
  input                                 op_div_i,
  input                                 op_div_signed_i,
  input                                 op_div_unsigned_i,
  output reg [OPTION_OPERAND_WIDTH-1:0] wb_div_result_o,
  output reg                            wb_div_rdy_o,

  // ALU results
  output     [OPTION_OPERAND_WIDTH-1:0] alu_nl_result_o, // nl: not latched, to WB_MUX
  output     [OPTION_OPERAND_WIDTH-1:0] exec_lsu_adr_o,  // not latched, address to LSU

  // FPU related
  input         [`OR1K_FPUOP_WIDTH-1:0] op_fpu_i,
  input       [`OR1K_FPCSR_RM_SIZE-1:0] fpu_round_mode_i,
  output        [`OR1K_FPCSR_WIDTH-1:0] exec_fpcsr_o,
  output                                exec_fpcsr_set_o,

  // flag related inputs
  input                                 op_setflag_i,
  input                                 flag_i, // fed back from ctrl (for cmov)
  // flag related outputs
  output                                exec_flag_set_o,
  output                                exec_flag_clear_o,

  // carry related inputs
  input                                 carry_i,
  // carry related outputs
  output                                exec_carry_set_o,
  output                                exec_carry_clear_o,

  // owerflow related outputs
  output                                exec_overflow_set_o,
  output                                exec_overflow_clear_o,

  // MSYNC related controls
  input                                 msync_done_i,

  // LSU related inputs
  input                                 lsu_valid_i,
  input                                 lsu_excepts_i,

  // ready flags
  input                                 exec_insn_1clk_i,
  output                                exec_valid_o
);

  localparam  EXEDW = OPTION_OPERAND_WIDTH; // short name


  wire [EXEDW-1:0] op_a = rfa_i;
  wire [EXEDW-1:0] op_b = immediate_sel_i ? immediate_i : rfb_i;


  //------------------//
  // Adder/subtractor //
  //------------------//
  // outputs
  wire             adder_carryout;
  wire [EXEDW-1:0] adder_result;
  // inputs
  wire [EXEDW-1:0] b_mux = adder_do_sub_i ? (~op_b) : op_b;
  wire carry_in = adder_do_sub_i | (adder_do_carry_i & carry_i);
  // Adder
  assign {adder_carryout, adder_result} =
           op_a + b_mux + {{(EXEDW-1){1'b0}},carry_in};
  // result sign
  wire adder_result_sign = adder_result[EXEDW-1];
  // signed overflow detection
  // Input signs are same and result sign is different to input signs
  wire adder_s_ovf =
         (op_a[EXEDW-1] == b_mux[EXEDW-1]) &
         (op_a[EXEDW-1] ^ adder_result[EXEDW-1]);
  // unsigned overflow detection
  wire adder_u_ovf = adder_carryout;


  //------------------//
  // Comparison logic //
  //------------------//
  wire a_eq_b  = (op_a == op_b); // Equal compare
  wire a_lts_b = (adder_result_sign ^ adder_s_ovf); // Signed compare (sign != ovf)
  wire a_ltu_b = ~adder_carryout; // Unsigned compare
  // comb.
  reg flag_set;
  always @*
    case(opc_alu_secondary_i)
      `OR1K_COMP_OPC_EQ:  flag_set = a_eq_b;
      `OR1K_COMP_OPC_NE:  flag_set = ~a_eq_b;
      `OR1K_COMP_OPC_GTU: flag_set = ~(a_eq_b | a_ltu_b);
      `OR1K_COMP_OPC_GTS: flag_set = ~(a_eq_b | a_lts_b);
      `OR1K_COMP_OPC_GEU: flag_set = ~a_ltu_b;
      `OR1K_COMP_OPC_GES: flag_set = ~a_lts_b;
      `OR1K_COMP_OPC_LTU: flag_set = a_ltu_b;
      `OR1K_COMP_OPC_LTS: flag_set = a_lts_b;
      `OR1K_COMP_OPC_LEU: flag_set = a_eq_b | a_ltu_b;
      `OR1K_COMP_OPC_LES: flag_set = a_eq_b | a_lts_b;
      default:            flag_set = 1'b0;
    endcase


  //------//
  // FFL1 //
  //------//
  wire [EXEDW-1:0] ffl1_result;
  assign ffl1_result = (opc_alu_secondary_i[2]) ?
           (op_a[31] ? 32 : op_a[30] ? 31 : op_a[29] ? 30 :
            op_a[28] ? 29 : op_a[27] ? 28 : op_a[26] ? 27 :
            op_a[25] ? 26 : op_a[24] ? 25 : op_a[23] ? 24 :
            op_a[22] ? 23 : op_a[21] ? 22 : op_a[20] ? 21 :
            op_a[19] ? 20 : op_a[18] ? 19 : op_a[17] ? 18 :
            op_a[16] ? 17 : op_a[15] ? 16 : op_a[14] ? 15 :
            op_a[13] ? 14 : op_a[12] ? 13 : op_a[11] ? 12 :
            op_a[10] ? 11 : op_a[9] ? 10 : op_a[8] ? 9 :
            op_a[7] ? 8 : op_a[6] ? 7 : op_a[5] ? 6 : op_a[4] ? 5 :
            op_a[3] ? 4 : op_a[2] ? 3 : op_a[1] ? 2 : op_a[0] ? 1 : 0 ) :
           (op_a[0] ? 1 : op_a[1] ? 2 : op_a[2] ? 3 : op_a[3] ? 4 :
            op_a[4] ? 5 : op_a[5] ? 6 : op_a[6] ? 7 : op_a[7] ? 8 :
            op_a[8] ? 9 : op_a[9] ? 10 : op_a[10] ? 11 : op_a[11] ? 12 :
            op_a[12] ? 13 : op_a[13] ? 14 : op_a[14] ? 15 :
            op_a[15] ? 16 : op_a[16] ? 17 : op_a[17] ? 18 :
            op_a[18] ? 19 : op_a[19] ? 20 : op_a[20] ? 21 :
            op_a[21] ? 22 : op_a[22] ? 23 : op_a[23] ? 24 :
            op_a[24] ? 25 : op_a[25] ? 26 : op_a[26] ? 27 :
            op_a[27] ? 28 : op_a[28] ? 29 : op_a[29] ? 30 :
            op_a[30] ? 31 : op_a[31] ? 32 : 0);


  //----------------//
  // Barrel shifter //
  //----------------//
  // Shifter wires
  wire [`OR1K_ALU_OPC_SECONDARY_WIDTH-1:0] opc_alu_shr;
  assign opc_alu_shr = opc_alu_secondary_i[`OR1K_ALU_OPC_SECONDARY_WIDTH-1:0];
  wire [EXEDW-1:0] shift_result;

  function [EXEDW-1:0] reverse;
  input [EXEDW-1:0] in;
  integer            i;
  begin
    for (i = 0; i < EXEDW; i=i+1) begin
      reverse[(EXEDW-1)-i] = in[i];
    end
  end
  endfunction

  wire op_sll = (opc_alu_shr==`OR1K_ALU_OPC_SECONDARY_SHRT_SLL);
  wire op_srl = (opc_alu_shr==`OR1K_ALU_OPC_SECONDARY_SHRT_SRL);
  wire op_sra = (opc_alu_shr==`OR1K_ALU_OPC_SECONDARY_SHRT_SRA);
  wire op_ror = (opc_alu_shr==`OR1K_ALU_OPC_SECONDARY_SHRT_ROR);

  wire [EXEDW-1:0] shift_right;
  wire [EXEDW-1:0] shift_lsw;
  wire [EXEDW-1:0] shift_msw;

  //
  // Bit-reverse on left shift, perform right shift,
  // bit-reverse result on left shift.
  //
  assign shift_lsw = op_sll ? reverse(op_a) : op_a;
  assign shift_msw = op_sra ? {EXEDW{op_a[EXEDW-1]}} :
                     op_ror ? op_a : {EXEDW{1'b0}};

  assign shift_right = {shift_msw, shift_lsw} >> op_b[4:0];
  assign shift_result = op_sll ? reverse(shift_right) : shift_right;


  //------------------//
  // Conditional move //
  //------------------//
  wire [EXEDW-1:0] cmov_result;
  assign cmov_result = flag_i ? op_a : op_b;


  //--------------------//
  // Logical operations //
  //--------------------//
  // Logic wires
  wire             op_logic;
  reg [EXEDW-1:0]  logic_result;
  // Create a look-up-table for AND/OR/XOR
  reg [3:0] logic_lut;
  always @(*) begin
    case(opc_alu_i)
      `OR1K_ALU_OPC_AND: logic_lut = 4'b1000;
      `OR1K_ALU_OPC_OR:  logic_lut = 4'b1110;
      `OR1K_ALU_OPC_XOR: logic_lut = 4'b0110;
      default:           logic_lut = 4'd0;
    endcase
  end

  // Extract the result, bit-for-bit, from the look-up-table
  integer i;
  always @(*)
    for (i = 0; i < EXEDW; i=i+1) begin
      logic_result[i] = logic_lut[{op_a[i], op_b[i]}];
    end

  assign op_logic = |logic_lut;


  //--------------------------------------//
  // Muxing and registering 1-clk results //
  //--------------------------------------//
  wire [EXEDW-1:0] alu_1clk_result_mux = op_shift_i ? shift_result      :
                                         op_ffl1_i  ? ffl1_result       :
                                         op_add_i   ? adder_result      :
                                         op_logic   ? logic_result      :
                                         op_cmov_i  ? cmov_result       :
                                         op_movhi_i ? immediate_i       :
                                         op_jal_i   ? exec_jal_result_i : // for GPR[9]
                                                      {EXEDW{1'b0}};
  //  registering
  always @(posedge clk `OR_ASYNC_RST) begin
    if (rst)
      wb_alu_1clk_result_o <= {EXEDW{1'b0}};
    else if (exec_insn_1clk_i & padv_wb_i)
      wb_alu_1clk_result_o <= alu_1clk_result_mux;
  end // posedge clock
  // 1clk instruction ready flag
  reg alu_1clk_rdy_stored;
  // ---
  always @(posedge clk `OR_ASYNC_RST) begin
    if (rst) begin
      wb_alu_1clk_rdy_o   <= 1'b0;
      alu_1clk_rdy_stored <= 1'b0;
    end
    else if (pipeline_flush_i) begin
      wb_alu_1clk_rdy_o   <= wb_alu_1clk_rdy_o;
      alu_1clk_rdy_stored <= 1'b0;
    end
    else if (padv_wb_i) begin
      wb_alu_1clk_rdy_o   <= (exec_insn_1clk_i | alu_1clk_rdy_stored);
      alu_1clk_rdy_stored <= 1'b0;
    end
    else if (~alu_1clk_rdy_stored) begin
      wb_alu_1clk_rdy_o   <= wb_alu_1clk_rdy_o;
      alu_1clk_rdy_stored <= exec_insn_1clk_i;
    end
  end // @clock



  //-------------------//
  // 32-bit multiplier //
  //-------------------//
  localparam MULHDW = (OPTION_OPERAND_WIDTH >> 1);

  // algorithm:
  //   AlBl[dw-1:0] = A[hdw-1:0] * B[hdw-1:0];
  //   AhBl[dw-1:0] = A[dw-1:hdw] * B[hdw-1:0];
  //   BhAl[dw-1:0] = B[dw-1:hdw] * A[hdw-1:0];
  //   Sum[dw-1:0]  = {BhAl[hdw-1:0],{hdw{0}}} +
  //                  {AlBl[hdw-1:0],{hdw{0}}} +
  //                  AlBl;

  wire mul_valid; // valid flag is 1-clock ahead of latching for WB
  wire mul_adv = ~mul_valid | padv_wb_i; // advance multiplier pipe

  // stage #1: register inputs & split them on halfed parts
  reg [MULHDW-1:0] mul_s1_al;
  reg [MULHDW-1:0] mul_s1_bl;
  reg [MULHDW-1:0] mul_s1_ah;
  reg [MULHDW-1:0] mul_s1_bh;
  //  registering
  always @(posedge clk) begin
    if (op_mul_i & mul_adv) begin
      mul_s1_al <= op_a[MULHDW-1:0];
      mul_s1_bl <= op_b[MULHDW-1:0];
      mul_s1_ah <= op_a[EXEDW-1:MULHDW];
      mul_s1_bh <= op_b[EXEDW-1:MULHDW];
    end
  end // posedge clock
  //  ready flag
  reg mul_s1_rdy;
  always @(posedge clk `OR_ASYNC_RST) begin
    if (rst)
      mul_s1_rdy <= 1'b0;
    else if (pipeline_flush_i)
      mul_s1_rdy <= 1'b0;
    else if (mul_adv)
      mul_s1_rdy <= op_mul_i;
  end // posedge clock

  // stage #2: partial products
  reg [EXEDW-1:0] mul_s2_albl;
  reg [EXEDW-1:0] mul_s2_ahbl;
  reg [EXEDW-1:0] mul_s2_bhal;
  //  registering
  always @(posedge clk) begin
    if (mul_s1_rdy & mul_adv) begin
      mul_s2_albl <= mul_s1_al * mul_s1_bl;
      mul_s2_ahbl <= mul_s1_ah * mul_s1_bl;
      mul_s2_bhal <= mul_s1_bh * mul_s1_al;
    end
  end // posedge clock
  //  ready flag
  reg mul_s2_rdy;
  always @(posedge clk `OR_ASYNC_RST) begin
    if (rst)
      mul_s2_rdy <= 1'b0;
    else if (pipeline_flush_i)
      mul_s2_rdy <= 1'b0;
    else if (mul_adv)
      mul_s2_rdy <= mul_s1_rdy;
  end // posedge clock
  // valid flag is 1-clock ahead of latching for WB
  assign mul_valid = mul_s2_rdy;

  // stage #3: result
  wire [EXEDW-1:0] mul_s3t_sum;
  assign mul_s3t_sum = {mul_s2_bhal[MULHDW-1:0],{MULHDW{1'b0}}} +
                       {mul_s2_ahbl[MULHDW-1:0],{MULHDW{1'b0}}} +
                        mul_s2_albl;
  //  registering
  always @(posedge clk `OR_ASYNC_RST) begin
    if (rst)
      wb_mul_result_o <= {EXEDW{1'b0}};
    else if (mul_valid & padv_wb_i)
      wb_mul_result_o <= mul_s3t_sum;
  end // posedge clock
  // multiplier ready flag
  reg mul_rdy_stored;
  // ---
  always @(posedge clk `OR_ASYNC_RST) begin
    if (rst) begin
      wb_mul_rdy_o   <= 1'b0;
      mul_rdy_stored <= 1'b0;
    end
    else if (pipeline_flush_i) begin
      wb_mul_rdy_o   <= wb_mul_rdy_o;
      mul_rdy_stored <= 1'b0;
    end
    else if (padv_wb_i) begin
      wb_mul_rdy_o   <= (mul_valid | mul_rdy_stored);
      mul_rdy_stored <= 1'b0;
    end
    else if (~mul_rdy_stored) begin
      wb_mul_rdy_o   <= wb_mul_rdy_o;
      mul_rdy_stored <= mul_valid;
    end
  end // @clock



  //----------------//
  // 32-bit divider //
  //----------------//
  reg       [5:0] div_count;
  reg [EXEDW-1:0] div_n;
  reg [EXEDW-1:0] div_d;
  reg [EXEDW-1:0] div_r;
  wire  [EXEDW:0] div_sub;
  reg             div_signed, div_unsigned;
  reg             div_neg;
  reg             div_valid;
  reg             div_by_zero;

  assign div_sub = {div_r[EXEDW-2:0],div_n[EXEDW-1]} - div_d;

  // Cycle counter
  always @(posedge clk `OR_ASYNC_RST) begin
    if (rst) begin
      div_valid <= 1'b0;
      div_count <= 6'd0;
    end
    if (padv_decode_i | pipeline_flush_i) begin // reset @ new decode data
      div_valid <= 1'b0;
      div_count <= 6'd0;
    end
    else if (op_div_i) begin
      div_valid <= 1'b0;
      div_count <= EXEDW;
    end
    else if (|div_count) begin
      if (div_count == 6'd1)
        div_valid <= 1'b1;
      else if (~div_valid)
        div_count <= div_count - 6'd1;
    end
  end // @clock

  always @(posedge clk) begin
    if (op_div_i) begin
      div_n        <= rfa_i;
      div_d        <= rfb_i;
      div_r        <= 0;
      div_neg      <= 1'b0;
      div_by_zero  <= ~(|rfb_i);
      div_signed   <= op_div_signed_i;
      div_unsigned <= op_div_unsigned_i;
      /*
       * Convert negative operands in the case of signed division.
       * If only one of the operands is negative, the result is
       * converted back to negative later on
       */
      if (op_div_signed_i) begin
        if (rfa_i[EXEDW-1] ^ rfb_i[EXEDW-1])
          div_neg <= 1'b1;

        if (rfa_i[EXEDW-1])
          div_n <= ~rfa_i + 1;

        if (rfb_i[EXEDW-1])
          div_d <= ~rfb_i + 1;
      end
    end
    else if (~div_valid) begin
      if (~div_sub[EXEDW]) begin // div_sub >= 0
        div_r <= div_sub[EXEDW-1:0];
        div_n <= {div_n[EXEDW-2:0], 1'b1};
      end
      else begin                 // div_sub < 0
        div_r <= {div_r[EXEDW-2:0],div_n[EXEDW-1]};
        div_n <= {div_n[EXEDW-2:0], 1'b0};
      end
    end // ~done
  end // @clock

  wire [EXEDW-1:0] div_result = div_neg ? ~div_n + 1 : div_n;
  
  //  registering
  always @(posedge clk `OR_ASYNC_RST) begin
    if (rst)
      wb_div_result_o <= {EXEDW{1'b0}};
    else if (div_valid & padv_wb_i)
      wb_div_result_o <= div_result;
  end // posedge clock
  // divider ready flag
  reg div_rdy_stored;
  // ---
  always @(posedge clk `OR_ASYNC_RST) begin
    if (rst) begin
      wb_div_rdy_o   <= 1'b0;
      div_rdy_stored <= 1'b0;
    end
    else if (pipeline_flush_i) begin
      wb_div_rdy_o   <= wb_div_rdy_o;
      div_rdy_stored <= 1'b0;
    end
    else if (padv_wb_i) begin
      wb_div_rdy_o   <= (div_valid | div_rdy_stored);
      div_rdy_stored <= 1'b0;
    end
    else if (~div_rdy_stored) begin
      wb_div_rdy_o   <= wb_div_rdy_o;
      div_rdy_stored <= div_valid;
    end
  end // @clock



  //-------------//
  // FPU related //
  //-------------//
  //  arithmetic part interface
  wire fpu_op_is_arith;
  wire fpu_arith_valid;
  wire [EXEDW-1:0] fpu_result;
  //  comparator part interface
  wire fpu_op_is_cmp;
  wire fpu_cmp_valid;
  wire fpu_cmp_flag;
  //  instance
  generate
    /* verilator lint_off WIDTH */
    if (FEATURE_FPU!="NONE") begin :  fpu_alu_ena
    /* verilator lint_on WIDTH */
      wire [(`OR1K_FPCSR_WIDTH-1):0] fpcsr_w;
      // fpu32 instance
      pfpu32_top u_pfpu32
      (
        .clk(clk),
        .rst(rst),
        .flush_i(pipeline_flush_i),
        .padv_decode_i(padv_decode_i),
        .padv_execute_i(padv_wb_i),
        .op_fpu_i(op_fpu_i),
        .round_mode_i(fpu_round_mode_i),
        .rfa_i(rfa_i),
        .rfb_i(rfb_i),
        .fpu_result_o(fpu_result),
        .fpu_arith_valid_o(fpu_arith_valid),
        .fpu_cmp_flag_o(fpu_cmp_flag),
        .fpu_cmp_valid_o(fpu_cmp_valid),
        .fpcsr_o(fpcsr_w)
      );
      // some glue logic
      assign fpu_op_is_arith = op_fpu_i[`OR1K_FPUOP_WIDTH-1] & (~op_fpu_i[3]);
      assign fpu_op_is_cmp   = op_fpu_i[`OR1K_FPUOP_WIDTH-1] &   op_fpu_i[3];
      // flag to update FPCSR
      assign exec_fpcsr_o     = fpcsr_w;
      assign exec_fpcsr_set_o = (fpu_arith_valid | fpu_cmp_valid);
    end
    else begin :  fpu_alu_none
      // arithmetic part
      assign fpu_op_is_arith = 1'b0;
      assign fpu_arith_valid = 1'b0;
      assign fpu_result      = {EXEDW{1'b0}};
      // comparator part
      assign fpu_op_is_cmp = 1'b0;
      assign fpu_cmp_valid = 1'b0;
      assign fpu_cmp_flag  = 1'b0;
      // fpu's common
      assign exec_fpcsr_o     = {`OR1K_FPCSR_WIDTH{1'b0}};
      assign exec_fpcsr_set_o = 1'b0;
    end // fpu_ena/fpu_none
  endgenerate // FPU related



  //----------------//
  // Results muxing //
  //----------------//
  assign alu_nl_result_o = fpu_result;
  assign exec_lsu_adr_o  = adder_result; // lsu address (not latched)

  // Update SR[F] either from integer or float point comparision
  assign exec_flag_set_o   = fpu_op_is_cmp ? (fpu_cmp_flag & fpu_cmp_valid) :
                                             (flag_set & op_setflag_i);
  assign exec_flag_clear_o = fpu_op_is_cmp ? ((~fpu_cmp_flag) & fpu_cmp_valid) :
                                             ((~flag_set) & op_setflag_i);

  // Overflow flag generation
  assign exec_overflow_set_o   = (FEATURE_OVERFLOW != "NONE") &
                                 ((op_add_i & adder_s_ovf) |
                                  (div_valid & div_signed & div_by_zero));
  assign exec_overflow_clear_o = (FEATURE_OVERFLOW != "NONE") &
                                 ((op_add_i & (~adder_s_ovf)) |
                                  (div_valid & div_signed & (~div_by_zero)));

  // Carry flag generation
  assign exec_carry_set_o   = (FEATURE_CARRY_FLAG != "NONE") &
                              ((op_add_i & adder_u_ovf) |
                               (div_valid & div_unsigned & div_by_zero));
  assign exec_carry_clear_o = (FEATURE_CARRY_FLAG!="NONE") &
                              ((op_add_i & (~adder_u_ovf)) |
                               (div_valid & div_unsigned & (~div_by_zero)));



  //-------------//
  // Stall logic //
  //-------------//

  // ALU ready flag
  assign exec_valid_o =
    exec_insn_1clk_i | div_valid | mul_valid |
    (fpu_op_is_arith & fpu_arith_valid) |
    (fpu_op_is_cmp & fpu_cmp_valid) |
    lsu_valid_i | lsu_excepts_i | msync_done_i;

endmodule // mor1kx_execute_marocchino
