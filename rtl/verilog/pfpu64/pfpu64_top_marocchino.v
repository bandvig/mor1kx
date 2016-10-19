/////////////////////////////////////////////////////////////////////
//                                                                 //
//    pfpu64_top_marocchino                                        //
//    64-bit floating point top level for MAROCCHINO pipeline      //
//                                                                 //
//    Author: Andrey Bacherov                                      //
//            avbacherov@opencores.org                             //
//                                                                 //
/////////////////////////////////////////////////////////////////////
//                                                                 //
//   Copyright (C) 2015 Andrey Bacherov                            //
//                      avbacherov@opencores.org                   //
//                                                                 //
//   This source file may be used and distributed without          //
//   restriction provided that this copyright statement is not     //
//   removed from the file and that any derivative work contains   //
//   the original copyright notice and the associated disclaimer.  //
//                                                                 //
//       THIS SOFTWARE IS PROVIDED ``AS IS'' AND WITHOUT ANY       //
//   EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED     //
//   TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS     //
//   FOR A PARTICULAR PURPOSE. IN NO EVENT SHALL THE AUTHOR        //
//   OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,           //
//   INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES      //
//   (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE     //
//   GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR          //
//   BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF    //
//   LIABILITY, WHETHER IN  CONTRACT, STRICT LIABILITY, OR TORT    //
//   (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT    //
//   OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE           //
//   POSSIBILITY OF SUCH DAMAGE.                                   //
//                                                                 //
/////////////////////////////////////////////////////////////////////

`include "mor1kx-defines.v"

//---------------------------------------------------------------//
// Order Control Buffer for FPU64                                //
//   simplified version of mor1kx_ocb_marocchino                 //
//   it also contents only 4 order taps and only last tap's      //
//   output is required here                                     //
//---------------------------------------------------------------//

module pfpu64_ocb_marocchino
(
  // clocks and resets
  input   clk,
  input   rst,
  // pipe controls
  input   pipeline_flush_i,
  input   taking_op_fp64_arith_i,  // write: an FPU pipe is taking operands
  input   rnd_taking_op_i,       // read:  rounding engine is taking result
  // data input
  input   add_start_i,
  input   mul_start_i,
  input   div_start_i,
  input   i2f_start_i,
  input   f2i_start_i,
  // data ouputs
  output  grant_rnd_to_add_o,
  output  grant_rnd_to_mul_o,
  output  grant_rnd_to_div_o,
  output  grant_rnd_to_i2f_o,
  output  grant_rnd_to_f2i_o,
  // "OCB is full" flag
  //   (a) external control logic must stop the "writing without reading"
  //       operation if OCB is full
  //   (b) however, the "writing + reading" is possible
  //       because it just pushes OCB and keeps it full
  output  pfpu64_ocb_full_o
);

  localparam NUM_TAPS = 4;

  // "pointers"
  reg   [NUM_TAPS:0] ptr_curr; // on current active tap
  reg [NUM_TAPS-1:0] ptr_prev; // on previous active tap

  // pointers are zero: tap #0 (output) is active
  wire ptr_curr_0 = ptr_curr[0];
  wire ptr_prev_0 = ptr_prev[0];

  // "OCB is full" flag
  //  # no more availaible taps, pointer is out of range
  assign pfpu64_ocb_full_o = ptr_curr[NUM_TAPS];

  // control to increment/decrement pointers
  wire rd_only = ~taking_op_fp64_arith_i &  rnd_taking_op_i;
  wire wr_only =  taking_op_fp64_arith_i & ~rnd_taking_op_i;
  wire wr_rd   =  taking_op_fp64_arith_i &  rnd_taking_op_i;


  // operation algorithm:
  //-----------------------------------------------------------------------------
  // read only    | push: tap[k-1] <= tap[k], tap[num_taps-1] <= reset_value;
  //              | update pointers: if(~ptr_prev_0) ptr_prev <= (ptr_prev >> 1);
  //              |                  if(~ptr_curr_0) ptr_curr <= (ptr_curr >> 1);
  //-----------------------------------------------------------------------------
  // write only   | tap[ptr_curr] <= ocbi_i
  //              | ptr_prev <= ptr_curr;
  //              | ptr_curr <= (ptr_curr << 1);
  //-----------------------------------------------------------------------------
  // read & write | push: tap[k-1] <= tap[k]
  //              |       tap[ptr_prev] <= ocbi_i;
  //-----------------------------------------------------------------------------


  wire ptr_curr_inc = wr_only; // increment pointer on current tap
  wire ptr_curr_dec = rd_only & ~ptr_curr_0; // decrement ...
  wire ptr_prev_dec = rd_only & ~ptr_prev_0; // decrement ... previous ...

  // update pointer on current tap
  always @(posedge clk `OR_ASYNC_RST) begin
    if (rst)
      ptr_curr <= {{NUM_TAPS{1'b0}},1'b1};
    else if(pipeline_flush_i)
      ptr_curr <= {{NUM_TAPS{1'b0}},1'b1};
    else if(ptr_curr_inc)
      ptr_curr <= (ptr_curr << 1);
    else if(ptr_curr_dec)
      ptr_curr <= (ptr_curr >> 1);
  end // posedge clock

  // update pointer on previous tap
  always @(posedge clk `OR_ASYNC_RST) begin
    if (rst)
      ptr_prev <= {{(NUM_TAPS-1){1'b0}},1'b1};
    else if(pipeline_flush_i)
      ptr_prev <= {{(NUM_TAPS-1){1'b0}},1'b1};
    else if(ptr_curr_inc)
      ptr_prev <= ptr_curr[NUM_TAPS-1:0];
    else if(ptr_prev_dec)
      ptr_prev <= (ptr_prev >> 1);
  end // posedge clock


  // enable signals for taps
  wire [NUM_TAPS-1:0] en_curr_tap = {NUM_TAPS{wr_only}} & ptr_curr[NUM_TAPS-1:0];
  wire [NUM_TAPS-1:0] push_taps =
    en_curr_tap |                // tap[ptr_curr] <= ocbi_i (note: by wr_only)
    {NUM_TAPS{rnd_taking_op_i}}; // tap[k-1] <= tap[k]

  // control for forwarding multiplexors
  wire [NUM_TAPS-1:0] use_forwarded_value =
    en_curr_tap |                   // tap[ptr_curr] <= ocbi_i (note: by wr_only)
    ({NUM_TAPS{wr_rd}} & ptr_prev); // tap[ptr_prev] <= ocbi_i;


  // order input
  wire [4:0] ocbi = {add_start_i, mul_start_i, div_start_i, i2f_start_i, f2i_start_i};

  // taps ouputs
  wire [4:0] ocbo00; // OCB output
  wire [4:0] ocbo01; // ...
  wire [4:0] ocbo02; // ...
  wire [4:0] ocbo03; // OCB entrance

  // granting flags output
  assign {grant_rnd_to_add_o, grant_rnd_to_mul_o, grant_rnd_to_div_o, grant_rnd_to_i2f_o, grant_rnd_to_f2i_o} = ocbo00;

  // taps
  //   tap #00
  ocb_tap
  #(
    .DATA_SIZE (4)
  )
  u_tap_00
  (
    .clk                    (clk),
    .rst                    (rst),
    .flush_i                (pipeline_flush_i),
    .push_i                 (push_taps[0]),
    .prev_tap_out_i         (ocbo01),
    .forwarded_value_i      (ocbi),
    .use_forwarded_value_i  (use_forwarded_value[0]),
    .out_o                  (ocbo00)
  );

  //   tap #01
  ocb_tap
  #(
    .DATA_SIZE (4)
  )
  u_tap_01
  (
    .clk                    (clk),
    .rst                    (rst),
    .flush_i                (pipeline_flush_i),
    .push_i                 (push_taps[1]),
    .prev_tap_out_i         (ocbo02),
    .forwarded_value_i      (ocbi),
    .use_forwarded_value_i  (use_forwarded_value[1]),
    .out_o                  (ocbo01)
  );

  //   tap #02
  ocb_tap
  #(
    .DATA_SIZE (4)
  )
  u_tap_02
  (
    .clk                    (clk),
    .rst                    (rst),
    .flush_i                (pipeline_flush_i),
    .push_i                 (push_taps[2]),
    .prev_tap_out_i         (ocbo03),
    .forwarded_value_i      (ocbi),
    .use_forwarded_value_i  (use_forwarded_value[2]),
    .out_o                  (ocbo02)
  );

  //   tap #03 (entrance)
  ocb_tap
  #(
    .DATA_SIZE (4)
  )
  u_tap_03
  (
    .clk                    (clk),
    .rst                    (rst),
    .flush_i                (pipeline_flush_i),
    .push_i                 (push_taps[3]),
    .prev_tap_out_i         (4'd0),
    .forwarded_value_i      (ocbi),
    .use_forwarded_value_i  (use_forwarded_value[3]),
    .out_o                  (ocbo03)
  );
endmodule // pfpu64_ocb_marocchino


// fpu operations:
// ===================
// 0000 = add
// 0001 = substract
// 0010 = multiply
// 0011 = divide
// 0100 = i2f
// 0101 = f2i
// 0110 = unused (rem)
// 0111 = reserved
// 1xxx = comparison


module pfpu64_top_marocchino
#(
  parameter DEST_REG_ADDR_WIDTH  =  8 // OPTION_RF_ADDR_WIDTH + log2(Re-Ordering buffer width)
)
(
  // clock & reset
  input                               clk,
  input                               rst,

  // pipeline control
  input                               pipeline_flush_i,
  input                               padv_decode_i,
  input                               padv_wb_i,
  input                               grant_wb_to_fp64_arith_i,
  input                               grant_wb_to_fp64_cmp_i,

  // pipeline control outputs
  output                              fp64_arith_busy_o,     // idicates that arihmetic units are busy
  output                              fp64_arith_valid_o,    // WB-latching ahead arithmetic ready flag

  // Configuration
  input     [`OR1K_FPCSR_RM_SIZE-1:0] round_mode_i,
  input                               except_fpu_enable_i,
  input   [`OR1K_FPCSR_ALLF_SIZE-1:0] ctrl_fpu_mask_flags_i,

  // Commands for arithmetic part
  input                               dcod_op_fp64_arith_i,
  input                               dcod_op_fp64_add_i,
  input                               dcod_op_fp64_sub_i,
  input                               dcod_op_fp64_mul_i,
  input                               dcod_op_fp64_div_i,
  input                               dcod_op_fp64_i2f_i,
  input                               dcod_op_fp64_f2i_i,

  // Commands for comparison part
  input                               dcod_op_fp64_cmp_i,
  input                         [2:0] dcod_opc_fp64_cmp_i,

  // Operands from DECODE
  input                        [31:0] dcod_rfa_i,
  input                        [31:0] dcod_rfa2_i, ?!
  input                        [31:0] dcod_rfb_i,
  input                        [31:0] dcod_rfb2_i, ?!

  // OMAN-to-DECODE hazards
  //  combined flag
  input                               omn2dec_hazards_i,
  //  by operands
  input                               busy_hazard_a_i,
  input     [DEST_REG_ADDR_WIDTH-1:0] busy_hazard_a_adr_i,
  input                               busy_hazard_b_i,
  input     [DEST_REG_ADDR_WIDTH-1:0] busy_hazard_b_adr_i,

  // EXEC-to-DECODE hazards
  //  combined flag
  input                               exe2dec_hazards_i,
  //  by operands
  input                               exe2dec_hazard_a_i,
  input                               exe2dec_hazard_b_i,

  // Data for hazards resolving
  //  hazard could be passed from DECODE to EXECUTE
  input                               exec_rf_wb_i,
  input     [DEST_REG_ADDR_WIDTH-1:0] exec_rfd_adr_i,
  //  hazard could be resolving
  input                               wb_rf_wb_i,
  input     [DEST_REG_ADDR_WIDTH-1:0] wb_rfd_adr_i,
  input                        [31:0] wb_result_i,
  input                        [31:0] wb_result2_i, ?!

  // FPU-64 arithmetic part
  output                       [31:0] wb_fp64_arith_res_hi_o,   // arithmetic result
  output                       [31:0] wb_fp64_arith_res_lo_o,   // arithmetic result 2
  output  [`OR1K_FPCSR_ALLF_SIZE-1:0] wb_fp64_arith_fpcsr_o,    // arithmetic exceptions
  output                              wb_fp64_arith_wb_fpcsr_o, // update FPCSR
  output                              wb_except_fp64_arith_o,   // generate exception

  // FPU-64 comparison part
  output                              wb_fp64_flag_set_o,      // comparison result
  output                              wb_fp64_flag_clear_o,    // comparison result
  output                              wb_fp64_cmp_inv_o,       // comparison flag 'invalid'
  output                              wb_fp64_cmp_inf_o,       // comparison flag 'infinity'
  output                              wb_fp64_cmp_wb_fpcsr_o,  // update FPCSR
  output                              wb_except_fp64_cmp_o     // exception by FP64-comparison
);

// fp32 arithmetic command
wire exec_op_fp64_add, exec_op_fp64_sub, exec_op_fp64_mul,
     exec_op_fp64_div, exec_op_fp64_i2f, exec_op_fp64_f2i;

// fp32 pipes controls
wire taking_op_fp64_arith;
wire taking_op_fp64_cmp = padv_wb_i & grant_wb_to_fp64_cmp_i;

// reservation station instance
mor1kx_rsrvs_marocchino ?!
#(
  .OPTION_OPERAND_WIDTH     (64), // FP64_ARITH_RSVRS
  .USE_OPC                  (1), // FP64_ARITH_RSVRS
  .OPC_WIDTH                (6 ?!), // FP64_ARITH_RSVRS
  .DEST_REG_ADDR_WIDTH      (DEST_REG_ADDR_WIDTH), // FP64_ARITH_RSVRS
  .USE_RSVRS_FLAG_CARRY     (0), // FP64_ARITH_RSVRS
  .DEST_FLAG_ADDR_WIDTH     (1) // FP64_ARITH_RSVRS
)
u_fp64_arith_rsrvs
(
  // clocks and resets
  .clk                      (clk), // FP64_ARITH_RSVRS
  .rst                      (rst), // FP64_ARITH_RSVRS
  // pipeline control signals in
  .pipeline_flush_i         (pipeline_flush_i), // FP64_ARITH_RSVRS
  .padv_decode_i            (padv_decode_i), // FP64_ARITH_RSVRS
  .taking_op_i              (taking_op_fp64_arith | taking_op_fp64_cmp), // FP64_ARITH_RSVRS
  // input data from DECODE
  .dcod_rfa_i               ({dcod_rfa_i,dcod_rfa2_i}), // FP64_ARITH_RSVRS
  .dcod_rfb_i               ({dcod_rfb_i,dcod_rfb2_i}), // FP64_ARITH_RSVRS
  // OMAN-to-DECODE hazards
  //  combined flag
  .omn2dec_hazards_i        (omn2dec_hazards_i), // FP64_ARITH_RSVRS
  //  by FLAG and CARRY
  .busy_hazard_f_i          (1'b0), // FP64_ARITH_RSVRS
  .busy_hazard_f_adr_i      (1'b0), // FP64_ARITH_RSVRS
  .busy_hazard_c_i          (1'b0), // FP64_ARITH_RSVRS
  .busy_hazard_c_adr_i      (1'b0), // FP64_ARITH_RSVRS
  //  by operands
  .busy_hazard_a_i          (busy_hazard_a_i), // FP64_ARITH_RSVRS
  .busy_hazard_a_adr_i      (busy_hazard_a_adr_i), // FP64_ARITH_RSVRS
  .busy_hazard_b_i          (busy_hazard_b_i), // FP64_ARITH_RSVRS
  .busy_hazard_b_adr_i      (busy_hazard_b_adr_i), // FP64_ARITH_RSVRS
  // EXEC-to-DECODE hazards
  //  combined flag
  .exe2dec_hazards_i        (exe2dec_hazards_i), // FP64_ARITH_RSVRS
  //  by operands
  .exe2dec_hazard_a_i       (exe2dec_hazard_a_i), // FP64_ARITH_RSVRS
  .exe2dec_hazard_b_i       (exe2dec_hazard_b_i), // FP64_ARITH_RSVRS
  // Data for hazards resolving
  //  hazard could be passed from DECODE to EXECUTE
  .exec_flag_wb_i           (1'b0), // FP64_ARITH_RSVRS
  .exec_carry_wb_i          (1'b0), // FP64_ARITH_RSVRS
  .exec_flag_carry_adr_i    (1'b0), // FP64_ARITH_RSVRS
  .exec_rf_wb_i             (exec_rf_wb_i), // FP64_ARITH_RSVRS
  .exec_rfd_adr_i           (exec_rfd_adr_i), // FP64_ARITH_RSVRS
  .padv_wb_i                (padv_wb_i), // FP64_ARITH_RSVRS
  //  hazard could be resolving
  .wb_flag_wb_i             (1'b0), // FP64_ARITH_RSVRS
  .wb_carry_wb_i            (1'b0), // FP64_ARITH_RSVRS
  .wb_flag_carry_adr_i      (1'b0), // FP64_ARITH_RSVRS
  .wb_rf_wb_i               (wb_rf_wb_i), // FP64_ARITH_RSVRS
  .wb_rfd_adr_i             (wb_rfd_adr_i), // FP64_ARITH_RSVRS
  .wb_result_i              (wb_result_i), // FP64_ARITH_RSVRS
  // command and its additional attributes
  .dcod_op_i                (dcod_op_fp64_arith_i), // FP64_ARITH_RSVRS
  .dcod_opc_i               ({dcod_op_fp64_add_i, dcod_op_fp64_sub_i, dcod_op_fp64_mul_i, // FP64_ARITH_RSVRS
                              dcod_op_fp64_div_i, dcod_op_fp64_i2f_i, dcod_op_fp64_f2i_i}), // FP64_ARITH_RSVRS
  // outputs
  //   command attributes from busy stage
  .busy_opc_o               (), // FP64_ARITH_RSVRS
  //   command and its additional attributes
  .exec_op_o                (), // FP64_ARITH_RSVRS
  .exec_opc_o               ({exec_op_fp64_add, exec_op_fp64_sub, exec_op_fp64_mul, // FP64_ARITH_RSVRS
                              exec_op_fp64_div, exec_op_fp64_i2f, exec_op_fp64_f2i}), // FP64_ARITH_RSVRS
  //   operands
  .exec_rfa_o               (fp64_arith_a), // FP64_ARITH_RSVRS
  .exec_rfb_o               (fp64_arith_b), // FP64_ARITH_RSVRS
  //   unit-is-busy flag
  .unit_busy_o              (fp64_arith_busy_o) // FP64_ARITH_RSVRS
);

// operand A and B  with forwarding from WB
wire [63:0] fp64_arith_a = ?!;
wire [63:0] fp64_arith_b = ?!;

// analysis of input values
//   split input a
wire        in_signa  = fp64_arith_a[63];
wire [10:0] in_expa   = fp64_arith_a[62:52];
wire [51:0] in_fracta = fp64_arith_a[51:0];
//   detect infinity a
wire in_expa_ff = &in_expa;
wire in_infa    = in_expa_ff & (~(|in_fracta));
//   signaling NaN: exponent is 8hff, [51] is zero,
//                  rest of fract is non-zero
//   quiet NaN: exponent is 8hff, [51] is 1
wire in_snan_a = in_expa_ff & (~in_fracta[51]) & (|in_fracta[50:0]);
wire in_qnan_a = in_expa_ff &   in_fracta[51];
//   denormalized/zero of a
wire in_opa_0  = ~(|fp64_arith_a[62:0]);
wire in_opa_dn = (~(|in_expa)) & (|in_fracta);

//   split input b
wire        in_signb  = fp64_arith_b[63];
wire [10:0] in_expb   = fp64_arith_b[62:52];
wire [51:0] in_fractb = fp64_arith_b[51:0];
//   detect infinity b
wire in_expb_ff = &in_expb;
wire in_infb    = in_expb_ff & (~(|in_fractb));
//   detect NaNs in b
wire in_snan_b = in_expb_ff & (~in_fractb[51]) & (|in_fractb[50:0]);
wire in_qnan_b = in_expb_ff &   in_fractb[51];
//   denormalized/zero of a
wire in_opb_0  = ~(|fp64_arith_b[62:0]);
wire in_opb_dn = (~(|in_expb)) & (|in_fractb);

// detection of some exceptions
//   a nan input -> qnan output
wire in_snan = in_snan_a | in_snan_b;
wire in_qnan = in_qnan_a | in_qnan_b;
//   sign of output nan
wire in_anan_sign = (in_snan_a | in_qnan_a) ? in_signa :
                                              in_signb;

// restored exponents
wire [12:0] in_exp13a = {2'd0,in_expa[10:1],(in_expa[0] | in_opa_dn)};
wire [12:0] in_exp13b = {2'd0,in_expb[10:1],(in_expb[0] | in_opb_dn)};
// restored fractionals
wire [52:0] in_fract53a = {((~in_opa_dn) & (~in_opa_0)),in_fracta};
wire [52:0] in_fract53b = {((~in_opb_dn) & (~in_opb_0)),in_fractb};


// Support for ADD/SUB (historically they were comparator's part)
//  # exponents
wire exp_gt = in_exp13a  > in_exp13b;
wire exp_eq = in_exp13a == in_exp13b;
//  # fractionals
wire fract_gt = in_fract53a  > in_fract53b;
wire fract_eq = in_fract53a == in_fract53b;
//  # comparisons for ADD/SUB
wire addsub_agtb = exp_gt | (exp_eq & fract_gt);
wire addsub_aeqb = exp_eq & fract_eq;


// order control buffer is full:
// we are waiting an arithmetic pipe result for rounding
wire pfpu64_ocb_full;

// unit-wise control signals
//  ## ADD / SUB
wire op_add             = exec_op_fp64_add & (~pfpu64_ocb_full);
wire op_sub             = exec_op_fp64_sub & (~pfpu64_ocb_full);
wire add_start          = op_add | op_sub;
wire add_taking_op;
wire add_rdy;
wire grant_rnd_to_add;
wire rnd_muxing_add     = add_rdy & grant_rnd_to_add; // to rounding input muxer
wire rnd_taking_add;

//  ## MUL
wire op_mul             = exec_op_fp64_mul & (~pfpu64_ocb_full);
wire mul_rdy;
wire grant_rnd_to_mul;
wire rnd_muxing_mul     = mul_rdy & grant_rnd_to_mul; // to rounding input muxer
wire rnd_taking_mul;

//  ## DIV
wire op_div             = exec_op_fp64_div & (~pfpu64_ocb_full);
wire div_rdy;
wire grant_rnd_to_div;
wire rnd_muxing_div     = div_rdy & grant_rnd_to_div; // to rounding input muxer
wire rnd_taking_div;

//  ## MUL/DIV
wire muldiv_taking_op;

//  ## i2f
wire i2f_start          = exec_op_fp64_i2f & (~pfpu64_ocb_full);
wire i2f_taking_op;
wire i2f_rdy;
wire grant_rnd_to_i2f;
wire rnd_muxing_i2f     = i2f_rdy & grant_rnd_to_i2f; // to rounding input muxer
wire rnd_taking_i2f;

//  ## f2i
wire f2i_start          = exec_op_fp64_f2i & (~pfpu64_ocb_full);
wire f2i_taking_op;
wire f2i_rdy;
wire grant_rnd_to_f2i;
wire rnd_muxing_f2i     = f2i_rdy & grant_rnd_to_f2i; // to rounding input muxer
wire rnd_taking_f2i;

// feedback to drop FP32 arithmetic related command
assign taking_op_fp64_arith = add_taking_op | muldiv_taking_op | i2f_taking_op | f2i_taking_op;

// rounding engine takes an OP
wire rnd_taking_op = rnd_taking_add | rnd_taking_mul | rnd_taking_div |
                     rnd_taking_i2f | rnd_taking_f2i;


// ---
pfpu64_ocb_marocchino  u_pfpu64_ocb
(
  // clocks and resets
  .clk                    (clk), // PFPU64_OCB
  .rst                    (rst), // PFPU64_OCB
  // pipe controls
  .pipeline_flush_i       (pipeline_flush_i), // PFPU64_OCB
  .taking_op_fp64_arith_i (taking_op_fp64_arith), // PFPU64_OCB
  .rnd_taking_op_i        (rnd_taking_op), // PFPU64_OCB
  // data input
  .add_start_i            (add_start), // PFPU64_OCB
  .mul_start_i            (op_mul), // PFPU64_OCB
  .div_start_i            (op_div), // PFPU64_OCB
  .i2f_start_i            (i2f_start), // PFPU64_OCB
  .f2i_start_i            (f2i_start), // PFPU64_OCB
  // data ouputs
  .grant_rnd_to_add_o     (grant_rnd_to_add), // PFPU64_OCB
  .grant_rnd_to_mul_o     (grant_rnd_to_mul ?!), // PFPU64_OCB
  .grant_rnd_to_div_o     (grant_rnd_to_div ?!), // PFPU64_OCB
  .grant_rnd_to_i2f_o     (grant_rnd_to_i2f), // PFPU64_OCB
  .grant_rnd_to_f2i_o     (grant_rnd_to_f2i), // PFPU64_OCB
  // "OCB is full" flag
  .pfpu64_ocb_full_o      (pfpu64_ocb_full) // PFPU64_OCB
);


// Addition / Substruction
//   connection wires
wire        add_sign;      // add/sub signum
wire        add_sub_0;     // flag that actual substruction is performed and result is zero
wire  [5:0] add_shl;       // do left shift in align stage
wire [12:0] add_exp13shl;  // exponent for left shift align
wire [12:0] add_exp13sh0;  // exponent for no shift in align
wire [56:0] add_fract57;   // fractional with appended {r,s} bits
wire        add_inv;       // add/sub invalid operation flag
wire        add_inf;       // add/sub infinity output reg
wire        add_snan;      // add/sub signaling NaN output reg
wire        add_qnan;      // add/sub quiet NaN output reg
wire        add_anan_sign; // add/sub signum for output nan
//   module istance
pfpu64_addsub_marocchino u_fp64_addsub
(
  // clocks and resets
  .clk              (clk), // FP64_ADDSUB
  .rst              (rst), // FP64_ADDSUB
  // ADD/SUB pipe controls
  .pipeline_flush_i (pipeline_flush_i), // FP64_ADDSUB
  .start_i          (add_start), // FP64_ADDSUB
  .is_sub_i         (op_sub), // FP64_ADDSUB
  .add_taking_op_o  (add_taking_op), // FP64_ADDSUB
  .add_rdy_o        (add_rdy), // FP64_ADDSUB
  .rnd_taking_add_i (rnd_taking_add), // FP64_ADDSUB
  // input 'a' related values
  .signa_i          (in_signa), // FP64_ADDSUB
  .exp13a_i         (in_exp13a), // FP64_ADDSUB
  .fract53a_i       (in_fract53a), // FP64_ADDSUB
  .infa_i           (in_infa), // FP64_ADDSUB
  // input 'b' related values
  .signb_i          (in_signb), // FP64_ADDSUB
  .exp13b_i         (in_exp13b), // FP64_ADDSUB
  .fract53b_i       (in_fract53b), // FP64_ADDSUB
  .infb_i           (in_infb), // FP64_ADDSUB
  // 'a'/'b' related
  .snan_i           (in_snan), // FP64_ADDSUB
  .qnan_i           (in_qnan), // FP64_ADDSUB
  .anan_sign_i      (in_anan_sign), // FP64_ADDSUB
  .addsub_agtb_i    (addsub_agtb), // FP64_ADDSUB
  .addsub_aeqb_i    (addsub_aeqb), // FP64_ADDSUB
  // outputs
  .add_sign_o       (add_sign), // FP64_ADDSUB
  .add_sub_0_o      (add_sub_0), // FP64_ADDSUB
  .add_shl_o        (add_shl), // FP64_ADDSUB
  .add_exp13shl_o   (add_exp13shl), // FP64_ADDSUB
  .add_exp13sh0_o   (add_exp13sh0), // FP64_ADDSUB
  .add_fract57_o    (add_fract57), // FP64_ADDSUB
  .add_inv_o        (add_inv), // FP64_ADDSUB
  .add_inf_o        (add_inf), // FP64_ADDSUB
  .add_snan_o       (add_snan), // FP64_ADDSUB
  .add_qnan_o       (add_qnan), // FP64_ADDSUB
  .add_anan_sign_o  (add_anan_sign) // FP64_ADDSUB
);

// MUL/DIV pipeline
//   MUL outputs
wire        mul_sign;      // mul signum
wire  [5:0] mul_shr;       // do right shift in align stage
wire [12:0] mul_exp13shr;  // exponent for right shift align
wire [12:0] mul_exp13sh0;  // exponent for no shift in align
wire [56:0] mul_fract57;   // fractional with appended {r,s} bits
wire        mul_inv;       // mul invalid operation flag
wire        mul_inf;       // mul infinity output reg
wire        mul_snan;      // mul signaling NaN output reg
wire        mul_qnan;      // mul quiet NaN output reg
wire        mul_anan_sign; // mul signum for output nan
// DIV outputs
wire        div_sign;      // signum
wire  [5:0] div_shr;       // do right shift in align stage
wire [12:0] div_exp13shr;  // exponent for right shift align
wire        div_shl;       // do left shift in align stage
wire [12:0] div_exp13shl;  // exponent for left shift align
wire [12:0] div_exp13sh0;  // exponent for no shift in align
wire [56:0] div_fract57;   // fractional with appended {r,s} bits
wire        div_dbz;        // div division by zero flag
wire        div_inv;       // invalid operation flag
wire        div_inf;       // infinity wire reg
wire        div_snan;      // signaling NaN wire reg
wire        div_qnan;      // quiet NaN wire reg
wire        div_anan_sign;  // signum for wire nan
//   DIV additional outputs
?! wire        div_op;        // operation is division
?! wire        div_sign_rmnd; // signum or reminder for IEEE compliant rounding
//   module istance
pfpu64_muldiv_marocchino u_fp64_muldiv
(
  // clocks and resets
  .clk                  (clk), // FP64_MULDIV
  .rst                  (rst), // FP64_MULDIV
  // pipe controls
  .pipeline_flush_i     (pipeline_flush_i), // FP64_MULDIV
  .is_mul_i             (op_mul), // FP64_MULDIV
  .is_div_i             (op_div), // FP64_MULDIV
  .muldiv_taking_op_o   (muldiv_taking_op), // FP64_MULDIV
  .mul_rdy_o            (mul_rdy), // FP64_MULDIV
  .rnd_taking_mul_i     (rnd_taking_mul), // FP64_MULDIV
  .div_rdy_o            (div_rdy), // FP64_MULDIV
  .rnd_taking_div_i     (rnd_taking_div), // FP64_MULDIV
  // input 'a' related values
  .signa_i              (in_signa), // FP64_MULDIV
  .exp13a_i             (in_exp13a), // FP64_MULDIV
  .fract53a_i           (in_fract53a), // FP64_MULDIV
  .infa_i               (in_infa), // FP64_MULDIV
  .zeroa_i              (in_opa_0), // FP64_MULDIV
  // input 'b' related values
  .signb_i              (in_signb), // FP64_MULDIV
  .exp13b_i             (in_exp13b), // FP64_MULDIV
  .fract53b_i           (in_fract53b), // FP64_MULDIV
  .infb_i               (in_infb), // FP64_MULDIV
  .zerob_i              (in_opb_0), // FP64_MULDIV
  // 'a'/'b' related
  .snan_i               (in_snan), // FP64_MULDIV
  .qnan_i               (in_qnan), // FP64_MULDIV
  .anan_sign_i          (in_anan_sign), // FP64_MULDIV
  // MUL outputs
  .mul_sign_o           (mul_sign), // FP64_MULDIV
  .mul_shr_o            (mul_shr), // FP64_MULDIV
  .mul_exp13shr_o       (mul_exp13shr), // FP64_MULDIV
  .mul_exp13sh0_o       (mul_exp13sh0), // FP64_MULDIV
  .mul_fract57_o        (mul_fract57), // FP64_MULDIV
  .mul_inv_o            (mul_inv), // FP64_MULDIV
  .mul_inf_o            (mul_inf), // FP64_MULDIV
  .mul_snan_o           (mul_snan), // FP64_MULDIV
  .mul_qnan_o           (mul_qnan), // FP64_MULDIV
  .mul_anan_sign_o      (mul_anan_sign), // FP64_MULDIV
  // DIV outputs
  .div_sign_o           (div_sign), // FP64_MULDIV
  .div_shr_o            (div_shr), // FP64_MULDIV
  .div_exp13shr_o       (div_exp13shr), // FP64_MULDIV
  .div_shl_o            (div_shl), // FP64_MULDIV
  .div_exp13shl_o       (div_exp13shl), // FP64_MULDIV
  .div_exp13sh0_o       (div_exp13sh0), // FP64_MULDIV
  .div_fract57_o        (div_fract57), // FP64_MULDIV
  .div_dbz_o            (div_dbz), // FP64_MULDIV
  .div_inv_o            (div_inv), // FP64_MULDIV
  .div_inf_o            (div_inf), // FP64_MULDIV
  .div_snan_o           (div_snan), // FP64_MULDIV
  .div_qnan_o           (div_qnan), // FP64_MULDIV
  .div_anan_sign_o      (div_anan_sign) // FP64_MULDIV
);

// convertors
//   i2f connection wires
wire        i2f_sign;
wire  [3:0] i2f_shr;
wire [10:0] i2f_exp11shr;
wire  [5:0] i2f_shl;
wire [10:0] i2f_exp11shl;
wire [10:0] i2f_exp11sh0;
wire [63:0] i2f_fract64;
//   i2f module instance
pfpu64_i2f_marocchino u_fp64_i2f_cnv
(
  // clocks and resets
  .clk                (clk), // FP64_I2F
  .rst                (rst), // FP64_I2F
  // I2F pipe controls
  .pipeline_flush_i   (pipeline_flush_i), // FP64_I2F
  .start_i            (i2f_start), // FP64_I2F
  .i2f_taking_op_o    (i2f_taking_op), // FP64_I2F
  .i2f_rdy_o          (i2f_rdy), // FP64_I2F
  .rnd_taking_i2f_i   (rnd_taking_i2f), // FP64_I2F
  // operand for conversion
  .opa_i              (fp64_arith_a), // FP64_I2F
  // ouputs for rounding
  .i2f_sign_o         (i2f_sign), // FP64_I2F
  .i2f_shr_o          (i2f_shr), // FP64_I2F
  .i2f_exp11shr_o     (i2f_exp11shr), // FP64_I2F
  .i2f_shl_o          (i2f_shl), // FP64_I2F
  .i2f_exp11shl_o     (i2f_exp11shl), // FP64_I2F
  .i2f_exp11sh0_o     (i2f_exp11sh0), // FP64_I2F
  .i2f_fract64_o      (i2f_fract64) // FP64_I2F
);
//   f2i connection wires
wire        f2i_sign;      // f2i signum
wire [52:0] f2i_int53;     // f2i fractional
wire  [5:0] f2i_shr;       // f2i required shift right value
wire  [3:0] f2i_shl;       // f2i required shift left value
wire        f2i_ovf;       // f2i overflow flag
wire        f2i_snan;      // f2i signaling NaN output reg
//    f2i module instance
pfpu64_f2i_marocchino u_fp64_f2i_cnv
(
  // clocks and resets
  .clk                  (clk), // FP64_F2I
  .rst                  (rst), // FP64_F2I
  // pipe controls
  .pipeline_flush_i     (pipeline_flush_i), // FP64_F2I
  .start_i              (f2i_start), // FP64_F2I
  .f2i_taking_op_o      (f2i_taking_op), // FP64_F2I
  .f2i_rdy_o            (f2i_rdy), // FP64_F2I
  .rnd_taking_f2i_i     (rnd_taking_f2i), // FP64_F2I
  // input data
  .signa_i              (in_signa), // FP64_F2I
  .exp13a_i             (in_exp13a), // FP64_F2I
  .fract53a_i           (in_fract53a), // FP64_F2I
  .snan_i               (in_snan), // FP64_F2I
  .qnan_i               (in_qnan), // FP64_F2I
  // output data for rounding
  .f2i_sign_o           (f2i_sign), // FP64_F2I
  .f2i_int53_o          (f2i_int53), // FP64_F2I
  .f2i_shr_o            (f2i_shr), // FP64_F2I
  .f2i_shl_o            (f2i_shl), // FP64_F2I
  .f2i_ovf_o            (f2i_ovf), // FP64_F2I
  .f2i_snan_o           (f2i_snan) // FP64_F2I
);


// multiplexing and rounding
pfpu64_rnd_marocchino u_fp64_rnd
(
  // clocks, resets
  .clk                      (clk), // FP64_RND
  .rst                      (rst), // FP64_RND
  // pipe controls
  .pipeline_flush_i         (pipeline_flush_i), // FP64_RND
  .rnd_taking_add_o         (rnd_taking_add), // FP64_RND
  .rnd_taking_mul_o         (rnd_taking_mul), // FP64_RND
  .rnd_taking_div_o         (rnd_taking_div), // FP64_RND
  .rnd_taking_i2f_o         (rnd_taking_i2f), // FP64_RND
  .rnd_taking_f2i_o         (rnd_taking_f2i), // FP64_RND
  .fp64_arith_valid_o       (fp64_arith_valid_o), // FP64_RND
  .padv_wb_i                (padv_wb_i), // FP64_RND
  .grant_wb_to_fp64_arith_i (grant_wb_to_fp64_arith_i), // FP64_RND
  // configuration
  .rmode_i                  (round_mode_i), // FP64_RND
  .except_fpu_enable_i      (except_fpu_enable_i), // FP64_RND
  .ctrl_fpu_mask_flags_i    (ctrl_fpu_mask_flags_i), // FP64_RND
  // from add/sub
  .add_rdy_i       (rnd_muxing_add), // FP64_RND
  .add_sign_i      (add_sign), // FP64_RND
  .add_sub_0_i     (add_sub_0), // FP64_RND
  .add_shl_i       (add_shl), // FP64_RND
  .add_exp13shl_i  (add_exp13shl), // FP64_RND
  .add_exp13sh0_i  (add_exp13sh0), // FP64_RND
  .add_fract57_i   (add_fract57), // FP64_RND
  .add_inv_i       (add_inv), // FP64_RND
  .add_inf_i       (add_inf), // FP64_RND
  .add_snan_i      (add_snan), // FP64_RND
  .add_qnan_i      (add_qnan), // FP64_RND
  .add_anan_sign_i (add_anan_sign), // FP64_RND
  // from mul
  .mul_rdy_i       (rnd_muxing_mul), // FP64_RND
  .mul_sign_i      (mul_sign), // FP64_RND
  .mul_shr_i       (mul_shr), // FP64_RND
  .mul_exp13shr_i  (mul_exp13shr), // FP64_RND
  .mul_exp13sh0_i  (mul_exp13sh0), // FP64_RND
  .mul_fract57_i   (mul_fract57), // FP64_RND
  .mul_inv_i       (mul_inv), // FP64_RND
  .mul_inf_i       (mul_inf), // FP64_RND
  .mul_snan_i      (mul_snan), // FP64_RND
  .mul_qnan_i      (mul_qnan), // FP64_RND
  .mul_anan_sign_i (mul_anan_sign), // FP64_RND
  // from div
  .div_rdy_i        (rnd_muxing_div), // FP64_RND
  .div_sign_i       (div_sign), // FP64_RND
  .div_shr_i        (div_shr), // FP64_RND
  .div_exp13shr_i   (div_exp13shr), // FP64_RND
  .div_shl_i        (div_shl), // FP64_RND
  .div_exp13shl_i   (div_exp13shl), // FP64_RND
  .div_exp13sh0_i   (div_exp13sh0), // FP64_RND
  .div_fract57_i    (div_fract57), // FP64_RND
  .div_dbz_i        (div_dbz), // FP64_RND
  .div_inv_i        (div_inv), // FP64_RND
  .div_inf_i        (div_inf), // FP64_RND
  .div_snan_i       (div_snan), // FP64_RND
  .div_qnan_i       (div_qnan), // FP64_RND
  .div_anan_sign_i  (div_anan_sign), // FP64_RND
  // from i2f
  .i2f_rdy_i       (rnd_muxing_i2f), // FP64_RND
  .i2f_sign_i      (i2f_sign), // FP64_RND
  .i2f_shr_i       (i2f_shr), // FP64_RND
  .i2f_exp11shr_i  (i2f_exp11shr), // FP64_RND
  .i2f_shl_i       (i2f_shl), // FP64_RND
  .i2f_exp11shl_i  (i2f_exp11shl), // FP64_RND
  .i2f_exp11sh0_i  (i2f_exp11sh0), // FP64_RND
  .i2f_fract64_i   (i2f_fract64), // FP64_RND
  // from f2i
  .f2i_rdy_i       (rnd_muxing_f2i), // FP64_RND
  .f2i_sign_i      (f2i_sign), // FP64_RND
  .f2i_int53_i     (f2i_int53), // FP64_RND
  .f2i_shr_i       (f2i_shr), // FP64_RND
  .f2i_shl_i       (f2i_shl), // FP64_RND
  .f2i_ovf_i       (f2i_ovf), // FP64_RND
  .f2i_snan_i      (f2i_snan), // FP64_RND
  // output WB latches
  .wb_fp64_arith_res_hi_o   (wb_fp64_arith_res_hi_o), // FP64_RND
  .wb_fp64_arith_res_lo_o   (wb_fp64_arith_res_lo_o), // FP64_RND
  .wb_fp64_arith_fpcsr_o    (wb_fp64_arith_fpcsr_o), // FP64_RND
  .wb_fp64_arith_wb_fpcsr_o (wb_fp64_arith_wb_fpcsr_o), // FP64_RND
  .wb_except_fp64_arith_o   (wb_except_fp64_arith_o) // FP64_RND
);


// FP64 Comparison
pfpu64_fcmp_marocchino u_fp64_cmp
(
  // clock and reset
  .clk                        (clk), // FP64_CMP
  .rst                        (rst), // FP64_CMP
  // pipeline controls
  .pipeline_flush_i           (pipeline_flush_i), // FP64_CMP
  .padv_wb_i                  (padv_wb_i), // FP64_CMP
  .grant_wb_to_fp64_cmp_i     (grant_wb_to_fp64_cmp_i), // FP64_CMP
  // command
  .op_fp64_cmp_i              (?!), // FP64_CMP
  .opc_fp64_cmp_i             (?!), // FP64_CMP
  // data related to operand A
  .in_signa_i                 (in_signa), // FP64_CMP
  .in_opa_0_i                 (in_opa_0), // FP64_CMP
  .in_infa_i                  (in_infa), // FP64_CMP
  // data related to operand B
  .in_signb_i                 (in_signb), // FP64_CMP
  .in_opb_0_i                 (in_opb), // FP64_CMP
  .in_infb_i                  (in_infb), // FP64_CMP
  // data related to operand A|B
  .in_snan_i                  (in_snan), // FP64_CMP
  .in_qnan_i                  (in_qnan), // FP64_CMP
  .exp_gt_i                   (exp_gt), // FP64_CMP
  .exp_eq_i                   (exp_eq), // FP64_CMP
  .fract_gt_i                 (fract_gt), // FP64_CMP
  .fract_eq_i                 (fract_eq), // FP64_CMP
  // Modes
  .except_fpu_enable_i        (except_fpu_enable_i), // FP64_CMP
  .ctrl_fpu_mask_flags_inv_i  (ctrl_fpu_mask_flags_i[`OR1K_FPCSR_IVF - `OR1K_FPCSR_OVF]), // FP64_CMP
  .ctrl_fpu_mask_flags_inf_i  (ctrl_fpu_mask_flags_i[`OR1K_FPCSR_INF - `OR1K_FPCSR_OVF]), // FP64_CMP
  // Outputs
  //  # WB-latched
  .wb_fp64_flag_set_o         (wb_fp64_flag_set_o), // FP64_CMP
  .wb_fp64_flag_clear_o       (wb_fp64_flag_clear_o), // FP64_CMP 
  .wb_fp64_cmp_inv_o          (wb_fp64_cmp_inv_o), // FP64_CMP      
  .wb_fp64_cmp_inf_o          (wb_fp64_cmp_inf_o), // FP64_CMP      
  .wb_fp64_cmp_wb_fpcsr_o     (wb_fp64_cmp_wb_fpcsr_o), // FP64_CMP  
  .wb_except_fp64_cmp_o       (wb_except_fp64_cmp_o) // FP64_CMP
);

endmodule // pfpu64_top_marocchino
