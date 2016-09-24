/////////////////////////////////////////////////////////////////////
////                                                             ////
////  pfpu32_top_marocchino                                      ////
////  32-bit floating point top level for MAROCCHINO pipeline    ////
////                                                             ////
////  Author: Andrey Bacherov                                    ////
////          avbacherov@opencores.org                           ////
////                                                             ////
/////////////////////////////////////////////////////////////////////
////                                                             ////
//// Copyright (C) 2015 Andrey Bacherov                          ////
////                    avbacherov@opencores.org                 ////
////                                                             ////
//// This source file may be used and distributed without        ////
//// restriction provided that this copyright statement is not   ////
//// removed from the file and that any derivative work contains ////
//// the original copyright notice and the associated disclaimer.////
////                                                             ////
////     THIS SOFTWARE IS PROVIDED ``AS IS'' AND WITHOUT ANY     ////
//// EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED   ////
//// TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS   ////
//// FOR A PARTICULAR PURPOSE. IN NO EVENT SHALL THE AUTHOR      ////
//// OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,         ////
//// INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES    ////
//// (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE   ////
//// GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR        ////
//// BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF  ////
//// LIABILITY, WHETHER IN  CONTRACT, STRICT LIABILITY, OR TORT  ////
//// (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT  ////
//// OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE         ////
//// POSSIBILITY OF SUCH DAMAGE.                                 ////
////                                                             ////
/////////////////////////////////////////////////////////////////////

`include "mor1kx-defines.v"

//---------------------------------------------------------------//
// Order Control Buffer for FPU32                                //
//   simplified version of mor1kx_ocb_marocchino                 //
//   it also contents only 4 order taps and only last tap's      //
//   output is required here                                     //
//---------------------------------------------------------------//

module pfpu32_ocb_marocchino
(
  // clocks and resets
  input   clk,
  input   rst,
  // pipe controls
  input   pipeline_flush_i,
  input   take_op_fp32_arith_i,  // write: an FPU pipe is taking operands
  input   rnd_taking_op_i,       // read:  rounding engine is taking result
  // data input
  input   add_start_i,
  input   mul_start_i,
  input   i2f_start_i,
  input   f2i_start_i,
  // data ouputs
  output  grant_rnd_to_add_o,
  output  grant_rnd_to_mul_o,
  output  grant_rnd_to_i2f_o,
  output  grant_rnd_to_f2i_o,
  // "OCB is full" flag
  //   (a) external control logic must stop the "writing without reading"
  //       operation if OCB is full
  //   (b) however, the "writing + reading" is possible
  //       because it just pushes OCB and keeps it full
  output  pfpu32_ocb_full_o
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
  assign pfpu32_ocb_full_o = ptr_curr[NUM_TAPS];

  // control to increment/decrement pointers
  wire rd_only = ~take_op_fp32_arith_i &  rnd_taking_op_i;
  wire wr_only =  take_op_fp32_arith_i & ~rnd_taking_op_i;
  wire wr_rd   =  take_op_fp32_arith_i &  rnd_taking_op_i;


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
    en_curr_tap |           // tap[ptr_curr] <= ocbi_i (note: by wr_only)
    {NUM_TAPS{rnd_taking_op_i}};  // tap[k-1] <= tap[k]

  // control for forwarding multiplexors
  wire [NUM_TAPS-1:0] use_forwarded_value =
    en_curr_tap |                   // tap[ptr_curr] <= ocbi_i (note: by wr_only)
    ({NUM_TAPS{wr_rd}} & ptr_prev); // tap[ptr_prev] <= ocbi_i;


  // order input
  wire [3:0] ocbi = {add_start_i, mul_start_i, i2f_start_i, f2i_start_i};

  // taps ouputs
  wire [3:0] ocbo00; // OCB output
  wire [3:0] ocbo01; // ...
  wire [3:0] ocbo02; // ...
  wire [3:0] ocbo03; // OCB entrance

  // granting flags output
  assign {grant_rnd_to_add_o, grant_rnd_to_mul_o, grant_rnd_to_i2f_o, grant_rnd_to_f2i_o} = ocbo00;

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
endmodule // pfpu32_ocb_marocchino


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


module pfpu32_top_marocchino
(
  // clock & reset
  input                             clk,
  input                             rst,

  // pipeline control
  input                             flush_i,
  input                             padv_decode_i,
  input                             padv_wb_i,
  input                             grant_wb_to_fp32_arith_i,

  // pipeline control outputs
  output                            fp32_arith_busy_o,     // idicates that arihmetic units are busy
  output                            fp32_arith_valid_o,    // WB-latching ahead arithmetic ready flag

  // Configuration
  input   [`OR1K_FPCSR_RM_SIZE-1:0] round_mode_i,
  input                             except_fpu_enable_i,
  input [`OR1K_FPCSR_ALLF_SIZE-1:0] ctrl_fpu_mask_flags_i,

  // Operands and commands
  input                             dcod_op_fp32_arith_i,
  input                       [2:0] dcod_opc_fp32_arith_i, // [000]...[101] : add...f2i
  //   from DECODE
  input                      [31:0] dcod_rfa_i,
  input                      [31:0] dcod_rfb_i,
  //   forwarding from WB
  input                             exe2dec_hazard_a_i,
  input                             exe2dec_hazard_b_i,
  input                      [31:0] wb_result_i,

  // FPU-32 arithmetic part
  output                      [31:0] wb_fp32_arith_res_o,      // arithmetic result
  output [`OR1K_FPCSR_ALLF_SIZE-1:0] wb_fp32_arith_fpcsr_o,    // arithmetic exceptions
  output                             wb_fp32_arith_wb_fpcsr_o, // update FPCSR
  output                             wb_except_fp32_arith_o    // generate exception

);

// fp32 arithmetic command
wire        op_fp32_arith;
wire  [2:0] opc_fp32_arith;

// fp32 pipes controls
wire        take_op_fp32_arith;

// operand A and B  with forwarding from WB
wire [31:0] fp32_arith_a;
wire [31:0] fp32_arith_b;

// reservation station insrtance
mor1kx_rsrvs_marocchino
#(
  .OPTION_OPERAND_WIDTH (32), // FP32_ARITH_RSVRS
  .OPC_WIDTH            (3) // FP32_ARITH_RSVRS
)
u_fp32_arith_rsrvs
(
  // clocks and resets
  .clk                  (clk),
  .rst                  (rst),
  // pipeline control signals in
  .pipeline_flush_i     (flush_i), // FP32_ARITH_RSVRS
  .padv_decode_i        (padv_decode_i), // FP32_ARITH_RSVRS
  .take_op_i            (take_op_fp32_arith), // FP32_ARITH_RSVRS
  // input data
  //   from DECODE
  .dcod_rfa_i           (dcod_rfa_i), // FP32_ARITH_RSVRS
  .dcod_rfb_i           (dcod_rfb_i), // FP32_ARITH_RSVRS
  //   forwarding from WB
  .exe2dec_hazard_a_i   (exe2dec_hazard_a_i), // FP32_ARITH_RSVRS
  .exe2dec_hazard_b_i   (exe2dec_hazard_b_i), // FP32_ARITH_RSVRS
  .wb_result_i          (wb_result_i), // FP32_ARITH_RSVRS
  // command and its additional attributes
  .dcod_op_i            (dcod_op_fp32_arith_i), // FP32_ARITH_RSVRS
  .dcod_opc_i           (dcod_opc_fp32_arith_i), // FP32_ARITH_RSVRS
  // outputs
  //   command and its additional attributes
  .exec_op_o            (op_fp32_arith), // FP32_ARITH_RSVRS
  .exec_opc_o           (opc_fp32_arith),
  //   operands
  .exec_rfa_o           (fp32_arith_a), // FP32_ARITH_RSVRS
  .exec_rfb_o           (fp32_arith_b), // FP32_ARITH_RSVRS
  //   unit-is-busy flag
  .unit_busy_o          (fp32_arith_busy_o) // FP32_ARITH_RSVRS
);

// analysis of input values
//   split input a
wire        in_signa  = fp32_arith_a[31];
wire [7:0]  in_expa   = fp32_arith_a[30:23];
wire [22:0] in_fracta = fp32_arith_a[22:0];
//   detect infinity a
wire in_expa_ff = &in_expa;
wire in_infa    = in_expa_ff & (~(|in_fracta));
//   signaling NaN: exponent is 8hff, [22] is zero,
//                  rest of fract is non-zero
//   quiet NaN: exponent is 8hff, [22] is 1
wire in_snan_a = in_expa_ff & (~in_fracta[22]) & (|in_fracta[21:0]);
wire in_qnan_a = in_expa_ff &   in_fracta[22];
//   denormalized/zero of a
wire in_opa_0  = ~(|fp32_arith_a[30:0]);
wire in_opa_dn = (~(|in_expa)) & (|in_fracta);

//   split input b
wire        in_signb  = fp32_arith_b[31];
wire [7:0]  in_expb   = fp32_arith_b[30:23];
wire [22:0] in_fractb = fp32_arith_b[22:0];
//   detect infinity b
wire in_expb_ff = &in_expb;
wire in_infb    = in_expb_ff & (~(|in_fractb));
//   detect NaNs in b
wire in_snan_b = in_expb_ff & (~in_fractb[22]) & (|in_fractb[21:0]);
wire in_qnan_b = in_expb_ff &   in_fractb[22];
//   denormalized/zero of a
wire in_opb_0  = ~(|fp32_arith_b[30:0]);
wire in_opb_dn = (~(|in_expb)) & (|in_fractb);

// detection of some exceptions
//   a nan input -> qnan output
wire in_snan = in_snan_a | in_snan_b;
wire in_qnan = in_qnan_a | in_qnan_b;
//   sign of output nan
wire in_anan_sign = (in_snan_a | in_qnan_a) ? in_signa :
                                              in_signb;

// restored exponents
wire [9:0] in_exp10a = {2'd0,in_expa[7:1],(in_expa[0] | in_opa_dn)};
wire [9:0] in_exp10b = {2'd0,in_expb[7:1],(in_expb[0] | in_opb_dn)};
// restored fractionals
wire [23:0] in_fract24a = {((~in_opa_dn) & (~in_opa_0)),in_fracta};
wire [23:0] in_fract24b = {((~in_opb_dn) & (~in_opb_0)),in_fractb};


// Support for ADD/SUB (historically they were comparator's part)
//  # exponents
wire exp_gt = in_exp10a  > in_exp10b;
wire exp_eq = in_exp10a == in_exp10b;
//  # fractionals
wire fract_gt = in_fract24a  > in_fract24b;
wire fract_eq = in_fract24a == in_fract24b;
//  # comparisons for ADD/SUB
wire addsub_agtb = exp_gt | (exp_eq & fract_gt);
wire addsub_aeqb = exp_eq & fract_eq;


// order control buffer is full:
// we are waiting an arithmetic pipe result for rounding
wire pfpu32_ocb_full;

// unit-wise control signals
//  ## ADD / SUB
wire op_add             = (opc_fp32_arith == 3'd0) & op_fp32_arith & (~pfpu32_ocb_full);
wire op_sub             = (opc_fp32_arith == 3'd1) & op_fp32_arith & (~pfpu32_ocb_full);
wire add_start          = op_add | op_sub;
wire add_takes_op;
wire add_rdy;
wire grant_rnd_to_add;
wire rnd_muxes_add      = add_rdy & grant_rnd_to_add; // to rounding input muxer
wire rnd_takes_add;
//  ## MUL/DIV
wire op_mul             = (opc_fp32_arith == 3'd2) & op_fp32_arith & (~pfpu32_ocb_full);
wire op_div             = (opc_fp32_arith == 3'd3) & op_fp32_arith & (~pfpu32_ocb_full);
wire mul_start          = op_mul | op_div;
wire muldiv_takes_op;
wire muldiv_rdy;
wire grant_rnd_to_mul;
wire rnd_muxes_muldiv   = muldiv_rdy & grant_rnd_to_mul; // to rounding input muxer
wire rnd_takes_muldiv;
//  ## i2f
wire i2f_start          = (opc_fp32_arith == 3'd4) & op_fp32_arith & (~pfpu32_ocb_full);
wire i2f_takes_op;
wire i2f_rdy;
wire grant_rnd_to_i2f;
wire rnd_muxes_i2f      = i2f_rdy & grant_rnd_to_i2f; // to rounding input muxer
wire rnd_takes_i2f;
//  ## f2i
wire f2i_start          = (opc_fp32_arith == 3'd5) & op_fp32_arith & (~pfpu32_ocb_full);
wire f2i_takes_op;
wire f2i_rdy;
wire grant_rnd_to_f2i;
wire rnd_muxes_f2i      = f2i_rdy & grant_rnd_to_f2i; // to rounding input muxer
wire rnd_takes_f2i;

// feedback to drop FP32 arithmetic related command
assign take_op_fp32_arith = add_takes_op | muldiv_takes_op | i2f_takes_op | f2i_takes_op;

// rounding engine takes an OP
wire rnd_taking_op = rnd_takes_add | rnd_takes_muldiv | rnd_takes_i2f | rnd_takes_f2i;


// ---
pfpu32_ocb_marocchino  u_pfpu32_ocb
(
  // clocks and resets
  .clk                    (clk), // PFPU32_OCB
  .rst                    (rst), // PFPU32_OCB
  // pipe controls
  .pipeline_flush_i       (flush_i), // PFPU32_OCB
  .take_op_fp32_arith_i   (take_op_fp32_arith), // PFPU32_OCB
  .rnd_taking_op_i        (rnd_taking_op), // PFPU32_OCB
  // data input
  .add_start_i            (add_start), // PFPU32_OCB
  .mul_start_i            (mul_start), // PFPU32_OCB
  .i2f_start_i            (i2f_start), // PFPU32_OCB
  .f2i_start_i            (f2i_start), // PFPU32_OCB
  // data ouputs
  .grant_rnd_to_add_o     (grant_rnd_to_add), // PFPU32_OCB
  .grant_rnd_to_mul_o     (grant_rnd_to_mul), // PFPU32_OCB
  .grant_rnd_to_i2f_o     (grant_rnd_to_i2f), // PFPU32_OCB
  .grant_rnd_to_f2i_o     (grant_rnd_to_f2i), // PFPU32_OCB
  // "OCB is full" flag
  .pfpu32_ocb_full_o      (pfpu32_ocb_full) // PFPU32_OCB
);


// Addition / Substruction
//   connection wires
wire        add_sign_o;      // add/sub signum
wire        add_sub_0_o;     // flag that actual substruction is performed and result is zero
wire  [4:0] add_shl_o;       // do left shift in align stage
wire  [9:0] add_exp10shl_o;  // exponent for left shift align
wire  [9:0] add_exp10sh0_o;  // exponent for no shift in align
wire [27:0] add_fract28_o;   // fractional with appended {r,s} bits
wire        add_inv_o;       // add/sub invalid operation flag
wire        add_inf_o;       // add/sub infinity output reg
wire        add_snan_o;      // add/sub signaling NaN output reg
wire        add_qnan_o;      // add/sub quiet NaN output reg
wire        add_anan_sign_o; // add/sub signum for output nan
//   module istance
pfpu32_addsub_marocchino u_f32_addsub
(
  // clocks and resets
  .clk              (clk),
  .rst              (rst),
  // ADD/SUB pipe controls
  .pipeline_flush_i (flush_i),   // flush pipe
  .start_i          (add_start), 
  .is_sub_i         (op_sub),    // 1: substruction, 0: addition
  .add_busy_o       (),
  .add_takes_op_o   (add_takes_op),
  .add_rdy_o        (add_rdy),         // add/sub is ready
  .rnd_takes_add_i  (rnd_takes_add),
  // input 'a' related values
  .signa_i          (in_signa),
  .exp10a_i         (in_exp10a),
  .fract24a_i       (in_fract24a),
  .infa_i           (in_infa),
  // input 'b' related values
  .signb_i          (in_signb),
  .exp10b_i         (in_exp10b),
  .fract24b_i       (in_fract24b),
  .infb_i           (in_infb),
  // 'a'/'b' related
  .snan_i           (in_snan),
  .qnan_i           (in_qnan),
  .anan_sign_i      (in_anan_sign),
  .addsub_agtb_i    (addsub_agtb),
  .addsub_aeqb_i    (addsub_aeqb),
  // outputs
  .add_sign_o       (add_sign_o),      // add/sub signum
  .add_sub_0_o      (add_sub_0_o),     // flag that actual substruction is performed and result is zero
  .add_shl_o        (add_shl_o),       // do left shift in align stage
  .add_exp10shl_o   (add_exp10shl_o),  // exponent for left shift align
  .add_exp10sh0_o   (add_exp10sh0_o),  // exponent for no shift in align
  .add_fract28_o    (add_fract28_o),   // fractional with appended {r,s} bits
  .add_inv_o        (add_inv_o),       // add/sub invalid operation flag
  .add_inf_o        (add_inf_o),       // add/sub infinity output reg
  .add_snan_o       (add_snan_o),      // add/sub signaling NaN output reg
  .add_qnan_o       (add_qnan_o),      // add/sub quiet NaN output reg
  .add_anan_sign_o  (add_anan_sign_o)  // add/sub signum for output nan
);

// MUL/DIV combined pipeline
//   MUL/DIV common outputs
wire        mul_sign_o;      // mul signum
wire  [4:0] mul_shr_o;       // do right shift in align stage
wire  [9:0] mul_exp10shr_o;  // exponent for right shift align
wire        mul_shl_o;       // do left shift in align stage
wire  [9:0] mul_exp10shl_o;  // exponent for left shift align
wire  [9:0] mul_exp10sh0_o;  // exponent for no shift in align
wire [27:0] mul_fract28_o;   // fractional with appended {r,s} bits
wire        mul_inv_o;       // mul invalid operation flag
wire        mul_inf_o;       // mul infinity output reg
wire        mul_snan_o;      // mul signaling NaN output reg
wire        mul_qnan_o;      // mul quiet NaN output reg
wire        mul_anan_sign_o; // mul signum for output nan
//   DIV additional outputs
wire        div_op_o;        // operation is division
wire        div_sign_rmnd_o; // signum or reminder for IEEE compliant rounding
wire        div_dbz_o;       // division by zero flag
//   module istance
pfpu32_muldiv_marocchino u_f32_muldiv
(
  // clocks and resets
  .clk                (clk),
  .rst                (rst),
  // pipe controls
  .pipeline_flush_i   (flush_i),  // flushe pipe
  .is_mul_i           (op_mul),
  .is_div_i           (op_div),
  .muldiv_busy_o      (),
  .muldiv_takes_op_o  (muldiv_takes_op),
  .muldiv_rdy_o       (muldiv_rdy),
  .rnd_takes_muldiv_i (rnd_takes_muldiv),
  // input 'a' related values
  .signa_i            (in_signa),
  .exp10a_i           (in_exp10a),
  .fract24a_i         (in_fract24a),
  .infa_i             (in_infa),
  .zeroa_i            (in_opa_0),
  // input 'b' related values
  .signb_i            (in_signb),
  .exp10b_i           (in_exp10b),
  .fract24b_i         (in_fract24b),
  .infb_i             (in_infb),
  .zerob_i            (in_opb_0),
  // 'a'/'b' related
  .snan_i             (in_snan),        
  .qnan_i             (in_qnan),
  .anan_sign_i        (in_anan_sign),
  // MUL/DIV common outputs
  .muldiv_sign_o      (mul_sign_o),      // mul signum
  .muldiv_shr_o       (mul_shr_o),       // do right shift in align stage
  .muldiv_exp10shr_o  (mul_exp10shr_o),  // exponent for right shift align
  .muldiv_shl_o       (mul_shl_o),       // do left shift in align stage
  .muldiv_exp10shl_o  (mul_exp10shl_o),  // exponent for left shift align
  .muldiv_exp10sh0_o  (mul_exp10sh0_o),  // exponent for no shift in align
  .muldiv_fract28_o   (mul_fract28_o),   // fractional with appended {r,s} bits
  .muldiv_inv_o       (mul_inv_o),       // mul invalid operation flag
  .muldiv_inf_o       (mul_inf_o),       // mul infinity output reg
  .muldiv_snan_o      (mul_snan_o),      // mul signaling NaN output reg
  .muldiv_qnan_o      (mul_qnan_o),      // mul quiet NaN output reg
  .muldiv_anan_sign_o (mul_anan_sign_o), // mul signum for output nan
  // DIV additional outputs
  .div_op_o           (div_op_o),        // operation is division
  .div_sign_rmnd_o    (div_sign_rmnd_o), // signum of reminder for IEEE compliant rounding
  .div_dbz_o          (div_dbz_o)        // division by zero flag
);

// convertors
//   i2f connection wires
wire        i2f_sign_o;      // i2f signum
wire  [3:0] i2f_shr_o;
wire  [7:0] i2f_exp8shr_o;
wire  [4:0] i2f_shl_o;
wire  [7:0] i2f_exp8shl_o;
wire  [7:0] i2f_exp8sh0_o;
wire [31:0] i2f_fract32_o;
//   i2f module instance
pfpu32_i2f_marocchino u_i2f_cnv
(
  // clocks and resets
  .clk                (clk),
  .rst                (rst),
  // I2F pipe controls
  .pipeline_flush_i   (flush_i),   // flush pipe
  .start_i            (i2f_start), // start conversion
  .i2f_busy_o         (),
  .i2f_takes_op_o     (i2f_takes_op),
  .i2f_rdy_o          (i2f_rdy),       // i2f is ready
  .rnd_takes_i2f_i    (rnd_takes_i2f),
  // operand for conversion
  .opa_i              (fp32_arith_a),
  // ouputs for rounding
  .i2f_sign_o         (i2f_sign_o),    // i2f signum
  .i2f_shr_o          (i2f_shr_o),
  .i2f_exp8shr_o      (i2f_exp8shr_o),
  .i2f_shl_o          (i2f_shl_o),
  .i2f_exp8shl_o      (i2f_exp8shl_o),
  .i2f_exp8sh0_o      (i2f_exp8sh0_o),
  .i2f_fract32_o      (i2f_fract32_o)
);
//   f2i connection wires
wire        f2i_sign_o;      // f2i signum
wire [23:0] f2i_int24_o;     // f2i fractional
wire  [4:0] f2i_shr_o;       // f2i required shift right value
wire  [3:0] f2i_shl_o;       // f2i required shift left value   
wire        f2i_ovf_o;       // f2i overflow flag
wire        f2i_snan_o;      // f2i signaling NaN output reg
//    f2i module instance
pfpu32_f2i_marocchino u_f2i_cnv
(
  // clocks and resets
  .clk                  (clk),
  .rst                  (rst),
  // pipe controls
  .pipeline_flush_i     (flush_i),        // flush pipe
  .start_i              (f2i_start),      // start conversion
  .f2i_busy_o           (),
  .f2i_takes_op_o       (f2i_takes_op),
  .f2i_rdy_o            (f2i_rdy),         // f2i is ready
  .rnd_takes_f2i_i      (rnd_takes_f2i),
  // input data
  .signa_i              (in_signa),       // input 'a' related values
  .exp10a_i             (in_exp10a),
  .fract24a_i           (in_fract24a),
  .snan_i               (in_snan),         // 'a'/'b' related
  .qnan_i               (in_qnan),
  // output data for rounding
  .f2i_sign_o           (f2i_sign_o),      // f2i signum
  .f2i_int24_o          (f2i_int24_o),     // f2i fractional
  .f2i_shr_o            (f2i_shr_o),       // f2i required shift right value
  .f2i_shl_o            (f2i_shl_o),       // f2i required shift left value   
  .f2i_ovf_o            (f2i_ovf_o),       // f2i overflow flag
  .f2i_snan_o           (f2i_snan_o)       // f2i signaling NaN output reg
);


// multiplexing and rounding
pfpu32_rnd_marocchino u_f32_rnd
(
  // clocks, resets
  .clk                      (clk),
  .rst                      (rst),
  // pipe controls
  .pipeline_flush_i         (flush_i),// flush pipe
  .fp32_rnd_busy_o          (),
  .rnd_takes_add_o          (rnd_takes_add),
  .rnd_takes_mul_o          (rnd_takes_muldiv),
  .rnd_takes_i2f_o          (rnd_takes_i2f),
  .rnd_takes_f2i_o          (rnd_takes_f2i),
  .fp32_arith_valid_o       (fp32_arith_valid_o),
  .padv_wb_i                (padv_wb_i),       // arith. advance output latches
  .grant_wb_to_fp32_arith_i (grant_wb_to_fp32_arith_i),
  // configuration
  .rmode_i                  (round_mode_i),    // rounding mode
  .except_fpu_enable_i      (except_fpu_enable_i),
  .ctrl_fpu_mask_flags_i    (ctrl_fpu_mask_flags_i),
  // from add/sub
  .add_rdy_i       (rnd_muxes_add),   // add/sub is ready
  .add_sign_i      (add_sign_o),      // add/sub signum
  .add_sub_0_i     (add_sub_0_o),     // flag that actual substruction is performed and result is zero
  .add_shl_i       (add_shl_o),       // do left shift in align stage
  .add_exp10shl_i  (add_exp10shl_o),  // exponent for left shift align
  .add_exp10sh0_i  (add_exp10sh0_o),  // exponent for no shift in align
  .add_fract28_i   (add_fract28_o),   // fractional with appended {r,s} bits
  .add_inv_i       (add_inv_o),       // add/sub invalid operation flag
  .add_inf_i       (add_inf_o),       // add/sub infinity
  .add_snan_i      (add_snan_o),      // add/sub signaling NaN
  .add_qnan_i      (add_qnan_o),      // add/sub quiet NaN
  .add_anan_sign_i (add_anan_sign_o), // add/sub signum for output nan
  // from mul
  .mul_rdy_i       (rnd_muxes_muldiv),// mul is ready
  .mul_sign_i      (mul_sign_o),      // mul signum
  .mul_shr_i       (mul_shr_o),       // do right shift in align stage
  .mul_exp10shr_i  (mul_exp10shr_o),  // exponent for right shift align
  .mul_shl_i       (mul_shl_o),       // do left shift in align stage
  .mul_exp10shl_i  (mul_exp10shl_o),  // exponent for left shift align
  .mul_exp10sh0_i  (mul_exp10sh0_o),  // exponent for no shift in align
  .mul_fract28_i   (mul_fract28_o),   // fractional with appended {r,s} bits
  .mul_inv_i       (mul_inv_o),       // mul invalid operation flag
  .mul_inf_i       (mul_inf_o),       // mul infinity 
  .mul_snan_i      (mul_snan_o),      // mul signaling NaN
  .mul_qnan_i      (mul_qnan_o),      // mul quiet NaN
  .mul_anan_sign_i (mul_anan_sign_o), // mul signum for output nan
  .div_op_i        (div_op_o),         // MUL/DIV output is division
  .div_sign_rmnd_i (div_sign_rmnd_o),  // signum or reminder for IEEE compliant rounding
  .div_dbz_i       (div_dbz_o),        // division by zero flag
  // from i2f
  .i2f_rdy_i       (rnd_muxes_i2f),   // i2f is ready
  .i2f_sign_i      (i2f_sign_o),      // i2f signum
  .i2f_shr_i       (i2f_shr_o),
  .i2f_exp8shr_i   (i2f_exp8shr_o),
  .i2f_shl_i       (i2f_shl_o),
  .i2f_exp8shl_i   (i2f_exp8shl_o),
  .i2f_exp8sh0_i   (i2f_exp8sh0_o),
  .i2f_fract32_i   (i2f_fract32_o),
  // from f2i
  .f2i_rdy_i       (rnd_muxes_f2i),   // f2i is ready
  .f2i_sign_i      (f2i_sign_o),      // f2i signum
  .f2i_int24_i     (f2i_int24_o),     // f2i fractional
  .f2i_shr_i       (f2i_shr_o),       // f2i required shift right value
  .f2i_shl_i       (f2i_shl_o),       // f2i required shift left value   
  .f2i_ovf_i       (f2i_ovf_o),       // f2i overflow flag
  .f2i_snan_i      (f2i_snan_o),      // f2i signaling NaN
  // output WB latches
  .wb_fp32_arith_res_o      (wb_fp32_arith_res_o),
  .wb_fp32_arith_fpcsr_o    (wb_fp32_arith_fpcsr_o),
  .wb_fp32_arith_wb_fpcsr_o (wb_fp32_arith_wb_fpcsr_o), // update FPCSR
  .wb_except_fp32_arith_o   (wb_except_fp32_arith_o)    // generate exception
);

endmodule // pfpu32_top_marocchino
