/////////////////////////////////////////////////////////////////////
//                                                                 //
//    pfpu32_rnd_marocchino                                        //
//                                                                 //
//    32-bit common rounding module for FPU                        //
//    adaptation for MAROCCHINO pipeline                           //
//      (a) remove comparison result from common output latches    //
//      (b) latch output by padv-wb-i                              //
//                                                                 //
//    This file is part of the mor1kx project                      //
//    https://github.com/openrisc/mor1kx                           //
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

module pfpu32_rnd_marocchino
(
  // clocks, resets
  input                             clk,
  input                             rst,
  // pipe controls
  input                             pipeline_flush_i,
  output                            rnd_taking_add_o,
  output                            rnd_taking_mul_o,
  output                            rnd_taking_i2f_o,
  output                            rnd_taking_f2i_o,
  output                            fp32_arith_valid_o,
  input                             padv_wb_i,
  input                             grant_wb_to_fp32_arith_i,
  // configuration
  input                       [1:0] rmode_i,  // rounding mode
  input                             except_fpu_enable_i,
  input [`OR1K_FPCSR_ALLF_SIZE-1:0] ctrl_fpu_mask_flags_i,
  // input from add/sub
  input        add_rdy_i,       // add/sub is ready
  input        add_sign_i,      // add/sub signum
  input        add_sub_0_i,     // flag that actual substruction is performed and result is zero
  input  [4:0] add_shl_i,       // do left shift in align stage
  input  [9:0] add_exp10shl_i,  // exponent for left shift align
  input  [9:0] add_exp10sh0_i,  // exponent for no shift in align
  input [27:0] add_fract28_i,   // fractional with appended {r,s} bits
  // input from mul
  input        mul_rdy_i,       // mul is ready
  input        mul_sign_i,      // mul signum
  input  [4:0] mul_shr_i,       // do right shift in align stage
  input  [9:0] mul_exp10shr_i,  // exponent for right shift align
  input        mul_shl_i,       // do left shift in align stage
  input  [9:0] mul_exp10shl_i,  // exponent for left shift align
  input  [9:0] mul_exp10sh0_i,  // exponent for no shift in align
  input [27:0] mul_fract28_i,   // fractional with appended {r,s} bits
  // input from div
  input        div_op_i,        // MUL/DIV output is division
  input        div_sign_rmnd_i, // signum or reminder for IEEE compliant rounding
  input        div_dbz_i,       // division by zero flag
  // input from i2f
  input        i2f_rdy_i,       // i2f is ready
  input        i2f_sign_i,      // i2f signum
  input  [3:0] i2f_shr_i,
  input  [7:0] i2f_exp8shr_i,
  input  [4:0] i2f_shl_i,
  input  [7:0] i2f_exp8shl_i,
  input  [7:0] i2f_exp8sh0_i,
  input [31:0] i2f_fract32_i,
  // input from f2i
  input        f2i_rdy_i,       // f2i is ready
  input        f2i_sign_i,      // f2i signum
  input [23:0] f2i_int24_i,     // f2i fractional
  input  [4:0] f2i_shr_i,       // f2i required shift right value
  input  [3:0] f2i_shl_i,       // f2i required shift left value
  input        f2i_ovf_i,       // f2i overflow flag
  // from order control buffer
  input        ocb_inv_i,
  input        ocb_inf_i,
  input        ocb_snan_i,
  input        ocb_qnan_i,
  input        ocb_anan_sign_i,
  // output WB latches
  output reg                      [31:0] wb_fp32_arith_res_o,      // result
  output reg [`OR1K_FPCSR_ALLF_SIZE-1:0] wb_fp32_arith_fpcsr_o,    // fp32 arithmetic flags
  output reg                             wb_fp32_arith_wb_fpcsr_o, // update FPCSR
  output reg                             wb_except_fp32_arith_o    // generate exception
);

  localparam INF  = 31'b1111111100000000000000000000000;
  localparam QNAN = 31'b1111111110000000000000000000000;
  localparam SNAN = 31'b1111111101111111111111111111111;

  // rounding mode isn't require pipelinization
  wire rm_nearest = (rmode_i==2'b00);
  wire rm_to_zero = (rmode_i==2'b01);
  wire rm_to_infp = (rmode_i==2'b10);
  wire rm_to_infm = (rmode_i==2'b11);

  /*
     Any stage's output is registered.
     Definitions:
       s??o_name - "S"tage number "??", "O"utput
       s??t_name - "S"tage number "??", "T"emporary (internally)
  */


  // rounding pipe controls
  //  ## resdy flags of stages
  reg s1o_ready;
  //  ## per stage busy flags
  wire s1_busy = s1o_ready & ~(padv_wb_i & grant_wb_to_fp32_arith_i);
  //  ## per stage advance
  wire s1_adv  = (add_rdy_i | mul_rdy_i | i2f_rdy_i | f2i_rdy_i) & ~s1_busy;
  // ## per execution unit reporting
  assign rnd_taking_add_o = add_rdy_i & ~s1_busy;
  assign rnd_taking_mul_o = mul_rdy_i & ~s1_busy;
  assign rnd_taking_i2f_o = i2f_rdy_i & ~s1_busy;
  assign rnd_taking_f2i_o = f2i_rdy_i & ~s1_busy;

  // output of rounding pipe state
  assign fp32_arith_valid_o = s1o_ready;


  /* Stage #1: common align */

  wire        s1t_sign;
  wire [34:0] s1t_fract35;
  wire  [4:0] s1t_shr;
  wire  [4:0] s1t_shl;

  // multiplexer for signums and flags
  wire s1t_add_sign = add_sub_0_i ? rm_to_infm : add_sign_i;

  assign s1t_sign = (add_rdy_i & s1t_add_sign) |
                    (mul_rdy_i & mul_sign_i)   |
                    (f2i_rdy_i & f2i_sign_i)   |
                    (i2f_rdy_i & i2f_sign_i);

  // multiplexer for fractionals
  assign s1t_fract35 =
    ({35{add_rdy_i}} & {7'd0, add_fract28_i}) |
    ({35{mul_rdy_i}} & {7'd0, mul_fract28_i}) |
    ({35{f2i_rdy_i}} & {8'd0, f2i_int24_i, 3'd0}) |
    ({35{i2f_rdy_i}} & {i2f_fract32_i,3'd0});

  // overflow bit for add/mul
  wire s1t_addmul_carry = (add_rdy_i & add_fract28_i[27]) |
                          (mul_rdy_i & mul_fract28_i[27]);

  // multiplexer for shift values
  wire [4:0] s1t_shr_t;
  assign {s1t_shr_t, s1t_shl} =
    ({10{add_rdy_i}} & {5'd0, add_shl_i}) |
    ({10{mul_rdy_i}} & {mul_shr_i, {4'd0,mul_shl_i}}) |
    ({10{f2i_rdy_i}} & {f2i_shr_i, {1'b0,f2i_shl_i}}) |
    ({10{i2f_rdy_i}} & {{1'b0,i2f_shr_i}, i2f_shl_i});

  assign s1t_shr = (|s1t_shr_t) ? s1t_shr_t : {4'd0,s1t_addmul_carry};

  // align
  wire [34:0] s1t_fract35sh =
    (|s1t_shr) ? (s1t_fract35 >> s1t_shr) :
                 (s1t_fract35 << s1t_shl);

  // update sticky bit for right shift case.
  // maximum right shift value is :
  //    27 for mul/div
  //     8 for i2f
  reg s1r_sticky;
  always @(s1t_fract35 or s1t_shr) begin
    case (s1t_shr)
      5'd0   : s1r_sticky = |s1t_fract35[ 1:0];
      5'd1   : s1r_sticky = |s1t_fract35[ 2:0];
      5'd2   : s1r_sticky = |s1t_fract35[ 3:0];
      5'd3   : s1r_sticky = |s1t_fract35[ 4:0];
      5'd4   : s1r_sticky = |s1t_fract35[ 5:0];
      5'd5   : s1r_sticky = |s1t_fract35[ 6:0];
      5'd6   : s1r_sticky = |s1t_fract35[ 7:0];
      5'd7   : s1r_sticky = |s1t_fract35[ 8:0];
      5'd8   : s1r_sticky = |s1t_fract35[ 9:0];
      5'd9   : s1r_sticky = |s1t_fract35[10:0];
      5'd10  : s1r_sticky = |s1t_fract35[11:0];
      5'd11  : s1r_sticky = |s1t_fract35[12:0];
      5'd12  : s1r_sticky = |s1t_fract35[13:0];
      5'd13  : s1r_sticky = |s1t_fract35[14:0];
      5'd14  : s1r_sticky = |s1t_fract35[15:0];
      5'd15  : s1r_sticky = |s1t_fract35[16:0];
      5'd16  : s1r_sticky = |s1t_fract35[17:0];
      5'd17  : s1r_sticky = |s1t_fract35[18:0];
      5'd18  : s1r_sticky = |s1t_fract35[19:0];
      5'd19  : s1r_sticky = |s1t_fract35[20:0];
      5'd20  : s1r_sticky = |s1t_fract35[21:0];
      5'd21  : s1r_sticky = |s1t_fract35[22:0];
      5'd22  : s1r_sticky = |s1t_fract35[23:0];
      5'd23  : s1r_sticky = |s1t_fract35[24:0];
      5'd24  : s1r_sticky = |s1t_fract35[25:0];
      5'd25  : s1r_sticky = |s1t_fract35[26:0];
      default: s1r_sticky = |s1t_fract35[27:0];
    endcase
  end // always

  // update sticky bit for left shift case.
  reg s1l_sticky;
  always @(s1t_fract35 or s1t_shl) begin
    case (s1t_shl)
      5'd0   : s1l_sticky = |s1t_fract35[1:0];
      5'd1   : s1l_sticky =  s1t_fract35[0];
      default: s1l_sticky = 1'b0;
    endcase
  end // always

  wire s1t_sticky = (|s1t_shr) ? s1r_sticky : s1l_sticky;

  // two stage multiplexer for exponents
  wire [9:0] s1t_exp10shr;
  wire [9:0] s1t_exp10shl;
  wire [9:0] s1t_exp10sh0;
  assign {s1t_exp10shr, s1t_exp10shl, s1t_exp10sh0} =
    ({30{add_rdy_i}} & {add_exp10sh0_i, add_exp10shl_i, add_exp10sh0_i}) |
    ({30{mul_rdy_i}} & {mul_exp10shr_i, mul_exp10shl_i, mul_exp10sh0_i}) |
    ({30{f2i_rdy_i}} & {10'd0, 10'd0, 10'd0}) |
    ({30{i2f_rdy_i}} & {{2'd0,i2f_exp8shr_i},{2'd0,i2f_exp8shl_i},{2'd0,i2f_exp8sh0_i}});

  wire [9:0] s1t_exp10 =
    (|s1t_shr_t)  ? s1t_exp10shr :
    (~(|s1t_shl)) ? (s1t_exp10sh0 + {9'd0,s1t_addmul_carry}) :
                    s1t_exp10shl;

  // output of align stage
  reg        s1o_sign;
  reg  [9:0] s1o_exp10;
  reg [31:0] s1o_fract32;
  reg  [1:0] s1o_rs;
  reg        s1o_inv;
  reg        s1o_inf;
  reg        s1o_snan_i;
  reg        s1o_qnan_i;
  reg        s1o_anan_sign_i;
  reg        s1o_div_op, s1o_div_sign_rmnd, s1o_div_dbz;
  reg        s1o_f2i_ovf, s1o_f2i;
  // registering
  always @(posedge clk) begin
    if(s1_adv) begin
      s1o_sign    <= s1t_sign;
      s1o_exp10   <= s1t_exp10;
      s1o_fract32 <= s1t_fract35sh[34:3];
      s1o_rs      <= {s1t_fract35sh[2],s1t_sticky};
      // various flags:
      s1o_inv         <= ocb_inv_i;
      s1o_inf         <= ocb_inf_i;
      s1o_snan_i      <= ocb_snan_i;
      s1o_qnan_i      <= ocb_qnan_i;
      s1o_anan_sign_i <= ocb_anan_sign_i;
      // DIV specials
      s1o_div_op        <= mul_rdy_i & div_op_i;
      s1o_div_sign_rmnd <= div_sign_rmnd_i;
      s1o_div_dbz       <= div_dbz_i;
      // I2F specials
      s1o_f2i_ovf <= f2i_ovf_i;
      s1o_f2i     <= f2i_rdy_i;
    end // advance
  end // posedge clock

  // ready is special case
  always @(posedge clk `OR_ASYNC_RST) begin
    if (rst)
      s1o_ready <= 1'b0;
    else if (pipeline_flush_i)
      s1o_ready <= 1'b0;
    else if (s1_adv)
      s1o_ready <= 1'b1;
    else if (padv_wb_i & grant_wb_to_fp32_arith_i)
      s1o_ready <= 1'b0;
  end // posedge clock


  /* Stage #2: rounding */


  wire s2t_dbz  = s1o_div_dbz;

  wire s2t_g    = s1o_fract32[0];
  wire s2t_r    = s1o_rs[1];
  wire s2t_s    = s1o_rs[0];
  wire s2t_lost = s2t_r | s2t_s;

  wire s2t_rnd_up = (rm_nearest & s2t_r & s2t_s) |
                    (rm_nearest & s2t_g & s2t_r & (~s2t_s)) |
                    (rm_to_infp & (~s1o_sign) & s2t_lost) |
                    (rm_to_infm &   s1o_sign  & s2t_lost);

  // IEEE compliance rounding for qutient
  wire s2t_div_rnd_up =
    (rm_nearest & s2t_r & s2t_s & (~s1o_div_sign_rmnd)) |
    ( ((rm_to_infp & (~s1o_sign)) | (rm_to_infm & s1o_sign)) &
      ((s2t_r & s2t_s) | ((~s2t_r) & s2t_s & (~s1o_div_sign_rmnd))) );
  wire s2t_div_rnd_dn = (~s2t_r) & s2t_s & s1o_div_sign_rmnd &
    ( (rm_to_infp &   s1o_sign)  |
      (rm_to_infm & (~s1o_sign)) |
       rm_to_zero );

  // set resulting direction of rounding
  //  a) normalized quotient is rounded by quotient related rules
  //  b) de-normalized quotient is rounded by common rules
  wire s2t_rnd_n_qtnt = s1o_div_op & s1o_fract32[23]; // normalized quotient
  wire s2t_set_rnd_up = s2t_rnd_n_qtnt ? s2t_div_rnd_up : s2t_rnd_up;
  wire s2t_set_rnd_dn = s2t_rnd_n_qtnt ? s2t_div_rnd_dn : 1'b0;

  // define value for rounding adder
  wire [31:0] s2t_rnd_v32 =
    s2t_set_rnd_up ? 32'd1        : // +1
    s2t_set_rnd_dn ? 32'hFFFFFFFF : // -1
                     32'd0;         // no rounding
  // rounded fractional
  wire [31:0] s2t_fract32_rnd = s1o_fract32 + s2t_rnd_v32;


  // floating point output
  wire s2t_f32_shr = s2t_fract32_rnd[24];
  // update exponent and fraction
  wire [9:0]  s2t_f32_exp10   = s1o_exp10 + {9'd0,s2t_f32_shr};
  wire [23:0] s2t_f32_fract24 = s2t_f32_shr ? s2t_fract32_rnd[24:1] :
                                              s2t_fract32_rnd[23:0];
   // denormalized or zero
  wire s2t_f32_fract24_dn = ~s2t_f32_fract24[23];


  // integer output (f2i)
  wire s2t_i32_carry_rnd = s2t_fract32_rnd[31];
  wire s2t_i32_inv = ((~s1o_sign) & s2t_i32_carry_rnd) | s1o_f2i_ovf;
  // two's complement for negative number
  wire [31:0] s2t_i32_int32 = (s2t_fract32_rnd ^ {32{s1o_sign}}) + {31'd0,s1o_sign};
  // zero
  wire s2t_i32_int32_00 = (~s2t_i32_inv) & (~(|s2t_i32_int32));
  // int32 output
  wire [31:0] s2t_i32_opc;
  assign s2t_i32_opc =
    s2t_i32_inv ? (32'h7fffffff ^ {32{s1o_sign}}) : s2t_i32_int32;


   // Generate result and flags
  wire s2t_ine, s2t_ovf, s2t_inf, s2t_unf, s2t_zer;
  wire [31:0] s2t_opc;
  assign {s2t_opc,s2t_ine,s2t_ovf,s2t_inf,s2t_unf,s2t_zer} =
    // f2i
    s1o_f2i ?       //  ine  ovf  inf  unf              zer
      {s2t_i32_opc,s2t_lost,1'b0,1'b0,1'b0,s2t_i32_int32_00} :
    // qnan output
    (s1o_snan_i | s1o_qnan_i) ? // ine  ovf  inf  unf  zer
      {{s1o_anan_sign_i,QNAN},    1'b0,1'b0,1'b0,1'b0,1'b0} :
    // snan output
    s1o_inv ?        // ine  ovf  inf  unf  zer
      {{s1o_sign,SNAN},1'b0,1'b0,1'b0,1'b0,1'b0} :
    // overflow and infinity
    ((s2t_f32_exp10 > 10'd254) | s1o_inf | s2t_dbz) ? // ine                       ovf  inf  unf  zer
      {{s1o_sign,INF},((s2t_lost | (~s1o_inf)) & (~s2t_dbz)),((~s1o_inf) & (~s2t_dbz)),1'b1,1'b0,1'b0} :
    // denormalized or zero
    (s2t_f32_fract24_dn) ?                     // ine  ovf  inf
      {{s1o_sign,8'd0,s2t_f32_fract24[22:0]},s2t_lost,1'b0,1'b0,
                                // unf        zer
       (s2t_lost & s2t_f32_fract24_dn),~(|s2t_f32_fract24)} :
    // normal result                                          ine  ovf  inf  unf  zer
    {{s1o_sign,s2t_f32_exp10[7:0],s2t_f32_fract24[22:0]},s2t_lost,1'b0,1'b0,1'b0,1'b0};


  // EXECUTE level FP32 arithmetic flags
  wire [`OR1K_FPCSR_ALLF_SIZE-1:0] exec_fp32_arith_fpcsr =
    {s2t_dbz, s2t_inf, (s1o_inv | (s2t_i32_inv & s1o_f2i) | s1o_snan_i),
     s2t_ine, s2t_zer, s1o_qnan_i,
     (s1o_inv | (s1o_snan_i & s1o_f2i)), s2t_unf, s2t_ovf} &
    ctrl_fpu_mask_flags_i & {`OR1K_FPCSR_ALLF_SIZE{grant_wb_to_fp32_arith_i}};

  // EXECUTE level FP32 arithmetic exception
  wire exec_except_fp32_arith = except_fpu_enable_i & (|exec_fp32_arith_fpcsr);


  // WB: result and flags
  always @(posedge clk `OR_ASYNC_RST) begin
    if (rst)
      wb_fp32_arith_res_o <= 32'd0;
    else if(padv_wb_i)
      wb_fp32_arith_res_o <= {32{grant_wb_to_fp32_arith_i}} & s2t_opc;
  end // posedge clock

  // WB: exception
  always @(posedge clk `OR_ASYNC_RST) begin
    if (rst) begin
      wb_fp32_arith_fpcsr_o  <= {`OR1K_FPCSR_ALLF_SIZE{1'b0}};
      wb_except_fp32_arith_o <= 1'b0;
    end
    else if(pipeline_flush_i) begin
      wb_fp32_arith_fpcsr_o  <= {`OR1K_FPCSR_ALLF_SIZE{1'b0}};
      wb_except_fp32_arith_o <= 1'b0;
    end
    else if(padv_wb_i) begin
      wb_fp32_arith_fpcsr_o  <= exec_fp32_arith_fpcsr;
      wb_except_fp32_arith_o <= exec_except_fp32_arith;
    end
  end // posedge clock

  // WB: update FPCSR (1-clock to prevent extra writes into FPCSR)
  always @(posedge clk `OR_ASYNC_RST) begin
    if (rst)
      wb_fp32_arith_wb_fpcsr_o <= 1'b0;
    else if (pipeline_flush_i)
      wb_fp32_arith_wb_fpcsr_o <= 1'b0;
    else if (padv_wb_i)
      wb_fp32_arith_wb_fpcsr_o <= grant_wb_to_fp32_arith_i;
    else
      wb_fp32_arith_wb_fpcsr_o <= 1'b0;
  end // @clock

endmodule // pfpu32_rnd_marocchino
