/////////////////////////////////////////////////////////////////////
////                                                             ////
////  pfpu32_f2i_marocchino                                      ////
////  32-bit floating point to integer converter                 ////
////  for MAROCCHINO pipeline                                    ////
////                                                             ////
////  Author: Andrey Bacherov                                    ////
////          avbacherov@opencores.org                           ////
////                                                             ////
/////////////////////////////////////////////////////////////////////
////                                                             ////
//// Copyright (C) 2014 - 2016 Andrey Bacherov                   ////
////                           avbacherov@opencores.org          ////
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

module pfpu32_f2i_marocchino
(
  // clocks and resets
  input             clk,
  input             rst,
  // pipe controls
  input             pipeline_flush_i,
  input             start_i,
  output            f2i_busy_o,
  output            f2i_takes_op_o,
  output reg        f2i_rdy_o,
  input             rnd_takes_f2i_i,
  // input data
  input             signa_i,
  input       [9:0] exp10a_i,
  input      [23:0] fract24a_i,
  input             snan_i,
  input             qnan_i,
  // output data for rounding
  output reg        f2i_sign_o,
  output reg [23:0] f2i_int24_o,
  output reg  [4:0] f2i_shr_o,
  output reg  [3:0] f2i_shl_o,
  output reg        f2i_ovf_o,
  output reg        f2i_snan_o
);

  /*
     Any stage's output is registered.
     Definitions:
       s??o_name - "S"tage number "??", "O"utput
       s??t_name - "S"tage number "??", "T"emporary (internally)
  */


  // F2I pipe controls
  //  ## per stage busy flags
  wire s1_busy = f2i_rdy_o & ~rnd_takes_f2i_i;
  //  ## per stage advance
  wire s1_adv  = start_i   & ~s1_busy;

  // F2I pipe is busy
  assign f2i_busy_o = s1_busy;

  // F2I pipe takes operands for computation
  assign f2i_takes_op_o = s1_adv;


  // exponent after moving binary point at the end of mantissa
  // bias is also removed
  wire [9:0] s1t_exp10m = exp10a_i - 10'd150; // (- 127 - 23)

  // detect if now shift right is required
  wire [9:0] s1t_shr_t = {10{s1t_exp10m[9]}} & (10'd150 - exp10a_i);
  // limit right shift by 31
  wire [4:0] s1t_shr = s1t_shr_t[4:0] | {5{|s1t_shr_t[9:5]}};

  // detect if left shift required for mantissa
  // (limited by 15)
  wire [3:0] s1t_shl = {4{~s1t_exp10m[9]}} & (s1t_exp10m[3:0] | {4{|s1t_exp10m[9:4]}});
  // check overflow
  wire s1t_is_shl_gt8 = s1t_shl[3] & (|s1t_shl[2:0]);
  wire s1t_is_shl_eq8 = s1t_shl[3] & (~(|s1t_shl[2:0]));
  wire s1t_is_shl_ovf =
     s1t_is_shl_gt8 |
    (s1t_is_shl_eq8 & (~signa_i)) |
    (s1t_is_shl_eq8 &   signa_i & (|fract24a_i[22:0]));


  // registering output
  always @(posedge clk) begin
    if (s1_adv) begin
        // input related
      f2i_snan_o  <= snan_i;
        // computation related
      f2i_sign_o  <= signa_i & (!(qnan_i | snan_i)); // if 'a' is a NaN than ouput is max. positive
      f2i_int24_o <= fract24a_i;
      f2i_shr_o   <= s1t_shr;
      f2i_shl_o   <= s1t_shl;
      f2i_ovf_o   <= s1t_is_shl_ovf;
    end // (reset or flush) / advance
  end // posedge clock

  // ready is special case
  always @(posedge clk `OR_ASYNC_RST) begin
    if (rst)
      f2i_rdy_o <= 1'b0;
    else if (pipeline_flush_i)
      f2i_rdy_o <= 1'b0;
    else if (s1_adv)
      f2i_rdy_o <= 1'b1;
    else if (rnd_takes_f2i_i)
      f2i_rdy_o <= 1'b0;
  end // posedge clock

endmodule // pfpu32_f2i_marocchino
