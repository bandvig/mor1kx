/////////////////////////////////////////////////////////////////////
//                                                                 //
//    pfpu_i2f_marocchino                                          //
//    32/64-bit integer to 32/64 floating point converter          //
//    for MAROCCHINO pipeline                                      //
//                                                                 //
//    Author: Andrey Bacherov                                      //
//            avbacherov@opencores.org                             //
//                                                                 //
/////////////////////////////////////////////////////////////////////
//                                                                 //
//   Copyright (C) 2014 - 2016 Andrey Bacherov                     //
//                             avbacherov@opencores.org            //
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

module pfpu_i2f_marocchino
(
  // clocks and resets
  input             clk,
  input             rst,
  // I2F pipe controls
  input             pipeline_flush_i,
  input             start_i,
  output            i2f_taking_op_o,
  output reg        i2f_rdy_o,
  input             rnd_taking_i2f_i,
  // operand for conversion
  input      [63:0] opa_i,
  input             exec_op_fp64_arith_i,
  // ouputs for rounding
  output reg        i2f_sign_o,
  output reg  [3:0] i2f_shr_o,
  output reg [10:0] i2f_exp11shr_o,
  output reg  [5:0] i2f_shl_o,
  output reg [10:0] i2f_exp11shl_o,
  output reg [10:0] i2f_exp11sh0_o,
  output reg [63:0] i2f_fract64_o
);

  /*
     Any stage's output is registered.
     Definitions:
       s??o_name - "S"tage number "??", "O"utput
       s??t_name - "S"tage number "??", "T"emporary (internally)
  */


  // I2F pipe controls
  //  ## per stage busy flags
  wire s1_busy = i2f_rdy_o & ~rnd_taking_i2f_i;
  //  ## per stage advance
  wire s1_adv  = start_i   & ~s1_busy;

  // I2F pipe takes operands for computation
  assign i2f_taking_op_o = s1_adv;


  // signum of input
  wire s1t_signa = opa_i[63];
  // magnitude (tow's complement for negative input)
  wire [63:0] s1t_fract64 = (opa_i ^ {64{s1t_signa}}) + {63'd0,s1t_signa};


  // normalization shifts for double precision
  reg [3:0] s1t_shrx;
  reg [5:0] s1t_shlx;
  // shift goal for double precision case:
  //   52 51                                                 0
  //   |  |                                                  |
  //   h  ffffffffffffffffffffffffffffffffffffffffffffffffffff
  // shift goal for single precision case:
  //                                23 22                    0
  //                                |  |                     |
  //                                h  fffffffffffffffffffffff
  // select bits for right shift computation
  wire [10:0] s1t_fract11 = exec_op_fp64_arith_i ? (s1t_fract64[63:53]) : ({3'd0,s1t_fract64[63:56]});
  // right shift amount computation
  always @(s1t_fract11) begin
    // synthesis parallel_case full_case
    casez(s1t_fract11)
      11'b1??????????:  s1t_shrx = 4'd11; // double precision starts from here
      11'b01?????????:  s1t_shrx = 4'd10;
      11'b001????????:  s1t_shrx = 4'd9;
      11'b0001???????:  s1t_shrx = 4'd8; // single precision starts from here
      11'b00001??????:  s1t_shrx = 4'd7;
      11'b000001?????:  s1t_shrx = 4'd6;
      11'b0000001????:  s1t_shrx = 4'd5;
      11'b00000001???:  s1t_shrx = 4'd4;
      11'b000000001??:  s1t_shrx = 4'd3;
      11'b0000000001?:  s1t_shrx = 4'd2;
      11'b00000000001:  s1t_shrx = 4'd1;
      11'b00000000000:  s1t_shrx = 4'd0;
    endcase
  end
  // select bits for left shift computation
  wire [52:0] s1t_fract53 = exec_op_fp64_arith_i ? (s1t_fract64[52:0]) : ({s1t_fract64[55:32],29'd0});
  // left shift
  always @(s1t_fract53) begin
    // synthesis parallel_case full_case
    casez(s1t_fract53)
      53'b1????????????????????????????????????????????????????:  s1t_shlx = 6'd0; // hidden '1' is in its plase
      53'b01???????????????????????????????????????????????????:  s1t_shlx = 6'd1;
      53'b001??????????????????????????????????????????????????:  s1t_shlx = 6'd2;
      53'b0001?????????????????????????????????????????????????:  s1t_shlx = 6'd3;
      53'b00001????????????????????????????????????????????????:  s1t_shlx = 6'd4;
      53'b000001???????????????????????????????????????????????:  s1t_shlx = 6'd5;
      53'b0000001??????????????????????????????????????????????:  s1t_shlx = 6'd6;
      53'b00000001?????????????????????????????????????????????:  s1t_shlx = 6'd7;
      53'b000000001????????????????????????????????????????????:  s1t_shlx = 6'd8;
      53'b0000000001???????????????????????????????????????????:  s1t_shlx = 6'd9;
      53'b00000000001??????????????????????????????????????????:  s1t_shlx = 6'd10;
      53'b000000000001?????????????????????????????????????????:  s1t_shlx = 6'd11;
      53'b0000000000001????????????????????????????????????????:  s1t_shlx = 6'd12;
      53'b00000000000001???????????????????????????????????????:  s1t_shlx = 6'd13;
      53'b000000000000001??????????????????????????????????????:  s1t_shlx = 6'd14;
      53'b0000000000000001?????????????????????????????????????:  s1t_shlx = 6'd15;
      53'b00000000000000001????????????????????????????????????:  s1t_shlx = 6'd16;
      53'b000000000000000001???????????????????????????????????:  s1t_shlx = 6'd17;
      53'b0000000000000000001??????????????????????????????????:  s1t_shlx = 6'd18;
      53'b00000000000000000001?????????????????????????????????:  s1t_shlx = 6'd19;
      53'b000000000000000000001????????????????????????????????:  s1t_shlx = 6'd20;
      53'b0000000000000000000001???????????????????????????????:  s1t_shlx = 6'd21;
      53'b00000000000000000000001??????????????????????????????:  s1t_shlx = 6'd22;
      53'b000000000000000000000001?????????????????????????????:  s1t_shlx = 6'd23;
      53'b0000000000000000000000001????????????????????????????:  s1t_shlx = 6'd24;
      53'b00000000000000000000000001???????????????????????????:  s1t_shlx = 6'd25;
      53'b000000000000000000000000001??????????????????????????:  s1t_shlx = 6'd26;
      53'b0000000000000000000000000001?????????????????????????:  s1t_shlx = 6'd27;
      53'b00000000000000000000000000001????????????????????????:  s1t_shlx = 6'd28;
      53'b000000000000000000000000000001???????????????????????:  s1t_shlx = 6'd29;
      53'b0000000000000000000000000000001??????????????????????:  s1t_shlx = 6'd30;
      53'b00000000000000000000000000000001?????????????????????:  s1t_shlx = 6'd31;
      53'b000000000000000000000000000000001????????????????????:  s1t_shlx = 6'd32;
      53'b0000000000000000000000000000000001???????????????????:  s1t_shlx = 6'd33;
      53'b00000000000000000000000000000000001??????????????????:  s1t_shlx = 6'd34;
      53'b000000000000000000000000000000000001?????????????????:  s1t_shlx = 6'd35;
      53'b0000000000000000000000000000000000001????????????????:  s1t_shlx = 6'd36;
      53'b00000000000000000000000000000000000001???????????????:  s1t_shlx = 6'd37;
      53'b000000000000000000000000000000000000001??????????????:  s1t_shlx = 6'd38;
      53'b0000000000000000000000000000000000000001?????????????:  s1t_shlx = 6'd39;
      53'b00000000000000000000000000000000000000001????????????:  s1t_shlx = 6'd40;
      53'b000000000000000000000000000000000000000001???????????:  s1t_shlx = 6'd41;
      53'b0000000000000000000000000000000000000000001??????????:  s1t_shlx = 6'd42;
      53'b00000000000000000000000000000000000000000001?????????:  s1t_shlx = 6'd43;
      53'b000000000000000000000000000000000000000000001????????:  s1t_shlx = 6'd44;
      53'b0000000000000000000000000000000000000000000001???????:  s1t_shlx = 6'd45;
      53'b00000000000000000000000000000000000000000000001??????:  s1t_shlx = 6'd46;
      53'b000000000000000000000000000000000000000000000001?????:  s1t_shlx = 6'd47;
      53'b0000000000000000000000000000000000000000000000001????:  s1t_shlx = 6'd48;
      53'b00000000000000000000000000000000000000000000000001???:  s1t_shlx = 6'd49;
      53'b000000000000000000000000000000000000000000000000001??:  s1t_shlx = 6'd50;
      53'b0000000000000000000000000000000000000000000000000001?:  s1t_shlx = 6'd51;
      53'b00000000000000000000000000000000000000000000000000001:  s1t_shlx = 6'd52;
      53'b00000000000000000000000000000000000000000000000000000:  s1t_shlx = 6'd0;
    endcase
  end


  // registering output
  always @(posedge clk) begin
    if (s1_adv) begin
        // computation related
      i2f_sign_o     <= s1t_signa;
      i2f_shr_o      <= s1t_shrx;
      i2f_exp11shr_o <= (exec_op_fp64_arith_i ? 11'd1075 : 11'd150) + {7'd0,s1t_shrx}; // 1075=1023+52, 150=127+23
      i2f_shl_o      <= s1t_shlx;
      i2f_exp11shl_o <= (exec_op_fp64_arith_i ? 11'd1075 : 11'd150) - {5'd0,s1t_shlx};
      i2f_exp11sh0_o <= exec_op_fp64_arith_i ? ({11{s1t_fract64[52]}} & 11'd1075) : // "1" is in [52] / zero
                                               ({11{s1t_fract64[55]}} & 11'd150);
      // for rounding engine we re-pack 32-bits integer to LSBs
      i2f_fract64_o  <= exec_op_fp64_arith_i ? (s1t_fract64) : ({32'd0,s1t_fract64[63:32]});
    end // advance
  end // posedge clock

  // ready is special case
  always @(posedge clk `OR_ASYNC_RST) begin
    if (rst)
      i2f_rdy_o <= 1'b0;
    else if (pipeline_flush_i)
      i2f_rdy_o <= 1'b0;
    else if (s1_adv)
      i2f_rdy_o <= 1'b1;
    else if (rnd_taking_i2f_i)
      i2f_rdy_o <= 1'b0;
  end // posedge clock

endmodule // pfpu_i2f_marocchino
