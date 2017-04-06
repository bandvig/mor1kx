////////////////////////////////////////////////////////////////////////
//                                                                    //
//  mor1kx_bus_if_wb32_cdc                                            //
//                                                                    //
//  Description: mor1kx processor Wishbone bus bridge                 //
//               with CDC (clock domain crossing)                     //
//                                                                    //
//    (a) Derived from Julius Baxter's mor1kx_bus_if_wb32.            //
//        Assumes 32-bit data and address.                            //
//    (b) Only CLASSIC and B3_REGISTERED_FEEDBACK modes               //
//        are implemented                                             //
//    (c) Actually, CDC is not implemented completely yet.            //
//        The CPU and WB clocks must be synchronous. The current      //
//        implementation is intermediate step.                        //
//                                                                    //
////////////////////////////////////////////////////////////////////////
//                                                                    //
//   Copyright (C) 2012 Julius Baxter                                 //
//                      juliusbaxter@gmail.com                        //
//                                                                    //
//   Copyright (C) 2017 Andrey Bacherov                               //
//                      avbacherov@opencores.org                      //
//                                                                    //
//      This Source Code Form is subject to the terms of the          //
//      Open Hardware Description License, v. 1.0. If a copy          //
//      of the OHDL was not distributed with this file, You           //
//      can obtain one at http://juliusbaxter.net/ohdl/ohdl.txt       //
//                                                                    //
////////////////////////////////////////////////////////////////////////

`include "mor1kx-defines.v"

module mor1kx_bus_if_wb32_cdc
#(
  parameter BUS_IF_TYPE  = "CLASSIC", // CLASSIC | B3_REGISTERED_FEEDBACK
  parameter BURST_LENGTH = 8
)
(
  // Wishbone side clock and reset
  input         wb_clk,
  input         wb_rst,

  // CPU side clock and reset: MAROCCHINO_TODO: implement it
  //input         cpu_clk,
  //output        cpu_rst,

  // CPU side
  output        cpu_err_o,
  output        cpu_ack_o,
  output [31:0] cpu_dat_o,
  output [31:0] cpu_burst_adr_o,
  output        cpu_burst_last_o,
  input  [31:0] cpu_adr_i,
  input  [31:0] cpu_dat_i,
  input         cpu_req_i,
  input   [3:0] cpu_bsel_i,
  input         cpu_we_i,
  input         cpu_burst_i,

  // Wishbone side
  output [31:0] wbm_adr_o,
  output        wbm_stb_o,
  output        wbm_cyc_o,
  output  [3:0] wbm_sel_o,
  output        wbm_we_o,
  output  [2:0] wbm_cti_o,
  output  [1:0] wbm_bte_o,
  output [31:0] wbm_dat_o,
  input         wbm_err_i,
  input         wbm_ack_i,
  input  [31:0] wbm_dat_i,
  input         wbm_rty_i
);

  localparam [1:0] BTE_ID = (BURST_LENGTH ==  4) ? 2'b01 :
                            (BURST_LENGTH ==  8) ? 2'b10 :
                            (BURST_LENGTH == 16) ? 2'b11 :
                                                   2'b00; // Linear burst

  generate
  if ((BUS_IF_TYPE == "CLASSIC") || (BUS_IF_TYPE == "B3_REGISTERED_FEEDBACK")) begin
    initial $display("%m: Wishbone bus IF is %s", BUS_IF_TYPE);
  end
  else begin
    initial begin
      $display("ERROR: Wishbone bus IF is incorrect");
      $finish;
    end
  end
  endgenerate


  // level -> pulse former or request access to intercommenct
  reg   req_wbm_acc_r1;
  reg   req_wbm_acc_r2;
  wire  req_wbm_acc_pulse; // 1-clock in Wishbone clock domain
  // ---
  always @(posedge wb_clk) begin
    if (wb_rst) begin
      req_wbm_acc_r1 <= 1'b0;
      req_wbm_acc_r2 <= 1'b0;
    end
    else begin
      req_wbm_acc_r1 <= cpu_req_i;
      req_wbm_acc_r2 <= req_wbm_acc_r1;
    end
  end // @wb-clock
  // ---
  assign req_wbm_acc_pulse = req_wbm_acc_r1 & (~req_wbm_acc_r2);

  // for burst control
  reg   [(BURST_LENGTH-1):0] burst_done_r;
  wire                [31:0] burst_next_adr;
  // WB-clock domain registered signals (to interconnect)
  reg [31:0] to_wbm_adr_r;
  reg [31:0] to_wbm_dat_r;
  reg        to_wbm_stb_r;
  reg        to_wbm_cyc_r;
  reg  [3:0] to_wbm_sel_r;
  reg        to_wbm_we_r;
  reg  [2:0] to_wbm_cti_r;
  reg  [1:0] to_wbm_bte_r;
  // ---
  // burst length is 8: 32 byte = (8 words x 32 bits/word) -> cache block length is 5
  // burst length is 4: 16 byte = (4 words x 32 bits/word) -> cache block length is 4
  assign burst_next_adr = (BURST_LENGTH == 8) ?
    {to_wbm_adr_r[31:5], to_wbm_adr_r[4:0] + 5'd4} : // 32 byte = (8 words x 32 bits/word)
    {to_wbm_adr_r[31:4], to_wbm_adr_r[3:0] + 4'd4};  // 16 byte = (4 words x 32 bits/word)
  // ---
  always @(posedge wb_clk) begin
    if (wb_rst) begin
      to_wbm_adr_r <= 32'd0;
      to_wbm_dat_r <= 32'd0;
      to_wbm_stb_r <=  1'b0;
      to_wbm_cyc_r <=  1'b0;
      to_wbm_sel_r <=  4'd0;
      to_wbm_we_r  <=  1'b0;
      to_wbm_cti_r <=  3'd0;
      to_wbm_bte_r <=  2'd0;
      // for burst control
      burst_done_r <= {BURST_LENGTH{1'b0}};
    end
    else if (to_wbm_cyc_r) begin // wait complete transaction
      if (wbm_err_i) begin // pipe flushed or transaction done or error
        to_wbm_adr_r <= 32'd0;
        to_wbm_dat_r <= 32'd0;
        to_wbm_stb_r <=  1'b0;
        to_wbm_cyc_r <=  1'b0;
        to_wbm_sel_r <=  4'd0;
        to_wbm_we_r  <=  1'b0;
        to_wbm_cti_r <=  3'd0;
        to_wbm_bte_r <=  2'd0;
        // for burst control
        burst_done_r <= {BURST_LENGTH{1'b0}};
      end
      else if (wbm_ack_i) begin
        // pay attention:
        // as DCACHE is write through, "data" and "we" are irrelevant for read burst
        to_wbm_adr_r <= (to_wbm_cti_r[1] & (~burst_done_r[0])) ? burst_next_adr : 32'd0;
        to_wbm_dat_r <= 32'd0;
        to_wbm_stb_r <= (to_wbm_cti_r[1] & (~burst_done_r[0]));
        to_wbm_cyc_r <= (to_wbm_cti_r[1] & (~burst_done_r[0]));
        to_wbm_sel_r <= (to_wbm_cti_r[1] & (~burst_done_r[0])) ? to_wbm_sel_r : 4'd0;
        to_wbm_we_r  <=  1'b0;
        to_wbm_cti_r <= (to_wbm_cti_r[1] ? (burst_done_r[1] ? 3'b111 : 3'b010) : 3'b000);
        to_wbm_bte_r <= (to_wbm_cti_r[1] & (~burst_done_r[0])) ? to_wbm_bte_r : 2'd0;
        // for burst control
        burst_done_r <= {1'b0, burst_done_r[(BURST_LENGTH-1):1]};
      end
    end
    else if (req_wbm_acc_pulse) begin // start transaction
      to_wbm_adr_r <= cpu_adr_i;
      to_wbm_dat_r <= cpu_dat_i;
      to_wbm_stb_r <= 1'b1;
      to_wbm_cyc_r <= 1'b1;
      to_wbm_sel_r <= cpu_bsel_i;
      to_wbm_we_r  <= cpu_we_i;
      to_wbm_cti_r <= {1'b0, cpu_burst_i, 1'b0}; // 010 if burst
      to_wbm_bte_r <= cpu_burst_i ? BTE_ID : 2'd0;
      // for burst control
      burst_done_r <= {cpu_burst_i, {(BURST_LENGTH-1){1'b0}}};
    end
  end

  // To interconnect output assignenment
  assign wbm_adr_o = to_wbm_adr_r;
  assign wbm_dat_o = to_wbm_dat_r;
  assign wbm_stb_o = to_wbm_stb_r;
  assign wbm_cyc_o = to_wbm_cyc_r;
  assign wbm_sel_o = to_wbm_sel_r;
  assign wbm_we_o  = to_wbm_we_r;
  assign wbm_cti_o = to_wbm_cti_r;
  assign wbm_bte_o = to_wbm_bte_r;



  // CPU-clock domain registered signals (to CPU)
  reg [31:0] to_cpu_dat_r;
  reg [31:0] to_cpu_burst_adr_r;
  reg        to_cpu_burst_last_r;
  reg        to_cpu_ack_r;
  reg        to_cpu_err_r;
  // ---
  always @(posedge wb_clk) begin
    if (wb_rst) begin
      to_cpu_dat_r        <= 32'd0;
      to_cpu_burst_adr_r  <= 32'd0;
      to_cpu_burst_last_r <=  1'b0;
      to_cpu_ack_r        <=  1'b0;
      to_cpu_err_r        <=  1'b0;
    end
    else if (to_wbm_cyc_r & (wbm_ack_i | wbm_err_i)) begin // wait complete transaction
      to_cpu_dat_r        <= wbm_dat_i;
      to_cpu_burst_adr_r  <= to_wbm_adr_r;
      to_cpu_burst_last_r <= to_wbm_cti_r[1] & burst_done_r[0];
      to_cpu_ack_r        <= wbm_ack_i;
      to_cpu_err_r        <= wbm_err_i;
    end
    else begin
      to_cpu_dat_r        <= 32'd0;
      to_cpu_burst_adr_r  <= 32'd0;
      to_cpu_burst_last_r <=  1'b0;
      to_cpu_ack_r        <=  1'b0;
      to_cpu_err_r        <=  1'b0;
    end
  end

  // To CPU output assignenment
  assign cpu_dat_o        = to_cpu_dat_r;
  assign cpu_burst_adr_o  = to_cpu_burst_adr_r;
  assign cpu_burst_last_o = to_cpu_burst_last_r;
  assign cpu_ack_o        = to_cpu_ack_r;
  assign cpu_err_o        = to_cpu_err_r;

endmodule // mor1kx_bus_if_wb_cdc
