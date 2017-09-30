////////////////////////////////////////////////////////////////////////
//                                                                    //
//  mor1kx_bus_if_wb32_marocchino                                     //
//                                                                    //
//  Description: mor1kx CPU <-> Wishbone bus bridge                   //
//               with CDC (clock domain crossing)                     //
//                                                                    //
//    (a) Assumes 32-bit data and address.                            //
//    (b) Only CLASSIC and B3_REGISTERED_FEEDBACK modes               //
//        are implemented                                             //
//    (c) Actually, CDC is not implemented completely yet.            //
//        The CPU clock could be greater or equal to Wishbone one,    //
//        buth them must be aligned. So, synchronizers consist of     //
//        single latch named "*_r2". To implement full synchronizers  //
//        latches *_r1 shuld be appropriatelly added.                 //
//    (d) Even with such incomplete CDC implementation, IFETCH's      //
//        ibus_req and LSU's dbus_req behavior has been changed       //
//        from "level" to "toggle" for correct transfering requests   //
//        tightly coupled on high CPU clock. That is why the modole   //
//        designed exclusively for MAROCCHINO                         //
//                                                                    //
////////////////////////////////////////////////////////////////////////
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

module mor1kx_bus_if_wb32_marocchino
#(
  parameter DRIVER_TYPE  = "I_CACHE", // I_CACHE / D_CACHE
  parameter BUS_IF_TYPE  = "B3_REGISTERED_FEEDBACK", // CLASSIC / B3_REGISTERED_FEEDBACK
  parameter BURST_LENGTH = 8
)
(
  // Wishbone side clock and reset
  input             wb_clk,
  input             wb_rst,

  // CPU side clock and reset
  input             cpu_clk,
  input             cpu_rst,

  // CPU side
  output            cpu_err_o,
  output            cpu_ack_o,
  output     [31:0] cpu_dat_o,
  output            cpu_burst_last_o,
  input      [31:0] cpu_adr_i,
  input      [31:0] cpu_dat_i,
  input             cpu_req_i,
  input       [3:0] cpu_bsel_i,
  input             cpu_we_i,
  input             cpu_burst_i,

  // Wishbone side
  output     [31:0] wbm_adr_o,
  output            wbm_stb_o,
  output            wbm_cyc_o,
  output      [3:0] wbm_sel_o,
  output            wbm_we_o,
  output      [2:0] wbm_cti_o,
  output      [1:0] wbm_bte_o,
  output     [31:0] wbm_dat_o,
  input             wbm_err_i,
  input             wbm_ack_i,
  input      [31:0] wbm_dat_i,
  input             wbm_rty_i
);

  generate
  if ((BUS_IF_TYPE != "CLASSIC") && (BUS_IF_TYPE != "B3_REGISTERED_FEEDBACK")) begin
    initial begin
      $display("ERROR: Wishbone bus IF is incorrect");
      $finish;
    end
  end
  endgenerate


  localparam [1:0] BTE_ID = (BURST_LENGTH ==  4) ? 2'b01 :
                            (BURST_LENGTH ==  8) ? 2'b10 :
                            (BURST_LENGTH == 16) ? 2'b11 :
                                                   2'b00; // Linear burst


  //-------------------------//
  // CPU-toggle -> WBM-pulse //
  //-------------------------//

  //  Request access to interconnect
  //  As CDC is not completely implemented (see note (c) at
  //  the begining of the file), each synchronizer
  //  contains only one register. So register enumeration
  //  starts from _r2 (*_r1 should be 1st in full sync. implementation).
  reg   cpu_req_r2;
  reg   cpu_req_r3;
  wire  cpu_req_pulse; // 1-clock in Wishbone clock domain
  // ---
  always @(posedge wb_clk) begin
    if (wb_rst) begin
      cpu_req_r2 <= 1'b0;
      cpu_req_r3 <= 1'b0;
    end
    else begin
      cpu_req_r2 <= cpu_req_i;
      cpu_req_r3 <= cpu_req_r2;
    end
  end // @wb-clock
  // ---
  assign cpu_req_pulse = cpu_req_r2 ^ cpu_req_r3;


  //----------------------------//
  // WBM access with read burst //
  //----------------------------//

  // WB-clock domain registered control signals (to interconnect)
  reg        to_wbm_stb_r;
  reg        to_wbm_cyc_r;
  reg  [3:0] to_wbm_sel_r;
  reg        to_wbm_we_r;
  reg  [2:0] to_wbm_cti_r;
  reg  [1:0] to_wbm_bte_r;

  // WB-clock domain register address (to interconnect)
  reg [31:0] to_wbm_adr_r;
  // initial value for simulation
 `ifndef SYNTHESIS
  // synthesis translate_off
  initial to_wbm_adr_r = 32'd0;
  // synthesis translate_on
 `endif // !synth

  // for burst control
  reg   [(BURST_LENGTH-1):0] burst_done_r;
  wire                [31:0] burst_next_adr;

  // ---
  always @(posedge wb_clk) begin
    if (wb_rst) begin
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
    else if (cpu_req_pulse) begin // start transaction : address and controls
      to_wbm_stb_r <= 1'b1;
      to_wbm_cyc_r <= 1'b1;
      to_wbm_sel_r <= cpu_bsel_i;
      to_wbm_we_r  <= cpu_we_i;
      to_wbm_cti_r <= {1'b0, cpu_burst_i, 1'b0}; // 010 if burst
      to_wbm_bte_r <= cpu_burst_i ? BTE_ID : 2'd0;
      // for burst control
      burst_done_r <= {cpu_burst_i, {(BURST_LENGTH-1){1'b0}}};
    end
  end // @wb-clock


  // WB-clock domain register address (to interconnect)
  // burst length is 8: 32 byte = (8 words x 32 bits/word) -> cache block length is 5
  // burst length is 4: 16 byte = (4 words x 32 bits/word) -> cache block length is 4
  assign burst_next_adr = (BURST_LENGTH == 8) ?
    {to_wbm_adr_r[31:5], to_wbm_adr_r[4:0] + 5'd4} : // 32 byte = (8 words x 32 bits/word)
    {to_wbm_adr_r[31:4], to_wbm_adr_r[3:0] + 4'd4};  // 16 byte = (4 words x 32 bits/word)
  // ---
  always @(posedge wb_clk) begin
    if (to_wbm_cyc_r) begin // wait complete transaction
      if (wbm_ack_i) begin
        // pay attention:
        // as DCACHE is write through, "data" and "we" are irrelevant for read burst
        to_wbm_adr_r <= burst_next_adr;
      end
    end
    else if (cpu_req_pulse) begin // start transaction : address
      to_wbm_adr_r <= cpu_adr_i;
    end
  end // @wb-clock


  // WB-clock domain register data (to interconnect)
  generate
  /* verilator lint_off WIDTH */
  if (DRIVER_TYPE == "I_CACHE") begin : drv_i_cache
  /* verilator lint_on WIDTH */
    assign wbm_dat_o = 32'd0;
  end
  /* verilator lint_off WIDTH */
  else if (DRIVER_TYPE == "D_CACHE") begin: drv_d_cache
  /* verilator lint_on WIDTH */
    reg [31:0] to_wbm_dat_r;
    // initial value for simulation
   `ifndef SYNTHESIS
    // synthesis translate_off
    initial to_wbm_dat_r = 32'd0;
    // synthesis translate_on
   `endif // !synth
    // ---
    always @(posedge wb_clk) begin
      if (cpu_req_pulse) // start transaction : data for d-cache driver
        to_wbm_dat_r <= cpu_dat_i;
    end
    // ---
    assign wbm_dat_o = to_wbm_dat_r;
  end
  else begin : drv_undef
    initial begin
      $display("ERROR: Bridge driver is undefined");
      $finish;
    end
  end
  endgenerate

  // --- to interconnect output assignenment ---
  assign wbm_adr_o = to_wbm_adr_r;
  assign wbm_stb_o = to_wbm_stb_r;
  assign wbm_cyc_o = to_wbm_cyc_r;
  assign wbm_sel_o = to_wbm_sel_r;
  assign wbm_we_o  = to_wbm_we_r;
  assign wbm_cti_o = to_wbm_cti_r;
  assign wbm_bte_o = to_wbm_bte_r;


  //------------------------//
  // WBM-to-CPU burst queue //
  //------------------------//

  // WBM-TO-CPU data layout
  localparam  WBM2CPU_DAT_LSB =  0;
  localparam  WBM2CPU_DAT_MSB = 31;
  localparam  WBM2CPU_LAST    = WBM2CPU_DAT_MSB + 1;
  localparam  WBM2CPU_ACK     = WBM2CPU_LAST    + 1;
  localparam  WBM2CPU_ERR     = WBM2CPU_ACK     + 1;
  // ---
  localparam  WBM2CPU_MSB     = WBM2CPU_ERR;

  // --- registered input data ---
  reg  [WBM2CPU_MSB:0] queue_in_r;
  // ---
  always @(posedge wb_clk) begin
    if (to_wbm_cyc_r & (wbm_err_i | wbm_ack_i))
      queue_in_r <= { wbm_err_i, wbm_ack_i,                // WBM-TO-CPU data layout
                      (to_wbm_cti_r[1] & burst_done_r[0]), // WBM-TO-CPU data layout
                      wbm_dat_i };                         // WBM-TO-CPU data layout
  end // @wb-clock

  // --- signaling to CPU ---
  reg   queue2cpu_rdy_toggle_r;
  // ---
  always @(posedge wb_clk) begin
    if (wb_rst)
      queue2cpu_rdy_toggle_r <= 1'b0;
    else if (queue_in_r[WBM2CPU_ERR])
      queue2cpu_rdy_toggle_r <= queue2cpu_rdy_toggle_r;
    else if (to_wbm_cyc_r & (wbm_err_i | wbm_ack_i))
      queue2cpu_rdy_toggle_r <= ~queue2cpu_rdy_toggle_r;
  end // @wb-clock
  //  As CDC is not completely implemented (see note (c) at
  //  the begining of the file), each synchronizer
  //  contains only one register. So register enumeration
  //  starts from _r2 (*_r1 should be 1st in full sync. implementation).
  reg   queue2cpu_rdy_r2;
  reg   queue2cpu_rdy_r3;
  wire  queue2cpu_rdy_pulse; // 1-clock in Wishbone clock domain
  // ---
  always @(posedge cpu_clk) begin
    if (cpu_rst) begin
      queue2cpu_rdy_r2 <= 1'b0;
      queue2cpu_rdy_r3 <= 1'b0;
    end
    else begin
      queue2cpu_rdy_r2 <= queue2cpu_rdy_toggle_r;
      queue2cpu_rdy_r3 <= queue2cpu_rdy_r2;
    end
  end // @cpu-clock
  // ---
  assign queue2cpu_rdy_pulse = queue2cpu_rdy_r2 ^ queue2cpu_rdy_r3;

  // two stages ACK/ERR buffer
  reg [2:0] queue_ack_err_r2;
  reg [2:0] queue_ack_err_r3;
  // ---
  always @(posedge cpu_clk) begin
    if (cpu_rst)
      queue_ack_err_r2 <= 3'd0;
    else
      queue_ack_err_r2 <= {queue_in_r[WBM2CPU_ERR],queue_in_r[WBM2CPU_ACK],queue_in_r[WBM2CPU_LAST]};
  end // at cpu-clock
  // ---
  always @(posedge cpu_clk) begin
    if (cpu_rst)
      queue_ack_err_r3 <= 3'd0;
    else if (queue2cpu_rdy_pulse)
      queue_ack_err_r3 <= queue_ack_err_r2;
    else // 1-clock
      queue_ack_err_r3 <= 3'd0;
  end // at cpu-clock

  // two stages DATA buffer
  reg [31:0] queue_dat_r2;
  reg [31:0] queue_dat_r3;
  // ---
  always @(posedge cpu_clk) begin
    queue_dat_r2 <= queue_in_r[WBM2CPU_DAT_MSB:WBM2CPU_DAT_LSB];
  end // at cpu-clock
  // ---
  always @(posedge cpu_clk) begin
    if (queue2cpu_rdy_pulse)
      queue_dat_r3 <= queue_dat_r2;
  end // at cpu-clock

  // output assignement
  assign cpu_burst_last_o = queue_ack_err_r3[0];
  assign cpu_ack_o        = queue_ack_err_r3[1];
  assign cpu_err_o        = queue_ack_err_r3[2];
  assign cpu_dat_o        = queue_dat_r3;

endmodule // mor1kx_bus_if_wb32_marocchino
