////////////////////////////////////////////////////////////////////////
//                                                                    //
//  mor1kx_bus_if_wb32_marocchino                                     //
//                                                                    //
//  Description: mor1kx CPU <-> Wishbone IBUS/DBUS bridges            //
//               with Pseudo CDC (clock domain crossing)              //
//                                                                    //
//    (a) Assumes 32-bit data and address.                            //
//    (b) Only CLASSIC and B3_REGISTERED_FEEDBACK modes               //
//        are implemented                                             //
//    (c) Pseudo CDC disclaimer:                                      //
//        As positive edges of wb-clock and cpu-clock assumed be      //
//        aligned, we use simplest clock domain pseudo-synchronizers. //
//    (d) Also atomic reservation implemeted here in Wishbone         //
//        clock domain                                                //
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
  //
  // Pseudo CDC disclaimer:
  // As positive edges of wb-clock and cpu-clock assumed be aligned,
  // we use simplest clock domain pseudo-synchronizers.
  //
  reg   cpu_req_r1;
  wire  cpu_req_pulse; // 1-clock in Wishbone clock domain
  // ---
  always @(posedge wb_clk) begin
    if (wb_rst)
      cpu_req_r1 <= 1'b0;
    else
      cpu_req_r1 <= cpu_req_i;
  end // @wb-clock
  // ---
  assign cpu_req_pulse = cpu_req_r1 ^ cpu_req_i;


  //----------------------------//
  // WBM access with read burst //
  //----------------------------//

  // WB-clock domain registered control signals (to interconnect)
  reg        to_wbm_stb_r;
  reg        to_wbm_cyc_r;
  reg  [2:0] to_wbm_cti_r;
  reg  [1:0] to_wbm_bte_r;

  // WB-clock domain register address (to interconnect)
  reg [31:0] to_wbm_adr_r;

  // for burst control
  reg   [(BURST_LENGTH-1):0] burst_done_r;
  wire                       burst_proc;
  wire                       burst_keep;
  wire                [31:0] burst_next_adr;

  // continue busrting
  assign burst_proc = to_wbm_cti_r[1];
  assign burst_keep = burst_proc & (~burst_done_r[0]);

  // ---
  always @(posedge wb_clk) begin
    if (wb_rst) begin
      to_wbm_stb_r <=  1'b0;
      to_wbm_cyc_r <=  1'b0;
      to_wbm_cti_r <=  3'd0;
      to_wbm_bte_r <=  2'd0;
      // for burst control
      burst_done_r <= {BURST_LENGTH{1'b0}};
    end
    else if (to_wbm_cyc_r) begin
      if (wbm_err_i) begin
        to_wbm_stb_r <=  1'b0;
        to_wbm_cyc_r <=  1'b0;
        to_wbm_cti_r <=  3'd0;
        to_wbm_bte_r <=  2'd0;
        // for burst control
        burst_done_r <= {BURST_LENGTH{1'b0}};
      end
      else if (wbm_ack_i) begin
        to_wbm_stb_r <= burst_keep;
        to_wbm_cyc_r <= burst_keep;
        to_wbm_cti_r <= (burst_proc ? (burst_done_r[1] ? 3'b111 : 3'b010) : 3'b000);
        to_wbm_bte_r <= burst_keep ? to_wbm_bte_r : 2'd0;
        // for burst control
        burst_done_r <= {1'b0, burst_done_r[(BURST_LENGTH-1):1]};
      end
    end
    else if (cpu_req_pulse) begin // a bridge latches address and controls
      to_wbm_stb_r <= 1'b1;
      to_wbm_cyc_r <= 1'b1;
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
      if (wbm_ack_i & burst_keep) begin // next burst address to WB
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

    // Constant values for IBUS
    assign wbm_dat_o = 32'd0;
    assign wbm_sel_o =  4'hf;
    assign wbm_we_o  =  1'b0;

  end
  /* verilator lint_off WIDTH */
  else if (DRIVER_TYPE == "D_CACHE") begin: drv_d_cache
  /* verilator lint_on WIDTH */

    // Data for write
    reg [31:0] to_wbm_dat_r;
    // ---
    always @(posedge wb_clk) begin
      if (cpu_req_pulse) // d-cache bridge latches data
        to_wbm_dat_r <= cpu_dat_i;
    end
    // ---
    assign wbm_dat_o = to_wbm_dat_r;

    // Byte select and write enable
    reg  [3:0] to_wbm_sel_r;
    reg        to_wbm_we_r;
    // ---
    always @(posedge wb_clk) begin
      if (wb_rst) begin
        to_wbm_sel_r <=  4'd0;
        to_wbm_we_r  <=  1'b0;
      end
      else if (to_wbm_cyc_r) begin
        if (wbm_err_i) begin
          to_wbm_sel_r <=  4'd0;
          to_wbm_we_r  <=  1'b0;
        end
        else if (wbm_ack_i) begin
          to_wbm_sel_r <= burst_keep ? to_wbm_sel_r : 4'd0;
          to_wbm_we_r  <= 1'b0; // DCACHE is write through: no write bursting
        end
      end
      else if (cpu_req_pulse) begin // d-cache bridge latches byte select and write enable
        to_wbm_sel_r <= cpu_bsel_i;
        to_wbm_we_r  <= cpu_we_i;
      end
    end // @wb-clock
    // ---
    assign wbm_sel_o = to_wbm_sel_r;
    assign wbm_we_o  = to_wbm_we_r;

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
      queue_in_r <= { wbm_err_i, wbm_ack_i,           // WBM-TO-CPU data layout
                      (burst_proc & burst_done_r[0]), // WBM-TO-CPU data layout
                      wbm_dat_i };                    // WBM-TO-CPU data layout
  end // @wb-clock

  // --- signaling to CPU ---
  reg   queue2cpu_rdy_toggle_r;
  // ---
  always @(posedge wb_clk) begin
    if (wb_rst)
      queue2cpu_rdy_toggle_r <= 1'b0;
    else if (to_wbm_cyc_r & (wbm_err_i | wbm_ack_i))
      queue2cpu_rdy_toggle_r <= ~queue2cpu_rdy_toggle_r;
  end // @wb-clock
  //
  // Pseudo CDC disclaimer:
  // As positive edges of wb-clock and cpu-clock assumed be aligned,
  // we use simplest clock domain pseudo-synchronizers.
  //
  reg   queue2cpu_rdy_r1;
  wire  queue2cpu_rdy_pulse; // from toggle to posedge of CPU clock
  // ---
  always @(posedge cpu_clk) begin
    if (cpu_rst)
      queue2cpu_rdy_r1 <= 1'b0;
    else
      queue2cpu_rdy_r1 <= queue2cpu_rdy_toggle_r;
  end // @cpu-clock
  // ---
  assign queue2cpu_rdy_pulse = queue2cpu_rdy_toggle_r ^ queue2cpu_rdy_r1;

  // ACK/ERR latches (with reset control)
  reg [2:0] queue_ack_err_r1;
  // ---
  always @(posedge cpu_clk) begin
    if (cpu_rst)
      queue_ack_err_r1 <= 3'd0;
    else if (queue2cpu_rdy_pulse)
      queue_ack_err_r1 <= {queue_in_r[WBM2CPU_ERR],queue_in_r[WBM2CPU_ACK],queue_in_r[WBM2CPU_LAST]};
    else // 1-clock
      queue_ack_err_r1 <= 3'd0;
  end // at cpu-clock

  // DATA latches (without reset control)
  reg [31:0] queue_dat_r1;
  // ---
  always @(posedge cpu_clk) begin
    if (queue2cpu_rdy_pulse)
      queue_dat_r1 <= queue_in_r[WBM2CPU_DAT_MSB:WBM2CPU_DAT_LSB];
  end // at cpu-clock

  // output assignement
  assign cpu_burst_last_o = queue_ack_err_r1[0];
  assign cpu_ack_o        = queue_ack_err_r1[1];
  assign cpu_err_o        = queue_ack_err_r1[2];
  assign cpu_dat_o        = queue_dat_r1;

endmodule // mor1kx_bus_if_wb32_marocchino
