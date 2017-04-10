////////////////////////////////////////////////////////////////////////
//                                                                    //
//  mor1kx_ticktimer_marocchino                                       //
//                                                                    //
//  Description:                                                      //
//    - Mor1kx tick timer unit decoupled from CTRL module.            //
//    - Derived from mor1kx_ticktimer originally designed by          //
//      Julius Baxter.                                                //
//    - It is able to operate either in CPU's clock domain or in      //
//      Wishbone BUS clock domain. Clock domain selection is          //
//      controlled by TIMER_CLOCK_DOMAIN parameter.                   //
//      Wishbone BUS clock domain is useful for bacward compatibility //
//      with already compiled applications and toolchains.            //
//                                                                    //
////////////////////////////////////////////////////////////////////////
//                                                                    //
//   Copyright (C) 2012 Julius Baxter                                 //
//                      juliusbaxter@gmail.com                        //
//                                                                    //
//   Copyright (C) 2016 Andrey Bacherov                               //
//                      avbacherov@opencores.org                      //
//                                                                    //
//      This Source Code Form is subject to the terms of the          //
//      Open Hardware Description License, v. 1.0. If a copy          //
//      of the OHDL was not distributed with this file, You           //
//      can obtain one at http://juliusbaxter.net/ohdl/ohdl.txt       //
//                                                                    //
////////////////////////////////////////////////////////////////////////

`include "mor1kx-defines.v"

module mor1kx_ticktimer_marocchino
#(
  parameter TIMER_CLOCK_DOMAIN = "CPU_CLOCK" // "WB_CLOCK" / "CPU_CLOCK" (default)
)
(
  // Wishbone side clock and reset
  input             wb_clk,
  input             wb_rst,

  // CPU side clock and reset
  input             cpu_clk,
  input             cpu_rst,

  // ready flag
  output            tt_rdy_o,

  // SPR interface
  input      [15:0] spr_bus_addr_i,
  input             spr_bus_we_i,
  input             spr_bus_stb_i,
  input      [31:0] spr_bus_dat_i,
  output reg [31:0] spr_bus_dat_tt_o,
  output reg        spr_bus_ack_tt_o
);

  // Timer's clock and reset
  wire       tt_clk;
  wire       tt_rst;

  // Timer's registers
  reg [31:0] spr_ttmr;
  reg [31:0] spr_ttcr;



  // SPR request decoders
  wire spr_tt_cs   = spr_bus_stb_i & (`SPR_BASE(spr_bus_addr_i) == `OR1K_SPR_TT_BASE);
  wire spr_ttmr_cs = (`SPR_OFFSET(spr_bus_addr_i) == `SPR_OFFSET(`OR1K_SPR_TTMR_ADDR));
  wire spr_ttcr_cs = (`SPR_OFFSET(spr_bus_addr_i) == `SPR_OFFSET(`OR1K_SPR_TTCR_ADDR));

  // pulse to initiate read/write transaction
  reg  spr_tt_cs_r;
  // ---
  always @(posedge cpu_clk) begin
    if (cpu_rst)
      spr_tt_cs_r <= 1'b0;
    else
      spr_tt_cs_r <= spr_tt_cs;
  end
  // ---
  wire spr_tt_cs_pulse = spr_tt_cs & (~spr_tt_cs_r);



  // CPU clock domain:
  wire        tt2cpu_ack;
  wire [31:0] tt2cpu_dat;

  // Timer's clock domain:
  //  # synched SPR access
  wire        cpu2tt_cs_pulse;
  wire        cpu2tt_ttmr_cs, cpu2tt_ttcr_cs, cpu2tt_we;
  wire [31:0] cpu2tt_dat;
  //  # read/write control
  reg         ttmr_cs_r, ttcr_cs_r, tt_we_r, tt_ack_r;
  

  generate
  /* verilator lint_off WIDTH */
  if (TIMER_CLOCK_DOMAIN == "WB_CLOCK") begin: tt_wb_domain
  /* verilator lint_on WIDTH */

    // clock and reset
    assign tt_clk = wb_clk;
    assign tt_rst = wb_rst;

    // CPU-toggle -> TT-pulse to lock SPR bus
    reg cpu2tt_toggle_r;
    // ---
    always @(posedge cpu_clk) begin
      if (cpu_rst)
        cpu2tt_toggle_r <= 1'b0;
      else if (spr_tt_cs_pulse)
        cpu2tt_toggle_r <= ~cpu2tt_toggle_r;
    end
    // --- synch. ---
    // As CDC is not completely implemented, that's why each synchronizer
    // contains only one register. So register enumeration
    // starts from _r2 (*_r1 should be 1st in full sync. implementation).
    reg cpu2tt_cs_r2, cpu2tt_cs_r3;
    //---
    always @(posedge wb_clk) begin
      if (wb_rst) begin
        cpu2tt_cs_r2 <= 1'b0;
        cpu2tt_cs_r3 <= 1'b0;
      end
      else begin
        cpu2tt_cs_r2 <= cpu2tt_toggle_r;
        cpu2tt_cs_r3 <= cpu2tt_cs_r2;
      end
    end
    // ---
    wire cpu2tt_cs_take = cpu2tt_cs_r2 ^ cpu2tt_cs_r3;
    // ---
    reg [34:0] cpu2tt_cmd_r; // {"ttmr-cs", "ttcr-cs", "we", dat}
    // ---
    always @(posedge wb_clk) begin
      if (wb_rst | tt_ack_r)
        cpu2tt_cmd_r <= 35'd0;
      else if (cpu2tt_cs_take)
        cpu2tt_cmd_r <= {spr_ttmr_cs, spr_ttcr_cs, spr_bus_we_i, spr_bus_dat_i};
    end
    // ---
    assign cpu2tt_ttmr_cs = cpu2tt_cmd_r[34];
    assign cpu2tt_ttcr_cs = cpu2tt_cmd_r[33];
    assign cpu2tt_we      = cpu2tt_cmd_r[32];
    assign cpu2tt_dat     = cpu2tt_cmd_r[31:0];
    // ---
    reg cpu2tt_cs_pulse_r;
    // ---
    always @(posedge wb_clk) begin
      if (wb_rst)
        cpu2tt_cs_pulse_r <= 1'b0;
      else
        cpu2tt_cs_pulse_r <= cpu2tt_cs_take;
    end
    // ---
    assign cpu2tt_cs_pulse = cpu2tt_cs_pulse_r;


    // snap data for output regardless of "we"
    reg [31:0] tt2cpu_dat_r;
    // ---
    always @(posedge wb_clk) begin
      if (cpu2tt_cs_pulse) begin
        tt2cpu_dat_r <= cpu2tt_ttmr_cs ? spr_ttmr :
                        cpu2tt_ttcr_cs ? spr_ttcr :
                                         32'd0;
      end
    end
    // --- 
    assign tt2cpu_dat = tt2cpu_dat_r;
    

    // TT-ack-pulse -> CPU-ack-pulse
    //   As CPU clock assumed to be faster or equal to TT's one, we
    // don't use toggle here.
    //   As CDC is not completely implemented, that's why each synchronizer
    // contains only one register. So register enumeration
    // starts from _r2 (*_r1 should be 1st in full sync. implementation).
    reg tt2cpu_ack_r2;
    reg tt2cpu_ack_r3;
    // ---
    always @(posedge cpu_clk) begin
      if (cpu_rst) begin
        tt2cpu_ack_r2 <= 1'b0;
        tt2cpu_ack_r3 <= 1'b0;
      end
      else begin
        tt2cpu_ack_r2 <= tt_ack_r;
        tt2cpu_ack_r3 <= tt2cpu_ack_r2;
      end
    end
    // ---
    assign tt2cpu_ack = tt2cpu_ack_r2 & (~tt2cpu_ack_r3);


    // TT-interrupt-level -> CPU-interrupt-level
    //   As CPU clock assumed to be faster or equal to TT's one, we
    // don't use toggle here.
    //   As CDC is not completely implemented, that's why each synchronizer
    // contains only one register. So register enumeration
    // starts from _r2 (*_r1 should be 1st in full sync. implementation).
    reg tt_rdy_r2;
    // ---
    always @(posedge cpu_clk) begin
      if (cpu_rst)
        tt_rdy_r2 <= 1'b0;
      else if (tt2cpu_ack & spr_ttmr_cs & spr_bus_we_i)
        tt_rdy_r2 <= spr_bus_dat_i[28];
      else
        tt_rdy_r2 <= spr_ttmr[28];
    end    
    // ---
    assign tt_rdy_o = tt_rdy_r2;

  end
  else begin : tt_cpu_domain

    // clock and reset
    assign tt_clk = cpu_clk;
    assign tt_rst = cpu_rst;

    // "cs" and "we"
    assign cpu2tt_cs_pulse = spr_tt_cs_pulse;
    assign cpu2tt_ttmr_cs  = spr_ttmr_cs;
    assign cpu2tt_ttcr_cs  = spr_ttcr_cs;
    assign cpu2tt_we       = spr_bus_we_i;

    // input data from SPR
    assign cpu2tt_dat = spr_bus_dat_i;

    // output ACK
    assign tt2cpu_ack = tt_ack_r;

    // output data for SPR BUS
    assign tt2cpu_dat = spr_ttmr_cs ? spr_ttmr :
                        spr_ttcr_cs ? spr_ttcr :
                                      32'd0;

    // interrupt flag
    assign tt_rdy_o = spr_ttmr[28];

  end
  endgenerate


  // SPR BUS: output data and ack (CPU clock domain)
  always @(posedge cpu_clk) begin
    if (cpu_rst | spr_bus_ack_tt_o) begin
      spr_bus_ack_tt_o <=  1'b0;
      spr_bus_dat_tt_o <= 32'd0;
    end
    else if (tt2cpu_ack) begin
      spr_bus_ack_tt_o <= 1'b1;
      spr_bus_dat_tt_o <= tt2cpu_dat;
    end
  end


  // Read/Write contol 
  always @(posedge tt_clk) begin
    if (tt_rst | tt_ack_r) begin
      ttmr_cs_r <= 1'b0;
      ttcr_cs_r <= 1'b0;
      tt_we_r   <= 1'b0;
      tt_ack_r  <= 1'b0;
    end
    else if (cpu2tt_cs_pulse) begin
      ttmr_cs_r <= cpu2tt_ttmr_cs;
      ttcr_cs_r <= cpu2tt_ttcr_cs;
      tt_we_r   <= cpu2tt_we;
      tt_ack_r  <= 1'b1;
    end
  end // at clock


  // Timer
  wire ttcr_match = (spr_ttcr[27:0] == spr_ttmr[27:0]);

  // Timer SPR control
  always @(posedge tt_clk) begin
    if (tt_rst)
      spr_ttmr <= 1'b0;
    else if (ttmr_cs_r & tt_we_r)
      spr_ttmr <= cpu2tt_dat;
    else if (ttcr_match & spr_ttmr[29])
      spr_ttmr[28] <= 1'b1; // Generate interrupt
  end

  // Modes (spr_ttmr[31:30]):
  // 00 Tick timer is disabled.
  // 01 Timer is restarted on ttcr_match.
  // 10 Timer stops when ttcr_match is true.
  // 11 Timer does not stop when ttcr_match is true
  wire ttcr_clear = ((spr_ttmr[31:30] == 2'b01) &  ttcr_match);
  wire ttcr_run   = ((spr_ttmr[31:30] != 2'b00) & ~ttcr_match) |
                     (spr_ttmr[31:30] == 2'b11);

  always @(posedge tt_clk) begin
    if (tt_rst)
      spr_ttcr <= 32'd0;
    else if (ttcr_cs_r & tt_we_r)
      spr_ttcr <= cpu2tt_dat;
    else if (ttcr_clear)
      spr_ttcr <= 32'd0;
    else if (ttcr_run)
      spr_ttcr <= spr_ttcr + 1'b1;
  end

endmodule // mor1kx_ticktimer_marocchino
