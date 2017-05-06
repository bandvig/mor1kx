////////////////////////////////////////////////////////////////////////
//                                                                    //
//  mor1kx_ticktimer_marocchino                                       //
//                                                                    //
//  Description:                                                      //
//    - Mor1kx tick timer unit decoupled from CTRL module.            //
//    - Derived from mor1kx_ticktimer originally designed by          //
//      Julius Baxter.                                                //
//    - It is operate  in Wishbone BUS clock domain.                  //
//      Wishbone BUS clock domain is useful for bacward compatibility //
//      with already compiled applications and toolchains.            //
//    - Actually, CDC is not implemented completely yet.              //
//      The CPU clock could be greater or equal to Wishbone one,      //
//      buth them must be aligned. So, synchronizers consist of       //
//      single latch named "*_r2". To implement full synchronizers    //
//      latches *_r1 shuld be appropriatelly added.                   //
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

module mor1kx_ticktimer_marocchino
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

  // Timer's registers
  reg [31:0] spr_ttmr;
  reg [31:0] spr_ttcr;


  // SPR request decoders
  wire spr_tt_cs   = spr_bus_stb_i & (`SPR_BASE(spr_bus_addr_i) == `OR1K_SPR_TT_BASE);
  wire spr_ttmr_cs = (`SPR_OFFSET(spr_bus_addr_i) == `SPR_OFFSET(`OR1K_SPR_TTMR_ADDR));
  wire spr_ttcr_cs = (`SPR_OFFSET(spr_bus_addr_i) == `SPR_OFFSET(`OR1K_SPR_TTCR_ADDR));


  // CPU clock domain:
  // CPU-toggle -> PIC-pulse
  //  to latch SPR bus signals in Wishbone clock domain
  reg        spr_bus_toggle_r;
  // Latch all necassary SPR access controls
  reg        spr_tt_cs_r;
  reg [34:0] spr_tt_cmd_r; // {"ttmr-cs", "ttcr-cs", "we", dat}

  // initial value for simulation
 `ifndef SYNTHESIS
  // synthesis translate_off
  initial begin
    spr_bus_toggle_r  = 1'b0;
    spr_bus_ack_tt_o = 1'b0;
  end
  // synthesis translate_on
 `endif // !synth

  // pulse to initiate read/write transaction
  // ---
  always @(posedge cpu_clk) begin
    if (cpu_rst | spr_bus_ack_tt_o) begin
      spr_tt_cs_r     <= 1'b0;
    end
    else if (~spr_tt_cs_r) begin
      spr_bus_toggle_r <= spr_tt_cs ? (~spr_bus_toggle_r) : spr_bus_toggle_r;
      spr_tt_cs_r      <= spr_tt_cs;
      spr_tt_cmd_r     <= spr_tt_cs ? {spr_ttmr_cs,spr_ttcr_cs,spr_bus_we_i,spr_bus_dat_i} : 35'd0;
    end
  end


  // Wishbone clock domain:
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
      cpu2tt_cs_r2 <= spr_bus_toggle_r;
      cpu2tt_cs_r3 <= cpu2tt_cs_r2;
    end
  end
  // ---
  wire       cpu2tt_cs_pulse = cpu2tt_cs_r2 ^ cpu2tt_cs_r3;
  reg        cpu2tt_cs_pulse_r;
  reg [34:0] cpu2tt_cmd_r; // {"ttmr-cs", "ttcr-cs", "we", dat}
  // ---
  // --- latch SPR access parameters in Wishbone clock domain ---
  always @(posedge wb_clk) begin
    if (wb_rst) begin
      cpu2tt_cs_pulse_r <= 1'b0;
    end
    else begin
      // re-clocked SPR BUS request is ready
      cpu2tt_cs_pulse_r <= cpu2tt_cs_pulse;
      // re-clocked SPR BUS request data
      if (cpu2tt_cs_pulse)
        cpu2tt_cmd_r <= spr_tt_cmd_r;
    end
  end
  // --- unpack SPR access parameters ---
  wire        cpu2tt_ttmr_cs = cpu2tt_cmd_r[34];
  wire        cpu2tt_ttcr_cs = cpu2tt_cmd_r[33];
  wire        cpu2tt_we      = cpu2tt_cmd_r[32];
  wire [31:0] cpu2tt_dat     = cpu2tt_cmd_r[31:0];


  // read/write done and data for output to SPR BUS
  reg        tt_ack_r;
  reg [31:0] tt_dato_r;
  // ---
  always @(posedge wb_clk) begin
    if (wb_rst) begin
      tt_ack_r <= 1'b0;
    end
    else begin
      // ack is single wb-clk
      tt_ack_r <= cpu2tt_cs_pulse_r;
      // data: latch regardless of "we"
      if (cpu2tt_cs_pulse_r) begin
        tt_dato_r <= cpu2tt_ttmr_cs ? spr_ttmr :
                     cpu2tt_ttcr_cs ? spr_ttcr :
                                      32'd0;
      end
    end
  end


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
  wire tt2cpu_ack = tt2cpu_ack_r2 & (~tt2cpu_ack_r3);


  // SPR BUS: output data and ack (CPU clock domain)
  always @(posedge cpu_clk) begin
    if (cpu_rst | spr_bus_ack_tt_o) begin
      spr_bus_ack_tt_o <=  1'b0;
      spr_bus_dat_tt_o <= 32'd0;
    end
    else if (tt2cpu_ack) begin
      spr_bus_ack_tt_o <= 1'd1;
      spr_bus_dat_tt_o <= tt_dato_r;
    end
  end


  // TT-interrupt-level -> CPU-interrupt-level
  //   As CPU clock assumed to be faster or equal to TT's one, we
  // don't use toggle here.
  //   As CDC is not completely implemented, that's why each synchronizer
  // contains only one register. So register enumeration
  // starts from _r2 (*_r1 should be 1st in full sync. implementation).
  reg tt_rdy_r2;
  // ---
  always @(posedge cpu_clk) begin
    if (cpu_rst | spr_bus_ack_tt_o)
      tt_rdy_r2 <= 1'b0;
    else
      tt_rdy_r2 <= spr_ttmr[28];
  end
  // ---
  assign tt_rdy_o = tt_rdy_r2;


  // Timer
  wire ttcr_match = (spr_ttcr[27:0] == spr_ttmr[27:0]);

  // Timer SPR control
  always @(posedge wb_clk) begin
    if (wb_rst)
      spr_ttmr <= 1'b0;
    else if (cpu2tt_cs_pulse_r & cpu2tt_ttmr_cs & cpu2tt_we)
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

  always @(posedge wb_clk) begin
    if (wb_rst)
      spr_ttcr <= 32'd0;
    else if (cpu2tt_cs_pulse_r & cpu2tt_ttcr_cs & cpu2tt_we)
      spr_ttcr <= cpu2tt_dat;
    else if (ttcr_clear)
      spr_ttcr <= 32'd0;
    else if (ttcr_run)
      spr_ttcr <= spr_ttcr + 1'b1;
  end

endmodule // mor1kx_ticktimer_marocchino
