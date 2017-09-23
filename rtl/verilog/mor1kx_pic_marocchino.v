////////////////////////////////////////////////////////////////////////
//                                                                    //
//  mor1kx_pic_marocchino                                             //
//                                                                    //
//  Description:                                                      //
//  Programmable Interrupt Controller for OR1K                        //
//    (a) Re-factored version of Julius Baxter's mor1kx PIC           //
//    (b) Decoupled of CTRL module                                    //
//    (c) PIC itself is in Wishbone clock domain wile SPR bus is in   //
//        CPU clock domain                                            //
//    (d) Actually, CDC is not implemented completely yet.            //
//        The CPU clock could be greater or equal to Wishbone one,    //
//        buth them must be aligned. So, synchronizers consist of     //
//        single latch named "*_r2". To implement full synchronizers  //
//        latches *_r1 shuld be appropriatelly added.                 //
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

module mor1kx_pic_marocchino
#(
  parameter OPTION_PIC_TRIGGER    = "LEVEL",
  parameter OPTION_PIC_NMI_WIDTH  = 0
 )
(
  // Wishbone side clock and reset
  input             wb_clk,
  input             wb_rst,
  // CPU side clock and reset
  input             cpu_clk,
  input             cpu_rst,
  // input interrupt lines
  input      [31:0] irq_i,
  // output interrupt lines
  output     [31:0] spr_picsr_o,
  // SPR BUS
  //  # inputs
  input      [15:0] spr_bus_addr_i,
  input             spr_bus_we_i,
  input             spr_bus_stb_i,
  input      [31:0] spr_bus_dat_i,
  //  # outputs
  output reg [31:0] spr_bus_dat_pic_o,
  output reg        spr_bus_ack_pic_o
);

  // Registers
  reg [31:0] spr_picmr;
  reg [31:0] spr_picsr;

  // enabled IRQs
  wire [31:0] irq_unmasked = spr_picmr & irq_i;


  // SPR BUS interface
  wire spr_pic_cs   = spr_bus_stb_i & (`SPR_BASE(spr_bus_addr_i) == `OR1K_SPR_PIC_BASE);
  wire spr_picmr_cs = (`SPR_OFFSET(spr_bus_addr_i) == `SPR_OFFSET(`OR1K_SPR_PICMR_ADDR));
  wire spr_picsr_cs = (`SPR_OFFSET(spr_bus_addr_i) == `SPR_OFFSET(`OR1K_SPR_PICSR_ADDR));


  // CPU clock domain:
  // CPU-toggle -> PIC-pulse
  //  to latch SPR bus signals in Wishbone clock domain
  reg        spr_bus_toggle_r;
  // Latch all necassary SPR access controls
  reg [34:0] spr_pic_cmd_r; // {"picmr-cs", "picsr-cs", "we", dat}

  // initial value for simulation
 `ifndef SYNTHESIS
  // synthesis translate_off
  initial begin
    spr_bus_toggle_r  = 1'b0;
    spr_bus_ack_pic_o = 1'b0;
  end
  // synthesis translate_on
 `endif // !synth

  // pulse to initiate read/write transaction
  reg  spr_pic_cs_r;
  wire spr_pic_cs_pulse;
  // ---
  always @(posedge cpu_clk) begin
    if (cpu_rst)
      spr_pic_cs_r <= 1'b0;
    else
      spr_pic_cs_r <= spr_pic_cs;
  end // @clock
  // ---
  assign spr_pic_cs_pulse = (~spr_pic_cs_r) & spr_pic_cs;
  // SPR BUS data for PIC
  always @(posedge cpu_clk) begin
    if (spr_pic_cs_pulse) begin
      spr_bus_toggle_r <= (~spr_bus_toggle_r);
      spr_pic_cmd_r    <= {spr_picmr_cs,spr_picsr_cs,spr_bus_we_i,spr_bus_dat_i};
    end
  end // @clock


  // Wishbone clock domain:
  // --- synch. ---
  // As CDC is not completely implemented, that's why each synchronizer
  // contains only one register. So register enumeration
  // starts from _r2 (*_r1 should be 1st in full sync. implementation).
  reg cpu2pic_cs_r2, cpu2pic_cs_r3;
  //---
  always @(posedge wb_clk) begin
    if (wb_rst) begin
      cpu2pic_cs_r2 <= 1'b0;
      cpu2pic_cs_r3 <= 1'b0;
    end
    else begin
      cpu2pic_cs_r2 <= spr_bus_toggle_r;
      cpu2pic_cs_r3 <= cpu2pic_cs_r2;
    end
  end
  // ---
  wire       cpu2pic_cs_pulse = cpu2pic_cs_r2 ^ cpu2pic_cs_r3;
  reg        cpu2pic_cs_pulse_r;
  reg [34:0] cpu2pic_cmd_r; // {"picmr-cs", "picsr-cs", "we", dat}
  // --- latch SPR access parameters in Wishbone clock domain ---
  always @(posedge wb_clk) begin
    if (wb_rst) begin
      cpu2pic_cs_pulse_r <= 1'b0;
    end
    else begin
      // re-clocked SPR BUS request is ready
      cpu2pic_cs_pulse_r <= cpu2pic_cs_pulse;
      // re-clocked SPR BUS request data
      if (cpu2pic_cs_pulse)
        cpu2pic_cmd_r <= spr_pic_cmd_r;
    end
  end
  // --- unpack SPR access parameters ---
  wire        cpu2pic_picmr_cs = cpu2pic_cmd_r[34];
  wire        cpu2pic_picsr_cs = cpu2pic_cmd_r[33];
  wire        cpu2pic_we       = cpu2pic_cmd_r[32];
  wire [31:0] cpu2pic_dat      = cpu2pic_cmd_r[31:0];


  // read/write done and data for output to SPR BUS
  reg         pic_ack_r;
  reg  [31:0] pic_dato_r;
  // ---
  always @(posedge wb_clk) begin
    if (wb_rst) begin
      pic_ack_r <= 1'b0;
    end
    else begin
      // ack is single wb-clk
      pic_ack_r <= cpu2pic_cs_pulse_r;
      // data: latch regardless of "we"
      if (cpu2pic_cs_pulse_r) begin
        pic_dato_r <= cpu2pic_picmr_cs ? spr_picmr :
                      cpu2pic_picsr_cs ? spr_picsr :
                                         32'd0;
      end
    end 
  end


  // PIC-ack-pulse -> CPU-ack-pulse
  //   As CPU clock assumed to be faster or equal to PIC's one, we
  // don't use toggle here.
  //   As CDC is not completely implemented, that's why each synchronizer
  // contains only one register. So register enumeration
  // starts from _r2 (*_r1 should be 1st in full sync. implementation).
  reg pic2cpu_ack_r2;
  reg pic2cpu_ack_r3;
  // ---
  always @(posedge cpu_clk) begin
    if (cpu_rst) begin
      pic2cpu_ack_r2 <= 1'b0;
      pic2cpu_ack_r3 <= 1'b0;
    end
    else begin
      pic2cpu_ack_r2 <= pic_ack_r;
      pic2cpu_ack_r3 <= pic2cpu_ack_r2;
    end
  end
  // ---
  wire pic2cpu_ack = pic2cpu_ack_r2 & (~pic2cpu_ack_r3);


  // SPR BUS: ACK (CPU clock domain, 1-clock)
  always @(posedge cpu_clk) begin
    if (cpu_rst)
      spr_bus_ack_pic_o <= 1'b0;
    else
      spr_bus_ack_pic_o <= pic2cpu_ack;
  end
  // SPR BUS: output data (CPU clock domain, valid for 1-clock)
  always @(posedge cpu_clk) begin
    if (pic2cpu_ack)
      spr_bus_dat_pic_o <= pic_dato_r;
    else
      spr_bus_dat_pic_o <= 32'd0;
  end


  // PIC-interrupt-level -> CPU-interrupt-level
  //   As CPU clock assumed to be faster or equal to PIC's one, we
  // needn't toggle here.
  //   As CDC is not completely implemented, that's why each synchronizer
  // contains only one register. So register enumeration
  // starts from _r2 (*_r1 should be 1st in full sync. implementation).
  reg [31:0] spr_picsr_r2;
  // ---
  always @(posedge cpu_clk) begin
    if (cpu_rst | spr_bus_ack_pic_o)
      spr_picsr_r2 <= 32'd0;
    else
      spr_picsr_r2 <= spr_picsr;
  end
  // ---
  assign spr_picsr_o = spr_picsr_r2;



  // PIC (un)mask register
  always @(posedge wb_clk) begin
    if (wb_rst)
      spr_picmr <= {{(32-OPTION_PIC_NMI_WIDTH){1'b0}},
                    {OPTION_PIC_NMI_WIDTH{1'b1}}};
    else if (cpu2pic_cs_pulse_r & cpu2pic_picmr_cs & cpu2pic_we)
      spr_picmr <= {cpu2pic_dat[31:OPTION_PIC_NMI_WIDTH],
                    {OPTION_PIC_NMI_WIDTH{1'b1}}};
  end


  generate

  genvar  irqline;

  /* verilator lint_off WIDTH */
  if (OPTION_PIC_TRIGGER == "EDGE") begin : edge_triggered
  /* verilator lint_on WIDTH */
    reg  [31:0] irq_unmasked_r;
    wire [31:0] irq_unmasked_edge;

    always @(posedge wb_clk) begin
      if (wb_rst)
        irq_unmasked_r <= 32'd0;
      else
        irq_unmasked_r <= irq_unmasked;
    end

    for(irqline=0; irqline<32; irqline=irqline+1)  begin : picgenerate
      assign irq_unmasked_edge[irqline] = irq_unmasked[irqline] &
                                           ~irq_unmasked_r[irqline];

      // PIC status register
      always @(posedge wb_clk) begin
        if (wb_rst)
          spr_picsr[irqline] <= 1'b0;
        // Set
        else if (irq_unmasked_edge[irqline])
          spr_picsr[irqline] <= 1'b1;
        // Clear
        else if (cpu2pic_cs_pulse_r & cpu2pic_picsr_cs & cpu2pic_we & cpu2pic_dat[irqline])
          spr_picsr[irqline] <= 1'b0;
      end
    end
  end // trigger is "edge"

  /* verilator lint_off WIDTH */
  else if (OPTION_PIC_TRIGGER == "LEVEL") begin : level_triggered
  /* verilator lint_on WIDTH */
    for(irqline=0; irqline<32; irqline=irqline+1) begin : picsrlevelgenerate
      // PIC status register
      always @(*)
        spr_picsr[irqline] = irq_unmasked[irqline];
    end
  end // trigger is "level"

  /* verilator lint_off WIDTH */
  else if (OPTION_PIC_TRIGGER == "LATCHED_LEVEL") begin : latched_level
  /* verilator lint_on WIDTH */
    for(irqline=0; irqline<32; irqline=irqline+1) begin : piclatchedlevelgenerate
      // PIC status register
      always @(posedge wb_clk) begin
        if (wb_rst)
          spr_picsr[irqline] <= 1'b0;
        else if (cpu2pic_cs_pulse_r & cpu2pic_picsr_cs & cpu2pic_we)
          spr_picsr[irqline] <= irq_unmasked[irqline] | cpu2pic_dat[irqline];
        else
          spr_picsr[irqline] <= spr_picsr[irqline] | irq_unmasked[irqline];
      end
    end // block: picgenerate
  end // trigger is "latched level"

  else begin : invalid
    initial begin
      $display("Error - invalid PIC level detection option %s",
                OPTION_PIC_TRIGGER);
      $finish;
    end
  end  // trigger switcher

  endgenerate

endmodule // mor1kx_pic_marocchino
