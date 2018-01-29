//////////////////////////////////////////////////////////////////////
//                                                                  //
//  mor1kx_rf_marocchino                                            //
//                                                                  //
//  Description:                                                    //
//    Register file for MAROCCHINO pipeline                         //
//    Handles reading the register file rams and register bypassing //
//    Derived from mor1kx_rf_cappuccino                             //
//                                                                  //
//////////////////////////////////////////////////////////////////////
//                                                                  //
//   Copyright (C) 2012 Julius Baxter                               //
//                      juliusbaxter@gmail.com                      //
//                                                                  //
//   Copyright (C) 2012-2014 Stefan Kristiansson                    //
//                           stefan.kristiansson@saunalahti.fi      //
//                                                                  //
//   Copyright (C) 2015-2016 Andrey Bacherov                        //
//                           avbacherov@opencores.org               //
//                                                                  //
//      This Source Code Form is subject to the terms of the        //
//      Open Hardware Description License, v. 1.0. If a copy        //
//      of the OHDL was not distributed with this file, You         //
//      can obtain one at http://juliusbaxter.net/ohdl/ohdl.txt     //
//                                                                  //
//////////////////////////////////////////////////////////////////////

`include "mor1kx-defines.v"

module mor1kx_rf_marocchino
#(
  parameter OPTION_RF_CLEAR_ON_INIT  =  0,
  parameter OPTION_RF_ADDR_WIDTH     =  5,
  parameter OPTION_OPERAND_WIDTH     = 32,
  parameter FEATURE_DEBUGUNIT        = "NONE",
  parameter OPTION_RF_NUM_SHADOW_GPR =  0       // for multicore mostly
)
(
  input                             cpu_clk,
  input                             cpu_rst,

  // pipeline control signals
  input                             pipeline_flush_i,
  input                             padv_dcod_i,

  // SPR bus
  input                          [15:0] spr_bus_addr_i,
  input                                 spr_bus_stb_i,
  input                                 spr_bus_we_i,
  input      [OPTION_OPERAND_WIDTH-1:0] spr_bus_dat_i,
  output                                spr_bus_ack_gpr0_o,
  output     [OPTION_OPERAND_WIDTH-1:0] spr_bus_dat_gpr0_o,
  output                                spr_bus_ack_gprS_o,
  output     [OPTION_OPERAND_WIDTH-1:0] spr_bus_dat_gprS_o,

  // from FETCH
  input  [OPTION_RF_ADDR_WIDTH-1:0] fetch_rfa1_adr_i,
  input  [OPTION_RF_ADDR_WIDTH-1:0] fetch_rfb1_adr_i,
  // for FPU64
  input  [OPTION_RF_ADDR_WIDTH-1:0] fetch_rfa2_adr_i,
  input  [OPTION_RF_ADDR_WIDTH-1:0] fetch_rfb2_adr_i,

  // from DECODE
  input  [OPTION_OPERAND_WIDTH-1:0] dcod_immediate_i,
  input                             dcod_immediate_sel_i,

  // Special WB-controls for RF
  input  [OPTION_RF_ADDR_WIDTH-1:0] wb_rf_even_addr_i,
  input                             wb_rf_even_wb_i,
  input  [OPTION_RF_ADDR_WIDTH-1:0] wb_rf_odd_addr_i,
  input                             wb_rf_odd_wb_i,

  // from WB
  input                             wb_rfd1_odd_i,
  input  [OPTION_OPERAND_WIDTH-1:0] wb_result1_i,
  // for FPU64
  input  [OPTION_OPERAND_WIDTH-1:0] wb_result2_i,

  // 1-clock "WB to DECODE operand forwarding" flags
  //  # relative operand A1
  input                            dcod_wb2dec_d1a1_fwd_i,
  input                            dcod_wb2dec_d2a1_fwd_i,
  //  # relative operand B1
  input                            dcod_wb2dec_d1b1_fwd_i,
  input                            dcod_wb2dec_d2b1_fwd_i,
  //  # relative operand A2
  input                            dcod_wb2dec_d1a2_fwd_i,
  input                            dcod_wb2dec_d2a2_fwd_i,
  //  # relative operand B2
  input                            dcod_wb2dec_d1b2_fwd_i,
  input                            dcod_wb2dec_d2b2_fwd_i,

  // outputs
  output [OPTION_OPERAND_WIDTH-1:0] dcod_rfa1_o,
  output [OPTION_OPERAND_WIDTH-1:0] dcod_rfb1_o,
  output [OPTION_OPERAND_WIDTH-1:0] dcod_rfa2_o,
  output [OPTION_OPERAND_WIDTH-1:0] dcod_rfb2_o,

  // we use adder for l.jl/l.jalr to compute return address: (pc+8)
  input                             dcod_op_jal_i,
  input  [OPTION_OPERAND_WIDTH-1:0] pc_decode_i,

  // Special case for l.jr/l.jalr
  output [OPTION_OPERAND_WIDTH-1:0] dcod_rfb1_jr_o
);

  // short names
  localparam RF_AW = OPTION_RF_ADDR_WIDTH;
  localparam RF_DW = OPTION_OPERAND_WIDTH;

  //------------------------------------//
  // LSB of operand addresses in DECODE //
  // for odd/even RAMs selection        //
  //------------------------------------//
  reg dcod_rfa1_adr_odd;
  reg dcod_rfb1_adr_odd;
  reg dcod_rfa2_adr_odd;
  reg dcod_rfb2_adr_odd;

  //---------------------------------------------//
  // odd/even sorted operand addresses in DECODE //
  //---------------------------------------------//
  reg [(RF_AW-1):0] dcod_rfa_even_adr;
  reg [(RF_AW-1):0] dcod_rfa_odd_adr;
  reg [(RF_AW-1):0] dcod_rfb_even_adr;
  reg [(RF_AW-1):0] dcod_rfb_odd_adr;


  //-----------//
  // FETCH->RF //
  //-----------//

  // ram blocks outputs
  //  # for DECODE
  wire [(RF_DW-1):0] rfa_even_dout;
  wire [(RF_DW-1):0] rfa_odd_dout;
  wire [(RF_DW-1):0] rfb_even_dout;
  wire [(RF_DW-1):0] rfb_odd_dout;
  //  # for SPR BUS access
  wire [(RF_DW-1):0] rfa_even_spr_dout;
  wire [(RF_DW-1):0] rfa_odd_spr_dout;


  // GPRs access from SPR bus
  wire               spr_gpr0_we_even;
  wire               spr_gpr0_we_odd;
  wire               spr_gpr0_re;
  wire [(RF_AW-1):0] spr_gpr0_addr;
  wire [(RF_DW-1):0] spr_gpr0_wdata;


  // short name for read request
  wire read_req = padv_dcod_i;


  // 1-clock witting strobes for GPR write
  //  - writting act could be blocked by exceptions processing
  //    because the istruction isn't completed and
  //    will be restarted by l.rfe

  //  write in even
  wire write_even_req = wb_rf_even_wb_i | spr_gpr0_we_even;
  // write in odd
  wire write_odd_req  = wb_rf_odd_wb_i  | spr_gpr0_we_odd;

  // if A(B)'s address is odd than A2(B2)=A(B)+1 is even and vise verse

  //    Write Back even data
  wire [(RF_DW-1):0] wb_even_data = wb_rfd1_odd_i ? wb_result2_i : wb_result1_i;
  //    Write Back odd data
  wire [(RF_DW-1):0] wb_odd_data  = wb_rfd1_odd_i ? wb_result1_i : wb_result2_i;

  //  write even address & data
  wire [(RF_AW-1):0] even_wadr = spr_gpr0_we_even ? spr_gpr0_addr  : wb_rf_even_addr_i;
  wire [(RF_DW-1):0] even_wdat = spr_gpr0_we_even ? spr_gpr0_wdata : wb_even_data;
  //  write odd address & data
  wire [(RF_AW-1):0] odd_wadr = spr_gpr0_we_odd ? spr_gpr0_addr  : wb_rf_odd_addr_i;
  wire [(RF_DW-1):0] odd_wdat = spr_gpr0_we_odd ? spr_gpr0_wdata : wb_odd_data;


  // Controls for A-even RAM block
  //  # read address
  //    ## from IFETCH side
  wire [(RF_AW-1):0] fetch_rfa_even_adr;
  assign fetch_rfa_even_adr = fetch_rfa1_adr_i[0] ? fetch_rfa2_adr_i : fetch_rfa1_adr_i;
  //    ## IFETCH / DECODE combined
  wire [(RF_AW-1):0] rfa_even_radr;
  assign rfa_even_radr = read_req ? fetch_rfa_even_adr : dcod_rfa_even_adr;
  //  # signals for Read/Write port *_rwp_*
  wire rfa_even_rwp_we = write_even_req & (even_wadr == rfa_even_radr);
  wire rfa_even_rwp_en = read_req | rfa_even_rwp_we;
  //  # signals for Write port *_wp_*
  //  # we also use this port for SPR GPR read access
  wire rfa_even_wp_en  = (write_even_req & (even_wadr != rfa_even_radr)) | spr_gpr0_re;
  wire rfa_even_wp_we  = (~pipeline_flush_i) & (~spr_gpr0_re);
  //  # Write port address
  wire [(RF_AW-2):0] rfa_even_wpadr;
  assign rfa_even_wpadr = spr_gpr0_re ? spr_gpr0_addr[(RF_AW-1):1] : even_wadr[(RF_AW-1):1];
  //  # RFA-even RAM-block instance
  mor1kx_dpram_en_w1st_sclk
  #(
    .ADDR_WIDTH     (RF_AW-1),
    .DATA_WIDTH     (RF_DW),
    .CLEAR_ON_INIT  (OPTION_RF_CLEAR_ON_INIT)
  )
  rfa_even
  (
    // common clock
    .clk    (cpu_clk),
    // port "a": Read / Write (for RW-conflict case)
    .en_a   (rfa_even_rwp_en),
    .we_a   (rfa_even_rwp_we & (~pipeline_flush_i)),
    .addr_a (rfa_even_radr[(RF_AW-1):1]),
    .din_a  (even_wdat),
    .dout_a (rfa_even_dout),
    // port "b": Write if no RW-conflict
    .en_b   (rfa_even_wp_en),
    .we_b   (rfa_even_wp_we),
    .addr_b (rfa_even_wpadr),
    .din_b  (even_wdat),
    .dout_b (rfa_even_spr_dout)
  );


  // Controls for A-odd RAM block
  //  # read address
  //    ## from IFETCH side
  wire [(RF_AW-1):0] fetch_rfa_odd_adr;
  assign fetch_rfa_odd_adr = fetch_rfa1_adr_i[0] ? fetch_rfa1_adr_i : fetch_rfa2_adr_i;
  //    ## IFETCH / DECODE combined
  wire [(RF_AW-1):0] rfa_odd_radr;
  assign rfa_odd_radr = read_req ? fetch_rfa_odd_adr : dcod_rfa_odd_adr;
  //  # signals for Read/Write port *_rwp_*
  wire rfa_odd_rwp_we = write_odd_req & (odd_wadr == rfa_odd_radr);
  wire rfa_odd_rwp_en = read_req | rfa_odd_rwp_we;
  //  # signals for Write port *_wp_*
  //  # we also use this port for SPR GPR read access
  wire rfa_odd_wp_en  = (write_odd_req & (odd_wadr != rfa_odd_radr)) | spr_gpr0_re;
  wire rfa_odd_wp_we  = (~pipeline_flush_i) & (~spr_gpr0_re);
  //  # Write port address
  wire [(RF_AW-2):0] rfa_odd_wpadr;
  assign rfa_odd_wpadr = spr_gpr0_re ? spr_gpr0_addr[(RF_AW-1):1] : odd_wadr[(RF_AW-1):1];
  //  # RFA-odd RAM-block instance
  mor1kx_dpram_en_w1st_sclk
  #(
    .ADDR_WIDTH     (RF_AW-1),
    .DATA_WIDTH     (RF_DW),
    .CLEAR_ON_INIT  (OPTION_RF_CLEAR_ON_INIT)
  )
  rfa_odd
  (
    // common clock
    .clk    (cpu_clk),
    // port "a": Read / Write (for RW-conflict case)
    .en_a   (rfa_odd_rwp_en),
    .we_a   (rfa_odd_rwp_we & (~pipeline_flush_i)),
    .addr_a (rfa_odd_radr[(RF_AW-1):1]),
    .din_a  (odd_wdat),
    .dout_a (rfa_odd_dout),
    // port "b": Write if no RW-conflict
    .en_b   (rfa_odd_wp_en),
    .we_b   (rfa_odd_wp_we),
    .addr_b (rfa_odd_wpadr),
    .din_b  (odd_wdat),
    .dout_b (rfa_odd_spr_dout)
  );


  // Controls for B-even RAM block
  //  # read address
  //    ## from IFETCH side
  wire [(RF_AW-1):0] fetch_rfb_even_adr;
  assign fetch_rfb_even_adr = fetch_rfb1_adr_i[0] ? fetch_rfb2_adr_i : fetch_rfb1_adr_i;
  //    ## IFETCH / DECODE combined
  wire [(RF_AW-1):0] rfb_even_radr;
  assign rfb_even_radr = read_req ? fetch_rfb_even_adr : dcod_rfb_even_adr;
  //  # signals for Read/Write port *_rwp_*
  wire rfb_even_rwp_we = write_even_req & (even_wadr == rfb_even_radr);
  wire rfb_even_rwp_en = read_req | rfb_even_rwp_we;
  //  # signals for Write port *_wp_*
  wire rfb_even_wp_en = write_even_req & (even_wadr != rfb_even_radr);
  //  # RFA-even RAM-block instance
  mor1kx_dpram_en_w1st_sclk
  #(
    .ADDR_WIDTH     (RF_AW-1),
    .DATA_WIDTH     (RF_DW),
    .CLEAR_ON_INIT  (OPTION_RF_CLEAR_ON_INIT)
  )
  rfb_even
  (
    // common clock
    .clk    (cpu_clk),
    // port "a": Read / Write (for RW-conflict case)
    .en_a   (rfb_even_rwp_en),
    .we_a   (rfb_even_rwp_we & (~pipeline_flush_i)),
    .addr_a (rfb_even_radr[(RF_AW-1):1]),
    .din_a  (even_wdat),
    .dout_a (rfb_even_dout),
    // port "b": Write if no RW-conflict
    .en_b   (rfb_even_wp_en),
    .we_b   (~pipeline_flush_i),
    .addr_b (even_wadr[(RF_AW-1):1]),
    .din_b  (even_wdat),
    .dout_b ()
  );


  // Controls for B-odd RAM block
  //  # read address
  //    ## from IFETCH side
  wire [(RF_AW-1):0] fetch_rfb_odd_adr;
  assign fetch_rfb_odd_adr = fetch_rfb1_adr_i[0] ? fetch_rfb1_adr_i : fetch_rfb2_adr_i;
  //    ## IFETCH / DECODE combined
  wire [(RF_AW-1):0] rfb_odd_radr;
  assign rfb_odd_radr = read_req ? fetch_rfb_odd_adr : dcod_rfb_odd_adr;
  //  # signals for Read/Write port *_rwp_*
  wire rfb_odd_rwp_we = write_odd_req & (odd_wadr == rfb_odd_radr);
  wire rfb_odd_rwp_en = read_req | rfb_odd_rwp_we;
  //  # signals for Write port *_wp_*
  wire rfb_odd_wp_en = write_odd_req & (odd_wadr != rfb_odd_radr);
  //  # RFA-odd RAM-block instance
  mor1kx_dpram_en_w1st_sclk
  #(
    .ADDR_WIDTH     (RF_AW-1),
    .DATA_WIDTH     (RF_DW),
    .CLEAR_ON_INIT  (OPTION_RF_CLEAR_ON_INIT)
  )
  rfb_odd
  (
    // common clock
    .clk    (cpu_clk),
    // port "a": Read / Write (for RW-conflict case)
    .en_a   (rfb_odd_rwp_en),
    .we_a   (rfb_odd_rwp_we & (~pipeline_flush_i)),
    .addr_a (rfb_odd_radr[(RF_AW-1):1]),
    .din_a  (odd_wdat),
    .dout_a (rfb_odd_dout),
    // port "b": Write if no RW-conflict
    .en_b   (rfb_odd_wp_en),
    .we_b   (~pipeline_flush_i),
    .addr_b (odd_wadr[(RF_AW-1):1]),
    .din_b  (odd_wdat),
    .dout_b ()
  );


  //---------------//
  // SPR interface //
  //---------------//

  //  we don't expect R/W-collisions for SPRbus vs WB cycles since
  //    SPRbus access start 1-clock later than WB
  //    thanks to MT(F)SPR processing logic (see OMAN)

  // Registering SPR BUS incoming signals.

  // SPR BUS strobe registering
  reg spr_bus_stb_r;
  // ---
  always @(posedge cpu_clk) begin
    if (cpu_rst)
      spr_bus_stb_r <= 1'b0;
    else if (spr_bus_ack_gpr0_o | spr_bus_ack_gprS_o)
      spr_bus_stb_r <= 1'b0;
    else
      spr_bus_stb_r <= spr_bus_stb_i;
  end // at clock

  // SPR BUS address registering
  reg [15:0] spr_bus_addr_r;
  // ---
  always @(posedge cpu_clk) begin
    spr_bus_addr_r <= spr_bus_addr_i;
  end

  // Registering SPR BUS "write strobe" and "data for write".
  wire               spr_bus_we_l;
  wire [(RF_DW-1):0] spr_bus_dat_l;

  generate
  /* verilator lint_off WIDTH */
  if ((FEATURE_DEBUGUNIT != "NONE") || (OPTION_RF_NUM_SHADOW_GPR > 0)) begin : spr_aux_enabled
  /* verilator lint_on WIDTH */

    // Registering SPR BUS "write strobe" and "data for write".
    reg               spr_bus_we_r; // DBGU or SHADOW
    reg [(RF_DW-1):0] spr_bus_dat_r; // DBGU or SHADOW
    // ---
    assign spr_bus_we_l  = spr_bus_we_r; // DBGU or SHADOW
    assign spr_bus_dat_l = spr_bus_dat_r; // DBGU or SHADOW
    // ---
    always @(posedge cpu_clk) begin
      spr_bus_we_r  <= spr_bus_we_i; // DBGU or SHADOW
      spr_bus_dat_r <= spr_bus_dat_i; // DBGU or SHADOW
    end // at clock

  end
  else begin : spr_aux_disabled

    assign spr_bus_we_l  = 1'b0;          // No DBGU and No SHADOW
    assign spr_bus_dat_l = {RF_DW{1'b0}}; // No DBGU and No SHADOW

  end
  endgenerate


  // GPR[0] chip select
  // low bits of SPR-address are used for addressing GPR[0]
  wire spr_gpr0_cs = spr_bus_stb_r &                  // GPR[0] access
                     (spr_bus_addr_r[15:9] == 7'd2) & // GPR[0] access, see OR1K_SPR_GPR0_ADDR
                     (spr_bus_addr_r[ 8:5] == 4'd0);  // GPR[0] access

  // We declare states or SPR BUS FSM here because some
  // of EDA tools deprecate local parameters declaration
  // inside of generate blocks.

  localparam  [4:0] SPR_GPR0_WAITING = 5'b00001,
                    SPR_GPR0_WE_STRB = 5'b00010,
                    SPR_GPR0_RE_STRB = 5'b00100,
                    SPR_GPR0_READ_MX = 5'b01000,
                    SPR_GPR0_ACK     = 5'b10000;

  generate
  /* verilator lint_off WIDTH */
  if (FEATURE_DEBUGUNIT != "NONE") begin : dbgu_enabled
  /* verilator lint_on WIDTH */

    // SPR BUS FSM states
    reg         [4:0] spr_gpr0_state;

    // Particular states
    wire              spr_gpr0_waiting_state = spr_gpr0_state[0];
    assign            spr_gpr0_re            = spr_gpr0_state[2]; // DBGU enabled
    wire              spr_gpr0_read_mx_state = spr_gpr0_state[3];
    assign            spr_bus_ack_gpr0_o     = spr_gpr0_state[4]; // DBGU enabled

    // Read/Write controls
    reg               spr_gpr0_we_even_r;
    reg               spr_gpr0_we_odd_r;

    // Read data
    reg [(RF_DW-1):0] spr_bus_dat_gpr0_r;


    // SPR BUS FSM
    always @(posedge cpu_clk) begin
      if (cpu_rst) begin
        // controls
        spr_gpr0_we_even_r <= 1'b0;
        spr_gpr0_we_odd_r  <= 1'b0;
        // state
        spr_gpr0_state     <= SPR_GPR0_WAITING;
      end
      else begin
        // synthesis parallel_case full_case
        case (spr_gpr0_state)
          // waiting GPR[0] access
          SPR_GPR0_WAITING: begin
            if (spr_gpr0_cs) begin
              // controls
              spr_gpr0_we_even_r <= spr_bus_we_l & (~spr_bus_addr_r[0]);
              spr_gpr0_we_odd_r  <= spr_bus_we_l &   spr_bus_addr_r[0];
              // next state
              spr_gpr0_state     <= spr_bus_we_l ? SPR_GPR0_WE_STRB : SPR_GPR0_RE_STRB;
            end
          end

          // write strobe
          SPR_GPR0_WE_STRB: begin
            // controls
            spr_gpr0_we_even_r <= 1'b0;
            spr_gpr0_we_odd_r  <= 1'b0;
            // next state
            spr_gpr0_state     <= SPR_GPR0_ACK;
          end

          // read strobe
          SPR_GPR0_RE_STRB: begin
            spr_gpr0_state <= SPR_GPR0_READ_MX;
          end

          // latch data for read
          SPR_GPR0_READ_MX: begin
            spr_gpr0_state <= SPR_GPR0_ACK;
          end

          // done
          SPR_GPR0_ACK: begin
            spr_gpr0_state <= SPR_GPR0_WAITING;
          end

          default;
        endcase
      end
    end // @ clock


    // Read/Write controls
    assign spr_gpr0_we_even = spr_gpr0_we_even_r; // DBGU enabled
    assign spr_gpr0_we_odd  = spr_gpr0_we_odd_r; // DBGU enabled

    // Acceess address and data to write
    assign spr_gpr0_addr  = spr_bus_addr_r[(RF_AW-1):0]; // DBGU enabled
    assign spr_gpr0_wdata = spr_bus_dat_l; // DBGU enabled


    // registered output data: valid for 1-clock only with rised ACK
    always @(posedge cpu_clk) begin
      spr_bus_dat_gpr0_r <= {RF_DW{spr_gpr0_read_mx_state}} &                          // SPR GPR[0] read latch
                            (spr_gpr0_addr[0] ? rfa_odd_spr_dout : rfa_even_spr_dout); // SPR GPR[0] read latch
    end
    // ---
    assign spr_bus_dat_gpr0_o = spr_bus_dat_gpr0_r; // DBGU enabled

  end
  else begin : dbgu_disabled

    // make ACK
    reg    spr_bus_ack_gpr0_r;
    assign spr_bus_ack_gpr0_o = spr_bus_ack_gpr0_r; // DBGU disabled
    // ---
    always @(posedge cpu_clk) begin
      if (cpu_rst)
        spr_bus_ack_gpr0_r <= 1'b0;
      else if (spr_bus_ack_gpr0_r)
        spr_bus_ack_gpr0_r <= 1'b0;
      else if (spr_gpr0_cs)
        spr_bus_ack_gpr0_r <= 1'b1;
    end

    // SPR data output
    assign spr_bus_dat_gpr0_o = {RF_DW{1'b0}}; // DBGU disabled

    // Write by SPR-bus command
    assign spr_gpr0_we_even = 1'b0; // DBGU disabled
    assign spr_gpr0_we_odd  = 1'b0; // DBGU disabled
    assign spr_gpr0_re      = 1'b0; // DBGU disabled

    // Address and data for write
    assign spr_gpr0_addr  = {RF_AW{1'b0}}; // DBGU disabled
    assign spr_gpr0_wdata = {RF_DW{1'b0}}; // DBGU disabled

  end
  endgenerate


  //--------------//
  // GPR [S]hadow //
  //--------------//

  `include "mor1kx_utils.vh"

  function integer calc_shadow_addr_width;
    input integer rf_addr_width;
    input integer rf_num_shadow_gpr;
    calc_shadow_addr_width  = rf_addr_width +
                              ((rf_num_shadow_gpr == 1) ? 1 :
                               `clog2(rf_num_shadow_gpr));
  endfunction

  localparam SHADOW_AW = calc_shadow_addr_width(OPTION_RF_ADDR_WIDTH,
                                                OPTION_RF_NUM_SHADOW_GPR);

  // GPR[S]hadow chip select
  // low bits of SPR-address are used for addressing GPR[S]hadow
  wire spr_gprS_cs = spr_bus_stb_r &                  // GPR[S]hadow access
                     (spr_bus_addr_r[15:9] == 7'd2) & // GPR[S]hadow access, same to OR1K_SPR_GPR0_ADDR
                     (spr_bus_addr_r[ 8:5] != 4'd0);  // GPR[S]hadow access

  // We declare states or SPR BUS FSM here because some
  // of EDA tools deprecate local parameters declaration
  // inside of generate blocks.

  localparam  [4:0] SPR_GPRS_WAITING = 5'b00001,
                    SPR_GPRS_WE_STRB = 5'b00010,
                    SPR_GPRS_RE_STRB = 5'b00100,
                    SPR_GPRS_READ_MX = 5'b01000,
                    SPR_GPRS_ACK     = 5'b10000;

  generate
  /* verilator lint_off WIDTH */
  if (OPTION_RF_NUM_SHADOW_GPR > 0) begin : shadow_enabled
  /* verilator lint_on WIDTH */

    // SPR BUS FSM states
    reg         [4:0] spr_gprS_state;

    // Particular states
    wire              spr_gprS_waiting_state = spr_gprS_state[0];
    wire              spr_gprS_we_strb_state = spr_gprS_state[1];
    wire              spr_gprS_re_strb_state = spr_gprS_state[2];
    wire              spr_gprS_read_mx_state = spr_gprS_state[3];
    assign            spr_bus_ack_gprS_o     = spr_gprS_state[4];

    // Read data
    reg [(RF_DW-1):0] spr_bus_dat_gprS_r;


    // SPR BUS FSM
    always @(posedge cpu_clk) begin
      if (cpu_rst) begin
        spr_gprS_state <= SPR_GPRS_WAITING;
      end
      else begin
        // synthesis parallel_case full_case
        case (spr_gprS_state)
          // waiting GPR[0] access
          SPR_GPRS_WAITING: begin
            if (spr_gprS_cs) begin
              spr_gprS_state <= spr_bus_we_l ? SPR_GPRS_WE_STRB : SPR_GPRS_RE_STRB;
            end
          end

          // write strobe
          SPR_GPRS_WE_STRB: begin
            spr_gprS_state <= SPR_GPRS_ACK;
          end

          // read strobe
          SPR_GPRS_RE_STRB: begin
            spr_gprS_state <= SPR_GPRS_READ_MX;
          end

          // latch data for read
          SPR_GPRS_READ_MX: begin
            spr_gprS_state <= SPR_GPRS_ACK;
          end

          // done
          SPR_GPRS_ACK: begin
            spr_gprS_state <= SPR_GPRS_WAITING;
          end

          default;
        endcase
      end
    end // @ clock


    // registered output data: valid for 1-clock only with rised ACK
    wire [(RF_DW-1):0] rfS_dout;
    // ---
    always @(posedge cpu_clk) begin
      spr_bus_dat_gprS_r <= {RF_DW{spr_gprS_read_mx_state}} & // SPR GPR[S]hadow read latch
                            rfS_dout; // SPR GPR[S]hadow read latch
    end
    // ---
    assign spr_bus_dat_gprS_o = spr_bus_dat_gprS_r; // SHADOW enabled


    // Shadow RAM instance
    mor1kx_spram_en_w1st
    #(
      .ADDR_WIDTH     (SHADOW_AW),
      .DATA_WIDTH     (RF_DW),
      .CLEAR_ON_INIT  (OPTION_RF_CLEAR_ON_INIT)
    )
    rfShadow
    (
      // clock
      .clk    (cpu_clk),
      // port
      .en     (spr_gprS_we_strb_state | spr_gprS_re_strb_state),
      .we     (spr_gprS_we_strb_state),
      .addr   (spr_bus_addr_r[(SHADOW_AW-1):0]),
      .din    (spr_bus_dat_l),
      .dout   (rfS_dout)
    );

  end
  else begin: shadow_disabled

    // make ACK
    reg    spr_bus_ack_gprS_r;
    assign spr_bus_ack_gprS_o = spr_bus_ack_gprS_r; // SHADOW disabled
    // ---
    always @(posedge cpu_clk) begin
      if (cpu_rst)
        spr_bus_ack_gprS_r <= 1'b0;
      else if (spr_bus_ack_gprS_r)
        spr_bus_ack_gprS_r <= 1'b0;
      else if (spr_gprS_cs)
        spr_bus_ack_gprS_r <= 1'b1;
    end

    // SPR data output
    assign spr_bus_dat_gprS_o = {RF_DW{1'b0}}; // SHADOW disabled

  end
  endgenerate


  //-----------------------//
  // DECODE stage (dcod_*) //
  //-----------------------//

  // update operand addresses
  always @(posedge cpu_clk) begin
    if (cpu_rst) begin
      // LSB of Ax/Bx operand addresses
      dcod_rfa1_adr_odd <= 1'b0;
      dcod_rfb1_adr_odd <= 1'b0;
      dcod_rfa2_adr_odd <= 1'b1;
      dcod_rfb2_adr_odd <= 1'b1;
      // Even/Odd sorted operand addresses
      dcod_rfa_even_adr <= {RF_AW{1'b0}};
      dcod_rfa_odd_adr  <= {{(RF_AW-1){1'b0}},1'b1};
      dcod_rfb_even_adr <= {RF_AW{1'b0}};
      dcod_rfb_odd_adr  <= {{(RF_AW-1){1'b0}},1'b1};
    end
    else if (padv_dcod_i) begin
      // Ax/Bx operand addresses
      dcod_rfa1_adr_odd <= fetch_rfa1_adr_i[0];
      dcod_rfb1_adr_odd <= fetch_rfb1_adr_i[0];
      dcod_rfa2_adr_odd <= fetch_rfa2_adr_i[0];
      dcod_rfb2_adr_odd <= fetch_rfb2_adr_i[0];
      // LSB of Even/Odd sorted operand addresses
      dcod_rfa_even_adr <= fetch_rfa_even_adr;
      dcod_rfa_odd_adr  <= fetch_rfa_odd_adr;
      dcod_rfb_even_adr <= fetch_rfb_even_adr;
      dcod_rfb_odd_adr  <= fetch_rfb_odd_adr;
    end
  end // at clock

  // Muxing and forwarding RFA1-output
  assign dcod_rfa1_o = dcod_op_jal_i          ? pc_decode_i  :
                       dcod_wb2dec_d1a1_fwd_i ? wb_result1_i :
                       dcod_wb2dec_d2a1_fwd_i ? wb_result2_i :
                       dcod_rfa1_adr_odd      ? rfa_odd_dout :
                                                rfa_even_dout;

  // Muxing and forwarding RFB1-output
  assign dcod_rfb1_o = dcod_op_jal_i          ? 4'd8             : // (FEATURE_DELAY_SLOT == "ENABLED")
                       dcod_immediate_sel_i   ? dcod_immediate_i :
                       dcod_wb2dec_d1b1_fwd_i ? wb_result1_i     :
                       dcod_wb2dec_d2b1_fwd_i ? wb_result2_i     :
                       dcod_rfb1_adr_odd      ? rfb_odd_dout     :
                                                rfb_even_dout;

  // Muxing and forwarding RFA2-output
  assign dcod_rfa2_o = dcod_wb2dec_d1a2_fwd_i ? wb_result1_i :
                       dcod_wb2dec_d2a2_fwd_i ? wb_result2_i :
                       dcod_rfa2_adr_odd      ? rfa_odd_dout :
                                                rfa_even_dout;

  // Muxing and forwarding RFB2-output
  assign dcod_rfb2_o = dcod_wb2dec_d1b2_fwd_i ? wb_result1_i :
                       dcod_wb2dec_d2b2_fwd_i ? wb_result2_i :
                       dcod_rfb2_adr_odd      ? rfb_odd_dout :
                                                rfb_even_dout;


  // Special case for l.jr/l.jalr
  //   (a) By default these instructions require B1 operand,
  //       so we implemented simlified multiplexor here
  //   (b) The output is used next clock to DECODE to form
  //       registered l.jr/l.jalr target
  //   (c) IFETCH generates bubbles till B1 completion
  assign dcod_rfb1_jr_o = dcod_rfb1_adr_odd ? rfb_odd_dout : rfb_even_dout;

endmodule // mor1kx_rf_marocchino
