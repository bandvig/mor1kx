/////////////////////////////////////////////////////////////////////
//                                                                 //
//  mor1kx_store_buffer_marocchino                                 //
//                                                                 //
//  Description:                                                   //
//    Store buffer                                                 //
//    Tightly coupled with MAROCCHINO LSU                          //
//    Based on mor1kx_store_buffer                                 //
//                                                                 //
/////////////////////////////////////////////////////////////////////
//                                                                 //
//   Copyright (C) 2013 Stefan Kristiansson                        //
//                      stefan.kristiansson@saunalahti.fi          //
//                                                                 //
//   Copyright (C) 2015 Andrey Bacherov                            //
//                      avbacherov@opencores.org                   //
//                                                                 //
//      This Source Code Form is subject to the terms of the       //
//      Open Hardware Description License, v. 1.0. If a copy       //
//      of the OHDL was not distributed with this file, You        //
//      can obtain one at http://juliusbaxter.net/ohdl/ohdl.txt    //
//                                                                 //
/////////////////////////////////////////////////////////////////////

`include "mor1kx-defines.v"

module mor1kx_store_buffer_marocchino
#(
  parameter DEPTH_WIDTH          =  4, // 16 taps
  parameter OPTION_OPERAND_WIDTH = 32,
  parameter CLEAR_ON_INIT        = 0
)
(
  input                               cpu_clk,
  input                               cpu_rst,
  // DBUS error during write data from store buffer (force empty)
  input                               sbuf_err_i,
  // entry port
  input    [OPTION_OPERAND_WIDTH-1:0] sbuf_epcr_i,
  input    [OPTION_OPERAND_WIDTH-1:0] virt_addr_i,
  input    [OPTION_OPERAND_WIDTH-1:0] phys_addr_i,
  input    [OPTION_OPERAND_WIDTH-1:0] dat_i,
  input  [OPTION_OPERAND_WIDTH/8-1:0] bsel_i,
  input                               write_i,
  // output port
  output   [OPTION_OPERAND_WIDTH-1:0] sbuf_epcr_o,
  output   [OPTION_OPERAND_WIDTH-1:0] virt_addr_o,
  output   [OPTION_OPERAND_WIDTH-1:0] phys_addr_o,
  output   [OPTION_OPERAND_WIDTH-1:0] dat_o,
  output [OPTION_OPERAND_WIDTH/8-1:0] bsel_o,
  input                               read_i,
  // status flags
  output reg                          full_o,
  output reg                          empty_o
);

  // The fifo stores (pc + virtual_address + physical_address + data + byte-sel)
  localparam DATA_WIDTH = (OPTION_OPERAND_WIDTH * 4) + (OPTION_OPERAND_WIDTH / 8);
  localparam DATA_MSB   = DATA_WIDTH  - 1;

  // For shorter
  localparam DEPTH_MSB = DEPTH_WIDTH - 1;

  // size of counter of booked cells:
  //  (+ 1) - takes into accaunt output register
  localparam BOOKED_CNT_SZ  = DEPTH_WIDTH   + 1;
  localparam BOOKED_CNT_MSB = BOOKED_CNT_SZ - 1;

  // special points of counter of booked cells
  localparam [BOOKED_CNT_MSB:0] FIFO_EMPTY     = 0;
  localparam [BOOKED_CNT_MSB:0] BOOKED_OUT_REG = 1;
  localparam [BOOKED_CNT_MSB:0] BOOKED_OUT_RAM = 2;
  localparam [BOOKED_CNT_MSB:0] FIFO_FULL      = (1 << DEPTH_WIDTH) + 1; // (+ 1): output register


  // counter of booked cells
  reg  [BOOKED_CNT_MSB:0] booked_cnt_r;
  wire [BOOKED_CNT_MSB:0] booked_cnt_inc;
  wire [BOOKED_CNT_MSB:0] booked_cnt_dec;
  reg  [BOOKED_CNT_MSB:0] booked_cnt_nxt; // combinatorial
  // registered FIFO states (driven by counter of booked cells)
  reg                     booked_outreg_r;  // output register is booked
  reg                     booked_outram_r;  // FIFO-RAM outputs is valid
  reg                     booked_intram_r;  // Internally cell in FIFO-RAM is booked


  // RAM_FIFO related
  // pointer for write
  reg       [DEPTH_MSB:0] write_pointer_r;
  wire      [DEPTH_MSB:0] write_pointer_inc;
  reg       [DEPTH_MSB:0] write_pointer_nxt; // combinatorial
  // pointer for read
  reg       [DEPTH_MSB:0] read_pointer_r;
  wire      [DEPTH_MSB:0] read_pointer_inc;
  reg       [DEPTH_MSB:0] read_pointer_nxt; // combinatorial
  // FIFO-RAM ports (combinatorial)
  reg                     rwp_en;   // "read / write" port enable
  reg                     rwp_we;   // "read / write" port writes
  reg       [DEPTH_MSB:0] rwp_addr;
  reg                     wp_en;    // "write only" port enabled
  // packed data
  wire       [DATA_MSB:0] fifo_din; // to FIFO_RAM or output register
  wire       [DATA_MSB:0] ram_dout; // FIFO_RAM output


  // Output register related
  reg        [DATA_MSB:0] outreg_din; // combinatorial
  reg        [DATA_MSB:0] outreg_r;   // output register


  // counter of booked cells
  assign booked_cnt_inc = booked_cnt_r + 1'b1;
  assign booked_cnt_dec = booked_cnt_r - 1'b1;

  // pointers increment
  assign write_pointer_inc = write_pointer_r + 1'b1;
  assign read_pointer_inc  = read_pointer_r  + 1'b1;


  // combinatorial computatition
  always @(read_i          or write_i           or
           booked_cnt_r    or booked_cnt_inc    or booked_cnt_dec  or
           booked_outreg_r or booked_outram_r   or booked_intram_r or
           write_pointer_r or write_pointer_inc or
           read_pointer_r  or read_pointer_inc  or
           fifo_din        or ram_dout          or outreg_r) begin
    // synthesis parallel_case
    case ({read_i, write_i})
      // keep state
      2'b00: begin
        // counter of booked cells
        booked_cnt_nxt = booked_cnt_r;
        // next values for read/write pointers
        write_pointer_nxt = write_pointer_r;
        read_pointer_nxt  = read_pointer_r;
        // FIFO-RAM ports
        rwp_en   = 1'b0;
        rwp_we   = 1'b0;
        rwp_addr = read_pointer_r;
        wp_en    = 1'b0;
        // Output register related
        outreg_din = outreg_r;
      end // keep state

      // "write only"
      2'b01: begin
        // counter of booked cells
        booked_cnt_nxt = booked_cnt_inc;
        // next values for read/write pointers
        write_pointer_nxt = booked_outreg_r ? write_pointer_inc : write_pointer_r;
        read_pointer_nxt  = ((~booked_outram_r) & booked_outreg_r) ? read_pointer_inc : read_pointer_r;
        // FIFO-RAM ports
        rwp_en   = (~booked_outram_r) & booked_outreg_r;
        rwp_we   = (~booked_outram_r) & booked_outreg_r;
        rwp_addr = write_pointer_r;
        wp_en    = booked_outram_r;
        // Output register related
        outreg_din = booked_outreg_r ? outreg_r : fifo_din;
      end // "write only"

      // "read only"
      2'b10: begin
        // counter of booked cells
        booked_cnt_nxt = booked_cnt_dec;
        // next values for read/write pointers
        write_pointer_nxt = write_pointer_r;
        read_pointer_nxt  = booked_intram_r ? read_pointer_inc : read_pointer_r;
        // FIFO-RAM ports
        rwp_en   = 1'b1;
        rwp_we   = 1'b0;
        rwp_addr = read_pointer_r;
        wp_en    = 1'b0;
        // Output register related
        outreg_din = ram_dout;
      end // "read only"

      // "read & write"
      2'b11: begin
        // counter of booked cells
        booked_cnt_nxt = booked_cnt_r;
        // next values for read/write pointers
        write_pointer_nxt = booked_outram_r ? write_pointer_inc : write_pointer_r;
        read_pointer_nxt  = booked_outram_r ? read_pointer_inc  : read_pointer_r;
        // FIFO-RAM ports
        rwp_en   = booked_outram_r;
        rwp_we   = (~booked_intram_r) & booked_outram_r;
        rwp_addr = read_pointer_r; // eq. write pointer for the write case here
        wp_en    = booked_intram_r;
        // Output register related
        outreg_din = booked_outram_r ? ram_dout : fifo_din;
      end // "read & write"
    endcase
  end


  // registering of new states
  always @(posedge cpu_clk) begin
    if (cpu_rst) begin
      // counter of booked cells
      booked_cnt_r    <= {BOOKED_CNT_SZ{1'b0}}; // reset
      // registered FIFO states
      booked_outreg_r <= 1'b0; // reset
      booked_outram_r <= 1'b0; // reset
      booked_intram_r <= 1'b0; // reset
      // write / read pointers
      write_pointer_r <= {DEPTH_WIDTH{1'b0}}; // reset
      read_pointer_r  <= {DEPTH_WIDTH{1'b0}}; // reset
      // status flags
      full_o  <= 1'b0; // reset
      empty_o <= 1'b1; // reset
    end
    else if (sbuf_err_i) begin
      // counter of booked cells
      booked_cnt_r    <= {BOOKED_CNT_SZ{1'b0}}; // sbuff error
      // registered FIFO states
      booked_outreg_r <= 1'b0; // sbuff error
      booked_outram_r <= 1'b0; // sbuff error
      booked_intram_r <= 1'b0; // sbuff error
      // write / read pointers
      write_pointer_r <= {DEPTH_WIDTH{1'b0}}; // sbuff error
      read_pointer_r  <= {DEPTH_WIDTH{1'b0}}; // sbuff error
      // status flags
      full_o  <= 1'b0; // sbuff error
      empty_o <= 1'b1; // sbuff error
    end
    else begin
      // counter of booked cells
      booked_cnt_r    <= booked_cnt_nxt; // update
      // registered FIFO states
      booked_outreg_r <= (booked_cnt_nxt > FIFO_EMPTY); // update
      booked_outram_r <= (booked_cnt_nxt > BOOKED_OUT_REG); // update
      booked_intram_r <= (booked_cnt_nxt > BOOKED_OUT_RAM); // update
      // write / read pointers
      write_pointer_r <= write_pointer_nxt; // update
      read_pointer_r  <= read_pointer_nxt; // update
      // status flags
      full_o  <= (booked_cnt_nxt == FIFO_FULL); // update
      empty_o <= (booked_cnt_nxt == FIFO_EMPTY); // update
    end
  end


  // data input/output
  assign fifo_din = {bsel_i, dat_i, phys_addr_i, virt_addr_i, sbuf_epcr_i};
  assign {bsel_o, dat_o, phys_addr_o, virt_addr_o, sbuf_epcr_o} = outreg_r;


  // instance RAM as FIFO
  mor1kx_dpram_en_w1st_sclk
  #(
    .ADDR_WIDTH     (DEPTH_WIDTH),
    .DATA_WIDTH     (DATA_WIDTH),
    .CLEAR_ON_INIT  (CLEAR_ON_INIT)
  )
  fifo_ram
  (
    // common clock
    .clk    (cpu_clk),
    // port "a": Read/Write
    .en_a   (rwp_en),
    .we_a   (rwp_we),
    .addr_a (rwp_addr),
    .din_a  (fifo_din),
    .dout_a (ram_dout),
    // port "b": Write
    .en_b   (wp_en),
    .we_b   (1'b1),
    .addr_b (write_pointer_r),
    .din_b  (fifo_din),
    .dout_b ()            // not used
  );

  // registered output
  always @(posedge cpu_clk) begin
    outreg_r <= outreg_din;
  end // at clock

endmodule // mor1kx_store_buffer_marocchino
