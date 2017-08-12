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
  localparam FIFO_DATA_WIDTH = OPTION_OPERAND_WIDTH*4 +
                               OPTION_OPERAND_WIDTH/8;

  wire  [FIFO_DATA_WIDTH-1:0] fifo_dout;
  wire  [FIFO_DATA_WIDTH-1:0] fifo_din;

  reg         [DEPTH_WIDTH:0] write_pointer;
  wire        [DEPTH_WIDTH:0] write_pointer_next;

  reg         [DEPTH_WIDTH:0] read_pointer;
  wire        [DEPTH_WIDTH:0] read_pointer_next;

  // write pointer update
  assign write_pointer_next = write_pointer + 1'b1;
  // ---
  always @(posedge cpu_clk) begin
    if (cpu_rst)
      write_pointer <= {(DEPTH_WIDTH+1){1'b0}};
    else if (sbuf_err_i)
      write_pointer <= {(DEPTH_WIDTH+1){1'b0}};
    else if (write_i)
      write_pointer <= write_pointer_next;
  end

  // read pointer update
  assign read_pointer_next = read_pointer + 1'b1;
  // ---
  always @(posedge cpu_clk) begin
    if (cpu_rst)
      read_pointer <= {(DEPTH_WIDTH+1){1'b0}};
    else if (sbuf_err_i)
      read_pointer <= {(DEPTH_WIDTH+1){1'b0}};
    else if (read_i)
      read_pointer <= read_pointer_next;
  end

  // "buffer is full" flag & "buffer is empty" flags
  wire full_by_write = (write_pointer_next[DEPTH_WIDTH] ^ read_pointer[DEPTH_WIDTH]) &
                       (write_pointer_next[DEPTH_WIDTH-1:0] == read_pointer[DEPTH_WIDTH-1:0]);
  wire empty_by_read = (write_pointer == read_pointer_next);
  // ---
  always @(posedge cpu_clk) begin
    if (cpu_rst) begin
      full_o  <= 1'b0;
      empty_o <= 1'b1;
    end
    else if (sbuf_err_i) begin
      full_o  <= 1'b0;
      empty_o <= 1'b1;
    end
    else begin
      // synthesis parallel_case full_case
      case ({read_i,write_i})
        2'b01: begin // write only
          full_o  <= full_by_write;
          empty_o <= 1'b0;
        end
        2'b10: begin // read only
          full_o  <= 1'b0;
          empty_o <= empty_by_read;
        end
        default:;
      endcase
    end
  end

  // data input/output
  assign fifo_din = {bsel_i, dat_i, phys_addr_i, virt_addr_i, sbuf_epcr_i};
  assign {bsel_o, dat_o, phys_addr_o, virt_addr_o, sbuf_epcr_o} = fifo_dout;


  // FIFO instance
  mor1kx_dpram_en_w1st_sclk
  #(
    .ADDR_WIDTH     (DEPTH_WIDTH),
    .DATA_WIDTH     (FIFO_DATA_WIDTH),
    .CLEAR_ON_INIT  (CLEAR_ON_INIT)
  )
  fifo_ram
  (
    // common clock
    .clk    (cpu_clk),
    // port "a": Read
    .en_a   (read_i),
    .we_a   (1'b0),
    .addr_a (read_pointer[DEPTH_WIDTH-1:0]),
    .din_a  ({FIFO_DATA_WIDTH{1'b0}}),
    .dout_a (fifo_dout),
    // port "b": Write
    .en_b   (write_i),
    .we_b   (write_i),
    .addr_b (write_pointer[DEPTH_WIDTH-1:0]),
    .din_b  (fifo_din),
    .dout_b ()            // not used
  );

endmodule // mor1kx_store_buffer_marocchino
