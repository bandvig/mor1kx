/////////////////////////////////////////////////////////////////////
//                                                                 //
//  Order Control Buffer and Reservation Station                   //
//  for MAROCCHINO pipeline                                        //
//                                                                 //
//  Author: Andrey Bacherov                                        //
//          avbacherov@opencores.org                               //
//                                                                 //
/////////////////////////////////////////////////////////////////////
//                                                                 //
//   Copyright (C) 2015 - 2018 Andrey Bacherov                     //
//                             avbacherov@opencores.org            //
//                                                                 //
//      This Source Code Form is subject to the terms of the       //
//      Open Hardware Description License, v. 1.0. If a copy       //
//      of the OHDL was not distributed with this file, You        //
//      can obtain one at http://juliusbaxter.net/ohdl/ohdl.txt    //
//                                                                 //
/////////////////////////////////////////////////////////////////////


`include "mor1kx-defines.v"


//-------------------------------//
// A Tap of Order Control Buffer //
//-------------------------------//

module ocb_tap
#(
  parameter DATA_SIZE = 2
)
(
  // clock
  input                      clk,
  // value at reset/flush
  input                      reset, // to default value; synchronous
  input      [DATA_SIZE-1:0] default_value_i,
  // input controls and data
  input                      push_i,
  input      [DATA_SIZE-1:0] prev_tap_out_i,
  input      [DATA_SIZE-1:0] forwarded_value_i,
  input                      use_forwarded_value_i,
  // data ouputs
  output reg [DATA_SIZE-1:0] out_o
);

  always @(posedge clk) begin
    if (reset)
      out_o <= default_value_i;
    else if (push_i)
      out_o <= use_forwarded_value_i ? forwarded_value_i :
                                       prev_tap_out_i;
  end // @clock

endmodule // ocb_tap



//---------------------------------------------------------------//
// Order Control Buffer                                          //
//   all outputs could be analized simultaneously for example to //
//   detect data dependancy                                      //
//---------------------------------------------------------------//
/*
module mor1kx_ocb_marocchino
#(
  parameter NUM_TAPS    = 8,
  parameter NUM_OUTS    = 1,
  parameter DATA_SIZE   = 2,
  parameter FULL_FLAG   = "NONE", // "ENABLED" / "NONE"
  parameter EMPTY_FLAG  = "NONE"  // "ENABLED" / "NONE"
)
(
  // clocks, resets
  input                  clk,
  input                  rst,
  // pipe controls
  input                  pipeline_flush_i, // flush pipe
  input                  write_i,
  input                  read_i,
  // value at reset/flush
  input                  reset_taps, // to default value; synchronous
  input  [DATA_SIZE-1:0] default_value_i,
  // data input
  input  [DATA_SIZE-1:0] ocbi_i,
  // "OCB is empty" flag
  output                 empty_o,
  // "OCB is full" flag
  //   (a) external control logic must stop the "writing without reading"
  //       operation if OCB is full
  //   (b) however, the "writing + reading" is possible
  //       because it just pushes OCB and keeps it full
  output                 full_o,
  // output layout
  // { out[n-1], out[n-2], ... out[0] } : DECODE (entrance) -> EXECUTE (exit)
  output [DATA_SIZE*NUM_OUTS-1:0] ocbo_o
);

  // "pointers"
  reg   [NUM_TAPS:0] ptr_curr; // on current active tap
  reg [NUM_TAPS-1:0] ptr_prev; // on previous active tap

  // "OCB is empty" flag
  generate
  // verilator lint_off WIDTH
  if (EMPTY_FLAG != "NONE") begin : ocb_empty_flag_enabled
  // verilator lint_on WIDTH
    assign empty_o = ptr_curr[0];
  end
  else begin : ocb_empty_flag_disabled
    assign empty_o = 1'b0;
  end
  endgenerate

  // "OCB is full" flag
  //  # no more availaible taps, pointer is out of range
  generate
  // verilator lint_off WIDTH
  if (FULL_FLAG != "NONE") begin : ocb_full_flag_enabled
  // verilator lint_on WIDTH
    assign full_o = ptr_curr[NUM_TAPS];
  end
  else begin : ocb_full_flag_disabled
    assign full_o = 1'b0;
  end
  endgenerate

  // control to increment/decrement pointers
  wire rd_only = ~write_i &  read_i;
  wire wr_only =  write_i & ~read_i;
  wire wr_rd   =  write_i &  read_i;


  // operation algorithm:
  //-----------------------------------------------------------------------------
  // read only    | push: tap[k-1] <= tap[k], tap[num_taps-1] <= reset_value;
  //              | update pointers: if(~ptr_prev_0) ptr_prev <= (ptr_prev >> 1);
  //              |                  if(~ptr_curr_0) ptr_curr <= (ptr_curr >> 1);
  //-----------------------------------------------------------------------------
  // write only   | tap[ptr_curr] <= ocbi_i
  //              | ptr_prev <= ptr_curr;
  //              | ptr_curr <= (ptr_curr << 1);
  //-----------------------------------------------------------------------------
  // read & write | push: tap[k-1] <= tap[k]
  //              |       tap[ptr_prev] <= ocbi_i;
  //-----------------------------------------------------------------------------

  wire ptrs_inc = wr_only; // try to increment pointers
  wire ptrs_dec = rd_only; // try to decrement pointers

  // update pointer on current tap
  always @(posedge clk) begin
    if (rst | pipeline_flush_i) begin
      ptr_prev <= {{(NUM_TAPS-1){1'b0}},1'b1};
      ptr_curr <= {{NUM_TAPS{1'b0}},1'b1};
    end
    else if (ptrs_inc) begin
      ptr_prev <= ptr_curr[NUM_TAPS-1:0];
      ptr_curr <= {ptr_curr[NUM_TAPS-1:0],1'b0};
    end
    else if (ptrs_dec) begin
      ptr_prev <= (ptr_prev[0] ? ptr_prev : {1'b0,ptr_prev[NUM_TAPS-1:1]});
      ptr_curr <= (ptr_curr[0] ? ptr_curr : {1'b0,ptr_curr[NUM_TAPS:1]});
    end
  end // @clock


  // enable signals for taps
  wire [NUM_TAPS-1:0] en_by_wr_only = {NUM_TAPS{wr_only}} & ptr_curr[NUM_TAPS-1:0];

  // enable signals for taps
  wire [NUM_TAPS-1:0] push_taps = en_by_wr_only |     // PUSH_TAPS: tap[ptr_curr] <= ocbi_i (particular by write only)
                                  {NUM_TAPS{read_i}}; // PUSH_TAPS: tap[k-1] <= tap[k]      (all by a read)

  // control for forwarding multiplexors
  wire [NUM_TAPS-1:0] use_forwarded_value = en_by_wr_only |                 // FWD_INPUT: tap[ptr_curr] <= ocbi_i (if write only)
                                            ({NUM_TAPS{wr_rd}} & ptr_prev); // FWD_INPUT: tap[ptr_prev] <= ocbi_i (if simultaneously write & read)


  // declare interconnection (one extra than taps number for input)
  wire [DATA_SIZE-1:0] ocb_bus[0:NUM_TAPS];

  // taps placement
  generate
  genvar k;
  for (k = 0; k < NUM_TAPS; k = k + 1) begin : tap_k
    ocb_tap
    #(
      .DATA_SIZE              (DATA_SIZE)
    )
    u_tap_k
    (
      .clk                    (clk),
      .reset                  (reset_taps),
      .default_value_i        (default_value_i),
      .push_i                 (push_taps[k]),
      .prev_tap_out_i         (ocb_bus[k+1]),
      .forwarded_value_i      (ocbi_i),
      .use_forwarded_value_i  (use_forwarded_value[k]),
      .out_o                  (ocb_bus[k])
    );
  end
  endgenerate

  // outputs assignement
  generate
  genvar m;
  for (m = 0; m < NUM_OUTS; m = m + 1) begin : out_m
    assign ocbo_o [(DATA_SIZE*(m+1)-1):(DATA_SIZE*m)] = ocb_bus[m];
  end
  endgenerate

  // and assign input of all queue:
  assign ocb_bus[NUM_TAPS] = default_value_i;

endmodule // mor1kx_ocb_marocchino
*/



//-----------------------------------------------------------------//
//       Order Control Buffer with "MISS" detection                //
//-----------------------------------------------------------------//
//   If input data is invalid (is_miss_i == 1'b1) the OCB goes to  //
// continously polling mode. It stays in the mode till resolving   //
// data "miss" i.e. till latching valid data.                      //
//   The module implemented separetaly from major OCB to avoid     //
// extra complexity in source code.                                //
//-----------------------------------------------------------------//

module mor1kx_ocb_miss_marocchino
#(
  parameter NUM_TAPS  = 8,
  parameter NUM_OUTS  = 1,
  parameter DATA_SIZE = 2,
  parameter FULL_FLAG = "NONE" // "ENABLED" / "NONE"
)
(
  // clocks, resets and other input controls
  input                  clk,
  input                  rst,
  // pipe controls
  input                  pipeline_flush_i, // flush pipe
  input                  write_i,
  input                  read_i,
  // value at reset/flush
  input                  reset_taps, // to default value; synchronous
  input  [DATA_SIZE-1:0] default_value_i,
  // data input
  input                  is_miss_i,
  input  [DATA_SIZE-1:0] ocbi_i,
  // "OCB is full" flag
  //   (a) external control logic must stop the "writing without reading"
  //       operation if OCB is full
  //   (b) however, the "writing + reading" is possible
  //       because it just pushes OCB and keeps it full
  output                 full_o,
  // output layout
  // { out[n-1], out[n-2], ... out[0] } : DECODE (entrance) -> EXECUTE (exit)
  output [DATA_SIZE*NUM_OUTS-1:0] ocbo_o
);

  // latched "miss" flag
  reg is_miss_r;

  // "pointers"
  reg   [NUM_TAPS:0] ptr_curr; // on current active tap, no miss
  reg [NUM_TAPS-1:0] ptr_prev; // on previous active tap, no miss

  // control to increment/decrement pointers
  // if miss then write continously, so no read only
  wire wr_only = ~read_i &  (write_i | is_miss_r);
  wire rd_only =  read_i & ~(write_i | is_miss_r);
  wire wr_rd   =  read_i &  (write_i | is_miss_r);

  // implementation latched "miss" flag
  always @(posedge clk) begin
    if (rst | pipeline_flush_i)
      is_miss_r <= 1'b0;
    else if (write_i | is_miss_r)
      is_miss_r <= is_miss_i;
  end // @clock

  // operation algorithm:
  //-----------------------------------------------------------------------------
  // read only    | push: tap[k-1] <= tap[k], tap[num_taps-1] <= reset_value;
  //              | update pointers: if(~ptr_prev_0) ptr_prev <= (ptr_prev >> 1);
  //              |                  if(~ptr_curr_0) ptr_curr <= (ptr_curr >> 1);
  //-----------------------------------------------------------------------------
  // write only   | tap[ptr_curr] <= ocbi_i
  //              | ptr_prev <= ptr_curr;
  //              | ptr_curr <= (ptr_curr << 1);
  //-----------------------------------------------------------------------------
  // read & write | push: tap[k-1] <= tap[k]
  //              |       tap[ptr_prev] <= ocbi_i;
  //-----------------------------------------------------------------------------

  wire ptrs_inc = wr_only; // try to increment pointers
  wire ptrs_dec = rd_only | (wr_rd & is_miss_r); // try to decrement pointers

  // "OCB is full" flag
  //  # no more availaible taps, pointer is out of range
  generate
  /* verilator lint_off WIDTH */
  if (FULL_FLAG != "NONE") begin : ocb_miss_full_flag_enabled
  /* verilator lint_on WIDTH */
    reg full_r;
    // ---
    always @(posedge clk) begin
      if (rst | pipeline_flush_i)
        full_r <= 1'b0;
      else if (ptrs_inc)
        full_r <= (|ptr_curr[NUM_TAPS:(NUM_TAPS-1)]) & (~is_miss_i);
      else if (ptrs_dec)
        full_r <= 1'b0;
    end // @clock
    // ---
    assign full_o = full_r;
  end
  else begin : ocb_miss_full_flag_disabled
    assign full_o = 1'b0;
  end
  endgenerate

  // update pointer on current tap
  always @(posedge clk) begin
    if (rst | pipeline_flush_i) begin
      ptr_prev <= {{(NUM_TAPS-1){1'b0}},1'b1};
      ptr_curr <= {{NUM_TAPS{1'b0}},1'b1};
    end
    else if (ptrs_inc) begin
      ptr_prev <= (is_miss_r ? ptr_prev :  ptr_curr[NUM_TAPS-1:0]);
      ptr_curr <= (is_miss_r ? ptr_curr : {ptr_curr[NUM_TAPS-1:0],1'b0});
    end
    else if (ptrs_dec) begin
      ptr_prev <= (ptr_prev[0] ? ptr_prev : {1'b0,ptr_prev[NUM_TAPS-1:1]});
      ptr_curr <= (ptr_curr[0] ? ptr_curr : {1'b0,ptr_curr[NUM_TAPS:1]});
    end
  end // @clock


  // enable by write only
  wire [NUM_TAPS-1:0] en_by_wr_only = {NUM_TAPS{wr_only}} & (is_miss_r ? ptr_prev : ptr_curr[NUM_TAPS-1:0]);

  // enable signals for taps
  wire [NUM_TAPS-1:0] push_taps = en_by_wr_only |     // PUSH_TAPS: tap[ptr_curr] <= ocbi_i (particular if write only)
                                  {NUM_TAPS{read_i}}; // PUSH_TAPS: tap[k-1] <= tap[k]      (all if a read)

  // use forwarding value for simultaneously write & read
  wire [NUM_TAPS-1:0] fw_by_wr_rd = {NUM_TAPS{wr_rd}} & (ptr_prev[0] ? ptr_prev : (is_miss_r ? {1'b0,ptr_prev[NUM_TAPS-1:1]} : ptr_prev));

  // control for forwarding multiplexors
  wire [NUM_TAPS-1:0] use_forwarded_value = en_by_wr_only | // FWD_INPUT: tap[ptr_curr] <= ocbi_i (if write only)
                                            fw_by_wr_rd;    // FWD_INPUT: tap[ptr_prev] <= ocbi_i (if simultaneously write & read)


  // declare interconnection (one extra than taps number for input)
  wire [DATA_SIZE-1:0] ocb_bus[0:NUM_TAPS];

  // taps placement
  generate
  genvar k;
  for (k = 0; k < NUM_TAPS; k = k + 1) begin : tap_k
    // taps
    ocb_tap
    #(
      .DATA_SIZE              (DATA_SIZE)
    )
    u_tap_k
    (
      .clk                    (clk),
      .reset                  (reset_taps),
      .default_value_i        (default_value_i),
      .push_i                 (push_taps[k]),
      .prev_tap_out_i         (ocb_bus[k+1]),
      .forwarded_value_i      (ocbi_i),
      .use_forwarded_value_i  (use_forwarded_value[k]),
      .out_o                  (ocb_bus[k])
    );
  end
  endgenerate

  // outputs assignement
  generate
  genvar m;
  for (m = 0; m < NUM_OUTS; m = m + 1) begin : out_m
    assign ocbo_o [(DATA_SIZE*(m+1)-1):(DATA_SIZE*m)] = ocb_bus[m];
  end
  endgenerate

  // and assign input of all queue:
  assign ocb_bus[NUM_TAPS] = default_value_i;

endmodule // mor1kx_ocb_miss_marocchino



//------------------------------------------------------//
// Order Control Buffer (RAM + REG)                     //
//------------------------------------------------------//
//   It based on combination of RAM and output register //
///  and quite similar to store buffer.                 //
//------------------------------------------------------//

module mor1kx_ocbuff_marocchino
#(
  parameter NUM_TAPS      = 8,  // range : 2 ... 32
  parameter DATA_WIDTH    = 2,
  parameter FULL_FLAG     = "NONE", // "ENABLED" / "NONE"
  parameter EMPTY_FLAG    = "NONE",  // "ENABLED" / "NONE"
  parameter CLEAR_ON_INIT = 0
)
(
  // clocks, resets
  input                         cpu_clk,
  input                         cpu_rst,
  // pipe controls
  input                         pipeline_flush_i, // flush controls
  input                         write_i,
  input                         read_i,
  // value at reset/flush
  input                         reset_ocbo_i, // logic for clean up output register
  // data input
  input      [(DATA_WIDTH-1):0] ocbi_i,
  // "OCB is empty" flag
  output                        empty_o,
  // "OCB is full" flag
  //   (a) external control logic must stop the "writing without reading"
  //       operation if OCB is full
  //   (b) however, the "writing + reading" is possible
  //       because it just pushes OCB and keeps it full
  output                        full_o,
  // output register
  output reg [(DATA_WIDTH-1):0] ocbo_o
);

  generate
  if ((NUM_TAPS < 2) || (NUM_TAPS > 32)) begin
    initial begin
      $display("OCB ERROR: Incorrect number of taps");
      $finish;
    end
  end
  endgenerate

  // Compute number of taps implemented in RAM
  // (one tap is output register)
  localparam NUM_RAM_TAPS = NUM_TAPS - 1;

  // Compute RAM address width (the approach avoids clog2 call)
  // (averall (with output register) taps number must be from 2 to 32)
  localparam RAM_AW = (NUM_RAM_TAPS > 16) ? 5 :
                      (NUM_RAM_TAPS >  8) ? 4 :
                      (NUM_RAM_TAPS >  4) ? 3 :
                      (NUM_RAM_TAPS >  2) ? 2 : 1;

  // size of counter of booked cells (the approach avoids clog2 call)
  //  - averall (with output register) taps number must be from 2 to 32
  //  - zero means "no booked cells", "buffer is empty"
  localparam BOOKED_CNT_SZ  = (NUM_TAPS > 31) ? 6 :
                              (NUM_TAPS > 15) ? 5 :
                              (NUM_TAPS >  7) ? 4 :
                              (NUM_TAPS >  3) ? 3 : 2;
  // for shorter notation
  localparam BOOKED_CNT_MSB = BOOKED_CNT_SZ - 1;

  // special points of counter of booked cells
  localparam [BOOKED_CNT_MSB:0] FIFO_EMPTY     = 0;
  localparam [BOOKED_CNT_MSB:0] BOOKED_OUT_REG = 1;
  localparam [BOOKED_CNT_MSB:0] BOOKED_OUT_RAM = 2;
  localparam [BOOKED_CNT_MSB:0] FIFO_FULL      = NUM_TAPS;


  // counter of booked cells
  reg  [BOOKED_CNT_MSB:0] booked_cnt_r;
  wire [BOOKED_CNT_MSB:0] booked_cnt_inc;
  wire [BOOKED_CNT_MSB:0] booked_cnt_dec;
  reg  [BOOKED_CNT_MSB:0] booked_cnt_nxt; // combinatorial
  // registered FIFO states (driven by counter of booked cells)
  reg                     booked_outreg_r; // output register is booked
  reg                     booked_outram_r; // FIFO-RAM outputs is valid
  reg                     booked_intram_r; // Internally cell in FIFO-RAM is booked


  // RAM_FIFO related
  // pointer for write
  reg      [(RAM_AW-1):0] write_pointer_r;
  wire     [(RAM_AW-1):0] write_pointer_inc;
  reg      [(RAM_AW-1):0] write_pointer_nxt; // combinatorial
  // pointer for read
  reg      [(RAM_AW-1):0] read_pointer_r;
  wire     [(RAM_AW-1):0] read_pointer_inc;
  reg      [(RAM_AW-1):0] read_pointer_nxt; // combinatorial
  // FIFO-RAM ports (combinatorial)
  reg                     rwp_en;   // "read / write" port enable
  reg                     rwp_we;   // "read / write" port writes
  reg      [(RAM_AW-1):0] rwp_addr;
  reg                     wp_en;    // "write only" port enabled
  // packed data
  wire [(DATA_WIDTH-1):0] ram_dout; // FIFO_RAM output


  // Output register related
  reg  [(DATA_WIDTH-1):0] ocbo_mux; // combinatorial


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
           ocbi_i          or ram_dout          or ocbo_o) begin
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
        ocbo_mux = ocbo_o;
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
        ocbo_mux = booked_outreg_r ? ocbo_o : ocbi_i;
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
        ocbo_mux = booked_outram_r ? ram_dout : {DATA_WIDTH{1'b0}};
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
        ocbo_mux = booked_outram_r ? ram_dout : ocbi_i;
      end // "read & write"
    endcase
  end


  // registering of new states
  always @(posedge cpu_clk) begin
    if (cpu_rst | pipeline_flush_i) begin
      // counter of booked cells
      booked_cnt_r    <= {BOOKED_CNT_SZ{1'b0}}; // reset / pipe flushing
      // registered FIFO states
      booked_outreg_r <= 1'b0; // reset / pipe flushing
      booked_outram_r <= 1'b0; // reset / pipe flushing
      booked_intram_r <= 1'b0; // reset / pipe flushing
      // write / read pointers
      write_pointer_r <= {RAM_AW{1'b0}}; // reset / pipe flushing
      read_pointer_r  <= {RAM_AW{1'b0}}; // reset / pipe flushing
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
    end
  end


  // "OCB is empty" flag
  generate
  /* verilator lint_off WIDTH */
  if (EMPTY_FLAG != "NONE") begin : ocb_empty_flag_enabled
  /* verilator lint_on WIDTH */

    reg    empty_r;
    assign empty_o = empty_r;
    // ---
    always @(posedge cpu_clk) begin
      if (cpu_rst | pipeline_flush_i)
        empty_r <= 1'b0; // reset / pipe flushing
      else 
        empty_r <= (booked_cnt_nxt == FIFO_EMPTY); // update
    end // cpu-clock

  end
  else begin : ocb_empty_flag_disabled

    assign empty_o = 1'b0;

  end
  endgenerate


  // "OCB is full" flag
  generate
  /* verilator lint_off WIDTH */
  if (FULL_FLAG != "NONE") begin : ocb_full_flag_enabled
  /* verilator lint_on WIDTH */

    reg    full_r;
    assign full_o = full_r;
    // ---
    always @(posedge cpu_clk) begin
      if (cpu_rst | pipeline_flush_i)
        full_r <= 1'b0; // reset / pipe flushing
      else 
        full_r <= (booked_cnt_nxt == FIFO_FULL); // update
    end // cpu-clock

  end
  else begin : ocb_full_flag_disabled

    assign full_o = 1'b0;

  end
  endgenerate


  // instance RAM as FIFO
  mor1kx_dpram_en_w1st_sclk
  #(
    .ADDR_WIDTH     (RAM_AW),
    .DATA_WIDTH     (DATA_WIDTH),
    .CLEAR_ON_INIT  (CLEAR_ON_INIT)
  )
  u_ocb_ram
  (
    // common clock
    .clk    (cpu_clk),
    // port "a": Read/Write
    .en_a   (rwp_en),
    .we_a   (rwp_we),
    .addr_a (rwp_addr),
    .din_a  (ocbi_i),
    .dout_a (ram_dout),
    // port "b": Write
    .en_b   (wp_en),
    .we_b   (1'b1),
    .addr_b (write_pointer_r),
    .din_b  (ocbi_i),
    .dout_b ()            // not used
  );

  // registered output
  always @(posedge cpu_clk) begin
    if (reset_ocbo_i)
      ocbo_o <= {DATA_WIDTH{1'b0}};
    else
      ocbo_o <= ocbo_mux;
  end // at clock

endmodule // mor1kx_store_buffer_marocchino



//---------------------------------//
// Reservation Station with 2 taps //
//---------------------------------//

module mor1kx_rsrvs_marocchino
#(
  parameter OPTION_OPERAND_WIDTH = 32,
  parameter OP_WIDTH             =  1, // width of command set
  parameter OPC_WIDTH            =  1, // width of additional attributes
  parameter DEST_EXT_ADDR_WIDTH  =  3, // log2(Order Control Buffer depth)
  // Reservation station is used for 1-clock execution module.
  // As 1-clock pushed if only it is granted by write-back access
  // all input operandes already forwarder. So we don't use
  // exec_op_o and we remove exra logic for it.
  parameter RSRVS_1CLK           =  1,
  // Reservation station is used for LSU.
  parameter RSRVS_LSU            =  0,
  // Reservation station is used for integer MUL/DIV.
  parameter RSRVS_MULDIV         =  0,
  // Reservation station is used for FPU3264.
  // Extra logic for the A2 and B2 related hazards is generated.
  parameter RSRVS_FPU            =  0,
  // Packed operands for various reservation stations:
  //  # LSU :   {   x,    x, rfb1, rfa1}
  //  # 1CLK:   {   x,    x, rfb1, rfa1}
  //  # MULDIV: {   x,    x, rfb1, rfa1}
  //  # FPU:    {rfb2, rfa2, rfb1, rfa1}
  parameter DCOD_RFXX_WIDTH      = 64, // (2 * OPTION_OPERAND_WIDTH) for LSU; etc...
  // OMAN-to-DECODE hazard flags layout for various reservation stations:
  //  # LSU :   {   x,    x,    x,     x,    x,    x,  dxb1, d2b1, d1b1,  dxa1, d2a1, d1a1 }
  //  # 1CLK:   {   x,    x,    x,     x,    x,    x,  dxb1, d2b1, d1b1,  dxa1, d2a1, d1a1 }
  //  # MULDIV: {   x,    x,    x,     x,    x,    x,  dxb1, d2b1, d1b1,  dxa1, d2a1, d1a1 }
  //  # FPU:    {dxb2, d2b2, d1b2,  dxa2, d2a2, d1a2,  dxb1, d2b1, d1b1,  dxa1, d2a1, d1a1 }
  parameter OMN2DEC_HAZARDS_FLAGS_WIDTH = 6, // 6: for 1CLK, MUL/DIV and LSU;  12: for FPU3264
  // OMAN-to-DECODE hazard id layout for various reservation stations:
  //  # LSU :   {   x,    x, dxb1, dxa1 }
  //  # 1CLK:   {   x,    x, dxb1, dxa1 }
  //  # MULDIV: {   x,    x, dxb1, dxa1 }
  //  # FPU:    {dxb2, dxa2, dxb1, dxa1 }
  parameter OMN2DEC_HAZARDS_ADDRS_WIDTH = 6  // (2 * DEST_EXT_ADDR_WIDTH) for LSU; etc...
)
(
  // clocks and resets
  input                                     cpu_clk,
  input                                     cpu_rst,

  // pipeline control signals
  input                                     pipeline_flush_i,
  input                                     padv_exec_i,
  input                                     taking_op_i,      // a unit is taking input for execution

  // input data from DECODE
  input             [(DCOD_RFXX_WIDTH-1):0] dcod_rfxx_i,

  // OMAN-to-DECODE hazards
  //  # hazards flags
  input [(OMN2DEC_HAZARDS_FLAGS_WIDTH-1):0] omn2dec_hazards_flags_i,
  //  # hasards addresses
  input [(OMN2DEC_HAZARDS_ADDRS_WIDTH-1):0] omn2dec_hazards_addrs_i,

  // Hazard could be resolving
  //  ## write-back attributes
  input         [(DEST_EXT_ADDR_WIDTH-1):0] wb_ext_bits_i,
  //  ## forwarding results
  input        [(OPTION_OPERAND_WIDTH-1):0] wb_result1_i,
  input        [(OPTION_OPERAND_WIDTH-1):0] wb_result2_i,

  // command and its additional attributes
  input                                     dcod_op_any_i,
  input                    [(OP_WIDTH-1):0] dcod_op_i,    // request the unit command
  input                   [(OPC_WIDTH-1):0] dcod_opc_i,   // additional attributes for command

  // outputs
  //   command and its additional attributes
  output                                    exec_op_any_o,
  output                   [(OP_WIDTH-1):0] exec_op_o,    // request the unit command
  output                  [(OPC_WIDTH-1):0] exec_opc_o,   // additional attributes for command
  //   operands
  output       [(OPTION_OPERAND_WIDTH-1):0] exec_rfa1_o,
  output       [(OPTION_OPERAND_WIDTH-1):0] exec_rfb1_o,
  //  ## for FPU3264
  output       [(OPTION_OPERAND_WIDTH-1):0] exec_rfa2_o,
  output       [(OPTION_OPERAND_WIDTH-1):0] exec_rfb2_o,
  //   unit-is-busy flag
  output                                    unit_free_o
);

  /**** parameters for fields extruction ****/

  //  # operands layouts for various reservation stations:
  //  # LSU : {   x,    x, rfb1, rfa1}
  //  # 1CLK: {   x,    x, rfb1, rfa1}
  //  # FPU3264: {rfb2, rfa2, rfb1, rfa1}
  //    A1
  localparam  RFA1_LSB = 0;
  localparam  RFA1_MSB = OPTION_OPERAND_WIDTH - 1;
  //    B1
  localparam  RFB1_LSB = OPTION_OPERAND_WIDTH;
  localparam  RFB1_MSB = 2 * OPTION_OPERAND_WIDTH - 1;
  //    A2
  localparam  RFA2_LSB = 2 * OPTION_OPERAND_WIDTH;
  localparam  RFA2_MSB = 3 * OPTION_OPERAND_WIDTH - 1;
  //    B2
  localparam  RFB2_LSB = 3 * OPTION_OPERAND_WIDTH;
  localparam  RFB2_MSB = 4 * OPTION_OPERAND_WIDTH - 1;

  // OMAN-to-DECODE hazard flags layout for various reservation stations:
  //  # LSU : {   x,    x,    x,     x,    x,    x,  dxb1, d2b1, d1b1,  dxa1, d2a1, d1a1 }
  //  # 1CLK: {   x,    x,    x,     x,    x,    x,  dxb1, d2b1, d1b1,  dxa1, d2a1, d1a1 }
  //  # FPU3264: {dxb2, d2b2, d1b2,  dxa2, d2a2, d1a2,  dxb1, d2b1, d1b1,  dxa1, d2a1, d1a1 }
  //  # relative operand A1
  localparam  HAZARD_D1A1_FLG_POS =  0;
  localparam  HAZARD_D2A1_FLG_POS =  1;
  localparam  HAZARD_DxA1_FLG_POS =  2;
  //  # relative operand B1
  localparam  HAZARD_D1B1_FLG_POS =  3;
  localparam  HAZARD_D2B1_FLG_POS =  4;
  localparam  HAZARD_DxB1_FLG_POS =  5;
  //  # relative operand A2
  localparam  HAZARD_D1A2_FLG_POS =  6;
  localparam  HAZARD_D2A2_FLG_POS =  7;
  localparam  HAZARD_DxA2_FLG_POS =  8;
  //  # relative operand B2
  localparam  HAZARD_D1B2_FLG_POS =  9;
  localparam  HAZARD_D2B2_FLG_POS = 10;
  localparam  HAZARD_DxB2_FLG_POS = 11;

  // OMAN-to-DECODE hazard id layout for various reservation stations:
  //  # LSU : {   x,    x, dxb1, dxa1 }
  //  # 1CLK: {   x,    x, dxb1, dxa1 }
  //  # FPU3264: {dxb2, dxa2, dxb1, dxa1 }
  //  # relative operand A1
  localparam  HAZARD_DxA1_ADR_LSB = 0;
  localparam  HAZARD_DxA1_ADR_MSB = DEST_EXT_ADDR_WIDTH - 1;
  //  # relative operand B1
  localparam  HAZARD_DxB1_ADR_LSB = DEST_EXT_ADDR_WIDTH;
  localparam  HAZARD_DxB1_ADR_MSB = 2 * DEST_EXT_ADDR_WIDTH - 1;
  //  # relative operand A2
  localparam  HAZARD_DxA2_ADR_LSB = 2 * DEST_EXT_ADDR_WIDTH;
  localparam  HAZARD_DxA2_ADR_MSB = 3 * DEST_EXT_ADDR_WIDTH - 1;
  //  # relative operand B2
  localparam  HAZARD_DxB2_ADR_LSB = 3 * DEST_EXT_ADDR_WIDTH;
  localparam  HAZARD_DxB2_ADR_MSB = 4 * DEST_EXT_ADDR_WIDTH - 1;


  // execute: command and attributes latches
  reg  [(OP_WIDTH-1):0] exec_op_r;
  reg                   exec_op_any_r;
  reg [(OPC_WIDTH-1):0] exec_opc_r;

  // all hazards are resolved
  wire exec_free_of_hazards;

  // local (extended) variant of taking-op-i
  wire taking_op_l = taking_op_i & exec_free_of_hazards;


  // DECODE->BUSY transfer
  wire dcod_pushing_busy = padv_exec_i & dcod_op_any_i  &   // DECODE pushing BUSY: Latch DECODE output ...
                           exec_op_any_r & (~taking_op_l);  // DECODE pushing BUSY: ... if EXECUTE is busy.

  // DECODE->EXECUTE transfer
  wire dcod_pushing_exec = padv_exec_i & dcod_op_any_i &    // DECODE pushing EXECUTE: New command ...
                           (~exec_op_any_r | taking_op_l);  // DECODE pushing EXECUTE: ... and unit is free.

  // BUSY->EXECUTE transfer
  wire busy_pushing_exec = busy_op_any_r & taking_op_l; // There is pending instruction and EXECUTE is free.



  /**** BUSY stage ****/


  // busy: command and additional attributes
  reg  [(OP_WIDTH-1):0] busy_op_r;
  reg                   busy_op_any_r;
  reg [(OPC_WIDTH-1):0] busy_opc_r;

  // latch command and its attributes
  always @(posedge cpu_clk) begin
    if (cpu_rst | pipeline_flush_i) begin
      busy_op_any_r <= 1'b0;
      busy_op_r     <= {OP_WIDTH{1'b0}};
      busy_opc_r    <= {OPC_WIDTH{1'b0}};
    end
    else if (dcod_pushing_busy) begin
      busy_op_any_r <= 1'b1;
      busy_op_r     <= dcod_op_i;
      busy_opc_r    <= dcod_opc_i;
    end
    else if (busy_pushing_exec) begin
      busy_op_any_r <= 1'b0;
      busy_op_r     <= {OP_WIDTH{1'b0}};
      busy_opc_r    <= {OPC_WIDTH{1'b0}};
    end
  end // @clock

  // output from busy stage
  //  ## unit-is-busy flag
  assign unit_free_o = (~busy_op_any_r);


  // busy: processing hazards wires (and regs) used across whole module
  // # common for all types of reservation station
  //  # relative operand A1
  reg                                 busy_hazard_d1a1_r;
  reg                                 busy_hazard_d2a1_r;
  reg                                 busy_hazard_dxa1_r;
  reg       [DEST_EXT_ADDR_WIDTH-1:0] busy_hazard_dxa1_adr_r;
  wire                                busy_dxa1_muxing_wb;
  //  # relative operand B1
  reg                                 busy_hazard_d1b1_r;
  reg                                 busy_hazard_d2b1_r;
  reg                                 busy_hazard_dxb1_r;
  reg       [DEST_EXT_ADDR_WIDTH-1:0] busy_hazard_dxb1_adr_r;
  wire                                busy_dxb1_muxing_wb;
  // # exclusively for FPU3264 reservation station
  //  # relative operand A2
  wire                                busy_hazard_d1a2_w;
  wire                                busy_hazard_d2a2_w;
  wire                                busy_hazard_dxa2_w;
  wire      [DEST_EXT_ADDR_WIDTH-1:0] busy_hazard_dxa2_adr_w;
  wire                                busy_dxa2_muxing_wb;
  //  # relative operand B2
  wire                                busy_hazard_d1b2_w;
  wire                                busy_hazard_d2b2_w;
  wire                                busy_hazard_dxb2_w;
  wire      [DEST_EXT_ADDR_WIDTH-1:0] busy_hazard_dxb2_adr_w;
  wire                                busy_dxb2_muxing_wb;

  // busy: operands
  //   ## registers for operands A & B
  reg      [OPTION_OPERAND_WIDTH-1:0] busy_rfa1_r;
  reg      [OPTION_OPERAND_WIDTH-1:0] busy_rfb1_r;
  //   ## multiplexed with forwarded value from WB
  wire     [OPTION_OPERAND_WIDTH-1:0] busy_rfa1;
  wire     [OPTION_OPERAND_WIDTH-1:0] busy_rfb1;
  wire     [OPTION_OPERAND_WIDTH-1:0] busy_rfa2_w; // makes sense in FPU3264 only
  wire     [OPTION_OPERAND_WIDTH-1:0] busy_rfb2_w; // makes sense in FPU3264 only

  // latches for common part
  //  # hazard flags
  always @(posedge cpu_clk) begin
    if (cpu_rst | pipeline_flush_i) begin
      //  # relative operand A1
      busy_hazard_d1a1_r <= 1'b0;
      busy_hazard_d2a1_r <= 1'b0;
      busy_hazard_dxa1_r <= 1'b0;
      //  # relative operand B1
      busy_hazard_d1b1_r <= 1'b0;
      busy_hazard_d2b1_r <= 1'b0;
      busy_hazard_dxb1_r <= 1'b0;
    end
    else if (dcod_pushing_busy) begin
      //  # relative operand A1
      busy_hazard_d1a1_r <= omn2dec_hazards_flags_i[HAZARD_D1A1_FLG_POS];
      busy_hazard_d2a1_r <= omn2dec_hazards_flags_i[HAZARD_D2A1_FLG_POS];
      busy_hazard_dxa1_r <= omn2dec_hazards_flags_i[HAZARD_DxA1_FLG_POS];
      //  # relative operand B1
      busy_hazard_d1b1_r <= omn2dec_hazards_flags_i[HAZARD_D1B1_FLG_POS];
      busy_hazard_d2b1_r <= omn2dec_hazards_flags_i[HAZARD_D2B1_FLG_POS];
      busy_hazard_dxb1_r <= omn2dec_hazards_flags_i[HAZARD_DxB1_FLG_POS];
    end
    else begin
      //  # relative operand A1
      if (busy_dxa1_muxing_wb | busy_pushing_exec) begin
        busy_hazard_d1a1_r <= 1'b0;
        busy_hazard_d2a1_r <= 1'b0;
        busy_hazard_dxa1_r <= 1'b0;
      end
      // d1b1 related
      if (busy_dxb1_muxing_wb | busy_pushing_exec) begin
        busy_hazard_d1b1_r <= 1'b0;
        busy_hazard_d2b1_r <= 1'b0;
        busy_hazard_dxb1_r <= 1'b0;
      end
    end
  end // @clock
  //  # hazard resolution extention bits
  //  # they make sence only with rized hazard flags
  always @(posedge cpu_clk) begin
    if (dcod_pushing_busy) begin
      busy_hazard_dxa1_adr_r <= omn2dec_hazards_addrs_i[HAZARD_DxA1_ADR_MSB:HAZARD_DxA1_ADR_LSB];
      busy_hazard_dxb1_adr_r <= omn2dec_hazards_addrs_i[HAZARD_DxB1_ADR_MSB:HAZARD_DxB1_ADR_LSB];
    end
  end // @cpu-clock

  // muxing write-back
  assign busy_dxa1_muxing_wb = busy_hazard_dxa1_r & (busy_hazard_dxa1_adr_r == wb_ext_bits_i);
  assign busy_dxb1_muxing_wb = busy_hazard_dxb1_r & (busy_hazard_dxb1_adr_r == wb_ext_bits_i);

  // forwarding operands A1 & B1
  always @(posedge cpu_clk) begin
    if (dcod_pushing_busy) begin
      busy_rfa1_r <= dcod_rfxx_i[RFA1_MSB:RFA1_LSB];
      busy_rfb1_r <= dcod_rfxx_i[RFB1_MSB:RFB1_LSB];
    end
    else begin
      // complete forwarding for operand A1
      if (busy_dxa1_muxing_wb) begin
        busy_rfa1_r <= busy_rfa1;
      end
      // complete forwarding for operand B1
      if (busy_dxb1_muxing_wb) begin
        busy_rfb1_r <= busy_rfb1;
      end
    end
  end // @clock
  //---
  //  operand A1
  assign busy_rfa1 = busy_hazard_d1a1_r ? wb_result1_i :
                     busy_hazard_d2a1_r ? wb_result2_i : busy_rfa1_r;
  //  operand B1
  assign busy_rfb1 = busy_hazard_d1b1_r ? wb_result1_i :
                     busy_hazard_d2b1_r ? wb_result2_i : busy_rfb1_r;


  // exclusive latches for FPU3264 reservation station
  generate
  /* verilator lint_off WIDTH */
  if (RSRVS_FPU == 1) begin : busy_fpxx_enabled
  /* verilator lint_on WIDTH */
    //  # relative operand A2
    reg                             busy_hazard_d1a2_r;
    reg                             busy_hazard_d2a2_r;
    reg                             busy_hazard_dxa2_r;
    reg   [DEST_EXT_ADDR_WIDTH-1:0] busy_hazard_dxa2_adr_r;
    //  # relative operand B2
    reg                             busy_hazard_d1b2_r;
    reg                             busy_hazard_d2b2_r;
    reg                             busy_hazard_dxb2_r;
    reg   [DEST_EXT_ADDR_WIDTH-1:0] busy_hazard_dxb2_adr_r;
    // ---
    always @(posedge cpu_clk) begin
      if (cpu_rst | pipeline_flush_i) begin
        //  # relative operand A2
        busy_hazard_d1a2_r <= 1'b0;
        busy_hazard_d2a2_r <= 1'b0;
        busy_hazard_dxa2_r <= 1'b0;
        //  # relative operand B2
        busy_hazard_d1b2_r <= 1'b0;
        busy_hazard_d2b2_r <= 1'b0;
        busy_hazard_dxb2_r <= 1'b0;
      end
      else if (dcod_pushing_busy) begin
        //  # relative operand A2
        busy_hazard_d1a2_r <= omn2dec_hazards_flags_i[HAZARD_D1A2_FLG_POS];
        busy_hazard_d2a2_r <= omn2dec_hazards_flags_i[HAZARD_D2A2_FLG_POS];
        busy_hazard_dxa2_r <= omn2dec_hazards_flags_i[HAZARD_DxA2_FLG_POS];
        //  # relative operand B2
        busy_hazard_d1b2_r <= omn2dec_hazards_flags_i[HAZARD_D1B2_FLG_POS];
        busy_hazard_d2b2_r <= omn2dec_hazards_flags_i[HAZARD_D2B2_FLG_POS];
        busy_hazard_dxb2_r <= omn2dec_hazards_flags_i[HAZARD_DxB2_FLG_POS];
      end
      else begin
        //  # relative operand A2
        if (busy_dxa2_muxing_wb | busy_pushing_exec) begin
          busy_hazard_d1a2_r <= 1'b0;
          busy_hazard_d2a2_r <= 1'b0;
          busy_hazard_dxa2_r <= 1'b0;
        end
        //  # relative operand B2
        if (busy_dxb2_muxing_wb | busy_pushing_exec) begin
          busy_hazard_d1b2_r <= 1'b0;
          busy_hazard_d2b2_r <= 1'b0;
          busy_hazard_dxb2_r <= 1'b0;
        end
      end
    end // @clock
    // ---
    always @(posedge cpu_clk) begin
      if (dcod_pushing_busy) begin
        busy_hazard_dxa2_adr_r <= omn2dec_hazards_addrs_i[HAZARD_DxA2_ADR_MSB:HAZARD_DxA2_ADR_LSB];
        busy_hazard_dxb2_adr_r <= omn2dec_hazards_addrs_i[HAZARD_DxB2_ADR_MSB:HAZARD_DxB2_ADR_LSB];
      end
    end
    // ---
    //  # relative operand A2
    assign busy_hazard_d1a2_w     = busy_hazard_d1a2_r;     // FPU3264
    assign busy_hazard_d2a2_w     = busy_hazard_d2a2_r;     // FPU3264
    assign busy_hazard_dxa2_w     = busy_hazard_dxa2_r;     // FPU3264
    assign busy_hazard_dxa2_adr_w = busy_hazard_dxa2_adr_r; // FPU3264
    assign busy_dxa2_muxing_wb    = busy_hazard_dxa2_r & (busy_hazard_dxa2_adr_r == wb_ext_bits_i);
    //  # relative operand B2
    assign busy_hazard_d1b2_w     = busy_hazard_d1b2_r;     // FPU3264
    assign busy_hazard_d2b2_w     = busy_hazard_d2b2_r;     // FPU3264
    assign busy_hazard_dxb2_w     = busy_hazard_dxb2_r;     // FPU3264
    assign busy_hazard_dxb2_adr_w = busy_hazard_dxb2_adr_r; // FPU3264
    assign busy_dxb2_muxing_wb    = busy_hazard_dxb2_r & (busy_hazard_dxb2_adr_r == wb_ext_bits_i);

    // A2 & B2 operands
    reg [OPTION_OPERAND_WIDTH-1:0] busy_rfa2_r;
    reg [OPTION_OPERAND_WIDTH-1:0] busy_rfb2_r;
    // ---
    always @(posedge cpu_clk) begin
      if (dcod_pushing_busy) begin
        busy_rfa2_r <= dcod_rfxx_i[RFA2_MSB:RFA2_LSB];
        busy_rfb2_r <= dcod_rfxx_i[RFB2_MSB:RFB2_LSB];
      end
      else begin
        // complete forwarding for operand A2
        if (busy_dxa2_muxing_wb) begin
          busy_rfa2_r <= busy_rfa2_w;
        end
        // complete forwarding for operand B2
        if (busy_dxb2_muxing_wb) begin
          busy_rfb2_r <= busy_rfb2_w;
        end
      end
    end // @clock
    // ---
    //  operand A2
    assign busy_rfa2_w = busy_hazard_d1a2_r ? wb_result1_i :
                         busy_hazard_d2a2_r ? wb_result2_i : busy_rfa2_r;
    //  operand B2
    assign busy_rfb2_w = busy_hazard_d1b2_r ? wb_result1_i :
                         busy_hazard_d2b2_r ? wb_result2_i : busy_rfb2_r;
  end
  else begin : busy_fpxx_disabled
    //  # relative operand A2
    assign busy_hazard_d1a2_w     = 1'b0; // not FPU3264
    assign busy_hazard_d2a2_w     = 1'b0; // not FPU3264
    assign busy_hazard_dxa2_w     = 1'b0; // not FPU3264
    assign busy_hazard_dxa2_adr_w = {DEST_EXT_ADDR_WIDTH{1'b0}}; // not FPU3264
    assign busy_dxa2_muxing_wb    = 1'b0; // not FPU3264
    //  # relative operand B2
    assign busy_hazard_d1b2_w     = 1'b0; // not FPU3264
    assign busy_hazard_d2b2_w     = 1'b0; // not FPU3264
    assign busy_hazard_dxb2_w     = 1'b0; // not FPU3264
    assign busy_hazard_dxb2_adr_w = {DEST_EXT_ADDR_WIDTH{1'b0}}; // not FPU3264
    assign busy_dxb2_muxing_wb    = 1'b0; // not FPU3264
    // operands
    assign busy_rfa2_w = {OPTION_OPERAND_WIDTH{1'b0}}; // not FPU3264
    assign busy_rfb2_w = {OPTION_OPERAND_WIDTH{1'b0}}; // not FPU3264
  end
  endgenerate // BUSY-FPU3264


  /**** EXECUTE stage latches ****/


  // execute: hazards for all reservation station types
  //  # relative operand A1
  reg                             exec_hazard_d1a1_r;
  reg                             exec_hazard_d2a1_r;
  reg                             exec_hazard_dxa1_r;
  reg   [DEST_EXT_ADDR_WIDTH-1:0] exec_hazard_dxa1_adr_r;
  wire                            exec_dxa1_muxing_wb;
  //  # relative operand B1
  reg                             exec_hazard_d1b1_r;
  reg                             exec_hazard_d2b1_r;
  reg                             exec_hazard_dxb1_r;
  reg   [DEST_EXT_ADDR_WIDTH-1:0] exec_hazard_dxb1_adr_r;
  wire                            exec_dxb1_muxing_wb;
  // # exclusively for FPU3264 reservation station
  //  # relative operand A2
  wire                            exec_hazard_dxa2_w;
  wire                            exec_dxa2_muxing_wb;
  //  # relative operand B2
  wire                            exec_hazard_dxb2_w;
  wire                            exec_dxb2_muxing_wb;

  // execute: operands
  //   ## registers
  reg  [OPTION_OPERAND_WIDTH-1:0] exec_rfa1_r;
  reg  [OPTION_OPERAND_WIDTH-1:0] exec_rfb1_r;
  //   ## multiplexed with forwarded value from WB
  wire [OPTION_OPERAND_WIDTH-1:0] exec_rfa1;
  wire [OPTION_OPERAND_WIDTH-1:0] exec_rfb1;
  //   ## for FPU3264
  wire [OPTION_OPERAND_WIDTH-1:0] exec_rfa2_w;
  wire [OPTION_OPERAND_WIDTH-1:0] exec_rfb2_w;


  // --- execute: command and attributes latches ---
  always @(posedge cpu_clk) begin
    if (cpu_rst | pipeline_flush_i) begin
      exec_op_any_r <= 1'b0;
      exec_op_r     <= {OP_WIDTH{1'b0}};
      exec_opc_r    <= {OPC_WIDTH{1'b0}};
    end
    else if (dcod_pushing_exec) begin
      exec_op_any_r <= 1'b1;
      exec_op_r     <= dcod_op_i;
      exec_opc_r    <= dcod_opc_i;
    end
    else if (busy_pushing_exec) begin
      exec_op_any_r <= busy_op_any_r;
      exec_op_r     <= busy_op_r;
      exec_opc_r    <= busy_opc_r;
    end
    else if (taking_op_l) begin
      exec_op_any_r <= 1'b0;
      exec_op_r     <= {OP_WIDTH{1'b0}};
      exec_opc_r    <= {OPC_WIDTH{1'b0}};
    end
  end // @clock

  // no more hazards in EXEC
  // when 1clk_exec granted wride-back access, all hazards are resolved
  assign exec_free_of_hazards = (RSRVS_1CLK == 1) ? 1'b1 :
                                  (((~exec_hazard_dxa1_r) | exec_dxa1_muxing_wb) &  // EXEC is hazadrs free
                                   ((~exec_hazard_dxb1_r) | exec_dxb1_muxing_wb) &  // EXEC is hazadrs free
                                   ((~exec_hazard_dxa2_w) | exec_dxa2_muxing_wb) &  // EXEC is hazadrs free
                                   ((~exec_hazard_dxb2_w) | exec_dxb2_muxing_wb));  // EXEC is hazadrs free

  // Commands to execution units
  assign exec_op_any_o = (RSRVS_LSU == 1) ? exec_op_any_r : 1'b0;
  // ---
  assign exec_op_o     = exec_op_r & {OP_WIDTH{exec_free_of_hazards}};
  // ---
  assign exec_opc_o    = exec_opc_r;

  // ---
  always @(posedge cpu_clk) begin
    if (cpu_rst | pipeline_flush_i) begin
      //  # relative operand A1
      exec_hazard_d1a1_r <= 1'b0;
      exec_hazard_d2a1_r <= 1'b0;
      exec_hazard_dxa1_r <= 1'b0;
      //  # relative operand B1
      exec_hazard_d1b1_r <= 1'b0;
      exec_hazard_d2b1_r <= 1'b0;
      exec_hazard_dxb1_r <= 1'b0;
    end
    else if (dcod_pushing_exec) begin
      //  # relative operand A1
      exec_hazard_d1a1_r <= omn2dec_hazards_flags_i[HAZARD_D1A1_FLG_POS];
      exec_hazard_d2a1_r <= omn2dec_hazards_flags_i[HAZARD_D2A1_FLG_POS];
      exec_hazard_dxa1_r <= omn2dec_hazards_flags_i[HAZARD_DxA1_FLG_POS];
      //  # relative operand B1
      exec_hazard_d1b1_r <= omn2dec_hazards_flags_i[HAZARD_D1B1_FLG_POS];
      exec_hazard_d2b1_r <= omn2dec_hazards_flags_i[HAZARD_D2B1_FLG_POS];
      exec_hazard_dxb1_r <= omn2dec_hazards_flags_i[HAZARD_DxB1_FLG_POS];
    end
    else if (busy_pushing_exec) begin
      //  # relative operand A1
      exec_hazard_d1a1_r <= busy_hazard_d1a1_r & (~busy_dxa1_muxing_wb);
      exec_hazard_d2a1_r <= busy_hazard_d2a1_r & (~busy_dxa1_muxing_wb);
      exec_hazard_dxa1_r <= busy_hazard_dxa1_r & (~busy_dxa1_muxing_wb);
      //  # relative operand B1
      exec_hazard_d1b1_r <= busy_hazard_d1b1_r & (~busy_dxb1_muxing_wb);
      exec_hazard_d2b1_r <= busy_hazard_d2b1_r & (~busy_dxb1_muxing_wb);
      exec_hazard_dxb1_r <= busy_hazard_dxb1_r & (~busy_dxb1_muxing_wb);
    end
    else begin
      //  # relative operand A1
      if (exec_dxa1_muxing_wb) begin
        exec_hazard_d1a1_r <= 1'b0;
        exec_hazard_d2a1_r <= 1'b0;
        exec_hazard_dxa1_r <= 1'b0;
      end
      //  # relative operand B1
      if (exec_dxb1_muxing_wb) begin
        exec_hazard_d1b1_r <= 1'b0;
        exec_hazard_d2b1_r <= 1'b0;
        exec_hazard_dxb1_r <= 1'b0;
      end
    end
  end // @clock

  // ---
  //  # hazard resolution extention bits
  //  # they make sence only with rized hazard flags
  always @(posedge cpu_clk) begin
    if (dcod_pushing_exec) begin
      exec_hazard_dxa1_adr_r <= omn2dec_hazards_addrs_i[HAZARD_DxA1_ADR_MSB:HAZARD_DxA1_ADR_LSB];
      exec_hazard_dxb1_adr_r <= omn2dec_hazards_addrs_i[HAZARD_DxB1_ADR_MSB:HAZARD_DxB1_ADR_LSB];
    end
    else if (busy_pushing_exec) begin
      exec_hazard_dxa1_adr_r <= busy_hazard_dxa1_adr_r;
      exec_hazard_dxb1_adr_r <= busy_hazard_dxb1_adr_r;
    end
  end // @cpu-clock

  // ---
  assign exec_dxa1_muxing_wb = exec_hazard_dxa1_r & (exec_hazard_dxa1_adr_r == wb_ext_bits_i);
  assign exec_dxb1_muxing_wb = exec_hazard_dxb1_r & (exec_hazard_dxb1_adr_r == wb_ext_bits_i);

  // ---
  always @(posedge cpu_clk) begin
    if (dcod_pushing_exec) begin
      exec_rfa1_r <= dcod_rfxx_i[RFA1_MSB:RFA1_LSB];
      exec_rfb1_r <= dcod_rfxx_i[RFB1_MSB:RFB1_LSB];
    end
    else if (busy_pushing_exec) begin
      exec_rfa1_r <= busy_rfa1;
      exec_rfb1_r <= busy_rfb1;
    end
    else begin
      // complete forwarding A1
      if (exec_dxa1_muxing_wb) begin
        exec_rfa1_r <= exec_rfa1;
      end
      // complete forwarding B1
      if (exec_dxb1_muxing_wb) begin
        exec_rfb1_r <= exec_rfb1;
      end
    end
  end // @clock
  // last forward (from WB)
  //  operand A1
  assign exec_rfa1 = exec_hazard_d1a1_r ? wb_result1_i :
                     exec_hazard_d2a1_r ? wb_result2_i : exec_rfa1_r;
  //  operand B1
  assign exec_rfb1 = exec_hazard_d1b1_r ? wb_result1_i :
                     exec_hazard_d2b1_r ? wb_result2_i : exec_rfb1_r;

  //  ## for FPU3264
  generate
  /* verilator lint_off WIDTH */
  if (RSRVS_FPU == 1) begin : exec_fpxx_enabled
  /* verilator lint_on WIDTH */
    // hazard flags and destination addresses
    //  # relative operand A2
    reg                           exec_hazard_d1a2_r;
    reg                           exec_hazard_d2a2_r;
    reg                           exec_hazard_dxa2_r;
    reg [DEST_EXT_ADDR_WIDTH-1:0] exec_hazard_dxa2_adr_r;
    //  # relative operand B2
    reg                           exec_hazard_d1b2_r;
    reg                           exec_hazard_d2b2_r;
    reg                           exec_hazard_dxb2_r;
    reg [DEST_EXT_ADDR_WIDTH-1:0] exec_hazard_dxb2_adr_r;
    // registers for operands A & B
    reg [OPTION_OPERAND_WIDTH-1:0] exec_rfa2_r;
    reg [OPTION_OPERAND_WIDTH-1:0] exec_rfb2_r;
    // ---
    always @(posedge cpu_clk) begin
      if (cpu_rst | pipeline_flush_i) begin
        //  # relative operand A2
        exec_hazard_d1a2_r <= 1'b0;
        exec_hazard_d2a2_r <= 1'b0;
        exec_hazard_dxa2_r <= 1'b0;
        //  # relative operand B2
        exec_hazard_d1b2_r <= 1'b0;
        exec_hazard_d2b2_r <= 1'b0;
        exec_hazard_dxb2_r <= 1'b0;
      end
      else if (dcod_pushing_exec) begin
        //  # relative operand A2
        exec_hazard_d1a2_r <= omn2dec_hazards_flags_i[HAZARD_D1A2_FLG_POS];
        exec_hazard_d2a2_r <= omn2dec_hazards_flags_i[HAZARD_D2A2_FLG_POS];
        exec_hazard_dxa2_r <= omn2dec_hazards_flags_i[HAZARD_DxA2_FLG_POS];
        //  # relative operand B2
        exec_hazard_d1b2_r <= omn2dec_hazards_flags_i[HAZARD_D1B2_FLG_POS];
        exec_hazard_d2b2_r <= omn2dec_hazards_flags_i[HAZARD_D2B2_FLG_POS];
        exec_hazard_dxb2_r <= omn2dec_hazards_flags_i[HAZARD_DxB2_FLG_POS];
      end
      else if (busy_pushing_exec) begin
        //  # relative operand A2
        exec_hazard_d1a2_r <= busy_hazard_d1a2_w & (~busy_dxa2_muxing_wb);
        exec_hazard_d2a2_r <= busy_hazard_d2a2_w & (~busy_dxa2_muxing_wb);
        exec_hazard_dxa2_r <= busy_hazard_dxa2_w & (~busy_dxa2_muxing_wb);
        //  # relative operand B2
        exec_hazard_d1b2_r <= busy_hazard_d1b2_w & (~busy_dxb2_muxing_wb);
        exec_hazard_d2b2_r <= busy_hazard_d2b2_w & (~busy_dxb2_muxing_wb);
        exec_hazard_dxb2_r <= busy_hazard_dxb2_w & (~busy_dxb2_muxing_wb);
      end
      else begin
        //  # relative operand A2
        if (exec_dxa2_muxing_wb) begin
          exec_hazard_d1a2_r <= 1'b0;
          exec_hazard_d2a2_r <= 1'b0;
          exec_hazard_dxa2_r <= 1'b0;
        end
        //  # relative operand B2
        if (exec_dxb2_muxing_wb) begin
          exec_hazard_d1b2_r <= 1'b0;
          exec_hazard_d2b2_r <= 1'b0;
          exec_hazard_dxb2_r <= 1'b0;
        end
      end
    end // @clock
    // ---
    always @(posedge cpu_clk) begin
      if (dcod_pushing_exec) begin
        exec_hazard_dxa2_adr_r <= omn2dec_hazards_addrs_i[HAZARD_DxA2_ADR_MSB:HAZARD_DxA2_ADR_LSB];
        exec_hazard_dxb2_adr_r <= omn2dec_hazards_addrs_i[HAZARD_DxB2_ADR_MSB:HAZARD_DxB2_ADR_LSB];
      end
      else if (busy_pushing_exec) begin
        exec_hazard_dxa2_adr_r <= busy_hazard_dxa2_adr_w;
        exec_hazard_dxb2_adr_r <= busy_hazard_dxb2_adr_w;
      end
    end
    // ---
    //  # relative operand A2
    assign exec_hazard_dxa2_w  = exec_hazard_dxa2_r; // FPU3264
    assign exec_dxa2_muxing_wb = exec_hazard_dxa2_r & (exec_hazard_dxa2_adr_r == wb_ext_bits_i);
    //  # relative operand B2
    assign exec_hazard_dxb2_w  = exec_hazard_dxb2_r; // FPU3264
    assign exec_dxb2_muxing_wb = exec_hazard_dxb2_r & (exec_hazard_dxb2_adr_r == wb_ext_bits_i);
    // ---
    always @(posedge cpu_clk) begin
      if (dcod_pushing_exec) begin
        exec_rfa2_r <= dcod_rfxx_i[RFA2_MSB:RFA2_LSB];
        exec_rfb2_r <= dcod_rfxx_i[RFB2_MSB:RFB2_LSB];
      end
      else if (busy_pushing_exec) begin
        exec_rfa2_r <= busy_rfa2_w;
        exec_rfb2_r <= busy_rfb2_w;
      end
      else begin
        // complete forwadring A2
        if (exec_dxa2_muxing_wb) begin
          exec_rfa2_r <= exec_rfa2_w;
        end
        // complete forwadring B2
        if (exec_dxb2_muxing_wb) begin
          exec_rfb2_r <= exec_rfb2_w;
        end
      end
    end // @clock
    // last forward (from WB)
    //  operand A2
    assign exec_rfa2_w = exec_hazard_d1a2_r ? wb_result1_i :
                         exec_hazard_d2a2_r ? wb_result2_i : exec_rfa2_r;
    //  operand B2
    assign exec_rfb2_w = exec_hazard_d1b2_r ? wb_result1_i :
                         exec_hazard_d2b2_r ? wb_result2_i : exec_rfb2_r;
  end
  else begin : exec_fpxx_disabled
    assign exec_rfa2_w  = {OPTION_OPERAND_WIDTH{1'b0}}; // not FPU3264
    assign exec_rfb2_w  = {OPTION_OPERAND_WIDTH{1'b0}}; // not FPU3264
    // # exclusively for FPU3264 reservation station
    //  # relative operand A2
    assign exec_hazard_dxa2_w  = 1'b0; // not FPU3264
    assign exec_dxa2_muxing_wb = 1'b0; // not FPU3264
    //  # relative operand B2
    assign exec_hazard_dxb2_w  = 1'b0; // not FPU3264
    assign exec_dxb2_muxing_wb = 1'b0; // not FPU3264
  end
  endgenerate // EXEC-FPU3264

  // outputs
  //   operands
  assign exec_rfa1_o = exec_rfa1;
  assign exec_rfb1_o = exec_rfb1;
  //   for FPU3264
  assign exec_rfa2_o = exec_rfa2_w;
  assign exec_rfb2_o = exec_rfb2_w;

endmodule // mor1kx_rsrvs_marocchino
