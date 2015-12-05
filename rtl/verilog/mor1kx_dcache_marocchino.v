////////////////////////////////////////////////////////////////////////
//                                                                    //
//  mor1kx_icache_marocchino                                          //
//                                                                    //
//  Description: Data CACHE implementation                            //
//               The variant is tightly coupled with                  //
//               MAROCCHINO LSU and DMMU                              //
//               (based on mor1kx_dcache)                             //
//                                                                    //
////////////////////////////////////////////////////////////////////////
//                                                                    //
//   Copyright (C) 2012-2013 Stefan Kristiansson                      //
//                           stefan.kristiansson@saunalahti.fi        //
//                                                                    //
//                           Stefan Wallentowitz                      //
//                           stefan.wallentowitz@tum.de               //
//                                                                    //
//   Copyright (C) 2015 Andrey Bacherov                               //
//                      avbacherov@opencores.org                      //
//                                                                    //
//      This Source Code Form is subject to the terms of the          //
//      Open Hardware Description License, v. 1.0. If a copy          //
//      of the OHDL was not distributed with this file, You           //
//      can obtain one at http://juliusbaxter.net/ohdl/ohdl.txt       //
//                                                                    //
////////////////////////////////////////////////////////////////////////

`include "mor1kx-defines.v"

module mor1kx_dcache_marocchino
#(
  parameter OPTION_OPERAND_WIDTH      = 32,
  parameter OPTION_DCACHE_BLOCK_WIDTH =  5,
  parameter OPTION_DCACHE_SET_WIDTH   =  9,
  parameter OPTION_DCACHE_WAYS        =  2,
  parameter OPTION_DCACHE_LIMIT_WIDTH = 32,
  parameter OPTION_DCACHE_SNOOP       = "NONE"
)
(
  // clock & reset
  input                                 clk,
  input                                 rst,

  // configuration
  input                                 dc_enable_i,

  // exceptions
  input                                 dc_dbus_err_i,

  // Regular operation
  input                                 dc_access_i,
  input                                 dc_req_i,
  input                                 dc_we_i,
  input                           [3:0] dbus_bsel_i,
  input      [OPTION_OPERAND_WIDTH-1:0] dbus_sdat_i,
  input      [OPTION_OPERAND_WIDTH-1:0] virt_addr_i,
  input      [OPTION_OPERAND_WIDTH-1:0] phys_addr_cmd_i,
  output                                dc_ack_o,
  output reg [OPTION_OPERAND_WIDTH-1:0] dc_dat_o,

  // re-fill
  output                                refill_req_o,
  input                                 refill_allowed_i,
  output                                refill_o,
  output     [OPTION_OPERAND_WIDTH-1:0] next_refill_adr_o,
  output                                refill_last_o,
  input      [OPTION_OPERAND_WIDTH-1:0] wrdat_i,
  input                                 we_i,

  // SNOOP
  // Snoop address
  input      [OPTION_OPERAND_WIDTH-1:0] snoop_adr_i,
  // Snoop event in this cycle
  input                                 snoop_valid_i,
  // Whether the snoop hit. If so, there will be no tag memory write
  // this cycle. The LSU may need to stall the pipeline.
  output                                snoop_hit_o,

  // SPR interface
  input                          [15:0] spr_bus_addr_i,
  input                                 spr_bus_we_i,
  input                                 spr_bus_stb_i,
  input      [OPTION_OPERAND_WIDTH-1:0] spr_bus_dat_i,
  output     [OPTION_OPERAND_WIDTH-1:0] spr_bus_dat_o,
  output                                spr_bus_ack_o
);

  // Address space in bytes for a way
  localparam WAY_WIDTH = OPTION_DCACHE_BLOCK_WIDTH + OPTION_DCACHE_SET_WIDTH;
  /*
   * Tag memory layout
   *            +---------------------------------------------------------+
   * (index) -> | LRU | wayN valid | wayN tag |...| way0 valid | way0 tag |
   *            +---------------------------------------------------------+
   */

  // The tag is the part left of the index
  localparam TAG_WIDTH = (OPTION_DCACHE_LIMIT_WIDTH - WAY_WIDTH);

  // The tag memory contains entries with OPTION_DCACHE_WAYS parts of
  // each TAGMEM_WAY_WIDTH. Each of those is tag and a valid flag.
  localparam TAGMEM_WAY_WIDTH = TAG_WIDTH + 1;
  localparam TAGMEM_WAY_VALID = TAGMEM_WAY_WIDTH - 1;

  // Additionally, the tag memory entry contains an LRU value. The
  // width of this is 0 for OPTION_DCACHE_LIMIT_WIDTH==1
  localparam TAG_LRU_WIDTH = OPTION_DCACHE_WAYS*(OPTION_DCACHE_WAYS-1) >> 1;

  // We have signals for the LRU which are not used for one way
  // caches. To avoid signal width [-1:0] this generates [0:0]
  // vectors for them, which are removed automatically then.
  localparam TAG_LRU_WIDTH_BITS = (OPTION_DCACHE_WAYS >= 2) ? TAG_LRU_WIDTH : 1;

  // Compute the total sum of the entry elements
  localparam TAGMEM_WIDTH = TAGMEM_WAY_WIDTH * OPTION_DCACHE_WAYS + TAG_LRU_WIDTH;

  // For convenience we define the position of the LRU in the tag
  // memory entries
  localparam TAG_LRU_MSB = TAGMEM_WIDTH - 1;
  localparam TAG_LRU_LSB = TAG_LRU_MSB - TAG_LRU_WIDTH + 1;


  // States
  localparam IDLE       = 5'b00001;
  localparam READ       = 5'b00010;
  localparam WRITE      = 5'b00100;
  localparam REFILL     = 5'b01000;
  localparam INVALIDATE = 5'b10000;
  // FSM state register
  reg [4:0] state;
  // FSM state signals
  wire   read      = (state == READ);
  wire   write     = (state == WRITE);
  assign refill_o  = (state == REFILL);


  wire                                          invalidate;
  reg   [WAY_WIDTH-1:OPTION_DCACHE_BLOCK_WIDTH] invalidate_adr;

  reg                [OPTION_OPERAND_WIDTH-1:0] way_wr_dat;

  reg                [OPTION_OPERAND_WIDTH-1:0] curr_refill_adr;
  reg                                           refill_hit_r;
  reg  [(1<<(OPTION_DCACHE_BLOCK_WIDTH-2))-1:0] refill_done;

  // The index we read and write from tag memory
  wire [OPTION_DCACHE_SET_WIDTH-1:0] tag_rindex;
  reg  [OPTION_DCACHE_SET_WIDTH-1:0] tag_windex;

  // The data from the tag memory
  wire       [TAGMEM_WIDTH-1:0] tag_dout;
  wire [TAG_LRU_WIDTH_BITS-1:0] tag_lru_out;
  wire   [TAGMEM_WAY_WIDTH-1:0] tag_way_out [OPTION_DCACHE_WAYS-1:0];

  // The data to the tag memory
  wire      [TAGMEM_WIDTH-1:0] tag_din;
  reg [TAG_LRU_WIDTH_BITS-1:0] tag_lru_in;
  reg   [TAGMEM_WAY_WIDTH-1:0] tag_way_in [OPTION_DCACHE_WAYS-1:0];

  reg   [TAGMEM_WAY_WIDTH-1:0] tag_way_save[OPTION_DCACHE_WAYS-1:0];

  // Whether to write to the tag memory in this cycle
  reg                          tag_we;

  // WAYs related
  wire [OPTION_OPERAND_WIDTH-1:0] way_dout[OPTION_DCACHE_WAYS-1:0];
  reg    [OPTION_DCACHE_WAYS-1:0] way_we;

  // Does any way hit?
  wire                          hit;
  wire [OPTION_DCACHE_WAYS-1:0] way_hit;

  // This is the least recently used value before access the memory.
  // Those are one hot encoded.
  wire [OPTION_DCACHE_WAYS-1:0] lru;

  // Register that stores the LRU value from lru
  reg  [OPTION_DCACHE_WAYS-1:0] tag_save_lru;

  // The access vector to update the LRU history is the way that has
  // a hit or is refilled. It is also one-hot encoded.
  reg [OPTION_DCACHE_WAYS-1:0] access;

  // The current LRU history as read from tag memory and the update
  // value after we accessed it to write back to tag memory.
  wire [TAG_LRU_WIDTH_BITS-1:0] current_lru_history;
  wire [TAG_LRU_WIDTH_BITS-1:0] next_lru_history;


  reg                  write_pending;

  // Extract index to read from snooped address
  wire [OPTION_DCACHE_SET_WIDTH-1:0] snoop_index;
  assign snoop_index = snoop_adr_i[WAY_WIDTH-1:OPTION_DCACHE_BLOCK_WIDTH];

  // Register that is high one cycle after the actual snoop event to
  // drive the comparison
  reg                               snoop_check;
  // Register that stores the tag for one cycle
  reg               [TAG_WIDTH-1:0] snoop_tag;
  // Also store the index for one cycle, for the succeeding write access
  reg [OPTION_DCACHE_SET_WIDTH-1:0] snoop_windex;

  // Snoop tag memory interface
  // Data out of tag memory
  wire       [TAGMEM_WIDTH-1:0] snoop_dout;
  // Each ways information in the tag memory
  wire   [TAGMEM_WAY_WIDTH-1:0] snoop_way_out [OPTION_DCACHE_WAYS-1:0];
  // Whether the way hits
  wire [OPTION_DCACHE_WAYS-1:0] snoop_way_hit;


  genvar                        i;


  assign dc_ack_o = ((read & hit & ~write_pending) | refill_hit_r) &
                    dc_req_i & ~snoop_hit_o;

  assign tag_rindex = virt_addr_i[WAY_WIDTH-1:OPTION_DCACHE_BLOCK_WIDTH];


  generate
  if (OPTION_DCACHE_WAYS >= 2) begin
    // Multiplex the LRU history from and to tag memory
    assign current_lru_history = tag_dout[TAG_LRU_MSB:TAG_LRU_LSB];
    assign tag_din[TAG_LRU_MSB:TAG_LRU_LSB] = tag_lru_in;
    assign tag_lru_out = tag_dout[TAG_LRU_MSB:TAG_LRU_LSB];
  end

  for (i = 0; i < OPTION_DCACHE_WAYS; i=i+1) begin : ways
    // Multiplex the way entries in the tag memory
    assign tag_din[(i+1)*TAGMEM_WAY_WIDTH-1:i*TAGMEM_WAY_WIDTH] = tag_way_in[i];
    assign tag_way_out[i] = tag_dout[(i+1)*TAGMEM_WAY_WIDTH-1:i*TAGMEM_WAY_WIDTH];

    // compare stored tag with incoming tag and check valid bit
    assign way_hit[i] = tag_way_out[i][TAGMEM_WAY_VALID] &
      (tag_way_out[i][TAG_WIDTH-1:0] ==
       phys_addr_cmd_i[OPTION_DCACHE_LIMIT_WIDTH-1:WAY_WIDTH]);

    // The same for the snoop tag memory
    if (OPTION_DCACHE_SNOOP != "NONE") begin
      assign snoop_way_out[i] = snoop_dout[(i+1)*TAGMEM_WAY_WIDTH-1:i*TAGMEM_WAY_WIDTH];
      // compare stored tag with incoming tag and check valid bit
      assign snoop_way_hit[i] = snoop_way_out[i][TAGMEM_WAY_VALID] &
        (snoop_way_out[i][TAG_WIDTH-1:0] == snoop_tag);
    end // DCACHE snoop
    else begin
      assign snoop_way_hit[i] = 1'b0; // no snoop
      assign snoop_way_out[i] = {TAGMEM_WAY_WIDTH{1'b0}}; // no snoop
    end
  end // loop by ways
  endgenerate

  assign hit = |way_hit;

  assign snoop_hit_o = (|snoop_way_hit) & snoop_check;

  integer w0;
  always @(*) begin
    dc_dat_o = {OPTION_OPERAND_WIDTH{1'b0}};
    // Put correct way on the data port
    for (w0 = 0; w0 < OPTION_DCACHE_WAYS; w0 = w0 + 1) begin
      if (way_hit[w0] | (refill_hit_r & tag_save_lru[w0])) begin
        dc_dat_o = way_dout[w0];
      end
    end
  end

  assign next_refill_adr_o = (OPTION_DCACHE_BLOCK_WIDTH == 5) ?
    {curr_refill_adr[31:5], curr_refill_adr[4:0] + 5'd4} : // 32 byte = (8 words x 32 bits/word) = (4 words x 64 bits/word)
    {curr_refill_adr[31:4], curr_refill_adr[3:0] + 4'd4};  // 16 byte = (4 words x 32 bits/word) = (2 words x 64 bits/word)

  assign refill_last_o = refill_done[next_refill_adr_o[OPTION_DCACHE_BLOCK_WIDTH-1:2]];

  assign refill_req_o = (read & dc_req_i & ~hit & ~write_pending & refill_allowed_i) | refill_o;

  /*
   * SPR bus interface
   */

  // The SPR interface is used to invalidate the cache blocks. When
  // an invalidation is started, the respective entry in the tag
  // memory is cleared. When another transfer is in progress, the
  // handling is delayed until it is possible to serve it.
  //
  // The invalidation is acknowledged to the SPR bus, but the cycle
  // is terminated by the core. We therefore need to hold the
  // invalidate acknowledgement. Meanwhile we continuously write the
  // tag memory which is no problem.

  // Net that signals an acknowledgement
  reg invalidate_ack;

   // An invalidate request is either a block flush or a block invalidate
   assign invalidate = spr_bus_stb_i & spr_bus_we_i &
          ((spr_bus_addr_i == `OR1K_SPR_DCBFR_ADDR) |
           (spr_bus_addr_i == `OR1K_SPR_DCBIR_ADDR));

  // Acknowledge to the SPR bus.
  assign spr_bus_ack_o = invalidate_ack;

  /*
   * Cache FSM
   * Starts in IDLE.
   * State changes between READ and WRITE happens dc_we_i is asserted or not.
   * dc_we_i is in sync with virt_addr_i, so that means that it's the
   * *upcoming* write that it is indicating. It only toggles for one cycle,
   * so if we are busy doing something else when this signal comes
   * (i.e. refilling) we assert the write_pending signal.
   * dc_req_i is in sync with phys_addr_cmd_i, so it can be used to
   * determined if a cache hit should cause a re-fill or if a write should
   * really be executed.
   */
  integer w1;
  always @(posedge clk `OR_ASYNC_RST) begin
    if (rst) begin
      state           <= IDLE;  // on reset
      write_pending   <= 1'b0;  // on reset
      curr_refill_adr <= {OPTION_OPERAND_WIDTH{1'b0}};  // on reset
      refill_hit_r    <= 1'b0;  // on reset
      refill_done     <= 0;     // on reset
      snoop_check     <= 1'b0;  // on reset
      snoop_tag       <= {TAG_WIDTH{1'b0}}; // on reset
      snoop_windex    <= {OPTION_DCACHE_SET_WIDTH{1'b0}}; // on reset
    end
    else if(dc_dbus_err_i) begin
      state         <= IDLE;  // on exceptions
      write_pending <= 1'b0;  // on exceptions
      refill_hit_r  <= 1'b0;  // on exceptions
      refill_done   <= 0;     // on exceptions
      snoop_check   <= 1'b0;  // on exceptions
      snoop_tag     <= {TAG_WIDTH{1'b0}}; // on exceptions
      snoop_windex  <= {OPTION_DCACHE_SET_WIDTH{1'b0}}; // on exceptions
    end
    else begin
      if (dc_we_i)
        write_pending <= 1'b1;
      else if (~dc_req_i)
        write_pending <= 1'b0;

      if (snoop_valid_i) begin
        //
        // If there is a snoop event, we need to store this
        // information. This happens independent of whether we
        // have a snoop tag memory or not.
        //
        snoop_check  <= 1'b1;
        snoop_windex <= snoop_index;
        snoop_tag    <= snoop_adr_i[OPTION_DCACHE_LIMIT_WIDTH-1:WAY_WIDTH];
      end
      else begin
        snoop_check  <= 1'b0;
      end

      case (state)
        IDLE: begin
          if (invalidate) begin
            // If there is an invalidation request
            //
            // Store address in invalidate_adr that is muxed to the tag
            // memory write address
            invalidate_adr <= spr_bus_dat_i[WAY_WIDTH-1:OPTION_DCACHE_BLOCK_WIDTH];
            // Change to invalidate state that actually accesses
            // the tag memory
            state <= INVALIDATE;
          end
          else if (dc_we_i | write_pending)
            state <= WRITE;
          else if (dc_req_i)
            state <= READ;
        end

        READ: begin
          if ((dc_access_i | dc_we_i) & dc_enable_i) begin
            if (~hit & dc_req_i & ~write_pending & refill_allowed_i) begin
              // Store the LRU information for correct replacement
              // on re-fill. Always one when only one way.
              tag_save_lru <= (OPTION_DCACHE_WAYS==1) | lru;
              // store tag state
              for (w1 = 0; w1 < OPTION_DCACHE_WAYS; w1 = w1 + 1) begin
                tag_way_save[w1] <= tag_way_out[w1];
              end
              // 1st re-fill addrress
              curr_refill_adr <= phys_addr_cmd_i;
              // to RE-FILL
              state <= REFILL;
            end
            else if (dc_we_i | write_pending) begin
              state <= WRITE;
            end
            else if (invalidate) begin
              state <= IDLE;
            end
          end
          else if (~dc_enable_i | invalidate) begin
            state <= IDLE;
          end
        end

        REFILL: begin
          refill_hit_r <= 1'b0;
          // Abort re-fill on snoop-hit
          // TODO: only abort on snoop-hits to re-fill address
          if (snoop_hit_o) begin
            refill_done <= 0;     // on snoop-hit
            state       <= IDLE;  // on snoop-hit
          end
          else if (we_i) begin
            if (refill_last_o) begin
              refill_done <= 0;     // on last re-fill
              state       <= IDLE;  // on last re-fill
            end
            else begin
              refill_hit_r <= (refill_done == 0); // 1st re-fill is requested insn
              refill_done[curr_refill_adr[OPTION_DCACHE_BLOCK_WIDTH-1:2]] <= 1'b1; // current re-fill
              curr_refill_adr <= next_refill_adr_o;
            end // last or regulat
          end // snoop-hit / we
        end // re-fill
  
        WRITE: begin
          if ((~dc_req_i | ~dc_we_i) & ~snoop_hit_o) begin
            write_pending <= 1'b0;
            state         <= READ;
          end
        end
  
        INVALIDATE: begin
          if (invalidate) begin
            // Store address in invalidate_adr that is muxed to the tag
            // memory write address
            invalidate_adr <= spr_bus_dat_i[WAY_WIDTH-1:OPTION_DCACHE_BLOCK_WIDTH];
  
            state <= INVALIDATE;
          end
          else begin
            state <= IDLE;
          end
        end
  
        default:
          state <= IDLE;
      endcase
    end
  end

  //
  // This is the combinational part of the state machine that
  // interfaces the tag and way memories.
  //
  integer w2;
  always @(*) begin
    // Default is to keep data, don't write and don't access
    tag_lru_in = tag_lru_out;
    for (w2 = 0; w2 < OPTION_DCACHE_WAYS; w2 = w2 + 1) begin
      tag_way_in[w2] = tag_way_out[w2];
    end

    tag_we = 1'b0;
    way_we = {(OPTION_DCACHE_WAYS){1'b0}};

    access = {(OPTION_DCACHE_WAYS){1'b0}};

    way_wr_dat = wrdat_i;

    // The default is (of course) not to acknowledge the invalidate
    invalidate_ack = 1'b0;

    if (snoop_hit_o) begin
    // This is the write access
      tag_we = 1'b1;
      tag_windex = snoop_windex;
      for (w2 = 0; w2 < OPTION_DCACHE_WAYS; w2 = w2 + 1) begin
        if (snoop_way_hit[w2]) begin
          tag_way_in[w2] = 0;
        end
        else begin
          tag_way_in[w2] = snoop_way_out[w2];
        end
      end
    end
    else begin
      //
      // The tag mem is written during reads and writes to write
      // the lru info and  during re-fill and invalidate.
      //
      tag_windex =
        (read | write)        ? phys_addr_cmd_i[WAY_WIDTH-1:OPTION_DCACHE_BLOCK_WIDTH] :
        (state == INVALIDATE) ? invalidate_adr :
                                curr_refill_adr[WAY_WIDTH-1:OPTION_DCACHE_BLOCK_WIDTH];   // at re-fill

      case (state)
        IDLE: begin
          //
          // When idle we can always acknowledge the invalidate as it
          // has the highest priority in handling. When something is
          // changed on the state machine handling above this needs
          // to be changed.
          //
          invalidate_ack = 1'b1;
        end

        READ: begin
          if (hit) begin
            //
            // We got a hit. The LRU module gets the access
            // information. Depending on this we update the LRU
            // history in the tag.
            //
            access = way_hit;

            // This is the updated LRU history after hit
            tag_lru_in = next_lru_history;

            tag_we = 1'b1;
          end
        end

        WRITE: begin
          way_wr_dat = dbus_sdat_i;
          if (hit & dc_req_i) begin
            /* Mux cache output with write data */
            if (~dbus_bsel_i[3])
              way_wr_dat[31:24] = dc_dat_o[31:24];
            if (~dbus_bsel_i[2])
              way_wr_dat[23:16] = dc_dat_o[23:16];
            if (~dbus_bsel_i[1])
              way_wr_dat[15:8] = dc_dat_o[15:8];
            if (~dbus_bsel_i[0])
              way_wr_dat[7:0] = dc_dat_o[7:0];

            way_we = way_hit; // on WRITE

            tag_lru_in = next_lru_history;

            tag_we = 1'b1;
          end
        end

        REFILL: begin
          if (we_i) begin
            //
            // Write the data to the way that is replaced (which is
            // the LRU)
            //
            way_we = tag_save_lru; // on RE-FILL

            // Access pattern
            access = tag_save_lru;

            /* Invalidate the way on the first write */
            if (refill_done == 0) begin
              for (w2 = 0; w2 < OPTION_DCACHE_WAYS; w2 = w2 + 1) begin
                if (tag_save_lru[w2]) begin
                  tag_way_in[w2][TAGMEM_WAY_VALID] = 1'b0;
                end
              end

              tag_we = 1'b1;
            end

            //
            // After re-fill update the tag memory entry of the
            // filled way with the LRU history, the tag and set
            // valid to 1.
            //
            if (refill_last_o) begin
              for (w2 = 0; w2 < OPTION_DCACHE_WAYS; w2 = w2 + 1) begin
                tag_way_in[w2] = tag_way_save[w2];
                if (tag_save_lru[w2]) begin
                  tag_way_in[w2] = { 1'b1, curr_refill_adr[OPTION_DCACHE_LIMIT_WIDTH-1:WAY_WIDTH] };
                end
              end
              tag_lru_in = next_lru_history;

              tag_we = 1'b1;
            end
          end // write
        end // RE-FILL

        INVALIDATE: begin
          invalidate_ack = 1'b1;
          // Lazy invalidation, invalidate everything that matches tag address
          tag_lru_in = 0;
          for (w2 = 0; w2 < OPTION_DCACHE_WAYS; w2 = w2 + 1) begin
            tag_way_in[w2] = 0;
          end

          tag_we = 1'b1;
        end

        default: begin
        end
      endcase
    end
  end


  // write signals for Read/Write ports (*_rwp_*)
  wire [OPTION_DCACHE_WAYS-1:0] way_rwp_we;
  // write signals for Write-only ports (*_wp_*)
  wire [OPTION_DCACHE_WAYS-1:0] way_wp_we;
  
  // WAY-RAM read address
  wire [WAY_WIDTH-3:0] way_raddr [OPTION_DCACHE_WAYS-1:0];
  // WAY-RAM write address
  wire [WAY_WIDTH-3:0] way_waddr = phys_addr_cmd_i[WAY_WIDTH-1:2]; // on WRITE

  generate
  for (i = 0; i < OPTION_DCACHE_WAYS; i=i+1) begin : way_memories
    // address for Read/Write port
    assign way_raddr[i] = (way_we[i] & refill_o) ? curr_refill_adr[WAY_WIDTH-1:2] :
                                                   virt_addr_i[WAY_WIDTH-1:2];

    // write signals for Read/Write port (*_rwp_*)
    assign way_rwp_we[i] = way_we[i] & ((way_raddr[i] == way_waddr) | refill_o);
    // write signals for Write-only port (*_wp_*)
    assign way_wp_we[i]  = way_we[i] & ~(way_raddr[i] == way_waddr) & write;

    // WAY-RAM instances
    mor1kx_dpram_en_w1st_sclk
    #(
      .ADDR_WIDTH     (WAY_WIDTH-2),
      .DATA_WIDTH     (OPTION_OPERAND_WIDTH),
      .CLEAR_ON_INIT  (0)
    )
    dc_way_ram
    (
      // common clock
      .clk    (clk),
      // port "a": Read / Write (for RW-conflict case)
      .en_a   (1'b1),          // enable port "a"
      .we_a   (way_rwp_we[i]), // operation is "write"
      .addr_a (way_raddr[i]),
      .din_a  (way_wr_dat),
      .dout_a (way_dout[i]),
      // port "b": Write if no RW-conflict
      .en_b   (way_wp_we[i]),  // enable port "b"
      .we_b   (way_wp_we[i]),  // operation is "write"
      .addr_b (way_waddr),
      .din_b  (way_wr_dat),
      .dout_b ()            // not used
    );
  end

  if (OPTION_DCACHE_WAYS >= 2) begin : gen_u_lru
    mor1kx_cache_lru
    #(
      .NUMWAYS(OPTION_DCACHE_WAYS)
    )
    dc_lru
    (
      // Outputs
      .update      (next_lru_history),
      .lru_pre     (lru),
      .lru_post    (),
      // Inputs
      .current     (current_lru_history),
      .access      (access)
    );
  end
  else begin // single way
    assign next_lru_history = current_lru_history;
  end
  endgenerate


  // Read/Write port (*_rwp_*) write
  wire tr_rwp_we = tag_we &  (tag_rindex == tag_windex);

  // Write-only port (*_wp_*) enable
  wire tr_wp_en  = tag_we & ~(tag_rindex == tag_windex);

  // TAG-RAM instance
  mor1kx_dpram_en_w1st_sclk
  #(
    .ADDR_WIDTH     (OPTION_DCACHE_SET_WIDTH),
    .DATA_WIDTH     (TAGMEM_WIDTH),
    .CLEAR_ON_INIT  (0)
  )
  dc_tag_ram
  (
    // common clock
    .clk    (clk),
    // port "a": Read / Write (for RW-conflict case)
    .en_a   (1'b1),    // enable port "a"
    .we_a   (tr_rwp_we),  // operation is "write"
    .addr_a (tag_rindex),
    .din_a  (tag_din),
    .dout_a (tag_dout),
    // port "b": Write if no RW-conflict
    .en_b   (tr_wp_en),   // enable port "b"
    .we_b   (tag_we),     // operation is "write"
    .addr_b (tag_windex),
    .din_b  (tag_din),
    .dout_b ()            // not used
  );


  generate
  /* verilator lint_off WIDTH */
  if (OPTION_DCACHE_SNOOP != "NONE") begin : st_memory
  /* verilator lint_on WIDTH */
    // Read/Write port (*_rwp_*) write
    wire str_rwp_we = tag_we &  (snoop_index == tag_windex);

    // Write-only port (*_wp_*) enable
    wire str_wp_en  = tag_we & ~(snoop_index == tag_windex);

    // TAG-RAM instance
    mor1kx_dpram_en_w1st_sclk
    #(
      .ADDR_WIDTH     (OPTION_DCACHE_SET_WIDTH),
      .DATA_WIDTH     (TAGMEM_WIDTH),
      .CLEAR_ON_INIT  (0)
    )
    dc_snoop_tag_ram
    (
      // common clock
      .clk    (clk),
      // port "a": Read / Write (for RW-conflict case)
      .en_a   (1'b1),         // enable port "a"
      .we_a   (str_rwp_we),   // operation is "write"
      .addr_a (snoop_index),
      .din_a  (tag_din),
      .dout_a (snoop_dout),
      // port "b": Write if no RW-conflict
      .en_b   (str_wp_en),    // enable port "b"
      .we_b   (tag_we),       // operation is "write"
      .addr_b (tag_windex),
      .din_b  (tag_din),
      .dout_b ()              // not used
    );
  end
  endgenerate

endmodule // mor1kx_dcache_marocchino
