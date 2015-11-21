////////////////////////////////////////////////////////////////////////
//                                                                    //
//  mor1kx_icache_marocchino                                          //
//                                                                    //
//  Description: Instruction CACHE implementation                     //
//               The variant is tightly coupled with                  //
//               MAROCCHINO FETCH and IMMU                            //
//               (based on mor1kx_immu)                               //
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

module mor1kx_icache_marocchino
#(
  parameter OPTION_OPERAND_WIDTH      = 32,
  parameter OPTION_ICACHE_BLOCK_WIDTH =  5,
  parameter OPTION_ICACHE_SET_WIDTH   =  9,
  parameter OPTION_ICACHE_WAYS        =  2,
  parameter OPTION_ICACHE_LIMIT_WIDTH = 32
)
(
  input                                 clk,
  input                                 rst,

  input                                 ic_imem_err_i,
  input                                 ic_access_i,
  output                                refill_o,
  output                                refill_req_o,
  output                                refill_last_o,

  // CPU Interface
  output                                ic_ack_o,
  output reg     [`OR1K_INSN_WIDTH-1:0] ic_dat_o,
  input      [OPTION_OPERAND_WIDTH-1:0] virt_addr_i,
  input      [OPTION_OPERAND_WIDTH-1:0] phys_addr_fetch_i,
  input                                 cpu_req_i,

  input      [OPTION_OPERAND_WIDTH-1:0] wradr_i,
  input          [`OR1K_INSN_WIDTH-1:0] wrdat_i,
  input                                 we_i,

  // SPR interface
  input                          [15:0] spr_bus_addr_i,
  input                                 spr_bus_we_i,
  input                                 spr_bus_stb_i,
  input      [OPTION_OPERAND_WIDTH-1:0] spr_bus_dat_i,
  output     [OPTION_OPERAND_WIDTH-1:0] spr_bus_dat_o,
  output reg                            spr_bus_ack_o
);

  // States
  localparam IDLE         = 4'b0001;
  localparam READ         = 4'b0010;
  localparam REFILL       = 4'b0100;
  localparam INVALIDATE   = 4'b1000;

  // Address space in bytes for a way
  localparam WAY_WIDTH = OPTION_ICACHE_BLOCK_WIDTH + OPTION_ICACHE_SET_WIDTH;
  /*
   * Tag memory layout
   *            +---------------------------------------------------------+
   * (index) -> | LRU | wayN valid | wayN tag |...| way0 valid | way0 tag |
   *            +---------------------------------------------------------+
   */

  // The tag is the part left of the index
  localparam TAG_WIDTH = (OPTION_ICACHE_LIMIT_WIDTH - WAY_WIDTH);

  // The tag memory contains entries with OPTION_ICACHE_WAYS parts of
  // each TAGMEM_WAY_WIDTH. Each of those is tag and a valid flag.
  localparam TAGMEM_WAY_WIDTH = TAG_WIDTH + 1;
  localparam TAGMEM_WAY_VALID = TAGMEM_WAY_WIDTH - 1;

  // Additionally, the tag memory entry contains an LRU value. The
  // width of this is actually 0 for OPTION_ICACHE_LIMIT_WIDTH==1
  localparam TAG_LRU_WIDTH = OPTION_ICACHE_WAYS*(OPTION_ICACHE_WAYS-1) >> 1;

  // We have signals for the LRU which are not used for one way
  // caches. To avoid signal width [-1:0] this generates [0:0]
  // vectors for them, which are removed automatically then.
  localparam TAG_LRU_WIDTH_BITS = (OPTION_ICACHE_WAYS >= 2) ? TAG_LRU_WIDTH : 1;

  // Compute the total sum of the entry elements
  localparam TAGMEM_WIDTH = TAGMEM_WAY_WIDTH * OPTION_ICACHE_WAYS + TAG_LRU_WIDTH;

  // For convenience we define the position of the LRU in the tag
  // memory entries
  localparam TAG_LRU_MSB = TAGMEM_WIDTH - 1;
  localparam TAG_LRU_LSB = TAG_LRU_MSB - TAG_LRU_WIDTH + 1;

  // FSM state signals
  reg [3:0] state;

  wire   read       = (state == READ);
  assign refill_o   = (state == REFILL);
  wire   invalidate = (state == INVALIDATE);

  wire [31:0] next_refill_adr;
  wire             refill_hit;
  reg [(1<<(OPTION_ICACHE_BLOCK_WIDTH-2))-1:0] refill_done;
  reg [(1<<(OPTION_ICACHE_BLOCK_WIDTH-2))-1:0] refill_done_r;

  // The index we read and write from tag memory
  wire [OPTION_ICACHE_SET_WIDTH-1:0] tag_rindex;
  wire [OPTION_ICACHE_SET_WIDTH-1:0] tag_windex;

  // The data from the tag memory
  wire       [TAGMEM_WIDTH-1:0] tag_dout;
  wire   [TAGMEM_WAY_WIDTH-1:0] tag_way_out [OPTION_ICACHE_WAYS-1:0];

  // The data to the tag memory
  reg   [TAGMEM_WAY_WIDTH-1:0] tag_way_in [OPTION_ICACHE_WAYS-1:0];
  reg [TAG_LRU_WIDTH_BITS-1:0] tag_lru_in;
  wire      [TAGMEM_WIDTH-1:0] tag_din;

  reg   [TAGMEM_WAY_WIDTH-1:0] tag_way_save [OPTION_ICACHE_WAYS-1:0];

  // Whether to write to the tag memory in this cycle
  reg                          tag_we;

  // Access to the way memories
  wire            [WAY_WIDTH-3:0] way_raddr [OPTION_ICACHE_WAYS-1:0];
  wire            [WAY_WIDTH-3:0] way_waddr [OPTION_ICACHE_WAYS-1:0];
  wire [OPTION_OPERAND_WIDTH-1:0] way_din   [OPTION_ICACHE_WAYS-1:0];
  wire [OPTION_OPERAND_WIDTH-1:0] way_dout  [OPTION_ICACHE_WAYS-1:0];
  wire   [OPTION_ICACHE_WAYS-1:0] way_we;

  // Does any way hit?
  wire                          hit;
  wire [OPTION_ICACHE_WAYS-1:0] way_hit;

  // This is the least recently used value before access the memory.
  // Those are one hot encoded.
  wire [OPTION_ICACHE_WAYS-1:0] lru_way;
  // Register that stores the LRU way for re-fill process.
  reg  [OPTION_ICACHE_WAYS-1:0] lru_way_r;

  // The access vector to update the LRU history is the way that has
  // a hit or is refilled. It is also one-hot encoded.
  reg  [OPTION_ICACHE_WAYS-1:0] access;

  // The current LRU history as read from tag memory and the update
  // value after we accessed it to write back to tag memory.
  wire [TAG_LRU_WIDTH_BITS-1:0] current_lru_history;
  wire [TAG_LRU_WIDTH_BITS-1:0] next_lru_history;

  genvar               i;

  // MAROCCHINO_TODO: potential improvement.
  // Allowing (out of the cache line being refilled) accesses during refill.
  assign ic_ack_o = (read & hit) | (refill_hit & ic_access_i);

  for (i = 0; i < OPTION_ICACHE_WAYS; i=i+1) begin : ways_out
    // WAY aliases of TAG-RAM output
    assign tag_way_out[i] = tag_dout[(i+1)*TAGMEM_WAY_WIDTH-1:i*TAGMEM_WAY_WIDTH];

    // hit: compare stored tag with incoming tag and check valid bit
    assign way_hit[i] = tag_way_out[i][TAGMEM_WAY_VALID] &
                        (tag_way_out[i][TAG_WIDTH-1:0] ==
                         phys_addr_fetch_i[OPTION_ICACHE_LIMIT_WIDTH-1:WAY_WIDTH]);
  end

  // read success
  assign hit = |way_hit;


  // read result if success
  integer w0;
  always @(*) begin
    ic_dat_o = {OPTION_OPERAND_WIDTH{1'b0}};
    // Put correct way on the data port
    for (w0 = 0; w0 < OPTION_ICACHE_WAYS; w0 = w0 + 1) begin
      if (way_hit[w0] | (refill_hit & lru_way_r[w0])) begin
        ic_dat_o = way_dout[w0];
      end
    end
  end

  assign next_refill_adr = (OPTION_ICACHE_BLOCK_WIDTH == 5) ?
         {wradr_i[31:5], wradr_i[4:0] + 5'd4} : // 32 byte = (8 words x 32 bits/word) = (4 words x 64 bits/word)
         {wradr_i[31:4], wradr_i[3:0] + 4'd4};  // 16 byte = (4 words x 32 bits/word) = (2 words x 64 bits/word)
 
  assign refill_last_o = refill_done[next_refill_adr[OPTION_ICACHE_BLOCK_WIDTH-1:2]];

  assign refill_hit = refill_done_r[phys_addr_fetch_i[OPTION_ICACHE_BLOCK_WIDTH-1:2]] &
    (phys_addr_fetch_i[OPTION_ICACHE_LIMIT_WIDTH-1:OPTION_ICACHE_BLOCK_WIDTH] ==
     wradr_i[OPTION_ICACHE_LIMIT_WIDTH-1:OPTION_ICACHE_BLOCK_WIDTH]) &
    refill_o;

  assign refill_req_o = (read & cpu_req_i & ~hit) | refill_o;

  /*
   * SPR bus interface
   */
  assign invalidate_spr = spr_bus_stb_i & spr_bus_we_i & (spr_bus_addr_i == `OR1K_SPR_ICBIR_ADDR);

  /*
   * Cache FSM
   */
  integer w1;
  always @(posedge clk `OR_ASYNC_RST) begin
    refill_done_r <= refill_done;
    spr_bus_ack_o <= 0;

    case (state)
      IDLE: begin
        if (cpu_req_i)
          state <= READ;
      end

      READ: begin
        if (ic_access_i) begin
          if (hit) begin
            state <= READ;
          end
          else if (cpu_req_i) begin
            refill_done <= 0;
            refill_done_r <= 0;

            // Store the LRU information for correct replacement
            // on refill. Always one when only one way.
            lru_way_r <= (OPTION_ICACHE_WAYS == 1) | lru_way;

            for (w1 = 0; w1 < OPTION_ICACHE_WAYS; w1 = w1 + 1) begin
              tag_way_save[w1] <= tag_way_out[w1];
            end

            state <= REFILL;
          end
        end
        else begin
          state <= IDLE;
        end
      end

      REFILL: begin
        if (we_i) begin
          refill_done[wradr_i[OPTION_ICACHE_BLOCK_WIDTH-1:2]] <= 1'b1;

          if (refill_last_o)
            state <= IDLE;
        end
      end

      INVALIDATE: begin
        if (~invalidate_spr)
          state <= IDLE;
          spr_bus_ack_o <= 1;
        end

      default:
        state <= IDLE;
    endcase

    if (invalidate_spr & ~refill_o) begin
      spr_bus_ack_o  <= 1;
      state          <= INVALIDATE;
    end

    if (rst)
      state <= IDLE;
    else if(ic_imem_err_i)
      state <= IDLE;
  end // @ clock


  // WAY-RAM write signal (for RE-FILL only)
  assign way_we = {OPTION_ICACHE_WAYS{we_i}} & lru_way_r;

  // WAY-RAM input data and addresses
  for (i = 0; i < OPTION_ICACHE_WAYS; i=i+1) begin : ways_in_data
    assign way_raddr[i] = virt_addr_i[WAY_WIDTH-1:2];
    assign way_waddr[i] = wradr_i[WAY_WIDTH-1:2];
    assign way_din[i]   = wrdat_i;
  end

  // WAY-RAM instance
  generate
  for (i = 0; i < OPTION_ICACHE_WAYS; i=i+1) begin : way_memories
    mor1kx_simple_dpram_sclk
    #(
      .ADDR_WIDTH     (WAY_WIDTH-2),
      .DATA_WIDTH     (OPTION_OPERAND_WIDTH),
      .ENABLE_BYPASS  (0)
    )
    way_data_ram
    (
      // Outputs
      .dout   (way_dout[i][OPTION_OPERAND_WIDTH-1:0]),
      // Inputs
      .clk    (clk),
      .raddr  (way_raddr[i][WAY_WIDTH-3:0]),
      .re     (1'b1),
      .waddr  (way_waddr[i][WAY_WIDTH-3:0]),
      .we     (way_we[i]),
      .din    (way_din[i][31:0])
    );
  end // block: way_memories
  endgenerate


  //--------------------//
  // TAG-RAM controller //
  //--------------------//

  // LRU relative
  assign current_lru_history = tag_dout[TAG_LRU_MSB:TAG_LRU_LSB];

  // LRU calculator
  generate
  /* verilator lint_off WIDTH */
  if (OPTION_ICACHE_WAYS >= 2) begin : gen_u_lru
  /* verilator lint_on WIDTH */
    mor1kx_cache_lru
    #(.NUMWAYS(OPTION_ICACHE_WAYS))
    u_lru
    (
      // Outputs
      .update      (next_lru_history),
      .lru_pre     (lru_way),
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


  // TAG "we" and data input
  integer w2;
  always @(*) begin
    // Default is to keep data, don't write and don't access
    tag_lru_in = current_lru_history;
    for (w2 = 0; w2 < OPTION_ICACHE_WAYS; w2 = w2 + 1) begin
      tag_way_in[w2] = tag_way_out[w2];
    end

    tag_we = 1'b0;

    access = {(OPTION_ICACHE_WAYS){1'b0}};

    case (state)
      READ: begin
        // The LRU module gets the access information.
        access = way_hit;
        // Depending on this we update the LRU history in the tag.
        if (hit) begin
          tag_lru_in = next_lru_history;
          tag_we     = 1'b1;
        end
      end

      REFILL: begin
        if (we_i) begin
          // Access pattern
          access = lru_way_r;

          /* Invalidate the way on the first write */
          if (refill_done == 0) begin
            for (w2 = 0; w2 < OPTION_ICACHE_WAYS; w2 = w2 + 1) begin
              if (lru_way_r[w2]) begin
                tag_way_in[w2][TAGMEM_WAY_VALID] = 1'b0;
              end
            end

            tag_we = 1'b1;
          end

          // After refill update the tag memory entry of the
          // filled way with the LRU history, the tag and set
          // valid to 1.
          if (refill_last_o) begin
            for (w2 = 0; w2 < OPTION_ICACHE_WAYS; w2 = w2 + 1) begin
              tag_way_in[w2] =
                lru_way_r[w2] ? {1'b1,wradr_i[OPTION_ICACHE_LIMIT_WIDTH-1:WAY_WIDTH]} :
                                tag_way_save[w2];
            end
            tag_lru_in = next_lru_history;

            tag_we = 1'b1;
          end
        end
      end

      INVALIDATE: begin
        // Lazy invalidation, invalidate everything that matches tag address
        tag_lru_in = 0;
        for (w2 = 0; w2 < OPTION_ICACHE_WAYS; w2 = w2 + 1) begin
          tag_way_in[w2] = 0;
        end

        tag_we = 1'b1;
      end

      default: begin
      end
    endcase
  end // always


  // TAG read address
  assign tag_rindex = virt_addr_i[WAY_WIDTH-1:OPTION_ICACHE_BLOCK_WIDTH];

  /*
   * The tag mem is written during reads to write the LRU info and during
   * refill and invalidate
   */
  assign tag_windex = read       ? phys_addr_fetch_i[WAY_WIDTH-1:OPTION_ICACHE_BLOCK_WIDTH] : // at update LRU field
                      invalidate ? spr_bus_dat_i[WAY_WIDTH-1:OPTION_ICACHE_BLOCK_WIDTH]     : // at invalidate
                                   wradr_i[WAY_WIDTH-1:OPTION_ICACHE_BLOCK_WIDTH];            // at re-fill

  // TAG-RAM data input
  //  # LRU part
  assign tag_din[TAG_LRU_MSB:TAG_LRU_LSB] = tag_lru_in;
  //  # WAY parts
  for (i = 0; i < OPTION_ICACHE_WAYS; i=i+1) begin : tag_data_in_collection
    assign tag_din[(i+1)*TAGMEM_WAY_WIDTH-1:i*TAGMEM_WAY_WIDTH] = tag_way_in[i];
  end


  // TAG-RAM instance
  mor1kx_simple_dpram_sclk
  #(
    .ADDR_WIDTH     (OPTION_ICACHE_SET_WIDTH),
    .DATA_WIDTH     (TAGMEM_WIDTH),
    .ENABLE_BYPASS  (0)
  )
  tag_ram
  (
    // Outputs
    .dout     (tag_dout[TAGMEM_WIDTH-1:0]),
    // Inputs
    .clk      (clk),
    .raddr    (tag_rindex),
    .re       (1'b1),
    .waddr    (tag_windex),
    .we       (tag_we),
    .din      (tag_din)
  );

endmodule // mor1kx_icache_marocchino
