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
  parameter OPTION_OPERAND_WIDTH        = 32,
  parameter OPTION_ICACHE_BLOCK_WIDTH   =  5,
  parameter OPTION_ICACHE_SET_WIDTH     =  8,
  parameter OPTION_ICACHE_WAYS          =  2,
  parameter OPTION_ICACHE_LIMIT_WIDTH   = 32,
  parameter OPTION_ICACHE_CLEAR_ON_INIT =  0
)
(
  // clock and reset
  input                                 clk,
  input                                 rst,

  // controls
  input                                 adv_i,        // advance
  input                                 force_off_i,  // drop stored "IMMU enable"
  input                                 fetch_excepts_i,

  // configuration
  input                                 enable_i,

  // regular requests in/out
  input      [OPTION_OPERAND_WIDTH-1:0] virt_addr_i,
  input      [OPTION_OPERAND_WIDTH-1:0] phys_addr_fetch_i,
  input                                 immu_cache_inhibit_i,
  output                                ic_access_o,
  output                                ic_ack_o,
  output reg     [`OR1K_INSN_WIDTH-1:0] ic_dat_o,

  // re-fill
  output                                refill_req_o,
  input                                 ic_refill_allowed_i,
  output     [OPTION_OPERAND_WIDTH-1:0] next_refill_adr_o,
  output                                refill_last_o,
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


  // States
  localparam IDLE         = 4'b0001;
  localparam READ         = 4'b0010;
  localparam REFILL       = 4'b0100;
  localparam INVALIDATE   = 4'b1000;
  // FSM state pointer
  reg [3:0] state;
  // Particular state indicators
  wire   read       = (state == READ);
  wire   invalidate = (state == INVALIDATE);


  reg               [OPTION_OPERAND_WIDTH-1:0] curr_refill_adr;
  reg                                          refill_hit_r;
  reg [(1<<(OPTION_ICACHE_BLOCK_WIDTH-2))-1:0] refill_done;

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
  wire   [OPTION_ICACHE_WAYS-1:0] way_en; // WAY-RAM block enable
  wire   [OPTION_ICACHE_WAYS-1:0] way_we; // WAY-RAM write command
  wire            [WAY_WIDTH-3:0] way_addr [OPTION_ICACHE_WAYS-1:0];
  wire [OPTION_OPERAND_WIDTH-1:0] way_dout [OPTION_ICACHE_WAYS-1:0];

  // FETCH reads ICACHE
  wire                          ic_read;

  // Does any way hit?
  wire                          ic_check_limit_width;
  wire                          hit;
  wire [OPTION_ICACHE_WAYS-1:0] way_hit;

  // This is the least recently used value before access the memory.
  // Those are one hot encoded.
  wire [OPTION_ICACHE_WAYS-1:0] lru_way;
  // Register that stores the LRU way for re-fill process.
  reg  [OPTION_ICACHE_WAYS-1:0] lru_way_r;


  // The access vector to update the LRU history is the way that has
  // a hit or is refilled. It is also one-hot encoded.
  reg  [OPTION_ICACHE_WAYS-1:0] access_lru_history;
  // The current LRU history as read from tag memory and the update
  // value after we accessed it to write back to tag memory.
  wire [TAG_LRU_WIDTH_BITS-1:0] current_lru_history;
  wire [TAG_LRU_WIDTH_BITS-1:0] next_lru_history;

  genvar i;



  generate
  //   Hack? Work around IMMU?
  // Today the thing is actual for DCACHE only.
  // Addresses 0x8******* are treated as non-cacheble regardless DMMU's flag.
  if (OPTION_ICACHE_LIMIT_WIDTH == OPTION_OPERAND_WIDTH)
    assign ic_check_limit_width = 1'b1;
  else if (OPTION_ICACHE_LIMIT_WIDTH < OPTION_OPERAND_WIDTH)
    assign ic_check_limit_width =
      (phys_addr_fetch_i[OPTION_OPERAND_WIDTH-1:OPTION_ICACHE_LIMIT_WIDTH] == 0);
  else begin
    initial begin
      $display("ICACHE ERROR: OPTION_ICACHE_LIMIT_WIDTH > OPTION_OPERAND_WIDTH");
      $finish();
    end
  end
  endgenerate


  // Update ICACHE enable/disable
  reg enable_r;
  // ---
  always @(posedge clk `OR_ASYNC_RST) begin
    if (rst)
      enable_r <= 1'b0;
    else if (force_off_i) // ICAHCE -> idle
      enable_r <= 1'b0;
    else if (adv_i)
      enable_r <= enable_i;
  end // @ clock


  // detect per-way hit
  generate
  for (i = 0; i < OPTION_ICACHE_WAYS; i=i+1) begin : ways_out
    // WAY aliases of TAG-RAM output
    assign tag_way_out[i] = tag_dout[(i+1)*TAGMEM_WAY_WIDTH-1:i*TAGMEM_WAY_WIDTH];

    // hit: compare stored tag with incoming tag and check valid bit
    assign way_hit[i] = tag_way_out[i][TAGMEM_WAY_VALID] &
                        (tag_way_out[i][TAG_WIDTH-1:0] ==
                         phys_addr_fetch_i[OPTION_ICACHE_LIMIT_WIDTH-1:WAY_WIDTH]);
  end
  endgenerate


  // read success
  assign hit = |way_hit;

  // ICACHE access
  assign ic_access_o = enable_r & ic_check_limit_width & ~immu_cache_inhibit_i;

  // MAROCCHINO_TODO: potential improvement.
  // Allowing (out of the cache line being refilled) accesses during refill.
  assign ic_ack_o = (ic_access_o & read & hit) | refill_hit_r;

  // read result if success
  integer w0;
  always @(*) begin
    ic_dat_o = {OPTION_OPERAND_WIDTH{1'b0}};
    // Put correct way on the data port
    for (w0 = 0; w0 < OPTION_ICACHE_WAYS; w0 = w0 + 1) begin
      if (way_hit[w0] | (refill_hit_r & lru_way_r[w0])) begin
        ic_dat_o = way_dout[w0];
      end
    end
  end


  assign next_refill_adr_o = (OPTION_ICACHE_BLOCK_WIDTH == 5) ?
    {curr_refill_adr[31:5], curr_refill_adr[4:0] + 5'd4} : // 32 byte = (8 words x 32 bits/word) = (4 words x 64 bits/word)
    {curr_refill_adr[31:4], curr_refill_adr[3:0] + 4'd4};  // 16 byte = (4 words x 32 bits/word) = (2 words x 64 bits/word)

  assign refill_last_o = refill_done[next_refill_adr_o[OPTION_ICACHE_BLOCK_WIDTH-1:2]];

  assign refill_req_o = ic_access_o & read & ~hit;

  // SPR bus interface
  //  # detect SPR request to ICACHE
  wire spr_bus_ic_stb = spr_bus_stb_i & spr_bus_we_i & (spr_bus_addr_i == `OR1K_SPR_ICBIR_ADDR);
  //  # data output
  assign spr_bus_dat_o = {OPTION_OPERAND_WIDTH{1'b0}};

  // FETCH reads ICACHE
  assign ic_read = adv_i & enable_i;

  //-----------//
  // Cache FSM //
  //-----------//
  integer w1;
  always @(posedge clk `OR_ASYNC_RST) begin
    if (rst) begin
      spr_bus_ack_o   <= 1'b0;
      curr_refill_adr <= {OPTION_OPERAND_WIDTH{1'b0}};
      lru_way_r       <= {OPTION_ICACHE_WAYS{1'b0}};
      refill_hit_r    <= 1'b0;
      refill_done     <= 0;
      state           <= IDLE;
    end
    else if (fetch_excepts_i) begin
      // FETCH exceptions
      lru_way_r     <= {OPTION_ICACHE_WAYS{1'b0}};
      refill_hit_r  <= 1'b0;
      refill_done   <= 0;
      state         <= IDLE;
    end
    else if (spr_bus_ic_stb) begin
      // SPR transaction
      if (invalidate) begin
        spr_bus_ack_o <= 1'b0;
        state         <= IDLE;
      end
      else begin
        spr_bus_ack_o <= 1'b1;
        state         <= INVALIDATE;
      end
    end
    else begin
      // states
      case (state)
        IDLE: begin
          if (ic_read)
            state <= READ;
        end

        READ: begin
          if (ic_access_o & ~hit & ic_refill_allowed_i) begin
            // Store the LRU information for correct replacement
            // on refill. Always one when only one way.
            lru_way_r <= (OPTION_ICACHE_WAYS == 1) | lru_way;
            // store tag state
            for (w1 = 0; w1 < OPTION_ICACHE_WAYS; w1 = w1 + 1) begin
              tag_way_save[w1] <= tag_way_out[w1];
            end
            // 1st re-fill addrress
            curr_refill_adr <= phys_addr_fetch_i;
            // to re-fill
            state <= REFILL;
          end
          else if (~ic_read)
            state <= IDLE;
        end

        REFILL: begin
          refill_hit_r <= 1'b0;
          if (we_i) begin
            if (refill_last_o) begin
              refill_done <= 0;
              state       <= IDLE;
            end
            else begin
              refill_hit_r <= (refill_done == 0); // 1st re-fill is requested insn
              refill_done[curr_refill_adr[OPTION_ICACHE_BLOCK_WIDTH-1:2]] <= 1'b1;
              curr_refill_adr <= next_refill_adr_o;
            end // last or regulat
          end // we
        end // RE-FILL

        default:
          state <= IDLE;
      endcase
    end // reset / regular update
  end // @ clock


  // WAY-RAM write signal (for RE-FILL only)
  assign way_we = {OPTION_ICACHE_WAYS{we_i}} & lru_way_r;

  // WAY-RAM enable
  assign way_en = {OPTION_ICACHE_WAYS{ic_read}} | way_we;

  generate
  for (i = 0; i < OPTION_ICACHE_WAYS; i=i+1) begin : ways_ram
    // WAY-RAM input data and addresses
    assign way_addr[i] = way_we[i] ? curr_refill_adr[WAY_WIDTH-1:2] : // for RE-FILL only
                                     virt_addr_i[WAY_WIDTH-1:2];

    // WAY-RAM instance
    mor1kx_spram_en_w1st
    #(
      .ADDR_WIDTH    (WAY_WIDTH-2),
      .DATA_WIDTH    (OPTION_OPERAND_WIDTH),
      .CLEAR_ON_INIT (OPTION_ICACHE_CLEAR_ON_INIT)
    )
    ic_way_ram
    (
      // clock
      .clk  (clk),
      // port
      .en   (way_en[i]),  // enable
      .we   (way_we[i]),  // operation is write
      .addr (way_addr[i]),
      .din  (wrdat_i),
      .dout (way_dout[i])
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
    #(
      .NUMWAYS(OPTION_ICACHE_WAYS)
    )
    ic_lru
    (
      // Outputs
      .update      (next_lru_history),
      .lru_pre     (lru_way),
      .lru_post    (),
      // Inputs
      .current     (current_lru_history),
      .access      (access_lru_history)
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

    access_lru_history = {(OPTION_ICACHE_WAYS){1'b0}};

    case (state)
      READ: begin
        // The LRU module gets the access information.
        access_lru_history = way_hit;
        // Depending on this we update the LRU history in the tag.
        if (hit) begin
          tag_lru_in = next_lru_history;
          tag_we     = 1'b1;
        end
      end

      REFILL: begin
        if (we_i) begin
          // Access pattern
          access_lru_history = lru_way_r;

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
                lru_way_r[w2] ? {1'b1,curr_refill_adr[OPTION_ICACHE_LIMIT_WIDTH-1:WAY_WIDTH]} :
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


  // TAG-RAM data input
  //  # LRU section
  assign tag_din[TAG_LRU_MSB:TAG_LRU_LSB] = tag_lru_in;
  //  # WAY sections collection
  generate
  for (i = 0; i < OPTION_ICACHE_WAYS; i=i+1) begin : tw_sections
    assign tag_din[(i+1)*TAGMEM_WAY_WIDTH-1:i*TAGMEM_WAY_WIDTH] = tag_way_in[i];
  end
  endgenerate

  /*
   * The tag mem is written during reads to write the LRU info
   * and during refill and invalidate
   */
  assign tag_windex = read       ? phys_addr_fetch_i[WAY_WIDTH-1:OPTION_ICACHE_BLOCK_WIDTH] : // at update LRU field
                      invalidate ? spr_bus_dat_i[WAY_WIDTH-1:OPTION_ICACHE_BLOCK_WIDTH]     : // at invalidate
                                   curr_refill_adr[WAY_WIDTH-1:OPTION_ICACHE_BLOCK_WIDTH];    // at re-fill

  // TAG read address
  assign tag_rindex = virt_addr_i[WAY_WIDTH-1:OPTION_ICACHE_BLOCK_WIDTH];

  // Read/Write port (*_rwp_*) write
  wire tr_rwp_we = tag_we & (tag_rindex == tag_windex);

  // Write-only port (*_wp_*) enable
  wire tr_wp_en = tag_we & (~(tag_rindex == tag_windex) | ~ic_read);

  // TAG-RAM instance
  mor1kx_dpram_en_w1st_sclk
  #(
    .ADDR_WIDTH     (OPTION_ICACHE_SET_WIDTH),
    .DATA_WIDTH     (TAGMEM_WIDTH),
    .CLEAR_ON_INIT  (OPTION_ICACHE_CLEAR_ON_INIT)
  )
  ic_tag_ram
  (
    // common clock
    .clk    (clk),
    // port "a": Read / Write (for RW-conflict case)
    .en_a   (ic_read),    // enable port "a"
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

endmodule // mor1kx_icache_marocchino
