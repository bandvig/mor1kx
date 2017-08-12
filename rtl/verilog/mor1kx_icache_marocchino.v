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
  input                                 cpu_clk,
  input                                 cpu_rst,

  // pipe controls
  input                                 padv_s1s2_i,
  input                                 flush_by_ctrl_i,
  // fetch exceptions
  input                                 immu_an_except_i,
  input                                 ibus_err_i,

  // configuration
  input                                 ic_enable_i,

  // regular requests in/out
  input      [OPTION_OPERAND_WIDTH-1:0] virt_addr_mux_i,
  input      [OPTION_OPERAND_WIDTH-1:0] virt_addr_s1o_i,
  input      [OPTION_OPERAND_WIDTH-1:0] phys_addr_s2t_i,
  input                                 fetch_req_hit_i,
  input                                 immu_cache_inhibit_i,
  output                                ic_ack_o,
  output reg     [`OR1K_INSN_WIDTH-1:0] ic_dat_o,

  // IBUS access request
  output                                ibus_read_req_o,

  // re-fill
  output                                refill_req_o,
  input                                 to_refill_i,
  output reg                            ic_refill_first_o,
  input      [OPTION_OPERAND_WIDTH-1:0] phys_addr_s2o_i,
  input          [`OR1K_INSN_WIDTH-1:0] ibus_dat_i,
  input                                 ibus_burst_last_i,
  input                                 ibus_ack_i,

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
  localparam [3:0] IC_READ       = 4'b0001,
                   IC_REFILL     = 4'b0010,
                   IC_REREAD     = 4'b0100, // after re-fill
                   IC_INVALIDATE = 4'b1000;
  // FSM state pointer
  reg [3:0] ic_state;
  // Particular state indicators
  wire ic_read       = ic_state[0];
  wire ic_refill     = ic_state[1];
  wire ic_reread     = ic_state[2];
  wire ic_invalidate = ic_state[3];


  // The index we read and write from tag memory
  wire [OPTION_ICACHE_SET_WIDTH-1:0] tag_rindex;
  wire [OPTION_ICACHE_SET_WIDTH-1:0] tag_windex;
  //  Latch for invalidate index to simplify routing of SPR BUS
  reg  [OPTION_ICACHE_SET_WIDTH-1:0] tag_invdex;

  // The data from the tag memory
  wire       [TAGMEM_WIDTH-1:0] tag_dout;
  wire   [TAGMEM_WAY_WIDTH-1:0] tag_dout_way [OPTION_ICACHE_WAYS-1:0];

  // The data to the tag memory
  wire       [TAGMEM_WIDTH-1:0] tag_din;
  reg    [TAGMEM_WAY_WIDTH-1:0] tag_din_way [OPTION_ICACHE_WAYS-1:0];
  reg  [TAG_LRU_WIDTH_BITS-1:0] tag_din_lru;

  reg    [TAGMEM_WAY_WIDTH-1:0] tag_way_fetcho_r [OPTION_ICACHE_WAYS-1:0];

  // Whether to write to the tag memory in this cycle
  reg                           tag_we;

  // outputs of way memories
  wire [OPTION_OPERAND_WIDTH-1:0] way_dout [OPTION_ICACHE_WAYS-1:0];

  // Does any way hit?
  wire                          hit;
  wire [OPTION_ICACHE_WAYS-1:0] hit_way;

  // This is the least recently used value before access the memory.
  // Those are one hot encoded.
  wire [OPTION_ICACHE_WAYS-1:0] lru_way;          // computed by mor1kx_cache_lru
  reg  [OPTION_ICACHE_WAYS-1:0] lru_way_refill_r; // register for re-fill process

  // The access vector to update the LRU history is the way that has
  // a hit or is refilled. It is also one-hot encoded.
  reg  [OPTION_ICACHE_WAYS-1:0] access_way_for_lru;
  reg  [OPTION_ICACHE_WAYS-1:0] access_way_for_lru_fetcho_r; // register on IFETCH output
  // The current LRU history as read from tag memory.
  reg  [TAG_LRU_WIDTH_BITS-1:0] current_lru_history;
  reg  [TAG_LRU_WIDTH_BITS-1:0] current_lru_history_fetcho_r; // register on IFETCH output
  // The update value after we accessed it to write back to tag memory.
  wire [TAG_LRU_WIDTH_BITS-1:0] next_lru_history;          // computed by mor1kx_cache_lru


  genvar i;

  //
  // Local copy of ICACHE-related control bit(s) to simplify routing
  //
  // MT(F)SPR_RULE:
  //   Before issuing MT(F)SPR, OMAN waits till order control buffer has become
  // empty. Also we don't issue new instruction till l.mf(t)spr completion.
  //   So, it is safely to detect changing ICACHE-related control bit(s) here
  // and update local copies.
  //
  reg ic_enable_r;
  // ---
  always @(posedge cpu_clk) begin
    if (cpu_rst)
      ic_enable_r <= 1'b0;
    else if (ic_enable_r != ic_enable_i) // pipeline_flush doesn't switch caches on/off
      ic_enable_r <= ic_enable_i;
  end // @ clock


  //   Hack? Work around IMMU?
  // Today the thing is actual for DCACHE only.
  // Addresses 0x8******* are treated as non-cacheble regardless DMMU's flag.
  wire ic_check_limit_width;
  // ---
  generate
  if (OPTION_ICACHE_LIMIT_WIDTH == OPTION_OPERAND_WIDTH)
    assign ic_check_limit_width = 1'b1;
  else if (OPTION_ICACHE_LIMIT_WIDTH < OPTION_OPERAND_WIDTH)
    assign ic_check_limit_width =
      (phys_addr_s2t_i[OPTION_OPERAND_WIDTH-1:OPTION_ICACHE_LIMIT_WIDTH] == 0);
  else begin
    initial begin
      $display("ICACHE ERROR: OPTION_ICACHE_LIMIT_WIDTH > OPTION_OPERAND_WIDTH");
      $finish();
    end
  end
  endgenerate


  // detect per-way hit
  generate
  for (i = 0; i < OPTION_ICACHE_WAYS; i=i+1) begin : ways_out
    // WAY aliases of TAG-RAM output
    assign tag_dout_way[i] = tag_dout[(i+1)*TAGMEM_WAY_WIDTH-1:i*TAGMEM_WAY_WIDTH];
    // hit: compare stored tag with incoming tag and check valid bit
    assign hit_way[i] = tag_dout_way[i][TAGMEM_WAY_VALID] &
                        (tag_dout_way[i][TAG_WIDTH-1:0] ==
                         phys_addr_s2t_i[OPTION_ICACHE_LIMIT_WIDTH-1:WAY_WIDTH]);
  end
  endgenerate


  // read success
  assign hit = |hit_way;


  // Is the area cachable?
  wire   is_cacheble  = ic_enable_r & ic_check_limit_width & (~immu_cache_inhibit_i);
  // ICACHE ACK
  assign ic_ack_o     = is_cacheble & fetch_req_hit_i & ic_read &   hit;
  // RE-FILL request
  assign refill_req_o = is_cacheble & fetch_req_hit_i & ic_read & (~hit);

  // IBUS access request
  assign ibus_read_req_o = (~is_cacheble) & fetch_req_hit_i;


  // read result if success
  integer w0;
  always @ (*) begin
    ic_dat_o = {OPTION_OPERAND_WIDTH{1'b0}};
    // ---
    for (w0 = 0; w0 < OPTION_ICACHE_WAYS; w0=w0+1) begin : mux_dat_o
      if (hit_way[w0])
        ic_dat_o = way_dout[w0];
    end
  end // always


  // SPR bus interface
  //  # detect SPR request to ICACHE
  //  # (only invalidation command is implemented)
  wire spr_bus_ic_invalidate = spr_bus_stb_i & spr_bus_we_i & (spr_bus_addr_i == `OR1K_SPR_ICBIR_ADDR);
  //  # data output
  assign spr_bus_dat_o = {OPTION_OPERAND_WIDTH{1'b0}};


  //-----------//
  // Cache FSM //
  //-----------//
  always @(posedge cpu_clk) begin
    if (cpu_rst) begin
      spr_bus_ack_o     <= 1'b0;    // reset
      ic_refill_first_o <= 1'b0;    // reset
      ic_state          <= IC_READ; // reset
    end
    else begin
      // states
      // synthesis parallel_case full_case
      case (ic_state)
        IC_READ: begin
          spr_bus_ack_o <= 1'b0; // read
          // next states
          if (immu_an_except_i | flush_by_ctrl_i) begin // FSM: keep read
            ic_state <= IC_READ;
          end
          else if (spr_bus_ic_invalidate) begin
            ic_state   <= IC_INVALIDATE;    // FSM: read -> invalidate
            tag_invdex <= spr_bus_dat_i[WAY_WIDTH-1:OPTION_ICACHE_BLOCK_WIDTH]; // FSM: read -> invalidate
          end
          else if (to_refill_i) begin       // FSM: read -> re-fill
            ic_refill_first_o <= 1'b1;      // FSM: read -> re-fill
            ic_state          <= IC_REFILL; // FSM: read -> re-fill
          end
        end // FSM-READ state

        IC_REFILL: begin
          // In according with WISHBONE-B3 rule 3.45:
          // "SLAVE MUST NOT assert more than one of ACK, ERR or RTY"
          if (ibus_err_i) begin           // FSM: during re-fill
            ic_refill_first_o <= 1'b0;    // FSM: IBUS error during re-fill
            ic_state          <= IC_READ; // FSM: IBUS error during re-fill
          end
          else if (ibus_ack_i) begin    // FSM: during re-fill
            ic_refill_first_o <= 1'b0;  // FSM: IBUS ack during re-fill
            if (ibus_burst_last_i)
              ic_state <=  flush_by_ctrl_i ? IC_READ : IC_REREAD;  // FSM: last re-fill
          end
        end // FRM-RE-FILL state

        IC_REREAD: begin
          ic_state <= IC_READ; // FSM: re-read after re-fill
        end

        IC_INVALIDATE: begin
          spr_bus_ack_o <= 1'b1;    // FSM: invalidate -> idling
          ic_state      <= IC_READ; // FSM: invalidate -> idling
        end

        default: begin
          spr_bus_ack_o     <= 1'b0;    // FSM: default
          ic_refill_first_o <= 1'b0;    // FSM: default
          ic_state          <= IC_READ; // FSM: default
        end
      endcase
    end // reset / regular update
  end // @ clock



  //   In fact we don't need different addresses per way
  // because we access WAY-RAM either for read or for re-fill, but
  // we don't do these simultaneously
  //   As way size is equal to page one we able to use either
  // physical or virtual indexing.

  // For re-fill we use local copy of bus-bridge's burst address
  //  accumulator to generate WAY-RAM index.
  // The approach increases logic locality and makes routing easier.
  reg  [OPTION_OPERAND_WIDTH-1:0] virt_addr_rfl_r;
  wire [OPTION_OPERAND_WIDTH-1:0] virt_addr_rfl_next;
  // cache block length is 5 -> burst length is 8: 32 bytes = (8 words x 32 bits/word)
  // cache block length is 4 -> burst length is 4: 16 bytes = (4 words x 32 bits/word)
  assign virt_addr_rfl_next = (OPTION_ICACHE_BLOCK_WIDTH == 5) ?
    {virt_addr_rfl_r[31:5], virt_addr_rfl_r[4:0] + 5'd4} : // 32 byte = (8 words x 32 bits/word)
    {virt_addr_rfl_r[31:4], virt_addr_rfl_r[3:0] + 4'd4};  // 16 byte = (4 words x 32 bits/word)
  // ---
  always @(posedge cpu_clk) begin
    if (padv_s1s2_i)
      virt_addr_rfl_r <= virt_addr_s1o_i;    // before re-fill it is copy of IFETCH::s2o_virt_addr
    else if (ic_refill & ibus_ack_i)
      virt_addr_rfl_r <= virt_addr_rfl_next;
  end // @ clock

  // way address for write
  wire [WAY_WIDTH-3:0] way_addr;
  // ---
  assign way_addr = ic_refill ? virt_addr_rfl_r[WAY_WIDTH-1:2] : // WAY_WR_ADDR at re-fill
                    ic_reread ? virt_addr_s1o_i[WAY_WIDTH-1:2] : // WAY_RE_ADDR after re-fill
                                virt_addr_mux_i[WAY_WIDTH-1:2];  // WAY_RE_ADDR default

  // "en" / "we" (for re-fill) per way
  wire [OPTION_ICACHE_WAYS-1:0] way_en;
  wire [OPTION_ICACHE_WAYS-1:0] way_we;

  // WAY-RAM instances
  generate
  for (i = 0; i < OPTION_ICACHE_WAYS; i=i+1) begin : ways_ram
    // "we" per way (for re-fill only)
    assign way_we[i] = ibus_ack_i & lru_way_refill_r[i];
    assign way_en[i] = padv_s1s2_i | ic_reread | way_we[i];

    // WAY-RAM instances
    mor1kx_spram_en_w1st
    #(
      .ADDR_WIDTH     (WAY_WIDTH-2), // ICACHE_WAY_RAM
      .DATA_WIDTH     (OPTION_OPERAND_WIDTH), // ICACHE_WAY_RAM
      .CLEAR_ON_INIT  (OPTION_ICACHE_CLEAR_ON_INIT) // ICACHE_WAY_RAM
    )
    ic_way_ram
    (
      // clock
      .clk            (cpu_clk), // ICACHE_WAY_RAM
      // port
      .en             (way_en[i]), // ICACHE_WAY_RAM
      .we             (way_we[i]), // ICACHE_WAY_RAM
      .addr           (way_addr), // ICACHE_WAY_RAM
      .din            (ibus_dat_i), // ICACHE_WAY_RAM
      .dout           (way_dout[i]) // ICACHE_WAY_RAM
    );
  end // ways_ram
  endgenerate


  //--------------------//
  // TAG-RAM controller //
  //--------------------//

  // Local copy of IFETCH's s2o_ic_ack, but 1-clock length
  //  to prevent extra LRU updates
  reg ic_ack_r;
  // ---
  always @(posedge cpu_clk) begin
    if (cpu_rst | flush_by_ctrl_i)
      ic_ack_r <= 1'b0;
    else if (padv_s1s2_i)
      ic_ack_r <= ic_ack_o;
    else
      ic_ack_r <= 1'b0;
  end // @clock

  // LRU calculator
  generate
  /* verilator lint_off WIDTH */
  if (OPTION_ICACHE_WAYS >= 2) begin : gen_u_lru
  /* verilator lint_on WIDTH */
    mor1kx_cache_lru
    #(
      .NUMWAYS(OPTION_ICACHE_WAYS) // ICACHE_LRU
    )
    ic_lru
    (
      // Outputs
      .update      (next_lru_history), // ICACHE_LRU
      .lru_pre     (lru_way), // ICACHE_LRU
      .lru_post    (), // ICACHE_LRU
      // Inputs
      .current     (current_lru_history), // ICACHE_LRU
      .access      (access_way_for_lru) // ICACHE_LRU
    );
  end
  else begin // single way
    assign next_lru_history = current_lru_history; // one way cache
    assign lru_way          = 1'b1;                // one way cache
  end
  endgenerate

  // LRU related data registered on IFETCH output
  always @(posedge cpu_clk) begin
    if (cpu_rst) begin
      access_way_for_lru_fetcho_r  <= {OPTION_ICACHE_WAYS{1'b0}}; // reset
      current_lru_history_fetcho_r <= {TAG_LRU_WIDTH_BITS{1'b0}}; // reset
    end
    else if (padv_s1s2_i) begin
      access_way_for_lru_fetcho_r  <= hit_way;
      current_lru_history_fetcho_r <= tag_dout[TAG_LRU_MSB:TAG_LRU_LSB];
    end
  end // @clock

  // LRU way registered for re-fill process
  // MAROCCHINO_TODO : move it to FSM ??
  always @(posedge cpu_clk) begin
    if (cpu_rst) begin            // clear lru way for re-fill
      lru_way_refill_r <= {OPTION_ICACHE_WAYS{1'b0}};                             // reset / flush
    end
    else if (to_refill_i) begin   // save lru way for re-fill
      lru_way_refill_r <= flush_by_ctrl_i ? {OPTION_ICACHE_WAYS{1'b0}} : lru_way; // to re-fill
    end
    else if (ic_refill) begin
      if ((ibus_ack_i & ibus_burst_last_i) | ibus_err_i) begin
        lru_way_refill_r <= {OPTION_ICACHE_WAYS{1'b0}};                           // last re-fill / IBUS error
      end
    end
  end // @clock

  // store tag state
  integer w1;
  always @(posedge cpu_clk) begin
    if (padv_s1s2_i) begin
      for (w1 = 0; w1 < OPTION_ICACHE_WAYS; w1 = w1 + 1) begin
        tag_way_fetcho_r[w1] <= tag_dout_way[w1];
      end
    end
  end // @clock

  // TAG-RAM "we" and data input
  integer w2;
  always @(*) begin
    // no write TAG-RAM by default
    tag_we = 1'b0; // by default

    // by default prepare data for LRU update at hit or for re-fill initiation
    //  -- input for LRU calculator
    access_way_for_lru  = access_way_for_lru_fetcho_r; // by default
    current_lru_history = current_lru_history_fetcho_r; // by default
    //  -- output of LRU calculator
    tag_din_lru = next_lru_history; // by default
    //  -- other TAG-RAM fields
    for (w2 = 0; w2 < OPTION_ICACHE_WAYS; w2 = w2 + 1) begin
      tag_din_way[w2] = tag_way_fetcho_r[w2]; // by default
    end

    // synthesis parallel_case full_case
    case (ic_state)
      IC_READ: begin
        // Update LRU data by read-hit only
        if (ic_ack_r & (~immu_an_except_i) & (~flush_by_ctrl_i)) begin // on read-hit
          tag_we = 1'b1; // on read-hit
        end
      end

      IC_REFILL: begin
        //  (a) In according with WISHBONE-B3 rule 3.45:
        // "SLAVE MUST NOT assert more than one of ACK, ERR or RTY"
        //  (b) We don't interrupt re-fill on flushing, so the only reason
        // for invalidation is IBUS error occurence
        if (ibus_err_i) begin
          for (w2 = 0; w2 < OPTION_ICACHE_WAYS; w2 = w2 + 1) begin
            if (lru_way_refill_r[w2]) begin
              tag_din_way[w2][TAGMEM_WAY_VALID] = 1'b0;
            end
          end
          tag_we = 1'b1;
          // MAROCCHINO_TODO: how to handle LRU in the case?
        end
        else if (ibus_ack_i & ibus_burst_last_i) begin
          // After refill update the tag memory entry of the
          // filled way with the LRU history, the tag and set
          // valid to 1.
          for (w2 = 0; w2 < OPTION_ICACHE_WAYS; w2 = w2 + 1) begin
            if (lru_way_refill_r[w2]) begin
              tag_din_way[w2] = {1'b1,phys_addr_s2o_i[OPTION_ICACHE_LIMIT_WIDTH-1:WAY_WIDTH]}; // last re-fill
            end
          end
          access_way_for_lru = lru_way_refill_r; // last re-fill
          tag_we             = 1'b1;
        end
      end

      IC_INVALIDATE: begin
        // Lazy invalidation, invalidate everything that matches tag address
        tag_din_lru = 0; // by invalidate
        for (w2 = 0; w2 < OPTION_ICACHE_WAYS; w2 = w2 + 1) begin
          tag_din_way[w2] = 0;
        end
        tag_we = 1'b1;
      end

      default: begin
      end
    endcase
  end // always


  // Packed TAG-RAM data input
  //  # LRU section
  assign tag_din[TAG_LRU_MSB:TAG_LRU_LSB] = tag_din_lru;
  //  # WAY sections collection
  generate
  for (i = 0; i < OPTION_ICACHE_WAYS; i=i+1) begin : tw_sections
    assign tag_din[(i+1)*TAGMEM_WAY_WIDTH-1:i*TAGMEM_WAY_WIDTH] = tag_din_way[i];
  end
  endgenerate

  /*
   * The tag mem is written during:
   *  - last refill
   *  - invalidate
   *  - update LRU info
   *
   *   As way size is equal to page one we able to use either
   * physical or virtual indexing.
   */
  assign tag_windex = ic_invalidate ? tag_invdex                                              : // TAG_WR_ADDR at invalidate
                                      virt_addr_rfl_r[WAY_WIDTH-1:OPTION_ICACHE_BLOCK_WIDTH];   // TAG_WR_ADDR at re-fill / update LRU

  // TAG read address
  assign tag_rindex = padv_s1s2_i ? virt_addr_mux_i[WAY_WIDTH-1:OPTION_ICACHE_BLOCK_WIDTH] : // TAG_RE_ADDR at regular advance
                                    virt_addr_s1o_i[WAY_WIDTH-1:OPTION_ICACHE_BLOCK_WIDTH];  // TAG_RE_ADDR at re-fill, re-read, current

  // Read/Write into same address
  wire tag_rw_same_addr = (tag_rindex == tag_windex);

  // Read/Write port (*_rwp_*) write
  wire tag_rwp_we = tag_we & tag_rw_same_addr;
  wire tag_rwp_en = padv_s1s2_i | ic_reread | tag_rwp_we;

  // Write-only port (*_wp_*) enable
  wire tag_wp_en = tag_we & (~tag_rw_same_addr);


  //------------------//
  // TAG-RAM instance //
  //------------------//

  mor1kx_dpram_en_w1st_sclk
  #(
    .ADDR_WIDTH     (OPTION_ICACHE_SET_WIDTH), // ICACHE_TAG_RAM
    .DATA_WIDTH     (TAGMEM_WIDTH), // ICACHE_TAG_RAM
    .CLEAR_ON_INIT  (OPTION_ICACHE_CLEAR_ON_INIT) // ICACHE_TAG_RAM
  )
  ic_tag_ram
  (
    // common clock
    .clk    (cpu_clk), // ICACHE_TAG_RAM
    // port "a": Read / Write (for RW-conflict case)
    .en_a   (tag_rwp_en), // ICACHE_TAG_RAM
    .we_a   (tag_rwp_we), // ICACHE_TAG_RAM
    .addr_a (tag_rindex), // ICACHE_TAG_RAM
    .din_a  (tag_din), // ICACHE_TAG_RAM
    .dout_a (tag_dout), // ICACHE_TAG_RAM
    // port "b": Write if no RW-conflict
    .en_b   (tag_wp_en), // ICACHE_TAG_RAM
    .we_b   (tag_we), // ICACHE_TAG_RAM
    .addr_b (tag_windex), // ICACHE_TAG_RAM
    .din_b  (tag_din), // ICACHE_TAG_RAM
    .dout_b () // ICACHE_TAG_RAM
  );

endmodule // mor1kx_icache_marocchino
