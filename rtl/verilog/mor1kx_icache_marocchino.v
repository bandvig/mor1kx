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
  input                                 padv_ic_i,
  input                                 flush_by_ctrl_i,
  // fetch exceptions
  input                                 immu_an_except_i,
  input                                 ibus_err_i,

  // configuration
  input                                 ic_enable_i,

  // regular requests in/out
  input      [OPTION_OPERAND_WIDTH-1:0] phys_addr_idx_i,
  input      [OPTION_OPERAND_WIDTH-1:0] phys_addr_tag_i,
  input                                 fetch_req_hit_i,
  input                                 immu_cache_inhibit_i,
  output                                ic_ack_o,
  output reg     [`OR1K_INSN_WIDTH-1:0] ic_dat_o,

  // IBUS access request
  output                                ibus_access_req_o,

  // re-fill
  output                                refill_req_o,
  output reg                            ic_refill_first_o,
  input          [`OR1K_INSN_WIDTH-1:0] ibus_dat_i,
  input      [OPTION_OPERAND_WIDTH-1:0] ibus_burst_adr_i,
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
  localparam [3:0] IC_IDLE       = 4'b0001,
                   IC_READ       = 4'b0010,
                   IC_REFILL     = 4'b0100,
                   IC_INVALIDATE = 4'b1000;
  // FSM state pointer
  reg [3:0] ic_state;
  // Particular state indicators
  wire ic_read       = ic_state[1];
  wire ic_refill     = ic_state[2];
  wire ic_invalidate = ic_state[3];


  // The index we read and write from tag memory
  wire [OPTION_ICACHE_SET_WIDTH-1:0] tag_rindex;
  wire [OPTION_ICACHE_SET_WIDTH-1:0] tag_windex;
  //  Latch for invalidate index to simplify routing of SPR BUS
  reg  [OPTION_ICACHE_SET_WIDTH-1:0] tag_invdex;

  // The data from the tag memory
  wire       [TAGMEM_WIDTH-1:0] tag_dout;
  wire   [TAGMEM_WAY_WIDTH-1:0] tag_way_out [OPTION_ICACHE_WAYS-1:0];

  // The data to the tag memory
  reg    [TAGMEM_WAY_WIDTH-1:0] tag_way_in [OPTION_ICACHE_WAYS-1:0];
  reg  [TAG_LRU_WIDTH_BITS-1:0] tag_lru_in;
  wire       [TAGMEM_WIDTH-1:0] tag_din;

  reg    [TAGMEM_WAY_WIDTH-1:0] tag_way_save [OPTION_ICACHE_WAYS-1:0];

  // Whether to write to the tag memory in this cycle
  reg                           tag_we;

  // Access to the way memories
  wire   [OPTION_ICACHE_WAYS-1:0] way_en; // WAY-RAM block enable
  wire   [OPTION_ICACHE_WAYS-1:0] way_we; // WAY-RAM write command
  wire [OPTION_OPERAND_WIDTH-1:0] way_dout [OPTION_ICACHE_WAYS-1:0];

  // FETCH reads ICACHE (doesn't include exceptions or flushing control)
  wire                          ic_fsm_adv;

  // RAM block read access (includes exceptions or flushing control)
  wire                          ic_ram_re;

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


  // FETCH reads ICACHE (doesn't include exceptions or flushing control)
  assign ic_fsm_adv = padv_ic_i & ic_enable_i;

  // RAM block read access
  assign ic_ram_re  = padv_ic_i & ic_enable_i;

  // Stored "ICACHE enable" flag for ibus_access_req_o
  reg ic_enable_r;
  // ---
  always @(posedge cpu_clk) begin
    if (cpu_rst | flush_by_ctrl_i)
      ic_enable_r <= 1'b0;
    else if (padv_ic_i)
      ic_enable_r <= ic_enable_i;
  end // @ clock


  generate
  //   Hack? Work around IMMU?
  // Today the thing is actual for DCACHE only.
  // Addresses 0x8******* are treated as non-cacheble regardless DMMU's flag.
  if (OPTION_ICACHE_LIMIT_WIDTH == OPTION_OPERAND_WIDTH)
    assign ic_check_limit_width = 1'b1;
  else if (OPTION_ICACHE_LIMIT_WIDTH < OPTION_OPERAND_WIDTH)
    assign ic_check_limit_width =
      (phys_addr_tag_i[OPTION_OPERAND_WIDTH-1:OPTION_ICACHE_LIMIT_WIDTH] == 0);
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
    assign tag_way_out[i] = tag_dout[(i+1)*TAGMEM_WAY_WIDTH-1:i*TAGMEM_WAY_WIDTH];
    // hit: compare stored tag with incoming tag and check valid bit
    assign way_hit[i] = tag_way_out[i][TAGMEM_WAY_VALID] &
                        (tag_way_out[i][TAG_WIDTH-1:0] ==
                         phys_addr_tag_i[OPTION_ICACHE_LIMIT_WIDTH-1:WAY_WIDTH]);
  end
  endgenerate


  // read success
  assign hit = |way_hit;

  //
  //   If ICACHE is in state read/refill it automatically means that
  // ICACHE is enabled (see ic_fsm_adv).
  //
  wire   is_cacheble  = ic_check_limit_width & ~immu_cache_inhibit_i;
  // ICACHE ACK
  assign ic_ack_o     = ic_read & is_cacheble & fetch_req_hit_i &  hit;
  // RE-FILL request
  assign refill_req_o = ic_read & is_cacheble & fetch_req_hit_i & ~hit;

  // IBUS access request
  assign ibus_access_req_o  = ((~ic_enable_r) | (~is_cacheble)) & fetch_req_hit_i;


  // read result if success
  integer w0;
  always @ (*) begin
    ic_dat_o = {OPTION_OPERAND_WIDTH{1'b0}};
    // ---
    for (w0 = 0; w0 < OPTION_ICACHE_WAYS; w0=w0+1) begin : mux_dat_o
      if (way_hit[w0])
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
  integer w1;
  always @(posedge cpu_clk) begin
    if (cpu_rst) begin
      lru_way_r         <= {OPTION_ICACHE_WAYS{1'b0}};    // reset
      ic_refill_first_o <= 1'b0;    // reset
      ic_state          <= IC_IDLE; // reset
      spr_bus_ack_o     <= 1'b0;    // reset
    end
    else begin
      // states
      // synthesis parallel_case full_case
      case (ic_state)
        IC_IDLE: begin
          spr_bus_ack_o <= 1'b0; // idling
          // next states
          if (flush_by_ctrl_i) // ICACHE FSM: keep idle
            ic_state <= IC_IDLE;
          else if (spr_bus_ic_invalidate) begin
            ic_state      <= IC_INVALIDATE; // idling -> invalidate
            tag_invdex    <= spr_bus_dat_i[WAY_WIDTH-1:OPTION_ICACHE_BLOCK_WIDTH]; // idling -> invalidate
          end
          else if (ic_fsm_adv)
            ic_state <= IC_READ;
        end

        IC_READ: begin // same to IMEM_REQ for IBUS
          if (immu_an_except_i | flush_by_ctrl_i)               // FSM: READ: read -> idle
            ic_state <= IC_IDLE;                                // FSM: READ: idle by IMMU exceptions or flushing
          else if (is_cacheble & fetch_req_hit_i & ~hit) begin  // FSM: READ: refill_req_o
            // Store the LRU information for correct replacement
            // on refill. Always one when only one way.
            lru_way_r <= (OPTION_ICACHE_WAYS == 1) | lru_way; // to re-fill
            // store tag state
            for (w1 = 0; w1 < OPTION_ICACHE_WAYS; w1 = w1 + 1) begin
              tag_way_save[w1] <= tag_way_out[w1];
            end
            // 1st re-fill addrress
            ic_refill_first_o <= 1'b1; // to re-fill
            // to re-fill
            ic_state <= IC_REFILL;
          end
          else if (ic_fsm_adv)    // FSM: READ: ibus_access_req_o ? idle : read
            ic_state <= IC_READ;  // FSM: READ:
          else // no advancing
            ic_state <= IC_IDLE;
        end

        IC_REFILL: begin
          // In according with WISHBONE-B3 rule 3.45:
          // "SLAVE MUST NOT assert more than one of ACK, ERR or RTY"
          if (ibus_err_i) begin // during re-fill
            lru_way_r   <= {OPTION_ICACHE_WAYS{1'b0}};  // IBUS error during re-fill
            ic_state    <= IC_IDLE;                     // IBUS error during re-fill
          end
          else if (ibus_ack_i) begin
            ic_refill_first_o <= 1'b0;                  // first re-fill
            // last re-fill
            if (ibus_burst_last_i) begin
              lru_way_r <= {OPTION_ICACHE_WAYS{1'b0}};  // last re-fill
              ic_state  <= IC_IDLE;                     // last re-fill
            end
          end // IBUS ACK
        end // RE-FILL

        IC_INVALIDATE: begin
          ic_state      <= IC_IDLE; // invalidate -> idling
          spr_bus_ack_o <= 1'b1;    // invalidate -> idling
        end

        default: begin
          lru_way_r         <= {OPTION_ICACHE_WAYS{1'b0}}; // default
          ic_refill_first_o <= 1'b0;    // default
          ic_state          <= IC_IDLE; // default
          spr_bus_ack_o     <= 1'b0;    // default
        end
      endcase
    end // reset / regular update
  end // @ clock


  // WAY-RAM write signal (for RE-FILL only)
  assign way_we = {OPTION_ICACHE_WAYS{ibus_ack_i}} & lru_way_r;

  // WAY-RAM enable
  assign way_en = {OPTION_ICACHE_WAYS{ic_ram_re}} | way_we;

  // In fact we don't need different addresses per way
  // because we access WAY-RAM either for read or for re-fill, but
  // we don't do these simultaneously
  wire [WAY_WIDTH-3:0] way_addr;
  // ---
  assign way_addr = ic_refill ? ibus_burst_adr_i[WAY_WIDTH-1:2] : // WAY_ADDR at re-fill
                                phys_addr_idx_i[WAY_WIDTH-1:2];   // WAY_ADDR default

  // WAY-RAM instances
  generate
  for (i = 0; i < OPTION_ICACHE_WAYS; i=i+1) begin : ways_ram
    // WAY-RAM instance
    mor1kx_spram_en_w1st
    #(
      .ADDR_WIDTH    (WAY_WIDTH-2), // ICACHE_WAY_RAM
      .DATA_WIDTH    (OPTION_OPERAND_WIDTH), // ICACHE_WAY_RAM
      .CLEAR_ON_INIT (OPTION_ICACHE_CLEAR_ON_INIT) // ICACHE_WAY_RAM
    )
    ic_way_ram
    (
      // clock
      .clk  (cpu_clk), // ICACHE_WAY_RAM
      // port
      .en   (way_en[i]), // ICACHE_WAY_RAM
      .we   (way_we[i]), // ICACHE_WAY_RAM
      .addr (way_addr), // ICACHE_WAY_RAM
      .din  (ibus_dat_i), // ICACHE_WAY_RAM
      .dout (way_dout[i]) // ICACHE_WAY_RAM
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
      .access      (access_lru_history) // ICACHE_LRU
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

    // synthesis parallel_case full_case
    case (ic_state)
      IC_READ: begin
        // potentially we should update LRU counters ...
        access_lru_history = way_hit;
        tag_lru_in = next_lru_history;
        // ... and we do it by read-hit only
        if (is_cacheble & fetch_req_hit_i & hit & ~(immu_an_except_i | flush_by_ctrl_i)) begin // on read-hit
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
            if (lru_way_r[w2]) begin
              tag_way_in[w2][TAGMEM_WAY_VALID] = 1'b0;
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
            tag_way_in[w2] =
              lru_way_r[w2] ? {1'b1,ibus_burst_adr_i[OPTION_ICACHE_LIMIT_WIDTH-1:WAY_WIDTH]} :
                              tag_way_save[w2];
          end
          access_lru_history = lru_way_r;
          tag_lru_in         = next_lru_history;
          tag_we             = 1'b1;
        end
      end

      IC_INVALIDATE: begin
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


  // Packed TAG-RAM data input
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
  assign tag_windex = ic_refill     ? ibus_burst_adr_i[WAY_WIDTH-1:OPTION_ICACHE_BLOCK_WIDTH]  : // TAG_WR_ADDR at re-fill
                      ic_invalidate ? tag_invdex                                               : // TAG_WR_ADDR at invalidate
                                      phys_addr_tag_i[WAY_WIDTH-1:OPTION_ICACHE_BLOCK_WIDTH];    // TAG_WR_ADDR at update LRU field

  // TAG read address
  assign tag_rindex = phys_addr_idx_i[WAY_WIDTH-1:OPTION_ICACHE_BLOCK_WIDTH];

  // Read/Write into same address
  wire tag_rw_same_addr = (tag_rindex == tag_windex);

  // Read/Write port (*_rwp_*) write
  wire tr_rwp_we = tag_we & ic_ram_re & tag_rw_same_addr;

  // Write-only port (*_wp_*) enable
  wire tr_wp_en = tag_we & (~ic_ram_re | ~tag_rw_same_addr);

  // TAG-RAM instance
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
    .en_a   (ic_ram_re), // ICACHE_TAG_RAM
    .we_a   (tr_rwp_we), // ICACHE_TAG_RAM
    .addr_a (tag_rindex), // ICACHE_TAG_RAM
    .din_a  (tag_din), // ICACHE_TAG_RAM
    .dout_a (tag_dout), // ICACHE_TAG_RAM
    // port "b": Write if no RW-conflict
    .en_b   (tr_wp_en), // ICACHE_TAG_RAM
    .we_b   (tag_we), // ICACHE_TAG_RAM
    .addr_b (tag_windex), // ICACHE_TAG_RAM
    .din_b  (tag_din), // ICACHE_TAG_RAM
    .dout_b () // ICACHE_TAG_RAM
  );

endmodule // mor1kx_icache_marocchino
