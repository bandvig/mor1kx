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
//   Copyright (C) 2015-2017 Andrey Bacherov                          //
//                           avbacherov@opencores.org                 //
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
  parameter OPTION_OPERAND_WIDTH        = 32,
  parameter OPTION_DCACHE_BLOCK_WIDTH   =  5,
  parameter OPTION_DCACHE_SET_WIDTH     =  9,
  parameter OPTION_DCACHE_WAYS          =  2,
  parameter OPTION_DCACHE_LIMIT_WIDTH   = 32,
  parameter OPTION_DCACHE_SNOOP         = "NONE",
  parameter OPTION_DCACHE_CLEAR_ON_INIT =  0
)
(
  // clock & reset
  input                                 cpu_clk,
  input                                 cpu_rst,

  // pipe controls
  input                                 lsu_s1_adv_i,
  input                                 lsu_s2_adv_i,
  input                                 pipeline_flush_i,

  // configuration
  input                                 dc_enable_i,

  // exceptions
  input                                 s2o_excepts_addr_i,
  input                                 dbus_err_i,

  // Regular operation
  //  # addresses and "DCHACHE inhibit" flag
  input      [OPTION_OPERAND_WIDTH-1:0] virt_addr_idx_i,
  input      [OPTION_OPERAND_WIDTH-1:0] virt_addr_s1o_i,
  input      [OPTION_OPERAND_WIDTH-1:0] virt_addr_s2o_i,
  input      [OPTION_OPERAND_WIDTH-1:0] phys_addr_s2t_i,
  input                                 dmmu_cache_inhibit_i,
  //  # DCACHE regular answer
  input                                 s1o_op_lsu_load_i,
  output                                dc_ack_read_o,
  output reg [OPTION_OPERAND_WIDTH-1:0] dc_dat_o,
  //  # STORE format / store data / do storing
  input                                 s1o_op_lsu_store_i,
  input                           [3:0] dbus_bsel_i,
  input      [OPTION_OPERAND_WIDTH-1:0] dbus_sdat_i,
  input      [OPTION_OPERAND_WIDTH-1:0] dc_dat_s2o_i,
  input                                 dc_store_allowed_i,
  input                                 dc_store_cancel_i,
  //  # Atomics
  input                                 s1o_op_lsu_atomic_i,

  // re-fill
  output                                dc_refill_req_o,
  input                                 dc_refill_allowed_i,
  output reg                            refill_first_o,
  input      [OPTION_OPERAND_WIDTH-1:0] phys_addr_s2o_i,
  input      [OPTION_OPERAND_WIDTH-1:0] dbus_dat_i,
  input                                 dbus_burst_last_i,
  input                                 dbus_ack_i,

  // DBUS read request
  output                                dbus_read_req_o,

  // SNOOP
  // Snoop address
  input      [OPTION_OPERAND_WIDTH-1:0] snoop_adr_i,
  // Snoop event in this cycle
  input                                 snoop_event_i,
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
  localparam  [6:0] DC_CHECK         = 7'b0000001,
                    DC_WRITE         = 7'b0000010,
                    DC_REFILL        = 7'b0000100,
                    DC_INV_BY_MTSPR  = 7'b0001000,
                    DC_INV_BY_ATOMIC = 7'b0010000,
                    DC_RST           = 7'b0100000,
                    DC_SPR_ACK       = 7'b1000000;
  // FSM state register
  reg [6:0] dc_state;
  // FSM state signals
  wire   dc_check         = dc_state[0];
  wire   dc_refill        = dc_state[2];
  wire   dc_inv_by_mtspr  = dc_state[3];
  wire   dc_inv_by_atomic = dc_state[4];
  assign spr_bus_ack_o    = dc_state[6];



  // The index we read and write from tag memory
  wire [OPTION_DCACHE_SET_WIDTH-1:0] tag_rindex;
  reg  [OPTION_DCACHE_SET_WIDTH-1:0] tag_windex;

  // The data from the tag memory
  wire       [TAGMEM_WIDTH-1:0] tag_dout;
  wire   [TAGMEM_WAY_WIDTH-1:0] tag_dout_way [OPTION_DCACHE_WAYS-1:0];

  // The data to the tag memory
  wire      [TAGMEM_WIDTH-1:0] tag_din;
  reg   [TAGMEM_WAY_WIDTH-1:0] tag_din_way [OPTION_DCACHE_WAYS-1:0];
  reg [TAG_LRU_WIDTH_BITS-1:0] tag_din_lru;

  reg   [TAGMEM_WAY_WIDTH-1:0] tag_s2o_way [OPTION_DCACHE_WAYS-1:0];

  // Whether to write to the tag memory in this cycle
  reg                          tag_we;



  // WAYs related
  reg    [OPTION_DCACHE_WAYS-1:0] way_we; // Write signals per way
  reg  [OPTION_OPERAND_WIDTH-1:0] way_din;
  reg             [WAY_WIDTH-3:0] way_windex;
  wire            [WAY_WIDTH-3:0] way_rindex;
  wire [OPTION_OPERAND_WIDTH-1:0] way_dout [OPTION_DCACHE_WAYS-1:0];


  // Does any way hit?
  wire                          dc_hit;
  wire [OPTION_DCACHE_WAYS-1:0] dc_hit_way;
  reg  [OPTION_DCACHE_WAYS-1:0] s2o_hit_way; // latched in stage #2 register
  reg  [OPTION_DCACHE_WAYS-1:0] lru_hit_way; // for update LRU history

  // This is the least recently used value before access the memory.
  // Those are one hot encoded.
  wire [OPTION_DCACHE_WAYS-1:0] lru_way;
  reg  [OPTION_DCACHE_WAYS-1:0] lru_way_refill_r; // for re-fill

  // The current LRU history as read from tag memory.
  reg  [TAG_LRU_WIDTH_BITS-1:0] lru_history_curr;
  reg  [TAG_LRU_WIDTH_BITS-1:0] lru_history_curr_s2o; // registered
  // The update value after we accessed it to write back to tag memory.
  wire [TAG_LRU_WIDTH_BITS-1:0] lru_history_next;



  // Extract index to read from snooped address
  wire [OPTION_DCACHE_SET_WIDTH-1:0] snoop_index;
  assign snoop_index = snoop_adr_i[WAY_WIDTH-1:OPTION_DCACHE_BLOCK_WIDTH];

  // Register that is high one cycle after the actual snoop event to
  // drive the comparison
  reg                                snoop_check;
  // Register that stores the tag for one cycle
  reg                [TAG_WIDTH-1:0] snoop_tag;
  // Also store the index for one cycle, for the succeeding write access
  reg  [OPTION_DCACHE_SET_WIDTH-1:0] snoop_windex;
  // Snoop Tag RAM input
  wire [OPTION_DCACHE_SET_WIDTH-1:0] snoop_rindex;

  // Snoop tag memory interface
  // Data out of tag memory
  wire       [TAGMEM_WIDTH-1:0] snoop_dout;
  // Each ways information in the tag memory
  wire   [TAGMEM_WAY_WIDTH-1:0] snoop_way_out [OPTION_DCACHE_WAYS-1:0];
  // Whether the way hits
  wire [OPTION_DCACHE_WAYS-1:0] snoop_way_hit;


  genvar i;

  //
  // Local copy of DCACHE-related control bit(s) to simplify routing
  //
  // MT(F)SPR_RULE:
  //   Before issuing MT(F)SPR, OMAN waits till order control buffer has become
  // empty. Also we don't issue new instruction till l.mf(t)spr completion.
  //   So, it is safely to detect changing DCACHE-related control bit(s) here
  // and update local copies.
  //
  reg dc_enable_r;
  // ---
  always @(posedge cpu_clk) begin
    if (cpu_rst)
      dc_enable_r <= 1'b0; // pipeline_flush doesn't switch caches on/off
    else if (dc_enable_r ^ dc_enable_i)
      dc_enable_r <= dc_enable_i;
  end


  //   Hack? Work around IMMU?
  // Addresses 0x8******* are treated as non-cacheble regardless DMMU's flag.
  wire dc_check_limit_width;
  // ---
  generate
  if (OPTION_DCACHE_LIMIT_WIDTH == OPTION_OPERAND_WIDTH)
    assign dc_check_limit_width = 1'b1;
  else if (OPTION_DCACHE_LIMIT_WIDTH < OPTION_OPERAND_WIDTH)
    assign dc_check_limit_width =
      (phys_addr_s2t_i[OPTION_OPERAND_WIDTH-1:OPTION_DCACHE_LIMIT_WIDTH] == 0);
  else begin
    initial begin
      $display("DCACHE ERROR: OPTION_DCACHE_LIMIT_WIDTH > OPTION_OPERAND_WIDTH");
      $finish();
    end
  end
  endgenerate


  // detect per-way hit
  generate
  for (i = 0; i < OPTION_DCACHE_WAYS; i=i+1) begin : gen_per_way_hit
    // Multiplex the way entries in the tag memory
    assign tag_din[(i+1)*TAGMEM_WAY_WIDTH-1:i*TAGMEM_WAY_WIDTH] = tag_din_way[i];
    assign tag_dout_way[i] = tag_dout[(i+1)*TAGMEM_WAY_WIDTH-1:i*TAGMEM_WAY_WIDTH];

    // compare stored tag with incoming tag and check valid bit
    assign dc_hit_way[i] = tag_dout_way[i][TAGMEM_WAY_VALID] &
      (tag_dout_way[i][TAG_WIDTH-1:0] == phys_addr_s2t_i[OPTION_DCACHE_LIMIT_WIDTH-1:WAY_WIDTH]); // hit detection

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

  assign dc_hit = |dc_hit_way;

  assign snoop_hit_o = (|snoop_way_hit) & snoop_check;


  // Is the area cachable?
  wire   is_cacheble     = dc_enable_r & dc_check_limit_width & (~dmmu_cache_inhibit_i);

  // for write processing
  wire   dc_ack_write    = s1o_op_lsu_store_i & (~s1o_op_lsu_atomic_i) & is_cacheble &   dc_hit  & (~snoop_hit_o);
  reg    s2o_dc_ack_write;

  // if requested data were fetched before snoop-hit, it is valid
  assign dc_ack_read_o   = s1o_op_lsu_load_i  & (~s1o_op_lsu_atomic_i) & is_cacheble &   dc_hit  & (~snoop_hit_o);
  reg    s2o_dc_ack_read;

  // re-fill reqest
  assign dc_refill_req_o = s1o_op_lsu_load_i  & (~s1o_op_lsu_atomic_i) & is_cacheble & (~dc_hit) & (~snoop_hit_o);

  // DBUS access request
  assign dbus_read_req_o = s1o_op_lsu_load_i  & (s1o_op_lsu_atomic_i | (~is_cacheble));


  // read result if success
  integer w0;
  always @ (*) begin
    dc_dat_o = {OPTION_OPERAND_WIDTH{1'b0}};
    // ---
    for (w0 = 0; w0 < OPTION_DCACHE_WAYS; w0=w0+1) begin : mux_dat_o
      if (dc_hit_way[w0])
        dc_dat_o = way_dout[w0];
    end
  end // always



  /*
   * SPR bus interface
   *  # Only invalidate command is implemented
   *  # In MAROCCHINO pipeline l.mf(t)spr instructions are executed
   *    if pipeline is stalled.
   */

  assign spr_bus_dat_o = {OPTION_OPERAND_WIDTH{1'b0}};

  // An invalidate request is either a block flush or a block invalidate
  wire spr_bus_dc_invalidate = spr_bus_stb_i & spr_bus_we_i &
                               ((spr_bus_addr_i == `OR1K_SPR_DCBFR_ADDR) |
                                (spr_bus_addr_i == `OR1K_SPR_DCBIR_ADDR));



  /*
   * DCACHE FSM controls
   */

  // go to idle and block update WAY/TAG RAM
  wire dc_cancel = s2o_excepts_addr_i | pipeline_flush_i;


  /*
   * DCACHE FSM
   */

  // DCACHE FSM: state switching
  always @(posedge cpu_clk) begin
    if (cpu_rst) begin
      dc_state <= DC_RST; // on reset
    end
    else begin
      // synthesis parallel_case full_case
      case (dc_state)
        DC_CHECK: begin
          if (dc_refill_allowed_i) begin // check -> re-fill
            dc_state <= DC_REFILL; // check -> re-fill
          end
          else if (dc_cancel | snoop_hit_o) begin // keep check
            dc_state <= DC_CHECK;
          end
          else if (spr_bus_dc_invalidate) begin // check -> invalidate by l.mtspr
            dc_state <= DC_INV_BY_MTSPR; // check -> invalidate by l.mtspr
          end
          else if (lsu_s2_adv_i) begin // check -> write / keep check
            dc_state <= s1o_op_lsu_atomic_i ? DC_INV_BY_ATOMIC :      // check -> invalidate by lwa/swa
                          (s1o_op_lsu_store_i ? DC_WRITE : DC_CHECK); // check -> write / keep check
          end
        end // check

        DC_WRITE: begin
          // stage #2 advance is impossible till either
          // write completion or DBUS error (for l.swa)
          if (dc_store_allowed_i | dc_store_cancel_i) // done / abort dc-write
            dc_state <= DC_CHECK; // done / abort write
        end // write

        DC_REFILL: begin
          // 1) Abort re-fill on snoop-hit
          //    TODO: only abort on snoop-hits to re-fill address
          // 2) In according with WISHBONE-B3 rule 3.45:
          //    "SLAVE MUST NOT assert more than one of ACK, ERR or RTY"
          if (snoop_hit_o) begin
            dc_state <= DC_CHECK;  // on snoop-hit during re-fill
          end
          else if (dbus_err_i | (dbus_ack_i & dbus_burst_last_i)) begin  // abort re-fill
            dc_state <= DC_CHECK;     // on dbus error during re-fill
          end
        end // re-fill

        DC_INV_BY_MTSPR: begin
          if (~snoop_hit_o) begin // wait till snoop-inv completion
            dc_state <= DC_SPR_ACK; // invalidate by l.mtspr -> ack for SPR BUS
          end
        end

        DC_INV_BY_ATOMIC: begin
          dc_state <= DC_CHECK; // invalidate by lwa/swa -> check
        end

        DC_SPR_ACK,
        DC_RST: begin
          dc_state <= DC_CHECK;  // on doing reset or ack for SPR BUS
        end

        default:;
      endcase
    end
  end // at clock

  //
  // DCACHE FSM: various data
  //
  always @(posedge cpu_clk) begin
    // snoop processing
    if (snoop_event_i) begin
      //
      // If there is a snoop event, we need to store this
      // information. This happens independent of whether we
      // have a snoop tag memory or not.
      //
      snoop_check  <= 1'b1;
      snoop_windex <= snoop_index; // on snoop-event
      snoop_tag    <= snoop_adr_i[OPTION_DCACHE_LIMIT_WIDTH-1:WAY_WIDTH];
    end
    else begin
      snoop_check  <= 1'b0;
    end

    // states switching
    // synthesis parallel_case full_case
    case (dc_state)
      DC_CHECK: begin
        // next states
        if (dc_refill_allowed_i) begin
          lru_way_refill_r <= lru_way;    // check -> re-fill
          refill_first_o   <= 1'b1;       // check -> re-fill
        end
        else if (dc_cancel | snoop_hit_o) begin // keep check
        end
        else if (lsu_s2_adv_i) begin // check -> write / keep check
          s2o_dc_ack_write <= dc_ack_write; // check -> write / keep check
        end
      end // check

      DC_WRITE: begin
        if (dc_store_allowed_i | dc_store_cancel_i) begin // done / abort dc-write
          s2o_dc_ack_write <= 1'b0;     // done / abort dc-write
        end
      end // write

      DC_REFILL: begin
        // 1) Abort re-fill on snoop-hit
        //    TODO: only abort on snoop-hits to re-fill address
        // 2) In according with WISHBONE-B3 rule 3.45:
        //    "SLAVE MUST NOT assert more than one of ACK, ERR or RTY"
        if (snoop_hit_o) begin
          refill_first_o   <= 1'b0;     // on snoop-hit during re-fill
          lru_way_refill_r <= {OPTION_DCACHE_WAYS{1'b0}};  // on snoop-hit during re-fill
        end
        else if (dbus_err_i) begin  // abort re-fill
          refill_first_o   <= 1'b0; // on dbus error during re-fill
          lru_way_refill_r <= {OPTION_DCACHE_WAYS{1'b0}};  // on dbus error during re-fill
        end
        else if (dbus_ack_i) begin
          refill_first_o <= 1'b0; // any re-fill
          if (dbus_burst_last_i) begin
            lru_way_refill_r <= {OPTION_DCACHE_WAYS{1'b0}}; // on last re-fill
          end
        end // snoop-hit / dbus-ack
      end // re-fill

      DC_RST: begin
        refill_first_o   <= 1'b0; // on default
        lru_way_refill_r <= {OPTION_DCACHE_WAYS{1'b0}}; // on default
        s2o_dc_ack_write <= 1'b0; // on default
        snoop_check      <= 1'b0; // on default
        snoop_tag        <= {TAG_WIDTH{1'b0}}; // on default
        snoop_windex     <= {OPTION_DCACHE_SET_WIDTH{1'b0}}; // on default
      end

      default:;
    endcase
  end // @ clock

  //
  // For re-fill we use local copy of bus-bridge's burst address
  //  accumulator to generate WAY-RAM index.
  // The approach increases logic locality and makes routing easier.
  //
  reg  [OPTION_OPERAND_WIDTH-1:0] virt_addr_rfl_r;
  wire [OPTION_OPERAND_WIDTH-1:0] virt_addr_rfl_next;
  // cache block length is 5 -> burst length is 8: 32 bytes = (8 words x 32 bits/word)
  // cache block length is 4 -> burst length is 4: 16 bytes = (4 words x 32 bits/word)
  assign virt_addr_rfl_next = (OPTION_DCACHE_BLOCK_WIDTH == 5) ?
    {virt_addr_rfl_r[31:5], virt_addr_rfl_r[4:0] + 5'd4} : // 32 byte = (8 words x 32 bits/word)
    {virt_addr_rfl_r[31:4], virt_addr_rfl_r[3:0] + 4'd4};  // 16 byte = (4 words x 32 bits/word)
  // ---
  always @(posedge cpu_clk) begin
    // synthesis parallel_case full_case
    case (dc_state)
      DC_CHECK: begin // re-fill address register
        if (spr_bus_dc_invalidate)  // set re-fill address register to invaldate by l.mtspr
          virt_addr_rfl_r <= spr_bus_dat_i;   // invaldate by l.mtspr
        else if (lsu_s2_adv_i)      // set re-fill address register to initial re-fill address
          virt_addr_rfl_r <= virt_addr_s1o_i; // prepare to re-fill (copy of LSU::s2o_virt_addr)
      end // check
      DC_REFILL: begin
        if (dbus_ack_i)
          virt_addr_rfl_r <= virt_addr_rfl_next;  // re-fill in progress
      end // re-fill
      DC_SPR_ACK: begin // re-fill address register
        virt_addr_rfl_r <= virt_addr_s2o_i; // restore after invalidation by l.mtspr
      end
      default:;
    endcase
  end // @ clock


  // s2o_* latches for WAY-fields of TAG
  integer w1;
  // ---
  always @(posedge cpu_clk) begin
    if (lsu_s2_adv_i) begin
      for (w1 = 0; w1 < OPTION_DCACHE_WAYS; w1 = w1 + 1) begin
        tag_s2o_way[w1] <= tag_dout_way[w1];
      end
    end
  end // @clock

  // s2o_* latch for hit and current LRU history
  always @(posedge cpu_clk) begin
    if (lsu_s2_adv_i) begin
      s2o_hit_way          <= dc_hit_way;
      lru_history_curr_s2o <= tag_dout[TAG_LRU_MSB:TAG_LRU_LSB];
    end
  end // @clock


  // Local copy of LSU's s2o_dc_ack_read, but 1-clock length
  always @(posedge cpu_clk) begin
    if (cpu_rst | pipeline_flush_i)
      s2o_dc_ack_read <= 1'b0;
    else if (lsu_s2_adv_i)
      s2o_dc_ack_read <= dc_ack_read_o;
    else
      s2o_dc_ack_read <= 1'b0;
  end // @clock


  //
  // This is the combinational part of the state machine that
  // interfaces the tag and way memories.
  //

  // TAG READ ADDR
  assign tag_rindex = lsu_s1_adv_i ?
                        virt_addr_idx_i[WAY_WIDTH-1:OPTION_DCACHE_BLOCK_WIDTH] : // TAG READ ADDR if advance stage #1
                        virt_addr_s1o_i[WAY_WIDTH-1:OPTION_DCACHE_BLOCK_WIDTH];  // TAG READ ADDR if stall   stage #1

  // WAY READ ADDR
  assign way_rindex = lsu_s1_adv_i ?
                        virt_addr_idx_i[WAY_WIDTH-1:2] :  // WAY READ ADDR if advance stage #1
                        virt_addr_s1o_i[WAY_WIDTH-1:2];   // WAY READ ADDR if stall   stage #1

  integer w2;
  always @(*) begin
    // default prepare data for LRU update at hit or for re-fill initiation
    //  -- input for LRU calculator
    lru_hit_way      = s2o_hit_way; // default
    lru_history_curr = lru_history_curr_s2o; // default
    //  -- output of LRU calculator
    tag_din_lru = lru_history_next; // default
    //  -- other TAG-RAM fields
    for (w2 = 0; w2 < OPTION_DCACHE_WAYS; w2 = w2 + 1) begin
      tag_din_way[w2] = tag_s2o_way[w2]; // default
    end

    //   As way size is equal to page one we able to use either
    // physical or virtual indexing.

    // TAG-RAM
    tag_we     = 1'b0; // default
    tag_windex = virt_addr_rfl_r[WAY_WIDTH-1:OPTION_DCACHE_BLOCK_WIDTH]; // default: write-hit / re-fill / invalidate_by_any

    // WAYS-RAM
    way_we     = {OPTION_DCACHE_WAYS{1'b0}};      // default
    way_din    = dbus_sdat_i;                     // default: for write-hit
    way_windex = virt_addr_rfl_r[WAY_WIDTH-1:2];  // default: for write-hit / re-fill


    if (snoop_hit_o) begin
      // This is the write access
      tag_we     = 1'b1; // on snoop-hit
      tag_windex = snoop_windex;
      for (w2 = 0; w2 < OPTION_DCACHE_WAYS; w2 = w2 + 1) begin
        tag_din_way[w2] = snoop_way_out[w2] & {TAGMEM_WAY_WIDTH{~snoop_way_hit[w2]}}; // zero where hit
      end
    end
    else begin
      // synthesis parallel_case full_case
      case (dc_state)
        DC_CHECK: begin
          if (s2o_dc_ack_read & (~s2o_excepts_addr_i)) begin // on read-hit
            tag_we = 1'b1; // on read-hit
          end
        end

        DC_WRITE: begin
          // prepare data for write ahead
          if (~dbus_bsel_i[3]) way_din[31:24] = dc_dat_s2o_i[31:24]; // on write-hit
          if (~dbus_bsel_i[2]) way_din[23:16] = dc_dat_s2o_i[23:16]; // on write-hit
          if (~dbus_bsel_i[1]) way_din[15:8]  = dc_dat_s2o_i[15: 8]; // on write-hit
          if (~dbus_bsel_i[0]) way_din[7:0]   = dc_dat_s2o_i[ 7: 0]; // on write-hit
          // real update on write-hit only
          //  # pipeline flush and address exceptions are in "allowed" flag
          if (s2o_dc_ack_write & dc_store_allowed_i & (~dc_store_cancel_i)) begin // on write-hit
            way_we = s2o_hit_way; // on write-hit
            tag_we = 1'b1;        // on write-hit
          end
        end

        DC_REFILL: begin
          // WAYs
          way_din = dbus_dat_i; // on re-fill
          // TAGs
          // For re-fill (tag_windex == virt_addr_rfl_r) is default setting
          //  (a) In according with WISHBONE-B3 rule 3.45:
          // "SLAVE MUST NOT assert more than one of ACK, ERR or RTY"
          //  (b) We don't interrupt re-fill on flushing, so the only reason
          // for invalidation is DBUS error occurence
          if (dbus_err_i) begin // during re-fill
            for (w2 = 0; w2 < OPTION_DCACHE_WAYS; w2 = w2 + 1) begin
              if (lru_way_refill_r[w2])
                tag_din_way[w2][TAGMEM_WAY_VALID] = 1'b0; // DBUS error  during re-fill
            end
            // MAROCCHINO_TODO: how to handle LRU in the case?
            // ---
            tag_we = 1'b1; // invalidate by DBUS error during re-fill
          end
          else if (dbus_ack_i) begin
            // LRU WAY content update each DBUS ACK
            way_we = lru_way_refill_r; // on re-fill: write the data to the way that is replaced (which is the LRU)
            // After re-fill update the tag memory entry of the
            // filled way with the LRU history, the tag and set
            // valid to 1.
            if (dbus_burst_last_i) begin
              for (w2 = 0; w2 < OPTION_DCACHE_WAYS; w2 = w2 + 1) begin
                if (lru_way_refill_r[w2])
                  tag_din_way[w2] = {1'b1, phys_addr_s2o_i[OPTION_DCACHE_LIMIT_WIDTH-1:WAY_WIDTH]};
              end
              lru_hit_way = lru_way_refill_r; // last re-fill
              // ---
              tag_we = 1'b1; // last re-fill
            end
          end
        end // re-fill

        DC_INV_BY_MTSPR: begin
          //
          // Lazy invalidation, invalidate everything that matches tag address
          //  # Pay attention we needn't to take into accaunt exceptions or
          //    pipe flusing here. It because, MARROCCHINO executes
          //    l.mf(t)spr commands after successfull completion of
          //    all previous instructions.
          //
          tag_we      = 1'b1;     // on invalidate by l.mtspr
          tag_din_lru = 0;        // on invalidate by l.mtspr
          for (w2 = 0; w2 < OPTION_DCACHE_WAYS; w2 = w2 + 1) begin
            tag_din_way[w2] = 0;  // on invalidate by l.mtspr
          end
        end

        DC_INV_BY_ATOMIC: begin
          //
          // With the simplest approach we just force linked
          // address to be not cachable.
          // MAROCCHINO_TODO: more accurate processing if linked address is cachable.
          //
          tag_we = 1'b1;                                  // on invalidate by lwa/swa
          for (w2 = 0; w2 < OPTION_DCACHE_WAYS; w2 = w2 + 1) begin
            if (s2o_hit_way[w2])                          // on invalidate by lwa/swa
              tag_din_way[w2][TAGMEM_WAY_VALID] = 1'b0;   // on invalidate by lwa/swa
          end
        end

        default:;
      endcase
    end
  end


  // Controls for read/write port.
  wire [OPTION_DCACHE_WAYS-1:0] way_rwp_en;
  wire [OPTION_DCACHE_WAYS-1:0] way_rwp_we;
  // ---
  wire way_rwp_same_addr = (way_windex == way_rindex);

  // Controls for write-only port
  wire [OPTION_DCACHE_WAYS-1:0] way_wp_we;
  // ---
  wire way_wp_diff_addr = (way_windex != way_rindex);

  // WAY-RAM Blocks
  generate
  for (i = 0; i < OPTION_DCACHE_WAYS; i=i+1) begin : way_memories
    // Controls for read/write port.
    assign way_rwp_we[i] = way_we[i] & way_rwp_same_addr;
    assign way_rwp_en[i] = lsu_s1_adv_i | way_rwp_we[i];

    // Controls for write-only port
    assign way_wp_we[i] = way_we[i] & way_wp_diff_addr;

    // WAY-RAM instances
    mor1kx_dpram_en_w1st_sclk
    #(
      .ADDR_WIDTH     (WAY_WIDTH-2), // DCACHE_WAY_RAM
      .DATA_WIDTH     (OPTION_OPERAND_WIDTH), // DCACHE_WAY_RAM
      .CLEAR_ON_INIT  (OPTION_DCACHE_CLEAR_ON_INIT) // DCACHE_WAY_RAM
    )
    dc_way_ram
    (
      // common clock
      .clk            (cpu_clk), // DCACHE_WAY_RAM
      // port "a"
      .en_a           (way_rwp_en[i]), // DCACHE_WAY_RAM
      .we_a           (way_rwp_we[i]), // DCACHE_WAY_RAM
      .addr_a         (way_rindex), // DCACHE_WAY_RAM
      .din_a          (way_din), // DCACHE_WAY_RAM
      .dout_a         (way_dout[i]), // DCACHE_WAY_RAM
      // port "b"
      .en_b           (way_wp_we[i]), // DCACHE_WAY_RAM
      .we_b           (1'b1), // DCACHE_WAY_RAM
      .addr_b         (way_windex), // DCACHE_WAY_RAM
      .din_b          (way_din), // DCACHE_WAY_RAM
      .dout_b         () // DCACHE_WAY_RAM
    );
  end
  endgenerate

  // LRU calculator
  generate
  /* verilator lint_off WIDTH */
  if (OPTION_DCACHE_WAYS >= 2) begin : gen_u_lru
  /* verilator lint_on WIDTH */
    mor1kx_cache_lru
    #(
      .NUMWAYS(OPTION_DCACHE_WAYS)
    )
    dc_lru
    (
      // Outputs
      .update      (lru_history_next),
      .lru_pre     (lru_way),
      .lru_post    (),
      // Inputs
      .current     (lru_history_curr),
      .access      (lru_hit_way)
    );

    // Multiplex the LRU history from and to tag memory
    assign tag_din[TAG_LRU_MSB:TAG_LRU_LSB] = tag_din_lru;
  end
  else begin // single way
    assign lru_history_next = lru_history_curr; // single way
    assign lru_way          = 1'b1;             // single way
  end
  endgenerate


  // Read/Write port (*_rwp_*) controls
  wire tag_rwp_we = tag_we & (tag_rindex == tag_windex);
  wire tag_rwp_en = lsu_s1_adv_i | tag_rwp_we;

  // Write-only port (*_wp_*) enable
  wire tag_wp_en = tag_we & (tag_rindex != tag_windex);

  // TAG-RAM instance
  mor1kx_dpram_en_w1st_sclk
  #(
    .ADDR_WIDTH     (OPTION_DCACHE_SET_WIDTH), // DCAHCE_TAG_RAM
    .DATA_WIDTH     (TAGMEM_WIDTH), // DCAHCE_TAG_RAM
    .CLEAR_ON_INIT  (OPTION_DCACHE_CLEAR_ON_INIT) // DCAHCE_TAG_RAM
  )
  dc_tag_ram
  (
    // common clock
    .clk            (cpu_clk), // DCAHCE_TAG_RAM
    // port "a": Read / Write (for RW-conflict case)
    .en_a           (tag_rwp_en), // DCAHCE_TAG_RAM
    .we_a           (tag_rwp_we), // DCAHCE_TAG_RAM
    .addr_a         (tag_rindex), // DCAHCE_TAG_RAM
    .din_a          (tag_din), // DCAHCE_TAG_RAM
    .dout_a         (tag_dout), // DCAHCE_TAG_RAM
    // port "b": Write if no RW-conflict
    .en_b           (tag_wp_en), // DCAHCE_TAG_RAM
    .we_b           (1'b1), // DCAHCE_TAG_RAM
    .addr_b         (tag_windex), // DCAHCE_TAG_RAM
    .din_b          (tag_din), // DCAHCE_TAG_RAM
    .dout_b         () // DCAHCE_TAG_RAM
  );



  generate
  /* verilator lint_off WIDTH */
  if (OPTION_DCACHE_SNOOP != "NONE") begin : st_ram
  /* verilator lint_on WIDTH */
    // snoop RAM read & write
    //  # we force snoop invalidation through RW-port
    //    to provide snoop-hit off
    wire str_re = (snoop_event_i & ~snoop_check) | snoop_hit_o;
    wire str_we = dc_inv_by_mtspr | snoop_hit_o;

    // address for Read/Write port
    //  # for soop-hit case tag-windex is equal to snoop-windex
    assign snoop_rindex = snoop_hit_o ? tag_windex : snoop_index;

    // same addresses for read and write
    wire str_rw_same_addr = (tag_windex == snoop_rindex);

    // Read / Write port (*_rwp_*) controls
    wire str_rwp_we = str_we & str_re & str_rw_same_addr;

    // Write-only port (*_wp_*) controls
    wire str_wp_en  = str_we & (~str_re | ~str_rw_same_addr);

    // SNOOP-TAG-RAM instance
    mor1kx_dpram_en_w1st_sclk
    #(
      .ADDR_WIDTH     (OPTION_DCACHE_SET_WIDTH), // DCACHE_SNOOP_TAG_RAM
      .DATA_WIDTH     (TAGMEM_WIDTH), // DCACHE_SNOOP_TAG_RAM
      .CLEAR_ON_INIT  (OPTION_DCACHE_CLEAR_ON_INIT) // DCACHE_SNOOP_TAG_RAM
    )
    dc_snoop_tag_ram
    (
      // clock
      .clk    (cpu_clk), // DCACHE_SNOOP_TAG_RAM
      // port "a": Read / Write (for RW-conflict case)
      .en_a   (str_re), // DCACHE_SNOOP_TAG_RAM
      .we_a   (str_rwp_we), // DCACHE_SNOOP_TAG_RAM
      .addr_a (snoop_rindex), // DCACHE_SNOOP_TAG_RAM
      .din_a  (tag_din), // DCACHE_SNOOP_TAG_RAM
      .dout_a (snoop_dout), // DCACHE_SNOOP_TAG_RAM
      // port "b": Write if no RW-conflict
      .en_b   (str_wp_en), // DCACHE_SNOOP_TAG_RAM
      .we_b   (1'b1), // DCACHE_SNOOP_TAG_RAM
      .addr_b (tag_windex), // DCACHE_SNOOP_TAG_RAM
      .din_b  (tag_din), // DCACHE_SNOOP_TAG_RAM
      .dout_b () // DCACHE_SNOOP_TAG_RAM
    );
  end
  endgenerate

endmodule // mor1kx_dcache_marocchino
