////////////////////////////////////////////////////////////////////////
//                                                                    //
//  mor1kx_marocchino_alone                                           //
//                                                                    //
//  Description: Top level for stand alone mor1kx with                //
//               MAROCCHINO pipeline.                                 //
//               Based on mor1kx.v with removed AVALON bus and        //
//               use exactly mor1kx_cpu_marocchino excluding          //
//               instances of other variants of pipeline.             //
//                                                                    //
////////////////////////////////////////////////////////////////////////
//                                                                    //
//   Copyright (C) 2012 Julius Baxter                                 //
//                      juliusbaxter@gmail.com                        //
//                                                                    //
//                      Stefan Kristiansson                           //
//                      stefan.kristiansson@saunalahti.fi             //
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

module mor1kx_marocchino_alone
#(
  parameter OPTION_OPERAND_WIDTH        = 32,

  // data cache configuration
  parameter OPTION_DCACHE_BLOCK_WIDTH   =  5,
  parameter OPTION_DCACHE_SET_WIDTH     =  9,
  parameter OPTION_DCACHE_WAYS          =  2,
  parameter OPTION_DCACHE_LIMIT_WIDTH   = 32,
  parameter OPTION_DCACHE_SNOOP         = "NONE",
  parameter OPTION_DCACHE_CLEAR_ON_INIT =  0, // !!! activate for simulation only !!!

  // data mmu
  parameter FEATURE_DMMU_HW_TLB_RELOAD  = "NONE",
  parameter OPTION_DMMU_SET_WIDTH       =  6,
  parameter OPTION_DMMU_WAYS            =  1,
  parameter OPTION_DMMU_CLEAR_ON_INIT   =  0, // !!! activate for simulation only !!!

  // store buffer
  parameter OPTION_STORE_BUFFER_DEPTH_WIDTH   = 4, // 16 taps
  parameter OPTION_STORE_BUFFER_CLEAR_ON_INIT = 0, // !!! activate for simulation only !!!

  // istruction cache
  parameter OPTION_ICACHE_BLOCK_WIDTH   =  5,
  parameter OPTION_ICACHE_SET_WIDTH     =  9,
  parameter OPTION_ICACHE_WAYS          =  2,
  parameter OPTION_ICACHE_LIMIT_WIDTH   = 32,
  parameter OPTION_ICACHE_CLEAR_ON_INIT =  0, // !!! activate for simulation only !!!

  // instruction mmu
  parameter FEATURE_IMMU_HW_TLB_RELOAD = "NONE",
  parameter OPTION_IMMU_SET_WIDTH      =  6,
  parameter OPTION_IMMU_WAYS           =  1,
  parameter OPTION_IMMU_CLEAR_ON_INIT  =  0, // !!! activate for simulation only !!!

  // Timer
  parameter TIMER_CLOCK_DOMAIN         = "CPU_CLOCK", // "WB_CLOCK" / "CPU_CLOCK" (default)

  // Debug unit
  parameter FEATURE_DEBUGUNIT          = "NONE",
  parameter FEATURE_PERFCOUNTERS       = "NONE",

  // PIC
  parameter OPTION_PIC_TRIGGER         = "LEVEL",
  parameter OPTION_PIC_NMI_WIDTH       =  0,

  parameter OPTION_RF_CLEAR_ON_INIT    =  0, // !!! activate for simulation only !!!
  parameter OPTION_RF_ADDR_WIDTH       =  5,
  parameter OPTION_RF_WORDS            = 32,

  parameter OPTION_RESET_PC            = {{(OPTION_OPERAND_WIDTH-13){1'b0}},
                                          `OR1K_RESET_VECTOR,8'd0},

  parameter FEATURE_DIVIDER            = "SERIAL",

  parameter FEATURE_PSYNC              = "NONE",
  parameter FEATURE_CSYNC              = "NONE",

  parameter FEATURE_MULTICORE          = "NONE",

  parameter FEATURE_TRACEPORT_EXEC     = "NONE",


  parameter IBUS_WB_TYPE               = "B3_REGISTERED_FEEDBACK",
  parameter DBUS_WB_TYPE               = "B3_REGISTERED_FEEDBACK"
)
(
  // Wishbone clock and reset
  input                             wb_clk,
  input                             wb_rst,

  // CPU clock and reset
  input                             cpu_clk,
  input                             cpu_rst,

  // Wishbone interface (istruction)
  output [31:0]                     iwbm_adr_o,
  output                            iwbm_stb_o,
  output                            iwbm_cyc_o,
  output [3:0]                      iwbm_sel_o,
  output                            iwbm_we_o,
  output [2:0]                      iwbm_cti_o,
  output [1:0]                      iwbm_bte_o,
  output [31:0]                     iwbm_dat_o,
  input                             iwbm_err_i,
  input                             iwbm_ack_i,
  input [31:0]                      iwbm_dat_i,
  input                             iwbm_rty_i,

  // Wishbone interface (data)
  output [31:0]                     dwbm_adr_o,
  output                            dwbm_stb_o,
  output                            dwbm_cyc_o,
  output [3:0]                      dwbm_sel_o,
  output                            dwbm_we_o,
  output [2:0]                      dwbm_cti_o,
  output [1:0]                      dwbm_bte_o,
  output [31:0]                     dwbm_dat_o,
  input                             dwbm_err_i,
  input                             dwbm_ack_i,
  input [31:0]                      dwbm_dat_i,
  input                             dwbm_rty_i,

  // IRQ
  input [31:0]                      irq_i,

  // Debug System accesses CPU SPRs through DU
  input [15:0]                      du_addr_i,
  input                             du_stb_i,
  input [OPTION_OPERAND_WIDTH-1:0]  du_dat_i,
  input                             du_we_i,
  output [OPTION_OPERAND_WIDTH-1:0] du_dat_o,
  output                            du_ack_o,
  // Stall control from debug interface
  input                             du_stall_i,
  output                            du_stall_o,

  output                            traceport_exec_valid_o,
  output [31:0]                     traceport_exec_pc_o,
  output [`OR1K_INSN_WIDTH-1:0]     traceport_exec_insn_o,
  output [OPTION_OPERAND_WIDTH-1:0] traceport_exec_wbdata_o,
  output [OPTION_RF_ADDR_WIDTH-1:0] traceport_exec_wbreg_o,
  output                            traceport_exec_wben_o,

  // The multicore core identifier
  input [OPTION_OPERAND_WIDTH-1:0]  multicore_coreid_i,
  // The number of cores
  input [OPTION_OPERAND_WIDTH-1:0]  multicore_numcores_i,

  input [31:0]                     snoop_adr_i,
  input                            snoop_en_i
);

  // BUS-Bridge <-> CPU data port
  wire [OPTION_OPERAND_WIDTH-1:0] dbus_adr_o;
  wire [3:0]                      dbus_bsel_o;
  wire                            dbus_burst_o;
  wire [OPTION_OPERAND_WIDTH-1:0] dbus_dat_o;
  wire                            dbus_req_o;
  wire                            dbus_we_o;
  wire                            dbus_err_i;
  wire                            dbus_ack_i;
  wire [OPTION_OPERAND_WIDTH-1:0] dbus_dat_i;
  wire [OPTION_OPERAND_WIDTH-1:0] dbus_burst_adr_i;
  wire                            dbus_burst_last_i;

  // BUS-Bridge <-> CPU instruction port
  wire [OPTION_OPERAND_WIDTH-1:0] ibus_adr_o;
  wire                            ibus_burst_o;
  wire                            ibus_req_o;
  wire                            ibus_err_i;
  wire                            ibus_ack_i;
  wire [OPTION_OPERAND_WIDTH-1:0] ibus_dat_i;
  wire [OPTION_OPERAND_WIDTH-1:0] ibus_burst_adr_i;
  wire                            ibus_burst_last_i;

  // SPR access ???
  wire [15:0]                     spr_bus_addr_o;
  wire [OPTION_OPERAND_WIDTH-1:0] spr_bus_dat_o;
  wire                            spr_bus_stb_o;
  wire                            spr_bus_we_o;


  // BUS-Bridge for CPU instruction port
  mor1kx_bus_if_wb32_marocchino
  #(
    .DRIVER_TYPE  ("I_CACHE"),
    .BUS_IF_TYPE  (IBUS_WB_TYPE),
    .BURST_LENGTH ((OPTION_ICACHE_BLOCK_WIDTH == 4) ? 4 :
                   (OPTION_ICACHE_BLOCK_WIDTH == 5) ? 8 : 1)
  )
  ibus_bridge
  (
    // WB-domain: clock and reset
    .wb_clk           (wb_clk),
    .wb_rst           (wb_rst),
    // CPU-domain: clock and reset
    .cpu_clk          (cpu_clk),
    .cpu_rst          (cpu_rst),
    // CPU side
    .cpu_err_o        (ibus_err_i), // IBUS_BRIDGE
    .cpu_ack_o        (ibus_ack_i), // IBUS_BRIDGE
    .cpu_dat_o        (ibus_dat_i[`OR1K_INSN_WIDTH-1:0]), // IBUS_BRIDGE
    .cpu_burst_adr_o  (ibus_burst_adr_i), // IBUS_BRIDGE
    .cpu_burst_last_o (ibus_burst_last_i), // IBUS_BRIDGE
    .cpu_adr_i        (ibus_adr_o), // IBUS_BRIDGE
    .cpu_dat_i        ({OPTION_OPERAND_WIDTH{1'b0}}), // IBUS_BRIDGE
    .cpu_req_i        (ibus_req_o), // IBUS_BRIDGE
    .cpu_bsel_i       (4'b1111), // IBUS_BRIDGE
    .cpu_we_i         (1'b0), // IBUS_BRIDGE
    .cpu_burst_i      (ibus_burst_o), // IBUS_BRIDGE
    // Wishbone side
    .wbm_adr_o        (iwbm_adr_o), // IBUS_BRIDGE
    .wbm_stb_o        (iwbm_stb_o), // IBUS_BRIDGE
    .wbm_cyc_o        (iwbm_cyc_o), // IBUS_BRIDGE
    .wbm_sel_o        (iwbm_sel_o), // IBUS_BRIDGE
    .wbm_we_o         (iwbm_we_o), // IBUS_BRIDGE
    .wbm_cti_o        (iwbm_cti_o), // IBUS_BRIDGE
    .wbm_bte_o        (iwbm_bte_o), // IBUS_BRIDGE
    .wbm_dat_o        (iwbm_dat_o), // IBUS_BRIDGE
    .wbm_err_i        (iwbm_err_i), // IBUS_BRIDGE
    .wbm_ack_i        (iwbm_ack_i), // IBUS_BRIDGE
    .wbm_dat_i        (iwbm_dat_i), // IBUS_BRIDGE
    .wbm_rty_i        (iwbm_rty_i) // IBUS_BRIDGE
  );

  // BUS-Bridge for CPU data port
  mor1kx_bus_if_wb32_marocchino
  #(
    .DRIVER_TYPE  ("D_CACHE"),
    .BUS_IF_TYPE  (DBUS_WB_TYPE),
    .BURST_LENGTH ((OPTION_DCACHE_BLOCK_WIDTH == 4) ? 4 :
                   (OPTION_DCACHE_BLOCK_WIDTH == 5) ? 8 : 1)
  )
  dbus_bridge
  (
    // WB-domain: clock and reset
    .wb_clk           (wb_clk),
    .wb_rst           (wb_rst),
    // CPU-domain: clock and reset
    .cpu_clk          (cpu_clk),
    .cpu_rst          (cpu_rst),
    // CPU side
    .cpu_err_o        (dbus_err_i), // DBUS_BRIDGE
    .cpu_ack_o        (dbus_ack_i), // DBUS_BRIDGE
    .cpu_dat_o        (dbus_dat_i), // DBUS_BRIDGE
    .cpu_burst_adr_o  (dbus_burst_adr_i), // DBUS_BRIDGE
    .cpu_burst_last_o (dbus_burst_last_i), // DBUS_BRIDGE
    .cpu_adr_i        (dbus_adr_o), // DBUS_BRIDGE
    .cpu_dat_i        (dbus_dat_o), // DBUS_BRIDGE
    .cpu_req_i        (dbus_req_o), // DBUS_BRIDGE
    .cpu_bsel_i       (dbus_bsel_o), // DBUS_BRIDGE
    .cpu_we_i         (dbus_we_o), // DBUS_BRIDGE
    .cpu_burst_i      (dbus_burst_o), // DBUS_BRIDGE
    // Wishbone side
    .wbm_adr_o        (dwbm_adr_o), // DBUS_BRIDGE
    .wbm_stb_o        (dwbm_stb_o), // DBUS_BRIDGE
    .wbm_cyc_o        (dwbm_cyc_o), // DBUS_BRIDGE
    .wbm_sel_o        (dwbm_sel_o), // DBUS_BRIDGE
    .wbm_we_o         (dwbm_we_o), // DBUS_BRIDGE
    .wbm_cti_o        (dwbm_cti_o), // DBUS_BRIDGE
    .wbm_bte_o        (dwbm_bte_o), // DBUS_BRIDGE
    .wbm_dat_o        (dwbm_dat_o), // DBUS_BRIDGE
    .wbm_err_i        (dwbm_err_i), // DBUS_BRIDGE
    .wbm_ack_i        (dwbm_ack_i), // DBUS_BRIDGE
    .wbm_dat_i        (dwbm_dat_i), // DBUS_BRIDGE
    .wbm_rty_i        (dwbm_rty_i) // DBUS_BRIDGE
  );

  // instance of MAROCCHINO pipeline
  mor1kx_cpu_marocchino
  #(
    .OPTION_OPERAND_WIDTH             (OPTION_OPERAND_WIDTH),
    // data cache
    .OPTION_DCACHE_BLOCK_WIDTH        (OPTION_DCACHE_BLOCK_WIDTH),
    .OPTION_DCACHE_SET_WIDTH          (OPTION_DCACHE_SET_WIDTH),
    .OPTION_DCACHE_WAYS               (OPTION_DCACHE_WAYS),
    .OPTION_DCACHE_LIMIT_WIDTH        (OPTION_DCACHE_LIMIT_WIDTH),
    .OPTION_DCACHE_SNOOP              (OPTION_DCACHE_SNOOP),
    .OPTION_DCACHE_CLEAR_ON_INIT      (OPTION_DCACHE_CLEAR_ON_INIT),
    // data mmu
    .FEATURE_DMMU_HW_TLB_RELOAD       (FEATURE_DMMU_HW_TLB_RELOAD),
    .OPTION_DMMU_SET_WIDTH            (OPTION_DMMU_SET_WIDTH),
    .OPTION_DMMU_WAYS                 (OPTION_DMMU_WAYS),
    .OPTION_DMMU_CLEAR_ON_INIT        (OPTION_DMMU_CLEAR_ON_INIT),
    // write buffer
    .OPTION_STORE_BUFFER_DEPTH_WIDTH    (OPTION_STORE_BUFFER_DEPTH_WIDTH),
    .OPTION_STORE_BUFFER_CLEAR_ON_INIT  (OPTION_STORE_BUFFER_CLEAR_ON_INIT),
    // instruction cache
    .OPTION_ICACHE_BLOCK_WIDTH        (OPTION_ICACHE_BLOCK_WIDTH),
    .OPTION_ICACHE_SET_WIDTH          (OPTION_ICACHE_SET_WIDTH),
    .OPTION_ICACHE_WAYS               (OPTION_ICACHE_WAYS),
    .OPTION_ICACHE_LIMIT_WIDTH        (OPTION_ICACHE_LIMIT_WIDTH),
    .OPTION_ICACHE_CLEAR_ON_INIT      (OPTION_ICACHE_CLEAR_ON_INIT),
    // instruction mmu
    .FEATURE_IMMU_HW_TLB_RELOAD       (FEATURE_IMMU_HW_TLB_RELOAD),
    .OPTION_IMMU_SET_WIDTH            (OPTION_IMMU_SET_WIDTH),
    .OPTION_IMMU_WAYS                 (OPTION_IMMU_WAYS),
    .OPTION_IMMU_CLEAR_ON_INIT        (OPTION_IMMU_CLEAR_ON_INIT),
    // interrupt controller
    .OPTION_PIC_TRIGGER               (OPTION_PIC_TRIGGER),
    .OPTION_PIC_NMI_WIDTH             (OPTION_PIC_NMI_WIDTH),
    // timer, debug unit, performance counters, m-core, trace
    .TIMER_CLOCK_DOMAIN               (TIMER_CLOCK_DOMAIN),
    .FEATURE_DEBUGUNIT                (FEATURE_DEBUGUNIT),
    .FEATURE_PERFCOUNTERS             (FEATURE_PERFCOUNTERS),
    .FEATURE_MULTICORE                (FEATURE_MULTICORE),
    .FEATURE_TRACEPORT_EXEC           (FEATURE_TRACEPORT_EXEC),
    // Redister File
    .OPTION_RF_CLEAR_ON_INIT          (OPTION_RF_CLEAR_ON_INIT),
    .OPTION_RF_ADDR_WIDTH             (OPTION_RF_ADDR_WIDTH),
    //.OPTION_RF_WORDS(OPTION_RF_WORDS), // MAROCCHINO_TODO
    // starting PC
    .OPTION_RESET_PC                  (OPTION_RESET_PC),
     // arithmetic modules
    .FEATURE_DIVIDER                  (FEATURE_DIVIDER),
     // special instructions
    .FEATURE_PSYNC                    (FEATURE_PSYNC),
    .FEATURE_CSYNC                    (FEATURE_CSYNC)
  )
  u_cpu_marocchino
  (
    // Wishbone clock and reset
    .wb_clk                   (wb_clk),
    .wb_rst                   (wb_rst),
    // CPU clock and reset
    .cpu_clk                  (cpu_clk),
    .cpu_rst                  (cpu_rst),
    // Outputs
    .ibus_adr_o               (ibus_adr_o),
    .ibus_req_o               (ibus_req_o),
    .ibus_burst_o             (ibus_burst_o),
    .dbus_adr_o               (dbus_adr_o),
    .dbus_dat_o               (dbus_dat_o),
    .dbus_req_o               (dbus_req_o),
    .dbus_bsel_o              (dbus_bsel_o),
    .dbus_we_o                (dbus_we_o),
    .dbus_burst_o             (dbus_burst_o),
    .du_dat_o                 (du_dat_o),
    .du_ack_o                 (du_ack_o),
    .du_stall_o               (du_stall_o),
    .traceport_exec_valid_o   (traceport_exec_valid_o),
    .traceport_exec_pc_o      (traceport_exec_pc_o),
    .traceport_exec_insn_o    (traceport_exec_insn_o[`OR1K_INSN_WIDTH-1:0]),
    .traceport_exec_wbdata_o  (traceport_exec_wbdata_o),
    .traceport_exec_wbreg_o   (traceport_exec_wbreg_o),
    .traceport_exec_wben_o    (traceport_exec_wben_o),
    .spr_bus_addr_o           (spr_bus_addr_o[15:0]),
    .spr_bus_we_o             (spr_bus_we_o),
    .spr_bus_stb_o            (spr_bus_stb_o),
    .spr_bus_dat_o            (spr_bus_dat_o),
    // Inputs
    .ibus_err_i               (ibus_err_i),
    .ibus_ack_i               (ibus_ack_i),
    .ibus_dat_i               (ibus_dat_i[`OR1K_INSN_WIDTH-1:0]),
    .ibus_burst_adr_i         (ibus_burst_adr_i),
    .ibus_burst_last_i        (ibus_burst_last_i),
    .dbus_err_i               (dbus_err_i),
    .dbus_ack_i               (dbus_ack_i),
    .dbus_dat_i               (dbus_dat_i),
    .dbus_burst_adr_i         (dbus_burst_adr_i),
    .dbus_burst_last_i        (dbus_burst_last_i),
    .irq_i                    (irq_i[31:0]),
    .du_addr_i                (du_addr_i),
    .du_stb_i                 (du_stb_i),
    .du_dat_i                 (du_dat_i),
    .du_we_i                  (du_we_i),
    .du_stall_i               (du_stall_i),
    .multicore_coreid_i       (multicore_coreid_i),
    .multicore_numcores_i     (multicore_numcores_i),
    .snoop_adr_i              (snoop_adr_i[31:0]),
    .snoop_en_i               (snoop_en_i)
  ); // pipe instance

endmodule // mor1kx_marocchino_alone
