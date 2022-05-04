// Copyright lowRISC contributors.
// Licensed under the Apache License, Version 2.0, see LICENSE for details.
// SPDX-License-Identifier: Apache-2.0

// VCS does not support overriding enum and string parameters via command line. Instead, a `define
// is used that can be set from the command line. If no value has been specified, this gives a
// default. Other simulators don't take the detour via `define and can override the corresponding
// parameters directly.
`ifndef RV32M
  `define RV32M ibex_pkg::RV32MFast
`endif

`ifndef RV32B
  `define RV32B ibex_pkg::RV32BNone
`endif

`ifndef RegFile
  `define RegFile ibex_pkg::RegFileFF
`endif

/**
 * Ibex simple system
 *
 * This is a basic system consisting of an ibex, a 1 MB sram for instruction/data
 * and a small memory mapped control module for outputting ASCII text and
 * controlling/halting the simulation from the software running on the ibex.
 *
 * It is designed to be used with verilator but should work with other
 * simulators, a small amount of work may be required to support the
 * simulator_ctrl module.
 */

module dbg (
  input IO_CLK,
  input IO_RST_N
);

  parameter bit                 SecureIbex               = 1'b0;
  parameter bit                 ICacheScramble           = 1'b0;
  parameter bit                 PMPEnable                = 1'b0;
  parameter int unsigned        PMPGranularity           = 0;
  parameter int unsigned        PMPNumRegions            = 4;
  parameter bit                 RV32E                    = 1'b0;
  parameter ibex_pkg::rv32m_e   RV32M                    = `RV32M;
  parameter ibex_pkg::rv32b_e   RV32B                    = `RV32B;
  parameter ibex_pkg::regfile_e RegFile                  = `RegFile;
  parameter bit                 BranchTargetALU          = 1'b0;
  parameter bit                 WritebackStage           = 1'b0;
  parameter bit                 ICache                   = 1'b0;
  parameter bit                 ICacheECC                = 1'b0;
  parameter bit                 BranchPredictor          = 1'b0;
  parameter                     SRAMInitFile             = "";

  logic clk_sys = 1'b0, rst_sys_n;
  typedef enum logic[1:0] {
    CoreD,
    DbgHost,
    MasterPort
  } bus_host_e;

  typedef enum logic[1:0] {
    Ram,
    SimCtrl,
    Timer,
    DbgDev
  } bus_device_e;

  logic haltpin;
  logic core_rst_n;

  localparam int NrDevices = 4;
  localparam int NrHosts = 3;


  // usys ctrl 

  logic usys_rstb      ;                  
  logic usys_cpu_rstb  ;             
  logic gated_clocks   ;              
  logic usys_halt      ;             

  // interrupts
  logic timer_irq;

  // host and device signals
  logic           host_req    [NrHosts];
  logic           host_gnt    [NrHosts];
  logic [31:0]    host_addr   [NrHosts];
  logic           host_we     [NrHosts];
  logic [ 3:0]    host_be     [NrHosts];
  logic [31:0]    host_wdata  [NrHosts];
  logic           host_rvalid [NrHosts];
  logic [31:0]    host_rdata  [NrHosts];
  logic           host_err    [NrHosts];

  logic [6:0]     data_rdata_intg;
  logic [6:0]     instr_rdata_intg;

  // devices (slaves)
  logic           device_req    [NrDevices];
  logic [31:0]    device_addr   [NrDevices];
  logic           device_we     [NrDevices];
  logic [ 3:0]    device_be     [NrDevices];
  logic [31:0]    device_wdata  [NrDevices];
  logic           device_rvalid [NrDevices];
  logic [31:0]    device_rdata  [NrDevices];
  logic           device_err    [NrDevices];

  // Device address mapping
  logic [31:0] cfg_device_addr_base [NrDevices];
  logic [31:0] cfg_device_addr_mask [NrDevices];
  assign cfg_device_addr_base[Ram] = 32'h100000;
  assign cfg_device_addr_mask[Ram] = ~32'hFFFFF; // 1 MB
  assign cfg_device_addr_base[SimCtrl] = 32'h20000;
  assign cfg_device_addr_mask[SimCtrl] = ~32'h3FF; // 1 kB
  assign cfg_device_addr_base[Timer] = 32'h30000;
  assign cfg_device_addr_mask[Timer] = ~32'h3FF; // 1 kB
  assign cfg_device_addr_base[DbgDev] = 32'h40000;
  assign cfg_device_addr_mask[DbgDev] = ~32'hFFFF; // 1 MB

  // Instruction fetch signals
  logic instr_req;
  logic instr_gnt;
  logic instr_rvalid;
  logic [31:0] instr_addr;
  logic [31:0] instr_rdata;
  logic instr_err;

//debug
  logic ndmreset_req;
  logic rst_core_n;
  logic dm_debug_req;
  logic dbg_slave_req;
  logic [31:0] dbg_slave_addr;
  logic        dbg_slave_we;
  logic [ 3:0] dbg_slave_be;
  logic [31:0] dbg_slave_wdata;
  logic        dbg_slave_rvalid;
  logic [31:0] dbg_slave_rdata;
  logic        dbg_instr_req;
  logic        mem_instr_req;

  logic        debug_req;

  
  assign instr_gnt = instr_req;
  assign instr_err = '0;

  `ifdef VERILATOR
    assign clk_sys = IO_CLK;
    assign rst_sys_n = IO_RST_N;
  `else
    initial begin
      rst_sys_n = 1'b0;
      #8
      rst_sys_n = 1'b1;
    end
    always begin
      #1 clk_sys = 1'b0;
      #1 clk_sys = 1'b1;
    end
  `endif

    // reg [31:0] counter ;
    // always @((posedge IO_CLK) or (negedge IO_RST_N) ) begin
    //   if(~IO_RST_N)
    //     counter <= 0;
    //   else 
    //   counter <= counter +1;
    // end
    // assign haltpin=0;
    // always @* begin 
    //   if(counter >10000 && counter <100000)
    //     //haltpin =1;
    //     debug_req=1;
    //   else 
    //     //haltpin  =0 ;
    //     debug_req=0;
    //     core_rst_n =1;

    // end
  // Tie-off unused error signals
  assign device_err[Ram] = 1'b0;
  assign device_err[SimCtrl] = 1'b0;

  bus #(
    .NrDevices    ( NrDevices ),
    .NrHosts      ( NrHosts   ),
    .DataWidth    ( 32        ),
    .AddressWidth ( 32        )
  ) u_bus (
    .clk_i               (clk_sys),
    .rst_ni              (rst_sys_n),

    .host_req_i          (host_req     ),
    .host_gnt_o          (host_gnt     ),
    .host_addr_i         (host_addr    ),
    .host_we_i           (host_we      ),
    .host_be_i           (host_be      ),
    .host_wdata_i        (host_wdata   ),
    .host_rvalid_o       (host_rvalid  ),
    .host_rdata_o        (host_rdata   ),
    .host_err_o          (host_err     ),

    .device_req_o        (device_req   ),
    .device_addr_o       (device_addr  ),
    .device_we_o         (device_we    ),
    .device_be_o         (device_be    ),
    .device_wdata_o      (device_wdata ),
    .device_rvalid_i     (device_rvalid),
    .device_rdata_i      (device_rdata ),
    .device_err_i        (device_err   ),

    .cfg_device_addr_base,
    .cfg_device_addr_mask
  );

  if (SecureIbex) begin : g_mem_rdata_ecc
    logic [31:0] unused_data_rdata;
    logic [31:0] unused_instr_rdata;

    prim_secded_inv_39_32_enc u_data_rdata_intg_gen (
      .data_i (host_rdata[CoreD]),
      .data_o ({data_rdata_intg, unused_data_rdata})
    );

    prim_secded_inv_39_32_enc u_instr_rdata_intg_gen (
      .data_i (instr_rdata),
      .data_o ({instr_rdata_intg, unused_instr_rdata})
    );
  end else begin : g_no_mem_rdata_ecc
    assign data_rdata_intg = '0;
    assign instr_rdata_intg = '0;
  end




  assign dbg_instr_req =
      instr_req & ((instr_addr & cfg_device_addr_mask[DbgDev]) == cfg_device_addr_base[DbgDev]);
  assign mem_instr_req =
      instr_req & ((instr_addr & cfg_device_addr_mask[Ram]) == cfg_device_addr_base[Ram]);      
  assign rst_core_n=rst_sys_n & ~ndmreset_req;
  assign dbg_slave_req         = device_req[DbgDev] | dbg_instr_req;
  assign dbg_slave_we          = device_req[DbgDev] & device_we[DbgDev];
  assign dbg_slave_addr        = device_req[DbgDev] ? device_addr[DbgDev] : instr_addr;

  assign dbg_slave_be          = device_be[DbgDev];
  assign dbg_slave_wdata       = device_wdata[DbgDev];
  assign device_rvalid[DbgDev] = dbg_slave_rvalid;
  assign device_rdata[DbgDev]  = dbg_slave_rdata;

  always @(posedge clk_sys or negedge rst_sys_n) begin
    if (!rst_sys_n) begin
      dbg_slave_rvalid <= 1'b0;
    end else begin
      dbg_slave_rvalid <= device_req[DbgDev];
    end
  end



  ibex_top_tracing #(
      .SecureIbex      ( SecureIbex      ),
      .ICacheScramble  ( ICacheScramble  ),
      .PMPEnable       ( PMPEnable       ),
      .PMPGranularity  ( PMPGranularity  ),
      .PMPNumRegions   ( PMPNumRegions   ),
      .MHPMCounterNum  ( 29              ),
      .RV32E           ( RV32E           ),
      .RV32M           ( RV32M           ),
      .RV32B           ( RV32B           ),
      .RegFile         ( RegFile         ),
      .BranchTargetALU ( BranchTargetALU ),
      .ICache          ( ICache          ),
      .ICacheECC       ( ICacheECC       ),
      .WritebackStage  ( WritebackStage  ),
      .BranchPredictor ( BranchPredictor ),
      .DbgTriggerEn(1'b1),
      .DbgHwBreakNum(2),
      .DmHaltAddr(32'h40000 + 32'(dm::HaltAddress)),
      .DmExceptionAddr(32'h40000 + 32'(dm::ExceptionAddress))
    ) u_top (
      .clk_i                  (clk_sys),
      .rst_ni                 (rst_core_n & usys_rstb),

      .test_en_i              ('b0),
      .scan_rst_ni            (1'b1),
      .ram_cfg_i              ('b0),

      .hart_id_i              (32'b0),
      // First instruction executed is at 0x0 + 0x80
      .boot_addr_i            (32'h00100000),

      .instr_req_o            (instr_req),
      .instr_gnt_i            (instr_gnt),
      .instr_rvalid_i         (instr_rvalid),
      .instr_addr_o           (instr_addr),
      .instr_rdata_i          (instr_rdata),
      .instr_rdata_intg_i     (instr_rdata_intg),
      .instr_err_i            (instr_err),

      .data_req_o             (host_req[CoreD]),
      .data_gnt_i             (host_gnt[CoreD]),
      .data_rvalid_i          (host_rvalid[CoreD]),
      .data_we_o              (host_we[CoreD]),
      .data_be_o              (host_be[CoreD]),
      .data_addr_o            (host_addr[CoreD]),
      .data_wdata_o           (host_wdata[CoreD]),
      .data_wdata_intg_o      (),
      .data_rdata_i           (host_rdata[CoreD]),
      .data_rdata_intg_i      (data_rdata_intg),
      .data_err_i             (host_err[CoreD]),

      .irq_software_i         (1'b0),
      .irq_timer_i            (timer_irq),
      .irq_external_i         (1'b0),
      .irq_fast_i             (15'b0),
      .irq_nm_i               (1'b0),

      .scramble_key_valid_i   ('0),
      .scramble_key_i         ('0),
      .scramble_nonce_i       ('0),
      .scramble_req_o         (),

      .debug_req_i            (dm_debug_req),
      .crash_dump_o           (),
      .double_fault_seen_o    (),

      .fetch_enable_i         (ibex_pkg::FetchEnableOn),
      .alert_minor_o          (),
      .alert_major_internal_o (),
      .alert_major_bus_o      (),
      .core_sleep_o           (),
      .haltpin                (usys_halt),
      .core_rst_n             (usys_cpu_rstb)
    );

  // SRAM block for instruction and data storage
  ram_2p #(
      .Depth(1024*1024/4),
      .MemInitFile(SRAMInitFile)
    ) u_ram (
      .clk_i       (clk_sys),
      .rst_ni      (rst_sys_n),

      .a_req_i     (device_req[Ram]),
      .a_we_i      (device_we[Ram]),
      .a_be_i      (device_be[Ram]),
      .a_addr_i    (device_addr[Ram]),
      .a_wdata_i   (device_wdata[Ram]),
      .a_rvalid_o  (device_rvalid[Ram]),
      .a_rdata_o   (device_rdata[Ram]),

      .b_req_i     (mem_instr_req),
      .b_we_i      (1'b0),
      .b_be_i      (4'b0),
      .b_addr_i    (instr_addr),
      .b_wdata_i   (32'b0),
      .b_rvalid_o  (instr_rvalid),
      .b_rdata_o   (instr_rdata)
    );

  simulator_ctrl #(
    .LogName("dbg.log")
    ) u_simulator_ctrl (
      .clk_i     (clk_sys),
      .rst_ni    (rst_sys_n),

      .req_i     (device_req[SimCtrl]),
      .we_i      (device_we[SimCtrl]),
      .be_i      (device_be[SimCtrl]),
      .addr_i    (device_addr[SimCtrl]),
      .wdata_i   (device_wdata[SimCtrl]),
      .rvalid_o  (device_rvalid[SimCtrl]),
      .rdata_o   (device_rdata[SimCtrl])
    );

  //**** DM TOP 

  dm_top #(
      .NrHarts       (1)
    ) u_dm_top (
      .clk_i         (clk_sys),
      .rst_ni        (rst_sys_n),
      .testmode_i    (1'b0),
      .ndmreset_o    (ndmreset_req),
      .dmactive_o    (),
      .debug_req_o   (dm_debug_req),
      .unavailable_i (1'b0),

      // bus device with debug memory (for execution-based debug)
      .slave_req_i       (dbg_slave_req),
      .slave_we_i        (dbg_slave_we),
      .slave_addr_i      (dbg_slave_addr),
      .slave_be_i        (dbg_slave_be),
      .slave_wdata_i     (dbg_slave_wdata),
      .slave_rdata_o     (dbg_slave_rdata),

      // bus host (for system bus accesses, SBA)
      .master_req_o      (host_req[DbgHost]),
      .master_add_o      (host_addr[DbgHost]),
      .master_we_o       (host_we[DbgHost]),
      .master_wdata_o    (host_wdata[DbgHost]),
      .master_be_o       (host_be[DbgHost]),
      .master_gnt_i      (host_gnt[DbgHost]),
      .master_r_valid_i  (host_rvalid[DbgHost]),
      .master_r_rdata_i  (host_rdata[DbgHost])
      // .halt_req_pin_i(debug_req)
    );





  timer #(
    .DataWidth    (32),
    .AddressWidth (32)
    ) u_timer (
      .clk_i          (clk_sys),
      .rst_ni         (rst_sys_n),

      .timer_req_i    (device_req[Timer]),
      .timer_we_i     (device_we[Timer]),
      .timer_be_i     (device_be[Timer]),
      .timer_addr_i   (device_addr[Timer]),
      .timer_wdata_i  (device_wdata[Timer]),
      .timer_rvalid_o (device_rvalid[Timer]),
      .timer_rdata_o  (device_rdata[Timer]),
      .timer_err_o    (device_err[Timer]),
      .timer_intr_o   (timer_irq)
    );
reg jtag_tms,jtag_tdi,jtag_tck;
wire jtag_tdo;
reg [31:0] TGPI;
wire [31:0] TGPO;
reg               slave_port_cmd_valid            ;
wire              slave_port_cmd_ready            ;
reg      [31:0]   slave_port_cmd_payload_addr     ;
reg               slave_port_cmd_payload_we       ;
reg      [3:0]    slave_port_cmd_payload_be       ;
reg      [31:0]   slave_port_cmd_payload_wdata    ;
wire              slave_port_rsp_valid            ;
wire     [31:0]   slave_port_rsp_payload_rdata    ;
wire              slave_port_rsp_payload_err      ;

TSysGwtWrapper dut(
  .rstb    (rst_sys_n    ),
  .eclk    (clk_sys    ),
  .jtag_tms('0),
  .jtag_tdi('0),
  .jtag_tdo(jtag_tdo),
  .jtag_tck('0),
  .TGPO    (TGPO    ),
  .TGPI    (TGPI    ),
  .master_port_cmd_valid        (host_req[MasterPort]        ) ,
  .master_port_cmd_ready        (host_gnt[MasterPort]        ) ,
  .master_port_cmd_payload_addr (host_addr[MasterPort] ) ,
  .master_port_cmd_payload_we   (host_we[MasterPort]   ) ,
  .master_port_cmd_payload_be   (host_be[MasterPort]   ) ,
  .master_port_cmd_payload_wdata(host_wdata[MasterPort]) ,
  .master_port_rsp_valid        (host_rvalid[MasterPort]        ) ,
  .master_port_rsp_payload_rdata(host_rdata[MasterPort]) ,
  .master_port_rsp_payload_err  (host_err[MasterPort]  ) ,
  .slave_port_cmd_valid         ('0         ) ,
  .slave_port_cmd_ready         (slave_port_cmd_ready         ) ,
  .slave_port_cmd_payload_addr  ('0  ) ,
  .slave_port_cmd_payload_we    ('0    ) ,
  .slave_port_cmd_payload_be    ('0    ) ,
  .slave_port_cmd_payload_wdata ('0 ) ,
  .slave_port_rsp_valid         (slave_port_rsp_valid         ) ,
  .slave_port_rsp_payload_rdata (slave_port_rsp_payload_rdata ) ,
  .slave_port_rsp_payload_err   (slave_port_rsp_payload_err   ) ,
  .usys_rstb                    (usys_rstb),
  .usys_cpu_rstb                (usys_cpu_rstb),
  .gated_clocks                 (gated_clocks),
  .usys_halt                    (usys_halt)

  );


  export "DPI-C" function mhpmcounter_get;

  function automatic longint unsigned mhpmcounter_get(int index);
    return u_top.u_ibex_top.u_ibex_core.cs_registers_i.mhpmcounter[index];
  endfunction

endmodule
