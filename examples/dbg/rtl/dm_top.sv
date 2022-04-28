// Copyright lowRISC contributors.
// Licensed under the Apache License, Version 2.0, see LICENSE for details.
// SPDX-License-Identifier: Apache-2.0

// Top-level debug module (DM)
//
// This module implements the RISC-V debug specification version 0.13,
//
// This toplevel wraps the PULP debug module available from
// https://github.com/pulp-platform/riscv-dbg to match the needs of
// the TL-UL-based lowRISC chip design.



module dm_halt_manager(
  input logic halt_pin,
  input logic clk_i,
  input logic rst_ni,
//

  output logic         dmi_rst_no_h, // hard reset
  output dm::dmi_req_t dmi_req_o_h,
  output logic         dmi_req_valid_o_h,
  input  logic         dmi_req_ready_i_h,

  input dm::dmi_resp_t dmi_resp_i_h,
  output logic         dmi_resp_ready_o_h,
  input  logic         dmi_resp_valid_i_h,

/// input from jtag tap 

  input  logic                              dmi_rst_ni,    
  input  dm::dmi_req_t                      dmi_req_i,
  input  logic                              dmi_req_valid_i,
  output logic                              dmi_req_ready_o,
  // every request needs a response one cycle later
  output dm::dmi_resp_t                     dmi_resp_o,
  output logic                              dmi_resp_valid_o,
  input  logic                              dmi_resp_ready_i

);


assign  dmi_rst_no_h =  dmi_rst_ni ; 
assign  dmi_req_o_h = dmi_req_i ;
assign  dmi_req_valid_o_h = dmi_req_valid_i ;
assign  dmi_req_ready_o  = dmi_req_ready_i_h ;
assign  dmi_resp_o  =dmi_resp_i_h;
assign  dmi_resp_ready_o_h =dmi_resp_ready_i  ;
assign  dmi_resp_valid_o= dmi_resp_valid_i_h ;








reg lasthalt_is_mine ;
reg [1:0] dm_is_active;
  always @(posedge clk_i or negedge rst_ni) begin
    if(! rst_ni) begin 
      lasthalt_is_mine<=0;
      dm_is_active <=0;
    end else begin
    if(halt_pin && !lasthalt_is_mine) begin
    lasthalt_is_mine<=1;
    
    end
    if(!halt_pin && lasthalt_is_mine)begin
    lasthalt_is_mine<=0;
    end
    if (dm_is_active==0) begin
    dm_is_active     <=1;
    end
    if (dm_is_active==1) begin
    dm_is_active     <=2;
    end
    end
    
  end 

  always_latch begin
    if (dm_is_active[0]) begin
      dmi_req_o_h <={7'h10,2'h2,31'h0,1'h1};
      dmi_req_valid_o_h<=1;
      dmi_resp_ready_o_h<=1;
    end
    if(halt_pin && !lasthalt_is_mine && dm_is_active[1]) begin
    dmi_req_o_h ={7'h10,2'h2,1'h1,30'h0,1'h1};
    dmi_req_valid_o_h=1;
    dmi_resp_ready_o_h=1;

    end
    if(!halt_pin && lasthalt_is_mine && dm_is_active[1])begin
    //dmi_req_o_h ={7'h10,2'h2,32'h0};
        dmi_req_o_h ={7'h10,2'h2,1'h0,1'h1,29'h0,1'h1};

    dmi_req_valid_o_h=1;
    dmi_resp_ready_o_h=1;
    end
  
  end










endmodule


`include "prim_assert.sv"

module dm_top #(
  parameter int              NrHarts = 1,
  parameter logic [31:0]     IdcodeValue = 32'h 0000_0001
) (
  input  logic               clk_i,       // clock
  input  logic               rst_ni,      // asynchronous reset active low, connect PoR
                                          // here, not the system reset
  input  logic               testmode_i,
  output logic               ndmreset_o,  // non-debug module reset
  output logic               dmactive_o,  // debug module is active
  output logic [NrHarts-1:0] debug_req_o, // async debug request
  input  logic [NrHarts-1:0] unavailable_i, // communicate whether the hart is unavailable
                                            // (e.g.: power down)

  // bus device with debug memory, for an execution based technique
  input  logic                  slave_req_i,
  input  logic                  slave_we_i,
  input  logic [BusWidth-1:0]   slave_addr_i,
  input  logic [BusWidth/8-1:0] slave_be_i,
  input  logic [BusWidth-1:0]   slave_wdata_i,
  output logic [BusWidth-1:0]   slave_rdata_o,

  // bus host, for system bus accesses
  output logic                  master_req_o,
  output logic [BusWidth-1:0]   master_add_o,
  output logic                  master_we_o,
  output logic [BusWidth-1:0]   master_wdata_o,
  output logic [BusWidth/8-1:0] master_be_o,
  input  logic                  master_gnt_i,
  input  logic                  master_r_valid_i,
  input  logic [BusWidth-1:0]   master_r_rdata_i,

  input  logic                  halt_req_pin_i
);

  `ASSERT_INIT(paramCheckNrHarts, NrHarts > 0)

  localparam int BusWidth = 32;
  // all harts have contiguous IDs
  localparam logic [NrHarts-1:0] SelectableHarts = {NrHarts{1'b1}};

  // Debug CSRs
  dm::hartinfo_t [NrHarts-1:0]      hartinfo;
  logic [NrHarts-1:0]               halted;
  // logic [NrHarts-1:0]               running;
  logic [NrHarts-1:0]               resumeack;
  logic [NrHarts-1:0]               haltreq;
  logic [NrHarts-1:0]               resumereq;




  logic                             clear_resumeack;
  logic                             cmd_valid;
  dm::command_t                     cmd;

  logic                             cmderror_valid;
  dm::cmderr_e                      cmderror;
  logic                             cmdbusy;
  logic [dm::ProgBufSize-1:0][31:0] progbuf;
  logic [dm::DataCount-1:0][31:0]   data_csrs_mem;
  logic [dm::DataCount-1:0][31:0]   data_mem_csrs;
  logic                             data_valid;
  logic [19:0]                      hartsel;
  // System Bus Access Module
  logic [BusWidth-1:0]              sbaddress_csrs_sba;
  logic [BusWidth-1:0]              sbaddress_sba_csrs;
  logic                             sbaddress_write_valid;
  logic                             sbreadonaddr;
  logic                             sbautoincrement;
  logic [2:0]                       sbaccess;
  logic                             sbreadondata;
  logic [BusWidth-1:0]              sbdata_write;
  logic                             sbdata_read_valid;
  logic                             sbdata_write_valid;
  logic [BusWidth-1:0]              sbdata_read;
  logic                             sbdata_valid;
  logic                             sbbusy;
  logic                             sberror_valid;
  logic [2:0]                       sberror;
  logic                             ndmreset;

  dm::dmi_req_t  dmi_req;
  dm::dmi_resp_t dmi_rsp;
  logic dmi_req_valid, dmi_req_ready;
  logic dmi_rsp_valid, dmi_rsp_ready;
  logic dmi_rst_n;

  dm::dmi_req_t  dmi_req_h;
  dm::dmi_resp_t dmi_rsp_h;
  logic dmi_req_valid_h, dmi_req_ready_h;
  logic dmi_rsp_valid_h, dmi_rsp_ready_h;
  logic dmi_rst_n_h;



  // static debug hartinfo
  localparam dm::hartinfo_t DebugHartInfo = '{
    zero1:      '0,
    nscratch:   2, // Debug module needs at least two scratch regs
    zero0:      0,
    dataaccess: 1'b1, // data registers are memory mapped in the debugger
    datasize:   dm::DataCount,
    dataaddr:   dm::DataAddr
  };
  for (genvar i = 0; i < NrHarts; i++) begin : gen_dm_hart_ctrl
    assign hartinfo[i] = DebugHartInfo;
  end

  assign ndmreset_o = ndmreset;
  


  dm_halt_manager halt_man(
    .halt_pin(halt_req_pin_i),
    .clk_i(clk_i),
    .rst_ni(rst_ni),
    .dmi_rst_no_h(dmi_rst_n_h),
    .dmi_req_o_h(dmi_req_h),
    .dmi_req_valid_o_h(dmi_req_valid_h),
    .dmi_req_ready_i_h(dmi_req_ready_h),
    .dmi_resp_i_h(dmi_rsp_h),
    .dmi_resp_ready_o_h(dmi_rsp_ready_h),
    .dmi_resp_valid_i_h(dmi_rsp_valid_h),

    .dmi_rst_ni(dmi_rst_n),
    .dmi_req_i(dmi_req),
    .dmi_req_valid_i(dmi_req_valid),
    .dmi_req_ready_o(dmi_req_ready),
    .dmi_resp_o(dmi_rsp),
    .dmi_resp_ready_i(dmi_rsp_ready),
    .dmi_resp_valid_o(dmi_rsp_valid)
  );

  dm_csrs #(
    .NrHarts(NrHarts),
    .BusWidth(BusWidth),
    .SelectableHarts(SelectableHarts)
  ) i_dm_csrs (
    .clk_i                   ( clk_i                 ),
    .rst_ni                  ( rst_ni                ),
    .testmode_i              ( testmode_i            ),
    .dmi_rst_ni              ( dmi_rst_n_h             ),
    .dmi_req_valid_i         ( dmi_req_valid_h         ),
    .dmi_req_ready_o         ( dmi_req_ready_h         ),
    .dmi_req_i               ( dmi_req_h               ),
    .dmi_resp_valid_o        ( dmi_rsp_valid_h         ),
    .dmi_resp_ready_i        ( dmi_rsp_ready_h         ),
    .dmi_resp_o              ( dmi_rsp_h               ),
    .ndmreset_o              ( ndmreset              ),
    .dmactive_o              ( dmactive_o            ),
    .hartsel_o               ( hartsel               ),
    .hartinfo_i              ( hartinfo              ),
    .halted_i                ( halted                ),
    .unavailable_i,
    .resumeack_i             ( resumeack             ),
    .haltreq_o               ( haltreq               ),
    .resumereq_o             ( resumereq             ),
    .clear_resumeack_o       ( clear_resumeack       ),
    .cmd_valid_o             ( cmd_valid             ),
    .cmd_o                   ( cmd                   ),
    .cmderror_valid_i        ( cmderror_valid        ),
    .cmderror_i              ( cmderror              ),
    .cmdbusy_i               ( cmdbusy               ),
    .progbuf_o               ( progbuf               ),
    .data_i                  ( data_mem_csrs         ),
    .data_valid_i            ( data_valid            ),
    .data_o                  ( data_csrs_mem         ),
    .sbaddress_o             ( sbaddress_csrs_sba    ),
    .sbaddress_i             ( sbaddress_sba_csrs    ),
    .sbaddress_write_valid_o ( sbaddress_write_valid ),
    .sbreadonaddr_o          ( sbreadonaddr          ),
    .sbautoincrement_o       ( sbautoincrement       ),
    .sbaccess_o              ( sbaccess              ),
    .sbreadondata_o          ( sbreadondata          ),
    .sbdata_o                ( sbdata_write          ),
    .sbdata_read_valid_o     ( sbdata_read_valid     ),
    .sbdata_write_valid_o    ( sbdata_write_valid    ),
    .sbdata_i                ( sbdata_read           ),
    .sbdata_valid_i          ( sbdata_valid          ),
    .sbbusy_i                ( sbbusy                ),
    .sberror_valid_i         ( sberror_valid         ),
    .sberror_i               ( sberror               )
  );

  dm_sba #(
    .BusWidth(BusWidth)
  ) i_dm_sba (
    .clk_i                   ( clk_i                 ),
    .rst_ni                  ( rst_ni                ),
    .master_req_o            ( master_req_o          ),
    .master_add_o            ( master_add_o          ),
    .master_we_o             ( master_we_o           ),
    .master_wdata_o          ( master_wdata_o        ),
    .master_be_o             ( master_be_o           ),
    .master_gnt_i            ( master_gnt_i          ),
    .master_r_valid_i        ( master_r_valid_i      ),
    .master_r_rdata_i        ( master_r_rdata_i      ),
    .dmactive_i              ( dmactive_o            ),
    .sbaddress_i             ( sbaddress_csrs_sba    ),
    .sbaddress_o             ( sbaddress_sba_csrs    ),
    .sbaddress_write_valid_i ( sbaddress_write_valid ),
    .sbreadonaddr_i          ( sbreadonaddr          ),
    .sbautoincrement_i       ( sbautoincrement       ),
    .sbaccess_i              ( sbaccess              ),
    .sbreadondata_i          ( sbreadondata          ),
    .sbdata_i                ( sbdata_write          ),
    .sbdata_read_valid_i     ( sbdata_read_valid     ),
    .sbdata_write_valid_i    ( sbdata_write_valid    ),
    .sbdata_o                ( sbdata_read           ),
    .sbdata_valid_o          ( sbdata_valid          ),
    .sbbusy_o                ( sbbusy                ),
    .sberror_valid_o         ( sberror_valid         ),
    .sberror_o               ( sberror               )
  );

  dm_mem #(
    .NrHarts(NrHarts),
    .BusWidth(BusWidth),
    .SelectableHarts(SelectableHarts),
    // The debug module provides a simplified ROM for systems that map the debug ROM to offset 0x0
    // on the system bus. In that case, only one scratch register has to be implemented in the core.
    // However, we require that the DM can be placed at arbitrary offsets in the system, which
    // requires the generalized debug ROM implementation and two scratch registers. We hence set
    // this parameter to a non-zero value (inside dm_mem, this just feeds into a comparison with 0).
    .DmBaseAddress(1)
  ) i_dm_mem (
    .clk_i                   ( clk_i                 ),
    .rst_ni                  ( rst_ni                ),
    .ndmreset_i              ( ndmreset              ),
    .debug_req_o             ( debug_req_o           ),
    .hartsel_i               ( hartsel              ),
    .haltreq_i               ( haltreq               ),
    .resumereq_i             ( resumereq             ),
    .clear_resumeack_i       ( clear_resumeack       ),
    .halted_o                ( halted                ),
    .resuming_o              ( resumeack             ),
    .cmd_valid_i             ( cmd_valid             ),
    .cmd_i                   ( cmd                   ),
    .cmderror_valid_o        ( cmderror_valid        ),
    .cmderror_o              ( cmderror              ),
    .cmdbusy_o               ( cmdbusy               ),
    .progbuf_i               ( progbuf               ),
    .data_i                  ( data_csrs_mem         ),
    .data_o                  ( data_mem_csrs         ),
    .data_valid_o            ( data_valid            ),
    .req_i                   ( slave_req_i           ),
    .we_i                    ( slave_we_i            ),
    .addr_i                  ( slave_addr_i          ),
    .wdata_i                 ( slave_wdata_i         ),
    .be_i                    ( slave_be_i            ),
    .rdata_o                 ( slave_rdata_o         )
  );

  // Bound-in DPI module replaces the TAP
`ifndef DMIDirectTAP
  // JTAG TAP
  dmi_jtag #(
    .IdcodeValue    (IdcodeValue)
  ) dap (
    .clk_i            (clk_i),
    .rst_ni           (rst_ni),
    .testmode_i       (testmode_i),

    .dmi_rst_no       (dmi_rst_n),
    .dmi_req_o        (dmi_req),
    .dmi_req_valid_o  (dmi_req_valid),
    .dmi_req_ready_i  (dmi_req_ready),

    .dmi_resp_i       (dmi_rsp      ),
    .dmi_resp_ready_o (dmi_rsp_ready),
    .dmi_resp_valid_i (dmi_rsp_valid),

    //JTAG
    .tck_i            (1'b0),
    .tms_i            (1'b0),
    .trst_ni          (1'b0),
    .td_i             (1'b0),
    .td_o             (),
    .tdo_oe_o         ()
  );
`endif

endmodule


