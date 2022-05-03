
MRAM_MODEL?=/home/sebastien.riou/Librairies/GlobalFoundries/GF22MRAM/verilog/MRAM_eFLASH_256Kx78B_model_okvcs.v



VERILOG_SOURCES=../vendor/lowrisc_ip/dv/sv/dv_utils/dv_fcov_macros.svh\
../vendor/lowrisc_ip/ip/prim/rtl/prim_assert.sv \
../vendor/lowrisc_ip/ip/prim/rtl/prim_ram_1p_pkg.sv \
../vendor/lowrisc_ip/ip/prim/rtl/prim_secded_pkg.sv \
../vendor/lowrisc_ip/ip/prim/rtl/prim_clock_gating_sync.sv \
../syn/rtl/prim_clock_gating.v \
../dv/uvm/core_ibex/common/prim/prim_buf.sv \
../shared/rtl/bus.sv \
../shared/rtl/ram_1p.sv \
../shared/rtl/ram_2p.sv \
../shared/rtl/timer.sv \
../shared/rtl/sim/simulator_ctrl.sv \
../vendor/lowrisc_ip/ip/prim_generic/rtl/prim_generic_ram_2p.sv \
../vendor/lowrisc_ip/ip/prim_generic/rtl/prim_generic_buf.sv \
../vendor/lowrisc_ip/ip/prim/rtl/prim_ram_2p_pkg.sv \
../build/lowrisc_ibex_ibex_simple_system_0/src/lowrisc_prim_abstract_ram_2p_0/prim_ram_2p.sv \
../rtl/ibex_pkg.sv \
../rtl/ibex_tracer_pkg.sv \
../rtl/ibex_alu.sv \
../rtl/ibex_branch_predict.sv \
../rtl/ibex_compressed_decoder.sv \
../rtl/ibex_controller.sv \
../rtl/ibex_core.sv \
../rtl/ibex_counter.sv \
../rtl/ibex_cs_registers.sv \
../rtl/ibex_csr.sv \
../rtl/ibex_decoder.sv \
../rtl/ibex_dummy_instr.sv \
../rtl/ibex_ex_block.sv \
../rtl/ibex_fetch_fifo.sv \
../rtl/ibex_icache.sv \
../rtl/ibex_id_stage.sv \
../rtl/ibex_if_stage.sv \
../rtl/ibex_load_store_unit.sv \
../rtl/ibex_lockstep.sv \
../rtl/ibex_multdiv_fast.sv \
../rtl/ibex_multdiv_slow.sv \
../rtl/ibex_pmp.sv \
../rtl/ibex_prefetch_buffer.sv \
../rtl/ibex_register_file_ff.sv \
../rtl/ibex_register_file_fpga.sv \
../rtl/ibex_register_file_latch.sv \
../rtl/ibex_top_tracing.sv \
../rtl/ibex_top.sv \
../rtl/ibex_tracer.sv \
../rtl/ibex_wb_stage.sv 
#simple_system/rtl/ibex_simple_system.sv
#../rtl/ibex_pmp_reset_default.svh\

 

TESTBENCH=simple_system/rtl/ibex_simple_system.sv
INCDIR=../vendor/lowrisc_ip/ip/prim/rtl/+../vendor/lowrisc_ip/dv/sv/dv_utils
TOP=ibex_simple_system
WAVES?=1
JTAG_BLOCKING?=0

TIMESCALE= -timescale=1ns/10ps

VCS=vcs 
VCS_OPTS = -sverilog 
QUESTA_OPTS = -64 -sv  -work "work" -timescale "1ns/10ps" -mfcu   \
              -l "questa.log" -quiet 
                                       
QUESTA_LIBRARIES := -v $(STD_PATH)/lib/$(STD_VER) \
               -v $(STD_PATH)/lib/std_primitives.v

QUESTA_SIM_OPTS = 

ifeq ($(WAVES),1)
    QUESTA_SIM_OPTS += -voptargs="+acc" +define+WAVES=1
	VCS_OPTS += -debug_access+all +define+WAVES=1
endif

defines =+define+RVFI+define+SIM_NODELAY +define+JTAG_BLOCKING=$(JTAG_BLOCKING) 

all : 	questa_run_sim

vcs_compile : $(VERILOG_SOURCES)
	@mkdir -p build
	$(VCS) $(VCS_OPTS) $(TIMESCALE) $(VERILOG_SOURCES) $(TESTBENCH) -o build/simv $(defines) +define+INIT_TO_ONE +define+VCS_SIM

vcs_run_sim : build/simv
	./build/simv -ucli -do bench/sim.tcl

vcs_sim : vcs_compile | vcs_run_sim

questa_compile : $(VERILOG_SOURCES)
	vlib work
	vlog $(QUESTA_OPTS) $(TESTBENCH)  $(VERILOG_SOURCES) $(defines) +define+WAVES=$(WAVES) +define+QUESTA_SIM +incdir+${INCDIR}

questa_run_sim : questa_compile
	vsim -64 work.$(TOP) $(QUESTA_SIM_OPTS) #-c  -do " run -all; quit -f" -l run-questa.log 

questa_sim : questa_compile | questa_run_sim

wave 	: vcdplus.vpd 
	dve -vpd vcdplus.vpd -full64 & 
build/simv: vcs_compile
vcdplus.vpd : vcs_run_sim 

dve :
	dve -full64 & 

clean:
	@echo cleaning old simulation files and libraries...
	@ -$(RM) -r csrc simv* work *.bak *.log *.vpd *.vcd transcript \
	waves.shm *.wlf mylib lib *.vstf .restart* vlog.opt modelsim.ini \
	urgReport *.so vc_hdrs.h DVEfiles ucli.key build


