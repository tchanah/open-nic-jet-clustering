# Cocotb testbench for the jet-clustering plugin box_250mhz.
#
# Prerequisites (one-time):
#   sudo apt install iverilog
#   pip3 install cocotb cocotbext-axi scapy
#
# Run:
#   make            # build + run
#   make clean      # drop sim_build/ and results.xml
#
# Assumes open-nic-shell is checked out as a sibling of this plugin repo
# (override with: make SHELL_SRC=/path/to/open-nic-shell/src).

SIM ?= icarus
TOPLEVEL_LANG ?= verilog

PLUGIN_ROOT := $(abspath ../..)
SHELL_SRC   ?= $(abspath ../../../open-nic-shell/src)

# sim_models.sv supplies behavioural stand-ins for the Vivado IP that Icarus
# cannot elaborate: axi_stream_pipeline (axis_register_slice) and the
# xpm_cdc_single macro used by generic_reset. Synthesis never reads it --
# build_box_250mhz.tcl pulls the real IP instead.
VERILOG_SOURCES := \
	$(SHELL_SRC)/utility/generic_reset.sv \
	$(SHELL_SRC)/utility/axi_lite_register.sv \
	$(SHELL_SRC)/utility/axi_lite_slave.sv \
	$(addprefix $(PLUGIN_ROOT)/, \
	  jc_dist.sv jc_mem.sv jc_sweep.sv jc_log2.sv jc_cordic.sv \
	  jc_setkin.sv jc_ctrl.sv jc_engine.sv \
	  jc_deframe.sv jc_ingest.sv jc_evbuf.sv \
	  jc_fp32.sv jc_reframe.sv jc_regs.sv) \
	$(PLUGIN_ROOT)/jet_clustering.sv \
	$(PLUGIN_ROOT)/p2p_250mhz.sv \
	$(PWD)/sim_models.sv \
	$(PWD)/p2p_250mhz_wrapper.sv

# open_nic_shell_macros.vh in the shell src directory; jc_defs.vh at the plugin
# root; the generated tables in gen/.
COMPILE_ARGS += -I$(SHELL_SRC) -I$(PLUGIN_ROOT) -I$(PLUGIN_ROOT)/gen

# `include-d files are invisible to make, so a regenerated table would
# otherwise leave a stale sim.vvp comparing new expectations against old data.
CUSTOM_COMPILE_DEPS += $(PLUGIN_ROOT)/jc_defs.vh $(wildcard $(PLUGIN_ROOT)/gen/*.vh)

TOPLEVEL := p2p_250mhz_wrapper
MODULE   := test_p2p_250mhz_wrapper

include $(shell cocotb-config --makefiles)/Makefile.sim
