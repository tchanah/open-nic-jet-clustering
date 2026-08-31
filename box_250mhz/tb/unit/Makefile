# Per-module cocotb benches for the jet clustering datapath.
#
# Run one:
#   make DUT=jc_deframe
#   make DUT=jc_deframe clean
#
# Each DUT needs jc_<name>.sv at the plugin root and test_jc_<name>.py here.
# The full-plugin bench lives one level up in ../Makefile.

DUT ?= jc_deframe

SIM ?= icarus
TOPLEVEL_LANG ?= verilog

PLUGIN_ROOT := $(abspath ../../..)
SHELL_SRC   ?= $(abspath ../../../../open-nic-shell/src)

# Most DUTs are a single self-contained file. A few need their submodules and
# a harness that wires them together; those declare it here rather than in a
# per-DUT Makefile, so `make DUT=<name>` stays the only thing to remember.
EXTRA_jc_sweep  := $(PLUGIN_ROOT)/jc_dist.sv $(PLUGIN_ROOT)/jc_mem.sv \
                   jc_sweep_top.sv
TOP_jc_sweep    := jc_sweep_top

EXTRA_jc_setkin := $(PLUGIN_ROOT)/jc_log2.sv $(PLUGIN_ROOT)/jc_cordic.sv

EXTRA_jc_engine := $(addprefix $(PLUGIN_ROOT)/, \
                     jc_dist.sv jc_mem.sv jc_sweep.sv \
                     jc_log2.sv jc_cordic.sv jc_setkin.sv jc_ctrl.sv)

EXTRA_jc_reframe := $(PLUGIN_ROOT)/jc_fp32.sv

# The whole chain. sim_models.sv supplies xpm_cdc_handshake, which jc_regs
# needs and Icarus cannot elaborate; axi_lite_register comes from the shell,
# read-only, exactly as the stock p2p sink already uses it.
EXTRA_jet_clustering := \
	$(addprefix $(PLUGIN_ROOT)/, \
	  jc_dist.sv jc_mem.sv jc_sweep.sv jc_log2.sv jc_cordic.sv \
	  jc_setkin.sv jc_ctrl.sv jc_engine.sv \
	  jc_deframe.sv jc_ingest.sv jc_evbuf.sv \
	  jc_fp32.sv jc_reframe.sv jc_regs.sv) \
	$(SHELL_SRC)/utility/axi_lite_register.sv \
	$(PLUGIN_ROOT)/box_250mhz/tb/sim_models.sv

VERILOG_SOURCES := \
	$(PLUGIN_ROOT)/$(DUT).sv \
	$(EXTRA_$(DUT))

# Headers are `include-d, not compiled: jc_defs.vh at the plugin root, the
# generated tables in gen/, and the shell's open_nic_shell_macros.vh.
GEN_DIR := $(PLUGIN_ROOT)/gen
COMPILE_ARGS += -I$(PLUGIN_ROOT) -I$(GEN_DIR) -I$(SHELL_SRC)

# `include-d files are invisible to make's dependency graph, so without this a
# regenerated table leaves a stale sim.vvp in place and the bench compares new
# expectations against old data -- which looks exactly like an RTL bug.
CUSTOM_COMPILE_DEPS += $(PLUGIN_ROOT)/jc_defs.vh
CUSTOM_COMPILE_DEPS += $(wildcard $(GEN_DIR)/*.vh)

TOPLEVEL := $(if $(TOP_$(DUT)),$(TOP_$(DUT)),$(DUT))
MODULE   := test_$(DUT)

# Keep per-DUT build output apart so switching DUT does not force a rebuild
SIM_BUILD := sim_build_$(DUT)

include $(shell cocotb-config --makefiles)/Makefile.sim
