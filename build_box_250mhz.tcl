# *************************************************************************
#
# Copyright 2020 Xilinx, Inc.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.
#
# *************************************************************************
if {$num_qdma > 1} {
    source box_250mhz/box_250mhz_axis_switch.tcl
}
# Headers are `include-d, not compiled: jc_defs.vh here, the generated tables
# in gen/. Both must be on the include path.
#
# APPEND TO THE SHELL'S OWN LIST -- set_property alone does not survive.
# open-nic-shell/script/build.tcl snapshots the fileset's include_dirs into a
# Tcl variable BEFORE sourcing this file (:355), and writes that snapshot back
# AFTER (:376). Anything this script sets on the fileset is therefore silently
# discarded, and synthesis fails hunting for jc_defs.vh. `source` runs in the
# caller's scope, so appending to the variable itself is what lands.
#
# The graph plugin never hit this: it has no headers, so its build_box_250mhz
# does not touch include_dirs at all. There is no prior art here to copy.
#
# build.tcl has already `cd`-ed to the plugin root, so [pwd] is that root.
lappend include_dirs [pwd] [pwd]/gen
set_property include_dirs $include_dirs [current_fileset]

foreach f {jc_dist.sv jc_log2.sv jc_cordic.sv jc_mem.sv jc_sweep.sv \
           jc_setkin.sv jc_ctrl.sv jc_engine.sv \
           jc_deframe.sv jc_ingest.sv jc_evbuf.sv \
           jc_fp32.sv jc_reframe.sv jc_regs.sv} {
    if {[file exists [file dirname [info script]]/$f]} {
        read_verilog -quiet -sv [file dirname [info script]]/$f
    }
}
read_verilog -quiet -sv jet_clustering.sv
read_verilog -quiet -sv p2p_250mhz.sv
