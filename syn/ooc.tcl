# *************************************************************************
#
# Out-of-context synthesis probe.
#
#   vivado -mode batch -source syn/ooc.tcl -tclargs jc_sweep
#   vivado -mode batch -source syn/ooc.tcl -tclargs jc_dist
#
# Run from the plugin root. Answers the two questions that cannot wait for a
# full build, in minutes rather than hours:
#
#   1. does the lane array close at 250 MHz?  If not the fix is a pipeline
#      stage, which changes sweep latency -> jc_ctrl's cycle accounting ->
#      jc_model.py. That is structural, so finding out before jc_ctrl exists
#      is worth far more than finding out after.
#   2. what does one engine cost?  Step 10 replicates from this number, and
#      the HLS figure it would otherwise use was measured on an empty device.
#
# Out of context means no I/O buffers and no surrounding shell, so the result
# is optimistic against an in-context build -- treat it as a screen, not a
# sign-off. A comfortable pass here does not guarantee closure at step 9; a
# fail here definitely means trouble.
#
# AND IT IS BLIND TO ANYTHING TOUCHING A PORT. The xdc below is a create_clock
# and nothing else -- no set_input_delay, no set_output_delay -- so Vivado
# never times a path that starts or ends at an input or output port. A leaf
# module's Fmax therefore covers its interior only. jc_ctrl reported +2.010 ns
# while carrying a -1.069 ns path from mem_p4_rd_px to jet_valid, because
# standalone that multiply's operand is a port; inside jc_engine it is a
# register in jc_mem and the path is real. For anything crossing a module
# boundary, only the jc_engine and jet_clustering rows have authority.
#
# *************************************************************************

if {[llength $argv] < 1} {
    puts "usage: vivado -mode batch -source syn/ooc.tcl -tclargs <top>"
    exit 1
}

set top   [lindex $argv 0]
set part  xcu250-figd2104-2L-e
set root  [file normalize [file dirname [info script]]/..]
set outdir $root/syn/reports
file mkdir $outdir

# Submodules each top needs. jc_sweep pulls in the lane and the memory.
switch -- $top {
    jc_sweep   { set srcs [list jc_dist.sv jc_mem.sv jc_sweep.sv] }
    jc_dist    { set srcs [list jc_dist.sv] }
    jc_mem     { set srcs [list jc_mem.sv] }
    jc_reframe { set srcs [list jc_fp32.sv jc_reframe.sv] }
    jc_setkin  { set srcs [list jc_log2.sv jc_cordic.sv jc_setkin.sv] }
    jc_engine  { set srcs [list jc_dist.sv jc_mem.sv jc_sweep.sv jc_log2.sv \
                                jc_cordic.sv jc_setkin.sv jc_ctrl.sv jc_engine.sv] }
    jet_clustering {
        # The whole plugin. Chiefly a SMOKE TEST before committing hours to a
        # full shell build: it proves the design elaborates and synthesises,
        # and gives a first area figure for step 10. Treat its Fmax with more
        # than the usual caution -- only aclk is constrained here, so the
        # AXI-Lite domain and the crossing between them are unconstrained.
        set srcs [list jc_dist.sv jc_log2.sv jc_cordic.sv jc_mem.sv \
                       jc_sweep.sv jc_setkin.sv jc_ctrl.sv jc_engine.sv \
                       jc_deframe.sv jc_ingest.sv jc_evbuf.sv \
                       jc_fp32.sv jc_reframe.sv jc_regs.sv jet_clustering.sv]
    }
    default    { set srcs [list $top.sv] }
}

# jc_regs instantiates the shell's axi_lite_register (common_clock, so no IP
# behind it). Read-only, the same way p2p's stock sink already uses it.
set shell_src [file normalize $root/../open-nic-shell/src]


create_project -in_memory -part $part

foreach f $srcs {
    read_verilog -sv $root/$f
}
if {[string equal $top jet_clustering]} {
    read_verilog -sv $shell_src/utility/axi_lite_register.sv
}
# Headers are `include-d, not compiled: jc_defs.vh at the root, the generated
# tables in gen/.
set_property include_dirs [list $root $root/gen] [current_fileset]

# Constrain before synthesis so the tool optimises against the real target
# rather than being measured against a clock it never saw.
set xdc $outdir/${top}_ooc.xdc
set fh [open $xdc w]
puts $fh "create_clock -period 4.000 -name aclk \[get_ports aclk\]"
close $fh
read_xdc -mode out_of_context $xdc

synth_design -top $top -mode out_of_context -part $part

report_timing_summary -delay_type max -max_paths 30 \
    -file $outdir/${top}_timing.rpt
report_utilization -file $outdir/${top}_util.rpt

report_timing -delay_type max -max_paths 20 -unique_pins -path_type summary \
    -file $outdir/${top}_worst20.rpt

# Pull the two numbers that actually decide anything to stdout, so the run
# does not require opening a report to read its own answer.
set period 4.000
set paths [get_timing_paths -max_paths 1 -delay_type max]

puts "=========================================================="
puts " $top out-of-context on $part"

# A module can have NO constrained register-to-register path at all -- every
# path touching a port, and ports unconstrained here. jc_fp32 is exactly that:
# one pipeline stage, fed from a port and read out to one.
#
# TEST THE SLACK, NOT THE PATH COUNT. get_timing_paths still returns a path
# object in that case; it is the SLACK property that comes back empty, so a
# guard on [llength $paths] passes straight into the else and throws anyway.
# Which is what it did: the error killed the script before the utilisation
# figures and printed FAILED in the summary for a design that had synthesised
# with zero errors.
set wns ""
if {[llength $paths] > 0} {
    set wns [get_property SLACK [lindex $paths 0]]
}

if {![string is double -strict $wns]} {
    puts "   WNS at 250 MHz : n/a  (no constrained reg-to-reg path)"
    puts "   implied Fmax   : n/a"
} else {
    set fmax [expr {1000.0 / ($period - $wns)}]
    puts "   WNS at 250 MHz : $wns ns"
    puts "   implied Fmax   : [format %.1f $fmax] MHz"
}

# Read the numbers out of the report rather than through get_utilization,
# which is not present in every Vivado build.
#
# Two traps in that report, both of which silently reported ZERO LUTs for a
# design using 57k of them:
#   - the row is "CLB LUTs*", with a footnote asterisk, so a pattern expecting
#     whitespace after the name never matches it;
#   - the same site names appear again in the per-SLR section further down,
#     where they are 0, so a plain foreach prints the wrong one last.
# Hence the optional asterisk and first-match-wins.
array set seen {}
foreach line [split [read [open $outdir/${top}_util.rpt r]] "\n"] {
    if {[regexp {^\|\s+(CLB LUTs|LUT as Logic|LUT as Memory|CLB Registers|Block RAM Tile|DSPs)\*?\s+\|\s+(\d+)} \
         $line -> name count]} {
        if {![info exists seen($name)]} {
            set seen($name) 1
            puts [format "   %-18s %s" $name $count]
        }
    }
}

# The worst path per DISTINCT ENDPOINT BUS.
#
# Neither report above gives this. They rank by slack, so one wide register
# fills the whole report: twenty paths once meant twenty bits of jet_pt_sq_r
# and one real problem, with the next problem invisible until that one was
# fixed and the sweep re-run -- twenty minutes per hidden path. -unique_pins
# does not help, because each bit of a bus IS a distinct pin. Stripping the
# bit index is what actually collapses them.
#
# In a catch, and last, so a wrong property name costs this report and not the
# WNS and area figures above it.
if {[catch {
    array set best {}
    foreach p [get_timing_paths -max_paths 400 -nworst 1 -delay_type max] {
        set sp [get_property STARTPOINT_PIN $p]
        set ep [get_property ENDPOINT_PIN $p]
        set slack [get_property SLACK $p]
        regsub -all {\[[0-9]+\]} $sp {[*]} spb
        regsub -all {\[[0-9]+\]} $ep {[*]} epb
        set key "$spb -> $epb"
        if {![info exists best($key)] || $slack < $best($key)} {
            set best($key) $slack
        }
    }
    set rows {}
    foreach key [array names best] { lappend rows [list $best($key) $key] }
    set rows [lsort -real -index 0 $rows]

    set fh [open $outdir/${top}_distinct.rpt w]
    puts $fh "worst path per distinct endpoint bus -- $top"
    puts $fh ""
    set n 0
    foreach r $rows {
        puts $fh [format "%9.3f  %s" [lindex $r 0] [lindex $r 1]]
        incr n
        if {$n >= 25} break
    }
    close $fh
    puts "   distinct paths     [llength $rows] -> ${top}_distinct.rpt"
} err]} {
    puts "   distinct-path report skipped: $err"
}

puts "=========================================================="
