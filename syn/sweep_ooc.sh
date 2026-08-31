#!/bin/bash
# Out-of-context synthesis of every module, one after another.
#
# WHY THIS EXISTS. Nine of twelve modules had never been synthesised when the
# whole plugin first went through: only jc_dist, jc_sweep and jc_reframe had
# Fmax numbers. The first full-plugin run failed timing at 172 MHz on a path
# inside jc_setkin -- mT2 = max(pt_sq, (E+pz)(E-pz)) in a single cycle, 26
# logic levels through two DSP cascades and a 96-bit compare.
#
# Fixing that one path only reveals the next. Synthesising every module in one
# pass gives the whole list at once, so the pipelining work can be planned
# rather than discovered an iteration at a time.
#
# Run from the plugin root:
#   nohup bash syn/sweep_ooc.sh > syn/reports/sweep.log 2>&1 &
#
# Then read the summary at the end of the log. Each module is a few minutes;
# jc_engine and jet_clustering are the long ones.
#
# NAME MODULES TO RUN ONLY THOSE, once the sweep has been done once and the
# question is whether one fix landed:
#
#   nohup bash syn/sweep_ooc.sh jc_ctrl jc_engine jet_clustering \
#         > syn/reports/sweep_4.log 2>&1 &
#
# ALWAYS INCLUDE jc_engine AND jet_clustering, whatever else you name. A leaf
# module's number covers its interior only -- ooc.tcl constrains no ports, so
# nothing that crosses a module boundary is timed there. jc_ctrl reported
# +2.010 ns while carrying the -1.069 ns path that gated the whole design.
# Re-running only the leaf you edited can therefore "pass" a fix that did not
# work.

set -u
cd "$(dirname "$0")/.."
mkdir -p syn/reports

# jc_fp32 is in the list as of step 8t, and it reports n/a. It used to be
# excluded for having no aclk for create_clock to attach to -- which meant the
# one module the sweep could not see was the one carrying the last failing
# path, 22 logic levels from jc_ctrl's jet register into jc_reframe's buffer.
# It has a clock now, but ONE register stage with a port on either side, so it
# still has no internal reg-to-reg path to time. It is listed anyway: n/a is
# the honest record of why nothing here can measure it, and jc_reframe now
# does measure its second half properly (norm_r -> bank, +1.923 ns).
ALL_MODULES="jc_dist jc_log2 jc_cordic jc_mem jc_ingest jc_deframe jc_evbuf \
             jc_sweep jc_fp32 jc_setkin jc_reframe jc_ctrl jc_engine \
             jet_clustering"

if [ $# -gt 0 ]; then
    MODULES="$*"
    for m in $MODULES; do
        case " $ALL_MODULES " in
            *" $m "*) ;;
            *) echo "unknown module '$m'"; echo "known: $ALL_MODULES"; exit 1 ;;
        esac
    done
    echo "=== subset run: $MODULES"
    case " $MODULES " in
        *" jet_clustering "*) ;;
        *) echo "=== WARNING: jet_clustering not in the list. Leaf modules"
           echo "===          cannot clear a path that crosses a boundary." ;;
    esac
else
    MODULES="$ALL_MODULES"
fi

for m in $MODULES; do
    echo "=============================================================="
    echo "=== $m   $(date +%H:%M:%S)"
    echo "=============================================================="
    vivado -mode batch -source syn/ooc.tcl -tclargs "$m" \
        > "syn/reports/${m}_ooc.log" 2>&1
    # The two lines that matter, pulled out of each run's log.
    #
    # `grep -v puts` because Vivado echoes the script's own source: the line
    # `# puts "   WNS at 250 MHz : $wns ns"` matches every pattern here and
    # comes FIRST, so an unfiltered first-match reads the tcl rather than the
    # answer. That is what made the summary below print `Fmax implied WNS 250`
    # for all thirteen modules -- the field positions were right for the wrong
    # line, so it looked like a formatting quirk rather than no data at all.
    grep -E "WNS at 250 MHz|implied Fmax|CLB LUTs|CLB Registers|DSPs|Block RAM Tile|ERROR" \
        "syn/reports/${m}_ooc.log" | grep -v 'puts' | sed 's/^/    /'
done

echo
echo "=============================================================="
echo "=== SUMMARY -- anything below 250 MHz needs pipelining"
echo "=============================================================="
for m in $MODULES; do
    f="syn/reports/${m}_ooc.log"
    [ -f "$f" ] || continue
    fmax=$(grep "implied Fmax"    "$f" | grep -v 'puts' | head -1 | awk '{print $4}')
    wns=$( grep "WNS at 250 MHz" "$f" | grep -v 'puts' | head -1 | awk '{print $6}')
    printf "  %-16s Fmax %-9s WNS %s\n" "$m" "${fmax:-FAILED}" "${wns:-?}"
done

echo
echo "=== worst path per distinct endpoint bus"
for m in $MODULES; do
    f="syn/reports/${m}_distinct.rpt"
    [ -f "$f" ] || continue
    echo "--- $m"
    sed -n '3,8p' "$f"
done
