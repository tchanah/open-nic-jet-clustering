#!/usr/bin/env python3
"""Read and write the jet-clustering plugin's AXI-Lite registers over BAR2.

Run as root. Takes the card's MASTER PF (function .0) as a BDF.

    sudo ./jc_regs.py 0000:17:00.0 dump
    sudo ./jc_regs.py 0000:17:00.0 set-dst-mac 00:11:22:33:44:55
    sudo ./jc_regs.py 0000:17:00.0 set-r 0.4
    sudo ./jc_regs.py 0000:17:00.0 set-floor 50

WHERE THE WINDOW IS, derived rather than remembered:

    BAR2 + 0x100000   system_config_address_map.sv:258, C_BOX0_BASE_ADDR --
                      and box_250mhz_inst is wired to axil_box0_* in
                      open_nic_shell.sv, so box0 IS the 250 MHz box
        + 0x0         box_250mhz_address_map.v, C_P2P_BASE_ADDR, channel 0
                      (C_SIZE is 0x80, so channels sit at 0x0/0x80/0x100/0x180)
        + offset      jc_regs.sv's own map

Access is a direct mmap of the PF's resource2, the same method
open-nic-graph-plugin/test/cmac_status.py uses -- no pcimem, no ethtool.
Loads and stores must be 32-bit aligned: OpenNIC's AXI-Lite rejects sub-dword
access, and byte-wise mmap slicing reads back 0xFFFFFFFF.

R and the floor are converted through model/luts.json, NOT through numbers
copied into this file. Both are functions of the generated formats, so a
rerun of gen_luts.py that changes a width would otherwise leave this tool
quietly writing the wrong integer.
"""

import ctypes
import json
import mmap
import os
import pathlib
import sys

JC_BASE = 0x100000              # box0 (250 MHz) + p2p channel 0

REG = {
    "ID":          0x00,
    "SCRATCH":     0x04,
    "STATUS":      0x08,
    "RSQ_LO":      0x10,
    "RSQ_HI":      0x14,
    "FLR_0":       0x18,
    "FLR_1":       0x1C,
    "FLR_2":       0x20,
    "DMAC_LO":     0x24,
    "DMAC_HI":     0x28,
    "SMAC_LO":     0x2C,
    "SMAC_HI":     0x30,
    "TUSER_SRC":   0x34,
}

# The ladder, in datapath order. The first stage reading zero is the stage
# that failed -- that is what it is for.
LADDER = [
    ("frames_in",   0x40), ("bad_header", 0x44), ("bad_length", 0x48),
    ("accept",      0x4C), ("drop_full",  0x50), ("drop_err",   0x54),
    ("events",      0x58), ("jets_out",   0x5C), ("frames_out", 0x60),
    ("suppressed",  0x64), ("last_cycles", 0x68),
]

EXPECT_ID = 0x4A430001          # "JC" and the map version

LUTS = json.loads(
    (pathlib.Path(__file__).resolve().parents[1] / "model" / "luts.json")
    .read_text())
P4_FRAC = LUTS["formats"]["jc_lut_energy"]["frac"]
COORD_SCALE = LUTS["coord_scale"]
DELTA_SHIFT = LUTS["delta_shift"]


def r_squared(r_rad):
    """R^2 in the delta units jc_dist produces. Mirrors Formats.r_squared."""
    return (int(round(r_rad * COORD_SCALE)) >> DELTA_SHIFT) ** 2


def pt_sq_floor(gev):
    """The floor as pt^2 in Q28.68, which is what cfg_pt_sq_floor holds."""
    return int(round(gev * gev * (1 << (2 * P4_FRAC))))


def mac_to_int(s):
    parts = s.replace("-", ":").split(":")
    if len(parts) != 6:
        raise ValueError("expected aa:bb:cc:dd:ee:ff, got %r" % s)
    v = 0
    for p in parts:
        v = (v << 8) | int(p, 16)
    return v


def int_to_mac(v):
    return ":".join("%02x" % ((v >> (8 * i)) & 0xFF) for i in range(5, -1, -1))


class Bar:
    def __init__(self, bdf):
        path = "/sys/bus/pci/devices/%s/resource2" % bdf
        self.f = open(path, "r+b", buffering=0)
        self.m = mmap.mmap(self.f.fileno(), os.path.getsize(path))

    def rd(self, off):
        return int(ctypes.c_uint32.from_buffer(self.m, JC_BASE + off).value)

    def wr(self, off, val):
        ctypes.c_uint32.from_buffer(self.m, JC_BASE + off).value = val & 0xFFFFFFFF

    def check_id(self):
        ident = self.rd(REG["ID"])
        if ident == 0xFFFFFFFF:
            sys.exit(
                "ID reads 0xFFFFFFFF -- the card is not decoding memory.\n"
                "  FIRST check PCI memory-decode, which is the usual cause and\n"
                "  has nothing to do with the bitfile:\n"
                "      cat /sys/bus/pci/devices/<bdf>/enable      # 0 = no driver\n"
                "      setpci -s <bdf> COMMAND                    # 0000 = decode off\n"
                "  Every BAR read returns 0xFFFFFFFF until something calls\n"
                "  pci_enable_device(). resource2 existing in sysfs only means\n"
                "  the BAR was ASSIGNED, not that it is enabled.\n"
                "  Fix: insmod onic, or `echo 1 > /sys/bus/pci/devices/<bdf>/enable`.\n"
                "  Only if decode is already on does this mean a wrong BDF or an\n"
                "  unprogrammed card -- then check `lspci -d 10ee: -v`.")
        if ident != EXPECT_ID:
            sys.exit("ID reads 0x%08X, expected 0x%08X -- this is not the jet "
                     "clustering plugin. A stock p2p build reads something "
                     "else here, which is exactly the silent-fallback case."
                     % (ident, EXPECT_ID))
        return ident


def do_dump(b):
    b.check_id()
    print("ID          0x%08X   STATUS 0x%08X (engine %s)"
          % (b.rd(REG["ID"]), b.rd(REG["STATUS"]),
             "idle" if b.rd(REG["STATUS"]) & 1 else "busy"))

    rsq = b.rd(REG["RSQ_LO"]) | (b.rd(REG["RSQ_HI"]) << 32)
    flr = (b.rd(REG["FLR_0"]) | (b.rd(REG["FLR_1"]) << 32)
           | (b.rd(REG["FLR_2"]) << 64))
    # Invert the conversions so the printed value is in the units a physicist
    # asked for, not the integer the datapath ranks by.
    r_rad = ((rsq ** 0.5) * (1 << DELTA_SHIFT)) / COORD_SCALE
    floor_gev = (flr / float(1 << (2 * P4_FRAC))) ** 0.5
    print("R^2         %-24d  R      %.4f rad" % (rsq, r_rad))
    print("pt_sq_floor %-24d  floor  %.2f GeV" % (flr, floor_gev))

    dmac = b.rd(REG["DMAC_LO"]) | (b.rd(REG["DMAC_HI"]) << 32)
    smac = b.rd(REG["SMAC_LO"]) | (b.rd(REG["SMAC_HI"]) << 32)
    print("dst MAC     %s%s" % (int_to_mac(dmac),
                                "   <-- BROADCAST, unset" if dmac ==
                                0xFFFFFFFFFFFF else ""))
    print("src MAC     %s%s" % (int_to_mac(smac),
                                "   <-- ALL ZERO, unset" if smac == 0 else ""))
    print("tuser_src   %d" % b.rd(REG["TUSER_SRC"]))

    print("\ncounters (cumulative since reset, wrapping):")
    for name, off in LADDER:
        print("  %-12s %d" % (name, b.rd(off)))


def main():
    if len(sys.argv) < 3:
        sys.exit(__doc__)
    bdf, cmd, args = sys.argv[1], sys.argv[2], sys.argv[3:]

    try:
        b = Bar(bdf)
    except Exception as e:                                     # noqa: BLE001
        sys.exit("cannot map BAR2 of %s (%s) -- run as root?" % (bdf, e))

    if cmd == "dump":
        do_dump(b)
        return

    b.check_id()

    if cmd in ("set-dst-mac", "set-src-mac"):
        v = mac_to_int(args[0])
        lo, hi = ("DMAC_LO", "DMAC_HI") if cmd == "set-dst-mac" \
            else ("SMAC_LO", "SMAC_HI")
        b.wr(REG[lo], v & 0xFFFFFFFF)
        b.wr(REG[hi], (v >> 32) & 0xFFFF)
        print("%s = %s" % (cmd[4:], int_to_mac(
            b.rd(REG[lo]) | (b.rd(REG[hi]) << 32))))

    elif cmd == "set-r":
        v = r_squared(float(args[0]))
        b.wr(REG["RSQ_LO"], v & 0xFFFFFFFF)
        b.wr(REG["RSQ_HI"], (v >> 32) & 0xFFFFFFFF)
        print("R = %s rad -> R^2 = %d" % (args[0], v))

    elif cmd == "set-floor":
        v = pt_sq_floor(float(args[0]))
        b.wr(REG["FLR_0"], v & 0xFFFFFFFF)
        b.wr(REG["FLR_1"], (v >> 32) & 0xFFFFFFFF)
        b.wr(REG["FLR_2"], (v >> 64) & 0xFFFFFFFF)
        print("floor = %s GeV -> pt_sq_floor = %d" % (args[0], v))

    elif cmd == "set-tuser-src":
        b.wr(REG["TUSER_SRC"], int(args[0], 0) & 0xFFFF)
        print("tuser_src = %d" % b.rd(REG["TUSER_SRC"]))

    elif cmd == "scratch":
        # The cheapest proof the bus works at all, before believing anything
        # else this tool prints.
        b.wr(REG["SCRATCH"], 0xDEADBEEF)
        got = b.rd(REG["SCRATCH"])
        print("scratch wrote 0xDEADBEEF, read 0x%08X -- %s"
              % (got, "OK" if got == 0xDEADBEEF else "MISMATCH"))

    else:
        sys.exit(__doc__)


if __name__ == "__main__":
    main()
