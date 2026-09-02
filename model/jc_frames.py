"""Wire-format helpers shared by the benches: build a cell event, read a jets frame.

The formats themselves are specified in jc_defs.vh; this is the one Python
statement of them, so a bench cannot drift from another bench. pixelize.py
already writes the INPUT format -- build_event_frame produces the same bytes,
which is what lets a test feed the RTL exactly what the model reads.
"""

import struct

ETH_TYPE_IN = 0x88B6            # cells, host -> card
ETH_TYPE_OUT = 0x88B7           # jets, card -> host. Deliberately different:
                                # a jets frame looped back into an RX port must
                                # be rejected, not parsed as sixteen cells/beat.
FMT_VERSION = 0x01
HDR_BYTES = 64
P4_FRAC = 34


# ------------------------------------------------------------------ input ---
def build_event_frame(cells, seq, ethertype=ETH_TYPE_IN, version=FMT_VERSION,
                      count=None, cells_total=None, dst_mac=b"\xff" * 6,
                      src_mac=b"\x00" * 6):
    """Cells -> the bytes the card receives. count/ethertype/version are
    overridable so a bench can build a deliberately malformed frame."""
    n = len(cells) if count is None else count
    hdr = bytearray(HDR_BYTES)
    hdr[0:6] = dst_mac
    hdr[6:12] = src_mac
    hdr[12:14] = ethertype.to_bytes(2, "big")
    hdr[14] = version
    hdr[16:18] = n.to_bytes(2, "big")
    hdr[18:22] = seq.to_bytes(4, "big")
    hdr[22:24] = (len(cells) if cells_total is None
                  else cells_total).to_bytes(2, "big")
    # {iy[7:0], iphi[7:0], ecode[15:0]} little-endian, so cell_data[31:24] is
    # iy exactly as jc_ingest slices it.
    body = b"".join((((iy << 24) | (iphi << 16) | ec) & 0xFFFFFFFF)
                    .to_bytes(4, "little") for iy, iphi, ec in cells)
    return bytes(hdr) + body


def to_beats(frame):
    """Frame bytes -> [(tdata, tkeep, tlast)], 64 B per beat."""
    out = []
    for off in range(0, len(frame), 64):
        chunk = frame[off:off + 64]
        out.append((int.from_bytes(chunk.ljust(64, b"\x00"), "little"),
                    (1 << len(chunk)) - 1,
                    (off + 64) >= len(frame)))
    return out


# ----------------------------------------------------------------- output ---
def fp32(v_float):
    """What a float64 becomes on the wire. jc_fp32 matches this bit for bit."""
    return struct.unpack(">f", struct.pack(">f", v_float))[0]


def expected_jets(model_jets):
    """Model four-momenta -> the sorted fp32 tuples the frame should carry."""
    return sorted(tuple(fp32(c) for c in j) for j in model_jets)


def parse_jets_frame(beats):
    """[(tdata, tkeep, tlast, tuser)] -> the header fields and the jets."""
    hdr = beats[0][0].to_bytes(64, "little")
    out = {
        "dst_mac": hdr[0:6],
        "src_mac": hdr[6:12],
        "ethertype": int.from_bytes(hdr[12:14], "big"),
        "version": hdr[14],
        "njets": int.from_bytes(hdr[16:18], "big"),
        "seq": int.from_bytes(hdr[18:22], "big"),
        "cycles": int.from_bytes(hdr[22:26], "big"),
        "drop_full": int.from_bytes(hdr[26:30], "big"),
        "drop_err": int.from_bytes(hdr[30:34], "big"),
        "bad_frame": int.from_bytes(hdr[34:38], "big"),
        "reserved": hdr[38:64],
        "jets": [],
    }
    if len(beats[0]) > 3:
        out["size"] = beats[0][3] & 0xFFFF
        out["src"] = (beats[0][3] >> 16) & 0xFFFF
    for beat in beats[1:]:
        data, keep = beat[0], beat[1]
        nbytes = bin(keep).count("1")
        raw = data.to_bytes(64, "little")
        for o in range(0, nbytes, 16):
            out["jets"].append(struct.unpack(">ffff", raw[o:o + 16]))
    return out


class ShortFrame(ValueError):
    """A captured frame is shorter than its own header says it should be."""


def parse_jets_bytes(payload):
    """Same, from a flat byte string -- for captures and cocotbext-axi benches.

    Raises ShortFrame rather than struct.error on a truncated capture. That is
    nearly always tcpdump killed before it flushed its last record, not a card
    fault, and the two have to be tellable apart at a glance -- so the check
    names the byte counts instead of dying somewhere inside struct.unpack.

    A short HEADER is the worse case and the reason this checks first: the
    slices below would read past the end and quietly return zero, so a
    truncated frame would parse as a perfectly valid one carrying no jets.
    """
    if len(payload) < HDR_BYTES:
        raise ShortFrame(
            f"frame is {len(payload)} bytes, shorter than the "
            f"{HDR_BYTES}-byte header -- truncated capture")
    hdr = payload[:HDR_BYTES]
    # The drop/bad counts are here as well as in jc_regs, and they are sampled
    # on the aclk side at jet_eoe -- so they stay readable even when the
    # register-file snapshot is not. That is how the duty cycle was measured
    # before the CDC re-arm fix reached hardware.
    out = {
        "ethertype": int.from_bytes(hdr[12:14], "big"),
        "version": hdr[14],
        "njets": int.from_bytes(hdr[16:18], "big"),
        "seq": int.from_bytes(hdr[18:22], "big"),
        "cycles": int.from_bytes(hdr[22:26], "big"),
        "drop_full": int.from_bytes(hdr[26:30], "big"),
        "drop_err": int.from_bytes(hdr[30:34], "big"),
        "bad_frame": int.from_bytes(hdr[34:38], "big"),
        "jets": [],
    }
    body = payload[HDR_BYTES:]
    need = out["njets"] * 16
    if len(body) < need:
        raise ShortFrame(
            f"seq {out['seq']}: header claims {out['njets']} jets, so the "
            f"frame should be {HDR_BYTES + need} bytes; it is {len(payload)} "
            f"-- truncated capture")
    for o in range(0, need, 16):
        out["jets"].append(struct.unpack(">ffff", body[o:o + 16]))
    return out
