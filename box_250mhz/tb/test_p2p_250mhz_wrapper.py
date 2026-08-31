import itertools
import logging

import cocotb
from cocotb.clock import Clock
from cocotb.regression import TestFactory
from cocotb.triggers import RisingEdge, with_timeout
from cocotbext.axi import (AxiLiteBus, AxiLiteMaster, AxiStreamBus,
                           AxiStreamFrame, AxiStreamSink, AxiStreamSource)
from scapy.all import IP, UDP, Ether

# UDP packet with 128B payload
PACKET = Ether(src='aa:bb:cc:dd:ee:ff', dst='11:22:33:44:55:66') \
    / IP(src='1.1.1.1', dst='2.2.2.2') \
    / UDP(sport=11111, dport=22222) / (b'\xaa'*128)


class TB:
    def __init__(self, dut):
        self.dut = dut

        self.log = logging.getLogger("cocotb.tb")
        self.log.setLevel(logging.DEBUG)
        self.log.info("Got DUT: {}".format(dut))

        cocotb.fork(Clock(dut.axis_aclk, 2, units="ns").start())
        cocotb.fork(Clock(dut.axil_aclk, 4, units="ns").start())

        # Note, cocotb by default assumes reset signals are active high, while
        # open nic shell has reset signals active low. This is why we pass
        # reset_active_level=False.
        self.source_tx = [AxiStreamSource(
            AxiStreamBus.from_prefix(
                dut, "s_axis_qdma_h2c_port{}".format(port)),
            dut.axis_aclk, dut.p2p_250mhz_inst.axil_aresetn,
            reset_active_level=False)
            for port in [0, 1]]
        self.source_rx = [AxiStreamSource(
            AxiStreamBus.from_prefix(
                dut, "s_axis_adap_rx_250mhz_port{}".format(port)),
            dut.axis_aclk, dut.p2p_250mhz_inst.axil_aresetn,
            reset_active_level=False)
            for port in [0, 1]]
        self.sink_tx = [AxiStreamSink(
            AxiStreamBus.from_prefix(
                dut, "m_axis_adap_tx_250mhz_port{}".format(port)),
            dut.axis_aclk, dut.p2p_250mhz_inst.axil_aresetn,
            reset_active_level=False)
            for port in [0, 1]]
        self.sink_rx = [AxiStreamSink(
            AxiStreamBus.from_prefix(
                dut, "m_axis_qdma_c2h_port{}".format(port)),
            dut.axis_aclk, dut.p2p_250mhz_inst.axil_aresetn,
            reset_active_level=False)
            for port in [0, 1]]
        self.control = AxiLiteMaster(
            AxiLiteBus.from_prefix(dut, "s_axil"),
            dut.axil_aclk, dut.p2p_250mhz_inst.axil_aresetn,
            reset_active_level=False)

    def set_idle_generator(self, generator=None):
        if generator:
            for source_tx in self.source_tx:
                source_tx.set_pause_generator(generator())

    def set_backpressure_generator(self, generator=None):
        if generator:
            for sink_tx in self.sink_tx:
                sink_tx.set_pause_generator(generator())

    async def reset(self):
        self.dut.mod_rstn.setimmediatevalue(1)
        # mod rst signals are synced with the axi_aclk
        await RisingEdge(self.dut.axil_aclk)
        await RisingEdge(self.dut.axil_aclk)
        self.dut.mod_rstn.value = 0
        await RisingEdge(self.dut.axil_aclk)
        await RisingEdge(self.dut.axil_aclk)
        self.dut.mod_rstn.value = 1
        await RisingEdge(self.dut.mod_rst_done)


async def check_connection(
        tb: TB, source: AxiStreamSource,
        sink: AxiStreamSink, test_packet=PACKET):
    # Pkts on source should arrive at sink
    test_frames = []
    test_frame = AxiStreamFrame(bytes(test_packet), tuser=0)
    await source.send(test_frame)
    test_frames.append(test_frame)
    tb.log.info("Frame sent")

    for test_frame in test_frames:
        tb.log.info("Trying to recv frame")
        # Bounded. An unbounded recv() on a frame that never arrives leaves
        # vvp advancing time at 100% CPU forever, which is indistinguishable
        # from a slow run until you look at ps.
        rx_frame = await with_timeout(sink.recv(), 50, "us")
        assert rx_frame.tdata == test_frame.tdata

    assert sink.empty()


async def run_test(dut, idle_inserter=None, backpressure_inserter=None):
    """The TX direction only -- it is still a pass-through.

    The RX direction is no longer one: it is the clustering datapath, so
    feeding it a UDP packet and expecting the same bytes back would be
    asserting that the plugin does nothing. Those two checks moved to the
    jets tests below, and the real chain-level coverage is one level down in
    unit/test_jet_clustering.py.
    """
    tb = TB(dut)

    await tb.reset()

    tb.set_idle_generator(idle_inserter)
    tb.set_backpressure_generator(backpressure_inserter)

    # Port 0 only. Port 1's TX is the jets egress now, and H2C[1] is drained,
    # so a host-TX pass-through there would have nowhere to go.
    await check_connection(tb, tb.source_tx[0], tb.sink_tx[0], PACKET)

    # Due to some bugs in cocotb following lines are needed.
    # Check cocotb gitter for details.
    await RisingEdge(dut.axis_aclk)
    await RisingEdge(dut.axis_aclk)


def cycle_pause():
    return itertools.cycle([1, 1, 1, 0])


# ---------------------------------------------------------------------------
# Jets through the shell's own plumbing.
#
# unit/test_jet_clustering.py is where the chain is verified against the model,
# many events and bit-exactly. THESE two tests cover only what that one cannot
# reach: the box's AXI-Lite channel routing, and the tuser/steering wiring
# between p2p_250mhz and the plugin. A smoke test on purpose -- if it fails
# while the unit bench passes, the fault is in the plumbing, not the datapath.
# ---------------------------------------------------------------------------
@cocotb.test(timeout_time=100, timeout_unit="us")
async def test_register_file_is_reachable(dut):
    """Channel 0 of the box's AXI-Lite reaches jc_regs, and channel 1 answers.

    Channel 1 matters: stock p2p left its *ready outputs undriven, so a read
    at 0x0080 would have hung the crossbar. It is a sink, not a register file,
    so only the fact that it completes is checked.
    """
    tb = TB(dut)
    await tb.reset()

    ident = await tb.control.read_dword(0x0000)
    assert ident == 0x4A430001, (
        f"ID register reads {ident:#010x}; the box's ingress AXI-Lite channel "
        f"does not reach jc_regs")

    await tb.control.write_dword(0x0004, 0xC0FFEE00)
    assert await tb.control.read_dword(0x0004) == 0xC0FFEE00

    await tb.control.read_dword(0x0080)     # must complete, value irrelevant


# A 35-cell event is ~4k cycles at 2 ns, so ~8 us; 150 us is 18x margin and
# still fails in under a minute. Without a timeout, recv() on a frame that
# never arrives leaves vvp advancing time at 100% CPU indefinitely -- which
# looks exactly like a slow compile from outside.
@cocotb.test(timeout_time=150, timeout_unit="us")
async def test_rx_produces_a_jets_frame(dut):
    """Bump in the wire: cells in on CMAC port 0 RX, jets out on port 1 TX.

    The topology check. Jets must reach the NETWORK, not the host -- QDMA C2H
    is tied off, so a frame appearing there instead would mean the plugin is
    wired back to the stock RX path.
    """
    import json
    import pathlib
    import sys
    root = pathlib.Path(__file__).resolve().parents[2]
    sys.path.insert(0, str(root / "model"))
    import jc_model as M
    from jc_frames import build_event_frame, parse_jets_bytes, ETH_TYPE_OUT

    luts_path = root / "model" / "luts.json"
    fmt = M.Formats(luts_path)
    default_r = json.loads(luts_path.read_text())["default_r"]
    src = root / "box_250mhz" / "tb" / "unit" / "data" / "events.pkt.bin"
    assert src.exists(), f"missing fixture {src}; run model/make_fixture.py"

    tb = TB(dut)
    await tb.reset()

    # Floor to zero, so any event produces jets and the test cannot pass by
    # the trigger legitimately staying silent.
    for off, word in ((0x0018, 0), (0x001C, 0), (0x0020, 0)):
        await tb.control.write_dword(off, word)
    for _ in range(40):
        await RisingEdge(dut.axil_aclk)

    seq, cells = next((s, c) for s, _t, c in M.read_packets(src) if len(c) <= 60)
    want = M.cluster_fixed(cells, fmt, default_r, 0.0)

    await tb.source_rx[0].send(
        AxiStreamFrame(build_event_frame(cells, seq), tuser=0))
    rx = await tb.sink_tx[1].recv()          # CMAC port 1 TX, not the host
    assert tb.sink_rx[0].empty() and tb.sink_rx[1].empty(), (
        "a jets frame went up QDMA C2H; the plugin is wired to the stock RX "
        "path rather than out CMAC port 1")

    f = parse_jets_bytes(bytes(rx.tdata))
    assert f["ethertype"] == ETH_TYPE_OUT, f"ethertype {f['ethertype']:#06x}"
    assert f["seq"] == seq, f"seq {f['seq']} vs {seq}"
    assert f["njets"] == len(want), f"{f['njets']} jets, model says {len(want)}"
    tb.log.info("seq %d: n=%d -> %d jets in %d cycles through the shell",
                seq, len(cells), f["njets"], f["cycles"])


if cocotb.SIM_NAME:
    factory = TestFactory(run_test)
    factory.add_option("idle_inserter", [None, cycle_pause])
    factory.add_option("backpressure_inserter", [None, cycle_pause])
    factory.generate_tests()
