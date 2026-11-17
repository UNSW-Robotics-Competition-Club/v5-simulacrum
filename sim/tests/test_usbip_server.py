"""scripts/v5usb_usbip.py: one Windows client at a time.

A second import of the brain used to run beside the first: both sessions
parked bulk IN URBs on the brain's one queue, so replies went to the wrong
client (VEXcode: "rx buffer size error") and Windows stacked a COM-port pair
per attach. Taking the brain over instead made two auto-reattaching
usbip-win2 attachments knock each other off every 30 s (USB unplug/plug).
So a second import is refused as busy, like Linux usbipd, until the first
client closes.
"""
import asyncio
import importlib.util
import os
import struct
import tempfile
from pathlib import Path

SCRIPT = Path(__file__).resolve().parents[2] / "scripts" / "v5usb_usbip.py"
spec = importlib.util.spec_from_file_location("v5usb_usbip", SCRIPT)
u = importlib.util.module_from_spec(spec)
spec.loader.exec_module(u)


async def fake_qemu(path):
    """A CDC chardev socket; returns (server, queue of connected writers)."""
    conns = asyncio.Queue()
    server = await asyncio.start_unix_server(lambda r, w: conns.put_nowait((r, w)), path)
    return server, conns


async def import_brain(port, expect=0):
    reader, writer = await asyncio.open_connection("127.0.0.1", port)
    writer.write(struct.pack(">HHI", u.VERSION, u.OP_REQ_IMPORT, 0) + u.BUSID.ljust(32, b"\0"))
    await writer.drain()
    _ver, code, status = struct.unpack(">HHI", await reader.readexactly(8))
    assert (code, status) == (u.OP_REP_IMPORT, expect)
    if status == 0:
        await reader.readexactly(len(u.device_record()))
    return reader, writer


async def reply_reaches(reader, writer, qemu_sys_w, seq):
    writer.write(bulk_in(seq, 2))
    await writer.drain()
    await asyncio.sleep(0.05)
    qemu_sys_w.write(b"\xaa\x55reply")
    await qemu_sys_w.drain()
    hdr = await asyncio.wait_for(reader.readexactly(48), 2)
    cmd, got = struct.unpack(">II", hdr[:8])
    status, actual = struct.unpack(">ii", hdr[20:28])
    assert (cmd, got, status) == (u.RET_SUBMIT, seq, 0)
    assert await reader.readexactly(actual) == b"\xaa\x55reply"


def bulk_in(seq, ep, length=64):
    return (struct.pack(">IIIII", u.CMD_SUBMIT, seq, 0, u.DIR_IN, ep)
            + struct.pack(">IiiiI", 0, length, 0, 0, 0) + bytes(8))


async def scenario():
    with tempfile.TemporaryDirectory() as d:
        sys_path, user_path = os.path.join(d, "sys.sock"), os.path.join(d, "user.sock")
        qemu_sys, sys_conns = await fake_qemu(sys_path)
        qemu_user, _ = await fake_qemu(user_path)
        brain = u.Brain({"sys": sys_path, "user": user_path})
        await brain.connect()
        _, qemu_sys_w = await sys_conns.get()
        server = await asyncio.start_server(lambda r, w: u.handle(brain, r, w), "127.0.0.1", 0)
        port = server.sockets[0].getsockname()[1]

        r1, w1 = await import_brain(port)
        r2, w2 = await import_brain(port, expect=u.ST_DEV_BUSY)    # busy: refused
        assert await asyncio.wait_for(r2.read(), 2) == b"", "the refused client is closed"
        await reply_reaches(r1, w1, qemu_sys_w, 1)                 # the first keeps working

        w1.close()                                                 # first client goes away
        await asyncio.sleep(0.1)
        r3, w3 = await import_brain(port)                          # now a new one may attach
        await reply_reaches(r3, w3, qemu_sys_w, 7)

        w3.close()
        server.close()
        qemu_sys.close()
        qemu_user.close()


def test_a_second_attach_is_refused_while_the_first_is_attached():
    asyncio.run(scenario())
