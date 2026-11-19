"""scripts/mkspiffs_vex.py: programs installed with --run, plain and linked
(PROS hot/cold, VEXcode Python), round-trip through the SPIFFS image with
the meta VEXos expects. No brain needed: e2e tests (hotcold_test.sh,
python_run_test.sh) check that VEXos runs them, but skip without a PROS
build or VEX's SDK, so the layout is checked here every time."""
import gzip
import importlib.util
import struct
import sys
from pathlib import Path

import pytest

SCRIPT = Path(__file__).resolve().parents[2] / "scripts" / "mkspiffs_vex.py"
spec = importlib.util.spec_from_file_location("mkspiffs_vex", SCRIPT)
mk = importlib.util.module_from_spec(spec)
sys.modules["mkspiffs_vex"] = mk               # dataclasses look the module up
spec.loader.exec_module(mk)


def v5_image(version: int, payload: bytes = b"code" * 300) -> bytes:
    """A program or library with an XVX5 header (version at bytes 28..31)."""
    return b"XVX5" + bytes(24) + struct.pack("<I", version) + payload


def meta_fields(meta: bytes) -> dict:
    vid, _opt, _, size, addr, crc, kind, _stamp, version = struct.unpack_from("<BBHIII4sII", meta)
    link = meta[32:64].split(b"\0")[0]
    return {"vid": vid, "size": size, "addr": addr, "crc": crc, "kind": kind.rstrip(b"\0"),
            "version": version, "link": "" if link[:1] in (b"", b"\xff") else link.decode()}


def check_stored(data: bytes, meta: bytes) -> dict:
    """Size and CRC in the meta are those of the bytes stored."""
    f = meta_fields(meta)
    assert f["size"] == len(data)
    assert f["crc"] == mk.crc32_v5(data)
    return f


@pytest.fixture
def files(tmp_path):
    def write(name: str, data: bytes) -> Path:
        p = tmp_path / name
        p.write_bytes(data)
        return p
    return write


def build(*specs: str) -> dict:
    return mk.read_files(mk.build(list(specs)))


def test_a_plain_program_is_written_as_before(files):
    prog = files("hello.bin", v5_image(1))
    out = build(f"hello={prog}@3,autorun")
    data, meta = out["user/hello.bin"]
    assert data == prog.read_bytes()                       # stored as it is
    f = check_stored(data, meta)
    assert f["addr"] == mk.USER_LOAD_ADDR and f["link"] == ""
    ini, ini_meta = out["user/hello.ini"]
    assert b"slot = 3" in ini and b"autorun = true" in ini
    fi = check_stored(ini, ini_meta)
    assert fi["kind"] == b"ini" and fi["addr"] == mk.USER_LOAD_ADDR


def test_a_pros_hot_cold_pair_links_the_hot_package_to_the_cold_one(files):
    hot = files("hot.package.bin", v5_image(0x01000000, b"hot" * 500))
    cold = files("cold.package.bin", v5_image(0x01000000, b"cold" * 4000))
    out = build(f"slot_1={hot}@1,link={cold}")
    cold_name = f"pros/cold_{mk.crc32_v5(cold.read_bytes()):08x}"
    lib, lib_meta = out[cold_name]
    assert gzip.decompress(lib) == cold.read_bytes()        # stored gzip, as uploads are
    fl = check_stored(lib, lib_meta)
    assert fl["vid"] == mk.VID_PROS and fl["addr"] == mk.LIB_LOAD_ADDR
    prog, prog_meta = out["user/slot_1.bin"]
    assert gzip.decompress(prog) == hot.read_bytes()
    fp = check_stored(prog, prog_meta)
    assert fp["addr"] == mk.HOT_LOAD_ADDR and fp["link"] == cold_name
    assert check_stored(*out["user/slot_1.ini"])["addr"] == mk.HOT_LOAD_ADDR


def test_programs_built_on_one_cold_package_share_it(files):
    cold = files("cold.package.bin", v5_image(0x01000000, b"cold" * 4000))
    a = files("a.bin", v5_image(0x01000000, b"a" * 100))
    b = files("b.bin", v5_image(0x01000000, b"b" * 100))
    out = build(f"a={a}@1,link={cold}", f"b={b}@2,link={cold}")
    assert sum(name.startswith("pros/cold_") for name in out) == 1
    assert meta_fields(out["user/a.bin"][1])["link"] == meta_fields(out["user/b.bin"][1])["link"]


def test_a_vexcode_python_program_links_to_vexs_vm(files):
    main = files("main.py", b"from vex import *\nbrain = Brain()\n")
    vm = files("python_vm.bin", v5_image(0x00010203, b"vm" * 3000))
    out = build(f"main={main}@4,link={vm}")
    lib, lib_meta = out["vxvm/python_vm.bin"]
    fl = check_stored(lib, lib_meta)
    assert fl["vid"] == mk.VID_VXVM and fl["addr"] == mk.LIB_LOAD_ADDR
    assert fl["version"] == 0x00010203                      # from the VM's header
    prog, prog_meta = out["user/main.bin"]
    assert gzip.decompress(prog) == main.read_bytes()
    fp = check_stored(prog, prog_meta)
    assert fp["addr"] == mk.PY_LOAD_ADDR and fp["kind"] == b"bina" and fp["link"] == "vxvm/python_vm.bin"


def test_a_swapped_library_is_refused_by_name(files):
    cold = files("cold.package.bin", v5_image(0x01000000))
    vm = files("python_vm.bin", v5_image(1))
    hot = files("hot.package.bin", v5_image(0x01000000))
    main = files("main.py", b"print(1)\n")
    with pytest.raises(SystemExit, match="PROS cold package.*Python VM"):
        build(f"main={main},link={cold}")
    with pytest.raises(SystemExit, match="Python VM.*only a Python program"):
        build(f"slot_1={hot},link={vm}")


def test_a_bad_vm_names_the_file(files):
    main = files("main.py", b"print(1)\n")
    with pytest.raises(SystemExit, match=r"vm\.bin: not a V5 program image"):
        build(f"main={main},link={files('vm.bin', b'not a program')}")
    with pytest.raises(SystemExit, match=r"vm\.bin: too short"):
        build(f"main={main},link={files('vm.bin', b'XVX5' + bytes(8))}")


def test_only_a_trailing_number_is_the_slot():
    assert mk.parse_spec("p=/home/a@b/x.bin@3,autorun") == mk.Spec("p", Path("/home/a@b/x.bin"), 3, True)
    assert mk.parse_spec("p=/home/a@b/x.bin").slot == 0
    assert mk.parse_spec("p=/home/a@b/x.bin").path == Path("/home/a@b/x.bin")
