from v5sim import protocol


def test_build_and_validate_at_any_length():
    for n in (16, 64):
        f = protocol.build(b"\x9d\x10", n)
        assert len(f) == n and protocol.is_valid(f)
        assert not protocol.is_valid(f[:-1] + bytes([f[-1] ^ 1]))


def test_reader_splits_with_a_length_change_mid_stream():
    short, long = protocol.build(b"\x13\x40"), protocol.build(b"\x11", 64)
    r = protocol.FrameReader()
    r.push(short + long[:30])
    assert r.next_frame() == short
    r.length = 64
    assert r.next_frame() is None                  # nothing consumed early
    r.push(long[30:])
    assert r.next_frame() == long


def test_reader_resyncs_after_stray_bytes():
    f = protocol.build(b"\x02\x76")
    r = protocol.FrameReader()
    r.push(b"\x5a\x01\x02" + f + f)               # joined mid-frame
    assert r.next_frame() == f and r.next_frame() == f


def test_reader_resyncs_short_frames_when_expecting_long():
    short, long = protocol.build(b"\x03"), protocol.build(b"\x11", 64)
    r = protocol.FrameReader(64)
    r.push(short + long)                          # brain still on 16 B
    assert r.next_frame() == short
    assert r.next_frame() == long


def test_feed_returns_whole_frames():
    f = protocol.build(b"\x0a")
    r = protocol.FrameReader()
    assert r.feed(f + f[:5]) == [f]
    assert r.feed(f[5:]) == [f]
