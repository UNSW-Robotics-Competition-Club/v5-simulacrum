import json
import socket
import urllib.request

import pytest

from v5sim.fieldview import TRAIL_MIN_STEP_M, FieldView


@pytest.fixture
def view():
    with socket.socket() as s:
        s.bind(("127.0.0.1", 0))
        port = s.getsockname()[1]
    v = FieldView(port)
    yield v, f"http://127.0.0.1:{port}"
    v.close()


def get(url):
    with urllib.request.urlopen(url, timeout=5) as r:
        return r.headers.get_content_type(), r.read()


def test_state_reflects_latest_publish(view):
    v, base = view
    v.publish(1.5, 0.25, -0.5, 1.0, 0.8)
    ctype, body = get(base + "/state.json")
    state = json.loads(body)
    assert ctype == "application/json"
    assert (state["t"], state["x"], state["y"], state["heading"], state["v"]) == \
        (1.5, 0.25, -0.5, 1.0, 0.8)


def test_trail_skips_tiny_moves(view):
    v, base = view
    v.publish(0.0, 0.0, 0.0, 0.0, 0.0)
    v.publish(0.1, TRAIL_MIN_STEP_M / 10, 0.0, 0.0, 0.0)
    v.publish(0.2, 0.5, 0.0, 0.0, 0.0)
    assert json.loads(get(base + "/state.json")[1])["trail"] == [[0.0, 0.0], [0.5, 0.0]]


def test_page_and_404(view):
    _, base = view
    ctype, body = get(base + "/")
    assert ctype == "text/html" and b"<canvas" in body
    with pytest.raises(urllib.error.HTTPError):
        get(base + "/nope")


def test_serves_override_field_image(view):
    _, base = view
    ctype, body = get(base + "/override-field.png")
    assert ctype == "image/png" and body.startswith(b"\x89PNG")


def post(url, body, ctype):
    req = urllib.request.Request(url, data=body, method="POST",
                                 headers={"Content-Type": ctype})
    with urllib.request.urlopen(req, timeout=5) as r:
        return r.status


def test_input_requires_json_content_type():
    # text/plain is a "simple" cross-site request a web page can send with
    # no CORS preflight; demanding application/json blocks that.
    got = []
    with socket.socket() as s:
        s.bind(("127.0.0.1", 0))
        port = s.getsockname()[1]
    v = FieldView(port, on_input=got.append)
    try:
        url, body = f"http://127.0.0.1:{port}/input", b'{"axes": {}}'
        with pytest.raises(urllib.error.HTTPError) as e:
            post(url, body, "text/plain")
        assert e.value.code == 415 and got == []
        assert post(url, body, "application/json") == 204 and got == [{"axes": {}}]
    finally:
        v.close()


def test_state_carries_blocks_and_held(view):
    v, base = view
    v.publish(0.0, 0.0, 0.0, 0.0, 0.0, blocks=[(0.5, -0.25, "red")], held=["blue"])
    state = json.loads(get(base + "/state.json")[1])
    assert state["blocks"] == [[0.5, -0.25, "red"]] and state["held"] == ["blue"]
    v.publish(0.1, 0.0, 0.0, 0.0, 0.0)
    assert json.loads(get(base + "/state.json")[1])["blocks"] == []


def test_page_draws_override_and_reads_gamepads(view):
    _, base = view
    _, body = get(base + "/")
    page = body.decode()
    assert 'id="override"' in page and "drawOverride(" in page
    assert "'override-field.png'" in page
    assert "navigator.getGamepads" in page
    # every V5 button the gamepad maps to is one the controller sim accepts
    from v5sim.smart_controller import BUTTONS
    import re
    mapped = re.search(r"PAD_BUTTONS = \{(.*?)\}", page, re.S).group(1)
    assert set(re.findall(r"'(\w+)'", mapped)) <= set(BUTTONS)


def test_field_control_page_and_state(view):
    v, base = view
    ctype, page = get(base + "/field")
    assert ctype == "text/html" and b"field.json" in page and b"Field Control" in page
    v.publish_field(2.0, {"competition": "DISABLED", "match": None})
    ctype, body = get(base + "/field.json")
    assert ctype == "application/json"
    assert json.loads(body) == {"t": 2.0, "competition": "DISABLED", "match": None}


def test_state_carries_the_controller(view):
    v, base = view
    v.publish(0.1, 0.0, 0.0, 0.0, 0.0, screen=["Auton: Left"],
              controller={"connected": True, "rumble": "-.-", "axes": {"Axis3": 50},
                          "buttons": ["A"], "competition": "NONE"})
    state = json.loads(get(base + "/state.json")[1])
    assert state["screen"] == ["Auton: Left"]
    assert state["controller"] == {"connected": True, "rumble": "-.-", "axes": {"Axis3": 50},
                                   "buttons": ["A"], "competition": "NONE"}
    v.publish(0.2, 0.0, 0.0, 0.0, 0.0)
    assert json.loads(get(base + "/state.json")[1])["controller"] is None


def test_controller_page_is_served(view):
    _, base = view
    ctype, page = get(base + "/controller")
    assert ctype == "text/html" and b'id="lcd"' in page
