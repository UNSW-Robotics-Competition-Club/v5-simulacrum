"""The four web pages share one look: the same colour tokens, and a header
whose navigation marks the page you're on."""
import re

import pytest

from v5sim import brainscreen, controllerpage, dashboard, fieldcontrol, fieldview

PAGES = {"./": fieldview.PAGE, "dashboard": dashboard.PAGE,
         "controller": controllerpage.PAGE, "field": fieldcontrol.PAGE,
         "screen": brainscreen.PAGE}
TOKENS = ("--bg:#111317", "--card:#1a1d23", "--card2:#22262d", "--line:#2f343d")


@pytest.mark.parametrize("href", PAGES)
def test_every_page_has_the_shared_header_and_marks_itself(href):
    page = PAGES[href]
    nav = re.search(r"<nav[^>]*>(.*?)</nav>", page, re.S)
    assert nav, "no <nav>"
    links = re.findall(r'<a( class="on")? href="([^"]+)"', nav.group(1))
    assert [h for _, h in links] == list(PAGES)
    assert [h for on, h in links if on] == [href]
    for token in TOKENS:
        assert token in page


def test_field_view_has_no_legend():
    assert 'class="legend"' not in fieldview.PAGE
    assert ".legend" not in fieldview.PAGE


def test_brain_screen_menu_starts_hidden_and_toggles():
    page = brainscreen.PAGE
    assert re.search(r'<header id="menu" hidden>', page)
    btn = re.search(r'<button id="menubtn"[^>]*>', page)
    assert btn and 'aria-controls="menu"' in btn.group(0) and 'aria-expanded="false"' in btn.group(0)
    assert "menu.hidden = !menu.hidden" in page


def test_controller_page_has_every_button_and_never_sends_competition():
    from v5sim.smart_controller import BUTTONS
    page = controllerpage.PAGE
    assert set(re.findall(r'data-btn="(\w+)"', page)) == set(BUTTONS)
    assert 'id="lcd"' in page and 'id="rumble"' in page
    assert "competition" not in page.split("<script>", 1)[1].split("JSON.stringify", 1)[1].split(")", 1)[0]
