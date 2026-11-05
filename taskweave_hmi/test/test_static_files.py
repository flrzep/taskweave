"""The server serves only the files it names (STATIC_FILES, server.py); every script and stylesheet index.html loads
must be among them, or the page is served without it (the mock serves any file, so only this test sees it)."""

import re
from pathlib import Path

WEB = Path(__file__).resolve().parents[1] / 'taskweave_hmi' / 'web'
SERVER = Path(__file__).resolve().parents[1] / 'taskweave_hmi' / 'server.py'


def static_files() -> set[str]:
    """STATIC_FILES as written in server.py, read as text: the module needs rclpy."""
    text = SERVER.read_text(encoding='utf-8')
    tuple_text = re.search(r'STATIC_FILES = \((.*?)\)\n', text, re.S).group(1)
    return set(re.findall(r"'([^']+)'", tuple_text))


def test_every_script_and_stylesheet_of_the_page_is_served():
    page = (WEB / 'index.html').read_text(encoding='utf-8')
    loaded = set(re.findall(r'<script[^>]+src="([^"/]+)"', page))
    loaded |= set(re.findall(r'<link[^>]+href="([^"/]+\.css)"', page))
    assert loaded, 'index.html loads nothing?'
    missing = sorted(loaded - static_files())
    assert not missing, f'index.html loads {missing}, which STATIC_FILES does not serve'
    gone = sorted(name for name in static_files() if name.endswith(('.js', '.css')) and not (WEB / name).is_file())
    assert not gone, f'STATIC_FILES names {gone}, which web/ does not have'
