"""The Run layout (spec 2026-11-05-run-layout.md): the strip is one 40px line without the step's words, the Run tab is
a band over a canvas."""
import re
from pathlib import Path
from test_page_ids import page_ids

WEB = Path(__file__).resolve().parents[1] / 'taskweave_hmi' / 'web'
CSS = (WEB / 'style.css').read_text(encoding='utf-8')
PAGE = (WEB / 'index.html').read_text(encoding='utf-8')

def test_strip_is_one_slim_line():
    assert re.search(r'--strip:\s*40px', CSS)
    assert {'strip-mission', 'strip-abort', 'current', 'progress-bar'} <= page_ids()
    assert not {'mission-run', 'step-instruction', 'step-detail', 'step-meta', 'goal', 'cycle'} & page_ids()
