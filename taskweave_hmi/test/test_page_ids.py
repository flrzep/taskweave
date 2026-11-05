"""Every element id app.js and graph-run.js read with $('…') exists in index.html (the page is restructured by hand:
a renamed id fails at run time, in the browser only)."""
import re
from pathlib import Path

WEB = Path(__file__).resolve().parents[1] / 'taskweave_hmi' / 'web'
SCRIPTS = ['app.js', 'graph-run.js', 'editor.js', 'record.js', 'teach.js', 'robot.js', 'held.js']

def page_ids() -> set[str]:
    return set(re.findall(r'\bid="([^"]+)"', (WEB / 'index.html').read_text(encoding='utf-8')))

BUILT_AT_RUN_TIME = {'arm-new'}  # teach.js builds this box itself and then looks it up with $()

def used_ids(name: str) -> set[str]:
    ids = set(re.findall(r"\$\('([\w-]+)'\)", (WEB / name).read_text(encoding='utf-8')))
    return ids - BUILT_AT_RUN_TIME

def test_every_id_a_script_reads_is_on_the_page():
    missing = {name: sorted(used_ids(name) - page_ids()) for name in SCRIPTS}
    assert not any(missing.values()), missing
