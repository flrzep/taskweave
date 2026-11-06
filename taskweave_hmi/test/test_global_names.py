"""No top-level name is declared in two of the page's classic scripts. They share one global scope, so a second
`const x` or `function x` throws a SyntaxError and the later script does not run at all (the browser shows it, the
mock tests do not)."""
import re
from pathlib import Path

WEB = Path(__file__).resolve().parents[1] / 'taskweave_hmi' / 'web'
DECLARED = re.compile(r'^(?:async\s+)?(?:function\*?|const|let|class|var)\s+([A-Za-z_$][\w$]*)', re.M)

def classic_scripts() -> list[str]:
    html = (WEB / 'index.html').read_text(encoding='utf-8')
    return [m.group(1) for m in re.finditer(r'<script\b(?![^>]*type="module")[^>]*\bsrc="([^"]+)"', html)]

def test_the_page_loads_classic_scripts():
    assert 'app.js' in classic_scripts()

def test_no_top_level_name_is_declared_in_two_scripts():
    where: dict[str, list[str]] = {}
    for name in classic_scripts():
        for found in DECLARED.finditer((WEB / name).read_text(encoding='utf-8')):
            where.setdefault(found.group(1), []).append(name)
    assert not {k: v for k, v in where.items() if len(v) > 1}
