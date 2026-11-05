"""The Run layout (spec 2026-11-05-run-layout.md, and the tune of 5 Nov 2026: a 16:10 window sets the band's height,
the robot walks the phases floor, nodes drag): the strip is one 40px line without the step's words, the Run tab is a
band over a canvas."""
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

def test_run_tab_is_a_band_over_a_canvas():
    ids = page_ids()
    assert {'band', 'canvas', 'board-kind', 'board-instruction', 'board-detail', 'board-next', 'board-act',
            'mission-select', 'btn-start', 'btn-pause', 'btn-abort', 'resources', 'viewer3d-run',
            'window-enlarge', 'board-phases'} <= ids
    assert not {'board-goal', 'recent', 'recent-title'} & ids
    run = PAGE[PAGE.index('id="view-run"'):PAGE.index('id="view-missions"')]
    assert run.index('id="band"') < run.index('id="canvas"')
    assert run.index('id="board"') < run.index('class="band-stage"') < run.index('class="window"')

def test_window_sets_the_band_height_and_the_robot_walks_the_floor():
    window = re.search(r'\.window \{[^}]*\}', CSS, re.S).group(0)
    assert 'aspect-ratio: 16 / 10' in window and 'max-height: 42dvh' in window
    assert not re.search(r'\.band \{[^}]*\bheight: var\(--band\)', CSS, re.S)   # the band takes the window's height
    app = (WEB / 'app.js').read_text(encoding='utf-8')
    assert "setProperty('--band'" in app and "setProperty('--p'" in app
    assert 'function renderPhases' in app and 'function phasesOf' in app
    assert '.board-phases .ph-bar' in CSS and 'calc(var(--p) * (100% - var(--bot-w)))' in CSS

def test_canvas_has_its_tools_and_metrics():
    assert {'canvas-fit', 'canvas-follow', 'canvas-reset', 'plan-view', 'plan-graph', 'plan'} <= page_ids()
    core = (WEB / 'graph-core.js').read_text(encoding='utf-8')
    assert re.search(r'\bcanvas:\s*\{[^}]*ports: false', core, re.S) and 'canvasPhone:' in core
    run = (WEB / 'graph-run.js').read_text(encoding='utf-8')
    assert 'pg-layer' in run and 'scrollLeft' not in run   # the strip and its scroll are gone
    assert '.pg-layer' in CSS and 'transform-origin: 0 0' in CSS

def test_nodes_drag_and_the_wheel_zooms():
    run = (WEB / 'graph-run.js').read_text(encoding='utf-8')
    assert "'tw-plan-places'" in run and 'function movePlaces' in run and 'function dragNode' in run
    assert 'HOLD_MS' in run                                  # a finger drags after a hold
    wheel = run[run.index('function canvasWheel'):run.index('function canvasFocusIn')]
    assert 'canvasZoom' in wheel and 'canvas.x -=' not in wheel   # the wheel zooms, it does not pan
    for cls in ('pg-pop-close', 'pg-sheet-close'):           # the x: an icon button labelled Close, no Close word
        assert re.search(r"class: 'icon " + cls + r"'[^)]*'aria-label': 'Close'", run)
    assert "text: 'Close'" not in run
