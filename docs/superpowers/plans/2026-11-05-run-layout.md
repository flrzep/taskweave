# Run layout implementation plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** The Run tab becomes a band (the robot, a condensed bubble, the 3D window) over a pan-and-zoom canvas of
the whole mission; mission control lives on Run only; the system strip slims to one 40px line.

**Architecture:** HMI only, no build step: `index.html` restructures the strip and `#view-run`, `app.js`
`renderMission` and `renderBoard` fill the slim strip and the bubble, `graph-run.js` replaces its scrolling strip
with a transformed layer under pointer and wheel handlers (`canvas` state: x, y, scale, follow), `graph-core.js`
gets the Run canvas metrics, `style.css` the band, the window and the canvas. Behaviour is checked in the browser
against the mock (`adient-uc1-frame/plans/hmi-session/mock/mock.py`), structure by text tests in
`taskweave_hmi/test`.

**Tech Stack:** vanilla JS (classic scripts, `el` from app.js), CSS custom properties, Python text tests (pytest),
the mock server, the built-in browser.

**Spec:** `docs/superpowers/specs/2026-11-05-run-layout.md` (the flow look it builds on:
`docs/superpowers/specs/2026-11-04-flow-look.md`).

## Global Constraints

- Branch `hmi-run-layout` in the worktree `../tw-worktrees/hmi-run-layout`, cut from `main` (rebased on it before
  review); the mock on port 8814 (launch config `hmi-run-layout`, the same pattern as `hmi-flow-minors`).
- No cell or robot words anywhere, comments and example ids included (`TASKWEAVE_STRICT_AGNOSTIC=1 python -m pytest
  taskweave_core/test/test_agnostic.py -q` must pass); neutral examples only (`to_shelf`, `Night round`).
- Every id app.js or graph-run.js reads with `$()` exists in index.html (Task 1's test); every script index.html
  loads is in `STATIC_FILES` (unchanged here: no new script).
- Sizes from the spec: `--strip: 40px`; the band `--band: 200px` (180px from 600 to 1000px, about 150px on a phone);
  the robot 120px (80px on a phone); the window a 96px thumbnail on a phone; zoom 0.4 to 1.5; follow moves in 300ms
  (none with `prefers-reduced-motion`); buttons 48px, touch targets 44px.
- The text rule: the bubble shows the kind word, the node name, the main argument and the next node's name, never the
  instruction sentence (that is the bubble's `title` and the pop-up's text).
- Commits end with `Co-Authored-By: Claude Fable 5.1 <noreply@anthropic.com>`; stage files by name.
- Before review: ruff clean, the HMI suite in frame-ros (`adient-uc1-frame/plans/hmi-session/twtest.sh <worktree> 95
  taskweave_hmi`), the agnostic test, and a browser pass over every mock scenario at desktop and phone width.

## Review Focus

- A one-step plan: the bubble's third line reads `then done` (`then again` with `loop: forever`), never `next  ` with
  nothing after it (Task 3's `test_bubble_next_words`).
- A mission wider than the canvas at the 0.4 floor: Fit stops at 0.4 and the operator can still pan to its end
  (Task 4's browser check with the `rules` scenario at phone width).
- A tap on a node after a tiny finger movement must open its pop-up, not count as a pan (Task 4: the 4px drag
  threshold, checked with a 2px pointer move in the browser).
- Status with a step but no plan yet (planning): the bubble says `Planning` and the canvas keeps the last plan or
  "No plan yet" without an error in the console (Task 3's browser check on the `planning` scenario Task 5 adds).
- A window resize while follow is off keeps the operator's view; with follow on it recentres (Task 4's browser check).

---

### Task 1: Worktree, launch config and the page-ids test

**Files:**
- Create: `taskweave_hmi/test/test_page_ids.py`
- Modify: `.claude/launch.json` (add `hmi-run-layout`, port 8814, the worktree's `web` dir)

**Interfaces:**
- Produces: `page_ids() -> set[str]` and `used_ids(path) -> set[str]` in the test module (text readers like
  `test_static_files.static_files()`), used by Task 2 and 3's tests.

- [ ] **Step 1: Cut the worktree**

```bash
git worktree add ../tw-worktrees/hmi-run-layout -b hmi-run-layout main
```

- [ ] **Step 2: Write the test**

```python
"""Every element id app.js and graph-run.js read with $('…') exists in index.html (the page is restructured by hand:
a renamed id fails at run time, in the browser only)."""
import re
from pathlib import Path

WEB = Path(__file__).resolve().parents[1] / 'taskweave_hmi' / 'web'
SCRIPTS = ['app.js', 'graph-run.js', 'editor.js', 'record.js', 'teach.js', 'robot.js', 'held.js']

def page_ids() -> set[str]:
    return set(re.findall(r'\bid="([^"]+)"', (WEB / 'index.html').read_text(encoding='utf-8')))

def used_ids(name: str) -> set[str]:
    return set(re.findall(r"\$\('([\w-]+)'\)", (WEB / name).read_text(encoding='utf-8')))

def test_every_id_a_script_reads_is_on_the_page():
    missing = {name: sorted(used_ids(name) - page_ids()) for name in SCRIPTS}
    assert not any(missing.values()), missing
```

- [ ] **Step 3: Run it**

Run: `cd ../tw-worktrees/hmi-run-layout && python -m pytest taskweave_hmi/test/test_page_ids.py -q`
Expected: PASS (the page is consistent today). If an id is reported, a script builds it dynamically: add that id to
the page or exclude it with a comment, whichever is true.

- [ ] **Step 4: Add the launch config and commit**

```bash
git add taskweave_hmi/test/test_page_ids.py
git commit -m "HMI test: every id a script reads exists on the page"
```
(`.claude/launch.json` is untracked: edit it in the main checkout, do not commit it.)

---

### Task 2: The slim strip

**Files:**
- Modify: `taskweave_hmi/taskweave_hmi/web/index.html:36-80` (the strip)
- Modify: `taskweave_hmi/taskweave_hmi/web/app.js:309-350` (`renderMission`)
- Modify: `taskweave_hmi/taskweave_hmi/web/style.css:44` (`--strip`), `:102-158` (the strip's rules)
- Test: `taskweave_hmi/test/test_run_layout.py` (new)

**Interfaces:**
- Produces: strip ids `conn`, `robot`, `state`, `strip-mission` (new: the mission's name), `strip-abort`, `current`,
  `progress-bar`, `speed*`, `pad-mode`, `start-msg`, `last-error*`. Removed: `mission-run`, `goal`, `cycle`,
  `step-meta`, `step-instruction`, `step-detail`, `resources` (Task 3 puts `resources` in the band).
- `renderMission(s) -> boolean` keeps its signature and return (active).

- [ ] **Step 1: Write the failing test**

```python
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
```

- [ ] **Step 2: Run it**

Run: `python -m pytest taskweave_hmi/test/test_run_layout.py -q`
Expected: FAIL on `--strip: 40px` (it is 52px) and on the ids.

- [ ] **Step 3: Restructure the strip in index.html**

Inside `.mission-row`, in order: `.strip-robot` (as is), `#state`, `<b id="strip-mission" class="strip-mission"
hidden></b>`, `#strip-abort` (glyph only: drop `.strip-abort-word`), `#speed`, `.sys` with `#pad-mode`. Keep
`#current`/`#progress-bar` as the line along the bottom edge, outside the row. Delete `#mission-run` and `#resources`.
Rewrite the comment above the strip to the spec's table. Keep `#speed-panel`, `#speed-msg`, `#start-msg`,
`#last-error`.

- [ ] **Step 4: Update renderMission in app.js**

Drop every write to the removed ids and to the resource chips. Add: `$('strip-mission').textContent =
humanise(missionOf(state.snapshot.plan) || s.plan_id.replace(/-\d+$/, ''))` while active (guard `typeof humanise ===
'function'`: graph-core.js loads after app.js, as the phases did with `bandsOf`), `show($('strip-mission'),
active)`. Progress: `show($('current'), active && !!s.step)` and the width as today. `showAbortArmed` no longer sets
`.strip-abort-word`; it keeps the `aria-label` and `confirm` class.

- [ ] **Step 5: CSS**

`--strip: 40px`; `.mission` padding `0 var(--sp-4)` (plus the safe-area inset on a phone), `min-height: var(--strip)`,
one row, `gap: var(--sp-3)`; `.strip-mission { flex: 1 1 auto; min-width: 0; font: 600 var(--fs-s) var(--sans);
white-space: nowrap; overflow: hidden; text-overflow: ellipsis; }`; `body.on-run .strip-mission { display: none; }`
(the band names it); `#state` padding `5px 8px`; the `.strip-abort` square `var(--tap-s)` at every width. Delete the
rules for `.mission-run`, `.mission-goal`, `.mission-meta`, `.mission-step`, `#step-instruction`, `#step-detail`,
`.resources` (Task 3 re-adds `.resources` under the band's robot), and the phone rules that mention them.

- [ ] **Step 6: Run the tests and the browser**

Run: `python -m pytest taskweave_hmi/test/test_run_layout.py taskweave_hmi/test/test_page_ids.py -q`
Expected: PASS.
Browser (mock scenario `running`, Program tab): the strip is 40px (`getComputedStyle($('mission')).height` is
`40px`), shows `Night round`-style mission name, the state badge, the stop glyph, Speed; no step sentence; the lime
progress line along its bottom. On the Run tab the name is hidden. Console: no errors.

- [ ] **Step 7: Commit**

```bash
git add taskweave_hmi/taskweave_hmi/web/index.html taskweave_hmi/taskweave_hmi/web/app.js taskweave_hmi/taskweave_hmi/web/style.css taskweave_hmi/test/test_run_layout.py
git commit -m "HMI strip: one 40px line, the mission's name and progress, no step words"
```

---

### Task 3: The band: robot, bubble, window

**Files:**
- Modify: `taskweave_hmi/taskweave_hmi/web/index.html:93-166` (`#view-run`)
- Modify: `taskweave_hmi/taskweave_hmi/web/app.js:219-300` (`renderRun`, `renderRecent`), `:384-470`
  (`renderBoard`, `renderPhases`), `:162-167` (`startRunViewer`)
- Modify: `taskweave_hmi/taskweave_hmi/web/style.css:160-305` (the board), `:370-392` (the run grid)
- Test: `taskweave_hmi/test/test_run_layout.py`

**Interfaces:**
- Consumes: `flowName(node)`, `humanise(s)` (graph-core.js), `stepTitle(step)`, `mainArg(step)`, `loopText(plan,
  mission)`, `missionCache` (graph-run.js); all load after app.js, so `renderBoard` runs again on `window` `load`
  (replace the phases' hook) and guards with `typeof`.
- Produces: `#view-run` = `<section id="band" class="band">` + `<section id="canvas" class="canvas">`. Band ids:
  `board` (the bubble's state class stays on it: `state-running` and so on), `board-kind` (new, line 1's kind
  word), `board-instruction` (line 1's name), `board-detail` (line 2, mono), `board-next` (new, line 3),
  `board-step`, `board-total`, `board-cycle`, `board-error`, `board-act`, `mission-start` (`mission-select`,
  `btn-start`), `mission-controls` (`btn-pause`, `btn-abort`), `resources` (the three dots under the robot),
  `viewer3d-run`, `window-enlarge` (new button). Removed: `board-goal`, `board-phases`, `recent`, `recent-title`.
  `#canvas` holds the toolbar and `#plan-graph` / `#plan` (Task 4 fills it; keep `plan-view` and `plan-graph` here
  so the page works between the tasks).
- `bubbleWords(s, plan, mission, index) -> {kind, name, detail, next}` in app.js, pure, for the running and paused
  states (the test below reads its words through the browser; the Python test checks the markup).

- [ ] **Step 1: Extend the test**

```python
def test_run_tab_is_a_band_over_a_canvas():
    ids = page_ids()
    assert {'band', 'canvas', 'board-kind', 'board-instruction', 'board-detail', 'board-next', 'board-act',
            'mission-select', 'btn-start', 'btn-pause', 'btn-abort', 'resources', 'viewer3d-run', 'window-enlarge'} <= ids
    assert not {'board-goal', 'board-phases', 'recent', 'recent-title'} & ids
    run = PAGE[PAGE.index('id="view-run"'):PAGE.index('id="view-missions"')]
    assert run.index('id="band"') < run.index('id="canvas"')
    assert re.search(r'--band:\s*200px', CSS)
```

- [ ] **Step 2: Run it**

Run: `python -m pytest taskweave_hmi/test/test_run_layout.py -q`
Expected: FAIL on the ids.

- [ ] **Step 3: The markup**

`#view-run` (class `view run`): `<section id="band" class="band">` with `<div class="band-robot">` (the `.bot` SVG
moved here unchanged, then `<span id="resources" class="resources">` with the three `data-res` dots), `<div
id="board" class="bubble state-idle">` with: `<div class="bubble-chips">` (`#board-step` + `#board-total`,
`#board-cycle`), line 1 `<div class="bubble-name"><span id="board-kind"></span><b id="board-instruction"
aria-live="polite"></b></div>`, `<div id="board-detail" class="mono"></div>`, `<div id="board-next"
class="muted"></div>`, `<div id="board-error" role="alert" hidden></div>`, `#board-state` (sr-only), the controls
row: `#mission-start` (select + Start), `#board-act`, `#mission-controls` (Pause, Abort); then `<div
class="window"><div id="viewer3d-run" class="viewer3d" …></div><button id="window-enlarge" class="icon window-enlarge"
aria-pressed="false" aria-label="Enlarge the 3D view"></button></div>`. Then `<section id="canvas" class="canvas">`
holding the existing `#plan-view` segmented control, `#plan-graph` and `#plan`. Delete the Recent steps card and the
`.run-side` wrapper. Rewrite the Run comments (the board's, the strip's) to describe the band.

- [ ] **Step 4: renderBoard from the spec's table**

Write `bubbleWords(s, plan, mission, index)` returning the four strings for an active status with `s.step`:
`kind = humanise(stepTitle(s.step))` (`Navigate to`), `name = flowName({ kind: 'step', step: s.step })`, `detail`
= the first field of `stepFields(s.step)` joined `key  value` (`to  bin_1/place`, empty when none), `next` = the
next step's `flowName` as `next  Grip close`, `then done` at the last step, `then again` when `loopText(plan,
mission)` is not empty. Then fill the other rows of the table: idle `Which mission?` (only the picker and Start
visible), planning `Planning` + the mission name, paused as running with `detail` kept and `next` = `paused`,
failed: `name` = the stopped step's name in coral (class `bad`), `detail` = `s.last_error` (one line, `title` holds
it all), `next` = the rule (`failureText(rules(plan, mission)[index])` from graph-run.js: `retry ×2`, `replan from Pick`), succeeded
`Done` + mission name + `×n` when `s.cycle > 1`, aborted `Stopped` + the stopped step's name. The chips, the act
button (`Start`, `Start at step N`, `Resume`, `Retry step N`, `Run again`) and `board.act` logic stay as today; the
bubble's `title` is `s.step?.instruction || ''`. Delete `renderPhases`, `board.phases`, `--p`, `renderRecent`,
`RECENT`, `recentKey` and the `window` `load` hook (replace it with one that calls `renderRun()` once so the names
come from graph-core). `renderMission` writes the resource dots (`#resources [data-res]` `.on`) as it did for the
strip.

- [ ] **Step 5: CSS**

`--band: 200px` on `:root`; `.run { display: flex; flex-direction: column; height: calc(100dvh - var(--top-h, 40px));
min-height: 0; }`, `body.on-run main { max-width: none; padding: 0; }`; `.band { flex: none; height: var(--band);
display: grid; grid-template-columns: auto minmax(0, 1fr) minmax(0, 1fr); gap: var(--sp-4); padding: var(--sp-3)
var(--sp-4); border-bottom: 1px solid var(--border); background: var(--surface-card); }`; `.band-robot` 120px tall
`.bot` (static position, the animations and the state classes moved from `.board` to `.band` via `#board`'s state
class: keep the selectors by adding `.bubble.state-*` where `.board.state-*` was); `.bubble` = the card with the
speech tail on its left edge (today's `.board-say::after` turned to the left), `.bubble-name` 20px 600 with
`#board-kind` in the kind colour (classes `kind-drive` … from `colourOf`, set by renderBoard); `.window { position:
relative; border-radius: var(--r-m); border: 1px solid var(--border); box-shadow: inset 0 0 0 2px var(--surface-page);
}` with `.bubble.state-running ~ .window { border-color: var(--run-fill); }`; `.window .viewer3d { aspect-ratio:
auto; height: 100%; }`; `.window.enlarged { position: fixed; inset: calc(var(--top-h, 40px) + var(--band)) 0 0 0;
z-index: 5; }`; breakpoints: `@media (max-width: 1000px) { :root { --band: 180px; } .band { grid-template-columns:
auto minmax(0, 3fr) minmax(0, 2fr); } }`, `@media (max-width: 600px) { :root { --band: 150px; } .bot 80px; .window {
width: 96px; height: 96px; } .run { height: calc(100dvh - var(--top-h, 40px) - var(--nav)); } }`. Delete the
`.board-stage`, `.board-phases`, `.run-grid`, `.run-side`, `.recent` rules.

- [ ] **Step 6: The window**

`#window-enlarge` toggles `.enlarged` on `.window`, `aria-pressed`, and calls `runViewer?.resize()`; Escape closes
it (a `keydown` listener on `document`, only while enlarged); on a phone the button covers the thumbnail, so a
tap on the window is the same control. `startRunViewer` unchanged.

- [ ] **Step 7: Tests and the browser**

Run: `python -m pytest taskweave_hmi/test/test_run_layout.py taskweave_hmi/test/test_page_ids.py -q`
Expected: PASS.
Browser, mock scenarios `idle`, `running`, `paused`, `failed` (desktop and phone): the band is 200px / 150px;
`running` reads e.g. `Grip · Grip close` / `hands  right` / `next  Lift`; `failed` shows the error line in coral and
the rule; `idle` shows the picker with Start; Enlarge fills the page below the band, Escape closes; no console error;
`$('board').title` holds the instruction sentence.

- [ ] **Step 8: Commit**

```bash
git add taskweave_hmi/taskweave_hmi/web/index.html taskweave_hmi/taskweave_hmi/web/app.js taskweave_hmi/taskweave_hmi/web/style.css taskweave_hmi/test/test_run_layout.py
git commit -m "HMI Run band: the robot, a condensed bubble and the 3D window over the plan"
```

---

### Task 4: The canvas: pan, zoom, follow, fit

**Files:**
- Modify: `taskweave_hmi/taskweave_hmi/web/graph-core.js:28-43` (`GRAPH_METRICS`)
- Modify: `taskweave_hmi/taskweave_hmi/web/graph-run.js:347-475` (`renderGraph`, the strip code, `setPlanView`)
- Modify: `taskweave_hmi/taskweave_hmi/web/index.html` (the `#canvas` toolbar)
- Modify: `taskweave_hmi/taskweave_hmi/web/style.css:846-860` (`.pg`, `.graph-box`)
- Test: `taskweave_hmi/test/test_run_layout.py`

**Interfaces:**
- Consumes: `layoutGraph(tree, ctx) -> {items, owner, width, height, lanes}`, `drawGraph(layout, ctx) -> {node,
  items, height, replans, flow}` (graph-core.js, unchanged).
- Produces: `GRAPH_METRICS.canvas` (the editor's sizes: node 220×64, head 24, line 38, end 88×36, gapX 64, pad 16,
  tail 0, `fields: true, ports: false, rules: false, kindWord: false`) and `GRAPH_METRICS.canvasPhone` (168×52 …,
  `fields: false`); in graph-run.js `const canvas = { x: 0, y: 0, scale: 1, follow: true, layer: null }`,
  `canvasFit(layout)`, `canvasCentre(x, y)`, `canvasApply(animate)` (sets `transform: translate(x, y) scale(s)` on
  `.pg-layer`, the 300ms transition only when `animate` and motion is allowed), `placePop()`.
- Toolbar ids: `canvas-fit`, `canvas-follow` (`aria-pressed`), the existing `plan-view` segmented (`graph` / `list`).
  Follow is one permanent toggle (pressed while on) rather than a chip that comes and goes: one control in one place.

- [ ] **Step 1: Extend the test**

```python
def test_canvas_has_its_tools_and_metrics():
    assert {'canvas-fit', 'canvas-follow', 'plan-view', 'plan-graph', 'plan'} <= page_ids()
    core = (WEB / 'graph-core.js').read_text(encoding='utf-8')
    assert re.search(r'\bcanvas:\s*\{[^}]*ports: false', core, re.S) and 'canvasPhone:' in core
    run = (WEB / 'graph-run.js').read_text(encoding='utf-8')
    assert 'pg-layer' in run and 'scrollLeft' not in run   # the strip and its scroll are gone
    assert '.pg-layer' in CSS and 'transform-origin: 0 0' in CSS
```

- [ ] **Step 2: Run it**

Run: `python -m pytest taskweave_hmi/test/test_run_layout.py -q`
Expected: FAIL (no `canvas-fit`, no `canvas:` metrics, `scrollLeft` present).

- [ ] **Step 3: Metrics and markup**

Add `canvas` and `canvasPhone` to `GRAPH_METRICS` as in Interfaces (copy `editor` / `editorPhone` sizes, drop
`terminal`, `rule`, `rowGap`, `laneGap`; keep `laneRowGap`). In `#canvas`: `<div class="canvas-tools">` with
`#plan-view` (as is), `<button id="canvas-fit" class="btn small">Fit</button>`, `<button id="canvas-follow"
class="btn small" aria-pressed="true">Follow</button>`; then `#plan-graph` (class `graph-box canvas-box`) and
`#plan`.

- [ ] **Step 4: renderGraph on a layer**

Replace the `.pg` strip: `wrap = div('pg-canvas')` (the viewport, `overflow: hidden`, `touch-action: none`) holding
`layer = div('pg-layer', drawn.node)`; metrics `PHONE.matches ? GRAPH_METRICS.canvasPhone : GRAPH_METRICS.canvas`;
`ctx.width` = 0 (the layout's own width; nothing is stretched). Blocks open by default: `isOpen` = `stored[key] ??
true` (the running frame always open as today). Delete the scroll bookkeeping (`view.setScroll`, `userScrolled`,
`centred*`, `popShown` and the strip scroll). Keep: the sheet's scrollTop, the refocus, the legend. On the first
draw of a plan (`canvas.planId !== plan.plan_id`): `canvasFit(layout)` then, when a live node exists, `canvas.follow
= true`. When the running (else the start) step's key changes and `canvas.follow`: `canvasCentre` on its node with
`animate`. Pointer handlers on `wrap` (set once per build): `pointerdown` records the start and captures; `pointermove`
beyond 4px pans (`canvas.x += dx` …, `canvas.follow = false`, `canvasApply(false)`) and marks the gesture a drag;
`pointerup` after a drag swallows the next `click` (a capture-phase `click` listener with `stopPropagation`, once).
Two pointers: pinch zoom around their midpoint. `wheel`: `ctrlKey || metaKey` zooms around the pointer by
`Math.exp(-deltaY / 300)`, else pans by `-deltaX, -deltaY`; `preventDefault` on both. Zoom clamps `[0.4, 1.5]`;
every pan or zoom sets `canvas.follow = false` and `aria-pressed` on `#canvas-follow`. `#canvas-fit` → `canvasFit`
(and follow off); `#canvas-follow` → follow on and `canvasCentre` on the running node. The pop-up (`planCard`
desktop) is appended to `wrap`, not the layer, and `placePop()` puts it at the node's screen position (`canvas.x +
scale * (n.cx + n.nw / 2) + POP_GAP` …, the left/under fallbacks as today measured against `wrap.clientWidth`);
`canvasApply` calls `placePop()`. A `ResizeObserver` on `#plan-graph` redraws; with follow on it recentres, else it
keeps the transform.

- [ ] **Step 5: CSS**

`.canvas { flex: 1 1 auto; min-height: 0; display: flex; flex-direction: column; }`, `.canvas-tools { display:
flex; gap: var(--sp-2); justify-content: flex-end; padding: var(--sp-2) var(--sp-3); }`, `.canvas-box { flex: 1 1
auto; min-height: 0; display: block; }`, `.pg-canvas { position: relative; width: 100%; height: 100%; overflow:
hidden; touch-action: none; cursor: grab; background: the dotted grid as `.ed-canvas` draws it }`,
`.pg-canvas.dragging { cursor: grabbing; }`, `.pg-layer { position: absolute; left: 0; top: 0; transform-origin: 0
0; will-change: transform; }`, `.pg-layer.animate { transition: transform .3s ease; }` with `@media
(prefers-reduced-motion: reduce) { .pg-layer.animate { transition: none; } }`; the legend stays in `.canvas-tools`'s
row at the left. Delete the `.pg` strip rules.

- [ ] **Step 6: Tests and the browser**

Run: `python -m pytest taskweave_hmi/test/test_run_layout.py taskweave_hmi/test/test_page_ids.py -q`
Expected: PASS.
Browser (mock `running`, desktop 1400px): the whole tree is in view after load (`canvas.scale` ≤ 1, every `.fl-box`
inside `wrap`); the running node is lit; `Follow` pressed. Drag 200px left: the layer moves, `Follow` unpressed. Ctrl
+ wheel: scale changes, clamped at 0.4 and 1.5. Click a node after a 2px move: the pop-up opens beside it and stays
beside it after a drag. Fit: the tree fits again. Phone (375px, `rules` scenario): pinch (two pointers) zooms; a tap
opens the sheet; Fit stops at 0.4 and a drag still reaches the last node. Resize the window with follow off: the
transform is unchanged; with follow on: the running node is centred again. Console: no errors.

- [ ] **Step 7: Commit**

```bash
git add taskweave_hmi/taskweave_hmi/web/graph-core.js taskweave_hmi/taskweave_hmi/web/graph-run.js taskweave_hmi/taskweave_hmi/web/index.html taskweave_hmi/taskweave_hmi/web/style.css taskweave_hmi/test/test_run_layout.py
git commit -m "HMI Run canvas: the whole mission on a pan-and-zoom layer that follows the running node"
```

---

### Task 5: Mock scenarios, docs, checks

**Files:**
- Modify: `../adient-uc1-frame/plans/hmi-session/mock/mock.py:84-115` (`snapshot`: add `planning`, `succeeded`,
  `aborted`)
- Modify: `taskweave_hmi/taskweave_hmi/web/DESIGN.md` (the Run page and the strip sections)
- Modify: `../adient-uc1-frame/plans/hmi-session/HANDOVER.md` (a note for this round)

**Interfaces:**
- Consumes: everything above.

- [ ] **Step 1: Mock scenarios**

In `snapshot(scenario)`: `planning` = state running, `step_index` -1, `step` None, `step_status 'planning'`,
`step_count` the plan's length, `plan` None; `succeeded` = every step SUCCEEDED, state succeeded, `cycle` 3,
`step_index` -1; `aborted` = `upto` 15 and state aborted with `last_error` empty. `current.txt` selects them as today.

- [ ] **Step 2: Browser pass**

For each of `idle`, `planning`, `running`, `paused`, `failed`, `succeeded`, `aborted`, `rules` at 1400px and 375px:
the bubble's words match the spec's table, the act button is the right one, no console error; screenshots of
`running` and `failed` at both widths saved to the scratchpad for the review.

- [ ] **Step 3: DESIGN.md and HANDOVER.md**

DESIGN.md: replace the Run and strip paragraphs with the band, the bubble's text rule, the window, the canvas (pan,
zoom, follow, fit) and the 40px strip; the tokens `--strip` and `--band`. HANDOVER.md: the branch, the spec, the
mock scenarios added, what the review should look at (the bubble's words per state, pan and zoom on touch).

- [ ] **Step 4: The checks**

Run, in the worktree: `ruff check .`; `TASKWEAVE_STRICT_AGNOSTIC=1 python -m pytest
taskweave_core/test/test_agnostic.py -q`; `python -m pytest taskweave_hmi/test/test_run_layout.py
taskweave_hmi/test/test_page_ids.py taskweave_hmi/test/test_static_files.py -q`; then the HMI suite in frame-ros:
`bash ../adient-uc1-frame/plans/hmi-session/twtest.sh ../tw-worktrees/hmi-run-layout 95 taskweave_hmi`.
Expected: all green (the two tests known to fail only on Windows pass in frame-ros).

- [ ] **Step 5: Commit, rebase, hand over**

```bash
git add taskweave_hmi/taskweave_hmi/web/DESIGN.md
git commit -m "HMI DESIGN.md: the Run band, the canvas and the slim strip"
git rebase main
```
Then send the branch to the taskweave core session for review (the usual message: branch, commits, spec, how to
look at it on 8814), and wait: no merge, no push, no deploy from here.
