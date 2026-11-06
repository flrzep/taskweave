# Editor Canvas Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** The Program tab becomes a pan-and-zoom canvas like Run with the panel floating over it, nodes show the kind word, the instruction sentence and their settings (editable on Program), both tabs lay nodes out in rows by type of movement, Run loses its band (the state and the robot go to the strip, the 3D window and the mission dock float over the graph) and gets trackpad panning, and the whole HMI takes the denser technical look.

**Architecture:** One shared layout (graph-core.js) gains rows and data-driven node heights; the camera, gestures, places and node drag move out of graph-run.js into a new graph-canvas.js that both tabs drive with render hooks; editor.js swaps its scrolling box for that viewport and renders controls into the nodes; style.css gets the new tokens and density rules. The mission files, the ops, the server routes and the planner stay as they are.

**Tech Stack:** vanilla JS (classic scripts, one global scope: no top-level name twice), CSS custom properties, the stdlib HMI server, pytest text tests, the mock server `adient-uc1-frame/plans/hmi-session/mock/mock.py` for browser checks.

**Spec:** `docs/superpowers/specs/2026-11-06-editor-canvas.md`

## Global Constraints

- Classic scripts share one global scope: every new top-level name is unique across the page's scripts (`taskweave_hmi/test/test_global_names.py` guards it); a new script goes into `STATIC_FILES` in `taskweave_hmi/taskweave_hmi/server.py` and into `SCRIPTS` of `test_page_ids.py`.
- No cell or robot words anywhere in web files, comments and example ids included (`TASKWEAVE_STRICT_AGNOSTIC=1 python -m pytest taskweave_core/test/test_agnostic.py`); neutral examples (`s2_s7`, `to_shelf`).
- Node heights come from the step's data (a row count), never from DOM measurement; Run and Program lay out the same tree the same way.
- Row order, fixed: `drive, arm, grip, wait, other` (spec section 3); row gap 24px (16 on a phone).
- Type scale (spec section 8): `--fs-xs` 11, `--fs-sm` 12, `--fs-s` 13, `--fs-base` 14, `--fs-md` 15, `--fs-lg` 17, `--fs-xl` 20, `--fs-2xl` 24; nothing under 11px; radius `--r-m` 4px, `--r-s` 3px; controls 32px on a fine pointer, 44px under `(pointer: coarse)`.
- Contrast: text 4.5:1, tracks and frames 3:1 (the 6 Nov review values stay).
- The dragged places store is `tw-plan-places` (mission -> key -> [dx, dy]) for both tabs; the window's width store is `tw-window-width`.
- Every node's header: the kind word only (`Move` for every move step); the instruction sentence under it, two lines max.
- Checks before a task is done: its own tests, `test_page_ids.py`, `test_static_files.py`, `test_global_names.py`, the strict agnostic test, ruff on the tests, and a look at the mock (`python mock.py <web dir> <port>`, scenario in `mock/current.txt`) for anything drawn.
- Commit messages end with `Co-Authored-By: Claude Fable 5.1 <noreply@anthropic.com>`.

## Review Focus

1. A mission whose steps all share one row (drives only): the layout must be one row, no empty rows, Start and Done on it; pinned in Task 3 (`test_a_container_opens_only_the_rows_it_uses`).
2. A node with no arguments (a bare `wait`, a policy skill with none set): the node is header + sentence only, its height the minimum, no empty settings row; pinned in Task 2 (`test_node_height_is_data_driven`).
3. A drag released over a "+" spot that is inside the dragged box (a box dropped into itself): no reorder, the box is placed where it was released; pinned in Task 5 (`test_a_drop_into_itself_places_instead`).
4. A wheel event from a trackpad with a fractional deltaY and no deltaX (a slow two-finger scroll): pans, never zooms; a notch of 120 (some mice) zooms; pinned in Task 4 (`test_wheel_intent_rule`).
5. The 3D window resized to its limit, then the browser window made narrower: the window clamps to half the new width and never covers the dock; pinned in Task 7 (`test_window_width_is_clamped_and_stored`).

---

### Task 1: The technical look: tokens, density by input, shape

**Files:**
- Modify: `taskweave_hmi/taskweave_hmi/web/style.css` (`:root` tokens lines 28-44; `.btn`, `.btn.small`, `.chip`, `#state`, `.segmented`, `.strip`, `.bubble`, `.bubble-name`, `.canvas-tools`, `.ed-head`, `.ed-panel .field`, `.window`, `.fl-node`, `.fl-box`)
- Modify: `taskweave_hmi/taskweave_hmi/web/DESIGN.md` (Colour, Typography, Spacing, Shape and depth, Touch targets sections)
- Test: `taskweave_hmi/test/test_look.py` (new)

**Interfaces:**
- Produces: the tokens every later task uses: `--fs-*` (the scale above), `--r-s: 3px`, `--r-m: 4px`, `--tap-s: 32px` and `@media (pointer: coarse) { :root { --tap-s: 44px; --tap-min: 44px } }`, `--row-gap: 24px` (16px at `max-width: 600px`).

- [ ] **Step 1: Write the failing test**

```python
# taskweave_hmi/test/test_look.py
"""The technical look (spec 2026-11-06-editor-canvas.md section 8): the small even type scale, density by pointer,
4px shapes."""
import re
from pathlib import Path

WEB = Path(__file__).resolve().parents[1] / 'taskweave_hmi' / 'web'
CSS = (WEB / 'style.css').read_text(encoding='utf-8')

def test_type_scale():
    root = re.search(r':root \{.*?\n\}', CSS, re.S).group(0)
    for name, px in [('xs', 11), ('sm', 12), ('s', 13), ('base', 14), ('md', 15), ('lg', 17), ('xl', 20), ('2xl', 24)]:
        assert re.search(rf'--fs-{name}:\s*{px}px', root), name
    assert not re.search(r'font-size:\s*(?:[1-9]|10)px\b', CSS)   # nothing under 11px

def test_density_follows_the_pointer():
    root = re.search(r':root \{.*?\n\}', CSS, re.S).group(0)
    assert re.search(r'--tap-s:\s*32px', root) and re.search(r'--r-m:\s*4px', root) and re.search(r'--r-s:\s*3px', root)
    coarse = re.search(r'@media \(pointer: coarse\)[^{]*\{(.*?)\n\}', CSS, re.S).group(1)
    assert '--tap-s: 44px' in coarse and '--tap-min: 44px' in coarse
    assert re.search(r'--row-gap:\s*24px', root)

def test_the_bubble_is_a_quiet_panel():
    bubble = re.search(r'\n\.bubble \{[^}]*\}', CSS, re.S).group(0)
    assert 'border: 1px solid' in bubble and 'border-left: 2px solid var(--state)' in bubble
    assert 'color-mix(in srgb, var(--state) 6%' not in bubble   # no tint of the whole bubble
```

- [ ] **Step 2: Run it to see it fail**

Run: `cd taskweave_hmi && python -m pytest test/test_look.py -v`
Expected: FAIL on `test_type_scale` (`--fs-xs: 12px` today).

- [ ] **Step 3: Change the tokens and the density rules in `style.css`**

The scale and radii as the Global Constraints say; `--tap-s: 32px`, `--tap: 40px`; a new `@media (pointer: coarse) { :root { --tap-s: 44px; --tap: 48px; --tap-min: 44px; } }` right after `:root`; `--strip: 36px`; `--row-gap: 24px` (16px inside the existing `max-width: 600px` root rule). The state badge `#state`: `--fs-xs` 700 uppercase, padding `2px 6px`, a pill. `.bubble`: `border: 1px solid var(--border-strong); border-left: 2px solid var(--state); background: var(--surface-card)`; `.bubble-name` `--fs-xl` (`--fs-lg` up to 1000px). `.canvas-tools`, `.ed-head`: 36px high rows with `--sp-2` gaps. `.ed-panel .field`: labels `--fs-xs`, fields 8px apart; sections separated by a hairline and `--sp-4`. Node and box radius `--r-m`; buttons `--r-s`. Keep every colour token.

- [ ] **Step 4: Run the test and the existing suites**

Run: `cd taskweave_hmi && python -m pytest test/test_look.py test/test_run_layout.py test/test_page_ids.py test/test_static_files.py -q`
Expected: all pass (test_run_layout's 44px phone assertions still hold under `(pointer: coarse)`, which the phone also is; adjust its `--strip: 40px` assertion to 36px).

- [ ] **Step 5: Look at the mock** at 1400px and 375px, scenarios `running` and `idle`: the strip 36px, the badge a small tag, the bubble a quiet panel with the state as its left bar, buttons 32px on a mouse. Fix what reads wrong.

- [ ] **Step 6: Rewrite DESIGN.md's Colour, Typography, Spacing, Shape and Touch targets sections** to the new values (keep the direction text).

- [ ] **Step 7: Commit**

```bash
git add taskweave_hmi/taskweave_hmi/web/style.css taskweave_hmi/taskweave_hmi/web/DESIGN.md taskweave_hmi/test/test_look.py taskweave_hmi/test/test_run_layout.py
git commit -m "HMI look: the small even type scale, density by pointer, 4px shapes, the bubble a quiet panel"
```

### Task 2: The node: kind word, sentence, settings rows, data-driven height, kind icon

**Files:**
- Modify: `taskweave_hmi/taskweave_hmi/web/graph-core.js` (`flowName` ~69, `stepFields` 83, `flowHead` 105, `GRAPH_METRICS` 28-50, `nodeItem`/`measure` in `layoutGraph` 196-261, `drawGraph` 431-434)
- Modify: `taskweave_hmi/taskweave_hmi/web/graph-run.js` (`chipButton` 159-182: the header's icon, the sentence, the settings rows), `app.js` (`bubbleWords` 375: the name is the kind word, the sentence on line 2)
- Modify: `taskweave_hmi/taskweave_hmi/web/style.css` (`.fl-node`, `.fl-head`, `.fl-sentence`, `.fl-settings`, `.fl-icon`)
- Test: `taskweave_hmi/test/test_graph_rows.py` (new; node part)

**Interfaces:**
- Produces in graph-core.js: `flowName(node) -> string` (the kind word: `'Move'` for `step.skill === 'move'`, else `humanise(step.skill)`; frames and ends as today); `stepSettings(step, order = []) -> [[key, valueText]]` (every set argument, in `order` first then the rest, an include's `with`); `nodeHeight(step, M, rows) -> number` = `M.head + M.sentence + rows * M.row + M.padY` where `rows` is the number of settings rows drawn; `GRAPH_METRICS.*` gain `sentence: 42, row: 26, rowEdit: 34, padY: 8, settingsMax: 4` (phone metrics: `sentence: 34, settingsMax: 0`); node width 240 (phone 200). `layoutGraph` reads each step item's height from `ctx.heightOf(node)` (default `M.node.h`).
- Consumes: Task 1's tokens.

- [ ] **Step 1: Write the failing tests**

```python
# taskweave_hmi/test/test_graph_rows.py
"""The node anatomy and the rows (spec 2026-11-06-editor-canvas.md sections 2 and 3), read from the scripts as text."""
import re
from pathlib import Path

WEB = Path(__file__).resolve().parents[1] / 'taskweave_hmi' / 'web'
CORE = (WEB / 'graph-core.js').read_text(encoding='utf-8')

def test_the_header_is_the_kind_word():
    body = CORE[CORE.index('function flowName'):CORE.index('function stepSettings')]
    assert "'Move'" in body and 'humanise(node.step.skill' in body
    assert 'ownId(' not in body   # the id is not drawn

def test_node_height_is_data_driven():
    assert re.search(r'function nodeHeight\(step, M, rows\)', CORE)
    assert 'M.head + M.sentence + rows * M.row + M.padY' in CORE
    assert 'getBoundingClientRect' not in CORE[CORE.index('function layoutGraph'):CORE.index('function graphCurve')]
    assert re.search(r'canvas:\s*\{[^}]*sentence: 42[^}]*row: 26[^}]*settingsMax: 4', CORE, re.S)
    assert re.search(r'canvasPhone:\s*\{[^}]*settingsMax: 0', CORE, re.S)

def test_settings_rows_follow_the_declared_order():
    assert re.search(r'function stepSettings\(step, order = \[\]\)', CORE)
```

- [ ] **Step 2: Run to see them fail**

Run: `cd taskweave_hmi && python -m pytest test/test_graph_rows.py -v`
Expected: FAIL, `stepSettings`/`nodeHeight` missing.

- [ ] **Step 3: Implement in graph-core.js**

`flowName`: a step says `'Move'` when `step.skill === 'move'`, else `humanise(step.skill)` (an include `'Mission'`, blocks and lanes and ends as today). `stepSettings(step, order)`: an include's `with` entries; else `Object.entries(step.args)` sorted by `order.indexOf(key)` (unknown keys after, in their own order), values through `flowValue`. `nodeHeight` as the interface says. `flowHead(name, kind, icon)`: a third argument, an element put before the name when given. `GRAPH_METRICS`: the new keys and widths. In `layoutGraph`, `measure(node)` for a step uses `{ w: M.node.w, h: ctx.heightOf ? ctx.heightOf(node) : M.node.h }`; the port positions (`M.line`, `M.fail`) stay relative to the top.

- [ ] **Step 4: Draw it on Run (graph-run.js `chipButton`) and in the bubble (app.js)**

`chipButton`: `flowHead(flowName(node), '', iconElement(iconOf(step, event), 'fl-icon'))`, then `el('div', {class: 'fl-sentence', text: step.instruction})`, then up to `M.settingsMax` rows from `stepSettings(step, argOrder(step))` as `.fl-settings > .fl-field`, a fifth `+n more` row when more; `argOrder(step)` reads `state.skills.find(s => s.name === step.skill)?.args` keys (app.js `state.skills`, the catalogue). `ctx.heightOf = (node) => nodeHeight(node.step, M, Math.min(M.settingsMax, rows))`. `bubbleWords`: `name` is `flowName`, `detail` the sentence (`s.step.instruction`, `oneLine`), the main argument moves to line 3 before `next`.

- [ ] **Step 5: Style** `.fl-sentence` (12px, two lines, `-webkit-line-clamp: 2`, `--text`), `.fl-settings` rows (11px mono, `--text-muted` keys), `.fl-icon` 14px before the name, header 12px 600.

- [ ] **Step 6: Run the tests and look at the mock** (running, failed): nodes 240 wide with the icon, `Move`, the sentence and up to four settings; the running node still lime; the bubble says `Move` and the sentence.

Run: `cd taskweave_hmi && python -m pytest test/test_graph_rows.py test/test_run_layout.py test/test_page_ids.py -q`
Expected: pass.

- [ ] **Step 7: Commit**

```bash
git add taskweave_hmi/taskweave_hmi/web/graph-core.js taskweave_hmi/taskweave_hmi/web/graph-run.js taskweave_hmi/taskweave_hmi/web/app.js taskweave_hmi/taskweave_hmi/web/style.css taskweave_hmi/test/test_graph_rows.py
git commit -m "HMI nodes: the kind word with its icon, the instruction sentence, the settings rows; heights from the data"
```

### Task 3: Rows by type of movement in the shared layout

**Files:**
- Modify: `taskweave_hmi/taskweave_hmi/web/graph-core.js` (`layoutGraph`: `row()` 205-213, `measure` 224-261, the root 262-267)
- Modify: `taskweave_hmi/taskweave_hmi/web/graph-run.js` (ctx: `rowOf`), `editor.js` (ctx: `rowOf` in `renderEditor` 527-529)
- Test: `taskweave_hmi/test/test_graph_rows.py` (rows part)

**Interfaces:**
- Produces: `KIND_ROWS = ['drive', 'arm', 'grip', 'wait', 'other']` in graph-core.js; `layoutGraph(root, ctx)` takes `ctx.rowOf(node) -> 0..4` (a step's row by `colourOf`; a stack's the row of its first leaf; absent: everything on row 0) and `M.rowGap` (`var(--row-gap)`'s value: 24, phone 16, in the metrics); a container's `row(sized, rowIndex)` lays its items on the rows they use.
- Consumes: Task 2's `heightOf`.

- [ ] **Step 1: Write the failing tests**

```python
def test_rows_are_by_kind_in_a_fixed_order():
    assert "KIND_ROWS = ['drive', 'arm', 'grip', 'wait', 'other']" in CORE

def test_a_container_opens_only_the_rows_it_uses():
    layout = CORE[CORE.index('function layoutGraph'):CORE.index('function graphCurve')]
    assert 'ctx.rowOf' in layout and 'M.rowGap' in layout
    assert 'used = [...new Set(' in layout           # the rows the container's children use, sorted
    assert 'rowTop' in layout                        # a row's top: the gap plus the tallest item of each row above

def test_both_tabs_pass_the_row_hook():
    for name in ('graph-run.js', 'editor.js'):
        text = (WEB / name).read_text(encoding='utf-8')
        assert 'rowOf:' in text and 'KIND_ROWS.indexOf(' in text
```

- [ ] **Step 2: Run to see them fail**

Run: `cd taskweave_hmi && python -m pytest test/test_graph_rows.py -v`
Expected: FAIL on `KIND_ROWS`.

- [ ] **Step 3: Implement the rows in `layoutGraph`**

In `row(sized, rowIndex)`: `used = [...new Set(sized.map(rowIndex))].sort()`; `rowH[r] = max h of the items in r`; `rowTop[r] = sum of rowH of the used rows above + M.rowGap per gap`; the row's `h = sum(rowH) + gaps`, its `line` = the line of the first item in its own row (`rowTop[r0] + first.line`); `place(x, y)`: each item at `(x advancing, y + rowTop[rowIndex(item)])`. A box's `measure` passes `rowIndex = (s) => ctx.rowOf ? ctx.rowOf(s.nodeRef) : 0` (nodeItem records `nodeRef: node`); a nested box or lane counts as the row of its first child. The root row: Start takes the row of the first item, Done the row of the last (`rowIndex` returns the neighbour's row for `start`/`end` items). The loop wire and the editor's rule row stay under the container's bottom as today.

- [ ] **Step 4: Pass the hook from both tabs**: `rowOf: (node) => KIND_ROWS.indexOf(node.kind === 'step' ? colourOf(node.step) : colourOf(plan.steps[node.leaves[0]]))` (graph-run.js, with `Math.max(0, …)`); the editor the same with its own `colourOf` and `node.step`. Metrics gain `rowGap: 24` (`canvas`, `editor`), `16` (phone ones).

- [ ] **Step 5: Run the tests, then look at the mock**: the frame cycle's drives on the top row, arm moves under them, grips lower; Start on the first node's row; a box spans the rows of its steps; wires bend between rows; Follow and Fit still land on the running node.

Run: `cd taskweave_hmi && python -m pytest test/test_graph_rows.py test/test_run_layout.py -q`
Expected: pass.

- [ ] **Step 6: Commit**

```bash
git add taskweave_hmi/taskweave_hmi/web/graph-core.js taskweave_hmi/taskweave_hmi/web/graph-run.js taskweave_hmi/taskweave_hmi/web/editor.js taskweave_hmi/test/test_graph_rows.py
git commit -m "HMI layout: rows by type of movement, each container opening only the rows it uses"
```

### Task 4: graph-canvas.js: the shared viewport, the wheel rule, the places

**Files:**
- Create: `taskweave_hmi/taskweave_hmi/web/graph-canvas.js`
- Modify: `taskweave_hmi/taskweave_hmi/web/graph-run.js` (the canvas object, `canvasFit`…`canvasView`, `movePlaces`, `storedPlaces`/`storePlaces`, the drag: lines 331-660 move out; `renderGraph` calls the shared object), `index.html` (the script tag before graph-run.js), `server.py` `STATIC_FILES`, `test_page_ids.py` `SCRIPTS`
- Test: `taskweave_hmi/test/test_graph_canvas.py` (new)

**Interfaces:**
- Produces: `class GraphCanvas` with `constructor(box, { mission: () => string, redraw: () => void, snap?: (point) => spot|null, drop?: (key, spot) => boolean })`; `setScene(layout, tree, node, cards = [])` (puts the drawn node on the layer with the places applied; `cards` are the pop-up or sheet elements), `fit()`, `centre(x, y)`, `follow(on)`, `tidy()`, `focusKey(key)`, `get places()` (the map), `get state()` (`{x, y, scale, follow}`), `placeCard(selector, key)` (the Run pop-up's placing), `dispose()`. The wheel rule as a top-level pure function `wheelIntent(e) -> 'zoom' | 'pan'`: Ctrl or Meta: `'zoom'`; `deltaMode !== 0`: `'zoom'`; `deltaX !== 0` or `Math.abs(deltaY) < 40` or `!Number.isInteger(deltaY)`: `'pan'`; else `'zoom'`. Shared names start with `graphCanvas`/`GraphCanvas` (no `canvas`, `gesture`, `pointers` globals: editor.js and graph-run.js keep none of those after this task).
- Consumes: Task 3's layouts (frames with `ox/oy` as movePlaces records them).

- [ ] **Step 1: Write the failing tests**

```python
# taskweave_hmi/test/test_graph_canvas.py
"""The shared viewport (spec 2026-11-06-editor-canvas.md sections 4 and 5): one script both tabs drive, the wheel rule."""
import re
from pathlib import Path

WEB = Path(__file__).resolve().parents[1] / 'taskweave_hmi' / 'web'
CANVAS = (WEB / 'graph-canvas.js').read_text(encoding='utf-8')

def test_the_script_is_served_and_loaded_before_its_users():
    html = (WEB / 'index.html').read_text(encoding='utf-8')
    assert html.index('graph-canvas.js') < html.index('graph-run.js') < html.index('editor.js')
    server = (WEB.parent / 'server.py').read_text(encoding='utf-8')
    assert "'graph-canvas.js'" in server

def test_wheel_intent_rule():
    rule = CANVAS[CANVAS.index('function wheelIntent'):]
    rule = rule[:rule.index('\n}') + 2]
    assert "e.ctrlKey || e.metaKey" in rule and "'zoom'" in rule and "'pan'" in rule
    assert 'e.deltaX !== 0' in rule and 'Math.abs(e.deltaY) < 40' in rule and 'Number.isInteger(e.deltaY)' in rule

def test_the_tabs_share_the_viewport_and_the_places():
    run = (WEB / 'graph-run.js').read_text(encoding='utf-8')
    assert 'new GraphCanvas(' in run and 'function movePlaces' not in run and "'tw-plan-places'" not in run
    assert "const PLACES_KEY = 'tw-plan-places'" in CANVAS and 'class GraphCanvas' in CANVAS
```

- [ ] **Step 2: Run to see them fail**

Run: `cd taskweave_hmi && python -m pytest test/test_graph_canvas.py -v`
Expected: FAIL, no graph-canvas.js.

- [ ] **Step 3: Create graph-canvas.js** by moving the camera (`x, y, scale, follow, focus, inset, base, places, moving`), `canvasFit`/`canvasCentre`/`canvasApply`/`canvasZoom`, `placePop` (as `placeCard`), the pointer gestures with the node drag and the hold, `movePlaces`, `storedPlaces`/`storePlaces`, the first-view rule (`FIRST_FIT_MIN`) and the ResizeObserver into the class; the wheel handler uses `wheelIntent`; on a `'pan'` the layer moves by `(-deltaX, -deltaY)` (lines scaled by 16). `snap(point)` is called on every drag move when given (the host lights the spot it returns); on release the host's `drop(key, spot)` runs first and, when it returns `true`, the drag stores no place.

- [ ] **Step 4: Make graph-run.js use it**: `const runCanvas = new GraphCanvas($('plan-graph'), { mission: () => missionOf(shownPlan().plan), redraw: renderGraph })`; `renderGraph` builds the layout and the drawn node as today and calls `runCanvas.setScene(layout, tree, drawn.node, card)`, then `runCanvas.focusKey(focusItemKey)`; Fit, Follow, Tidy up and the step chip call the methods. Add the script to index.html, `STATIC_FILES`, `SCRIPTS`.

- [ ] **Step 5: Run the suites and look at the mock** (running): pan, pinch, a wheel notch zooms, a trackpad scroll (emulate with `dispatchEvent(new WheelEvent('wheel', {deltaY: 3.5, deltaMode: 0, bubbles: true, cancelable: true}))` on the viewport) pans; a node drag stores; Tidy up clears; Follow and the pop-up as before.

Run: `cd taskweave_hmi && python -m pytest test/test_graph_canvas.py test/test_run_layout.py test/test_page_ids.py test/test_static_files.py test/test_global_names.py -q`
Expected: pass.

- [ ] **Step 6: Commit**

```bash
git add taskweave_hmi/taskweave_hmi/web/graph-canvas.js taskweave_hmi/taskweave_hmi/web/graph-run.js taskweave_hmi/taskweave_hmi/web/index.html taskweave_hmi/taskweave_hmi/server.py taskweave_hmi/test/test_graph_canvas.py taskweave_hmi/test/test_page_ids.py
git commit -m "HMI graph-canvas.js: the viewport, the gestures, the places and the node drag shared by both tabs; a trackpad scroll pans"
```

### Task 5: The Program tab as a canvas with a floating panel

**Files:**
- Modify: `taskweave_hmi/taskweave_hmi/web/editor.js` (`renderEditor` 505-541, `keepInView` 556, `addSpots` 607, `openPalette` 638, `openRuleMenu` 773, `nodeDown` 820-872, `canvasDown` 873-921, `editorOpen` 1090)
- Modify: `taskweave_hmi/taskweave_hmi/web/index.html` (the `#editor` markup 194-233: Fit and Tidy up in `.ed-head`, `#ed-panel-toggle`), `style.css` (`.ed`, `.ed-work`, `.ed-canvas`, `.ed-panel`, `.ed-palette`, `.ed-menu`, `.ed-spot`)
- Test: `taskweave_hmi/test/test_editor_canvas.py` (new)

**Interfaces:**
- Consumes: `GraphCanvas` (Task 4) with `snap` and `drop`; Task 3's `rowOf`.
- Produces: `#ed-fit`, `#ed-tidy`, `#ed-panel-toggle` (aria-expanded; the collapsed state stored as `tw-ed-panel`); `editorCanvas` (the GraphCanvas of the Program tab); `ed.spots` keep `{list, index, x, y, el}` in layout units.

- [ ] **Step 1: Write the failing tests**

```python
# taskweave_hmi/test/test_editor_canvas.py
"""The Program tab as a canvas (spec 2026-11-06-editor-canvas.md section 4)."""
import re
from pathlib import Path

from test_page_ids import page_ids

WEB = Path(__file__).resolve().parents[1] / 'taskweave_hmi' / 'web'
ED = (WEB / 'editor.js').read_text(encoding='utf-8')
CSS = (WEB / 'style.css').read_text(encoding='utf-8')

def test_the_editor_has_its_viewport_and_tools():
    assert {'ed-fit', 'ed-tidy', 'ed-panel-toggle', 'ed-canvas', 'ed-panel'} <= page_ids()
    assert 'new GraphCanvas(' in ED and 'scrollLeft' not in ED
    assert re.search(r'\.ed-panel \{[^}]*position: absolute', CSS, re.S) and 'overflow: auto; min-height: 480px' not in CSS

def test_a_drop_on_a_spot_reorders_and_elsewhere_places():
    assert "drop: (key, spot) =>" in ED and "op: 'move'" in ED
    assert 'snap: (p) =>' in ED and 'SPOT_REACH' in ED

def test_a_drop_into_itself_places_instead():
    drop = ED[ED.index('drop: (key, spot) =>'):]
    drop = drop[:drop.index('\n  }') + 4]
    assert 'inside(spot.list, ' in drop and 'return false' in drop

def test_the_panel_floats_and_collapses():
    assert "'tw-ed-panel'" in ED and 'aria-expanded' in ED
```

- [ ] **Step 2: Run to see them fail**

Run: `cd taskweave_hmi && python -m pytest test/test_editor_canvas.py -v`
Expected: FAIL, no `ed-fit`.

- [ ] **Step 3: Markup and style**: `.ed` fills the window under the strip (as `.run` does: `height: calc(100dvh - var(--top-h))`, no page scroll on the Program tab while a mission is open); `.ed-head` keeps Back, the name, Undo, Redo, Check, Save and gains `#ed-fit` and `#ed-tidy` (icon buttons, Task 7 draws the icons; words until then); `.ed-work` is `position: relative; flex: 1; min-height: 0`; `#ed-canvas` is the viewport box (`.graph-box.canvas-box`); `#ed-panel` is `position: absolute; top: 0; right: 0; bottom: 0; width: 380px` (320 up to 1000px), its own scroll, `--shadow-2`, with `#ed-panel-toggle` (a chevron, `aria-expanded`) at its top edge; collapsed it is a 44px tab at the right edge (`.ed-panel.closed`). Phone (`max-width: 600px`): `.ed-canvas` 60vh and the panel in flow under it as today.

- [ ] **Step 4: Drive the viewport from editor.js**: `const editorCanvas = new GraphCanvas($('ed-canvas'), { mission: () => ed.name, redraw: renderEditor, snap, drop })`; `renderEditor` builds the layout and the drawn node, adds the spots (in layout units, positioned inside the layer) and calls `editorCanvas.setScene(layout, root, drawn.node)`, then keeps the selection in view with `editorCanvas.centre(cx, cy)` only when it is outside the viewport (replaces `keepInView`); `editorOpen` resets the camera (`fit()` on the first draw). `snap(p)`: the nearest spot within `SPOT_REACH` (layout units) that is not inside the dragged node's own path (`inside(s.list, key)`), lit with `.near`. `drop(key, spot)`: no spot: `return false`; a spot inside the dragged box: `return false` (placed, Review Focus 3); the same slot or the next: `return true` with no op; else `commit([{ op: 'move', from: path, to: joinPath(spot.list, spot.index) }], { select: landed })` and `return true`. Remove `nodeDown`'s own drag (the ghost) and `canvasDown`'s pan; keep the rubber-band selection on Shift+drag on the ground (`shiftKey` at pointerdown: the box selection as today in viewport coordinates), a plain drag pans. The palette and the rule menu open at `editorCanvas.state`-transformed points and are placed by `placeCard` so they follow their node.

- [ ] **Step 5: Run the tests and the mock** (scenario `rules`, open a mission on the Program tab): pan, zoom, drag a node onto a `+` (reorders), drag it elsewhere (stays, survives a reload, shows on Run too), Tidy up, the panel collapses and opens, Shift+A and the keyboard still work, the phone keeps the sheet.

Run: `cd taskweave_hmi && python -m pytest test/test_editor_canvas.py test/test_graph_canvas.py test/test_page_ids.py test/test_global_names.py -q`
Expected: pass.

- [ ] **Step 6: Commit**

```bash
git add taskweave_hmi/taskweave_hmi/web/editor.js taskweave_hmi/taskweave_hmi/web/index.html taskweave_hmi/taskweave_hmi/web/style.css taskweave_hmi/test/test_editor_canvas.py
git commit -m "HMI Program tab: the shared viewport, a drop on a + reorders and elsewhere places, the panel floating at the right"
```

### Task 6: Settings on the node (editable, Program)

**Files:**
- Modify: `taskweave_hmi/taskweave_hmi/web/editor.js` (`nodeEl` 341-387: the settings controls; a new `settingsRows(step, path)`; `cardChanged` 975 reused for `args`), `app.js` (`argEditor(key, spec, value, required)` 1253: export its field element unchanged; a `skillArgs(skill) -> [[key, spec, required]]` helper next to `stepCard`)
- Modify: `style.css` (`.fl-settings .fl-ctl`, inputs and selects inside a node, 32px/44px)
- Test: `taskweave_hmi/test/test_editor_canvas.py` (settings part)

**Interfaces:**
- Consumes: Task 2's `stepSettings`, `nodeHeight` with `M.rowEdit`; Task 5's viewport.
- Produces: `settingsRows(step, path) -> HTMLElement[]` (one `.fl-field.fl-ctl` per set or required argument, a control built by `argEditor`, and a last `+ argument` row opening a small `.ed-menu` of the skill's unset arguments); a change writes `commit([{ op: 'set', at: path, key: 'args', value }], { tag: 'card:' + path })`.

- [ ] **Step 1: Write the failing tests**

```python
def test_settings_are_edited_on_the_node():
    assert re.search(r'function settingsRows\(step, path\)', ED)
    assert 'argEditor(' in ED and "key: 'args'" in ED and "'+ argument'" in ED
    assert 'M.rowEdit' in ED

def test_a_control_press_does_not_drag_or_select():
    node = ED[ED.index('function nodeEl'):ED.index('function chevron')]
    assert ".closest('.fl-ctl')" in node
```

- [ ] **Step 2: Run to see them fail**: `cd taskweave_hmi && python -m pytest test/test_editor_canvas.py -v` → FAIL on `settingsRows`.

- [ ] **Step 3: Implement**: `skillArgs(skill)` in app.js from the catalogue's args schema (`state.skills`, `properties` and `required`); `settingsRows` builds the rows (the set arguments in declared order, then unset required ones, then `+ argument`); `nodeEl` appends them after the sentence and sets `ctx.heightOf` with `rows * M.rowEdit`; `nodeDown`/`nodeClick` return early on `e.target.closest('.fl-ctl')`; a change in a control: read the node's controls into an args object and commit as the interface says (`cardChanged` keeps the panel in step: the panel's card re-reads the step). A move step keeps the panel's target pickers inside the node's `to` control (the same `argEditor`).

- [ ] **Step 4: Style** the controls in a node: `--fs-xs`, 32px high (44 under coarse), the field surface, no label (the key is the row's label).

- [ ] **Step 5: Tests and the mock**: change a `seconds` on a wait node → the YAML preview (Check) shows it, Undo takes it back; add an argument; a node with none shows the `+ argument` row only.

Run: `cd taskweave_hmi && python -m pytest test/test_editor_canvas.py test/test_graph_rows.py test/test_page_ids.py -q`
Expected: pass.

- [ ] **Step 6: Commit**

```bash
git add taskweave_hmi/taskweave_hmi/web/editor.js taskweave_hmi/taskweave_hmi/web/app.js taskweave_hmi/taskweave_hmi/web/style.css taskweave_hmi/test/test_editor_canvas.py
git commit -m "HMI Program tab: a step's settings edited on its node, + argument from the skill's schema"
```

### Task 7: Run without the band: the strip's robot and step line, the progress bar, the window and the dock over the graph

**Files:**
- Modify: `taskweave_hmi/taskweave_hmi/web/index.html` (the strip 34-79: `.strip-bot` before `#conn`, `#strip-step` after `#state`, `#progress` under the strip; `#view-run` 85-164: the band section goes, `.canvas` is the whole page, `.window` and `.mission-dock` as the canvas's first children; the Run tools and the editor head as icon buttons with `aria-label`s)
- Modify: `app.js` (`renderBoard` 386-483 fills `#strip-step`, the chips and the dock; `renderPhases` 500-530 draws segments into `#progress`; the band's `--p`, `--band` observer and `enlargeWindow`'s band min-height go; `setWindowWidth(px)` and the `#window-grip` drag, `tw-window-width`; `window.resize` re-clamps), `graph-run.js` (Fit's room: the top 72px under the dock), `icons.js` (new ids `fit`, `follow`, `tidy`, `graph`, `list`, `panel`), `style.css` (`.strip-bot` 24px with the state poses, `#strip-step`, `#progress` segments, `.window` and `.mission-dock` floating, `.window-grip`, icon buttons; the `.band*` rules go)
- Modify: `taskweave_hmi/taskweave_hmi/web/DESIGN.md` (Strip, Run page, Mission graph, Program tab, Icons sections), `adient-uc1-frame/plans/hmi-session/HANDOVER.md` (the round's section), the mock's `planning`/`succeeded`/`aborted` scenarios unchanged
- Test: `taskweave_hmi/test/test_run_layout.py` (rewritten), `taskweave_hmi/test/test_icons.py` (the new ids, as that test checks ids today)

**Interfaces:**
- Consumes: Task 1's tokens; Task 2's `bubbleWords` (the kind word and the sentence); Task 4's `GraphCanvas` (`fit()` takes an `inset` of `{top: 72}` on Run).
- Produces: `#strip-step` (the kind word `.strip-kind`, the sentence `.strip-sentence`, the chips `#board-step`, `#board-total`, `#board-cycle` keep their ids), `#progress` (an `ol` of `li` segments with `--steps` and `--done-part` as the floor's bars had), `.mission-dock` holding `#mission-start`, `#board-act`, `#mission-controls`; `#window-grip`; `setWindowWidth(px|null)` (null: the 320px default; clamps `Math.max(240, Math.min(px, innerWidth / 2))`; stores `tw-window-width`). The ids `band`, `board`, `board-kind`, `board-instruction`, `board-detail`, `board-next`, `board-phases`, `resources` go (`#board-error` stays as the sr-only alert under the strip; `#board-state` stays).

- [ ] **Step 1: Rewrite the failing tests**

```python
# test_run_layout.py, replacing the band tests
def test_the_strip_carries_the_robot_the_state_and_the_step():
    ids = page_ids()
    assert {'strip-step', 'progress', 'board-step', 'board-total', 'board-cycle', 'state', 'conn'} <= ids
    assert not {'band', 'board', 'board-kind', 'board-instruction', 'board-detail', 'board-next', 'board-phases', 'resources'} & ids
    strip = PAGE[PAGE.index('<header id="mission"'):PAGE.index('</header>')]
    assert strip.index('class="strip-bot') < strip.index('id="conn"') < strip.index('id="state"') < strip.index('id="strip-step"')
    assert re.search(r'--strip:\s*36px', CSS) and re.search(r'#progress \{[^}]*height: 6px', CSS, re.S)

def test_the_window_and_the_dock_float_over_the_canvas():
    run = PAGE[PAGE.index('id="view-run"'):PAGE.index('id="view-missions"')]
    assert run.index('id="canvas"') < run.index('class="mission-dock"') < run.index('class="window"') < run.index('id="plan-graph"')
    assert {'window-grip', 'window-enlarge', 'mission-select', 'btn-start', 'board-act', 'btn-pause', 'btn-abort'} <= page_ids()
    assert re.search(r'\.window \{[^}]*position: absolute[^}]*top: 12px[^}]*right: 12px', CSS, re.S)
    assert re.search(r'\.mission-dock \{[^}]*position: absolute', CSS, re.S)

def test_window_width_is_clamped_and_stored():
    app = (WEB / 'app.js').read_text(encoding='utf-8')
    assert "'tw-window-width'" in app and re.search(r'function setWindowWidth\(px\)', app)
    body = app[app.index('function setWindowWidth'):]
    body = body[:body.index('\n}') + 2]
    assert 'Math.max(240,' in body and 'innerWidth / 2' in body
    assert "addEventListener('resize'" in app   # a narrower window clamps it again

def test_the_toolbars_are_icon_buttons():
    for button in ('canvas-fit', 'canvas-follow', 'canvas-reset', 'ed-fit', 'ed-tidy'):
        assert re.search(rf'id="{button}"[^>]*aria-label="', PAGE), button
```

- [ ] **Step 2: Run to see them fail**: `cd taskweave_hmi && python -m pytest test/test_run_layout.py -v` → FAIL on `strip-step`.

- [ ] **Step 3: The strip**: `.strip-bot` (the `.bot` SVG, 24px high, `aria-hidden`) before `#conn`; its state poses keyed on `.mission:has(#state.<state>)` as the band keyed on the bubble (the walk cycle stays, with no `left` move); `#strip-step` after `#state`, shown on Run only (`body.on-run`), its kind word in `--k`, the sentence with an ellipsis, the chips at its end; `#progress` an `ol` under the strip (`position: absolute; left: 0; right: 0; top: 100%; height: 6px; display: flex; gap: 2px`), `renderPhases` drawing one `li` per phase with `--steps`, `--done-part`, `now`/`failed` and an `aria-label`, into `#progress`; `renderMission` stops writing the old `#progress-bar`.

- [ ] **Step 4: The Run page**: delete the band section; `#view-run` is `.run` with `.canvas` filling it; inside `.canvas`, before the tools, `.mission-dock` (the picker, `#board-act`, `#mission-controls`) and `.window` (`#viewer3d-run`, `#window-enlarge`, `#window-grip`), both `position: absolute; top: 12px` with the window at `right: 12px` and the dock at `right: calc(12px + var(--window-w, 320px) + 12px)`; `setWindowWidth` sets `--window-w` on `.canvas`; the window `aspect-ratio: 16 / 10`; the dock's content row as section 5 says; the canvas tools move to the bottom right (`.canvas-tools { position: absolute; bottom: 12px; right: 12px }`) so the top is the window's and the dock's. `renderBoard` writes the strip's line and the dock's button as before (the same `act` table); the bubble-only branches (`board-detail`, `board-next`) go; the error stays the sr-only alert. `enlargeWindow`'s band code goes: enlarged, the window is `inset: 0` of the canvas. Phone: the window a 96px thumbnail, the dock a full-width row under the bar, `#strip-step` in its own row under the strip (`--strip` grows by a row on Run).

- [ ] **Step 5: Icons**: six ids in icons.js (24-unit stroked paths in the set's style), the Run tools and the editor head as `.icon` buttons with the words as `aria-label` and `title` (Follow lit when pressed as today, Graph/List a segmented pair of icons), the panel's headings with a 14px icon.

- [ ] **Step 6: DESIGN.md**: the Strip (the robot, the step line, the bar), the Run page (the canvas, the window, the dock), the Mission graph section's node anatomy and the rows, the Program tab's viewport, panel and settings on the node, the Icons section's new ids. HANDOVER.md: the round's section (branch, what, checks, what was not tried by hand).

- [ ] **Step 7: Run everything and look at the mock** (running, idle, failed, at 1400px and 375px): the robot marches in the strip's corner, the step line and the chips in the strip, the segmented bar under it, the window and the dock over the graph's top right, the window resizes and clamps, Enlarge and Escape, Start from the dock, the canvas tools at the bottom right.

Run: `cd taskweave_hmi && python -m pytest test -q -p no:cacheprovider` (locally two tests are known to fail on Windows; the suite is green in frame-ros: `plans/hmi-session/twtest.sh <worktree> 95 taskweave_hmi`), `TASKWEAVE_STRICT_AGNOSTIC=1 python -m pytest taskweave_core/test/test_agnostic.py -q`, `python -m ruff check taskweave_hmi/test`.
Expected: pass (221+ in frame-ros), 6 passed, clean.

- [ ] **Step 8: Commit**

```bash
git add taskweave_hmi/taskweave_hmi/web/index.html taskweave_hmi/taskweave_hmi/web/app.js taskweave_hmi/taskweave_hmi/web/graph-run.js taskweave_hmi/taskweave_hmi/web/icons.js taskweave_hmi/taskweave_hmi/web/style.css taskweave_hmi/taskweave_hmi/web/DESIGN.md taskweave_hmi/test/test_run_layout.py taskweave_hmi/test/test_icons.py
git commit -m "HMI Run: the band goes; the robot and the step in the strip, a segmented progress bar, the 3D window and the mission dock over the graph; icon toolbars"
```
