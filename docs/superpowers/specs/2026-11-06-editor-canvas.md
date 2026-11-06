# The editor canvas: Program as a canvas like Run, settings on the node, rows by movement

Status: design, 6 Nov 2026, from the user's direction after the Run tune (hmi-run-tune, main 18a615d). It changes the
Program tab, the node look and the shared layout of both tabs, and two things on Run. The mission files, the ops
format, the server and the planner stay as they are. Parallel-as-wiring (two nodes wired from one done port run in
parallel) is a change to the mission data and the executive: it gets its own spec with the core session and is not
in here.

## 1. What the user asked for

- **The editor in the Run canvas's style**: a full-window viewport that pans and zooms, nodes dragged where the
  operator wants them, the mission panel floating over the canvas at the right.
- **Settings on the node, as Blender does**: a node's arguments are edited on the node itself, not only in a panel.
- **Rows by type of movement**, on both tabs: the standard layout puts move, grab and the rest at different heights,
  one row per node, not everything in one line. (The old graph's resource lanes, in that form.)
- **Names**: a node's header says just what it is, `Move`, `Grip`, `Wait`; the exact description (the instruction
  sentence) sits below it. The same for every kind.
- **Run**: the band's height adjustable by a drag; a trackpad's two-finger drag pans the canvas.

## 2. The node (both tabs)

A node (`.fl-node`) keeps its dark body, its 1px border and its header strip in the kind colour. What changes:

| Part | Was | Now |
|---|---|---|
| header | the humanised id (`To shelf`), the kind word at the right | the **kind word only**: `Move`, `Grip`, `Wait`, `Hold level`, …; an include `Mission`, a block `Group` / `Repeat` / `Parallel`, a lane `Lane`; Start `Start`, Done `Done` |
| line 1 of the body | the main argument (`to  bin_1/place`) | the **instruction sentence**, 13px, up to two lines with an ellipsis (the whole of it the tooltip); an include shows the included mission's name; a block its instruction or nothing |
| the rest of the body | – | the **settings**: one row per argument, `key value`, mono as today |

The kind word: every move step says `Move` (its kind of motion shows in the header's colour and its row, section
3), else the skill's name humanised (`grip` → `Grip`, `hold_level` → `Hold level`). The step's id is not drawn: it
is in the tooltip, the pop-up and the panel.

**Settings, Run tab** (read-only): every argument the step sets, in the skill's declared order, up to four rows;
more say `+n more` as a fifth row (the pop-up has them all). An include: its `with` values. A block: as today.

**Settings, Program tab** (editable, the Blender part): the same rows, each a control instead of a value: a select
for an enum or a target (the pickers `stepCard` already builds: points, poses, objects), a number input, a text
input, a checkbox. The rows are the arguments the step sets plus the skill's required ones; `+ argument` as the
last row adds one from the skill's schema (a small menu). A change writes a `set` op for `args` as `cardChanged`
does today (one undo entry while the node stays selected). The panel keeps the whole form (instruction, id,
timeout, retries, the failure rule, the replan target) and the block and mission forms.

**Sizes.** Nodes are 240 wide (200 on a phone). The height follows the content: header 24, the sentence 2 × 17 + 8,
each settings row 26 (34 with a control), padding 8; the layout reads each node's height from its data (a count of
rows), never from the DOM, so Run and Program agree and the layout is pure. A phone draws the sentence only (no
settings rows) as it draws no fields today.

## 3. Rows by type of movement (the shared layout)

Every step belongs to one **row** by what it moves, the kind colour's own order top to bottom:

| Row | Kind (`colourOf`) | Steps |
|---|---|---|
| 1 | drive | navigate, drive straight, a move step whose kind is navigate or straight |
| 2 | arm | hand, posture, joints moves, move_object, set_posture |
| 3 | grip | grip, move_gripper |
| 4 | wait | wait |
| 5 | other | everything else (a policy skill, an unknown skill) |

- The flow still runs left to right in run order; a node is placed in its row at the next x after the previous
  node. A wire from one row to another is the S-curve `graphWire` already draws (level, then the bend).
- A row that no step in a container uses takes no space: each **container** (the root flow, a box, a lane) lays its
  children on the rows those children use, in order, so a mission of drives and grips has two rows, not five.
- A **box** (an include, a group, a repeat; a lane) spans the rows its children use; its title bar stays at its
  top. A closed box (a stack node) sits in the row of its first step. A parallel block's lanes stack as today, each
  lane with its own rows inside.
- **Start** sits in the row of the first node, **Done** in the row of the last; the loop wire runs under the lowest
  row. The editor's rule row (Fail terminals, Retry nodes) sits under the lowest row of its container as today; a
  replan wire along the bottom as today.
- The row gap is 24px (16 on a phone). A node's y inside its row is the row's top; rows are as high as their
  tallest node (the heights of section 2).
- The dragged places (`tw-plan-places`) are offsets from this layout, as on Run today; they are the same store for
  both tabs, so the layout made while programming is what Run shows. Tidy up on either tab clears them.

## 4. The Program tab as a canvas

- **The viewport.** `#ed-canvas` becomes the Run canvas's viewport: the tab fills the window under the strip with
  no page scroll (the editor's head row stays above it: Back, the name, Undo, Redo, Check, Save), the dotted grid,
  one transformed layer, pan by a drag on the ground, zoom 0.4 to 1.5 by the wheel (section 5's rule), Ctrl+wheel
  or a pinch, Fit and Tidy up in the head row. The first view is a fit, or scale 1 on the selection as the Run
  canvas opens on its node. The camera, the gestures, the places and the drag code move from graph-run.js into a
  shared `graph-canvas.js` that both tabs use.
- **Dragging a node.** As on Run (a mouse at once, a finger after a hold). A drop on a wire's `+` spot (within
  `SPOT_REACH` as today) **reorders** the mission as today; a release anywhere else **places** the node (its
  offset stored). While a drag is over a spot the spot lights as today and the node snaps to it. Shift-drag never
  reorders. The fail port's drag (a replan target) is unchanged.
- **The panel** (`#ed-panel`) floats over the canvas at the right, 380px, full height under the head row, its own
  scroll, a `--shadow-2`; a chevron at its top edge collapses it to a 44px tab at the right edge (remembered in the
  browser). The canvas is the full width under it: Fit fits the tree into the room left of the panel. On a phone the
  panel stays a sheet under the canvas as today.
- **Selection, the palette, the rule menu, the keyboard** (Shift+A, F, Delete, Ctrl+Z/Y/D, the arrows): unchanged;
  the palette and the menu open in viewport coordinates as the Run pop-up does and follow their node under pan and
  zoom.
- **Settings on the node** as section 2: a click in a control edits it and does not select or drag the node; a
  click on the header or the sentence selects it; a press on them past the threshold drags it.

## 5. Run: the band's height and the trackpad

- **The band splitter.** A grab handle along the band's bottom edge (`.band-grip`, the full width, 8px high, a
  44px hit area; `aria-label="Resize the band"`, a slider role with the height as its value, arrow keys 16px).
  Dragging it sets the band's height between 200px and 60% of the window; the window fills its column at that
  height (no 16:10 then). The height is remembered in the browser (`tw-band-height`); a double tap or Enter on the
  handle puts the 16:10 default back. On a phone there is no handle.
- **The wheel.** A notch of a mouse wheel zooms around the pointer as today; a trackpad's two-finger scroll pans.
  The rule, per wheel event: with Ctrl or Cmd, zoom (a pinch); else when `deltaMode` is 0 and the event has a
  sideways part or a vertical part under 40 in magnitude or not a whole number, pan by `(deltaX, deltaY)`; else
  (whole steps of 40 or more, no sideways part: a wheel notch) zoom. Both tabs.

## 6. Sizes and breakpoints

| Width | Program | Run |
|---|---|---|
| ≥ 1000px | the canvas the full window under the head row, the panel floating at the right | as today, plus the splitter |
| 600 to 1000px | the same; the panel 320px | as today |
| ≤ 600px (phone) | the canvas 60vh with the panel sheet under it as today; nodes 200 wide, the sentence only | no splitter |

Touch targets 44px, the node's controls 32px high on a laptop (44 on touch), the sentence 13px, settings 12px mono.

## 7. Code

- `graph-core.js`: `flowName` returns the kind word; `nodeRows(step, M)` gives a node's height from its data; the
  layout takes a `rowOf(node)` hook and lays each container's children on rows (section 3); `drawGraph` draws the
  sentence and the settings rows (`ctx.renderSettings(step)` for the editor's controls; the Run view's plain rows).
- New `graph-canvas.js`: the camera, the gestures (pan, pinch, wheel, node drag with the hold), `movePlaces`, the
  places store, Fit and Tidy up, the pop-up placement; graph-run.js and editor.js call it with their own render
  hooks (the editor adds the spot snapping to the drag's move and drop).
- `editor.js`: the viewport instead of the scrolling box; the drag goes through graph-canvas with spot snapping;
  the node's settings controls and their `set` ops; the floating panel and its collapse.
- `app.js`: the band splitter (`--band` set by the operator overrides the window's height); `GRAPH_METRICS` rows.
- `style.css`: the node's sentence and settings rows, the controls in a node, the Program viewport and the floating
  panel, the band grip.
- `DESIGN.md`: the node anatomy, the layout rows, the Program tab, the band.
- Tests: `test_page_ids.py` and `test_static_files.py` as today (the new script in `STATIC_FILES`);
  `test_run_layout.py` gains the grip and the shared store; a new `test_graph_rows.py` reads graph-core.js as text
  for the row order and the pure height rule; `test_global_names.py` for the new script. Mock scenarios as today.

## 8. The technical look (a redesign check, both tabs and the strip)

The user's read of the HMI today: large type and little space between things make it look child-friendly; the
reference is the professional tools (Blender, Cinema 4D): dense, quiet, small type with room to breathe, icons for
depth. The direction in DESIGN.md (dark technical panel, Montserrat + JetBrains Mono, lime signals) stays; the scale
and the density change. Density follows the **input**, not the width: a coarse pointer (a touch screen, gloves)
keeps the 44px targets; a mouse gets the dense scale.

- **Type scale.** `--fs-xs` 11, `--fs-sm` 12, `--fs-s` 13, `--fs-base` 14, `--fs-md` 15, `--fs-lg` 17, `--fs-xl` 20,
  `--fs-2xl` 24. Body and controls 13px; the bubble's name 20px (17 up to 1000px); the state badge 11px 700
  uppercase as a small tag, not a block; node headers 12px 600, the sentence 12px, the settings 11px mono; the phase
  names 12px; the strip's robot name 12px. Nothing under 11px.
- **Spacing.** Controls 32px high on a fine pointer (`--tap-s`), 44 on a coarse one; the strip 36px; the canvas
  toolbar and the editor's head row 36px; the bubble's padding `--sp-3`; the panel's fields 8px apart with 11px
  labels; the band's bubble and stage gap `--sp-2`. More air between groups, less inside them: the sections of the
  panel separated by a hairline and 16px, their rows by 6px.
- **Shape and surface.** Radius 4px on nodes, boxes, buttons and the window (`--r-m` 4, `--r-s` 3); pills only on
  the state tag and the chips; the bubble a 1px border, not 2px; the state colour as a 2px left bar on the bubble
  and as the badge, not as a tint of the whole bubble; the band and the canvas toolbar separated by hairlines
  (`--border`) on the panel surface, no card shadows but under what floats.
- **Icons.** The kind icon (icons.js `skillIcon`, 14px) at the left of every node header before the kind word, on
  both tabs; the toolbar buttons as icon buttons with tooltips and a word only where the icon is not standard: Fit
  (a frame), Follow (a crosshair, lit when on), Tidy up (a grid with an arrow), Graph / List (an icon each), the
  3D window's Enlarge as today; the panel's sections with a small icon in their heading (mission, step, block,
  rule); the rail's icons as today, 20px, with 11px words. The state badge gets no icon (its colour and word are
  the signal).
- **Colour.** The kind colours stay on the header strips only; bodies are the card surface. The lime run fill on
  the running node stays (it is the andon). Muted text `--text-muted` for every secondary line; no coloured text
  except the state words and errors.
- **The robot.** Stays; 120px on a desktop (the band is the operator's to size).

The result should read as a control surface, not a toy: small, even type, aligned on a 4px grid, with the colour
reserved for state and kind. Contrast stays at 4.5:1 for text and 3:1 for the tracks and frames (the review of
6 Nov). DESIGN.md is rewritten where these rules change it (tokens, type, spacing, shape, components).

## 9. Out of scope

Parallel as wiring and detached branches (a spec with core), positions stored in the mission file, a minimap, the
Teach, Robot and Log tabs, the planner.
