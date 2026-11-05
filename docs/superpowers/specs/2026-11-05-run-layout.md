# The Run page: a band for the robot, a canvas for the mission

Status: design, 5 Nov 2026, from the user's direction after the flow look landed. It changes the Run tab and the
system strip only: the mission files, the plan, the server and the Program tab stay as they are.

## 1. What the user asked for

- **Top: one band.** The robot and the 3D view side by side, the 3D as the window the robot looks into. The bubble
  condensed: the skill it uses and a short name of the task, not the long instruction, and what comes next.
- **Below: the mission as a canvas.** A large space that scales, pans and shows the layout the mission was built
  in, where the operator moves around, checks the tree and sees which node runs.
- **Mission control on Run only.** The picker, Start, Pause and Abort live on the Run page, not in the strip.
- **A slimmer strip.**

## 2. The strip (every tab)

One line, 40px (`--strip`), no second row:

| Part | What | When |
|---|---|---|
| connection dot + robot name | as today | always |
| state badge | as today | always |
| mission name | the running mission's name, humanised (`Night round`), ellipsis; its progress as the lime line along the strip's bottom edge | a mission is active, on every tab but Run |
| Abort | the stop glyph only, two taps as today | a mission is active, on every tab (Run too: it is the same button as the band's) |
| speed | the value, opens the speed panel as today | always |
| pad mode | as today | while a pad is in jog mode |

Gone from the strip: the step sentence, the step detail, the resource chips, the cycle count. The band has them.
Start's answer and the last error keep their place under the strip on the other tabs; on Run the bubble says them.

## 3. The band (Run, top)

A full-width panel, 200px high on a desktop (`--band`), two halves:

**Left: the robot and its bubble.** The robot figure (today's `.bot`, 120px high, its walk, freeze and fault
animations kept) stands at the left; the bubble beside it with the speech tail toward the robot. The bubble's
content by state:

| State | Line 1 (name, 20px 600) | Line 2 (mono) | Line 3 (muted) | Action |
|---|---|---|---|---|
| idle, no plan | `Which mission?` | – | – | the picker and Start, in the bubble |
| planning | `Planning` | the mission name | – | Abort |
| running | the node's name as the flow draws it (`To shelf`), the kind word in front in the kind colour (`Navigate to`) | the main argument (`to  bin_1/place`) | `next  Grip close` (the next node's name; `then done` at the end; `then again` before a loop) | Pause, Abort |
| paused | as running, amber | as running | `paused` | Resume, Abort |
| failed | the node's name, coral | the error, one line, ellipsis | the rule that applies (`retry ×2`, `replan from Pick`) | Retry / Run again, Abort |
| succeeded | `Done` | the mission name | `×n` cycles when more than one | Run again |
| aborted | `Stopped` | the node it stopped at | – | Run again |

The step count (`3/12`) and the cycle (`×2`) sit at the bubble's top right as small chips. The resource dots
(`base`, `hands`, `torso`) sit under the robot's feet, lit while held, as the strip's chips do today. The
instruction sentence is the bubble's tooltip and the node pop-up's text; it is not drawn in the band.

Pause and Abort are 48px buttons at the bubble's right end; the picker is a select of the mission names with
Start beside it, in the bubble's place while idle. The strip's Abort is the same two-tap control.

**Right: the window.** The 3D view (`viewer3d-run`, unchanged inside) fills the right half, framed as the robot's
window: the band's panel surface with an inset border and the robot's antenna light colour on its edge while the
mission runs. A button at its corner opens it full height over the canvas (`Enlarge`, a glyph); Escape or the same
button closes it. Phone: the window is a 96px thumbnail at the bubble's right that opens full screen on a tap.

Gone from the Run page: the phases floor under the robot (the canvas shows where the run is), the "Recent steps"
card (the done path on the canvas and the Log tab have it), the board's goal headline (the mission name is on the
canvas's Start node and in the strip on the other tabs).

## 4. The canvas (Run, below the band)

The rest of the window (`100vh - strip - band`, no page scroll on Run), the dotted grid, the mission as the
Program tab lays it out: the same `layoutGraph` call, the same boxes, lanes, repeat return wire and loop wire, drawn
in Run mode (no ports, no Fail terminals, no Retry nodes, the replan wire thin and coral, as the strip draws today).
Not a strip any more: the whole tree at once, blocks open.

- **Pan and zoom.** Drag the ground to pan (touch: one finger), the wheel scrolls, Ctrl+wheel and pinch zoom from
  0.4 to 1.5 around the pointer. The content layer takes one CSS transform; nodes keep their size in layout units.
- **Follow.** The camera keeps the running node centred and moves there with a 300ms ease when the step changes
  (none with reduced motion). A pan or zoom by the operator turns follow off; a `Follow` chip appears at the
  canvas's top right and a tap on it (or on the running node's step chip in the band) turns it on again and recentres.
- **Fit.** A `Fit` button beside it zooms the whole tree into view; the canvas opens fitted, then follows.
- **State on the nodes** as the flow look defines it: the running node lit in its kind colour with the lime ring and
  breathe, done nodes dimmed, the done path's wires lime, pending dotted, failed coral, paused amber, skipped dashed.
  The step-number badge stays.
- **Closed blocks.** None by default: an open tree is the point of the canvas. A box's chevron still closes it to
  a stack node (remembered per mission as today), for a mission too wide to read.
- **Click a node:** the pop-up as today (desktop) or the sheet (phone), with the instruction sentence, the args,
  the rule and the outcome. The pop-up follows its node under pan and zoom and closes on Escape or a tap on the
  ground.
- **List.** The step list stays as a `List` toggle beside Fit and Follow, for screen readers and a long mission on
  a phone; it replaces the canvas while on.
- **Idle:** the canvas shows the selected mission's last plan when one exists (as today), else the mission's steps
  from its file through the same builder, all pending, so the operator sees what Start will do.

## 5. Sizes and breakpoints

| Width | Band | Canvas |
|---|---|---|
| ≥ 1000px | 200px, robot + bubble left, window right | the rest of the window |
| 600 to 1000px | 180px, the window 40% | the rest |
| ≤ 600px (phone) | robot 80px, the bubble, the window a thumbnail; about 150px | the rest; pinch zoom; the bottom nav stays |

Touch targets 44px, buttons 48px, the band's name at 20px and the badges large text, as the design system says.

## 6. Code

- `index.html`: the strip loses `#mission-run` and `#resources`; `#view-run` becomes `.band` (robot, bubble,
  window) and `.canvas` (toolbar, `#plan-graph`, `#plan` list); the board's ids that app.js and the tests use keep
  their names where the element survives (`board-instruction` becomes the bubble's name line and so on; the spec's
  table above is the mapping).
- `app.js` `renderBoard`: fills the bubble from the table in section 3; the picker, Start, Pause, Abort and the act
  button keep their handlers. `renderMission` stops writing the step sentence and the resources into the strip.
- `graph-run.js`: the strip and its scroll code go; a `canvas` object holds `{x, y, scale, follow}`, the pointer
  and wheel handlers, `fit()` and `follow(node)`; `renderGraph` draws into the transformed layer and recentres when
  the running step changes and follow is on. The pop-up code is kept and positioned in content coordinates.
- `style.css`: `--strip: 40px`, `--band`, `.band`, `.bubble`, `.window`, `.canvas`, the toolbar chips; the board
  and the phases rules go.
- `viewer3d.js`: a `resize` call after the band's layout and on enlarge; nothing else.
- `DESIGN.md`: the Run page section rewritten.
- Tests: `test_static_files.py` as today, plus a new `test_page_ids.py` (every id app.js and graph-run.js read exists
  in index.html, read as text like the static test); a mock scenario for each bubble state.

## 7. Out of scope

Stored node positions (the layout stays the Program tab's), a minimap, editing from Run, the planner's "suggest a
fix" button, the other tabs.
