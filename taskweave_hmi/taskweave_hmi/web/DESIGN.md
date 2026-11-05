# taskweave HMI design system

The look of the operator interface (Run, Program, Robot, Teach, Record, Log). All values live as CSS custom
properties on `:root` in `style.css`. One theme, dark, so there is one theme to keep right. Components use the
tokens, never raw values; the only exception is the black video background.

## Direction: a technical panel

The P3 technical look of the reBot control page, made for a factory hall: a laptop at the cell, a phone in a gloved
hand, a screen read from two metres. A behaviour-flow editor is the reference for the plan graph and the editor: a
dark canvas, a flow from left to right, nodes whose coloured header carries the colour.

- **Navy ground, solid panels.** The page is P3 navy (`--surface-page`) with a faint blueprint grid and a blue glow
  at the top right; cards, the rail and the strip sit on it as solid panels with hairline borders. No shadows except
  under what floats (pop-ups, the step sheet).
- **Electric blue is action.** The P3 blue fills primary buttons and the selected 3D tool; a lighter blue marks the
  selected section and is the link colour. Nothing else is blue.
- **Colour means state, like a stack light.** Lime runs (also planning, a manual skill, a healthy link, a save);
  amber paused or to look at; coral stopped on a fault or near a limit; grey idle; finished as planned is neutral,
  not green, so lime means one thing: it runs. Never used for decoration.
- **Andon status.** The state is a solid badge in the state colour, in the strip on every tab and in the Run band;
  the running mission's progress is a lime line along the strip's bottom edge. Paused differs from running by its
  word and amber, and the band's robot freezes mid-stride.
- **Big, flat, legible.** Montserrat with tabular figures, 16px body, 48px main buttons, 44px minimum touch
  targets, borders at 3:1 on fields and buttons, two radii (4px controls, 8px panels), no gradients on components.

## Colour

Contrast was measured with the WCAG formula against the panel (`--surface-card`) unless noted. Text is at least
4.5:1; the state badges and the bubble's name are large text.

| Token | Value | Role |
| --- | --- | --- |
| `--surface-page` | `#03031c` | the ground (P3 navy), with `--grid` lines at 5% and a blue glow |
| `--surface-card` | `#07072e` | panels: cards, the rail, the strip, the 3D view's tools |
| `--surface-field` | `#0b0b3d` | inputs, secondary buttons, tracks, the segmented control's track |
| `--surface-hover` | `#11114a` | hover on rows, sections, buttons |
| `--border` | `#1c1d5a` | hairlines: panels, rows, dividers, the resource chips |
| `--border-strong` | `#5b5f9e` | field and button outlines (3.1:1 on the field surface) |
| `--text` | `#e9ebf5` | body text (16.5:1) |
| `--text-muted` | `#9a9ec8` | secondary text, labels (7.5:1; 7.1:1 on a field) |
| `--brand` | `#3d3dff` | fill of primary buttons and the pressed 3D tool (white on it 6.3:1) |
| `--brand-hover` | `#5a5aff` | primary button hover |
| `--brand-edge` | `#6b6bff` | primary button border |
| `--brand-text` | `#9b9bff` | links, the selected section and segment, markers (7.9:1) |
| `--brand-soft` | `#16164f` | brand tint: the selected section in the rail, the open mission in the list |
| `--on-brand` | `#ffffff` | text on `--brand` |
| `--logo` | `#ffffff` | the P3 logo |
| `--run-fill`, `--run`, `--ok` | `#dbff55` | lime: running, planning, a manual skill, the live link, a save; as a fill with `--on-run` (`#141a00`) on it, as text and rings 17:1 |
| `--done` | `#c5c9e6` | succeeded and done steps: neutral, not green (12:1) |
| `--warn` | `#ffc95c` | paused, jog mode, canceled, a save that asks first (12.8:1) |
| `--bad` | `#ff7f6a` | failed, aborted, errors, near a joint limit, danger buttons (7.9:1) |
| `--idle` | `#9a9ec8` | idle, pending |
| `--on-fill` | `#03031c` | navy text on a solid state fill (7.9:1 and up on every fill) and the glyph on a solid skill colour: the running or paused node, the card's tile (6.3 to 10.9:1) |
| `--kind-*` | drive `#7a8cff`, arm `#b394ff`, grip `#ff9a4d`, wait `#8a8ea8`, other `#2dd4bf` | skill colours in the plan graph, categorical only (6:1 and up): an action node's header, `--on-fill` on it |
| `--flow-rule` | `#c98a3a` | the header of a Retry or Replan rule node in the editor, `--on-fill` on it (5.9:1) |
| `--flow-end` | `#1d7a4c` | the Start and Done nodes, white (`--on-brand`) on it (5.4:1) |
| `--flow-fail` | `#8a1f2e` | a Fail terminal's header, white on it (9:1) |
| `--viewer-bg`, `--viewer-line`, `--viewer-ink`, `--viewer-tools` | the ground's navy, a hairline, body text, the panel at 82% | the 3D view |
| `--rail`, `--nav` | 92px, 60px | the section rail's width; the phone's bottom nav |
| `--strip` | 40px | the system strip: one line on every tab |
| `--band` | 200px (180px up to 1000px, 150px on a phone) | the Run band's height: the robot, its bubble, its window |
| `--spin-arrows` | ▲ ▼ in `#9a9ec8` | a number field's arrows (an SVG image: `--text-muted`'s value) |

Tints are mixed from the state colour with `color-mix()`: chips 11%, messages 9%, the bubble and status card 5 to 6%.

## Typography

Montserrat (the P3 brand face, geometric and wide) for everything; JetBrains Mono for ids, poses, values, the
resource chips and the log. Both come from Google Fonts with a system fallback (the cell network may have no
internet). `font-variant-numeric: tabular-nums` throughout, so live values do not jitter. `--cond` is an alias of
`--sans`: the condensed face went with the light theme; big figures get `letter-spacing: -.03em` instead.

| Token | Size | Use |
| --- | --- | --- |
| `--fs-xs` | 12px | joint names in small grids, the resource chips |
| `--fs-sm` | 13px | monospace meta, log, legend, the state badge, section eyebrows (`.sub`) |
| `--fs-s` | 14px | field labels, chips, small buttons, help text, the strip's words (the rail's section names are 13px) |
| `--fs-base` | 16px / 1.5 | body, inputs, buttons, card titles (`.label`, 600) |
| `--fs-md` | 18px | brand name, record stats |
| `--fs-lg` | 20px / 1.35 | current step instruction, the bubble's name |
| `--fs-xl` | 24px / 1.2 | `h2`, readout values |
| `--fs-2xl` | 30px / 1.12 | speed value, recording headline (26px on a phone) |

Weights: 400 body, 500 instruction, 600 titles and buttons, 700 badges, big figures and the brand name.

## Spacing

A 4px scale: `--sp-1` 4, `--sp-2` 8, `--sp-3` 12, `--sp-4` 16, `--sp-5` 20, `--sp-6` 24, `--sp-12` 48.
Card padding 16; gap inside cards 12; gap between cards 12 on a phone, 16 from 601px. Content width `--content`
760px, side gutter 16 (12 at 400px and below).

## Shape and depth

- Radius: `--r-s` 4px (buttons, inputs, rows, messages, the state badge), `--r-m` 8px (cards, the 3D view, the
  Teach hero and job tiles), `--r-xs` 2px (bar markers), `--r-pill` (chips, progress, limit bars). The graph's
  nodes have their own 6px (a header strip and a body, like a node editor's); its boxes 8px.
- Depth: `--shadow-1` is none (borders carry the edges); `--shadow-2` under what floats (pop-ups, the step sheet).
  The sticky top is the panel at 88% over a blur, so the page shows through it faintly.
- Focus: a 3px ring in `--focus` (brand text at 60%), offset 2px; inset on segments.

## Touch targets

`--tap` 48px for main buttons, Pause and Abort in the bubble, `--tap-min` 44px for inputs, selects and summaries,
`--tap-s` 36px for small and icon buttons on a laptop, raised to 44px on touch screens and at 600px and below.

## Components

- **Buttons** (`.btn`): secondary by default (field surface, strong border, text colour). `.primary`: brand fill,
  white text. `.danger`: red text and border on the card; `.danger.confirm` (the second press) is a solid red fill.
  Save on an existing location or stance and Capture on an existing pose or point ask first: "Tap again to overwrite"
  on an amber fill (`.btn.confirm`, 8 s), the change (mm, °) in a `.msg.warn` under it; the change is asked again at the
  second tap and shown again instead of saving when it differs.
  `.danger.solid`: red fill from the first press, for a stop that needs no second tap (the manual skill's Abort).
  `.small`: `--tap-s` high, 14px. `.ghost`: a text link in brand colour. Hover darkens or tints, active moves down
  1px, disabled is 45% opacity. `.icon`: a square small button (the error's ✕, the speed's − and +).
- **Inputs and selects**: field surface, strong border, 6px radius, 44px high; brand border on focus; red border and
  tint when invalid. Checkboxes 20px in the brand colour, inside a 44px-high label.
- **Number fields** (`input.num`, made by `numberField` in app.js): text fields, so a decimal is always shown with a
  dot (a browser's `type="number"` follows its own locale, a comma in German); a typed comma becomes a dot. On a
  laptop (`pointer: fine`) ▲ ▼ arrows in `--text-muted` (the `--spin-arrows` image, one per theme) appear at the right
  end on hover and focus and step the value; touch screens get none, as with `type="number"`.
- **Segmented control** (`.segmented`): a field-coloured track; the chosen option (`aria-pressed` or
  `aria-selected`) is raised on the card surface in brand text with a strong ring. `.choice` and `.teach-panes`
  wrap and share the width.
- **Shell**: the section rail (`.rail`, 92px, sticky, full height) at the left: the P3 mark and "taskweave" and
  the sections (`.tabs`, buttons: a 22px stroke icon over the name, 64px high, the selected one in brand text on
  `--brand-soft`, `aria-current="page"`): Run, Program, Robot, Teach, Record (hidden unless recording is set up),
  Log. The page beside it, the sticky top (`.top`: the system strip, manual skill strip) over the views. At 600px
  and below the rail is a bottom nav (60px, fixed, the selected section with a 3px brand rule on top) without the
  mark; the step sheet sits above it.
- **System strip** (`.mission`, `#mission`, the one bar on every tab, 40px, one line): the link dot and the robot's
  name (`.strip-robot`), the state as a solid badge (`#state`, uppercase 13px, 700; `planning` while the task handler
  prepares the plan), and while a mission is active (`.mission.active`) the mission's name, humanised, one line with
  an ellipsis (`.strip-mission`; not on the Run tab, `body.on-run`, where the band names it). The mission's progress is
  a 3px line along the strip's bottom edge in the state colour. Beside it, only while a mission runs or is paused,
  **Abort** (`#strip-abort`, a small `.danger` with the stop square alone): Abort is in reach on every tab. Two taps,
  one control with the bubble's Abort (a first tap on either arms both: solid red, "Tap again", 3 s; a second tap on
  either aborts). At the right Speed and the pad mode (only while it jogs an arm, up to 1000px). Not in the strip: the
  step, its words, the resources it holds, the cycle; the band has them. Mission control (the picker, Start, Pause,
  Resume) lives on the Run tab. Under the strip, small: Start's answer (`.msg`, idle only) and the last error (`.error`
  with a ✕ that hides that error until another one or the next start); on the Run tab the bubble says the error. On a
  phone: dot, badge, speed in one row (while a mission is active the robot's name gives way, so the row stays one
  line); Abort is the stop square alone at 44 × 44. A pad press's answer is a toast for a while (`.pad-notice`, fixed
  outside the top, taps pass through: bottom left of the page on a laptop, clear of the 3D view's tools; just under the
  top on a phone, clear of the step sheet and the Save bar), so the strip keeps its height. On a phone the error text,
  the speed message and the manual skill's text scroll in their own boxes (the error's ✕ stays), so the strip's and
  the manual skill's Abort stay in sight.
- **Run page** (`#view-run`, Run tab): the window under the strip with no page scroll. The band (`.band`, `--band`
  high) at the top, the canvas under it taking the rest.
- **Run band** (`.band`): full width, the panel surface (`--surface-card`) with a hairline under it; three columns,
  the robot, the bubble, the window. 200px on a desktop; from 1000px down 180px, the bubble 3 : 2 against the window;
  on a phone 150px with the robot at 80px and the window a 96px thumbnail. The robot is a little comic figure (the
  user's direction): a big head with a dark visor (`--bot-ink`) and light eyes, a small torso, and head, hands and feet
  floating apart from it on bendy limbs (round strokes whose curve changes as they swing, CSS `d`) and a long floppy
  antenna that lags the walk; two greys (`--bot`, `--bot-2`); its antenna tip and chest light are the state colour.
  It stands still at the left, 120px high, and its pose is the state: blinking when idle, walking (legs, arms, a bob,
  its shadow) while running, frozen mid-stride with sleepy eyes when paused, looking about with a blinking light
  while planning, arms up and three hops when done, slumped with crossed red eyes when failed, flat eyes when aborted.
  Reduced motion keeps the poses without the motion. Under its feet three dots (`#resources`: base, hands, torso), lit
  lime while the running step holds them.
- **Bubble** (`.bubble`, `#board`; a 2px border in the state colour at 45%, its tail toward the robot): three lines,
  each one line with an ellipsis, so the band never changes height. The text rule: the bubble reads by the node's name
  and colour, never by the instruction sentence (that is the bubble's tooltip and the node pop-up's text).
  Line 1, 20px 600: the node's name as the canvas draws it, the kind word in front in the kind colour when the name
  does not already say it; amber while paused, coral when failed; `Which mission?` while idle, `Planning` while the
  plan is made, `Done` and `Stopped` at the end. Line 2, mono: the main argument (`to  bin_1/place`, the two spaces
  kept), the error on a failure, the mission's name on `Planning` and `Done`, the node it stopped at on `Stopped`.
  Line 3, muted 13px: what comes next (`next  Grip close`, `then done`, `then again` before a loop), `paused`, or the
  rule that applies to a failed step (`retry ×2`, `replan from Pick`), and on `Done` the cycles (`×3`). At the top
  right two small chips: the step count (`3/12`, a tap on it turns Follow on) and the cycle (`×2`). At its right end
  the controls, 48px: idle, the mission picker (`#mission-select`) and the one button on (`#board-act`: Start, Start at
  step N, Resume, Retry step N, Run again; `#btn-start` stays in the page, hidden: the button is its wire); active,
  Pause (Resume takes its place while paused) and Abort (`.danger`, two taps, armed together with the strip's). Up to
  1000px the controls take a row under the words; on a phone the chips move to line 3's row and the picker and the
  button share a row, their words cut short. The error, when line 2 shows something else, is for screen readers.
- **Window** (`.window`): the 3D view (`#viewer3d-run`) framed as the robot's window: an inset border in the page
  colour, the state's lime on its edge while the mission runs. A button at its bottom right (`#window-enlarge`,
  `aria-pressed`) opens it over the canvas, under the band; the same button or Escape closes it. On a phone the
  window is the thumbnail and a tap anywhere on it enlarges it to fill the screen between the strip and the bottom nav.
- **Run canvas** (`.canvas`, under the band): a toolbar (the legend at the left; Graph / List, Fit and Follow at the
  right) over a viewport on the dotted grid, drawn by graph-run.js. The mission as the Program tab lays it out (the
  same boxes, lanes and loop wire), in Run mode: no ports, no Fail terminals, no rule nodes. Pan: drag the ground (one
  finger on touch) or the wheel; zoom: Ctrl+wheel or a pinch, 0.4 to 1.5 around the pointer; the graph sits on one
  layer that takes one CSS transform, nodes keep their size in layout units, the pop-up is never scaled and follows its
  node. **Follow** (a toolbar chip, `aria-pressed`, on at first) keeps the running node, else the start node, centred
  and moves there with a 300ms ease when the step changes (none with reduced motion); a pan or a zoom by the operator
  turns it off, and a tap on Follow or on the step chip in the bubble turns it on and recentres. **Fit** zooms the whole
  tree into view, from the top left when it is larger than the viewport, and turns Follow off. The first view of a plan
  is a fit; when a fit would go under 0.6 (a tree that fits only as tiny chips) it opens at scale 1 on the running
  node instead, following it. A resize keeps the camera, or recentres the followed node while Follow is on.
  **List** shows the step list in place of the canvas, for screen readers and a long mission on a phone. Idle, the
  canvas shows the picked mission's last plan, else its steps from its file, all pending.
- **Speed override** (`.speed`): in the status bar, a small button "Speed 60 %" (the value in Montserrat 700
  20px; amber with "TEST" while test mode caps it; greyed out without the task handler; on a phone the value only).
  It opens a panel under the bar (`aria-expanded`, focus to its first control; a click elsewhere or Escape closes
  it, and Escape still aborts a manual skill as everywhere): − / + around a range slider in brand colour, the presets as a segmented control, and the test mode toggle,
  amber while on. The handler's message on a clamped or refused request under the bar, for 8 s.
- **Manual skill** (`.manual`): a strip in the sticky top below the status bar, on every tab, only while a skill
  started from the HMI outside a mission runs (Drive here, Move here, Go here, a carry pose's Test). "Running
  <label> (<skill>) · <n> s" in 600 weight on a 5% `--run` tint with a 4px `--run` left rule (running green), one line
  with an ellipsis on a laptop, wrapping on a phone, and a small `.danger.solid` **Abort** on the right: one press,
  Escape does the same (not while typing in a field). The button that started the skill then says "aborted" as a
  `.msg.warn`.
- **Cards** (`.card`): card surface, hairline border, 10px radius, 16px padding, a grid with 12px gaps.
  `details.card` opens from its whole title row.
- **Status card** (`.card.status`): the andon panel of a recording (Record tab). A 10px rail and a solid badge
  (`.chip` inside it) in the state colour, a 5% tint on the card, the title as the headline.
- **Program tab: the missions** (`.missions-md`; the tab is called Program, the things it lists are missions): up to 1599px the list or the open mission (← Missions back): the flow wants the width. From 1600px the list
  (300px, sticky under the top, its own scroll) beside the open mission, or a hint while none is open; the open one is
  marked (`.open`, `--brand-soft`, `aria-current`). A search field filters by name and goal ("No mission matches the
  search." when none does; cleared after a save); only the list scrolls. With unsaved edits the first tap on another
  mission or on New mission says "Unsaved changes in <name>: tap again to drop them" in the list's status line
  (`#mission-note`, amber, `role="status"`, 3 s); a second tap on the same one goes on. Focus goes to the back link
  when the list gives way to the editor, to the list's title when the editor closes.
- **Program tab: the editor** (`.ed`, editor.js on graph-core.js): the open mission as a flow from left to right. The
  head row: ← Missions (narrow screens), the name with "● Unsaved changes" in amber beside it, Undo, Redo, Check, Save;
  under it the problems (red) and the save message (lime, with "Undo this save"). The work area: the canvas
  (`.ed-canvas`, scrolls both ways, not the page; a faint panel tint over the grid, 480px high at least, never taller
  than the window; it opens at Start, or scrolled to the selection) and the properties panel at its right (`.ed-panel`,
  380px, sticky; under the canvas up to 1000px). The panel shows the mission (name, goal, loop, defaults, + Add step,
  Delete) while nothing is selected, else the selected step's form (the step card: "Step n" is its title, not a
  control, so no key folds it; the instruction its first field; an include step gets a brand-soft banner "Linked: the
  steps of <mission> run here …" with Open to edit) or a block's form (schema v2: instruction, id, a repeat's rule and
  max passes, a parallel's join, "run only when"), with the actions under it: ↑ ↓, Duplicate, Group / Repeat / In
  parallel (only when the running core's schema has blocks), Unwrap, Remove. A lane of a parallel block goes only with
  its block: Unwrap puts every lane's steps in the block's place ("+" on its title bar adds one).
  The canvas (the flow look, see Mission graph for the shared parts): a Start node at the left, the nodes 220 × 64
  (168 × 52 on a phone) 64px apart (56 on a phone), Done at the right, or with a loop a dashed wire from the last node
  back to Start along the bottom with its words (`forever`, `until x`, `×5`). A node (`.pg-node.fl-node`): the header
  (24px) in its colour with the name (the step's id humanised, else its skill: `To shelf`, `Move`; an include's mission:
  `Pick frame`) and the kind word (`Action`, `Mission`, `Group`, `Repeat`, `Parallel`, `Lane`), the body (40px) with at
  most two fields in mono (the main argument: `to bin_1/place`, `hands all` + `action open`, `seconds 2`; an include's
  `with` values; a closed block's `steps 4` and its rule; none on a phone). The instruction sentence is not on the
  canvas: it is the node's tooltip and the panel's first field. Ports: in on the left edge, done and fail on the right
  (10px dots on the flow line and 16px under it; their words `done`, `fail` inside the body at its right edge, 10px;
  the fail port coral, filled when the step has its own rule, a 44px target). A closed block is a stack node (cards
  behind it), a collapsed include a Mission node; the chevron in the header opens it and selects it. An open block is
  a box (`.fl-box`: the field surface at 60%, 1px border, 8px radius, 16px padding) with a 28px title bar
  (`.pg-band.fl-title`: a 4px strip in its colour, the name, which selects the block, the kind word, a repeat's rule,
  "+" on a parallel block, the chevron at the right, which closes it and selects it); a repeat's return wire runs
  from its right edge to its left along its bottom, no words; a parallel block holds its lanes as rows (each a thinner
  box with a tinted 24px title bar), its input fans to every lane's first node, every lane's end joins into the port
  on the box's right edge; an opened include is a box like a group with its steps at 70%, not editable.
  The failure routes run under the flow, from the fail port: abort (the mission default too) a Fail terminal
  (120 × 40, `--flow-fail` header, its retries `retry ×2` as field) under its node, in a row under the lowest thing in its column (its
  node, or a tall block beside it), so the flow line stays straight and the fail wire short (moved right where two
  would overlap; a node in a parallel lane has its terminal in a row
  of its own lane, under its node, so no fail wire crosses another lane); retry a Retry rule node (100 × 36, `--flow-rule`,
  `×n`) under the node, its wire back into the node's input; skip a dashed coral wire into the next node's input;
  replan a coral wire back along the bottom (one lane each, under the terminals' row) into the input of the step it
  restarts from, a Replan rule node (`≤ n` when max_replans is set) on its middle; a replan without a target or to
  itself a Replan node under the node, like a retry. Terminals, rule nodes and "+" spots are not tab stops (the
  keyboard has Shift+A, F and the step form's rules). A click on the fail port, a terminal or a rule node opens the rule
  menu (`.ed-menu`: mission default, retry, replan, skip, abort; above the node when the graph has no room under it;
  the canvas is not redrawn while it or the palette is open); a drag from the fail port draws a coral wire to the
  step a replan restarts from (the target gets a dashed red outline; only a step node takes it, not a closed block or
  an include, since replan_from names a step of the plan; a touch that turns into a scroll only cleans up). Linked
  nodes and closed blocks have no failure route; an open include box none either.
  The selected node has a 2px brand outline; a problem a Check or Save names is a red left bar on its body and a red
  border, and a red field in the panel. "+" spots (`.ed-spot`, 24px dashed rings, a 44px target) sit on every wire's
  middle and at the end of every open box, on its flow line; one opens the palette (`.ed-palette`: a search over the
  skills, the other missions and the blocks; ↑ ↓ walk the options, the active one marked with `aria-selected` and the
  hover tint, the focused one with the focus ring; Enter in the search takes the first) at that place; while a node is
  dragged they turn solid brand and the nearest lights up as the drop. Keys on the canvas: Shift+A adds after the
  selection (Tab walks the nodes; on a block's or lane's title bar it adds at the start inside it, so an empty group
  or lane gets its first step; with nothing selected at the end of the mission), F opens the focused or selected
  step's failure-rule menu at its fail port (Escape closes it and gives the focus back to the node), Delete removes, Ctrl+D duplicates, Ctrl+↑ / ↓ move, Ctrl+Z / Ctrl+Y undo and redo,
  Escape clears; shift-click extends a selection along one list, ctrl-click toggles, a drag on empty canvas draws a box;
  a node is dragged by its body, a block by its title bar. Every edit is an op on a path in the mission file (insert,
  remove, move, wrap, unwrap, set); Save sends the ops since the open to `/api/missions/<name>/edit`, which applies them
  on the file's own tree (comments kept) and validates the whole; nothing is written when it refuses. When the file
  changed since it was opened (a 409), "Reload from file" beside the problem opens it as it is saved now and drops the
  edits here.
  A structural edit (insert, remove, move, wrap, unwrap, duplicate) keeps every `replan_from` on the step it named:
  a place name `s<n>` (a top-level step without an id, numbered from 1) follows its step to its new number, or to a
  new id when the step went into a block; a `replan_from` whose step was removed is cleared with its `on_failure:
  replan` (the mission default applies). These are set ops in the same undo entry, sent at Save with the rest, and
  the save message line says what changed in amber ("replan_from of 'carry' is s4 now (was s3).", "The replan
  target of 'd' was removed: on failure it does the mission default (abort) now."), as does a note on the step's
  form. The keyboard stays on the graph after Delete, a move, Undo and Redo: on the selection, else the node now
  at the change (the next one, else the one before, else the box that holds it), scrolled into sight. The Run graph
  and the editor give their arrow heads their own ids (`pg-run-head-*`, `pg-ed-head-*`).
- **Teach in 3D** (`.teach-grid`, teach3d.js): the cell view (4 : 3, at most half the screen high) above the lists up
  to 1179px; from 1180px 7 : 5, the lists at the left and the view (16 : 13, sticky, never taller than the window) at
  the right. Markers: locations (an arrow), stances (a fainter arrow in a ring labelled "<object> · <variant>", from `/api/scene` stances, Teach only; a
  symmetric object's on the side the robot works it from), grasp points
  (a gripper outline per placed object), placed objects (a triad). A pick opens the entry in its pane with a brand
  line on top, "At <object> · saves to <file>" (`.pick-where`); a placed object opens a card under the view
  (`.teach-pick`) with its stances and grasp points as buttons and two primary ones, "Teach a stance here" and "Teach
  a grasp point here". Stance markers on one spot (within 5 cm: an override where a neighbour's type stance is)
  open the same card as "Stances on this spot" with one button per stance. One `.pick-where` in the tab, on the
  entry picked last. Opening an entry highlights and focuses its marker (and follows its "at" choice, which also takes the note away);
  the points event redraws the markers and the object card. The teach-here buttons are off while a mission runs.
- **Teach by task** (`.teach-flow`, teachtask.js): the Teach tab opens on one big brand button (`.teach-hero`, 72px,
  64 on a phone, a plus and "Teach") over the lists. It opens a card in their place: four big job tiles (`.flow-job`,
  2 x 2 on every width, a 44px line icon in brand, the word big and a short hint): Stand, Grasp, Pose, Spot. Stand and
  Grasp ask for the object next: the placed ones nearest first with their distance (`.flow-object`, 6 and "All n"),
  or a click on it in the 3D view (which then belongs to the flow); Stand also offers "No object: a spot in the
  cell". Then one guided screen, its job and object as chips in the head (the object highlighted and turned to in
  3D, its distance live): the pad strip (`.flow-pad`, lime tint, amber with "switch to drive" / "switch to an arm"
  when the pad is in the other mode), the live check (`.flow-reach`: a stance's "Reaches n of m" over the type's
  grasp points with an L / R tick or cross each, lime when all, amber when some; `.flow-hand`: a grasp's or pose's
  hands as toggles, a grasp's with each hand's distance from the object, the near ones on), the name (a free one
  suggested: stand_n, pick_n, pose_n, spot_n; "exists: Save overwrites" in amber), what Capture took (amber "Moved
  since: capture again" when the robot moved 5 mm or 0.5° since: the base, each saved hand's TCP and its approach and
  closing, a pose's joints; checked again at the Save tap, Save off then), and Capture / Save as 56px buttons. The live
  check pauses while a mission runs ("Paused while a mission runs"; a capture is refused then anyway) and stops, with
  the pad target cleared, when the tab is left; Escape closes the flow.
  Saved: a lime card with a tick, where and into which file, then Preview (dry run) / Go here / Undo and Teach
  another / Done. Saves go through the lists' endpoints (teach node, overwrite confirm, Undo); a pad teach press for
  this grasp or pose shows as "Taught by the pad".
- **Robot dashboard** (`.robot-grid`): up to 1179px one column, 3D view, base, grippers, joints, settings. From
  1180px 7 : 5 up to 1600px wide: the base over the joints at the left, the 3D view (16 : 11) over the grippers at the
  right (sticky under the top), the settings full width under both. The base as a readout (`.readout`: x, y, yaw in
  Montserrat 600 24px over muted 13px names), the nearest placed object under it in muted monospace, or in
  red why there is no pose.
- **Torso line** (side card and step sheet, a `torso` row after the target): what the carry's torso look-ahead
  planned for the step (`data.torso_plan` of a finished carry step whose window holds it, and the step's own
  `data.torso`): "Torso 0.35 m · planned change at this step (from 0.494)", "Torso 0.494 m · kept (plan)", "chosen per
  step (cut)" when the plan did not cover the hold, ", replanned: …" when the real state differed, and a certain
  infeasible plan's message in `--bad`. No row without a report.
- **Step sheet** (phone, `.pg-sheet`): a tap on a step's node (also the running one) opens a bottom sheet over a
  scrim (`.pg-scrim`, 45% ink; under the status bar, so the strip's Abort stays in reach, and the bottom nav): a 44 × 5
  handle, the step's tile (52px) with its state and number over the instruction (20px, 600), the step's progress
  while it runs, the pop-up's rows (command, target, locks, on failure, status; 96px labels), each note in an amber
  box (`.pg-sheet-note`), and a full-width Close (48px). A non-modal dialog (`role="dialog"`, labelled by the title):
  focus goes to Close when it opens (idle too, where the tap also picks the start step: Close keeps it, a second tap
  on the node unpicks it), Tab stays on Close, and focus goes back to the node when it closes (Close, the scrim,
  Escape or the node again). Escape does not close it while a manual skill runs: then it aborts the skill, as
  everywhere. Rebuilt every second, it keeps its scroll place. As high as the room between the status bar and the nav,
  at least 160px; it scrolls inside.
- **Lists and rows**: mission rows (`.list li`) with a brand left rule (red when invalid); plan steps (`.steps`) with a
  numbered dot: a neutral ✓ when done, red when it failed, ringed green while running; points, poses and recordings (`.point`)
  open into an editor, the pad's target gets a brand inset rule. The step form in the editor's
  panel (`.edit-step.ed-card`, from the page's step template) is the step's fields without a frame: "Step n" as its
  title, the instruction, the skill, the arguments, the rules; the graph moves, copies and removes steps. A field
  Check or Save names gets a red rule and its problem under it until it is edited.
- **Step arguments** (`.args`, made by `argField` in app.js from the skill's schema): each argument a `.field` with
  its label (unit in brackets: "offset (m)", "turn (°)", "speed (0–1)"; the schema's description as the tooltip) and,
  for structured ones, an "edit as text" ghost link at the right of the label row. Nested objects are indented on a
  2px `--border` rule (`.arg-nest`), their fields in a grid of 200px columns (`.arg-fields`; vectors, variants, hands
  and lists take the full row); four or more empty optional fields fold under a muted "more (…)" summary. A oneOf of
  objects is a `.segmented.choice` of the variants' titles above the chosen one's fields. Vectors are a `.grid3` of
  number fields labelled "x (m)", "y (m)", "z (m)". Hands are 20px checkboxes per hand, a muted "or", and the rule
  words as a segmented control. An object marked as a group (Motion (advanced)) is a muted `details` summary, open
  when it has a value. Text fallbacks are monospace textareas.
- **Move preview** (`.step-preview`): under a move step's arguments a small secondary **Preview** button (a dry run:
  nothing moves), off with a muted "No preview: …" beside it while a mission or a manual skill runs or an argument is
  a placeholder. The answer stays until the step is edited: a `.msg` with the skill's message over label / value rows
  (`.preview-goal`, monospace 13px, labels muted: base, torso, each joint group in ° or m, carrying, on the way; the
  joint names as the tooltip), or a `.msg.bad` line with the outcome and message, as the step's problems.
- **Chips** (`.chip`): a dot and the word, in the state colour on an 11% tint. Each state sets `--c` (text), `--fill`
  and `--ink` (for the solid badge).
- **Details and summary**: summaries in brand text, 44px high; nested ones inside a point are muted and monospace.
- **Messages** (`.msg`, `.msg.bad`, `.msg.warn`, `.error`): a 4px left rule in green, red or amber (canceled,
  aborted by the operator) on a 9% tint, body text colour.
- **Last save** (`.saved-note`, Teach tab): a `.msg` under the entry just saved or taught (from here or the pad): what,
  at which placed object and how far the robot was from it, the file, and a small **Undo** (until the next save).
  Which placed object a teach uses shows before it as `.at-hint` (muted; red when another one is nearer).
- **Joint and limit bars** (`.joint-row`, `.limit-bar`): a pill track with a 4px brand marker at `--at`; within 10° of
  a limit the track, marker, name and value turn red. On a laptop one row per joint (name, bar, value, −, target, +;
  gripper rows put Open and Close under the last three columns). On a phone the row wraps: name and value, the bar
  full width, then the controls at 44px.
- **3D view** (`.viewer3d`): a full-width viewport card, 16:10 on a laptop and 4:3 at 600px and below, dark in both
  themes, 10px radius; a `canvas` child fills it. `.viewer3d-tools` is an overlay toolbar top right for icon buttons
  (`--tap-s` square, light icons; `aria-pressed="true"` in brand fill).
- **Mission graph** (`.pg`, graph-core.js and graph-run.js): the plan as a flow from left to right, laptop and phone
  alike, the look of a behaviour-flow editor: a Start node at the left (`--flow-end`), the nodes, Done at the right
  (`--flow-end`), or with a loop a dashed wire from the last node back to Start along the bottom, its words on its
  middle (`forever`, `until x`, `×5`: the only words on a wire). The layout is automatic and the same on every draw (a
  structured program has one picture): a sequence is a row of nodes on one flow line. A node (`.fl-node`) is a dark
  body (`--surface-card`, 1px `--border`, 6px radius) with a header strip in its colour: an action by what moves
  (`--kind-*`, navy text), an include and a group in `--brand` (white), a repeat and a parallel block in `--warn`
  (their lanes a 60% tint), the name left (600, 14px) and the kind word right (11px, 80%; none where it says the name
  again). The text rule: a node reads by its name and colour; the instruction sentence is its tooltip, the editor
  panel's first field and the pop-up's text, never on the canvas; no counts in headers, no "click a step". Wires are
  2px bezier S-curves between the ports (a long one between two levels runs level and bends before its end). Blocks
  are boxes with a title bar (see the editor); a closed one is a stack node, cards behind it, done/total in its body
  filled left to right to its share done; the box with the running step is always open, the others as the operator
  left them (`tw-plan-bands`), else the one with the failed or the start step. The canvas keeps the dotted grid; the
  colour is in the headers and the state, nothing else.
  The Run tab (graph-run.js) draws the simplified flow on the canvas: nodes 160 × 48 (header 22; 132 × 44 on a phone), 36px apart,
  no ports, no terminals, no rule nodes, headers with the name and the step's number at the right (mono 10px; a box's
  title bar keeps its kind word and a repeat's rule). The only field is the running node's main argument (none on a phone). State: the running node's
  whole body takes its colour (navy text) with a lime ring and the breathing glow; paused the same with an amber ring;
  done keeps its header, its body's ground dimmed toward the page (70%; the text keeps its colour), a neutral tick on its top left corner; pending as drawn; failed a coral outline and a
  ✕; timeout and canceled amber; skipped dashed; the picked start step a dashed brand outline. Wires: the run so far
  lime, the one into the running node flowing lime, the way ahead dotted; a running box's outline lime, a failed one
  coral. A replan is a thin coral wire under the nodes into the step it replans from. Blocks are open: a box's chevron
  closes it to a stack node (remembered per mission), for a mission too wide to read; the box with the running step
  stays open. Nodes and title bars are buttons in run order, each title bar before its steps.
  The resources a step locks are in its pop-up and its tooltip, a box's in its title bar's tooltip. A click on a node
  opens its details in a 320px pop-up beside it (`.pg-pop`, right if it fits, else left, else under it, in the
  viewport's coordinates so it follows its node under pan and zoom; Close, Escape, a tap on the ground or the node
  again close it); at 600px and below a bottom
  sheet (`.pg-sheet`). Under the canvas only the replan key, when a replan wire is drawn: the nodes say what they do, the icons live in the pop-up. It reads `--g-*`
  tokens mapped onto the global ones. The Run graph is rebuilt on every status update only while the Run tab is shown;
  the tab's button draws it again when it comes back.

## Icons and imagery

The P3 logo inline at the top of the rail (takes `--logo`). The rail's sections, the plan graph's skills
(`icons.js`) and the Teach jobs are inline SVG line icons with a 2px stroke in `currentColor`; the 3D view toolbar
expects the same at 1.75px. The editor panel's ↑ ↓ and the marks ✓ ! ↻ are still Unicode. No photos; one illustration, the Run
band's robot (inline SVG in index.html, flat greys and the state colour).
