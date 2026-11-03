# taskweave HMI design system

The look of the operator interface (Run, Missions, Robot, Teach, Record, Log). All values live as CSS custom
properties on `:root` in `style.css` and are redefined for dark mode under `prefers-color-scheme: dark`. Components
use the tokens, never raw values; the only exceptions are the plan graph's own edge colours and the black video
background.

## Direction: steel and signal

An operator panel, not a marketing page. It is used on a laptop and on phones in a factory hall: bright light,
gloves, standing at the robot. So:

- **One brand colour, used for action.** The P3 brand blue `#0000ff` is the logo, primary buttons, links and the
  selected section. Nothing else is blue.
- **Steel neutrals.** Cool grey-blue surfaces and borders that recede, so the state colours stand out.
- **Colour means state, like a stack light.** Green running (also planning, a manual skill), amber paused or to look
  at, red stopped on a fault or near a limit, grey idle; finished as planned is neutral (dark slate), not green, so
  green means one thing: it runs. The same green marks a healthy link and a save. Never used for decoration.
- **Andon status.** The status bar has a rail and a solid badge in the state colour and a light tint of it while a
  mission runs. Paused also gets a striped rail, so it differs from running by more than hue.
- **Big, flat, legible.** Barlow (a signage face) with tabular figures, 16px body, 48px main buttons, 44px minimum
  touch targets on phones, borders at 3:1 on fields and buttons, no glass, no gradients.

This direction comes from the existing P3 look of the HMI and the rules of industrial HMIs (andon boards, state
colours with one meaning each); it was not derived from a survey of other sites.

## Colour

Contrast was measured with the WCAG formula. Text is at least 4.5:1 on its surface; the state badges and the status
headline are large text and clear 3:1 by a wide margin.

| Token | Light | Dark | Role |
| --- | --- | --- | --- |
| `--surface-page` | `#edf0f4` | `#0b0d17` | page background |
| `--surface-card` | `#ffffff` | `#141827` | cards, status bar, rail |
| `--surface-field` | `#f4f6f9` | `#0f121f` | inputs, secondary buttons, tracks |
| `--surface-hover` | `#e6e9f0` | `#1d2236` | hover on rows, sections, buttons |
| `--border` | `#d5d9e2` | `#272c42` | hairlines: cards, rows, dividers |
| `--border-strong` | `#8d93a6` | `#5d6485` | field and button outlines (3:1 on their surface) |
| `--text` | `#0d1026` | `#e9ebf5` | body text (18.8:1 / 14.9:1 on card) |
| `--text-muted` | `#545a70` | `#a3a8bf` | secondary text, labels (6.8:1 / 7.5:1 on card) |
| `--brand` | `#0000ff` | `#3d3dff` | fill of primary buttons |
| `--brand-hover` | `#0000c4` | `#5252ff` | primary button hover |
| `--brand-edge` | `#0000ff` | `#6b6bff` | primary button border (lifts it off the dark card) |
| `--brand-text` | `#0000ff` | `#9494ff` | links, selected section and segment, markers (8.6:1 / 6.7:1) |
| `--brand-soft` | `#e8e8ff` | `#1d1f4d` | brand tint: the selected section in the rail (its text 7.3:1 / 5.5:1) |
| `--on-brand` | `#ffffff` | `#ffffff` | text on `--brand` (8.6:1 / 6.3:1) |
| `--logo` | `#0000ff` | `#ffffff` | the P3 logo |
| `--run` | `#0a7a4a` | `#45c98f` | running, planning, a manual skill: badge, rails, the running step (5.4:1 / 8.5:1) |
| `--ok` | `#0a7a4a` | `#45c98f` | healthy: connected, saved (the same green as running) |
| `--done` | `#2e3447` | `#c9cddb` | succeeded and done steps and phases: neutral, not green (12.4:1 on card) |
| `--warn` | `#9a5200` | `#f2a64f` | paused, jog mode, canceled |
| `--bad` | `#c0112d` | `#ff7385` | failed, aborted, errors, near a joint limit, danger buttons |
| `--idle` | `#5c6177` | `#8e93ab` | idle, pending |
| `--rail`, `--nav` | 92px, 60px | | the section rail's width; the phone's bottom nav height |
| `--on-fill` | `#ffffff` | `#0b0d17` | text on a solid state fill (5.4 to 9.5:1) |
| `--kind-*` | drive, arm, grip, wait, other | lighter in dark | skill colours in the plan graph, categorical only |
| `--viewer-bg` | `#161a2a` | `#080a12` | 3D view background, dark in both themes |
| `--viewer-line`, `--viewer-ink`, `--viewer-tools` | | | 3D view border, text and toolbar backing |
| `--spin-arrows` | ▲ ▼ in `#545a70` | in `#a3a8bf` | a number field's arrows (an SVG image: `--text-muted`'s value) |

Tints are mixed from the state colour with `color-mix()`: chips 11%, messages 9%, the mission bar and status card 5%.

## Typography

Barlow for everything, Barlow Semi Condensed for state words, step numbers and figures that need to be narrow, a
system monospace for ids, poses and the log. Both Barlow faces come from Google Fonts with a system fallback (the cell
network may have no internet). `font-variant-numeric: tabular-nums` throughout, so live values do not jitter.

| Token | Size | Use |
| --- | --- | --- |
| `--fs-xs` | 12px | joint names in small grids |
| `--fs-sm` | 13px | monospace meta, log, legend, section eyebrows (`.sub`, uppercase, 0.06em) |
| `--fs-s` | 14px | field labels, chips, small buttons, help text (the rail's section names are `--fs-sm`, 13px) |
| `--fs-base` | 16px / 1.5 | body, inputs, buttons, card titles (`.label`, 600) |
| `--fs-md` | 18px | brand name, state badge (700, uppercase), record stats |
| `--fs-lg` | 20px / 1.35 | current step instruction |
| `--fs-xl` | 24px / 1.2 | `h2` |
| `--fs-2xl` | 30px / 1.12 | speed value, recording headline (26px on a phone), the board's "/ N" (`--fs-xl` on a phone) |
| `--fs-3xl` | 36px | the status board's state word (30px on a phone) |
| `--fs-4xl` | 60px | the status board's step number (a 44px literal on a phone, its only use) |

Weights: 400 body, 500 instruction, 600 titles and buttons, 700 badges and the brand name.

## Spacing

A 4px scale: `--sp-1` 4, `--sp-2` 8, `--sp-3` 12, `--sp-4` 16, `--sp-5` 20, `--sp-6` 24, `--sp-8` 32, `--sp-12` 48.
Card padding 16; gap inside cards 12; gap between cards 12 on a phone, 16 from 601px. Content width `--content`
760px, side gutter 16 (12 at 400px and below).

## Shape and depth

- Radius: `--r-xs` 3px (bar markers), `--r-s` 6px (buttons, inputs, rows, messages), `--r-m` 10px (cards, 3D view),
  `--r-pill` (chips, progress, limit bars).
- Shadows: `--shadow-1` a faint lift on cards (none in dark, where borders carry the edge), `--shadow-2` under the
  sticky status bar.
- Focus: a 3px ring in `--focus` (brand text at 55%), offset 2px; inset on segments.

## Touch targets

`--tap` 48px for main buttons, `--tap-min` 44px for inputs, selects and summaries, `--tap-s` 36px for small and icon
buttons on a laptop, raised to 44px on touch screens and at 600px and below.

## Components

- **Buttons** (`.btn`): secondary by default (field surface, strong border, text colour). `.primary`: brand fill,
  white text. `.danger`: red text and border on the card; `.danger.confirm` (the second press) is a solid red fill.
  Save on an existing location or stance and Capture on an existing pose or point ask first: "Tap again to overwrite"
  on an amber fill (`.btn.confirm`, 8 s), the change (mm, °) in a `.msg.warn` under it; the change is asked again at the
  second tap and shown again instead of saving when it differs.
  `.danger.solid`: red fill from the first press, for a stop that needs no second tap (the manual skill's Abort).
  `.small`: `--tap-s` high, 14px. `.ghost`: a text link in brand colour. Hover darkens or tints, active moves down
  1px, disabled is 45% opacity. `.icon`: a square small button for ↑ ↓ ⧉ ✕; a step's ✕ asks first, as a solid red
  "Remove?" (`.icon.confirm`, 3 s).
- **Inputs and selects**: field surface, strong border, 6px radius, 44px high; brand border on focus; red border and
  tint when invalid. Checkboxes 20px in the brand colour, inside a 44px-high label.
- **Number fields** (`input.num`, made by `numberField` in app.js): text fields, so a decimal is always shown with a
  dot (a browser's `type="number"` follows its own locale, a comma in German); a typed comma becomes a dot. On a
  laptop (`pointer: fine`) ▲ ▼ arrows in `--text-muted` (the `--spin-arrows` image, one per theme) appear at the right
  end on hover and focus and step the value; touch screens get none, as with `type="number"`.
- **Segmented control** (`.segmented`): a field-coloured track; the chosen option (`aria-pressed` or
  `aria-selected`) is raised on the card surface in brand text with a strong ring. `.choice` and `.teach-panes`
  wrap and share the width.
- **Shell**: the section rail (`.rail`, 92px, sticky, full height) at the left: the P3 mark and "taskweave", the
  sections (`.tabs`, buttons: a 22px stroke icon over the name, 64px high, the selected one in brand text on
  `--brand-soft`, `aria-current="page"`), the robot's name at the foot. The page beside it, the sticky top (`.top`: status bar, manual skill
  strip) over the views. At 600px and below the rail is a bottom nav (60px, fixed, the selected section with a 3px
  brand rule on top) without mark and robot; the step sheet sits above it.
- **Status bar** (`.mission`, the one bar on every tab, 76px): the mission, then Speed, Pause / Abort and the pad
  mode and connection dot (`.sys`) at the right. On the Run tab, while a mission is active, it keeps only the badge
  and Pause / Abort, and its error hides (the status board shows both); idle it keeps the picker and Start everywhere. A 4px rail in the state colour (striped amber when paused) and the
  state as a solid badge (`#state`, uppercase Barlow Semi Condensed 14px; `planning` while the task handler prepares
  the plan). While a mission is active (`.mission.active`, a 5% state tint): the goal (18px, 600) with plan id and
  cycle right, under it "step n/N" and the step's instruction, one line with an ellipsis, then skill, status, seconds
  and attempt in muted monospace, a 4px progress bar of the step, and small Pause / Resume and Abort (`.danger`, two
  taps) on the right. Idle: badge, the mission picker (up to 520px) and a small primary Start. Under the bar, small:
  Start's answer (`.msg`, idle only) and the last error (`.error` with a ✕ that hides that error until another one or
  the next start). From 601 to 1000px: badge, step n/N, Speed, Pause, Abort in one row, the instruction under it, no
  goal; the pad mode shows only while it jogs an arm (also on a phone). On a phone: badge, step n/N, the speed and the
  connection in one row, the instruction and progress under it, then Pause and Abort side by side, full width; idle:
  badge, speed, then picker and Start. A pad press's answer is a toast for a while (`.pad-notice`, fixed outside the
  top, taps pass through: bottom left of the page on a laptop, clear of the 3D view's tools; just under the top on a
  phone, clear of the step sheet and the Save bar), so the bar keeps its height. On a phone the error text, the speed
  message and the manual skill's text scroll in their own boxes (the error's ✕ stays), so both Aborts stay in sight.
- **Run workspace** (`.run-grid`, Run tab): up to 1179px one column, board, 3D view, plan, recent steps. From 1180px
  two columns, 7 : 5 with 20px gaps and the page up to 1600px wide: the board over the plan at the left, the 3D view
  (16 : 11) over the recent steps at the right, sticky under the top (`--top-h`, the top's measured height) and never
  taller than the window (it scrolls on its own on a short screen).
- **Recent steps** (`.recent`): the last six finished steps of this run (after `status.run_from_seq`: the bridge arms
  a run when the state turns running and starts it at its first step event, so a restart of the same mission is a new
  run and a start refused while planning is none; "Preparing the plan." while a new run plans), newest first, one row
  each: a mark (✓ in `--done`, ↻ in `--warn` for a step tried again, ! in `--warn` when canceled, else in `--bad`; its
  meaning as hidden text), the instruction with the command and message in muted 13px under it, and the time on the
  run's clock in muted monospace.
- **Status board** (`.board`, Run tab): the andon panel of the run, read from a few metres. A 10px rail in the state
  colour (striped amber when paused) on a 5% tint; the state word (Barlow Semi Condensed 36px, caps, in the state's
  text colour: "done" for succeeded), the mission and cycle; the step number at 60px with "/ N", the instruction at
  24px (two lines, three on a phone), and under it the command, status, seconds and attempt. Planning, idle ("26 steps ·
  Ready to start", or the chosen start step), done, failed (the failed step, from the outcomes when the status has no
  step) and aborted each have their own words. The last error (not after a success) in a red-ruled panel, with how to go on once it stopped. After a run the board
  keeps showing that run (and names a different picked mission as next) while the plan below previews the pick. Under
  it the phases: the plan graph's bands as one bar each, as wide as their steps, the done part neutral, the current one ringed
  in the state colour, name and done/total under it (done in `--done`, neutral). On the Run tab the mission bar keeps only the badge and the
  controls (`body.on-run`), so the words are not shown twice, and its error hides (the board shows it).
- **Speed override** (`.speed`): in the status bar, a small button "Speed 60 %" (the value in Barlow Semi Condensed
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
- **Missions** (`.missions-md`): up to 1179px the list or the open mission (← Missions back). From 1180px the list
  (300px, sticky under the top, its own scroll) beside the open mission, or a hint while none is open; the open one is
  marked (`.open`, `--brand-soft`, `aria-current`). A search field filters by name and goal ("No mission matches the
  search." when none does; cleared after a save); only the list scrolls. With unsaved edits the first tap on another
  mission or on New mission says "Unsaved changes in <name>: tap again to drop them" in the list's status line
  (`#mission-note`, amber, `role="status"`, 3 s); a second tap on the same one goes on. Focus goes to the back link
  when the list gives way to the editor, to the list's title when the editor closes. The save bar (`.sticky-actions`)
  starts with "● Unsaved changes" in amber; on a phone it sits above the bottom nav, the note on its own line.
- **Robot dashboard** (`.robot-grid`): up to 1179px one column, 3D view, base, grippers, joints, settings. From
  1180px 7 : 5 up to 1600px wide: the base over the joints at the left, the 3D view (16 : 11) over the grippers at the
  right (sticky under the top), the settings full width under both. The base as a readout (`.readout`: x, y, yaw in
  Barlow Semi Condensed 24px over muted 13px names), the nearest placed object under it in muted monospace, or in
  red why there is no pose.
- **Step sheet** (phone, `.pg-sheet`): a tap on a step's chip (also the running one) opens a bottom sheet over a
  scrim (`.pg-scrim`, 45% ink; under the status bar, so Pause and Abort stay in reach, and the bottom nav): a 44 × 5
  handle, the step's tile (52px) with its state and number over the instruction (20px, 600), the step's progress
  while it runs, the pop-up's rows (command, target, locks, on failure, status; 96px labels), each note in an amber
  box (`.pg-sheet-note`), and a full-width Close (48px). A non-modal dialog (`role="dialog"`, labelled by the title):
  focus goes to Close when it opens (idle too, where the tap also picks the start step: Close keeps it, a second tap
  on the chip unpicks it), Tab stays on Close, and focus goes back to the chip when it closes (Close, the scrim,
  Escape or the chip again). Escape does not close it while a manual skill runs: then it aborts the skill, as
  everywhere. Rebuilt every second, it keeps its scroll place. As high as the room between the status bar and the nav,
  at least 160px; it scrolls inside.
- **Lists and rows**: mission rows (`.list li`) with a brand left rule (red when invalid); plan steps (`.steps`) with a
  numbered dot: a neutral ✓ when done, red when it failed, ringed green while running; points, poses and recordings (`.point`)
  open into an editor, the pad's target gets a brand inset rule. Mission editor steps (`.edit-step`) have a drag
  handle left of the step number (an `.icon` with six dots in `--text-muted` on a hairline border, `touch-action:
  none`), then the fold toggle (`.step-toggle`: a ▶ marker turned down when open, the step number in brand text and,
  folded, the instruction over the skill and its arguments in a few words, monospace muted, each one line with an
  ellipsis), then ↑ ↓ ⧉ ✕; on a phone the buttons go to a second row. Folded (`.collapsed`), only that head shows;
  "Collapse all / Expand all" is a ghost button by the Steps title. While dragged the step folds and follows the
  pointer with a brand border and `--shadow-2`, and a 3px brand line in the gap marks where it lands. A step that Check or Save names in its problems
  gets a red border, a 4px red inset rule and a red step number (`.has-problem`) until it is edited.
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
- **Plan graph** (`.pg`, graph.js): steps top to bottom in resource lanes (base, hands, torso, joints) that share the
  Plan card's width (at least 64px each; drawn again when the card resizes), rows of 32px, step numbers in Barlow Semi
  Condensed in a 28px gutter. Each step is one 28px bar (Figma B) across the lanes it holds, from its first to its
  last, 4px inside them, its icon (icons.js) at the left end. A step on lanes that are not next to each other (base
  and right arm, the left arm between) spans the lane between too, as in Figma B: the bar reads as the step's
  reach, the pop-up's Locks row says which lanes it holds. Bars take the skill colour (`--k` from `--kind-*`):
  filled pastel with no outline (22% done, 11% pending) and a neutral ✓ corner when done, running solid with a 2px
  ring and the breathing glow (amber ring paused), failed red with a ✕ dot, timeout / canceled / skipped amber.
  Phases are bands (`.pg-band`, `--surface-field`, 24px) with a chevron, done/total and the usual failure rule;
  closed, one row with the lanes as mini chips and the state word, remembered per mission. Only exceptions are
  marked: rule badges on the bar's top edge, a dashed replan line in the left gutter, dashed boxes round parallel
  candidates. A click on a step (the running one too; the status board shows that one anyway) opens its details in
  a 320px pop-up beside its bar (`.pg-pop`, right of it if it fits, else left, else under the bar, the graph growing
  to hold it; a non-modal dialog with Close; Escape, a click
  elsewhere or the step again close it, the focus goes to Close and back to the step). At 600px and below: a rail of
  44px chips per band and a bottom sheet (`.pg-sheet`) for a step the operator taps. It reads `--g-*` tokens mapped
  onto the global ones.

## Icons and imagery

The P3 logo inline at the top of the rail (takes `--logo`). Icons are plain Unicode arrows and marks today (↑ ↓ ✕ ✓ ←); the 3D
view toolbar expects inline SVG icons with a 1.75px stroke in `currentColor`. No photos or illustrations.
