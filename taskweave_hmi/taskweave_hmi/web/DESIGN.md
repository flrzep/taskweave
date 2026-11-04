# taskweave HMI design system

The look of the operator interface (Run, Program, Robot, Teach, Record, Log). All values live as CSS custom
properties on `:root` in `style.css`. One theme, dark, so there is one theme to keep right. Components use the
tokens, never raw values; the only exception is the black video background.

## Direction: a technical panel

The P3 technical look of the reBot control page, made for a factory hall: a laptop at the cell, a phone in a gloved
hand, a screen read from two metres. Nuke and Blender are the references for the plan graph and the editor: a dark
canvas where the nodes carry the colour.

- **Navy ground, solid panels.** The page is P3 navy (`--surface-page`) with a faint blueprint grid and a blue glow
  at the top right; cards, the rail and the strip sit on it as solid panels with hairline borders. No shadows except
  under what floats (pop-ups, the step sheet).
- **Electric blue is action.** The P3 blue fills primary buttons and the selected 3D tool; a lighter blue marks the
  selected section and is the link colour. Nothing else is blue.
- **Colour means state, like a stack light.** Lime runs (also planning, a manual skill, a healthy link, a save);
  amber paused or to look at; coral stopped on a fault or near a limit; grey idle; finished as planned is neutral,
  not green, so lime means one thing: it runs. Never used for decoration.
- **Andon status.** The state is a solid badge in the state colour, in the strip on every tab and on the Run board;
  the running step's progress is a lime line along the strip's bottom edge. Paused differs from running by its word
  and amber, and the board's robot freezes mid-stride.
- **Big, flat, legible.** Montserrat with tabular figures, 16px body, 48px main buttons, 44px minimum touch
  targets, borders at 3:1 on fields and buttons, two radii (4px controls, 8px panels), no gradients on components.

## Colour

Contrast was measured with the WCAG formula against the panel (`--surface-card`) unless noted. Text is at least
4.5:1; the state badges and the board's headline are large text.

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
| `--done` | `#c5c9e6` | succeeded and done steps and phases: neutral, not green (12:1) |
| `--warn` | `#ffc95c` | paused, jog mode, canceled, a save that asks first (12.8:1) |
| `--bad` | `#ff7f6a` | failed, aborted, errors, near a joint limit, danger buttons (7.9:1) |
| `--idle` | `#9a9ec8` | idle, pending |
| `--on-fill` | `#03031c` | navy text on a solid state fill (7.9:1 and up on every fill) |
| `--kind-*` | drive `#7a8cff`, arm `#b394ff`, grip `#ff9a4d`, wait `#8a8ea8`, other `#2dd4bf` | skill colours in the plan graph, categorical only (6:1 and up) |
| `--viewer-bg`, `--viewer-line`, `--viewer-ink`, `--viewer-tools` | the ground's navy, a hairline, body text, the panel at 82% | the 3D view |
| `--rail`, `--nav`, `--strip` | 92px, 60px, 52px | the section rail's width; the phone's bottom nav; the system strip |
| `--spin-arrows` | ▲ ▼ in `#9a9ec8` | a number field's arrows (an SVG image: `--text-muted`'s value) |

Tints are mixed from the state colour with `color-mix()`: chips 11%, messages 9%, the board and status card 5 to 6%.

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
| `--fs-lg` | 20px / 1.35 | current step instruction, the board's goal |
| `--fs-xl` | 24px / 1.2 | `h2`, the board's instruction |
| `--fs-2xl` | 30px / 1.12 | speed value, recording headline (26px on a phone), the board's "/ N" (`--fs-xl` on a phone) |
| `--fs-4xl` | 60px | the run board's step number (a 44px literal on a phone, its only use) |

Weights: 400 body, 500 instruction, 600 titles and buttons, 700 badges, big figures and the brand name.

## Spacing

A 4px scale: `--sp-1` 4, `--sp-2` 8, `--sp-3` 12, `--sp-4` 16, `--sp-5` 20, `--sp-6` 24, `--sp-8` 32, `--sp-12` 48.
Card padding 16; gap inside cards 12; gap between cards 12 on a phone, 16 from 601px. Content width `--content`
760px, side gutter 16 (12 at 400px and below).

## Shape and depth

- Radius: `--r-s` 4px (buttons, inputs, rows, messages, the state badge), `--r-m` 8px (cards, the 3D view, the
  Teach hero and job tiles), `--r-xs` 2px (bar markers), `--r-pill` (chips, progress, limit bars). The graph's
  nodes keep their own 14px: they are the one rounded thing, by design.
- Depth: `--shadow-1` is none (borders carry the edges); `--shadow-2` under what floats (pop-ups, the step sheet).
  The sticky top is the panel at 88% over a blur, so the page shows through it faintly.
- Focus: a 3px ring in `--focus` (brand text at 60%), offset 2px; inset on segments.

## Touch targets

`--tap` 48px for main buttons, Pause and Abort on the board, `--tap-min` 44px for inputs, selects and summaries,
`--tap-s` 36px for small and icon buttons on a laptop, raised to 44px on touch screens and at 600px and below.

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
- **Shell**: the section rail (`.rail`, 92px, sticky, full height) at the left: the P3 mark and "taskweave" and
  the sections (`.tabs`, buttons: a 22px stroke icon over the name, 64px high, the selected one in brand text on
  `--brand-soft`, `aria-current="page"`): Run, Program, Robot, Teach, Record (hidden unless recording is set up),
  Log. The page beside it, the sticky top (`.top`: the system strip, manual skill strip) over the views. At 600px
  and below the rail is a bottom nav (60px, fixed, the selected section with a 3px brand rule on top) without the
  mark; the step sheet sits above it.
- **System strip** (`.mission`, `#mission`, the one bar on every tab, 52px): the link dot and the robot's name
  (`.strip-robot`), the state as a solid badge (`#state`, uppercase 13px, 700; `planning` while the task handler
  prepares the plan), and while a mission is active (`.mission.active`) the goal with plan id and cycle, "step
  n/N" in monospace and the step's instruction, one line each with an ellipsis, with skill, status, seconds and
  attempt in muted monospace; the step's progress is a 3px line along the strip's bottom edge in the state colour.
  At the right the resources the step holds (`.resources`: base, hands, torso as monospace chips, lime while
  locked, from `step.resources`), Speed and the pad mode (only while it jogs an arm up to 1000px). Mission control
  is not in the strip: the picker, Start, Pause and Abort live on the Run board, and on the Run tab (`body.on-run`)
  the strip shows no mission words and no error either, since the board says them. Under the strip, small: Start's
  answer (`.msg`, idle only) and the last error (`.error` with a ✕ that hides that error until another one or the
  next start). On a phone: dot and name, badge, speed in one row, the step n/N and instruction on a second row; the
  resources and the goal are not shown. A pad press's answer is a toast for a while (`.pad-notice`, fixed outside
  the top, taps pass through: bottom left of the page on a laptop, clear of the 3D view's tools; just under the top
  on a phone, clear of the step sheet and the Save bar), so the strip keeps its height. On a phone the error text,
  the speed message and the manual skill's text scroll in their own boxes (the error's ✕ stays), so both Aborts stay
  in sight.
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
- **Run board** (`.board`, Run tab): a little robot walks the mission. Its floor is the phases (the plan graph's
  bands as 8px bars, as wide as their steps, the done part filled, the current one in the state colour, names and
  done/total under them; one unnamed floor for a plan without phases). It stands where the run is (`--p`: done steps
  plus the step's progress, over the total; at the chosen start step while idle) and moves there in 1.2 s. Its pose is
  the state: blinking when idle, walking (legs, arms, a bob, its shadow) while running, frozen mid-stride with sleepy
  eyes when paused, looking about with a blinking light while planning, arms up and three hops when done, slumped with
  crossed red eyes when failed, flat eyes when aborted; its antenna tip and chest light are the state colour. A small
  comic figure (the user's direction): a big head with a dark visor (`--bot-ink`) and light eyes, a small torso, and
  head, hands and feet floating apart from it on bendy limbs (round strokes whose curve changes as they swing, CSS `d`)
  and a long floppy antenna that lags the walk; two greys (`--bot`, `--bot-2`). Above it a bubble of one fixed height (132px, 200px on a phone: the lines clip, the board does not jump from
  step to step; an error takes the room of the second instruction line) (2px border in the state colour at 45%, its tail over the robot) says what it does: the
  step number at 60px with "/ N" (or "26 steps" while idle), the instruction at 24px (two lines, three on a phone),
  the command, status, seconds and attempt, the error in red; and the next move as one button: Start / Start at step
  N, Resume, Retry step N (the step it stopped at), Run again (Start <mission> when another one is picked). No
  "press Start in the bar above" lines: the button is the instruction. The head above holds the mission's goal and the cycle (↻ 2); idle, the mission picker (`#mission-select`,
  48px) takes the goal's place; active, Pause / Resume and Abort (`.danger`, two taps; 48px, full width on a
  phone) sit at its right. `#btn-start` stays in the page, hidden: the bubble's button is its wire. Reduced motion keeps the poses without the motion. On the Run tab the strip shows no mission words (`body.on-run`), so they are not shown twice, and its error
  hides (the board shows it).
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
- **Program tab: the missions** (`.missions-md`; the tab is called Program, the things it lists are missions): up to 1179px the list or the open mission (← Missions back). From 1180px the list
  (300px, sticky under the top, its own scroll) beside the open mission, or a hint while none is open; the open one is
  marked (`.open`, `--brand-soft`, `aria-current`). A search field filters by name and goal ("No mission matches the
  search." when none does; cleared after a save); only the list scrolls. With unsaved edits the first tap on another
  mission or on New mission says "Unsaved changes in <name>: tap again to drop them" in the list's status line
  (`#mission-note`, amber, `role="status"`, 3 s); a second tap on the same one goes on. Focus goes to the back link
  when the list gives way to the editor, to the list's title when the editor closes. The save bar (`.sticky-actions`)
  starts with "● Unsaved changes" in amber; on a phone it sits above the bottom nav, the note on its own line.
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
  Barlow Semi Condensed 24px over muted 13px names), the nearest placed object under it in muted monospace, or in
  red why there is no pose.
- **Torso line** (side card and step sheet, a `torso` row after the target): what the carry's torso look-ahead
  planned for the step (`data.torso_plan` of a finished carry step whose window holds it, and the step's own
  `data.torso`): "Torso 0.35 m · planned change at this step (from 0.494)", "Torso 0.494 m · kept (plan)", "chosen per
  step (cut)" when the plan did not cover the hold, ", replanned: …" when the real state differed, and a certain
  infeasible plan's message in `--bad`. No row without a report.
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
  ellipsis), then ↑ ↓ ⧉ ✕; on a phone the buttons go to a second row. Folded (`.collapsed`), only that head shows; a
  mission opens with every step folded;
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
- **Plan graph** (`.pg`, graph.js): a state machine, laptop and phone alike (the resource lanes looked like a Gantt
  chart and spent the width on empty lanes). Steps are 44px icon nodes (14px radius, skill colour `--k`: pastel when
  done or pending, solid with the breathing ring when running, red when failed, amber timeout / canceled / skipped),
  their number and target under them, joined by curved arrows in run order: solid (`--text-muted`) where the run has
  been, flowing lime into the running step, dotted ahead. A phase of 4 to 16 steps is a ring (clockwise from its
  upper left, open at the left, its done/total big and faint in the middle); a shorter or longer one a snake on a
  gentle wave. Each node is nudged a few px off its spot (fixed per step), so it reads drawn, not gridded. An open
  phase has a soft outline (28px radius; lime while it runs, red when it failed) with its header pill on the top
  edge (name, done/total, usual failure rule); a nested include is a soft band through its steps with a small tag;
  a closed phase is one stack node (cards, filled from below to its share done), and the arrows point at it.
  Phases flow across the box in lines; an arrow to a phase further down runs out to the left of the outlines, down
  and in from the left. A black dot points at the start step; a dashed brand loop runs up the left gutter back to
  the first step when the mission repeats (its rule along it), else a final ring follows the last step. A replan is
  a dashed red arrow to the step it replans from, labelled; other rule exceptions are badges on the node's top edge.
  A ring's opening is wide enough that its first and last node stay apart; where the box is too narrow for a ring
  (a phone, mostly from 8 steps) the phase is a snake. The operator can drag a node or a phase (by its header or
  stack) with a mouse (a pen may pan instead: a finger must scroll the page from a node, also on a touch laptop), kept per mission (`tw-plan-layout`, offsets of steps or phases no longer in the plan
  dropped) and kept inside the box with its outline; ↺ bottom right puts the drawn layout back. The layout is
  optional, so it has no keyboard path; the nodes and headers are buttons in run order, each header before its
  steps. The resources a step locks are in its pop-up and its tooltip, a phase's in its header's tooltip. A click on a step opens its details in a 320px pop-up
  beside it (`.pg-pop`, right if it fits, else left, else under it; Close, Escape, a click elsewhere or the step
  again close it); at 600px and below a bottom sheet (`.pg-sheet`). It reads `--g-*` tokens mapped onto the global
  ones.

## Icons and imagery

The P3 logo inline at the top of the rail (takes `--logo`). The rail's sections, the plan graph's skills
(`icons.js`) and the Teach jobs are inline SVG line icons with a 2px stroke in `currentColor`; the 3D view toolbar
expects the same at 1.75px. The step editor's ↑ ↓ ⧉ ✕ and the marks ✓ ! ↻ are still Unicode. No photos; one illustration, the run
board's robot (inline SVG in index.html, flat greys and the state colour).
