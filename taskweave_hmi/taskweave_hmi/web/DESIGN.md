# taskweave HMI design system

The look of the operator interface (Run, Missions, Robot, Teach, Record, Log). All values live as CSS custom
properties on `:root` in `style.css` and are redefined for dark mode under `prefers-color-scheme: dark`. Components
use the tokens, never raw values; the only exceptions are the plan graph's own edge colours and the black video
background.

## Direction: steel and signal

An operator panel, not a marketing page. It is used on a laptop and on phones in a factory hall: bright light,
gloves, standing at the robot. So:

- **One brand colour, used for action.** The P3 brand blue `#0000ff` is the logo, the header rule, primary buttons,
  links and selected tabs. Nothing else is blue.
- **Steel neutrals.** Cool grey-blue surfaces and borders that recede, so the state colours stand out.
- **Colour means state.** Green succeeded, amber paused or warning, red failed or near a limit, grey idle, blue
  running. They are never used for decoration.
- **Andon status.** The run status card has a rail and a solid badge in the state colour, a light tint of it on the
  card, and the goal as a large headline. Paused also gets a striped rail, so it differs from running by more than
  hue.
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
| `--surface-card` | `#ffffff` | `#141827` | cards, header, tabs |
| `--surface-field` | `#f4f6f9` | `#0f121f` | inputs, secondary buttons, tracks |
| `--surface-hover` | `#e6e9f0` | `#1d2236` | hover on rows, tabs, buttons |
| `--border` | `#d5d9e2` | `#272c42` | hairlines: cards, rows, dividers |
| `--border-strong` | `#8d93a6` | `#5d6485` | field and button outlines (3:1 on their surface) |
| `--text` | `#0d1026` | `#e9ebf5` | body text (18.8:1 / 14.9:1 on card) |
| `--text-muted` | `#545a70` | `#a3a8bf` | secondary text, labels (6.8:1 / 7.5:1 on card) |
| `--brand` | `#0000ff` | `#3d3dff` | fill of primary buttons, running badge and rail, header rule |
| `--brand-hover` | `#0000c4` | `#5252ff` | primary button hover |
| `--brand-edge` | `#0000ff` | `#6b6bff` | primary button border (lifts it off the dark card) |
| `--brand-text` | `#0000ff` | `#9494ff` | links, selected tab and segment, markers (8.6:1 / 6.7:1) |
| `--brand-soft` | `#e8e8ff` | `#1d1f4d` | brand tint |
| `--on-brand` | `#ffffff` | `#ffffff` | text on `--brand` (8.6:1 / 6.3:1) |
| `--logo` | `#0000ff` | `#ffffff` | the P3 logo |
| `--ok` | `#0a7a4a` | `#45c98f` | succeeded, connected, saved |
| `--warn` | `#9a5200` | `#f2a64f` | paused, jog mode, canceled |
| `--bad` | `#c0112d` | `#ff7385` | failed, aborted, errors, near a joint limit, danger buttons |
| `--idle` | `#5c6177` | `#8e93ab` | idle, pending |
| `--on-fill` | `#ffffff` | `#0b0d17` | text on a solid state fill (5.4 to 9.5:1) |
| `--kind-*` | drive, arm, grip, wait, other | lighter in dark | skill colours in the plan graph, categorical only |
| `--viewer-bg` | `#161a2a` | `#080a12` | 3D view background, dark in both themes |
| `--viewer-line`, `--viewer-ink`, `--viewer-tools` | | | 3D view border, text and toolbar backing |
| `--spin-arrows` | ▲ ▼ in `#545a70` | in `#a3a8bf` | a number field's arrows (an SVG image: `--text-muted`'s value) |

Tints are mixed from the state colour with `color-mix()`: chips 11%, messages 9%, the status card 5%.

## Typography

Barlow for everything, Barlow Semi Condensed for state words, step numbers and figures that need to be narrow, a
system monospace for ids, poses and the log. Both Barlow faces come from Google Fonts with a system fallback (the cell
network may have no internet). `font-variant-numeric: tabular-nums` throughout, so live values do not jitter.

| Token | Size | Use |
| --- | --- | --- |
| `--fs-xs` | 12px | joint names in small grids |
| `--fs-sm` | 13px | monospace meta, log, legend, section eyebrows (`.sub`, uppercase, 0.06em) |
| `--fs-s` | 14px | field labels, chips, tabs (uppercase, 0.05em), small buttons, help text |
| `--fs-base` | 16px / 1.5 | body, inputs, buttons, card titles (`.label`, 600) |
| `--fs-md` | 18px | brand name, state badge (700, uppercase), record stats |
| `--fs-lg` | 20px / 1.35 | current step instruction |
| `--fs-xl` | 24px / 1.2 | `h2` |
| `--fs-2xl` | 30px / 1.12 | status headline (26px on a phone) |

Weights: 400 body, 500 instruction, 600 titles and buttons, 700 badges and the brand name.

## Spacing

A 4px scale: `--sp-1` 4, `--sp-2` 8, `--sp-3` 12, `--sp-4` 16, `--sp-5` 20, `--sp-6` 24, `--sp-8` 32, `--sp-12` 48.
Card padding 16; gap inside cards 12; gap between cards 12 on a phone, 16 from 601px. Content width `--content`
760px, side gutter 16 (12 at 400px and below).

## Shape and depth

- Radius: `--r-xs` 3px (bar markers), `--r-s` 6px (buttons, inputs, rows, messages), `--r-m` 10px (cards, 3D view),
  `--r-pill` (chips, progress, limit bars).
- Shadows: `--shadow-1` a faint lift on cards (none in dark, where borders carry the edge), `--shadow-2` under the
  sticky header and tabs.
- Focus: a 3px ring in `--focus` (brand text at 55%), offset 2px; inset on tabs and segments.

## Touch targets

`--tap` 48px for main buttons, `--tap-min` 44px for inputs, selects and summaries, `--tap-s` 36px for small and icon
buttons on a laptop, raised to 44px on touch screens and at 600px and below.

## Components

- **Buttons** (`.btn`): secondary by default (field surface, strong border, text colour). `.primary`: brand fill,
  white text. `.danger`: red text and border on the card; `.danger.confirm` (the second press) is a solid red fill.
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
- **Tabs** (`.tabs`): in the sticky top bar with the header (`.top` wraps both). Uppercase 14px, 48px high, equal
  width; the selected one in brand text with a 3px underline. At 400px and below they drop the uppercase so all six
  fit; if they still do not, they scroll inside the bar, never the page.
- **Speed override** (`.speed`): a strip in the sticky top bar between header and tabs, on every tab. The percent
  big (Barlow Semi Condensed 30px), − / + icon buttons around a range slider in brand colour, the presets as a
  segmented control, and a test mode toggle that turns amber with its cap (the value turns amber too). The
  handler's message on a clamped or refused request below, for 8 s. Greyed out with a hint without the task handler.
  On a phone: value and presets, then slider and test mode.
- **Manual skill** (`.manual`): a strip in the sticky top bar below the speed strip, on every tab, only while a skill
  started from the HMI outside a mission runs (Drive here, Move here, Go here, a carry pose's Test). "Running
  <label> (<skill>) · <n> s" in 600 weight on a 5% brand tint with a 4px brand left rule (running blue), one line
  with an ellipsis on a laptop, wrapping on a phone, and a small `.danger.solid` **Abort** on the right: one press,
  Escape does the same (not while typing in a field). The button that started the skill then says "aborted" as a
  `.msg.warn`.
- **Cards** (`.card`): card surface, hairline border, 10px radius, 16px padding, a grid with 12px gaps.
  `details.card` opens from its whole title row.
- **Status card** (`.card.status`): the andon panel. A 10px rail and a solid badge (`.chip` inside it) in the state
  colour, a 5% tint on the card, the goal as the headline, a 10px progress bar in the state colour.
- **Lists and rows**: mission rows (`.list li`) with a brand left rule (red when invalid); plan steps (`.steps`) with a
  numbered dot that fills green or red when done, rings blue while running; points, poses and recordings (`.point`)
  open into an editor, the pad's target gets a brand inset rule. Mission editor steps (`.edit-step`) have a drag
  handle left of the step number (an `.icon` with six dots in `--text-muted` on a hairline border, `touch-action:
  none`); while dragged the step shrinks to its head and instruction and follows the pointer with a brand border and
  `--shadow-2`, and a 3px brand line in the gap marks where it lands. A step that Check or Save names in its problems
  gets a red border, a 4px red inset rule and a red step number (`.has-problem`) until it is edited.
- **Chips** (`.chip`): a dot and the word, in the state colour on an 11% tint. Each state sets `--c` (text), `--fill`
  and `--ink` (for the solid badge).
- **Details and summary**: summaries in brand text, 44px high; nested ones inside a point are muted and monospace.
- **Messages** (`.msg`, `.msg.bad`, `.msg.warn`, `.error`): a 4px left rule in green, red or amber (canceled,
  aborted by the operator) on a 9% tint, body text colour.
- **Joint and limit bars** (`.joint-row`, `.limit-bar`): a pill track with a 4px brand marker at `--at`; within 10° of
  a limit the track, marker, name and value turn red. On a laptop one row per joint (name, bar, value, −, target, +;
  gripper rows put Open and Close under the last three columns). On a phone the row wraps: name and value, the bar
  full width, then the controls at 44px.
- **3D view** (`.viewer3d`): a full-width viewport card, 16:10 on a laptop and 4:3 at 600px and below, dark in both
  themes, 10px radius; a `canvas` child fills it. `.viewer3d-tools` is an overlay toolbar top right for icon buttons
  (`--tap-s` square, light icons; `aria-pressed="true"` in brand fill).
- **Plan graph** (`.plan-graph`): nodes tinted by skill colour (`--k` from `--kind-*`), state by badge, the running
  node breathes; it reads `--g-*` tokens mapped onto the global ones.

## Icons and imagery

The P3 logo inline in the header (takes `--logo`). Icons are plain Unicode arrows and marks today (↑ ↓ ✕ ✓ ←); the 3D
view toolbar expects inline SVG icons with a 1.75px stroke in `currentColor`. No photos or illustrations.
