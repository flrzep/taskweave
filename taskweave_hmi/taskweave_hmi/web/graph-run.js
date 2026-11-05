'use strict';
// taskweave HMI, Run tab: the running plan as the simplified flow on graph-core.js: Start, the nodes left to right,
// Done (or the loop back to Start), the whole tree on a canvas that pans and zooms and follows the running node. Nodes
// are a header in the colour of what moves (--kind-*: drive, arm, grip, wait, other) with the step's name, and a body;
// the running one is filled with its colour, a lime ring and its main argument as the one field. Blocks (an include of
// the top mission, a nested include, and with schema v2 groups, repeats and parallel lanes) are boxes with a title bar,
// open unless closed to one stack node (remembered per mission); the box with the running step is always open. No ports, no
// rule nodes, no words on wires but the loop's: a replan is a thin coral wire under the nodes. A step opens its details
// (the instruction, the rule) in a pop-up beside it (a bottom sheet at 600 px and below). Uses $,
// el, api, state, show, missionOf, shownPlan, focusIndex, pickStart, moveLabel, moveParts, carriesByArgs from app.js,
// skillIcon, iconElement, moveKindOfArgs from icons.js and flowName, flowHead, flowField, stepFields, blockFields from graph-core.js.

// what moves: a move step's kind, else the skill (cell schema 3 names too)
const KIND_COLOURS = { navigate: 'drive', straight: 'drive', hand: 'arm', posture: 'arm', joints: 'arm' };
const SKILL_COLOURS = { navigate_to: 'drive', drive_straight: 'drive', move_joints: 'arm', move_gripper: 'arm',
  move_object: 'arm', set_posture: 'arm', grip: 'grip', wait: 'wait' };
const POP_W = 320;     // the step pop-up's width (style.css .pg-pop), the gap to its node, the least room it gets below
const POP_GAP = 12;
const POP_MIN_H = 260;
const BREATHE_MS = 2800;  // period of node-breathe for the running step (style.css)
const PHONE = window.matchMedia('(max-width: 600px)');
const WARN_STATES = ['timeout', 'canceled', 'skipped'];  // amber chips; failed and rejected are red
const BANDS_KEY = 'tw-plan-bands';  // localStorage: {mission: {frame key: open}}
const missionCache = {};  // mission name -> its file (frame names, defaults, loop), until a new plan or a save
const missionPlan = {};   // mission name -> the plan_id its cached file was read for
const view = { selected: null };  // the step (id) whose card shows instead of the running one
const div = (cls, ...children) => el('div', { class: cls }, ...children);

function mainArg(step) {
  const a = step.args || {};
  if (step.skill === 'move') {  // app.js: "tray_1/place", "hand left by 0 / 0 / +6 cm" (the command is the title)
    const { head, rest } = moveParts(a);
    return [head, rest].filter(Boolean).join(' · ');
  }
  if (a.target) return String(a.target);
  if (a.pose) return String(a.pose);
  if (a.to && typeof a.to === 'object') {  // move_gripper / move_object: the target variant's main value
    const [key, value] = Object.entries(a.to)[0] || [];
    if (key === 'point' || key === 'pose') return String(value);
    if (key === 'onto') return `${value}/${a.to.contact ?? '?'}`;
    if (key) return `${key} ${Array.isArray(value) ? `[${value.join(', ')}]` : value}`;
  }
  if (a.action) return `${[].concat(a.hands ?? []).join('+')} ${a.action}`.trim();
  if (a.elbow) return `elbow ${a.elbow}`;
  if (a.seconds !== undefined) return `${a.seconds} s`;
  const first = Object.entries(a)[0];
  return first ? `${first[0]} ${typeof first[1] === 'object' ? '…' : first[1]}` : '';
}

async function missionFor(plan) {
  const name = missionOf(plan);
  if (!(name in missionCache)) {
    missionCache[name] = null;  // one request per mission
    const { ok, data } = await api('GET', `/api/missions/${encodeURIComponent(name)}`);
    missionCache[name] = ok ? data : {};
    renderGraph();
  }
  return missionCache[name];
}
// the Program tab saved this mission: read its file again at the next draw
function forgetMission(name) { delete missionCache[name]; delete missionPlan[name]; }

// The rules each step runs with: the plan's values (mission defaults where a step says "default": retries -1, an
// empty on_failure) and the step a replan restarts from (replan_from, resolved by core; itself when empty).
function rules(plan, mission) {
  const defaults = mission?.defaults || {};
  const ids = plan.steps.map((s) => s.id);
  return plan.steps.map((step, i) => {
    const retries = step.retries >= 0 ? step.retries : (defaults.retries ?? 0);
    const onFailure = step.on_failure || defaults.on_failure || 'abort';
    const to = step.replan_from ? ids.indexOf(step.replan_from) : -1;
    return { retries, onFailure, replanTo: to >= 0 ? to : i, maxReplans: step.max_replans || 0,
      key: `${onFailure}|${retries}` };
  });
}
// a rule in words: "abort", "↻2 then skip"
const ruleText = (r) => `${r.retries > 0 ? `↻${r.retries} then ` : ''}${r.onFailure}`;
const jumps = (r, i) => r.onFailure === 'replan' && r.replanTo !== i;  // a replan to another step: the one line

// a step's command: a move step's (its kind's label; "move" while a placeholder hides it), else the skill
const stepTitle = (step) => (step.skill === 'move' ? (step.kind ? moveLabel(step.kind) : 'move')
  : `${step.skill}${step.kind ? ` ${step.kind}` : ''}`);
// what a move step carries: from its result (data.carrying, the message "· carrying <instance> ·"), else true when its
// arguments carry (hand: holding, the old object form, to an object target); a hand move of a hand that holds
// becomes a carry only at run time
function carrying(step, event) {
  if (step.skill !== 'move') return null;
  const done = event?.event === 'step_finished' ? event : null;
  const found = done?.data?.carrying || done?.message?.match(/· carrying ([^\s·]+)/)?.[1];
  if (found) return found;
  return done?.data?.module === 'move.carry' || carriesByArgs(step.args) ? true : null;
}
// The torso line of a step (the side card, the phone's sheet): from the look-ahead's report (data.torso_plan of a
// finished carry step: TorsoPlan.report() in taskweave_skills torso_plan.py) whose window holds the step, and the
// height the step ran at (data.torso, its last leg). Planned: "Torso 0.35 m · planned change at this step (from 0.494)" or
// "Torso 0.494 m · kept (plan)"; cut, capped or stopped: "chosen per step"; infeasible and certain: its message, red.
// A step that ran a planned change on a state that differed says "replanned". null: no report covers the step.
const metres = (v) => `${+Number(v).toFixed(3)} m`;
function torsoLine(step, events) {
  const reports = Object.values(events || {}).map((e) => e?.data?.torso_plan).filter((p) => p?.window?.includes(step.id));
  const own = events?.[step.id]?.data;
  const plan = own?.torso_plan?.window?.includes(step.id) ? own.torso_plan : reports.at(-1);
  if (!plan) return null;
  const ran = Array.isArray(own?.torso) ? own.torso.at(-1)?.height : own?.torso;
  const height = ran ?? plan.heights?.[step.id];
  const at = height != null ? `Torso ${metres(height)}` : 'Torso';
  const replanned = own?.torso_plan?.source === 'replanned'
    ? ` · replanned${own.torso_plan.message ? `: ${own.torso_plan.message}` : ''}` : '';
  if (plan.outcome === 'infeasible' && plan.certain) return { text: `${at} · ${plan.message || 'no torso height fits the hold'}`, bad: true };
  if (plan.outcome !== 'planned') return { text: `${at} · chosen per step (${plan.outcome})${replanned}`, bad: false };
  const change = (plan.changes || []).find((c) => c.at === step.id);
  return { text: change ? `Torso ${metres(change.to)} · planned change at this step (from ${+Number(change.from).toFixed(3)})${replanned}`
    : `${at} · kept (plan)${replanned}`, bad: false };
}
const colourOf = (step) => KIND_COLOURS[step.kind || (step.skill === 'move' ? moveKindOfArgs(step.args) : '')]
  || SKILL_COLOURS[step.skill] || 'other';
function iconOf(step, event) {
  const id = skillIcon(step, event?.event === 'step_finished' ? event : undefined);
  return id === 'hand' && carrying(step, event) ? 'carry' : id;
}

function stepState(snap, plan, live, step, index) {
  const s = snap.status;
  const active = (s.state === 'running' || s.state === 'paused') && s.plan_id === plan.plan_id;
  if (active && s.step_index === index) return s.state === 'paused' ? 'paused' : 'running';
  const event = live ? snap.outcomes[step.id] : null;
  if (event?.event === 'step_finished') return event.outcome.toLowerCase();
  return 'pending';
}
const isLive = (st) => st === 'running' || st === 'paused';
const isFailed = (st) => st === 'failed' || st === 'rejected';
const isFinished = (st) => st !== 'pending' && !isLive(st);

function storedBands(mission) {
  try { return (JSON.parse(localStorage.getItem(BANDS_KEY) || '{}') || {})[mission] || {}; } catch (_) { return {}; }
}
function storeBand(mission, key, open) {
  try {
    const all = JSON.parse(localStorage.getItem(BANDS_KEY) || '{}') || {};
    all[mission] = { ...(all[mission] || {}), [key]: open };
    localStorage.setItem(BANDS_KEY, JSON.stringify(all));
  } catch (_) { /* private mode: the choice lasts until the page reloads */ }
}

// A frame's head data: done/total, its state word, the usual failure rule (the most common one), what it uses.
function frameInfo(frame, states, r, steps) {
  const done = frame.leaves.filter((i) => isFinished(states[i])).length;
  const count = {};
  frame.leaves.forEach((i) => { count[r[i].key] = (count[r[i].key] || 0) + 1; });
  const usual = Object.entries(count).sort((a, b) => b[1] - a[1])[0]?.[0] || 'abort|0';
  const st = frame.leaves.map((i) => states[i]);
  const word = st.includes('paused') ? 'paused' : st.includes('running') ? 'running' : st.some(isFailed) ? 'failed'
    : frame.leaves.length && done === frame.leaves.length ? 'done' : done ? 'stopped' : 'pending';
  const used = [...new Set(frame.leaves.flatMap((i) => steps[i].resources || []))];
  return { done, usual, rule: r[frame.leaves.find((i) => r[i].key === usual)] || { retries: 0, onFailure: 'abort' }, word, used };
}

// a step's node: the header (its name), the body; running or paused filled with its colour, its main argument in it
function chipButton(step, i, ctx) {
  const { states, r, events, pickable } = ctx;
  const status = states[i];
  const start = pickable && step.id === state.startStep;
  const live = isLive(status);
  const field = live && ctx.metrics.fields ? stepFields(step)[0] : null;
  const chip = el('button', { type: 'button', 'data-step': step.id,
    class: `pg-chip fl-node kind-${colourOf(step)} state-${status}${WARN_STATES.includes(status) ? ' state-warn' : ''}`
      + `${start ? ' start-here' : view.selected === step.id ? ' selected' : ''}`,
    'aria-label': `Step ${i + 1}: ${step.instruction} (${status})` },
  flowHead(flowName(ctx.nodes[i] || { kind: 'step', step }), ''), el('div', { class: 'fl-body' }, field ? flowField(field) : ''));
  chip.firstChild.append(el('span', { class: 'fl-num', 'aria-hidden': 'true', text: String(i + 1) }));  // the step's number
  const held = carrying(step, events[step.id]);
  chip.title = [`${i + 1}. ${step.instruction}`, `${stepTitle(step)} · ${mainArg(step)}`,
    held ? (held === true ? 'carries a held object' : `carrying ${held}`) : '',
    step.resources?.length ? `uses ${step.resources.join(', ')}` : '', `on failure: ${failureText(r[i])}`,
    ...(step.notes || []).map((note) => `note: ${note}`)].filter(Boolean).join('\n');
  // the graph is rebuilt on every status update: breathe at the phase of a page-wide clock, so it runs on
  if (live) chip.style.animationDelay = `-${(performance.now() % BREATHE_MS) / 1000}s`;
  if (status === 'succeeded') chip.append(el('span', { class: 'pg-dot ok' }));
  if (isFailed(status)) chip.append(el('span', { class: 'pg-dot bad', text: '✕' }));
  chip.addEventListener('click', () => choose(step, pickable));
  return chip;
}
const failureText = (rule) => `${ruleText(rule)}${rule.onFailure === 'replan' ? ` from step ${rule.replanTo + 1}` : ''}`
  + `${rule.onFailure === 'replan' && rule.maxReplans ? ` · max ${rule.maxReplans}` : ''}`;

// A click on a chip: idle, it picks the start step (app.js) and shows its card; running, it shows the step's card
// (again, or the running chip: back to the running step). Escape does that too.
// On a phone the sheet opens (idle too: Close keeps the start step picked, a second tap on the chip unpicks it) and the
// focus goes into it.
function choose(step, pickable) {
  let opening;
  if (pickable) {
    opening = state.startStep !== step.id;
    view.selected = opening ? step.id : null;
    pickStart(step);
  } else {
    opening = view.selected !== step.id;
    view.selected = opening ? step.id : null;
    renderGraph();
  }
  if (opening) $('plan-graph').querySelector('[data-sheet-close]')?.focus({ preventScroll: true });  // into the sheet or pop-up
}
// Escape: on the Run tab it closes the card or sheet, but not while a manual skill runs: then Escape aborts that skill
// (app.js) and nothing else
document.addEventListener('keydown', (e) => {
  if (e.key !== 'Escape' || !view.selected || $('view-run').hidden || state.snapshot?.manual
    || e.target.closest?.('textarea, select, input, [contenteditable]')) return;
  closeSheet(view.selected);
});
// a click outside the canvas closes the pop-up (laptop; a tap on the canvas's ground closes it too, canvasPointerUp);
// a node's own click opens or closes as above
document.addEventListener('pointerdown', (e) => {
  if (PHONE.matches || !view.selected || e.target.closest?.('.pg-canvas')) return;
  view.selected = null;
  renderGraph();
});

// a box's tooltip: its instruction, done/total, state and usual rule, a repeat's rule and pass, what it uses
const frameTitle = (frame, info, snap) => `${frame.name}${frame.include && frame.include !== frame.name ? ` (${frame.include})` : ''}`
  + `\n${info.done} of ${frame.leaves.length} done · ${info.word} · fails: ${ruleText(info.rule)}`
  + `${frame.kind === 'repeat' ? `\n${repeatText(frame, snap)}` : ''}`
  + `${info.used.length ? `\nuses ${info.used.join(', ')}` : ''}`;
function repeatText(frame, snap) {
  const r = frame.block?.repeat || {};
  const rule = r.times ? `${r.times}×` : r.while ? `while ${r.while}` : r.until ? `until ${r.until}` : 'repeat';
  const pass = snap?.status?.passes?.[frame.id];
  return [`repeat ${rule}`, pass ? `pass ${pass}` : '', frame.block?.max_cycles ? `max ${frame.block.max_cycles}` : ''].filter(Boolean).join(' · ');
}
// An open box's title bar: a strip in its colour, the name, the kind word, a repeat's rule, and the chevron at the
// right; a click closes the box. A lane (a parallel block's branch) has no chevron: it closes with its block.
const FRAME_HUES = { include: 'hd-brand', group: 'hd-brand', repeat: 'hd-warn', parallel: 'hd-warn', branch: 'hd-lane' };
function frameHead(frame, info, open, toggle, snap) {
  const lane = frame.kind === 'branch';
  const head = el(lane ? 'div' : 'button', { class: `pg-band fl-title ${FRAME_HUES[frame.kind] || 'hd-brand'} ${frame.kind}${open ? ' open' : ''}`,
    'data-band': frame.key, title: frameTitle(frame, info, snap) },
  el('span', { class: 'fl-name', text: flowName(frame) }),
  el('span', { class: 'fl-kind', text: FLOW_KIND_WORDS[frame.kind] === flowName(frame) ? '' : FLOW_KIND_WORDS[frame.kind] }),
  ...blockFields(frame).map(flowField), lane ? '' : el('span', { class: 'pg-chev', 'aria-hidden': 'true' }));
  if (!lane) {
    head.type = 'button';
    head.setAttribute('aria-expanded', String(open));
    head.addEventListener('click', toggle);
  }
  return head;
}
// a closed block: one node with cards behind it, its header in the block's colour, done/total in its body filled to
// its share done; a click opens it
function stackButton(frame, info, toggle, snap) {
  const total = frame.leaves.length;
  const node = el('button', { type: 'button', class: `pg-stack fl-node fl-stack ${FRAME_HUES[frame.kind] || 'hd-brand'} ${frame.kind} ${info.word}`,
    'data-band': frame.key, 'aria-expanded': 'false', title: frameTitle(frame, info, snap),
    'aria-label': `${frame.name}: ${info.done} of ${total} done, ${info.word}`,
    style: `--done-share: ${Math.round((total ? info.done / total : 0) * 100)}%` },
  flowHead(flowName(frame), ''), el('div', { class: 'fl-body' }, el('span', { class: 'pg-stack-count', text: `${info.done}/${total}` })));
  node.addEventListener('click', toggle);
  return node;
}

// the card of one step: the side card on a laptop, the bottom sheet on a phone
function planCard(i, ctx, phone) {
  const { states, r, events, snap, plan, pickable } = ctx;
  const step = plan.steps[i];
  const status = states[i];
  const event = events[step.id];
  const s = snap.status;
  const start = pickable && step.id === state.startStep;
  const live = isLive(status);
  const held = carrying(step, event);
  const command = `${stepTitle(step)}${held ? ` · carrying${held === true ? '' : ` ${held}`}` : ''}`;
  const finished = event?.event === 'step_finished';
  const statusText = live
    ? [s.step_status, `attempt ${s.attempt}`, `${(s.step_elapsed_s || 0).toFixed(1)} s`].filter(Boolean).join(' · ')
    : finished ? [event.outcome.toLowerCase(), event.message].filter(Boolean).join(' · ') : '';
  const eyebrow = el('div', { class: 'pg-eyebrow', text: `${start ? 'start here' : status} · step ${i + 1} of ${plan.steps.length}` });
  const tile = div(`pg-tile kind-${colourOf(step)}`, iconElement(iconOf(step, event)));
  const progress = live ? div('pg-progress', el('i', { style: `width: ${Math.max(0, Math.min(1, s.step_progress)) * 100}%` })) : '';
  const row = (label, value, cls = '') => (value ? [el('dt', { text: label }), el('dd', { class: cls, text: value })] : []);
  const torso = torsoLine(step, events);
  const rows = (notes) => el('dl', { class: 'pg-rows' }, ...row('command', command), ...row('target', mainArg(step), 'mono'),
    ...row('torso', torso?.text, torso?.bad ? 'bad' : ''),
    ...row('locks', (step.resources || []).join(', ') || '–'), ...row('on failure', failureText(r[i])),
    ...row('status', statusText, live ? 'mono' : ''), ...(notes ? (step.notes || []).flatMap((note) => row('note', note)) : []));
  if (phone) {
    // the bottom sheet (a dialog, not modal: the strip and its Abort stay in reach above the scrim):
    // the same rows as the side card, the notes as amber boxes, the ✕ at its top right; a tap on the scrim, Escape or
    // the chip close it too
    const title = el('div', { class: 'pg-sheet-title', id: 'pg-sheet-title', text: `${i + 1} · ${step.instruction}` });
    const close = el('button', { type: 'button', class: 'icon pg-sheet-close', 'data-sheet-close': '', 'aria-label': 'Close',
      title: 'Close', text: '✕' });
    close.addEventListener('click', () => closeSheet(step.id));
    const scrim = div('pg-scrim');
    scrim.addEventListener('click', () => closeSheet(step.id));
    const sheet = el('section', { class: `pg-sheet state-${status}`, role: 'dialog', 'aria-labelledby': 'pg-sheet-title' },
      div('pg-handle'), div('pg-sheet-row', tile, div('pg-sheet-text', eyebrow, title), close), progress, rows(false),
      ...(step.notes || []).map((note) => el('p', { class: 'pg-sheet-note', text: note })));
    sheet.addEventListener('keydown', (e) => { if (e.key === 'Tab') { e.preventDefault(); close.focus(); } });  // stays in
    return [scrim, sheet];
  }
  // the laptop: a pop-up beside the step's chip (a non-modal dialog; the ✕ at its top right, Escape, a click
  // elsewhere or the chip again close it)
  const close = el('button', { type: 'button', class: 'icon pg-pop-close', 'data-sheet-close': '', 'aria-label': 'Close',
    title: 'Close', text: '✕' });
  close.addEventListener('click', () => closeSheet(step.id));
  const pop = el('aside', { class: `pg-card pg-pop state-${status}${live || start ? ' marked' : ''}`, role: 'dialog',
    'aria-labelledby': 'pg-pop-title' },
  div('pg-card-head', tile, div('pg-card-titles', eyebrow,
    el('div', { class: 'pg-card-title', id: 'pg-pop-title', text: `${i + 1} · ${step.instruction}` })), close),
  progress, rows(true));
  pop.addEventListener('keydown', (e) => { if (e.key === 'Tab') { e.preventDefault(); close.focus(); } });  // stays in
  return pop;
}
// the sheet closes: nothing selected, the keyboard focus back on the step's chip
function closeSheet(id) {
  view.selected = null;
  renderGraph();
  $('plan-graph').querySelector(`[data-step="${CSS.escape(id)}"]`)?.focus();
}

// under the plan: only the replan key, and only when a replan wire is drawn (the nodes say what they do; the
// icons live in the pop-up)
function legend(plan, events, anyReplan) {
  return div('legend', anyReplan ? el('span', { class: 'legend-item' }, el('i', { class: 'swatch-fail' }), 'replan') : '');
}

// the words on the mission's loop wire, the only words on a wire: "forever", "until no_frames", "×5"
function loopText(plan, mission) {
  const loop = plan.loop || mission?.loop || 'once';
  if (loop === 'once') return '';
  if (mission?.stop_when) return `until ${mission.stop_when}`;
  if (mission?.max_cycles) return `×${mission.max_cycles}`;
  return loop === 'forever' ? 'forever' : String(loop);
}

// ---------- the canvas: the viewport, the layer it moves, the camera ----------
// One viewport (.pg-canvas, built once, kept across rebuilds) holds the layer (.pg-layer: the drawn graph, rebuilt on
// every status update) and the pop-up, which is never scaled. The camera is one transform on the layer: translate(x, y)
// scale(scale), kept across rebuilds. Follow keeps the running (else the stopped, else the start) node centred; a pan
// or a zoom by the operator turns it off, the Follow button on again. layout: the last drawn layout; planId: the plan
// last fitted; focus: the item key of the node follow centres.
const ZOOM_MIN = 0.4;
const ZOOM_MAX = 1.5;
const FIRST_FIT_MIN = 0.6;  // a new plan opens fitted down to this; smaller, at scale 1 on the followed node
const PAN_PX = 4;  // a pointer that moves less is a click on what is under it
const canvas = { x: 0, y: 0, scale: 1, follow: true, layer: null, wrap: null, layout: null, planId: null, focus: null,
  refocusing: false };  // refocusing: a rebuild puts the keyboard focus back (not a Tab: the camera stays)
const clampZoom = (s) => Math.min(ZOOM_MAX, Math.max(ZOOM_MIN, s));

// the whole tree in view: smaller than the viewport, centred on each axis; larger, from its top left
function canvasFit(layout) {
  const w = canvas.wrap.clientWidth;
  const h = canvas.wrap.clientHeight;
  canvas.scale = clampZoom(Math.min(w / layout.width, h / layout.height, 1));
  canvas.x = Math.max(0, (w - layout.width * canvas.scale) / 2);
  canvas.y = Math.max(0, (h - layout.height * canvas.scale) / 2);
}
// layout point (x, y) at the viewport's centre, at the current scale
function canvasCentre(x, y) {
  canvas.x = canvas.wrap.clientWidth / 2 - canvas.scale * x;
  canvas.y = canvas.wrap.clientHeight / 2 - canvas.scale * y;
}
// the camera onto the layer; animate: the 300ms ease of a follow move (style.css drops it under reduced motion)
function canvasApply(animate) {
  if (!canvas.layer) return;
  canvas.layer.classList.toggle('animate', animate);
  canvas.layer.style.transform = `translate(${canvas.x}px, ${canvas.y}px) scale(${canvas.scale})`;
  placePop();
}
// zoom by factor around viewport point (px, py): that point stays where it is
function canvasZoom(factor, px, py) {
  const scale = clampZoom(canvas.scale * factor);
  canvas.x = px - (px - canvas.x) * (scale / canvas.scale);
  canvas.y = py - (py - canvas.y) * (scale / canvas.scale);
  canvas.scale = scale;
}
function canvasFollow(on) {
  canvas.follow = on;
  $('canvas-follow').setAttribute('aria-pressed', String(on));
}
// the followed node's centre in layout coordinates, or null
function focusPoint() {
  const n = canvas.layout?.items.get(canvas.focus);
  return n ? [n.cx, n.cy] : null;
}
// the laptop's pop-up beside its node's place on screen: right of it if it fits in the viewport, else left, else under
function placePop() {
  const pop = canvas.wrap?.querySelector('.pg-pop');
  const n = pop && canvas.layout?.items.get(canvas.layout.owner.get(view.selected));
  if (!n) return;
  const { x, y, scale } = canvas;
  const w = canvas.wrap.clientWidth;
  const right = x + scale * (n.cx + n.nw / 2) + POP_GAP;
  const left = x + scale * (n.cx - n.nw / 2) - POP_GAP - POP_W;
  let top = y + scale * (n.cy - n.nh / 2) - 8;
  if (right + POP_W <= w) pop.style.left = `${right}px`;
  else if (left >= 0) pop.style.left = `${left}px`;
  else {
    pop.style.left = `${Math.max(0, Math.min(x + scale * n.cx - POP_W / 2, w - POP_W))}px`;
    top = y + scale * (n.cy + n.nh / 2) + POP_GAP;
  }
  top = Math.max(0, Math.min(top, canvas.wrap.clientHeight - POP_MIN_H));
  pop.style.top = `${top}px`;
  pop.style.maxHeight = `${canvas.wrap.clientHeight - top - 8}px`;  // scrolls inside, never past the viewport's bottom
}

// The gestures on the viewport. One pointer: a move past PAN_PX pans (and captures the pointer; under it the pointer is
// a click on its node, or on the ground: that closes the pop-up); two: a pinch zooms around their midpoint and pans with
// it. After a pan or a pinch the click that follows is swallowed once. The wheel zooms around the pointer (a notch
// about 15%; a trackpad's pinch, which comes with Ctrl, in finer steps); only a drag pans. Not on the pop-up or the
// sheet: they keep their own pointer and wheel.
const pointers = new Map();  // pointerId -> its last { x, y } in the viewport
const gesture = { sx: 0, sy: 0, drag: false, swallow: false };
const canvasPoint = (e) => { const r = canvas.wrap.getBoundingClientRect(); return { x: e.clientX - r.left, y: e.clientY - r.top }; };
const offCanvas = (e) => e.target.closest?.('.pg-pop, .pg-sheet, .pg-scrim');
function canvasMoved() {
  canvasFollow(false);
  canvasApply(false);
}
function canvasPointerDown(e) {
  gesture.swallow = false;  // first: a touch pan ends without a click, and the next tap (the pop-up's too) is real
  if (offCanvas(e) || (e.pointerType === 'mouse' && e.button !== 0)) return;
  const p = canvasPoint(e);
  pointers.set(e.pointerId, p);
  if (pointers.size === 1) Object.assign(gesture, { sx: p.x, sy: p.y, drag: false });
  else {  // a second finger: a pinch, both pointers held by the viewport
    gesture.drag = true;
    pointers.forEach((_, id) => canvas.wrap.setPointerCapture(id));
  }
}
function canvasPointerMove(e) {
  const last = pointers.get(e.pointerId);
  if (!last) return;
  const p = canvasPoint(e);
  if (pointers.size > 1) {
    const other = [...pointers].find(([id]) => id !== e.pointerId)[1];
    const mid = (a) => ({ x: (a.x + other.x) / 2, y: (a.y + other.y) / 2 });
    const [m0, m1] = [mid(last), mid(p)];
    const d0 = Math.hypot(last.x - other.x, last.y - other.y);
    if (d0 > 0) canvasZoom(Math.hypot(p.x - other.x, p.y - other.y) / d0, m0.x, m0.y);
    canvas.x += m1.x - m0.x;
    canvas.y += m1.y - m0.y;
  } else {
    if (!gesture.drag) {
      if (Math.hypot(p.x - gesture.sx, p.y - gesture.sy) < PAN_PX) return;
      gesture.drag = true;
      canvas.wrap.setPointerCapture(e.pointerId);
      canvas.wrap.classList.add('dragging');
    }
    canvas.x += p.x - last.x;
    canvas.y += p.y - last.y;
  }
  pointers.set(e.pointerId, p);
  canvasMoved();
}
function canvasPointerUp(e) {
  if (!pointers.delete(e.pointerId) || pointers.size) return;
  canvas.wrap.classList.remove('dragging');
  if (gesture.drag) gesture.swallow = true;
  else if (e.type === 'pointerup' && view.selected && !PHONE.matches
    && !e.target.closest?.('.pg-chip, .pg-band, .pg-stack')) {  // a tap on the ground
    view.selected = null;
    renderGraph();
  }
}
function canvasWheel(e) {
  if (offCanvas(e)) {
    if (e.ctrlKey || e.metaKey) e.preventDefault();  // a pinch over the pop-up: not the browser's page zoom
    return;
  }
  e.preventDefault();
  const unit = e.deltaMode === 1 ? 16 : e.deltaMode === 2 ? canvas.wrap.clientHeight : 1;  // lines or pages (Firefox) as px
  const p = canvasPoint(e);
  canvasZoom(Math.exp(-e.deltaY * unit / (e.ctrlKey || e.metaKey ? 300 : 600)), p.x, p.y);
  canvasMoved();
}
// Tab onto a node or a title bar outside the viewport: centred (follow off, as after a pan), so the focus is in sight.
// Not the focus a press gives (a pointer is down: pointerdown filled the map first): that is a click on the node
function canvasFocusIn(e) {
  if (canvas.refocusing || pointers.size || offCanvas(e)) return;
  const c = canvas.wrap.getBoundingClientRect();
  const n = e.target.getBoundingClientRect();
  if (n.left >= c.left && n.right <= c.right && n.top >= c.top && n.bottom <= c.bottom) return;
  canvas.x += c.left + c.width / 2 - (n.left + n.width / 2);
  canvas.y += c.top + c.height / 2 - (n.top + n.height / 2);
  canvasMoved();
}
function canvasView() {
  const wrap = div('pg-canvas');
  wrap.addEventListener('pointerdown', canvasPointerDown);
  wrap.addEventListener('pointermove', canvasPointerMove);
  wrap.addEventListener('pointerup', canvasPointerUp);
  wrap.addEventListener('pointercancel', canvasPointerUp);
  wrap.addEventListener('wheel', canvasWheel, { passive: false });
  wrap.addEventListener('focusin', canvasFocusIn);
  wrap.addEventListener('click', (e) => {  // the click a pan or a pinch ends with: not a click on a node
    if (!gesture.swallow) return;
    gesture.swallow = false;
    e.stopPropagation();
    e.preventDefault();
  }, true);
  return wrap;
}
$('canvas-fit').addEventListener('click', () => {
  if (!canvas.layout) return;
  canvasFollow(false);
  canvasFit(canvas.layout);
  canvasApply(false);
});
// Follow, or a tap on the step chip in the band's bubble (the running step's number): follow on, the node centred
function followNow() {
  if ($('canvas-follow').disabled) return;  // the List view: nothing to follow
  canvasFollow(true);
  const at = focusPoint();
  if (!at) return;
  canvasCentre(...at);
  canvasApply(true);
}
$('canvas-follow').addEventListener('click', followNow);
$('board-step').parentElement.addEventListener('click', followNow);

function renderGraph() {
  const box = $('plan-graph');
  const snap = state.snapshot;
  // not while the Run tab is hidden: the tab's button draws it again when the tab comes back (app.js)
  if (!box || box.hidden || $('view-run').hidden || !snap) return;
  const key = $('canvas').querySelector('.canvas-tools .legend');
  const { plan, live } = shownPlan();  // app.js: the live plan, or a preview to pick a start step in
  if (!plan || !plan.steps.length) {
    box.replaceChildren(el('p', { class: 'muted', text: plan?.message || 'No plan yet' }));
    key.replaceWith(legend(plan, {}, false));
    canvas.layer = canvas.layout = canvas.planId = null;  // the next plan with steps (the same id re-read) opens afresh
    return;
  }
  const active = document.activeElement;  // rebuilt on every update: keep the keyboard focus on its chip or head
  const refocus = !box.contains(active) ? null : active.dataset.step ? `[data-step="${CSS.escape(active.dataset.step)}"]`
    : active.dataset.band ? `[data-band="${CSS.escape(active.dataset.band)}"]` : 'sheetClose' in active.dataset ? '[data-sheet-close]' : null;
  const missionName = missionOf(plan);
  if (missionPlan[missionName] !== plan.plan_id) {  // a new plan: the mission file may have changed
    delete missionCache[missionName];
    missionPlan[missionName] = plan.plan_id;
  }
  const mission = missionCache[missionName];
  if (mission === undefined) missionFor(plan);
  const states = plan.steps.map((step, i) => stepState(snap, plan, live, step, i));
  const r = rules(plan, mission);
  const tree = treeFromPlan(plan, mission);
  const frames = framesOf(tree);
  const info = new Map(frames.map((f) => [f, frameInfo(f, states, r, plan.steps)]));
  // open: the frame with the running step always (the operator's choice does not hide it in a later cycle); else the
  // operator's choice, else open (an open tree is the canvas's point). A lane opens with its block.
  const stored = storedBands(missionName);
  const isOpen = (frame) => frame.kind === 'branch' || frame.leaves.some((i) => isLive(states[i])) || (stored[frame.key] ?? true);
  const events = live ? snap.outcomes : {};
  const pickable = snap.status.state !== 'running' && snap.status.state !== 'paused';
  const nodes = [];  // a step's tree node (its name), by its index
  const walk = (n) => { if (n.kind === 'step') nodes[n.index] = n; childrenOf(n).forEach(walk); };
  walk(tree);
  // width 0: the layout takes its own width (nothing stretched to the box), the camera moves it
  const ctx = { plan, states, r, events, snap, pickable, nodes, metrics: PHONE.matches ? GRAPH_METRICS.canvasPhone : GRAPH_METRICS.canvas,
    width: 0, isOpen, keyOf: (i) => plan.steps[i].id, loop: loopText(plan, mission),
    label: `Plan ${plan.plan_id}: ${plan.steps.length} steps`, ids: 'pg-run',
    start: state.startStep && plan.steps.some((s) => s.id === state.startStep) ? state.startStep : plan.steps[0].id,
    replans: plan.steps.map((s, i) => (jumps(r[i], i) ? [s.id, plan.steps[r[i].replanTo].id] : null)).filter(Boolean),
    stateOf: (it) => (it.node.kind === 'step' ? states[it.node.index] : info.get(it.node)?.word === 'done' ? 'succeeded'
      : ['running', 'paused', 'failed'].includes(info.get(it.node)?.word) ? info.get(it.node).word : 'pending'),
    frameWord: (frame) => info.get(frame).word,
    renderItem: (it) => (it.kind === 'stack' ? stackButton(it.node, info.get(it.node), toggle(it.node), snap) : chipButton(it.node.step, it.node.index, ctx)),
    renderHead: (frame) => frameHead(frame, info.get(frame), true, toggle(frame), snap) };
  const toggle = (frame) => () => { storeBand(missionName, frame.key, !isOpen(frame)); renderGraph(); };
  const sheetScroll = box.querySelector('.pg-sheet')?.scrollTop ?? 0;  // the sheet is rebuilt every second: keep its place
  const layout = layoutGraph(tree, ctx);
  const drawn = drawGraph(layout, ctx);
  canvas.wrap ||= canvasView();
  canvas.layer = div('pg-layer', drawn.node);
  canvas.layout = layout;
  // the picked step's details: on a phone the bottom sheet; on a laptop a pop-up beside its node (placePop)
  const picked = view.selected ? plan.steps.findIndex((s) => s.id === view.selected) : -1;
  if (picked < 0) view.selected = null;
  const card = picked < 0 ? [] : [planCard(picked, ctx, PHONE.matches)].flat();
  canvas.wrap.replaceChildren(canvas.layer, ...card);
  if (!box.contains(canvas.wrap)) box.replaceChildren(canvas.wrap);
  key.replaceWith(legend(plan, events, drawn.replans > 0));
  // the camera as it was on the new layer; a new plan opens fitted (following when it runs; at scale 1 on its node
  // when the fit would be tiny); a new followed node is centred with a follow move while follow is on
  canvasApply(false);
  // the followed node: the running step's, else the stopped one's, else the start step's (app.js focusIndex)
  const focus = layout.owner.get(plan.steps[focusIndex(snap.status, plan, events, live)].id) ?? null;
  if (canvas.planId !== plan.plan_id && canvas.wrap.clientWidth) {
    canvas.planId = plan.plan_id;
    canvas.focus = focus;
    canvasFit(layout);
    const at = canvas.scale < FIRST_FIT_MIN && focusPoint();
    if (at) {  // a tree that fits only as tiny chips: at full size on the followed node, following it
      canvas.scale = 1;
      canvasCentre(...at);
    }
    canvasFollow(Boolean(at) || states.some(isLive));
    canvasApply(false);
  } else if (focus !== canvas.focus) {
    canvas.focus = focus;
    const at = canvas.follow && focusPoint();
    if (at) {
      void canvas.layer.offsetWidth;  // the layer is new: its present place first, so the move eases from there
      canvasCentre(...at);
      canvasApply(true);
    }
  }
  const sheet = box.querySelector('.pg-sheet');
  if (sheet) sheet.scrollTop = sheetScroll;
  if (refocus) {
    canvas.refocusing = true;
    (box.querySelector(refocus) || box.querySelector('[data-step]'))?.focus({ preventScroll: true });
    canvas.refocusing = false;
  }
}
PHONE.addEventListener('change', () => { canvas.planId = null; renderGraph(); });  // other metrics: the plan opens afresh (fit or full size)
// a new size of the canvas (a window resize, the band's breakpoint): drawn again; with follow on the followed node is
// centred again, else the camera stays
let canvasSize = '';
new ResizeObserver(([entry]) => {
  const { width, height } = entry.contentRect;
  const size = `${Math.round(width)}x${Math.round(height)}`;
  if (!width || size === canvasSize) return;
  canvasSize = size;
  const at = canvas.follow && canvas.wrap && focusPoint();
  if (at) canvasCentre(...at);
  renderGraph();
}).observe($('plan-graph'));

function setPlanView(choice) {
  try { localStorage.setItem('tw-plan-view', choice); } catch (_) { /* private mode */ }
  document.querySelectorAll('#plan-view button').forEach((b) => b.setAttribute('aria-pressed', String(b.dataset.view === choice)));
  show($('plan'), choice === 'list');
  show($('plan-graph'), choice === 'graph');
  $('canvas-fit').disabled = choice !== 'graph';  // the camera's tools: nothing to move under the list
  $('canvas-follow').disabled = choice !== 'graph';
  renderGraph();
}
document.querySelectorAll('#plan-view button').forEach((b) => b.addEventListener('click', () => setPlanView(b.dataset.view)));
let initialView = 'graph';
try { initialView = localStorage.getItem('tw-plan-view') || 'graph'; } catch (_) { /* private mode */ }
setPlanView(initialView);
