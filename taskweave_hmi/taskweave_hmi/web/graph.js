'use strict';
// taskweave HMI, Run tab: the running plan as a state machine (nodeGraph). Steps are icon nodes joined by arrows in
// run order. Phases are groups: one per include of the top mission (origin.top + origin.include[0]) and one per run of
// its own steps, nested includes inside them; each collapses to one stack node, remembered per mission. Only
// exceptions are marked: the phase header names the usual failure rule, a step whose rule differs gets a badge (↻2,
// skip), a replan is a dashed red arrow to the step it replans from (replan_from). Nodes and phases can be dragged.
// A step opens its details in a pop-up beside it (a bottom sheet at 600 px and below). Nodes are tinted by what moves
// (--kind-*: drive, arm, grip, wait, other) and show the step's icon (icons.js). Loop rules (stop_when, max_cycles)
// and phase names come from the mission file. Uses $, el, api, state, show, missionOf, shownPlan, pickStart, moveLabel,
// moveParts, carriesByArgs from app.js and skillIcon, iconElement, moveKindOfArgs from icons.js.

const SVG = 'http://www.w3.org/2000/svg';
const LANES = ['base', 'hands', 'torso', 'joints'];  // core's resource names (PlanStep.resources), in this order
// what moves: a move step's kind, else the skill (cell schema 3 names too)
const KIND_COLOURS = { navigate: 'drive', straight: 'drive', hand: 'arm', posture: 'arm', joints: 'arm' };
const SKILL_COLOURS = { navigate_to: 'drive', drive_straight: 'drive', move_joints: 'arm', move_gripper: 'arm',
  move_object: 'arm', set_posture: 'arm', grip: 'grip', wait: 'wait' };
const ICON_COLOURS = { navigate: 'drive', straight: 'drive', hand: 'arm', carry: 'arm', posture: 'arm', joints: 'arm',
  'grip-close': 'grip', 'grip-open': 'grip', wait: 'wait', other: 'other' };
// the node graph (px): a step node and a closed phase's stack, the slot pitch, a phase's header room and padding, the
// gap between phases, the box's left gutter (the start dot, the loop arrow) and right margin, the room below (the
// loop arrow), the most a node is nudged off its slot, the height of the wave a row runs in
const N = { node: 44, stack: 52, pitchX: 104, pitchY: 92, head: 30, pad: 16, gap: 30, left: 56, right: 12, top: 16,
  bottom: 56, wobble: 12, wave: 12 };
const N_PHONE = { ...N, pitchX: 78, pitchY: 88, left: 36, gap: 22, wobble: 7, wave: 8 };
// a phase of 4 to 16 steps is a ring: its steps this far apart (of the slot pitch), the opening at its left (rad)
const RING = { min: 4, max: 16, spacing: 0.86, phoneSpacing: 1.12, gap: 0.7 };
const LAYOUT_KEY = 'tw-plan-layout';  // localStorage: {mission: {node or phase key: [dx, dy]}}
const layouts = {};
const drag = { active: false };  // a drag is on: the status updates do not redraw under it
const POP_W = 320;     // the step pop-up's width (style.css .pg-pop), the gap to its node, the least room it gets below
const POP_GAP = 12;
const POP_MIN_H = 260;
const BREATHE_MS = 2800;  // period of node-breathe for the running step (style.css)
const PHONE = window.matchMedia('(max-width: 600px)');
const WARN_STATES = ['timeout', 'canceled', 'skipped'];  // amber chips; failed and rejected are red
const BANDS_KEY = 'tw-plan-bands';  // localStorage: {mission: {band key: open}}
const missionCache = {};  // mission name -> its file (band names, defaults, loop), until a new plan or a save
const missionPlan = {};   // mission name -> the plan_id its cached file was read for
const view = { selected: null };  // the step (id) whose card shows instead of the running one

function svg(tag, attrs = {}, text) {
  const node = document.createElementNS(SVG, tag);
  Object.entries(attrs).forEach(([key, value]) => node.setAttribute(key, value));
  if (text !== undefined) node.textContent = text;
  return node;
}
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
// the Missions tab saved this mission: read its file again at the next draw
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
  // data.torso: the step's per-leg torso reports [{leg, height, ...}] (object_motion, gripper_motion): the last leg's
  // height; a plain number kept for safety; else the plan's height for the step
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

// The phases of a plan: one band per include occurrence of the top mission and one per run of its own steps; the
// steps of a nested include are a sub-band (one level: deeper includes stay in their sub-band). A plan without
// origins (an older task handler) is one band. A band: {key, name, include, steps: [index], items: [{step}|{sub}]}.
function bandsOf(plan, mission) {
  const bands = [];
  plan.steps.forEach((step, i) => {
    const o = step.origin;
    const chain = o?.include || [];
    const group = !o ? 'all' : chain.length ? `i${o.top}:${chain[0]}` : 'own';
    let band = bands[bands.length - 1];
    if (band?.group !== group) {
      band = { group, key: group === 'own' ? `own${o.top}` : group, steps: [], items: [], include: chain[0] || '',
        name: chain.length ? (mission?.steps?.[o.top]?.instruction || chain[0]) : missionOf(plan) };
      bands.push(band);
    }
    band.steps.push(i);
    if (chain.length < 2) { band.items.push({ step: i }); return; }
    let sub = band.items[band.items.length - 1]?.sub;
    // a new sub-band when the include changes, or the same one starts over (one level deep, origin.step is the index
    // in chain[1]'s own steps: it falls at the next include of the same mission)
    if (sub?.group !== chain[1] || (chain.length === 2 && sub.inner !== undefined && o.step <= sub.inner)) {
      sub = { group: chain[1], key: `${band.key}/${chain[1]}@${i}`, name: chain[1], include: chain[1], steps: [],
        items: [] };
      band.items.push({ sub });
    }
    sub.steps.push(i);
    sub.items.push({ step: i });
    if (chain.length === 2) sub.inner = o.step;
  });
  return bands;
}

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

// A band's header data: done/total, its state word, the usual failure rule (the most common one), the lanes it uses.
function bandInfo(band, states, r, steps, lanes) {
  const done = band.steps.filter((i) => isFinished(states[i])).length;
  const count = {};
  band.steps.forEach((i) => { count[r[i].key] = (count[r[i].key] || 0) + 1; });
  const usual = Object.entries(count).sort((a, b) => b[1] - a[1])[0][0];
  const st = band.steps.map((i) => states[i]);
  const word = st.includes('paused') ? 'paused' : st.includes('running') ? 'running' : st.some(isFailed) ? 'failed'
    : done === band.steps.length ? 'done' : done ? 'stopped' : 'pending';
  const used = new Set(band.steps.flatMap((i) => steps[i].resources || []));
  return { done, usual, rule: r[band.steps.find((i) => r[i].key === usual)], word,
    lanes: lanes.filter((lane) => used.has(lane)) };
}

// The badge of a step whose rule differs from its band's (retries ↻2, or ↻0 for fewer than the band's none; skip,
// abort, retry; a replan to another step is its arrow). cls: 'fail' for failure-type badges, 'skip' muted.
function badgeOf(rule, i, usual) {
  const words = [];
  if (rule.key !== usual && rule.retries !== Number(usual.split('|')[1])) words.push(`↻${Math.max(rule.retries, 0)}`);
  if (!jumps(rule, i) && rule.key !== usual && rule.onFailure !== usual.split('|')[0]) words.push(rule.onFailure);
  if (!words.length) return null;
  return { text: words.join(' '), cls: words.length === 1 && rule.onFailure === 'skip' && rule.retries <= 0 ? 'skip' : 'fail' };
}

function chipButton(step, i, ctx) {
  const { states, r, events, pickable } = ctx;
  const status = states[i];
  const start = pickable && step.id === state.startStep;
  const chip = el('button', { type: 'button', 'data-step': step.id,
    class: `pg-chip kind-${colourOf(step)} state-${status}${WARN_STATES.includes(status) ? ' state-warn' : ''}`
      + `${start ? ' start-here' : view.selected === step.id ? ' selected' : ''}`,
    'aria-label': `Step ${i + 1}: ${step.instruction} (${status})` }, iconElement(iconOf(step, events[step.id])));
  const held = carrying(step, events[step.id]);
  chip.title = [`${i + 1}. ${step.instruction}`, `${stepTitle(step)} · ${mainArg(step)}`,
    held ? (held === true ? 'carries a held object' : `carrying ${held}`) : '',
    step.resources?.length ? `uses ${step.resources.join(', ')}` : '', `on failure: ${failureText(r[i])}`,
    ...(step.notes || []).map((note) => `note: ${note}`)].filter(Boolean).join('\n');
  // the graph is rebuilt on every status update: breathe at the phase of a page-wide clock, so it runs on
  if (isLive(status)) chip.style.animationDelay = `-${(performance.now() % BREATHE_MS) / 1000}s`;
  if (status === 'succeeded') chip.append(el('span', { class: 'pg-dot ok' }));
  if (isFailed(status)) chip.append(el('span', { class: 'pg-dot bad', text: '✕' }));
  const badge = badgeOf(r[i], i, ctx.usual[i]);
  if (badge) chip.append(el('span', { class: `pg-badge ${badge.cls}`, text: badge.text }));
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
// a click outside the pop-up (laptop) closes it; a node's own click opens or closes as above
document.addEventListener('pointerdown', (e) => {
  if (PHONE.matches || !view.selected || e.target.closest?.('.pg-pop, .pg-chip, .pg-band, .pg-stack, .pg-tidy')) return;
  view.selected = null;
  renderGraph();
});

const bandTitle = (band, info) => `${band.name}${band.include && band.include !== band.name ? ` (${band.include})` : ''}`
  + `\n${info.done} of ${band.steps.length} done · ${info.word} · fails: ${ruleText(info.rule)}`
  + `${info.lanes.length ? `\nuses ${info.lanes.join(', ')}` : ''}`;
// An open phase's header on its outline: chevron and name, done/total · fails: rule; a click closes the phase
function bandHeader(band, info, open, toggle) {
  const head = el('button', { type: 'button', class: `pg-band${open ? ' open' : ''}`, 'aria-expanded': String(open),
    'data-band': band.key, title: bandTitle(band, info) },
  el('span', { class: 'pg-chev', 'aria-hidden': 'true' }), el('span', { class: 'pg-band-name', text: band.name }),
  el('span', { class: 'pg-band-meta', text: `${info.done}/${band.steps.length} · fails: ${ruleText(info.rule)}` }));
  head.addEventListener('click', toggle);
  return head;
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
    // the bottom sheet (a dialog, not modal: the status bar's Pause and Abort stay in reach above the scrim): the same
    // rows as the side card, the notes as amber boxes, Close; a tap on the scrim, Escape or the chip close it too
    const title = el('div', { class: 'pg-sheet-title', id: 'pg-sheet-title', text: `${i + 1} · ${step.instruction}` });
    const close = el('button', { type: 'button', class: 'btn pg-sheet-close', 'data-sheet-close': '', text: 'Close' });
    close.addEventListener('click', () => closeSheet(step.id));
    const scrim = div('pg-scrim');
    scrim.addEventListener('click', () => closeSheet(step.id));
    const sheet = el('section', { class: `pg-sheet state-${status}`, role: 'dialog', 'aria-labelledby': 'pg-sheet-title' },
      div('pg-handle'), div('pg-sheet-row', tile, div('pg-sheet-text', eyebrow, title)), progress, rows(false),
      ...(step.notes || []).map((note) => el('p', { class: 'pg-sheet-note', text: note })), close);
    sheet.addEventListener('keydown', (e) => { if (e.key === 'Tab') { e.preventDefault(); close.focus(); } });  // stays in
    return [scrim, sheet];
  }
  // the laptop: a pop-up beside the step's chip (a non-modal dialog; Close, Escape, a click elsewhere or the chip
  // again close it)
  const close = el('button', { type: 'button', class: 'btn small pg-pop-close', 'data-sheet-close': '', text: 'Close' });
  close.addEventListener('click', () => closeSheet(step.id));
  const pop = el('aside', { class: `pg-card pg-pop state-${status}${live || start ? ' marked' : ''}`, role: 'dialog',
    'aria-labelledby': 'pg-pop-title' },
  div('pg-card-head', tile, div('pg-card-titles', eyebrow,
    el('div', { class: 'pg-card-title', id: 'pg-pop-title', text: `${i + 1} · ${step.instruction}` }))),
  progress, rows(true), el('div', { class: 'row pg-pop-foot' }, close));
  pop.addEventListener('keydown', (e) => { if (e.key === 'Tab') { e.preventDefault(); close.focus(); } });  // stays in
  return pop;
}
// the sheet closes: nothing selected, the keyboard focus back on the step's chip
function closeSheet(id) {
  view.selected = null;
  renderGraph();
  $('plan-graph').querySelector(`[data-step="${CSS.escape(id)}"]`)?.focus();
}

// the step the card shows: the selected one, else the running one, else (a live run that failed) the failed one,
// else the start step, else the first
function focusIndex(plan, states) {
  const ids = plan.steps.map((s) => s.id);
  if (view.selected && !ids.includes(view.selected)) view.selected = null;
  return [ids.indexOf(view.selected), states.findIndex(isLive), states.findIndex(isFailed), ids.indexOf(state.startStep), 0]
    .find((i) => i >= 0);
}

function legend(plan, events, anyReplan) {
  const labels = { navigate: moveLabel('navigate'), straight: moveLabel('straight'), hand: moveLabel('hand'),
    carry: 'carrying', posture: moveLabel('posture'), joints: moveLabel('joints'), 'grip-close': 'grip close',
    'grip-open': 'grip open', wait: 'wait', other: 'other' };
  const icons = [...new Set(plan.steps.map((step) => iconOf(step, events[step.id])))];
  return div('legend',
    ...icons.map((id) => el('span', { class: `legend-item kind-${ICON_COLOURS[id] || 'other'}` }, iconElement(id),
      labels[id] || id)),
    anyReplan ? el('span', { class: 'legend-item' }, el('i', { class: 'swatch-fail' }), 'replan') : '');
}

// the line under the last band when the plan repeats: "↻ until no_frames · cycle 1 · max 10"
function loopText(plan, mission, snap) {
  const loop = plan.loop || mission?.loop || 'once';
  if (loop === 'once') return '';
  const until = mission?.stop_when ? `until ${mission.stop_when}` : (loop === 'forever' ? 'forever' : loop);
  const cycle = snap.status.plan_id === plan.plan_id && snap.status.cycle ? `cycle ${snap.status.cycle}` : '';
  return [`↻ ${until}`, cycle, mission?.max_cycles ? `max ${mission.max_cycles}` : ''].filter(Boolean).join(' · ');
}

const lanesOf = (plan) => [...new Set([...LANES, ...plan.steps.flatMap((s) => s.resources || [])])];

// the operator's drags, per mission: {node or group key: [dx, dy]} on top of the drawn layout (a group's key moves
// all of its nodes); kept here too, so private mode keeps them until the page reloads
function storedLayout(mission) {
  if (!(mission in layouts)) {
    try { layouts[mission] = (JSON.parse(localStorage.getItem(LAYOUT_KEY) || '{}') || {})[mission] || {}; } catch (_) { layouts[mission] = {}; }
  }
  return layouts[mission];
}
function storeLayout(mission, offsets) {
  layouts[mission] = offsets;
  try {
    const all = JSON.parse(localStorage.getItem(LAYOUT_KEY) || '{}') || {};
    if (Object.keys(offsets).length) all[mission] = offsets; else delete all[mission];
    localStorage.setItem(LAYOUT_KEY, JSON.stringify(all));
  } catch (_) { /* private mode */ }
}
// a small fixed nudge per node (from its key): the graph looks drawn, not gridded, and the same on every draw
function wobble(key, amount) {
  let h = 2166136261;
  for (let k = 0; k < key.length; k += 1) h = Math.imul(h ^ key.charCodeAt(k), 16777619);
  return [((h & 255) / 255 - 0.5) * 2 * amount, (((h >>> 8) & 255) / 255 - 0.5) * 2 * amount];
}

// a closed phase: one node, a stack of cards filled to its share done, its name under it; a click opens it
function stackButton(band, info, toggle) {
  const total = band.steps.length;
  const node = el('button', { type: 'button', class: `pg-stack ${info.word}`, 'data-band': band.key,
    'aria-expanded': 'false', title: bandTitle(band, info),
    style: `--done-share: ${Math.round((info.done / total) * 100)}%` },
  el('span', { class: 'pg-stack-count', text: `${info.done}/${total}` }));
  node.addEventListener('click', toggle);
  return node;
}

// where the line from a node's centre toward (tx, ty) leaves its box, gap px outside it
function rim(n, tx, ty, gap) {
  const dx = tx - n.x;
  const dy = ty - n.y;
  const s = Math.min((n.w / 2 + gap) / Math.abs(dx || 1e-9), (n.h / 2 + gap) / Math.abs(dy || 1e-9), 1);
  return { x: n.x + dx * s, y: n.y + dy * s };
}
// a transition from node a to node b, bowed sideways by bend px: its path and its middle (for a label)
function wire(a, b, bend) {
  const d = Math.hypot(b.x - a.x, b.y - a.y) || 1;
  const mx = (a.x + b.x) / 2 - ((b.y - a.y) / d) * bend;
  const my = (a.y + b.y) / 2 + ((b.x - a.x) / d) * bend;
  const p = rim(a, mx, my, 3);
  const q = rim(b, mx, my, 5);
  return { d: `M${p.x},${p.y} Q${mx},${my} ${q.x},${q.y}`, mid: { x: (p.x + 2 * mx + q.x) / 4, y: (p.y + 2 * my + q.y) / 4 } };
}
// a polyline with its corners rounded (radius px)
function rounded(points, radius) {
  let d = `M${points[0][0]},${points[0][1]}`;
  for (let k = 1; k < points.length - 1; k += 1) {
    const [px, py] = points[k - 1];
    const [x, y] = points[k];
    const [nx, ny] = points[k + 1];
    const r1 = Math.min(radius, Math.hypot(x - px, y - py) / 2);
    const r2 = Math.min(radius, Math.hypot(nx - x, ny - y) / 2);
    const a = Math.hypot(x - px, y - py) || 1;
    const b = Math.hypot(nx - x, ny - y) || 1;
    d += ` L${x - ((x - px) / a) * r1},${y - ((y - py) / a) * r1} Q${x},${y} ${x + ((nx - x) / b) * r2},${y + ((ny - y) / b) * r2}`;
  }
  const [lx, ly] = points[points.length - 1];
  return `${d} L${lx},${ly}`;
}

// The plan as a state machine: steps are nodes joined by transitions in run order, a phase an outline round its
// steps with its header on the edge (a closed phase one stack node: the transitions point at it), a nested include a
// soft band through its steps. The phases flow across the box and wind inside (a snake, nudged per node); the
// operator can drag a node or a phase by its header (mouse or pen; stored per mission, the ↺ button tidies). A dot
// points at the start step, a dashed loop runs from the last step back to the first when the mission repeats (else a
// final ring), a replan is a dashed red arrow to the step it replans from. Done transitions are solid, the one into
// the running step flows, those ahead are dotted.
function nodeGraph(bands, ctx, end) {
  const { plan, states, r } = ctx;
  const K = ctx.phone ? N_PHONE : N;
  const width = Math.max(ctx.width || 0, 300);
  const offsets = storedLayout(ctx.missionName);
  const nodes = new Map();  // key -> {key, x, y, w, h, el, label, moves: [offset keys]}
  const stepKey = [];       // per step: the node that shows it (its own, or its closed phase's)
  const outlines = [];      // open phases: {band, keys, head, sub}
  const graph = el('div', { class: 'pg-graph', role: 'group', 'aria-label': `Plan ${plan.plan_id}: ${plan.steps.length} steps` });
  const back = svg('svg', { class: 'pg-lines', 'aria-hidden': 'true' });
  const wires = svg('svg', { class: 'pg-lines', 'aria-hidden': 'true' });
  const notes = svg('svg', { class: 'pg-lines', 'aria-hidden': 'true' });  // the arrows' words, over the nodes
  graph.append(back, wires);

  const addStep = (i, moves) => {
    const step = plan.steps[i];
    const node = chipButton(step, i, ctx);
    const label = el('span', { class: `pg-label${isLive(states[i]) ? ' now' : ''}` }, el('b', { text: String(i + 1) }),
      ` ${mainArg(step) || stepTitle(step)}`);
    graph.append(node, label);
    stepKey[i] = step.id;
    nodes.set(step.id, { key: step.id, w: K.node, h: K.node, el: node, label, moves: [...moves, step.id] });
    return step.id;
  };
  const addStack = (band, moves) => {
    const key = `g:${band.key}`;
    const node = stackButton(band, ctx.info.get(band), ctx.toggle(band));
    const label = el('span', { class: 'pg-label' }, band.name);
    graph.append(node, label);
    band.steps.forEach((i) => { stepKey[i] = key; });
    nodes.set(key, { key, w: K.stack, h: K.stack, el: node, label, moves: [...new Set([...moves, key])] });
    return key;
  };
  const addHead = (band, keys, sub, moves) => {
    const head = bandHeader(band, ctx.info.get(band), true, ctx.toggle(band));
    if (sub) head.classList.add('pg-sub');
    graph.append(head);
    outlines.push({ band, keys, head, sub, moves });
  };

  // 1. blocks: a closed phase is one slot, an open one a snake of its steps (an open nested include's steps in it, a
  // closed one a slot)
  const maxCols = Math.max(2, Math.floor((width - K.left - K.right - 2 * K.pad - K.pitchX / 2) / K.pitchX));
  const blocks = bands.map((band) => {
    const group = `g:${band.key}`;
    if (!ctx.isOpen(band)) return { keys: [addStack(band, [group])], cols: 1, rows: 1, head: 0, w: K.pitchX, h: K.pitchY };
    const keys = [];
    band.items.forEach((it) => {
      if (!it.sub) { keys.push(addStep(it.step, [group])); return; }
      const moves = [group, `g:${it.sub.key}`];
      if (!ctx.isOpen(it.sub)) { keys.push(addStack(it.sub, moves)); return; }
      const own = it.sub.items.map((s) => addStep(s.step, moves));
      keys.push(...own);
      addHead(it.sub, own, true, moves);
    });
    addHead(band, keys, false, [group]);
    if (keys.length >= RING.min && keys.length <= RING.max) {  // a loop of its steps round an ellipse
      const R = (K.pitchX * (ctx.phone ? RING.phoneSpacing : RING.spacing) * (keys.length - 1)) / (2 * Math.PI - RING.gap);  // a circle's radius for them
      const rx = Math.min(R * 1.25, (width - K.left - K.right - 2 * K.pad - K.pitchX) / 2);
      const ry = Math.sqrt(Math.max(2 * R * R - rx * rx, R * R * 0.36));  // about the same way round
      return { keys, ring: { rx, ry }, head: K.head, w: 2 * rx + K.pitchX + 2 * K.pad, h: K.head + 2 * ry + K.pitchY };
    }
    const cols = Math.min(maxCols, keys.length, Math.max(2, Math.round(Math.sqrt(keys.length * 2.2))));
    const rows = Math.ceil(keys.length / cols);
    return { keys, cols, rows, head: K.head, w: cols * K.pitchX + (rows > 1 ? K.pitchX / 2 : 0) + 2 * K.pad,
      h: K.head + rows * K.pitchY };
  });
  // 2. the blocks flow across the box in lines, spread over the spare width and set off a little from one another
  const lines = [];
  blocks.forEach((b) => {
    const last = lines[lines.length - 1];
    if (!last || last.w + K.gap + b.w > width - K.left - K.right) lines.push({ blocks: [b], w: b.w });
    else { last.blocks.push(b); last.w += K.gap + b.w; }
  });
  let y = K.top;
  lines.forEach((line, l) => {
    line.blocks.forEach((b) => { b.line = l; });
    const spare = Math.max(0, width - K.left - K.right - line.w);
    const many = line.blocks.length > 1;
    let x = K.left + (many ? spare * 0.15 : 0);  // one alone: at the left, by the rings' opening and the loop
    let bottom = y;
    line.blocks.forEach((b, k) => {
      b.x = x;
      b.y = y + [0, 28, 10, 36][(l + k) % 4];
      x += b.w + K.gap + (many ? (spare * 0.7) / (line.blocks.length - 1) : 0);
      bottom = Math.max(bottom, b.y + b.h);
    });
    y = bottom + K.gap;
  });
  // 3. the nodes: round the ring clockwise from its upper left, evenly by length, the gap left open at the left (the
  // way in and out); else a snake through the block's rows (every other row back and half a slot over) on a gentle
  // wave. Each nudged, then the operator's drags.
  blocks.forEach((b) => {
    const spots = b.ring ? ringSpots(b, b.keys.length) : null;
    b.keys.forEach((key, k) => {
      const [jx, jy] = wobble(key, K.wobble);
      const n = nodes.get(key);
      n.line = b.line;
      const moved = n.moves.reduce((sum, m) => [sum[0] + (offsets[m]?.[0] || 0), sum[1] + (offsets[m]?.[1] || 0)], [0, 0]);
      if (spots) {
        n.ring = b.ring;
        n.x = spots[k][0] + jx * 0.6 + moved[0];
        n.y = spots[k][1] + jy * 0.6 + moved[1];
      } else {
        const row = Math.floor(k / b.cols);
        const col = row % 2 ? b.cols - 1 - (k % b.cols) : k % b.cols;
        n.x = b.x + K.pad + (col + 0.5 + (row % 2 ? 0.5 : 0)) * K.pitchX + jx + moved[0];
        n.y = b.y + b.head + row * K.pitchY + K.pitchY / 2 - 10 + Math.sin(col * 1.25 + row * 2) * K.wave + jy + moved[1];
      }
      clamp(n);
    });
  });
  function ringSpots(b, count) {
    const { rx, ry } = b.ring;
    b.ring.cx = b.x + b.w / 2;
    b.ring.cy = b.y + b.head + ry + K.node / 2 + 4;
    const from = Math.PI + RING.gap / 2;
    const pts = [];
    for (let k = 0; k <= 720; k += 1) {
      const a = from + ((2 * Math.PI - RING.gap) * k) / 720;
      pts.push([b.ring.cx + rx * Math.cos(a), b.ring.cy + ry * Math.sin(a)]);
    }
    const along = [0];
    for (let k = 1; k < pts.length; k += 1) along.push(along[k - 1] + Math.hypot(pts[k][0] - pts[k - 1][0], pts[k][1] - pts[k - 1][1]));
    const total = along[along.length - 1];
    let j = 0;
    return Array.from({ length: count }, (_, k) => {
      const want = (total * k) / (count - 1);
      while (j < along.length - 1 && along[j] < want) j += 1;
      return pts[j];
    });
  }
  function clamp(n) {
    n.x = Math.max(n.w / 2 + 4, Math.min(width - n.w / 2 - 4, n.x));
    n.y = Math.max(n.h / 2 + K.top, n.y);
  }

  const replans = plan.steps.map((_, i) => (jumps(r[i], i) ? i : -1)).filter((i) => i >= 0);
  const startAt = Math.max(0, plan.steps.findIndex((s) => s.id === state.startStep));
  const marker = (name) => {
    const m = svg('marker', { id: `pg-head-${name}`, class: `pg-head ${name}`, viewBox: '0 0 10 10', refX: 8, refY: 5,
      markerWidth: 9, markerHeight: 9, markerUnits: 'userSpaceOnUse', orient: 'auto-start-reverse' });
    m.append(svg('path', { d: 'M0,1 L9,5 L0,9 z' }));
    return m;
  };

  // the way down between phases: left of both nodes' outlines, right of the loop
  const bus = (a, b) => Math.max(K.left / 2 + 8, Math.min(a.x, b.x) - K.pitchX / 2 - K.pad / 2 - 10);
  // draws everything from the nodes' positions: on every build and on every step of a drag
  let height = 0;
  function paint() {
    const label = 18;  // a node's label under it
    let bottom = 0;
    nodes.forEach((n) => {
      n.el.style.left = `${n.x - n.w / 2}px`;
      n.el.style.top = `${n.y - n.h / 2}px`;
      n.label.style.left = `${n.x - K.pitchX / 2 + 4}px`;
      n.label.style.width = `${K.pitchX - 8}px`;
      n.label.style.top = `${n.y + n.h / 2 + 3}px`;
      bottom = Math.max(bottom, n.y + n.h / 2 + label);
    });
    back.replaceChildren();
    outlines.forEach((o) => {
      const members = o.keys.map((key) => nodes.get(key));
      if (o.sub) {  // a nested include: a soft band through its steps, its header over the first (in a ring: on
        // the inner side of the band's middle, in the ring's open middle)
        const pts = members.map((n) => `${n.x},${n.y}`).join(' ');
        back.append(svg('polyline', { class: 'pg-subband', points: pts, 'stroke-width': K.node + 18 }));
        let [hx, hy] = [members[0].x - K.node / 2 - 6, members[0].y - K.node / 2 - 30];
        if (members[0].ring) {
          const ring = [...nodes.values()].filter((n) => n.ring === members[0].ring);
          const [cx, cy] = [ring.reduce((t, n) => t + n.x, 0) / ring.length, ring.reduce((t, n) => t + n.y, 0) / ring.length];
          const mid = members[Math.floor((members.length - 1) / 2)];
          const d = Math.hypot(mid.x - cx, mid.y - cy) || 1;
          const reach = K.node / 2 + 30;
          hx = mid.x - ((mid.x - cx) / d) * reach;
          o.head.style.transform = 'translateX(-50%)';  // centred there (not in the page yet: no width to read)
          hy = mid.y - ((mid.y - cy) / d) * reach - 11;
        }
        o.head.style.left = `${hx}px`;
        o.head.style.top = `${hy}px`;
        return;
      }
      const left = Math.min(...members.map((n) => n.x - Math.max(n.w / 2, K.pitchX / 2 - 4))) - K.pad / 2;
      const right = Math.max(...members.map((n) => n.x + Math.max(n.w / 2, K.pitchX / 2 - 4))) + K.pad / 2;
      const top = Math.min(...members.map((n) => n.y - n.h / 2)) - K.head;
      const low = Math.max(...members.map((n) => n.y + n.h / 2 + label)) + K.pad / 2;
      back.append(svg('rect', { class: `pg-outline ${ctx.info.get(o.band).word}`, x: left, y: top, width: right - left,
        height: low - top, rx: 28 }));
      if (members[0].ring) {  // a ring's open middle: how far the phase is
        const cx = members.reduce((sum, n) => sum + n.x, 0) / members.length;
        const cy = members.reduce((sum, n) => sum + n.y, 0) / members.length;
        back.append(svg('text', { class: 'pg-ring-count', x: cx, y: cy + 12, 'text-anchor': 'middle' },
          `${ctx.info.get(o.band).done}/${o.band.steps.length}`));
      }
      o.head.style.left = `${left + 10}px`;
      o.head.style.top = `${top - 13}px`;
      o.head.style.maxWidth = `${right - left - 20}px`;
      o.box = { left, right, top, low };
      bottom = Math.max(bottom, low);
    });
    height = bottom + (end ? K.bottom : K.bottom / 2);
    [back, wires, notes].forEach((s) => { s.setAttribute('width', width); s.setAttribute('height', height); });
    graph.style.cssText = `width: ${width}px; height: ${height}px`;

    const defs = svg('defs');
    ['done', 'now', 'todo', 'fail', 'loop'].forEach((name) => defs.append(marker(name)));
    wires.replaceChildren(defs);
    notes.replaceChildren();
    const seen = new Set();
    for (let i = 1; i < plan.steps.length; i += 1) {
      const a = stepKey[i - 1];
      const b = stepKey[i];
      if (a === b || seen.has(`${a}>${b}`)) continue;
      seen.add(`${a}>${b}`);
      const cls = isLive(states[i]) ? 'now' : states[i] !== 'pending' ? 'done' : 'todo';
      const na = nodes.get(a);
      const nb = nodes.get(b);
      // round a ring the arrows bow outwards, with it; elsewhere they bow either way in turn
      const bend = Math.min(28, Math.hypot(nb.x - na.x, nb.y - na.y) * 0.16);
      const out = na.ring && na.ring === nb.ring
        ? (((nb.x - na.x) * (na.y + nb.y - 2 * na.ring.cy) - (nb.y - na.y) * (na.x + nb.x - 2 * na.ring.cx)) > 0 ? 1 : -1) : 0;
      // into a phase further down: out to the left of the outlines, down, and in from the left (rings open there)
      const d = nb.line > na.line ? rounded([[na.x - na.w / 2 - 3, na.y], [bus(na, nb), na.y], [bus(na, nb), nb.y],
        [nb.x - nb.w / 2 - 5, nb.y]], 16) : wire(na, nb, out ? bend * 0.7 * out : bend * (i % 2 ? 1 : -1)).d;
      wires.append(svg('path', { class: `pg-wire ${cls}`, d, 'marker-end': `url(#pg-head-${cls})` }));
    }
    replans.forEach((i) => {
      const a = nodes.get(stepKey[i]);
      const b = nodes.get(stepKey[r[i].replanTo]);
      if (a === b) return;
      const w = wire(a, b, Math.min(70, 26 + Math.hypot(b.x - a.x, b.y - a.y) * 0.25));
      wires.append(svg('path', { class: 'pg-wire fail', d: w.d, 'marker-end': 'url(#pg-head-fail)' }));
      notes.append(svg('text', { class: 'pg-wire-label fail', x: w.mid.x, y: w.mid.y + 4, 'text-anchor': 'middle' }, 'replan'));
    });
    // the start: a dot beside the start step, where no node is
    const first = nodes.get(stepKey[startAt]);
    const free = ([dx, dy]) => [...nodes.values()].every((n) => Math.hypot(n.x - first.x - dx, n.y - first.y - dy) > 40);
    const [sx, sy] = [[-58, -28], [-58, 26], [0, -64], [58, -30], [0, 64]].find(free) || [-58, -28];
    const dot = { x: first.x + sx, y: first.y + sy, w: 12, h: 12 };
    wires.append(svg('circle', { class: 'pg-init', cx: dot.x, cy: dot.y, r: 6 }),
      svg('path', { class: 'pg-wire done', d: wire(dot, first, 0).d, 'marker-end': 'url(#pg-head-done)' }));
    const last = nodes.get(stepKey[plan.steps.length - 1]);
    const head = nodes.get(stepKey[0]);
    if (end) {  // the loop: from the last step out to the left gutter (under everything when a node is in the way),
      // up it and into the first step; its rule along the gutter
      const low = height - K.bottom / 2 - 4;
      const lx = Math.max(12, Math.min(head.x - head.w / 2 - 26, K.left / 2 - 2));
      const done = states.every((st) => st !== 'pending') && !states.some(isLive);
      const blocked = [...nodes.values()].some((n) => n !== last && n.x < last.x && Math.abs(n.y - last.y) < n.h);
      const way = blocked ? [[last.x, last.y + last.h / 2 + label + 2], [last.x, low], [lx, low]]
        : [[last.x - last.w / 2 - 3, last.y], [lx, last.y]];
      const mid = (way[way.length - 1][1] + head.y) / 2;
      wires.append(svg('path', { class: `pg-wire loop${done ? ' now' : ''}`, 'marker-end': 'url(#pg-head-loop)',
        d: rounded([...way, [lx, head.y], [head.x - head.w / 2 - 5, head.y]], 18) }));
      notes.append(svg('text', { class: 'pg-wire-label loop', x: lx - 4, y: mid, 'text-anchor': 'middle',
        transform: `rotate(-90 ${lx - 4} ${mid})` }, end));
    } else {  // a final ring beside the last step
      const freeEnd = ([dx, dy]) => [...nodes.values()].every((n) => Math.hypot(n.x - last.x - dx, n.y - last.y - dy) > 40);
      const [ex, ey] = [[62, 0], [0, 66], [-62, 0], [0, -66]].find(freeEnd) || [0, 66];
      const ring = { x: last.x + ex, y: last.y + ey, w: 18, h: 18 };
      wires.append(svg('path', { class: 'pg-wire todo', d: wire(last, ring, 0).d, 'marker-end': 'url(#pg-head-todo)' }),
        svg('circle', { class: 'pg-final', cx: ring.x, cy: ring.y, r: 9 }), svg('circle', { class: 'pg-final-dot', cx: ring.x, cy: ring.y, r: 5 }));
    }
  }

  // drag (mouse or pen; a finger scrolls): a node, a closed phase, or an open phase by its header (all its nodes)
  const grab = (handle, key, keys) => handle.addEventListener('pointerdown', (e) => {
    if (e.button !== 0 || e.pointerType === 'touch') return;
    const moving = keys().map((k) => nodes.get(k)).map((n) => ({ n, x: n.x, y: n.y }));
    let on = false;
    const move = (ev) => {
      const dx = ev.clientX - e.clientX;
      const dy = ev.clientY - e.clientY;
      if (!on && Math.hypot(dx, dy) < 5) return;
      if (!on) { on = true; drag.active = true; handle.classList.add('dragging'); }
      ev.preventDefault();
      moving.forEach((m) => { m.n.x = m.x + dx; m.n.y = m.y + dy; clamp(m.n); });
      paint();
    };
    const up = () => {  // pointerup or pointercancel, once
      window.removeEventListener('pointermove', move);
      window.removeEventListener('pointerup', up);
      window.removeEventListener('pointercancel', up);
      if (!on) return;
      const m = moving[0];
      const was = offsets[key] || [0, 0];
      storeLayout(ctx.missionName, { ...offsets, [key]: [was[0] + m.n.x - m.x, was[1] + m.n.y - m.y] });
      // the click that ends a drag is not a pick
      handle.addEventListener('click', (c) => { c.stopImmediatePropagation(); c.preventDefault(); }, { capture: true, once: true });
      setTimeout(() => { drag.active = false; renderGraph(); }, 0);
    };
    window.addEventListener('pointermove', move);
    window.addEventListener('pointerup', up);
    window.addEventListener('pointercancel', up);
  });
  nodes.forEach((n) => grab(n.el, n.moves[n.moves.length - 1], () => [n.key]));
  outlines.forEach((o) => grab(o.head, o.moves[o.moves.length - 1], () => o.keys));
  paint();
  graph.append(notes);

  const wrap = div('pg', graph);
  if (Object.keys(offsets).length) {  // the operator moved something: tidy puts the drawn layout back
    const tidy = el('button', { type: 'button', class: 'btn small pg-tidy', title: 'Tidy the layout', 'aria-label': 'Tidy the layout', text: '↺' });
    tidy.addEventListener('click', () => { storeLayout(ctx.missionName, {}); renderGraph(); });
    graph.append(tidy);
  }
  // the picked step's details: on a phone the bottom sheet; on a laptop a pop-up beside its node (right if it fits,
  // else left, else under it; renderGraph makes the graph tall enough for it)
  const picked = view.selected ? plan.steps.findIndex((s) => s.id === view.selected) : -1;
  if (picked >= 0 && ctx.phone) {
    wrap.classList.add('with-sheet');
    wrap.append(...planCard(picked, ctx, true));
  } else if (picked >= 0) {
    const n = nodes.get(stepKey[picked]);
    const pop = planCard(picked, ctx, false);
    const right = n.x + n.w / 2 + POP_GAP;
    const left = n.x - n.w / 2 - POP_GAP - POP_W;
    if (right + POP_W <= width) pop.style.left = `${right}px`;
    else if (left >= 0) pop.style.left = `${left}px`;
    else {
      pop.style.left = `${Math.max(0, Math.min(n.x - POP_W / 2, width - POP_W))}px`;
      pop.style.top = `${n.y + n.h / 2 + 22 + POP_GAP}px`;
    }
    pop.style.top ||= `${Math.max(0, Math.min(n.y - n.h / 2 - 8, height - POP_MIN_H))}px`;
    graph.append(pop);
  }
  return { node: wrap, replans: replans.length };
}

function renderGraph() {
  const box = $('plan-graph');
  const snap = state.snapshot;
  if (!box || box.hidden || !snap || drag.active) return;
  const { plan, live } = shownPlan();  // app.js: the live plan, or a preview to pick a start step in
  if (!plan || !plan.steps.length) {
    box.replaceChildren(el('p', { class: 'muted', text: plan?.message || 'No plan yet' }));
    return;
  }
  const active = document.activeElement;  // rebuilt on every update: keep the keyboard focus on its chip or band
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
  const bands = bandsOf(plan, mission);
  const lanes = lanesOf(plan);
  const info = new Map();
  const usual = [];
  const visit = (band) => {
    info.set(band, bandInfo(band, states, r, plan.steps, lanes));
    band.items.forEach((it) => it.sub && visit(it.sub));
  };
  bands.forEach(visit);
  bands.forEach((band) => band.steps.forEach((i) => { usual[i] = info.get(band).usual; }));
  // open: the band with the running step always (the operator's choice does not hide it in a later cycle); else the
  // operator's choice, else the band with the failed step (else the start step's, the first band)
  const anchor = [states.findIndex(isLive), states.findIndex(isFailed),
    plan.steps.findIndex((s) => s.id === state.startStep), 0].find((i) => i >= 0);
  const stored = storedBands(missionName);
  const isOpen = (band) => band.steps.some((i) => isLive(states[i]))
    || (band.key in stored ? stored[band.key] : band.steps.includes(anchor));
  const end = loopText(plan, mission, snap);
  const ctx = { plan, states, r, events: live ? snap.outcomes : {}, snap, usual, info, isOpen, phone: PHONE.matches,
    width: box.clientWidth, missionName,
    pickable: snap.status.state !== 'running' && snap.status.state !== 'paused', focus: focusIndex(plan, states),
    toggle: (band) => () => { storeBand(missionName, band.key, !isOpen(band)); renderGraph(); } };
  const sheetScroll = box.querySelector('.pg-sheet')?.scrollTop ?? 0;  // the sheet is rebuilt every second: keep its place
  const drawn = nodeGraph(bands, ctx, end);
  box.replaceChildren(drawn.node, legend(plan, ctx.events, drawn.replans > 0));
  const sheet = box.querySelector('.pg-sheet');
  if (sheet) sheet.scrollTop = sheetScroll;
  // the pop-up is placed over the graph: the graph grows to hold it (one under a low node, a tall one)
  const pop = box.querySelector('.pg-pop');
  const graph = box.querySelector('.pg-graph');
  if (pop && graph) graph.style.height = `${Math.max(graph.offsetHeight, pop.offsetTop + pop.offsetHeight)}px`;
  if (refocus) (box.querySelector(refocus) || box.querySelector('[data-step]'))?.focus({ preventScroll: true });
}
PHONE.addEventListener('change', () => renderGraph());
// the layout follows the box's width: drawn again when it changes (a window resize, the Run grid's breakpoint)
let graphWidth = 0;
new ResizeObserver(([entry]) => {
  const width = Math.round(entry.contentRect.width);
  if (width && width !== graphWidth) { graphWidth = width; renderGraph(); }
}).observe($('plan-graph'));

function setPlanView(view) {
  try { localStorage.setItem('tw-plan-view', view); } catch (_) { /* private mode */ }
  document.querySelectorAll('#plan-view button').forEach((b) => b.setAttribute('aria-pressed', String(b.dataset.view === view)));
  show($('plan'), view === 'list');
  show($('plan-graph'), view === 'graph');
  renderGraph();
}
document.querySelectorAll('#plan-view button').forEach((b) => b.addEventListener('click', () => setPlanView(b.dataset.view)));
let initialView = 'graph';
try { initialView = localStorage.getItem('tw-plan-view') || 'graph'; } catch (_) { /* private mode */ }
setPlanView(initialView);
