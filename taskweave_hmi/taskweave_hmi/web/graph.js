'use strict';
// taskweave HMI, Run tab: the running plan as a graph of icon chips (G3). Steps run top to bottom, one row each; a
// step's chip sits in the lane of the resource it holds (base, hands, torso, joints: step.resources), a bar with dots
// runs across the other lanes it holds. Phases are bands: one per include of the top mission (origin.top +
// origin.include[0]) and one per run of its own steps, nested includes as sub-bands; each band collapses to one
// row, remembered per mission. Only exceptions are marked: the band header names the usual failure rule, a step
// whose rule differs gets a badge (↻2, skip), a replan is the one line (dashed, through the left gutter to the step
// it replans from: replan_from). Neighbouring steps on disjoint resources get a dashed box (parallel candidates,
// display only). The running or selected step opens in the side card. At 600 px and below the lanes collapse into
// a rail of 44 px chips per band and the card becomes a bottom sheet. Chips are tinted by what moves (--kind-*:
// drive, arm, grip, wait, other) and show the step's icon (icons.js). Loop rules (stop_when, max_cycles) and band
// names come from the mission file. Uses $, el, api, state, show, missionOf, shownPlan, pickStart, moveLabel,
// moveParts, carriesByArgs from app.js and skillIcon, iconElement, moveKindOfArgs from icons.js.

const SVG = 'http://www.w3.org/2000/svg';
const LANES = ['base', 'hands', 'torso', 'joints'];  // core's resource names (PlanStep.resources), left to right
const FIRST_LANE = 'hands';  // a first step that holds nothing goes here (later ones: the previous step's lane)
// what moves: a move step's kind, else the skill (cell schema 3 names too)
const KIND_COLOURS = { navigate: 'drive', straight: 'drive', hand: 'arm', posture: 'arm', joints: 'arm' };
const SKILL_COLOURS = { navigate_to: 'drive', drive_straight: 'drive', move_joints: 'arm', move_gripper: 'arm',
  move_object: 'arm', set_posture: 'arm', grip: 'grip', wait: 'wait' };
const ICON_COLOURS = { navigate: 'drive', straight: 'drive', hand: 'arm', carry: 'arm', posture: 'arm', joints: 'arm',
  'grip-close': 'grip', 'grip-open': 'grip', wait: 'wait', other: 'other' };
// the laptop graph (px): number gutter, lane, row pitch, chip, band header, gaps; the replan lines' area left of it
// (Figma has rows of 34 and bands 14 apart: 32 and 10 let frame_cycle's running phase fit under the sticky header at 768)
const G = { gutter: 28, lane: 64, row: 32, chip: 28, inset: 4, head: 24, laneHead: 24, bandGap: 10, headGap: 6, subIndent: 12,
  pairOut: 3, replanArea: 22, replanGap: 6, endGap: 6, endH: 18 };
const POP_W = 320;     // the step pop-up's width (style.css .pg-pop), the gap to its chip, the least room it gets below
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
// height the step ran at (data.torso). Planned: "Torso 0.35 m · planned change at this step (from 0.494)" or
// "Torso 0.494 m · kept (plan)"; cut, capped or stopped: "chosen per step"; infeasible and certain: its message, red.
// A step that ran a planned change on a state that differed says "replanned". null: no report covers the step.
const metres = (v) => `${+Number(v).toFixed(3)} m`;
function torsoLine(step, events) {
  const reports = Object.values(events || {}).map((e) => e?.data?.torso_plan).filter((p) => p?.window?.includes(step.id));
  const own = events?.[step.id]?.data;
  const plan = own?.torso_plan?.window?.includes(step.id) ? own.torso_plan : reports.at(-1);
  if (!plan) return null;
  const height = own?.torso ?? plan.heights?.[step.id];
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
// abort, retry), and on the phone a replan's target ("to 7"; on the laptop that is the line). cls: 'fail' for
// failure-type badges, 'skip' muted.
function badgeOf(rule, i, usual, phone) {
  const words = [];
  if (rule.key !== usual && rule.retries !== Number(usual.split('|')[1])) words.push(`↻${Math.max(rule.retries, 0)}`);
  if (jumps(rule, i)) { if (phone) words.push(`to ${rule.replanTo + 1}`); }
  else if (rule.key !== usual && rule.onFailure !== usual.split('|')[0]) words.push(rule.onFailure);
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
  const badge = badgeOf(r[i], i, ctx.usual[i], ctx.phone);
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
// a click outside the pop-up (laptop) closes it; a chip's own click opens or closes as above
document.addEventListener('pointerdown', (e) => {
  if (PHONE.matches || !view.selected || e.target.closest?.('.pg-pop, .pg-chip, .pg-band')) return;
  view.selected = null;
  renderGraph();
});

// A band's header: chevron and name, then open: done/total · fails: rule; closed: the lanes it uses as mini chips,
// done/total and its state word.
function bandHeader(band, info, open, toggle, extra) {
  const total = band.steps.length;
  const head = el('button', { type: 'button', class: `pg-band${open ? ' open' : ''}`, 'aria-expanded': String(open),
    'data-band': band.key, title: `${band.name}${band.include && band.include !== band.name ? ` (${band.include})` : ''}`
      + `\n${info.done} of ${total} done · ${info.word} · fails: ${ruleText(info.rule)}`
      + `${info.lanes.length ? `\nuses ${info.lanes.join(', ')}` : ''}` },
  el('span', { class: 'pg-chev', 'aria-hidden': 'true' }), el('span', { class: 'pg-band-name', text: band.name }));
  if (!open && info.lanes.length) {
    head.append(el('span', { class: 'pg-minis', 'aria-label': `uses ${info.lanes.join(', ')}` },
      ...info.lanes.map((lane) => el('i', { text: lane[0] }))));
  }
  head.append(el('span', { class: 'pg-band-meta', text: [extra, `${info.done}/${total}`,
    open ? `fails: ${ruleText(info.rule)}` : ''].filter(Boolean).join(' · ') }));
  if (!open) head.append(el('span', { class: `pg-band-word ${info.word}`, text: info.word }));
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

function legend(plan, events, anyReplan, anyPair) {
  const labels = { navigate: moveLabel('navigate'), straight: moveLabel('straight'), hand: moveLabel('hand'),
    carry: 'carrying', posture: moveLabel('posture'), joints: moveLabel('joints'), 'grip-close': 'grip close',
    'grip-open': 'grip open', wait: 'wait', other: 'other' };
  const icons = [...new Set(plan.steps.map((step) => iconOf(step, events[step.id])))];
  return div('legend',
    ...icons.map((id) => el('span', { class: `legend-item kind-${ICON_COLOURS[id] || 'other'}` }, iconElement(id),
      labels[id] || id)),
    anyPair ? el('span', { class: 'legend-item' }, el('i', { class: 'swatch-pair' }), 'parallel candidates') : '',
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

// the rail (phone): per band a header and a wrapping grid of 44 px chips, sub-bands indented; no lines
function rail(bands, ctx, end) {
  const section = (band, depth, extra) => {
    const open = ctx.isOpen(band);
    const node = el('section', { class: `pg-rband${depth ? ' sub' : ''}` },
      bandHeader(band, ctx.info.get(band), open, ctx.toggle(band), extra));
    let grid = null;
    if (open) band.items.forEach((it) => {
      if (it.sub) { grid = null; node.append(section(it.sub, depth + 1, '')); return; }
      if (!grid) { grid = div('pg-rchips'); node.append(grid); }
      const chip = chipButton(ctx.plan.steps[it.step], it.step, ctx);
      chip.append(el('span', { class: 'pg-rnum', text: String(it.step + 1) }));
      grid.append(chip);
    });
    return node;
  };
  // the sheet for a step the operator picked; the running step is on the status board above (app.js renderBoard)
  const sheet = view.selected ? ctx.focus : -1;
  return div(`pg-rail${sheet >= 0 ? ' with-sheet' : ''}`,
    ...bands.map((band, b) => section(band, 0, b === 0 ? ctx.cycleNote : '')),
    end ? el('div', { class: 'pg-end', text: end }) : '',
    ...(sheet >= 0 ? planCard(sheet, ctx, true) : []));
}

// the lanes: core's resource names in their order, then any other resource a step holds
const lanesOf = (plan) => [...new Set([...LANES, ...plan.steps.flatMap((s) => s.resources || [])])];

// the laptop graph: rows in lanes, chips and band headers positioned in a box; lane dividers, the bars of steps on
// several lanes and the pair boxes in an SVG underneath, the replan lines in one on top
function lanesGraph(bands, ctx, end) {
  const { plan, states, r } = ctx;
  const lanes = lanesOf(plan);
  const laneIndex = (name) => lanes.indexOf(name);
  const replans = plan.steps.map((_, i) => (jumps(r[i], i) ? i : -1)).filter((i) => i >= 0);
  const x0 = replans.length ? G.replanArea + G.replanGap * (replans.length - 1) : 0;
  // the lanes share the plan box's width (at least G.lane each): the graph fills its card
  const lane = Math.max(G.lane, Math.floor(((ctx.width || 0) - x0 - G.gutter) / lanes.length));
  const laneX = (l) => x0 + G.gutter + l * lane;
  const width = laneX(lanes.length);
  const chipLane = [];
  let prev = Math.max(0, laneIndex(FIRST_LANE));
  plan.steps.forEach((step, i) => {  // the leftmost lane it holds, else the previous step's
    const held = (step.resources || []).map(laneIndex).filter((l) => l >= 0);
    chipLane[i] = prev = held.length ? Math.min(...held) : prev;
  });
  // Figma B: a step is one bar across the lanes it holds, from its first to its last (a step on one lane: one lane
  // wide), inset in its lanes; the icon at its left end
  const lastLane = plan.steps.map((step, i) => Math.max(chipLane[i], ...(step.resources || []).map(laneIndex)));
  const chipX = (i) => laneX(chipLane[i]) + G.inset;
  const chipW = (i) => laneX(lastLane[i] + 1) - laneX(chipLane[i]) - 2 * G.inset;
  const items = [];  // {band, y, depth, extra, open} and {step, y, owner}
  const rowY = [];   // per step: its chip's centre (a step of a closed band: the band header's)
  const shown = new Set();
  let y = G.laneHead;
  const place = (band, depth, extra) => {
    const open = ctx.isOpen(band);
    items.push({ band, y, depth, extra, open });
    if (!open) { band.steps.forEach((i) => { rowY[i] ??= y + G.head / 2; }); y += G.head; return; }
    y += G.head + G.headGap;
    band.items.forEach((it) => {
      if (it.sub) { place(it.sub, depth + 1, ''); y += G.headGap; return; }
      items.push({ step: it.step, y, owner: band });
      rowY[it.step] = y + G.row / 2;
      shown.add(it.step);
      y += G.row;
    });
  };
  bands.forEach((band, b) => { if (b) y += G.bandGap; place(band, 0, b === 0 ? ctx.cycleNote : ''); });
  const height = y + (end ? G.endGap + G.endH : 0);
  const under = svg('svg', { class: 'pg-lines', width, height, 'aria-hidden': 'true' });
  for (let l = 0; l <= lanes.length; l += 1) {
    under.append(svg('line', { x1: laneX(l), y1: 0, x2: laneX(l), y2: height, class: 'pg-divider' }));
  }
  const graph = el('div', { class: 'pg-graph', style: `width: ${width}px; height: ${height}px`, role: 'group',
    'aria-label': `Plan ${plan.plan_id}: ${plan.steps.length} steps` }, under,
  ...lanes.map((name, l) => el('span', { class: 'pg-lane', text: name, style: `left: ${laneX(l)}px; width: ${lane}px` })));
  const rows = [];
  items.forEach((it) => {
    if (it.band) {
      const indent = it.depth * G.subIndent;
      const head = bandHeader(it.band, ctx.info.get(it.band), it.open, ctx.toggle(it.band), it.extra);
      head.style.cssText = `left: ${x0 + indent}px; top: ${it.y}px; width: ${width - x0 - indent}px`;
      graph.append(head);
      return;
    }
    const i = it.step;
    const step = plan.steps[i];
    const top = it.y + (G.row - G.chip) / 2;
    rows.push({ i, top, owner: it.owner });
    graph.append(el('span', { class: `pg-num${isLive(states[i]) ? ' now' : ''}`, text: String(i + 1),
      style: `left: ${x0}px; top: ${it.y}px; width: ${G.gutter}px; height: ${G.row}px` }));
    const chip = chipButton(step, i, ctx);
    chip.style.left = `${chipX(i)}px`;
    chip.style.top = `${top}px`;
    chip.style.width = `${chipW(i)}px`;
    graph.append(chip);
  });
  // parallel candidates: neighbouring shown steps of one band on disjoint resources, paired left to right
  let pairs = 0;
  for (let k = 0; k + 1 < rows.length; k += 1) {
    const [a, b] = [rows[k], rows[k + 1]];
    const ra = plan.steps[a.i].resources || [];
    const rb = plan.steps[b.i].resources || [];
    if (a.owner !== b.owner || b.i !== a.i + 1 || !ra.length || !rb.length || ra.some((x) => rb.includes(x))) continue;
    const left = Math.min(chipX(a.i), chipX(b.i)) - G.pairOut;
    const right = Math.max(chipX(a.i) + chipW(a.i), chipX(b.i) + chipW(b.i)) + G.pairOut;
    under.append(svg('rect', { x: left, y: a.top - G.pairOut, width: right - left, height: b.top - a.top + G.chip + 2 * G.pairOut, rx: 7,
      class: 'pg-pair' }));
    pairs += 1;
    k += 1;
  }
  // replans: from the step's left edge into the left area, up or down, into the target from the left
  const over = svg('svg', { class: 'pg-lines over', width, height, 'aria-hidden': 'true' });
  const marker = svg('marker', { id: 'pg-arrow', viewBox: '0 0 10 10', refX: 9, refY: 5, markerWidth: 6, markerHeight: 6,
    orient: 'auto-start-reverse' });
  marker.append(svg('path', { d: 'M0,0 L10,5 L0,10 z' }));
  const defs = svg('defs');
  defs.append(marker);
  over.append(defs);
  replans.forEach((i, k) => {
    const t = r[i].replanTo;
    if (!shown.has(i) && !shown.has(t)) return;  // both ends in closed bands
    const lx = G.replanArea - 6 + G.replanGap * (replans.length - 1 - k);
    const from = rowY[i] + (shown.has(i) ? 4 : 0);
    const fromX = shown.has(i) ? chipX(i) : x0;
    const toX = shown.has(t) ? chipX(t) - 2 : x0 - 2;
    over.append(svg('path', { d: `M${fromX},${from} L${lx},${from} L${lx},${rowY[t]} L${toX},${rowY[t]}`,
      class: 'pg-replan', 'marker-end': 'url(#pg-arrow)' }));
    if (!over.querySelector('text')) {  // one label: the lines run side by side
      const mid = (from + rowY[t]) / 2;
      over.append(svg('text', { x: lx - 5, y: mid, class: 'pg-replan-label', 'text-anchor': 'middle',
        transform: `rotate(-90 ${lx - 5} ${mid})` }, 'replan'));
    }
  });
  graph.append(over);
  if (end) graph.append(el('div', { class: 'pg-end', text: end, style: `left: ${x0}px; top: ${y + G.endGap}px` }));
  // the picked step's details: a pop-up beside its bar (right of it if it fits, else left), its top at the bar's row
  // and kept inside the graph's height; with room on neither side (a bar to the last lane, a narrow card) under the
  // bar, inside the graph's width (renderGraph makes the graph tall enough for it)
  const picked = view.selected ? plan.steps.findIndex((s) => s.id === view.selected) : -1;
  if (picked >= 0) {
    const pop = planCard(picked, ctx, false);
    const x = shown.has(picked) ? chipX(picked) : x0 + G.gutter;
    const right = shown.has(picked) ? x + chipW(picked) : x;
    const top = rowY[picked] - G.row / 2;
    if (width - (right + POP_GAP) >= POP_W) pop.style.left = `${right + POP_GAP}px`;
    else if (x >= POP_W + POP_GAP) pop.style.left = `${x - POP_W - POP_GAP}px`;
    else {
      pop.style.left = `${Math.max(0, Math.min(x, width - POP_W))}px`;
      pop.style.top = `${top + G.chip + POP_GAP}px`;
    }
    pop.style.top ||= `${Math.max(0, Math.min(top, height - POP_MIN_H))}px`;
    graph.append(pop);
  }
  return { node: div('pg', graph), pairs, replans: replans.length };
}

function renderGraph() {
  const box = $('plan-graph');
  const snap = state.snapshot;
  if (!box || box.hidden || !snap) return;
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
    width: box.clientWidth,
    pickable: snap.status.state !== 'running' && snap.status.state !== 'paused', focus: focusIndex(plan, states),
    toggle: (band) => () => { storeBand(missionName, band.key, !isOpen(band)); renderGraph(); },
    cycleNote: end && snap.status.plan_id === plan.plan_id && snap.status.cycle ? `cycle ${snap.status.cycle}` : '' };
  const sheetScroll = box.querySelector('.pg-sheet')?.scrollTop ?? 0;  // the sheet is rebuilt every second: keep its place
  if (ctx.phone) {
    box.replaceChildren(rail(bands, ctx, end), legend(plan, ctx.events, false, false));
    const sheet = box.querySelector('.pg-sheet');
    if (sheet) sheet.scrollTop = sheetScroll;
  } else {
    const drawn = lanesGraph(bands, ctx, end);
    box.replaceChildren(drawn.node, legend(plan, ctx.events, drawn.replans > 0, drawn.pairs > 0));
    // the pop-up is placed over the graph: the graph grows to hold it (one under a low bar, a tall one)
    const pop = box.querySelector('.pg-pop');
    const graph = box.querySelector('.pg-graph');
    if (pop && graph) graph.style.height = `${Math.max(graph.offsetHeight, pop.offsetTop + pop.offsetHeight)}px`;
  }
  if (refocus) (box.querySelector(refocus) || box.querySelector('[data-step]'))?.focus({ preventScroll: true });
}
PHONE.addEventListener('change', () => renderGraph());
// the lanes follow the box's width: drawn again when it changes (a window resize, the Run grid's breakpoint)
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
