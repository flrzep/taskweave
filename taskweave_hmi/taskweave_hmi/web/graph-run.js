'use strict';
// taskweave HMI, Run tab: the running plan as a compact mission graph on graph-core.js. Steps are icon nodes with their
// number and target under them, joined by wires in run order; frames hold groups (an include of the top mission, a
// nested include, and with schema v2 groups, repeats and parallel lanes), each collapsible to one stack node, the open
// ones remembered per mission; the frame with the running step is always open. Only exceptions are marked: the frame
// head names the usual failure rule, a step whose rule differs gets a badge (↻2, skip), a replan is a dashed red arrow
// to the step it replans from. A step opens its details in a pop-up beside it (a bottom sheet at 600 px and below).
// Nodes are tinted by what moves (--kind-*: drive, arm, grip, wait, other) and show the step's icon (icons.js). Uses $,
// el, api, state, show, missionOf, shownPlan, pickStart, moveLabel, moveParts, carriesByArgs from app.js and
// skillIcon, iconElement, moveKindOfArgs from icons.js.

// what moves: a move step's kind, else the skill (cell schema 3 names too)
const KIND_COLOURS = { navigate: 'drive', straight: 'drive', hand: 'arm', posture: 'arm', joints: 'arm' };
const SKILL_COLOURS = { navigate_to: 'drive', drive_straight: 'drive', move_joints: 'arm', move_gripper: 'arm',
  move_object: 'arm', set_posture: 'arm', grip: 'grip', wait: 'wait' };
const ICON_COLOURS = { navigate: 'drive', straight: 'drive', hand: 'arm', carry: 'arm', posture: 'arm', joints: 'arm',
  'grip-close': 'grip', 'grip-open': 'grip', wait: 'wait', other: 'other' };
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

// The phases of a plan for the run board's floor (app.js renderPhases): the top-level frames of the mission tree and
// each run of the mission's own steps between them. A band: {key, name, steps: [index]}.
function bandsOf(plan, mission) {
  const root = treeFromPlan(plan, mission);
  const bands = [];
  root.children.forEach((node) => {
    if (node.kind === 'step') {
      const last = bands[bands.length - 1];
      if (last?.own) { last.steps.push(node.index); return; }
      bands.push({ key: `own${node.index}`, name: missionOf(plan), steps: [node.index], own: true });
      return;
    }
    bands.push({ key: node.key, name: node.name, steps: [...node.leaves], include: node.include || '' });
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

// The badge of a step whose rule differs from its frame's (retries ↻2, or ↻0 for fewer than the frame's none; skip,
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
  if (PHONE.matches || !view.selected || e.target.closest?.('.pg-pop, .pg-chip, .pg-band, .pg-stack')) return;
  view.selected = null;
  renderGraph();
});

const frameTitle = (frame, info) => `${frame.name}${frame.include && frame.include !== frame.name ? ` (${frame.include})` : ''}`
  + `\n${info.done} of ${frame.leaves.length} done · ${info.word} · fails: ${ruleText(info.rule)}`
  + `${info.used.length ? `\nuses ${info.used.join(', ')}` : ''}`;
// a frame's words beside its count: a repeat's rule and pass, a parallel's lanes and join
function frameMeta(frame, info, snap) {
  if (frame.kind === 'repeat') return repeatText(frame, snap);
  if (frame.kind === 'parallel') return `${frame.branches.length} lanes · join ${frame.block?.join || 'all'}`;
  if (frame.kind === 'branch') return '';
  return `fails: ${ruleText(info.rule)}`;
}
function repeatText(frame, snap) {
  const r = frame.block?.repeat || {};
  const rule = r.times ? `${r.times}×` : r.while ? `while ${r.while}` : r.until ? `until ${r.until}` : 'repeat';
  const pass = snap?.status?.passes?.[frame.id];
  return [`↻ ${rule}`, pass ? `pass ${pass}` : '', frame.block?.max_cycles ? `max ${frame.block.max_cycles}` : ''].filter(Boolean).join(' · ');
}
// An open frame's head on its outline: chevron and name, done/total and its words; a click closes the frame. A
// branch (a parallel lane) has only its name and count, and no chevron: it closes with its block.
function frameHead(frame, info, open, toggle, snap) {
  const lane = frame.kind === 'branch';
  const head = el('button', { type: 'button', class: `pg-band ${frame.kind}${open ? ' open' : ''}`, 'data-band': frame.key,
    'aria-expanded': String(open), title: frameTitle(frame, info) },
  lane ? '' : el('span', { class: 'pg-chev', 'aria-hidden': 'true' }),
  frame.kind === 'include' ? el('span', { class: 'pg-link', 'aria-label': 'linked: shared with other missions', title: 'shared' }, '⛓') : '',
  el('span', { class: 'pg-band-name', text: frame.name }),
  el('span', { class: 'pg-band-meta', text: [`${info.done}/${frame.leaves.length}`, frameMeta(frame, info, snap)].filter(Boolean).join(' · ') }));
  if (!lane) head.addEventListener('click', toggle);
  return head;
}
// a closed frame: one node, a stack of cards filled to its share done, its name under it; a click opens it
function stackButton(frame, info, toggle) {
  const total = frame.leaves.length;
  const node = el('button', { type: 'button', class: `pg-stack ${frame.kind} ${info.word}`, 'data-band': frame.key,
    'aria-expanded': 'false', title: frameTitle(frame, info),
    'aria-label': `${frame.name}: ${info.done} of ${total} done, ${info.word}`,
    style: `--done-share: ${Math.round((total ? info.done / total : 0) * 100)}%` },
  el('span', { class: 'pg-stack-count', text: `${info.done}/${total}` }));
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
    // the bottom sheet (a dialog, not modal: the strip and the board's Pause and Abort stay in reach above the scrim):
    // the same rows as the side card, the notes as amber boxes, Close; a tap on the scrim, Escape or the chip close it
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

// the words along the mission's loop wire: "↻ until no_frames · cycle 1 · max 10"
function loopText(plan, mission, snap) {
  const loop = plan.loop || mission?.loop || 'once';
  if (loop === 'once') return '';
  const until = mission?.stop_when ? `until ${mission.stop_when}` : (loop === 'forever' ? 'forever' : loop);
  const cycle = snap.status.plan_id === plan.plan_id && snap.status.cycle ? `cycle ${snap.status.cycle}` : '';
  return [`↻ ${until}`, cycle, mission?.max_cycles ? `max ${mission.max_cycles}` : ''].filter(Boolean).join(' · ');
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
  // a step's usual rule: its innermost frame's (the root's own steps: the whole plan's)
  const usual = [];
  const all = frameInfo({ leaves: plan.steps.map((_, i) => i) }, states, r, plan.steps).usual;
  plan.steps.forEach((_, i) => { usual[i] = all; });
  frames.forEach((f) => f.leaves.forEach((i) => { usual[i] = info.get(f).usual; }));
  // open: the frame with the running step always (the operator's choice does not hide it in a later cycle); else the
  // operator's choice, else the frame with the failed step (else the start step's, the first frame). A lane opens with
  // its block.
  const anchor = [states.findIndex(isLive), states.findIndex(isFailed),
    plan.steps.findIndex((s) => s.id === state.startStep), 0].find((i) => i >= 0);
  const stored = storedBands(missionName);
  const isOpen = (frame) => frame.kind === 'branch' || frame.leaves.some((i) => isLive(states[i]))
    || (frame.key in stored ? stored[frame.key] : frame.leaves.includes(anchor));
  const events = live ? snap.outcomes : {};
  const pickable = snap.status.state !== 'running' && snap.status.state !== 'paused';
  const ctx = { plan, states, r, events, snap, usual, pickable, metrics: PHONE.matches ? GRAPH_METRICS.phone : GRAPH_METRICS.compact,
    width: box.clientWidth || 300, isOpen, keyOf: (i) => plan.steps[i].id, loop: loopText(plan, mission, snap),
    label: `Plan ${plan.plan_id}: ${plan.steps.length} steps`,
    start: state.startStep && plan.steps.some((s) => s.id === state.startStep) ? state.startStep : plan.steps[0].id,
    replans: plan.steps.map((s, i) => (jumps(r[i], i) ? [s.id, plan.steps[r[i].replanTo].id] : null)).filter(Boolean),
    stateOf: (it) => (it.node.kind === 'step' ? states[it.node.index] : info.get(it.node)?.word === 'done' ? 'succeeded'
      : ['running', 'paused', 'failed'].includes(info.get(it.node)?.word) ? info.get(it.node).word : 'pending'),
    frameWord: (frame) => info.get(frame).word,
    renderItem: (it) => (it.kind === 'stack' ? stackButton(it.node, info.get(it.node), toggle(it.node)) : chipButton(it.node.step, it.node.index, ctx)),
    renderHead: (frame) => frameHead(frame, info.get(frame), true, toggle(frame), snap),
    labelOf: (it) => (it.kind === 'stack' ? it.node.name
      : [el('b', { text: String(it.node.index + 1) }), ` ${mainArg(it.node.step) || stepTitle(it.node.step)}`]),
    repeatText: (frame) => repeatText(frame, snap) };
  const toggle = (frame) => () => { storeBand(missionName, frame.key, !isOpen(frame)); renderGraph(); };
  const sheetScroll = box.querySelector('.pg-sheet')?.scrollTop ?? 0;  // the sheet is rebuilt every second: keep its place
  const layout = layoutGraph(tree, ctx);
  const drawn = drawGraph(layout, ctx);
  const wrap = div('pg', drawn.node);
  // the picked step's details: on a phone the bottom sheet; on a laptop a pop-up beside its node (right if it fits,
  // else left, else under it; the graph grows to hold it)
  const picked = view.selected ? plan.steps.findIndex((s) => s.id === view.selected) : -1;
  if (picked < 0) view.selected = null;
  if (picked >= 0 && PHONE.matches) {
    wrap.classList.add('with-sheet');
    wrap.append(...planCard(picked, ctx, true));
  } else if (picked >= 0) {
    const n = layout.items.get(layout.owner.get(plan.steps[picked].id));
    const pop = planCard(picked, ctx, false);
    const right = n.cx + n.nw / 2 + POP_GAP;
    const left = n.cx - n.nw / 2 - POP_GAP - POP_W;
    if (right + POP_W <= layout.width) pop.style.left = `${right}px`;
    else if (left >= 0) pop.style.left = `${left}px`;
    else {
      pop.style.left = `${Math.max(0, Math.min(n.cx - POP_W / 2, layout.width - POP_W))}px`;
      pop.style.top = `${n.cy + n.nh / 2 + 22 + POP_GAP}px`;
    }
    pop.style.top ||= `${Math.max(0, Math.min(n.cy - n.nh / 2 - 8, layout.height - POP_MIN_H))}px`;
    drawn.node.append(pop);
  }
  box.replaceChildren(wrap, legend(plan, events, drawn.replans > 0));
  const sheet = box.querySelector('.pg-sheet');
  if (sheet) sheet.scrollTop = sheetScroll;
  const pop = box.querySelector('.pg-pop');
  if (pop) drawn.node.style.height = `${Math.max(drawn.node.offsetHeight, pop.offsetTop + pop.offsetHeight)}px`;
  if (refocus) (box.querySelector(refocus) || box.querySelector('[data-step]'))?.focus({ preventScroll: true });
}
PHONE.addEventListener('change', () => renderGraph());
// the layout follows the box's width: drawn again when it changes (a window resize, the Run grid's breakpoint)
let graphWidth = 0;
new ResizeObserver(([entry]) => {
  const width = Math.round(entry.contentRect.width);
  if (width && width !== graphWidth) { graphWidth = width; renderGraph(); }
}).observe($('plan-graph'));

function setPlanView(choice) {
  try { localStorage.setItem('tw-plan-view', choice); } catch (_) { /* private mode */ }
  document.querySelectorAll('#plan-view button').forEach((b) => b.setAttribute('aria-pressed', String(b.dataset.view === choice)));
  show($('plan'), choice === 'list');
  show($('plan-graph'), choice === 'graph');
  renderGraph();
}
document.querySelectorAll('#plan-view button').forEach((b) => b.addEventListener('click', () => setPlanView(b.dataset.view)));
let initialView = 'graph';
try { initialView = localStorage.getItem('tw-plan-view') || 'graph'; } catch (_) { /* private mode */ }
setPlanView(initialView);
