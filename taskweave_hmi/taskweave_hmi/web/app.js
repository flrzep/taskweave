'use strict';
// taskweave HMI: live status over server-sent events, mission control, mission editor (between runs).

const $ = (id) => document.getElementById(id);
const ON_FAILURE = ['', 'retry', 'replan', 'skip', 'abort'];
const POINT_KINDS = { base_target: ['locations', 'stances'], joint_pose: ['joint poses'], object_point: ['object points'],
  instance_point: ['placed object points'], instance_contact: ['placed object contacts'], instance: ['placed objects'],
  tool_pose: ['tool poses'], joint_pose_only: ['joint-only poses'], joint_group: ['joint groups'],
  held_pose: ['held poses'], held_object: ['placed objects'], via_target: ['via targets'],
  target: ['base targets', 'hand targets', 'object targets', 'joint targets'] };
// cell schema 4: a target field lists the kinds it takes (x-taskweave-target-kinds), each a group of /api/points
const TARGET_GROUPS = { base: 'base targets', hand: 'hand targets', object: 'object targets', joints: 'joint targets' };
// The /api/points lists a point field picks from (none: not a point field)
function pointLists(spec) {
  const kinds = spec['x-taskweave-target-kinds'];
  if (Array.isArray(kinds) && kinds.some((kind) => TARGET_GROUPS[kind])) return kinds.map((kind) => TARGET_GROUPS[kind]).filter(Boolean);
  return POINT_KINDS[spec['x-taskweave-point']];
}
const INCLUDE = 'mission:';  // skill choice that includes another mission's steps
const state = { snapshot: null, skills: [], points: {}, missions: [], editing: null, dirty: false, previews: {}, startStep: null };

// ---------- helpers ----------
async function api(method, path, body) {
  const response = await fetch(path, {
    method, headers: body ? { 'Content-Type': 'application/json' } : {}, body: body ? JSON.stringify(body) : undefined,
  });
  let data = {};
  try { data = await response.json(); } catch (_) { /* empty */ }
  return { ok: response.ok, status: response.status, data };
}
function el(tag, attrs = {}, ...children) {
  const node = document.createElement(tag);
  for (const [key, value] of Object.entries(attrs)) {
    if (key === 'class') node.className = value; else if (key === 'text') node.textContent = value; else node.setAttribute(key, value);
  }
  if (tag === 'input' && node.type === 'number') numberField(node);  // shown with a dot in every browser locale
  children.flat().forEach((child) => node.append(child));
  return node;
}

// ---------- number fields ----------
// Every number field is a text field: Chrome and Edge show <input type="number"> with the browser's decimal separator
// (a comma in a German browser), whatever the page's lang. el() turns type 'number' inputs into one, the page's own
// are turned at load (below), the step editor's in stepCard. They show a dot; a comma typed or pasted becomes a dot as
// it is typed, letters are dropped. ↑ / ↓ step the value by the step attribute (1 for 'any' or none; with Shift ten
// steps) from where it is, within min and max, and so do the arrows at the field's right end on a laptop (held down,
// they repeat; Shift-click: ten steps); 'change' follows when the key or button is let go. Readers keep Number(input.value). Not a number, or outside min / max:
// :invalid (red), never dropped. data-spin="off": no arrows (a field with its own − / + buttons).
const NUMBER_PATTERN = '[\\-+]?(\\d+\\.?\\d*|\\.\\d+)([eE][\\-+]?\\d+)?';
const SPIN_W = 24;  // px at the field's right end that are the arrows
const decimalsOf = (text) => (String(text).split('.')[1] || '').replace(/e.*$/i, '').length;
function numberField(input) {
  input.type = 'text';
  input.classList.add('num');
  input.pattern = NUMBER_PATTERN;
  if (!input.inputMode) input.inputMode = 'decimal';
  input.autocomplete = 'off';
  input.spellcheck = false;
  const tidy = (text) => text.replace(/,/g, '.').replace(/[^0-9.eE+-]/g, '');
  const checkRange = () => {
    const value = Number(input.value);
    const min = input.getAttribute('min'), max = input.getAttribute('max');
    const out = input.value.trim() !== '' && Number.isFinite(value)
      && ((min !== null && value < Number(min)) || (max !== null && value > Number(max)));
    input.setCustomValidity(out ? `${min ?? '–∞'} to ${max ?? '∞'}` : '');
  };
  input.addEventListener('input', () => {
    const clean = tidy(input.value);
    if (clean !== input.value) {
      const at = tidy(input.value.slice(0, input.selectionStart ?? input.value.length)).length;
      input.value = clean;
      input.setSelectionRange(at, at);
    }
    checkRange();
  });
  input._check = checkRange;
  let stepped = false;
  input.addEventListener('keydown', (e) => {
    if ((e.key !== 'ArrowUp' && e.key !== 'ArrowDown') || e.altKey || e.ctrlKey || e.metaKey) return;
    e.preventDefault();
    stepNumber(input, (e.key === 'ArrowUp' ? 1 : -1) * (e.shiftKey ? 10 : 1));
    stepped = true;
  });
  input.addEventListener('keyup', (e) => {
    if (stepped && (e.key === 'ArrowUp' || e.key === 'ArrowDown')) { stepped = false; input.dispatchEvent(new Event('change', { bubbles: true })); }
  });
  if (input.dataset.spin === 'off') return;
  // the arrows (a laptop's mouse): the upper or lower half of the field's right end
  const onArrows = (e) => e.pointerType === 'mouse' && e.offsetX > input.clientWidth - SPIN_W;
  let repeat = null;
  const stop = () => {
    if (!repeat) return;
    clearTimeout(repeat); clearInterval(repeat); repeat = null;
    input.dispatchEvent(new Event('change', { bubbles: true }));
  };
  input.addEventListener('pointermove', (e) => { input.style.cursor = onArrows(e) ? 'default' : ''; });
  input.addEventListener('pointerdown', (e) => {
    if (!onArrows(e) || e.button !== 0 || input.disabled || input.readOnly) return;
    e.preventDefault();
    input.focus();
    const direction = (e.offsetY < input.clientHeight / 2 ? 1 : -1) * (e.shiftKey ? 10 : 1);
    stepNumber(input, direction);
    repeat = setTimeout(() => { repeat = setInterval(() => stepNumber(input, direction), 60); }, 400);
  });
  ['pointerup', 'pointerleave', 'pointercancel', 'blur'].forEach((type) => input.addEventListener(type, stop));
}
function stepNumber(input, direction) {  // direction: steps, signed
  const step = Number(input.getAttribute('step')) || 1;  // 'any' or none: 1
  const min = input.getAttribute('min'), max = input.getAttribute('max');
  const now = Number(input.value);
  const base = input.value.trim() !== '' && Number.isFinite(now) ? now : 0;
  let next = +(base + direction * step).toFixed(Math.min(10, Math.max(decimalsOf(step), decimalsOf(input.value))));
  if (min !== null) next = Math.max(next, Number(min));
  if (max !== null) next = Math.min(next, Number(max));
  input.value = String(next);
  input.dispatchEvent(new Event('input', { bubbles: true }));
}
document.querySelectorAll('input[type="number"]').forEach(numberField);  // the page's own (the step editor's: stepCard)
function show(node, visible) { node.hidden = !visible; }
function message(node, text, bad = false) {  // bad: true (red), 'warn' (amber) or false
  const warn = bad === 'warn';
  node.textContent = text; node.classList.toggle('bad', !!bad && !warn); node.classList.toggle('warn', warn); show(node, !!text);
}
// a manual skill's answer (drive, pose and point tests, carry poses) under the button that started it; aborted from the
// top bar is the operator's doing, not a failure
function skillResult(node, result) {
  if (result.aborted && result.outcome === 'canceled') { message(node, 'aborted', 'warn'); return; }
  message(node, `${result.outcome}${result.message ? ': ' + result.message : ''}`, result.outcome !== 'succeeded');
}
function argsText(args) { return Object.entries(args || {}).map(([k, v]) => `${k}=${typeof v === 'object' ? JSON.stringify(v) : v}`).join(' '); }

// the sticky top's height (status bar, manual skill strip, messages) as --top-h: what sticks under it starts there
if (document.querySelector('.top')) {
  new ResizeObserver(([entry]) => document.documentElement.style.setProperty('--top-h', `${Math.ceil(entry.target.offsetHeight)}px`))
    .observe(document.querySelector('.top'));
}

// ---------- tabs ----------
document.querySelectorAll('.tabs button').forEach((button) => button.addEventListener('click', () => {
  document.querySelectorAll('.tabs button').forEach((b) => {
    if (b === button) b.setAttribute('aria-current', 'page'); else b.removeAttribute('aria-current');
  });
  document.querySelectorAll('.view').forEach((view) => show(view, view.id === `view-${button.dataset.view}`));
  document.body.classList.toggle('on-run', button.dataset.view === 'run');  // the board shows the run: the bar keeps its controls
  document.body.dataset.view = button.dataset.view;  // style.css: the wide layouts of Run and Missions
  if (button.dataset.view === 'missions') { if (state.editing) refreshCatalog(); else loadMissions(); }
  pointsVisible = button.dataset.view === 'points';
  robotVisible = button.dataset.view === 'robot';
  if (pointsVisible) { loadPoints(); loadArm(); refreshPose(); }  // teach.js: poses and points
  else if (teach.open) { teach.open = null; setTeachTarget(null); }  // the pad teaches only into an open position
  if (robotVisible) { loadRobot(); refreshPose(); }  // robot.js: live grippers and joints, settings
  if (button.dataset.view === 'record') loadRecordings();  // record.js
  if (button.dataset.view === 'run') startRunViewer();
}));

// ---------- Run tab: the robot live in its cell (viewer3d.js, cell mode), for following a task ----------
let runViewer = null;
function startRunViewer() {
  if (runViewer || !window.Viewer3D || $('view-run').hidden) return;
  runViewer = new window.Viewer3D($('viewer3d-run'));
  runViewer.load();
}
window.addEventListener('viewer3d-ready', startRunViewer);  // Run is the tab the page opens on

// ---------- live state ----------
function connect() {
  const source = new EventSource('/api/events');
  const link = (on) => { $('conn').classList.toggle('on', on); $('conn').setAttribute('aria-label', on ? 'Connected' : 'Not connected'); };
  source.onopen = () => link(true);
  source.onerror = () => { link(false); renderSpeed(); };
  source.onmessage = (event) => {
    const { type, data } = JSON.parse(event.data);
    if (type === 'state') { state.snapshot = data; renderSpeed(); renderManual(); refreshCatalog(); }  // (re)connected: may have missed points
    else if (type === 'points') { refreshCatalog(); return; }  // saved or taught anywhere (any browser, the pad)
    else if (!state.snapshot) return;
    else if (type === 'status') state.snapshot.status = data;
    else if (type === 'pad_mode') state.snapshot.pad_mode = data;
    else if (type === 'speed') { state.snapshot.speed = data; renderSpeed(); return; }
    else if (type === 'manual') { state.snapshot.manual = data; renderManual(); return; }
    else if (type === 'notice') {
      padNotice(data);
      if (data.taught) { noteSaved(`arm:${data.taught.scope}/${data.taught.name}`, data.text, data.undo, data.notes); teachTaught(); }
      return;
    }
    else if (type === 'plan') { if (state.snapshot.plan?.plan_id !== data.plan_id) state.snapshot.outcomes = {}; state.snapshot.plan = data; }
    else if (type === 'episode') {
      state.snapshot.outcomes[data.step_id] = data;
      state.snapshot.events.push(data);
      state.snapshot.events = state.snapshot.events.slice(-200);
    }
    renderRun();
  };
}

let noticeTimer = null;
function padNotice({ text, ok }) {  // what a pad press did (e.g. a taught point), shown for a while in the header
  const node = $('pad-notice');
  node.textContent = text; node.title = text;
  node.classList.toggle('bad', !ok);
  show(node, true);
  clearTimeout(noticeTimer);
  noticeTimer = setTimeout(() => show(node, false), 10000);
}

function renderRun() {
  const snap = state.snapshot;
  if (!snap) return;
  const s = snap.status;
  $('robot').textContent = `robot ${snap.robot}`;
  const pad = snap.pad_mode || 'drive';  // the PS4 pad: drives the base, or jogs one arm's TCP
  $('pad-mode').textContent = pad === 'drive' ? 'pad: drive' : `pad: ${pad} arm`;
  $('pad-mode').className = `chip pad ${pad === 'drive' ? '' : 'jog'}`;
  const active = renderMission(s);
  updatePreviews();
  const { plan, live } = shownPlan();
  const outcomes = live ? snap.outcomes : {};
  const startIndex = plan && state.startStep ? plan.steps.findIndex((step) => step.id === state.startStep) : -1;
  if (startIndex < 0) state.startStep = null;  // another mission, or the step is gone
  renderBoard(s, plan, outcomes, startIndex);
  $('btn-start').textContent = !active && startIndex >= 0 ? `Start at step ${startIndex + 1}` : 'Start';
  const pickable = !active && !!plan?.steps.length;
  show($('start-hint'), pickable);
  if (pickable) {
    $('start-hint').textContent = startIndex >= 0
      ? `Starts at step ${startIndex + 1} (${plan.steps[startIndex].instruction}): the steps before it are left out, `
        + 'the robot and the objects stay where they are now. Click it again to start from the beginning.'
      : 'Click a step to start the mission there.';
  }
  setPointsBusy(active ? `A mission is ${s.state}. Points and robot settings can be changed when it has finished or was aborted.` : '');

  const list = $('plan');
  list.replaceChildren();
  if (!plan?.steps.length) { list.append(el('li', { class: 'muted', text: plan?.message || 'No plan yet' })); }
  else plan.steps.forEach((step, index) => {
    const event = outcomes[step.id];
    let kind = 'pending', mark = String(index + 1), note = '';
    if (event?.event === 'step_started') { kind = 'running'; }
    if (event?.event === 'step_finished') { kind = event.outcome.toLowerCase(); mark = kind === 'succeeded' ? '✓' : '!'; note = event.message || ''; }
    if (s.step_index === index && active) kind = 'running';
    const item = el('li', { class: `${pickable ? 'pickable' : ''} ${index === startIndex ? 'start-here' : ''}` },
      el('span', { class: `dot ${kind}`, text: mark }),
      el('div', { class: 'step-text' },
        el('div', { text: step.instruction }),
        el('div', { class: 'muted mono', text: `${step.skill} ${argsText(step.args)}` }),
        ...(step.notes || []).map((text) => el('div', { class: 'step-note', text })),  // validation's notes
        note ? el('div', { class: 'muted', text: note }) : ''));
    if (pickable) { item.title = 'Start the mission at this step'; item.addEventListener('click', () => pickStart(step)); }
    list.append(item);
  });

  if (typeof renderGraph === 'function') renderGraph();  // graph.js

  renderRecent(snap);
  const log = $('log');
  log.replaceChildren(...snap.events.slice().reverse().map((e) => el('li', {
    text: `t=${Number(e.time).toFixed(1)}s ${e.plan_id} c${e.cycle} ${e.step_id} ${e.skill} ${e.event.replace('step_', '')}`
      + `${e.outcome ? ' ' + e.outcome : ''}${e.attempt > 1 ? ' attempt ' + e.attempt : ''}${e.message ? ' · ' + e.message : ''}`,
  })));
}

// The Run tab's recent steps: the last RECENT finished steps of the run shown, newest first: the step events of the
// last plan after the run's start (status.run_from_seq: a restart of the same mission has the same plan id). A
// neutral check for a success, ! red for a failure, amber ! for one canceled (Abort, Pause), ↻ amber for a retry (a
// step that ended without success and started again in this run), the instruction with the command and its message,
// and the time on the run's clock. Rebuilt only when its rows change.
const RECENT = 6;
let recentKey = '';
function renderRecent(snap) {
  const plan = snap.plan?.plan_id;
  const from = snap.status?.run_from_seq ?? 0;
  const run = snap.events.filter((e) => e.plan_id === plan && e.seq > from);
  const finished = run.filter((e) => e.event === 'step_finished').slice(-RECENT).reverse();
  const key = `${plan}|${from}|${finished.map((e) => e.seq).join(',')}|${run.at(-1)?.seq ?? ''}`;
  if (key === recentKey) return;
  recentKey = key;
  if (!finished.length) {
    $('recent').replaceChildren(el('li', { class: 'empty', text: plan ? 'No step has finished yet.' : 'No run yet.' }));
    return;
  }
  $('recent').replaceChildren(...finished.map((e) => {
    const ok = e.outcome === 'SUCCEEDED';
    const retried = !ok && run.some((later) => later.seq > e.seq && later.step_id === e.step_id && later.event === 'step_started');
    const canceled = e.outcome === 'CANCELED';
    const [mark, kind, said] = ok ? ['✓', '', 'done'] : retried ? ['↻', 'warn', 'retried']
      : ['!', canceled ? 'warn' : 'bad', canceled ? 'canceled' : e.outcome.toLowerCase()];
    const label = e.skill === 'move' && e.data?.kind ? moveLabel(e.data.kind) : e.skill;
    return el('li', {},
      el('span', { class: `mark ${kind}`, text: mark, 'aria-hidden': 'true' }), el('span', { class: 'sr-only', text: `${said}: ` }),
      el('span', { class: 'what' }, e.instruction || e.step_id,
        el('small', { text: [label, e.attempt > 1 ? `attempt ${e.attempt}` : '', e.message].filter(Boolean).join(' · ') })),
      el('time', { text: `${Number(e.time).toFixed(1)} s`, title: 'seconds on the run\'s clock' }));
  }));
}

// ---------- mission bar ----------
// In the sticky top bar, on every tab. While a mission is active (running, planning, paused): the state badge, the
// goal, "step n/N" with the step's instruction (one line, ellipsis), a thin progress bar of the step, Pause / Resume
// and Abort (two taps). Idle (also after the end of one): the badge, the mission picker and Start. Under the bar,
// Start's answer (while idle) and the last error: it follows the status as before (gone on success, on the next
// failure a new one), and ✕ hides that error until another comes.
const missionUi = { dismissed: null };
function renderMission(s) {
  const active = s.state === 'running' || s.state === 'paused';
  // the task handler prepares the plan: running, no step yet, step_status 'planning'
  const planning = s.state === 'running' && s.step_index === -1 && s.step_status === 'planning';
  $('state').textContent = planning ? 'planning' : s.state;
  $('state').className = `chip ${s.state}`;
  $('mission').classList.toggle('active', active);
  show($('mission-run'), active);
  show($('mission-controls'), active);
  show($('mission-start'), !active);
  $('goal').textContent = s.goal || 'No mission running';
  $('goal').title = $('goal').textContent;
  $('cycle').textContent = s.plan_id ? `${s.plan_id} · cycle ${s.cycle}` : '';
  show($('current'), active && !!s.step);
  if (planning) {
    $('step-meta').textContent = ''; $('step-detail').textContent = '';  // in the instruction's place: a phone shows it
    $('step-instruction').textContent = `planning ${s.step_count} step${s.step_count === 1 ? '' : 's'}…`;
  } else if (s.step) {
    $('step-meta').textContent = `step ${s.step_index + 1}/${s.step_count}`;
    $('step-instruction').textContent = s.step.instruction;
    $('progress-bar').style.width = `${Math.max(0, Math.min(1, s.step_progress)) * 100}%`;
    $('step-detail').textContent = `${s.step.skill} · ${s.step_status || ''} · ${s.step_elapsed_s.toFixed(1)} s`
      + `${s.attempt > 1 ? ` · attempt ${s.attempt}` : ''}`;
  }
  $('step-instruction').title = $('step-instruction').textContent;  // all of it, where the bar cuts it
  if (active) message($('start-msg'), '');  // its answer has been seen: the bar shows the run now
  const error = s.last_error && s.state !== 'succeeded' ? s.last_error : '';
  $('last-error-text').textContent = error;
  show($('last-error'), !!error && error !== missionUi.dismissed);
  $('btn-pause').disabled = !active;
  $('btn-pause').textContent = s.state === 'paused' ? 'Resume' : 'Pause';
  $('btn-abort').disabled = !active;
  $('btn-start').disabled = active;
  return active;
}
// ---------- status board (Run tab) ----------
// What the robot is doing, readable from a few metres: the state word, the mission and cycle, "16 / 26" big, the
// step's instruction and under it the command, status and time; when it stopped, the error and how to go on; a strip
// of the phases (the plan graph's bands) sized by their steps, done in green, the current one ringed.
const BOARD_WORDS = { idle: 'idle', planning: 'planning', running: 'running', paused: 'paused', succeeded: 'done',
  failed: 'failed', aborted: 'aborted' };
const board = { phases: null, error: null };  // the phases' last arguments, the error text shown
// a step that ended without success (the failed band, where a stopped run stopped)
const NOT_DONE = ['FAILED', 'REJECTED', 'CANCELED', 'TIMEOUT'];
// graph.js (bandsOf) loads after this file: draw the phases once more when every script is there
window.addEventListener('load', () => { if (board.phases) renderPhases(...board.phases); });
// plan / outcomes: what the Run tab shows (the picked mission's preview while idle); the board shows the run itself
// while one is active or has ended, the preview only before any run
function renderBoard(s, shown, shownOutcomes, startIndex) {
  const planning = s.state === 'running' && s.step_index === -1 && s.step_status === 'planning';
  const word = planning ? 'planning' : s.state;
  $('board').className = `card board state-${word}`;
  $('board-state').textContent = BOARD_WORDS[word] || word;
  const active = s.state === 'running' || s.state === 'paused';
  const ended = ['succeeded', 'failed', 'aborted'].includes(s.state) && !!s.plan_id;
  const run = active || ended;
  const plan = run ? state.snapshot.plan : shown;
  const outcomes = run ? state.snapshot.outcomes || {} : shownOutcomes;
  const picked = $('mission-select').value;
  const next = ended && plan && picked && picked !== missionOf(plan) ? ` Next: ${picked}, Start in the bar above.` : '';
  $('board-goal').textContent = run ? s.goal || '' : missionOf(plan) || '';
  $('board-cycle').textContent = s.plan_id && (active || ended) ? `cycle ${s.cycle} · ${s.plan_id}` : '';
  const total = plan?.steps.length || s.step_count || 0;
  const at = s.step_index >= 0 ? s.step_index + 1 : null;
  let count = '–', of = '', text = '', detail = '';
  if (planning) {
    of = `${s.step_count} steps`; text = 'Preparing the plan';
    detail = 'The robot starts when the plan is ready.';
  } else if (active && s.step) {
    count = at ? String(at) : '–'; of = `/ ${total}`; text = s.step.instruction;
    const label = s.step.skill === 'move' && s.step.kind ? moveLabel(s.step.kind) : s.step.skill;
    detail = [label, s.state === 'paused' ? 'paused: Resume in the bar above' : s.step_status,
      `${(s.step_elapsed_s ?? 0).toFixed(0)} s`, s.attempt > 1 ? `attempt ${s.attempt}` : ''].filter(Boolean).join(' · ');
  } else if (active) {  // between steps (or a replan): no step in the status
    count = at ? String(at) : '–'; of = total ? `/ ${total}` : '';
    text = s.state === 'paused' ? 'Paused between steps' : 'Going on to the next step';
    detail = s.state === 'paused' ? 'Resume in the bar above.' : s.step_status || '';
  } else if (s.state === 'succeeded' && s.plan_id) {
    count = String(total); of = `/ ${total}`; text = 'Finished';
    detail = `Start runs it again.${next}`;
  } else if ((s.state === 'failed' || s.state === 'aborted') && s.plan_id) {
    // where it stopped: the status's step, else the last step that finished without success
    const steps = plan?.steps || [];
    const last = [...steps.keys()].reverse().find((i) => NOT_DONE.includes(outcomes[steps[i].id]?.outcome));
    const index = at ? at - 1 : last ?? -1;
    count = index >= 0 ? String(index + 1) : '–'; of = total ? `/ ${total}` : '';
    text = (s.step || steps[index])?.instruction || (s.state === 'aborted' ? 'Aborted' : 'The mission stopped');
    detail = (s.state === 'aborted' ? 'Aborted by the operator.'
      : index >= 0 ? `Failed at step ${index + 1}.` : 'The mission stopped.') + next;
  } else if (plan?.steps.length) {
    count = String(total); of = 'steps';
    text = startIndex >= 0 ? `Ready to start at step ${startIndex + 1}` : 'Ready to start';
    detail = startIndex >= 0 ? plan.steps[startIndex].instruction : 'Press Start in the bar above, or click a step in the plan to start there.';
  } else {
    text = 'No mission running'; detail = 'Pick a mission in the bar above and press Start.';
  }
  $('board-step').textContent = count;
  $('board-total').textContent = of;
  $('board-instruction').textContent = text;
  $('board-instruction').title = text;
  $('board-detail').textContent = detail;
  // the last error, as the bar shows it off the Run tab (not after a success); how to go on once it stopped. Rewritten
  // only when it changed, so a screen reader announces it once. No dismiss here: the Run tab is where the error stays
  // until the next start (the bar's close button hides only the bar's copy)
  const stopped = s.state === 'failed' || s.state === 'aborted';
  const error = s.last_error && s.state !== 'succeeded' ? s.last_error : '';
  const key = error ? `${stopped}|${error}` : '';
  show($('board-error'), !!error);
  if (key !== board.error) {
    board.error = key;
    $('board-error').replaceChildren(...(error ? [document.createTextNode(error), stopped ? el('small',
      { text: 'To go on: click a step in the plan and press Start; the steps before it are left out.' }) : ''] : []));
  }
  board.phases = [s, plan, outcomes, active];
  renderPhases(s, plan, outcomes, active);
}
// the phases: the plan graph's bands (graph.js bandsOf, from each step's origin); one band: none shown
function renderPhases(s, plan, outcomes, active) {
  const list = $('board-phases');
  const bands = plan?.steps.length && typeof bandsOf === 'function'
    ? bandsOf(plan, typeof missionCache === 'object' ? missionCache[missionOf(plan)] : null) : [];
  show(list, bands.length > 1);
  if (bands.length < 2) { list.replaceChildren(); return; }
  list.replaceChildren(...bands.map((band) => {
    const done = band.steps.filter((i) => outcomes[plan.steps[i].id]?.event === 'step_finished').length;
    const now = active && band.steps.includes(s.step_index);
    const failed = band.steps.some((i) => NOT_DONE.includes(outcomes[plan.steps[i].id]?.outcome));
    const item = el('li', { class: `${now ? 'now' : ''}${failed ? ' failed' : ''}`,
      'aria-label': `${band.name}: ${done} of ${band.steps.length} steps done${now ? ', running' : ''}` },
    el('div', { class: 'ph-bar' }, el('i')), el('div', { class: 'name' }, el('span', { text: band.name }),
      el('b', { text: `${done}/${band.steps.length}` })));
    item.style.setProperty('--steps', band.steps.length);
    item.style.setProperty('--done-part', `${(done / band.steps.length) * 100}%`);
    item.title = `${band.name} · ${done}/${band.steps.length}`;
    return item;
  }));
}

$('last-error-close').addEventListener('click', () => { missionUi.dismissed = $('last-error-text').textContent; show($('last-error'), false); });

// ---------- run controls ----------
$('btn-pause').addEventListener('click', async () => {
  await api('POST', state.snapshot?.status.state === 'paused' ? '/api/resume' : '/api/pause');
});
let abortArmed = null;
$('btn-abort').addEventListener('click', async () => {
  const button = $('btn-abort');
  if (!abortArmed) {  // two taps: no browser dialogs
    button.classList.add('confirm'); button.textContent = 'Tap again to abort';
    abortArmed = setTimeout(() => { button.classList.remove('confirm'); button.textContent = 'Abort'; abortArmed = null; }, 3000);
    return;
  }
  clearTimeout(abortArmed); abortArmed = null; button.classList.remove('confirm'); button.textContent = 'Abort';
  await api('POST', '/api/abort');
});
$('btn-start').addEventListener('click', async () => {
  const mission = $('mission-select').value;
  if (!mission) return;
  const startStep = state.startStep;
  const { ok, data } = await api('POST', '/api/start', startStep ? { mission, start_step: startStep } : { mission });
  if (ok) { state.startStep = null; missionUi.dismissed = null; }
  message($('start-msg'), ok ? `Started ${data.plan_id}${startStep ? ` at ${startStep}` : ''}`
    : (data.message || data.error || 'not started'), !ok);
});
$('mission-select').addEventListener('change', () => { state.startStep = null; renderRun(); });

// ---------- speed override ----------
// The task handler's global override (1-100 %), shown and set from the top bar on every tab. It is meant for a
// running mission: the skills follow a change at once. Requests go one at a time; clicks while one is on its way
// are merged into the next. While the slider is dragged, live updates do not move it.
const SPEED_STEP = 5;  // the − / + buttons, in percent (the slider sets single percents)
const speedUi = { pending: null, aim: null, busy: false, dragging: false, note: null, noteTimer: null };
function speedNow() { return state.snapshot?.speed || null; }
function speedUsable() { const s = speedNow(); return !!s?.available && s.percent !== null && $('conn').classList.contains('on'); }
function renderSpeed() {
  const s = speedNow();
  const usable = speedUsable();
  const box = $('speed');
  box.classList.toggle('off', !usable);
  box.classList.toggle('test', !!s?.test_mode);
  $('speed-value').textContent = s?.percent != null ? `${s.percent} %` : '–';
  $('speed-open').setAttribute('aria-label', `Speed ${s?.percent != null ? `${s.percent} %` : 'unknown'}${s?.test_mode ? ', test mode' : ''}`);
  if (!speedUi.dragging && s?.percent != null) $('speed-range').value = s.percent;
  if (s && !$('speed-presets').children.length) {  // made once: the presets come with the override
    $('speed-presets').append(...s.presets.map((p) => {
      const button = el('button', { text: `${p} %`, 'data-percent': String(p), 'aria-pressed': 'false' });
      button.addEventListener('click', () => setSpeed({ percent: p }));
      return button;
    }));
  }
  $('speed-presets').querySelectorAll('button').forEach((b) => {
    b.setAttribute('aria-pressed', String(Number(b.dataset.percent) === s?.percent));
    b.disabled = !usable;
  });
  ['speed-down', 'speed-up', 'speed-range', 'speed-test'].forEach((id) => { $(id).disabled = !usable; });
  if (s) {
    $('speed-down').disabled ||= s.percent <= s.min;
    $('speed-up').disabled ||= s.percent >= s.max;
  }
  const test = $('speed-test');
  test.setAttribute('aria-pressed', String(!!s?.test_mode));
  test.textContent = s?.test_mode ? `Test · max ${s.cap_percent} %` : 'Test mode';
  test.title = s ? `Test mode: a run starts at ${s.test_percent} %, the speed stays at ${s.cap_percent} % or below` : '';
  // why the control is grey, else the handler's word on a clamped or refused request (for a while)
  const note = s && !usable ? { text: 'Task handler not reachable: the speed cannot be set now', bad: false } : speedUi.note;
  message($('speed-msg'), note?.text || '', !!note?.bad);
}
function speedMessage(text, bad = false) {
  speedUi.note = { text, bad };
  clearTimeout(speedUi.noteTimer);
  speedUi.noteTimer = setTimeout(() => { speedUi.note = null; renderSpeed(); }, 8000);
  renderSpeed();
}
async function setSpeed(body) {
  speedUi.pending = { ...speedUi.pending, ...body };
  if (body.percent) speedUi.aim = body.percent;  // − / + count on from what was asked, not from the last answer
  if (speedUi.busy) return;
  speedUi.busy = true;
  while (speedUi.pending) {
    const sent = speedUi.pending;
    speedUi.pending = null;
    const { ok, data } = await api('POST', '/api/speed', sent);
    if (ok && state.snapshot?.speed) { state.snapshot.speed.percent = data.percent; renderSpeed(); }  // the event follows
    if (!ok) speedMessage(data.message || data.error || 'not set', true);
    else if (/clamp/.test(data.message)) speedMessage(data.message);
  }
  speedUi.busy = false;
  speedUi.aim = null;
}
function stepSpeed(direction) {  // to the next multiple of SPEED_STEP; the bottom step goes to the minimum
  const s = speedNow();
  if (!s) return;
  const base = speedUi.aim ?? s.percent;
  const next = direction > 0 ? Math.floor(base / SPEED_STEP) * SPEED_STEP + SPEED_STEP
    : Math.ceil(base / SPEED_STEP) * SPEED_STEP - SPEED_STEP;
  setSpeed({ percent: Math.max(s.min, Math.min(s.max, next)) });
}
$('speed-down').addEventListener('click', () => stepSpeed(-1));
$('speed-up').addEventListener('click', () => stepSpeed(1));
$('speed-range').addEventListener('input', () => { speedUi.dragging = true; $('speed-value').textContent = `${$('speed-range').value} %`; });
$('speed-range').addEventListener('change', () => { speedUi.dragging = false; setSpeed({ percent: Number($('speed-range').value) }); });
$('speed-test').addEventListener('click', () => setSpeed({ test_mode: !speedNow()?.test_mode }));
// Speed in the status bar opens the panel under it (the slider, presets, test mode); a click elsewhere or Escape closes
// it. Escape is not stopped: it still aborts a manual skill
function openSpeed(open) {
  show($('speed-panel'), open);
  $('speed-open').setAttribute('aria-expanded', String(open));
  if (open) $('speed-panel').querySelector('button:not(:disabled), input:not(:disabled)')?.focus();
}
$('speed-open').addEventListener('click', () => openSpeed($('speed-panel').hidden));
document.addEventListener('pointerdown', (e) => {
  if (!$('speed-panel').hidden && !e.target.closest('#speed-panel, #speed-open')) openSpeed(false);
});
document.addEventListener('keydown', (e) => {
  if (e.key !== 'Escape' || $('speed-panel').hidden) return;
  const inside = $('speed-panel').contains(document.activeElement);
  openSpeed(false);
  if (inside) $('speed-open').focus();
});

// ---------- manual skill: Abort ----------
// A skill started from here outside a mission (Drive here, Move here, Go here, a carry pose's Test) shows in the top
// bar while it runs, on every tab, with Abort: one press, no second tap (stopping is the safe direction), and Escape
// does the same unless a text field has the focus. The seconds count on the browser's clock from the server's
// elapsed_s, so the two clocks need not agree. Missions keep their own Abort (two taps, mission bar).
const manualUi = { since: null, start: 0, timer: null };
function manualText() {
  const m = state.snapshot?.manual;
  if (!m) return;
  const seconds = Math.floor((performance.now() - manualUi.start) / 1000);
  $('manual-text').textContent = `Running ${m.label} (${m.skill}) · ${seconds} s`;
  $('manual-text').title = $('manual-text').textContent;
}
function renderManual() {
  const m = state.snapshot?.manual || null;
  updatePreviews();
  show($('manual'), !!m);
  if (!m) { clearInterval(manualUi.timer); manualUi.timer = null; manualUi.since = null; return; }
  if (manualUi.since !== m.since) {  // a new one: count from its elapsed time, Abort pressable again
    manualUi.since = m.since;
    manualUi.start = performance.now() - m.elapsed_s * 1000;
    $('manual-abort').disabled = false; $('manual-abort').textContent = 'Abort';
  }
  if (!manualUi.timer) manualUi.timer = setInterval(manualText, 1000);
  manualText();
}
async function abortManual() {
  const button = $('manual-abort');
  if (!state.snapshot?.manual || button.disabled) return;
  button.disabled = true; button.textContent = 'Aborting…';  // the bar goes when the skill has stopped
  const { ok, data } = await api('POST', '/api/skill/abort');
  if (!ok) { button.disabled = false; button.textContent = 'Abort'; padNotice({ text: data.message || data.error || 'not aborted', ok: false }); }
}
$('manual-abort').addEventListener('click', abortManual);
document.addEventListener('keydown', (e) => {
  if (e.key !== 'Escape' || e.repeat || !state.snapshot?.manual) return;
  const typing = e.target.closest?.('textarea, select, [contenteditable], input:not([type="range"], [type="checkbox"], [type="radio"], [type="button"])');
  if (!typing) abortManual();
});

// ---------- start at a step ----------
// With no mission active, the Run tab shows the selected mission's last run (with its outcomes) or, if it has not
// run since the task handler started, the plan a start would run now. Clicking a step there makes the next start
// begin at it (the first cycle only; later cycles run all steps).
function missionOf(plan) { return plan ? plan.plan_id.replace(/-(\d+|preview)$/, '') : ''; }
function shownPlan() {
  const snap = state.snapshot;
  const s = snap.status;
  const mission = $('mission-select').value;
  if (s.state === 'running' || s.state === 'paused' || !mission || missionOf(snap.plan) === mission) {
    return { plan: snap.plan, live: true };
  }
  if (!(mission in state.previews)) loadPreview(mission);
  return { plan: state.previews[mission], live: false };
}
async function loadPreview(mission) {
  state.previews[mission] = null;  // one request per mission
  const { ok, data } = await api('GET', `/api/missions/${encodeURIComponent(mission)}/plan`);
  state.previews[mission] = ok ? data
    : { plan_id: `${mission}-preview`, steps: [], message: (data.problems || []).join('; ') || data.error || 'no plan' };
  renderRun();
}
function pickStart(step) {
  const s = state.snapshot?.status.state;
  if (s === 'running' || s === 'paused') return;
  state.startStep = state.startStep === step.id ? null : step.id;
  renderRun();
}

async function loadMissions() {
  const { ok, data } = await api('GET', '/api/missions');
  if (!ok) return;
  state.previews = {};  // missions may have been edited
  state.missions = data;
  const select = $('mission-select');
  const current = select.value;
  const block = (m) => Object.keys(m.params || {}).length > 0;  // takes parameters: only for other missions to include
  select.replaceChildren(...data.filter((m) => m.valid && !block(m)).map((m) => el('option', { value: m.name, text: `${m.name} — ${m.goal}` })));
  if (current) select.value = current;
  $('mission-list').replaceChildren(...data.map((m) => {
    const item = el('li', { tabindex: '0', 'data-name': m.name, 'data-find': `${m.name} ${m.goal || ''}`.toLowerCase() },
      el('b', { text: m.name }),
      el('span', { class: 'muted', text: m.valid ? `${m.goal} · ${m.steps} steps${block(m) ? ' · block (include it)' : ''}` : '' }),
      m.valid ? '' : el('span', { class: 'bad', text: m.error }));
    item.addEventListener('click', () => pickMission(item, m.name));
    item.addEventListener('keydown', (e) => { if (e.key === 'Enter') pickMission(item, m.name); });
    return item;
  }));
  markOpen();
  filterMissions();
}
// The open mission is marked in the list (beside the editor from 1180px). With unsaved edits open, another mission or
// New mission asks first: the first tap says so in the list's status line ("tap again to drop them", 3 s), a second tap
// on the same one goes on (dropDirty).
function markOpen() {
  $('mission-list').querySelectorAll('li').forEach((li) => {
    const open = !!state.editing && li.dataset.name === state.editing.name;
    li.classList.toggle('open', open);
    if (open) li.setAttribute('aria-current', 'true'); else li.removeAttribute('aria-current');
  });
}
const dropping = { node: null, timer: null };
function dropNote(text) { $('mission-note').textContent = text; }
function dropDirty(node, next) {
  if (state.dirty && dropping.node !== node) {
    clearTimeout(dropping.timer);
    dropping.node = node;
    dropNote(`Unsaved changes in ${state.editing.name || 'the new mission'}: tap again to drop them`);
    dropping.timer = setTimeout(() => { dropping.node = null; dropNote(''); }, 3000);
    return;
  }
  clearTimeout(dropping.timer); dropping.node = null; dropNote('');
  next();
}
function pickMission(item, name) {
  if (state.editing?.name === name) return;
  dropDirty(item, () => openEditor(name));
}
function filterMissions() {
  const words = $('mission-search').value.trim().toLowerCase().split(/\s+/).filter(Boolean);
  let shown = 0;
  $('mission-list').querySelectorAll('li').forEach((li) => { li.hidden = !words.every((w) => li.dataset.find.includes(w)); shown += !li.hidden; });
  show($('mission-none'), !shown && words.length > 0);
}
$('mission-search').addEventListener('input', filterMissions);
// the list or the editor (narrow) / both (wide): .editing on the view; nothing open: the empty side says what to do
function showEditor(open) {
  const had = $('view-missions').contains(document.activeElement);
  show($('editor'), open);
  show($('editor-empty'), !open);
  $('view-missions').classList.toggle('editing', open);
  markOpen();
  // the keyboard focus stays on the page: on the back link when the list went away (narrow), on the list's title when
  // the editor closed; not moved while the list stays (wide)
  if (!had) return;
  if (open && $('mission-list-card').offsetParent === null) $('btn-back').focus();
  else if (!open) $('missions-title').focus();
}

// ---------- mission editor ----------
async function loadCatalog() {
  const [skills, points] = await Promise.all([api('GET', '/api/skills'), api('GET', '/api/points')]);
  if (skills.ok) state.skills = skills.data;
  if (points.ok) state.points = points.data;
}
// The argument pickers' point lists again while a mission is open in the editor: when the Missions tab is shown, when
// points were saved or taught (the live event points: from any browser or the pad) and after a reconnect. The pickers
// get the new lists in place; the choices and unsaved edits stay.
async function refreshCatalog() {
  if (!state.editing) return;
  const { ok, data } = await api('GET', '/api/points');
  if (!ok || !state.editing) return;
  state.points = data;
  $('step-editor').querySelectorAll('select[data-points]').forEach((select) => select._fill?.());
}
function fillSelect(select, values, selected) {
  select.replaceChildren(...values.map((v) => el('option', { value: v, text: v === '' ? 'default' : v })));
  select.value = selected ?? '';
}
function markDirty() { state.dirty = true; show($('editor-dirty'), true); }
// Leaving or reloading the page with unsaved mission edits asks first (the browser's own question). Switching tabs keeps
// them: the editor stays open.
window.addEventListener('beforeunload', (e) => { if (state.editing && state.dirty) { e.preventDefault(); e.returnValue = ''; } });

async function openEditor(name) {
  await loadCatalog();
  const isNew = name === null;
  const { ok, data } = await api('GET', isNew ? '/api/template' : `/api/missions/${encodeURIComponent(name)}`);
  if (!ok) { alertInline(data); return; }
  state.editing = { name, isNew, mission: data };
  state.dirty = false;
  show($('editor-dirty'), false);
  showEditor(true);
  message($('save-msg'), ''); show($('problems'), false);
  $('m-name').value = name || '';
  disarmDelete();
  show($('btn-delete'), !isNew);
  $('m-goal').value = data.goal || '';
  $('m-loop').value = data.loop || 'once';
  $('m-max').value = data.max_cycles ?? '';
  $('m-stop').value = data.stop_when ?? '';
  const d = data.defaults || {};
  $('d-timeout').value = d.timeout_s ?? '';
  $('d-retries').value = d.retries ?? '';
  fillSelect($('d-fail'), ON_FAILURE, d.on_failure ?? '');
  $('d-replans').value = d.max_replans ?? '';
  const steps = data.steps || [];
  renderSteps(steps, steps.map(() => steps.length <= FOLD_ABOVE));  // a long mission opens folded, to reorder it
}
function alertInline(data) { message($('save-msg'), data.error || (data.problems || []).join('\n'), true); }

// expanded: per step, open (true) or folded to its summary line (false)
function renderSteps(steps, expanded) {
  const list = $('step-editor');
  list.replaceChildren();
  steps.forEach((step, index) => list.append(stepCard(step, index, steps, expanded[index])));
  foldButton();
}
function readSteps() { return [...$('step-editor').children].map((card) => card._read()); }

function stepCard(step, index, steps, expanded) {
  const card = $('step-template').content.firstElementChild.cloneNode(true);
  card.querySelectorAll('input[type="number"]').forEach(numberField);
  const q = (key) => card.querySelector(`[data-k="${key}"]`);
  card.querySelector('.step-no').replaceChildren(el('span', { class: 'step-word', text: 'Step ' }), String(index + 1));
  card.querySelector('[data-act="fold"]').addEventListener('click', () => { setExpanded(card, !isExpanded(card)); foldButton(); });
  q('instruction').value = step.instruction || '';
  const missions = state.missions.filter((m) => m.valid && m.name !== state.editing?.name).map((m) => INCLUDE + m.name);
  const chosen = step.include ? INCLUDE + step.include : step.skill;
  const names = state.skills.map((s) => s.name).concat(missions);
  fillSelect(q('skill'), names.concat(names.includes(chosen) || !chosen ? [] : [chosen]), chosen);
  q('id').value = step.id || '';
  q('timeout_s').value = step.timeout_s ?? '';
  q('retries').value = step.retries ?? '';
  fillSelect(q('on_failure'), ON_FAILURE, step.on_failure ?? '');
  const ids = steps.map((s, i) => s.id || `s${i + 1}`).filter((_, i) => i !== index);
  fillSelect(q('replan_from'), [''].concat(ids), step.replan_from ?? '');
  q('replan_from').options[0].textContent = 'this step';
  const argsBox = card.querySelector('.args');
  let order = [];  // the arguments' keys as the step had them: read back in that order
  const renderArgs = (args) => {
    order = Object.keys(args);
    // an argument the skill (or the included mission) does not have stays, as text, so Check can name it
    const strays = (known) => order.filter((key) => !(key in known)).map((key) => argField(key,
      { description: 'not an argument of this skill: remove the text to drop it' }, false, args[key]));
    if (q('skill').value.startsWith(INCLUDE)) {  // another mission's steps: its params
      const params = state.missions.find((m) => INCLUDE + m.name === q('skill').value)?.params || {};
      argsBox.replaceChildren(...Object.entries(params).map(([key, p]) => argField(key,
        { type: 'string', 'x-taskweave-point': p.point, description: p.description }, true, args[key])), ...strays(params));
      return;
    }
    const skill = state.skills.find((s) => s.name === q('skill').value);
    if (formsOf(skill?.args_schema)) { formArgs(argsBox, skill.args_schema, args, strays); return; }
    const props = skill?.args_schema?.properties || {};
    const required = skill?.args_schema?.required || [];
    argsBox.replaceChildren(...Object.entries(props).map(([key, spec]) => argField(key, spec, required.includes(key), args[key])),
      ...strays(props));
    showIf(argsBox, [...argsBox.children].map((node) => ({ key: node.dataset.arg, spec: props[node.dataset.arg], node, read: node._value })));
  };
  renderArgs(step.include ? step.with || {} : step.args || {});
  const preview = movePreview(() => card._read());
  argsBox.after(preview);
  q('skill').addEventListener('change', () => { renderArgs({}); markDirty(); });
  ['input', 'change'].forEach((type) => card.addEventListener(type, () => { preview._clear(); preview._refresh(); }));
  card.addEventListener('input', markDirty);
  card.addEventListener('change', markDirty);
  ['input', 'change'].forEach((type) => card.addEventListener(type, () => stepSummary(card)));
  ['input', 'change'].forEach((type) => card.addEventListener(type, () => card.classList.remove('has-problem')));
  card.querySelector('[data-act="up"]').disabled = index === 0;
  card.querySelector('[data-act="down"]').disabled = index === steps.length - 1;
  card.querySelector('[data-act="up"]').addEventListener('click', () => move(index, -1, 'up'));
  card.querySelector('[data-act="down"]').addEventListener('click', () => move(index, 1, 'down'));
  const remove = card.querySelector('[data-act="remove"]');
  let removeArmed = null;
  remove.addEventListener('click', () => {
    if (!removeArmed) {  // two taps, as Delete elsewhere: there is no undo
      remove.classList.add('confirm'); remove.textContent = 'Remove?'; remove.title = 'Tap again to remove this step';
      removeArmed = setTimeout(() => { remove.classList.remove('confirm'); remove.textContent = '✕'; remove.title = ''; removeArmed = null; }, 3000);
      return;
    }
    clearTimeout(removeArmed);
    editSteps((items) => items.splice(index, 1));
  });
  card.querySelector('[data-act="copy"]').addEventListener('click', () => {  // below it, without its id (ids are unique)
    focusStep(editSteps((items) => {
      const copy = structuredClone(items[index].step);
      delete copy.id;
      items.splice(index + 1, 0, { step: copy, open: items[index].open, expanded: true, was: null });
    })[index + 1]);
  });
  const handle = card.querySelector('[data-act="drag"]');
  handle.addEventListener('pointerdown', (e) => dragStep(card, e));
  handle.addEventListener('keydown', (e) => {
    if (!e.altKey || (e.key !== 'ArrowUp' && e.key !== 'ArrowDown')) return;
    e.preventDefault();
    move(index, e.key === 'ArrowUp' ? -1 : 1, 'drag');
  });
  card._read = () => {
    const including = q('skill').value.startsWith(INCLUDE);
    const out = including ? { instruction: q('instruction').value.trim(), include: q('skill').value.slice(INCLUDE.length), with: {} }
      : { instruction: q('instruction').value.trim(), skill: q('skill').value, args: {} };
    const fields = [...argsBox.querySelectorAll(':scope > [data-arg]')];
    const keys = [...new Set([...order, ...fields.map((field) => field.dataset.arg)])];
    keys.forEach((key) => {
      const value = fields.find((field) => field.dataset.arg === key)?._value();
      if (value !== undefined) out[including ? 'with' : 'args'][key] = value;
    });
    if (q('id').value.trim()) out.id = q('id').value.trim();
    if (q('timeout_s').value !== '') out.timeout_s = Number(q('timeout_s').value);
    if (q('retries').value !== '') out.retries = Number(q('retries').value);
    if (q('on_failure').value) out.on_failure = q('on_failure').value;
    if (q('replan_from').value) out.replan_from = q('replan_from').value;
    return out;
  };
  setExpanded(card, expanded);
  stepSummary(card);
  preview._refresh();
  return card;
}

// ---------- a move step's preview ----------
// Preview on a move step: its arguments as they are in the editor, run as a dry run on the robot (POST
// /api/skills/move/dry_run: resolved and solved, nothing moves). The answer stays under the arguments until the step
// is edited: the skill's message (the command and target) over the joint goal per group (deg, m), the base pose and
// torso of a drive, what it carries and the goals on the way; a refusal or a failure as a red line, as the step's
// problems. Not while a mission or a manual skill runs (they share the move skill on the robot), not with a
// placeholder ({fact}, {param}: filled at run time): the button is off and says why.
const PREVIEW_SKILL = 'move';
const placeholdersIn = (value) => (typeof value === 'string' ? (isPlaceholder(value) ? [value] : [])
  : Array.isArray(value) ? value.flatMap(placeholdersIn) : isPlain(value) ? Object.values(value).flatMap(placeholdersIn) : []);
function previewBlocked(args) {
  const snap = state.snapshot;
  const mission = snap?.status?.state;
  if (mission === 'running' || mission === 'paused') return `a mission is ${mission}`;
  if (snap?.manual) return `${snap.manual.label} (${snap.manual.skill}) is running`;
  const held = [...new Set(placeholdersIn(args))];
  if (held.length) return `${held.join(', ')} ${held.length > 1 ? 'are' : 'is'} filled at run time`;
  return Object.keys(args || {}).length ? '' : 'no arguments yet';
}
function movePreview(read) {
  const button = el('button', { type: 'button', class: 'btn small', text: 'Preview',
    title: 'Resolve and solve this move on the robot with the arguments as they are here; nothing moves' });
  const why = el('span', { class: 'muted small-text' });
  const out = el('div', { class: 'preview-out', hidden: '' });
  const node = el('div', { class: 'step-preview' }, el('div', { class: 'row wrap' }, button, why), out);
  let busy = false;
  let step = { skill: '', args: {} };  // the step as last read: _refresh on its edits, _update on status events
  node._update = () => {
    const move = step.skill === PREVIEW_SKILL;
    show(node, move);
    const blocked = move ? previewBlocked(step.args) : '';
    button.disabled = busy || !!blocked;
    why.textContent = busy ? 'Solving… nothing moves.' : blocked ? `No preview: ${blocked}.` : '';
  };
  node._refresh = () => { step = read(); node._update(); };
  node._clear = () => { out.replaceChildren(); show(out, false); };
  button.addEventListener('click', async () => {
    const sent = JSON.stringify(step.args);
    busy = true;
    node._clear();
    node._update();
    let answer;
    try {
      const { ok, data } = await api('POST', `/api/skills/${PREVIEW_SKILL}/dry_run`, { args: step.args });
      answer = ok ? previewAnswer(data)
        : [el('div', { class: 'msg bad', text: (data.problems || [data.error || 'no answer']).join('\n') })];
    } catch (error) {  // no connection (the HMI restarting): say so, never stick on "Solving"
      answer = [el('div', { class: 'msg bad', text: `no answer from the HMI: ${error.message || error}` })];
    } finally {
      busy = false;
      node._update();
    }
    if (JSON.stringify(step.args) !== sent) return;  // edited while it solved: the answer is for other args
    out.replaceChildren(...answer);
    show(out, true);
  });
  return node;
}
// the preview's answer: the message, then label / value rows (a group's values with the joint names as the tooltip)
function previewAnswer(answer) {
  if (answer.outcome !== 'succeeded') return [el('div', { class: 'msg bad', text: `${answer.outcome}: ${answer.message}` })];
  const d = answer.data || {};
  const rows = [];
  const num = (n, digits) => n.toFixed(digits).replace('-', '−');
  const value = (j) => (j.unit === 'm' ? `${num(j.value, 3)} m` : `${num(j.value, 1)}°`);
  const groups = (view) => view.forEach((g) => rows.push([g.group, g.joints.map(value).join('  '), g.joints.map((j) => `${j.name} ${value(j)}`).join('\n')]));
  if (d.pose) rows.push(['base', `x ${num(d.pose.x, 2)} m · y ${num(d.pose.y, 2)} m · yaw ${num(d.pose.yaw * DEG, 1)}°`]);
  if (Array.isArray(d.by)) rows.push(['base', `by ${signed(+d.by[0].toFixed(3))} m${d.by[1] ? ` · ${signed(+d.by[1].toFixed(3))} m sideways` : ''}`]);
  if (answer.torso) groups(answer.torso);
  else if (['navigate', 'straight'].includes(d.kind)) rows.push(['torso', d.torso === 'keep' ? 'kept' : d.torso == null ? 'drive height' : String(d.torso)]);
  groups(answer.goal || []);
  if (d.carrying) rows.push(['carrying', d.carrying]);
  if (Array.isArray(d.legs) && d.legs.length > 1) rows.push(['on the way', d.legs.slice(0, -1).map((leg) => leg.label).join(' → ')]);
  if (!rows.length) rows.push(['goal', 'nothing to move']);
  return [el('div', { class: 'msg preview-answer' }, el('div', { text: answer.message }),
    el('dl', { class: 'preview-goal' }, rows.flatMap(([label, text, tip]) => [el('dt', { text: label }), el('dd', tip ? { text, title: tip } : { text })])))];
}
// the previews follow the mission and the manual skill (a mission start turns them off); each from the step as last
// read, so a status event reads no fields
function updatePreviews() { document.querySelectorAll('#step-editor .step-preview').forEach((node) => node._update()); }

// ---------- folded steps ----------
// A step folds to its summary line (number, instruction, skill and a short reading of its arguments): a long mission
// is reordered by dragging folded steps. The summary is the toggle (click, Enter, Space); the open state stays through
// reorders (editSteps), Check, Save and point refreshes. A mission of more than FOLD_ABOVE steps opens folded; new and
// duplicated steps open, and so do the steps a Check or Save names.
const FOLD_ABOVE = 6;
function setExpanded(card, open) {
  card.classList.toggle('collapsed', !open);
  card.querySelector('[data-act="fold"]').setAttribute('aria-expanded', String(open));
}
const isExpanded = (card) => !card.classList.contains('collapsed');
// "Collapse all" while any step is open, else "Expand all"; none for a single step
function foldButton() {
  const cards = [...$('step-editor').children];
  show($('btn-fold-steps'), cards.length > 1);
  $('btn-fold-steps').textContent = cards.some(isExpanded) ? 'Collapse all' : 'Expand all';
}
$('btn-fold-steps').addEventListener('click', () => {
  const cards = [...$('step-editor').children];
  const open = !cards.some(isExpanded);
  cards.forEach((card) => setExpanded(card, open));
  foldButton();
});
function stepSummary(card) {
  const step = card._read();
  const instruction = card.querySelector('.sum-instruction');
  instruction.textContent = step.instruction || 'no instruction';
  instruction.classList.toggle('muted', !step.instruction);
  const what = [step.include ? `include ${step.include}` : step.skill,
    step.include ? argsSummary(step.with) : step.skill === 'move' ? moveSummary(step.args) : argsSummary(step.args)];
  card.querySelector('.sum-args').textContent = what.filter(Boolean).join(' · ');
  card.querySelector('[data-act="fold"]').title = `${step.instruction}\n${what.filter(Boolean).join(' · ')}`;
}
// Arguments in a few words: "by 0 / −15 / +6 cm · left", "onto tray_1/seat · base · 10 cm short", "linear · speed 50 %".
const signed = (n) => (n > 0 ? `+${n}` : n < 0 ? `−${-n}` : '0');
const cm = (m) => +(m * 100).toFixed(1);
function argsSummary(args) {
  return Object.entries(args || {}).map(([key, value]) => argSummary(key, value)).filter(Boolean).join(' · ');
}
// The move skill's commands: the operator's names of the kinds of motion (core config/targets.py MOVE_LABELS; the
// catalog's x-taskweave-move labels win), the form a target's kind gives (FORM_OF_TARGET), the forms of the option
// table (MOVE_FORMS) and the hand value of a carry relative to now (HOLDING), for a catalog without them.
const MOVE_LABELS = { navigate: 'navigate to', straight: 'move straight', hand: 'move to target', posture: 'change posture',
  joints: 'move to joint positions' };
const TARGET_FORMS = { base: 'navigate', hand: 'hand', object: 'carry', joints: 'joints' };
const MOVE_FORMS = ['navigate', 'straight', 'hand', 'carry', 'posture', 'joints'];
const moveMeta = () => state.skills.find((s) => s.name === 'move')?.args_schema?.['x-taskweave-move'];
const holdingWord = () => moveMeta()?.holding?.value || 'holding';
// a kind's (or form's: carry is a hand move) label
const moveLabel = (kind) => (moveMeta()?.labels || MOVE_LABELS)[kind === 'carry' ? 'hand' : kind] || `move ${kind}`;
// the form of a "to" step from its target's kind (a base target driven straight: straight); null: not known here
function targetForm(args) {
  const form = (moveMeta()?.target_forms || TARGET_FORMS)[targetKind(args?.to)];
  return form === 'navigate' && args?.motion?.mode === 'straight' ? 'straight' : form || null;
}
// a move step that carries a held object, by its arguments: hand: holding, the old object form (object with by / at, as
// icons.js carries() and core's step_info.old_object_form), a "to" an object target
const carriesByArgs = (args) => !!args && (args.hand === holdingWord() || targetForm(args) === 'carry'
  || (args.object !== undefined && args.to === undefined && args.hand === undefined && ('by' in args || 'at' in args)));
// A move step in parts: its form (null: a "to" whose target is not known here), the head (what it goes to or by) and
// the rest (its options). moveSummary puts the command before them; the plan graph has it in the node's title.
function moveParts(args) {
  const rest = { ...(args || {}) };
  const take = (key) => { const value = rest[key]; delete rest[key]; return value; };
  const relative = () => ['by', 'at'].map((key) => (rest[key] === undefined ? '' : argSummary(key, take(key)))).filter(Boolean).join(' ');
  const pairs = (value) => (isPlain(value) ? Object.entries(value).map(([k, v]) => `${k} ${Array.isArray(v) ? v.map((n) => n ?? '–').join(' / ') : v}`).join(', ') : String(value));
  const carrying = () => `carrying${rest.object === undefined ? '' : ` ${take('object')}`}`;
  let form = null, head = '';
  if (rest.to !== undefined) {
    form = targetForm(rest);
    head = `${take('to')}${form === 'carry' ? ` · ${carrying()}` : ''}`;
  } else if (rest.hand !== undefined) {
    const hand = take('hand');
    form = hand === holdingWord() ? 'carry' : 'hand';
    head = `${form === 'carry' && rest.object !== undefined ? carrying() : `hand ${[].concat(hand).join(' + ')}`} ${relative()}`;
  } else if (rest.object !== undefined) [form, head] = ['carry', `carrying ${take('object')} ${relative()}`];  // cell schema 4
  else if (rest.base !== undefined) {
    const by = take('base')?.by;
    const metres = (n) => signed(+Number(n).toFixed(3));
    [form, head] = ['straight', Array.isArray(by) ? `by ${by[1] ? `${metres(by[0])} / ${metres(by[1])}` : metres(by[0])} m` : ''];
  } else if (rest.elbow !== undefined) [form, head] = ['posture', `elbow ${pairs(take('elbow'))}`];
  else if (rest.joints !== undefined) [form, head] = ['joints', pairs(take('joints'))];
  delete rest.dry_run;
  const torso = take('torso');
  return { form, head: head.trim(), rest: [argsSummary(rest), torso === undefined ? '' : `torso ${[].concat(torso).join(' / ')}`].filter(Boolean).join(' · ') };
}
// A move step by its command and form: "navigate to tray_1/place", "move straight by −0.3 m", "move to target · hand
// left by 0 / 0 / +6 cm", "move to target · carrying frame_1 by …", "change posture · elbow left", then its options;
// "to tray_1/place" while the target's kind is not known here.
function moveSummary(args) {
  const { form, head, rest } = moveParts(args);
  const label = form ? moveLabel(form) : '';
  const lead = !label ? `${args?.to !== undefined ? 'to ' : ''}${head}`
    : !head ? label : label.endsWith(' to') || head.startsWith('by ') ? `${label} ${head}` : `${label} · ${head}`;
  return [lead, rest].filter(Boolean).join(' · ');
}
function argSummary(key, value) {
  if (value === undefined || value === null) return '';
  const vector = Array.isArray(value) && value.length === 3 && value.every((n) => typeof n === 'number');
  if (vector && key === 'by') return `by ${value.map((n) => signed(cm(n))).join(' / ')} cm`;
  if (vector && key.endsWith('_deg')) return `${key.replace(/_deg$/, '')} ${value.join(' / ')}°`;
  if (vector) return `${key} ${value.join(' / ')} m`;
  if (Array.isArray(value)) return value.map((item) => (typeof item === 'object' ? JSON.stringify(item) : item)).join(', ');
  if (typeof value === 'object') return argsSummary(value);
  if (typeof value === 'boolean') return value ? key : `no ${key}`;
  if (typeof value === 'string') return key === 'onto' ? `onto ${value}` : value;
  if (key === 'offset_m') return `${cm(value)} cm short`;
  if (key === 'speed' || key === 'accel') return `${key} ${+(value * 100).toFixed(1)} %`;
  if (key === 'seconds') return `${value} s`;
  const unit = key.match(/_(m|mm|s|deg|rad)$/)?.[1];
  return unit ? `${key.slice(0, -unit.length - 1)} ${value}${unit === 'deg' ? '°' : ` ${unit}`}` : `${key} ${value}`;
}

// ---------- argument fields ----------
// A step's arguments as fields made from the skill's JSON schema (the catalog's args_schema): objects as nested fields
// (many optional ones under "more"), oneOf / anyOf of objects as a variant chooser (the variants' titles) with that
// variant's fields, three-number arrays as x / y / z, hands as checkboxes and the rule words, enums and points as
// selects (points from state.points, refilled in place: refreshCatalog), numbers in the schema's unit (metres and
// degrees step by 1 mm and 1°, as on the Teach tab), x-taskweave-show-if hides a field its siblings do not ask for.
// Each editor is { node, read }: read() gives the value, or undefined for one left out. What the schema cannot
// describe (an object of free keys, a value of another shape, a "{fact}" where a list or object belongs) is a JSON
// text field; a structured argument has "edit as text" for the same. An argument that was not edited reads back as it
// was loaded, so opening and saving a mission changes nothing; edited ones keep their keys in the order they had.
const STRUCTURED = new Set(['object', 'variants', 'vec', 'hands', 'list']);
const UNIT_LABEL = { m: 'm', mm: 'mm', deg: '°', s: 's', rad: 'rad', fraction: '0–1' };
const unitStep = (unit) => ({ m: STEP_M, mm: '1', deg: STEP_DEG, fraction: '0.05' })[unit];  // STEP_*: Teach fields
const isPlain = (value) => !!value && typeof value === 'object' && !Array.isArray(value);
const isPlaceholder = (value) => typeof value === 'string' && value.includes('{');  // a param or a fact, filled at run time
const variantsOf = (spec) => spec.oneOf || spec.anyOf || [];
// the choices of an enum field: its enum, or the enum of an anyOf / oneOf branch (elbow: a side, or {hand: side})
const enumOf = (spec) => spec.enum || variantsOf(spec).find((v) => Array.isArray(v.enum))?.enum;
function argKind(spec) {
  const variants = variantsOf(spec);
  if (spec['x-taskweave-widget'] === 'hands' && spec.anyOf?.[1]?.items?.enum) return 'hands';
  const numbers = spec.type === 'array' && ['number', 'integer'].includes(spec.items?.type);
  if (numbers && (spec['x-taskweave-widget'] === 'vec3' || (spec.minItems === spec.maxItems && spec.minItems >= 2 && spec.minItems <= 4))) return 'vec';
  if (variants.length && variants.every((v) => v.properties)) return 'variants';
  if (pointLists(spec)) return 'point';
  if (enumOf(spec)) return 'enum';
  if (spec.type === 'number' || spec.type === 'integer') return 'number';
  if (spec.type === 'boolean') return 'bool';
  if (spec.type === 'string') return 'string';
  if (spec.properties && (spec.type || 'object') === 'object') return 'object';
  if (spec.type === 'array' && spec.items && ['point', 'enum', 'string', 'number'].includes(argKind(spec.items))) return 'list';
  return 'text';
}
// Can the fields show this value as it is (else it is shown as text)?
function fits(spec, value) {
  if (value === undefined) return true;
  const kind = argKind(spec);
  if (kind === 'object') return isPlain(value) && Object.entries(value).every(([k, v]) => k in spec.properties && fits(spec.properties[k], v));
  if (kind === 'variants') return variantOf(spec, value) >= 0;
  if (kind === 'vec') return Array.isArray(value) && value.length === spec.minItems && value.every((n) => typeof n === 'number');
  if (kind === 'hands') {
    const names = Array.isArray(value) ? value : [value];
    return names.length > 0 && names.every((n) => typeof n === 'string' && !isPlaceholder(n));
  }
  if (kind === 'list') return Array.isArray(value) && value.every((item) => fits(spec.items, item));
  if (kind === 'number') return typeof value === 'number' || isPlaceholder(value);
  if (kind === 'bool') return typeof value === 'boolean';
  if (kind === 'enum') return !isPlain(value) && !Array.isArray(value);
  if (kind === 'point' || kind === 'string') return typeof value === 'string';
  return true;  // text
}
const variantOf = (spec, value) => variantsOf(spec).findIndex((v) => isPlain(value)
  && (v.required || []).every((k) => k in value) && fits(v, value));
// the unit: the schema's (x-taskweave-unit), else the key's suffix (offset_m, turn_deg), else the description's words
function unitOf(key, spec) {
  if (spec['x-taskweave-unit']) return spec['x-taskweave-unit'];
  const suffix = key.match(/_(m|mm|deg|s|rad)$/)?.[1];
  if (suffix) return suffix;
  if (/\bin metres\b/.test(spec.description || '')) return 'm';
  return /\bin degrees\b/.test(spec.description || '') ? 'deg' : '';
}
function labelText(key, spec) {
  const unit = unitOf(key, spec);
  const name = key.replace(/_(m|mm|deg|s|rad)$/, '').replace(/_/g, ' ');
  return unit && UNIT_LABEL[unit] ? `${name} (${UNIT_LABEL[unit]})` : name;
}
function labelFor(key, spec, text = labelText(key, spec)) {
  const label = el('label', { text });
  if (spec.description) label.title = `${key}: ${spec.description}`;
  return label;
}

// One argument of a step (or a param of an included mission): a .field with data-arg and _value(). Structured ones
// can be switched to text and back ("edit as fields" only once the text fits the fields again).
function argField(key, spec, required, value) {
  const kind = argKind(spec);
  const field = el('div', { class: `field arg${STRUCTURED.has(kind) ? ' wide' : ''}` });
  field.dataset.arg = key;
  const group = kind === 'object' && spec['x-taskweave-group'];  // e.g. Motion (advanced): folded until it has a value
  const label = labelFor(key, spec, `${group || labelText(key, spec)}${required || group ? '' : ' (optional)'}`);
  const body = el('div', { class: 'arg-body' });
  const note = el('div', { class: 'msg bad', hidden: '' });
  let asText = (STRUCTURED.has(kind) || kind === 'enum') && !fits(spec, value);  // an enum's other shape: as text
  let editor = null;
  const toggle = STRUCTURED.has(kind) ? el('button', { class: 'btn small ghost arg-mode', type: 'button' }) : '';
  const build = (current) => {
    editor = asText ? textEditor(current) : argEditor(key, spec, current, required);
    body.replaceChildren(editor.node);
    if (toggle) toggle.textContent = asText ? 'edit as fields' : 'edit as text';
  };
  build(value);
  if (toggle) {
    toggle.addEventListener('click', () => {
      const current = editor.read();
      if (asText && !fits(spec, current)) {
        message(note, 'This value does not fit the fields: keep it as text, or change it to fit the schema.', true);
        return;
      }
      message(note, '');
      asText = !asText;
      build(current);
    });
  }
  let edited = false;  // not edited: read back exactly as loaded
  ['input', 'change'].forEach((type) => field.addEventListener(type, () => { edited = true; }));
  field._value = () => (edited ? editor.read() : value);
  field._fields = () => editor.read();  // what the fields give now, edited or not (round-trip checks)
  if (group) {
    const fold = el('details', { class: 'arg-group' }, el('summary', {}, label), el('div', { class: 'arg-head' }, toggle), body, note);
    fold.open = value !== undefined;
    field.append(fold);
  } else {
    field.append(el('div', { class: 'arg-head' }, label, toggle), body, note);
  }
  return field;
}
// a field's value changed by a click (a variant, a rule word, a list row): as if typed
const changed = (node) => node.dispatchEvent(new Event('change', { bubbles: true }));

function argEditor(key, spec, value, required) {
  const kind = argKind(spec);
  if (kind === 'object') return objectEditor(spec, value);
  if (kind === 'variants') return variantsEditor(spec, value);
  if (kind === 'vec') return vecEditor(key, spec, value);
  if (kind === 'hands') return handsEditor(spec, value);
  if (kind === 'list') return listEditor(key, spec, value);
  if (kind === 'text') return textEditor(value);
  return scalarEditor(key, spec, value, required);
}

// A point picker or enum (select), number, string or boolean. A placeholder ("{part}/grasp") is a text field.
function scalarEditor(key, spec, value, required) {
  const kind = argKind(spec);
  // the schema's default is a hint only (a skill or robot may set another): "default", left out when empty
  const fallback = spec.default !== undefined ? 'default' : '';
  let input;
  if ((kind === 'point' || kind === 'enum') && !isPlaceholder(value)) {
    const kinds = pointLists(spec) || [];
    input = el('select');
    input.dataset.points = '';
    const choices = enumOf(spec);
    input._fill = (chosen = input.value) => {  // the choices from state.points (again: refreshCatalog), chosen kept
      const options = (choices || kinds.flatMap((k) => state.points[k] || [])).map(String);
      if (!choices && kinds.length > 1) {  // several lists (base, hand, object, joint targets): one group each
        input.replaceChildren(el('option', { value: '' }), ...kinds.filter((k) => (state.points[k] || []).length).map((k) => el('optgroup',
          { label: k.charAt(0).toUpperCase() + k.slice(1) }, state.points[k].map((v) => el('option', { value: String(v), text: String(v) })))));
      } else fillSelect(input, ['', ...options], chosen);
      if (chosen !== '' && !options.includes(chosen)) input.append(el('option', { value: chosen, text: `${chosen} (unknown)` }));
      input.value = chosen;
      input.options[0].textContent = required ? 'choose…' : fallback || 'none';
    };
    input._fill(value === undefined ? '' : String(value));
    const read = () => {
      if (input.value === '') return undefined;
      const known = choices?.find((option) => String(option) === input.value);
      return known !== undefined ? known : input.value;
    };
    return { node: input, read };
  }
  if (kind === 'number' && !isPlaceholder(value)) {
    const unit = unitOf(key, spec);
    input = el('input', { type: 'number', step: unitStep(unit) || (spec.type === 'integer' ? '1' : 'any') });
    const min = spec.minimum ?? spec.exclusiveMinimum, max = spec.maximum ?? spec.exclusiveMaximum;
    if (min !== undefined) input.setAttribute('min', String(min));
    if (max !== undefined) input.setAttribute('max', String(max));
    input.value = value ?? '';
    if (fallback) input.placeholder = fallback;
    return { node: input, read: () => (input.value === '' ? undefined : Number(input.value)) };
  }
  if (kind === 'bool') {
    input = el('input', { type: 'checkbox' });
    input.checked = value === true;
    // unchecked is left out, unless the step said false (or it must be given)
    return { node: el('label', { class: 'check' }, input, 'yes'), read: () => (input.checked ? true : (value === false || required ? false : undefined)) };
  }
  input = el('input', { spellcheck: 'false' });
  input.value = value ?? '';
  if (spec.description && !isPlaceholder(value)) input.placeholder = spec.description;
  return { node: input, read: () => (input.value === '' ? undefined : input.value) };
}

// JSON text: what the fields cannot show; text that is not JSON is kept as a string, and a string that is no JSON
// is shown as it is (torso: keep, not "keep")
const isJson = (text) => { try { JSON.parse(text); return true; } catch (_) { return false; } };
function textEditor(value) {
  const input = el('textarea', { rows: '2', spellcheck: 'false', class: 'mono' });
  input.value = value === undefined ? '' : typeof value === 'string' && !isJson(value) ? value : JSON.stringify(value);
  const read = () => {
    if (!input.value.trim()) return undefined;
    try { return JSON.parse(input.value); } catch (_) { return input.value; }
  };
  return { node: input, read };
}

// Nested fields in a grid, required ones first; more than MORE_FROM optional ones without a value go under "more".
// Keys come back in the order the value had them, new ones in the schema's order.
const MORE_FROM = 4;
function objectEditor(spec, value) {
  const required = spec.required || [];
  const props = Object.entries(spec.properties).sort(([a], [b]) => required.includes(b) - required.includes(a));
  const fields = props.map(([key, sub]) => {
    const editor = argEditor(key, sub, value?.[key], required.includes(key));
    const wide = STRUCTURED.has(argKind(sub));
    const label = labelFor(key, sub, wide ? key.replace(/_/g, ' ') : labelText(key, sub));  // a vector's unit: on x, y, z
    const node = el('div', { class: `field${wide ? ' wide' : ''}` }, label, editor.node);
    return { key, spec: sub, node, read: editor.read, required: required.includes(key) };
  });
  const box = el('div', { class: 'arg-fields' });
  const spare = fields.filter((f) => !f.required && value?.[f.key] === undefined);
  const fold = !spec['x-taskweave-group'] && spare.length >= MORE_FROM;
  box.append(...fields.filter((f) => !fold || !spare.includes(f)).map((f) => f.node));
  const names = spare.map((f) => f.key).join(', ');
  if (fold) box.append(el('details', { class: 'arg-more wide' }, el('summary', { text: spare.length < fields.length ? `more (${names})` : names }),
    el('div', { class: 'arg-fields' }, spare.map((f) => f.node))));
  showIf(box, fields);
  const read = () => {
    const out = {};
    const order = [...Object.keys(value || {}), ...fields.map((f) => f.key)];
    [...new Set(order)].forEach((key) => {
      const v = fields.find((f) => f.key === key)?.read();
      if (v !== undefined) out[key] = v;
    });
    return Object.keys(out).length || (isPlain(value) && !Object.keys(value).length) ? out : undefined;  // {} stays {}
  };
  return { node: el('div', { class: 'arg-nest' }, box), read };
}
// x-taskweave-show-if {sibling: [values]}: the field shows while a sibling has one of them, or while it has a value
function showIf(box, fields) {
  const conditional = fields.filter((f) => f.spec?.['x-taskweave-show-if']);
  ['input', 'change'].forEach((type) => box.removeEventListener(type, box._showIf));  // the step's args drawn again
  if (!conditional.length) return;
  const update = () => conditional.forEach((f) => {
    const wanted = Object.entries(f.spec['x-taskweave-show-if']).every(([sibling, values]) => {
      const other = fields.find((g) => g.key === sibling);
      return other && values.includes(other.read() ?? other.spec.default);
    });
    show(f.node, wanted || f.read() !== undefined);
  });
  box._showIf = update;
  ['input', 'change'].forEach((type) => box.addEventListener(type, update));
  update();
}

// oneOf / anyOf of objects: a segmented chooser of the variants (their titles), then the chosen one's fields. A
// switch keeps the values the new variant also has (offset_m from a point to a contact); going back to a variant
// brings its values back (while the step is open in the editor).
function variantsEditor(spec, value) {
  const variants = variantsOf(spec);
  let chosen = Math.max(0, variantOf(spec, value));
  let editor = null;
  const memory = {};  // variant index -> its values when it was left
  const body = el('div', { class: 'arg-variant' });
  const chooser = el('div', { class: 'segmented choice', role: 'group' });
  const title = (v) => {
    const text = v.title || (v.required || Object.keys(v.properties)).join(' + ');
    return text.charAt(0).toUpperCase() + text.slice(1);
  };
  const pick = (index, current) => {
    chosen = index;
    editor = objectEditor(variants[index], current);
    body.replaceChildren(editor.node);
    chooser.querySelectorAll('button').forEach((b, i) => b.setAttribute('aria-pressed', String(i === index)));
  };
  chooser.append(...variants.map((v, index) => {
    const button = el('button', { type: 'button', text: title(v), 'aria-pressed': 'false' });
    if (v.description) button.title = v.description;
    button.addEventListener('click', () => {
      if (index === chosen) return;
      const current = editor.read() || {};
      memory[chosen] = current;
      const kept = Object.fromEntries(Object.entries({ ...memory[index], ...current })
        .filter(([k, x]) => variants[index].properties[k] && fits(variants[index].properties[k], x)));
      pick(index, Object.keys(kept).length ? kept : undefined);
      changed(chooser);
    });
    return button;
  }));
  pick(chosen, value);
  return { node: el('div', { class: 'arg-variants' }, chooser, body), read: () => editor.read() };
}

// ---------- a skill of several forms (move) ----------
// A skill whose args schema is a oneOf of forms (each only its required keys, the properties shared), as move: a
// chooser of the forms (their titles), then the chosen form's own keys and the options that apply to it. Which apply
// comes from the catalog (x-taskweave-move: the options per form of the move, core's MOVE_OPTIONS on navigate,
// straight, hand, carry, posture, joints); for a form "to" a target it depends on the target's kind (state.points
// groups, x-taskweave-move target_forms), for "hand" on the hand (hand: holding is a carry), so the options of the
// other forms hide while they have no value. A key the step has is always shown (nothing is dropped unseen) but
// dry_run (never saved in a mission) and the reserved look_at hide. Switching the form keeps the options the new form
// also takes, and going back to a form brings its values back (while the step is open in the editor). A deprecated
// form (x-taskweave-deprecated: the held object relative to now of cell schema 4) still opens and edits, marked.
const HIDDEN_ARGS = { dry_run: 'never saved in a mission (it is for trying a move by hand)', look_at: 'reserved, not implemented yet' };
const NEVER_SAVED = ['dry_run'];
function formsOf(schema) {
  const forms = schema?.oneOf;
  return Array.isArray(forms) && forms.length > 1 && schema.properties
    && forms.every((f) => Array.isArray(f.required) && f.required.length && !f.properties) ? forms : null;
}
const formIndex = (forms, args) => forms.findIndex((f) => f.required.every((key) => key in (args || {})));
// the kind of target a name is (base, hand, object, joints), from the pickers' groups; null: not known here
function targetKind(name) {
  const found = Object.entries(TARGET_GROUPS).find(([, group]) => (state.points[group] || []).includes(name));
  return found ? found[0] : null;
}
// the forms whose options a step of this form takes: the selector's (base: straight, elbow: posture, ...), hand:
// holding a carry, for "to" the target's (a base target: navigate, and straight unless it already drives straight;
// every form while the target is not known)
function optionForms(schema, form, args) {
  const meta = schema['x-taskweave-move'];
  const selector = form.required[0];
  if (selector === 'hand' && args?.hand === holdingWord()) return meta?.holding?.forms || ['carry'];
  if (meta?.forms?.[selector]) return meta.forms[selector];
  if (selector !== 'to') return [selector];
  const target = (meta?.target_forms || TARGET_FORMS)[targetKind(args?.to)];
  if (target === 'navigate') return args?.motion?.mode === 'straight' ? ['straight'] : ['navigate', 'straight'];
  return target ? [target] : MOVE_FORMS;
}
// every form a step of this form can take, what its fields are drawn for (the others hide while empty): every form
// for "to", hand and carry for "hand"
function allForms(schema, form) {
  if (form.required[0] === 'to') return MOVE_FORMS;
  const holding = form.required[0] === 'hand' ? optionForms(schema, form, { hand: holdingWord() }) : [];
  return [...new Set([...optionForms(schema, form, {}), ...holding])];
}
// the keys a form shows for these forms, in the schema's order: its selector, its relative keys, the options
function formKeys(schema, form, kinds) {
  const meta = schema['x-taskweave-move'];
  const selectors = new Set(formsOf(schema).flatMap((f) => f.required));
  const hidden = meta?.hidden || HIDDEN_ARGS;
  const selector = form.required[0];
  const relative = selector === 'to' ? [] : kinds.flatMap((kind) => meta?.relative?.[kind] || []);
  return Object.keys(schema.properties).filter((key) => {
    if (key in hidden) return false;
    if (form.required.includes(key)) return true;
    if (!meta) return !selectors.has(key);  // no table: every option
    if (relative.includes(key)) return true;
    if (relative.length && (meta.not_relative || []).includes(key)) return false;  // offset_m, reference: a target's
    if (key === 'object') return ['to', 'hand'].includes(selector) && kinds.includes('carry');  // which held object goes
    return !selectors.has(key) && (meta.options?.[key] || []).some((k) => kinds.includes(k));
  });
}
function formArgs(argsBox, schema, args, strays) {
  const forms = formsOf(schema);
  const props = schema.properties;
  let chosen = formIndex(forms, args);
  if (chosen < 0 && !Object.keys(args || {}).length) chosen = 0;  // a new step: the first form
  const memory = {};  // form index -> its values when it was left
  const chooser = el('div', { class: 'segmented choice', role: 'group', 'aria-label': 'Form' });
  const head = el('div', { class: 'field wide arg-forms' }, chooser);
  const dropped = NEVER_SAVED.filter((key) => key in (args || {}));
  if (dropped.length) head.append(el('div', { class: 'msg warn', text: `${dropped.join(', ')} is left out: ${HIDDEN_ARGS.dry_run}.` }));
  const note = el('div', { class: 'msg warn', hidden: '' });  // the deprecated form: what it is now
  head.append(note);
  const fields = () => [...argsBox.querySelectorAll(':scope > [data-arg]')];
  const current = () => Object.fromEntries(fields().map((f) => [f.dataset.arg, f._value()]).filter(([, v]) => v !== undefined));
  const draw = (values) => {
    chooser.querySelectorAll('button').forEach((b, i) => b.setAttribute('aria-pressed', String(i === chosen)));
    const form = forms[chosen];
    const all = form ? formKeys(schema, form, allForms(schema, form)) : [];
    const shown = Object.keys(values).filter((key) => key in props && !all.includes(key) && !NEVER_SAVED.includes(key));
    argsBox.replaceChildren(head, ...[...all, ...shown].map((key) => argField(key, props[key], !!form?.required.includes(key), values[key])),
      ...strays(props));
    hideOptions();
  };
  // a "to" or "hand" form: the options of the forms the step does not take (the target's, hand: holding), while they
  // have no value; the deprecated form's note
  const hideOptions = () => {
    const form = forms[chosen];
    if (!form) return;
    const deprecated = form['x-taskweave-deprecated'];
    note.textContent = deprecated ? `The form of cell schema 4; now ${deprecated}.` : '';
    show(note, !!deprecated);
    if (!['to', 'hand'].includes(form.required[0])) return;
    const values = current();
    const keys = formKeys(schema, form, optionForms(schema, form, values));
    fields().forEach((f) => show(f, keys.includes(f.dataset.arg) || f._value() !== undefined || !(f.dataset.arg in props)));
  };
  chooser.append(...forms.map((form, index) => {
    const deprecated = form['x-taskweave-deprecated'];
    const button = el('button', { type: 'button', text: form.title || form.required.join(' + '), 'aria-pressed': 'false' });
    if (deprecated) button.classList.add('deprecated');
    button.title = [form.description, deprecated ? `cell schema 4; now ${deprecated}` : ''].filter(Boolean).join('\n');
    button.addEventListener('click', () => {
      if (index === chosen) return;
      const values = current();
      if (chosen >= 0) memory[chosen] = values;
      chosen = index;
      const keys = formKeys(schema, forms[index], allForms(schema, forms[index]));
      const others = new Set(forms.flatMap((f) => f.required).filter((key) => !forms[index].required.includes(key)));
      draw(Object.fromEntries(Object.entries({ ...memory[index], ...values }).filter(([key]) => keys.includes(key) && !others.has(key))));
      changed(chooser);
    });
    return button;
  }));
  ['input', 'change'].forEach((type) => argsBox.removeEventListener(type, argsBox._showIf));
  argsBox._showIf = hideOptions;
  ['input', 'change'].forEach((type) => argsBox.addEventListener(type, hideOptions));
  draw(Object.fromEntries(Object.entries(args || {}).filter(([key]) => key in props)));
}

// [x, y, z] (or roll, pitch, yaw) as three number fields in the unit; all empty: left out, one empty: 0
function vecEditor(key, spec, value) {
  const unit = unitOf(key, spec);
  // the components as the description names them ("[forward, left] in metres"), else x y z
  const named = (spec.description || '').match(/^\[([a-z_]+(?:, [a-z_]+)+)\]/i)?.[1].split(', ');
  const names = key.startsWith('rpy') ? ['roll', 'pitch', 'yaw']
    : named?.length === spec.minItems ? named : ['x', 'y', 'z', 'w'].slice(0, spec.minItems);
  const inputs = names.map((name, i) => {
    const input = el('input', { type: 'number', step: unitStep(unit) || 'any', 'aria-label': `${key} ${name}` });
    input.value = value?.[i] ?? '';
    return input;
  });
  const grid = el('div', { class: names.length === 3 ? 'grid3' : 'arg-fields' }, names.map((name, i) => el('div', { class: 'field' },
    el('label', { text: unit && UNIT_LABEL[unit] ? `${name} (${UNIT_LABEL[unit]})` : name }), inputs[i])));
  const read = () => (inputs.every((input) => input.value === '') ? undefined : inputs.map((input) => (input.value === '' ? 0 : Number(input.value))));
  return { node: grid, read };
}

// hands: a checkbox per hand of the robot, or one of the rule words (auto, any_one, all) as a segmented choice; a
// rule clears the hands and a hand clears the rule, the chosen rule pressed again clears it (left out: the default).
// A list is written in the robot's hand order, unless it holds the same hands as the step had (kept as it was).
function handsEditor(spec, value) {
  const names = [...spec.anyOf[1].items.enum];
  const rules = spec.anyOf[0].enum.filter((word) => !names.includes(word));
  const given = value === undefined ? [] : Array.isArray(value) ? value : [value];
  given.filter((n) => !rules.includes(n) && !names.includes(n)).forEach((n) => names.push(n));  // unknown here: kept
  let rule = typeof value === 'string' && rules.includes(value) ? value : null;
  const boxes = names.map((name) => {
    const input = el('input', { type: 'checkbox', value: name });
    input.checked = !rule && given.includes(name);
    input.addEventListener('change', () => { if (input.checked) setRule(null); });
    return input;
  });
  const chooser = el('div', { class: 'segmented', role: 'group', 'aria-label': 'Hands rule' });
  const ruleButtons = rules.map((word) => {
    const button = el('button', { type: 'button', text: word, 'aria-pressed': 'false' });
    button.addEventListener('click', () => {
      setRule(rule === word ? null : word);
      if (rule) boxes.forEach((input) => { input.checked = false; });
      changed(chooser);
    });
    return button;
  });
  chooser.append(...ruleButtons);
  function setRule(word) {
    rule = word;
    ruleButtons.forEach((b) => b.setAttribute('aria-pressed', String(b.textContent === word)));
  }
  setRule(rule);
  const read = () => {
    if (rule) return rule;
    const picked = boxes.filter((input) => input.checked).map((input) => input.value);
    if (!picked.length) return undefined;
    if (typeof value === 'string' && picked.length === 1 && picked[0] === value) return value;
    if (Array.isArray(value) && picked.length === value.length && value.every((n) => picked.includes(n))) return [...value];
    return picked;
  };
  const node = el('div', { class: 'hands-pick' },
    names.map((name, i) => el('label', { class: 'check' }, boxes[i], name)), el('span', { class: 'muted', text: 'or' }), chooser);
  return { node, read };
}

// a list of points, names or numbers (only, via): a row per item with ✕, + Add; order kept
function listEditor(key, spec, value) {
  const rows = el('div', { class: 'arg-list' });
  const editors = [];
  const add = (item) => {
    const editor = argEditor(key, spec.items, item, true);
    const remove = el('button', { class: 'icon', type: 'button', text: '✕', 'aria-label': 'Remove' });
    const row = el('div', { class: 'row gap' }, editor.node, remove);
    editors.push(editor);
    remove.addEventListener('click', () => { editors.splice(editors.indexOf(editor), 1); row.remove(); changed(rows); });
    rows.append(row);
  };
  (value || []).forEach(add);
  const more = el('button', { class: 'btn small', type: 'button', text: '+ Add' });
  more.addEventListener('click', () => { add(undefined); changed(rows); });
  const read = () => {
    const items = editors.map((editor) => editor.read()).filter((item) => item !== undefined);
    return items.length || Array.isArray(value) ? items : undefined;
  };
  return { node: el('div', { class: 'arg-list-box' }, rows, el('div', {}, more)), read };
}

// Change the step list and draw it again: change gets [{step, open, expanded}] (the steps as edited now, rules folded
// out or not, the step open or folded) to reorder, remove or insert. A step without an id of its own is s<position>,
// so a moved step's id changes: replan_from follows the step it named; one that named a removed step falls back to the
// step itself (and says so).
// Returns the new step cards.
function editSteps(change) {
  const items = [...$('step-editor').children].map((card, i) => {
    const step = card._read();
    return { step, open: card.querySelector('.rules').open, expanded: isExpanded(card), was: step.id || `s${i + 1}` };
  });
  const before = new Set(items.map((item) => item.was));
  change(items);
  const now = new Map();
  items.forEach((item, i) => { if (item.was) now.set(item.was, item.step.id || `s${i + 1}`); });
  const lost = [];
  items.forEach((item, i) => {
    const target = item.step.replan_from;
    if (!target || !before.has(target)) return;  // not a step of this list (already wrong): Check reports it
    if (now.has(target)) item.step.replan_from = now.get(target);
    else { delete item.step.replan_from; lost.push(i + 1); }
  });
  renderSteps(items.map((item) => item.step), items.map((item) => item.expanded));
  const cards = [...$('step-editor').children];
  cards.forEach((card, i) => { card.querySelector('.rules').open = items[i].open; });
  message($('save-msg'), lost.length ? `Step ${lost.join(', ')} replanned from the removed step: now from itself.` : '', 'warn');
  markDirty();
  return cards;
}

// A new or duplicated step: its top just below the sticky top bar, its instruction focused to type.
function focusStep(card) {
  window.scrollBy(0, card.getBoundingClientRect().top - document.querySelector('.top').offsetHeight - 8);
  card.querySelector('[data-k="instruction"]').focus({ preventScroll: true });
}

// One place up or down (the arrow buttons, Alt + arrow on the handle); the focus stays on the control used.
function move(index, delta, control) {
  const target = index + delta;
  if (target < 0 || target >= $('step-editor').children.length) return;
  const card = editSteps((items) => items.splice(target, 0, ...items.splice(index, 1)))[target];
  const button = card.querySelector(`[data-act="${control}"]`);
  (button.disabled ? card.querySelector('[data-act="drag"]') : button).focus();
}

// Drag a step by its handle (mouse, pen or touch: pointer events; the handle has touch-action none, so a touch drags
// instead of scrolling). The step follows the pointer (folded to its summary line while carried), a line marks where it
// lands, and near the top bar or the bottom of the window the page scrolls. Escape or a lost pointer puts it back.
const DRAG_EDGE = 72;  // px from the top bar or the window's bottom where the page scrolls while dragging
function dragStep(card, e) {
  const list = $('step-editor');
  const cards = [...list.children];
  if (e.button !== 0 || cards.length < 2) return;
  e.preventDefault();
  const handle = e.currentTarget;
  handle.setPointerCapture(e.pointerId);
  const from = cards.indexOf(card);
  const start = { y: e.clientY, scroll: window.scrollY };
  let y = e.clientY, to = from, frame = 0, done = false;
  list.classList.add('dragging'); card.classList.add('dragging');
  const mark = () => {
    card.style.transform = `translateY(${y - start.y + window.scrollY - start.scroll}px)`;
    let slot = cards.length;  // it lands before cards[slot]: the first other step whose middle is below the pointer
    for (let i = 0; i < cards.length; i += 1) {
      const box = cards[i].getBoundingClientRect();
      if (i !== from && y < box.top + box.height / 2) { slot = i; break; }
    }
    to = slot > from ? slot - 1 : slot;
    cards.forEach((c) => c.classList.remove('drop-before', 'drop-after'));
    if (to !== from) cards[Math.min(slot, cards.length - 1)].classList.add(slot < cards.length ? 'drop-before' : 'drop-after');
  };
  const scroll = () => {
    const top = document.querySelector('.top').getBoundingClientRect().bottom + DRAG_EDGE;
    const rail = document.querySelector('.rail').getBoundingClientRect();
    const floor = rail.top > window.innerHeight / 2 ? rail.top : window.innerHeight;  // the phone's bottom nav, else the window
    const bottom = Math.min(floor, $('editor').querySelector('.sticky-actions').getBoundingClientRect().top) - DRAG_EDGE;
    const speed = y < top ? -Math.min(top - y, DRAG_EDGE) : y > bottom ? Math.min(y - bottom, DRAG_EDGE) : 0;
    if (speed) { window.scrollBy(0, speed / 4); mark(); }
    frame = requestAnimationFrame(scroll);
  };
  const onMove = (ev) => { y = ev.clientY; mark(); };
  const onKey = (ev) => { if (ev.key === 'Escape') end(false); };  // not stopped: Escape still aborts a manual skill
  function end(drop) {
    if (done) return;
    done = true;
    cancelAnimationFrame(frame);
    handle.removeEventListener('pointermove', onMove);
    document.removeEventListener('keydown', onKey);
    list.classList.remove('dragging'); card.classList.remove('dragging'); card.style.transform = '';
    cards.forEach((c) => c.classList.remove('drop-before', 'drop-after'));
    if (drop && to !== from) {
      editSteps((items) => items.splice(to, 0, ...items.splice(from, 1)))[to].querySelector('[data-act="drag"]').focus({ preventScroll: true });
    }
  }
  handle.addEventListener('pointermove', onMove);
  handle.addEventListener('pointerup', () => end(true), { once: true });
  handle.addEventListener('pointercancel', () => end(false), { once: true });
  handle.addEventListener('lostpointercapture', () => end(false), { once: true });
  document.addEventListener('keydown', onKey);
  frame = requestAnimationFrame(scroll);
}

function readMission() {
  const mission = { goal: $('m-goal').value.trim(), loop: $('m-loop').value, steps: readSteps() };
  if (state.editing.mission.robot) mission.robot = state.editing.mission.robot;  // keep a robot-specific mission
  for (const key of ['params', 'facts']) {  // not edited here (block parameters, cycle facts): kept as they are
    if (state.editing.mission[key]) mission[key] = state.editing.mission[key];
  }
  if ($('m-max').value) mission.max_cycles = Number($('m-max').value);
  if ($('m-stop').value.trim()) mission.stop_when = $('m-stop').value.trim();
  const defaults = {};
  if ($('d-timeout').value !== '') defaults.timeout_s = Number($('d-timeout').value);
  if ($('d-retries').value !== '') defaults.retries = Number($('d-retries').value);
  if ($('d-fail').value) defaults.on_failure = $('d-fail').value;
  if ($('d-replans').value !== '') defaults.max_replans = Number($('d-replans').value);
  if (Object.keys(defaults).length) mission.defaults = defaults;
  return mission;
}

// The problems of a Check or a refused Save, above the buttons; the steps they name are marked red until they are
// edited: by the server's places ({problem, step, path}: the step of the mission's own steps and the path in it, e.g.
// ['args', 'motion', 'mode'] or ['skill']), else by the text ("step 's3' (...)", a step of an included mission as
// "<include step id>_..."). A place in a step's arguments or settings also marks that field, with the problem under it.
function showProblems(problems, places = []) {
  show($('problems'), problems.length > 0);
  $('problems').textContent = problems.join('\n');
  const named = problems.flatMap((p) => [...p.matchAll(/step '([^']+)'/g)].map((m) => m[1]));
  const marks = {};
  places.forEach((place) => { (marks[place.step] = marks[place.step] || []).push({ path: place.path || [], text: place.message || problems[place.problem] }); });
  [...$('step-editor').children].forEach((card, i) => {
    const step = card._read();
    const id = step.id || `s${i + 1}`;
    const bad = i in marks || named.some((n) => n === id || (!!step.include && n.startsWith(`${id}_`)));
    card.classList.toggle('has-problem', bad);
    card.querySelectorAll('.field.has-problem').forEach(unmarkField);
    (marks[i] || []).forEach(({ path, text }) => markField(card, path, text));
    if (bad) setExpanded(card, true);  // opens to show what is wrong
  });
  foldButton();
}
// the field of a step a problem's path names: an argument (args.<key>, also nested: args.motion.mode marks motion),
// the skill, or a setting under the rules (on_failure, replan_from, ...)
function markField(card, path, text) {
  let field = null;
  if ((path[0] === 'args' || path[0] === 'with') && path.length > 1) {
    field = card.querySelector(`.args > [data-arg="${CSS.escape(String(path[1]))}"]`);
    field?.querySelectorAll('details').forEach((d) => { d.open = true; });
    if (field) show(field, true);
  } else if (path.length) {
    field = card.querySelector(`[data-k="${CSS.escape(String(path[0]))}"]`)?.closest('.field');
    if (field && card.querySelector('.rules')?.contains(field)) card.querySelector('.rules').open = true;
  }
  if (!field) return;
  field.classList.add('has-problem');
  const note = el('div', { class: 'msg bad problem-note', text: path.length > 2 ? `${path.slice(1).join('.')}: ${text}` : text });  // motion.mode: ...
  field.append(note);
  const clear = () => unmarkField(field);
  ['input', 'change'].forEach((type) => field.addEventListener(type, clear, { once: true }));
}
// The notes of a Check ({step, path, include, message}: what runs but is noted, e.g. a form of cell schema 4): a
// warning line on the step, above its arguments, until the next Check
function showNotes(notes) {
  [...$('step-editor').children].forEach((card, i) => {
    card.querySelectorAll('.step-note').forEach((node) => node.remove());
    notes.filter((n) => n.step === i).forEach((n) => card.querySelector('.args')?.before(el('div', { class: 'msg step-note',
      text: `${n.include ? `${n.include}: ` : ''}${n.message}` })));
  });
}
function unmarkField(field) {
  field.classList.remove('has-problem');
  field.querySelectorAll(':scope > .problem-note').forEach((note) => note.remove());
}

$('btn-add-step').addEventListener('click', () => {
  const cards = editSteps((items) => items.push({ step: { instruction: '', skill: state.skills[0]?.name || 'wait', args: {} },
    open: false, expanded: true, was: null }));
  focusStep(cards[cards.length - 1]);
});
$('btn-validate').addEventListener('click', async () => {
  const name = $('m-name').value.trim() || 'new';
  const { ok, data } = await api('POST', `/api/missions/${encodeURIComponent(name)}/validate`, readMission());
  if (!ok) { alertInline(data); return; }
  showProblems(data.problems, data.places);
  showNotes(data.notes || []);
  const noted = (data.notes || []).length;
  message($('save-msg'), data.problems.length ? '' : `Mission is valid${noted ? ` (${noted} note${noted > 1 ? 's' : ''} on its steps)` : ''}.`);
});
$('btn-save').addEventListener('click', async () => {
  const name = $('m-name').value.trim();
  if (!name) { showProblems(['Give the mission a name.']); return; }
  const old = state.editing.isNew ? null : state.editing.name;
  const rename = old && old !== name ? `?rename_from=${encodeURIComponent(old)}` : '';
  const { ok, data } = await api('PUT', `/api/missions/${encodeURIComponent(name)}${rename}`, readMission());
  if (!ok) { showProblems(data.problems || [data.error || 'not saved'], data.places); message($('save-msg'), ''); return; }
  showProblems([]);
  state.dirty = false; show($('editor-dirty'), false);
  $('mission-search').value = '';  // the saved mission is in the list, whatever was searched
  state.editing = { ...state.editing, name, isNew: false };
  show($('btn-delete'), true);
  // the plan graph reads its band names again
  if (typeof forgetMission === 'function') [name, old].filter(Boolean).forEach(forgetMission);
  message($('save-msg'), rename ? `Renamed ${old} to ${name} and saved. It is used at the next start.`
    : `Saved ${name}. It is used at the next start.`);
  loadMissions();
});
let deleteArmed = null;
function disarmDelete() {
  clearTimeout(deleteArmed); deleteArmed = null;
  $('btn-delete').classList.remove('confirm'); $('btn-delete').textContent = 'Delete';
}
$('btn-delete').addEventListener('click', async () => {
  const button = $('btn-delete');
  if (!deleteArmed) {  // first tap arms, the second within 3 s deletes
    button.classList.add('confirm'); button.textContent = `Tap again to delete ${state.editing.name}`;
    deleteArmed = setTimeout(disarmDelete, 3000);
    return;
  }
  disarmDelete();
  const name = state.editing.name;
  const { ok, data } = await api('DELETE', `/api/missions/${encodeURIComponent(name)}`);
  if (!ok) { showProblems(data.problems || [data.error || 'not deleted']); return; }
  state.editing = null; state.dirty = false;
  showEditor(false);
  loadMissions();
});
$('btn-new').addEventListener('click', () => dropDirty($('btn-new'), () => openEditor(null)));
$('btn-back').addEventListener('click', () => {
  if (state.dirty && !$('btn-back').classList.contains('confirm')) {
    $('btn-back').classList.add('confirm'); $('btn-back').textContent = 'Discard changes?';
    setTimeout(() => { $('btn-back').classList.remove('confirm'); $('btn-back').textContent = '← Missions'; }, 3000);
    return;
  }
  $('btn-back').classList.remove('confirm'); $('btn-back').textContent = '← Missions';
  state.editing = null; state.dirty = false;
  showEditor(false);
  loadMissions();
});
['m-name', 'm-goal', 'm-loop', 'm-max', 'm-stop', 'd-timeout', 'd-retries', 'd-fail', 'd-replans'].forEach((id) => $(id).addEventListener('input', markDirty));

// ---------- points ----------
// Locations (cell frame) and stance offsets (object frame) from config; yaw is shown in degrees, stored in radians.
let pointsVisible = false;
let robotVisible = false;
let pointsBusy = '';
const DEG = 180 / Math.PI;
// arrow steps (↑ / ↓ and the field's arrows; Shift: 10 times): positions 1 mm, angles 1° (the fields show degrees).
// Stored values keep their own precision (e.g. 0.6899 m): a step starts from the value, it does not round it.
const STEP_M = '0.001';
const STEP_DEG = '1';
const fmt = (p) => `x ${p.x.toFixed(3)}  y ${p.y.toFixed(3)}  yaw ${(p.yaw * DEG).toFixed(1)}°`;
const NAME_RE = /^[A-Za-z_][A-Za-z0-9_-]*$/;

function setPointsBusy(text) {
  pointsBusy = text;
  message($('points-busy'), text, true);
  message($('robot-busy'), text, true);
  document.querySelectorAll('#view-points [data-edit], #view-robot [data-edit]').forEach((button) => { button.disabled = !!text; });
}

// The base pose, in the Robot tab and at the top of the Teach tab (what Capture would save for a location).
async function refreshPose() {
  const { ok, data } = await api('GET', '/api/robot_pose');
  const text = ok
    ? fmt(data.pose) + (data.relative[0] ? `  ·  nearest ${data.relative[0].instance} (${data.relative[0].distance} m)` : '')
    : (data.error || 'no robot pose');
  ['robot-pose', 'teach-pose'].forEach((id) => { $(id).textContent = text; $(id).classList.toggle('muted', !ok); });
  state.robotPose = ok ? data : null;
  document.dispatchEvent(new Event('robot-pose'));  // the "at" hints follow (atHint)
  return ok ? data : null;
}
setInterval(() => { if ((pointsVisible || robotVisible) && !document.hidden) refreshPose(); }, 2000);

// Which placed object a teach uses, before it saves: its name and how far the robot is from it (the operator should
// stand at it), with the nearest one when that is another. pick() -> instance name, or '' for the nearest of `type`.
// One robot-pose listener for all of them, holding them weakly: a hint is updated while it is on the page (also when
// it is put there later or moved), and drops out once the page has let go of it.
const atHints = new Set();
document.addEventListener('robot-pose', () => atHints.forEach((ref) => {
  const node = ref.deref();
  if (!node) atHints.delete(ref); else if (node.isConnected) node._update();
}));
function atHint(pick, type) {
  const node = el('span', { class: 'muted small-text at-hint' });
  const update = () => {
    const relative = (state.robotPose?.relative || []).filter((r) => r.object === type);
    const nearest = relative[0];
    const chosen = pick() ? relative.find((r) => r.instance === pick()) : nearest;
    if (!chosen) { node.textContent = pick() ? `at ${pick()}` : ''; node.classList.remove('bad'); return; }
    const other = nearest && nearest.instance !== chosen.instance ? ` · nearest is ${nearest.instance} (${nearest.distance} m)` : '';
    node.textContent = `at ${chosen.instance}, ${chosen.distance} m from the robot${other}`;
    node.classList.toggle('bad', !!other);
  };
  node._update = update;
  update();
  atHints.add(new WeakRef(node));
  return node;
}

// Each entry's last Teach save (from this browser or the pad), shown under it until the page reloads: what was saved,
// where it was measured, and Undo (POST /api/teach/undo: the files as they were before that save; refused once they
// changed, so an older one says so). warnings: what the save left out (the server's notes), each on its own line and
// the note amber.
const savedNotes = {};
function noteSaved(key, text, undo, warnings = []) { savedNotes[key] = { text, undo: undo || null, warnings }; }
function savedNote(key) {
  const saved = savedNotes[key];
  if (!saved) return '';
  const warnings = saved.warnings || [];
  const box = el('div', { class: `msg saved-note${warnings.length ? ' warn' : ''}` }, el('span', { text: saved.text }),
    ...warnings.map((text) => el('div', { text: text.charAt(0).toUpperCase() + text.slice(1) + (/[.!?]$/.test(text) ? '' : '.') })));
  if (saved.undo) {
    const button = el('button', { class: 'btn small', text: 'Undo', title: `Put ${saved.undo.files.join(', ')} back as before this save`, 'data-edit': '' });
    button.addEventListener('click', async () => {
      button.disabled = true;
      const { ok, data } = await api('POST', '/api/teach/undo', { id: saved.undo.id });
      if (!ok) { message(box, (data.problems || [data.error || 'not undone']).join('\n'), true); return; }
      noteSaved(key, `Undone: ${data.label} (${data.files.join(', ')} as before).`, null);
      await Promise.all([loadPoints(), typeof loadArm === 'function' ? loadArm() : null]);
    });
    box.append(' ', button);
  }
  return box;
}

// Before a save overwrites a pose: two taps, the change shown between them (the delete pattern, longer: there is
// something to read). True on the first tap (the change shown, nothing saved), false on the second.
function confirmOverwrite(button, msg, label, text) {
  if (button._armed) { clearTimeout(button._armed); button._armed = null; button.classList.remove('confirm'); button.textContent = label; return false; }
  button.classList.add('confirm'); button.textContent = 'Tap again to overwrite';
  button._shown = text;
  message(msg, text, 'warn');
  button._armed = setTimeout(() => {
    button.classList.remove('confirm'); button.textContent = label; button._armed = null;
    if (msg.classList.contains('warn') && msg.textContent === text) message(msg, '');
  }, 8000);
  return true;
}
// The two taps of an overwrite whose change is asked again at each tap (the robot may move in between): true when
// this tap only shows it, the first time or because it is not the change shown at the first tap ("moved").
function overwriteAsked(button, msg, label, text) {
  if (button._armed && button._shown === text) { confirmOverwrite(button, msg, label, ''); return false; }
  const moved = button._armed && text;
  if (button._armed) confirmOverwrite(button, msg, label, '');  // disarm
  if (!text) return false;
  confirmOverwrite(button, msg, label, moved ? `Changed since the first tap. ${text}` : text);
  button._shown = text;
  return true;
}
// old -> new of a base pose ({x, y, yaw} m, rad) for the confirm; '' when nothing changes
function poseChange(old, now) {
  const moved = Math.hypot(now.x - old.x, now.y - old.y) * 1000;
  const turned = Math.abs(((now.yaw - old.yaw) * DEG + 540) % 360 - 180);
  if (moved < 0.5 && turned < 0.05) return '';
  return `x ${old.x.toFixed(3)} → ${now.x.toFixed(3)}, y ${old.y.toFixed(3)} → ${now.y.toFixed(3)}, yaw ${(old.yaw * DEG).toFixed(1)}° → `
    + `${(now.yaw * DEG).toFixed(1)}°: moves ${moved.toFixed(0)} mm, turns ${turned.toFixed(1)}°`;
}

// Teach sub-tabs: one list at a time; the choice is kept for this browser (a convenience only)
function selectTeachPane(pane) {
  document.querySelectorAll('#teach-panes button').forEach((b) => b.setAttribute('aria-selected', String(b.dataset.pane === pane)));
  document.querySelectorAll('.teach-pane').forEach((node) => show(node, node.dataset.pane === pane));
  try { localStorage.setItem('teach-pane', pane); } catch (_) { /* storage blocked: not kept */ }
}
document.querySelectorAll('#teach-panes button').forEach((b) => b.addEventListener('click', () => selectTeachPane(b.dataset.pane)));
let savedPane = null;
try { savedPane = localStorage.getItem('teach-pane'); } catch (_) { /* storage blocked */ }
selectTeachPane(document.querySelector(`#teach-panes button[data-pane="${savedPane}"]`) ? savedPane : 'locations');

async function loadPoints() {
  const { ok, data } = await api('GET', '/api/points/all');
  if (!ok) { $('location-list').replaceChildren(el('li', { class: 'error', text: data.error || (data.problems || []).join('\n') })); return; }
  state.pointsData = data;
  $('locations-file').textContent = data.files.locations;
  $('location-list').replaceChildren(...data.locations.map((loc) => pointItem({
    title: loc.name, pose: loc.pose, usedBy: loc.used_by, name: loc.name, address: ['location', loc.name],
    fromRobot: (robot) => robot.pose,
  })));
  $('stance-cards').replaceChildren(...data.objects.map((obj) => stanceCard(obj, data)));
  setPointsBusy(pointsBusy);
}

function stanceCard(obj, data) {
  const instances = obj.instances.map((i) => i.name);
  const fromInstance = (pick) => (robot) => robot.relative.find((r) => r.instance === pick())?.offset;
  const variantItem = (v, isNew = false) => {
    const picker = el('select', { 'aria-label': 'Measure at object' });
    fillSelect(picker, instances, instances[0]);
    const hint = atHint(() => picker.value, obj.object);
    picker.addEventListener('change', hint._update);
    return pointItem({
      title: v.variant, pose: v.offset, usedBy: v.used_by, name: v.variant, isNew, nameLabel: 'Variant',
      address: ['stance', 'objects', obj.object, v.variant], instancePicker: instances.length ? picker : null,
      fromRobot: fromInstance(() => picker.value), at: () => picker.value, atHintNode: instances.length ? hint : null,
    });
  };
  const overrideItem = (instance, o, isNew = false) => pointItem({
    title: `${instance} / ${o.variant}`, pose: o.offset, usedBy: o.used_by, name: o.variant, isNew, nameLabel: 'Variant',
    address: ['stance', 'instances', instance, o.variant], fromRobot: fromInstance(() => instance),
    at: () => instance, atHintNode: atHint(() => instance, obj.object),
    note: o.fallback ? `Without the override, ${instance} uses the ${obj.object} ${o.variant} stance.` : '',
  });
  const variants = el('ul', { class: 'points' }, obj.variants.map((v) => variantItem(v)));
  const overrides = el('ul', { class: 'points' }, obj.instances.flatMap((i) => i.overrides.map((o) => overrideItem(i.name, o))));
  const addVariant = el('button', { class: 'btn small', text: '+ Variant' });
  addVariant.addEventListener('click', () => variants.append(variantItem(
    { variant: '', offset: obj.variants[0]?.offset || { x: -1, y: 0, yaw: 0 }, used_by: [] }, true)));
  const overrideInstance = el('select', { 'aria-label': 'Object for a new override' });
  fillSelect(overrideInstance, instances, instances[0]);
  const addOverride = el('button', { class: 'btn small', text: '+ Override' });
  addOverride.addEventListener('click', () => {
    const base = obj.variants[0] || { variant: 'default', offset: { x: -1, y: 0, yaw: 0 } };
    overrides.append(overrideItem(overrideInstance.value, { variant: base.variant, offset: base.offset, used_by: [], fallback: !!obj.variants[0] }, true));
  });
  const placed = el('ul', { class: 'placed' }, obj.instances.map((i) => el('li', {
    text: `${i.name}  ${fmt(i.pose)}${Object.entries(i.stances).map(([n, p]) => `\n  stance ${n.split('/')[1]}: ${fmt(p)}`).join('')}`,
  })));
  placed.style.whiteSpace = 'pre-wrap';
  // one object, two views: where the robot stands, where the objects are (its points: the Points list, teach.js)
  const panes = {
    stances: el('div', { class: 'pane' },
      el('div', { class: 'row between' }, el('p', { class: 'muted small-text', text: `Robot ${data.robot}: base pose in the object's frame, so every ${obj.object} follows · ${data.files.stances}` }), addVariant),
      variants,
      el('div', { class: 'sub', text: 'Per-object overrides' }),
      overrides,
      el('div', { class: 'row gap' }, overrideInstance, addOverride)),
    placed: el('div', { class: 'pane' },
      el('p', { class: 'muted small-text', text: `Placed ${obj.object} objects (from the cell layout, read-only) and the stances that result.` }), placed),
  };
  const titles = { stances: 'Stances', placed: `Placed (${obj.instances.length})` };
  state.objectPane = state.objectPane || {};
  const switcher = el('div', { class: 'segmented', role: 'group', 'aria-label': `${obj.object} view` });
  const select = (key) => {
    state.objectPane[obj.object] = key;
    Object.entries(panes).forEach(([k, pane]) => show(pane, k === key));
    switcher.querySelectorAll('button').forEach((b) => b.setAttribute('aria-pressed', String(b.dataset.pane === key)));
  };
  switcher.replaceChildren(...Object.keys(panes).map((key) => {
    const button = el('button', { text: titles[key] });
    button.dataset.pane = key;
    button.addEventListener('click', () => select(key));
    return button;
  }));
  select(state.objectPane[obj.object] || 'stances');
  return el('div', { class: 'card' },
    el('div', { class: 'row between wrap' }, el('div', { class: 'label', text: obj.object }), switcher),
    ...Object.values(panes));
}

// One editable point. address = ['location', name] or ['stance', scope, owner, variant]; the last element is the name.
// Set from robot fills the form; saved untouched, the teach node saves where the robot stands then (its stale and clock
// jump checks: POST .../teach); a number edited after that is saved as typed (PUT). at(): the placed object a stance is
// measured at, atHintNode: where that is shown (atHint).
function pointItem({ title, pose, usedBy, name, address, isNew = false, nameLabel = 'Name', instancePicker = null, fromRobot, note = '',
  at = () => null, atHintNode = null }) {
  const head = el('button', { class: 'point-head', 'aria-expanded': String(isNew) },
    el('b', { text: title || 'new' }), el('span', { class: 'mono', text: fmt(pose) }),
    usedBy.length ? el('span', { class: 'used', text: `used by ${usedBy.join(', ')}` }) : '');
  const input = (label, value, attrs = {}) => {
    const field = el('input', { autocomplete: 'off', spellcheck: 'false', ...attrs });
    field.value = value;
    return [el('div', { class: 'field' }, el('label', { text: label }), field), field];
  };
  const [nameField, nameInput] = input(nameLabel, name);
  const [xField, xInput] = input('x (m)', String(pose.x), { type: 'number', step: STEP_M, inputmode: 'decimal' });
  const [yField, yInput] = input('y (m)', String(pose.y), { type: 'number', step: STEP_M, inputmode: 'decimal' });
  const [yawField, yawInput] = input('yaw (°)', String(+(pose.yaw * DEG).toFixed(2)), { type: 'number', step: STEP_DEG, inputmode: 'decimal' });
  const msg = el('div', { class: 'msg', role: 'status', hidden: '' });
  const robotButton = el('button', { class: 'btn small', text: 'Set from robot' });
  const driveButton = el('button', { class: 'btn small', text: 'Drive here', 'data-edit': '' });
  const saveButton = el('button', { class: 'btn primary', text: 'Save', 'data-edit': '' });
  const deleteButton = el('button', { class: 'btn danger', text: isNew ? 'Cancel' : 'Delete', 'data-edit': '' });
  const edit = el('div', { class: 'point-edit' }, nameField, el('div', { class: 'grid3' }, xField, yField, yawField),
    el('div', { class: 'row gap' }, instancePicker || '', robotButton, driveButton),
    atHintNode || '',
    note ? el('p', { class: 'muted small-text', text: note }) : '',
    msg, el('div', { class: 'row gap' }, deleteButton, saveButton));
  edit.hidden = !isNew;
  const item = el('li', { class: 'point' }, head, savedNote(address.join('/')), edit);
  head.addEventListener('click', () => { edit.hidden = !edit.hidden; head.setAttribute('aria-expanded', String(!edit.hidden)); });
  if (isNew) setTimeout(() => nameInput.focus(), 0);
  // Drive here goes to the saved point, so it waits for a save after edits
  let changed = isNew;
  const markChanged = () => { changed = true; driveButton.disabled = true; driveButton.title = 'Save first'; };
  driveButton.disabled = changed;
  [nameInput, xInput, yInput, yawInput].forEach((field) => field.addEventListener('input', markChanged));
  // filled from the robot and not edited since: Save asks the teach node (null: the numbers are saved as typed)
  let fromRobotRead = null;
  [xInput, yInput, yawInput].forEach((field) => field.addEventListener('input', () => {
    if (fromRobotRead) message(msg, 'Edited: Save stores the numbers as typed, not checked against the robot.', 'warn');
    fromRobotRead = null;
  }));
  instancePicker?.addEventListener('change', () => {
    if (fromRobotRead) message(msg, 'Another object: Set from robot again.', 'warn');
    fromRobotRead = null;
  });
  // navigate_to target: a location by name, a stance at a placed object (the picked one for a type-level stance)
  const driveTarget = () => {
    if (address[0] === 'location') return name;
    const [, scope, owner, variant] = address;
    const instance = scope === 'instances' ? owner : instancePicker?.value;
    return instance ? `stance:${instance}/${variant}` : null;
  };
  driveButton.addEventListener('click', async () => {
    const target = driveTarget();
    if (!target) { message(msg, 'Pick the object to drive to.', true); return; }
    message(msg, `Driving to ${target}…`);
    driveButton.disabled = true;
    const { ok, data } = await api('POST', '/api/points/drive', { target });
    driveButton.disabled = changed;
    if (!ok) { message(msg, (data.problems || [data.error || 'not driven']).join('\n'), true); return; }
    skillResult(msg, data);
  });

  robotButton.addEventListener('click', async () => {
    const robot = await refreshPose();
    const value = robot && fromRobot(robot);
    if (!value) { message(msg, 'No current robot pose.', true); return; }
    xInput.value = value.x; yInput.value = value.y; yawInput.value = +(value.yaw * DEG).toFixed(2);
    markChanged();
    fromRobotRead = { ...value };
    atHintNode?._update();
    message(msg, 'Filled in from the robot. Save has the teach node store where the robot stands then (it refuses a '
      + 'stale or jumped pose); edit a number to save it as typed instead.');
  });
  // one save at a time: a tap while it compares or saves does nothing (a double click would arm and save at once; the
  // line under it says what it is doing). aria-disabled, not disabled: the button keeps the keyboard focus
  saveButton.addEventListener('click', async () => {
    if (saveButton.getAttribute('aria-disabled') === 'true') return;
    saveButton.setAttribute('aria-disabled', 'true');
    try { await save(); } finally { saveButton.removeAttribute('aria-disabled'); }
  });
  async function save() {
    const newName = nameInput.value.trim();
    if (!NAME_RE.test(newName)) { message(msg, 'Names use letters, digits, "_" and "-", starting with a letter.', true); return; }
    const values = [xInput.value, yInput.value, yawInput.value].map(Number);
    if ([xInput.value, yInput.value, yawInput.value].some((v) => v.trim() === '') || values.some((v) => !Number.isFinite(v))) {
      message(msg, 'x, y and yaw must be numbers.', true); return;
    }
    const target = isNew ? [...address.slice(0, -1), newName] : address;
    const body = { x: values[0], y: values[1], yaw: values[2] / DEG };
    const where = at();
    const change = isNew ? '' : poseChange(pose, body);
    let confirm = change ? `Overwrites ${name}: ${change}` : '';
    if (fromRobotRead && !isNew) {  // the node saves the robot's pose at Save: compare that one, asked at each tap
      if (!saveButton._armed) message(msg, 'Comparing the robot now with the saved one…');
      const { ok, data: preview } = await api('POST', `/api/points/${address.map(encodeURIComponent).join('/')}/preview`, { at: where });
      confirm = !ok ? `Cannot compare with the saved ${name} (${(preview.problems || [preview.error || 'no answer']).join('; ')}). `
        + 'Tap again to overwrite anyway; the teach node still checks the pose when it saves.'
        : preview.exists && poseChange(preview.old, preview.new) ? `Overwrites ${name}: ${poseChange(preview.old, preview.new)}` : '';
      if (ok) {  // the form shows what will be saved
        xInput.value = preview.new.x; yInput.value = preview.new.y; yawInput.value = +(preview.new.yaw * DEG).toFixed(2);
      }
    }
    if (overwriteAsked(saveButton, msg, 'Save', confirm)) return;
    if (!isNew && newName !== name) body.rename = newName;
    if (isNew) body.create = true;
    const key = target.slice(0, -1).concat(body.rename || target[target.length - 1]).join('/');
    message(msg, 'Saving…');
    if (fromRobotRead) {  // through the teach node: it reads the pose now, with its checks
      const { ok, data } = await api('POST', `/api/points/${target.map(encodeURIComponent).join('/')}/teach`,
        { at: where, create: body.create, rename: body.rename });
      if (!ok && data.undo) {  // refused after a part was written (a rename): that part can be undone
        noteSaved(key, `Not finished: ${(data.problems || []).join('; ')}`, data.undo);
        await loadPoints();
        return;
      }
      if (!ok) { message(msg, (data.problems || [data.error || 'not taught']).join('\n'), true); return; }
      const distance = data.distance_m != null ? `, ${data.distance_m} m from the robot` : '';
      noteSaved(key, `Taught ${key.split('/').pop()}${data.at ? ` at ${data.at}${distance}` : ''}: ${data.message} → ${data.file}`, data.undo);
    } else {
      const { ok, data } = await api('PUT', `/api/points/${target.map(encodeURIComponent).join('/')}`, body);
      if (!ok) { message(msg, (data.problems || [data.error || 'not saved']).join('\n'), true); return; }
      noteSaved(key, `Saved ${key.split('/').pop()} as typed → ${data.file}`, data.undo);
    }
    await loadPoints();
  }
  let armed = null;
  deleteButton.addEventListener('click', async () => {
    if (isNew) { item.remove(); return; }
    if (!armed) {  // two taps: no browser dialogs
      deleteButton.classList.add('confirm'); deleteButton.textContent = 'Tap again to delete';
      armed = setTimeout(() => { deleteButton.classList.remove('confirm'); deleteButton.textContent = 'Delete'; armed = null; }, 3000);
      return;
    }
    clearTimeout(armed); armed = null;
    const { ok, data } = await api('DELETE', `/api/points/${address.map(encodeURIComponent).join('/')}`);
    if (!ok) { deleteButton.classList.remove('confirm'); deleteButton.textContent = 'Delete'; message(msg, (data.problems || [data.error || 'not deleted']).join('\n'), true); return; }
    await loadPoints();
  });
  return item;
}
$('btn-add-location').addEventListener('click', () => {
  $('location-list').append(pointItem({
    title: '', pose: { x: 0, y: 0, yaw: 0 }, usedBy: [], name: '', address: ['location', ''], isNew: true,
    fromRobot: (robot) => robot.pose,
  }));
});

connect();
loadMissions();
