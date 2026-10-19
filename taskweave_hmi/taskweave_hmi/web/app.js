'use strict';
// taskweave HMI: live status over server-sent events, mission control, mission editor (between runs).

const $ = (id) => document.getElementById(id);
const ON_FAILURE = ['', 'retry', 'replan', 'skip', 'abort'];
const POINT_KINDS = { base_target: ['locations', 'stances'], joint_pose: ['joint poses'], object_point: ['object points'],
  instance_point: ['placed object points'], instance_contact: ['placed object contacts'], instance: ['placed objects'],
  tool_pose: ['tool poses'], joint_pose_only: ['joint-only poses'], joint_group: ['joint groups'],
  held_pose: ['held poses'], held_object: ['placed objects'], via_target: ['via targets'] };
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
  children.flat().forEach((child) => node.append(child));
  return node;
}
function show(node, visible) { node.hidden = !visible; }
function message(node, text, bad = false) { node.textContent = text; node.classList.toggle('bad', bad); show(node, !!text); }
function argsText(args) { return Object.entries(args || {}).map(([k, v]) => `${k}=${typeof v === 'object' ? JSON.stringify(v) : v}`).join(' '); }

// ---------- tabs ----------
document.querySelectorAll('.tabs button').forEach((button) => button.addEventListener('click', () => {
  document.querySelectorAll('.tabs button').forEach((b) => b.setAttribute('aria-selected', String(b === button)));
  document.querySelectorAll('.view').forEach((view) => show(view, view.id === `view-${button.dataset.view}`));
  if (button.dataset.view === 'missions' && !state.editing) loadMissions();
  pointsVisible = button.dataset.view === 'points';
  robotVisible = button.dataset.view === 'robot';
  if (pointsVisible) { loadPoints(); loadArm(); refreshPose(); }  // teach.js: poses and points
  else if (teach.open) { teach.open = null; setTeachTarget(null); }  // the pad teaches only into an open position
  if (robotVisible) { loadRobot(); refreshPose(); }  // robot.js: live grippers and joints, settings
  if (button.dataset.view === 'record') loadRecordings();  // record.js
}));

// ---------- live state ----------
function connect() {
  const source = new EventSource('/api/events');
  source.onopen = () => $('conn').classList.add('on');
  source.onerror = () => { $('conn').classList.remove('on'); };
  source.onmessage = (event) => {
    const { type, data } = JSON.parse(event.data);
    if (type === 'state') { state.snapshot = data; }
    else if (!state.snapshot) return;
    else if (type === 'status') state.snapshot.status = data;
    else if (type === 'pad_mode') state.snapshot.pad_mode = data;
    else if (type === 'notice') { padNotice(data); if (data.taught) teachTaught(); return; }
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
  $('state').textContent = s.state;
  $('state').className = `chip ${s.state}`;
  $('goal').textContent = s.goal || 'No mission running';
  $('cycle').textContent = s.plan_id ? `${s.plan_id} · cycle ${s.cycle}` : '';
  const active = s.state === 'running' || s.state === 'paused';
  show($('current'), active && !!s.step);
  if (s.step) {
    $('step-meta').textContent = `step ${s.step_index + 1}/${s.step_count} · ${s.step.skill} · attempt ${s.attempt}`;
    $('step-instruction').textContent = s.step.instruction;
    $('progress-bar').style.width = `${Math.max(0, Math.min(1, s.step_progress)) * 100}%`;
    $('step-detail').textContent = `${s.step_status || ''} · ${s.step_elapsed_s.toFixed(1)} s`;
  }
  message($('last-error'), s.last_error && s.state !== 'succeeded' ? s.last_error : '', true);
  $('btn-pause').disabled = !active;
  $('btn-pause').textContent = s.state === 'paused' ? 'Resume' : 'Pause';
  $('btn-abort').disabled = !active;
  $('btn-start').disabled = active;
  const { plan, live } = shownPlan();
  const outcomes = live ? snap.outcomes : {};
  const startIndex = plan && state.startStep ? plan.steps.findIndex((step) => step.id === state.startStep) : -1;
  if (startIndex < 0) state.startStep = null;  // another mission, or the step is gone
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
        note ? el('div', { class: 'muted', text: note }) : ''));
    if (pickable) { item.title = 'Start the mission at this step'; item.addEventListener('click', () => pickStart(step)); }
    list.append(item);
  });

  if (typeof renderGraph === 'function') renderGraph();  // graph.js

  const log = $('log');
  log.replaceChildren(...snap.events.slice().reverse().map((e) => el('li', {
    text: `t=${Number(e.time).toFixed(1)}s ${e.plan_id} c${e.cycle} ${e.step_id} ${e.skill} ${e.event.replace('step_', '')}`
      + `${e.outcome ? ' ' + e.outcome : ''}${e.attempt > 1 ? ' attempt ' + e.attempt : ''}${e.message ? ' · ' + e.message : ''}`,
  })));
}

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
  if (ok) state.startStep = null;
  message($('start-msg'), ok ? `Started ${data.plan_id}${startStep ? ` at ${startStep}` : ''}`
    : (data.message || data.error || 'not started'), !ok);
});
$('mission-select').addEventListener('change', () => { state.startStep = null; renderRun(); });

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
    const item = el('li', { tabindex: '0' }, el('b', { text: m.name }),
      el('span', { class: 'muted', text: m.valid ? `${m.goal} · ${m.steps} steps${block(m) ? ' · block (include it)' : ''}` : '' }),
      m.valid ? '' : el('span', { class: 'bad', text: m.error }));
    item.addEventListener('click', () => openEditor(m.name));
    item.addEventListener('keydown', (e) => { if (e.key === 'Enter') openEditor(m.name); });
    return item;
  }));
}

// ---------- mission editor ----------
async function loadCatalog() {
  const [skills, points] = await Promise.all([api('GET', '/api/skills'), api('GET', '/api/points')]);
  if (skills.ok) state.skills = skills.data;
  if (points.ok) state.points = points.data;
}
function fillSelect(select, values, selected) {
  select.replaceChildren(...values.map((v) => el('option', { value: v, text: v === '' ? 'default' : v })));
  select.value = selected ?? '';
}
function markDirty() { state.dirty = true; show($('editor-dirty'), true); }

async function openEditor(name) {
  await loadCatalog();
  const isNew = name === null;
  const { ok, data } = await api('GET', isNew ? '/api/template' : `/api/missions/${encodeURIComponent(name)}`);
  if (!ok) { alertInline(data); return; }
  state.editing = { name, isNew, mission: data };
  state.dirty = false;
  show($('editor-dirty'), false);
  show($('mission-list-card'), false);
  show($('editor'), true);
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
  renderSteps(data.steps || []);
}
function alertInline(data) { message($('save-msg'), data.error || (data.problems || []).join('\n'), true); }

function renderSteps(steps) {
  const list = $('step-editor');
  list.replaceChildren();
  steps.forEach((step, index) => list.append(stepCard(step, index, steps)));
}
function readSteps() { return [...$('step-editor').children].map((card) => card._read()); }

function stepCard(step, index, steps) {
  const card = $('step-template').content.firstElementChild.cloneNode(true);
  const q = (key) => card.querySelector(`[data-k="${key}"]`);
  card.querySelector('.step-no').textContent = `Step ${index + 1}`;
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
  const renderArgs = (args) => {
    if (q('skill').value.startsWith(INCLUDE)) {  // another mission's steps: its params
      const params = state.missions.find((m) => INCLUDE + m.name === q('skill').value)?.params || {};
      argsBox.replaceChildren(...Object.entries(params).map(([key, p]) => argField(key,
        { type: 'string', 'x-taskweave-point': p.point, description: p.description }, true, args[key])));
      return;
    }
    const skill = state.skills.find((s) => s.name === q('skill').value);
    const props = skill?.args_schema?.properties || {};
    const required = skill?.args_schema?.required || [];
    argsBox.replaceChildren(...Object.entries(props).map(([key, spec]) => argField(key, spec, required.includes(key), args[key])));
  };
  renderArgs(step.include ? step.with || {} : step.args || {});
  q('skill').addEventListener('change', () => { renderArgs({}); markDirty(); });
  card.addEventListener('input', markDirty);
  card.addEventListener('change', markDirty);
  card.querySelector('[data-act="up"]').addEventListener('click', () => move(index, -1));
  card.querySelector('[data-act="down"]').addEventListener('click', () => move(index, 1));
  card.querySelector('[data-act="remove"]').addEventListener('click', () => { const s = readSteps(); s.splice(index, 1); renderSteps(s); markDirty(); });
  card._read = () => {
    const including = q('skill').value.startsWith(INCLUDE);
    const out = including ? { instruction: q('instruction').value.trim(), include: q('skill').value.slice(INCLUDE.length), with: {} }
      : { instruction: q('instruction').value.trim(), skill: q('skill').value, args: {} };
    argsBox.querySelectorAll('[data-arg]').forEach((input) => {
      const value = input._value();
      if (value !== undefined) out[including ? 'with' : 'args'][input.dataset.arg] = value;
    });
    if (q('id').value.trim()) out.id = q('id').value.trim();
    if (q('timeout_s').value !== '') out.timeout_s = Number(q('timeout_s').value);
    if (q('retries').value !== '') out.retries = Number(q('retries').value);
    if (q('on_failure').value) out.on_failure = q('on_failure').value;
    if (q('replan_from').value) out.replan_from = q('replan_from').value;
    return out;
  };
  return card;
}

function argField(key, spec, required, value) {
  const label = el('label', { text: `${key}${required ? '' : ' (optional)'}` });
  let input;
  const kinds = POINT_KINDS[spec['x-taskweave-point']];
  const isPlaceholder = typeof value === 'string' && value.includes('{');
  if ((kinds || spec.enum) && !isPlaceholder) {
    const options = spec.enum || kinds.flatMap((k) => state.points[k] || []);
    input = el('select');
    fillSelect(input, ['', ...options.map(String)], value === undefined ? '' : String(value));
    if (value !== undefined && !options.map(String).includes(String(value))) input.append(el('option', { value: String(value), text: `${value} (unknown)` }));
    input.value = value === undefined ? '' : String(value);
    input.options[0].textContent = required ? 'choose…' : 'none';
    input._value = () => (input.value === '' ? undefined : (spec.type === 'number' ? Number(input.value) : input.value));
  } else if (spec.type === 'number' || spec.type === 'integer') {
    input = el('input', { type: 'number', step: spec.type === 'integer' ? '1' : 'any' });
    input.value = value ?? '';
    input._value = () => (input.value === '' ? undefined : Number(input.value));
  } else if (spec.type === 'string' || isPlaceholder) {
    input = el('input', { spellcheck: 'false' });
    input.value = value ?? '';
    if (spec.description) input.placeholder = spec.description;
    input._value = () => (input.value === '' ? undefined : input.value);
  } else {
    input = el('textarea', { rows: '2', spellcheck: 'false' });
    input.value = value === undefined ? '' : JSON.stringify(value);
    input._value = () => { if (!input.value.trim()) return undefined; try { return JSON.parse(input.value); } catch (_) { return input.value; } };
  }
  input.dataset.arg = key;
  return el('div', { class: 'field' }, label, input);
}

function move(index, delta) {
  const steps = readSteps();
  const target = index + delta;
  if (target < 0 || target >= steps.length) return;
  [steps[index], steps[target]] = [steps[target], steps[index]];
  renderSteps(steps);
  markDirty();
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

function showProblems(problems) {
  show($('problems'), problems.length > 0);
  $('problems').textContent = problems.join('\n');
}

$('btn-add-step').addEventListener('click', () => {
  const steps = readSteps();
  steps.push({ instruction: '', skill: state.skills[0]?.name || 'wait', args: {} });
  renderSteps(steps);
  markDirty();
});
$('btn-validate').addEventListener('click', async () => {
  const name = $('m-name').value.trim() || 'new';
  const { ok, data } = await api('POST', `/api/missions/${encodeURIComponent(name)}/validate`, readMission());
  if (!ok) { alertInline(data); return; }
  showProblems(data.problems);
  message($('save-msg'), data.problems.length ? '' : 'Mission is valid.');
});
$('btn-save').addEventListener('click', async () => {
  const name = $('m-name').value.trim();
  if (!name) { showProblems(['Give the mission a name.']); return; }
  const old = state.editing.isNew ? null : state.editing.name;
  const rename = old && old !== name ? `?rename_from=${encodeURIComponent(old)}` : '';
  const { ok, data } = await api('PUT', `/api/missions/${encodeURIComponent(name)}${rename}`, readMission());
  if (!ok) { showProblems(data.problems || [data.error || 'not saved']); message($('save-msg'), ''); return; }
  showProblems([]);
  state.dirty = false; show($('editor-dirty'), false);
  state.editing = { ...state.editing, name, isNew: false };
  show($('btn-delete'), true);
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
  show($('editor'), false); show($('mission-list-card'), true);
  loadMissions();
});
$('btn-new').addEventListener('click', () => openEditor(null));
$('btn-back').addEventListener('click', () => {
  if (state.dirty && !$('btn-back').classList.contains('confirm')) {
    $('btn-back').classList.add('confirm'); $('btn-back').textContent = 'Discard changes?';
    setTimeout(() => { $('btn-back').classList.remove('confirm'); $('btn-back').textContent = '← Missions'; }, 3000);
    return;
  }
  $('btn-back').classList.remove('confirm'); $('btn-back').textContent = '← Missions';
  state.editing = null; state.dirty = false;
  show($('editor'), false); show($('mission-list-card'), true);
  loadMissions();
});
['m-name', 'm-goal', 'm-loop', 'm-max', 'm-stop', 'd-timeout', 'd-retries', 'd-fail', 'd-replans'].forEach((id) => $(id).addEventListener('input', markDirty));

// ---------- points ----------
// Locations (cell frame) and stance offsets (object frame) from config; yaw is shown in degrees, stored in radians.
let pointsVisible = false;
let robotVisible = false;
let pointsBusy = '';
const DEG = 180 / Math.PI;
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
  return ok ? data : null;
}
setInterval(() => { if ((pointsVisible || robotVisible) && !document.hidden) refreshPose(); }, 2000);

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
    return pointItem({
      title: v.variant, pose: v.offset, usedBy: v.used_by, name: v.variant, isNew, nameLabel: 'Variant',
      address: ['stance', 'objects', obj.object, v.variant], instancePicker: instances.length ? picker : null,
      fromRobot: fromInstance(() => picker.value),
    });
  };
  const overrideItem = (instance, o, isNew = false) => pointItem({
    title: `${instance} / ${o.variant}`, pose: o.offset, usedBy: o.used_by, name: o.variant, isNew, nameLabel: 'Variant',
    address: ['stance', 'instances', instance, o.variant], fromRobot: fromInstance(() => instance),
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
function pointItem({ title, pose, usedBy, name, address, isNew = false, nameLabel = 'Name', instancePicker = null, fromRobot, note = '' }) {
  const head = el('button', { class: 'point-head', 'aria-expanded': String(isNew) },
    el('b', { text: title || 'new' }), el('span', { class: 'mono', text: fmt(pose) }),
    usedBy.length ? el('span', { class: 'used', text: `used by ${usedBy.join(', ')}` }) : '');
  const input = (label, value, attrs = {}) => {
    const field = el('input', { autocomplete: 'off', spellcheck: 'false', ...attrs });
    field.value = value;
    return [el('div', { class: 'field' }, el('label', { text: label }), field), field];
  };
  const [nameField, nameInput] = input(nameLabel, name);
  const [xField, xInput] = input('x (m)', String(pose.x), { type: 'number', step: 'any', inputmode: 'decimal' });
  const [yField, yInput] = input('y (m)', String(pose.y), { type: 'number', step: 'any', inputmode: 'decimal' });
  const [yawField, yawInput] = input('yaw (°)', String(+(pose.yaw * DEG).toFixed(2)), { type: 'number', step: 'any', inputmode: 'decimal' });
  const msg = el('div', { class: 'msg', hidden: '' });
  const robotButton = el('button', { class: 'btn small', text: 'Set from robot' });
  const driveButton = el('button', { class: 'btn small', text: 'Drive here', 'data-edit': '' });
  const saveButton = el('button', { class: 'btn primary', text: 'Save', 'data-edit': '' });
  const deleteButton = el('button', { class: 'btn danger', text: isNew ? 'Cancel' : 'Delete', 'data-edit': '' });
  const edit = el('div', { class: 'point-edit' }, nameField, el('div', { class: 'grid3' }, xField, yField, yawField),
    el('div', { class: 'row gap' }, instancePicker || '', robotButton, driveButton),
    note ? el('p', { class: 'muted small-text', text: note }) : '',
    msg, el('div', { class: 'row gap' }, deleteButton, saveButton));
  edit.hidden = !isNew;
  const item = el('li', { class: 'point' }, head, edit);
  head.addEventListener('click', () => { edit.hidden = !edit.hidden; head.setAttribute('aria-expanded', String(!edit.hidden)); });
  if (isNew) setTimeout(() => nameInput.focus(), 0);
  // Drive here goes to the saved point, so it waits for a save after edits
  let changed = isNew;
  const markChanged = () => { changed = true; driveButton.disabled = true; driveButton.title = 'Save first'; };
  driveButton.disabled = changed;
  [nameInput, xInput, yInput, yawInput].forEach((field) => field.addEventListener('input', markChanged));
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
    message(msg, `${data.outcome}${data.message ? ': ' + data.message : ''}`, data.outcome !== 'succeeded');
  });

  robotButton.addEventListener('click', async () => {
    const robot = await refreshPose();
    const value = robot && fromRobot(robot);
    if (!value) { message(msg, 'No current robot pose.', true); return; }
    xInput.value = value.x; yInput.value = value.y; yawInput.value = +(value.yaw * DEG).toFixed(2);
    markChanged();
    message(msg, 'Filled in from the robot. Check, then save.');
  });
  saveButton.addEventListener('click', async () => {
    const newName = nameInput.value.trim();
    if (!NAME_RE.test(newName)) { message(msg, 'Names use letters, digits, "_" and "-", starting with a letter.', true); return; }
    const values = [xInput.value, yInput.value, yawInput.value].map(Number);
    if ([xInput.value, yInput.value, yawInput.value].some((v) => v.trim() === '') || values.some((v) => !Number.isFinite(v))) {
      message(msg, 'x, y and yaw must be numbers.', true); return;
    }
    const target = isNew ? [...address.slice(0, -1), newName] : address;
    const body = { x: values[0], y: values[1], yaw: values[2] / DEG };
    if (!isNew && newName !== name) body.rename = newName;
    if (isNew) body.create = true;
    const { ok, data } = await api('PUT', `/api/points/${target.map(encodeURIComponent).join('/')}`, body);
    if (!ok) { message(msg, (data.problems || [data.error || 'not saved']).join('\n'), true); return; }
    await loadPoints();
  });
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
