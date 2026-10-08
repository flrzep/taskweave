'use strict';
// taskweave HMI: live status over server-sent events, mission control, mission editor (between runs).

const $ = (id) => document.getElementById(id);
const ON_FAILURE = ['', 'retry', 'replan', 'skip', 'abort'];
const POINT_KINDS = { base_target: ['locations', 'stances'], joint_pose: ['joint poses'], object_point: ['object points'] };
const state = { snapshot: null, skills: [], points: {}, missions: [], editing: null, dirty: false };

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
    else if (type === 'plan') { if (state.snapshot.plan?.plan_id !== data.plan_id) state.snapshot.outcomes = {}; state.snapshot.plan = data; }
    else if (type === 'episode') {
      state.snapshot.outcomes[data.step_id] = data;
      state.snapshot.events.push(data);
      state.snapshot.events = state.snapshot.events.slice(-200);
    }
    renderRun();
  };
}

function renderRun() {
  const snap = state.snapshot;
  if (!snap) return;
  const s = snap.status;
  $('robot').textContent = `robot ${snap.robot}`;
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

  const plan = snap.plan;
  const list = $('plan');
  list.replaceChildren();
  if (!plan) { list.append(el('li', { class: 'muted', text: 'No plan yet' })); }
  else plan.steps.forEach((step, index) => {
    const event = snap.outcomes[step.id];
    let kind = 'pending', mark = String(index + 1), note = '';
    if (event?.event === 'step_started') { kind = 'running'; }
    if (event?.event === 'step_finished') { kind = event.outcome.toLowerCase(); mark = kind === 'succeeded' ? '✓' : '!'; note = event.message || ''; }
    if (s.step_index === index && active) kind = 'running';
    list.append(el('li', {},
      el('span', { class: `dot ${kind}`, text: mark }),
      el('div', { class: 'step-text' },
        el('div', { text: step.instruction }),
        el('div', { class: 'muted mono', text: `${step.skill} ${argsText(step.args)}` }),
        note ? el('div', { class: 'muted', text: note }) : '')));
  });

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
  const { ok, data } = await api('POST', '/api/start', { mission });
  message($('start-msg'), ok ? `Started ${data.plan_id}` : (data.message || data.error || 'not started'), !ok);
});

async function loadMissions() {
  const { ok, data } = await api('GET', '/api/missions');
  if (!ok) return;
  state.missions = data;
  const select = $('mission-select');
  const current = select.value;
  select.replaceChildren(...data.filter((m) => m.valid).map((m) => el('option', { value: m.name, text: `${m.name} — ${m.goal}` })));
  if (current) select.value = current;
  $('mission-list').replaceChildren(...data.map((m) => {
    const item = el('li', { tabindex: '0' }, el('b', { text: m.name }),
      el('span', { class: 'muted', text: m.valid ? `${m.goal} · ${m.steps} steps` : '' }),
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
  $('m-name').readOnly = !isNew;
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
  fillSelect(q('skill'), state.skills.map((s) => s.name).concat(state.skills.some((s) => s.name === step.skill) ? [] : [step.skill]), step.skill);
  q('id').value = step.id || '';
  q('timeout_s').value = step.timeout_s ?? '';
  q('retries').value = step.retries ?? '';
  fillSelect(q('on_failure'), ON_FAILURE, step.on_failure ?? '');
  const ids = steps.map((s, i) => s.id || `s${i + 1}`).filter((_, i) => i !== index);
  fillSelect(q('replan_from'), [''].concat(ids), step.replan_from ?? '');
  q('replan_from').options[0].textContent = 'this step';
  const argsBox = card.querySelector('.args');
  const renderArgs = (args) => {
    const skill = state.skills.find((s) => s.name === q('skill').value);
    const props = skill?.args_schema?.properties || {};
    const required = skill?.args_schema?.required || [];
    argsBox.replaceChildren(...Object.entries(props).map(([key, spec]) => argField(key, spec, required.includes(key), args[key])));
  };
  renderArgs(step.args || {});
  q('skill').addEventListener('change', () => { renderArgs({}); markDirty(); });
  card.addEventListener('input', markDirty);
  card.addEventListener('change', markDirty);
  card.querySelector('[data-act="up"]').addEventListener('click', () => move(index, -1));
  card.querySelector('[data-act="down"]').addEventListener('click', () => move(index, 1));
  card.querySelector('[data-act="remove"]').addEventListener('click', () => { const s = readSteps(); s.splice(index, 1); renderSteps(s); markDirty(); });
  card._read = () => {
    const out = { instruction: q('instruction').value.trim(), skill: q('skill').value, args: {} };
    argsBox.querySelectorAll('[data-arg]').forEach((input) => {
      const value = input._value();
      if (value !== undefined) out.args[input.dataset.arg] = value;
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
  const { ok, data } = await api('PUT', `/api/missions/${encodeURIComponent(name)}`, readMission());
  if (!ok) { showProblems(data.problems || [data.error || 'not saved']); message($('save-msg'), ''); return; }
  showProblems([]);
  state.dirty = false; show($('editor-dirty'), false);
  state.editing = { ...state.editing, name, isNew: false };
  $('m-name').readOnly = true;
  message($('save-msg'), `Saved ${name}. It is used at the next start.`);
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
['m-goal', 'm-loop', 'm-max', 'm-stop', 'd-timeout', 'd-retries', 'd-fail', 'd-replans'].forEach((id) => $(id).addEventListener('input', markDirty));

connect();
loadMissions();
