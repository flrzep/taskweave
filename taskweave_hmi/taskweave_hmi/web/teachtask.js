'use strict';
// taskweave HMI, Teach tab: teach by task (P7-2). The operator starts from the job, not the file: Teach -> what
// (where the robot stands, where a hand grasps, a robot pose, a spot in the cell) -> which placed object (a short list,
// nearest first, or a click on it in the 3D view; it stays highlighted) -> one guided screen: drive or jog with the
// pad (its mode shown), a live check (a stance: which of the object's grasp points the hands reach from here; a grasp:
// which hand is on the object), Capture, a free name, Save, then Preview (dry run) / Go here and Undo. The flow picks
// the list, file, frame and hands behind the scenes and saves through the same endpoints as the lists (the teach
// node's checks, the overwrite confirm, Undo); the pad's teach press (L1 + L3) saves a grasp or pose here too. The
// lists stay below for editing what exists.
// Uses helpers and state from app.js ($, el, api, message, show, state, DEG, NAME_RE, refreshPose, loadPoints,
// overwriteAsked, poseChange, skillResult, pointsBusy), teach.js (teach, loadArm, setTeachTarget, changeText,
// ROBOT_SCOPE, keyOf) and teach3d.js (teachViewer).

// the four jobs; icons drawn on the skill icons' 24 px grid (2 px strokes, currentColor)
const JOBS = {
  stance: { label: 'Stand', hint: 'where the robot stands at an object', object: true, prefix: 'stand', pad: 'drive',
    icon: '<rect x="8" y="2.5" width="8" height="6" rx="1"/><circle cx="12" cy="17" r="4.5"/><path d="M12 12.5v-2"/>' },
  grasp: { label: 'Grasp', hint: 'where a hand grasps an object', object: true, prefix: 'pick', pad: 'arm',
    icon: '<rect x="9" y="12" width="6" height="7" rx="1"/><path d="M7 21v-9.5a2 2 0 0 1 2-2h1M17 21v-9.5a2 2 0 0 0-2-2h-1M12 9.5V3"/>' },
  pose: { label: 'Pose', hint: 'a robot pose', object: false, prefix: 'pose', pad: 'arm',
    icon: '<circle cx="12" cy="4.5" r="2"/><path d="M12 7v7M12 9l-5 3M12 9l5-3.5M12 14l-3 7M12 14l3 7"/>' },
  spot: { label: 'Spot', hint: 'a spot in the cell', object: false, prefix: 'spot', pad: 'drive',
    icon: '<path d="M12 21s-6-5.6-6-11a6 6 0 0 1 12 0c0 5.4-6 11-6 11z"/><circle cx="12" cy="10" r="2.2"/>' },
};
const REACH_MS = 1500;    // the live check's period while the guided screen is open
const MOVED_M = 0.005;    // the robot moved this far since Capture (a hand, the base, a linear joint): capture again
const MOVED_DEG = 0.5;    // or turned this far (the base, a hand's approach or closing, a revolute joint)
const flow = { step: 'closed', job: null, at: null, name: '', hands: [], captured: null, saved: null, checked: null,
  timer: null, asking: false };

function jobIcon(job, size = 24) {
  const node = document.createElementNS('http://www.w3.org/2000/svg', 'svg');
  node.setAttribute('viewBox', '0 0 24 24');
  node.setAttribute('width', size); node.setAttribute('height', size);
  node.setAttribute('class', 'job-icon'); node.setAttribute('aria-hidden', 'true');
  node.innerHTML = JOBS[job].icon;
  return node;
}

// ---------- where things are ----------
const typeOf = (instance) => (state.robotPose?.relative || []).find((r) => r.instance === instance)?.object
  || Object.entries(teach.data?.placed || {}).find(([, names]) => names.includes(instance))?.[0] || '';
const distanceOf = (instance) => (state.robotPose?.relative || []).find((r) => r.instance === instance)?.distance;
// the names taken where this job saves (the free name and the overwrite confirm)
function takenNames() {
  const type = typeOf(flow.at);
  if (flow.job === 'stance') return (state.pointsData?.objects.find((o) => o.object === type)?.variants || []).map((v) => v.variant);
  if (flow.job === 'spot') return (state.pointsData?.locations || []).map((l) => l.name);
  const scope = flow.job === 'pose' ? ROBOT_SCOPE : type;
  return (teach.data?.positions || []).filter((p) => p.scope === scope).map((p) => p.name);
}
function freeName() {
  const taken = new Set(takenNames());
  let n = 1;
  while (taken.has(`${JOBS[flow.job].prefix}_${n}`)) n += 1;
  return `${JOBS[flow.job].prefix}_${n}`;
}
// the target a move goes to (Preview, Go here), and its address for the saves
function moveTarget() { return JOBS[flow.job].object ? `${flow.at}/${flow.name}` : flow.name; }

// ---------- the steps ----------
function openFlow(job = null) {
  Object.assign(flow, { step: job ? (JOBS[job].object ? 'object' : 'teach') : 'job', job, at: null, captured: null,
    saved: null, checked: null, allObjects: false });
  hidePick();  // teach3d.js: an object card from before acts on the lists, hidden now
  if (job && !JOBS[job].object) startTeach();
  renderFlow();
}
function closeFlow() {
  stopLive();
  if (flow.job === 'grasp' || flow.job === 'pose') setTeachTarget(null);
  flow.step = 'closed';
  teachViewer?.setHighlight(null);
  renderFlow();
  $('btn-teach').focus();
}
function chooseJob(job) {
  flow.job = job;
  if (JOBS[job].object) { flow.step = 'object'; renderFlow(); return; }
  flow.at = null;  // a spot or a pose: no object (also when Stand went on to "a spot in the cell")
  teachViewer?.setHighlight(null);
  startTeach();
}
function chooseObject(instance) {
  flow.at = instance;
  teachViewer?.setHighlight({ kind: 'placed', name: instance });
  teachViewer?.focus({ kind: 'placed', name: instance });
  startTeach();
}
function startTeach() {
  Object.assign(flow, { step: 'teach', captured: null, saved: null, checked: null, hands: [], handsSet: false });
  flow.name = freeName();
  if (flow.job === 'pose') flow.hands = [...(teach.data?.tools || [])];
  renderFlow();
  padTarget();
  startLive();
}
// a grasp or a pose: the pad's teach press saves it here (the name as it stands)
function padTarget() {
  if (flow.step !== 'teach' || flow.saved || !(flow.job === 'grasp' || flow.job === 'pose')) return;
  if (!NAME_RE.test(flow.name)) setTeachTarget(null);  // no name to teach into: the pad teaches nothing
  else if (flow.job === 'grasp') setTeachTarget({ scope: typeOf(flow.at), name: flow.name, at: flow.at, instance_override: false });
  else if (flow.job === 'pose') setTeachTarget({ scope: ROBOT_SCOPE, name: flow.name, at: null, form: poseForm() });
}
const poseForm = () => ((teach.data?.tools || []).length ? 'hands' : 'joints');
// The Teach tab shown or left (app.js): the guided screen's live check and pad target only while it is on screen, so a
// pad press elsewhere teaches nothing
function teachFlowShown(visible) {
  if (flow.step !== 'teach' || flow.saved) return;
  if (visible) { padTarget(); startLive(); return; }
  stopLive();
  if (flow.job === 'grasp' || flow.job === 'pose') setTeachTarget(null);
}
// Escape closes the flow (not while typing the name: it leaves the field)
document.addEventListener('keydown', (e) => {
  if (e.key !== 'Escape' || flow.step === 'closed' || $('view-points').hidden || e.target.closest?.('input, select')) return;
  closeFlow();
});

// The Teach tab's 3D view picked something (teach3d.js): on the object step a placed object is the choice, on the
// guided screen another placed object moves the job there. True: the flow took it (the lists do nothing).
function teachFlowPick(info) {
  if (flow.step === 'closed' || flow.step === 'job') return false;
  if (info.kind !== 'placed') return true;
  if (flow.step === 'object' || (flow.step === 'teach' && JOBS[flow.job].object && !flow.saved)) {
    if (flow.job === 'grasp' && !(teach.data?.object_types || []).includes(info.object)) return true;
    chooseObject(info.name);
  }
  return true;
}
// the pad taught a position (app.js 'notice'): the one this screen teaches is saved
function teachFlowTaught(data) {
  if (flow.step !== 'teach' || flow.saved || !data.taught) return;
  const scope = flow.job === 'pose' ? ROBOT_SCOPE : typeOf(flow.at);
  if (data.taught.scope !== scope || data.taught.name !== flow.name) return;
  flow.saved = { text: data.text, undo: data.undo || null, pad: true };
  stopLive();
  renderFlow();
}

// ---------- the live check ----------
function startLive() {
  stopLive();
  const tick = async () => {
    if (flow.step !== 'teach' || $('view-points').hidden || document.hidden) return;
    renderPad();
    if (pointsBusy) { renderLive(); return; }  // a mission runs: no IK or FK asked of its MoveIt
    if (flow.asking) return;
    flow.asking = true;
    try { await liveCheck(); } finally { flow.asking = false; }
  };
  flow.timer = setInterval(tick, REACH_MS);
  tick();
}
function stopLive() { clearInterval(flow.timer); flow.timer = null; }
async function liveCheck() {
  if (flow.saved) return;
  const [job, at] = [flow.job, flow.at];
  if (job === 'stance' || job === 'grasp') {
    // a stance: every grasp point of the type from here; a grasp: only where the hands are (no points solved)
    let answer;
    try {
      answer = await api('POST', '/api/robot/reach', { at, ...(job === 'grasp' ? { points: [] } : {}) });
    } catch (_) {
      answer = { ok: false, data: { error: 'no answer from the HMI server' } };
    }
    if (flow.job !== job || flow.at !== at || flow.step !== 'teach') return;  // another object meanwhile: dropped
    const { ok, data } = answer;
    flow.checked = ok ? data : { error: data.error || (data.problems || []).join('; ') || 'no answer' };
    if (ok && job === 'grasp' && !flow.handsSet) {  // the hands on the object, else the nearest one (kept from Capture)
      flow.hands = data.near || [];  // the server's choice (arm.near_hands): within 0.4 m, else the nearest
    }
  }
  const was = flow.captured;
  if (was && !was.pending && !was.error) {
    const moved = await movedSinceCapture(was);
    if (flow.captured === was) was.moved = moved;
  }
  renderLive();
}
// what Capture saw: a stance or spot the base pose (the preview's new pose), a grasp or pose the hands' targets
async function snapshotNow() {
  if (flow.job === 'stance' || flow.job === 'spot') {
    const { ok, data } = await api('POST', `/api/points/${addressOf().map(encodeURIComponent).join('/')}/preview`,
      { at: flow.at });
    return ok ? { base: data.new, exists: data.exists, old: data.old } : { error: data.error || (data.problems || []).join('; ') };
  }
  if (flow.job !== 'pose') {
    const { ok, data } = await api('GET', '/api/robot/tools');
    return ok ? { tools: data } : { error: data.error || 'no robot hands' };
  }
  // a pose: its joints (those a pose of its form stores), and the hands when it is stored as hands
  const joints = await api('GET', '/api/robot/joints');
  if (!joints.ok) return { error: joints.data.error || 'no joint states' };
  const watched = poseJoints();
  const all = joints.data.joints || joints.data;
  const shot = { tools: {}, joints: Object.fromEntries(Object.entries(all).filter(([name]) => !watched || watched.has(name))) };
  if (poseForm() !== 'hands') return shot;
  const { ok, data } = await api('GET', '/api/robot/tools');
  return ok ? { ...shot, tools: data } : { error: data.error || 'no robot hands' };
}
// the joints a new pose stores: those of the robot's poses of the same form (the server copies the first one's
// list); null when there is none yet (all are watched). Wheels and fingers stay out, so their jitter is no "move".
function poseJoints() {
  const hands = poseForm() === 'hands';
  const same = (teach.data?.positions || []).filter((p) => p.scope === ROBOT_SCOPE
    && Object.keys(p.hands || {}).length > 0 === hands);
  return same.length ? new Set(same.flatMap((p) => Object.keys(p.joints || {}))) : null;
}
// the angle between two direction vectors, degrees
function turnedDeg(a, b) {
  const dot = a.reduce((sum, v, i) => sum + v * b[i], 0) / (Math.hypot(...a) * Math.hypot(...b) || 1);
  return Math.acos(Math.max(-1, Math.min(1, dot))) * DEG;
}
// Has the robot left what Capture took (`was`)? The base for a stance or spot; for a grasp or pose each saved hand's
// TCP, approach and closing, and for a pose every joint. Unknown (no answer): not moved, the save's own check decides.
async function movedSinceCapture(was) {
  let now;
  try { now = await snapshotNow(); } catch (_) { return false; }
  if (now.error || !was || was.error || was.pending) return false;
  if (now.base) return Math.hypot(now.base.x - was.base.x, now.base.y - was.base.y) > MOVED_M
    || Math.abs(((now.base.yaw - was.base.yaw) * DEG + 540) % 360 - 180) > MOVED_DEG;
  const hand = flow.hands.some((tool) => {
    const [a, b] = [now.tools[tool], was.tools[tool]];
    if (!a || !b) return false;
    return Math.hypot(...a.tcp.map((v, i) => v - b.tcp[i])) > MOVED_M
      || turnedDeg(a.approach, b.approach) > MOVED_DEG || turnedDeg(a.closing, b.closing) > MOVED_DEG;
  });
  const joint = was.joints && Object.entries(was.joints).some(([name, value]) => now.joints?.[name] != null
    && Math.abs(now.joints[name] - value) > (isLinear(name) ? MOVED_M : MOVED_DEG / DEG));
  return hand || !!joint;
}

// ---------- saving ----------
function addressOf() {
  const name = flow.name || '_';
  return flow.job === 'stance' ? ['stance', 'objects', typeOf(flow.at), name] : ['location', name];
}
async function capture() {
  flow.captured = { pending: true };
  flow.handsSet = true;  // the hands shown now are the ones saved: no longer chosen by distance under the finger
  renderLive();
  try { flow.captured = await snapshotNow(); } catch (_) { flow.captured = { error: 'no answer from the HMI server' }; }
  renderFlow();
}
async function save(button, msg) {
  if (button._saving) return;
  button._saving = true;
  try { await saveNow(button, msg); } catch (_) {
    message(msg, 'No answer from the HMI server: nothing known about this save. Check the list before trying again.', true);
  } finally { button._saving = false; renderLive(); }
}
// still where Capture took it, at this tap? (the pad may have moved it within the live check's period)
async function stillCaptured(msg) {
  const was = flow.captured;
  const moved = await movedSinceCapture(was);
  if (flow.captured !== was || !canSave()) return false;
  if (!moved) return true;
  was.moved = true;
  renderLive();
  message(msg, 'The robot moved since Capture: capture again.', true);
  return false;
}
async function saveNow(button, msg) {
  if (!canSave() || !(await stillCaptured(msg))) return;
  const exists = takenNames().includes(flow.name);
  const arm = flow.job === 'grasp' || flow.job === 'pose';
  const scope = flow.job === 'pose' ? ROBOT_SCOPE : typeOf(flow.at);
  const armUrl = `/api/teach/arm/${encodeURIComponent(scope)}/${encodeURIComponent(flow.name)}`;
  let change = '';
  if (exists) {  // what it overwrites, asked at each tap (the robot may move in between); a second tap saves
    message(msg, 'Comparing with the saved one…');
    const { ok, data } = arm ? await api('POST', `${armUrl}/preview`, { at: flow.at, tools: flow.hands })
      : await api('POST', `/api/points/${addressOf().map(encodeURIComponent).join('/')}/preview`, { at: flow.at });
    change = !ok ? `Cannot compare with the saved ${flow.name}. Tap again to overwrite anyway.`
      : arm ? `Overwrites ${flow.name}: ${changeText(data)}`
        : `Overwrites ${flow.name}: ${poseChange(data.old, data.new) || 'no change'}`;
  }
  if (overwriteAsked(button, msg, 'Save', change)) return;
  if (exists && !(await stillCaptured(msg))) return;  // the second tap of an overwrite: checked again
  message(msg, 'Saving…');
  button.disabled = true;
  const { ok, data } = arm
    ? await api('POST', `${armUrl}/capture`, { at: flow.at, tools: flow.hands, form: flow.job === 'pose' ? poseForm() : null })
    : await api('POST', `/api/points/${addressOf().map(encodeURIComponent).join('/')}/teach`, { at: flow.at, create: !exists });
  if (!ok) { message(msg, (data.problems || [data.error || 'not saved']).join('\n'), true); return; }
  const where = data.at ? ` at ${data.at}${data.distance_m != null ? `, ${data.distance_m} m away` : ''}` : '';
  flow.saved = { text: `${flow.name}${where} → ${data.file}`, undo: data.undo || null, notes: data.notes || [] };
  setTeachTarget(null);
  stopLive();
  renderFlow();
  await Promise.all([loadPoints(), loadArm()]);
}
async function undoSave(button, msg) {
  button.disabled = true;
  const { ok, data } = await api('POST', '/api/teach/undo', { id: flow.saved.undo.id });
  button.disabled = false;
  if (!ok) { message(msg, (data.problems || [data.error || 'not undone']).join('\n'), true); return; }
  flow.saved = null;
  flow.captured = null;
  await Promise.all([loadPoints(), loadArm()]);
  startTeach();
}
async function tryIt(button, msg, go) {
  button.disabled = true;
  message(msg, go ? `Going to ${moveTarget()}…` : `Checking ${moveTarget()}…`);
  const target = moveTarget();
  let answer;
  if (!go) answer = await api('POST', '/api/skills/move/dry_run', { args: { to: target } });
  else if (flow.job === 'stance' || flow.job === 'spot') {
    answer = await api('POST', '/api/points/drive', { target: flow.job === 'stance' ? `stance:${target}` : target });
  } else {
    const scope = flow.job === 'pose' ? ROBOT_SCOPE : typeOf(flow.at);
    answer = await api('POST', `/api/teach/arm/${encodeURIComponent(scope)}/${encodeURIComponent(flow.name)}/test`, { at: flow.at });
  }
  button.disabled = !!pointsBusy;
  if (!answer.ok) { message(msg, (answer.data.problems || [answer.data.error || 'no answer']).join('\n'), true); return; }
  if (go) skillResult(msg, answer.data);
  else message(msg, answer.data.outcome === 'succeeded' ? `✓ ${answer.data.message || 'it solves from here'}`
    : `${answer.data.outcome}: ${answer.data.message}`, answer.data.outcome !== 'succeeded');
}

// ---------- drawing ----------
const button = (text, onClick, cls = 'btn', attrs = {}) => {
  const node = el('button', { type: 'button', class: cls, text, ...attrs });
  node.addEventListener('click', onClick);
  return node;
};
const edits = (node) => { node.dataset.edit = ''; node.disabled = !!pointsBusy; return node; };

function renderFlow() {
  const box = $('teach-flow');
  const open = flow.step !== 'closed';
  show($('teach-list'), !open);
  show($('btn-teach'), !open);
  show(box, open);
  if (!open) { box.replaceChildren(); return; }
  const back = button('←', () => {
    stopLive();
    if (flow.step === 'teach' && JOBS[flow.job].object && !flow.saved) flow.step = 'object';
    else flow.step = 'job';
    setTeachTarget(null);
    renderFlow();
  }, 'icon flow-back', { 'aria-label': 'Back' });
  const close = button('✕', closeFlow, 'icon', { 'aria-label': 'Close' });
  const head = el('div', { class: 'flow-head' }, flow.step === 'job' ? el('span') : back,
    el('div', { class: 'flow-crumbs' }, ...crumbs()), close);
  box.replaceChildren(head, ...(flow.step === 'job' ? jobStep() : flow.step === 'object' ? objectStep() : teachStep()));
  if (flow.step === 'teach' && !flow.saved) renderLive();  // the pad, the live check and the capture, filled now
  box.querySelector('[data-first]')?.focus({ preventScroll: true });
}
function crumbs() {
  if (flow.step === 'job') return [el('b', { class: 'flow-title', text: 'Teach' })];
  const parts = [el('span', { class: 'flow-chip' }, jobIcon(flow.job, 18), JOBS[flow.job].label)];
  if (flow.at) parts.push(el('span', { class: 'flow-chip at', id: 'flow-at' }, el('b', { text: flow.at }), el('span', { text: atText() })));
  return parts;
}
function atText() {
  const m = distanceOf(flow.at);
  return `${typeOf(flow.at)}${m != null ? ` · ${m} m` : ''}`;
}

function jobStep() {
  return [el('div', { class: 'flow-jobs' }, ...Object.entries(JOBS).map(([job, def], i) => {
    const tile = button('', () => chooseJob(job), 'flow-job', { 'aria-label': `${def.label}: ${def.hint}`, ...(i ? {} : { 'data-first': '' }) });
    tile.append(jobIcon(job, 44), el('b', { text: def.label }), el('span', { text: def.hint }));
    return tile;
  }))];
}

function objectStep() {
  // the placed objects, nearest first; a grasp only on types that have a points file
  const types = flow.job === 'grasp' ? new Set(teach.data?.object_types || []) : null;
  const relative = (state.robotPose?.relative || []).filter((r) => !types || types.has(r.object));
  const known = new Set(relative.map((r) => r.instance));
  const rest = Object.entries(teach.data?.placed || {}).flatMap(([type, names]) => (!types || types.has(type)
    ? names.filter((n) => !known.has(n)).map((n) => ({ instance: n, object: type })) : []));
  const all = [...relative, ...rest];
  const many = all.length > 6 && !flow.allObjects;
  const list = el('div', { class: 'flow-objects' }, ...(many ? all.slice(0, 6) : all).map((r, i) => {
    const item = button('', () => chooseObject(r.instance), 'flow-object', { ...(i ? {} : { 'data-first': '' }),
      'aria-label': `${r.instance}, ${r.object}${r.distance != null ? `, ${r.distance} m away` : ''}` });
    item.append(el('b', { text: r.instance }), el('span', { text: r.object }),
      el('span', { class: 'flow-m', text: r.distance != null ? `${r.distance} m` : '' }));
    return item;
  }));
  const nodes = [el('div', { class: 'flow-ask' }, el('span', { class: 'flow-3d', 'aria-hidden': 'true' }), 'Pick the object here or in 3D'), list];
  if (many) nodes.push(button(`All ${all.length}`, () => { flow.allObjects = true; renderFlow(); }, 'btn small'));
  if (flow.job === 'stance') nodes.push(button('No object: a spot in the cell', () => chooseJob('spot'), 'btn small flow-alt'));
  if (!all.length) nodes.push(el('p', { class: 'muted', text: 'No placed objects.' }));
  return nodes;
}

function teachStep() {
  const msg = el('div', { class: 'msg', role: 'status', hidden: '' });
  if (flow.saved) return savedStep(msg);
  const name = el('input', { id: 'flow-name', autocomplete: 'off', spellcheck: 'false', 'aria-label': 'Name' });
  name.value = flow.name;
  const exists = el('span', { class: 'flow-exists', id: 'flow-exists' });
  const mark = () => {
    const taken = takenNames().includes(flow.name);
    exists.textContent = !NAME_RE.test(flow.name) ? 'letters, digits, _ and -' : taken ? 'exists: Save overwrites' : '';
    exists.classList.toggle('bad', !NAME_RE.test(flow.name));
  };
  name.addEventListener('input', () => { flow.name = name.value.trim(); mark(); padTarget(); renderLive(); });
  mark();
  const captureButton = edits(button(flow.captured && !flow.captured.error ? 'Capture again' : 'Capture', capture,
    `btn flow-capture${flow.captured && !flow.captured.error ? '' : ' primary'}`, { 'data-first': '' }));
  // not data-edit: setPointsBusy would turn it on without a capture; canSave holds the busy state
  const saveButton = button('Save', () => save(saveButton, msg), 'btn primary flow-save');
  saveButton.disabled = !canSave();
  return [
    el('div', { class: 'flow-pad', id: 'flow-pad' }),
    el('div', { class: 'flow-live', id: 'flow-live' }),
    el('div', { class: 'flow-name' }, el('label', { for: 'flow-name', text: 'Name' }), name, exists),
    el('div', { class: 'flow-captured', id: 'flow-captured', role: 'status' }),
    el('div', { class: 'flow-actions' }, captureButton, saveButton),
    msg,
  ];
}

function savedStep(msg) {
  const saved = flow.saved;
  const nodes = [el('div', { class: 'flow-done' }, el('span', { class: 'flow-tick', 'aria-hidden': 'true', text: '✓' }),
    el('div', {}, el('b', { text: saved.pad ? 'Taught by the pad' : 'Saved' }), el('span', { text: saved.text }),
      ...(saved.notes || []).map((note) => el('span', { class: 'warn', text: note }))))];
  const row = el('div', { class: 'flow-actions' },
    edits(button('Preview', function () { tryIt(this, msg, false); }, 'btn', { 'data-first': '' })),
    edits(button('Go here', function () { tryIt(this, msg, true); })));
  if (saved.undo) row.append(edits(button('Undo', function () { undoSave(this, msg); }, 'btn')));
  nodes.push(row, msg, el('div', { class: 'flow-actions' },
    button('Teach another', () => { const at = flow.at; openFlow(flow.job); if (at) chooseObject(at); }, 'btn'),
    button('Done', closeFlow, 'btn primary')));
  return nodes;
}

// the pad strip: its mode now, amber when this job wants the other (a stance or spot drives, a grasp or pose jogs)
function renderPad() {
  const node = $('flow-pad');
  if (!node) return;
  const pad = state.snapshot?.pad_mode || 'drive';
  const wants = JOBS[flow.job].pad;
  const right = wants === 'drive' ? pad === 'drive' : pad !== 'drive';
  node.className = `flow-pad${right ? '' : ' off'}`;
  node.replaceChildren(el('span', { class: 'flow-pad-icon', 'aria-hidden': 'true' }),
    el('b', { text: pad === 'drive' ? 'Pad drives' : `Pad jogs the ${pad} arm` }),
    right ? '' : el('span', { text: wants === 'drive' ? 'switch to drive' : 'switch to an arm' }));
}

function renderLive() {
  renderPad();
  const at = $('flow-at');
  if (at) at.lastChild.textContent = atText();
  const live = $('flow-live');
  if (live) live.replaceChildren(...liveNodes());
  const captured = $('flow-captured');
  if (captured) captured.replaceChildren(...capturedNodes());
  const save = document.querySelector('#teach-flow .flow-save');
  if (save) save.disabled = !canSave() || !!save._saving;
}
// Save needs a capture the robot has not moved away from, a name, and for a grasp (or a pose stored as hands) a hand;
// nothing while a mission runs
const needsHand = () => flow.job === 'grasp' || (flow.job === 'pose' && poseForm() === 'hands');
const canSave = () => !pointsBusy && !!flow.captured && !flow.captured.pending && !flow.captured.error
  && !flow.captured.moved && NAME_RE.test(flow.name) && !(needsHand() && !flow.hands.length);
function liveNodes() {
  if (pointsBusy) return [el('div', { class: 'flow-reach wait' }, 'Paused while a mission runs')];
  const check = flow.checked;
  if (flow.job === 'spot') {
    const p = state.robotPose?.pose;
    return [el('div', { class: 'flow-big' }, p ? `x ${p.x.toFixed(2)}  y ${p.y.toFixed(2)}  ${(p.yaw * DEG).toFixed(0)}°` : '–')];
  }
  if (flow.job === 'pose' || flow.job === 'grasp') {
    // the hands this saves: on/off; a grasp shows how far each is from the object
    const tools = teach.data?.tools || [];
    return [el('div', { class: 'flow-hands' }, ...tools.map((tool) => {
      const on = flow.hands.includes(tool);
      const m = check?.hands?.[tool];
      const hand = button('', () => {
        flow.handsSet = true;
        flow.hands = on ? flow.hands.filter((t) => t !== tool) : [...flow.hands, tool];
        renderLive();
      }, `flow-hand${on ? ' on' : ''}`, { 'aria-pressed': String(on), 'aria-label': `${tool} hand${m != null ? `, ${m} m from the object` : ''}` });
      hand.append(iconElement('hand'), el('b', { text: tool }), el('span', { text: m != null ? `${m} m` : on ? 'saved' : 'not saved' }));
      return hand;
    })), check?.error ? el('p', { class: 'muted small-text', text: check.error }) : ''];
  }
  // a stance: per grasp point of the type, which hands reach it from here
  if (!check) return [el('div', { class: 'flow-reach wait' }, 'Checking reach…')];
  if (check.error) return [el('div', { class: 'flow-reach off' }, el('b', { text: 'No reach check' }), el('span', { text: check.error }))];
  const entries = Object.entries(check.points || {});
  if (!entries.length) return [el('div', { class: 'flow-reach' }, el('b', { text: 'No grasp points yet' }))];
  const reached = (tools) => !tools.error && Object.values(tools).length > 0 && Object.values(tools).every((t) => t.ok);
  const good = entries.filter(([, tools]) => reached(tools)).length;
  return [el('div', { class: `flow-reach ${good === entries.length ? 'ok' : good ? 'some' : 'off'}` },
    el('b', { text: `Reaches ${good} of ${entries.length}` }),
    el('div', { class: 'flow-points' }, ...entries.map(([point, tools]) => el('span', { class: 'flow-point',
      title: tools.error || Object.entries(tools).map(([t, e]) => `${t}: ${e.ok ? 'reaches' : e.message}`).join('\n') },
    el('b', { text: point }), tools.error ? el('i', { class: 'bad', role: 'img', text: '!', 'aria-label': tools.error })
      : '', ...Object.entries(tools.error ? {} : tools).map(([tool, e]) => el('i', { class: e.ok ? 'ok' : 'bad', role: 'img',
      text: `${tool[0].toUpperCase()}${e.ok ? '✓' : '✕'}`, 'aria-label': `${tool} ${e.ok ? 'reaches' : 'does not reach'}` }))))))];
}
function capturedNodes() {
  const c = flow.captured;
  if (!c) return [el('span', { class: 'muted', text: JOBS[flow.job].pad === 'drive' ? 'Drive there, then Capture' : 'Put the hand there, then Capture or L1 + L3' })];
  if (c.pending) return [el('span', { class: 'muted', text: 'Capturing…' })];
  if (c.error) return [el('span', { class: 'bad', text: c.error })];
  const text = c.base ? `x ${c.base.x.toFixed(3)}  y ${c.base.y.toFixed(3)}  ${(c.base.yaw * DEG).toFixed(1)}°`
    : flow.hands.map((t) => `${t} ${(c.tools[t]?.tcp || []).map((v) => v.toFixed(3)).join(' ')}`).join(' · ');
  return [el('span', { class: `flow-shot${c.moved ? ' moved' : ''}` }, el('b', { text: c.moved ? 'Moved since: capture again' : 'Captured' }),
    el('span', { class: 'mono', text }))];
}

$('btn-teach').addEventListener('click', () => openFlow());
