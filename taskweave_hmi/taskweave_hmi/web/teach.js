'use strict';
// taskweave HMI, Teach tab: arm positions (robot poses and points on objects, one list, one editor) and the live
// joints. A position that is open receives the pad's teach press (L1 + L3); Capture does the same from here.
// Uses helpers and state from app.js ($, el, api, message, show, state, DEG, NAME_RE, fillSelect) and robot.js
// (jointGroup).

const ROBOT_SCOPE = 'robot';
const teach = { data: null, open: null, joints: null, jointInputs: {} };

const keyOf = (p) => `${p.scope}/${p.name}`;
const isLinear = (joint) => (teach.data?.linear_joints || []).includes(joint);
const showJoint = (joint, value) => (isLinear(joint) ? +value.toFixed(4) : +(value * DEG).toFixed(2));
const unshowJoint = (joint, value) => (isLinear(joint) ? value : value / DEG);
const jointUnit = (joint) => (isLinear(joint) ? 'm' : '°');
const failText = (data, fallback) => (data.problems || [data.error || fallback]).join('\n');

async function loadArm() {
  const { ok, data } = await api('GET', '/api/teach/arm');
  if (!ok) { $('arm-list').replaceChildren(el('li', { class: 'error', text: failText(data, 'arm positions unavailable') })); return; }
  teach.data = data;
  renderArm();
  loadJoints(true);
}

// Where the pad's teach press goes: the open position (or none).
function setTeachTarget(target) {
  api('PUT', '/api/teach/target', target || {});
  const text = target ? `pad L1 + L3 teaches ${target.scope === ROBOT_SCOPE ? '' : `${target.scope} `}${target.name}` : '';
  message($('arm-target'), text);
}

function renderArm() {
  const data = teach.data;
  const groups = [[ROBOT_SCOPE, 'Relative to the robot', 'Poses in the robot\'s own frame: the same wherever it stands.'],
    ...data.object_types.map((type) => [type, `On ${type}`, `Points in the ${type}'s frame: every ${type} follows. Taught and tested at the placed one chosen here.`])];
  const nodes = [];
  groups.forEach(([scope, title, note]) => {
    const positions = data.positions.filter((p) => p.scope === scope);
    if (scope !== ROBOT_SCOPE && !positions.length && !(data.placed[scope] || []).length) return;
    const head = el('div', { class: 'row between wrap' }, el('div', { class: 'sub', text: title }));
    if (scope !== ROBOT_SCOPE) head.append(atPicker(scope));
    nodes.push(el('li', { class: 'arm-group' }, head, el('p', { class: 'muted small-text', text: note }),
      el('ul', { class: 'points' }, positions.map((p) => positionItem(p)))));
  });
  $('arm-list').replaceChildren(...nodes);
  if (typeof renderHeld === 'function') renderHeld();  // held.js
  if (teach.open && !data.positions.some((p) => keyOf(p) === teach.open)) { teach.open = null; setTeachTarget(null); }
}

// teach-at choice per object type: '' = the placed one nearest the robot
teach.at = {};
function atPicker(scope) {
  const select = el('select', { 'aria-label': `Teach ${scope} at` });
  select.replaceChildren(el('option', { value: '', text: 'nearest placed' }),
    ...(teach.data.placed[scope] || []).map((name) => el('option', { value: name, text: name })));
  select.value = teach.at[scope] || '';
  select.addEventListener('change', () => {
    teach.at[scope] = select.value;
    if (teach.open?.startsWith(`${scope}/`)) setTeachTarget({ scope, name: teach.open.slice(scope.length + 1), at: select.value || null });
  });
  return el('label', { class: 'row gap-s muted small-text' }, 'at', select);
}

function summary(p) {
  const tools = Object.keys(p.hands);
  const parts = [];
  if (tools.length) parts.push(`${tools.join(' + ')} hand${tools.length > 1 ? 's' : ''}`);
  const joints = Object.keys(p.joints);
  const lifts = joints.filter(isLinear);
  if (lifts.length) parts.push(lifts.map((j) => `lift ${p.joints[j].toFixed(2)} m`).join(', '));
  if (!tools.length) parts.push(`${joints.length} joints`);
  if (p.elbow) parts.push(`elbow ${p.elbow}`);
  if (p.approach_m != null) parts.push(`approach ${p.approach_m} m`);
  Object.entries(p.grip || {}).forEach(([tool, value]) => { if (value != null) parts.push(`${tool} grip ${(+value).toFixed(2)}`); });
  return parts.join(' · ');
}

function numberInput(label, value, attrs = {}) {
  const input = el('input', { type: 'number', step: 'any', inputmode: 'decimal', 'aria-label': label, ...attrs });
  input.value = value;
  return [el('div', { class: 'field' }, el('label', { text: label }), input), input];
}

// One arm position: capture / test, numbers, advanced settings, rename, delete.
function positionItem(p) {
  const data = teach.data;
  const robotScope = p.scope === ROBOT_SCOPE;
  const head = el('button', { class: 'point-head', 'aria-expanded': 'false' },
    el('b', { text: p.name }), el('span', { class: 'mono', text: summary(p) }),
    p.used_by.length ? el('span', { class: 'used', text: `used by ${p.used_by.join(', ')}` }) : '',
    p.problems.length ? el('span', { class: 'bad', text: `outside this robot's limits: ${p.problems.join('; ')}` }) : '');
  const msg = el('div', { class: 'msg', hidden: '' });
  const busy = (button, text) => { button.disabled = true; message(msg, text); };
  const at = () => (robotScope ? null : teach.at[p.scope] || null);

  const capture = el('button', { class: 'btn primary', text: 'Capture from robot', 'data-edit': '' });
  const test = el('button', { class: 'btn', text: robotScope ? 'Move here' : 'Go here', 'data-edit': '' });
  capture.addEventListener('click', async () => {
    busy(capture, 'Saving the robot\'s current arm…');
    const { ok, data: result } = await api('POST', `/api/teach/arm/${encodeURIComponent(p.scope)}/${encodeURIComponent(p.name)}/capture`, { at: at() });
    capture.disabled = false;
    if (!ok) { message(msg, failText(result, 'not captured'), true); return; }
    await loadArm();
    padNotice({ text: `taught ${p.name} -> ${result.file}`, ok: true });
  });
  test.addEventListener('click', async () => {
    busy(test, robotScope ? `Moving to ${p.name}…` : `Going to ${p.name}${at() ? ` at ${at()}` : ' at the nearest placed one'}…`);
    const { ok, data: result } = await api('POST', `/api/teach/arm/${encodeURIComponent(p.scope)}/${encodeURIComponent(p.name)}/test`, { at: at() });
    test.disabled = changed;
    if (!ok) { message(msg, failText(result, 'not moved'), true); return; }
    message(msg, `${result.outcome}${result.message ? ': ' + result.message : ''}`, result.outcome !== 'succeeded');
  });

  // numbers: each hand as x y z (m) and roll pitch yaw (°), then joints (lift first)
  const hands = {};
  const handNodes = Object.entries(p.hands).map(([tool, hand]) => {
    const xyz = ['x', 'y', 'z'].map((a, i) => numberInput(`${a} (m)`, +hand.xyz[i].toFixed(4)));
    const rpy = ['roll', 'pitch', 'yaw'].map((a, i) => numberInput(`${a} (°)`, +(hand.rpy[i] * DEG).toFixed(2)));
    hands[tool] = { xyz: xyz.map(([, i]) => i), rpy: rpy.map(([, i]) => i) };
    return el('div', { class: 'joint-group' }, el('label', { text: `${tool} hand` }),
      el('div', { class: 'grid3' }, ...xyz.map(([f]) => f)), el('div', { class: 'grid3' }, ...rpy.map(([f]) => f)));
  });
  const jointInputs = {};
  const jointNames = Object.keys(p.joints).sort((a, b) => isLinear(b) - isLinear(a));
  const jointNodes = jointNames.length ? [el('div', { class: 'joint-group' }, el('label', { text: 'Joints' }),
    el('div', { class: 'grid3' }, ...jointNames.map((joint) => {
      const [field, input] = numberInput(`${joint} ${jointUnit(joint)}`, showJoint(joint, p.joints[joint]));
      jointInputs[joint] = input;
      return field;
    })))] : [];
  const frame = robotScope ? 'the robot\'s base frame' : `the ${p.scope}'s frame`;
  const numbers = el('details', {}, el('summary', { text: 'Numbers' }),
    el('p', { class: 'muted small-text', text: `Hands: tool centre point in ${frame}; z points along the approach, y across the fingers.` }),
    ...handNodes, ...jointNodes);

  // advanced: elbow side (hands), straight approach (points on objects)
  const elbow = el('select', { 'aria-label': 'Elbow side' });
  elbow.replaceChildren(el('option', { value: '', text: 'robot default (Robot tab)' }), ...data.elbow_sides.map((s) => el('option', { value: s, text: s })));
  elbow.value = p.elbow || '';
  const [approachField, approach] = numberInput('Straight approach (m)', p.approach_m ?? '', { min: '0', max: '1', step: '0.01', placeholder: `default ${data.approach_default}` });
  // grip (points on objects): the gripper joint each hand closes to here (rad); empty = close fully
  const grips = {};
  const gripFields = robotScope ? [] : Object.keys(p.hands).map((tool) => {
    const [field, input] = numberInput(`${tool} grip (rad)`, p.grip?.[tool] ?? '', { step: '0.01', placeholder: 'close fully' });
    grips[tool] = input;
    return field;
  });
  const advancedFields = [];
  if (Object.keys(p.hands).length) advancedFields.push(el('div', { class: 'field' }, el('label', { text: 'Elbow side' }), elbow));
  if (!robotScope) advancedFields.push(approachField, ...gripFields);
  const advanced = advancedFields.length
    ? el('details', {}, el('summary', { text: 'Advanced' }), el('div', { class: 'grid2' }, ...advancedFields)) : '';

  const nameInput = el('input', { autocomplete: 'off', spellcheck: 'false' });
  nameInput.value = p.name;
  const save = el('button', { class: 'btn primary', text: 'Save', 'data-edit': '' });
  const remove = el('button', { class: 'btn danger', text: 'Delete', 'data-edit': '' });
  const edit = el('div', { class: 'point-edit' },
    el('div', { class: 'row gap' }, capture, test),
    el('div', { class: 'field' }, el('label', { text: 'Name' }), nameInput),
    numbers, advanced, msg, el('div', { class: 'row gap' }, remove, save));
  edit.hidden = true;
  let changed = false;  // Go here / Move here use the saved position
  edit.addEventListener('input', () => { changed = true; test.disabled = true; test.title = 'Save first'; });

  save.addEventListener('click', async () => {
    const name = nameInput.value.trim();
    if (!NAME_RE.test(name)) { message(msg, 'Names use letters, digits, "_" and "-", starting with a letter.', true); return; }
    const body = { hands: {}, elbow: elbow.value || null };
    for (const [tool, inputs] of Object.entries(hands)) {
      const values = [...inputs.xyz, ...inputs.rpy].map((i) => Number(i.value));
      if ([...inputs.xyz, ...inputs.rpy].some((i) => i.value.trim() === '') || values.some((v) => !Number.isFinite(v))) {
        message(msg, `${tool} hand: six numbers.`, true); return;
      }
      body.hands[tool] = { xyz: values.slice(0, 3), rpy: values.slice(3).map((v) => v / DEG) };
    }
    if (robotScope) {
      body.joints = {};
      for (const [joint, input] of Object.entries(jointInputs)) {
        const value = Number(input.value);
        if (input.value.trim() === '' || !Number.isFinite(value)) { message(msg, `${joint}: enter a number.`, true); return; }
        body.joints[joint] = unshowJoint(joint, value);
      }
    } else {
      body.approach_m = approach.value === '' ? null : Number(approach.value);
      body.grip = {};
      for (const [tool, input] of Object.entries(grips)) {
        const value = Number(input.value);
        if (input.value.trim() !== '' && !Number.isFinite(value)) { message(msg, `${tool} grip: a number (rad), or empty to close fully.`, true); return; }
        body.grip[tool] = input.value.trim() === '' ? null : value;
      }
    }
    if (name !== p.name) body.rename = name;
    const { ok, data: result } = await api('PUT', `/api/teach/arm/${encodeURIComponent(p.scope)}/${encodeURIComponent(p.name)}`, body);
    if (!ok) { message(msg, failText(result, 'not saved'), true); return; }
    if (name !== p.name) teach.open = `${p.scope}/${name}`;
    await loadArm();
  });
  let armed = null;
  remove.addEventListener('click', async () => {
    if (!armed) {  // two taps: no browser dialogs
      remove.classList.add('confirm'); remove.textContent = 'Tap again to delete';
      armed = setTimeout(() => { remove.classList.remove('confirm'); remove.textContent = 'Delete'; armed = null; }, 3000);
      return;
    }
    clearTimeout(armed); armed = null;
    const { ok, data: result } = await api('DELETE', `/api/teach/arm/${encodeURIComponent(p.scope)}/${encodeURIComponent(p.name)}`);
    if (!ok) { remove.classList.remove('confirm'); remove.textContent = 'Delete'; message(msg, failText(result, 'not deleted'), true); return; }
    await loadArm();
  });

  const item = el('li', { class: 'point' }, head, edit);
  const open = (opened) => {
    edit.hidden = !opened;
    head.setAttribute('aria-expanded', String(opened));
    item.classList.toggle('pad-target', opened);
  };
  head.addEventListener('click', () => {
    const opening = edit.hidden;
    document.querySelectorAll('#arm-list li.point.pad-target').forEach((other) => { if (other !== item) other._open(false); });
    open(opening);
    teach.open = opening ? keyOf(p) : null;
    setTeachTarget(opening ? { scope: p.scope, name: p.name, at: at() } : null);
  });
  item._open = open;
  if (teach.open === keyOf(p)) open(true);
  return item;
}

// New position: name, relative to what, stored as hands or joints; made by capturing the robot (button or pad).
$('btn-add-arm').addEventListener('click', () => {
  if (!teach.data || $('arm-new')) return;
  const name = el('input', { autocomplete: 'off', spellcheck: 'false', placeholder: 'e.g. pre_grasp' });
  const scope = el('select', { 'aria-label': 'Relative to' });
  scope.replaceChildren(el('option', { value: ROBOT_SCOPE, text: 'the robot' }),
    ...teach.data.object_types.map((t) => el('option', { value: t, text: `${t} (a point on it)` })));
  const form = el('select', { 'aria-label': 'Stored as' });
  form.replaceChildren(...(teach.data.tools.length ? [el('option', { value: 'hands', text: 'hand positions (IK)' })] : []),
    el('option', { value: 'joints', text: 'joint angles' }));
  const formField = el('div', { class: 'field' }, el('label', { text: 'Stored as' }), form);
  const msg = el('div', { class: 'msg', hidden: '' });
  const capture = el('button', { class: 'btn primary', text: 'Capture from robot', 'data-edit': '' });
  const cancel = el('button', { class: 'btn', text: 'Cancel' });
  const target = () => ({ scope: scope.value, name: name.value.trim(), at: scope.value === ROBOT_SCOPE ? null : teach.at[scope.value] || null,
    form: scope.value === ROBOT_SCOPE ? form.value : null });
  const update = () => {
    formField.hidden = scope.value !== ROBOT_SCOPE;
    const t = target();
    setTeachTarget(NAME_RE.test(t.name) ? t : null);
  };
  [name, scope, form].forEach((input) => input.addEventListener('input', update));
  const box = el('div', { class: 'point-edit', id: 'arm-new' },
    el('div', { class: 'grid2' }, el('div', { class: 'field' }, el('label', { text: 'Name' }), name),
      el('div', { class: 'field' }, el('label', { text: 'Relative to' }), scope), formField),
    el('p', { class: 'muted small-text', text: 'Put the arm where it should go (pad), then capture here or press L1 + L3.' }),
    msg, el('div', { class: 'row gap' }, cancel, capture));
  cancel.addEventListener('click', () => { box.remove(); setTeachTarget(null); });
  capture.addEventListener('click', async () => {
    const t = target();
    if (!NAME_RE.test(t.name)) { message(msg, 'Names use letters, digits, "_" and "-", starting with a letter.', true); return; }
    if (teach.data.positions.some((p) => keyOf(p) === keyOf(t))) { message(msg, `${t.name} already exists: open it and capture there.`, true); return; }
    capture.disabled = true; message(msg, 'Saving the robot\'s current arm…');
    const { ok, data } = await api('POST', `/api/teach/arm/${encodeURIComponent(t.scope)}/${encodeURIComponent(t.name)}/capture`, t);
    capture.disabled = false;
    if (!ok) { message(msg, failText(data, 'not captured'), true); return; }
    box.remove();
    teach.open = keyOf(t);
    await loadArm();
  });
  $('arm-list').before(box);
  if (teach.open) {
    document.querySelectorAll('#arm-list li.point.pad-target').forEach((other) => other._open(false));
    teach.open = null;
  }
  update();
  name.focus();
});

// A pad teach press saved a position: show the new numbers.
function teachTaught() { if (pointsVisible) loadArm(); }

// ---------- live joints: watch every joint against its limits, set one directly ----------
const JOINT_SKIP = /wheel|passive|active|^base_/;

async function loadJoints(rebuild = false) {
  if (!teach.data) return;
  const { ok, data } = await api('GET', '/api/robot/joints');
  if (!ok) { message($('joints-msg'), data.error || 'no joint states', true); return; }
  message($('joints-msg'), '');
  teach.joints = data;
  const limits = teach.data.joint_limits;
  const names = Object.keys(data).filter((j) => !JOINT_SKIP.test(j) && limits[j]);
  if (rebuild || names.some((j) => !teach.jointInputs[j])) buildJoints(names);
  names.forEach((joint) => {
    const row = teach.jointInputs[joint];
    const value = data[joint];
    const [lower, upper] = limits[joint];
    row.now.textContent = `${showJoint(joint, value)}`;
    if (lower !== null && upper !== null) {
      const share = (value - lower) / (upper - lower);
      row.bar.style.setProperty('--at', `${Math.min(Math.max(share, 0), 1) * 100}%`);
      const room = Math.min(value - lower, upper - value);
      row.row.classList.toggle('near-limit', room < (isLinear(joint) ? 0.02 : 10 / DEG));
    }
  });
}

function buildJoints(names) {
  const limits = teach.data.joint_limits;
  const groups = {};
  names.forEach((joint) => { (groups[jointGroup(joint)] ||= []).push(joint); });
  teach.jointInputs = {};
  $('joint-list').replaceChildren(...Object.entries(groups).map(([group, members]) => el('details', { class: 'joint-live' },
    el('summary', { text: group }),
    ...members.map((joint) => {
      const [lower, upper] = limits[joint];
      const range = {};  // a bound may be missing (null: unbounded)
      if (lower !== null) range.min = showJoint(joint, lower);
      if (upper !== null) range.max = showJoint(joint, upper);
      const input = el('input', { type: 'number', step: isLinear(joint) ? '0.01' : '1', inputmode: 'decimal', 'aria-label': `${joint} target`, ...range });
      input.value = showJoint(joint, teach.joints[joint]);
      const now = el('span', { class: 'mono' });
      const bar = el('span', { class: 'limit-bar', title: `${range.min ?? '–∞'} to ${range.max ?? '∞'} ${jointUnit(joint)}` });
      const send = async (value) => {
        input.value = value;
        const { ok, data } = await api('PUT', '/api/robot/joints', { joints: { [joint]: unshowJoint(joint, Number(value)) } });
        message($('joints-msg'), ok ? '' : failText(data, 'not sent'), !ok);
        setTimeout(() => loadJoints(), 600);
      };
      const nudge = (delta) => el('button', { class: 'btn small', text: delta > 0 ? '+' : '−', 'data-edit': '' });
      const minus = nudge(-1);
      const plus = nudge(1);
      const stepOf = () => (isLinear(joint) ? 0.01 : 5);
      minus.addEventListener('click', () => send(+(Number(input.value) - stepOf()).toFixed(4)));
      plus.addEventListener('click', () => send(+(Number(input.value) + stepOf()).toFixed(4)));
      input.addEventListener('change', () => send(input.value));
      const row = el('div', { class: 'joint-row' },
        el('span', { class: 'joint-name', text: `${joint.startsWith(`${group}_`) ? joint.slice(group.length + 1) : joint} ${jointUnit(joint)}` }),
        bar, now, minus, input, plus);
      teach.jointInputs[joint] = { input, now, bar, row };
      return row;
    }))));
  setPointsBusy(pointsBusy);
}
setInterval(() => { if (pointsVisible && !document.hidden && teach.data) loadJoints(); }, 1000);
