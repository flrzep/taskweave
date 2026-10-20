'use strict';
// taskweave HMI, Teach tab: Poses (relative to the robot, poses.yaml) and Points (on objects, objects/<type>.yaml),
// one editor for both (the server's arm positions, arm.py). A pose or point that is open receives the pad's teach
// press (L1 + L3); Capture does the same from here.
// Uses helpers and state from app.js ($, el, api, message, show, state, DEG, NAME_RE, fillSelect).

const ROBOT_SCOPE = 'robot';
const teach = { data: null, open: null };
const OPEN_ITEMS = '#pose-list li.point.pad-target, #point-list li.point.pad-target';

const keyOf = (p) => `${p.scope}/${p.name}`;
const isLinear = (joint) => (teach.data?.linear_joints || []).includes(joint);
const showJoint = (joint, value) => (isLinear(joint) ? +value.toFixed(4) : +(value * DEG).toFixed(2));
const unshowJoint = (joint, value) => (isLinear(joint) ? value : value / DEG);
const jointUnit = (joint) => (isLinear(joint) ? 'm' : '°');
const failText = (data, fallback) => (data.problems || [data.error || fallback]).join('\n');

async function loadArm() {
  const { ok, data } = await api('GET', '/api/teach/arm');
  if (!ok) { $('pose-list').replaceChildren(el('li', { class: 'error', text: failText(data, 'poses and points unavailable') })); return; }
  teach.data = data;
  renderArm();
}

// Where the pad's teach press goes: the open position (or none).
function setTeachTarget(target) {
  api('PUT', '/api/teach/target', target || {});
  const text = target ? `pad L1 + L3 teaches ${target.scope === ROBOT_SCOPE ? '' : `${target.scope} `}${target.name}` : '';
  message($('arm-target'), text);
}

const empty = (text) => el('li', { class: 'muted small-text empty', text });

function renderArm() {
  const data = teach.data;
  const poses = data.positions.filter((p) => p.scope === ROBOT_SCOPE);
  $('pose-list').replaceChildren(...(poses.length ? poses.map((p) => positionItem(p))
    : [empty('No poses yet: put the arms where they should be, then + Pose.')]));
  const groups = data.object_types.filter((type) => data.positions.some((p) => p.scope === type) || (data.placed[type] || []).length);
  $('point-list').replaceChildren(...(groups.length ? groups.map((type) => {
    const points = data.positions.filter((p) => p.scope === type);
    const head = el('div', { class: 'row between wrap' }, el('div', { class: 'sub', text: `On ${type}` }), atPicker(type));
    return el('li', { class: 'arm-group' }, head,
      el('ul', { class: 'points' }, points.length ? points.map((p) => positionItem(p))
        : [empty(`No points on ${type} yet: put the gripper on one, then + Point.`)]));
  }) : [empty(data.object_types.length
    ? 'No placed objects yet: place one in cell/objects.yaml to teach points on it.'
    : 'No object types yet: add config/objects/<type>.yaml, place one in cell/objects.yaml, then teach points on it.')]));
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
  Object.entries(p.grip_m || {}).forEach(([tool, value]) => { if (value != null) parts.push(`${tool} grip ${Math.round(value * 1000)} mm`); });
  return parts.join(' · ');
}

function numberInput(label, value, attrs = {}) {
  const input = el('input', { type: 'number', step: 'any', inputmode: 'decimal', 'aria-label': label, ...attrs });
  input.value = value;
  return [el('div', { class: 'field' }, el('label', { text: label }), input), input];
}

// One pose or point: capture / test, numbers, advanced settings, rename, delete.
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
    skillResult(msg, result);
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
  // grip (points on objects): how far each hand's fingers stay open when it closes here (mm, within this robot's
  // range from the profile's opening_m table); empty = close fully
  const grips = {};
  const ranges = data.grip_range_m || {};
  const gripFields = robotScope ? [] : Object.keys(p.hands).filter((tool) => ranges[tool]).map((tool) => {
    const [low, high] = ranges[tool].map((v) => Math.round(v * 1000));
    const value = p.grip_m?.[tool];
    const [field, input] = numberInput(`${tool} grip (mm, ${low}–${high})`, value == null ? '' : Math.round(value * 10000) / 10,
      { min: String(low), max: String(high), step: '1', placeholder: 'close fully' });
    grips[tool] = { input, low, high };
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
      body.grip_m = {};
      for (const [tool, { input, low, high }] of Object.entries(grips)) {
        const value = Number(input.value);
        if (input.value.trim() !== '' && !(Number.isFinite(value) && value >= low && value <= high)) {
          message(msg, `${tool} grip: ${low}–${high} mm, or empty to close fully.`, true); return;
        }
        body.grip_m[tool] = input.value.trim() === '' ? null : value / 1000;
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
    document.querySelectorAll(OPEN_ITEMS).forEach((other) => { if (other !== item) other._open(false); });
    open(opening);
    teach.open = opening ? keyOf(p) : null;
    setTeachTarget(opening ? { scope: p.scope, name: p.name, at: at() } : null);
  });
  item._open = open;
  if (teach.open === keyOf(p)) open(true);
  return item;
}

// New pose (relative to the robot: hands or joints) or point (on an object type); made by capturing the robot (button
// or pad).
function newPosition(kind) {
  if (!teach.data || $('arm-new')) return;
  const isPose = kind === 'pose';
  const list = $(isPose ? 'pose-list' : 'point-list');
  if (!isPose && !teach.data.object_types.length) return;
  const name = el('input', { autocomplete: 'off', spellcheck: 'false', placeholder: isPose ? 'e.g. look_down' : 'e.g. grasp_top' });
  const scope = el('select', { 'aria-label': 'On object type' });
  scope.replaceChildren(...(isPose ? [el('option', { value: ROBOT_SCOPE, text: 'the robot' })]
    : teach.data.object_types.map((t) => el('option', { value: t, text: t }))));
  const scopeField = el('div', { class: 'field' }, el('label', { text: 'On' }), scope);
  scopeField.hidden = isPose;
  const form = el('select', { 'aria-label': 'Stored as' });
  form.replaceChildren(...(teach.data.tools.length ? [el('option', { value: 'hands', text: 'hand positions (IK)' })] : []),
    el('option', { value: 'joints', text: 'joint angles' }));
  const formField = el('div', { class: 'field' }, el('label', { text: 'Stored as' }), form);
  formField.hidden = !isPose;
  const msg = el('div', { class: 'msg', hidden: '' });
  const capture = el('button', { class: 'btn primary', text: 'Capture from robot', 'data-edit': '' });
  const cancel = el('button', { class: 'btn', text: 'Cancel' });
  const target = () => ({ scope: scope.value, name: name.value.trim(), at: isPose ? null : teach.at[scope.value] || null,
    form: isPose ? form.value : null });
  const update = () => {
    const t = target();
    setTeachTarget(NAME_RE.test(t.name) ? t : null);
  };
  [name, scope, form].forEach((input) => input.addEventListener('input', update));
  const box = el('div', { class: 'point-edit', id: 'arm-new' },
    el('div', { class: 'grid2' }, el('div', { class: 'field' }, el('label', { text: 'Name' }), name), scopeField, formField),
    el('p', { class: 'muted small-text', text: isPose ? 'Put the arms where they should be (pad), then capture here or press L1 + L3.'
      : 'Put the gripper on the object (pad), then capture here or press L1 + L3.' }),
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
  list.before(box);
  if (teach.open) {
    document.querySelectorAll(OPEN_ITEMS).forEach((other) => other._open(false));
    teach.open = null;
  }
  update();
  name.focus();
}
$('btn-add-pose').addEventListener('click', () => newPosition('pose'));
$('btn-add-point').addEventListener('click', () => newPosition('point'));

// A pad teach press saved a position: show the new numbers.
function teachTaught() { if (pointsVisible) loadArm(); }
