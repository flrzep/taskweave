'use strict';
// taskweave HMI, Robot tab: what the robot is doing now (pose: app.js refreshPose; grippers and joints, set directly)
// and its settings: navigation clearance and driving (applied live to the path planner), arm solutions.
// Named poses and points are taught in the Teach tab (teach.js).
// Uses helpers and state from app.js ($, el, api, message, show, state, DEG, setPointsBusy, pointsBusy, robotVisible).

const NAV_FIELDS = ['front', 'back', 'left', 'right'];
const SVG_NS = 'http://www.w3.org/2000/svg';

// The robot alone in 3D (viewer3d.js robot mode: an ES module, it runs after these classic scripts), its navigation
// areas from the Navigation settings below, redrawn while they are edited (drawSketch). Made when the tab first opens.
let robotViewer = null;
function startRobotViewer() {
  if (robotViewer || !window.Viewer3D) return;
  robotViewer = new window.Viewer3D($('viewer3d-robot'), { mode: 'robot' });
  robotViewer.load();
  if (state.robotData) drawSketch();
}
window.addEventListener('viewer3d-ready', () => { if (robotVisible) startRobotViewer(); });

async function loadRobot() {
  startRobotViewer();
  const { ok, data } = await api('GET', '/api/robot');
  if (!ok) { message($('nav-msg'), data.error || (data.problems || []).join('\n'), true); return; }
  state.robotData = data;
  $('nav-file').textContent = data.files.navigation;
  const nav = data.navigation || {};
  const [length, width] = data.footprint || [0.6, 0.6];
  const envelope = nav.envelope || { front: length / 2, back: length / 2, left: width / 2, right: width / 2 };
  NAV_FIELDS.forEach((side) => { $(`nav-${side}`).value = envelope[side]; });
  $('nav-padding').value = nav.padding ?? 0;
  $('nav-inflation').value = nav.inflation_radius ?? 0.65;
  message($('nav-msg'), nav.envelope ? '' : 'No navigation.yaml yet: the values shown are the chassis footprint.');
  // driving: defaults are the planner's start-up tuning (ros/nav2_params.yaml)
  const driving = { max_forward: 0.6, max_reverse: 0.4, max_turn: 1.0, max_sideways: 0.5, prefer_forward: 0, ...nav.driving };
  const holonomic = data.base_type === 'holonomic';
  $('drv-forward').value = driving.max_forward;
  $('drv-reverse').value = driving.max_reverse;
  $('drv-turn').value = +(driving.max_turn * DEG).toFixed(1);
  $('drv-prefer').value = driving.prefer_forward;
  $('drv-sideways').checked = holonomic && (driving.sideways ?? true);
  $('drv-sideways').disabled = !holonomic;
  $('drv-sideways-max').value = driving.max_sideways;
  $('drv-sideways-note').textContent = holonomic ? '' : `This robot's base is ${data.base_type}: it cannot drive sideways.`;
  updateSideways();
  drawSketch();
  showIkChoice(data.ik_choice);
  live.rows = {};
  loadJoints();
  setPointsBusy(pointsBusy);
}

// ---------- arm solutions (profile ik.choose) ----------
const IK_WEIGHTS = ['limits', 'elbow', 'motion', 'symmetry'];
const ELBOW_TEXT = { '': 'no preference', down: 'down', up: 'up', out: 'out', in: 'in', back: 'back', forward: 'forward' };

function showIkChoice(choice) {
  show($('ik-card'), !!choice);
  if (!choice) return;
  $('ik-file').textContent = `robots/${state.robotData.robot}/profile.yaml`;
  const current = choice.elbow || '';
  $('ik-elbow').replaceChildren(...['', ...choice.elbow_sides].map((side) => {
    const button = el('button', { text: ELBOW_TEXT[side] || side, 'aria-pressed': String(side === current) });
    button.dataset.side = side;
    button.addEventListener('click', () => $('ik-elbow').querySelectorAll('button').forEach(
      (b) => b.setAttribute('aria-pressed', String(b === button))));
    return button;
  }));
  $('ik-margin').value = choice.limit_margin_deg;
  IK_WEIGHTS.forEach((k) => { $(`ik-w-${k}`).value = choice.weights[k]; });
  message($('ik-msg'), choice.elbow_vector ? `The profile sets the elbow as a vector ${JSON.stringify(choice.elbow_vector)}; saving replaces it.` : '');
}

$('btn-ik-save').addEventListener('click', async () => {
  const side = $('ik-elbow').querySelector('[aria-pressed="true"]')?.dataset.side || '';
  const body = { elbow: side || null, limit_margin_deg: Number($('ik-margin').value), weights: {} };
  IK_WEIGHTS.forEach((k) => { body.weights[k] = Number($(`ik-w-${k}`).value); });
  const { ok, data } = await api('PUT', '/api/robot/ik', body);
  message($('ik-msg'), ok ? `Saved to ${data.file}. Used from the next arm move.` : (data.problems || [data.error || 'not saved']).join('\n'), !ok);
});

function navSettings() {
  const settings = { envelope: {} };
  NAV_FIELDS.forEach((side) => { settings.envelope[side] = Number($(`nav-${side}`).value); });
  settings.padding = Number($('nav-padding').value);
  settings.inflation_radius = Number($('nav-inflation').value);
  settings.driving = {
    max_forward: Number($('drv-forward').value), max_reverse: Number($('drv-reverse').value),
    max_turn: Number($('drv-turn').value) / DEG, prefer_forward: Number($('drv-prefer').value),
    sideways: $('drv-sideways').checked, max_sideways: Number($('drv-sideways-max').value),
  };
  return settings;
}

function updateSideways() {
  $('drv-sideways-max').disabled = !$('drv-sideways').checked;
}
$('drv-sideways').addEventListener('change', updateSideways);

// Top view, robot x (forward) up and y (left) to the left, 1 unit = 1 m: inflation zone, chassis, padding, envelope.
function drawSketch() {
  const svg = $('nav-sketch');
  const { envelope: e, padding = 0, inflation_radius: inflation = 0 } = navSettings();
  const [length, width] = state.robotData?.footprint || [0.6, 0.6];
  robotViewer?.setAreas({ footprint: [length, width], envelope: e, padding, inflation });
  const rect = (back, right, front, left, cls, round = 0) => {
    const node = document.createElementNS(SVG_NS, 'rect');
    const attrs = { x: -left, y: -front, width: left + right, height: front + back, rx: round, class: cls };
    Object.entries(attrs).forEach(([key, value]) => node.setAttribute(key, value));
    return node;
  };
  const outer = padding + inflation;
  const extent = Math.max(e.front, e.back, e.left, e.right) + outer + 0.1;
  svg.setAttribute('viewBox', `${-extent} ${-extent} ${2 * extent} ${2 * extent}`);
  const arrow = document.createElementNS(SVG_NS, 'path');
  arrow.setAttribute('d', 'M 0 -0.18 L 0.07 0 L -0.07 0 Z');
  arrow.setAttribute('class', 'heading');
  svg.replaceChildren(
    rect(e.back + outer, e.right + outer, e.front + outer, e.left + outer, 'inflation', outer),
    rect(length / 2, width / 2, length / 2, width / 2, 'chassis'),
    rect(e.back + padding, e.right + padding, e.front + padding, e.left + padding, 'padding'),
    rect(e.back, e.right, e.front, e.left, 'envelope'),
    arrow);
}
[...NAV_FIELDS.map((side) => `nav-${side}`), 'nav-padding', 'nav-inflation']
  .forEach((id) => $(id).addEventListener('input', drawSketch));

$('btn-nav-save').addEventListener('click', async () => {
  const { ok, data } = await api('PUT', '/api/robot/navigation', navSettings());
  if (!ok) { message($('nav-msg'), (data.problems || [data.error || 'not saved']).join('\n'), true); return; }
  const results = Object.entries(data.applied).map(([node, result]) => `${node.split('/').pop()}: ${result}`);
  const allApplied = Object.values(data.applied).every((result) => result === 'applied');
  message($('nav-msg'), `Saved to ${data.file}. ${results.join(' · ')}`, !allApplied);
});

// left_joint3 -> left, left_arm_joint3 -> left_arm, body_joint -> body, head_joint1 -> head
function jointGroup(joint) {
  return joint.replace(/_?joint_?\d*$/, '') || joint;
}

// ---------- live: grippers and joints, against their limits; set one directly ----------
// Wheels and the Ge103In linkage are not set by hand; mimic joints are not in joint_limits (server); the grippers'
// driven joints get their own rows (opening in mm, Open / Close) instead of a joint row.
const JOINT_SKIP = /wheel|passive|active|^base_/;
const live = { rows: {}, grippers: {} };
const liveLinear = (joint) => (state.robotData?.linear_joints || []).includes(joint);
const liveShow = (joint, value) => (liveLinear(joint) ? +value.toFixed(4) : +(value * DEG).toFixed(2));
const liveUnit = (joint) => (liveLinear(joint) ? 'm' : '°');

// finger opening (m) at a gripper joint value, from the profile's [[joint, opening], ...] table (null: no table)
function gripperOpening(gripper, value) {
  const table = [...(gripper.opening_m || [])].sort((a, b) => a[0] - b[0]);
  if (!table.length) return null;
  if (value <= table[0][0]) return table[0][1];
  for (let i = 1; i < table.length; i += 1) {
    const [j0, o0] = table[i - 1];
    const [j1, o1] = table[i];
    if (value <= j1) return o0 + ((o1 - o0) * (value - j0)) / (j1 - j0);
  }
  return table[table.length - 1][1];
}

async function sendJoints(joints, msgNode) {
  const { ok, data } = await api('PUT', '/api/robot/joints', { joints });
  message(msgNode, ok ? '' : (data.problems || [data.error || 'not sent']).join('\n'), !ok);
  setTimeout(() => loadJoints(), 600);
}

async function loadJoints() {
  const robot = state.robotData;
  if (!robot) return;
  const { ok, data } = await api('GET', '/api/robot/joints');
  if (!ok) { message($('joints-msg'), data.error || 'no joint states', true); return; }
  message($('joints-msg'), '');
  const limits = robot.joint_limits;
  const gripperJoints = new Set(Object.values(robot.grippers || {}).map((g) => g.joint));
  const names = Object.keys(data).filter((j) => !JOINT_SKIP.test(j) && limits[j] && !gripperJoints.has(j));
  if (names.some((j) => !live.rows[j]) || Object.keys(live.rows).length !== names.length) buildJoints(names, data);
  names.forEach((joint) => {
    const row = live.rows[joint];
    const value = data[joint];
    const [lower, upper] = limits[joint];
    row.now.textContent = `${liveShow(joint, value)}`;
    if (lower !== null && upper !== null) {
      const share = (value - lower) / (upper - lower);
      row.bar.style.setProperty('--at', `${Math.min(Math.max(share, 0), 1) * 100}%`);
      const room = Math.min(value - lower, upper - value);
      row.row.classList.toggle('near-limit', room < (liveLinear(joint) ? 0.02 : 10 / DEG));
    }
  });
  renderGrippers(data);
}

function buildJoints(names, values) {
  const limits = state.robotData.joint_limits;
  const groups = {};
  names.forEach((joint) => { (groups[jointGroup(joint)] ||= []).push(joint); });
  live.rows = {};
  $('joint-list').replaceChildren(...Object.entries(groups).map(([group, members]) => el('details', { class: 'joint-live' },
    el('summary', { text: group }),
    ...members.map((joint) => {
      const [lower, upper] = limits[joint];
      const range = {};  // a bound may be missing (null: unbounded)
      if (lower !== null) range.min = liveShow(joint, lower);
      if (upper !== null) range.max = liveShow(joint, upper);
      const input = el('input', { type: 'number', step: liveLinear(joint) ? '0.01' : '1', inputmode: 'decimal', 'aria-label': `${joint} target`,
        'data-spin': 'off', ...range });  // − / + beside it
      input.value = liveShow(joint, values[joint]);
      const now = el('span', { class: 'mono' });
      const bar = el('span', { class: 'limit-bar', title: `${range.min ?? '–∞'} to ${range.max ?? '∞'} ${liveUnit(joint)}` });
      const send = (value) => {
        input.value = value;
        sendJoints({ [joint]: liveLinear(joint) ? Number(value) : Number(value) / DEG }, $('joints-msg'));
      };
      const nudge = (delta) => el('button', { class: 'btn small', text: delta > 0 ? '+' : '−', 'data-edit': '' });
      const minus = nudge(-1);
      const plus = nudge(1);
      const stepOf = () => (liveLinear(joint) ? 0.01 : 5);
      minus.addEventListener('click', () => send(+(Number(input.value) - stepOf()).toFixed(4)));
      plus.addEventListener('click', () => send(+(Number(input.value) + stepOf()).toFixed(4)));
      input.addEventListener('change', () => send(input.value));
      const row = el('div', { class: 'joint-row' },
        el('span', { class: 'joint-name', text: `${joint.startsWith(`${group}_`) ? joint.slice(group.length + 1) : joint} ${liveUnit(joint)}` }),
        bar, now, minus, input, plus);
      live.rows[joint] = { input, now, bar, row };
      return row;
    }))));
  setPointsBusy(pointsBusy);
}

// One row per joint gripper: opening now (mm, or the joint angle without a table), a bar from open to closed, Open / Close.
function renderGrippers(values) {
  const grippers = Object.entries(state.robotData?.grippers || {}).filter(([, g]) => g.joint in values);
  show($('gripper-card'), grippers.length > 0);
  if (Object.keys(live.grippers).join() !== grippers.map(([tool]) => tool).join()) {
    live.grippers = {};
    $('gripper-list').replaceChildren(...grippers.map(([tool, g]) => {
      const now = el('span', { class: 'mono' });
      const bar = el('span', { class: 'limit-bar', title: 'open to closed' });
      const open = el('button', { class: 'btn small', text: 'Open', 'data-edit': '' });
      const close = el('button', { class: 'btn small', text: 'Close', 'data-edit': '' });
      open.addEventListener('click', () => sendJoints({ [g.joint]: g.open }, $('gripper-msg')));
      close.addEventListener('click', () => sendJoints({ [g.joint]: g.closed }, $('gripper-msg')));
      live.grippers[tool] = { now, bar };
      return el('div', { class: 'joint-row' }, el('span', { class: 'joint-name', text: `${tool} gripper` }), bar, now, open, close);
    }));
    setPointsBusy(pointsBusy);
  }
  grippers.forEach(([tool, g]) => {
    const value = values[g.joint];
    const opening = gripperOpening(g, value);
    live.grippers[tool].now.textContent = opening === null ? `${(value * DEG).toFixed(1)}°` : `${Math.round(opening * 1000)} mm`;
    const share = (value - g.open) / (g.closed - g.open);
    live.grippers[tool].bar.style.setProperty('--at', `${Math.min(Math.max(share, 0), 1) * 100}%`);
  });
}
setInterval(() => { if (robotVisible && !document.hidden && state.robotData) loadJoints(); }, 1000);
