'use strict';
// taskweave HMI, Robot tab: navigation clearance and driving (applied live to the path planner) and arm solutions.
// Named poses are arm positions in the Teach tab (teach.js).
// Uses helpers and state from app.js ($, el, api, message, state, DEG, NAME_RE, setPointsBusy, pointsBusy).

const NAV_FIELDS = ['front', 'back', 'left', 'right'];
const SVG_NS = 'http://www.w3.org/2000/svg';

async function loadRobot() {
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
