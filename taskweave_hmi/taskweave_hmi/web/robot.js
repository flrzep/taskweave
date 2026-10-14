'use strict';
// taskweave HMI, Robot tab: navigation clearance (applied live to the path planner) and named poses; tool targets
// are checked for reachability (IK) while they are edited.
// Uses helpers and state from app.js ($, el, api, message, state, DEG, NAME_RE, setPointsBusy, pointsBusy).

const NAV_FIELDS = ['front', 'back', 'left', 'right'];
const SVG_NS = 'http://www.w3.org/2000/svg';

async function loadRobot() {
  const { ok, data } = await api('GET', '/api/robot');
  if (!ok) { message($('nav-msg'), data.error || (data.problems || []).join('\n'), true); return; }
  state.robotData = data;
  $('nav-file').textContent = data.files.navigation;
  $('poses-file').textContent = data.files.poses;
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
  $('pose-list').replaceChildren(...data.poses.map((pose) => poseItem(pose)));
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

// One joint pose: grouped joint inputs (a group can be left out), set from robot, move here, save, delete.
function poseItem(pose, isNew = false) {
  const linear = new Set(state.robotData.linear_joints);
  const show = (joint, value) => (linear.has(joint) ? +value.toFixed(4) : +(value * DEG).toFixed(2));
  const unshow = (joint, value) => (linear.has(joint) ? value : value / DEG);
  // a pose with tools places each tool's TCP (solved by IK); a new pose does so when the robot has IK tools
  const toolNames = isNew ? state.robotData.tools : Object.keys(pose.tools || {});
  const toolPoses = state.robotData.poses.filter((p) => Object.keys(p.tools || {}).length);
  const joints = !isNew ? Object.keys(pose.joints)
    : [...new Set((toolNames.length ? toolPoses : state.robotData.poses).flatMap((p) => Object.keys(p.joints)))];
  const inputs = {};
  const groups = {};
  // lift (linear joints) first: it sets the robot's height, the arms follow it
  [...joints.filter((j) => linear.has(j)), ...joints.filter((j) => !linear.has(j))]
    .forEach((joint) => { (groups[jointGroup(joint)] ||= []).push(joint); });
  const included = {};
  const groupNodes = Object.entries(groups).map(([group, members]) => {
    const box = el('input', { type: 'checkbox' });
    box.checked = true;
    included[group] = box;
    const fields = members.map((joint) => {
      // the robot's range for this joint, in the shown unit: the field flags values outside it
      const [lower, upper] = (state.robotData.joint_limits || {})[joint] || [null, null];
      const range = {};
      if (lower !== null) range.min = show(joint, lower);
      if (upper !== null) range.max = show(joint, upper);
      const input = el('input', { type: 'number', step: 'any', inputmode: 'decimal', ...range });
      input.value = pose.joints[joint] === undefined ? '' : show(joint, pose.joints[joint]);
      inputs[joint] = input;
      const unit = linear.has(joint) ? 'm' : '°';
      const bounds = 'min' in range && 'max' in range ? ` (${range.min} to ${range.max})` : '';
      const label = `${joint.startsWith(`${group}_`) ? joint.slice(group.length + 1) : joint} ${unit}${bounds}`;
      return el('div', { class: 'field' }, el('label', { text: label }), input);
    });
    box.addEventListener('change', () => fields.forEach((field) => { field.hidden = !box.checked; }));
    return el('div', { class: 'joint-group' }, el('label', {}, box, group), el('div', { class: 'grid3' }, fields));
  });
  // tool targets: TCP position (m) and approach / closing directions, base frame
  const toolInputs = {};
  const reach = {};  // per tool: reachability line (checked while editing)
  const toolNodes = toolNames.map((tool) => {
    const target = (pose.tools || {})[tool] || { tcp: [0.5, 0, 0.3], approach: [1, 0, 0], closing: [0, 1, 0] };
    toolInputs[tool] = {};
    const rows = [['tcp', 'TCP (m)'], ['approach', 'approach'], ['closing', 'closing']].map(([key, title]) => {
      toolInputs[tool][key] = ['x', 'y', 'z'].map((axis, i) => {
        const input = el('input', { type: 'number', step: 'any', inputmode: 'decimal', 'aria-label': `${tool} ${key} ${axis}` });
        input.value = +Number(target[key][i]).toFixed(4);
        return input;
      });
      return el('div', { class: 'field' }, el('label', { text: `${title}  x · y · z` }), el('div', { class: 'grid3' }, toolInputs[tool][key]));
    });
    reach[tool] = el('div', { class: 'reach' });
    return el('div', { class: 'joint-group' }, el('label', { text: `${tool} tool (IK)` }), ...rows, reach[tool]);
  });
  const lifts = Object.keys(pose.joints).filter((joint) => linear.has(joint));
  const liftText = lifts.map((joint) => `${lifts.length > 1 ? joint : 'lift'} ${pose.joints[joint].toFixed(2)} m`);
  const shape = toolNames.length ? `${toolNames.join(', ')} TCP` : `${Object.keys(pose.joints).length} joints`;
  const head = el('button', { class: 'point-head', 'aria-expanded': String(isNew) },
    el('b', { text: pose.name || 'new pose' }),
    el('span', { class: 'mono', text: [...liftText, shape, Object.keys(groups).join(', ')].join(' · ') }),
    pose.used_by.length ? el('span', { class: 'used', text: `used by ${pose.used_by.join(', ')}` }) : '',
    (pose.problems || []).length ? el('span', { class: 'bad', text: `outside this robot's limits: ${pose.problems.join('; ')}` }) : '');
  const nameInput = el('input', { autocomplete: 'off', spellcheck: 'false' });
  nameInput.value = pose.name || '';
  const msg = el('div', { class: 'msg', hidden: '' });
  const robotButton = el('button', { class: 'btn small', text: 'Set from robot' });
  const moveButton = el('button', { class: 'btn small', text: 'Move here', 'data-edit': '' });
  const saveButton = el('button', { class: 'btn primary', text: 'Save', 'data-edit': '' });
  const deleteButton = el('button', { class: 'btn danger', text: isNew ? 'Cancel' : 'Delete', 'data-edit': '' });
  let changed = isNew;  // Move here runs the saved pose, so it waits for a save after edits
  const markChanged = () => { changed = true; moveButton.disabled = true; moveButton.title = 'Save first'; };
  const edit = el('div', { class: 'point-edit' },
    el('div', { class: 'field' }, el('label', { text: 'Name' }), nameInput), ...groupNodes, ...toolNodes,
    el('div', { class: 'row gap' }, robotButton, moveButton), msg, el('div', { class: 'row gap' }, deleteButton, saveButton));
  edit.hidden = !isNew;
  edit.addEventListener('input', markChanged);

  // Reachability: solved the way Move here solves it (current lift / head values from the form), while editing.
  const formJoints = () => {
    const values = {};
    Object.entries(groups).forEach(([group, members]) => {
      if (!included[group].checked) return;
      members.forEach((joint) => {
        const value = Number(inputs[joint].value);
        if (inputs[joint].value.trim() !== '' && Number.isFinite(value)) values[joint] = unshow(joint, value);
      });
    });
    return values;
  };
  const formTarget = (tool) => {
    const target = {};
    for (const key of ['tcp', 'approach', 'closing']) {
      target[key] = toolInputs[tool][key].map((input) => Number(input.value));
      if (toolInputs[tool][key].some((input) => input.value.trim() === '') || target[key].some((v) => !Number.isFinite(v))) return null;
    }
    return target;
  };
  const AXES = [['forward', 'back'], ['left', 'right'], ['up', 'down']];
  const shiftText = (from, to) => to.map((v, i) => [v - from[i], i]).filter(([d]) => Math.abs(d) > 1e-6)
    .map(([d, i]) => `${Math.abs(d).toFixed(2)} m ${AXES[i][d > 0 ? 0 : 1]}`).join(', ');
  const showReach = (tool, result, target) => {
    const line = reach[tool];
    const parts = [];
    if (result.ok) {
      parts.push(el('span', { class: 'ok', text: '✓ reachable' }));
    } else {
      parts.push(el('span', { class: 'bad', text: '✗ not reachable' }), el('div', { class: 'small-text muted', text: result.message }));
      if (result.suggestion) {
        const tcp = result.suggestion.tcp;
        const use = el('button', { class: 'btn small', text: 'Use' });
        use.addEventListener('click', () => {
          toolInputs[tool].tcp.forEach((input, i) => { input.value = tcp[i]; });
          markChanged(); scheduleReach();
        });
        parts.push(el('div', { class: 'row gap' }, el('span', {
          text: `Nearest reachable with this orientation: TCP (${tcp.map((v) => v.toFixed(2)).join(', ')}), ${shiftText(target.tcp, tcp)}` }), use));
      } else if (result.searched) {
        parts.push(el('div', { class: 'small-text', text: 'Nothing reachable within 0.4 m with this orientation: change approach / closing or the lift.' }));
      } else {
        const find = el('button', { class: 'btn small', text: 'Find nearest reachable' });
        find.addEventListener('click', () => checkReach(tool));
        parts.push(find);
      }
    }
    result.warnings.forEach((warning) => parts.push(el('div', { class: 'small-text warn', text: `⚠ ${warning}` })));
    line.replaceChildren(...parts);
  };
  let reachRun = 0;
  async function checkReach(searchTool = null) {
    if (!toolNames.length) return;
    const run = ++reachRun;
    const tools = {};
    toolNames.forEach((tool) => {
      const target = formTarget(tool);
      if (target) tools[tool] = target;
      else reach[tool].replaceChildren(el('span', { class: 'muted', text: 'enter three numbers per row' }));
    });
    if (!Object.keys(tools).length) return;
    Object.keys(tools).forEach((tool) => {
      if (!searchTool || tool === searchTool) {
        reach[tool].replaceChildren(el('span', { class: 'muted', text: tool === searchTool ? 'searching nearby TCP positions…' : 'checking…' }));
      }
    });
    const { ok, data } = await api('POST', '/api/robot/reach',
      { joints: formJoints(), tools, search: searchTool ? [searchTool] : [] });
    if (run !== reachRun) return;  // a newer check is under way
    if (!ok) {
      Object.keys(tools).forEach((tool) => reach[tool].replaceChildren(
        el('span', { class: 'muted', text: data.error || (data.problems || []).join(' ') || 'reach check unavailable' })));
      return;
    }
    Object.entries(data).forEach(([tool, result]) => {
      if (searchTool && tool === searchTool) result.searched = true;
      showReach(tool, result, tools[tool]);
    });
  }
  let reachTimer = null;
  const scheduleReach = () => { clearTimeout(reachTimer); reachTimer = setTimeout(() => checkReach(), 700); };
  if (toolNames.length) edit.addEventListener('input', scheduleReach);
  moveButton.disabled = changed;
  const item = el('li', { class: 'point' }, head, edit);
  head.addEventListener('click', () => {
    edit.hidden = !edit.hidden;
    head.setAttribute('aria-expanded', String(!edit.hidden));
    if (!edit.hidden) checkReach();
  });
  if (isNew) checkReach();

  robotButton.addEventListener('click', async () => {
    const { ok, data } = await api('GET', '/api/robot/joints');
    if (!ok) { message(msg, data.error || 'no joint states', true); return; }
    const missing = Object.keys(inputs).filter((joint) => data[joint] === undefined);
    Object.entries(inputs).forEach(([joint, input]) => { if (data[joint] !== undefined) input.value = show(joint, data[joint]); });
    if (toolNames.length) {
      const tools = await api('GET', '/api/robot/tools');
      if (!tools.ok) { message(msg, tools.data.error || 'no tool poses (is MoveIt running?)', true); return; }
      toolNames.forEach((tool) => ['tcp', 'approach', 'closing'].forEach((key) => toolInputs[tool][key]
        .forEach((input, i) => { input.value = tools.data[tool][key][i]; })));
    }
    markChanged();
    checkReach();
    message(msg, missing.length ? `Not in the joint states: ${missing.join(', ')}` : 'Filled in from the robot. Check, then save.',
      missing.length > 0);
  });
  moveButton.addEventListener('click', async () => {
    message(msg, `Moving to ${pose.name}…`);
    moveButton.disabled = true;
    const { ok, data } = await api('POST', '/api/robot/move', { pose: pose.name });
    moveButton.disabled = changed;
    if (!ok) { message(msg, (data.problems || [data.error || 'not moved']).join('\n'), true); return; }
    message(msg, `${data.outcome}${data.message ? ': ' + data.message : ''}`, data.outcome !== 'succeeded');
  });
  saveButton.addEventListener('click', async () => {
    const name = nameInput.value.trim();
    if (!NAME_RE.test(name)) { message(msg, 'Names use letters, digits, "_" and "-", starting with a letter.', true); return; }
    const values = {};
    for (const [group, members] of Object.entries(groups)) {
      if (!included[group].checked) continue;
      for (const joint of members) {
        const value = Number(inputs[joint].value);
        if (inputs[joint].value.trim() === '' || !Number.isFinite(value)) { message(msg, `${joint}: enter a number.`, true); return; }
        const field = inputs[joint];
        if (field.validity.rangeUnderflow || field.validity.rangeOverflow) {
          message(msg, `${joint}: ${field.value} is outside this robot's range ${field.min} to ${field.max}.`, true); return;
        }
        values[joint] = unshow(joint, value);
      }
    }
    const tools = {};
    for (const tool of toolNames) {
      tools[tool] = {};
      for (const key of ['tcp', 'approach', 'closing']) {
        tools[tool][key] = toolInputs[tool][key].map((input) => Number(input.value));
        if (toolInputs[tool][key].some((input) => input.value.trim() === '') || tools[tool][key].some((v) => !Number.isFinite(v))) {
          message(msg, `${tool} ${key}: enter three numbers.`, true); return;
        }
      }
    }
    if (!Object.keys(values).length && !toolNames.length) { message(msg, 'Keep at least one group.', true); return; }
    const body = toolNames.length ? { joints: values, tools } : { joints: values };
    if (isNew) body.create = true; else if (name !== pose.name) body.rename = name;
    const { ok, data } = await api('PUT', `/api/robot/poses/${encodeURIComponent(isNew ? name : pose.name)}`, body);
    if (!ok) { message(msg, (data.problems || [data.error || 'not saved']).join('\n'), true); return; }
    await loadRobot();
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
    const { ok, data } = await api('DELETE', `/api/robot/poses/${encodeURIComponent(pose.name)}`);
    if (!ok) {
      deleteButton.classList.remove('confirm'); deleteButton.textContent = 'Delete';
      message(msg, (data.problems || [data.error || 'not deleted']).join('\n'), true);
      return;
    }
    await loadRobot();
  });
  return item;
}
$('btn-add-pose').addEventListener('click', () => {
  if (!state.robotData) return;
  $('pose-list').append(poseItem({ name: '', joints: {}, used_by: [] }, true));
});
