'use strict';
// taskweave HMI, Teach tab, Carry poses: held poses (robots/<robot>/held_poses.yaml), where `move_object to: {pose: <name>}` takes
// a held object: some joints first (e.g. the body), then the object's origin along straight lines through the waypoints (in
// the robot's base frame). Typed in, not captured: a held object cannot be jogged. Uses app.js ($, el, api, message,
// DEG, NAME_RE), robot.js (JOINT_SKIP) and teach.js (teach, showJoint, unshowJoint, jointUnit, numberInput, failText).

const heldOpen = new Set();  // `${type}/${name}` of the open editors, kept over reloads

function renderHeld() {
  const data = teach.data;
  const held = data.held || [];
  const types = [...new Set([...data.object_types, ...held.map((h) => h.object_type)])];
  $('held-list').replaceChildren(...types.map((type) => {
    const list = el('ul', { class: 'points' }, held.filter((h) => h.object_type === type).map((h) => heldItem(h)));
    const add = el('button', { class: 'btn small', text: '+ Carry pose', 'data-edit': '' });
    add.addEventListener('click', () => {
      if (list.querySelector('.held-new')) return;
      const item = heldItem({ object_type: type, name: '', joints: {}, path: [], used_by: [] }, true);
      list.prepend(item);
      item.querySelector('input')?.focus();
    });
    return el('li', { class: 'arm-group' },
      el('div', { class: 'row between wrap' }, el('div', { class: 'sub', text: `Holding ${type}` }), add), list);
  }));
}

function heldSummary(h) {
  const parts = [];
  const joints = Object.keys(h.joints);
  if (joints.length) parts.push(joints.map((j) => `${j} ${showJoint(j, h.joints[j])}${jointUnit(j)}`).join(', '));
  parts.push(`${h.path.length} waypoint${h.path.length === 1 ? '' : 's'}`);
  const last = h.path[h.path.length - 1];
  if (last) parts.push(`to ${last.xyz.map((v) => (+v).toFixed(2)).join(' ')}`);
  return parts.join(' · ');
}

function heldItem(h, isNew = false) {
  const key = `${h.object_type}/${h.name}`;
  const url = (name) => `/api/teach/held/${encodeURIComponent(h.object_type)}/${encodeURIComponent(name)}`;
  const head = el('button', { class: 'point-head', 'aria-expanded': 'false' },
    el('b', { text: isNew ? 'New carry pose' : h.name }), el('span', { class: 'mono', text: isNew ? '' : heldSummary(h) }),
    h.used_by.length ? el('span', { class: 'used', text: `used by ${h.used_by.join(', ')}` }) : '');
  const msg = el('div', { class: 'msg', hidden: '' });

  // joints moved first: one row each, removable; more from the robot's joints
  const jointBox = el('div', { class: 'grid3' });
  const jointInputs = {};
  const addJointRow = (joint, value) => {
    const [field, input] = numberInput(`${joint} ${jointUnit(joint)}`, showJoint(joint, value));
    const drop = el('button', { class: 'icon', text: '✕', title: `Leave ${joint} out`, 'aria-label': `Leave ${joint} out`, 'data-edit': '' });
    drop.addEventListener('click', () => { field.remove(); delete jointInputs[joint]; fillJointPicker(); edited(); });
    field.querySelector('label').append(' ', drop);
    jointInputs[joint] = input;
    jointBox.append(field);
  };
  Object.entries(h.joints).forEach(([joint, value]) => addJointRow(joint, value));
  const jointPicker = el('select', { 'aria-label': 'Add a joint' });
  const fillJointPicker = () => {
    const names = Object.keys(teach.data.joint_limits).filter((j) => !JOINT_SKIP.test(j) && !(j in jointInputs)).sort();
    jointPicker.replaceChildren(el('option', { value: '', text: '+ joint…' }), ...names.map((j) => el('option', { value: j, text: j })));
  };
  fillJointPicker();
  jointPicker.addEventListener('change', async () => {
    const joint = jointPicker.value;
    if (!joint) return;
    const { ok, data: now } = await api('GET', '/api/robot/joints');
    addJointRow(joint, ok ? now[joint] ?? 0 : 0);  // start from where the joint is now
    fillJointPicker();
    edited();
  });

  // waypoints of the object's origin: x y z (m), roll pitch yaw (°); up, down, remove
  const pathBox = el('ol', { class: 'waypoints' });
  let rows = [];
  const readPath = () => rows.map((r) => ({ xyz: r.xyz.map((i) => i.value), rpy: r.rpy.map((i) => i.value) }));
  const drawPath = (path) => {  // path values as shown (m, °)
    rows = [];
    pathBox.replaceChildren(...path.map((wp, index) => {
      const xyz = ['x', 'y', 'z'].map((a, i) => numberInput(`${a} (m)`, wp.xyz[i]));
      const rpy = ['roll', 'pitch', 'yaw'].map((a, i) => numberInput(`${a} (°)`, wp.rpy[i]));
      rows.push({ xyz: xyz.map(([, i]) => i), rpy: rpy.map(([, i]) => i) });
      const button = (text, label, action, disabled) => {
        const b = el('button', { class: 'icon', text, title: label, 'aria-label': label, 'data-edit': '' });
        b.disabled = disabled;
        b.addEventListener('click', () => { const p = readPath(); action(p); drawPath(p); edited(); });
        return b;
      };
      return el('li', { class: 'joint-group' },
        el('div', { class: 'row between' }, el('label', { text: `Waypoint ${index + 1}` }), el('div', { class: 'row gap-s' },
          button('↑', 'Move up', (p) => p.splice(index - 1, 0, ...p.splice(index, 1)), index === 0),
          button('↓', 'Move down', (p) => p.splice(index + 1, 0, ...p.splice(index, 1)), index === path.length - 1),
          button('✕', 'Remove', (p) => p.splice(index, 1), false))),
        el('div', { class: 'grid3' }, ...xyz.map(([f]) => f)), el('div', { class: 'grid3' }, ...rpy.map(([f]) => f)));
    }));
  };
  drawPath(h.path.map((wp) => ({ xyz: wp.xyz.map((v) => +(+v).toFixed(4)), rpy: wp.rpy.map((v) => +(v * DEG).toFixed(2)) })));
  const addWaypoint = el('button', { class: 'btn small', text: '+ Waypoint', 'data-edit': '' });
  addWaypoint.addEventListener('click', () => {
    const p = readPath();
    p.push(p.length ? { ...p[p.length - 1] } : { xyz: [0.5, 0, 1.0], rpy: [0, 0, 0] });  // a copy of the last one
    drawPath(p);
    edited();
  });

  const nameInput = el('input', { autocomplete: 'off', spellcheck: 'false', placeholder: 'e.g. carry_high' });
  nameInput.value = h.name;
  const test = el('button', { class: 'btn', text: 'Test (holding one)', 'data-edit': '' });
  const save = el('button', { class: 'btn primary', text: 'Save', 'data-edit': '' });
  const remove = el('button', { class: 'btn danger', text: isNew ? 'Cancel' : 'Delete', 'data-edit': '' });
  test.hidden = isNew;
  const edit = el('div', { class: 'point-edit' },
    el('div', { class: 'row gap' }, test),
    el('div', { class: 'field' }, el('label', { text: 'Name' }), nameInput),
    el('div', { class: 'joint-group' }, el('label', { text: 'Joints, moved first' }), jointBox, jointPicker),
    el('p', { class: 'muted small-text', text: `Then the held ${h.object_type}'s origin moves along straight lines through `
      + 'these waypoints, in the robot\'s base frame.' }),
    pathBox, el('div', { class: 'row gap' }, addWaypoint), msg, el('div', { class: 'row gap' }, remove, save));
  let changed = false;  // Test runs the saved pose
  function edited() { changed = true; test.disabled = true; test.title = 'Save first'; }
  edit.addEventListener('input', edited);

  test.addEventListener('click', async () => {
    test.disabled = true;
    message(msg, `Moving the held ${h.object_type} to ${h.name}…`);
    const { ok, data: result } = await api('POST', `${url(h.name)}/test`, {});
    test.disabled = changed;
    if (!ok) { message(msg, failText(result, 'not moved'), true); return; }
    skillResult(msg, result);
  });
  save.addEventListener('click', async () => {
    const name = nameInput.value.trim();
    if (!NAME_RE.test(name)) { message(msg, 'Names use letters, digits, "_" and "-", starting with a letter.', true); return; }
    const body = { joints: {}, path: [] };
    for (const [joint, input] of Object.entries(jointInputs)) {
      const value = Number(input.value);
      if (input.value.trim() === '' || !Number.isFinite(value)) { message(msg, `${joint}: enter a number.`, true); return; }
      body.joints[joint] = unshowJoint(joint, value);
    }
    for (const [index, r] of rows.entries()) {
      const inputs = [...r.xyz, ...r.rpy];
      const values = inputs.map((i) => Number(i.value));
      if (inputs.some((i) => i.value.trim() === '') || values.some((v) => !Number.isFinite(v))) {
        message(msg, `Waypoint ${index + 1}: six numbers.`, true); return;
      }
      body.path.push({ xyz: values.slice(0, 3), rpy: values.slice(3).map((v) => v / DEG) });
    }
    if ((isNew || name !== h.name) && (teach.data.held || []).some((o) => o.object_type === h.object_type && o.name === name)) {
      message(msg, `${name} already exists for ${h.object_type}.`, true); return;
    }
    if (!isNew && name !== h.name) body.rename = name;
    const { ok, data: result } = await api('PUT', url(isNew ? name : h.name), body);
    if (!ok) { message(msg, failText(result, 'not saved'), true); return; }
    heldOpen.delete(key);
    heldOpen.add(`${h.object_type}/${name}`);
    await loadArm();
  });
  let armed = null;
  remove.addEventListener('click', async () => {
    if (isNew) { item.remove(); return; }
    if (!armed) {  // two taps: no browser dialogs
      remove.classList.add('confirm'); remove.textContent = 'Tap again to delete';
      armed = setTimeout(() => { remove.classList.remove('confirm'); remove.textContent = 'Delete'; armed = null; }, 3000);
      return;
    }
    clearTimeout(armed); armed = null;
    const { ok, data: result } = await api('DELETE', url(h.name));
    if (!ok) { remove.classList.remove('confirm'); remove.textContent = 'Delete'; message(msg, failText(result, 'not deleted'), true); return; }
    heldOpen.delete(key);
    await loadArm();
  });

  const item = el('li', { class: `point${isNew ? ' held-new' : ''}` }, head, edit);
  const open = (opened) => {
    edit.hidden = !opened;
    head.setAttribute('aria-expanded', String(opened));
    if (!isNew) { if (opened) heldOpen.add(key); else heldOpen.delete(key); }
  };
  head.addEventListener('click', () => open(edit.hidden));
  open(isNew || heldOpen.has(key));
  return item;
}
