'use strict';
// taskweave HMI, Program tab: the mission graph editor on graph-core.js. The mission file is the model: the editor
// keeps a working copy and every change is an op (insert, remove, move, wrap, unwrap, set) on a path in it, applied
// here at once and sent to the server at Save (POST /api/missions/<name>/edit), which applies the same ops on the
// file's own tree, so comments stay, validates and writes. Nodes are steps (an icon, the instruction, the command),
// frames hold includes (linked, read-only here) and, with schema v2, groups, repeats and parallel lanes. The
// interaction is a node editor's (Nuke, Blender): click selects, shift-click extends, a box selects several; the
// properties panel at the right edits the selected step with the step form (app.js stepCard) or the mission; "+" on
// a wire (or Shift+A) opens the palette of skills, missions and blocks; a node is dragged onto a wire to move it; the
// failed port (the red dot at the right) is dragged to the step a replan restarts from, or clicked for its rule;
// Delete removes, Ctrl+Z and Ctrl+Y undo and redo. A structural edit keeps every replan_from on the step it named.
// Uses $, el, api, state, show, message, markDirty, stepCard, markField, moveSummary, argsSummary, dropDirty,
// clearDrop, openEditor, loadMissions, showEditor from app.js; iconElement, skillIcon from icons.js; div, colourOf,
// iconOf, forgetMission from graph-run.js; GRAPH_METRICS, fillLeaves, layoutGraph, drawGraph, gsvg from graph-core.js.

const BLOCK_KINDS = ['group', 'repeat', 'parallel'];
const BLOCK_WORDS = { group: 'Group', repeat: 'Repeat', parallel: 'In parallel' };
const BLOCK_GLYPH = { group: '▣', repeat: '↻', parallel: '⫴', include: '⛓' };
const CARD_KEYS = ['instruction', 'skill', 'args', 'include', 'with', 'id', 'timeout_s', 'retries', 'on_failure', 'replan_from'];
const MISSION_KEYS = { 'm-goal': 'goal', 'm-loop': 'loop', 'm-max': 'max_cycles', 'm-stop': 'stop_when' };
const DRAG_PX = 6;      // a press that moves less is a click
const SPOT_REACH = 56;  // px: a dragged node snaps to the nearest "+" within this
const ed = { mission: null, name: null, isNew: false, ops: [], undo: [], redo: [], selected: [], closed: new Set(),
  opened: new Set(), includes: {}, blocks: false, leaves: [], layout: null, drawn: null, card: null, cardPath: null,
  marks: new Map(), notes: new Map(), savedUndo: null, palette: null, menu: null, base: null };
const clone = (v) => (v === undefined ? v : JSON.parse(JSON.stringify(v)));
// a value as the server keeps it: no null keys, at any depth (the file has no place for them)
const strip = (v) => (Array.isArray(v) ? v.map(strip) : v && typeof v === 'object'
  ? Object.fromEntries(Object.entries(v).filter(([, x]) => x !== null && x !== undefined).map(([k, x]) => [k, strip(x)])) : v);
const same = (a, b) => JSON.stringify(a ?? null) === JSON.stringify(b ?? null);
const blockKind = (step) => BLOCK_KINDS.find((k) => k in (step || {})) || '';
const pathParts = (path) => String(path || '').split('/').filter(Boolean);
const slotOf = (path) => { const p = pathParts(path); return { list: p.slice(0, -1).join('/'), index: Number(p[p.length - 1]) }; };
const joinPath = (list, index) => `${list}/${index}`;
const inside = (path, container) => path === container || path.startsWith(`${container}/`);
function nodeAt(mission, path) {
  let node = mission;
  for (const p of pathParts(path)) node = Array.isArray(node) ? node[Number(p)] : node?.[p];
  return node;
}
function listAt(mission, list, make = false) {
  const p = pathParts(list);
  const parent = p.length > 1 ? nodeAt(mission, p.slice(0, -1).join('/')) : mission;
  const key = p[p.length - 1];
  if (!parent[key] && make) parent[key] = [];
  return parent[key];
}

// ---------- the ops, applied here as the server applies them ----------
function applyOp(m, op) {
  if (op.op === 'insert') {
    const { list, index } = slotOf(op.at);
    listAt(m, list, true).splice(index, 0, strip(clone(op.step)));
  } else if (op.op === 'remove') {
    const { list, index } = slotOf(op.at);
    listAt(m, list).splice(index, 1);
  } else if (op.op === 'move') {
    const from = slotOf(op.from);
    const to = slotOf(op.to);
    const src = listAt(m, from.list);
    const dst = listAt(m, to.list, true);
    const [item] = src.splice(from.index, 1);
    dst.splice(dst === src && to.index > from.index ? to.index - 1 : to.index, 0, item);
  } else if (op.op === 'wrap') {
    const from = slotOf(op.from);
    const to = slotOf(op.to);
    const list = listAt(m, from.list);
    const inner = list.splice(from.index, to.index - from.index + 1);
    const block = strip({ ...clone(op.block) });
    if (op.as === 'group') block.group = true;
    if (op.as === 'repeat') block.repeat = block.repeat && typeof block.repeat === 'object' && !Array.isArray(block.repeat) ? block.repeat : {};
    if (op.as === 'parallel') block.parallel = [{ steps: inner }]; else block.steps = inner;
    list.splice(from.index, 0, block);
  } else if (op.op === 'unwrap') {
    const { list, index } = slotOf(op.at);
    const arr = listAt(m, list);
    const block = arr[index];
    const steps = block.parallel ? block.parallel.flatMap((lane) => lane.steps || []) : block.steps || [];
    arr.splice(index, 1, ...steps);
  } else if (op.op === 'set') {
    const node = op.at ? nodeAt(m, op.at) : m;
    if (op.value === null || op.value === undefined) delete node[op.key]; else node[op.key] = strip(clone(op.value));
  }
}
// Apply ops: one undo entry (tag: a following commit with the same tag joins it, so typing in a field is one undo).
// A structural edit (insert, remove, move, wrap, unwrap) also keeps every replan_from on the step it named (set ops in
// the same entry, sent at Save with the rest), says what it changed, and keeps the keyboard on the graph: the node at
// the selection, else the one nearest the change.
function commit(ops, { tag = null, select } = {}) {
  if (!ops.length) return;
  const structural = ops.some((op) => op.op !== 'set');
  const last = ed.undo[ed.undo.length - 1];
  if (!(tag && last && last.tag === tag)) ed.undo.push({ mission: clone(ed.mission), ops: ed.ops.length, selected: [...ed.selected], tag });
  ed.redo = [];
  const refs = structural ? replanRefs(ops) : null;
  ops.forEach((op) => applyOp(ed.mission, op));
  const fixed = refs ? replanFixes(refs) : { ops: [], notes: [] };
  fixed.ops.forEach((op) => applyOp(ed.mission, op));
  ed.ops.push(...ops, ...fixed.ops);
  ops.forEach((op) => { if (op.at !== undefined) ed.marks.delete(op.at); });
  if (structural) {  // paths shifted
    ed.marks.clear(); ed.notes.clear(); ed.closed.clear(); ed.opened.clear();
    fixed.notes.forEach(({ path, text }) => ed.notes.set(path, [...(ed.notes.get(path) || []), text]));
    if (fixed.notes.length) message($('save-msg'), fixed.notes.map((n) => n.text).join(' '), 'warn');
    else if ($('save-msg').classList.contains('warn')) message($('save-msg'), '');  // the last such note is old now
    const active = document.activeElement;
    ed.focusNear = changedAt(ops.every((op) => op.op === 'remove') ? ops[ops.length - 1] : ops[0]);  // removes: back to front
    ed.refocus = $('ed-canvas').contains(active) || !!active?.closest?.('.ed-actions, .ed-palette, .ed-menu');
  }
  if (select !== undefined) ed.selected = select;
  markDirty();
  renderEditor();
  if (select !== undefined || structural) renderPanel();
  tools();
}
// where an op changed the mission: the path the nearest node is looked for from
const changedAt = (op) => op.at ?? (op.op === 'move' ? op.to : op.from) ?? '';

// ---------- replan_from follows its step ----------
// Every own step of a mission (blocks too, not the steps of an include) with its path, in document order.
function ownSteps(m) {
  const out = [];
  const walk = (steps, list) => (steps || []).forEach((step, i) => {
    const path = joinPath(list, i);
    out.push({ step, path });
    if (step?.parallel) step.parallel.forEach((lane, j) => walk(lane.steps, `${path}/parallel/${j}/steps`));
    else if (step && !step.include) walk(step.steps, `${path}/steps`);
  });
  walk(m.steps, 'steps');
  return out;
}
// The step a replan_from names in a mission: the step with that id, else for s<n> the n-th of the mission's own top
// level steps when it has no id (core's fallback for a step without one; numbered from 1), else null.
function replanTarget(m, ref, all = ownSteps(m)) {
  const byId = all.find((n) => n.step?.id === ref);
  if (byId) return byId.step;
  const place = /^s(\d+)$/.exec(ref || '');
  const top = place && m.steps?.[place[1] - 1];
  return top && !top.id ? top : null;
}
// Before a structural edit: every step with a replan_from and the step it names now. A step the edit inserts (a
// duplicate) names a step of the mission as it is before the insert, so its place is read now too.
function replanRefs(ops) {
  const all = ownSteps(ed.mission);
  const refs = all.filter((n) => n.step?.replan_from)
    .map((n) => ({ step: n.step, ref: n.step.replan_from, target: replanTarget(ed.mission, n.step.replan_from, all) }));
  const insert = ops.length === 1 && ops[0].op === 'insert' ? ops[0] : null;
  if (insert) {
    refs.inserted = { at: insert.at, refs: ownSteps({ steps: [insert.step] })
      .map((n) => (n.step?.replan_from ? { ref: n.step.replan_from, target: replanTarget(ed.mission, n.step.replan_from, all) } : null)) };
  }
  return refs;
}
// After it: set ops that keep each replan_from on its step (a place number that moved becomes the new one, or an id
// when the step left the top level), and clear one whose step was removed (with on_failure: replan, back to the
// mission's default). notes: [{path, text}] for the editor to say what changed.
function replanFixes(refs) {
  const all = ownSteps(ed.mission);
  const pathOf = new Map(all.map((n) => [n.step, n.path]));
  const ops = [];
  const notes = [];
  const ids = new Map();  // a target given an id here -> the id
  const list = [...refs];
  if (refs.inserted) {  // the inserted copy: its steps in the same order as the op's
    const copy = ownSteps({ steps: [nodeAt(ed.mission, refs.inserted.at)] });
    refs.inserted.refs.forEach((r, k) => { if (r && copy[k]) list.push({ step: copy[k].step, ...r }); });
  }
  list.forEach(({ step, ref, target }) => {
    const at = pathOf.get(step);
    if (at === undefined || step.replan_from !== ref || !target) return;  // removed, set in this edit, or named nothing
    const name = `'${step.id || step.instruction || at}'`;
    if (!pathOf.has(target)) {
      ops.push({ op: 'set', at, key: 'replan_from', value: null });
      if (step.on_failure === 'replan') ops.push({ op: 'set', at, key: 'on_failure', value: null });
      const fallback = step.on_failure === 'replan' ? `does the mission default (${ed.mission.defaults?.on_failure || 'abort'})`
        : 'replans from itself';  // its rule is the mission's replan
      notes.push({ path: at, text: `The replan target of ${name} was removed: on failure it ${fallback} now.` });
      return;
    }
    if (replanTarget(ed.mission, ref, all) === target) return;  // still names it
    const top = (ed.mission.steps || []).indexOf(target);
    let value = top >= 0 && !target.id ? `s${top + 1}` : target.id || ids.get(target);
    if (!value) {
      value = freshId(target, [...ids.values()]);
      ids.set(target, value);
      ops.push({ op: 'set', at: pathOf.get(target), key: 'id', value });
    }
    ops.push({ op: 'set', at, key: 'replan_from', value });
    notes.push({ path: at, text: `replan_from of ${name} is ${value} now (was ${ref}).` });
  });
  return { ops, notes };
}
function undoEdit() {
  const entry = ed.undo.pop();
  if (!entry) return;
  const undone = ed.ops.slice(entry.ops);
  ed.redo.push({ mission: clone(ed.mission), ops: ed.ops.length, selected: [...ed.selected], after: undone });
  ed.mission = entry.mission;
  ed.ops = ed.ops.slice(0, entry.ops);
  ed.selected = entry.selected.filter((path) => nodeAt(ed.mission, path));
  afterHistory(undone[0]);
}
function redoEdit() {
  const entry = ed.redo.pop();
  if (!entry) return;
  ed.undo.push({ mission: clone(ed.mission), ops: ed.ops.length, selected: [...ed.selected], tag: null });
  entry.after.forEach((op) => applyOp(ed.mission, op));
  ed.ops.push(...entry.after);
  ed.selected = entry.selected.filter((path) => nodeAt(ed.mission, path));
  afterHistory(entry.after[0]);
}
// after an undo or redo: the keyboard stays on the graph, at the selection, else at the node nearest the change
function afterHistory(op) {
  ed.focusNear = op ? changedAt(op) : null;
  ed.notes.clear();
  ed.marks.clear();
  markDirty();
  fillMissionFields();
  renderEditor();
  renderPanel();
  tools();
}
// the mission's fields in the panel from the working copy (after undo, redo and a save); no events, no ops
function fillMissionFields() {
  const m = ed.mission;
  $('m-goal').value = m.goal || '';
  $('m-loop').value = m.loop || 'once';
  $('m-max').value = m.max_cycles ?? '';
  $('m-stop').value = m.stop_when ?? '';
  const d = m.defaults || {};
  $('d-timeout').value = d.timeout_s ?? '';
  $('d-retries').value = d.retries ?? '';
  $('d-fail').value = d.on_failure ?? '';
  $('d-replans').value = d.max_replans ?? '';
}
function tools() {
  $('ed-undo').disabled = !ed.undo.length;
  $('ed-redo').disabled = !ed.redo.length;
}

// ---------- the tree from the mission ----------
// Every node has path (its place in this mission's file; a linked node: inside the include it came from, not
// editable) and key = path. Steps count as leaves in document order (ctx.keyOf maps an index to its path).
function buildTree() {
  const leaves = [];
  const nodes = (steps, list, linked) => (steps || []).map((step, i) => {
    const path = joinPath(list, i);
    const base = { key: path, path, step, linked };
    if (step && step.include) {
      const inc = ed.includes[step.include];
      const open = ed.opened.has(path) && inc?.steps;
      return { ...base, kind: 'include', include: step.include, name: step.instruction || step.include,
        children: open ? nodes(inc.steps, `${path}/steps`, true) : [], inner: inc?.steps?.length };
    }
    const kind = blockKind(step);
    if (kind === 'parallel') {
      return { ...base, kind, block: step, name: step.instruction || BLOCK_WORDS.parallel,
        branches: (step.parallel || []).map((lane, j) => ({ kind: 'branch', key: `${path}/parallel/${j}`, path: `${path}/parallel/${j}`,
          lane, linked, name: lane.instruction || `lane ${j + 1}`, children: nodes(lane.steps, `${path}/parallel/${j}/steps`, linked) })) };
    }
    if (kind) return { ...base, kind, block: step, name: step.instruction || BLOCK_WORDS[kind], children: nodes(step.steps, `${path}/steps`, linked) };
    const index = leaves.length;
    leaves.push(path);
    return { ...base, kind: 'step', index, name: step.instruction };
  });
  const root = { kind: 'root', key: 'root', path: '', children: nodes(ed.mission.steps, 'steps', false) };
  root.loop = !!ed.mission.loop && ed.mission.loop !== 'once';
  fillLeaves(root);
  return { root, leaves };
}
const isOpenFrame = (frame) => (frame.kind === 'branch' ? true : frame.kind === 'include' ? ed.opened.has(frame.path) && !!ed.includes[frame.include]?.steps
  : !ed.closed.has(frame.path));
// every step node with its path, in document order, linked ones left out
function allSteps(root = buildTree().root) {
  const out = [];
  const walk = (node) => {
    if (node.linked) return;
    if (node.kind !== 'root' && node.kind !== 'branch') out.push(node);
    (node.children || node.branches || []).forEach(walk);
  };
  walk(root);
  return out;
}
const stepIds = () => allSteps().map((n) => n.step?.id).filter(Boolean);
// a step by its id; s<n> names the n-th of the mission's own steps when that one has no id (core's fallback)
function stepById(id) {
  const all = ownSteps(ed.mission);
  const step = replanTarget(ed.mission, id, all);
  return step ? all.find((n) => n.step === step) : undefined;
}

// the words of the mission's own loop wire and of a repeat frame
function loopWords() {
  const m = ed.mission;
  if (!m.loop || m.loop === 'once') return '';
  return [`↻ ${m.stop_when ? `until ${m.stop_when}` : 'forever'}`, m.max_cycles ? `max ${m.max_cycles}` : ''].filter(Boolean).join(' · ');
}
function repeatWords(frame) {
  const r = frame.block?.repeat || {};
  const rule = r.times ? `${r.times}×` : r.while ? `while ${r.while}` : r.until ? `until ${r.until}` : 'repeat';
  return [`↻ ${rule}`, frame.block?.max_cycles ? `max ${frame.block.max_cycles}` : ''].filter(Boolean).join(' · ');
}
// a step's failure rule in words, with the mission's defaults
function ruleWords(step) {
  const d = ed.mission.defaults || {};
  const retries = step.retries ?? d.retries ?? 0;
  const on = step.on_failure || d.on_failure || 'abort';
  return `${retries > 0 ? `↻${retries} then ` : ''}${on}${on === 'replan' && step.replan_from ? ` from ${step.replan_from}` : ''}`;
}
const commandWords = (step) => (step.include ? `mission ${step.include}` : step.skill || '');
const argWords = (step) => (step.include ? argsSummary(step.with) : step.skill === 'move' ? moveSummary(step.args) : argsSummary(step.args));

// ---------- the nodes and frame heads ----------
function nodeEl(it) {
  const node = it.node;
  const step = node.step;
  const selected = ed.selected.includes(node.path);
  const bad = ed.marks.has(node.path);
  const frame = it.kind === 'stack';
  const kind = frame ? node.kind : colourOf(step);
  const button = el('button', { type: 'button', 'data-path': node.path,
    class: `pg-node${frame ? ` frame ${node.kind}` : ` kind-${kind}`}${selected ? ' selected' : ''}${node.linked ? ' linked' : ''}${bad ? ' has-problem' : ''}`,
    'aria-pressed': String(selected) });
  if (frame) {
    const count = node.kind === 'include' ? (node.inner ?? state.missions.find((m) => m.name === node.include)?.steps) : node.leaves.length;
    button.append(div('pg-node-ico', el('span', { class: 'pg-glyph', text: BLOCK_GLYPH[node.kind] })),
      div('pg-node-text', el('div', { class: `pg-node-title${node.name ? '' : ' empty'}`, text: node.name || 'Describe this block' }),
        el('div', { class: 'pg-node-sub', text: node.kind === 'include' ? `mission ${node.include}${count != null ? ` · ${count} steps` : ''}`
          : `${BLOCK_WORDS[node.kind].toLowerCase()} · ${count} steps` })));
    button.title = node.kind === 'include' ? `${node.name}\nlinked: the mission ${node.include}, shared by the missions that include it\nclick to edit the call, the chevron shows its steps`
      : `${node.name}\n${BLOCK_WORDS[node.kind]} of ${count} steps · closed`;
    const chev = el('span', { class: 'pg-node-chev', role: 'button', tabindex: '-1', title: 'Show the steps inside', 'aria-label': 'Open' });
    chev.addEventListener('pointerdown', (e) => e.stopPropagation());
    chev.addEventListener('click', (e) => { e.stopPropagation(); toggleFrame(node); });
    button.append(chev);
  } else {
    button.append(div('pg-node-ico', iconElement(iconOf(step))),
      div('pg-node-text', el('div', { class: `pg-node-title${step.instruction ? '' : ' empty'}`, text: step.instruction || 'Describe this step' }),
        el('div', { class: 'pg-node-sub', text: [commandWords(step), argWords(step)].filter(Boolean).join(' · ') })));
    button.title = `${step.instruction || 'no instruction yet'}\n${[commandWords(step), argWords(step)].filter(Boolean).join(' · ')}\non failure: ${ruleWords(step)}`;
  }
  if (bad) button.title += `\n${ed.marks.get(node.path).map((m) => m.text).join('\n')}`;
  if (!node.linked) {
    const fail = el('span', { class: `pg-port fail${step.on_failure ? ' set' : ''}`, title: `on failure: ${ruleWords(step)}\nclick for the rule, drag to the step a replan restarts from`, 'aria-hidden': 'true' });
    fail.addEventListener('pointerdown', (e) => failPortDown(e, node));
    fail.addEventListener('click', (e) => e.stopPropagation());  // not a click on the node: the menu would close
    button.append(el('span', { class: 'pg-port in', 'aria-hidden': 'true' }), el('span', { class: 'pg-port done', 'aria-hidden': 'true' }), fail);
  }
  button.addEventListener('pointerdown', (e) => nodeDown(e, node));
  button.addEventListener('click', (e) => nodeClick(e, node));
  button.addEventListener('keydown', (e) => { if (e.key === 'Enter' && frame) toggleFrame(node); });
  button.addEventListener('dblclick', () => { if (frame) toggleFrame(node); });
  return button;
}
function headEl(frame) {
  const lane = frame.kind === 'branch';
  const selected = ed.selected.includes(frame.path);
  const head = div(`pg-band ${frame.kind}${selected ? ' selected' : ''}${frame.linked ? ' linked' : ''}`);
  head.dataset.path = frame.path;
  if (!lane) {
    const chev = el('button', { type: 'button', class: 'pg-band-btn pg-chev-btn', 'aria-expanded': 'true', 'aria-label': `Close ${frame.name}`, title: 'Close' },
      el('span', { class: 'pg-chev', 'aria-hidden': 'true' }));
    chev.addEventListener('click', (e) => { e.stopPropagation(); toggleFrame(frame); });
    head.append(chev);
  }
  if (frame.kind === 'include') head.append(el('span', { class: 'pg-link', title: 'linked: shared with other missions', text: '⛓' }));
  const name = el('button', { type: 'button', class: 'pg-band-btn pg-band-name', text: frame.name, title: frame.name });
  name.addEventListener('click', (e) => { e.stopPropagation(); if (!frame.linked) select(lane ? pathParts(frame.path).slice(0, -2).join('/') : frame.path, e); });
  const meta = frame.kind === 'include' ? `linked · ${frame.include}` : frame.kind === 'repeat' ? repeatWords(frame)
    : frame.kind === 'parallel' ? `${frame.branches.length} lanes · join ${frame.block.join || 'all'}` : lane ? '' : `${frame.leaves.length} steps`;
  head.append(name, el('span', { class: 'pg-band-meta', text: meta }));
  if (frame.kind === 'include' && !frame.linked) {
    const open = el('button', { type: 'button', class: 'pg-band-btn pg-band-open', text: 'Open', title: `Edit the mission ${frame.include}` });
    open.addEventListener('click', (e) => { e.stopPropagation(); openIncluded(frame.include); });
    head.append(open);
  }
  if (frame.kind === 'parallel' && !frame.linked) {
    const add = el('button', { type: 'button', class: 'pg-band-btn pg-band-open', text: '+ lane', title: 'Add a lane' });
    add.addEventListener('click', (e) => { e.stopPropagation(); commit([{ op: 'insert', at: `${frame.path}/parallel/${frame.branches.length}`, step: { steps: [] } }]); });
    head.append(add);
  }
  head.addEventListener('pointerdown', (e) => { if (!lane && !frame.linked && !e.target.closest('button')) nodeDown(e, frame); });
  return head;
}
async function toggleFrame(frame) {
  if (frame.kind === 'include') {
    if (ed.opened.has(frame.path)) ed.opened.delete(frame.path);
    else {
      ed.opened.add(frame.path);
      if (!ed.includes[frame.include]) {
        ed.includes[frame.include] = {};
        const { ok, data } = await api('GET', `/api/missions/${encodeURIComponent(frame.include)}`);
        ed.includes[frame.include] = ok ? data : { steps: [] };
      }
    }
  } else if (ed.closed.has(frame.path)) ed.closed.delete(frame.path); else ed.closed.add(frame.path);
  renderEditor();
}
function openIncluded(name) {
  const go = () => openEditor(name);
  if (state.dirty) dropDirty($('ed-canvas'), go); else go();
}

// ---------- selection ----------
function select(path, e = {}) {
  const node = nodeAt(ed.mission, path);
  if (!node) return;
  if (slotOf(path).list.endsWith('/parallel')) { select(pathParts(path).slice(0, -2).join('/'), e); return; }  // a lane: its block
  const list = slotOf(path).list;
  if (e.shiftKey && ed.selected.length && slotOf(ed.selected[0]).list === list) {
    const a = slotOf(ed.selected[0]).index;
    const b = slotOf(path).index;
    ed.selected = [];
    for (let i = Math.min(a, b); i <= Math.max(a, b); i += 1) ed.selected.push(joinPath(list, i));
    if (a > b) ed.selected.reverse();
  } else if ((e.ctrlKey || e.metaKey) && ed.selected.length && slotOf(ed.selected[0]).list === list) {
    ed.selected = ed.selected.includes(path) ? ed.selected.filter((p) => p !== path) : [...ed.selected, path];
  } else ed.selected = [path];
  renderEditor();
  renderPanel();
}
function clearSelection() {
  if (!ed.selected.length) return;
  ed.selected = [];
  renderEditor();
  renderPanel();
}
function nodeClick(e, node) {
  if (ed.dragged) { ed.dragged = false; return; }  // the pointer-up of a drag
  if (node.linked) { const owner = ownerInclude(node.path); if (owner) select(owner, e); return; }
  select(node.path, e);
}
// the include frame a linked node sits in (the deepest path of this mission's own tree)
function ownerInclude(path) {
  let p = path;
  while (p) {
    const node = nodeAt(ed.mission, p);
    if (node?.include) return p;
    p = pathParts(p).slice(0, -2).join('/');
  }
  return null;
}
// the selection as one run in one list, in order: {list, first, last} or null
function selectedRun() {
  if (!ed.selected.length) return null;
  const list = slotOf(ed.selected[0]).list;
  const idx = ed.selected.map(slotOf);
  if (idx.some((s) => s.list !== list)) return null;
  const first = Math.min(...idx.map((s) => s.index));
  const last = Math.max(...idx.map((s) => s.index));
  if (last - first + 1 !== idx.length) return null;
  return { list, first, last };
}

// ---------- the editor's drawing: the graph, the "+" spots ----------
function renderEditor() {
  const canvas = $('ed-canvas');
  if (!ed.mission || $('editor').hidden) return;
  const focused = canvas.contains(document.activeElement) || ed.refocus;
  const near = ed.focusNear;
  const had = canvas.contains(document.activeElement) ? document.activeElement.closest('[data-path]')?.dataset.path : null;  // a redraw keeps it
  ed.refocus = false;
  ed.focusNear = null;
  const { root, leaves } = buildTree();
  ed.leaves = leaves;
  closePalette();
  closeMenu();
  if (!root.children.length) {
    const add = el('button', { type: 'button', class: 'btn primary ed-first', text: '+ Add the first step' });
    add.addEventListener('click', () => openPalette({ list: 'steps', index: 0 }, canvas.clientWidth / 2 - 150, 80));
    canvas.replaceChildren(div('ed-empty', el('p', { class: 'muted', text: 'This mission has no steps yet.' }), add));
    if (focused) add.focus({ preventScroll: true });
    return;
  }
  const replans = allSteps(root).filter((n) => n.step.on_failure === 'replan' && n.step.replan_from)
    .map((n) => [n.path, stepById(n.step.replan_from)?.path]).filter(([, to]) => to);
  const ctx = { metrics: GRAPH_METRICS.editor, width: Math.max(320, canvas.clientWidth - 2), isOpen: isOpenFrame,
    keyOf: (i) => leaves[i], stateOf: () => 'succeeded', frameWord: () => '', renderItem: nodeEl, renderHead: headEl,
    labelOf: () => '', replans, start: null, loop: loopWords(), repeatText: repeatWords, label: 'Mission graph editor', ids: 'pg-ed' };
  const layout = layoutGraph(root, ctx);
  const drawn = drawGraph(layout, ctx);
  addSpots(drawn, layout, root);
  canvas.replaceChildren(drawn.node);
  ed.layout = layout;
  ed.drawn = drawn;
  // the node to keep in sight (and on the keyboard): the selection, else the one nearest the last change, else the
  // one that had the focus, else the first; a graph wider than the canvas (lanes on a phone) scrolls it in, the start of a mission to its first node
  const target = canvas.querySelector('.pg-node.selected, .pg-band.selected .pg-band-name') || nearestNode(near)
    || nearestNode(had) || canvas.querySelector('.pg-node');
  if (target && (focused || ed.selected.length || ed.reveal)) keepInView(canvas, target.closest('.pg-node, .pg-band'));
  ed.reveal = false;
  if (focused) target?.focus({ preventScroll: true });
}
// the element of the node at a path, else of the one before it in its list, else of the frame that holds it
function nearestNode(path) {
  const canvas = $('ed-canvas');
  let p = path;
  while (p) {
    const found = canvas.querySelector(`.pg-node[data-path="${CSS.escape(p)}"]`)
      || canvas.querySelector(`.pg-band[data-path="${CSS.escape(p)}"] .pg-band-name`);
    if (found) return found;
    const { list, index } = slotOf(p);
    p = index > 0 ? joinPath(list, index - 1) : pathParts(list).slice(0, -1).join('/');  // a lane, then its block
  }
  return null;
}
// scroll the canvas (not the page) so a node is in sight: only the way it is out, with a margin
function keepInView(canvas, node) {
  if (!node) return;
  const c = canvas.getBoundingClientRect();
  const n = node.getBoundingClientRect();
  const margin = 24;
  if (n.left < c.left + margin || n.right > c.right - margin) {
    canvas.scrollLeft += n.left + n.width / 2 - (c.left + canvas.clientWidth / 2);  // centred across
  }
  if (n.top < c.top + margin) canvas.scrollTop += n.top - c.top - margin;
  else if (n.bottom > c.top + canvas.clientHeight - margin) canvas.scrollTop += n.bottom - (c.top + canvas.clientHeight) + margin;
}
// Where a step can go: on every wire (before the item the wire leads to, at the level the wire runs), at the end of
// every open frame, in an empty frame and lane. A spot: {list, index, x, y}.
function spotsOf(drawn, layout, root) {
  const spots = [];
  const seen = new Set();
  const add = (list, index, x, y) => { const k = `${list}/${index}`; if (seen.has(k)) return; seen.add(k); spots.push({ list, index, x, y }); };
  const stepPath = (it) => (it.node.kind === 'branch' ? null : it.node.path);
  drawn.flow.forEach(({ a, b, mid }) => {
    const A = a ? (a.node.linked ? ownerInclude(stepPath(a)) : stepPath(a)) : '';  // out of an opened include: the include
    const B = b ? stepPath(b) : null;
    if (A === null) return;
    if (B === null) { add('steps', (ed.mission.steps || []).length, mid.x, mid.y); return; }
    if (b.node.linked) {  // into an opened include: before the include itself
      const owner = ownerInclude(B);
      if (owner && !inside(A, owner)) { const s = slotOf(owner); add(s.list, s.index, mid.x, mid.y); }
      return;
    }
    if (A === '') { add('steps', 0, mid.x, mid.y); return; }
    if (inside(B, A)) { const s = slotOf(B); add(s.list, s.index, mid.x, mid.y); return; }   // down into a block
    if (inside(A, B)) { const s = slotOf(A); add(s.list, s.index + 1, mid.x, mid.y); return; }   // up out of a block
    // the common list: B's ancestor beside A's
    const bp = pathParts(B);
    let depth = bp.length;
    while (depth > 2 && !inside(A, bp.slice(0, depth - 2).join('/'))) depth -= 2;
    const target = bp.slice(0, depth).join('/');
    const s = slotOf(target);
    add(s.list, s.index, mid.x, mid.y);
  });
  layout.frames.forEach((f) => {
    const node = f.node;
    if (node.linked || node.kind === 'include') return;
    if (node.kind === 'parallel') return;
    const list = `${node.path}/steps`;
    const n = (node.children || []).length;
    if (!n) add(list, 0, f.x + f.w / 2, f.y + f.h / 2 + (node.kind === 'branch' ? 0 : 8));
    else add(list, n, f.x + f.w / 2, f.y + f.h - GRAPH_METRICS.editor.pad / 2 - 2);
  });
  return spots;
}
function addSpots(drawn, layout, root) {
  ed.spots = spotsOf(drawn, layout, root);
  ed.spots.forEach((spot) => {
    const button = el('button', { type: 'button', class: 'ed-spot', text: '+', title: 'Add a step here', 'aria-label': 'Add a step here' });
    button.style.left = `${spot.x}px`;
    button.style.top = `${spot.y}px`;
    button.addEventListener('click', (e) => { e.stopPropagation(); openPalette(spot, spot.x + 16, spot.y - 20); });
    button.addEventListener('pointerdown', (e) => e.stopPropagation());
    spot.el = button;
    drawn.node.append(button);
  });
}

// ---------- the palette: what to add ----------
function paletteEntries() {
  const self = ed.name;
  const skills = state.skills.map((s) => ({ kind: 'skill', name: s.name, text: (s.description || '').split(/\.\s/)[0],
    icon: skillIcon({ skill: s.name, args: {} }) }));
  const missions = state.missions.filter((m) => m.valid && m.name !== self).map((m) => ({ kind: 'include', name: m.name, text: m.goal, icon: '⛓' }));
  const blocks = ed.blocks ? BLOCK_KINDS.map((k) => ({ kind: k, name: BLOCK_WORDS[k], icon: BLOCK_GLYPH[k],
    text: { group: 'steps that belong together', repeat: 'steps run again while a condition holds', parallel: 'lanes that run at the same time' }[k] })) : [];
  return [...skills, ...missions, ...blocks];
}
function stepFor(entry) {
  if (entry.kind === 'skill') return { instruction: '', skill: entry.name, args: {} };
  if (entry.kind === 'include') return { instruction: '', include: entry.name, with: {} };
  if (entry.kind === 'parallel') return { instruction: '', parallel: [{ steps: [] }, { steps: [] }] };
  if (entry.kind === 'repeat') return { instruction: '', repeat: { times: 2 }, steps: [] };
  return { instruction: '', group: true, steps: [] };
}
function openPalette(spot, x, y) {
  closePalette();
  const canvas = $('ed-canvas');
  const box = div('ed-palette');
  const search = el('input', { type: 'search', placeholder: 'Skill, mission or block', 'aria-label': 'What to add', autocomplete: 'off', spellcheck: 'false' });
  const list = el('ul', { class: 'ed-palette-list', role: 'listbox', 'aria-label': 'Skills, missions and blocks' });
  const entries = paletteEntries();
  const fill = () => {
    const words = search.value.trim().toLowerCase().split(/\s+/).filter(Boolean);
    list.replaceChildren(...entries.filter((e) => words.every((w) => `${e.name} ${e.text}`.toLowerCase().includes(w))).map((e) => {
      const item = el('li', { role: 'option', tabindex: '-1' },
        typeof e.icon === 'string' && e.icon.length <= 2 ? el('span', { class: 'ed-pal-glyph', text: e.icon }) : iconElement(e.icon, 'ed-pal-icon'),
        div('ed-pal-text', el('b', { text: e.name }), el('span', { class: 'muted', text: e.text || '' })));
      item.addEventListener('click', () => pickEntry(e, spot));
      item.addEventListener('keydown', (k) => { if (k.key === 'Enter') pickEntry(e, spot); });
      return item;
    }));
    if (!list.children.length) list.append(el('li', { class: 'muted', text: 'nothing matches' }));
    choose(list.querySelector('li[role="option"]'));
  };
  // the active option (aria-selected): the focused one, or while the search has the focus the first, which Enter takes
  const choose = (item) => list.querySelectorAll('li[role="option"]').forEach((li) => li.setAttribute('aria-selected', String(li === item)));
  list.addEventListener('focusin', (e) => choose(e.target.closest('li[role="option"]')));
  search.addEventListener('focus', () => choose(list.querySelector('li[role="option"]')));
  search.addEventListener('input', fill);
  box.addEventListener('keydown', (e) => {
    if (e.key === 'Escape') { e.stopPropagation(); closePalette(); return; }
    if (e.key === 'Enter' && e.target === search) { list.querySelector('li[role="option"]')?.click(); return; }
    if (e.key === 'ArrowDown' || e.key === 'ArrowUp') {
      e.preventDefault();
      const items = [...list.querySelectorAll('li[role="option"]')];
      const at = items.indexOf(document.activeElement);
      (e.key === 'ArrowDown' ? items[at + 1] || items[0] : at <= 0 ? search : items[at - 1])?.focus();
    }
  });
  box.append(search, list);
  fill();
  const graph = canvas.querySelector('.pg-graph') || canvas;
  box.style.left = `${Math.max(4, Math.min(x, (ed.layout?.width || canvas.clientWidth) - 304))}px`;
  box.style.top = `${Math.max(4, y)}px`;
  graph.append(box);
  if (graph !== canvas) graph.style.height = `${Math.max(graph.offsetHeight, y + 380)}px`;
  ed.palette = box;
  search.focus();
}
function closePalette() { ed.palette?.remove(); ed.palette = null; }
function pickEntry(entry, spot) {
  closePalette();
  const at = joinPath(spot.list, spot.index);
  commit([{ op: 'insert', at, step: stepFor(entry) }], { select: [at] });
  $('ed-sel').querySelector('[data-k="instruction"], input, textarea')?.focus();
}
// the spot after the selection (or the end of the mission) for the keyboard
function spotAfterSelection() {
  const run = selectedRun();
  if (run) return { list: run.list, index: run.last + 1 };
  return { list: 'steps', index: (ed.mission.steps || []).length };
}
function paletteAtSelection() {
  const spot = spotAfterSelection();
  const near = ed.spots?.find((s) => s.list === spot.list && s.index === spot.index);
  openPalette(spot, near ? near.x + 16 : 40, near ? near.y - 20 : 40);
}

// ---------- the failed port: the rule, or a drag to the replan step ----------
function failPortDown(e, node) {
  e.stopPropagation();
  e.preventDefault();
  if (e.button !== 0) return;
  const canvas = $('ed-canvas');
  const graph = canvas.querySelector('.pg-graph');
  const port = e.currentTarget;
  const start = { x: e.clientX, y: e.clientY };
  let line = null;
  let target = null;
  const origin = () => { const r = port.getBoundingClientRect(); const g = graph.getBoundingClientRect(); return { x: r.left + r.width / 2 - g.left, y: r.top + r.height / 2 - g.top }; };
  const move = (m) => {
    if (!line) {
      if (Math.hypot(m.clientX - start.x, m.clientY - start.y) < DRAG_PX) return;
      line = gsvg('svg', { class: 'pg-lines ed-overlay', 'aria-hidden': 'true' });
      line.append(gsvg('path', { class: 'pg-wire fail' }));
      graph.append(line);
      canvas.classList.add('wiring');
    }
    const g = graph.getBoundingClientRect();
    const o = origin();
    line.firstChild.setAttribute('d', `M${o.x},${o.y} L${m.clientX - g.left},${m.clientY - g.top}`);
    // a replan restarts from a step of the plan: a step tile, not a closed block or an include
    const under = document.elementFromPoint(m.clientX, m.clientY)?.closest('.pg-node[data-path]:not(.frame, .linked)');
    const next = under && under.dataset.path !== node.path ? under : null;
    if (target !== next) { target?.classList.remove('ed-target'); next?.classList.add('ed-target'); target = next; }
  };
  const done = () => {
    port.removeEventListener('pointermove', move);
    port.removeEventListener('pointerup', up);
    port.removeEventListener('pointercancel', done);  // a touch that became a scroll: only clean up
    canvas.classList.remove('wiring');
    line?.remove();
    target?.classList.remove('ed-target');
  };
  const up = () => {
    done();
    if (!line) { openRuleMenu(node, port); return; }
    if (!target) return;
    const toPath = target.dataset.path;
    const to = nodeAt(ed.mission, toPath);
    const ops = [];
    if (!to.id) ops.push({ op: 'set', at: toPath, key: 'id', value: freshId(to) });
    const id = to.id || ops[0].value;
    ops.push({ op: 'set', at: node.path, key: 'on_failure', value: 'replan' }, { op: 'set', at: node.path, key: 'replan_from', value: id });
    commit(ops, { select: [node.path] });
  };
  port.setPointerCapture(e.pointerId);
  port.addEventListener('pointermove', move);
  port.addEventListener('pointerup', up);
  port.addEventListener('pointercancel', done);
}
// an id for a step that has none: from its instruction's first words, unique in the mission
function freshId(step, more = []) {
  const taken = new Set([...stepIds(), ...more]);
  const base = (step.instruction || 'step').toLowerCase().replace(/[^a-z0-9]+/g, '_').replace(/^_+|_+$/g, '').split('_').slice(0, 3).join('_') || 'step';
  const first = /^[a-z_]/.test(base) ? base : `s_${base}`;
  let id = first;
  for (let n = 2; taken.has(id); n += 1) id = `${first}_${n}`;
  return id;
}
function openRuleMenu(node, port) {
  closeMenu();
  const step = node.step;
  const menu = div('ed-menu');
  menu.setAttribute('role', 'menu');
  const choice = (label, on, hint) => {
    const b = el('button', { type: 'button', role: 'menuitem', class: (step.on_failure || '') === on ? 'on' : '', text: label, title: hint || '' });
    b.addEventListener('click', () => {
      closeMenu();
      const ops = [{ op: 'set', at: node.path, key: 'on_failure', value: on || null }];
      if (on !== 'replan') ops.push({ op: 'set', at: node.path, key: 'replan_from', value: null });
      commit(ops, { select: [node.path] });
    });
    return b;
  };
  menu.append(el('div', { class: 'ed-menu-title', text: 'When this step fails' }),
    choice(`mission default (${ed.mission.defaults?.on_failure || 'abort'})`, ''), choice('retry', 'retry'),
    choice(step.replan_from ? `replan from ${step.replan_from}` : 'replan from this step', 'replan', 'drag the red port onto another step to replan from it'),
    choice('skip', 'skip'), choice('abort', 'abort'));
  const graph = $('ed-canvas').querySelector('.pg-graph');
  const r = port.getBoundingClientRect();
  const g = graph.getBoundingClientRect();
  menu.style.left = `${Math.min(r.right - g.left + 6, (ed.layout?.width || 600) - 230)}px`;
  menu.style.top = `${r.top - g.top - 8}px`;
  graph.append(menu);
  ed.menu = menu;
  menu.querySelector('button')?.focus();
}
function closeMenu() { ed.menu?.remove(); ed.menu = null; }

// ---------- drag a node (or a frame by its head) onto a wire to move it; a box on the canvas selects ----------
function nodeDown(e, node) {
  if (e.button !== 0 || node.linked || e.target.closest('.pg-port')) return;
  const canvas = $('ed-canvas');
  const graph = canvas.querySelector('.pg-graph');
  const start = { x: e.clientX, y: e.clientY };
  let ghost = null;
  let near = null;
  const source = e.currentTarget;
  const move = (m) => {
    if (!ghost) {
      if (Math.hypot(m.clientX - start.x, m.clientY - start.y) < DRAG_PX) return;
      ghost = (source.classList.contains('pg-band') ? canvas.querySelector(`.pg-node[data-path="${CSS.escape(node.path)}"]`) || source : source).cloneNode(true);
      ghost.classList.add('ed-ghost');
      document.body.append(ghost);
      canvas.classList.add('dragging');
      ed.dragged = true;
    }
    ghost.style.left = `${m.clientX + 8}px`;
    ghost.style.top = `${m.clientY + 8}px`;
    const g = graph.getBoundingClientRect();
    const px = m.clientX - g.left;
    const py = m.clientY - g.top;
    let best = null;
    (ed.spots || []).forEach((s) => {
      if (inside(s.list, node.path)) return;  // not into itself
      const d = Math.hypot(s.x - px, s.y - py);
      if (d < SPOT_REACH && (!best || d < best.d)) best = { s, d };
    });
    const next = best?.s || null;
    if (near !== next) { near?.el.classList.remove('near'); next?.el.classList.add('near'); near = next; }
  };
  const up = () => {
    source.removeEventListener('pointermove', move);
    source.removeEventListener('pointerup', up);
    source.removeEventListener('pointercancel', up);
    canvas.classList.remove('dragging');
    ghost?.remove();
    near?.el.classList.remove('near');
    if (!ghost) return;
    setTimeout(() => { ed.dragged = false; }, 0);
    if (!near) return;
    const from = slotOf(node.path);
    const to = joinPath(near.list, near.index);
    if (near.list === from.list && (near.index === from.index || near.index === from.index + 1)) return;  // where it is
    // the landed path is only sure within one list (a move out of a block shifts what follows it)
    const landed = near.list !== from.list ? [] : [joinPath(near.list, near.index > from.index ? near.index - 1 : near.index)];
    commit([{ op: 'move', from: node.path, to }], { select: landed });
  };
  source.setPointerCapture(e.pointerId);
  source.addEventListener('pointermove', move);
  source.addEventListener('pointerup', up);
  source.addEventListener('pointercancel', up);
}
function canvasDown(e) {
  if (e.button !== 0 || e.target.closest('.pg-node, .pg-band, .ed-spot, .ed-palette, .ed-menu, .ed-first')) return;
  const canvas = $('ed-canvas');
  const graph = canvas.querySelector('.pg-graph');
  if (!graph) return;
  closePalette();
  closeMenu();
  const g = () => graph.getBoundingClientRect();
  const start = { x: e.clientX - g().left, y: e.clientY - g().top };
  let box = null;
  const move = (m) => {
    const x = m.clientX - g().left;
    const y = m.clientY - g().top;
    if (!box) {
      if (Math.hypot(x - start.x, y - start.y) < DRAG_PX) return;
      box = div('ed-box');
      graph.append(box);
    }
    Object.assign(box.style, { left: `${Math.min(x, start.x)}px`, top: `${Math.min(y, start.y)}px`, width: `${Math.abs(x - start.x)}px`, height: `${Math.abs(y - start.y)}px` });
  };
  const up = (m) => {
    canvas.removeEventListener('pointermove', move);
    canvas.removeEventListener('pointerup', up);
    canvas.removeEventListener('pointercancel', up);
    if (!box) { if (m.type !== 'pointercancel') clearSelection(); return; }
    const r = box.getBoundingClientRect();
    box.remove();
    const hit = [];
    ed.layout.items.forEach((it) => {
      if (it.kind === 'dot' || it.node.linked) return;
      const n = it.el?.getBoundingClientRect();
      if (!n) return;
      const cx = n.left + n.width / 2;
      const cy = n.top + n.height / 2;
      if (cx >= r.left && cx <= r.right && cy >= r.top && cy <= r.bottom) hit.push(it.node.path);
    });
    if (!hit.length) { clearSelection(); return; }
    const list = slotOf(hit[0]).list;
    ed.selected = hit.filter((p) => slotOf(p).list === list);
    renderEditor();
    renderPanel();
    m.preventDefault();
  };
  canvas.setPointerCapture(e.pointerId);
  canvas.addEventListener('pointermove', move);
  canvas.addEventListener('pointerup', up);
  canvas.addEventListener('pointercancel', up);
}

// ---------- the properties panel ----------
function renderPanel() {
  const sel = $('ed-sel');
  const had = sel.contains(document.activeElement) && document.activeElement.tagName === 'BUTTON' ? document.activeElement.textContent : null;
  const missionPane = $('ed-mission');
  ed.card = null;
  ed.cardPath = null;
  const paths = ed.selected.filter((p) => nodeAt(ed.mission, p));
  ed.selected = paths;
  show(missionPane, !paths.length);
  show(sel, paths.length > 0);
  if (!paths.length) { sel.replaceChildren(); return; }
  const actions = div('ed-actions');
  const run = selectedRun();
  const act = (label, handler, title = '', cls = 'btn small') => { const b = el('button', { type: 'button', class: cls, text: label, title }); b.addEventListener('click', handler); return b; };
  if (paths.length > 1) {
    sel.replaceChildren(el('h3', { class: 'label', text: `${paths.length} steps selected` }),
      run ? el('p', { class: 'muted small-text', text: 'They follow each other: they can become one block.' })
        : el('p', { class: 'muted small-text', text: 'Pick steps that follow each other to make a block of them.' }), actions);
    if (run && ed.blocks) actions.append(...BLOCK_KINDS.map((k) => act(BLOCK_WORDS[k], () => wrapSelection(k), `Put the selected steps in a ${k}`)));
    actions.append(act('Remove', removeSelection, 'Remove the selected steps', 'btn small danger'));
    return;
  }
  const path = paths[0];
  const step = nodeAt(ed.mission, path);
  const kind = step.include ? 'include' : blockKind(step);
  const { list, index } = slotOf(path);
  const siblings = listAt(ed.mission, list) || [];
  sel.replaceChildren();
  if (kind === 'include') sel.append(div('ed-linked', el('span', { class: 'pg-link', text: '⛓' }),
    el('span', { text: `Linked: the steps of ${step.include} run here. Every mission that includes it shares them.` }),
    act('Open to edit', () => openIncluded(step.include), '', 'btn small ghost')));
  if (kind && kind !== 'include') sel.append(blockForm(path, step, kind));
  else {
    const card = stepCard(step, index, siblings, true);
    card.classList.add('ed-card');
    ['input', 'change'].forEach((type) => card.addEventListener(type, () => cardChanged(path)));
    sel.append(card);
    ed.card = card;
    ed.cardPath = path;
    (ed.marks.get(path) || []).forEach((m) => markField(card, m.field, m.text));
    (ed.notes.get(path) || []).forEach((text) => card.querySelector('.args')?.before(el('div', { class: 'msg step-note', text })));
    if (ed.marks.has(path)) { card.querySelector('.rules').open = true; }
  }
  actions.append(act('↑', () => moveSelected(-1), 'Move up (Ctrl+↑)', 'btn small icon-btn'), act('↓', () => moveSelected(1), 'Move down (Ctrl+↓)', 'btn small icon-btn'),
    act('Duplicate', duplicateSelected, 'A copy below it (Ctrl+D)'));
  if (kind && kind !== 'include') actions.append(act('Unwrap', () => commit([{ op: 'unwrap', at: path }], { select: [] }), 'Replace the block by its steps'));
  if (ed.blocks && (!kind || kind === 'include')) actions.append(...BLOCK_KINDS.map((k) => act(BLOCK_WORDS[k], () => wrapSelection(k), `Put this step in a ${k}`)));
  actions.append(act('Remove', removeSelection, 'Remove this step (Delete)', 'btn small danger'));
  sel.append(actions);
  if (had) [...actions.querySelectorAll('button')].find((b) => b.textContent === had)?.focus();
}
// the step form's fields changed: set ops for what differs (one undo entry per step while it stays selected)
function cardChanged(path) {
  if (!ed.card || ed.cardPath !== path) return;
  const next = ed.card._read();
  const cur = nodeAt(ed.mission, path);
  if (!cur) return;
  const ops = CARD_KEYS.filter((key) => !same(cur[key], next[key])).map((key) => ({ op: 'set', at: path, key, value: next[key] ?? null }));
  if (!ops.length) return;
  const byPlace = /^s(\d+)$/.exec(next.replan_from || '');  // the form offers s<n> for a sibling without an id
  if (byPlace && next.replan_from !== cur.replan_from) {
    const { list } = slotOf(path);
    const targetPath = joinPath(list, byPlace[1] - 1);
    const target = nodeAt(ed.mission, targetPath);
    if (target && !target.id && targetPath !== path) {
      const id = freshId(target);
      ops.unshift({ op: 'set', at: targetPath, key: 'id', value: id });
      ops.find((op) => op.key === 'replan_from').value = id;
      ed.card.querySelector('[data-k="replan_from"]').replaceChildren(el('option', { value: id, text: id }));
    }
  }
  const keep = ed.card;
  commit(ops, { tag: `card:${path}` });
  ed.card = keep;  // the panel was not rebuilt
  ed.cardPath = path;
}
// a block's own fields (schema v2): id, instruction, its rule
function blockForm(path, block, kind) {
  const form = div('ed-block');
  const field = (label, key, make) => {
    const input = make();
    input.addEventListener('input', () => { const v = input.type === 'number' ? (input.value === '' ? null : Number(input.value)) : input.value.trim() || null; commit([{ op: 'set', at: path, key, value: v }], { tag: `card:${path}` }); });
    return div('field', el('label', { text: label }), input);
  };
  form.append(el('h3', { class: 'label', text: `${BLOCK_WORDS[kind]}${block.steps || block.parallel ? '' : ''}` }),
    field('Instruction', 'instruction', () => Object.assign(el('textarea', { rows: '2' }), { value: block.instruction || '' })),
    field('Block id', 'id', () => el('input', { value: block.id || '', placeholder: 'auto', spellcheck: 'false' })));
  if (kind === 'repeat') {
    const r = block.repeat || {};
    const mode = el('select', {}, ...['times', 'while', 'until'].map((m) => el('option', { value: m, text: m })));
    mode.value = ['times', 'while', 'until'].find((m) => m in r) || 'times';
    const value = el('input', { value: r.times ?? r.while ?? r.until ?? '', placeholder: 'count or world key' });
    const push = () => {
      const m = mode.value;
      const v = m === 'times' ? Number(value.value) || null : value.value.trim() || null;
      commit([{ op: 'set', at: path, key: 'repeat', value: v === null ? {} : { [m]: v } }], { tag: `card:${path}` });
    };
    mode.addEventListener('change', push);
    value.addEventListener('input', push);
    form.append(div('field', el('label', { text: 'Repeat' }), div('row gap-s', mode, value)),
      field('Max passes', 'max_cycles', () => el('input', { type: 'number', min: '1', value: block.max_cycles ?? '', placeholder: 'none' })));
  }
  if (kind === 'parallel') {
    const join = el('select', {}, ...['all', 'first'].map((j) => el('option', { value: j, text: j === 'all' ? 'wait for every lane' : 'the first lane done ends it' })));
    join.value = block.join || 'all';
    join.addEventListener('change', () => commit([{ op: 'set', at: path, key: 'join', value: join.value === 'all' ? null : join.value }]));
    form.append(div('field', el('label', { text: 'Join' }), join));
  }
  form.append(field('Run only when (world key)', 'when', () => el('input', { value: block.when || '', placeholder: 'always', spellcheck: 'false' })));
  return form;
}
function wrapSelection(kind) {
  const run = selectedRun();
  if (!run) return;
  const from = joinPath(run.list, run.first);
  commit([{ op: 'wrap', from, to: joinPath(run.list, run.last), as: kind, block: { instruction: '' } }], { select: [from] });
}
function removeSelection() {
  const paths = [...ed.selected].sort((a, b) => slotOf(b).index - slotOf(a).index);  // from the back: the indexes hold
  if (!paths.length) return;
  commit(paths.map((at) => ({ op: 'remove', at })), { select: [] });
}
function moveSelected(delta) {
  const run = selectedRun();
  if (!run || run.first !== run.last) return;
  const n = (listAt(ed.mission, run.list) || []).length;
  const target = run.first + delta;
  if (target < 0 || target >= n) return;
  const from = joinPath(run.list, run.first);
  commit([{ op: 'move', from, to: joinPath(run.list, delta > 0 ? target + 1 : target) }], { select: [joinPath(run.list, target)] });
}
function duplicateSelected() {
  const run = selectedRun();
  if (!run || run.first !== run.last) return;
  const copy = clone(nodeAt(ed.mission, joinPath(run.list, run.first)));
  const dropIds = (step) => { delete step.id; (step.steps || []).forEach(dropIds); (step.parallel || []).forEach((lane) => (lane.steps || []).forEach(dropIds)); };
  dropIds(copy);
  const at = joinPath(run.list, run.first + 1);
  commit([{ op: 'insert', at, step: copy }], { select: [at] });
}

// ---------- the mission's own fields (the panel with nothing selected) ----------
function missionFieldChanged(id) {
  const key = MISSION_KEYS[id];
  const input = $(id);
  let value = input.type === 'number' ? (input.value === '' ? null : Number(input.value)) : input.value.trim() || null;
  if (key === 'loop' && value === 'once') value = 'once';
  if (same(ed.mission[key], value)) return;
  commit([{ op: 'set', at: '', key, value }], { tag: `mission:${key}` });
}
function defaultsChanged() {
  const defaults = {};
  if ($('d-timeout').value !== '') defaults.timeout_s = Number($('d-timeout').value);
  if ($('d-retries').value !== '') defaults.retries = Number($('d-retries').value);
  if ($('d-fail').value) defaults.on_failure = $('d-fail').value;
  if ($('d-replans').value !== '') defaults.max_replans = Number($('d-replans').value);
  const value = Object.keys(defaults).length ? defaults : null;
  if (same(ed.mission.defaults, value)) return;
  commit([{ op: 'set', at: '', key: 'defaults', value }], { tag: 'mission:defaults' });
}
Object.keys(MISSION_KEYS).forEach((id) => $(id).addEventListener('input', () => ed.mission && missionFieldChanged(id)));
['d-timeout', 'd-retries', 'd-fail', 'd-replans'].forEach((id) => $(id).addEventListener('input', () => ed.mission && defaultsChanged()));
$('m-name').addEventListener('input', () => { $('ed-name-text').textContent = $('m-name').value.trim() || 'new mission'; });

// ---------- open, check, save ----------
// app.js openEditor filled the mission's fields; here the graph: a saved mission as its file, a new one as the
// template, recorded as the ops that make its file
function editorOpen(name, mission, isNew, base = null) {
  ed.mission = clone(mission);
  ed.base = isNew ? null : base;
  ed.name = name;
  ed.isNew = isNew;
  ed.ops = [];
  ed.undo = [];
  ed.redo = [];
  ed.selected = [];
  ed.closed = new Set();
  ed.opened = new Set();
  ed.includes = {};
  ed.marks = new Map();
  ed.notes = new Map();
  ed.savedUndo = null;
  ed.reveal = true;  // a graph wider than the canvas starts at its first node, not at its left margin
  $('ed-canvas').scrollTo(0, 0);
  if (isNew) {
    ['goal', 'robot', 'params', 'facts', 'loop', 'max_cycles', 'stop_when', 'defaults'].forEach((key) => { if (mission[key] !== undefined) ed.ops.push({ op: 'set', at: '', key, value: clone(mission[key]) }); });
    (mission.steps || []).forEach((step, i) => ed.ops.push({ op: 'insert', at: joinPath('steps', i), step: clone(step) }));
  }
  $('ed-name-text').textContent = name || 'new mission';
  if (!ed.schemaAsked) { ed.schemaAsked = true; api('GET', '/api/schema/mission').then(({ ok, data }) => {
    ed.blocks = ok && !!data?.properties?.steps?.items?.properties?.group;
    renderPanel();
  }); }
  renderEditor();
  renderPanel();
  tools();
}
// the mission as the file would be now (the name is the file's)
const currentMission = () => clone(ed.mission);
function showProblems(problems, places = []) {
  show($('problems'), problems.length > 0);
  $('problems').textContent = problems.join('\n');
  ed.marks = new Map();
  places.forEach((place) => {
    const path = joinPath('steps', place.step);
    if (!ed.marks.has(path)) ed.marks.set(path, []);
    ed.marks.get(path).push({ field: place.path || [], text: place.message || problems[place.problem] });
  });
  renderEditor();
  renderPanel();
}
function showNotes(notes) {
  ed.notes = new Map();
  notes.forEach((n) => { const path = joinPath('steps', n.step); if (!ed.notes.has(path)) ed.notes.set(path, []); ed.notes.get(path).push(`${n.include ? `${n.include}: ` : ''}${n.message}`); });
  renderPanel();
}
$('btn-validate').addEventListener('click', async () => {
  const name = $('m-name').value.trim() || 'new';
  const { ok, data } = await api('POST', `/api/missions/${encodeURIComponent(name)}/validate`, currentMission());
  if (!ok) { message($('save-msg'), data.error || (data.problems || []).join('\n'), true); return; }
  showProblems(data.problems, data.places);
  showNotes(data.notes || []);
  const noted = (data.notes || []).length;
  message($('save-msg'), data.problems.length ? '' : `Mission is valid${noted ? ` (${noted} note${noted > 1 ? 's' : ''} on its steps)` : ''}.`);
});
$('btn-save').addEventListener('click', async () => {
  const name = $('m-name').value.trim();
  if (!name) { showProblems(['Give the mission a name.']); return; }
  const old = ed.isNew ? null : ed.name;
  const rename = old && old !== name ? `?rename_from=${encodeURIComponent(old)}` : '';
  const { ok, status, data } = await api('POST', `/api/missions/${encodeURIComponent(name)}/edit${rename}`,
    { ops: ed.ops, base: ed.base, create: ed.isNew });
  if (!ok) {
    showProblems(data.problems || [data.error || 'not saved'], data.places);
    message($('save-msg'), '');
    // the file changed since it was opened (not a 409 for a running mission): offer it as it is now, the edits dropped
    if (status === 409 && old && data.conflict === 'stale') {
      const reload = el('button', { type: 'button', class: 'btn small ed-reload', text: 'Reload from file',
        title: `Open ${old} as it is saved now; the changes here are dropped` });
      reload.addEventListener('click', () => { state.dirty = false; openEditor(old); });
      $('problems').append(' ', reload);
    }
    return;
  }
  showProblems([]);
  ed.mission = data.mission;
  ed.ops = [];
  ed.undo = [];
  ed.redo = [];
  ed.isNew = false;
  ed.name = name;
  ed.base = data.hash || null;
  ed.savedUndo = data.undo;
  fillMissionFields();
  state.editing = { ...state.editing, name, isNew: false, mission: data.mission };
  state.dirty = false;
  show($('editor-dirty'), false);
  $('mission-search').value = '';
  clearDrop();
  show($('btn-delete'), true);
  [name, old].filter(Boolean).forEach(forgetMission);
  const note = $('save-msg');
  message(note, rename ? `Renamed ${old} to ${name} and saved. It is used at the next start.` : `Saved ${name}. It is used at the next start.`);
  if (data.undo) {
    const undoSave = el('button', { type: 'button', class: 'btn small ghost ed-undo-save', text: 'Undo this save' });
    undoSave.addEventListener('click', async () => {
      const answer = await api('POST', '/api/teach/undo', { id: data.undo.id });
      if (!answer.ok) { message(note, (answer.data.problems || [answer.data.error || 'not undone']).join('\n'), true); return; }
      if (old) openEditor(old); else { state.editing = null; state.dirty = false; showEditor(false); }
      loadMissions();
    });
    note.append(' ', undoSave);
  }
  renderEditor();
  renderPanel();
  tools();
  loadMissions();
});
$('ed-undo').addEventListener('click', undoEdit);
$('ed-redo').addEventListener('click', redoEdit);
$('ed-add').addEventListener('click', () => paletteAtSelection());
$('ed-canvas').addEventListener('pointerdown', canvasDown);
// the keyboard on the canvas and the page while the editor is open (not while typing in a field)
document.addEventListener('keydown', (e) => {
  if (!ed.mission || $('editor').hidden || $('view-missions').hidden) return;
  const typing = e.target.closest?.('textarea, select, input, [contenteditable]');
  if ((e.ctrlKey || e.metaKey) && !e.shiftKey && e.key.toLowerCase() === 'z') { if (typing) return; e.preventDefault(); undoEdit(); return; }
  if ((e.ctrlKey || e.metaKey) && (e.key.toLowerCase() === 'y' || (e.shiftKey && e.key.toLowerCase() === 'z'))) { if (typing) return; e.preventDefault(); redoEdit(); return; }
  if (typing) return;
  const onCanvas = $('ed-canvas').contains(e.target) || e.target === document.body;
  if (e.key === 'Escape') { if (ed.palette || ed.menu) { closePalette(); closeMenu(); } else clearSelection(); return; }
  if (!onCanvas) return;
  if (e.key === 'Delete' || e.key === 'Backspace') { if (ed.selected.length) { e.preventDefault(); removeSelection(); } return; }
  if (e.shiftKey && e.key.toLowerCase() === 'a') { e.preventDefault(); paletteAtSelection(); return; }
  if ((e.ctrlKey || e.metaKey) && e.key.toLowerCase() === 'd') { e.preventDefault(); duplicateSelected(); return; }
  if ((e.ctrlKey || e.metaKey) && (e.key === 'ArrowUp' || e.key === 'ArrowDown')) { e.preventDefault(); moveSelected(e.key === 'ArrowUp' ? -1 : 1); }
});
// the layout follows the canvas width
let editorWidth = 0;
new ResizeObserver(([entry]) => {
  const width = Math.round(entry.contentRect.width);
  if (width && width !== editorWidth) { editorWidth = width; ed.reveal = true; if (!ed.palette) renderEditor(); }
}).observe($('ed-canvas'));
