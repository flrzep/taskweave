'use strict';
// taskweave HMI: the mission graph's core. One tree model of a mission (steps in groups, repeats, parallel lanes and
// linked includes), one automatic layout (a structured program has one canonical picture: no stored positions) and one
// drawing, used by the Run tab's compact view (graph-run.js) and the Program tab's editor (editor.js). The look is a
// node editor's (Nuke, Blender): nodes carry the colour, frames hold groups, the flow runs top to bottom in the editor
// and snakes across the box on the Run tab, where the room is scarce. Who draws what: the caller renders the nodes,
// the stacks (a closed frame) and the frame heads through ctx hooks; this file places them and draws the wires, the
// frames, the split and join of a parallel block, the loop of a repeat and of the mission, the replan arrows, the
// start dot and the final ring. Uses el from app.js.

const GRAPH_SVG = 'http://www.w3.org/2000/svg';
function gsvg(tag, attrs = {}, text) {
  const node = document.createElementNS(GRAPH_SVG, tag);
  Object.entries(attrs).forEach(([key, value]) => node.setAttribute(key, value));
  if (text !== undefined) node.textContent = text;
  return node;
}

// The room things take (px). A slot is a node with its label; a frame adds its head and padding; the gutter is the
// room a repeat keeps at its left for the return wire; left is the box's own gutter (the start dot, the mission's
// loop). column: one item per row (the editor); else the items of a sequence snake across the width.
const GRAPH_METRICS = {
  compact: { slotW: 92, slotH: 84, node: 44, stack: 52, gapX: 10, gapY: 10, pad: 14, head: 30, left: 56, right: 12,
    top: 16, bottom: 56, gutter: 22, dot: 14, column: false, label: true, frameRadius: 20 },
  phone: { slotW: 74, slotH: 80, node: 44, stack: 52, gapX: 4, gapY: 8, pad: 10, head: 28, left: 36, right: 8,
    top: 14, bottom: 52, gutter: 18, dot: 12, column: false, label: true, frameRadius: 18 },
  editor: { slotW: 208, slotH: 60, node: { w: 196, h: 56 }, stack: { w: 196, h: 56 }, gapX: 24, gapY: 26, pad: 22,
    head: 36, left: 64, right: 28, top: 24, bottom: 48, gutter: 30, dot: 16, column: true, label: false, frameRadius: 12 },
};
const sizeOf = (size) => (typeof size === 'number' ? { w: size, h: size } : size);
const FRAME_KINDS = ['include', 'group', 'repeat', 'parallel', 'branch'];
const isFrame = (node) => FRAME_KINDS.includes(node.kind);

// ---------- the tree ----------
// A node: {kind, key, name, children} for include | group | repeat | branch, {kind: 'parallel', branches: [branch]},
// {kind: 'step', key: id, step, index, name} for a skill call (index: its place in plan.steps). The root is a group
// without a frame ({kind: 'root'}); root.loop says the mission repeats. Every frame gets leaves: the indices of the
// steps inside it, in order.
function treeFromPlan(plan, mission) {
  const root = plan.tree ? { kind: 'root', key: 'root', children: treeFromNodes(plan.tree, plan, '') }
    : treeFromFlat(plan, mission);
  root.loop = !!plan.loop && plan.loop !== 'once';
  fillLeaves(root);
  return root;
}
// today's plan: a flat list whose steps carry origin {top, include: [chain], step}; an include occurrence of the top
// mission is a frame, a nested include a frame inside it (any depth), the mission's own steps sit in the root
function treeFromFlat(plan, mission) {
  const root = { kind: 'root', key: 'root', children: [] };
  let open = [];  // the include frames the last step sat in, outermost first: {node, include, top, inner}
  plan.steps.forEach((step, i) => {
    const o = step.origin;
    const chain = o?.include || [];
    const top = o?.top ?? 0;
    let d = 0;
    while (d < open.length && d < chain.length && open[d].include === chain[d] && open[d].top === top) d += 1;
    // the innermost include starting over (its own step index not rising: the next occurrence of the same include)
    if (d === chain.length && d === open.length && d > 0 && open[d - 1].inner !== undefined && o.step <= open[d - 1].inner) d -= 1;
    open = open.slice(0, d);
    for (; d < chain.length; d += 1) {
      const parent = d ? open[d - 1].node : root;
      const key = d ? `${open[d - 1].node.key}/${chain[d]}@${i}` : `i${top}:${chain[d]}`;
      const node = { kind: 'include', key, include: chain[d], top, children: [],
        name: d ? chain[d] : (mission?.steps?.[top]?.instruction || chain[d]) };
      parent.children.push(node);
      open.push({ node, include: chain[d], top });
    }
    (open.length ? open[open.length - 1].node : root).children.push({ kind: 'step', key: step.id, id: step.id, step, index: i,
      name: step.instruction });
    if (open.length) open[open.length - 1].inner = o.step;
  });
  return root;
}
// a plan that carries the mission tree (schema v2): blocks with steps or branches, steps by id
function treeFromNodes(nodes, plan, parentKey) {
  return nodes.map((n, k) => {
    if (n.skill || !n.kind || n.kind === 'step') {
      const index = plan.steps.findIndex((s) => s.id === n.id);
      return { kind: 'step', key: n.id, id: n.id, step: index >= 0 ? plan.steps[index] : n, index, name: n.instruction };
    }
    const key = `g:${n.origin?.path || `${parentKey}/${k}`}`;
    const node = { kind: n.kind, key, id: n.id, name: n.instruction || n.id, block: n, include: n.include || '' };
    if (n.kind === 'parallel') {
      node.branches = (n.branches || []).map((b, j) => ({ kind: 'branch', key: `${key}/b${j}`, id: b.id,
        name: b.instruction || b.id, children: treeFromNodes(b.steps || [], plan, `${key}/b${j}`) }));
    } else node.children = treeFromNodes(n.steps || [], plan, key);
    return node;
  });
}
const childrenOf = (node) => node.children || node.branches || [];
function fillLeaves(node) {
  if (node.kind === 'step') return [node.index];
  node.leaves = childrenOf(node).flatMap(fillLeaves);
  return node.leaves;
}
// every frame of the tree, outermost first, in order
function framesOf(root) {
  const out = [];
  const walk = (node) => { if (isFrame(node)) out.push(node); childrenOf(node).forEach(walk); };
  walk(root);
  return out;
}

// ---------- the layout ----------
// ctx: {metrics, width, isOpen(frame) -> bool}. Places every node: items (key -> {node, kind: step | stack | dot,
// x, y, w, h: its slot; cx, cy: the node's centre; nw, nh: the node's size}), frames (open: {node, x, y, w, h}), and
// owner (a step's key -> the key of the item that shows it: itself, or the stack of its closed frame; a frame's own
// key -> its stack, or the item of its first step while open).
function layoutGraph(root, ctx) {
  const M = ctx.metrics;
  const items = new Map();
  const frames = [];
  const owner = new Map();
  const nodeSize = sizeOf(M.node);
  const stackSize = sizeOf(M.stack);

  const slot = (node, kind) => {
    const { w: nw, h: nh } = kind === 'stack' ? stackSize : nodeSize;
    return { w: M.slotW, h: M.slotH, place(x, y) {
      const cy = M.label ? y + nh / 2 : y + M.slotH / 2;
      items.set(node.key, { node, kind, x, y, w: M.slotW, h: M.slotH, cx: x + M.slotW / 2, cy, nw, nh });
      owner.set(node.key, node.key);
      if (kind === 'stack') node.leaves.forEach((i) => owner.set(ctx.keyOf(i), node.key));
    } };
  };
  const dotItem = (key, node, x, y) => items.set(key, { node, kind: 'dot', x: x - M.dot / 2, y: y - M.dot / 2, w: M.dot, h: M.dot,
    cx: x, cy: y, nw: M.dot, nh: M.dot });

  function measure(node, maxW) {
    if (node.kind === 'step') return slot(node, 'step');
    const open = node.kind === 'root' || ctx.isOpen(node);
    if (!open) return slot(node, 'stack');
    const padX = node.kind === 'root' ? 0 : M.pad;
    const head = node.kind === 'root' ? 0 : M.head;
    const foot = node.kind === 'root' ? 0 : M.pad;
    if (node.kind === 'parallel') {
      const n = Math.max(1, node.branches.length);
      const laneW = Math.max(M.slotW + 2 * M.pad, (maxW - 2 * padX - (n - 1) * M.gapX) / n);
      const lanes = node.branches.map((b) => measure(b, laneW));
      const w = Math.max(M.slotW, lanes.reduce((t, l) => t + l.w, 0) + (n - 1) * M.gapX) + 2 * padX;
      const h = head + M.dot + M.gapY + Math.max(0, ...lanes.map((l) => l.h)) + M.gapY + M.dot + foot;
      return { w, h, place(x, y) {
        frames.push({ node, x, y, w, h });
        dotItem(`${node.key}:split`, node, x + w / 2, y + head + M.dot / 2);
        let lx = x + padX + (w - 2 * padX - (lanes.reduce((t, l) => t + l.w, 0) + (n - 1) * M.gapX)) / 2;
        lanes.forEach((l) => { l.place(lx, y + head + M.dot + M.gapY); lx += l.w + M.gapX; });
        dotItem(`${node.key}:join`, node, x + w / 2, y + h - foot - M.dot / 2);
        owner.set(node.key, `${node.key}:split`);
      } };
    }
    const gutter = node.kind === 'repeat' || (node.kind === 'root' && root.loop) ? M.gutter : 0;
    const inner = Math.max(M.slotW, maxW - 2 * padX - gutter);
    const sized = node.children.map((c) => measure(c, inner));
    const rows = [];
    let row = null;
    sized.forEach((s) => {
      if (M.column || !row || row.w + M.gapX + s.w > inner) { row = { items: [], w: 0, h: 0 }; rows.push(row); }
      row.items.push(s);
      row.w += (row.items.length > 1 ? M.gapX : 0) + s.w;
      row.h = Math.max(row.h, s.h);
    });
    const empty = node.kind !== 'root' && !sized.length;  // an empty block keeps a dot, so the wires and the editor's "+" reach it
    const body = empty ? M.dot + 2 * M.gapY : rows.reduce((t, r) => t + r.h, 0) + Math.max(0, rows.length - 1) * M.gapY;
    const w = Math.max(M.slotW, ...rows.map((r) => r.w)) + 2 * padX + gutter;
    const h = head + body + foot;
    return { w, h, place(x, y) {
      if (node.kind !== 'root') frames.push({ node, x, y, w, h });
      if (empty) { dotItem(`${node.key}:empty`, node, x + w / 2, y + head + M.gapY + M.dot / 2); owner.set(node.key, `${node.key}:empty`); }
      let cy = y + head;
      rows.forEach((r, ri) => {
        const list = !M.column && ri % 2 ? [...r.items].reverse() : r.items;  // a snake: every other row runs back
        const room = w - 2 * padX - gutter;  // the frame's own inner width
        let cx = x + padX + gutter + (M.column ? (room - r.w) / 2 : ri % 2 ? room - r.w : 0);
        list.forEach((s) => { s.place(cx, cy + (r.h - s.h) / 2); cx += s.w + M.gapX; });
        cy += r.h + M.gapY;
      });
      if (node.kind !== 'root' && node.leaves.length) owner.set(node.key, owner.get(ctx.keyOf(node.leaves[0])));
    } };
  }
  const sized = measure(root, Math.max(M.slotW + M.gutter, ctx.width - M.left - M.right));
  sized.place(M.left, M.top);
  let bottom = M.top;
  items.forEach((it) => { bottom = Math.max(bottom, it.y + it.h); });
  frames.forEach((f) => { bottom = Math.max(bottom, f.y + f.h); });
  return { root, items, frames, owner, width: Math.max(ctx.width, sized.w + M.left + M.right), height: bottom + M.bottom, metrics: M };
}

// ---------- the drawing ----------
// where the line from an item's centre toward (tx, ty) leaves its node, gap px outside it
function graphRim(n, tx, ty, gap) {
  const dx = tx - n.cx;
  const dy = ty - n.cy;
  const s = Math.min((n.nw / 2 + gap) / Math.abs(dx || 1e-9), (n.nh / 2 + gap) / Math.abs(dy || 1e-9), 1);
  return { x: n.cx + dx * s, y: n.cy + dy * s };
}
// a wire from item a to item b, bowed sideways by bend px: its path and its middle (for a label)
function graphWire(a, b, bend) {
  const d = Math.hypot(b.cx - a.cx, b.cy - a.cy) || 1;
  const mx = (a.cx + b.cx) / 2 - ((b.cy - a.cy) / d) * bend;
  const my = (a.cy + b.cy) / 2 + ((b.cx - a.cx) / d) * bend;
  const p = graphRim(a, mx, my, 3);
  const q = graphRim(b, mx, my, 5);
  return { d: `M${p.x},${p.y} Q${mx},${my} ${q.x},${q.y}`, mid: { x: (p.x + 2 * mx + q.x) / 4, y: (p.y + 2 * my + q.y) / 4 } };
}
// a polyline with its corners rounded (radius px)
function graphRounded(points, radius) {
  let d = `M${points[0][0]},${points[0][1]}`;
  for (let k = 1; k < points.length - 1; k += 1) {
    const [px, py] = points[k - 1];
    const [x, y] = points[k];
    const [nx, ny] = points[k + 1];
    const a = Math.hypot(x - px, y - py) || 1;
    const b = Math.hypot(nx - x, ny - y) || 1;
    const r1 = Math.min(radius, a / 2);
    const r2 = Math.min(radius, b / 2);
    d += ` L${x - ((x - px) / a) * r1},${y - ((y - py) / a) * r1} Q${x},${y} ${x + ((nx - x) / b) * r2},${y + ((ny - y) / b) * r2}`;
  }
  const [lx, ly] = points[points.length - 1];
  return `${d} L${lx},${ly}`;
}
const graphMarker = (name) => {
  const m = gsvg('marker', { id: `pg-head-${name}`, class: `pg-head ${name}`, viewBox: '0 0 10 10', refX: 8, refY: 5,
    markerWidth: 9, markerHeight: 9, markerUnits: 'userSpaceOnUse', orient: 'auto-start-reverse' });
  m.append(gsvg('path', { d: 'M0,1 L9,5 L0,9 z' }));
  return m;
};

// Draws a layout into a new .pg-graph. ctx hooks: renderItem(item) -> the node's element (a step or a stack),
// renderHead(frame) -> the frame's head element, labelOf(item) -> the label text under a compact node (or ''),
// stateOf(item) -> 'running' | 'paused' | 'pending' | a finished outcome (for the wires), frameWord(frame) -> its
// state word (for the outline), replans: [[from step key, to step key]], start: the start step's key, loop: the
// mission loop's words ('' when it does not repeat), repeatText(frame) -> a repeat frame's words.
// Returns {node, items, height, replans: how many replan arrows were drawn, flow: the wires in run order as
// {a, b, mid} (the items joined and the wire's middle; a null a: from the start, a null b: to the end), so the
// editor can put its "+" on them}.
function drawGraph(layout, ctx) {
  const { items, frames, owner, metrics: M } = layout;
  const width = layout.width;
  const height = layout.height;
  const graph = el('div', { class: 'pg-graph', role: 'group', 'aria-label': ctx.label || 'Mission graph' });
  const back = gsvg('svg', { class: 'pg-lines', 'aria-hidden': 'true' });
  const wires = gsvg('svg', { class: 'pg-lines', 'aria-hidden': 'true' });
  const notes = gsvg('svg', { class: 'pg-lines', 'aria-hidden': 'true' });
  [back, wires, notes].forEach((s) => { s.setAttribute('width', width); s.setAttribute('height', height); });
  graph.style.cssText = `width: ${width}px; height: ${height}px`;
  graph.append(back, wires);
  const defs = gsvg('defs');
  ['done', 'now', 'todo', 'fail', 'loop'].forEach((name) => defs.append(graphMarker(name)));
  wires.append(defs);

  // the frames: an outline with its head on the top edge (a branch: a fainter lane with its name)
  frames.forEach((f) => {
    const word = ctx.frameWord ? ctx.frameWord(f.node) : '';
    back.append(gsvg('rect', { class: `pg-outline ${f.node.kind} ${word}`, x: f.x, y: f.y, width: f.w, height: f.h, rx: M.frameRadius }));
    const head = ctx.renderHead(f.node, f);
    if (head) {
      head.style.left = `${f.x + 10}px`;
      head.style.top = `${f.y - 13}px`;
      head.style.maxWidth = `${f.w - 20}px`;
      graph.append(head);
    }
  });
  // the items: the caller's node elements, the label under them, the split and join dots
  items.forEach((it) => {
    if (it.kind === 'dot') {
      wires.append(gsvg('circle', { class: `pg-dot-${it.node.key.endsWith(':split') ? 'split' : 'join'} ${ctx.stateOf(it)}`,
        cx: it.cx, cy: it.cy, r: M.dot / 2 - 1 }));
      return;
    }
    const node = ctx.renderItem(it);
    node.style.left = `${it.cx - it.nw / 2}px`;
    node.style.top = `${it.cy - it.nh / 2}px`;
    graph.append(node);
    it.el = node;
    const text = M.label ? ctx.labelOf(it) : '';
    if (text) {
      const label = el('span', { class: `pg-label${ctx.stateOf(it) === 'running' || ctx.stateOf(it) === 'paused' ? ' now' : ''}`,
        'aria-hidden': 'true' }, ...(Array.isArray(text) ? text : [text]));
      label.style.left = `${it.x + 2}px`;
      label.style.width = `${it.w - 4}px`;
      label.style.top = `${it.cy + it.nh / 2 + 3}px`;
      graph.append(label);
    }
  });

  // the flow: entry and exit items of every node, then a wire from each item to the next in run order
  const live = (st) => st === 'running' || st === 'paused';
  const cls = (target) => { const st = ctx.stateOf(target); return live(st) ? 'now' : st !== 'pending' ? 'done' : 'todo'; };
  const entry = (node) => {
    if (node.kind === 'step' || (isFrame(node) && !ctx.isOpen(node))) return items.get(owner.get(node.kind === 'step' ? node.key : ctx.keyOf(node.leaves[0])) || node.key);
    if (node.kind === 'parallel') return items.get(`${node.key}:split`);
    for (const c of node.children || []) { const e = entry(c); if (e) return e; }
    return items.get(`${node.key}:empty`) || null;
  };
  const exit = (node) => {
    if (node.kind === 'step' || (isFrame(node) && !ctx.isOpen(node))) return entry(node);
    if (node.kind === 'parallel') return items.get(`${node.key}:join`);
    const list = node.children || [];
    for (let k = list.length - 1; k >= 0; k -= 1) { const x = exit(list[k]); if (x) return x; }
    return items.get(`${node.key}:empty`) || null;
  };
  const seen = new Set();
  const flow = [];
  const flowing = (path) => {  // the graph is rebuilt every second: a flowing wire runs at the phase of a page-wide clock
    if (path.classList.contains('now')) path.style.animationDelay = `-${(performance.now() % 700) / 1000}s`;
    return path;
  };
  const join = (a, b, bend) => {
    if (!a || !b || a === b || seen.has(`${a.node.key}>${b.node.key}`)) return;
    seen.add(`${a.node.key}>${b.node.key}`);
    const c = cls(b);
    const w = graphWire(a, b, bend);
    flow.push({ a, b, mid: w.mid });
    wires.append(flowing(gsvg('path', { class: `pg-wire ${c}`, d: w.d, 'marker-end': `url(#pg-head-${c})` })));
  };
  const wireSequence = (node, frame) => {
    const list = node.children || [];
    list.forEach((c, k) => {
      if (k) join(exit(list[k - 1]), entry(c), M.column ? 0 : Math.min(22, k % 2 ? 14 : -14));
      wireNode(c);
    });
    if ((node.kind === 'repeat' || (node.kind === 'root' && ctx.loop)) && list.length) {  // the return wire up the gutter
      const from = exit(list[list.length - 1]);
      const to = entry(list[0]);
      if (!from || !to) return;
      const gx = node.kind === 'root' ? Math.max(10, M.left / 2 - 2) : frame.x + M.gutter / 2 + 2;
      const low = node.kind === 'root' ? height - M.bottom / 2 - 4 : frame.y + frame.h - M.pad / 2;
      const blocked = [...items.values()].some((n) => n !== from && n.cx < from.cx && n.cy > from.cy - n.nh && n.cy < from.cy + n.nh);
      const way = blocked ? [[from.cx, from.cy + from.nh / 2 + (M.label ? 20 : 4)], [from.cx, low], [gx, low]]
        : [[from.cx - from.nw / 2 - 3, from.cy], [gx, from.cy]];
      const done = node.leaves ? node.leaves.every((i) => { const st = ctx.stateOf(items.get(owner.get(ctx.keyOf(i)))); return st !== 'pending' && !live(st); })
        : false;
      wires.append(flowing(gsvg('path', { class: `pg-wire loop${done ? ' now' : ''}`, 'marker-end': 'url(#pg-head-loop)',
        d: graphRounded([...way, [gx, to.cy], [to.cx - to.nw / 2 - 5, to.cy]], 16) })));
      const text = node.kind === 'root' ? ctx.loop : (ctx.repeatText ? ctx.repeatText(node) : '');
      const from_y = way[way.length - 1][1];
      const mid = (from_y + to.cy) / 2;
      if (text) notes.append(text.length * 7 < Math.abs(from_y - to.cy) - 24
        ? gsvg('text', { class: 'pg-wire-label loop', x: gx - 4, y: mid, 'text-anchor': 'middle', transform: `rotate(-90 ${gx - 4} ${mid})` }, text)
        : gsvg('text', { class: 'pg-wire-label loop', x: gx + 8, y: low + 14 }, text));
    }
  };
  const wireNode = (node) => {
    if (node.kind === 'step') return;
    if (isFrame(node) && !ctx.isOpen(node)) return;
    const frame = frames.find((f) => f.node === node);
    if (node.kind === 'parallel') {
      const split = items.get(`${node.key}:split`);
      const joinDot = items.get(`${node.key}:join`);
      node.branches.forEach((b) => { join(split, entry(b), 0); wireNode(b); join(exit(b), joinDot, 0); });
      return;
    }
    wireSequence(node, frame);
  };
  wireNode(layout.root);

  // the replan arrows: dashed, labelled, to the step the plan restarts from
  let drawnReplans = 0;
  (ctx.replans || []).forEach(([fromKey, toKey]) => {
    const a = items.get(owner.get(fromKey));
    const b = items.get(owner.get(toKey));
    if (!a || !b || a === b) return;
    drawnReplans += 1;
    // bowed sideways, clear of the nodes: wide ones (the editor's column) need the bow outside their edge
    const w = graphWire(a, b, Math.max(M.column ? a.nw / 2 + 56 : 0, Math.min(70, 26 + Math.hypot(b.cx - a.cx, b.cy - a.cy) * 0.25)));
    wires.append(gsvg('path', { class: 'pg-wire fail', d: w.d, 'marker-end': 'url(#pg-head-fail)' }));
    notes.append(gsvg('text', { class: 'pg-wire-label fail', x: w.mid.x, y: w.mid.y + 4, 'text-anchor': 'middle' }, 'replan'));
  });
  // the start: a dot beside the start step, where nothing is; the end: a final ring after the last step (unless the
  // mission repeats: then the loop wire)
  const first = items.get(owner.get(ctx.start)) || entry(layout.root);
  if (first) {
    const free = ([dx, dy]) => [...items.values()].every((n) => n === first || Math.hypot(n.cx - first.cx - dx, n.cy - first.cy - dy) > 40);
    const [sx, sy] = (M.column ? [[0, -56], [-70, -20], [70, -20]] : [[-58, -28], [-58, 26], [0, -64], [58, -30], [0, 64]]).find(free) || [-58, -28];
    const dot = { cx: first.cx + sx, cy: first.cy + sy, nw: 12, nh: 12 };
    const w = graphWire(dot, first, 0);
    flow.unshift({ a: null, b: first, mid: w.mid });
    wires.append(gsvg('circle', { class: 'pg-init', cx: dot.cx, cy: dot.cy, r: 6 }),
      gsvg('path', { class: 'pg-wire done', d: w.d, 'marker-end': 'url(#pg-head-done)' }));
  }
  const last = exit(layout.root);
  if (last && ctx.loop) flow.push({ a: last, b: null, mid: { x: last.cx, y: last.cy + last.nh / 2 + 22 } });  // the end, under the last node
  if (last && !ctx.loop) {
    const free = ([dx, dy]) => [...items.values()].every((n) => n === last || Math.hypot(n.cx - last.cx - dx, n.cy - last.cy - dy) > 40);
    const [ex, ey] = (M.column ? [[0, 56], [66, 0], [-66, 0]] : [[62, 0], [0, 66], [-62, 0], [0, -66]]).find(free) || [0, 66];
    const ring = { cx: last.cx + ex, cy: last.cy + ey, nw: 18, nh: 18 };
    const w = graphWire(last, ring, 0);
    flow.push({ a: last, b: null, mid: w.mid });
    wires.append(gsvg('path', { class: 'pg-wire todo', d: w.d, 'marker-end': 'url(#pg-head-todo)' }),
      gsvg('circle', { class: 'pg-final', cx: ring.cx, cy: ring.cy, r: 9 }), gsvg('circle', { class: 'pg-final-dot', cx: ring.cx, cy: ring.cy, r: 5 }));
  }
  graph.append(notes);
  return { node: graph, items, height, replans: drawnReplans, flow };
}
