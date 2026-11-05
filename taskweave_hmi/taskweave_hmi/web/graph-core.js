'use strict';
// taskweave HMI: the mission graph's core. One tree model of a mission (steps in groups, repeats, parallel lanes and
// linked includes), one automatic layout (a structured program has one canonical picture: no stored positions) and one
// drawing, used by the Run tab's compact view (graph-run.js) and the Program tab's editor (editor.js). The look is a
// behaviour-flow editor's: the flow runs left to right from a Start node to a Done node, nodes have a coloured header
// (the name, the kind word) and fields in mono, blocks are translucent boxes with a title bar, wires are bezier curves
// between ports. The editor adds the failure routes under the flow: a Fail terminal per aborting step in one row, Retry
// and Replan rule nodes, skip wires, replan wires along the bottom. Who draws what: the caller renders the nodes, the
// stacks (a closed block), the box title bars and the rule nodes through ctx hooks; this file places them and draws the
// boxes, the wires, the split and join of a parallel block, the loops of a repeat and of the mission, the replan wires
// and the Start and Done nodes. Uses el from app.js.

const GRAPH_SVG = 'http://www.w3.org/2000/svg';
function gsvg(tag, attrs = {}, text) {
  const node = document.createElementNS(GRAPH_SVG, tag);
  Object.entries(attrs).forEach(([key, value]) => node.setAttribute(key, value));
  if (text !== undefined) node.textContent = text;
  return node;
}

// The room things take (px). node: a node's size, head its header strip, line the y of its input and done ports (the
// flow line) and fail the y of its fail port, from its top; end: the Start and Done nodes; gapX between nodes (the
// wire and its "+" spot); pad inside a box, boxHead and laneHead its title bar, gutter the room a repeat keeps at its
// bottom for the return wire, tail the room at a box's right end for its "+" spot; terminal and rule: the Fail terminal
// and the Retry / Replan nodes under the flow, rowGap above their row (laneRowGap above a parallel lane's own row),
// laneGap between replan wires. fields: the node
// shows its fields; ports: the editor's ports; rules: the failure routes.
const GRAPH_METRICS = {
  compact: { node: { w: 160, h: 48 }, head: 22, line: 35, end: { w: 72, h: 30 }, gapX: 36, gapY: 12, pad: 12, boxHead: 28,
    laneHead: 24, boxMin: 160, gutter: 14, tail: 0, left: 16, right: 24, top: 14, bottom: 18, dot: 10, fields: true,
    ports: false, rules: false, kindWord: false },
  phone: { node: { w: 132, h: 44 }, head: 20, line: 32, end: { w: 64, h: 28 }, gapX: 28, gapY: 10, pad: 10, boxHead: 28,
    laneHead: 24, boxMin: 132, gutter: 12, tail: 0, left: 12, right: 16, top: 12, bottom: 16, dot: 10, fields: false,
    ports: false, rules: false, kindWord: false },
  editor: { node: { w: 220, h: 64 }, head: 24, line: 38, fail: 54, end: { w: 88, h: 36 }, gapX: 64, gapY: 14, pad: 16,
    boxHead: 28, laneHead: 24, boxMin: 220, gutter: 18, tail: 40, left: 24, right: 48, top: 24, bottom: 28, dot: 12,
    terminal: { w: 120, h: 40 }, rule: { w: 100, h: 36 }, rowGap: 44, laneRowGap: 22, laneGap: 52, fields: true, ports: true, rules: true,
    kindWord: true },
  editorPhone: { node: { w: 168, h: 52 }, head: 22, line: 33, fail: 45, end: { w: 72, h: 32 }, gapX: 56, gapY: 12, pad: 12,
    boxHead: 28, laneHead: 24, boxMin: 168, gutter: 16, tail: 40, left: 16, right: 40, top: 20, bottom: 24, dot: 12,
    terminal: { w: 104, h: 36 }, rule: { w: 92, h: 34 }, rowGap: 40, laneRowGap: 20, laneGap: 48, fields: false, ports: true, rules: true,
    kindWord: true },
};
const FRAME_KINDS = ['include', 'group', 'repeat', 'parallel', 'branch'];
const isFrame = (node) => FRAME_KINDS.includes(node.kind);

// ---------- the words on the canvas (the text rule: a name, a kind word, at most two fields) ----------
const FLOW_KIND_WORDS = { step: 'Action', include: 'Mission', group: 'Group', repeat: 'Repeat', parallel: 'Parallel',
  branch: 'Lane', start: 'Start', end: 'End', abort: 'Fail', retry: 'Rule', replan: 'Rule' };
// "to_shelf" -> "To shelf"
const humanise = (s) => { const t = String(s ?? '').replace(/[_-]+/g, ' ').replace(/\s+/g, ' ').trim(); return t ? t[0].toUpperCase() + t.slice(1) : ''; };
// a step's own id: the plan's place ids (s3, s1_s7: core's names for a step without one) are none; an include's
// prefix (s1_) goes
function ownId(id) {
  let s = String(id ?? '');
  while (/^s\d+_/.test(s)) s = s.replace(/^s\d+_/, '');
  return /^s\d+$/.test(s) ? '' : s;
}
// a node's name: a step's id, else what it does (stepTitle, graph-run.js: the kind of motion of a move from the plan or
// its args, "navigate to", "grip open"; else the skill); an include's mission; a block's id, else its kind; a lane's id,
// else Lane n
function flowName(node) {
  if (node.kind === 'step') {
    const s = node.step || {};
    const does = s.skill === 'grip' && s.args?.action ? `grip ${s.args.action}`   // the plan gives a grip no kind
      : typeof stepTitle === 'function' ? stepTitle(s) : s.skill;
    return humanise(ownId(s.id ?? node.id)) || humanise(does) || 'Step';
  }
  if (node.kind === 'include') return humanise(node.include) || 'Mission';
  if (node.kind === 'branch') return humanise(ownId(node.id)) || `Lane ${node.n || 1}`;
  return humanise(ownId(node.id)) || FLOW_KIND_WORDS[node.kind] || '';
}
const flowValue = (v) => (Array.isArray(v) ? `[${v.map(flowValue).join(', ')}]`
  : v && typeof v === 'object' ? Object.entries(v).map(([k, x]) => `${k} ${flowValue(x)}`).join(' ') : String(v));
// a step's fields: its main argument (to, hands + action, seconds, else the first), an include's with values (two)
function stepFields(step) {
  if (!step) return [];
  if (step.include) return Object.entries(step.with || {}).slice(0, 2).map(([k, v]) => [k, flowValue(v)]);
  const a = step.args || {};
  if (a.to !== undefined) return [['to', flowValue(a.to)]];
  if (a.action !== undefined) return [['hands', flowValue(a.hands ?? 'all')], ['action', flowValue(a.action)]];
  if (a.seconds !== undefined) return [['seconds', flowValue(a.seconds)]];
  const first = Object.entries(a)[0];
  return first ? [[first[0], flowValue(first[1])]] : [];
}
// a block's field: a repeat's rule, a parallel's join when it is first
function blockFields(node) {
  const b = node.block || {};
  if (node.kind === 'repeat') {
    const r = b.repeat || {};
    return r.times ? [['', `×${r.times}`]] : r.while ? [['while', String(r.while)]] : r.until ? [['until', String(r.until)]] : [];
  }
  if (node.kind === 'parallel' && b.join === 'first') return [['join', 'first']];
  return [];
}
const flowField = ([k, v]) => el('span', { class: 'fl-field' }, k ? el('span', { class: 'fl-k', text: k }) : '', el('span', { class: 'fl-v', text: v }));
// a node's header strip: the name, the kind word (none when it says the name again)
const flowHead = (name, kind) => el('div', { class: 'fl-head' }, el('span', { class: 'fl-name', text: name }),
  kind && kind !== name ? el('span', { class: 'fl-kind', text: kind }) : '');

// ---------- the tree ----------
// A node: {kind, key, name, children} for include | group | repeat | branch, {kind: 'parallel', branches: [branch]},
// {kind: 'step', key: id, step, index, name} for a skill call (index: its place in plan.steps). The root is a group
// without a box ({kind: 'root'}); root.loop says the mission repeats. Every frame gets leaves: the indices of the
// steps inside it, in order. name: the instruction (the tooltip, the pop-up); the canvas shows flowName.
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
      node.branches = (n.branches || []).map((b, j) => ({ kind: 'branch', key: `${key}/b${j}`, id: b.id, n: j + 1,
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
// ctx: {metrics, width, isOpen(frame) -> bool, keyOf(leaf index) -> step key, routeOf(item) -> the editor's failure
// route {kind: abort | retry | skip | replan, field, to: a step key} or null, replans: [[from key, to key]] (the Run
// view's replan wires)}. The flow runs left to right: a sequence is a row whose nodes share one flow line, a block is a
// box around its row, a parallel block a box with its lanes as rows under each other. Places every node: items (key ->
// {node, kind: step | stack | dot | start | end, x, y, w, h: its box; cx, cy: its centre; nw, nh: its size; line: the y
// of its flow ports; inX, outX: the x of its input and output}), frames (open: {node, x, y, w, h, head, line}), owner
// (a step's key -> the key of the item that shows it: itself, or the stack of its closed frame; a frame's own key -> its
// stack, or the item of its first step while open), rules (the rule row and the Replan nodes: {kind, item, field, x, y,
// w, h}), lanes (the replan wires: {from, to, y, rule}) and loopY (the mission's loop wire).
function layoutGraph(root, ctx) {
  const M = ctx.metrics;
  const items = new Map();
  const frames = [];
  const owner = new Map();

  const laneStack = [];  // the parallel lanes being placed, innermost last: a node's terminal goes in its lane's row
  const put = (key, rec) => { items.set(key, rec); return rec; };
  const nodeItem = (node, kind, size, line) => ({ w: size.w, h: size.h, line, place(x, y) {
    put(node.key, { node, kind, x, y, w: size.w, h: size.h, nw: size.w, nh: size.h, cx: x + size.w / 2, cy: y + size.h / 2,
      line: y + line, inX: x, outX: x + size.w, lane: laneStack[laneStack.length - 1] || null });
    owner.set(node.key, node.key);
    if (kind === 'stack') node.leaves.forEach((i) => owner.set(ctx.keyOf(i), node.key));
  } });
  const dotAt = (key, node, x, y) => put(key, { node, kind: 'dot', x: x - M.dot / 2, y: y - M.dot / 2, w: M.dot, h: M.dot,
    nw: M.dot, nh: M.dot, cx: x, cy: y, line: y, inX: x, outX: x });
  // a row of placed things on one flow line
  const row = (sized) => {
    const line = Math.max(0, ...sized.map((s) => s.line));
    const below = Math.max(0, ...sized.map((s) => s.h - s.line));
    const w = sized.reduce((t, s) => t + s.w, 0) + Math.max(0, sized.length - 1) * M.gapX;
    return { w, h: line + below, line, place(x, y) {
      let cx = x;
      sized.forEach((s) => { s.place(cx, y + line - s.line); cx += s.w + M.gapX; });
    } };
  };

  // the editor's rule row: a node whose route puts a node under it (a Fail terminal, a Retry, a Replan of itself); a
  // lane holds its own row under its nodes when one of its nodes (not those of a lane nested in it) has one
  const rowRoute = (route, key) => !!route && (route.kind === 'abort' || route.kind === 'retry'
    || (route.kind === 'replan' && (!route.to || route.to === key)));
  const needsRow = (node, kind) => rowRoute(ctx.routeOf({ node, kind }), node.key);
  const laneHasRow = (node) => (node.children || []).some((c) => (c.kind === 'step' ? needsRow(c, 'step')
    : !ctx.isOpen(c) ? needsRow(c, 'stack') : c.kind === 'parallel' ? false : laneHasRow(c)));
  const laneRowH = M.rules && ctx.routeOf ? (M.laneRowGap || 0) + Math.max(M.terminal.h, M.rule.h) : 0;

  function measure(node) {
    if (node.kind === 'step') return nodeItem(node, 'step', M.node, M.line);
    if (!ctx.isOpen(node)) return nodeItem(node, 'stack', M.node, M.line);
    const head = node.kind === 'branch' ? M.laneHead : M.boxHead;
    if (node.kind === 'parallel') {  // lanes as rows; the split on the box's left edge, the join on its right
      const lanes = node.branches.map(measure);
      const fan = M.gapX * 0.75;  // room for the fan wires and their "+" spots
      const laneW = Math.max(M.boxMin, ...lanes.map((l) => l.w));
      lanes.forEach((l) => { l.w = laneW; });
      const body = lanes.length ? lanes.reduce((t, l) => t + l.h, 0) + (lanes.length - 1) * M.gapY : 2 * M.dot;
      const self = { w: laneW + 2 * (M.pad + fan), h: head + M.pad + body + M.pad,
        line: head + M.pad + (lanes.length ? lanes[0].line : M.dot), place(x, y) {
          frames.push({ node, x, y, w: self.w, h: self.h, head, line: y + self.line });
          dotAt(`${node.key}:split`, node, x, y + self.line);
          let ly = y + head + M.pad;
          lanes.forEach((l) => { l.place(x + M.pad + fan, ly); ly += l.h + M.gapY; });
          dotAt(`${node.key}:join`, node, x + self.w, y + self.line);
          owner.set(node.key, `${node.key}:split`);
        } };
      return self;
    }
    // a box around a row (an empty one keeps a dot, so the wires and the editor's "+" reach it)
    const kids = (node.children || []).map(measure);
    const r = kids.length ? row(kids) : null;
    const inner = r || { w: 2 * M.dot + 24, h: 2 * M.dot + 8, line: M.dot + 4 };
    const laneRow = node.kind === 'branch' && laneRowH && laneHasRow(node) ? laneRowH : 0;
    const bottom = M.pad + (node.kind === 'repeat' ? M.gutter : 0) + laneRow;
    const self = { w: Math.max(M.boxMin, M.pad + inner.w + M.pad + M.tail), h: head + M.pad + inner.h + bottom,
      line: head + M.pad + inner.line, place(x, y) {
        frames.push({ node, x, y, w: self.w, h: self.h, head, line: y + self.line });
        if (node.kind === 'branch') laneStack.push({ rowY: laneRow ? y + head + M.pad + inner.h + M.laneRowGap : null });
        if (r) r.place(x + M.pad, y + head + M.pad);
        else { dotAt(`${node.key}:empty`, node, x + M.pad + M.dot, y + self.line); owner.set(node.key, `${node.key}:empty`); }
        if (node.kind === 'branch') laneStack.pop();
        if (node.leaves.length) owner.set(node.key, owner.get(ctx.keyOf(node.leaves[0])));
      } };
    return self;
  }
  // the root: Start, the mission's own row, Done (unless the mission repeats: then the loop wire back to Start)
  const start = { kind: 'start', key: '@start' };
  const end = { kind: 'end', key: '@end' };
  const parts = [nodeItem(start, 'start', M.end, M.end.h / 2), ...root.children.map(measure)];
  if (!root.loop) parts.push(nodeItem(end, 'end', M.end, M.end.h / 2));
  row(parts).place(M.left, M.top);

  let flowBottom = M.top;
  let right = 0;
  items.forEach((it) => { flowBottom = Math.max(flowBottom, it.y + it.h); right = Math.max(right, it.x + it.w); });
  frames.forEach((f) => { flowBottom = Math.max(flowBottom, f.y + f.h); right = Math.max(right, f.x + f.w); });
  let y = flowBottom;
  const rules = [];
  const lanes = [];
  if (M.rules && ctx.routeOf) {
    // the rule row: a Fail terminal, a Retry or a Replan of itself under each node, in one row (moved right on overlap);
    // a node in a parallel lane has it in its lane's own row, under its node, so no fail wire crosses another lane
    const rowItems = [];
    const jumps = [];
    items.forEach((it) => {
      if (it.kind !== 'step' && it.kind !== 'stack') return;
      const route = ctx.routeOf(it);
      if (!route) return;
      it.route = route;
      const target = route.kind === 'replan' && route.to ? items.get(owner.get(route.to)) : null;
      if (route.kind === 'replan' && target && target !== it) { jumps.push({ it, target, route }); return; }
      if (route.kind === 'skip') return;
      const size = route.kind === 'abort' ? M.terminal : M.rule;
      rowItems.push({ kind: route.kind, item: it, field: route.field || '', w: size.w, h: size.h, want: it.cx - size.w / 2,
        lane: it.lane?.rowY != null ? it.lane : null });
    });
    const rowY = flowBottom + M.rowGap;
    const edges = new Map();  // a row (a lane, or null: the flow's) -> its right edge so far
    rowItems.sort((a, b) => a.want - b.want).forEach((r) => {
      const edge = edges.has(r.lane) ? edges.get(r.lane) : -Infinity;
      r.x = Math.max(r.want, edge + 12);
      r.y = r.lane ? r.lane.rowY : rowY;
      edges.set(r.lane, r.x + r.w);
      right = Math.max(right, r.x + r.w);
      rules.push(r);
    });
    const main = rowItems.filter((r) => !r.lane);
    if (main.length) y = rowY + Math.max(...main.map((r) => r.h));
    // the replan wires along the bottom, one lane each, a Replan node on the wire's middle
    jumps.forEach(({ it, target, route }, k) => {
      const ly = y + M.laneGap * (k + 0.75);
      const a = it.outX + 18;
      const b = target.inX - 18;
      const rule = { kind: 'replan', item: it, field: route.field || '', w: M.rule.w, h: M.rule.h, onWire: true };
      rule.x = (a + b) / 2 - rule.w / 2;
      rule.y = ly - rule.h / 2;
      rules.push(rule);
      lanes.push({ from: it, to: target, y: ly, rule });
    });
    if (jumps.length) y += M.laneGap * (jumps.length + 0.25);
  } else {
    (ctx.replans || []).forEach(([fromKey, toKey]) => {
      const a = items.get(owner.get(fromKey));
      const b = items.get(owner.get(toKey));
      if (!a || !b || a === b) return;
      lanes.push({ from: a, to: b, y: y + 14 + lanes.length * 10 });
    });
    if (lanes.length) y = lanes[lanes.length - 1].y + 4;
  }
  const loopY = root.loop ? y + 26 : null;
  if (loopY !== null) y = loopY + 10;
  return { root, items, frames, owner, rules, lanes, loopY, metrics: M,
    width: Math.max(ctx.width, right + M.right), height: y + M.bottom };
}

// ---------- the drawing ----------
// a bezier S-curve from (x1, y1) to (x2, y2), leaving and arriving level; a long one between two levels runs level
// and bends only before its end (a lane's join), so it does not cut across the nodes between
const graphCurve = (x1, y1, x2, y2) => {
  if (x2 - x1 > 200 && Math.abs(y2 - y1) > 4) {
    const bx = x2 - 120;
    return `M${x1},${y1} L${bx},${y1} C${bx + 60},${y1} ${x2 - 60},${y2} ${x2},${y2}`;
  }
  const dx = Math.max(24, Math.abs(x2 - x1) / 2);
  return `M${x1},${y1} C${x1 + dx},${y1} ${x2 - dx},${y2} ${x2},${y2}`;
};
// a wire from item a's output to item b's input: its path and its middle (for a "+" spot)
function graphWire(a, b) {
  const long = b.inX - a.outX > 200 && Math.abs(b.line - a.line) > 4;  // level, then the bend (graphCurve)
  return { d: graphCurve(a.outX, a.line, b.inX, b.line),
    mid: long ? { x: (a.outX + b.inX - 120) / 2, y: a.line } : { x: (a.outX + b.inX) / 2, y: (a.line + b.line) / 2 } };
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
// an arrow head for the wires that run back (loops, replans, retries); ids: the drawing's own prefix, so two graphs on
// one page do not point at each other's markers
const graphMarker = (ids, name) => {
  const m = gsvg('marker', { id: `${ids}-head-${name}`, class: `pg-head ${name}`, viewBox: '0 0 10 10', refX: 8, refY: 5,
    markerWidth: 8, markerHeight: 8, markerUnits: 'userSpaceOnUse', orient: 'auto-start-reverse' });
  m.append(gsvg('path', { d: 'M0,1 L9,5 L0,9 z' }));
  return m;
};

// Draws a layout into a new .pg-graph. ctx hooks: renderItem(item) -> the node's element (a step or a stack),
// renderHead(frame, f) -> a box's title bar, renderRule(rule) -> a Fail terminal, Retry or Replan node (the editor),
// stateOf(item) -> 'running' | 'paused' | 'pending' | a finished outcome (the Run view's wires; none: the editor's
// plain wires), frameWord(frame) -> its state word (for the box outline), loop: the mission loop's words ('' when it
// does not repeat), ids: the prefix of the drawing's element ids (its arrow heads; one per graph on the page).
// Returns {node, items, height, replans: how many replan wires were drawn, flow: the wires in run order as {a, b, mid}
// (the items joined and the wire's middle; a null a: from Start, a null b: to Done), so the editor can put its "+" on
// them}.
function drawGraph(layout, ctx) {
  const { items, frames, metrics: M } = layout;
  const width = layout.width;
  const height = layout.height;
  const graph = el('div', { class: `pg-graph fl${M.ports ? ' fl-ports' : ''}`, role: 'group', 'aria-label': ctx.label || 'Mission graph' });
  const back = gsvg('svg', { class: 'pg-lines', 'aria-hidden': 'true' });
  const wires = gsvg('svg', { class: 'pg-lines', 'aria-hidden': 'true' });
  const notes = gsvg('svg', { class: 'pg-lines', 'aria-hidden': 'true' });
  [back, wires, notes].forEach((s) => { s.setAttribute('width', width); s.setAttribute('height', height); });
  graph.style.cssText = `width: ${width}px; height: ${height}px; --fl-line: ${M.line}px; --fl-fail: ${M.fail || 0}px; --fl-head: ${M.head}px`;
  graph.append(back, wires);
  const ids = ctx.ids || 'pg';
  const arrow = (name) => `url(#${ids}-head-${name})`;
  const defs = gsvg('defs');
  ['back', 'fail', 'loop', 'now'].forEach((name) => defs.append(graphMarker(ids, name)));
  wires.append(defs);
  const place = (node, x, y, w, h) => {
    node.style.left = `${x}px`;
    node.style.top = `${y}px`;
    node.style.width = `${w}px`;
    node.style.height = `${h}px`;
    graph.append(node);
    return node;
  };

  // the boxes: a translucent box with its title bar (a lane: a thinner one)
  frames.forEach((f) => {
    const word = ctx.frameWord ? ctx.frameWord(f.node) : '';
    back.append(gsvg('rect', { class: `fl-box ${f.node.kind} ${word}`, x: f.x + 0.5, y: f.y + 0.5, width: f.w - 1, height: f.h - 1, rx: 8 }));
    const head = ctx.renderHead(f.node, f);
    if (head) place(head, f.x, f.y, f.w, f.head);
  });
  // the items: the caller's nodes, the Start and Done nodes, the dots (a parallel's split and join, an empty box's),
  // each dot classed by its own key's end: pg-dot-split, pg-dot-join, pg-dot-empty
  items.forEach((it, key) => {
    if (it.kind === 'dot') {
      wires.append(gsvg('circle', { class: `pg-dot-${key.slice(key.lastIndexOf(':') + 1)} ${ctx.stateOf ? ctx.stateOf(it) : ''}`,
        cx: it.cx, cy: it.cy, r: M.dot / 2 - 1 }));
      return;
    }
    const node = it.kind === 'start' || it.kind === 'end'
      ? el('div', { class: `fl-node fl-end ${it.kind}`, role: 'img', 'aria-label': it.kind === 'start' ? 'Start' : 'Done' },
        flowHead(it.kind === 'start' ? 'Start' : 'Done', M.kindWord ? FLOW_KIND_WORDS[it.kind] : ''))
      : ctx.renderItem(it);
    if (M.ports && it.kind === 'start') node.append(el('span', { class: 'pg-port done', 'aria-hidden': 'true' }));
    if (M.ports && it.kind === 'end') node.append(el('span', { class: 'pg-port in', 'aria-hidden': 'true' }));
    it.el = place(node, it.x, it.y, it.w, it.h);
  });

  // the flow: entry and exit items of every node, then a wire from each item to the next in run order
  const live = (st) => st === 'running' || st === 'paused';
  const finished = (it) => { const st = ctx.stateOf(it); return st !== 'pending' && !live(st); };
  const cls = (a, b) => {
    if (!ctx.stateOf) return '';
    if (b.kind === 'end') return finished(a) ? 'done' : 'todo';
    const st = ctx.stateOf(b);
    return live(st) ? 'now' : st !== 'pending' ? 'done' : 'todo';
  };
  const entry = (node) => {
    if (node.kind === 'step' || (isFrame(node) && !ctx.isOpen(node))) return items.get(layout.owner.get(node.kind === 'step' ? node.key : ctx.keyOf(node.leaves[0])) || node.key);
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
  const next = new Map();  // an item -> the item its done wire leads to (a skip's fail wire goes there too)
  const flowing = (path) => {  // the graph is rebuilt every second: a flowing wire runs at the phase of a page-wide clock
    if (path.classList.contains('now')) path.style.animationDelay = `-${(performance.now() % 700) / 1000}s`;
    return path;
  };
  const join = (a, b) => {
    if (!a || !b || a === b || seen.has(`${a.node.key}>${b.node.key}`)) return;
    seen.add(`${a.node.key}>${b.node.key}`);
    if (!next.has(a)) next.set(a, b);
    const w = graphWire(a, b);
    flow.push({ a: a.kind === 'start' ? null : a, b: b.kind === 'end' ? null : b, mid: w.mid });
    wires.append(flowing(gsvg('path', { class: `pg-wire ${cls(a, b)}`, d: w.d })));
  };
  const wireSequence = (node, frame) => {
    const list = node.children || [];
    list.forEach((c, k) => {
      if (k) join(exit(list[k - 1]), entry(c));
      wireNode(c);
    });
    if (node.kind === 'repeat' && list.length && frame) {  // the return wire along the box's bottom, no words
      const from = exit(list[list.length - 1]);
      const to = entry(list[0]);
      if (!from || !to) return;
      const rx = frame.x + frame.w - 8;
      const lx = frame.x + 8;
      const low = frame.y + frame.h - M.gutter / 2 - 3;
      wires.append(gsvg('path', { class: 'pg-wire loop', 'marker-end': arrow('loop'),
        d: graphRounded([[from.outX, from.line], [rx, from.line], [rx, low], [lx, low], [lx, to.line], [to.inX - 2, to.line]], 10) }));
    }
  };
  const wireNode = (node) => {
    if (node.kind === 'step') return;
    if (isFrame(node) && !ctx.isOpen(node)) return;
    const frame = frames.find((f) => f.node === node);
    if (node.kind === 'parallel') {
      const split = items.get(`${node.key}:split`);
      const joinDot = items.get(`${node.key}:join`);
      node.branches.forEach((b) => { join(split, entry(b)); wireNode(b); join(exit(b), joinDot); });
      return;
    }
    wireSequence(node, frame);
  };
  const first = items.get('@start');
  const last = items.get('@end');
  join(first, entry(layout.root));
  wireNode(layout.root);
  const tail = exit(layout.root);
  if (last) join(tail || first, last);
  // the mission's loop: from the last node back to Start along the bottom, its words on the wire's middle
  if (layout.loopY !== null && tail) {
    const x = tail.outX + 22;
    flow.push({ a: tail, b: null, mid: { x, y: tail.line } });
    wires.append(gsvg('path', { class: 'pg-wire loop', 'marker-end': arrow('loop'),
      d: graphRounded([[tail.outX, tail.line], [x, tail.line], [x, layout.loopY], [first.cx, layout.loopY], [first.cx, first.y + first.h + 3]], 14) }));
    if (ctx.loop) notes.append(gsvg('text', { class: 'pg-wire-label loop', x: (x + first.cx) / 2, y: layout.loopY + 4, 'text-anchor': 'middle' }, ctx.loop));
  }

  // the failure routes (the editor): a fail wire from the fail port to the terminal, the rule node and back, the next
  // node (skip), or along the bottom to the step a replan starts from
  const failAt = (it) => [it.x + it.w, it.y + M.fail];
  layout.rules.forEach((r) => {
    const node = ctx.renderRule(r);
    if (node) r.el = place(node, r.x, r.y, r.w, r.h);
    if (r.kind === 'replan' && layout.lanes.some((l) => l.rule === r)) return;
    const [fx, fy] = failAt(r.item);
    const tx = r.x + r.w / 2;
    wires.append(gsvg('path', { class: 'pg-wire fail', d: `M${fx},${fy} C${fx + 36},${fy} ${tx},${r.y - 36} ${tx},${r.y}` }));
    if (r.kind !== 'abort') {  // a retry, a replan of itself: back into the node's input
      const my = r.y + r.h / 2;
      const it = r.item;
      wires.append(gsvg('path', { class: 'pg-wire fail back', 'marker-end': arrow('fail'),
        d: `M${r.x},${my} C${r.x - 40},${my} ${it.inX - 40},${it.line} ${it.inX - 2},${it.line}` }));
    }
  });
  items.forEach((it) => {
    if (it.route?.kind !== 'skip') return;
    const to = next.get(it);
    if (!to) return;
    const [fx, fy] = failAt(it);
    wires.append(gsvg('path', { class: 'pg-wire fail skip', d: graphCurve(fx, fy, to.inX, to.line) }));
  });
  layout.lanes.forEach(({ from, to, y, rule }) => {
    if (rule) {  // the editor: from the fail port, along the lane, into the target's input
      const [fx, fy] = failAt(from);
      wires.append(gsvg('path', { class: 'pg-wire fail', 'marker-end': arrow('fail'),
        d: graphRounded([[fx, fy], [fx + 18, fy], [fx + 18, y], [to.inX - 18, y], [to.inX - 18, to.line], [to.inX - 2, to.line]], 12) }));
    } else {  // the Run view: thin, under the nodes, into the target from below
      wires.append(gsvg('path', { class: 'pg-wire fail thin', 'marker-end': arrow('fail'),
        d: graphRounded([[from.cx, from.y + from.h], [from.cx, y], [to.cx, y], [to.cx, to.y + to.h + 2]], 8) }));
    }
  });
  graph.append(notes);
  return { node: graph, items, height, replans: layout.lanes.length, flow };
}
