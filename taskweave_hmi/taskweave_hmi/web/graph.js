'use strict';
// taskweave HMI, Run tab: the running plan as a graph. Steps top to bottom, coloured by what moves (a move step's kind:
// base and straight drive, hand, object, posture and joints are the arm; other skills by name); edges for the decision
// rules (retry, replan, skip, abort, loop); the active step lights up. Rules not in the live plan message (mission
// defaults, replan_from, stop_when) come from the mission file. Uses $, el, api, state from app.js.

const SVG = 'http://www.w3.org/2000/svg';
// the lane of a step: its kind of motion (step.kind, filled when the plan is validated: the live plan, the preview), else
// its skill (cell schema 3, and a step whose kind a placeholder hides)
const MOVE_LANES = { base: 'drive', straight: 'drive', hand: 'arm', object: 'arm', posture: 'arm', joints: 'arm' };
const SKILL_KINDS = { navigate_to: 'drive', drive_straight: 'drive', move_joints: 'arm', move_gripper: 'arm',
  move_object: 'arm', set_posture: 'arm', grip: 'grip', wait: 'wait' };
const laneOf = (step) => MOVE_LANES[step.kind] || SKILL_KINDS[step.skill] || 'other';
const G = { width: 384, nodeX: 70, nodeW: 220, nodeH: 52, gap: 30, top: 44, lane: 34, abortLane: 368 };
const BREATHE_MS = 2800;  // period of node-breathe for the running step (style.css)
const missionCache = {};

function svg(tag, attrs = {}, text) {
  const node = document.createElementNS(SVG, tag);
  Object.entries(attrs).forEach(([key, value]) => node.setAttribute(key, value));
  if (text !== undefined) node.textContent = text;
  return node;
}

function mainArg(step) {
  const a = step.args || {};
  if (step.skill === 'move' && typeof moveSummary === 'function') return moveSummary(a);  // app.js: "to tray_1/place"
  if (a.target) return String(a.target);
  if (a.pose) return String(a.pose);
  if (a.to && typeof a.to === 'object') {  // move_gripper / move_object: the target variant's main value
    const [key, value] = Object.entries(a.to)[0] || [];
    if (key === 'point' || key === 'pose') return String(value);
    if (key === 'onto') return `${value}/${a.to.contact ?? '?'}`;
    if (key) return `${key} ${Array.isArray(value) ? `[${value.join(', ')}]` : value}`;
  }
  if (a.action) return `${[].concat(a.hands ?? []).join('+')} ${a.action}`.trim();
  if (a.elbow) return `elbow ${a.elbow}`;
  if (a.seconds !== undefined) return `${a.seconds} s`;
  const first = Object.entries(a)[0];
  return first ? `${first[0]} ${typeof first[1] === 'object' ? '…' : first[1]}` : '';
}

async function missionFor(plan) {
  const name = missionOf(plan);
  if (!(name in missionCache)) {
    missionCache[name] = null;  // one request per mission
    const { ok, data } = await api('GET', `/api/missions/${encodeURIComponent(name)}`);
    missionCache[name] = ok ? data : {};
    renderGraph();
  }
  return missionCache[name];
}

// The rules each step runs with: the plan's values, mission defaults where the plan says "default".
function rules(plan, mission) {
  const defaults = mission?.defaults || {};
  const ids = plan.steps.map((s) => s.id);
  return plan.steps.map((step, i) => {
    const source = mission?.steps?.[i] || {};
    const replanFrom = source.replan_from && ids.includes(source.replan_from) ? ids.indexOf(source.replan_from) : i;
    return {
      retries: step.retries >= 0 ? step.retries : (defaults.retries ?? 0),
      onFailure: step.on_failure || defaults.on_failure || 'abort',
      replanTo: replanFrom,
    };
  });
}

function stepState(snap, plan, live, step, index) {
  const s = snap.status;
  const active = (s.state === 'running' || s.state === 'paused') && s.plan_id === plan.plan_id;
  if (active && s.step_index === index) return s.state === 'paused' ? 'paused' : 'running';
  const event = live ? snap.outcomes[step.id] : null;
  if (event?.event === 'step_finished') return event.outcome.toLowerCase();
  return 'pending';
}

function renderGraph() {
  const box = $('plan-graph');
  const snap = state.snapshot;
  if (!box || box.hidden || !snap) return;
  const { plan, live } = shownPlan();  // app.js: the live plan, or a preview to pick a start step in
  if (!plan || !plan.steps.length) { box.replaceChildren(el('p', { class: 'muted', text: plan?.message || 'No plan yet' })); return; }
  const pickable = snap.status.state !== 'running' && snap.status.state !== 'paused';
  const mission = missionCache[missionOf(plan)];
  if (mission === undefined) missionFor(plan);
  const r = rules(plan, mission);
  const n = plan.steps.length;
  const rowY = (i) => G.top + i * (G.nodeH + G.gap);
  const cx = G.nodeX + G.nodeW / 2;
  const right = G.nodeX + G.nodeW;
  const loop = plan.loop || mission?.loop || 'once';
  const anyAbort = r.some((x) => x.onFailure === 'abort');
  const endY = rowY(n);
  const height = endY + 70;
  const root = svg('svg', { viewBox: `0 0 ${G.width} ${height}`, class: 'plan-graph', role: 'img',
    'aria-label': `Plan ${plan.plan_id}: ${n} steps` });
  const defs = svg('defs');
  root.append(defs);
  ['ok', 'fail', 'loop'].forEach((kind) => {
    const marker = svg('marker', { id: `arrow-${kind}`, viewBox: '0 0 10 10', refX: 9, refY: 5, markerWidth: 6, markerHeight: 6,
      orient: 'auto-start-reverse' });
    marker.append(svg('path', { d: 'M0,0 L10,5 L0,10 z', class: `head-${kind}` }));
    defs.append(marker);
  });
  const edge = (d, kind, dashed = false) => svg('path', { d, class: `edge edge-${kind}${dashed ? ' dashed' : ''}`,
    'marker-end': `url(#arrow-${kind})` });
  const label = (x, y, text, cls = '', anchor = 'start') => svg('text', { x, y, class: `edge-label ${cls}`, 'text-anchor': anchor }, text);

  // start, step chain, end
  root.append(svg('rect', { x: cx - 34, y: 6, width: 68, height: 22, rx: 11, class: 'terminal' }),
    svg('text', { x: cx, y: 21, class: 'terminal-text', 'text-anchor': 'middle' }, 'start'),
    edge(`M${cx},28 L${cx},${rowY(0) - 2}`, 'ok'));
  plan.steps.forEach((step, i) => {
    const y = rowY(i);
    const kind = laneOf(step);
    const status = stepState(snap, plan, live, step, i);
    const group = svg('g', { class: `node kind-${kind} state-${status}${pickable ? ' pickable' : ''}`
      + `${pickable && step.id === state.startStep ? ' start-here' : ''}` });
    if (pickable) group.addEventListener('click', () => pickStart(step));  // app.js
    const uses = step.resources?.length ? `\nuses ${step.resources.join(', ')}` : '';
    const title = svg('title', {}, `${i + 1}. ${step.instruction}\n${step.skill}${step.kind ? ` ${step.kind}` : ''} `
      + `${JSON.stringify(step.args)}${uses}`);
    const nodeBox = svg('rect', { x: G.nodeX, y, width: G.nodeW, height: G.nodeH, rx: 14, class: 'node-box' });
    // the graph is rebuilt on every status update: start the breathing at the phase of a page-wide clock, so it
    // runs on (in and out) instead of restarting with every update
    if (status === 'running') nodeBox.style.animationDelay = `-${(performance.now() % BREATHE_MS) / 1000}s`;
    group.append(title, nodeBox,
      svg('rect', { x: G.nodeX + 9, y: y + 11, width: 5, height: G.nodeH - 22, rx: 2.5, class: 'node-kind' }),
      svg('text', { x: G.nodeX + 24, y: y + 22, class: 'node-title' }, `${i + 1} · ${step.skill}${step.kind ? ` ${step.kind}` : ''}`),
      svg('text', { x: G.nodeX + 24, y: y + 39, class: 'node-arg' }, mainArg(step).slice(0, 28)));
    if (status === 'running' || status === 'paused') {
      group.append(svg('text', { x: right - 12, y: y + 22, class: 'node-state', 'text-anchor': 'end' }, status));
    } else if (status !== 'pending') {  // finished: a round badge
      const glyph = { succeeded: '✓', failed: '✕', timeout: '⏱', rejected: '✕', canceled: '–' }[status] || '•';
      group.append(svg('circle', { cx: right - 18, cy: y + G.nodeH / 2, r: 10, class: 'node-badge' }),
        svg('text', { x: right - 18, y: y + G.nodeH / 2 + 4.5, class: 'node-badge-text', 'text-anchor': 'middle' }, glyph),
        svg('title', {}, status));
    }
    root.append(group);
    // next step (or end): success, and skip when the step fails with on_failure skip
    const nextY = i + 1 < n ? rowY(i + 1) - 2 : endY - 2;
    root.append(edge(`M${cx},${y + G.nodeH} L${cx},${nextY}`, 'ok'));
    if (r[i].onFailure === 'skip') root.append(label(cx + 6, y + G.nodeH + 18, 'also if it fails (skip)', 'muted-label'));
    // retries: a loop on the right edge of the node
    if (r[i].retries > 0) {
      // upper half of the node; the abort stub uses the lower half
      root.append(edge(`M${right},${y + 8} C${right + 24},${y + 2} ${right + 24},${y + 30} ${right + 2},${y + 26}`, 'fail', true),
        label(right + 26, y + 20, `retry ×${r[i].retries}`, 'fail-label'));
    }
    // replan: back to the step it replans from, through the left lane
    if (r[i].onFailure === 'replan') {
      const ty = rowY(r[i].replanTo) + G.nodeH / 2;
      const lane = G.lane - (i % 3) * 6;
      root.append(edge(`M${G.nodeX},${y + G.nodeH / 2 + 6} L${lane},${y + G.nodeH / 2 + 6} L${lane},${ty} L${G.nodeX - 2},${ty}`, 'fail', true),
        label(lane + 3, y + G.nodeH / 2 + 18, 'replan', 'fail-label'));
    }
    // abort: a stub into the right failure lane
    if (r[i].onFailure === 'abort') {
      root.append(svg('path', { d: `M${right},${y + G.nodeH - 10} L${G.abortLane},${y + G.nodeH - 10}`, class: 'edge edge-fail' }));
    }
  });
  // end: done, or loop back to the first step
  if (loop === 'once') {
    root.append(svg('rect', { x: cx - 34, y: endY, width: 68, height: 22, rx: 11, class: 'terminal' }),
      svg('text', { x: cx, y: endY + 15, class: 'terminal-text', 'text-anchor': 'middle' }, 'done'));
  } else {
    const until = mission?.stop_when ? `until ${mission.stop_when}` : (loop === 'forever' ? 'forever' : loop);
    const ly = endY + 8;
    root.append(svg('path', { d: `M${cx},${endY - 2} L${cx},${ly} L14,${ly} L14,${rowY(0) + 12} L${G.nodeX - 2},${rowY(0) + 12}`,
      class: 'edge edge-loop', 'marker-end': 'url(#arrow-loop)' }),
    label(20, ly + 16, [`loop ${until}`, mission?.max_cycles ? `max ${mission.max_cycles}` : '',
      snap.status.plan_id === plan.plan_id && snap.status.cycle ? `now cycle ${snap.status.cycle}` : ''].filter(Boolean).join(' · '),
    'loop-label'));
    if (mission?.stop_when || mission?.max_cycles) {
      root.append(svg('rect', { x: right - 68, y: endY + 26, width: 68, height: 22, rx: 11, class: 'terminal' }),
        svg('text', { x: right - 34, y: endY + 41, class: 'terminal-text', 'text-anchor': 'middle' }, 'done'));
    }
  }
  if (anyAbort) {
    const last = Math.max(...r.map((x, i) => (x.onFailure === 'abort' ? i : -1)));
    const abortY = endY + 26;
    root.append(svg('path', { d: `M${G.abortLane},${rowY(r.findIndex((x) => x.onFailure === 'abort')) + G.nodeH - 10} L${G.abortLane},${abortY - 2}`,
      class: 'edge edge-fail' }),
    svg('rect', { x: G.abortLane - 52, y: abortY, width: 56, height: 22, rx: 11, class: 'terminal terminal-abort' }),
    svg('text', { x: G.abortLane - 24, y: abortY + 15, class: 'terminal-text', 'text-anchor': 'middle' }, 'abort'),
    label(G.abortLane - 4, rowY(last) + G.nodeH + 2, 'fails', 'fail-label', 'end'));
  }
  box.replaceChildren(root, el('div', { class: 'legend' },
    ...Object.entries({ drive: 'drive', arm: 'arm', grip: 'gripper', wait: 'wait', other: 'other' })
      .filter(([kind]) => kind !== 'other' || plan.steps.some((s) => laneOf(s) === 'other'))
      .map(([kind, text]) => el('span', { class: `legend-item kind-${kind}` }, el('i'), text)),
    el('span', { class: 'legend-item' }, el('i', { class: 'swatch-fail' }), 'on failure')));
}

function setPlanView(view) {
  try { localStorage.setItem('tw-plan-view', view); } catch (_) { /* private mode */ }
  document.querySelectorAll('#plan-view button').forEach((b) => b.setAttribute('aria-pressed', String(b.dataset.view === view)));
  show($('plan'), view === 'list');
  show($('plan-graph'), view === 'graph');
  renderGraph();
}
document.querySelectorAll('#plan-view button').forEach((b) => b.addEventListener('click', () => setPlanView(b.dataset.view)));
let initialView = 'graph';
try { initialView = localStorage.getItem('tw-plan-view') || 'graph'; } catch (_) { /* private mode */ }
setPlanView(initialView);
