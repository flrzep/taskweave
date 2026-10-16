'use strict';
// taskweave HMI, Record tab (recording.py): start a recording (ROS bag, sim videos, optionally a mission from the sim's
// start), follow it live, and play or delete past ones. The tab shows only when the HMI has a recordings_dir.
// Uses app.js ($, el, api, show, message).

const rec = { open: new Set(), timer: null, missionsLoaded: false };
const VIEW_NAMES = { fixed: 'Fixed view', follow: 'Following the robot', head_camera_color: 'Head camera',
  head_camera_depth: 'Head camera depth (red near, blue far)' };
function videoName(file, id) {  // <id>_fixed.mp4 -> Fixed view; <id>_left_camera_color.mp4 -> left camera color
  const view = file.replace(/\.mp4$/, '').replace(`${id}_`, '');
  return VIEW_NAMES[view] || view.replace(/_/g, ' ');
}
const recTab = document.querySelector('.tabs button[data-view="record"]');
const recVisible = () => !$('view-record').hidden;

function gb(bytes) {
  if (bytes == null) return '–';
  return bytes >= 1e9 ? `${(bytes / 1e9).toFixed(1)} GB` : `${Math.max(bytes / 1e6, 0).toFixed(0)} MB`;
}
function clock(seconds) {
  if (seconds == null) return '–';
  const s = Math.max(0, Math.round(seconds));
  const h = Math.floor(s / 3600), m = Math.floor((s % 3600) / 60), r = s % 60;
  return h ? `${h}:${String(m).padStart(2, '0')}:${String(r).padStart(2, '0')}` : `${m}:${String(r).padStart(2, '0')}`;
}
function simSpan(v) { return v && v.last_time != null && v.first_time != null ? v.last_time - v.first_time : null; }

async function initRecord() {
  const r = await api('GET', '/api/recordings');
  show(recTab, r.ok);
  if (r.ok && r.data.status?.active) recTab.textContent = 'Record ●';
}

async function loadRecordings() {
  if (!rec.missionsLoaded) {
    const m = await api('GET', '/api/missions');
    if (m.ok) {
      const names = m.data.filter((x) => x.valid).map((x) => x.name);
      $('rec-mission').replaceChildren(el('option', { value: '', text: '– none: record what happens –' }),
        ...names.map((n) => el('option', { value: n, text: n })));
      rec.missionsLoaded = true;
    }
  }
  await refreshRecordings();
}

async function refreshRecordings() {
  clearTimeout(rec.timer);
  const r = await api('GET', '/api/recordings');
  if (r.ok) {
    renderRecStatus(r.data.status);
    renderRecList(r.data.recordings);
    $('rec-free').textContent = r.data.free_bytes != null ? `${gb(r.data.free_bytes)} free` : '';
    const keep = r.data.keep;
    $('rec-keep-note').textContent = keep === 0 ? 'Every recording is kept: delete old ones here.'
      : `Starting a recording deletes the older ones${keep > 1 ? ` beyond the newest ${keep - 1}` : ''}, except those marked Keep. `
        + `It needs ${r.data.min_free_gb} GB free.`;
  }
  const active = r.ok && r.data.status?.active;
  recTab.textContent = active ? 'Record ●' : 'Record';
  if (recVisible()) rec.timer = setTimeout(refreshRecordings, active ? 2000 : 10000);
}

function renderRecStatus(s) {
  const active = !!s.active;
  const chip = $('rec-state');
  const failed = s.outcome && /error|failed|aborted/.test(s.outcome);
  const state = active ? 'running' : s.id ? (failed ? 'failed' : 'succeeded') : 'idle';
  chip.className = `chip ${state}`;
  chip.textContent = active ? 'recording' : s.id ? 'finished' : 'idle';
  $('rec-id').textContent = s.id || '';
  $('rec-title').textContent = !s.id ? 'Not recording'
    : `${s.options.mission || 'Free recording'}${active ? '' : ` — ${s.outcome || 'done'}`}`;
  $('rec-stop').disabled = !active;
  $('rec-start').disabled = active;
  show($('rec-now'), !!s.id);
  if (!s.id) return;
  $('rec-phase').textContent = active ? s.phase : `ended after ${clock(s.wall_s)}`;
  const v = s.video || {};
  const span = simSpan(v);
  const stats = [
    ['wall clock', clock(s.wall_s)],
    ['video frames', v.frames != null ? String(v.frames) : '–'],
    ['sim time recorded', span != null ? `${span.toFixed(1)} s` : '–'],
    ['sim speed', span && s.wall_s ? `${(span / s.wall_s).toFixed(2)}×` : '–'],
    ['size', gb(s.bytes)],
    ['video', v.state || (s.options.video ? 'waiting' : 'off')],
  ];
  if (s.options.cycles) stats.push(['stops after', `${s.options.cycles} cycle${s.options.cycles > 1 ? 's' : ''}`]);
  $('rec-stats').replaceChildren(...stats.map(([label, value]) => el('div', {}, el('b', { text: value }), el('span', { text: label }))));
  const problems = s.problems || [];
  $('rec-problems').textContent = problems.join('\n');
  show($('rec-problems'), problems.length > 0);
}

function renderRecList(list) {
  if (!list.length) { $('rec-list').replaceChildren(el('li', { class: 'muted', text: 'None yet' })); return; }
  // keep open players (and their position) over refreshes: rebuild only what changed
  const old = new Map([...$('rec-list').children].filter((li) => li.dataset.id).map((li) => [li.dataset.id, li]));
  $('rec-list').replaceChildren(...list.map((r) => {
    const key = JSON.stringify([r.files, r.outcome, r.bytes, r.active, r.keep]);
    const keep = old.get(r.id);
    if (keep && keep.dataset.key === key) return keep;
    if (keep && rec.open.has(r.id) && !r.active && keep.querySelector('video')) return keep;  // don't stop playback
    return recItem(r, key);
  }));
}

function recItem(r, key) {
  const url = (file) => `/recordings/${encodeURIComponent(r.id)}/${file}`;
  const span = simSpan(r);
  const when = r.started ? r.started.replace('T', ' ') : r.id.slice(0, 15).replace(/^(\d{4})(\d\d)(\d\d)_(\d\d)(\d\d)(\d\d)$/, '$1-$2-$3 $4:$5:$6');
  const summary = [when, span != null ? `${clock(span)} sim` : '', gb(r.bytes)].filter(Boolean).join(' · ');
  const head = el('button', { class: 'point-head', 'aria-expanded': String(rec.open.has(r.id)) },
    el('b', { text: `${r.keep ? '★ ' : ''}${r.id}${r.active ? ' (recording)' : ''}` }), el('span', { class: 'mono', text: summary }),
    r.outcome ? el('span', { class: /error|failed/.test(r.outcome) ? 'bad' : 'used', text: r.outcome }) : '');
  const body = el('div', { class: 'point-edit' });
  const item = el('li', { class: 'point', 'data-id': r.id, 'data-key': key }, head, body);
  const fill = () => {
    const videos = r.files.filter((f) => f.endsWith('.mp4'));
    const parts = [];
    if (videos.length && !r.active) {
      parts.push(el('div', { class: 'rec-videos' }, videos.map((f) => el('figure', {},
        el('video', { src: url(f), controls: '', preload: 'metadata', playsinline: '' }),
        el('figcaption', { text: videoName(f, r.id) })))));
    } else if (!r.active) parts.push(el('p', { class: 'muted small-text', text: 'No videos in this recording.' }));
    const links = r.files.filter((f) => f !== 'status.json')
      .map((f) => el('a', { href: url(f), download: f.startsWith(r.id) ? f : `${r.id}_${f}`, text: f }));
    if (r.bag) links.push(el('span', { class: 'muted mono', text: `bag/ on the box: ${r.id}/bag` }));
    parts.push(el('div', { class: 'rec-files' }, links));
    if (!r.active) {
      const keepBox = el('input', { type: 'checkbox' });
      keepBox.checked = !!r.keep;
      keepBox.addEventListener('change', async () => {
        const res = await api('PUT', `/api/recordings/${encodeURIComponent(r.id)}/keep`, { keep: keepBox.checked });
        if (!res.ok) { keepBox.checked = !keepBox.checked; message(msg, res.data.error || (res.data.problems || []).join('\n'), true); }
        refreshRecordings();
      });
      parts.push(el('label', { class: 'check' }, keepBox, ' Keep (a new recording does not replace it)'));
      const del = el('button', { class: 'btn danger small', text: 'Delete recording' });
      const msg = el('div', { class: 'msg', hidden: '' });
      let armed = false;
      del.addEventListener('click', async () => {
        if (!armed) { armed = true; del.classList.add('confirm'); del.textContent = `Delete ${gb(r.bytes)} for good?`; return; }
        const res = await api('DELETE', `/api/recordings/${encodeURIComponent(r.id)}`);
        if (res.ok) { rec.open.delete(r.id); refreshRecordings(); } else message(msg, res.data.error || (res.data.problems || []).join('\n'), true);
      });
      parts.push(el('div', { class: 'row gap' }, del), msg);
    }
    body.replaceChildren(...parts);
  };
  const toggle = (open) => {
    head.setAttribute('aria-expanded', String(open));
    show(body, open);
    if (open) { rec.open.add(r.id); fill(); } else { rec.open.delete(r.id); body.replaceChildren(); }
  };
  head.addEventListener('click', () => toggle(head.getAttribute('aria-expanded') !== 'true'));
  toggle(rec.open.has(r.id));
  return item;
}

$('rec-start').addEventListener('click', async () => {
  const body = {
    mission: $('rec-mission').value, name: $('rec-name').value.trim(), cycles: $('rec-cycles').value ? Number($('rec-cycles').value) : null,
    quality: $('rec-quality').value, max_min: Number($('rec-max').value) || 120, reset: $('rec-reset').checked,
    video: $('rec-video').checked, bag: $('rec-bag').checked, camera: $('rec-camera').checked,
  };
  $('rec-start').disabled = true;
  const r = await api('POST', '/api/recordings/start', body);
  const replaced = r.ok && r.data.deleted?.length ? ` (replaced ${r.data.deleted.join(', ')})` : '';
  message($('rec-msg'), r.ok ? `started ${r.data.id}${replaced}` : (r.data.error || (r.data.problems || []).join('\n')), !r.ok);
  refreshRecordings();
});

$('rec-stop').addEventListener('click', async () => {
  $('rec-stop').disabled = true;
  const r = await api('POST', '/api/recordings/stop');
  message($('rec-msg'), r.ok ? 'stopping: the videos are encoded, then the bag is closed' : (r.data.error || (r.data.problems || []).join('\n')), !r.ok);
  refreshRecordings();
});

$('rec-mission').addEventListener('change', () => { $('rec-cycles').disabled = !$('rec-mission').value; });
$('rec-bag').addEventListener('change', () => { $('rec-camera').disabled = !$('rec-bag').checked; });

$('rec-cycles').disabled = true;  // no mission picked yet
initRecord();
