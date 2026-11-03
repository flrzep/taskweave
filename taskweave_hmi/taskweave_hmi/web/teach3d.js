'use strict';
// taskweave HMI, Teach tab: the cell in 3D (viewer3d.js, cell mode) beside the lists (above them up to 1179px), to
// teach by pointing (P7-1). A click on a marker opens its entry: a location, a stance (a type's variant measured at
// that object, or the object's own override) or a grasp point (a point of the object's type, taught at that object);
// the Teach pane switches to it, and the entry says at which object and into which file it saves. A click on a placed
// object lists what it has (stances, grasp points) and offers to teach a new stance or grasp point there. The other way
// round, opening an entry highlights its marker and turns the view to it. After a save (any browser, the pad, Undo) the
// markers are drawn again.
// Uses helpers and state from app.js ($, el, show, state, pointsVisible, selectTeachPane, loadPoints) and teach.js
// (teach, keyOf, renderArm, newPosition).

let teachViewer = null;
function startTeachViewer() {
  if (teachViewer || !window.Viewer3D || $('view-points').hidden) return;
  teachViewer = new window.Viewer3D($('viewer3d-teach'));
  teachViewer.load();
  teachViewer.onPick(picked);
}
window.addEventListener('viewer3d-ready', () => { if (pointsVisible) startTeachViewer(); });

// points changed somewhere (app.js, the 'points' event): both cell views draw their markers again
function reloadScenes() {
  teachViewer?.reloadScene();
  if (typeof runViewer === 'object' && runViewer) runViewer.reloadScene();
}

// ---------- 3D -> the lists ----------
function picked(info) {
  hidePick();
  if (info.kind !== 'placed') teachViewer?.setHighlight(info);
  if (info.kind === 'location') openLocation(info.name);
  else if (info.kind === 'stance') openStance(info);
  else if (info.kind === 'point') openPoint(info.object_type, info.name, info.placed);
  else if (info.kind === 'placed') showPick(info);
}

const fileOf = {
  location: () => state.pointsData?.files.locations || 'cell/targets.yaml',
  stance: (type) => (state.pointsData?.files.stances || '').replace('<type>', type),
  point: (type) => (teach.data?.files.objects || 'objects/<type>.yaml').replace('<type>', type),
};

// The entry opened from 3D says where it saves: the object (for a stance or point) and the file.
function noteWhere(item, at, file) {
  item.querySelector('.pick-where')?.remove();
  const text = `${at ? `At ${at} · ` : ''}saves to ${file}`;
  const note = el('p', { class: 'pick-where', text });
  item.querySelector('.point-edit')?.prepend(note);
}
function reveal(item) {
  item.scrollIntoView({ block: 'nearest', behavior: 'smooth' });
  item.querySelector('.point-head')?.focus({ preventScroll: true });
}
// opens a list entry (app.js pointItem) if it is closed
function openItem(item) {
  const edit = item.querySelector('.point-edit');
  if (edit?.hidden) item.querySelector('.point-head').click();
}

function openLocation(name) {
  selectTeachPane('locations');
  const item = document.querySelector(`#location-list li[data-address="${CSS.escape(`location/${name}`)}"]`);
  if (!item) return;
  openItem(item);
  noteWhere(item, '', fileOf.location());
  reveal(item);
}

// a stance marker: the object's own override if it has one, else the type's variant with the object as where it is
// measured
function openStance({ name, placed, object, override }) {
  selectTeachPane('stances');
  const address = override ? `stance/instances/${placed}/${name}` : `stance/objects/${object}/${name}`;
  const item = document.querySelector(`#stance-cards li[data-address="${CSS.escape(address)}"]`);
  if (!item) return;
  item.closest('.card')?._select?.('stances');
  const picker = item.querySelector('select[data-measure-at]');
  if (picker && picker.value !== placed) { picker.value = placed; picker.dispatchEvent(new Event('change')); }
  openItem(item);
  noteWhere(item, placed, fileOf.stance(object));
  reveal(item);
}

// a grasp point at a placed object: the point of the type, taught (and tested) at that object
function openPoint(type, name, placed) {
  selectTeachPane('points');
  if (placed) teach.at[type] = placed;
  teach.open = keyOf({ scope: type, name });
  renderArm();  // opens it (teach.open) with the "at" choice set; the pad's target follows
  const item = document.querySelector(`#point-list li[data-key="${CSS.escape(teach.open)}"]`);
  if (!item) return;
  noteWhere(item, placed, fileOf.point(type));
  reveal(item);
  setTeachTarget({ scope: type, name, at: placed || null, instance_override: !!item._override?.() });
}

// ---------- a placed object: what it has, and teach new here ----------
function showPick({ name, object }) {
  const scene = teachViewer?.sceneData || {};
  const stances = (scene.stances || []).filter((s) => s.instance === name);
  const pointsOf = (scene.points || []).filter((p) => p.object_type === object);
  const button = (text, onClick, cls = 'btn small') => {
    const node = el('button', { type: 'button', class: cls, text });
    node.addEventListener('click', () => { hidePick(); onClick(); });
    return node;
  };
  const close = el('button', { type: 'button', class: 'icon', 'aria-label': 'Close', text: '✕' });
  close.addEventListener('click', hidePick);
  const group = (title, nodes, none) => el('div', { class: 'pick-group' }, el('div', { class: 'sub', text: title }),
    nodes.length ? el('div', { class: 'row wrap' }, ...nodes) : el('p', { class: 'muted small-text', text: none }));
  $('teach-pick').replaceChildren(
    el('div', { class: 'row between' }, el('b', { text: `${name} · ${object}` }), close),
    group('Where the robot stands', stances.map((s) => button(`${s.variant}${s.override ? ' (this one)' : ''}`,
      () => openStance({ name: s.variant, placed: name, object, override: s.override }))), 'No stance yet.'),
    group('Where a hand grasps', pointsOf.map((p) => button(p.name, () => openPoint(object, p.name, name))), 'No grasp point yet.'),
    el('div', { class: 'row wrap' },
      button('Teach a stance here', () => newStanceAt(object, name), 'btn small primary'),
      button('Teach a grasp point here', () => newPointAt(object, name), 'btn small primary')));
  show($('teach-pick'), true);
  teachViewer?.setHighlight({ kind: 'placed', name });
  close.focus();
}
function hidePick() { show($('teach-pick'), false); }

// a new stance variant of the type, measured at this object (the stance card's + Variant, its picker set to it)
function newStanceAt(object, placed) {
  selectTeachPane('stances');
  const card = document.querySelector(`#stance-cards .card[data-object="${CSS.escape(object)}"]`);
  if (!card) return;
  card._select?.('stances');
  card.querySelector('[data-add-variant]')?.click();
  const item = [...card.querySelectorAll('li.point')].at(-1);
  if (!item) return;
  const picker = item.querySelector('select[data-measure-at]');
  if (picker) { picker.value = placed; picker.dispatchEvent(new Event('change')); }
  noteWhere(item, placed, fileOf.stance(object));
  reveal(item);
  item.querySelector('input')?.focus();
}

// a new grasp point on the type, taught at this object (teach.js newPosition, its type set)
function newPointAt(object, placed) {
  selectTeachPane('points');
  teach.at[object] = placed;
  newPosition('point');
  const box = $('arm-new');
  if (!box) return;
  const scope = box.querySelector('select[aria-label="On object type"]');
  if (scope) { scope.value = object; scope.dispatchEvent(new Event('input')); }
  box.prepend(el('p', { class: 'pick-where', text: `At ${placed} · saves to ${fileOf.point(object)}` }));
  box.scrollIntoView({ block: 'nearest', behavior: 'smooth' });
}

// ---------- the lists -> 3D ----------
// An entry opened in a list (app.js pointItem, teach.js positionItem): its marker highlighted and in view; none for a
// robot pose (no marker) or a closed entry.
function teach3dShow(target) {
  if (!teachViewer) return;
  teachViewer.setHighlight(target);
  if (target) teachViewer.focus(target);
}
