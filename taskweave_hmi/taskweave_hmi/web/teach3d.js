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
  teachViewer = new window.Viewer3D($('viewer3d-teach'), { stances: true });
  teachViewer.load();
  teachViewer.onPick(picked);
}
window.addEventListener('viewer3d-ready', () => { if (pointsVisible) startTeachViewer(); });

// points changed somewhere (app.js, the 'points' event): both cell views draw their markers again, and an open
// object menu lists what the object has now
async function reloadScenes() {
  runViewer?.reloadScene();
  await teachViewer?.reloadScene();
  if (!$('teach-pick').hidden && pickedObject) showPick(pickedObject, false);
}

// ---------- 3D -> the lists ----------
function picked(info) {
  if (teachFlowPick(info)) return;  // teaching by task takes the picks (teachtask.js)
  hidePick();
  if (info.here) { showSpot(info.here); return; }  // several stances on one spot: which one
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

// The entry opened from 3D says where it saves: the object (for a stance or point) and the file; one note in the tab,
// on the entry picked last. Another object chosen in its picker (the stance's "Measure at", the type's "at") takes
// the note away: it would name the old one.
function noteWhere(item, at, file, picker = null) {
  document.querySelectorAll('#view-points .pick-where').forEach((old) => old.remove());
  const text = `${at ? `At ${at} · ` : ''}saves to ${file}`;
  const note = el('p', { class: 'pick-where', text });
  (item.classList.contains('point-edit') ? item : item.querySelector('.point-edit'))?.prepend(note);
  picker?.addEventListener('change', () => note.remove(), { once: true });
}
// a type's "at" choice (teach.js atPicker) set as the operator would: its hint, teach.at and the pad's target follow
function setTeachAt(type, placed) {
  const picker = document.querySelector(`#point-list select[aria-label="${CSS.escape(`Teach ${type} at`)}"]`);
  if (!picker) { teach.at[type] = placed; return null; }
  if (picker.value !== placed) { picker.value = placed; picker.dispatchEvent(new Event('change')); }
  return picker;
}
function reveal(item) {
  item.scrollIntoView({ block: 'nearest', behavior: 'smooth' });
  item.querySelector('.point-head')?.focus({ preventScroll: true });
}
// opens a list entry (app.js pointItem) if it is closed (its head highlights the marker and turns the view to it);
// an open one: the same for its marker here
function openItem(item, target) {
  const edit = item.querySelector('.point-edit');
  if (edit?.hidden) item.querySelector('.point-head').click();
  else teach3dShow(target);
}

function openLocation(name) {
  selectTeachPane('locations');
  const item = document.querySelector(`#location-list li[data-address="${CSS.escape(`location/${name}`)}"]`);
  if (!item) return;
  openItem(item, { kind: 'location', name });
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
  openItem(item, { kind: 'stance', name, placed });
  noteWhere(item, placed, fileOf.stance(object), override ? null : picker);
  reveal(item);
}

// a grasp point at a placed object: the point of the type, taught (and tested) at that object. The type's "at"
// choice is set first, then the entry opened as by its head (the pad's target and the highlight follow); nothing
// else in the lists is drawn again, so values typed elsewhere stay.
function openPoint(type, name, placed) {
  if (!teach.data) return;  // /api/teach/arm not answered yet
  selectTeachPane('points');
  const picker = placed ? setTeachAt(type, placed) : null;
  const item = document.querySelector(`#point-list li[data-key="${CSS.escape(keyOf({ scope: type, name }))}"]`);
  if (!item) return;
  if (teach.open !== keyOf({ scope: type, name })) item.querySelector('.point-head').click();
  else teach3dShow({ kind: 'point', name, object_type: type, ...(placed ? { placed } : {}) });
  noteWhere(item, placed, fileOf.point(type), picker);
  reveal(item);
}

// ---------- a placed object: what it has, and teach new here ----------
// a button of the card under the view: closes it, then acts
function pickButton(text, onClick, cls = 'btn small', edits = false) {
  // the teach-new buttons are off while a mission runs, as + Point and + Variant (setPointsBusy, data-edit)
  const node = el('button', { type: 'button', class: cls, text, ...(edits ? { 'data-edit': '' } : {}) });
  node.disabled = edits && !!pointsBusy;
  node.addEventListener('click', () => { hidePick(); onClick(); });
  return node;
}
function closeButton() {
  const close = el('button', { type: 'button', class: 'icon', 'aria-label': 'Close', text: '✕' });
  close.addEventListener('click', hidePick);
  return close;
}
let pickedObject = null;  // the object whose menu is open: listed again after a save
function showPick(info, focus = true) {
  const { name, object } = info;
  pickedObject = info;
  const scene = teachViewer?.sceneData || {};
  const stances = (scene.stances || []).filter((s) => s.instance === name);
  const pointsOf = (scene.points || []).filter((p) => p.object_type === object);
  const close = closeButton();
  const group = (title, nodes, none) => el('div', { class: 'pick-group' }, el('div', { class: 'sub', text: title }),
    nodes.length ? el('div', { class: 'row wrap' }, ...nodes) : el('p', { class: 'muted small-text', text: none }));
  $('teach-pick').replaceChildren(
    el('div', { class: 'row between' }, el('b', { text: `${name} · ${object}` }), close),
    group('Where the robot stands', stances.map((s) => pickButton(`${s.variant}${s.override ? ' (this one)' : ''}`,
      () => openStance({ name: s.variant, placed: name, object, override: s.override }))), 'No stance yet.'),
    group('Where a hand grasps', pointsOf.map((p) => pickButton(p.name, () => openPoint(object, p.name, name))), 'No grasp point yet.'),
    el('div', { class: 'row wrap' },
      pickButton('Teach a stance here', () => newStanceAt(object, name), 'btn small primary', true),
      pickButton('Teach a grasp point here', () => newPointAt(object, name), 'btn small primary', true)));
  show($('teach-pick'), true);
  if (!focus) return;
  teachViewer?.setHighlight({ kind: 'placed', name });
  close.focus();
}
function hidePick() { show($('teach-pick'), false); pickedObject = null; }

// several stance markers on one spot (an object's override where a neighbour's type stance is): one button each
function showSpot(here) {
  const close = closeButton();
  $('teach-pick').replaceChildren(
    el('div', { class: 'row between' }, el('b', { text: 'Stances on this spot' }), close),
    el('div', { class: 'row wrap' }, ...here.map((stance) => pickButton(
      `${stance.placed} · ${stance.name}${stance.override ? ' (this one)' : ''}`,
      () => { teachViewer?.setHighlight(stance); openStance(stance); }))));
  show($('teach-pick'), true);
  close.focus();
}

// a new stance variant of the type, measured at this object (the stance card's + Variant, its picker set to it)
function newStanceAt(object, placed) {
  selectTeachPane('stances');
  const card = document.querySelector(`#stance-cards .card[data-object="${CSS.escape(object)}"]`);
  if (!card) return;
  card._select?.('stances');
  card.querySelector('[data-add-variant]')?.click();
  const item = card.querySelector('ul[data-variants]')?.lastElementChild;  // the variant + Variant just added
  if (!item) return;
  const picker = item.querySelector('select[data-measure-at]');
  if (picker) { picker.value = placed; picker.dispatchEvent(new Event('change')); }
  noteWhere(item, placed, fileOf.stance(object), picker);
  reveal(item);
  item.querySelector('input')?.focus();
}

// a new grasp point on the type, taught at this object (teach.js newPosition, its type set; the type's "at" choice set
// as its picker would). A + Pose box still open (the Poses pane) is cancelled; an open + Point box is reused.
function newPointAt(object, placed) {
  if (!teach.data || pointsBusy) return;
  selectTeachPane('points');
  const picker = setTeachAt(object, placed);
  const open = $('arm-new');
  if (open?.querySelector('select[aria-label="On object type"]')?.closest('.field')?.hidden) {  // a + Pose box
    open.querySelector('[data-cancel]')?.click();
  }
  newPosition('point');
  const box = $('arm-new');
  if (!box) return;
  const scope = box.querySelector('select[aria-label="On object type"]');
  if (scope) { scope.value = object; scope.dispatchEvent(new Event('input')); }
  noteWhere(box, placed, fileOf.point(object), picker);
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
