'use strict';
// taskweave HMI: the icon of a step (Figma "taskweave skill icons", 24 px grid, 2 px stroke, colour currentColor so CSS
// tints them by kind and state). The SVG files are web/icons/<id>.svg; ICON_SVG below holds the same markup inline (the
// server lists its static files one by one, so no fetch per icon). After changing a file run
// `python3 taskweave_hmi/test/test_icons.py --regenerate`; test_icons.py fails while the two differ.
// Uses nothing from the other files; graph.js and the editors call skillIcon(step) and iconSvg(id).

// a move step's kind -> its icon; the ids before cell schema 5 too (base, object, elbow)
const KIND_ICONS = { navigate: 'navigate', straight: 'straight', hand: 'hand', carry: 'carry', posture: 'posture',
  joints: 'joints', base: 'navigate', object: 'carry', elbow: 'posture' };
// a skill -> its icon: the motion skills of the old layout (move's parts now) and the others; grip by its action
const SKILL_ICONS = { navigate_to: 'navigate', drive_straight: 'straight', move_gripper: 'hand', move_object: 'carry',
  set_posture: 'posture', move_joints: 'joints', wait: 'wait' };
const GRIP_ICONS = { close: 'grip-close', open: 'grip-open', opening: 'grip-open' };
const HOLDING = 'holding';  // move's hand: holding (the hands that hold), taskweave_core config/targets.py HOLDING

// Does this move step carry what the hands hold? hand: holding, the form of cell schema 4 (object: held with by / at
// and no to / hand), or a result that says so (data.carrying, the held instance).
function carries(step, result) {
  const args = (step && step.args) || {};
  if (args.hand === HOLDING) return true;
  if (args.object !== undefined && ('by' in args || 'at' in args) && !('to' in args) && !('hand' in args)) return true;
  const data = (result && result.data) || (step && step.result && step.result.data) || {};
  return Boolean(data.carrying);
}

// the kind of a move step from its arguments, when step.kind is not filled (a step not validated yet, a target name
// hidden by a placeholder: then undefined)
function moveKindOfArgs(args) {
  if (!args) return undefined;
  if ('base' in args) return 'straight';
  if ('elbow' in args) return 'posture';
  if ('joints' in args) return 'joints';
  if ('hand' in args || 'object' in args) return 'hand';
  return undefined;
}

// the icon id of a step: a move step by its kind (carry when it carries), else by its skill, else other
function skillIcon(step, result) {
  if (!step) return 'other';
  const kind = step.kind || (step.skill === 'move' ? moveKindOfArgs(step.args) : undefined);
  if (kind && KIND_ICONS[kind]) {
    const icon = KIND_ICONS[kind];
    return (icon === 'hand' && carries(step, result)) ? 'carry' : icon;
  }
  if (step.skill === 'grip') return GRIP_ICONS[(step.args || {}).action] || 'grip-close';
  return SKILL_ICONS[step.skill] || 'other';
}

// ICON_SVG begin (generated from web/icons/*.svg by test_icons.py --regenerate)
const ICON_SVG = {
  'carry': '<svg xmlns="http://www.w3.org/2000/svg" viewBox="0 0 24 24" fill="none" stroke="currentColor" stroke-width="2" stroke-linecap="round" stroke-linejoin="round"> <path d="M12.5 10H4.5C3.67 10 3 10.67 3 11.5V19.5C3 20.33 3.67 21 4.5 21H12.5C13.33 21 14 20.33 14 19.5V11.5C14 10.67 13.33 10 12.5 10Z"/> <path d="M8.5 10V13.5"/> <path d="M15.5 8.5L21 3M21 7.5V3H16.5"/> </svg>',
  'grip-close': '<svg xmlns="http://www.w3.org/2000/svg" viewBox="0 0 24 24" fill="none" stroke="currentColor" stroke-width="2" stroke-linecap="round" stroke-linejoin="round"> <path d="M12 2V6M7 6H17M8 6V18M16 6V18"/> <path opacity="0.35" d="M16 12H8V18H16V12Z" fill="currentColor" stroke="none"/> <path d="M2 15H5M3.6 13.4L5.2 15L3.6 16.6M22 15H19M20.4 13.4L18.8 15L20.4 16.6"/> </svg>',
  'grip-open': '<svg xmlns="http://www.w3.org/2000/svg" viewBox="0 0 24 24" fill="none" stroke="currentColor" stroke-width="2" stroke-linecap="round" stroke-linejoin="round"> <path d="M12 2V6M3 16L5 6H19L21 16"/> <path d="M14 13H10C9.45 13 9 13.45 9 14V18C9 18.55 9.45 19 10 19H14C14.55 19 15 18.55 15 18V14C15 13.45 14.55 13 14 13Z"/> </svg>',
  'hand': '<svg xmlns="http://www.w3.org/2000/svg" viewBox="0 0 24 24" fill="none" stroke="currentColor" stroke-width="2" stroke-linecap="round" stroke-linejoin="round"> <path d="M10 2.5V7M5 12V7H15V12"/> <path d="M10 21.5C11.93 21.5 13.5 19.93 13.5 18C13.5 16.07 11.93 14.5 10 14.5C8.07 14.5 6.5 16.07 6.5 18C6.5 19.93 8.07 21.5 10 21.5Z"/> <path d="M10 19C10.55 19 11 18.55 11 18C11 17.45 10.55 17 10 17C9.45 17 9 17.45 9 18C9 18.55 9.45 19 10 19Z" fill="currentColor" stroke="none"/> <path d="M19 15.5V20.5M16.5 18H21.5"/> </svg>',
  'joints': '<svg xmlns="http://www.w3.org/2000/svg" viewBox="0 0 24 24" fill="none" stroke="currentColor" stroke-width="2" stroke-linecap="round" stroke-linejoin="round"> <path d="M12 15C13.66 15 15 13.66 15 12C15 10.34 13.66 9 12 9C10.34 9 9 10.34 9 12C9 13.66 10.34 15 12 15Z"/> <path d="M12 4C13.72 4.01 15.39 4.56 16.76 5.59C18.14 6.62 19.15 8.06 19.64 9.71C20.13 11.35 20.08 13.11 19.5 14.72C18.91 16.34 17.82 17.72 16.39 18.67C14.96 19.62 13.26 20.08 11.55 19.99C9.83 19.9 8.19 19.26 6.87 18.16C5.55 17.06 4.61 15.57 4.21 13.91C3.8 12.24 3.93 10.48 4.6 8.9"/> <path d="M3.9 5.6L4.6 8.9L7.8 8.2"/> </svg>',
  'navigate': '<svg xmlns="http://www.w3.org/2000/svg" viewBox="0 0 24 24" fill="none" stroke="currentColor" stroke-width="2" stroke-linecap="round" stroke-linejoin="round"> <path d="M8.5 14H4.5C3.67 14 3 14.67 3 15.5V19.5C3 20.33 3.67 21 4.5 21H8.5C9.33 21 10 20.33 10 19.5V15.5C10 14.67 9.33 14 8.5 14Z"/> <path d="M6.5 14C6.5 9.5 10 7 15 7" stroke-dasharray="2 3"/> <path d="M19 2.5C19.93 2.5 20.82 2.87 21.47 3.53C22.13 4.18 22.5 5.07 22.5 6C22.5 8.6 19 11.5 19 11.5C19 11.5 15.5 8.6 15.5 6C15.5 5.07 15.87 4.18 16.53 3.53C17.18 2.87 18.07 2.5 19 2.5Z"/> <path d="M19 7C19.55 7 20 6.55 20 6C20 5.45 19.55 5 19 5C18.45 5 18 5.45 18 6C18 6.55 18.45 7 19 7Z" fill="currentColor" stroke="none"/> </svg>',
  'other': '<svg xmlns="http://www.w3.org/2000/svg" viewBox="0 0 24 24" fill="none" stroke="currentColor" stroke-width="2" stroke-linecap="round" stroke-linejoin="round"> <path d="M17 3H7C4.79 3 3 4.79 3 7V17C3 19.21 4.79 21 7 21H17C19.21 21 21 19.21 21 17V7C21 4.79 19.21 3 17 3Z"/> <path d="M8 13.25C8.69 13.25 9.25 12.69 9.25 12C9.25 11.31 8.69 10.75 8 10.75C7.31 10.75 6.75 11.31 6.75 12C6.75 12.69 7.31 13.25 8 13.25Z" fill="currentColor" stroke="none"/> <path d="M12 13.25C12.69 13.25 13.25 12.69 13.25 12C13.25 11.31 12.69 10.75 12 10.75C11.31 10.75 10.75 11.31 10.75 12C10.75 12.69 11.31 13.25 12 13.25Z" fill="currentColor" stroke="none"/> <path d="M16 13.25C16.69 13.25 17.25 12.69 17.25 12C17.25 11.31 16.69 10.75 16 10.75C15.31 10.75 14.75 11.31 14.75 12C14.75 12.69 15.31 13.25 16 13.25Z" fill="currentColor" stroke="none"/> </svg>',
  'posture': '<svg xmlns="http://www.w3.org/2000/svg" viewBox="0 0 24 24" fill="none" stroke="currentColor" stroke-width="2" stroke-linecap="round" stroke-linejoin="round"> <path d="M4 20L10 11L19.5 14"/> <path d="M4 21.6C4.88 21.6 5.6 20.88 5.6 20C5.6 19.12 4.88 18.4 4 18.4C3.12 18.4 2.4 19.12 2.4 20C2.4 20.88 3.12 21.6 4 21.6Z" fill="currentColor" stroke="none"/> <path d="M10 12.6C10.88 12.6 11.6 11.88 11.6 11C11.6 10.12 10.88 9.4 10 9.4C9.12 9.4 8.4 10.12 8.4 11C8.4 11.88 9.12 12.6 10 12.6Z" fill="currentColor" stroke="none"/> <path d="M6 6.5C6.47 5.85 7.06 5.3 7.74 4.88C8.42 4.46 9.18 4.18 9.98 4.06C10.77 3.93 11.58 3.97 12.35 4.17C13.13 4.37 13.86 4.72 14.5 5.2"/> <path d="M12.2 3.6L14.5 5.2L12.6 7.3"/> </svg>',
  'straight': '<svg xmlns="http://www.w3.org/2000/svg" viewBox="0 0 24 24" fill="none" stroke="currentColor" stroke-width="2" stroke-linecap="round" stroke-linejoin="round"> <path d="M7.5 8.5H3.5C2.67 8.5 2 9.17 2 10V14C2 14.83 2.67 15.5 3.5 15.5H7.5C8.33 15.5 9 14.83 9 14V10C9 9.17 8.33 8.5 7.5 8.5Z"/> <path d="M12 12H21M18 15L21 12L18 9"/> </svg>',
  'wait': '<svg xmlns="http://www.w3.org/2000/svg" viewBox="0 0 24 24" fill="none" stroke="currentColor" stroke-width="2" stroke-linecap="round" stroke-linejoin="round"> <path d="M6 3H18M6 21H18"/> <path d="M7.5 3V5.5C7.5 6.3 7.68 7.09 8.05 7.81C8.42 8.52 8.95 9.14 9.6 9.6L12 12M12 12L14.4 9.6C15.05 9.14 15.58 8.52 15.95 7.81C16.32 7.09 16.5 6.3 16.5 5.5V3M12 12L9.6 14.4C8.95 14.87 8.42 15.48 8.05 16.19C7.68 16.91 7.5 17.7 7.5 18.5V21M12 12L14.4 14.4C15.05 14.87 15.58 15.48 15.95 16.19C16.32 16.91 16.5 17.7 16.5 18.5V21"/> <path d="M10 19.5H14"/> </svg>',
};
// ICON_SVG end

// the SVG markup of an icon (unknown ids: other's)
function iconSvg(id) {
  return ICON_SVG[id] || ICON_SVG.other;
}

// an <svg> element of an icon, for a DOM (class: 'icon icon-<id>' and any extra classes)
function iconElement(id, extra = '') {
  const tpl = document.createElement('template');
  tpl.innerHTML = iconSvg(id).trim();
  const node = tpl.content.firstElementChild;
  node.setAttribute('class', `icon icon-${ICON_SVG[id] ? id : 'other'}${extra ? ` ${extra}` : ''}`);
  node.setAttribute('aria-hidden', 'true');
  return node;
}
