// taskweave HMI: 3D view of the robot live in its cell (three.js r160 + urdf-loader 0.12.7, vendored under vendor/).
// An ES module; it also sets window.Viewer3D for the classic scripts. Those run before deferred modules, so they
// wait for the 'viewer3d-ready' event on window (or check window.Viewer3D first).
//
// The page needs this import map, before any module script:
//   <script type="importmap">
//   {"imports": {"three": "./vendor/three/three.module.min.js",
//                "three/addons/": "./vendor/three/addons/",
//                "three/examples/jsm/": "./vendor/three/addons/"}}
//   </script>
// 'three' and 'three/addons/' are for this file and the three addons; 'three/examples/jsm/' is for urdf-loader, which
// imports 'three/examples/jsm/loaders/STLLoader.js' and '.../ColladaLoader.js' (ColladaLoader pulls in
// TGALoader.js relatively). Both STLLoader paths map to one URL, so the module is loaded once.
//
// Server: GET /api/robot/model (404: no URDF, the cell is shown without the robot), /robot/meshes/*.stl,
// /api/robot/joints, /api/robot/base (503: no TF; the robot stays where it was, at the origin until a pose came, and
// the toolbar shows a .viewer3d-note 'no robot pose'), /api/scene (placed objects' points are composed at top_z).
// The world is ROS z-up: three.js keeps it as is and the camera's up is +z (set before OrbitControls reads it).

import * as THREE from 'three';
import { OrbitControls } from 'three/addons/controls/OrbitControls.js';
import { STLLoader } from 'three/addons/loaders/STLLoader.js';
import URDFLoader from './vendor/urdf-loader/URDFLoader.js';

const ONE = new THREE.Vector3(1, 1, 1);
const NEUTRAL = 0x5a6878;      // markers and arrows when not highlighted
const BOX_COLOUR = 0x8a96a3;   // collision objects
const SCENE_POLL_MS = 2000;    // the scene changes rarely: re-read it now and then, rebuild only when 'updated' moves

// ---------- ROS pose helpers ----------
const quatWXYZ = ([w, x, y, z]) => new THREE.Quaternion(x, y, z, w);
// ROS rpy is fixed-axis X, Y, Z = R = Rz(yaw) Ry(pitch) Rx(roll): three's Euler order 'ZYX' (as urdf-loader does)
const quatRPY = ([r, p, y] = [0, 0, 0]) => new THREE.Quaternion().setFromEuler(new THREE.Euler(r, p, y, 'ZYX'));
const quatYaw = (yaw) => new THREE.Quaternion().setFromAxisAngle(new THREE.Vector3(0, 0, 1), yaw || 0);
const pose = (xyz, quat) => new THREE.Matrix4().compose(new THREE.Vector3(...xyz), quat, ONE);

function el(tag, cls, text) {
  const node = document.createElement(tag);
  if (cls) node.className = cls;
  if (text) node.textContent = text;
  return node;
}

// x red, y green, z blue, from the origin
function triadGeometry(size) {
  const geometry = new THREE.BufferGeometry();
  geometry.setAttribute('position', new THREE.Float32BufferAttribute(
    [0, 0, 0, size, 0, 0, 0, 0, 0, 0, size, 0, 0, 0, 0, 0, 0, size], 3));
  geometry.setAttribute('color', new THREE.Float32BufferAttribute(
    [0.9, 0.1, 0.1, 0.9, 0.1, 0.1, 0.1, 0.7, 0.1, 0.1, 0.7, 0.1, 0.1, 0.3, 0.95, 0.1, 0.3, 0.95], 3));
  return geometry;
}

// A two-finger outline in the tool frame: approach along +z, fingertips at the TCP, fingers apart along y.
function gripperGeometry() {
  const w = 0.025;
  const d = 0.04;
  const v = [-w, -d, w, -d, -w, -d, -w, 0, w, -d, w, 0, 0, -d, 0, -2 * d];  // palm, fingers, stem (y, z pairs)
  const points = [];
  for (let i = 0; i < v.length; i += 2) points.push(0, v[i], v[i + 1]);
  const geometry = new THREE.BufferGeometry();
  geometry.setAttribute('position', new THREE.Float32BufferAttribute(points, 3));
  return geometry;
}

// Flat arrow on the floor pointing along +x, centred on the location.
function arrowGeometry() {
  const shape = new THREE.Shape();
  shape.moveTo(-0.15, -0.03); shape.lineTo(0.08, -0.03); shape.lineTo(0.08, -0.08); shape.lineTo(0.2, 0);
  shape.lineTo(0.08, 0.08); shape.lineTo(0.08, 0.03); shape.lineTo(-0.15, 0.03); shape.closePath();
  return new THREE.ShapeGeometry(shape);
}

// The 12 edges of a unit box centred on the origin, as segment end points.
const BOX_EDGES = new THREE.EdgesGeometry(new THREE.BoxGeometry(1, 1, 1)).getAttribute('position').array;

export class Viewer3D {
  constructor(container, { fetchJSON = (path) => fetch(path).then((r) => (r.ok ? r.json() : Promise.reject(r))), pollMs = 200 } = {}) {
    this.container = container;
    this.fetchJSON = fetchJSON;
    this.pollMs = pollMs;
    this.model = null;        // /api/robot/model
    this.sceneData = null;    // /api/scene
    this.robot = null;        // URDFRobot, live
    this.ghost = null;        // see-through copy for previews (made once the meshes are in)
    this.status = { model: 'loading', scene: 'loading' };
    this.highlight = null;
    this.followRobot = false;
    this.labelsOn = true;
    this._ghostValues = null;
    this._liveJoints = {};
    this._base = null;        // last /api/robot/base answer
    this._basePos = null;     // where the base child frame is in the world (follow robot)
    this._markers = [];       // {kind, name, root, body, baseMaterial, label, info}
    this._pickables = [];
    this._labels = [];        // {node, object}
    this._pickCallbacks = [];
    this._geometries = new Map();  // STL url -> Promise<BufferGeometry>, a mesh used twice is fetched once
    this._urdfMaterials = new Set();
    this._pointers = new Set();
    this._frame = 0;
    this._pollTimer = 0;
    this._sceneAt = 0;
    this._disposed = false;

    container.classList.add('viewer3d');
    if (getComputedStyle(container).position === 'static') container.style.position = 'relative';  // overlays anchor here

    // phones: no MSAA on high-density screens (the pixels are small anyway), at most 2x pixels, no shadows
    const dpr = Math.min(window.devicePixelRatio || 1, 2);
    this.renderer = new THREE.WebGLRenderer({ antialias: dpr < 2, alpha: true });  // transparent: the page's background shows
    this.renderer.setPixelRatio(dpr);
    this.renderer.setClearColor(0x000000, 0);
    const canvas = this.renderer.domElement;
    Object.assign(canvas.style, { display: 'block', width: '100%', height: '100%' });

    this.scene = new THREE.Scene();
    this.camera = new THREE.PerspectiveCamera(45, 1, 0.02, 200);
    this.camera.up.set(0, 0, 1);
    this.camera.position.set(3, -3.5, 2.5);
    this.scene.add(this.camera);
    this.scene.add(new THREE.HemisphereLight(0xffffff, 0x8a8a8a, 2.2));
    const light = new THREE.DirectionalLight(0xffffff, 1.6);  // on the camera: whatever faces the viewer is lit
    light.position.set(1, 2, 3);
    this.camera.add(light);
    this.camera.add(light.target);

    const accent = this._accent();
    this._mats = {
      metal: new THREE.MeshPhongMaterial({ color: 0x9ea4ab, specular: 0x4a4a4a, shininess: 40 }),
      ghost: new THREE.MeshPhongMaterial({ color: accent, transparent: true, opacity: 0.35, depthWrite: false }),
      box: new THREE.MeshLambertMaterial({ color: BOX_COLOUR, transparent: true, opacity: 0.18, depthWrite: false }),
      edge: new THREE.LineBasicMaterial({ color: BOX_COLOUR, transparent: true, opacity: 0.7 }),
      triad: new THREE.LineBasicMaterial({ vertexColors: true }),
      location: new THREE.MeshBasicMaterial({ color: NEUTRAL, transparent: true, opacity: 0.75, side: THREE.DoubleSide, depthWrite: false }),
      marker: new THREE.MeshBasicMaterial({ color: NEUTRAL }),
      gripper: new THREE.LineBasicMaterial({ color: NEUTRAL }),
      accentMesh: new THREE.MeshBasicMaterial({ color: accent, side: THREE.DoubleSide }),
      accentLine: new THREE.LineBasicMaterial({ color: accent }),
      pick: new THREE.MeshBasicMaterial({ visible: false }),  // not drawn, still hit by the raycaster
    };
    this._geoms = {
      box: new THREE.BoxGeometry(1, 1, 1),
      // SolidPrimitive axes are along z, three's along y: turned once here, the instances only scale
      sphere: new THREE.SphereGeometry(1, 16, 12),
      cylinder: new THREE.CylinderGeometry(1, 1, 1, 20).rotateX(Math.PI / 2),
      cone: new THREE.ConeGeometry(1, 1, 20).rotateX(Math.PI / 2),
      arrow: arrowGeometry(),
      triad: triadGeometry(0.1),
      tool: triadGeometry(0.05),
      gripper: gripperGeometry(),
      dot: new THREE.SphereGeometry(0.012, 8, 6),
      pick: new THREE.SphereGeometry(0.05, 8, 6),
    };

    this.base = new THREE.Group();  // world <- URDF root, from /api/robot/base
    this.base.matrixAutoUpdate = false;
    this.cell = new THREE.Group();      // collision objects
    this.markers = new THREE.Group();   // locations, placed objects, points
    this.scene.add(this.base, this.cell, this.markers);
    this.grid = null;
    this._setGrid(4);

    this.controls = new OrbitControls(this.camera, canvas);
    this.controls.target.set(0, 0, 0.5);
    this.controls.update();
    this.controls.addEventListener('change', () => this._requestRender());

    this._labelLayer = el('div', 'viewer3d-labels');
    Object.assign(this._labelLayer.style, { position: 'absolute', inset: '0', overflow: 'hidden', pointerEvents: 'none' });
    this._tools = this._toolbar();
    this._sceneBounds = new THREE.Box3();
    container.append(canvas, this._labelLayer, this._tools);

    this._onPointerDown = (e) => {
      this._pointers.add(e.pointerId);
      this._down = this._pointers.size === 1 ? { x: e.clientX, y: e.clientY, t: performance.now() } : null;  // pinch: no pick
    };
    this._onPointerUp = (e) => {
      this._pointers.delete(e.pointerId);
      const down = this._down;
      this._down = null;
      // a tap, not the end of an orbit drag
      if (!down || Math.hypot(e.clientX - down.x, e.clientY - down.y) > 6 || performance.now() - down.t > 600) return;
      const hit = this._pickAt(e.clientX, e.clientY);
      if (hit) this._emitPick(hit);
    };
    this._onPointerCancel = (e) => { this._pointers.delete(e.pointerId); this._down = null; };
    canvas.addEventListener('pointerdown', this._onPointerDown);
    canvas.addEventListener('pointerup', this._onPointerUp);
    canvas.addEventListener('pointercancel', this._onPointerCancel);
    this._onVisibility = () => { if (!document.hidden) { this._requestRender(); this._pollSoon(); } };
    document.addEventListener('visibilitychange', this._onVisibility);
    this._resizeObserver = new ResizeObserver(() => this.resize());
    this._resizeObserver.observe(container);
    this.resize();
  }

  // ---------- loading ----------
  // Fetches the model and the scene, builds both, starts polling; resolves once the meshes are in (this.status says
  // what was missing: model 'missing' without a URDF, 'error' if it could not be read).
  async load() {
    const [model, scene] = await Promise.all([
      this.fetchJSON('/api/robot/model').catch((e) => ({ failed: e?.status === 404 ? 'missing' : 'error' })),
      this.fetchJSON('/api/scene').catch(() => null),
    ]);
    if (this._disposed) return this;
    this.status.scene = scene ? 'ok' : 'error';
    this._sceneAt = Date.now();
    if (scene) this._buildScene(scene);
    let meshes = Promise.resolve();
    if (model.failed) this.status.model = model.failed;
    else {
      this.model = model;
      try { meshes = this._buildRobot(model); this.status.model = 'ok'; } catch (error) { console.error('Viewer3D: URDF', error); this.status.model = 'error'; }
    }
    this._poll();
    await meshes;
    if (this._disposed) return this;
    this.fit();
    return this;
  }

  _buildRobot(model) {
    const loader = new URDFLoader();
    const pending = [];
    // STLs through one cached STLLoader, all with the one metal material; Collada the default way
    loader.loadMeshCb = (path, manager, done) => {
      if (/\.dae$/i.test(path)) {
        pending.push(new Promise((resolve) => loader.defaultMeshLoader(path, manager, (obj, err) => {
          done(obj, err);
          obj?.traverse((c) => { if (c.isMesh) { this._urdfMaterials.add(c.material); c.material = this._mats.metal; } });
          this._requestRender();
          resolve();
        })));
        return;
      }
      if (!/\.stl$/i.test(path)) { console.warn(`Viewer3D: no loader for ${path}`); return; }
      pending.push(this._meshGeometry(path).then((geometry) => {
        const mesh = new THREE.Mesh(geometry, this._mats.metal);
        done(mesh);
        this._urdfMaterials.add(mesh.material);  // urdf-loader sets the URDF's <material> in done(): back to metal
        mesh.material = this._mats.metal;
        this._requestRender();
      }, (error) => done(null, error)));
    };
    const robot = loader.parse(model.urdf);
    robot.traverse((c) => {  // primitives (box, cylinder, sphere) come with the URDF's material
      if (c.isMesh && c.material !== this._mats.metal) { this._urdfMaterials.add(c.material); c.material = this._mats.metal; }
    });
    this.robot = robot;
    this.base.add(robot);
    Object.entries(model.tool_frames || {}).forEach(([tool, frame]) => {  // small TCP triads, live robot only
      const link = robot.links[frame.link];
      if (!link) return;
      const triad = new THREE.LineSegments(this._geoms.tool, this._mats.triad);
      triad.position.set(...(frame.xyz || [0, 0, 0]));
      triad.quaternion.copy(quatRPY(frame.rpy));
      triad.name = `tool:${tool}`;
      triad.userData.noGhost = true;
      link.add(triad);
    });
    if (Object.keys(this._liveJoints).length) this._applyJoints(this._liveJoints);
    this._requestRender();
    return Promise.all(pending).then(() => { if (!this._disposed) this._makeGhost(); });
  }

  _meshGeometry(url) {
    if (!this._geometries.has(url)) {
      this._stl ||= new STLLoader();
      this._geometries.set(url, this._stl.loadAsync(url));
    }
    return this._geometries.get(url);
  }

  // The ghost shares the live robot's geometries; only the material differs.
  _makeGhost() {
    const ghost = this.robot.clone();
    const drop = [];
    ghost.traverse((c) => {
      if (c.userData.noGhost) drop.push(c);
      else if (c.isMesh) { c.material = this._mats.ghost; c.renderOrder = 1; }
    });
    drop.forEach((c) => c.removeFromParent());
    ghost.visible = false;
    this.ghost = ghost;
    this.base.add(ghost);
    this._updateGhost();
  }

  // ---------- live: joints and base ----------
  _active() {
    const c = this.container;
    return !document.hidden && c.isConnected && c.offsetWidth > 0 && c.offsetHeight > 0;  // offsetWidth 0: display:none
  }

  _pollSoon() {
    if (this._disposed || !this._polling) return;
    clearTimeout(this._pollTimer);
    this._pollTimer = setTimeout(() => this._poll(), 0);
  }

  // One request of each at a time (a slow server is not piled up); skipped while hidden or in a background tab.
  async _poll() {
    if (this._disposed || this._busy) return;  // the request in flight schedules the next one
    this._polling = true;
    clearTimeout(this._pollTimer);
    if (this._active()) {
      this._busy = true;
      const now = Date.now();
      const sceneDue = now - this._sceneAt > SCENE_POLL_MS;
      if (sceneDue) this._sceneAt = now;
      const [joints, base, scene] = await Promise.all([
        this.robot ? this.fetchJSON('/api/robot/joints').catch(() => null) : null,
        this.fetchJSON('/api/robot/base').catch(() => ({ failed: true })),  // 503: no TF
        sceneDue ? this.fetchJSON('/api/scene').catch(() => null) : null,
      ]);
      this._busy = false;
      if (this._disposed) return;
      try {  // a malformed answer must not end the loop
        let changed = false;
        if (joints) changed = this._applyJoints(joints) || changed;
        if (base?.xyz && base?.quat) changed = this._applyBase(base, changed) || changed;
        // without TF the robot stays where it was (at the origin until a pose came) and the toolbar says so
        this._poseNote.hidden = !base?.failed;
        if (scene && scene.updated !== this.sceneData?.updated) { this._buildScene(scene); changed = true; }
        if (changed) this._requestRender();
      } catch (error) { console.error('Viewer3D: poll', error); }
    }
    clearTimeout(this._pollTimer);  // a _pollSoon() while waiting: still one loop
    this._pollTimer = setTimeout(() => this._poll(), this.pollMs);
  }

  // Mimic joints are left out: urdf-loader parses <mimic> and moves them with their master joint.
  _applyJoints(values) {
    this._liveJoints = values;
    if (!this.robot) return false;
    let changed = false;
    Object.entries(values).forEach(([name, value]) => {
      const joint = this.robot.joints[name];
      if (!joint || joint.mimicJoint || typeof value !== 'number') return;
      changed = joint.setJointValue(value) || changed;
    });
    if (changed && this.ghost?.visible) this._updateGhost();  // joints the preview does not set follow the live robot
    return changed;
  }

  // The pose is of base.child; the URDF root may sit above it (base_footprint -> base_link), so the root goes to
  // world <- child <- root. Recomputed when the joints moved, in case a movable joint is in between.
  _applyBase(base, jointsMoved = false) {
    const key = JSON.stringify([base.child, base.xyz, base.quat]);
    if (key === this._baseKey && !jointsMoved) return false;
    this._baseKey = key;
    this._base = base;
    const matrix = pose(base.xyz, quatWXYZ(base.quat));
    const link = this.robot?.links?.[base.child];
    if (link && link !== this.robot) {
      this.robot.updateMatrixWorld(true);
      const rootToChild = this.robot.matrixWorld.clone().invert().multiply(link.matrixWorld);
      matrix.multiply(rootToChild.invert());
    }
    this.base.matrix.copy(matrix);
    this.base.matrixWorldNeedsUpdate = true;
    const position = new THREE.Vector3(...base.xyz);
    if (this.followRobot && this._basePos) {  // keep the view's offset to the robot
      const delta = position.clone().sub(this._basePos);
      this.controls.target.add(delta);
      this.camera.position.add(delta);
      this.controls.update();
    }
    this._basePos = position;
    return true;
  }

  // ---------- ghost (previews) ----------
  setGhost(jointValues) {
    this._ghostValues = jointValues ? { ...jointValues } : null;
    this._mats.ghost.color.copy(this._accent());
    this._updateGhost();
  }

  _updateGhost() {
    if (!this.ghost) return;  // not built yet: applied when the meshes are in
    this.ghost.visible = !!this._ghostValues;
    if (this._ghostValues) {
      Object.entries({ ...this._liveJoints, ...this._ghostValues }).forEach(([name, value]) => {
        const joint = this.ghost.joints[name];
        if (joint && !joint.mimicJoint && typeof value === 'number') joint.setJointValue(value);
      });
    }
    this._requestRender();
  }

  // ---------- scene: collision objects, floor, locations, placed objects, points ----------
  _buildScene(data) {
    this.sceneData = data;
    this._clearScene();
    const bounds = new THREE.Box3();
    const objects = data.objects || [];
    const boxes = objects.filter((o) => o.type === 'box' && o.size);
    if (boxes.length) {
      // one instanced mesh for the faces, one merged line set for the edges: two draw calls for all boxes
      const faces = new THREE.InstancedMesh(this._geoms.box, this._mats.box, boxes.length);
      const edges = new Float32Array(boxes.length * BOX_EDGES.length);
      const v = new THREE.Vector3();
      boxes.forEach((box, i) => {
        const matrix = new THREE.Matrix4().compose(new THREE.Vector3(...box.xyz), quatWXYZ(box.quat || [1, 0, 0, 0]),
          new THREE.Vector3(...box.size));
        faces.setMatrixAt(i, matrix);
        for (let k = 0; k < BOX_EDGES.length; k += 3) {
          v.set(BOX_EDGES[k], BOX_EDGES[k + 1], BOX_EDGES[k + 2]).applyMatrix4(matrix);
          edges.set([v.x, v.y, v.z], i * BOX_EDGES.length + k);
          bounds.expandByPoint(v);
        }
      });
      faces.instanceMatrix.needsUpdate = true;
      faces.frustumCulled = false;  // its bounding sphere would be the unit box
      const geometry = new THREE.BufferGeometry();
      geometry.setAttribute('position', new THREE.BufferAttribute(edges, 3));
      this.cell.add(faces, new THREE.LineSegments(geometry, this._mats.edge));
    }
    // spheres [r], cylinders and cones [height, r] (SolidPrimitive dimensions): faces only, one draw call per type.
    // Prisms are left out.
    const scaleOf = { sphere: ([r]) => [r, r, r], cylinder: ([h, r]) => [r, r, h], cone: ([h, r]) => [r, r, h] };
    Object.entries(scaleOf).forEach(([type, scale]) => {
      const list = objects.filter((o) => o.type === type && o.dimensions?.length >= (type === 'sphere' ? 1 : 2));
      if (!list.length) return;
      const faces = new THREE.InstancedMesh(this._geoms[type], this._mats.box, list.length);
      list.forEach((o, i) => {
        const size = new THREE.Vector3(...scale(o.dimensions));
        faces.setMatrixAt(i, new THREE.Matrix4().compose(new THREE.Vector3(...o.xyz), quatWXYZ(o.quat || [1, 0, 0, 0]), size));
        const reach = Math.hypot(size.x, size.z / 2);  // the bounding sphere: holds it however it is turned
        bounds.expandByPoint(new THREE.Vector3(...o.xyz).addScalar(reach)).expandByPoint(new THREE.Vector3(...o.xyz).subScalar(reach));
      });
      faces.instanceMatrix.needsUpdate = true;
      faces.frustumCulled = false;
      this.cell.add(faces);
    });

    (data.locations || []).forEach((loc) => {
      const root = new THREE.Group();
      root.position.set(loc.x, loc.y, 0.003);  // just above the grid
      root.quaternion.copy(quatYaw(loc.yaw));
      const body = new THREE.Mesh(this._geoms.arrow, this._mats.location);
      body.renderOrder = 2;
      root.add(body);
      this._addMarker({ kind: 'location', name: loc.name }, root, body, body, loc.name);
      bounds.expandByPoint(root.position);
    });

    const placedPose = new Map();
    (data.placed || []).forEach((obj) => {
      const root = new THREE.Group();
      root.position.set(obj.x, obj.y, obj.z ?? 0);
      root.quaternion.copy(quatYaw(obj.yaw));
      const body = new THREE.Mesh(this._geoms.dot, this._mats.marker);
      const pick = new THREE.Mesh(this._geoms.pick, this._mats.pick);
      root.add(new THREE.LineSegments(this._geoms.triad, this._mats.triad), body, pick);
      this._addMarker({ kind: 'placed', name: obj.name, object: obj.object }, root, body, pick, obj.name);
      // points are stored relative to the top item of a stack: their frame is at top_z, not z
      placedPose.set(obj, pose([obj.x, obj.y, obj.top_z ?? obj.z ?? 0], root.quaternion));
      bounds.expandByPoint(root.position);
    });

    // a point's tool poses are in its object type's frame: one marker per tool, per placed object of that type
    (data.points || []).forEach((point) => {
      (data.placed || []).filter((obj) => obj.object === point.object_type).forEach((obj) => {
        Object.entries(point.tools || {}).forEach(([tool, toolPose], i) => {
          const root = new THREE.Group();
          placedPose.get(obj).clone().multiply(pose(toolPose.xyz || [0, 0, 0], quatRPY(toolPose.rpy)))
            .decompose(root.position, root.quaternion, root.scale);
          const body = new THREE.LineSegments(this._geoms.gripper, this._mats.gripper);
          const pick = new THREE.Mesh(this._geoms.pick, this._mats.pick);
          root.add(new THREE.LineSegments(this._geoms.tool, this._mats.triad), body, pick);
          const info = { kind: 'point', name: point.name, object_type: point.object_type, placed: obj.name, tool };
          this._addMarker(info, root, body, pick, i === 0 ? point.name : null);  // one label per point and object
          bounds.expandByPoint(root.position);
        });
      });
    });

    this._sceneBounds = bounds;
    const extent = bounds.isEmpty() ? 2 : Math.max(...[bounds.min.x, bounds.min.y, bounds.max.x, bounds.max.y].map(Math.abs));
    this._setGrid(Math.max(2, Math.ceil(extent + 1)));
    this._applyHighlight();
    this._requestRender();
  }

  _addMarker(info, root, body, pick, labelText) {
    pick.userData.pick = info;
    this._pickables.push(pick);
    this.markers.add(root);
    const label = labelText ? this._label(labelText, info, root) : null;
    this._markers.push({ ...info, info, root, body, baseMaterial: body.material, label });
  }

  _clearScene() {
    this.cell.children.forEach((c) => {  // the per-build parts; shared geometries and materials stay
      if (c.isInstancedMesh) c.dispose();
      else c.geometry.dispose();
    });
    this.cell.clear();
    this.markers.clear();
    this._markers = [];
    this._pickables = [];
    this._labels.forEach(({ node }) => node.remove());
    this._labels = [];
  }

  // The floor: a subtle grid at z=0, 0.5 m cells, half-size in metres (rebuilt only when the cell grows).
  _setGrid(half) {
    if (this.grid?.userData.half === half) return;
    if (this.grid) { this.grid.removeFromParent(); this.grid.geometry.dispose(); this.grid.material.dispose(); }
    const grid = new THREE.GridHelper(2 * half, 4 * half, 0x9099a3, 0xb4bac1);
    grid.rotation.x = Math.PI / 2;  // GridHelper lies in x-z; the floor is x-y
    grid.material.transparent = true;
    grid.material.opacity = 0.35;
    grid.material.depthWrite = false;
    grid.userData.half = half;
    this.grid = grid;
    this.scene.add(grid);
  }

  // ---------- labels: HTML, placed over the canvas on each render ----------
  _label(text, info, object) {
    const node = el('div', `viewer3d-label ${info.kind}`, text);
    Object.assign(node.style, { position: 'absolute', left: '0', top: '0', pointerEvents: 'auto', cursor: 'pointer' });
    node.addEventListener('click', () => this._emitPick(info));  // labels are easier to hit than markers on a phone
    this._labelLayer.append(node);
    this._labels.push({ node, object });
    return node;
  }

  _placeLabels() {
    const { w, h } = this._size || {};
    if (!w) return;
    const v = new THREE.Vector3();
    this._labels.forEach(({ node, object }) => {
      v.setFromMatrixPosition(object.matrixWorld).project(this.camera);
      const visible = v.z < 1 && Math.abs(v.x) <= 1.05 && Math.abs(v.y) <= 1.05;
      node.style.display = visible ? '' : 'none';
      if (visible) node.style.transform = `translate(${((v.x + 1) / 2) * w}px, ${((1 - v.y) / 2) * h}px) translate(-50%, -130%)`;
    });
  }

  // ---------- highlight, focus, pick ----------
  _accent() {
    const colour = new THREE.Color('#0000ff');
    const css = getComputedStyle(this.container).getPropertyValue('--accent').trim();
    if (css) colour.setStyle(css);  // a colour three cannot parse keeps the fallback (with a console warning)
    return colour;
  }

  // {kind: 'location' | 'placed' | 'point', name} or null. A point marker can be narrowed with placed / tool.
  setHighlight(target) {
    this.highlight = target ? { ...target } : null;
    const accent = this._accent();  // the theme may have changed since
    [this._mats.accentMesh, this._mats.accentLine, this._mats.ghost].forEach((m) => m.color.copy(accent));
    this._applyHighlight();
    this._requestRender();
  }

  _matches(marker, h) {
    return !!h && marker.kind === h.kind && marker.name === h.name
      && ['object_type', 'placed', 'tool'].every((key) => h[key] === undefined || h[key] === marker[key]);
  }

  _applyHighlight() {
    this._markers.forEach((m) => {
      const on = this._matches(m, this.highlight);
      m.body.material = on ? (m.body.isLine ? this._mats.accentLine : this._mats.accentMesh) : m.baseMaterial;
      m.root.scale.setScalar(on ? 1.6 : 1);
      m.label?.classList.toggle('active', on);
    });
  }

  // Moves the camera's target to the marker, closer if the view is far away.
  focus(kind, name) {
    const marker = this._markers.find((m) => this._matches(m, { kind, name }));
    if (!marker) return false;
    marker.root.updateMatrixWorld();
    const at = new THREE.Vector3().setFromMatrixPosition(marker.root.matrixWorld);
    const offset = this.camera.position.clone().sub(this.controls.target);
    offset.setLength(Math.min(offset.length(), 2));
    this.controls.target.copy(at);
    this.camera.position.copy(at).add(offset);
    this.controls.update();
    this._requestRender();
    return true;
  }

  onPick(callback) {
    this._pickCallbacks.push(callback);
    return () => { this._pickCallbacks = this._pickCallbacks.filter((c) => c !== callback); };
  }

  _pickAt(x, y) {
    const rect = this.renderer.domElement.getBoundingClientRect();
    const ndc = new THREE.Vector2(((x - rect.left) / rect.width) * 2 - 1, -((y - rect.top) / rect.height) * 2 + 1);
    this._raycaster ||= new THREE.Raycaster();
    this._raycaster.setFromCamera(ndc, this.camera);
    return this._raycaster.intersectObjects(this._pickables, false)[0]?.object.userData.pick || null;
  }

  _emitPick(info) { this._pickCallbacks.forEach((callback) => callback({ ...info })); }

  // ---------- camera ----------
  // Frames the robot and the scene (not the grid). The collision objects by the bounds kept when building them: Box3
  // would take an instanced mesh as its one unit shape.
  fit() {
    this.scene.updateMatrixWorld();  // the base's matrix may not have been rendered yet
    const box = this._sceneBounds.clone();
    [this.robot, this.markers].forEach((object) => { if (object) box.expandByObject(object); });
    if (box.isEmpty()) box.set(new THREE.Vector3(-1, -1, 0), new THREE.Vector3(1, 1, 1));
    const center = box.getCenter(new THREE.Vector3());
    const radius = Math.max(box.getSize(new THREE.Vector3()).length() / 2, 0.5);
    const distance = radius / Math.sin(THREE.MathUtils.degToRad(this.camera.fov / 2)) * 1.05;
    const direction = this.camera.position.clone().sub(this.controls.target);
    if (direction.lengthSq() < 1e-6) direction.set(1, -1.2, 0.9);
    this.camera.position.copy(center).add(direction.setLength(distance));
    this.camera.near = distance / 200;
    this.camera.far = distance * 20;
    this.camera.updateProjectionMatrix();
    this.controls.target.copy(center);
    this.controls.update();
    this._requestRender();
  }

  setFollow(on) {
    this.followRobot = !!on;
    this._followButton?.setAttribute('aria-pressed', String(this.followRobot));
    if (this.followRobot && this._basePos) {  // jump to the robot, keep the view's offset
      const delta = this._basePos.clone().sub(this.controls.target);
      this.controls.target.add(delta);
      this.camera.position.add(delta);
      this.controls.update();
    }
    this._requestRender();
  }

  setLabels(on) {
    this.labelsOn = !!on;
    this._labelsButton?.setAttribute('aria-pressed', String(this.labelsOn));
    this._labelLayer.style.display = this.labelsOn ? '' : 'none';
    this._requestRender();
  }

  _toolbar() {
    const tools = el('div', 'viewer3d-tools');
    const button = (text, title, onClick) => {
      const node = el('button', 'btn small', text);
      node.type = 'button';
      node.title = title;
      node.addEventListener('click', onClick);
      tools.append(node);
      return node;
    };
    button('Fit', 'Frame the robot and the cell', () => this.fit());
    this._followButton = button('Follow robot', 'Keep the robot in the middle as it drives', () => this.setFollow(!this.followRobot));
    this._followButton.setAttribute('aria-pressed', 'false');
    this._labelsButton = button('Labels', 'Show or hide the names', () => this.setLabels(!this.labelsOn));
    this._labelsButton.setAttribute('aria-pressed', 'true');
    this._poseNote = el('span', 'viewer3d-note', 'no robot pose');
    this._poseNote.title = 'No base pose from TF (/api/robot/base): the robot is drawn where it was last, or at the origin';
    this._poseNote.hidden = true;
    tools.append(this._poseNote);
    return tools;
  }

  // ---------- rendering: only when something changed ----------
  _requestRender() {
    if (this._frame || this._disposed) return;
    this._frame = requestAnimationFrame(() => {
      this._frame = 0;
      this.renderer.render(this.scene, this.camera);
      if (this.labelsOn) this._placeLabels();
    });
  }

  resize() {
    const w = this.container.clientWidth;
    const h = this.container.clientHeight;
    if (!w || !h || this._disposed) return;  // hidden: keep the last size
    this._size = { w, h };
    this.renderer.setSize(w, h, false);
    this.camera.aspect = w / h;
    this.camera.updateProjectionMatrix();
    this._requestRender();
  }

  dispose() {
    if (this._disposed) return;
    this._disposed = true;
    clearTimeout(this._pollTimer);
    cancelAnimationFrame(this._frame);
    this._resizeObserver.disconnect();
    document.removeEventListener('visibilitychange', this._onVisibility);
    const canvas = this.renderer.domElement;
    canvas.removeEventListener('pointerdown', this._onPointerDown);
    canvas.removeEventListener('pointerup', this._onPointerUp);
    canvas.removeEventListener('pointercancel', this._onPointerCancel);
    this.controls.dispose();
    const geometries = new Set(Object.values(this._geoms));
    const materials = new Set([...Object.values(this._mats), ...this._urdfMaterials]);
    this.scene.traverse((o) => {
      if (o.isInstancedMesh) o.dispose();
      if (o.geometry) geometries.add(o.geometry);
      if (o.material) [].concat(o.material).forEach((m) => materials.add(m));
    });
    this._geometries.forEach((p) => p.then((g) => g.dispose(), () => {}));  // STLs still loading are freed when in
    geometries.forEach((g) => g.dispose());
    materials.forEach((m) => { m.map?.dispose(); m.dispose(); });
    this.scene.clear();
    this.renderer.dispose();
    this.renderer.forceContextLoss();
    canvas.remove();
    this._labelLayer.remove();
    this._tools.remove();
    this.container.classList.remove('viewer3d');
    this._pickCallbacks = [];
    this.robot = null;
    this.ghost = null;
  }
}

window.Viewer3D = Viewer3D;
window.dispatchEvent(new Event('viewer3d-ready'));
