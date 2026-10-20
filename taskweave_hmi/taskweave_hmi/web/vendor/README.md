# Vendored browser libraries

Copied unchanged from the npm packages, so the HMI's 3D view (viewer3d.js) works on a cell network without internet.
The licence files are named LICENSE.txt so the HMI server serves them as text. No build step: the page maps the bare `three` imports with an import map (see the top of viewer3d.js).

| Files | Package | From the package |
|---|---|---|
| three/three.module.min.js | three 0.160.0 (r160), MIT | build/three.module.min.js |
| three/addons/controls/OrbitControls.js | three 0.160.0 | examples/jsm/controls/OrbitControls.js |
| three/addons/loaders/STLLoader.js | three 0.160.0 | examples/jsm/loaders/STLLoader.js |
| three/addons/loaders/ColladaLoader.js | three 0.160.0 | examples/jsm/loaders/ColladaLoader.js (imported by URDFLoader.js) |
| three/addons/loaders/TGALoader.js | three 0.160.0 | examples/jsm/loaders/TGALoader.js (imported by ColladaLoader.js) |
| three/LICENSE.txt | three 0.160.0 | LICENSE |
| urdf-loader/URDFLoader.js, URDFClasses.js | urdf-loader 0.12.7, Apache-2.0 | src/URDFLoader.js, src/URDFClasses.js |
| urdf-loader/LICENSE.txt | urdf-loader 0.12.7 | not in the npm package: github.com/gkjohnson/urdf-loaders, tag v0.12.7 |

Sources:

- https://registry.npmjs.org/three/-/three-0.160.0.tgz (https://github.com/mrdoob/three.js, tag r160)
- https://registry.npmjs.org/urdf-loader/-/urdf-loader-0.12.7.tgz (https://github.com/gkjohnson/urdf-loaders)
- https://raw.githubusercontent.com/gkjohnson/urdf-loaders/v0.12.7/LICENSE

To update: `npm pack three@<version> urdf-loader@<version>`, copy the same files, check the import statements
(`grep -n "from '" ...`) against the import map in viewer3d.js and viewer3d-demo.html.
