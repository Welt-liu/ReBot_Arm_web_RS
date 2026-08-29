import * as THREE from 'three';
import { OrbitControls } from 'three/addons/controls/OrbitControls.js';
import { RectAreaLightUniformsLib } from 'three/addons/lights/RectAreaLightUniformsLib.js';

const GEOM = {
  PLANE: 0,
  HFIELD: 1,
  SPHERE: 2,
  CAPSULE: 3,
  ELLIPSOID: 4,
  CYLINDER: 5,
  BOX: 6,
  MESH: 7
};

function enumValue(mujoco, name, fallback) {
  const value = mujoco.mjtGeom?.[name]?.value;
  return Number.isInteger(value) ? value : fallback;
}

function geomTypes(mujoco) {
  return {
    plane: enumValue(mujoco, 'mjGEOM_PLANE', GEOM.PLANE),
    sphere: enumValue(mujoco, 'mjGEOM_SPHERE', GEOM.SPHERE),
    capsule: enumValue(mujoco, 'mjGEOM_CAPSULE', GEOM.CAPSULE),
    ellipsoid: enumValue(mujoco, 'mjGEOM_ELLIPSOID', GEOM.ELLIPSOID),
    cylinder: enumValue(mujoco, 'mjGEOM_CYLINDER', GEOM.CYLINDER),
    box: enumValue(mujoco, 'mjGEOM_BOX', GEOM.BOX),
    mesh: enumValue(mujoco, 'mjGEOM_MESH', GEOM.MESH)
  };
}

function copyFloats(source, start, count) {
  return Float32Array.from(source.subarray(start, start + count));
}

function createMeshGeometry(model, meshId) {
  const vertadr = model.mesh_vertadr[meshId];
  const vertnum = model.mesh_vertnum[meshId];
  const faceadr = model.mesh_faceadr[meshId];
  const facenum = model.mesh_facenum[meshId];
  if (!vertnum || !facenum) return new THREE.BufferGeometry();

  const geometry = new THREE.BufferGeometry();
  geometry.setAttribute(
    'position',
    new THREE.BufferAttribute(copyFloats(model.mesh_vert, vertadr * 3, vertnum * 3), 3)
  );
  const indices = Array.from(model.mesh_face.subarray(faceadr * 3, (faceadr + facenum) * 3));
  geometry.setIndex(indices);

  if (model.mesh_normal && model.mesh_normaladr && model.mesh_normalnum) {
    const normaladr = model.mesh_normaladr[meshId];
    const normalnum = model.mesh_normalnum[meshId];
    if (normalnum === vertnum) {
      geometry.setAttribute(
        'normal',
        new THREE.BufferAttribute(copyFloats(model.mesh_normal, normaladr * 3, normalnum * 3), 3)
      );
    }
  }
  if (!geometry.getAttribute('normal')) geometry.computeVertexNormals();
  geometry.computeBoundingSphere();
  return geometry;
}

function primitiveGeometry(type, size, types) {
  if (type === types.plane) {
    return new THREE.PlaneGeometry(2 * (size[0] || 5), 2 * (size[1] || 5));
  }
  if (type === types.sphere) {
    return new THREE.SphereGeometry(size[0], 24, 16);
  }
  if (type === types.capsule) {
    const geometry = new THREE.CapsuleGeometry(size[0], 2 * size[2], 8, 16);
    geometry.rotateX(Math.PI / 2);
    return geometry;
  }
  if (type === types.ellipsoid) {
    const geometry = new THREE.SphereGeometry(1, 24, 16);
    geometry.scale(size[0], size[1], size[2]);
    return geometry;
  }
  if (type === types.cylinder) {
    const geometry = new THREE.CylinderGeometry(size[0], size[0], 2 * size[2], 24);
    geometry.rotateX(Math.PI / 2);
    return geometry;
  }
  if (type === types.box) {
    return new THREE.BoxGeometry(2 * size[0], 2 * size[1], 2 * size[2]);
  }
  return null;
}

function geomMaterial(model, index) {
  const rgba = model.geom_rgba.subarray(index * 4, index * 4 + 4);
  let color = new THREE.Color(rgba[0], rgba[1], rgba[2]);
  let opacity = rgba[3];
  let metalness = 0.12;
  let roughness = 0.55;
  const matid = model.geom_matid ? model.geom_matid[index] : -1;
  if (matid >= 0) {
    if (model.mat_rgba) {
      const matRgba = model.mat_rgba.subarray(matid * 4, matid * 4 + 4);
      color = new THREE.Color(matRgba[0], matRgba[1], matRgba[2]);
      opacity = matRgba[3];
    }
    if (model.mat_metallic) metalness = model.mat_metallic[matid];
    if (model.mat_roughness) roughness = model.mat_roughness[matid];
  }
  return new THREE.MeshStandardMaterial({
    color,
    metalness: Number.isFinite(metalness) ? metalness : 0.12,
    roughness: Number.isFinite(roughness) ? roughness : 0.55,
    transparent: opacity < 0.999,
    opacity,
    side: THREE.DoubleSide
  });
}

function shouldDrawGeom(model, index) {
  const group = model.geom_group ? model.geom_group[index] : 0;
  if (group >= 3) return false;
  const alpha = model.geom_rgba[index * 4 + 3];
  if (alpha < 0.02) return false;
  return true;
}

function setPose(mesh, xpos, xmat, index) {
  mesh.matrixAutoUpdate = false;
  mesh.matrix.set(
    xmat[index * 9 + 0], xmat[index * 9 + 1], xmat[index * 9 + 2], xpos[index * 3 + 0],
    xmat[index * 9 + 3], xmat[index * 9 + 4], xmat[index * 9 + 5], xpos[index * 3 + 1],
    xmat[index * 9 + 6], xmat[index * 9 + 7], xmat[index * 9 + 8], xpos[index * 3 + 2],
    0, 0, 0, 1
  );
  mesh.matrixWorldNeedsUpdate = true;
}

export function createSceneView(host) {
  const scene = new THREE.Scene();
  scene.background = new THREE.Color(0x101418);

  const camera = new THREE.PerspectiveCamera(45, 1, 0.02, 20);
  camera.up.set(0, 0, 1);
  camera.position.set(0.85, -0.95, 0.62);

  const renderer = new THREE.WebGLRenderer({ antialias: true });
  renderer.setPixelRatio(Math.min(window.devicePixelRatio || 1, 2));
  renderer.shadowMap.enabled = true;
  renderer.shadowMap.type = THREE.PCFSoftShadowMap;
  renderer.domElement.style.zIndex = '0';
  host.appendChild(renderer.domElement);
  ['callouts', 'drag-cluster'].forEach((id) => {
    const node = host.querySelector(`#${id}`);
    if (node) host.appendChild(node);
  });

  const ndc = new THREE.Vector3();

  const controls = new OrbitControls(camera, renderer.domElement);
  controls.target.set(0.28, 0, 0.16);
  controls.enableDamping = true;

  RectAreaLightUniformsLib.init();
  scene.add(new THREE.AmbientLight(0xf3efe6, 0.34));
  scene.add(new THREE.HemisphereLight(0xe7eef8, 0x4a4742, 0.36));

  const ceiling = new THREE.RectAreaLight(0xfff2e2, 1.35, 1.9, 1.45);
  ceiling.position.set(0.28, 0, 1.4);
  ceiling.lookAt(0.28, 0, 0);
  scene.add(ceiling);

  const overhead = new THREE.SpotLight(0xfff6ea, 1.8, 6.2, Math.PI / 2.15, 1, 1.35);
  overhead.position.set(0.28, 0, 1.85);
  overhead.target.position.set(0.28, 0, 0);
  overhead.castShadow = true;
  overhead.shadow.mapSize.set(2048, 2048);
  overhead.shadow.radius = 20;
  overhead.shadow.blurSamples = 24;
  overhead.shadow.bias = -0.00012;
  overhead.shadow.normalBias = 0.02;
  overhead.shadow.camera.near = 0.3;
  overhead.shadow.camera.far = 4.6;
  scene.add(overhead);
  scene.add(overhead.target);

  const fill = new THREE.DirectionalLight(0xe6edf6, 0.06);
  fill.position.set(0.55, -0.75, 0.85);
  scene.add(fill);

  const tcpMarker = new THREE.Mesh(
    new THREE.SphereGeometry(0.02, 24, 16),
    new THREE.MeshStandardMaterial({
      color: 0x33d6b0,
      emissive: 0x0a4d3d,
      emissiveIntensity: 1.2
    })
  );
  tcpMarker.visible = false;
  scene.add(tcpMarker);

  const targetGhost = new THREE.Mesh(
    new THREE.SphereGeometry(0.018, 28, 18),
    new THREE.MeshBasicMaterial({ color: 0xf2a541, transparent: true, opacity: 0.85 })
  );
  targetGhost.visible = false;
  scene.add(targetGhost);

  const dragErrorLine = new THREE.Line(
    new THREE.BufferGeometry().setFromPoints([new THREE.Vector3(), new THREE.Vector3()]),
    new THREE.LineBasicMaterial({ color: 0xff6b5f, transparent: true, opacity: 0.82 })
  );
  dragErrorLine.visible = false;
  scene.add(dragErrorLine);

  const raycaster = new THREE.Raycaster();
  const ndcMouse = new THREE.Vector2();
  const planeHit = new THREE.Vector3();

  const meshes = [];
  const meshGeometries = new Map();
  let types = geomTypes({});

  function resize() {
    const width = Math.max(1, host.clientWidth);
    const height = Math.max(1, host.clientHeight);
    camera.aspect = width / height;
    camera.updateProjectionMatrix();
    renderer.setSize(width, height, false);
  }

  function build(mujoco, model) {
    clear();
    types = geomTypes(mujoco);
    for (let i = 0; i < model.ngeom; i += 1) {
      if (!shouldDrawGeom(model, i)) continue;
      const type = model.geom_type[i];
      const size = model.geom_size.subarray(i * 3, i * 3 + 3);
      let geometry = null;
      if (type === types.mesh) {
        const meshId = model.geom_dataid[i];
        if (!meshGeometries.has(meshId)) {
          meshGeometries.set(meshId, createMeshGeometry(model, meshId));
        }
        geometry = meshGeometries.get(meshId);
      } else {
        geometry = primitiveGeometry(type, size, types);
      }
      if (!geometry) continue;
      const mesh = new THREE.Mesh(geometry, geomMaterial(model, i));
      mesh.castShadow = type !== types.plane;
      mesh.receiveShadow = true;
      mesh.userData.geomIndex = i;
      scene.add(mesh);
      meshes.push(mesh);
    }
  }

  function sync(data) {
    const xpos = data.geom_xpos;
    const xmat = data.geom_xmat;
    meshes.forEach((mesh) => setPose(mesh, xpos, xmat, mesh.userData.geomIndex));
  }

  function render() {
    controls.update();
    renderer.render(scene, camera);
  }

  function projectWorld(x, y, z) {
    ndc.set(x, y, z).project(camera);
    const width = Math.max(1, host.clientWidth);
    const height = Math.max(1, host.clientHeight);
    return {
      x: (ndc.x * 0.5 + 0.5) * width,
      y: (-ndc.y * 0.5 + 0.5) * height,
      visible: ndc.z > -1 && ndc.z < 1 && ndc.x > -1.35 && ndc.x < 1.35
    };
  }

  function intersectPlane(clientX, clientY, plane) {
    const rect = host.getBoundingClientRect();
    ndcMouse.set(
      ((clientX - rect.left) / rect.width) * 2 - 1,
      -((clientY - rect.top) / rect.height) * 2 + 1
    );
    raycaster.setFromCamera(ndcMouse, camera);
    return raycaster.ray.intersectPlane(plane, planeHit) ? planeHit.clone() : null;
  }

  function setOrbitEnabled(enabled) {
    controls.enabled = enabled;
  }

  function setDragVisuals({ tcp, target, dragMode, dragging }) {
    tcpMarker.visible = Boolean(dragMode && tcp);
    if (tcp) tcpMarker.position.set(tcp.x, tcp.y, tcp.z);
    targetGhost.visible = Boolean(dragMode && dragging && target);
    if (target) targetGhost.position.set(target.x, target.y, target.z);
    const showLine = Boolean(dragMode && dragging && tcp && target && tcp.distanceTo(target) > 0.001);
    dragErrorLine.visible = showLine;
    if (showLine) {
      dragErrorLine.geometry.setFromPoints([tcp, target]);
    }
  }

  function clear() {
    meshes.forEach((mesh) => {
      scene.remove(mesh);
      mesh.material.dispose();
    });
    meshes.length = 0;
    meshGeometries.forEach((geometry) => geometry.dispose());
    meshGeometries.clear();
  }

  const observer = new ResizeObserver(resize);
  observer.observe(host);
  resize();

  return {
    build,
    sync,
    render,
    resize,
    projectWorld,
    intersectPlane,
    setOrbitEnabled,
    setDragVisuals,
    camera,
    dispose() {
      observer.disconnect();
      clear();
      controls.dispose();
      renderer.dispose();
      renderer.domElement.remove();
    }
  };
}
