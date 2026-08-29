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
    const geometry = new THREE.CapsuleGeometry(size[0], 2 * size[1], 8, 16);
    geometry.rotateX(Math.PI / 2);
    return geometry;
  }
  if (type === types.ellipsoid) {
    const geometry = new THREE.SphereGeometry(1, 24, 16);
    geometry.scale(size[0], size[1], size[2]);
    return geometry;
  }
  if (type === types.cylinder) {
    const geometry = new THREE.CylinderGeometry(size[0], size[0], 2 * size[1], 24);
    geometry.rotateX(Math.PI / 2);
    return geometry;
  }
  if (type === types.box) {
    return new THREE.BoxGeometry(2 * size[0], 2 * size[1], 2 * size[2]);
  }
  return null;
}

function geomMaterial(model, index, materialProps) {
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
    const mp = materialProps && materialProps[matid];
    if (mp && (mp.metallic != null || mp.roughness != null)) {
      if (mp.metallic != null) metalness = mp.metallic;
      if (mp.roughness != null) roughness = mp.roughness;
    } else {
      if (model.mat_metallic) {
        metalness = model.mat_metallic[matid];
      } else if (model.mat_specular) {
        const spec = model.mat_specular.subarray(matid * 4, matid * 4 + 4);
        metalness = (spec[0] + spec[1] + spec[2]) / 3;
      }
      if (model.mat_roughness) {
        roughness = model.mat_roughness[matid];
      } else if (model.mat_shininess) {
        roughness = 1 - model.mat_shininess[matid];
      }
    }
  }
  const finish = materialProps && materialProps[matid];
  const isCnc = finish?.name === 'rs_anodized_silver_mat';
  const isTable = finish?.name === 'rs_table';
  const isMetal = metalness >= 0.45;
  if (isCnc) color.multiplyScalar(0.72);
  if (isTable) color.multiplyScalar(0.58);
  return new THREE.MeshPhysicalMaterial({
    color,
    metalness: Number.isFinite(metalness) ? Math.max(0, Math.min(1, metalness)) : 0.12,
    roughness: Number.isFinite(roughness) ? Math.max(0, Math.min(1, roughness)) : 0.55,
    envMapIntensity: isCnc ? 1.9 : isMetal ? 1.2 : 0.72,
    clearcoat: isCnc ? 0.25 : 0,
    clearcoatRoughness: isCnc ? 0.14 : 0.4,
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

function createGridTexture() {
  const size = 1024;
  const canvas = document.createElement('canvas');
  canvas.width = canvas.height = size;
  const ctx = canvas.getContext('2d');

  ctx.fillStyle = '#020617';
  ctx.fillRect(0, 0, size, size);

  const glow = ctx.createRadialGradient(size / 2, size / 2, 0, size / 2, size / 2, size * 0.7);
  glow.addColorStop(0, 'rgba(8, 47, 105, 0.48)');
  glow.addColorStop(0.5, 'rgba(3, 21, 56, 0.18)');
  glow.addColorStop(1, 'rgba(2, 6, 23, 0)');
  ctx.fillStyle = glow;
  ctx.fillRect(0, 0, size, size);

  const sub = 10;
  ctx.strokeStyle = 'rgba(0, 126, 255, 0.34)';
  ctx.lineWidth = 1;
  ctx.beginPath();
  for (let i = 1; i < sub; i += 1) {
    const p = (i / sub) * size;
    ctx.moveTo(p, 0); ctx.lineTo(p, size);
    ctx.moveTo(0, p); ctx.lineTo(size, p);
  }
  ctx.stroke();

  ctx.shadowColor = 'rgba(0, 214, 255, 0.9)';
  ctx.shadowBlur = 10;
  ctx.strokeStyle = 'rgba(0, 207, 255, 0.96)';
  ctx.lineWidth = 3;
  ctx.strokeRect(0, 0, size, size);
  ctx.shadowBlur = 0;

  const texture = new THREE.CanvasTexture(canvas);
  texture.wrapS = texture.wrapT = THREE.RepeatWrapping;
  texture.repeat.set(5, 5);
  texture.anisotropy = 8;
  texture.colorSpace = THREE.SRGBColorSpace;
  return texture;
}

function createBackdropTexture() {
  const canvas = document.createElement('canvas');
  canvas.width = 1600;
  canvas.height = 900;
  const ctx = canvas.getContext('2d');
  const { width, height } = canvas;

  const sky = ctx.createLinearGradient(0, 0, 0, height);
  sky.addColorStop(0, '#010319');
  sky.addColorStop(0.56, '#030629');
  sky.addColorStop(0.72, '#06154b');
  sky.addColorStop(0.76, '#03153d');
  sky.addColorStop(1, '#01030f');
  ctx.fillStyle = sky;
  ctx.fillRect(0, 0, width, height);

  const horizonY = height * 0.73;
  const horizon = ctx.createLinearGradient(0, horizonY - 60, 0, horizonY + 60);
  horizon.addColorStop(0, 'rgba(0, 87, 255, 0)');
  horizon.addColorStop(0.42, 'rgba(0, 118, 255, 0.24)');
  horizon.addColorStop(0.5, 'rgba(0, 229, 255, 0.98)');
  horizon.addColorStop(0.58, 'rgba(0, 94, 255, 0.30)');
  horizon.addColorStop(1, 'rgba(0, 30, 110, 0)');
  ctx.fillStyle = horizon;
  ctx.fillRect(0, horizonY - 60, width, 120);

  ctx.globalCompositeOperation = 'lighter';
  [0.035, 0.18, 0.50, 0.82, 0.965].forEach((ratio, index) => {
    const x = width * ratio;
    const beam = ctx.createLinearGradient(x - 22, 0, x + 22, 0);
    beam.addColorStop(0, 'rgba(0, 55, 255, 0)');
    beam.addColorStop(0.42, 'rgba(0, 84, 255, 0.18)');
    beam.addColorStop(0.5, index % 2 ? 'rgba(0, 121, 255, 0.78)' : 'rgba(0, 199, 255, 0.92)');
    beam.addColorStop(0.58, 'rgba(0, 84, 255, 0.18)');
    beam.addColorStop(1, 'rgba(0, 55, 255, 0)');
    ctx.fillStyle = beam;
    ctx.fillRect(x - 22, 0, 44, horizonY + 20);
    ctx.fillStyle = 'rgba(92, 220, 255, 0.64)';
    ctx.fillRect(x - 1, 0, 2, horizonY + 20);
  });
  ctx.globalCompositeOperation = 'source-over';

  ctx.strokeStyle = 'rgba(16, 78, 178, 0.18)';
  ctx.lineWidth = 1;
  for (let y = 120; y < horizonY; y += 8) {
    ctx.beginPath();
    ctx.moveTo(0, y);
    ctx.lineTo(width, y);
    ctx.stroke();
  }

  const vignette = ctx.createRadialGradient(width / 2, height * 0.55, height * 0.18, width / 2, height * 0.55, width * 0.62);
  vignette.addColorStop(0, 'rgba(0, 0, 0, 0)');
  vignette.addColorStop(1, 'rgba(0, 0, 14, 0.64)');
  ctx.fillStyle = vignette;
  ctx.fillRect(0, 0, width, height);

  const texture = new THREE.CanvasTexture(canvas);
  texture.colorSpace = THREE.SRGBColorSpace;
  return texture;
}

function createEnvironmentTexture() {
  const canvas = document.createElement('canvas');
  canvas.width = 1024;
  canvas.height = 512;
  const ctx = canvas.getContext('2d');
  const { width, height } = canvas;

  const base = ctx.createLinearGradient(0, 0, 0, height);
  base.addColorStop(0, '#d8f6ff');
  base.addColorStop(0.14, '#3977b8');
  base.addColorStop(0.5, '#081b45');
  base.addColorStop(0.76, '#0b3c75');
  base.addColorStop(1, '#d8f7ff');
  ctx.fillStyle = base;
  ctx.fillRect(0, 0, width, height);

  [0.08, 0.30, 0.57, 0.82].forEach((ratio, index) => {
    const x = width * ratio;
    const panel = ctx.createLinearGradient(x - 55, 0, x + 55, 0);
    panel.addColorStop(0, 'rgba(37, 99, 235, 0)');
    panel.addColorStop(0.35, 'rgba(61, 193, 255, 0.28)');
    panel.addColorStop(0.5, index % 2 ? 'rgba(235, 252, 255, 0.98)' : 'rgba(74, 222, 255, 0.92)');
    panel.addColorStop(0.65, 'rgba(61, 193, 255, 0.28)');
    panel.addColorStop(1, 'rgba(37, 99, 235, 0)');
    ctx.fillStyle = panel;
    ctx.fillRect(x - 55, 0, 110, height);
  });

  const horizon = ctx.createLinearGradient(0, height * 0.62, 0, height * 0.83);
  horizon.addColorStop(0, 'rgba(0, 41, 120, 0)');
  horizon.addColorStop(0.48, 'rgba(60, 220, 255, 0.9)');
  horizon.addColorStop(0.55, 'rgba(230, 252, 255, 0.98)');
  horizon.addColorStop(1, 'rgba(0, 41, 120, 0)');
  ctx.fillStyle = horizon;
  ctx.fillRect(0, height * 0.62, width, height * 0.21);

  const texture = new THREE.CanvasTexture(canvas);
  texture.colorSpace = THREE.SRGBColorSpace;
  texture.mapping = THREE.EquirectangularReflectionMapping;
  return texture;
}

export function createSceneView(host) {
  const scene = new THREE.Scene();
  const backdropTexture = createBackdropTexture();
  scene.background = backdropTexture;

  const camera = new THREE.PerspectiveCamera(45, 1, 0.02, 20);
  camera.up.set(0, 0, 1);
  camera.position.set(0.85, -0.95, 0.62);

  const renderer = new THREE.WebGLRenderer({ antialias: true, powerPreference: 'high-performance' });
  renderer.setPixelRatio(Math.min(window.devicePixelRatio || 1, 2));
  renderer.shadowMap.enabled = true;
  renderer.shadowMap.type = THREE.PCFSoftShadowMap;
  renderer.toneMapping = THREE.ACESFilmicToneMapping;
  renderer.toneMappingExposure = 1.02;
  if ('outputColorSpace' in renderer) renderer.outputColorSpace = THREE.SRGBColorSpace;
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

  const pmrem = new THREE.PMREMGenerator(renderer);
  const environmentSource = createEnvironmentTexture();
  const environmentTexture = pmrem.fromEquirectangular(environmentSource).texture;
  scene.environment = environmentTexture;
  environmentSource.dispose();
  pmrem.dispose();

  RectAreaLightUniformsLib.init();
  scene.add(new THREE.AmbientLight(0x6f9bd4, 0.26));
  scene.add(new THREE.HemisphereLight(0xd9f6ff, 0x021026, 0.48));

  const softbox = new THREE.RectAreaLight(0xeaf8ff, 2.0, 1.6, 1.15);
  softbox.position.set(0.28, -0.58, 1.28);
  softbox.lookAt(0.25, 0, 0.16);
  scene.add(softbox);

  const cyanRim = new THREE.RectAreaLight(0x10cfff, 2.8, 0.75, 1.05);
  cyanRim.position.set(-0.58, 0.2, 0.68);
  cyanRim.lookAt(0.23, 0, 0.18);
  scene.add(cyanRim);

  const blueRim = new THREE.RectAreaLight(0x245cff, 1.9, 0.7, 0.9);
  blueRim.position.set(0.86, 0.32, 0.62);
  blueRim.lookAt(0.25, 0, 0.18);
  scene.add(blueRim);

  const key = new THREE.DirectionalLight(0xf2fbff, 1.0);
  key.position.set(1.2, 0.8, 2.0);
  key.castShadow = true;
  key.shadow.mapSize.set(2048, 2048);
  key.shadow.camera.near = 0.1;
  key.shadow.camera.far = 6;
  key.shadow.camera.left = -1.4;
  key.shadow.camera.right = 1.4;
  key.shadow.camera.top = 1.4;
  key.shadow.camera.bottom = -1.4;
  key.shadow.bias = -0.00012;
  key.shadow.normalBias = 0.02;
  scene.add(key);

  const side = new THREE.DirectionalLight(0x8bdcff, 0.38);
  side.position.set(-1, -0.5, 0.8);
  scene.add(side);

  const rim = new THREE.DirectionalLight(0x286fff, 0.62);
  rim.position.set(-0.5, -1, 1.5);
  scene.add(rim);

  const tcpMarker = new THREE.Mesh(
    new THREE.SphereGeometry(0.024, 16, 12),
    new THREE.MeshBasicMaterial({
      color: 0x33d6b0,
      wireframe: true,
      transparent: true,
      opacity: 0.4
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
  let gridTexture = null;
  let types = geomTypes({});

  function resize() {
    const width = Math.max(1, host.clientWidth);
    const height = Math.max(1, host.clientHeight);
    camera.aspect = width / height;
    camera.updateProjectionMatrix();
    renderer.setSize(width, height, false);
  }

  function build(mujoco, model, materialProps) {
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
      let material;
      if (type === types.plane) {
        if (!gridTexture) gridTexture = createGridTexture();
        material = new THREE.MeshStandardMaterial({
          map: gridTexture,
          color: 0x41658f,
          metalness: 0.1,
          roughness: 0.82,
          envMapIntensity: 0.4
        });
      } else {
        material = geomMaterial(model, i, materialProps);
      }
      const mesh = new THREE.Mesh(geometry, material);
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
      if (gridTexture) { gridTexture.dispose(); gridTexture = null; }
      backdropTexture.dispose();
      environmentTexture.dispose();
      controls.dispose();
      renderer.dispose();
      renderer.domElement.remove();
    }
  };
}
