import * as THREE from 'three';
import { ARM_JOINTS } from './kinematics.js';
import { t, onLangChange } from './i18n.js';

const EPS = 0.004;
const GAIN = 12;
const DAMPING = 0.04;
const MAX_JOINT_SPEED = 2.8;
const REACH_ANGLE = 0.012;
const WRIST_NAMES = ['joint4', 'joint5', 'joint6'];
const RAD = 180 / Math.PI;

function clip(value, min, max) {
  return Math.min(max, Math.max(min, value));
}

function copyQpos(data) {
  const values = new Float64Array(data.qpos.length);
  for (let i = 0; i < data.qpos.length; i += 1) values[i] = data.qpos[i];
  return values;
}

function restoreQpos(data, values) {
  for (let i = 0; i < values.length; i += 1) data.qpos[i] = values[i];
}

function rotationVector(fromMat4, toMat4) {
  const qFrom = new THREE.Quaternion().setFromRotationMatrix(fromMat4);
  const qTo = new THREE.Quaternion().setFromRotationMatrix(toMat4);
  const qErr = qTo.clone().multiply(qFrom.clone().conjugate());
  if (qErr.w < 0) qErr.set(-qErr.x, -qErr.y, -qErr.z, -qErr.w);
  const angle = 2 * Math.acos(clip(qErr.w, -1, 1));
  const sine = Math.sqrt(Math.max(0, 1 - qErr.w * qErr.w));
  if (sine < 1e-8 || angle < 1e-8) return new THREE.Vector3();
  return new THREE.Vector3(qErr.x, qErr.y, qErr.z).multiplyScalar(angle / sine);
}

function solve3x3(a, b) {
  const det =
    a[0][0] * (a[1][1] * a[2][2] - a[1][2] * a[2][1]) -
    a[0][1] * (a[1][0] * a[2][2] - a[1][2] * a[2][0]) +
    a[0][2] * (a[1][0] * a[2][1] - a[1][1] * a[2][0]);
  if (Math.abs(det) < 1e-9) return null;
  const inv = [
    [
      (a[1][1] * a[2][2] - a[1][2] * a[2][1]) / det,
      (a[0][2] * a[2][1] - a[0][1] * a[2][2]) / det,
      (a[0][1] * a[1][2] - a[0][2] * a[1][1]) / det
    ],
    [
      (a[1][2] * a[2][0] - a[1][0] * a[2][2]) / det,
      (a[0][0] * a[2][2] - a[0][2] * a[2][0]) / det,
      (a[0][2] * a[1][0] - a[0][0] * a[1][2]) / det
    ],
    [
      (a[1][0] * a[2][1] - a[1][1] * a[2][0]) / det,
      (a[0][1] * a[2][0] - a[0][0] * a[2][1]) / det,
      (a[0][0] * a[1][1] - a[0][1] * a[1][0]) / det
    ]
  ];
  return [
    inv[0][0] * b[0] + inv[0][1] * b[1] + inv[0][2] * b[2],
    inv[1][0] * b[0] + inv[1][1] * b[1] + inv[1][2] * b[2],
    inv[2][0] * b[0] + inv[2][1] * b[1] + inv[2][2] * b[2]
  ];
}

function dls(jacobian, error) {
  const gram = [
    [
      jacobian[0][0] * jacobian[0][0] + jacobian[0][1] * jacobian[0][1] + jacobian[0][2] * jacobian[0][2],
      jacobian[0][0] * jacobian[1][0] + jacobian[0][1] * jacobian[1][1] + jacobian[0][2] * jacobian[1][2],
      jacobian[0][0] * jacobian[2][0] + jacobian[0][1] * jacobian[2][1] + jacobian[0][2] * jacobian[2][2]
    ],
    [0, 0, 0],
    [0, 0, 0]
  ];
  gram[1][0] = gram[0][1];
  gram[2][0] = gram[0][2];
  gram[1][1] = jacobian[1][0] * jacobian[1][0] + jacobian[1][1] * jacobian[1][1] + jacobian[1][2] * jacobian[1][2];
  gram[1][2] = jacobian[1][0] * jacobian[2][0] + jacobian[1][1] * jacobian[2][1] + jacobian[1][2] * jacobian[2][2];
  gram[2][1] = gram[1][2];
  gram[2][2] = jacobian[2][0] * jacobian[2][0] + jacobian[2][1] * jacobian[2][1] + jacobian[2][2] * jacobian[2][2];
  const lambda2 = DAMPING * DAMPING;
  const y = solve3x3(
    [
      [gram[0][0] + lambda2, gram[0][1], gram[0][2]],
      [gram[1][0], gram[1][1] + lambda2, gram[1][2]],
      [gram[2][0], gram[2][1], gram[2][2] + lambda2]
    ],
    [error.x, error.y, error.z]
  );
  if (!y) return null;
  return [
    jacobian[0][0] * y[0] + jacobian[1][0] * y[1] + jacobian[2][0] * y[2],
    jacobian[0][1] * y[0] + jacobian[1][1] * y[1] + jacobian[2][1] * y[2],
    jacobian[0][2] * y[0] + jacobian[1][2] * y[1] + jacobian[2][2] * y[2]
  ];
}

function makeAxis(color) {
  const mat = new THREE.MeshBasicMaterial({ color, depthTest: true });
  const shaft = new THREE.Mesh(new THREE.CylinderGeometry(0.0035, 0.0035, 0.07, 8), mat);
  shaft.rotation.x = Math.PI / 2;
  shaft.position.z = 0.035;
  const head = new THREE.Mesh(new THREE.ConeGeometry(0.008, 0.018, 8), mat);
  head.rotation.x = Math.PI / 2;
  head.position.z = 0.078;
  const group = new THREE.Group();
  group.rotation.y = Math.PI / 2;
  group.add(shaft, head);
  return group;
}

function createLeaderRig() {
  const root = new THREE.Group();
  root.name = 'leader-rig';
  root.position.set(0.08, -0.36, 0);

  const base = new THREE.Mesh(
    new THREE.CylinderGeometry(0.045, 0.05, 0.02, 24),
    new THREE.MeshStandardMaterial({ color: 0x1b3a44, metalness: 0.4, roughness: 0.45 })
  );
  base.rotation.x = Math.PI / 2;
  base.position.z = 0.01;
  root.add(base);

  const yaw = new THREE.Group();
  yaw.position.z = 0.02;
  root.add(yaw);

  const column = new THREE.Mesh(
    new THREE.CylinderGeometry(0.018, 0.022, 0.08, 16),
    new THREE.MeshStandardMaterial({ color: 0x5ee0ff, metalness: 0.35, roughness: 0.4 })
  );
  column.rotation.x = Math.PI / 2;
  column.position.z = 0.04;
  yaw.add(column);

  const shoulder = new THREE.Group();
  shoulder.position.set(0.02, 0, 0.08);
  yaw.add(shoulder);

  const upper = new THREE.Mesh(
    new THREE.CapsuleGeometry(0.014, 0.2, 6, 12),
    new THREE.MeshStandardMaterial({ color: 0x8fd7c1, metalness: 0.2, roughness: 0.45 })
  );
  upper.rotation.z = Math.PI / 2;
  upper.position.x = 0.118;
  shoulder.add(upper);

  const elbow = new THREE.Group();
  elbow.position.x = 0.236;
  shoulder.add(elbow);

  const forearm = new THREE.Mesh(
    new THREE.CapsuleGeometry(0.012, 0.2, 6, 12),
    new THREE.MeshStandardMaterial({ color: 0xf5d76e, metalness: 0.2, roughness: 0.45 })
  );
  forearm.rotation.z = Math.PI / 2;
  forearm.position.x = -0.118;
  elbow.add(forearm);

  const tip = new THREE.Group();
  tip.position.x = -0.236;
  elbow.add(tip);

  const forearmAxis = makeAxis(0x5ee0ff);
  tip.add(forearmAxis);

  const imu = new THREE.Group();
  const handle = new THREE.Mesh(
    new THREE.BoxGeometry(0.04, 0.028, 0.018),
    new THREE.MeshStandardMaterial({ color: 0xc4f25a, metalness: 0.15, roughness: 0.4, emissive: 0x234010, emissiveIntensity: 0.35 })
  );
  imu.add(handle, makeAxis(0xc4f25a));
  root.add(imu);

  return { root, yaw, shoulder, elbow, tip, imu };
}

export function createLeaderTeleop({
  mujoco,
  model,
  data,
  joints,
  physics,
  panel,
  view,
  toggleEl,
  holdEl,
  followEl,
  readoutEl,
  sliders,
  onStatus,
  onEnabledChange
}) {
  const bodyType = mujoco.mjtObj.mjOBJ_BODY.value;
  const forearmId = mujoco.mj_name2id(model, bodyType, 'link3');
  let tcpId = mujoco.mj_name2id(model, bodyType, 'gripper_end');
  if (tcpId < 0) tcpId = mujoco.mj_name2id(model, bodyType, 'link6');
  if (forearmId < 0 || tcpId < 0) {
    throw new Error('找不到 link3 / gripper_end，无法结算腕部耦合');
  }

  const arm = ARM_JOINTS.map((joint) => ({
    ...joint,
    qposadr: joints.byName[joint.name].qposadr
  }));
  const wrist = WRIST_NAMES.map((name) => arm.find((joint) => joint.name === name));

  const rig = createLeaderRig();
  view.attachHelper(rig.root);
  rig.root.visible = false;

  const followerImuAxis = makeAxis(0xc4f25a);
  const followerForearmAxis = makeAxis(0x5ee0ff);
  view.attachHelper(followerImuAxis);
  view.attachHelper(followerForearmAxis);
  followerImuAxis.visible = false;
  followerForearmAxis.visible = false;

  const imuEuler = new THREE.Euler(0, 0, 0, 'ZYX');
  const imuMatrix = new THREE.Matrix4();
  const imuNowQ = new THREE.Quaternion();
  const imuCmdQ = new THREE.Quaternion();
  let imuRefQ = null;
  let gripRefQ = null;
  let imuLive = false;
  const gripMatrix = new THREE.Matrix4();
  const forearmMatrix = new THREE.Matrix4();
  const worldX = new THREE.Vector3();
  let enabled = false;
  let lastTime = 0;
  const wristAngles = { joint4: 0, joint5: 0, joint6: 0 };

  function bodyMatrix(bodyId, target) {
    const m = data.xmat;
    const p = data.xpos;
    const i = bodyId * 9;
    const j = bodyId * 3;
    target.set(
      m[i], m[i + 1], m[i + 2], p[j],
      m[i + 3], m[i + 4], m[i + 5], p[j + 1],
      m[i + 6], m[i + 7], m[i + 8], p[j + 2],
      0, 0, 0, 1
    );
    return target;
  }

  function applyArm(angles) {
    arm.forEach((joint) => {
      data.qpos[joint.qposadr] = clip(angles[joint.name], joint.min, joint.max);
    });
    mujoco.mj_forward(model, data);
  }

  function readEncoders() {
    return {
      joint1: Number(sliders.joint1.value),
      joint2: Number(sliders.joint2.value),
      joint3: Number(sliders.joint3.value)
    };
  }

  function readImu() {
    imuEuler.set(
      Number(sliders.roll.value),
      Number(sliders.pitch.value),
      Number(sliders.yaw.value),
      'ZYX'
    );
    imuMatrix.makeRotationFromEuler(imuEuler);
    return imuMatrix;
  }

  function captureWrist() {
    WRIST_NAMES.forEach((name) => {
      const joint = joints.byName[name];
      wristAngles[name] = physics.targets[name] ?? data.qpos[joint.qposadr];
    });
  }

  function applyImuDegrees(sample) {
    if (!enabled) setEnabled(true);
    const roll = -Number(sample.roll) * Math.PI / 180;
    const pitch = Number(sample.pitch) * Math.PI / 180;
    const yaw = Number(sample.yaw) * Math.PI / 180;
    if (![roll, pitch, yaw].every(Number.isFinite)) return;
    imuNowQ.setFromEuler(new THREE.Euler(roll, pitch, yaw, 'ZYX'));
    if (!imuRefQ) {
      imuRefQ = imuNowQ.clone();
      gripRefQ = new THREE.Quaternion().setFromEuler(imuEuler);
    }
    imuCmdQ.copy(gripRefQ).multiply(imuRefQ.clone().conjugate()).multiply(imuNowQ);
    imuEuler.setFromQuaternion(imuCmdQ, 'ZYX');
    imuMatrix.makeRotationFromEuler(imuEuler);
    sliders.roll.value = String(imuEuler.x);
    sliders.pitch.value = String(imuEuler.y);
    sliders.yaw.value = String(imuEuler.z);
    imuLive = true;
  }

  function zeroImu() {
    imuRefQ = null;
    gripRefQ = null;
  }

  function writeImuFromMatrix(matrix) {
    const euler = new THREE.Euler().setFromRotationMatrix(matrix, 'ZYX');
    sliders.roll.value = String(euler.x);
    sliders.pitch.value = String(euler.y);
    sliders.yaw.value = String(euler.z);
    imuEuler.copy(euler);
    imuMatrix.makeRotationFromEuler(imuEuler);
  }

  function syncFromFollower() {
    ['joint1', 'joint2', 'joint3'].forEach((name) => {
      const joint = joints.byName[name];
      const value = physics.targets[name] ?? data.qpos[joint.qposadr];
      sliders[name].value = String(clip(value, joint.min, joint.max));
    });
    captureWrist();
    const saved = copyQpos(data);
    applyArm(poseForIk());
    bodyMatrix(tcpId, gripMatrix);
    writeImuFromMatrix(gripMatrix);
    restoreQpos(data, saved);
    mujoco.mj_forward(model, data);
    updateLeaderVisual();
  }

  function poseForIk() {
    const encoders = readEncoders();
    return {
      joint1: encoders.joint1,
      joint2: encoders.joint2,
      joint3: encoders.joint3,
      ...wristAngles
    };
  }

  const plusMatrix = new THREE.Matrix4();
  const minusMatrix = new THREE.Matrix4();
  const baseMatrix = new THREE.Matrix4();

  function servoWrist(dt) {
    const saved = copyQpos(data);
    const pose = poseForIk();
    const desired = readImu();
    applyArm(pose);
    bodyMatrix(tcpId, gripMatrix);
    const error = rotationVector(gripMatrix, desired);
    const errorNorm = error.length();
    if (errorNorm < REACH_ANGLE) {
      restoreQpos(data, saved);
      mujoco.mj_forward(model, data);
      return { error: errorNorm, reached: true };
    }

    const scale = Math.min(0.65, Math.max(0.08, GAIN * dt));
    const step = error.clone().multiplyScalar(scale);
    bodyMatrix(tcpId, baseMatrix);
    const jacobian = [[], [], []];
    wrist.forEach((joint) => {
      applyArm({ ...pose, [joint.name]: pose[joint.name] + EPS });
      bodyMatrix(tcpId, plusMatrix);
      const dPlus = rotationVector(baseMatrix, plusMatrix);
      applyArm({ ...pose, [joint.name]: pose[joint.name] - EPS });
      bodyMatrix(tcpId, minusMatrix);
      const dMinus = rotationVector(baseMatrix, minusMatrix);
      jacobian[0].push((dPlus.x - dMinus.x) / (2 * EPS));
      jacobian[1].push((dPlus.y - dMinus.y) / (2 * EPS));
      jacobian[2].push((dPlus.z - dMinus.z) / (2 * EPS));
    });
    const delta = dls(jacobian, step);
    if (delta) {
      wrist.forEach((joint, index) => {
        const limited = clip(delta[index] || 0, -MAX_JOINT_SPEED * dt, MAX_JOINT_SPEED * dt);
        wristAngles[joint.name] = clip(wristAngles[joint.name] + limited, joint.min, joint.max);
      });
    }
    restoreQpos(data, saved);
    mujoco.mj_forward(model, data);
    return { error: errorNorm, reached: false };
  }

  function syncFollowerTargets() {
    const encoders = readEncoders();
    ['joint1', 'joint2', 'joint3'].forEach((name) => {
      physics.setTarget(name, encoders[name]);
      panel.setTarget(name, encoders[name]);
    });
    WRIST_NAMES.forEach((name) => {
      physics.setTarget(name, wristAngles[name]);
      panel.setTarget(name, wristAngles[name]);
    });
  }

  function updateLeaderVisual() {
    const encoders = readEncoders();
    rig.yaw.rotation.z = -encoders.joint1 + Math.PI;
    rig.shoulder.rotation.y = -encoders.joint2;
    rig.elbow.rotation.y = encoders.joint3;
    rig.tip.updateWorldMatrix(true, false);
    const tipPos = new THREE.Vector3();
    rig.tip.getWorldPosition(tipPos);
    rig.root.worldToLocal(tipPos);
    rig.imu.position.copy(tipPos);
    rig.imu.quaternion.setFromEuler(imuEuler);
  }

  function updateFollowerFrames() {
    bodyMatrix(forearmId, forearmMatrix);
    followerForearmAxis.matrixAutoUpdate = false;
    followerForearmAxis.matrix.copy(forearmMatrix);
    followerForearmAxis.matrixWorldNeedsUpdate = true;
    followerImuAxis.matrixAutoUpdate = false;
    const tcpPos = data.xpos;
    followerImuAxis.matrix.copy(readImu());
    followerImuAxis.matrix.setPosition(
      tcpPos[tcpId * 3],
      tcpPos[tcpId * 3 + 1],
      tcpPos[tcpId * 3 + 2]
    );
    followerImuAxis.matrixWorldNeedsUpdate = true;
  }

  function formatDeg(rad) {
    return `${(rad * RAD).toFixed(0)}°`;
  }

  function renderReadout(error) {
    if (!readoutEl) return;
    const encoders = readEncoders();
    worldX.set(1, 0, 0).applyMatrix4(readImu()).setLength(1);
    const imuYaw = Math.atan2(worldX.y, worldX.x);
    bodyMatrix(forearmId, forearmMatrix);
    const fx = new THREE.Vector3(1, 0, 0).transformDirection(forearmMatrix);
    const forearmYaw = Math.atan2(fx.y, fx.x);
    readoutEl.innerHTML = t('leader.readout', {
      q4: formatDeg(wristAngles.joint4),
      q5: formatDeg(wristAngles.joint5),
      q6: formatDeg(wristAngles.joint6),
      err: ((error || 0) * RAD).toFixed(1),
      rel: formatDeg(imuYaw - forearmYaw)
    });
  }

  function setSliderDegrees(name, degValue) {
    const joint = joints.byName[name];
    sliders[name].value = String(clip((degValue * Math.PI) / 180, joint.min, joint.max));
  }

  function applyLang() {
    if (toggleEl) {
      toggleEl.textContent = t(enabled ? 'leader.off' : 'leader.on');
      toggleEl.classList.toggle('active', enabled);
    }
    if (holdEl) holdEl.textContent = t('leader.holdImu');
    if (followEl) followEl.textContent = t('leader.followArm');
    if (enabled) renderReadout();
  }

  function setEnabled(next) {
    enabled = next;
    rig.root.visible = enabled;
    followerImuAxis.visible = enabled;
    followerForearmAxis.visible = enabled;
    applyLang();
    if (enabled) {
      syncFromFollower();
      zeroImu();
      lastTime = performance.now();
      onStatus(t('status.leaderOn'));
    } else {
      zeroImu();
      imuLive = false;
      onStatus(t('status.leaderOff'));
    }
    onEnabledChange?.(enabled);
  }

  function update(now) {
    if (!enabled) return;
    const dt = Math.min(0.05, Math.max(0.012, (now - lastTime) / 1000 || 0.016));
    lastTime = now;
    const substeps = Math.max(1, Math.ceil(dt / 0.016));
    let result = null;
    for (let i = 0; i < substeps; i += 1) result = servoWrist(dt / substeps);
    syncFollowerTargets();
    updateLeaderVisual();
    updateFollowerFrames();
    renderReadout(result?.error);
  }

  Object.values(sliders).forEach((input) => {
    input.addEventListener('input', () => {
      if (!enabled) return;
      lastTime = performance.now();
    });
  });

  toggleEl?.addEventListener('click', () => setEnabled(!enabled));
  holdEl?.addEventListener('click', () => {
    setSliderDegrees('joint1', 90);
    setSliderDegrees('joint2', 20);
    setSliderDegrees('joint3', 40);
    sliders.yaw.value = '0';
    sliders.pitch.value = '0';
    sliders.roll.value = '0';
    if (!enabled) setEnabled(true);
    onStatus(t('status.leaderHold'));
  });
  followEl?.addEventListener('click', () => {
    setSliderDegrees('joint1', 90);
    setSliderDegrees('joint2', 20);
    setSliderDegrees('joint3', 40);
    const saved = copyQpos(data);
    applyArm({ ...readEncoders(), joint4: 0, joint5: 0, joint6: 0 });
    bodyMatrix(tcpId, gripMatrix);
    const euler = new THREE.Euler().setFromRotationMatrix(gripMatrix, 'ZYX');
    sliders.roll.value = String(euler.x);
    sliders.pitch.value = String(euler.y);
    sliders.yaw.value = String(euler.z);
    restoreQpos(data, saved);
    mujoco.mj_forward(model, data);
    if (!enabled) setEnabled(true);
    onStatus(t('status.leaderFollow'));
  });
  onLangChange(applyLang);
  applyLang();
  syncFromFollower();

  return {
    update,
    setEnabled,
    syncFromFollower,
    applyImuDegrees,
    zeroImu,
    applyLang,
    isEnabled() {
      return enabled;
    }
  };
}
