import { ARM_JOINTS } from './kinematics.js';

export const VISION_TARGETS = [
  { id: 'red', body: 'red_cube', labelKey: 'vision.red', color: '#ef5260' },
  { id: 'blue', body: 'blue_block', labelKey: 'vision.blue', color: '#45aef2' },
  { id: 'yellow', body: 'yellow_cylinder', labelKey: 'vision.yellow', color: '#e6d957' }
];

const OPEN_WIDTH = 0.046;
const MOVE_TIMEOUT = 4.5;
const STAGE_PROGRESS = {
  idle: 0,
  opening: 0.08,
  approach: 0.22,
  descend: 0.38,
  closing: 0.50,
  lift: 0.64,
  transfer: 0.76,
  place: 0.86,
  release: 0.94,
  retreat: 0.98,
  complete: 1,
  failed: 1
};

function distance(a, b) {
  return Math.hypot(a.x - b.x, a.y - b.y, a.z - b.z);
}

export function createGraspDemo({ mujoco, model, data, joints, physics, ik, onChange }) {
  const bodyType = mujoco.mjtObj.mjOBJ_BODY.value;
  const targets = VISION_TARGETS.map((target) => ({
    ...target,
    bodyId: mujoco.mj_name2id(model, bodyType, target.body)
  }));
  if (targets.some((target) => target.bodyId < 0)) {
    throw new Error('抓取演示缺少颜色目标 body');
  }

  let selectedId = targets[0].id;
  let running = false;
  let stage = 'idle';
  let stageStartedAt = 0;
  let lastUpdateAt = 0;
  let ikAngles = {};
  let objectStart = null;
  let moveTarget = null;
  let dropTarget = null;
  let lastError = 0;
  let message = '';

  function selected() {
    return targets.find((target) => target.id === selectedId) || targets[0];
  }

  function bodyPosition(bodyId) {
    return {
      x: data.xpos[bodyId * 3],
      y: data.xpos[bodyId * 3 + 1],
      z: data.xpos[bodyId * 3 + 2]
    };
  }

  function snapshot() {
    return {
      running,
      stage,
      progress: STAGE_PROGRESS[stage] ?? 0,
      selectedId,
      error: lastError,
      message,
      objectPosition: bodyPosition(selected().bodyId),
      objectStart
    };
  }

  function notify() {
    onChange?.(snapshot());
  }

  function enter(next, target = null) {
    stage = next;
    stageStartedAt = data.time;
    moveTarget = target;
    lastError = target ? distance(ik.tcpPosition(), target) : 0;
    notify();
  }

  function setSelected(next) {
    if (running || !targets.some((target) => target.id === next)) return false;
    selectedId = next;
    notify();
    return true;
  }

  function start(next = selectedId) {
    if (running) return false;
    setSelected(next);
    const target = selected();
    objectStart = bodyPosition(target.bodyId);
    dropTarget = {
      x: 0.445,
      y: objectStart.y >= 0 ? -0.125 : 0.125,
      z: objectStart.z
    };
    ikAngles = Object.fromEntries(
      ARM_JOINTS.map((joint) => [joint.name, data.qpos[joints.byName[joint.name].qposadr]])
    );
    running = true;
    message = '';
    lastUpdateAt = data.time;
    physics.setTarget('joint7', OPEN_WIDTH);
    enter('opening');
    return true;
  }

  function cancel(reason = 'cancelled') {
    if (!running && stage === 'idle') return false;
    running = false;
    stage = 'idle';
    message = reason;
    lastError = 0;
    moveTarget = null;
    notify();
    return true;
  }

  function fail(reason) {
    running = false;
    message = reason;
    physics.setTarget('joint7', OPEN_WIDTH);
    enter('failed');
  }

  function moveStep(dt) {
    const result = ik.servoStep(moveTarget, Math.max(0.001, Math.min(0.035, dt)), ikAngles);
    if (!result) return false;
    ikAngles = result.angles;
    physics.setTargets(ikAngles);
    lastError = distance(ik.tcpPosition(), moveTarget);
    return lastError < 0.006;
  }

  function timedOut() {
    return data.time - stageStartedAt > MOVE_TIMEOUT;
  }

  function update() {
    if (!running) return snapshot();
    const now = data.time;
    const dt = Math.max(0.001, now - lastUpdateAt);
    lastUpdateAt = now;
    const elapsed = now - stageStartedAt;

    if (stage === 'opening') {
      const width = data.qpos[joints.byName.joint7.qposadr];
      if ((width > OPEN_WIDTH - 0.006 && elapsed > 0.18) || elapsed > 1.1) {
        enter('approach', { x: objectStart.x, y: objectStart.y, z: objectStart.z + 0.10 });
      }
    } else if (stage === 'approach') {
      if (moveStep(dt)) enter('descend', { x: objectStart.x, y: objectStart.y, z: objectStart.z + 0.003 });
      else if (timedOut()) fail('approach-timeout');
    } else if (stage === 'descend') {
      if (moveStep(dt)) {
        physics.setTarget('joint7', 0);
        enter('closing');
      } else if (timedOut()) fail('descend-timeout');
    } else if (stage === 'closing') {
      if (elapsed > 0.8) enter('lift', { x: objectStart.x, y: objectStart.y, z: objectStart.z + 0.13 });
    } else if (stage === 'lift') {
      if (moveStep(dt)) {
        const object = bodyPosition(selected().bodyId);
        if (object.z < objectStart.z + 0.035) fail('grasp-missed');
        else enter('transfer', { x: dropTarget.x, y: dropTarget.y, z: objectStart.z + 0.13 });
      } else if (timedOut()) fail('lift-timeout');
    } else if (stage === 'transfer') {
      if (moveStep(dt)) enter('place', { x: dropTarget.x, y: dropTarget.y, z: dropTarget.z + 0.006 });
      else if (timedOut()) fail('transfer-timeout');
    } else if (stage === 'place') {
      if (moveStep(dt)) {
        physics.setTarget('joint7', OPEN_WIDTH);
        enter('release');
      } else if (timedOut()) fail('place-timeout');
    } else if (stage === 'release') {
      if (elapsed > 0.65) enter('retreat', { x: dropTarget.x, y: dropTarget.y, z: dropTarget.z + 0.11 });
    } else if (stage === 'retreat') {
      if (moveStep(dt)) {
        running = false;
        message = 'complete';
        enter('complete');
      } else if (timedOut()) fail('retreat-timeout');
    }
    notify();
    return snapshot();
  }

  function reset() {
    running = false;
    stage = 'idle';
    stageStartedAt = data.time;
    lastUpdateAt = data.time;
    objectStart = null;
    moveTarget = null;
    dropTarget = null;
    lastError = 0;
    message = '';
    notify();
  }

  return {
    targets: targets.map(({ bodyId, ...target }) => ({ ...target })),
    setSelected,
    start,
    cancel,
    update,
    reset,
    state: snapshot,
    isRunning() {
      return running;
    }
  };
}
