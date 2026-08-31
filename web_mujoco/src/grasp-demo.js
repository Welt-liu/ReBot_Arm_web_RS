import { ARM_JOINTS, homePose } from './kinematics.js';

export const VISION_TARGETS = [
  { id: 'red', body: 'red_cube', labelKey: 'vision.red', color: '#ef5260' },
  { id: 'blue', body: 'blue_block', labelKey: 'vision.blue', color: '#45aef2' },
  { id: 'yellow', body: 'yellow_cylinder', labelKey: 'vision.yellow', color: '#e6d957' }
];

export const STORAGE_ZONES = {
  red: { x: 0.49, y: -0.16, z: 0.1225 },
  blue: { x: 0.49, y: 0.00, z: 0.119 },
  yellow: { x: 0.49, y: 0.16, z: 0.126 }
};

export const STACK_TARGETS = {
  blue: { x: 0.32, y: 0.18, z: 0.119 },
  red: { x: 0.32, y: 0.18, z: 0.1605 },
  yellow: { x: 0.32, y: 0.18, z: 0.209 }
};

const OPEN_WIDTH = 0.046;
const MOVE_TIMEOUT = 4.5;
const STAGE_PROGRESS = {
  idle: 0,
  homing: 0.02,
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
  const fingerBodies = new Set([
    mujoco.mj_name2id(model, bodyType, 'gripper_left'),
    mujoco.mj_name2id(model, bodyType, 'gripper_right')
  ]);
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
  let mode = 'put-away';
  let stackQueue = [];
  let pendingStackId = null;
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
      mode,
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

  function begin(nextId, nextDropTarget, nextMode = 'put-away') {
    selectedId = nextId;
    mode = nextMode;
    objectStart = bodyPosition(selected().bodyId);
    dropTarget = nextDropTarget;
    ikAngles = Object.fromEntries(
      ARM_JOINTS.map((joint) => [joint.name, data.qpos[joints.byName[joint.name].qposadr]])
    );
    message = '';
    lastUpdateAt = data.time;
    physics.setTarget('joint7', OPEN_WIDTH);
    enter('opening');
  }

  function start(next = selectedId) {
    if (running) return false;
    setSelected(next);
    running = true;
    stackQueue = [];
    begin(next, STORAGE_ZONES[next]);
    return true;
  }

  function startStack() {
    if (running) return false;
    running = true;
    stackQueue = ['red', 'yellow'];
    begin('blue', STACK_TARGETS.blue, 'stack');
    return true;
  }

  function cancel(reason = 'cancelled') {
    if (!running && stage === 'idle') return false;
    running = false;
    stage = 'idle';
    message = reason;
    lastError = 0;
    moveTarget = null;
    stackQueue = [];
    mode = 'put-away';
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

  function objectHasSupportContact() {
    const selectedBodyId = selected().bodyId;
    const count = Math.min(data.ncon, data.contact.size());
    for (let index = 0; index < count; index += 1) {
      const contact = data.contact.get(index);
      if (!contact) continue;
      const body1 = model.geom_bodyid[contact.geom1];
      const body2 = model.geom_bodyid[contact.geom2];
      const touchesSelected = body1 === selectedBodyId || body2 === selectedBodyId;
      const otherIsFinger = fingerBodies.has(body1) || fingerBodies.has(body2);
      if (touchesSelected && !otherIsFinger) return true;
    }
    return false;
  }

  function objectHasGripContact() {
    const selectedBodyId = selected().bodyId;
    const count = Math.min(data.ncon, data.contact.size());
    for (let index = 0; index < count; index += 1) {
      const contact = data.contact.get(index);
      if (!contact) continue;
      const body1 = model.geom_bodyid[contact.geom1];
      const body2 = model.geom_bodyid[contact.geom2];
      const touchesSelected = body1 === selectedBodyId || body2 === selectedBodyId;
      if (touchesSelected && (fingerBodies.has(body1) || fingerBodies.has(body2))) return true;
    }
    return false;
  }

  function objectIsAtDropTarget() {
    const object = bodyPosition(selected().bodyId);
    const horizontalError = Math.hypot(object.x - dropTarget.x, object.y - dropTarget.y);
    const verticalError = object.z - dropTarget.z;
    return horizontalError < 0.035 && verticalError > -0.025 && verticalError < 0.045;
  }

  function update() {
    if (!running) return snapshot();
    const now = data.time;
    const dt = Math.max(0.001, now - lastUpdateAt);
    lastUpdateAt = now;
    const elapsed = now - stageStartedAt;

    if (stage === 'homing') {
      if (elapsed > 0.35) begin(pendingStackId, STACK_TARGETS[pendingStackId], 'stack');
    } else if (stage === 'opening') {
      const width = data.qpos[joints.byName.joint7.qposadr];
      if ((width > OPEN_WIDTH - 0.006 && elapsed > 0.18) || elapsed > 1.1) {
        enter('approach', { x: objectStart.x, y: objectStart.y, z: objectStart.z + 0.10 });
      }
    } else if (stage === 'approach') {
      if (moveStep(dt)) enter('descend', { x: objectStart.x, y: objectStart.y, z: objectStart.z + 0.003 });
      else if (timedOut()) fail('approach-timeout');
    } else if (stage === 'descend') {
      const gripReady =
        elapsed > 0.15 &&
        distance(ik.tcpPosition(), moveTarget) < 0.018 &&
        objectHasGripContact();
      if (moveStep(dt) || gripReady) {
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
      const supported = elapsed > 0.15 && objectIsAtDropTarget() && objectHasSupportContact();
      if (moveStep(dt) || supported) {
        physics.setTarget('joint7', OPEN_WIDTH);
        enter('release');
      } else if (timedOut()) fail('place-timeout');
    } else if (stage === 'release') {
      if (elapsed > 0.65) enter('retreat', { x: dropTarget.x, y: dropTarget.y, z: dropTarget.z + 0.11 });
    } else if (stage === 'retreat') {
      if (moveStep(dt)) {
        if (mode === 'stack' && stackQueue.length > 0) {
          const nextId = stackQueue.shift();
          pendingStackId = nextId;
          stage = 'homing';
          stageStartedAt = data.time;
          moveTarget = null;
          physics.setTargets(homePose(joints));
          physics.setTarget('joint7', OPEN_WIDTH);
          notify();
        } else {
          running = false;
          message = mode === 'stack' ? 'stack-complete' : 'complete';
          enter('complete');
        }
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
    startStack,
    cancel,
    update,
    reset,
    state: snapshot,
    isRunning() {
      return running;
    }
  };
}
