import { readFile } from 'node:fs/promises';
import path from 'node:path';
import { fileURLToPath } from 'node:url';
import loadMujoco from '@mujoco/mujoco';
import { ARM_JOINTS, bindJoints } from '../src/kinematics.js';
import { createPhysicsController } from '../src/pd-control.js';
import { createTcpIk } from '../src/tcp-ik.js';
import { createGraspDemo, STORAGE_ZONES, STACK_TARGETS } from '../src/grasp-demo.js';

const here = path.dirname(fileURLToPath(import.meta.url));
const modelsDir = path.resolve(here, '../../rebotarm_ros2_RS/src/rebotarm_mujoco_rs/models');

function namedId(mujoco, model, type, name) {
  const id = mujoco.mj_name2id(model, type, name);
  if (id < 0) throw new Error(`找不到 ${name}`);
  return id;
}

function bodyPos(mujoco, model, data, name) {
  const id = namedId(mujoco, model, mujoco.mjtObj.mjOBJ_BODY.value, name);
  return [data.xpos[id * 3], data.xpos[id * 3 + 1], data.xpos[id * 3 + 2]];
}

function meshExtent(model, meshId) {
  const start = model.mesh_vertadr[meshId] * 3;
  const count = model.mesh_vertnum[meshId];
  const min = [Infinity, Infinity, Infinity];
  const max = [-Infinity, -Infinity, -Infinity];
  for (let index = 0; index < count; index += 1) {
    for (let axis = 0; axis < 3; axis += 1) {
      const value = model.mesh_vert[start + index * 3 + axis];
      min[axis] = Math.min(min[axis], value);
      max[axis] = Math.max(max[axis], value);
    }
  }
  return max.map((value, axis) => value - min[axis]);
}

function geomPosition(mujoco, model, data, name) {
  const id = namedId(mujoco, model, mujoco.mjtObj.mjOBJ_GEOM.value, name);
  return Array.from(data.geom_xpos.subarray(id * 3, id * 3 + 3));
}

function assert(condition, message) {
  if (!condition) throw new Error(message);
}

async function loadScene(mujoco) {
  const vfs = new mujoco.MjVFS();
  const sceneXml = await readFile(path.join(modelsDir, 'rs_grasp_scene.xml'));
  const armXml = await readFile(path.join(modelsDir, 'rs_arm.xml'));
  vfs.addBuffer('rs_grasp_scene.xml', sceneXml);
  vfs.addBuffer('rs_arm.xml', armXml);

  const meshRe = /\sfile="([^"]+\.(?:stl|obj|msh))"/gi;
  const armText = armXml.toString('utf8');
  let match;
  while ((match = meshRe.exec(armText))) {
    const name = `meshes/${match[1]}`;
    vfs.addBuffer(name, await readFile(path.join(modelsDir, name)));
  }

  const model = mujoco.MjModel.from_xml_string(sceneXml.toString('utf8'), vfs);
  const data = new mujoco.MjData(model);
  mujoco.mj_forward(model, data);
  return { model, data, vfs };
}

async function main() {
  const mujoco = await loadMujoco();
  const { model, data, vfs } = await loadScene(mujoco);
  const joints = bindJoints(mujoco, model);
  const physics = createPhysicsController(mujoco, model, data, joints);
  const wristCameraId = namedId(
    mujoco,
    model,
    mujoco.mjtObj.mjOBJ_CAMERA.value,
    'wrist_rgb'
  );
  const wristCameraPosition = Array.from(
    data.cam_xpos.subarray(wristCameraId * 3, wristCameraId * 3 + 3)
  );
  const mountGeomId = namedId(
    mujoco,
    model,
    mujoco.mjtObj.mjOBJ_GEOM.value,
    'd405_wrist_mount'
  );
  const mountExtent = meshExtent(model, model.geom_dataid[mountGeomId]);
  const gripperBodyId = namedId(
    mujoco,
    model,
    mujoco.mjtObj.mjOBJ_BODY.value,
    'gripper_end'
  );
  assert(model.ncam >= 2, `应包含俯视和腕部相机，ncam=${model.ncam}`);
  assert(model.geom_bodyid[mountGeomId] === gripperBodyId, 'D405 支架没有安装在 gripper_end');
  assert(model.geom_group[mountGeomId] < 3, 'D405 支架被放入隐藏渲染组');
  assert(
    Math.max(...mountExtent) > 0.04 && Math.max(...mountExtent) < 0.20,
    `D405 支架尺寸异常：${mountExtent.join(',')}`
  );
  assert(
    wristCameraPosition.every(Number.isFinite),
    `腕部相机位姿无效：${wristCameraPosition.join(',')}`
  );
  assert(Number.isInteger(joints.byName.joint2.id), '关节 id 无效');
  assert(Number.isFinite(data.xanchor[joints.byName.joint2.id * 3]), 'xanchor 无法读取');

  const ik = createTcpIk(mujoco, model, data, joints);
  const tableGeomId = namedId(
    mujoco,
    model,
    mujoco.mjtObj.mjOBJ_GEOM.value,
    'task_table_geom'
  );
  assert(
    Math.abs(model.geom_size[tableGeomId * 3] - 0.25) < 0.001 &&
      Math.abs(model.geom_size[tableGeomId * 3 + 1] - 0.28) < 0.001,
    '桌面尺寸应为约 50 x 56 cm'
  );

  const activePoints = [
    ...Object.values(STORAGE_ZONES),
    ...Object.values(STACK_TARGETS)
  ];
  assert(
    activePoints.every((point) => Math.hypot(point.x, point.y) <= 0.535),
    `抓取/叠放目标超出可达包络：${JSON.stringify(activePoints)}`
  );

  const tcp0 = ik.tcpPosition();
  const tcpMatrix = Array.from(data.xmat.subarray(ik.bodyId * 9, ik.bodyId * 9 + 9));
  const ikAngles = Object.fromEntries(
    ARM_JOINTS.map((joint) => [joint.name, data.qpos[joints.byName[joint.name].qposadr]])
  );
  const ikResult = ik.servoStep(
    { x: tcp0.x + 0.03, y: tcp0.y, z: tcp0.z + 0.02 },
    0.016,
    ikAngles
  );
  assert(ikResult && Number.isFinite(ikResult.error), 'TCP IK servo 失败');

  data.ctrl[0] = 1.25;
  assert(Math.abs(data.ctrl[0] - 1.25) < 1e-9, 'data.ctrl 无法写入');

  const cubeBefore = bodyPos(mujoco, model, data, 'red_cube');
  const initialPositions = {
    red: cubeBefore,
    blue: bodyPos(mujoco, model, data, 'blue_block'),
    yellow: bodyPos(mujoco, model, data, 'yellow_cylinder')
  };
  Object.entries(STORAGE_ZONES).forEach(([id, zone]) => {
    const initial = initialPositions[id];
    assert(
      Math.hypot(initial[0] - zone.x, initial[1] - zone.y) > 0.15,
      `${id} 初始位置距离收纳区过近：${initial.join(',')} vs ${zone.x},${zone.y}`
    );
  });
  physics.step(200);
  const cubeSettled = bodyPos(mujoco, model, data, 'red_cube');
  assert(data.ncon > 0, `色块落地后应有接触，ncon=${data.ncon}`);
  console.log('memory exports', Object.keys(mujoco).filter((key) => /malloc|free|heap|memory/i.test(key)));
  const contactForceBuffer = new mujoco.DoubleBuffer(6);
  const contactForce = contactForceBuffer.GetView();
  let maxContactForce = 0;
  for (let index = 0; index < data.ncon; index += 1) {
    contactForce.fill(0);
    mujoco.mj_contactForce(model, data, index, contactForceBuffer);
    assert(contactForce.every(Number.isFinite), `接触力读取失败：${Array.from(contactForce).join(',')}`);
    maxContactForce = Math.max(maxContactForce, Math.abs(contactForce[0]));
  }
  contactForceBuffer.delete();
  assert(maxContactForce > 0.01, `接触力应为正值：${maxContactForce}`);
  assert(
    cubeSettled[2] > 0.09 && cubeSettled[2] < 0.16,
    `红块应停在桌面上，z=${cubeSettled[2]}`
  );

  const qposadr = joints.byName.joint2.qposadr;
  physics.setTarget('joint2', 0.3);
  physics.step(1);
  const afterOne = data.qpos[qposadr];
  assert(Math.abs(afterOne - 0.3) > 0.15, `joint2 不应瞬移，一步后 q=${afterOne}`);

  physics.step(500);
  const afterHold = data.qpos[qposadr];
  assert(Math.abs(afterHold - 0.3) < 0.05, `joint2 应变到目标附近，q=${afterHold}`);
  const wristCameraAfterJointMotion = Array.from(
    data.cam_xpos.subarray(wristCameraId * 3, wristCameraId * 3 + 3)
  );
  const wristCameraTravel = Math.hypot(
    ...wristCameraAfterJointMotion.map((value, index) => value - wristCameraPosition[index])
  );
  assert(wristCameraTravel > 0.01, `腕部相机没有随机械臂运动：${wristCameraTravel}`);

  physics.reset();
  physics.setTarget('joint7', 0.03);
  physics.step(400);
  const gripper = data.qpos[joints.byName.joint7.qposadr];
  assert(Math.abs(gripper - 0.03) < 0.008, `夹爪应变到 30 mm 附近，q=${gripper}`);

  physics.reset();
  const cubeReset = bodyPos(mujoco, model, data, 'red_cube');
  assert(Math.abs(data.qpos[qposadr]) < 1e-6, `复位后 joint2 应为 0，q=${data.qpos[qposadr]}`);
  assert(Math.abs(data.qpos[joints.byName.joint7.qposadr]) < 1e-6, '复位后夹爪应为 0');
  assert(
    Math.abs(cubeReset[0] - cubeBefore[0]) < 0.01 &&
      Math.abs(cubeReset[1] - cubeBefore[1]) < 0.01 &&
      Math.abs(cubeReset[2] - cubeBefore[2]) < 0.01,
    `复位后红块应回到初始位，got=${cubeReset.join(',')}`
  );

  const graspResults = {};
  for (const [target, body] of [
    ['red', 'red_cube'],
    ['blue', 'blue_block'],
    ['yellow', 'yellow_cylinder']
  ]) {
    physics.reset();
    let graspState = null;
    const graspDemo = createGraspDemo({
      mujoco, model, data, joints, physics, ik,
      onChange: (state) => { graspState = state; }
    });
    const zone = STORAGE_ZONES[target];
    graspDemo.start(target);
    for (let frame = 0; frame < 3200 && graspDemo.isRunning(); frame += 1) {
      graspDemo.update();
      physics.step(12);
    }
    graspState = graspDemo.state();
    const finalPosition = bodyPos(mujoco, model, data, body);
    assert(
      graspState.stage === 'complete',
      `${target} 一键抓取未完成：${graspState.stage}/${graspState.message}`
    );
    assert(
      Math.abs(finalPosition[0] - zone.x) < 0.025 &&
        Math.abs(finalPosition[1] - zone.y) < 0.025 &&
        Math.abs(finalPosition[2] - zone.z) < 0.02,
      `${target} 没有放回对应收纳区：${finalPosition.join(',')}, expected=${zone.x},${zone.y},${zone.z}`
    );
    assert(
      finalPosition[0] > 0.13 && finalPosition[0] < 0.63 &&
        Math.abs(finalPosition[1]) < 0.28,
      `${target} 放置后超出桌面：${finalPosition.join(',')}`
    );
    graspResults[target] = { stage: graspState.stage, position: finalPosition };
  }

  physics.reset();
  const stackDemo = createGraspDemo({
    mujoco, model, data, joints, physics, ik,
    onChange: () => {}
  });
  stackDemo.startStack();
  for (let frame = 0; frame < 12000 && stackDemo.isRunning(); frame += 1) {
    stackDemo.update();
    physics.step(12);
  }
  const stackState = stackDemo.state();
  const stackedBlue = bodyPos(mujoco, model, data, 'blue_block');
  const stackedRed = bodyPos(mujoco, model, data, 'red_cube');
  const stackedYellow = bodyPos(mujoco, model, data, 'yellow_cylinder');
  assert(stackState.stage === 'complete', `叠叠乐未完成：${stackState.stage}/${stackState.message}`);
  [
    ['blue', stackedBlue],
    ['red', stackedRed],
    ['yellow', stackedYellow]
  ].forEach(([id, position]) => {
    const target = STACK_TARGETS[id];
    assert(
      Math.abs(position[0] - target.x) < 0.035 &&
        Math.abs(position[1] - target.y) < 0.035,
      `叠叠乐 ${id} 没有对准堆叠中心：${position.join(',')}, expected=${target.x},${target.y}`
    );
  });
  assert(
    stackedBlue[2] < stackedRed[2] && stackedRed[2] < stackedYellow[2],
    `叠叠乐顺序错误：blue=${stackedBlue[2]}, red=${stackedRed[2]}, yellow=${stackedYellow[2]}`
  );

  console.log(JSON.stringify({
    ngeom: model.ngeom,
    ncam: model.ncam,
    wristCameraPosition,
    wristCameraTravel,
    d405MountExtent: mountExtent,
    d405MountPosition: geomPosition(mujoco, model, data, 'd405_wrist_mount'),
    d405BodyPosition: geomPosition(mujoco, model, data, 'd405_camera_body'),
    d405FrontPosition: geomPosition(mujoco, model, data, 'd405_camera_front'),
    tcpHome: tcp0,
    tcpMatrix,
    ncon: data.ncon,
    maxContactForce,
    timestep: model.opt.timestep,
    joint2: { afterOne, afterHold },
    gripper,
    cube: { settledZ: cubeSettled[2], reset: cubeReset },
    graspDemo: graspResults,
    stackDemo: {
      stage: stackState.stage,
      blue: stackedBlue,
      red: stackedRed,
      yellow: stackedYellow
    }
  }, null, 2));

  data.delete();
  model.delete();
  vfs.delete();
}

main().catch((error) => {
  console.error(error);
  process.exit(1);
});
