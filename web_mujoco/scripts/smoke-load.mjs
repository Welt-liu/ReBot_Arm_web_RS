import { readFile } from 'node:fs/promises';
import path from 'node:path';
import { fileURLToPath } from 'node:url';
import loadMujoco from '@mujoco/mujoco';
import { ARM_JOINTS, bindJoints } from '../src/kinematics.js';
import { createPhysicsController } from '../src/pd-control.js';
import { createTcpIk } from '../src/tcp-ik.js';

const here = path.dirname(fileURLToPath(import.meta.url));
const modelsDir = path.resolve(here, '../../rebotarm_ros2/src/rebotarm_mujoco_rs/models');

function namedId(mujoco, model, type, name) {
  const id = mujoco.mj_name2id(model, type, name);
  if (id < 0) throw new Error(`找不到 ${name}`);
  return id;
}

function bodyPos(mujoco, model, data, name) {
  const id = namedId(mujoco, model, mujoco.mjtObj.mjOBJ_BODY.value, name);
  return [data.xpos[id * 3], data.xpos[id * 3 + 1], data.xpos[id * 3 + 2]];
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

  const meshRe = /\sfile="([^"]+\.STL)"/g;
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
  assert(Number.isInteger(joints.byName.joint2.id), '关节 id 无效');
  assert(Number.isFinite(data.xanchor[joints.byName.joint2.id * 3]), 'xanchor 无法读取');

  const ik = createTcpIk(mujoco, model, data, joints);
  const tcp0 = ik.tcpPosition();
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
  physics.step(200);
  const cubeSettled = bodyPos(mujoco, model, data, 'red_cube');
  assert(data.ncon > 0, `色块落地后应有接触，ncon=${data.ncon}`);
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

  console.log(JSON.stringify({
    ngeom: model.ngeom,
    ncon: data.ncon,
    timestep: model.opt.timestep,
    joint2: { afterOne, afterHold },
    gripper,
    cube: { settledZ: cubeSettled[2], reset: cubeReset }
  }, null, 2));

  data.delete();
  model.delete();
  vfs.delete();
}

main().catch((error) => {
  console.error(error);
  process.exit(1);
});
