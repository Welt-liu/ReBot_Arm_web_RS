import { readFile } from 'node:fs/promises';
import path from 'node:path';
import { fileURLToPath } from 'node:url';
import loadMujoco from '@mujoco/mujoco';
import { applyKinematicPose, bindJoints, homePose } from '../src/kinematics.js';

const here = path.dirname(fileURLToPath(import.meta.url));
const modelsDir = path.resolve(here, '../../rebotarm_ros2/src/rebotarm_mujoco_rs/models');

async function main() {
  const mujoco = await loadMujoco();
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
  const joints = bindJoints(mujoco, model);
  const pose = homePose(joints);
  pose.joint2 = 0.4;
  pose.joint7 = 0.03;
  applyKinematicPose(mujoco, model, data, joints, pose);

  console.log(JSON.stringify({
    ngeom: model.ngeom,
    nq: model.nq,
    joint2: data.qpos[joints.byName.joint2.qposadr],
    gripper: {
      joint7: data.qpos[joints.byName.joint7.qposadr],
      left: data.qpos[joints.byName.joint_left.qposadr],
      right: data.qpos[joints.byName.joint_right.qposadr]
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
