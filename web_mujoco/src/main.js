import { loadMujocoModule, loadRsScene } from './load-model.js';
import { applyKinematicPose, bindJoints, homePose, readAngles } from './kinematics.js';
import { createSceneView } from './scene-view.js';
import { createJointPanel } from './ui.js';

const statusEl = document.getElementById('status');
const jointsEl = document.getElementById('joints');
const resetEl = document.getElementById('reset');
const viewportEl = document.getElementById('viewport');

function setStatus(text) {
  statusEl.textContent = text;
}

async function main() {
  const view = createSceneView(viewportEl);
  setStatus('正在加载 MuJoCo WASM…');
  const mujoco = await loadMujocoModule();

  const { model, data, files } = await loadRsScene(mujoco, setStatus);
  const joints = bindJoints(mujoco, model);
  const pose = homePose(joints);
  applyKinematicPose(mujoco, model, data, joints, pose);

  setStatus(`模型已编译（ngeom=${model.ngeom}），正在构建 Three.js 网格…`);
  view.build(mujoco, model);
  view.sync(data);

  const panel = createJointPanel(jointsEl, joints, (name, value) => {
    pose[name] = value;
    applyKinematicPose(mujoco, model, data, joints, pose);
    const angles = readAngles(data, joints);
    Object.entries(angles).forEach(([jointName, amount]) => panel.set(jointName, amount));
  });
  Object.entries(readAngles(data, joints)).forEach(([name, amount]) => panel.set(name, amount));

  resetEl.addEventListener('click', () => {
    Object.assign(pose, homePose(joints));
    applyKinematicPose(mujoco, model, data, joints, pose);
    Object.entries(readAngles(data, joints)).forEach(([name, amount]) => panel.set(name, amount));
  });

  setStatus(`已加载 ${files.length} 个资源，ngeom=${model.ngeom}。拖动滑块验证运动学。`);

  const loop = () => {
    view.sync(data);
    view.render();
    requestAnimationFrame(loop);
  };
  requestAnimationFrame(loop);
}

main().catch((error) => {
  console.error(error);
  setStatus(`启动失败：${error && error.message ? error.message : error}`);
});
