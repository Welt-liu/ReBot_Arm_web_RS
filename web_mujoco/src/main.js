import { loadMujocoModule, loadRsScene } from './load-model.js';
import { bindJoints, homePose, readAngles } from './kinematics.js';
import { STEPS_PER_FRAME, createPhysicsController } from './pd-control.js';
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
  const physics = createPhysicsController(mujoco, model, data, joints);

  setStatus(`模型已编译（ngeom=${model.ngeom}），正在构建 Three.js 网格…`);
  view.build(mujoco, model);
  view.sync(data);

  const panel = createJointPanel(jointsEl, joints, (name, value) => {
    physics.setTarget(name, value);
  });
  Object.entries(homePose(joints)).forEach(([name, amount]) => {
    panel.setTarget(name, amount);
    panel.setActual(name, amount);
  });

  resetEl.addEventListener('click', () => {
    physics.reset();
    Object.entries(physics.targets).forEach(([name, amount]) => {
      panel.setTarget(name, amount);
      panel.setActual(name, amount);
    });
  });

  setStatus(
    `已加载 ${files.length} 个资源。滑块只改 PD 目标，每帧 ${STEPS_PER_FRAME} 次 mj_step（约 ${60 * STEPS_PER_FRAME} Hz）。`
  );

  const loop = () => {
    physics.step(STEPS_PER_FRAME);
    const angles = readAngles(data, joints);
    Object.entries(angles).forEach(([name, amount]) => panel.setActual(name, amount));
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
