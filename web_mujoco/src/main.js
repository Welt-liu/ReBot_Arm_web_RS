import { loadMujocoModule, loadRsScene } from './load-model.js';
import { bindJoints, homePose, readAngles } from './kinematics.js';
import { STEPS_PER_FRAME, createPhysicsController } from './pd-control.js';
import { createSceneView } from './scene-view.js';
import { createJointCallouts } from './ui.js';
import { createTcpIk } from './tcp-ik.js';
import { createTcpDrag } from './tcp-drag.js';
import { t, bindLangSwitch, applyStaticI18n, onLangChange } from './i18n.js';

const statusEl = document.getElementById('status');
const calloutsEl = document.getElementById('callouts');
const resetEl = document.getElementById('reset');
const toggleDragEl = document.getElementById('toggle-drag');
const dragMarkerEl = document.getElementById('drag-marker');
const dragClusterEl = document.getElementById('drag-cluster');
const gripperOpenEl = document.getElementById('gripper-open');
const gripperCloseEl = document.getElementById('gripper-close');
const viewportEl = document.getElementById('viewport');
const langSwitchEl = document.getElementById('lang-select');

bindLangSwitch(langSwitchEl);
applyStaticI18n();

function setStatus(text) {
  statusEl.textContent = text;
}

setStatus(t('status.booting'));

let panel = null;
let tcpDrag = null;
let readyCount = null;

onLangChange(() => {
  applyStaticI18n();
  panel?.applyLang();
  tcpDrag?.applyLang();
  if (!tcpDrag?.isEnabled() && readyCount != null) {
    setStatus(t('status.ready', { n: readyCount }));
  }
});

async function main() {
  const view = createSceneView(viewportEl);
  setStatus(t('status.loadingWasm'));
  const mujoco = await loadMujocoModule();

  const { model, data, files } = await loadRsScene(mujoco, setStatus);
  const joints = bindJoints(mujoco, model);
  const physics = createPhysicsController(mujoco, model, data, joints);
  const ik = createTcpIk(mujoco, model, data, joints);

  setStatus(t('status.compiled', { ngeom: model.ngeom }));
  view.build(mujoco, model);
  view.sync(data);

  panel = createJointCallouts(calloutsEl, joints, (name, value) => {
    physics.setTarget(name, value);
  });
  Object.entries(homePose(joints)).forEach(([name, amount]) => {
    panel.setTarget(name, amount);
    panel.setActual(name, amount);
  });

  tcpDrag = createTcpDrag({
    view,
    ik,
    physics,
    panel,
    clusterEl: dragClusterEl,
    markerEl: dragMarkerEl,
    hostEl: viewportEl,
    toggleEl: toggleDragEl,
    openEl: gripperOpenEl,
    closeEl: gripperCloseEl,
    onStatus: setStatus
  });

  view.render();
  panel.layout(view.projectWorld, data);

  resetEl.addEventListener('click', () => {
    tcpDrag.stop();
    physics.reset();
    Object.entries(physics.targets).forEach(([name, amount]) => {
      panel.setTarget(name, amount);
      panel.setActual(name, amount);
    });
  });

  setStatus(t('status.ready', { n: files.length }));
  readyCount = files.length;

  const loop = () => {
    physics.step(STEPS_PER_FRAME);
    const angles = readAngles(data, joints);
    Object.entries(angles).forEach(([name, amount]) => panel.setActual(name, amount));
    view.sync(data);
    tcpDrag.update(performance.now());
    view.render();
    panel.layout(view.projectWorld, data);
    requestAnimationFrame(loop);
  };
  requestAnimationFrame(loop);
}

main().catch((error) => {
  console.error(error);
  setStatus(t('status.fail', { error: error && error.message ? error.message : error }));
});
