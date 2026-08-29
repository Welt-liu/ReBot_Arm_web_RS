import { loadMujocoModule, loadRsScene } from './load-model.js';
import { bindJoints, homePose, readAngles } from './kinematics.js';
import { STEPS_PER_FRAME, createPhysicsController } from './pd-control.js';
import { createSceneView } from './scene-view.js';
import { createJointCallouts } from './ui.js';
import { createTcpIk } from './tcp-ik.js';
import { createTcpDrag } from './tcp-drag.js';
import { t, bindLangSwitch, applyStaticI18n, onLangChange } from './i18n.js';
import { persistAppShell } from './register-service-worker.js';

const statusEl = document.getElementById('status');
const calloutsEl = document.getElementById('callouts');
const resetEl = document.getElementById('reset');
const toggleDragEl = document.getElementById('toggle-drag');
const toggleGuidesEl = document.getElementById('toggle-guides');
const dragMarkerEl = document.getElementById('drag-marker');
const dragClusterEl = document.getElementById('drag-cluster');
const gripperOpenEl = document.getElementById('gripper-open');
const gripperCloseEl = document.getElementById('gripper-close');
const viewportEl = document.getElementById('viewport');
const langSwitchEl = document.getElementById('lang-select');
const loadingOverlayEl = document.getElementById('loading-overlay');
const loadingTextEl = document.getElementById('loading-text');
const loadingProgressEl = document.getElementById('loading-progress');
const loadingProgressFillEl = document.getElementById('loading-progress-fill');
const loadingProgressValueEl = document.getElementById('loading-progress-value');

const LOAD_STAGE_PROGRESS = {
  'status.booting': 12,
  'status.loadingWasm': 49,
  'status.download': 69,
  'status.downloadProgress': 69,
  'status.loadingAssets': 69,
  'status.compiling': 89,
  'status.compiled': 99
};

bindLangSwitch(langSwitchEl);
applyStaticI18n();

function setStatus(text) {
  statusEl.textContent = text;
}

let loadProgress = { key: 'status.booting', vars: {} };
let loadingComplete = false;
let visualProgress = 0;
let visualProgressTarget = 0;
let visualProgressFrame = 0;
let visualProgressQueue = Promise.resolve();

function renderVisualProgress(value) {
  const rounded = Math.round(value);
  if (loadingProgressFillEl) loadingProgressFillEl.style.width = `${value}%`;
  if (loadingProgressValueEl) loadingProgressValueEl.textContent = `${rounded}%`;
  loadingProgressEl?.setAttribute('aria-valuenow', String(rounded));
}

function animateVisualProgress(next) {
  return new Promise((resolve) => {
    const start = visualProgress;
    const startedAt = performance.now();
    const duration = next >= 100 ? 120 : 240;

    const animate = (now) => {
      const ratio = Math.max(0, Math.min(1, (now - startedAt) / duration));
      const eased = 1 - (1 - ratio) ** 3;
      visualProgress = start + (next - start) * eased;
      renderVisualProgress(visualProgress);
      if (ratio < 1) {
        visualProgressFrame = requestAnimationFrame(animate);
      } else {
        visualProgress = next;
        renderVisualProgress(next);
        resolve();
      }
    };
    visualProgressFrame = requestAnimationFrame(animate);
  });
}

function setVisualProgress(next) {
  if (next <= visualProgressTarget) return visualProgressQueue;
  visualProgressTarget = next;
  visualProgressQueue = visualProgressQueue.then(() => animateVisualProgress(next));
  return visualProgressQueue;
}

function renderLoadProgress() {
  const text = t(loadProgress.key, loadProgress.vars);
  setStatus(text);
  if (loadingTextEl) loadingTextEl.textContent = text;
}

function setLoadProgress(progress) {
  loadProgress = typeof progress === 'string' ? { key: progress, vars: {} } : progress;
  setVisualProgress(LOAD_STAGE_PROGRESS[loadProgress.key] ?? visualProgressTarget);
  renderLoadProgress();
}

function finishLoading() {
  loadingComplete = true;
  void setVisualProgress(100).then(() => {
    window.setTimeout(() => loadingOverlayEl?.classList.add('is-hidden'), 80);
  });
  void persistAppShell();
}

renderLoadProgress();
setVisualProgress(LOAD_STAGE_PROGRESS['status.booting']);

let panel = null;
let tcpDrag = null;
let readyCount = null;
let guidesVisible = false;

function renderGuidesToggle() {
  if (!toggleGuidesEl) return;
  toggleGuidesEl.textContent = t(guidesVisible ? 'btn.guidesOff' : 'btn.guidesOn');
  toggleGuidesEl.classList.toggle('active', guidesVisible);
  toggleGuidesEl.setAttribute('aria-pressed', String(guidesVisible));
}

onLangChange(() => {
  applyStaticI18n();
  panel?.applyLang();
  tcpDrag?.applyLang();
  renderGuidesToggle();
  if (!loadingComplete) {
    renderLoadProgress();
  } else if (!tcpDrag?.isEnabled() && readyCount != null) {
    setStatus(t('status.ready', { n: readyCount }));
  }
});

async function main() {
  const view = createSceneView(viewportEl);
  setLoadProgress('status.loadingWasm');
  const mujoco = await loadMujocoModule();

  const { model, data, files, materialProps } = await loadRsScene(mujoco, setLoadProgress);
  const joints = bindJoints(mujoco, model);
  const physics = createPhysicsController(mujoco, model, data, joints);
  const ik = createTcpIk(mujoco, model, data, joints);

  setLoadProgress({ key: 'status.compiled', vars: { ngeom: model.ngeom } });
  view.build(mujoco, model, materialProps);
  view.sync(data);

  panel = createJointCallouts(calloutsEl, joints, (name, value) => {
    physics.setTarget(name, value);
  });
  panel.setGuidesVisible(guidesVisible);
  toggleGuidesEl?.addEventListener('click', () => {
    guidesVisible = !guidesVisible;
    panel.setGuidesVisible(guidesVisible);
    renderGuidesToggle();
  });
  renderGuidesToggle();
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
  finishLoading();

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
  const message = error && error.message ? error.message : error;
  loadingOverlayEl?.classList.add('is-error');
  setLoadProgress({ key: 'status.fail', vars: { error: message } });
});
