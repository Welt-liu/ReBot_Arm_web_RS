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
const toggleExplosionEl = document.getElementById('toggle-explosion');
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
  'status.preparingRuntime': 89,
  'status.compiling': 94,
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
let visualProgressCreepFrame = 0;
let visualProgressCreepActive = false;

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

function stopVisualProgressCreep() {
  visualProgressCreepActive = false;
  cancelAnimationFrame(visualProgressCreepFrame);
}

function startVisualProgressCreep() {
  if (visualProgressCreepActive) return;
  visualProgressCreepActive = true;

  void visualProgressQueue.then(() => {
    if (!visualProgressCreepActive || visualProgressTarget > 69) return;
    const start = visualProgress;
    const startedAt = performance.now();

    const creep = (now) => {
      if (!visualProgressCreepActive || visualProgressTarget > 69) return;
      const elapsedSeconds = Math.max(0, now - startedAt) / 1000;
      visualProgress = Math.min(88, Math.max(visualProgress, start + elapsedSeconds * 0.7));
      renderVisualProgress(visualProgress);
      visualProgressCreepFrame = requestAnimationFrame(creep);
    };

    visualProgressCreepFrame = requestAnimationFrame(creep);
  });
}

function renderLoadProgress() {
  const text = t(loadProgress.key, loadProgress.vars);
  setStatus(text);
  if (loadingTextEl) loadingTextEl.textContent = text;
}

function setLoadProgress(progress) {
  loadProgress = typeof progress === 'string' ? { key: progress, vars: {} } : progress;
  const nextProgress = LOAD_STAGE_PROGRESS[loadProgress.key];
  if (nextProgress === 69) {
    void setVisualProgress(nextProgress);
    startVisualProgressCreep();
  } else {
    stopVisualProgressCreep();
    if (nextProgress != null) void setVisualProgress(nextProgress);
  }
  renderLoadProgress();
}

function finishLoading() {
  loadingComplete = true;
  stopVisualProgressCreep();
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
let explosionActive = false;
let explosionAnimating = false;
let explosionStatusTimer = 0;

function renderGuidesToggle() {
  if (!toggleGuidesEl) return;
  toggleGuidesEl.textContent = t(guidesVisible ? 'btn.guidesOff' : 'btn.guidesOn');
  toggleGuidesEl.classList.toggle('active', guidesVisible);
  toggleGuidesEl.setAttribute('aria-pressed', String(guidesVisible));
}

function renderExplosionToggle() {
  if (!toggleExplosionEl) return;
  toggleExplosionEl.textContent = t(explosionActive ? 'btn.assemble' : 'btn.explode');
  toggleExplosionEl.classList.toggle('active', explosionActive);
  toggleExplosionEl.setAttribute('aria-pressed', String(explosionActive));
}

onLangChange(() => {
  applyStaticI18n();
  panel?.applyLang();
  tcpDrag?.applyLang();
  renderGuidesToggle();
  renderExplosionToggle();
  if (!loadingComplete) {
    renderLoadProgress();
  } else if (explosionActive) {
    setStatus(t(explosionAnimating ? 'status.exploding' : 'status.exploded'));
  } else if (!tcpDrag?.isEnabled() && readyCount != null) {
    setStatus(t('status.ready', { n: readyCount }));
  }
});

async function main() {
  setLoadProgress('status.loadingWasm');
  const mujocoPromise = loadMujocoModule();
  const { mujoco, model, data, files, materialProps } = await loadRsScene(mujocoPromise, setLoadProgress);
  // Renderer, PMREM and lighting initialization are deliberately deferred so
  // they do not block the WASM and model requests on the first load.
  const view = createSceneView(viewportEl);
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
  toggleExplosionEl?.addEventListener('click', () => {
    explosionActive = !explosionActive;
    if (explosionActive && tcpDrag?.isEnabled()) tcpDrag.setEnabled(false);
    window.clearTimeout(explosionStatusTimer);
    const animationDuration = view.setExplosion(explosionActive);
    explosionAnimating = explosionActive;
    if (explosionActive) {
      explosionStatusTimer = window.setTimeout(() => {
        if (!explosionActive) return;
        explosionAnimating = false;
        setStatus(t('status.exploded'));
      }, animationDuration);
    }
    panel.closeChips();
    renderExplosionToggle();
    setStatus(t(explosionActive ? 'status.exploding' : 'status.assembled'));
  });
  renderExplosionToggle();
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
    window.clearTimeout(explosionStatusTimer);
    explosionActive = false;
    explosionAnimating = false;
    view.setExplosion(false);
    renderExplosionToggle();
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
