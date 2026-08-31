import { loadMujocoModule, loadRsScene } from './load-model.js';
import { bindJoints, homePose, readAngles } from './kinematics.js';
import { STEPS_PER_FRAME, createPhysicsController } from './pd-control.js';
import { createSceneView } from './scene-view.js';
import { createJointCallouts } from './ui.js';
import { createTcpIk } from './tcp-ik.js';
import { createTcpDrag } from './tcp-drag.js';
import { createPartExplorer } from './part-explorer.js';
import { createGraspDemo } from './grasp-demo.js';
import { createContactTelemetry } from './contact-telemetry.js';
import { createTelemetryPanel } from './telemetry-panel.js';
import { t, bindLangSwitch, applyStaticI18n, onLangChange } from './i18n.js';
import { persistAppShell } from './register-service-worker.js';

const statusEl = document.getElementById('status');
const calloutsEl = document.getElementById('callouts');
const resetEl = document.getElementById('reset');
const toggleExplosionEl = document.getElementById('toggle-explosion');
const pauseExplosionEl = document.getElementById('pause-explosion');
const reverseExplosionEl = document.getElementById('reverse-explosion');
const explosionProgressEl = document.getElementById('explosion-progress');
const explosionProgressValueEl = document.getElementById('explosion-progress-value');
const explosionStageEl = document.getElementById('explosion-stage');
const partExplorerEl = document.getElementById('part-explorer');
const toggleDragEl = document.getElementById('toggle-drag');
const toggleGuidesEl = document.getElementById('toggle-guides');
const toggleCameraViewsEl = document.getElementById('toggle-camera-views');
const cameraFeedGridEl = document.getElementById('camera-feed-grid');
const cameraLayoutRowEl = document.getElementById('camera-layout-row');
const cameraLayoutColumnEl = document.getElementById('camera-layout-column');
const toggleCameraModelEl = document.getElementById('toggle-camera-model');
const overheadCameraFeedEl = document.getElementById('overhead-camera-feed');
const wristCameraFeedEl = document.getElementById('wrist-camera-feed');
const collapseOverheadCameraEl = document.getElementById('collapse-overhead-camera');
const collapseWristCameraEl = document.getElementById('collapse-wrist-camera');
const graspControlsEl = document.getElementById('grasp-controls');
const visionTargetsEl = document.getElementById('vision-targets');
const startGraspEl = document.getElementById('start-grasp');
const cancelGraspEl = document.getElementById('cancel-grasp');
const graspProgressEl = document.getElementById('grasp-progress');
const graspProgressValueEl = document.getElementById('grasp-progress-value');
const graspStageEl = document.getElementById('grasp-stage');
const telemetryControlsEl = document.getElementById('telemetry-controls');
const toggleContactVisualsEl = document.getElementById('toggle-contact-visuals');
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
let partExplorer = null;
let graspDemo = null;
let telemetryPanel = null;
let readyCount = null;
let guidesVisible = false;
let overheadCameraVisible = false;
let wristCameraVisible = false;
let cameraViewsVisible = false;
let cameraViewsAvailable = false;
let cameraModelVisible = true;
let cameraModelAvailable = false;
let cameraLayout = 'row';
let overheadCameraCollapsed = false;
let wristCameraCollapsed = false;
let selectedVisionTarget = 'red';
let contactVisualsEnabled = true;
let graspState = { running: false, stage: 'idle', progress: 0, message: '' };
let explosionState = { progress: 0, direction: 0, stageKey: 'assembled' };

function renderGuidesToggle() {
  if (!toggleGuidesEl) return;
  toggleGuidesEl.textContent = t(guidesVisible ? 'btn.guidesOff' : 'btn.guidesOn');
  toggleGuidesEl.classList.toggle('active', guidesVisible);
  toggleGuidesEl.setAttribute('aria-pressed', String(guidesVisible));
}

function renderCameraViewsToggle() {
  if (toggleCameraViewsEl) {
    toggleCameraViewsEl.textContent = t(cameraViewsVisible ? 'camera.hideAll' : 'camera.showAll');
    toggleCameraViewsEl.classList.toggle('active', cameraViewsVisible);
    toggleCameraViewsEl.setAttribute('aria-checked', String(cameraViewsVisible));
    toggleCameraViewsEl.disabled = !cameraViewsAvailable || explosionState.progress > 0;
  }
  if (cameraFeedGridEl) cameraFeedGridEl.hidden = !cameraViewsVisible;
  overheadCameraFeedEl?.classList.toggle('is-live', overheadCameraVisible);
  wristCameraFeedEl?.classList.toggle('is-live', wristCameraVisible);
}

function renderCameraModelToggle() {
  if (!toggleCameraModelEl) return;
  toggleCameraModelEl.textContent = t(cameraModelVisible ? 'camera.modelHide' : 'camera.modelShow');
  toggleCameraModelEl.setAttribute('aria-checked', String(cameraModelVisible));
  toggleCameraModelEl.classList.toggle('active', cameraModelVisible);
  toggleCameraModelEl.disabled = !cameraModelAvailable;
}

function renderCameraLayout() {
  const column = cameraLayout === 'column';
  cameraFeedGridEl?.classList.toggle('is-column', column);
  cameraLayoutRowEl?.classList.toggle('active', !column);
  cameraLayoutColumnEl?.classList.toggle('active', column);
  cameraLayoutRowEl?.setAttribute('aria-pressed', String(!column));
  cameraLayoutColumnEl?.setAttribute('aria-pressed', String(column));

  const renderCollapse = (feed, button, collapsed) => {
    feed?.classList.toggle('is-collapsed', collapsed);
    if (!button) return;
    button.textContent = collapsed ? '⌄' : '⌃';
    button.setAttribute('aria-expanded', String(!collapsed));
    button.setAttribute('aria-label', t(collapsed ? 'camera.expand' : 'camera.collapse'));
  };
  renderCollapse(overheadCameraFeedEl, collapseOverheadCameraEl, overheadCameraCollapsed);
  renderCollapse(wristCameraFeedEl, collapseWristCameraEl, wristCameraCollapsed);
}

function renderGraspControls() {
  visionTargetsEl?.querySelectorAll('[data-target]').forEach((button) => {
    const active = button.dataset.target === selectedVisionTarget;
    button.classList.toggle('active', active);
    button.setAttribute('aria-pressed', String(active));
    button.disabled = graspState.running;
  });
  if (startGraspEl) startGraspEl.disabled = graspState.running || explosionState.progress > 0;
  if (cancelGraspEl) cancelGraspEl.disabled = !graspState.running;
  if (graspProgressEl) graspProgressEl.value = graspState.progress || 0;
  if (graspProgressValueEl) graspProgressValueEl.textContent = `${Math.round((graspState.progress || 0) * 100)}%`;
  if (graspStageEl) graspStageEl.textContent = t(`grasp.stage.${graspState.stage}`);
  document.body.classList.toggle('grasp-running', graspState.running);
  if (toggleDragEl) toggleDragEl.disabled = graspState.running;
  if (gripperOpenEl) gripperOpenEl.disabled = graspState.running;
  if (gripperCloseEl) gripperCloseEl.disabled = graspState.running;
}

function renderContactVisualsToggle() {
  if (!toggleContactVisualsEl) return;
  toggleContactVisualsEl.textContent = t(contactVisualsEnabled ? 'telemetry.hideContacts' : 'telemetry.showContacts');
  toggleContactVisualsEl.setAttribute('aria-checked', String(contactVisualsEnabled));
  toggleContactVisualsEl.classList.toggle('active', contactVisualsEnabled);
}

function renderExplosionControls() {
  const percent = Math.round(explosionState.progress * 100);
  explosionProgressEl.value = String(explosionState.progress * 100);
  explosionProgressValueEl.textContent = `${percent}%`;
  explosionProgressEl.setAttribute('aria-valuenow', String(percent));
  explosionStageEl.textContent = t(`timeline.stage.${explosionState.stageKey}`);
  toggleExplosionEl.disabled = graspState.running || (explosionState.progress >= 1 && explosionState.direction === 0);
  pauseExplosionEl.disabled = explosionState.direction === 0;
  reverseExplosionEl.disabled = explosionState.progress <= 0 && explosionState.direction === 0;
  toggleExplosionEl.classList.toggle('active', explosionState.direction > 0);
  pauseExplosionEl.classList.toggle('active', explosionState.direction === 0 && explosionState.progress > 0 && explosionState.progress < 1);
  reverseExplosionEl.classList.toggle('active', explosionState.direction < 0);
  toggleExplosionEl.setAttribute('aria-pressed', String(explosionState.direction > 0));
  reverseExplosionEl.setAttribute('aria-pressed', String(explosionState.direction < 0));
}

onLangChange(() => {
  applyStaticI18n();
  panel?.applyLang();
  tcpDrag?.applyLang();
  partExplorer?.applyLang();
  renderGuidesToggle();
  renderCameraViewsToggle();
  renderCameraModelToggle();
  renderCameraLayout();
  renderGraspControls();
  renderContactVisualsToggle();
  renderExplosionControls();
  if (!loadingComplete) {
    renderLoadProgress();
  } else if (explosionState.direction > 0) {
    setStatus(t('status.exploding'));
  } else if (explosionState.direction < 0) {
    setStatus(t('status.assembling'));
  } else if (explosionState.progress >= 1) {
    setStatus(t('status.exploded'));
  } else if (explosionState.progress > 0) {
    setStatus(t('status.explosionPaused', { percent: Math.round(explosionState.progress * 100) }));
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
  const contactTelemetry = createContactTelemetry(mujoco, model, data);

  setLoadProgress({ key: 'status.compiled', vars: { ngeom: model.ngeom } });
  view.build(mujoco, model, materialProps);
  view.sync(data);
  cameraViewsAvailable = view.hasOverheadCamera() || view.hasWristCamera();
  overheadCameraVisible = view.setOverheadCameraEnabled(true);
  wristCameraVisible = view.setWristCameraEnabled(true);
  cameraViewsVisible = overheadCameraVisible || wristCameraVisible;
  cameraModelAvailable = view.hasCameraModel();
  cameraModelVisible = view.setCameraModelVisible(true);

  function setCameraViewsVisible(enabled) {
    overheadCameraVisible = view.setOverheadCameraEnabled(enabled);
    wristCameraVisible = view.setWristCameraEnabled(enabled);
    cameraViewsVisible = overheadCameraVisible || wristCameraVisible;
    renderCameraViewsToggle();
    return cameraViewsVisible;
  }

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
  toggleCameraViewsEl?.addEventListener('click', () => {
    const visible = setCameraViewsVisible(!cameraViewsVisible);
    setStatus(t(visible ? 'status.cameraViewsOn' : 'status.cameraViewsOff'));
  });
  renderCameraViewsToggle();
  toggleCameraModelEl?.addEventListener('click', () => {
    cameraModelVisible = view.setCameraModelVisible(!cameraModelVisible);
    renderCameraModelToggle();
    setStatus(t(cameraModelVisible ? 'status.cameraModelOn' : 'status.cameraModelOff'));
  });
  renderCameraModelToggle();
  cameraLayoutRowEl?.addEventListener('click', () => {
    cameraLayout = 'row';
    renderCameraLayout();
  });
  cameraLayoutColumnEl?.addEventListener('click', () => {
    cameraLayout = 'column';
    renderCameraLayout();
  });
  collapseOverheadCameraEl?.addEventListener('click', () => {
    overheadCameraCollapsed = !overheadCameraCollapsed;
    renderCameraLayout();
  });
  collapseWristCameraEl?.addEventListener('click', () => {
    wristCameraCollapsed = !wristCameraCollapsed;
    renderCameraLayout();
  });
  renderCameraLayout();
  let previousGraspStage = 'idle';
  graspDemo = createGraspDemo({
    mujoco,
    model,
    data,
    joints,
    physics,
    ik,
    onChange: (state) => {
      graspState = state;
      selectedVisionTarget = state.selectedId;
      renderGraspControls();
      renderExplosionControls();
      if (state.stage !== previousGraspStage) {
        previousGraspStage = state.stage;
        if (state.stage === 'complete') setStatus(t('status.graspComplete'));
        else if (state.stage === 'failed') setStatus(t('status.graspFailed'));
        else if (state.running) setStatus(t(`status.grasp.${state.stage}`));
      }
    }
  });
  telemetryPanel = createTelemetryPanel(telemetryControlsEl);
  visionTargetsEl?.addEventListener('click', (event) => {
    const button = event.target.closest('[data-target]');
    if (!button || graspState.running) return;
    selectedVisionTarget = button.dataset.target;
    graspDemo.setSelected(selectedVisionTarget);
    renderGraspControls();
  });
  startGraspEl?.addEventListener('click', () => {
    if (graspState.running || explosionState.progress > 0) return;
    if (tcpDrag?.isEnabled()) tcpDrag.setEnabled(false);
    view.clearPartSelection();
    graspDemo.start(selectedVisionTarget);
  });
  cancelGraspEl?.addEventListener('click', () => {
    if (graspDemo.cancel()) setStatus(t('status.graspCancelled'));
  });
  toggleContactVisualsEl?.addEventListener('click', () => {
    contactVisualsEnabled = !contactVisualsEnabled;
    if (!contactVisualsEnabled) view.setContactVisuals([], false);
    renderContactVisualsToggle();
  });
  renderGraspControls();
  renderContactVisualsToggle();
  toggleExplosionEl?.addEventListener('click', () => {
    if (cameraViewsVisible) setCameraViewsVisible(false);
    if (tcpDrag?.isEnabled()) tcpDrag.setEnabled(false);
    view.playExplosion(1);
    panel.closeChips();
    setStatus(t('status.exploding'));
  });
  pauseExplosionEl?.addEventListener('click', () => {
    view.pauseExplosion();
    setStatus(t('status.explosionPaused', {
      percent: Math.round(view.getExplosionState().progress * 100)
    }));
  });
  reverseExplosionEl?.addEventListener('click', () => {
    view.playExplosion(-1);
    setStatus(t('status.assembling'));
  });
  explosionProgressEl?.addEventListener('pointerdown', (event) => event.stopPropagation());
  explosionProgressEl?.addEventListener('input', () => {
    const progress = Number(explosionProgressEl.value) / 100;
    if (progress > 0 && tcpDrag?.isEnabled()) tcpDrag.setEnabled(false);
    if (progress > 0 && cameraViewsVisible) setCameraViewsVisible(false);
    view.setExplosionProgress(progress);
    panel.closeChips();
    setStatus(t('status.explosionProgress', { percent: Math.round(progress * 100) }));
  });
  let previousExplosionDirection = 0;
  view.onExplosionChange((state) => {
    const stoppedAtEndpoint = previousExplosionDirection !== 0 && state.direction === 0;
    previousExplosionDirection = state.direction;
    explosionState = state;
    renderExplosionControls();
    renderCameraViewsToggle();
    if (stoppedAtEndpoint && state.progress >= 1) setStatus(t('status.exploded'));
    if (stoppedAtEndpoint && state.progress <= 0) setStatus(t('status.assembled'));
  });
  renderExplosionControls();
  partExplorer = createPartExplorer(partExplorerEl, view);
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
    graspDemo.reset();
    telemetryPanel.reset();
    view.resetExplosion();
    view.clearPartSelection();
    renderExplosionControls();
    physics.reset();
    Object.entries(physics.targets).forEach(([name, amount]) => {
      panel.setTarget(name, amount);
      panel.setActual(name, amount);
    });
    setStatus(t('status.ready', { n: readyCount }));
  });

  setStatus(t('status.ready', { n: files.length }));
  readyCount = files.length;
  finishLoading();

  let lastTelemetryAt = -Infinity;
  const loop = () => {
    graspDemo.update();
    physics.step(graspDemo.isRunning() ? 12 : STEPS_PER_FRAME);
    const angles = readAngles(data, joints);
    Object.entries(angles).forEach(([name, amount]) => panel.setActual(name, amount));
    if (graspDemo.isRunning()) {
      Object.entries(physics.targets).forEach(([name, amount]) => panel.setTarget(name, amount));
    }
    view.sync(data);
    tcpDrag.update(performance.now());
    const now = performance.now();
    if (now - lastTelemetryAt >= 80) {
      const contacts = contactTelemetry.sample(selectedVisionTarget);
      view.setContactVisuals(contacts.contacts, contactVisualsEnabled);
      telemetryPanel.update(physics.telemetry(), contacts);
      lastTelemetryAt = now;
    }
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
