import { WasmForceClient, hapticSchema, defaultSettings } from './force-feedback-client.js';
import { t, onLangChange } from './i18n.js';

export function createForceFeedback(root, { onGripperTarget = () => {}, onGripperControl = () => {} } = {}) {
  const connect = root.querySelector('#feedback-connect');
  const test = root.querySelector('#feedback-test');
  const portSelect = root.querySelector('#feedback-port');
  const refreshPorts = root.querySelector('#feedback-refresh-ports');
  const portStatus = root.querySelector('#feedback-port-status');
  let loadingPorts = false;
  let portsError = '';
  let availablePorts = [];
  const status = root.querySelector('#feedback-status');
  const angleError = root.querySelector('#feedback-angle-error');
  const raw = root.querySelector('#feedback-raw');
  const force = root.querySelector('#feedback-force');
  const angle = root.querySelector('#feedback-angle');
  const width = root.querySelector('#feedback-width');
  const wallAngle = root.querySelector('#feedback-wall-angle');
  const wallStatus = root.querySelector('#feedback-wall-status');
  const angleControl = root.querySelector('#feedback-angle-control');
  const reverse = root.querySelector('#feedback-reverse');
  const zero = root.querySelector('#feedback-zero');
  let lastState = { connected: false, raw: 0 };
  let connecting = false;
  let settingsError = '';
  const inputs = new Map();
  const settingsStatus = root.querySelector('#feedback-settings-status');
  const render = () => {
    connect.textContent = t(lastState.connected ? 'feedback.disconnect' : 'feedback.connect');
    connect.disabled = connecting || (!lastState.connected && (!portSelect.value || loadingPorts));
    portSelect.disabled = connecting || Boolean(lastState.connected) || loadingPorts || !availablePorts.length;
    refreshPorts.disabled = connecting || Boolean(lastState.connected) || loadingPorts;
    portSelect.options[0].textContent = t('feedback.portPlaceholder');
    portStatus.textContent = portsError ? t('feedback.portError', { error: portsError }) :
      t(loadingPorts ? 'feedback.portLoading' : !availablePorts.length ? 'feedback.noPorts' : 'feedback.portHint');
    portStatus.classList.toggle('is-error', Boolean(portsError));
    connect.setAttribute('aria-pressed', String(Boolean(lastState.connected)));
    test.disabled = !lastState.connected;
    zero.disabled = !lastState.connected;
    angle.textContent = Number.isFinite(lastState.angle_deg) ? `${lastState.angle_deg.toFixed(1)}°` : '—';
    width.textContent = Number.isFinite(lastState.gripper_mm) ? `${lastState.gripper_mm.toFixed(1)} mm` : '—';
    wallAngle.textContent = Number.isFinite(lastState.wall_angle_deg) ? `${lastState.wall_angle_deg.toFixed(2)}°` : '—';
    wallStatus.textContent = t(lastState.trigger_return_active ? 'feedback.wallState.returning' :
      `feedback.wallState.${lastState.wall_released ? 'released' : lastState.wall_active ? 'active' : Number.isFinite(lastState.wall_angle_deg) ? 'armed' : 'free'}`);
    raw.textContent = `${lastState.raw ?? 0} / ${lastState.max_raw ?? 150}`;
    status.textContent = lastState.reason && !lastState.connected
      ? t('feedback.error', { error: lastState.reason })
      : t(lastState.connected ? 'feedback.connected' : 'feedback.disconnected');
    const diagnostic = lastState.angle_error;
    angleError.hidden = !diagnostic;
    if (diagnostic) {
      const flags = diagnostic.status_flags;
      const alarms = [1, 2, 4, 8, 32].filter(bit => flags & bit).map(bit => t(`feedback.alarm.${bit}`));
      if (Number.isInteger(flags) && (flags & ~47)) alarms.push(t('feedback.alarm.unknown'));
      const servoStatus = diagnostic.status_available && Number.isInteger(flags)
        ? `0x${flags.toString(16).toUpperCase().padStart(2, '0')} · ${alarms.length ? alarms.join(' / ') : t('feedback.alarm.none')}`
        : t('feedback.alarm.unreadable');
      const details = [];
      if (Number.isFinite(diagnostic.voltage_v)) details.push(`${diagnostic.voltage_v.toFixed(1)} V`);
      if (Number.isFinite(diagnostic.temperature_c)) details.push(`${diagnostic.temperature_c} °C`);
      if (diagnostic.torque_enable === 0) details.push(t('feedback.alarm.outputOff'));
      const comm = diagnostic.reads?.status?.communication;
      if (Number.isInteger(comm) && comm !== 0) details.push(t('feedback.alarm.communication', { code: comm }));
      angleError.textContent = t('feedback.angleError', { error: diagnostic.original_error || '',
        status: servoStatus, details: details.length ? `；${details.join(' / ')}` : '' });
    }
    root.querySelector('#feedback-grip-state').textContent = t(`feedback.state.${lastState.hard_wall ? 'hard_wall' : lastState.grip_state || 'free'}`);
    root.querySelector('#feedback-fingers').textContent = `${(lastState.left_force_n || 0).toFixed(1)} / ${(lastState.right_force_n || 0).toFixed(1)} N`;
    root.querySelector('#feedback-wall').textContent = `${(lastState.wall_error_mm || 0).toFixed(1)} mm`;
    root.querySelector('#feedback-parts').textContent = ['force_term', 'spring_term', 'damping_term'].map(k => Math.round(lastState[k] || 0)).join(' / ');
    settingsStatus.textContent = settingsError || t(lastState.connected ? 'feedback.live' : 'feedback.saved');
    settingsStatus.classList.toggle('is-error', Boolean(settingsError));
    for (const item of hapticSchema) {
      const input = inputs.get(item.key);
      if (input) {
        input.previousElementSibling.textContent = t(`feedback.param.${item.key}`);
        input.parentElement.title = t(`feedback.help.${item.key}`);
      }
    }
    root.classList.toggle('is-active', Boolean(lastState.connected));
  };
  const client = new WasmForceClient({ onStatus: (state) => { lastState = state; render(); } });
  const loadPorts = async () => {
    if (client.token || connecting || loadingPorts) return;
    loadingPorts = true;
    portsError = '';
    render();
    const previous = portSelect.value;
    try {
      const result = await client.request('ports', {});
      if (!Array.isArray(result.ports)) throw new Error(t('feedback.invalidPorts'));
      availablePorts = result.ports.filter(port => typeof port.device === 'string' && typeof port.label === 'string');
      portSelect.replaceChildren(new Option(t('feedback.portPlaceholder'), ''),
        ...availablePorts.map(port => new Option(port.label, port.device)));
      portSelect.value = availablePorts.some(port => port.device === previous) ? previous : '';
    } catch (error) {
      availablePorts = [];
      portSelect.replaceChildren(new Option(t('feedback.portPlaceholder'), ''));
      portsError = error.message;
    } finally { loadingPorts = false; render(); }
  };
  portSelect.addEventListener('change', render);
  refreshPorts.addEventListener('click', () => { void loadPorts(); });
  connect.addEventListener('click', async () => {
    if (client.token) { await client.disconnect(); return; }
    connecting = true;
    render();
    try { await client.connect(portSelect.value); }
    catch (error) { lastState = { connected: false, reason: error.message,
      angle_error: error.angleError || client.lastAngleError }; }
    finally { connecting = false; render(); }
  });
  const persist = () => {
    try { localStorage.setItem('rebotarm.haptics.v1', JSON.stringify({ settings: client.settings,
      angleControl: client.angleControl, direction: client.gripperDirection })); } catch { /* Optional persistence. */ }
  };
  const valid = (values) => hapticSchema.every(item => Number.isFinite(values[item.key]) &&
    values[item.key] >= item.min && values[item.key] <= item.max && (item.step !== 1 || Number.isInteger(values[item.key]))) &&
    values.contact_off_n < values.contact_on_n && values.force_deadband_n < values.full_scale_n;
  try {
    const stored = JSON.parse(localStorage.getItem('rebotarm.haptics.v1'));
    const values = { ...defaultSettings, ...stored?.settings };
    if (valid(values)) client.settings = Object.fromEntries(hapticSchema.map(item => [item.key, values[item.key]]));
    if (typeof stored?.angleControl === 'boolean') client.angleControl = stored.angleControl;
    if ([-1, 1].includes(stored?.direction)) client.gripperDirection = stored.direction;
  } catch { /* Use validated defaults if stored preferences are unavailable. */ }
  angleControl.checked = client.angleControl;
  reverse.checked = client.gripperDirection === -1;
  for (const item of hapticSchema) {
    if (item.hidden) continue;
    const label = document.createElement('label');
    label.className = 'feedback-scale';
    const name = document.createElement('span');
    const input = document.createElement('input');
    Object.assign(input, { id: `feedback-param-${item.key}`, type: 'number', min: String(item.min),
      max: String(item.max), step: String(item.step), value: String(client.settings[item.key]) });
    label.append(name, input);
    inputs.set(item.key, input);
    root.querySelector(`#feedback-${item.group}`).append(label);
    input.addEventListener('input', () => {
      const candidate = { ...client.settings, [item.key]: input.value.trim() ? Number(input.value) : NaN };
      if (valid(candidate)) {
        client.settings = candidate;
        settingsError = '';
        persist();
        void client.flush();
        input.setAttribute('aria-invalid', 'false');
      } else {
        settingsError = t('feedback.invalid');
        input.setAttribute('aria-invalid', 'true');
      }
      render();
    });
    const commit = () => {
      const candidate = { ...client.settings, [item.key]: input.value.trim() ? Number(input.value) : NaN };
      if (!valid(candidate)) {
        settingsError = t('feedback.invalid');
        input.value = String(client.settings[item.key]);
      } else {
        settingsError = '';
        client.settings = candidate;
        persist();
        void client.flush();
      }
      input.setAttribute('aria-invalid', 'false');
      render();
    };
    input.addEventListener('change', commit);
    input.addEventListener('blur', commit);
  }
  root.querySelector('#feedback-defaults').addEventListener('click', () => {
    client.settings = { ...defaultSettings };
    for (const [key, input] of inputs) {
      input.value = String(client.settings[key]);
      input.setAttribute('aria-invalid', 'false');
    }
    settingsError = '';
    persist();
    void client.flush();
    render();
  });
  angleControl.addEventListener('change', () => { client.angleControl = angleControl.checked; persist(); void client.flush(); });
  reverse.addEventListener('change', () => { client.gripperDirection = reverse.checked ? -1 : 1; persist(); void client.flush(); });
  zero.addEventListener('click', async () => {
    try { await client.calibrate(); }
    catch (error) { await client.disconnect(error.message); }
  });
  const stopTest = () => { client.test = false; test.classList.remove('active'); void client.flush(); };
  test.addEventListener('pointerdown', (event) => {
    if (!client.token) return;
    event.preventDefault();
    test.setPointerCapture(event.pointerId);
    client.test = true;
    test.classList.add('active');
    void client.flush();
  });
  for (const event of ['pointerup', 'pointercancel', 'lostpointercapture', 'blur']) test.addEventListener(event, stopTest);
  test.addEventListener('keydown', (event) => {
    if (client.token && [' ', 'Enter'].includes(event.key)) {
      event.preventDefault(); client.test = true; test.classList.add('active'); void client.flush();
    }
  });
  test.addEventListener('keyup', stopTest);
  const release = () => { stopTest(); void client.disconnect(); };
  window.addEventListener('pagehide', release);
  window.addEventListener('blur', stopTest);
  document.addEventListener('visibilitychange', () => { if (document.hidden) release(); });
  setInterval(() => { void client.flush(); }, 50);
  onLangChange(render);
  render();
  void loadPorts();
  return {
    sample(frame) { client.sample(frame); force.textContent = `${frame.force_n.toFixed(1)} N`; },
    updateGripper() {
      const target = client.gripperTarget();
      onGripperControl(target !== null);
      if (target !== null) onGripperTarget(target);
    },
    reset() { client.sample(0); client.test = false; void client.flush(); }
  };
}
