import schema from '../../rebotarm_ros2_RS/src/rebotarmcontroller/rebotarmcontroller/haptic_settings.json' with { type: 'json' };
export const hapticSchema = schema;
export const defaultSettings = Object.fromEntries(schema.map(item => [item.key, item.default]));

export async function bridgeRequest(action, body) {
  const controller = new AbortController();
  const timer = setTimeout(() => controller.abort(), ['connect', 'ports'].includes(action) ? 5000 : 600);
  try {
    const response = await fetch(`/feetech/${action}`, {
      method: action === 'ports' ? 'GET' : 'POST', headers: { 'Content-Type': 'application/json' },
      body: action === 'ports' ? undefined : JSON.stringify(body), signal: controller.signal, keepalive: true, cache: 'no-store'
    });
    const result = await response.json();
    if (!response.ok || result.fault) {
      const error = new Error(result.error || result.fault || 'Feedback bridge unavailable');
      error.angleError = result.angle_error;
      throw error;
    }
    return result;
  } finally { clearTimeout(timer); }
}

export function gripperTargetFromStatus(state, enabled = true) {
  if (!enabled || !state?.connected || !state.angle_control || state.fault ||
      !Number.isFinite(state.angle_age_s) || state.angle_age_s < 0 || state.angle_age_s >= 0.3 ||
      !Number.isFinite(state.gripper_mm)) return null;
  return Math.min(50, Math.max(0, state.gripper_mm)) / 1000;
}

/** Two-way WASM gripper contacts and HLS angle feedback; no ROS or arm mapping. */
export class WasmForceClient {
  constructor({ request = bridgeRequest, onStatus = () => {} } = {}) {
    this.request = request;
    this.onStatus = onStatus;
    this.token = null;
    this.generation = 0;
    this.sequence = 0;
    this.busy = false;
    this.test = false;
    this.latest = null;
    this.settings = { ...defaultSettings };
    this.angleControl = true;
    this.gripperDirection = 1;
    this.state = null;
    this.stateAt = -Infinity;
    this.pendingCalibration = false;
    this.lastAngleError = null;
  }

  acceptState(state) {
    if (state.angle_error && typeof state.angle_error === 'object') this.lastAngleError = state.angle_error;
    this.state = { ...state, angle_error: this.lastAngleError };
    this.stateAt = performance.now();
    this.onStatus(this.state);
  }

  gripperTarget(now = performance.now()) {
    if (!this.token || now - this.stateAt > 200) return null;
    return gripperTargetFromStatus(this.state, this.angleControl);
  }

  async calibrate() {
    if (!this.token) return;
    this.pendingCalibration = true;
    await this.flush();
  }

  async connect(port) {
    if (typeof port !== 'string' || !port.trim()) throw new Error('Select a serial port before connecting');
    const generation = ++this.generation;
    let state;
    try { state = await this.request('connect', { port, settings: { ...this.settings } }); }
    catch (error) {
      if (error.angleError) this.lastAngleError = error.angleError;
      throw error;
    }
    if (generation !== this.generation) {
      void this.request('disconnect', { token: state.token }).catch(() => {});
      return;
    }
    this.token = state.token;
    this.sequence = 0;
    this.busy = false;
    this.pendingCalibration = false;
    this.acceptState(state);
  }

  async disconnect(reason = '') {
    const token = this.token;
    this.token = null;
    this.test = false;
    this.pendingCalibration = false;
    this.generation += 1;
    this.acceptState({ connected: false, raw: 0, reason });
    if (token) {
      try { await this.request('disconnect', { token }); } catch { /* Watchdog also releases. */ }
    }
  }

  get fullScale() { return this.settings.full_scale_n; }
  set fullScale(value) { this.settings.full_scale_n = value; }

  sample(frame, atMs = Date.now(), now = performance.now()) {
    const forceN = typeof frame === 'number' ? frame : frame.force_n;
    this.latest = { frame: typeof frame === 'object' ? frame : {}, forceN: Number.isFinite(forceN) && forceN >= 0 ? forceN : 0, atMs, now };
  }

  async flush(now = performance.now()) {
    if (!this.token || this.busy) return;
    if (!this.latest || now - this.latest.now > 200) {
      await this.disconnect('simulation samples stopped');
      return;
    }
    const generation = this.generation;
    this.busy = true;
    try {
      const calibrating = this.pendingCalibration;
      this.pendingCalibration = false;
      const state = await this.request(calibrating ? 'calibrate' : 'sample', calibrating ? { token: this.token } : {
        token: this.token, seq: ++this.sequence, at_ms: this.latest.atMs,
        ...this.latest.frame, force_n: this.latest.forceN, settings: { ...this.settings },
        full_scale_n: this.fullScale, deadband_n: this.settings.force_deadband_n, test: this.test,
        angle_control: this.angleControl, gripper_direction: this.gripperDirection
      });
      if (generation === this.generation) this.acceptState(state);
    } catch (error) {
      if (generation === this.generation) {
        if (error.angleError) this.lastAngleError = error.angleError;
        await this.disconnect(error.message);
      }
    } finally {
      if (generation === this.generation) this.busy = false;
    }
  }
}
