import assert from 'node:assert/strict';
import { WasmForceClient, gripperTargetFromStatus, hapticSchema, defaultSettings, bridgeRequest } from '../src/force-feedback-client.js';
import { createContactTelemetry } from '../src/contact-telemetry.js';

const calls = [];
assert.equal(defaultSettings.wall_hold_raw, 35);
assert.equal(defaultSettings.release_angle_deg, 0.35);
assert.equal(defaultSettings.rearm_angle_deg, 0.35);
assert.equal(defaultSettings.opening_fraction, 0);
assert(hapticSchema.find(item => item.key === 'opening_fraction').hidden);
assert(hapticSchema.find(item => item.key === 'slip_pulse_raw').hidden);
const request = async (action, body) => {
  calls.push({ action, body });
  return action === 'connect' ? { token: 'session', connected: true, raw: 0 }
    : { connected: true, raw: 50 };
};
const client = new WasmForceClient({ request });
await assert.rejects(client.connect(), /Select a serial port/);
assert.equal(calls.length, 0);
await client.connect('mock://hls3604m');
assert.equal(calls.at(-1).body.port, 'mock://hls3604m');
client.sample(10.25, 1000, 100);
await client.flush(101);
assert.equal(calls.at(-1).action, 'sample');
assert.equal(calls.at(-1).body.force_n, 10.25);
assert.equal(calls.at(-1).body.at_ms, 1000);
assert.equal(calls.at(-1).body.full_scale_n, 20);
assert.equal(calls.at(-1).body.angle_control, true);
assert.equal(calls.at(-1).body.gripper_direction, 1);
assert.equal(calls.at(-1).body.settings.resistance_speed, 2);
assert.equal(calls.at(-1).body.settings.wall_hold_raw, 35);
assert.equal(calls.at(-1).body.settings.release_angle_deg, 0.35);
client.test = true;
await client.flush(102);
assert.equal(calls.at(-1).body.test, true);
assert.equal(calls.at(-1).body.seq, 2);
await client.flush(350);
assert.equal(calls.at(-1).action, 'disconnect');
assert.equal(client.token, null);

const measured = { connected: true, angle_control: true, angle_age_s: 0.02, gripper_mm: 20 };
assert.equal(gripperTargetFromStatus(measured), 0.020);
assert.equal(gripperTargetFromStatus({ ...measured, gripper_mm: 80 }), 0.05);
assert.equal(gripperTargetFromStatus({ ...measured, gripper_mm: -10 }), 0);
for (const override of [{ connected: false }, { angle_control: false }, { angle_age_s: 0.31 }, { gripper_mm: NaN }]) {
  assert.equal(gripperTargetFromStatus({ ...measured, ...override }), null);
}
assert.equal(gripperTargetFromStatus(measured, false), null);
const motionCalls = [];
const motion = new WasmForceClient({ request: async (action, body) => {
  motionCalls.push({ action, body });
  return { ...measured, token: 'angle-session', gripper_mm: action === 'calibrate' ? 0 : 20 };
} });
await motion.connect('mock://hls3604m');
motion.sample(0);
await motion.flush();
assert.equal(motion.gripperTarget(), 0.020);
assert.equal(motion.gripperTarget(motion.stateAt + 201), null);
await motion.calibrate();
assert.equal(motionCalls.at(-1).action, 'calibrate');
assert.equal(motion.gripperTarget(), 0);
motion.angleControl = false;
assert.equal(motion.gripperTarget(), null);
await motion.disconnect();
assert.equal(motion.gripperTarget(), null);

let completeConnect;
const raceCalls = [];
const race = new WasmForceClient({ request: (action, body) => {
  raceCalls.push({ action, body });
  return action === 'connect' ? new Promise((resolve) => { completeConnect = resolve; }) : Promise.resolve({});
} });
const connecting = race.connect('mock://hls3604m');
await race.disconnect();
completeConnect({ token: 'late', connected: true });
await connecting;
assert.equal(race.token, null);
assert.equal(raceCalls.at(-1).body.token, 'late');

const bodies = { gripper_end: 9, gripper_left: 10, gripper_right: 11, red_cube: 20, blue_block: 21, yellow_cylinder: 22 };
const contacts = [
  { geom1: 0, geom2: 2 }, { geom1: 1, geom2: 3 },
  { geom1: 4, geom2: 5 }, { geom1: 2, geom2: 3 }
].map((entry) => ({ ...entry, pos: [0, 0, 0], frame: [1, 0, 0] }));
const telemetry = createContactTelemetry({
  DoubleBuffer: class { constructor(n) { this.values = new Float64Array(n); } GetView() { return this.values; } delete() {} },
  mjtObj: { mjOBJ_BODY: { value: 1 } },
  mj_name2id: (_model, _type, name) => bodies[name],
  mj_contactForce: (_model, _data, index, force) => { force.GetView()[0] = [5, 7, 100, 40][index]; }
}, { geom_bodyid: [10, 11, 20, 21, 22, 0] }, {
  xpos: new Float64Array(69), xmat: new Float64Array(207).map((_, i) => i % 9 % 4 === 0 ? 1 : 0),
  ncon: contacts.length, contact: { size: () => contacts.length, get: (index) => contacts[index] }
});
assert.equal(telemetry.sample('red').graspForce, 5);
assert.equal(telemetry.sample('red').gripperForce, 12);
assert.equal(telemetry.sample('yellow').gripperForce, 12);
assert.equal(telemetry.sample('yellow').graspForce, 0);
assert.equal(telemetry.sample('red').gripContacts.find(c => c.id === 'red').left_n, 5);
assert.equal(telemetry.sample('red').gripContacts.find(c => c.id === 'blue').right_n, 7);
assert.deepEqual(telemetry.sample('red').gripContacts[0].relative_mm, [0, 0, 0]);
const rich = new WasmForceClient({ request });
rich.settings.stiffness = 12;
await rich.connect('mock://hls3604m');
rich.sample({ force_n: 8, grip_contacts: [], actual_width_mm: 25, target_width_mm: 24 });
await rich.flush();
assert.equal(calls.at(-1).body.settings.stiffness, 12);
assert.deepEqual(calls.at(-1).body.grip_contacts, []);
assert.equal(calls.at(-1).body.actual_width_mm, 25);
rich.settings.max_raw = 40;
rich.settings.resistance_speed = 3;
await rich.flush();
assert.equal(calls.at(-1).body.settings.max_raw, 40);
assert.equal(calls.at(-1).body.settings.resistance_speed, 3);
const angleDiagnostic = { status_available: true, status_raw: 8, status_flags: 8,
  original_error: 'angle timeout', torque_enable: 0, temperature_c: 72 };
const savedFetch = globalThis.fetch;
try {
  globalThis.fetch = async () => ({ ok: false, json: async () => ({ error: 'angle timeout', angle_error: angleDiagnostic }) });
  await assert.rejects(bridgeRequest('sample', {}), error => error.message === 'angle timeout' && error.angleError === angleDiagnostic);
} finally { globalThis.fetch = savedFetch; }
const diagnosticClient = new WasmForceClient({ request: async (action) => {
  if (action === 'connect') return { token: 'fault-session', connected: true, raw: 0 };
  if (action === 'sample') {
    const error = new Error('angle timeout');
    error.angleError = angleDiagnostic;
    throw error;
  }
  return {};
} });
await diagnosticClient.connect('mock://hls3604m');
diagnosticClient.sample(0);
await diagnosticClient.flush();
assert.equal(diagnosticClient.state.connected, false);
assert.deepEqual(diagnosticClient.state.angle_error, angleDiagnostic);
await diagnosticClient.connect('mock://hls3604m');
assert.deepEqual(diagnosticClient.state.angle_error, angleDiagnostic);
await diagnosticClient.disconnect();
assert.deepEqual(diagnosticClient.state.angle_error, angleDiagnostic);
console.log('WASM feedback: contact forwarding, 1°:0.5mm gripper targets, zeroing, bounds, stale/disconnected control and session races passed.');
