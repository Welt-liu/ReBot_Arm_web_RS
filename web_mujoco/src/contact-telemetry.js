const MAX_VISIBLE_CONTACTS = 40;

function namedId(mujoco, model, type, name) {
  const id = mujoco.mj_name2id(model, type, name);
  if (id < 0) throw new Error(`找不到 ${name}`);
  return id;
}

export function createContactTelemetry(mujoco, model, data) {
  const bodyType = mujoco.mjtObj.mjOBJ_BODY.value;
  const gripperBody = namedId(mujoco, model, bodyType, 'gripper_end');
  const leftBody = namedId(mujoco, model, bodyType, 'gripper_left');
  const fingerBodies = new Set([
    namedId(mujoco, model, bodyType, 'gripper_left'),
    namedId(mujoco, model, bodyType, 'gripper_right')
  ]);
  const objectBodies = {
    red: namedId(mujoco, model, bodyType, 'red_cube'),
    blue: namedId(mujoco, model, bodyType, 'blue_block'),
    yellow: namedId(mujoco, model, bodyType, 'yellow_cylinder')
  };
  // The WASM binding writes into a MuJoCo-owned buffer, not a JS typed-array
  // copy. Passing Float64Array leaves the sampled contact force at zero.
  const forceBuffer = new mujoco.DoubleBuffer(6);
  const graspableBodies = new Set(Object.values(objectBodies));
  const bodyObject = new Map(Object.entries(objectBodies).map(([id, body]) => [body, id]));

  function sample(selectedId = 'red') {
    const visible = [];
    const perObject = Object.fromEntries(Object.entries(objectBodies).map(([id, body]) => {
      const delta = [0, 1, 2].map(i => data.xpos[body * 3 + i] - data.xpos[gripperBody * 3 + i]);
      const relative = [0, 1, 2].map(i => 1000 * delta.reduce((sum, d, j) =>
        sum + d * data.xmat[gripperBody * 9 + j * 3 + i], 0));
      return [id, { id, left_n: 0, right_n: 0, relative_mm: relative }];
    }));
    const selectedBody = objectBodies[selectedId] ?? -1;
    let graspForce = 0;
    let gripperForce = 0;
    let activeContacts = 0;
    let peakForce = 0;
    const count = Math.min(data.ncon, data.contact.size());

    for (let index = 0; index < count; index += 1) {
      const contact = data.contact.get(index);
      if (!contact) continue;
      forceBuffer.GetView().fill(0);
      mujoco.mj_contactForce(model, data, index, forceBuffer);
      // Any binding call can grow WASM memory and detach an older JS view.
      const normalForce = Math.abs(forceBuffer.GetView()[0]);
      if (!Number.isFinite(normalForce) || normalForce < 0.01) continue;
      activeContacts += 1;
      peakForce = Math.max(peakForce, normalForce);

      const body1 = model.geom_bodyid[contact.geom1];
      const body2 = model.geom_bodyid[contact.geom2];
      const finger = fingerBodies.has(body1) ? body1 : fingerBodies.has(body2) ? body2 : null;
      const object = bodyObject.get(body1) ?? bodyObject.get(body2);
      if (finger !== null && object) perObject[object][finger === leftBody ? 'left_n' : 'right_n'] += normalForce;
      const selectedGrip =
        (fingerBodies.has(body1) && body2 === selectedBody) ||
        (fingerBodies.has(body2) && body1 === selectedBody);
      if (selectedGrip) graspForce += normalForce;
      if ((fingerBodies.has(body1) && graspableBodies.has(body2)) ||
          (fingerBodies.has(body2) && graspableBodies.has(body1))) gripperForce += normalForce;

      if (visible.length < MAX_VISIBLE_CONTACTS) {
        const direction = body2 === selectedBody ? 1 : -1;
        visible.push({
          position: [contact.pos[0], contact.pos[1], contact.pos[2]],
          normal: [
            contact.frame[0] * direction,
            contact.frame[1] * direction,
            contact.frame[2] * direction
          ],
          force: normalForce,
          selectedGrip
        });
      }
    }

    return { contacts: visible, activeContacts, graspForce, gripperForce, peakForce, gripContacts: Object.values(perObject) };
  }

  return { sample, dispose: () => forceBuffer.delete() };
}
