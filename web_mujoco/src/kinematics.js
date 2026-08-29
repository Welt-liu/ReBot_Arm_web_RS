export const ARM_JOINTS = [
  { name: 'joint1', label: 'J1', min: -2.8, max: 2.8, home: 0, unit: 'rad' },
  { name: 'joint2', label: 'J2', min: 0, max: 3.14, home: 0, unit: 'rad' },
  { name: 'joint3', label: 'J3', min: 0, max: 3.14, home: 0, unit: 'rad' },
  { name: 'joint4', label: 'J4', min: -1.57, max: 1.57, home: 0, unit: 'rad' },
  { name: 'joint5', label: 'J5', min: -1.57, max: 1.57, home: 0, unit: 'rad' },
  { name: 'joint6', label: 'J6', min: -3.14, max: 3.14, home: 0, unit: 'rad' }
];

export const GRIPPER_JOINTS = [
  { name: 'joint7', label: '夹爪', min: 0, max: 0.05, home: 0, unit: 'm' },
  { name: 'joint_left', min: 0, max: 0.05, home: 0, unit: 'm', hidden: true },
  { name: 'joint_right', min: 0, max: 0.05, home: 0, unit: 'm', hidden: true }
];

function qposAddress(mujoco, model, name) {
  const type = mujoco.mjtObj.mjOBJ_JOINT.value;
  const id = mujoco.mj_name2id(model, type, name);
  if (id < 0) throw new Error(`找不到关节 ${name}`);
  return model.jnt_qposadr[id];
}

export function bindJoints(mujoco, model) {
  const entries = [...ARM_JOINTS, ...GRIPPER_JOINTS].map((joint) => ({
    ...joint,
    qposadr: qposAddress(mujoco, model, joint.name)
  }));
  return {
    all: entries,
    visible: entries.filter((joint) => !joint.hidden),
    byName: Object.fromEntries(entries.map((joint) => [joint.name, joint]))
  };
}

export function readAngles(data, joints) {
  const angles = {};
  joints.all.forEach((joint) => {
    angles[joint.name] = data.qpos[joint.qposadr];
  });
  return angles;
}

export function applyKinematicPose(mujoco, model, data, joints, pose) {
  joints.all.forEach((joint) => {
    if (!Number.isFinite(pose[joint.name])) return;
    const value = Math.min(joint.max, Math.max(joint.min, pose[joint.name]));
    data.qpos[joint.qposadr] = value;
  });
  const gripper = pose.joint7;
  if (Number.isFinite(gripper)) {
    const width = Math.min(0.05, Math.max(0, gripper));
    data.qpos[joints.byName.joint7.qposadr] = width;
    data.qpos[joints.byName.joint_left.qposadr] = width;
    data.qpos[joints.byName.joint_right.qposadr] = width;
  }
  mujoco.mj_forward(model, data);
}

export function homePose(joints) {
  const pose = {};
  joints.all.forEach((joint) => {
    pose[joint.name] = joint.home;
  });
  return pose;
}
