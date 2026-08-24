# rebotarm_mujoco_rs

ROS 2 integration for the B601-RS MuJoCo model. The required model XML and STL
meshes are tracked directly in `models/`, based on
`LAN-GER/reBot-B601-RS-for-mujoco_sim` revision
`1249cb6efdf393ba636056fc41df30dc6ba389aa` with local gripper, material, and
Seeed-badge updates. The upstream repository had no LICENSE file at that
revision.

Only model assets are used. This package does not import the upstream
`rebot_b601_rs_sim` algorithms, QP solver, examples, or tests. ROS integration,
scene detection, IK, and task execution are implemented in this package.

The wrapper does not open SocketCAN or send hardware commands. It subscribes to
ROS `JointState`, so the real arm continues to have a single owner: the
`reBotArmController` node.

The RS package now also includes a physics grasp environment with red, blue,
and yellow objects, an overhead ROS camera, object detections, Cartesian IK,
trajectory actions, and task recording services used by `rebotarm_agent`.

Modes:

- `kinematic`: directly synchronizes ROS joint state into MuJoCo.
- `physics`: tracks ROS targets with conservative PD plus MuJoCo bias forces.

```bash
ros2 launch rebotarm_mujoco_rs mujoco_rs.launch.py \
  arm_namespace:=rebotarm_rs simulation_mode:=physics use_viewer:=true
```

Important topics:

- `/rebotarm_rs/mujoco/object_states`
- `/rebotarm_rs/mujoco/overhead_rgb/image_raw`
- `/rebotarm_rs/vision/color_blocks/detections`

Task endpoints:

- `/rebotarm_rs/move_to_pose_ik`
- `/rebotarm_rs/move_to_pose`
- `/rebotarm_rs/follow_joint_trajectory`
