# Vendored control SDK

This directory is ordinary source tracked by the parent
`ReBot_Arm_web_RS` repository. It is intentionally **not** a Git submodule or
nested repository.

| Directory | Upstream | Baseline revision |
|---|---|---|
| `reBotArm_control_py` | `vectorBH6/reBotArm_control_py` | `40ab6ce58fec3c58cb603efb3f30240d6f5849e4` |
The SDK includes the RS Cartesian trajectory duration safety adjustment. The
MuJoCo model derived from `LAN-GER/reBot-B601-RS-for-mujoco_sim` revision
`1249cb6efdf393ba636056fc41df30dc6ba389aa` is integrated directly under
`rebotarm_ros2_RS/src/rebotarm_mujoco_rs/models`; its algorithm package is not used.
These integrated files are the single source of truth; no patch or override copy
is required.

Do not run `git init` or clone another repository inside this directory. Update
vendored files through the parent repository so one normal clone contains the
complete build inputs.
