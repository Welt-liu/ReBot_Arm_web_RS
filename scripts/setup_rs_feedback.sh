#!/usr/bin/env bash
set -euo pipefail
REBOTARM_TASK_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
source "${REBOTARM_TASK_ROOT}/scripts/rs_env.sh"
SDK_ROOT="${REBOTARM_TASK_ROOT}/.deps/FTServo_Python"
SDK_REV=9e6e342f6011dc727ba7144fc835bca5357b3ca9
if [[ ! -d "${SDK_ROOT}" ]]; then
  git clone https://github.com/ftservo/FTServo_Python.git "${SDK_ROOT}"
  git -C "${SDK_ROOT}" checkout --detach "${SDK_REV}"
fi
if [[ "$(git -C "${SDK_ROOT}" rev-parse HEAD)" != "${SDK_REV}" ]] ||
   [[ -n "$(git -C "${SDK_ROOT}" status --porcelain)" ]]; then
  echo "SDK must be clean and at revision ${SDK_REV}: ${SDK_ROOT}" >&2
  exit 1
fi
python -m pip install -r "${REBOTARM_TASK_ROOT}/requirements-rs-feedback.txt"
cd "${REBOTARM_TASK_ROOT}/rebotarm_ros2_RS"
colcon build --symlink-install --packages-select rebotarm_msgs rebotarmcontroller
echo 'HLS3604M feedback ready. Use scripts/start_rs_feedback.sh with port and servo_id.'
