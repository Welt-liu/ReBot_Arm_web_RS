#!/usr/bin/env bash
# Browser WASM feedback entry point. ROS is optional and explicitly selected.
set -eo pipefail
REBOTARM_FEEDBACK_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
REBOTARM_FEEDBACK_ACTION="${1:-start}"
if (($#)); then shift; fi
REBOTARM_FEEDBACK_CONDA="${CONDA_EXE:-$(type -P conda || true)}"
if [[ ! -x "${REBOTARM_FEEDBACK_CONDA}" ]]; then
  for candidate in "${HOME}/miniforge3/bin/conda" "${HOME}/miniconda3/bin/conda"; do
    if [[ -x "${candidate}" ]]; then REBOTARM_FEEDBACK_CONDA="${candidate}"; break; fi
  done
fi
if [[ ! -x "${REBOTARM_FEEDBACK_CONDA}" ]]; then
  echo 'Conda not found. Set CONDA_EXE to its executable path.' >&2
  exit 1
fi
REBOTARM_FEEDBACK_CONDA_BASE="$("${REBOTARM_FEEDBACK_CONDA}" info --base)"
source "${REBOTARM_FEEDBACK_CONDA_BASE}/etc/profile.d/conda.sh"
conda activate rebotarm-feedback
export PYTHONPATH="${REBOTARM_FEEDBACK_ROOT}/rebotarm_ros2_RS/src/rebotarmcontroller:${REBOTARM_FEEDBACK_ROOT}/.deps/FTServo_Python${PYTHONPATH:+:${PYTHONPATH}}"

case "${REBOTARM_FEEDBACK_ACTION}" in
  check)
    exec python -c 'import sys, serial; from scservo_sdk.hls import hls; from rebotarmcontroller.feetech_wasm_bridge import WasmFeedback; print("Conda: rebotarm-feedback\nPython: " + sys.version.split()[0] + "\nWASM serial bridge and HLS SDK: OK (no ROS required)")'
    ;;
  test)
    # These unit/wire tests need no external pytest plugins. ROS launch-test
    # entry points otherwise pull unrelated launch dependencies into this env.
    export PYTEST_DISABLE_PLUGIN_AUTOLOAD=1
    exec python -m pytest -q \
      "${REBOTARM_FEEDBACK_ROOT}/rebotarm_ros2_RS/src/rebotarmcontroller/test/test_feetech_feedback.py" \
      "${REBOTARM_FEEDBACK_ROOT}/rebotarm_ros2_RS/src/rebotarmcontroller/test/test_feetech_feedback_node.py" \
      "${REBOTARM_FEEDBACK_ROOT}/rebotarm_ros2_RS/src/rebotarmcontroller/test/test_feetech_wasm_bridge.py" \
      "${REBOTARM_FEEDBACK_ROOT}/rebotarm_ros2_RS/src/rebotarmcontroller/test/test_feetech_sdk_wire.py" \
      "${REBOTARM_FEEDBACK_ROOT}/rebotarm_ros2_RS/src/rebotarmcontroller/test/test_grip_haptics.py"
    ;;
  start|probe|ros) ;;
  *) echo "Usage: bash scripts/feedback_conda.sh [start|probe|check|test|ros] [extra arguments]" >&2; exit 2 ;;
esac

if [[ "${REBOTARM_FEEDBACK_ACTION}" == start ]]; then
  exec python -m rebotarmcontroller.feetech_wasm_bridge \
    --servo-id "${FEETECH_SERVO_ID:-1}" \
    --baudrate "${FEETECH_BAUDRATE:-1000000}" \
    --web-root "${REBOTARM_FEEDBACK_ROOT}/web_mujoco/dist" "$@"
fi

if [[ -z "${FEETECH_PORT:-}" && -f "${REBOTARM_FEEDBACK_ROOT}/.deps/feetech_port" ]]; then
  IFS= read -r FEETECH_PORT < "${REBOTARM_FEEDBACK_ROOT}/.deps/feetech_port"
fi
if [[ -z "${FEETECH_PORT:-}" ]]; then
  echo 'Set FEETECH_PORT to the USB serial device path.' >&2
  exit 1
fi
if [[ "${REBOTARM_FEEDBACK_ACTION}" == probe ]]; then
  exec python -m rebotarmcontroller.feetech_probe --port "${FEETECH_PORT}" "$@"
fi
# Legacy ROS path is available only through the explicit `ros` action.
source /opt/ros/humble/setup.bash
source "${CONDA_PREFIX}/ros_ws/install/setup.bash"
exec python -m rebotarmcontroller.feetech_feedback_node --ros-args \
  -p "port:=${FEETECH_PORT}" \
  -p "servo_id:=${FEETECH_SERVO_ID:-1}" \
  -p "baudrate:=${FEETECH_BAUDRATE:-1000000}" \
  -p enabled:=true "$@"
