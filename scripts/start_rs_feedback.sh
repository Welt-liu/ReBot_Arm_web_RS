#!/usr/bin/env bash
set -euo pipefail
REBOTARM_TASK_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
source "${REBOTARM_TASK_ROOT}/scripts/rs_env.sh"
SDK_ROOT="${REBOTARM_TASK_ROOT}/.deps/FTServo_Python"
if [[ ! -f "${SDK_ROOT}/scservo_sdk/hls.py" ]]; then
  echo 'Run bash scripts/setup_rs_feedback.sh first.' >&2
  exit 1
fi
export PYTHONPATH="${SDK_ROOT}${PYTHONPATH:+:${PYTHONPATH}}"
exec python -m rebotarmcontroller.feetech_feedback_node --ros-args "$@"
