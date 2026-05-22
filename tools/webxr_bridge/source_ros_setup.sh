#!/usr/bin/env bash

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
WORKSPACE_ROOT="${WORKSPACE_ROOT:-$(cd "$SCRIPT_DIR/../.." && pwd)}"

for setup in \
  "$WORKSPACE_ROOT/devel/setup.bash" \
  "$HOME/openpi-robot/questVR_ws/devel/setup.bash" \
  "$HOME/yuanyou2_ws/devel/setup.bash" \
  "$HOME/piper_vr_ws/devel/setup.bash" \
  "/opt/ros/noetic/setup.bash"; do
  if [[ -f "$setup" ]]; then
    # shellcheck disable=SC1090
    source "$setup"
    break
  fi
done
