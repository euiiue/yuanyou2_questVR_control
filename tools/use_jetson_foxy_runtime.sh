#!/usr/bin/env bash
# Source this before running QuestArmTeleop on Jetson Ubuntu 20.04 + ROS 2 Foxy.
# It keeps Python user-site packages disabled while adding the workspace-local
# runtime dependencies used by arm_ik_pose_node and agx_arm_ctrl.

set -e

QUESTARM_HOME="${QUESTARM_HOME:-/home/yuanyou/QuestArmTeleop}"
ROS_DISTRO_TARGET="${ROS_DISTRO_TARGET:-foxy}"
ROS_SETUP="/opt/ros/${ROS_DISTRO_TARGET}/setup.bash"
LOCAL_PY_DEPS="${QUESTARM_HOME}/.python_deps"
LOCAL_ROS_OVERLAY="${QUESTARM_HOME}/.ros_foxy_pin_overlay/opt/ros/${ROS_DISTRO_TARGET}"

if [[ ! -f "${ROS_SETUP}" ]]; then
  echo "ROS setup not found: ${ROS_SETUP}" >&2
  return 1 2>/dev/null || exit 1
fi

export PYTHONNOUSERSITE=1
source "${ROS_SETUP}"

prepend_path() {
  local name="$1"
  local value="$2"
  [[ -d "${value}" ]] || return 0
  if [[ -n "${!name:-}" ]]; then
    export "${name}=${value}:${!name}"
  else
    export "${name}=${value}"
  fi
}

prepend_path PYTHONPATH "${LOCAL_PY_DEPS}"
prepend_path PYTHONPATH "${LOCAL_ROS_OVERLAY}/lib/python3.8/site-packages"
prepend_path PYTHONPATH "/usr/lib/python3/dist-packages"

prepend_path LD_LIBRARY_PATH "${LOCAL_ROS_OVERLAY}/lib"
prepend_path LD_LIBRARY_PATH "${LOCAL_ROS_OVERLAY}/lib/aarch64-linux-gnu"

if [[ -f "${QUESTARM_HOME}/install/setup.bash" ]]; then
  source "${QUESTARM_HOME}/install/setup.bash"
fi
