#!/usr/bin/env bash
set -Eeuo pipefail

IMAGE_NAME="${IMAGE_NAME:-questarmteleop:humble-jetson}"
WORKSPACE="${WORKSPACE:-/home/yuanyou/VRteleop/QuestArmTeleop}"
PYAGXARM_DIR="${PYAGXARM_DIR:-/home/yuanyou/VRteleop/pyAgxArm}"
ROS_DOMAIN_ID_VALUE="${ROS_DOMAIN_ID:-23}"
LEFT_CAN_PORT="${LEFT_CAN_PORT:-left_piper}"
RIGHT_CAN_PORT="${RIGHT_CAN_PORT:-right_piper}"

if [[ ! -d "${WORKSPACE}" ]]; then
  echo "Workspace not found: ${WORKSPACE}" >&2
  exit 1
fi

if [[ ! -d "${PYAGXARM_DIR}" ]]; then
  echo "pyAgxArm not found: ${PYAGXARM_DIR}" >&2
  exit 1
fi

sudo docker run --rm -it \
  --name questarmteleop-humble \
  --network host \
  --privileged \
  -e ROS_DOMAIN_ID="${ROS_DOMAIN_ID_VALUE}" \
  -e ROS_LOCALHOST_ONLY=0 \
  -e PYTHONUNBUFFERED=1 \
  -v /dev:/dev \
  -v "${WORKSPACE}:/work/QuestArmTeleop" \
  -v "${PYAGXARM_DIR}:/work/pyAgxArm" \
  "${IMAGE_NAME}" \
  bash -lc "
    set -Eeuo pipefail
    source /opt/ros/humble/setup.bash
    python3 -m pip install --no-cache-dir --no-deps -e /work/pyAgxArm
    cd /work/QuestArmTeleop
    colcon build --symlink-install --packages-skip agx_arm_moveit
    source install/setup.bash
    ros2 launch oculus_reader teleop_double_piper.launch.py \
      use_rviz:=false \
      left_can_port:=${LEFT_CAN_PORT} \
      right_can_port:=${RIGHT_CAN_PORT}
  "
