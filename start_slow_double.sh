#!/usr/bin/env bash
set -euo pipefail

cd "$(dirname "$0")"

POSITION_SCALE="${POSITION_SCALE:-1.6}"
ORIENTATION_SCALE="${ORIENTATION_SCALE:-0.75}"
RIGHT_ORIENTATION_SCALE="${RIGHT_ORIENTATION_SCALE:-1.3}"
LEFT_ORIENTATION_SCALE="${LEFT_ORIENTATION_SCALE:-1.3}"
NEAR_BASE_POSITION_PRIORITY="${NEAR_BASE_POSITION_PRIORITY:-true}"
NEAR_BASE_X_THRESHOLD="${NEAR_BASE_X_THRESHOLD:-0.38}"
NEAR_BASE_DOWN_DELTA="${NEAR_BASE_DOWN_DELTA:--0.001}"
NEAR_BASE_ORIENTATION_SCALE="${NEAR_BASE_ORIENTATION_SCALE:-0.60}"
NEAR_BASE_ORIENTATION_WEIGHT="${NEAR_BASE_ORIENTATION_WEIGHT:-0.16}"
NEAR_BASE_REF_BLEND="${NEAR_BASE_REF_BLEND:-0.65}"
NEAR_BASE_JOINT_REFERENCE="${NEAR_BASE_JOINT_REFERENCE:-[0.0, 0.65, -1.25, 0.0, 0.05, 0.0]}"
COMMAND_HZ="${COMMAND_HZ:-18.0}"
JOINT_COMMAND_HZ="${JOINT_COMMAND_HZ:-18.0}"
DRIVER_COMMAND_HZ="${DRIVER_COMMAND_HZ:-20.0}"
CONTROL_SPEED="${CONTROL_SPEED:-16}"
MAX_JOINT_STEP_DEG="${MAX_JOINT_STEP_DEG:-4.0}"
INIT_DURATION="${INIT_DURATION:-4.5}"
DEBUG_JOINT3="${DEBUG_JOINT3:-false}"
DEBUG_BUTTONS="${DEBUG_BUTTONS:-false}"
DEBUG_POSE="${DEBUG_POSE:-false}"
JOINT_REFERENCE="${JOINT_REFERENCE:-[0.0, 0.35, -0.75, 0.0, 0.05, 0.0]}"
IK_FILTER_ALPHA="${IK_FILTER_ALPHA:-0.4}"
IK_DEADBAND="${IK_DEADBAND:-0.003}"
IK_POSITION_COST_WEIGHT="${IK_POSITION_COST_WEIGHT:-180.0}"
IK_ORIENTATION_WEIGHT="${IK_ORIENTATION_WEIGHT:-0.45}"
IK_REGULARIZATION_WEIGHT="${IK_REGULARIZATION_WEIGHT:-0.03}"
IK_REGULARIZATION_WEIGHTS="${IK_REGULARIZATION_WEIGHTS:-[0.001, 0.001, 0.0004, 0.012, 0.012, 0.012]}"
COMMAND_DEADBAND_DEG="${COMMAND_DEADBAND_DEG:-0.08}"
IK_MAX_SOLUTION_JUMP_DEG="${IK_MAX_SOLUTION_JUMP_DEG:-55.0}"
IK_MAX_WRIST_JUMP_DEG="${IK_MAX_WRIST_JUMP_DEG:-45.0}"
JOINT5_SOFT_ABS_LIMIT_DEG="${JOINT5_SOFT_ABS_LIMIT_DEG:-65.0}"
JOINT6_IK_ABS_LIMIT_DEG="${JOINT6_IK_ABS_LIMIT_DEG:-180.0}"
MAX_DELTA_POSITION="${MAX_DELTA_POSITION:-0.035}"
MAX_DELTA_RPY_DEG="${MAX_DELTA_RPY_DEG:-14.0}"
RPY_LIMIT="${RPY_LIMIT:-2.2}"
ENABLE_GRIP_THRESHOLD="${ENABLE_GRIP_THRESHOLD:-0.25}"
RESET_HOLD_SEC="${RESET_HOLD_SEC:-0.0}"
AUTO_INIT_ON_START="${AUTO_INIT_ON_START:-false}"
CONTROL_MODE="${CONTROL_MODE:-delta_ik}"
ENABLE_COLLISION_CHECK="${ENABLE_COLLISION_CHECK:-false}"
COLLISION_LOG_PERIOD="${COLLISION_LOG_PERIOD:-2.0}"
POSITION_AXIS_MAP="${POSITION_AXIS_MAP:-[0, 1, 2]}"
POSITION_AXIS_SIGN="${POSITION_AXIS_SIGN:-[1.0, 1.0, 1.0]}"
RIGHT_POSITION_AXIS_SIGN="${RIGHT_POSITION_AXIS_SIGN:-[1.0, 1.0, 1.0]}"
LEFT_POSITION_AXIS_SIGN="${LEFT_POSITION_AXIS_SIGN:-[1.0, 1.0, 1.0]}"
POSITION_AXIS_SCALE="${POSITION_AXIS_SCALE:-[1.0, 2.8, 1.0]}"
RIGHT_POSITION_AXIS_SCALE="${RIGHT_POSITION_AXIS_SCALE:-[1.3, 0.7, 1.0]}"
LEFT_POSITION_AXIS_SCALE="${LEFT_POSITION_AXIS_SCALE:-[1.3, 0.7, 1.0]}"
ORIENTATION_AXIS_MAP="${ORIENTATION_AXIS_MAP:-[0, 1, 2]}"
ORIENTATION_AXIS_SIGN="${ORIENTATION_AXIS_SIGN:-[1.0, 1.0, 1.0]}"
RIGHT_ORIENTATION_AXIS_SIGN="${RIGHT_ORIENTATION_AXIS_SIGN:-[1.0, -1.0, 1.0]}"
LEFT_ORIENTATION_AXIS_SIGN="${LEFT_ORIENTATION_AXIS_SIGN:-[1.0, -1.0, 1.0]}"
WORKSPACE_MIN="${WORKSPACE_MIN:-[0.08, -0.35, -0.10]}"
WORKSPACE_MAX="${WORKSPACE_MAX:-[0.58, 0.35, 0.55]}"

echo "[1/5] Stopping stale questVR roslaunch processes..."
pkill -f "roslaunch oculus_reader teleop_double_piper.launch" 2>/dev/null || true
pkill -f "rosmaster --core" 2>/dev/null || true
sleep 1

echo "[2/5] Configuring CAN interfaces..."
(
  cd src/Piper_ros
  bash can_config.sh
)

echo "[3/5] Checking CAN interfaces..."
ip -br link show type can
if ! ip -br link show type can | grep -q '^left_piper[[:space:]]'; then
  echo "ERROR: left_piper CAN interface does not exist. Run src/Piper_ros/can_config.sh and check USB-CAN cables."
  exit 1
fi
if ! ip -br link show type can | grep -q '^right_piper[[:space:]]'; then
  echo "ERROR: right_piper CAN interface does not exist. Run src/Piper_ros/can_config.sh and check USB-CAN cables."
  exit 1
fi

echo "[4/5] Checking Quest ADB access..."
ADB_DEVICES_OUTPUT=""
for attempt in 1 2 3; do
  adb kill-server >/dev/null 2>&1 || true
  pkill -9 adb 2>/dev/null || true
  sleep 1
  if ! adb start-server >/tmp/questvr_adb_start.log 2>&1; then
    cat /tmp/questvr_adb_start.log
    echo "WARN: adb start-server failed, retrying ($attempt/3)..."
    continue
  fi
  if ADB_DEVICES_OUTPUT="$(adb devices -l 2>&1)"; then
    echo "$ADB_DEVICES_OUTPUT"
    break
  fi
  echo "$ADB_DEVICES_OUTPUT"
  echo "WARN: adb devices failed, retrying ($attempt/3)..."
done

if echo "$ADB_DEVICES_OUTPUT" | grep -q "no permissions"; then
  cat <<'MSG'
ERROR: Quest is visible but Linux has no ADB permission.
Fix the Quest udev rule, then unplug/replug the Quest USB cable.
MSG
  exit 1
fi
if ! echo "$ADB_DEVICES_OUTPUT" | awk 'NR > 1 && $2 == "device" { found=1 } END { exit found ? 0 : 1 }'; then
  cat <<'MSG'
ERROR: Quest is not authorized as an ADB device.
Wear the headset and allow USB debugging. Then run this script again.
MSG
  exit 1
fi

echo "[5/5] Launching slow double-arm teleop..."
source ~/miniconda3/etc/profile.d/conda.sh
conda activate vt
source /opt/ros/noetic/setup.bash
source devel/setup.bash

exec roslaunch oculus_reader teleop_double_piper.launch \
  position_scale:="$POSITION_SCALE" \
  orientation_scale:="$ORIENTATION_SCALE" \
  right_orientation_scale:="$RIGHT_ORIENTATION_SCALE" \
  left_orientation_scale:="$LEFT_ORIENTATION_SCALE" \
  near_base_position_priority:="$NEAR_BASE_POSITION_PRIORITY" \
  near_base_x_threshold:="$NEAR_BASE_X_THRESHOLD" \
  near_base_down_delta:="$NEAR_BASE_DOWN_DELTA" \
  near_base_orientation_scale:="$NEAR_BASE_ORIENTATION_SCALE" \
  near_base_orientation_weight:="$NEAR_BASE_ORIENTATION_WEIGHT" \
  near_base_ref_blend:="$NEAR_BASE_REF_BLEND" \
  near_base_joint_reference:="$NEAR_BASE_JOINT_REFERENCE" \
  command_hz:="$COMMAND_HZ" \
  joint_command_hz:="$JOINT_COMMAND_HZ" \
  driver_command_hz:="$DRIVER_COMMAND_HZ" \
  control_speed:="$CONTROL_SPEED" \
  max_joint_step_deg:="$MAX_JOINT_STEP_DEG" \
  init_duration:="$INIT_DURATION" \
  debug_joint3:="$DEBUG_JOINT3" \
  debug_buttons:="$DEBUG_BUTTONS" \
  debug_pose:="$DEBUG_POSE" \
  joint_reference:="$JOINT_REFERENCE" \
  ik_filter_alpha:="$IK_FILTER_ALPHA" \
  ik_deadband:="$IK_DEADBAND" \
  ik_position_cost_weight:="$IK_POSITION_COST_WEIGHT" \
  ik_orientation_weight:="$IK_ORIENTATION_WEIGHT" \
  ik_regularization_weight:="$IK_REGULARIZATION_WEIGHT" \
  ik_regularization_weights:="$IK_REGULARIZATION_WEIGHTS" \
  command_deadband_deg:="$COMMAND_DEADBAND_DEG" \
  ik_max_solution_jump_deg:="$IK_MAX_SOLUTION_JUMP_DEG" \
  ik_max_wrist_jump_deg:="$IK_MAX_WRIST_JUMP_DEG" \
  joint5_soft_abs_limit_deg:="$JOINT5_SOFT_ABS_LIMIT_DEG" \
  joint6_ik_abs_limit_deg:="$JOINT6_IK_ABS_LIMIT_DEG" \
  max_delta_position:="$MAX_DELTA_POSITION" \
  max_delta_rpy_deg:="$MAX_DELTA_RPY_DEG" \
  rpy_limit:="$RPY_LIMIT" \
  enable_grip_threshold:="$ENABLE_GRIP_THRESHOLD" \
  reset_hold_sec:="$RESET_HOLD_SEC" \
  auto_init_on_start:="$AUTO_INIT_ON_START" \
  control_mode:="$CONTROL_MODE" \
  enable_collision_check:="$ENABLE_COLLISION_CHECK" \
  collision_log_period:="$COLLISION_LOG_PERIOD" \
  position_axis_map:="$POSITION_AXIS_MAP" \
  position_axis_sign:="$POSITION_AXIS_SIGN" \
  right_position_axis_sign:="$RIGHT_POSITION_AXIS_SIGN" \
  left_position_axis_sign:="$LEFT_POSITION_AXIS_SIGN" \
  position_axis_scale:="$POSITION_AXIS_SCALE" \
  right_position_axis_scale:="$RIGHT_POSITION_AXIS_SCALE" \
  left_position_axis_scale:="$LEFT_POSITION_AXIS_SCALE" \
  orientation_axis_map:="$ORIENTATION_AXIS_MAP" \
  orientation_axis_sign:="$ORIENTATION_AXIS_SIGN" \
  right_orientation_axis_sign:="$RIGHT_ORIENTATION_AXIS_SIGN" \
  left_orientation_axis_sign:="$LEFT_ORIENTATION_AXIS_SIGN" \
  workspace_min:="$WORKSPACE_MIN" \
  workspace_max:="$WORKSPACE_MAX"
