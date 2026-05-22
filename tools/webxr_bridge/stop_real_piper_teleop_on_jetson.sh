#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
cd "$SCRIPT_DIR"
source "$SCRIPT_DIR/source_ros_setup.sh"

if rostopic list >/dev/null 2>&1; then
  echo "[1/4] Disabling Quest adapter output..."
  rostopic pub -1 /quest_vr/piper_enable std_msgs/Bool "data: false" >/dev/null 2>&1 || true

  echo "[2/4] Disabling and blocking Piper arms..."
  rostopic pub -1 /left/enable_flag std_msgs/Bool "data: false" >/dev/null 2>&1 || true
  rostopic pub -1 /right/enable_flag std_msgs/Bool "data: false" >/dev/null 2>&1 || true
  rostopic pub -1 /left/block_arm std_msgs/Bool "data: true" >/dev/null 2>&1 || true
  rostopic pub -1 /right/block_arm std_msgs/Bool "data: true" >/dev/null 2>&1 || true
fi

echo "[3/4] Stopping real Piper adapter and Piper control nodes..."
pkill -f "quest_vr_piper_adapter.py" 2>/dev/null || true
pkill -f "roslaunch piper start_double_piper.launch" 2>/dev/null || true
rosnode kill /left/piper_ctrl_left_node /right/piper_ctrl_right_node /joint_remapper_node >/dev/null 2>&1 || true
sleep 2

echo "[4/4] Restarting preview-only adapter..."
setsid ./start_piper_adapter_preview.sh > piper_adapter_preview.log 2>&1 &
echo $! > /tmp/quest_vr_piper_adapter.pid

echo "Stopped real teleop. Preview adapter is running again."
