#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
cd "$SCRIPT_DIR"

ENABLE_ARMS=0
POSITION_SCALE="${POSITION_SCALE:-0.35}"
RATE="${RATE:-30}"

while [[ $# -gt 0 ]]; do
  case "$1" in
    --enable-arms)
      ENABLE_ARMS=1
      shift
      ;;
    --position-scale)
      POSITION_SCALE="$2"
      shift 2
      ;;
    --rate)
      RATE="$2"
      shift 2
      ;;
    -h|--help)
      cat <<'EOF'
Usage:
  ./start_real_piper_teleop_on_jetson.sh [--enable-arms] [--position-scale 0.35] [--rate 30]

Starts:
  - ROS master if needed
  - Piper CAN interfaces can_leftarm/can_rightarm
  - piper start_double_piper.launch with auto_enable=false
  - Quest VR Piper adapter with real /left/pos_cmd and /right/pos_cmd output

The adapter still requires holding each controller deadman/grip button before
it publishes real commands. Without --enable-arms, Piper nodes start but arms
remain disabled.
EOF
      exit 0
      ;;
    *)
      echo "Unknown argument: $1" >&2
      exit 2
      ;;
  esac
done

source "$SCRIPT_DIR/source_ros_setup.sh"

if ! rostopic list >/dev/null 2>&1; then
  setsid roscore > /tmp/yuanyou_remote_roscore.log 2>&1 &
  echo $! > /tmp/yuanyou_remote_roscore.pid
  sleep 4
fi
rostopic list >/dev/null || {
  echo "roscore is not reachable." >&2
  exit 1
}

CAN_SCRIPT="${CAN_SCRIPT:-$WORKSPACE_ROOT/src/Piper_ros/can_config.sh}"
if [[ ! -x "$CAN_SCRIPT" ]]; then
  chmod +x "$CAN_SCRIPT"
fi
echo "[1/5] Activating Piper CAN interfaces..."
if [[ -n "${SUDO_PASSWORD:-}" ]]; then
  printf '%s\n' "$SUDO_PASSWORD" | sudo -S bash "$CAN_SCRIPT"
else
  sudo bash "$CAN_SCRIPT"
fi

echo "[2/5] Checking CAN names..."
for iface in left_piper right_piper; do
  ip -br link show "$iface"
done

echo "[3/5] Starting Piper ROS control nodes..."
PIPER_LAUNCH_FILE="${PIPER_LAUNCH_FILE:-$WORKSPACE_ROOT/src/Piper_ros/src/piper/launch/start_double_piper.launch}"
if ! pgrep -f "start_double_piper.launch" >/dev/null; then
  if rospack find piper >/dev/null 2>&1; then
    LAUNCH_TARGET=(piper start_double_piper.launch)
  else
    LAUNCH_TARGET=("$PIPER_LAUNCH_FILE")
  fi
  setsid roslaunch "${LAUNCH_TARGET[@]}" \
    left_auto_enable:=false \
    right_auto_enable:=false \
    > piper_real_control.log 2>&1 &
  echo $! > /tmp/piper_real_control.pid
  sleep 6
fi

timeout 15 bash -c 'until rostopic list | grep -q "^/left/joint_states_single$"; do sleep 0.5; done'
timeout 15 bash -c 'until rostopic list | grep -q "^/right/joint_states_single$"; do sleep 0.5; done'

echo "[4/5] Starting Quest adapter in REAL mode..."
pkill -f "quest_vr_piper_adapter.py" 2>/dev/null || true
sleep 1
setsid ./start_piper_adapter_real.sh \
  _position_scale:="$POSITION_SCALE" \
  _rate:="$RATE" \
  > piper_adapter_real.log 2>&1 &
echo $! > /tmp/quest_vr_piper_adapter.pid
sleep 2

if [[ "$ENABLE_ARMS" == "1" ]]; then
  echo "[5/5] Unblocking and enabling arms..."
  rostopic pub -1 /left/block_arm std_msgs/Bool "data: false" >/dev/null 2>&1 || true
  rostopic pub -1 /right/block_arm std_msgs/Bool "data: false" >/dev/null 2>&1 || true
  rostopic pub -1 /left/enable_flag std_msgs/Bool "data: true" >/dev/null
  rostopic pub -1 /right/enable_flag std_msgs/Bool "data: true" >/dev/null
else
  echo "[5/5] Arms are NOT enabled. Re-run with --enable-arms when ready."
fi

echo
echo "Real teleop stack is ready."
echo "Hold each Quest controller grip/deadman button to publish real commands."
echo "Release the grip/deadman button to stop publishing commands."
echo "Preview topics: /quest_vr_piper/left_pos_cmd /quest_vr_piper/right_pos_cmd"
echo "Real topics:    /left/pos_cmd /right/pos_cmd"
