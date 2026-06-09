# Quick Start

This guide starts the current Jetson-based Quest 3S to AgileX Piper dual-arm teleoperation stack.

## Runtime Layout

| Machine | Path |
| --- | --- |
| Development PC | `/home/handx/cyf/yuanyou2_quest/QuestArmTeleop/` |
| Jetson runtime | `/home/yuanyou/QuestArmTeleop/` |

Main launch file:

```text
src/oculus_reader/launch/teleop_double_piper.launch.py
```

## 1. Connect to Jetson

```bash
ssh yuanyou@192.168.31.230
cd ~/QuestArmTeleop
```

## 2. Clean Old Runtime Processes

```bash
kill -9 $(pgrep -f "agx_arm_ctrl|teleop|arm_ik_pose|pub_delta|pub_pose|joint_command_mux") 2>/dev/null
sleep 2
```

## 3. Bring Up CAN

Check CAN state:

```bash
ip -br link | grep can
```

If `can0` or `can1` is missing or down:

```bash
sudo bash src/agx_arm_ros/scripts/can_activate.sh can0 1000000
sudo bash src/agx_arm_ros/scripts/can_activate.sh can1 1000000
ip -br link | grep can
```

Expected mapping:

| Arm | CAN |
| --- | --- |
| Left Piper | `can0` |
| Right Piper | `can1` |

## 4. Check Quest ADB

```bash
adb devices -l
```

Expected output includes one `device`, for example:

```text
340YC20G7N0D1C    device
```

If the list is empty:

```bash
adb kill-server
adb start-server
adb devices -l
```

Also check that the Quest is awake, unlocked, in developer mode, and has accepted USB debugging.

## 5. Start the Teleop App on Quest

Wear the Quest and open the teleop app. The runtime expects:

```text
com.rail.oculus.teleop
```

to be running in the foreground.

## 6. Launch Dual-Arm Teleoperation

```bash
cd ~/QuestArmTeleop
source tools/use_jetson_foxy_runtime.sh

ros2 launch oculus_reader teleop_double_piper.launch.py \
  use_rviz:=false \
  use_ik:=true \
  left_can_port:=can0 \
  right_can_port:=can1 \
  auto_enable:=true \
  control_enabled:=true
```

## 7. Controller Map

| Function | Left Controller | Right Controller |
| --- | --- | --- |
| Start teleop | `X` | `A` |
| Stop teleop | `Y` | `B` |
| HOME reset | Hold left joystick for 1 second | Hold right joystick for 1 second |
| Gripper | Left trigger | Right trigger |

## 8. Verify from a Second SSH Terminal

```bash
ssh yuanyou@192.168.31.230
cd ~/QuestArmTeleop
source tools/use_jetson_foxy_runtime.sh
```

Quest pose rate:

```bash
ros2 topic hz /left_openxr_pose
ros2 topic hz /right_openxr_pose
```

Single publisher check:

```bash
ros2 topic info /left_arm/control/joint_states -v | grep "Publisher count"
ros2 topic info /right_arm/control/joint_states -v | grep "Publisher count"
```

Expected:

```text
Publisher count: 1
```

Key parameters:

```bash
ros2 param get /right_arm_ik_pose_node orientation_mode
ros2 param get /right_arm_ik_pose_node orientation_gain
ros2 param get /right_joint_command_mux_node gripper_max_step_open
```

Expected reference values:

```text
track_limited
0.60
0.0035
```

## Troubleshooting

### `ros2: command not found`

Load the runtime environment:

```bash
cd ~/QuestArmTeleop
source tools/use_jetson_foxy_runtime.sh
source install/setup.bash
```

### `Device not found`

Quest is not visible through ADB. Re-check USB, headset unlock state, developer mode and USB debugging authorization:

```bash
adb devices -l
```

### `Device is DOWN`

The CAN interface is down or the arm is not reachable:

```bash
ip -br link | grep can
sudo bash src/agx_arm_ros/scripts/can_activate.sh can1 1000000
```

### Multiple publishers on `/control/joint_states`

Kill stale nodes and restart:

```bash
kill -9 $(pgrep -f "agx_arm_ctrl|teleop|arm_ik_pose|pub_delta|pub_pose|joint_command_mux") 2>/dev/null
sleep 2
```

## Safety

- Keep people and obstacles outside the robot workspace.
- Start with small single-arm motions when validating a new setup.
- Do not press the start button while Quest tracking is unstable.
- Stop immediately if CAN, ADB, ROS topics or IK feedback look abnormal.
