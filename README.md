# Quest 3S Teleoperation for Yuanyou2
<p align="left">
  <img src="https://img.shields.io/badge/version-v0.1.0-blue" />
  <img src="https://img.shields.io/badge/ROS%202-Foxy-22314E?logo=ros&logoColor=white" />
  <img src="https://img.shields.io/badge/Ubuntu-20.04-E95420?logo=ubuntu&logoColor=white" />
  <img src="https://img.shields.io/badge/Jetson-Orin%20NX-76B900?logo=nvidia&logoColor=white" />
  <img src="https://img.shields.io/badge/VR-Meta%20Quest%203S-0467DF?logo=meta&logoColor=white" />
  <img src="https://img.shields.io/badge/Robot-Yuanyou2%20Dual%20Piper-red" />
  <img src="https://img.shields.io/badge/Input-OpenXR%20%2B%20ADB-lightgrey" />
  <img src="https://img.shields.io/badge/Control-IK%20Teleoperation-informational" />
  <img src="https://img.shields.io/badge/CAN-can0%20%7C%20can1-yellow" />
  <img src="https://img.shields.io/badge/Safety-Workspace%20Guard-success" />
</p>
[![zread](https://img.shields.io/badge/Ask_Zread-_.svg?style=for-the-badge&color=00b0aa&labelColor=000000&logo=data%3Aimage%2Fsvg%2Bxml%3Bbase64%2CPHN2ZyB3aWR0aD0iMTYiIGhlaWdodD0iMTYiIHZpZXdCb3g9IjAgMCAxNiAxNiIgZmlsbD0ibm9uZSIgeG1sbnM9Imh0dHA6Ly93d3cudzMub3JnLzIwMDAvc3ZnIj4KPHBhdGggZD0iTTQuOTYxNTYgMS42MDAxSDIuMjQxNTZDMS44ODgxIDEuNjAwMSAxLjYwMTU2IDEuODg2NjQgMS42MDE1NiAyLjI0MDFWNC45NjAxQzEuNjAxNTYgNS4zMTM1NiAxLjg4ODEgNS42MDAxIDIuMjQxNTYgNS42MDAxSDQuOTYxNTZDNS4zMTUwMiA1LjYwMDEgNS42MDE1NiA1LjMxMzU2IDUuNjAxNTYgNC45NjAxVjIuMjQwMUM1LjYwMTU2IDEuODg2NjQgNS4zMTUwMiAxLjYwMDEgNC45NjE1NiAxLjYwMDFaIiBmaWxsPSIjZmZmIi8%2BCjxwYXRoIGQ9Ik00Ljk2MTU2IDEwLjM5OTlIMi4yNDE1NkMxLjg4ODEgMTAuMzk5OSAxLjYwMTU2IDEwLjY4NjQgMS42MDE1NiAxMS4wMzk5VjEzLjc1OTlDMS42MDE1NiAxNC4xMTM0IDEuODg4MSAxNC4zOTk5IDIuMjQxNTYgMTQuMzk5OUg0Ljk2MTU2QzUuMzE1MDIgMTQuMzk5OSA1LjYwMTU2IDE0LjExMzQgNS42MDE1NiAxMy43NTk5VjExLjAzOTlDNS42MDE1NiAxMC42ODY0IDUuMzE1MDIgMTAuMzk5OSA0Ljk2MTU2IDEwLjM5OTlaIiBmaWxsPSIjZmZmIi8%2BCjxwYXRoIGQ9Ik0xMy43NTg0IDEuNjAwMUgxMS4wMzg0QzEwLjY4NSAxLjYwMDEgMTAuMzk4NCAxLjg4NjY0IDEwLjM5ODQgMi4yNDAxVjQuOTYwMUMxMC4zOTg0IDUuMzEzNTYgMTAuNjg1IDUuNjAwMSAxMS4wMzg0IDUuNjAwMUgxMy43NTg0QzE0LjExMTkgNS42MDAxIDE0LjM5ODQgNS4zMTM1NiAxNC4zOTg0IDQuOTYwMVYyLjI0MDFDMTQuMzk4NCAxLjg4NjY0IDE0LjExMTkgMS42MDAxIDEzLjc1ODQgMS42MDAxWiIgZmlsbD0iI2ZmZiIvPgo8cGF0aCBkPSJNNCAxMkwxMiA0TDQgMTJaIiBmaWxsPSIjZmZmIi8%2BCjxwYXRoIGQ9Ik00IDEyTDEyIDQiIHN0cm9rZT0iI2ZmZiIgc3Ryb2tlLXdpZHRoPSIxLjUiIHN0cm9rZS1saW5lY2FwPSJyb3VuZCIvPgo8L3N2Zz4K&logoColor=ffffff)](https://zread.ai/euiiue/yuanyou2_questVR_control)
<p align="left">
  <img src="https://img.shields.io/github/last-commit/euiiue/yuanyou2_questVR_control" />
  <img src="https://img.shields.io/github/languages/top/euiiue/yuanyou2_questVR_control" />
  <img src="https://img.shields.io/github/repo-size/euiiue/yuanyou2_questVR_control" />
  <img src="https://img.shields.io/github/stars/euiiue/yuanyou2_questVR_control?style=social" />
</p>
基于 ROS 2 Foxy、Quest 3S 和 Yuanyou2 的双臂遥操作系统。当前版本面向 Jetson 实机运行，核心目标是用 Quest 手柄的 OpenXR 位姿数据控制左右两台 Piper 机械臂，同时保留夹爪控制、HOME 复位、IK 安全护栏和单发布源关节控制链路。

## 快速入口

| 文档 | 说明 |
| --- | --- |
| [Quick Start](docs/quick_start.md) | 最短实机启动流程 |
| [requirements.txt](requirements.txt) | Python 运行依赖 |
| [LICENSE](LICENSE) | 仓库顶层许可证 |

## 项目状态

本仓库是当前实机运行版本，运行路径如下：

| 环境 | 路径 |
| --- | --- |
| 本地开发机 | `/home/handx/cyf/yuanyou2_quest/QuestArmTeleop/` |
| Jetson 实机 | `/home/yuanyou/QuestArmTeleop/` |

当前默认运行方式：

| 项目 | 配置 |
| --- | --- |
| ROS 版本 | ROS 2 Foxy |
| VR 设备 | Meta Quest 3S |
| VR 数据入口 | ADB logcat + OculusReader |
| 机械臂 | Yuanyou双臂 |
| 左臂 CAN | `can0` |
| 右臂 CAN | `can1` |
| 主启动文件 | `src/oculus_reader/launch/teleop_double_piper.launch.py` |
| IK 模式 | `use_ik:=true` |

## 核心特性

- OpenXR 位姿输入：从 Quest 3S 读取左右手柄位姿，并发布为 ROS 2 `PoseStamped`。
- Delta Mapping：使用启动瞬间的 VR 位姿和机械臂末端位姿作为锚点，避免绝对位姿导致方向错乱。
- 姿态连续性保护：使用四元数相对旋转映射，降低末端 180 度翻转风险。
- IK 安全护栏：工作空间限制、IK 解异常捕获、无解跳帧，避免节点直接崩溃。
- HOME 复位：左右摇杆长按 1 秒，将对应机械臂送回预设关节姿态。
- 单发布源控制：通过 `joint_command_mux_node.py` 合并 IK 关节命令和夹爪命令，避免多个节点同时写 `/control/joint_states`。

## 文件清单

| 文件 | 路径 | 作用 |
| --- | --- | --- |
| `arm_ik_pose_node.py` | `src/oculus_reader/scripts/` | IK 解算、Delta 位姿映射、位置/姿态滤波、adaptive tremor |
| `pub_delta_pose.py` | `src/oculus_reader/scripts/` | 手柄按钮桥：启动、停止、HOME、trigger 夹爪 |
| `pub_pose.py` | `src/oculus_reader/scripts/` | 从 OculusReader 读取 Quest 数据，发布 handle pose 和 OpenXR pose |
| `oculus_reader.py` | `src/oculus_reader/scripts/` | ADB logcat 底层读取与 Quest 原始数据解析 |
| `joint_command_mux_node.py` | `src/oculus_reader/scripts/` | 合并 IK 关节命令和夹爪命令，输出唯一 `/control/joint_states` 发布源 |
| `arm_ik_pose_node.piper.yaml` | `src/oculus_reader/config/` | Piper IK、姿态、位置、滤波和 adaptive 参数 |
| `teleop_double_piper.launch.py` | `src/oculus_reader/launch/` | 双臂遥操作主启动入口 |
| `CMakeLists.txt` | `src/oculus_reader/` | ROS 2 编译和安装配置 |
| `can_activate.sh` | `src/agx_arm_ros/scripts/` | CAN 接口激活脚本 |
| `use_jetson_foxy_runtime.sh` | `tools/` | Jetson Foxy 运行环境加载脚本 |

## 数据流

```text
Quest 3S (ADB)
  |
  v
oculus_reader.py -> pub_pose.py
  |-- /left_openxr_pose  -> left_arm_ik_pose_node  -> /left_arm/internal/ik_joint_states
  |-- /right_openxr_pose -> right_arm_ik_pose_node -> /right_arm/internal/ik_joint_states
  |-- /left_handle_pose  -> left_pub_delta_pose    -> /left_arm/internal/gripper_joint_states
  `-- /right_handle_pose -> right_pub_delta_pose   -> /right_arm/internal/gripper_joint_states

left_joint_command_mux_node
  -> /left_arm/control/joint_states
  -> agx_arm_ctrl
  -> CAN0
  -> left Piper

right_joint_command_mux_node
  -> /right_arm/control/joint_states
  -> agx_arm_ctrl
  -> CAN1
  -> right Piper
```

## 启动流程

### 1. SSH 到 Jetson

```bash
ssh yuanyou@ip
```

### 2. 清理残留进程

```bash
kill -9 $(pgrep -f "agx_arm_ctrl|teleop|arm_ik_pose|pub_delta|pub_pose|joint_command_mux") 2>/dev/null
sleep 2
```

### 3. 确认 CAN 接口

```bash
ip -br link | grep can
```

正常情况下应看到 `can0` 和 `can1`。如果接口没有启动，执行：

```bash
cd ~/QuestArmTeleop
sudo bash src/agx_arm_ros/scripts/can_activate.sh can0 1000000
sudo bash src/agx_arm_ros/scripts/can_activate.sh can1 1000000
```

再次确认：

```bash
ip -br link | grep can
```

### 4. 确认 Quest ADB 连接

```bash
adb devices -l
```

正常情况下应看到类似：

```text
340YC20G7N0D1C    device
```

如果列表为空：

```bash
adb kill-server
adb start-server
adb devices -l
```

仍然为空时，检查：

- Quest 已开机并解锁。
- USB 线支持数据传输。
- Quest 已开启开发者模式。
- Quest 内已允许 USB 调试。
- `com.rail.oculus.teleop` 已在 Quest 前台运行。

### 5. 启动遥操作

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

## 手柄操作

| 功能 | 左手柄 | 右手柄 |
| --- | --- | --- |
| 启动控制 | `X` | `A` |
| 停止控制 | `Y` | `B` |
| HOME 复位 | 左摇杆按下保持 1 秒 | 右摇杆按下保持 1 秒 |
| 夹爪开合 | 左 trigger | 右 trigger |

## 验证命令

新开一个 SSH 终端：

```bash
ssh yuanyou@ip
cd ~/QuestArmTeleop
source tools/use_jetson_foxy_runtime.sh
```

检查 Quest 位姿数据流：

```bash
ros2 topic hz /left_openxr_pose
ros2 topic hz /right_openxr_pose
```

检查关节控制是否只有一个发布源：

```bash
ros2 topic info /left_arm/control/joint_states -v | grep "Publisher count"
ros2 topic info /right_arm/control/joint_states -v | grep "Publisher count"
```

期望输出：

```text
Publisher count: 1
```

检查关键参数：

```bash
ros2 param get /right_arm_ik_pose_node orientation_mode
ros2 param get /right_arm_ik_pose_node orientation_gain
ros2 param get /right_joint_command_mux_node gripper_max_step_open
```

参考值：

```text
orientation_mode: track_limited
orientation_gain: 0.60
gripper_max_step_open: 0.0035
```

## RViz 查看机械臂真实姿态

如果只想查看 TF 和 RobotModel，不启动遥操作控制，可在 Jetson 上运行机械臂描述和反馈节点，在本地电脑打开 RViz。

Jetson 端：

```bash
cd ~/QuestArmTeleop
source tools/use_jetson_foxy_runtime.sh
source install/setup.bash
export ROS_DOMAIN_ID=23
export ROS_LOCALHOST_ONLY=0

ros2 launch agx_arm_description display.launch.py \
  arm_type:=piper \
  effector_type:=agx_gripper \
  use_rviz:=false \
  gui:=false \
  control:=true \
  follow:=false \
  tcp_offset:="[0.0, 0.0, 0.13, 0.0, 0.0, 0.0]"
```

本地电脑：

```bash
source /opt/ros/<your_ros_distro>/setup.bash
export ROS_DOMAIN_ID=23
export ROS_LOCALHOST_ONLY=0
rviz2
```

RViz 中设置：

- `Global Options` 的 `Fixed Frame` 设为 `base_link`。
- 添加 `TF` Display。
- 添加 `RobotModel` Display。
- `RobotModel` 的 `Description Source` 选择 `Topic`。

## HOME 姿态配置

左右臂 HOME 关节角在主启动文件中维护：

```text
src/oculus_reader/launch/teleop_double_piper.launch.py
```

关键变量：

```python
left_home_positions = [...]
right_home_positions = [...]
```

这两组值会同时传给：

- `agx_arm_ctrl` 的 `home_joint_positions`
- `pub_delta_pose.py` 的 `prep_home_positions`

因此修改这两组变量后，驱动 HOME 和手柄 HOME 复位会保持一致。

## 常见问题

### `ros2: command not found`

当前终端没有加载 ROS 2 环境。执行：

```bash
cd ~/QuestArmTeleop
source tools/use_jetson_foxy_runtime.sh
source install/setup.bash
```

### `Device not found`

Quest 没有被 ADB 识别。检查：

```bash
adb devices -l
```

如果为空，重新启动 ADB：

```bash
adb kill-server
adb start-server
adb devices -l
```

并确认 Quest 内已经允许 USB 调试。

### `Failed to get firmware version` 或 `Device is DOWN`

通常是 CAN 接口未启动、机械臂未通电或 CAN 线接错。检查：

```bash
ip -br link | grep can
```

必要时重新激活：

```bash
sudo bash src/agx_arm_ros/scripts/can_activate.sh can0 1000000
sudo bash src/agx_arm_ros/scripts/can_activate.sh can1 1000000
```

### `/control/joint_states` 有多个 publisher

IK 模式下应只有 `joint_command_mux_node.py` 发布 `/control/joint_states`。检查：

```bash
ros2 topic info /right_arm/control/joint_states -v | grep "Publisher count"
```

如果不是 `1`，先清理残留进程再重启：

```bash
kill -9 $(pgrep -f "agx_arm_ctrl|teleop|arm_ik_pose|pub_delta|pub_pose|joint_command_mux") 2>/dev/null
sleep 2
```

### 手柄方向不对或末端翻转

当前版本默认使用 OpenXR delta mapping 和相对姿态映射。优先检查：

- 是否使用 `use_ik:=true`。
- `pub_pose.py` 是否正常发布 `/left_openxr_pose` 和 `/right_openxr_pose`。
- `arm_ik_pose_node.piper.yaml` 中 `orientation_mode` 是否为 `track_limited`。
- 控制前是否先按启动键完成当前 VR 位姿和机械臂位姿锁存。

## 备份

当前稳定备份：

```text
backups/stable_before_gripper_tremor_20260608_172445/
├── arm_ik_pose_node.py
├── pub_delta_pose.py
├── joint_command_mux_node.py
├── arm_ik_pose_node.piper.yaml
└── teleop_double_piper.launch.py
```

Jetson 上每次修改运行文件前建议创建备份：

```bash
cd ~/QuestArmTeleop
mkdir -p backups/<backup_name>
cp src/oculus_reader/launch/teleop_double_piper.launch.py backups/<backup_name>/
```

## 开发与构建

本地或 Jetson 修改后，至少执行：

```bash
python3 -m py_compile src/oculus_reader/scripts/arm_ik_pose_node.py
python3 -m py_compile src/oculus_reader/scripts/pub_delta_pose.py
python3 -m py_compile src/oculus_reader/scripts/pub_pose.py
python3 -m py_compile src/oculus_reader/scripts/joint_command_mux_node.py
python3 -m py_compile src/oculus_reader/launch/teleop_double_piper.launch.py
```

Jetson 构建：

```bash
cd ~/QuestArmTeleop
source tools/use_jetson_foxy_runtime.sh
colcon build --packages-select oculus_reader --symlink-install
```

只解析 launch 参数，不启动硬件：

```bash
source install/setup.bash
ros2 launch oculus_reader teleop_double_piper.launch.py --show-args
```

## 安全注意事项

- 启动前确认机械臂周围没有人员和障碍物。
- 首次验证建议单臂、低速、小幅移动。
- Quest 位姿不稳定时不要按启动键。
- CAN、ADB、ROS 2 topic 任一异常时不要继续实机控制。
- 出现机械臂异常运动时，立即停止控制并断开使能。
