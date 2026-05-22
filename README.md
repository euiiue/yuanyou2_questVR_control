# yuanyou2 QuestVR Piper 双臂遥操

这是基于 `agilexrobotics/questVR_ws` 和 `Piper_ros` 整理的 yuanyou2 双 Piper 机械臂 VR 遥操包。当前版本面向 Jetson / Ubuntu 20.04 / ROS Noetic / Meta Quest 2 或 Quest 3，包含我们已经调过的双臂 CAN 命名、Quest ADB 检查、低速平滑归零、移动机器人双臂安装场景下的 IK 参数、左右臂独立位移/姿态映射，以及 VR APK 安装包。

## 目录

```text
.
├── start_slow_double.sh                 # 推荐启动脚本：CAN、ADB、ROS 环境和双臂 teleop
├── start_yuanyou_questvr_double_piper.sh # 旧启动入口，保留参考
├── src/
│   ├── oculus_reader/
│   │   ├── APK/                         # Quest 安装包
│   │   │   ├── teleop-debug.apk
│   │   │   └── alvr_client_android.apk
│   │   ├── launch/teleop_double_piper.launch
│   │   └── scripts/
│   │       ├── teleop_double_piper.py    # 双臂 VR -> IK -> 关节命令主逻辑
│   │       ├── piper_control.py          # ROS 关节命令发布、平滑限速
│   │       └── pub_pose.py               # Quest 手柄位姿发布
│   └── Piper_ros/                        # Piper 驱动、消息、URDF、CAN 脚本
├── tools/webxr_bridge/                   # 可选 WebXR/转发工具源码
├── docs/
│   ├── DEPENDENCIES.md
│   ├── CONTROL_NOTES.md
│   ├── PACKAGE_MANIFEST.md
│   └── UPSTREAM_QUESTVR_README.md
├── environment.yml
├── requirements.txt
└── scripts/
    ├── install_jetson_noetic.sh
    └── install_quest_apk.sh
```

## 快速开始

在 Jetson 上：

```bash
cd ~/openpi-robot/yuanyou2_questVR_control
bash scripts/install_jetson_noetic.sh
catkin_make
```

安装 Quest APK：

```bash
bash scripts/install_quest_apk.sh
```

启动双臂遥操：

```bash
cd ~/openpi-robot/yuanyou2_questVR_control
DEBUG_POSE=true LOG_JOINT_COMMANDS=true ./start_slow_double.sh
```

正常运行时会完成：

```text
[1/5] Stopping stale questVR roslaunch processes...
[2/5] Configuring CAN interfaces...
[3/5] Checking CAN interfaces...
[4/5] Checking Quest ADB access...
[5/5] Launching slow double-arm teleop...
```

## 当前硬件约定

当前 CAN 线映射写在 `src/Piper_ros/can_config.sh`：

```text
left_piper  -> USB 端口 1-2.1.1:1.0
right_piper -> USB 端口 1-2.1.3:1.0
bitrate     -> 1000000
```

如果 USB-CAN 插口换了，先运行：

```bash
cd src/Piper_ros
bash find_all_can_port.sh
```

然后更新 `can_config.sh` 里的 USB 端口映射。

## 常用调参

只改右臂灵敏度：

```bash
RIGHT_POSITION_AXIS_SCALE="[1.0, 0.5, 0.8]" RIGHT_ORIENTATION_SCALE=1.0 ./start_slow_double.sh
```

只改左右腕部方向：

```bash
LEFT_ORIENTATION_AXIS_SIGN="[1.0, -1.0, 1.0]" RIGHT_ORIENTATION_AXIS_SIGN="[1.0, -1.0, 1.0]" ./start_slow_double.sh
```

降低速度和抖动：

```bash
CONTROL_SPEED=12 MAX_JOINT_STEP_DEG=3.0 COMMAND_HZ=15 JOINT_COMMAND_HZ=15 ./start_slow_double.sh
```

允许更低抓取位置：

```bash
WORKSPACE_MIN="[0.08, -0.35, -0.10]" ./start_slow_double.sh
```

## 关键改动摘要

- 禁用默认 RViz，避免无显示环境下 Qt `xcb` 崩溃。
- 双臂 CAN 接口固定为 `left_piper` / `right_piper`。
- 增加 Quest ADB 重试和授权检查。
- 增加归零位平滑插值，避免瞬间回零。
- 使用增量 IK，不再用启动时手柄绝对位置直接同步机械臂。
- 增加左右臂独立位置和姿态映射参数。
- 放宽工作空间 z 下限到 `-0.10m`，匹配当前机械臂低位抓取姿态。
- 右臂 joint6 IK 限位扩到 `±180°`，避免右臂反馈 seed 超出旧 URDF `±120°` 后 IK 失效。
- 近底座下移时启用 position-priority，降低腕部姿态对下移动作的干扰。

## 参考

- 上游 questVR README: `docs/UPSTREAM_QUESTVR_README.md`
- 依赖安装: `docs/DEPENDENCIES.md`
- 当前遥操参数说明: `docs/CONTROL_NOTES.md`
- 打包内容清单: `docs/PACKAGE_MANIFEST.md`
