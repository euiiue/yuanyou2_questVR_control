# 当前控制参数说明

## 启动入口

推荐使用：

```bash
./start_slow_double.sh
```

这个脚本会：

1. 停止残留 roslaunch / rosmaster。
2. 运行 `src/Piper_ros/can_config.sh` 配置左右 CAN。
3. 检查 `left_piper` / `right_piper` 是否存在。
4. 检查 Quest ADB 是否授权。
5. 激活 conda `vt` 和 ROS Noetic 后启动 `teleop_double_piper.launch`。

## 手柄关系

当前逻辑：

- 右臂：按住右手 `B` / `RG` / 右 grip 进入遥操，`A` 单击触发右臂平滑回零。
- 左臂：按住左手 `Y` / `LG` / 左 grip 进入遥操，`X` 单击触发左臂平滑回零。
- 左右 trigger 控制夹爪开合。

具体按钮名来自 `oculus_reader` 的 Quest 按键字典；如果某些固件版本按键缺失，代码会用默认值，避免 `KeyError`。

## 重要参数

### 位置映射

```bash
POSITION_AXIS_MAP="[0, 1, 2]"
POSITION_AXIS_SIGN="[1.0, 1.0, 1.0]"
RIGHT_POSITION_AXIS_SIGN="[1.0, 1.0, 1.0]"
LEFT_POSITION_AXIS_SIGN="[1.0, 1.0, 1.0]"
POSITION_AXIS_SCALE="[1.0, 2.8, 1.0]"
RIGHT_POSITION_AXIS_SCALE="[1.3, 0.7, 1.0]"
LEFT_POSITION_AXIS_SCALE="[1.3, 0.7, 1.0]"
```

含义：VR 手柄增量先按 `POSITION_AXIS_MAP` 重新排序，再乘全局符号、侧向符号、全局缩放和侧向缩放。

### 姿态映射

```bash
ORIENTATION_AXIS_MAP="[0, 1, 2]"
ORIENTATION_AXIS_SIGN="[1.0, 1.0, 1.0]"
RIGHT_ORIENTATION_AXIS_SIGN="[1.0, -1.0, 1.0]"
LEFT_ORIENTATION_AXIS_SIGN="[1.0, -1.0, 1.0]"
RIGHT_ORIENTATION_SCALE=1.3
LEFT_ORIENTATION_SCALE=1.3
```

如果 5 关节方向反，优先调 `*_ORIENTATION_AXIS_SIGN` 的第二项。

### IK 与限位

```bash
IK_ORIENTATION_WEIGHT=0.45
IK_REGULARIZATION_WEIGHTS="[0.001, 0.001, 0.0004, 0.012, 0.012, 0.012]"
JOINT5_SOFT_ABS_LIMIT_DEG=65.0
JOINT6_IK_ABS_LIMIT_DEG=180.0
MAX_DELTA_RPY_DEG=14.0
RPY_LIMIT=2.2
WORKSPACE_MIN="[0.08, -0.35, -0.10]"
WORKSPACE_MAX="[0.58, 0.35, 0.55]"
```

右臂曾出现 joint6 反馈约 `174°`，旧 IK 模型 `±120°` 会导致初值越界，因此当前把 joint6 IK 范围扩到 `±180°`。

### 近底座低位抓取

```bash
NEAR_BASE_POSITION_PRIORITY=true
NEAR_BASE_X_THRESHOLD=0.38
NEAR_BASE_DOWN_DELTA=-0.001
NEAR_BASE_ORIENTATION_SCALE=0.60
NEAR_BASE_ORIENTATION_WEIGHT=0.16
NEAR_BASE_REF_BLEND=0.65
NEAR_BASE_JOINT_REFERENCE="[0.0, 0.65, -1.25, 0.0, 0.05, 0.0]"
```

当目标靠近底座并且手柄请求下移时，代码临时降低姿态约束，让 IK 优先满足下移位置，避免腕部姿态把解算拉到翻腕分支。

## 日志诊断

推荐调试启动：

```bash
DEBUG_POSE=true LOG_JOINT_COMMANDS=true ./start_slow_double.sh
```

常见日志：

- `Rejecting IK branch jump`: IK 解到了另一个分支，通常是腕部跳变。
- `IK solver failed ... Invalid_Number_Detected`: 目标接近边界、姿态不可达或初值不稳定。
- `near-base position priority`: 当前触发了近底座位置优先模式。
- `CAN socket ... does not exist`: CAN 接口未配置或名字不匹配。
- `no permissions / unauthorized`: Quest ADB 未授权或 udev 权限问题。

