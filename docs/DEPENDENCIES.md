# 依赖说明

## 系统环境

推荐环境：

- Ubuntu 20.04
- ROS Noetic
- Python 3.9 Conda 环境，环境名 `vt`
- Jetson aarch64 / x86_64 均可，Piper SDK 与 Pinocchio 需按对应架构安装

## APT 依赖

```bash
sudo apt update
sudo apt install -y \
  android-tools-adb \
  can-utils \
  ethtool \
  git \
  python3-catkin-tools \
  python3-rosdep \
  ros-noetic-desktop-full \
  ros-noetic-tf \
  ros-noetic-tf-conversions \
  ros-noetic-joint-state-publisher \
  ros-noetic-robot-state-publisher \
  ros-noetic-rviz
```

如果是 Jetson / Ubuntu ports 镜像，APT 502 通常是镜像或代理问题，先修复 `/etc/apt/sources.list` 与代理，再安装依赖。

## Conda 环境

```bash
conda env create -f environment.yml
conda activate vt
pip install -r requirements.txt
```

如果已有 `vt` 环境：

```bash
conda activate vt
conda install -y -c conda-forge pinocchio=3.2.0 casadi=3.6.7 numpy pyyaml
pip install -r requirements.txt
```

## ROS 编译

```bash
source /opt/ros/noetic/setup.bash
catkin_make
source devel/setup.bash
```

## Quest APK

APK 文件已随仓库放在：

```text
src/oculus_reader/APK/teleop-debug.apk
src/oculus_reader/APK/alvr_client_android.apk
```

安装：

```bash
bash scripts/install_quest_apk.sh
```

安装前需要在 Quest 中打开开发者模式，并在头显里允许 USB 调试。

