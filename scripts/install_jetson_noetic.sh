#!/usr/bin/env bash
set -euo pipefail

sudo apt update
sudo apt install -y \
  android-tools-adb \
  can-utils \
  ethtool \
  git \
  python3-catkin-tools \
  python3-rosdep \
  ros-noetic-tf \
  ros-noetic-tf-conversions \
  ros-noetic-joint-state-publisher \
  ros-noetic-robot-state-publisher \
  ros-noetic-rviz

if command -v conda >/dev/null 2>&1; then
  CONDA_BIN="$(command -v conda)"
elif [ -x "$HOME/miniconda3/bin/conda" ]; then
  CONDA_BIN="$HOME/miniconda3/bin/conda"
else
  echo "conda not found. Install Miniconda first, then rerun this script."
  exit 1
fi

eval "$("$CONDA_BIN" shell.bash hook)"

if ! conda env list | awk '{print $1}' | grep -qx vt; then
  conda env create -f environment.yml
else
  conda activate vt
  conda install -y -c conda-forge pinocchio=3.2.0 casadi=3.6.7 numpy pyyaml
  pip install -r requirements.txt
fi

echo "Done. Build with:"
echo "  source /opt/ros/noetic/setup.bash"
echo "  catkin_make"

