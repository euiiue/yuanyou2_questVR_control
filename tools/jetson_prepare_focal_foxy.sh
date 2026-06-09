#!/usr/bin/env bash
set -Eeuo pipefail

# Prepare a Jetson running Ubuntu 20.04/focal for the QuestArmTeleop ROS 2 stack.
# This is a compatibility path: upstream agx_arm_ros is validated on Humble/Jazzy,
# but Ubuntu 20.04 has binary ROS 2 packages for Foxy.

TARGET_USER="${TARGET_USER:-yuanyou}"
WORKSPACE="${WORKSPACE:-/home/${TARGET_USER}/VRteleop/QuestArmTeleop}"
PYAGXARM_DIR="${PYAGXARM_DIR:-/home/${TARGET_USER}/VRteleop/pyAgxArm}"
ROS_KEY_LOCAL="${ROS_KEY_LOCAL:-/home/${TARGET_USER}/VRteleop/ros.key}"
PROXY_HOST="${PROXY_HOST:-192.168.31.179}"
PROXY_PORT="${PROXY_PORT:-7890}"
PROXY_URL="http://${PROXY_HOST}:${PROXY_PORT}"
ROS_DISTRO_TARGET="${ROS_DISTRO_TARGET:-foxy}"
CONDA_ENV="${CONDA_ENV:-vt_foxy}"
CONDA_BIN="/home/${TARGET_USER}/miniconda3/bin/conda"
FALLBACK_DATE_UTC="${FALLBACK_DATE_UTC:-2026-05-27 05:38:21 UTC}"

if [[ "${EUID}" -ne 0 ]]; then
  echo "Run with sudo:"
  echo "  sudo PROXY_HOST=${PROXY_HOST} bash $0"
  exit 1
fi

export http_proxy="${PROXY_URL}"
export https_proxy="${PROXY_URL}"
export HTTP_PROXY="${PROXY_URL}"
export HTTPS_PROXY="${PROXY_URL}"
export all_proxy="${PROXY_URL}"
export ALL_PROXY="${PROXY_URL}"
export no_proxy="127.0.0.1,localhost,::1"
export NO_PROXY="${no_proxy}"

run_as_user() {
  sudo -H -u "${TARGET_USER}" env \
    http_proxy="${PROXY_URL}" https_proxy="${PROXY_URL}" \
    HTTP_PROXY="${PROXY_URL}" HTTPS_PROXY="${PROXY_URL}" \
    all_proxy="${PROXY_URL}" ALL_PROXY="${PROXY_URL}" \
    no_proxy="${no_proxy}" NO_PROXY="${NO_PROXY}" \
    bash -lc "$*"
}

echo "[1/8] Setting system clock from packages.ros.org Date header"
date_header="$(
  curl -k -fsSI --max-time 30 -x "${PROXY_URL}" \
    "https://packages.ros.org/ros2/ubuntu/dists/focal/InRelease" |
    awk 'BEGIN{IGNORECASE=1} /^Date:/{sub(/\r$/, ""); print substr($0, 7)}' |
    tail -n 1
)" || true
if [[ -z "${date_header}" ]]; then
  echo "Could not read Date header through ${PROXY_URL}; using FALLBACK_DATE_UTC=${FALLBACK_DATE_UTC}"
  date_header="${FALLBACK_DATE_UTC}"
fi
date -u -s "${date_header}"
timedatectl set-timezone Asia/Shanghai || true
timedatectl set-ntp true || true

echo "[2/8] Configuring apt proxy ${PROXY_URL}"
cat >/etc/apt/apt.conf.d/99local-proxy <<EOF
Acquire::http::Proxy "${PROXY_URL}";
Acquire::https::Proxy "${PROXY_URL}";
EOF

echo "[3/8] Configuring Ubuntu ports source"
stamp="$(date +%Y%m%d_%H%M%S)"
cp -a /etc/apt/sources.list "/etc/apt/sources.list.bak-${stamp}" || true
cat >/etc/apt/sources.list <<'EOF'
deb https://mirrors.aliyun.com/ubuntu-ports/ focal main restricted universe multiverse
deb https://mirrors.aliyun.com/ubuntu-ports/ focal-updates main restricted universe multiverse
deb https://mirrors.aliyun.com/ubuntu-ports/ focal-backports main restricted universe multiverse
deb https://mirrors.aliyun.com/ubuntu-ports/ focal-security main restricted universe multiverse
EOF

echo "[4/8] Configuring ROS 2 ${ROS_DISTRO_TARGET} apt source for focal"
install -d -m 0755 /usr/share/keyrings
if [[ -s "${ROS_KEY_LOCAL}" ]]; then
  cp "${ROS_KEY_LOCAL}" /usr/share/keyrings/ros-archive-keyring.gpg
else
  curl -k -fsSL --max-time 60 -x "${PROXY_URL}" \
    https://raw.githubusercontent.com/ros/rosdistro/master/ros.key \
    -o /usr/share/keyrings/ros-archive-keyring.gpg
fi
cat >/etc/apt/sources.list.d/ros2.list <<EOF
deb [arch=arm64 signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu focal main
EOF

echo "[5/8] Installing ROS 2 ${ROS_DISTRO_TARGET} build/runtime packages"
apt-get update
apt-get install -y \
  ca-certificates curl gnupg lsb-release \
  can-utils ethtool android-tools-adb \
  python3-pip python3-scipy python3-colcon-common-extensions \
  "ros-${ROS_DISTRO_TARGET}-ros-base" \
  "ros-${ROS_DISTRO_TARGET}-ament-cmake" \
  "ros-${ROS_DISTRO_TARGET}-ament-cmake-python" \
  "ros-${ROS_DISTRO_TARGET}-rosidl-default-generators" \
  "ros-${ROS_DISTRO_TARGET}-rosidl-default-runtime" \
  "ros-${ROS_DISTRO_TARGET}-tf2-ros" \
  "ros-${ROS_DISTRO_TARGET}-joint-state-publisher" \
  "ros-${ROS_DISTRO_TARGET}-joint-state-publisher-gui" \
  "ros-${ROS_DISTRO_TARGET}-robot-state-publisher" \
  "ros-${ROS_DISTRO_TARGET}-xacro" \
  "ros-${ROS_DISTRO_TARGET}-rviz2"

echo "[6/8] Installing optional ROS 2 control packages"
apt-get install -y \
  "ros-${ROS_DISTRO_TARGET}-ros2-control" \
  "ros-${ROS_DISTRO_TARGET}-ros2-controllers" \
  "ros-${ROS_DISTRO_TARGET}-controller-manager" \
  "ros-${ROS_DISTRO_TARGET}-trajectory-msgs" \
  "ros-${ROS_DISTRO_TARGET}-gripper-controllers" || true

echo "[7/8] Preparing conda environment ${CONDA_ENV}"
if [[ ! -x "${CONDA_BIN}" ]]; then
  echo "Conda not found at ${CONDA_BIN}"
  exit 1
fi

if ! run_as_user "${CONDA_BIN} env list | awk '{print \$1}' | grep -qx ${CONDA_ENV}"; then
  run_as_user "${CONDA_BIN} create -y -n ${CONDA_ENV} -c conda-forge python=3.8 pinocchio==3.2.0 casadi meshcat pyyaml scipy numpy"
else
  run_as_user "${CONDA_BIN} install -y -n ${CONDA_ENV} -c conda-forge python=3.8 pinocchio==3.2.0 casadi meshcat pyyaml scipy numpy"
fi

run_as_user "${CONDA_BIN} run -n ${CONDA_ENV} python -m pip install --proxy ${PROXY_URL} -U pure-python-adb python-can typing-extensions"
if [[ -d "${PYAGXARM_DIR}" ]]; then
  run_as_user "${CONDA_BIN} run -n ${CONDA_ENV} python -m pip install --no-deps -e ${PYAGXARM_DIR}"
else
  echo "WARNING: ${PYAGXARM_DIR} not found; pyAgxArm will be missing."
fi

echo "[8/8] Building QuestArmTeleop workspace without MoveIt package"
run_as_user "source /opt/ros/${ROS_DISTRO_TARGET}/setup.bash && \
  source /home/${TARGET_USER}/miniconda3/etc/profile.d/conda.sh && \
  conda activate ${CONDA_ENV} && \
  export PYTHONPATH=/opt/ros/${ROS_DISTRO_TARGET}/lib/python3.8/site-packages:\${PYTHONPATH:-} && \
  cd ${WORKSPACE} && \
  colcon build --symlink-install --packages-skip agx_arm_moveit"

echo
echo "Done. Runtime command:"
echo "  source /opt/ros/${ROS_DISTRO_TARGET}/setup.bash"
echo "  source /home/${TARGET_USER}/miniconda3/etc/profile.d/conda.sh"
echo "  conda activate ${CONDA_ENV}"
echo "  export PYTHONPATH=/opt/ros/${ROS_DISTRO_TARGET}/lib/python3.8/site-packages:\${PYTHONPATH:-}"
echo "  cd ${WORKSPACE}"
echo "  source install/setup.bash"
echo "  ros2 launch oculus_reader teleop_double_piper.launch.py use_rviz:=false"
