FROM ros:humble-ros-base-jammy

ARG DEBIAN_FRONTEND=noninteractive
ARG http_proxy
ARG https_proxy
ARG HTTP_PROXY
ARG HTTPS_PROXY
ARG no_proxy
ARG NO_PROXY

ENV http_proxy=${http_proxy}
ENV https_proxy=${https_proxy}
ENV HTTP_PROXY=${HTTP_PROXY}
ENV HTTPS_PROXY=${HTTPS_PROXY}
ENV no_proxy=${no_proxy}
ENV NO_PROXY=${NO_PROXY}

RUN apt-get update && apt-get install -y --no-install-recommends \
    android-tools-adb \
    build-essential \
    can-utils \
    ethtool \
    git \
    iproute2 \
    python3-colcon-common-extensions \
    python3-pip \
    python3-setuptools \
    python3-wheel \
    ros-humble-ament-cmake \
    ros-humble-ament-cmake-python \
    ros-humble-builtin-interfaces \
    ros-humble-controller-manager \
    ros-humble-geometry-msgs \
    ros-humble-gripper-controllers \
    ros-humble-joint-state-publisher \
    ros-humble-joint-state-publisher-gui \
    ros-humble-launch \
    ros-humble-launch-ros \
    ros-humble-robot-state-publisher \
    ros-humble-ros2-control \
    ros-humble-ros2-controllers \
    ros-humble-rosidl-default-generators \
    ros-humble-rosidl-default-runtime \
    ros-humble-rviz2 \
    ros-humble-sensor-msgs \
    ros-humble-std-msgs \
    ros-humble-std-srvs \
    ros-humble-tf2-ros \
    ros-humble-topic-tools \
    ros-humble-trajectory-msgs \
    ros-humble-xacro \
  && rm -rf /var/lib/apt/lists/*

RUN python3 -m pip install --no-cache-dir --upgrade pip setuptools wheel \
  && python3 -m pip install --no-cache-dir \
    casadi \
    meshcat \
    numpy \
    pure-python-adb \
    python-can \
    pyyaml \
    scipy \
  && (python3 -m pip install --no-cache-dir pin==3.2.0 || python3 -m pip install --no-cache-dir pin)

ENV ROS_DISTRO=humble
ENV ROS_LOCALHOST_ONLY=0
ENV PYTHONUNBUFFERED=1

WORKDIR /work/QuestArmTeleop
