#!/usr/bin/env bash
set -euo pipefail

cd "$(dirname "$0")"

if [[ -f devel/setup.bash ]]; then
  set +u
  source devel/setup.bash
  set -u
elif [[ -f /opt/ros/noetic/setup.bash ]]; then
  set +u
  source /opt/ros/noetic/setup.bash
  set -u
fi

if ! rostopic list >/dev/null 2>&1; then
  setsid roscore > /tmp/questvr_roscore.log 2>&1 &
  sleep 4
fi

roslaunch oculus_reader yuanyou_teleop_double_piper.launch "$@"
