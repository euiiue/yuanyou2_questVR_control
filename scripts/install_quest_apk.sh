#!/usr/bin/env bash
set -euo pipefail

ROOT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
APK="${1:-$ROOT_DIR/src/oculus_reader/APK/teleop-debug.apk}"

if ! command -v adb >/dev/null 2>&1; then
  echo "adb not found. Install it with: sudo apt install android-tools-adb"
  exit 1
fi

if [ ! -f "$APK" ]; then
  echo "APK not found: $APK"
  exit 1
fi

adb kill-server >/dev/null 2>&1 || true
adb start-server >/dev/null
adb devices -l

if ! adb devices | awk 'NR > 1 && $2 == "device" { found=1 } END { exit found ? 0 : 1 }'; then
  echo "Quest is not authorized. Wear the headset and allow USB debugging, then rerun."
  exit 1
fi

adb install -r "$APK"

