#!/usr/bin/env bash
set -euo pipefail

cd "$(dirname "$0")"

HTTP_PORT="${HTTP_PORT:-8088}"
WS_PORT="${WS_PORT:-8765}"
HTTP_HOST="${HTTP_HOST:-127.0.0.1}"
WS_HOST="${WS_HOST:-127.0.0.1}"
ADB_REVERSE="${ADB_REVERSE:-1}"

args=(
  --http-host "$HTTP_HOST"
  --http-port "$HTTP_PORT"
  --ws-host "$WS_HOST"
  --ws-port "$WS_PORT"
)

if [[ "$ADB_REVERSE" == "1" ]]; then
  args+=(--adb-reverse)
fi

python3 quest_webxr_bridge.py \
  "${args[@]}" \
  "$@"
