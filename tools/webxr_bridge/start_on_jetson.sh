#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
cd "$SCRIPT_DIR"

source "$SCRIPT_DIR/source_ros_setup.sh"

export ADB_REVERSE=0
export HTTP_HOST="${HTTP_HOST:-127.0.0.1}"
export WS_HOST="${WS_HOST:-127.0.0.1}"

exec ./start_webxr_bridge.sh --ros "$@"
