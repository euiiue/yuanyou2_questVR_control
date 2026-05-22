#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
cd "$SCRIPT_DIR"
source "$SCRIPT_DIR/source_ros_setup.sh"

echo "WARNING: real Piper output is enabled. Hold each controller grip/deadman to send commands."
exec python3 quest_vr_piper_adapter.py _publish_real:=true _require_deadman:=true "$@"
