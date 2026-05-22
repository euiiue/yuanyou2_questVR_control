#!/usr/bin/env bash
set -euo pipefail

cd "$(dirname "$0")"

python3 jetson_port_forward.py \
  --host "${JETSON_HOST:-192.168.31.230}" \
  --user "${JETSON_USER:-yuanyou}"
