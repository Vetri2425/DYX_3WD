#!/usr/bin/env bash
# DYX 3WD — dyx3-backend launcher: REST + Socket.IO + path engine. No ROS, no rclpy (talks to dyx3_system_gateway over its Unix socket).
set -euo pipefail

REL="${DYX3_RELEASE_DIR:-/opt/dyx3/current}"
PY="${REL}/venv/bin/python"
[ -x "${PY}" ] || {
  echo "dyx3-backend: ${PY} missing (the release's backend venv was not built)" >&2
  exit 1
}
# Bind the hotspot address (architecture 4.4), never 0.0.0.0: eth0 is the FCU link and usb0 is WAN.
# DERIVED — NOT FROM V1 SPEC: port 8000. OPEN.
exec "${PY}" -m uvicorn dyx3_backend.main:app --host "${DYX3_BACKEND_HOST:-10.42.0.1}" --port "${DYX3_BACKEND_PORT:-8000}"
