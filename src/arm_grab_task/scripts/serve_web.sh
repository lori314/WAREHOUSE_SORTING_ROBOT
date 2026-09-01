#!/bin/bash
# serve_web.sh
# Start a minimal HTTP server serving the dashboard web/ directory.
# Used so remote computers on the same network can open the dashboard.

set -e

PORT="${1:-8000}"

# Locate web directory
if command -v rospack &>/dev/null; then
  WEB_DIR="$(rospack find arm_grab_task 2>/dev/null)/web"
fi
if [ -z "$WEB_DIR" ] || [ ! -d "$WEB_DIR" ]; then
  SCRIPT_DIR="$(cd "$(dirname "$0")" && pwd)"
  WEB_DIR="$SCRIPT_DIR/../web"
fi

if [ ! -d "$WEB_DIR" ]; then
  echo "[serve_web] ERROR: web directory not found" >&2
  exit 1
fi

echo "[serve_web] Serving $WEB_DIR on port $PORT"
echo "[serve_web] Remote access: http://$(hostname -I 2>/dev/null | awk '{print $1}' || echo '<robot-ip>'):$PORT/dashboard.html"

cd "$WEB_DIR"
exec python3 -m http.server "$PORT"
