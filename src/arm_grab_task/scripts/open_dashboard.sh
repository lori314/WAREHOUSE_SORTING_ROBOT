#!/bin/bash
# open_dashboard.sh
# Wait for rosbridge and web_video_server to be ready, then open the dashboard.
# If running on the robot, opens local browser via file://
# Always prints the remote access URL for other computers on the network.
# Invoked as a ROS node by web_dashboard.launch / web_bridges_only.launch.

set -e

WEB_PORT="${1:-8000}"

# Locate dashboard.html — works both in devel space and source space
if command -v rospack &>/dev/null; then
  DASHBOARD_PATH="$(rospack find arm_grab_task 2>/dev/null)/web/dashboard.html"
fi
if [ -z "$DASHBOARD_PATH" ] || [ ! -f "$DASHBOARD_PATH" ]; then
  SCRIPT_DIR="$(cd "$(dirname "$0")" && pwd)"
  DASHBOARD_PATH="$SCRIPT_DIR/../web/dashboard.html"
fi

# Detect robot IP for remote access hint
ROBOT_IP=$(hostname -I 2>/dev/null | awk '{print $1}')
if [ -z "$ROBOT_IP" ]; then
  ROBOT_IP="<robot-ip>"
fi

# Wait for rosbridge
echo "[dashboard] Waiting for rosbridge on port 9090..."
for i in $(seq 1 15); do
  if curl -s -o /dev/null -w '%{http_code}' http://localhost:9090 2>/dev/null | grep -q '200\|426\|400'; then
    echo "[dashboard] rosbridge is ready (attempt $i)"
    break
  fi
  sleep 1
done

# Wait for web_video_server
echo "[dashboard] Waiting for web_video_server on port 8080..."
for i in $(seq 1 15); do
  if curl -s -o /dev/null -w '%{http_code}' http://localhost:8080 2>/dev/null | grep -q '200\|404\|301'; then
    echo "[dashboard] web_video_server is ready (attempt $i)"
    break
  fi
  sleep 1
done

# Wait for static file server
echo "[dashboard] Waiting for static file server on port $WEB_PORT..."
for i in $(seq 1 10); do
  if curl -s -o /dev/null -w '%{http_code}' "http://localhost:$WEB_PORT/dashboard.html" 2>/dev/null | grep -q '200'; then
    echo "[dashboard] static file server is ready (attempt $i)"
    break
  fi
  sleep 1
done

# Open local browser on the robot
if [ -f "$DASHBOARD_PATH" ]; then
  echo "[dashboard] Opening local browser..."
  xdg-open "file://$DASHBOARD_PATH" 2>/dev/null || \
    sensible-browser "file://$DASHBOARD_PATH" 2>/dev/null || \
    echo "[dashboard] (could not auto-open browser)"
else
  echo "[dashboard] ERROR: dashboard.html not found at $DASHBOARD_PATH"
  exit 1
fi

echo ""
echo "━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━"
echo "  🌐 仪表盘已启动"
echo ""
echo "  工控机本地:    file://$DASHBOARD_PATH"
echo "  远程电脑访问:  http://$ROBOT_IP:$WEB_PORT/dashboard.html"
echo ""
echo "  端口占用:"
echo "    :9090  rosbridge (WebSocket — ROS ↔ 浏览器)"
echo "    :8080  web_video_server (MJPEG 相机流)"
echo "    :$WEB_PORT   static file server (仪表盘 HTML)"
echo "━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━"
echo ""

# Stay alive so roslaunch doesn't kill us
sleep 3
exit 0
