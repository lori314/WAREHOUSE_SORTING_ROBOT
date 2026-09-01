#!/bin/bash
# start_dashboard.sh
# 一条命令启动全部：硬件底层 + 分拣 pipeline + Web 仪表盘
# 用法: ./start_dashboard.sh

set -e

# ── 路径（自动从脚本位置推导，无需修改）──
REPO_DIR="$(cd "$(dirname "$0")" && pwd)"
# start_dashboard.sh 放在 catkin 工作空间根目录；如果被移动到包目录，再向上查找 src/。
if [ -z "${CATKIN_WS:-}" ]; then
  if [ -d "$REPO_DIR/src" ] && [ -f "$REPO_DIR/src/CMakeLists.txt" ]; then
    CATKIN_WS="$REPO_DIR"
  elif [ -d "$REPO_DIR/../src" ] && [ -f "$REPO_DIR/../src/CMakeLists.txt" ]; then
    CATKIN_WS="$(cd "$REPO_DIR/.." && pwd)"
  elif [ -d "$REPO_DIR/../../src" ] && [ -f "$REPO_DIR/../../src/CMakeLists.txt" ]; then
    CATKIN_WS="$(cd "$REPO_DIR/../.." && pwd)"
  else
    CATKIN_WS="$REPO_DIR"
  fi
fi
# 约定: 标定文件放在 ~/maps/ 下（与 REAL_ROBOT_START_HERE.md 一致）
MAPS_DIR="${MAPS_DIR:-$HOME/maps}"
MAP_FILE="${MAP_FILE:-$MAPS_DIR/lab.yaml}"
ZONE_FILE="${ZONE_FILE:-$MAPS_DIR/abc_zones.yaml}"
FEATURE_FILE="${FEATURE_FILE:-$MAPS_DIR/cargo_features.yaml}"

# ── 可选覆盖（环境变量优先）──
START_JOY="${START_JOY:-true}"
CONFIRM_BEFORE_START="${CONFIRM_BEFORE_START:-true}"
START_ROBOT_BASE="${START_ROBOT_BASE:-true}"
AUTO_INIT_POSE="${AUTO_INIT_POSE:-false}"
INIT_POSE_ZONE="${INIT_POSE_ZONE:-A}"
SHOW_RVIZ="${SHOW_RVIZ:-false}"
WPB_GRAB_Y_OFFSET="${WPB_GRAB_Y_OFFSET:-0.00}"
WPB_GRAB_Y_SIDE_OFFSET_ENABLED="${WPB_GRAB_Y_SIDE_OFFSET_ENABLED:-true}"
WPB_GRAB_Y_LEFT_OFFSET="${WPB_GRAB_Y_LEFT_OFFSET:--0.05}"
WPB_GRAB_Y_CENTER_OFFSET="${WPB_GRAB_Y_CENTER_OFFSET:-0.00}"
WPB_GRAB_Y_RIGHT_OFFSET="${WPB_GRAB_Y_RIGHT_OFFSET:--0.025}"
WPB_GRAB_Y_SIDE_DEADBAND="${WPB_GRAB_Y_SIDE_DEADBAND:-0.015}"
WPB_GRAB_Y_COLOR_OFFSET_ENABLED="${WPB_GRAB_Y_COLOR_OFFSET_ENABLED:-true}"
WPB_GRAB_Y_GREEN_OFFSET_DELTA="${WPB_GRAB_Y_GREEN_OFFSET_DELTA:-0.00}"
WPB_GRAB_Y_RED_OFFSET_DELTA="${WPB_GRAB_Y_RED_OFFSET_DELTA:-0.04}"

# ── 命令行选项 ──
while [ $# -gt 0 ]; do
  case "$1" in
    --map-file=*)       MAP_FILE="${1#*=}" ;;
    --zone-file=*)      ZONE_FILE="${1#*=}" ;;
    --feature-file=*)   FEATURE_FILE="${1#*=}" ;;
    --catkin-ws=*)      CATKIN_WS="${1#*=}" ;;
    --grab-y-offset=*)  WPB_GRAB_Y_OFFSET="${1#*=}" ;;
    --grab-y-left-offset=*)   WPB_GRAB_Y_LEFT_OFFSET="${1#*=}" ;;
    --grab-y-center-offset=*) WPB_GRAB_Y_CENTER_OFFSET="${1#*=}" ;;
    --grab-y-right-offset=*)  WPB_GRAB_Y_RIGHT_OFFSET="${1#*=}" ;;
    --grab-y-side-deadband=*) WPB_GRAB_Y_SIDE_DEADBAND="${1#*=}" ;;
    --grab-y-green-delta=*)   WPB_GRAB_Y_GREEN_OFFSET_DELTA="${1#*=}" ;;
    --grab-y-red-delta=*)     WPB_GRAB_Y_RED_OFFSET_DELTA="${1#*=}" ;;
    --disable-grab-y-side-offset) WPB_GRAB_Y_SIDE_OFFSET_ENABLED=false ;;
    --disable-grab-y-color-offset) WPB_GRAB_Y_COLOR_OFFSET_ENABLED=false ;;
    --maps-dir=*)       MAPS_DIR="${1#*=}"
                        MAP_FILE="$MAPS_DIR/lab.yaml"
                        ZONE_FILE="$MAPS_DIR/abc_zones.yaml"
                        FEATURE_FILE="$MAPS_DIR/cargo_features.yaml" ;;
    --no-joy)           START_JOY=false ;;
    --confirm)          CONFIRM_BEFORE_START=true ;;
    --sim)              START_ROBOT_BASE=false ;;
    --auto)             AUTO_INIT_POSE=true
                        CONFIRM_BEFORE_START=false ;;
    --rviz)             SHOW_RVIZ=true ;;
    --pose-zone=*)      INIT_POSE_ZONE="${1#*=}" ;;
    --help|-h)
      echo "用法: $0 [选项]"
      echo ""
      echo "路径均自动推导:"
      echo "  仓库:       $REPO_DIR"
      echo "  工作空间:   $CATKIN_WS"
      echo "  标定文件:   $MAPS_DIR/"
      echo ""
      echo "选项:"
      echo "  --map-file=PATH        地图文件"
      echo "  --zone-file=PATH       区域文件"
      echo "  --feature-file=PATH    特征文件"
      echo "  --catkin-ws=PATH       覆盖工作空间路径"
      echo "  --grab-y-offset=VALUE        关闭左右补偿时使用的全局横向补偿，默认 0.00"
      echo "  --grab-y-left-offset=VALUE   摄像机中线左侧物体补偿，默认 -0.05"
      echo "  --grab-y-center-offset=VALUE 摄像机中线附近物体补偿，默认 0.00"
      echo "  --grab-y-right-offset=VALUE  摄像机中线右侧物体补偿，默认 -0.025"
      echo "  --grab-y-side-deadband=VALUE 左右分区死区，默认 0.015m"
      echo "  --grab-y-green-delta=VALUE   绿色颜色增量，默认 0.00"
      echo "  --grab-y-red-delta=VALUE     红色颜色增量，默认 0.04"
      echo "  --disable-grab-y-side-offset 禁用左右补偿，回退到 --grab-y-offset"
      echo "  --disable-grab-y-color-offset 禁用颜色补偿"
      echo "  --maps-dir=PATH        覆盖标定文件目录"
      echo "  --no-joy               禁用手柄（演示时推荐）"
      echo "  --rviz                 同时打开 RViz（需手动 2D Pose Estimate 时用）"
      echo "  --confirm              定位后等待前端按钮确认再开跑（默认）"
      echo "  --sim                  仿真模式，跳过硬件底层"
      echo "  --auto                 自动设置初始位姿并跳过确认（机器人必须在标定原点）"
      echo "  --pose-zone=ZONE       初始位姿来源 (默认: A)"
      echo ""
      echo "常用组合:"
      echo "  # 全自动，机器人停在 A 区原点"
      echo "  $0 --auto"
      echo ""
      echo "  # 需要手动点位姿（开 RViz），点完后在网页点确认"
      echo "  $0 --rviz"
      echo ""
      echo "  # 仿真调试"
      echo "  $0 --sim"
      echo ""
      echo "环境变量:"
      echo "  CATKIN_WS, MAPS_DIR, MAP_FILE, ZONE_FILE, FEATURE_FILE"
      echo "  START_JOY, CONFIRM_BEFORE_START, START_ROBOT_BASE"
      echo "  AUTO_INIT_POSE, INIT_POSE_ZONE, SHOW_RVIZ"
      echo "  WPB_GRAB_Y_OFFSET, WPB_GRAB_Y_LEFT_OFFSET, WPB_GRAB_Y_CENTER_OFFSET"
      echo "  WPB_GRAB_Y_RIGHT_OFFSET, WPB_GRAB_Y_SIDE_OFFSET_ENABLED"
      echo "  WPB_GRAB_Y_GREEN_OFFSET_DELTA, WPB_GRAB_Y_RED_OFFSET_DELTA"
      exit 0
      ;;
    *) echo "未知参数: $1 (用 --help 查看帮助)"; exit 1 ;;
  esac
  shift
done

# ── 预检 ──
echo "══════════════════════════════════════"
echo "  仓库分拣机器人 — Web 仪表盘启动"
echo "══════════════════════════════════════"
echo ""

missing=()
[ -f "$MAP_FILE" ]     || missing+=("地图文件: $MAP_FILE")
[ -f "$ZONE_FILE" ]    || missing+=("区域文件: $ZONE_FILE")
[ -f "$FEATURE_FILE" ] || missing+=("特征文件: $FEATURE_FILE")

if [ ${#missing[@]} -gt 0 ]; then
  echo "⚠  以下文件不存在:"
  for f in "${missing[@]}"; do echo "   - $f"; done
  echo ""
  echo "请先运行标定向导:"
  echo "  rosrun warehouse_tuning field_calibration_wizard.py --manage-stack --keep-managed-stack --rviz \\"
  echo "    --map-prefix $MAPS_DIR/lab --zone-file $ZONE_FILE --feature-file $FEATURE_FILE --table-height 0.75"
  echo ""
  read -rp "缺少文件，是否继续启动？(y/N) " yn
  case "$yn" in [yY]*) ;; *) exit 1 ;; esac
fi

# ── 依赖检查 ──
echo "[check] 检查 ROS 依赖..."
if [ -f /opt/ros/noetic/setup.bash ]; then
  # shellcheck disable=SC1091
  source /opt/ros/noetic/setup.bash
else
  echo "[check] 未找到 /opt/ros/noetic/setup.bash，请先安装 ROS Noetic"
  exit 1
fi

check_ros_pkg() {
  local ros_pkg="$1"
  local deb_pkg="$2"
  if ! rospack find "$ros_pkg" >/dev/null 2>&1; then
    echo "[check] 缺少 ROS 包 $ros_pkg，请安装:"
    echo "  sudo apt-get install $deb_pkg"
    exit 1
  fi
}

check_ros_pkg rosbridge_server ros-noetic-rosbridge-suite
check_ros_pkg web_video_server ros-noetic-web-video-server
echo "[check] 依赖 OK"

# ── 编译检查 ──
if [ ! -f "$CATKIN_WS/devel/setup.bash" ]; then
  echo "[check] 工作空间未编译，正在编译..."
  ( cd "$CATKIN_WS"
    source /opt/ros/noetic/setup.bash
    catkin_make -DCATKIN_WHITELIST_PACKAGES="wpb_home_behaviors;arm_grab_task;warehouse_sorting;warehouse_sorting_msgs;warehouse_tuning"
  )
fi

# ── 环境加载 ──
echo "[env] 加载 ROS 环境..."
source /opt/ros/noetic/setup.bash
source "$CATKIN_WS/devel/setup.bash"

# ── 确认 ──
echo ""
echo "启动参数:"
echo "  地图:       $MAP_FILE"
echo "  区域:       $ZONE_FILE"
echo "  特征:       $FEATURE_FILE"
echo "  硬件层:     $START_ROBOT_BASE"
echo "  手柄:       $START_JOY"
echo "  人工确认:   $CONFIRM_BEFORE_START"
echo "  RViz:       $SHOW_RVIZ"
echo "  自动位姿:   $AUTO_INIT_POSE (zone=$INIT_POSE_ZONE)"
echo "  抓取Y全局补偿: $WPB_GRAB_Y_OFFSET"
echo "  抓取Y左右补偿: $WPB_GRAB_Y_SIDE_OFFSET_ENABLED"
echo "    左侧:     $WPB_GRAB_Y_LEFT_OFFSET"
echo "    中线:     $WPB_GRAB_Y_CENTER_OFFSET"
echo "    右侧:     $WPB_GRAB_Y_RIGHT_OFFSET"
echo "    死区:     $WPB_GRAB_Y_SIDE_DEADBAND m"
echo "  抓取Y颜色补偿: $WPB_GRAB_Y_COLOR_OFFSET_ENABLED"
echo "    绿色增量: $WPB_GRAB_Y_GREEN_OFFSET_DELTA"
echo "    红色增量: $WPB_GRAB_Y_RED_OFFSET_DELTA"
echo ""

# ── 启动 ──
# --auto: 从标定文件取初始位姿，跳过 RViz 手动点位姿步骤
if [ "$AUTO_INIT_POSE" = "true" ]; then
  SET_INIT_POSE="set_initial_pose_from_zone:=true"
  REQUIRE_POSE="require_initial_pose_before_start:=false"
  POSE_ZONE="initial_pose_zone:=$INIT_POSE_ZONE"
else
  SET_INIT_POSE=""
  REQUIRE_POSE=""
  POSE_ZONE=""
fi

echo "[launch] 启动 web_dashboard.launch ..."
exec roslaunch arm_grab_task web_dashboard.launch \
  map_file:="$MAP_FILE" \
  use_field_override:=true \
  field_override:="$ZONE_FILE" \
  use_feature_override:=true \
  feature_override:="$FEATURE_FILE" \
  start_robot_base:="$START_ROBOT_BASE" \
  start_joy:="$START_JOY" \
  confirm_before_start:="$CONFIRM_BEFORE_START" \
  wpb_grab_y_offset:="$WPB_GRAB_Y_OFFSET" \
  wpb_grab_y_side_offset_enabled:="$WPB_GRAB_Y_SIDE_OFFSET_ENABLED" \
  wpb_grab_y_left_offset:="$WPB_GRAB_Y_LEFT_OFFSET" \
  wpb_grab_y_center_offset:="$WPB_GRAB_Y_CENTER_OFFSET" \
  wpb_grab_y_right_offset:="$WPB_GRAB_Y_RIGHT_OFFSET" \
  wpb_grab_y_side_deadband:="$WPB_GRAB_Y_SIDE_DEADBAND" \
  wpb_grab_y_color_offset_enabled:="$WPB_GRAB_Y_COLOR_OFFSET_ENABLED" \
  wpb_grab_y_green_offset_delta:="$WPB_GRAB_Y_GREEN_OFFSET_DELTA" \
  wpb_grab_y_red_offset_delta:="$WPB_GRAB_Y_RED_OFFSET_DELTA" \
  rviz:="$SHOW_RVIZ" \
  $SET_INIT_POSE \
  $REQUIRE_POSE \
  $POSE_ZONE
