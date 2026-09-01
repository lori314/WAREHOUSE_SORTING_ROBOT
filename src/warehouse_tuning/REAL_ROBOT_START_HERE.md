# 真机分拣测试手册

只想快速交给老师或同学上手，先看 `warehouse_tuning/EASY_START.md`。这份文档保留完整流程和排障细节。

这份文档只写真机流程。不要运行任何 `sim_*`、`*_demo.launch`、`run_stack_sort_*acceptance.py`，这些只用于本机仿真。

当前真机流程不需要 Gazebo。若出现 `gazebo_msgs` 相关编译或 import 错误，说明用的是旧包或旧环境缓存。

## 0. 现场测试原则

每个新终端先执行：

```bash
cd ~/catkin_ws
source /opt/ros/noetic/setup.bash
source devel/setup.bash
```

长期运行终端建议固定为：

```text
终端 1: 真机底层，底盘/雷达/Kinect/手柄，只开一份
终端 2: 建图/标定向导，或单独定位测试
终端 3: 分拣主流程，机械臂和底盘会真实动作
终端 4: 状态、参数和日志观察
```

现场每次测试按这个顺序走：

1. 终端 1 启动真机底层，确认 `/odom`、`/scan`、Kinect 图像都有数据。
2. 如果地图和 A/B/C 点已经采好，先跑 `field_nav_smoke.launch dry_run:=true`。
3. 必须再跑一次 A/B/C 真巡航，确认不识别、不夹取时机器人能稳定从 A 到 B/C。
4. 最后启动 `stack_sort_field.launch` 做分拣。
5. 出问题时先看 `/stack_sort/status` 的 `state`，不要凭 RViz 或相机画面猜。

安全要求：

- 分拣 launch 启动后机械臂和底盘会真实动作，桌边不要放手。
- 急停和电源开关必须有人能立即碰到。
- 同一时间只能保留一个终端 1；重复启动会让 Kinect 掉到 `0Hz` 或 USB busy。
- 只测导航时不要启动分拣。

默认颜色是 `green` 和 `red`。如果现场换了颜色，先重新采颜色特征，不要只看相机窗口觉得“识别到了”就继续跑。

## 1. 初次部署或更新代码

假设 zip 已经放在 `~/Downloads/field_ready_sorting_src_20260602.zip`：

```bash
export SORTING_ZIP=$HOME/Downloads/field_ready_sorting_src_20260602.zip
mkdir -p ~/catkin_ws/src
cd ~/catkin_ws/src

STAMP=$(date +%Y%m%d_%H%M%S)
for d in arm_grab_task warehouse_sorting warehouse_sorting_msgs warehouse_tuning; do
  if [ -d "$d" ]; then mv "$d" "$d.bak.$STAMP"; fi
done

unzip -o "$SORTING_ZIP"

cd ~/catkin_ws
source /opt/ros/noetic/setup.bash
catkin_make -DCATKIN_WHITELIST_PACKAGES="kinect2_registration;kinect2_bridge;wpb_home_bringup;wpb_home_behaviors;arm_grab_task;warehouse_sorting;warehouse_sorting_msgs;warehouse_tuning"
source devel/setup.bash
```

检查包是否能找到：

```bash
for p in arm_grab_task warehouse_tuning warehouse_sorting warehouse_sorting_msgs \
         wpb_home_bringup wpb_home_tutorials wpb_home_behaviors rplidar_ros kinect2_registration kinect2_bridge \
         gmapping map_server amcl rviz; do
  rospack find "$p" >/dev/null && echo "[OK] $p" || echo "[MISSING] $p"
done
```

必须全部是 `[OK]`。如果 `wpb_home_*`、`rplidar_ros`、`kinect2_bridge` 缺失，不要继续，这是实验室机器人基础包没有在当前工作空间里。

代码改过后至少重新编译一次，并重启相关 launch：

```bash
cd ~/catkin_ws
source /opt/ros/noetic/setup.bash
catkin_make -DCATKIN_WHITELIST_PACKAGES="kinect2_registration;kinect2_bridge;wpb_home_bringup;wpb_home_behaviors;arm_grab_task;warehouse_sorting;warehouse_sorting_msgs;warehouse_tuning"
source devel/setup.bash
```

## 2. 终端 1：启动真机底层

这个终端不要关。它启动底盘、里程计、雷达、雷达滤波、Kinect 和手柄遥控。

```bash
cd ~/catkin_ws
source /opt/ros/noetic/setup.bash
source devel/setup.bash

roslaunch warehouse_tuning field_robot_base.launch \
  start_core:=true \
  start_lidar:=true \
  start_kinect:=true \
  start_joy:=true
```

建图和人工遥控需要 `start_joy:=true`。只测自动导航或分拣时，建议关掉手柄 teleop，避免多个节点同时抢 `/cmd_vel`：

```bash
roslaunch warehouse_tuning field_robot_base.launch \
  start_core:=true \
  start_lidar:=true \
  start_kinect:=true \
  start_joy:=false
```

如果已经重编过当前源码，`wpb_home_js_vel` 只会在摇杆有输入时发速度，松开后只补发一次 0；旧版本会在 30Hz 持续发 0 速度，自动导航会被打断。

当前仓库默认使用稳定的 `/dev/serial/by-id` 设备名。正常情况下不要把雷达口手动写成 `/dev/ttyUSB1`，因为 `ttyUSB*` 会随插拔顺序变化。如果更换了 USB 转串口线，先看当前设备：

```bash
ls -l /dev/serial/by-id /dev/input/js* /dev/ttyUSB*
```

确认没有重复底层进程：

```bash
pgrep -af "field_robot_base|kinect2_bridge|nodelet"
```

如果看到多组 `field_robot_base` 或多组 `kinect2_bridge`，先关闭重复终端，再拔插 Kinect 电源和 USB。不要叠加启动新的终端 1。

## 3. 基础硬件检查

另开终端检查 topic：

```bash
cd ~/catkin_ws
source /opt/ros/noetic/setup.bash
source devel/setup.bash

timeout 5 rostopic hz /odom
timeout 5 rostopic hz /scan
timeout 5 rostopic hz /kinect2/sd/image_color_rect
timeout 5 rostopic hz /kinect2/sd/image_depth_rect
```

期望现象：

- `/odom` 有数据：底盘在线。
- `/scan` 有数据：雷达在线。
- `/kinect2/sd/image_color_rect` 有数据：彩色相机在线。
- `/kinect2/sd/image_depth_rect` 有数据：深度在线。
- 推手柄时终端 1 出现 `TeleopJoy publish /cmd_vel` 和 `[wpb_home_core] recv /cmd_vel`：手柄速度链路在线。

自动导航或分拣前检查 `/cmd_vel`：

```bash
rostopic info /cmd_vel
timeout 3 rostopic hz /cmd_vel
```

`rostopic info` 用来看有哪些节点 advertise 了速度 topic；`rostopic hz` 用来看当前有没有节点正在持续发速度。自动测试还没启动时，`/cmd_vel` 不应该有持续频率。

如果推摇杆机器人不动，先看终端 1 是否有这些日志：

```text
TeleopJoy input axes ...
TeleopJoy publish /cmd_vel ...
[wpb_home_core] recv /cmd_vel ...
[SerialCom]Open OK ...
```

立刻停车命令：

```bash
rostopic pub /cmd_vel geometry_msgs/Twist \
  '{linear: {x: 0.0, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.0}}' -1
```

## 4. 地图和 A/B/C 点检查

先确认文件存在：

```bash
ls -l $HOME/maps/lab.yaml $HOME/maps/lab.pgm
ls -l $HOME/maps/abc_zones.yaml
ls -l $HOME/maps/cargo_features.yaml
```

如果 `abc_zones.yaml` 里的 A/B/C 还是 `(0,0,0)` 占位，脚本会直接停止；先重新跑第 6 节标定。

分拣前必须先验证 A/B/C 导航，不夹取、不识别物体：

```bash
roslaunch warehouse_tuning field_nav_smoke.launch dry_run:=true
```

`dry_run:=true` 只检查地图、AMCL、TF 和 A/B/C 目标，不发实际巡航速度。默认 `localization_mode:=manual`，启动后必须在 RViz 用 `2D Pose Estimate` 点机器人真实位置和朝向；脚本收到 `/initialpose` 后才会继续。

通过后再跑真导航：

```bash
roslaunch warehouse_tuning field_nav_smoke.launch \
  sequence:=A,B,C,A
```

`field_nav_smoke.launch` 会启动 RViz 显示 A/B/C 标签、黄色计划路线和蓝色实际轨迹。真机移动前先确认 RViz 里的机器人、雷达扫描和现实方向一致。导航移动默认使用类似手柄前进的 `drive_mode:=forward`，主要发 `linear.x`，少用横移。

如果确认机器人就在采集过的 A 点，且车头朝向和采 A 点时一致，可以显式使用 A 点初始化：

```bash
roslaunch warehouse_tuning field_nav_smoke.launch \
  sequence:=A,B,C,A \
  localization_mode:=zone \
  initial_pose_zone:=A
```

只想专测 A->B->C，直接用：

```bash
roslaunch warehouse_tuning field_nav_abc_demo.launch
```

这一步是分拣前置门槛。通过标准：

```text
1. RViz 中红色 /scan 能贴住黑色地图墙线。
2. 机器人能从 A 稳定走到 B，再走到 C。
3. 现实运动方向和 RViz 方向一致，没有撞墙、反向或大幅漂移。
```

定位收敛后，真机巡航前需要按回车确认。如果 roslaunch 终端无法接收回车，另开终端执行：

```bash
rosservice call /field_nav_smoke/confirm_localized
```

如果机器人被人挪走、不确定是否还在 A 区，可以临时切到 AMCL 全局定位；它会提示用手柄慢速前后移动或原地转动帮助收敛：

```bash
roslaunch warehouse_tuning field_nav_smoke.launch localization_mode:=global
```

如果现场空间很开阔、确认全向横移更合适，可以临时加：

```bash
roslaunch warehouse_tuning field_nav_smoke.launch \
  sequence:=A,B,C,A \
  drive_mode:=holonomic
```

如果不想开 RViz，启动时加 `rviz:=false`。

### 导航慢、顿挫的快速判断

当前 `field_nav_smoke` 和分拣里的 A/B/C 回点不是标准 `move_base` 全局导航；它们主要是读取 `map -> base_footprint`，对目标点直接闭环发 `/cmd_vel`。所以它不会绕障碍，A/B/C 之间最好是直线可走，点位不要贴桌太近。

现实机器人和 RViz 运动方向不一致时，先停下，不要继续跑。这通常不是“控制器慢”，而是 AMCL 初始位姿、地图或 TF 错了。重新在 RViz 点 `2D Pose Estimate`，确认 `/scan` 能贴合地图墙线后再按回车开始。

先排除 `/cmd_vel` 抢占：

```bash
rostopic info /cmd_vel
timeout 3 rostopic hz /cmd_vel
```

自动导航还没启动时，如果 `/cmd_vel` 已经有持续频率，说明 `teleop`、旧版手柄节点或其他 demo 节点在抢速度；先关掉它们，或者重启终端 1 时用 `start_joy:=false`。

再看定位是否抖：

```bash
rostopic echo -n 1 /amcl_pose
rosrun tf tf_echo map base_footprint
```

机器人静止时，如果 RViz 里的机器人位姿或 `tf_echo` 数值明显跳动，是地图/雷达/AMCL 定位问题；优先重设 `2D Pose Estimate`，必要时重新建图。

如果定位不抖，但 `field_nav_smoke` 日志里 `cmd=(...)` 一会儿有速度、一会儿接近 0，或者角速度正负来回切，是控制器在反复修正目标方向。空旷环境可以试一组更平滑的参数：

```bash
roslaunch warehouse_tuning field_nav_smoke.launch \
  sequence:=A,B,C,A \
  min_linear:=0.10 \
  min_angular:=0.16 \
  path_yaw_tolerance:=0.35 \
  log_interval:=0.3
```

如果只在靠近 B/C 桌前最后 30cm 慢，是采点/目标 yaw 问题；重新采 B/C 点时让车离桌边稍远一点，车头正对桌面中心。

## 5. 分拣前检查清单

启动分拣前必须同时满足：

- 终端 1 仍在运行，而且 `/odom`、`/scan`、Kinect 彩色和深度 topic 都有数据。
- `$HOME/maps/lab.yaml`、`$HOME/maps/abc_zones.yaml`、`$HOME/maps/cargo_features.yaml` 都存在。
- `field_nav_smoke.launch dry_run:=true` 能通过。
- 真导航 `sequence:=A,B,C,A` 能稳定回到 A。
- 待分拣小方块只放在 A 桌，B/C 桌清空。
- 机器人真实位置在 A 区附近，车头朝向和采 A 点时一致。

快速看标定文件里的关键值：

```bash
grep -n -A4 "tabletop_return_base_target" $HOME/maps/abc_zones.yaml
grep -n -A12 "tabletop_drop_base_targets" $HOME/maps/abc_zones.yaml
grep -n -A4 "active_colors" $HOME/maps/cargo_features.yaml
```

如果 A 区取货位或 B/C 放置位接近 `(0,0,0)`，不要启动分拣，先重新标定 A/B/C。

## 6. 建图、定位和标定

没有地图，或者桌子位置变化明显时，跑完整向导：

```bash
cd ~/catkin_ws
source /opt/ros/noetic/setup.bash
source devel/setup.bash

mkdir -p $HOME/maps

rosrun warehouse_tuning field_calibration_wizard.py \
  --manage-stack \
  --keep-managed-stack \
  --rviz \
  --map-prefix $HOME/maps/lab \
  --zone-file $HOME/maps/abc_zones.yaml \
  --feature-file $HOME/maps/cargo_features.yaml \
  --debug-output-dir $HOME/maps/debug_images \
  --table-height 0.75
```

向导步骤：

1. 建图：用手柄遥控机器人绕 A/B/C 三张桌子和通道走一圈。RViz 里地图稳定后，在终端按回车保存。
2. 定位：向导会重载 `$HOME/maps/lab.yaml` 并启动 AMCL。必须在 RViz 点 `2D Pose Estimate`，把机器人箭头放到真实位置和朝向。
3. A/B/C 区域：按提示把机器人开到 A 桌、B 桌、C 桌前，车头对准桌面中心，停稳后按回车。
4. 小方块特征：每次只放一种颜色小方块到相机 ROI 中心，停 1 秒后按回车。

如果已经建过地图，只重新做定位、A/B/C 和颜色特征：

```bash
rosrun warehouse_tuning field_calibration_wizard.py \
  --manage-stack \
  --keep-managed-stack \
  --skip-mapping \
  --rviz \
  --map-prefix $HOME/maps/lab \
  --zone-file $HOME/maps/abc_zones.yaml \
  --feature-file $HOME/maps/cargo_features.yaml \
  --debug-output-dir $HOME/maps/debug_images \
  --table-height 0.75
```

一定带 `--keep-managed-stack`，否则标定完成后 AMCL 会被关掉，后面机器人又会不知道自己在地图里的位置。

标定成功后检查输出：

```bash
ls -l $HOME/maps/lab.yaml $HOME/maps/lab.pgm
ls -l $HOME/maps/abc_zones.yaml
ls -l $HOME/maps/cargo_features.yaml
ls -lt $HOME/maps/debug_images | head
```

## 7. 终端 3：启动分拣

启动分拣前必须完成第 4 节的 A/B/C 导航 dry run 和真巡航验收。如果巡航失败，不要启动分拣，先修地图、定位、A/B/C 点或 `/cmd_vel` 抢占问题。

确认终端 1 还开着，再运行分拣：

```bash
cd ~/catkin_ws
source /opt/ros/noetic/setup.bash
source devel/setup.bash

roslaunch arm_grab_task stack_sort_field.launch \
  map_file:=$HOME/maps/lab.yaml \
  use_field_override:=true \
  field_override:=$HOME/maps/abc_zones.yaml \
  use_feature_override:=true \
  feature_override:=$HOME/maps/cargo_features.yaml \
  rviz:=true
```

启动后主流程会像导航测试一样先停在 `LOCALIZING`。脚本会先等 `/map`、`/scan`、`/odom`，然后等 RViz 的 `/initialpose`，再请求一次 AMCL no-motion update，等待 `/amcl_pose` 和 TF 稳定。点完 `2D Pose Estimate` 不会自动开跑；确认红色 `/scan` 能贴合黑色地图墙线后，在分拣终端按回车。如果终端不能接收回车，另开终端执行：

```bash
rosservice call /stack_sort_pipeline/confirm_start
```

如果红线明显偏，立刻 `Ctrl-C` 停分拣，重新启动后再点位姿。

默认相关参数已经在 launch 里打开：

```text
require_initial_pose_before_start:=true
initial_pose_settle_time:=1.0
initial_pose_request_nomotion_update:=true
confirm_before_start:=true
search_spin_when_no_target:=false
start_wpb_grab_runtime:=true
use_wpb_grab_action:=true
```

期望状态流转：

```text
LOCALIZING -> SEARCH -> PICK -> DROP -> SEARCH -> FINISH
```

现在主流程只用颜色视觉确认目标颜色，锁定颜色后直接进入 `PICK`，默认复用 `grab_demo.launch` 里那套 WaterPlus 抓取行为，但不会重复启动底盘和 Kinect。`stack_sort_field.launch` 只额外启动 `wpb_home_objects_3d`、`wpb_home_grab_action` 和 `/kinect2/qhd/points` 点云生成。日志里应看到：

```text
[STATE] SEARCH -> PICK reason=wpb_direct_color_locked
[PICK-WPB] grab object=...
/wpb_home/grab_result: object x -> hand up -> forward -> grab -> object up -> backward -> done
```

收到 `done` 后，主流程继续按锁定的 `target_color` 去 B 或 C。

视觉检测默认只接受“像正方体块”的目标：颜色检测会用 2D 轮廓长宽比和填充率过滤杂物，`wpb_home_objects_3d` 会用 3D 包围盒尺寸过滤非方块聚类。启动日志里应看到：

```text
[VISION] square_filter ...
[objects_3d] cube_filter=...
```

现场默认不弹 OpenCV 相机调试窗口，避免图像窗口卡死拖住主流程。需要临时看 `stack_sort_debug` 窗口时，在分拣命令末尾加 `show_debug:=true`；正常测试优先看 RViz、`/stack_sort/status` 和 `/wpb_home/grab_result`。

每次启动分拣时，程序会先执行 `[ARM] vision stow ...`，把夹爪收窄并放到不遮挡相机的低位。主流程默认不再提前抬高夹爪或前进靠近桌面；进入 `PICK` 后，WPB 抓取行为会先移动到底盘安全距离，再 `hand up` 抬到物体高度，最后才前进夹取。WPB 的抬臂前安全距离由 `grab_target_x` 控制，当前是 `1.05m`。

夹得不紧时，调小 `wpb_home_bringup/config/wpb_home.yaml` 里的 `grab_gripper_value`。当前默认是 `0.012`；这个值表示闭合后的手指间距，不是力矩，越小夹得越紧，但太小会卡住或把物体顶偏。

`DROP` 阶段会先保持夹爪闭合并抬到安全高度，再导航到 B/C；到点后降到安全释放高度，张开到最大，等待确认，然后保持张开后退离桌。回到下一轮取货前会无条件执行 `[ARM] vision stow reason=after_drop ...`，把机械臂收回低位窄爪状态。正常日志应该有：

```text
[DROP] travel safe pose ...
[DROP] release ...
[DROP] release confirm wait ...
[DROP] retreat from tabletop ...
```

如果 AMCL 崩溃、重启，或者 `/scan` 断流，主流程会触发定位熔断：

```text
[LOCALIZATION-FAULT] ...
```

程序会立刻发零速度、停止 WPB 抓取行为、清除当前目标并回到 `LOCALIZING`。这时必须重新在 RViz 点 `2D Pose Estimate`，确认红色 `/scan` 贴住地图墙线后再按回车；不要直接继续原任务。

如果确认机器人就在采集过的 A 点，且车头朝向和采 A 点时一致，可以显式使用 A 点自动初始化：

```bash
roslaunch arm_grab_task stack_sort_field.launch \
  map_file:=$HOME/maps/lab.yaml \
  use_field_override:=true \
  field_override:=$HOME/maps/abc_zones.yaml \
  use_feature_override:=true \
  feature_override:=$HOME/maps/cargo_features.yaml \
  set_initial_pose_from_zone:=true \
  initial_pose_zone:=A \
  require_initial_pose_before_start:=false \
  rviz:=true
```

机器人只要被人推过、搬过，或者不确定是否还在 A 点，就用默认手动定位流程，不要用这组自动初始化参数。

无论自动还是手动，开始动作前都要确认 `/map`、`/amcl_pose` 和 `map -> base_footprint` 正常：

```bash
rostopic list | grep -E '^/map$|^/amcl_pose$'
rosrun tf tf_echo map base_footprint
```

## 8. 终端 4：监控分拣

看状态：

```bash
cd ~/catkin_ws
source /opt/ros/noetic/setup.bash
source devel/setup.bash

rostopic echo /stack_sort/status
```

`/stack_sort/status` 是 JSON 字符串，重点看：

```text
state                 当前状态
target_color          当前锁定颜色
detections            相机识别到的颜色、中心点、深度、面积
last_depth            最近目标深度，单位是米
last_align_error_px   目标中心和相机中心的像素误差
stack_count           每种颜色已放几个
pick_retry_count      当前目标重试次数
```

看日志关键词：

```text
[CONFIG]  参数是否加载了现场 yaml
[STATE]   状态切换
[SEARCH]  搜索、颜色选择、未选中原因
[ALIGN]   视觉对准
[PICK]    夹取序列
[DROP]    放置目标和层数
[METRICS] 每次循环成功/失败原因
[REPORT]  测试报告路径
```

关键参数：

```bash
rosparam get /stack_sort_pipeline/active_colors
rosparam get /stack_sort_pipeline/pick_stop_depth
rosparam get /stack_sort_pipeline/use_wpb_grab_action
rosparam get /wpb_home_grab_action/grab/grab_gripper_value
rosparam get /wpb_home_grab_action/grab/grab_target_x
rostopic echo /wpb_home/grab_result
rostopic hz /kinect2/qhd/points
rosparam get /stack_sort_pipeline/localization_watchdog_enabled
rosparam get /stack_sort_pipeline/scan_watchdog_timeout
rosparam get /stack_sort_pipeline/amcl_watchdog_timeout
rosparam get /stack_sort_pipeline/show_debug
rosparam get /stack_sort_pipeline/square_filter_enabled
rosparam get /stack_sort_pipeline/square_max_aspect_ratio
rosparam get /stack_sort_pipeline/square_min_fill_ratio
rosparam get /wpb_home_objects_3d/cube_filter_enabled
rosparam get /wpb_home_objects_3d/cube_min_x
rosparam get /wpb_home_objects_3d/cube_max_x
rosparam get /wpb_home_objects_3d/cube_min_z
rosparam get /wpb_home_objects_3d/cube_max_z
rosparam get /wpb_home_objects_3d/table_height_min
rosparam get /wpb_home_objects_3d/table_height_max
rosparam get /stack_sort_pipeline/reset_arm_on_start
rosparam get /stack_sort_pipeline/startup_arm_lift_height
rosparam get /stack_sort_pipeline/vision_arm_stow_enabled
rosparam get /stack_sort_pipeline/vision_arm_lift_height
rosparam get /stack_sort_pipeline/vision_arm_gripper
rosparam get /stack_sort_pipeline/source_travel_lift_height
rosparam get /stack_sort_pipeline/approach_arm_guard_enabled
rosparam get /stack_sort_pipeline/wpb_direct_pick_after_color_lock
rosparam get /stack_sort_pipeline/wpb_pregrab_distance_guard_enabled
rosparam get /stack_sort_pipeline/wpb_pregrab_min_depth
rosparam get /stack_sort_pipeline/wpb_pregrab_target_depth
rosparam get /stack_sort_pipeline/drop_hold_gripper
rosparam get /stack_sort_pipeline/drop_open_gripper
rosparam get /stack_sort_pipeline/drop_release_clearance
rosparam get /stack_sort_pipeline/drop_safe_lift_height
rosparam get /stack_sort_pipeline/drop_release_confirm_seconds
rosparam get /stack_sort_pipeline/align_deadband_px
rosparam get /stack_sort_pipeline/near_pick_align_deadband_px
rosparam get /stack_sort_pipeline/depth_unit_auto_scale
rosparam get /stack_sort_pipeline/approach_realign_error_px
rosparam get /stack_sort_pipeline/tabletop_return_base_target
rosparam get /stack_sort_pipeline/tabletop_drop_base_targets
rosparam get /stack_sort_pipeline/gazebo_use_source_pick_targets
```

真机分拣时 `gazebo_use_source_pick_targets` 必须是 `false`。

## 9. 识别到了但不夹，或者还在乱跑

先不要重启所有东西。先看当前状态：

```bash
rostopic echo -n 1 /stack_sort/status
```

按 `state` 判断：

| `state` | 现象 | 先查什么 |
| --- | --- | --- |
| `LOCALIZING` | 不开始找物体 | `/map`、`/amcl_pose`、`map -> base_footprint`，必要时 RViz 重新点 `2D Pose Estimate` |
| `SEARCH` 且 `detections` 为空 | 相机窗口看得到，但算法没选到 | 颜色特征、ROI、深度 topic、桌面光照，重新跑颜色特征采集 |
| 相机框到杂物 | 方块过滤太宽，或杂物颜色和尺寸太接近 | 看 `[VISION] square_filter` 和 `[objects_3d] cube_filter`，收紧 `square_max_aspect_ratio`、`cube_max_*` |
| `SEARCH` 且有 `[SEARCH] detections present but none selectable` | 检测到了颜色但不能作为目标 | `active_colors`、`stack_count`、颜色名称是否和 `cargo_features.yaml` 一致 |
| `SEARCH` 停住并打印 `[SEARCH] no selectable target` | 没有可夹目标 | 看 A 点车头是否正对 A 桌、目标是否在画面中、颜色特征和 Kinect RGB/depth topic 是否正常 |
| `SEARCH` 一直转圈 | 启用了 `search_spin_when_no_target:=true` 或旧代码 | 真机默认应为 `false`；确认代码已更新并重启分拣 |
| `ALIGN` | 已锁定目标，正在转向对准 | 看 `last_align_error_px` 是否逐渐接近 0 |
| `APPROACH` | 已对准，正在靠近目标 | 看 `last_depth` 是否逐渐下降到 `pick_stop_depth`；如果 `last_align_error_px` 太大，程序会先停住前进只转向重对准 |
| `PICK` 没抓取或动作不对 | WPB 抓取行为没有拿到 3D 物体，或抓取参数不合适 | 看 `[PICK-WPB]`、`/wpb_home/grab_result`、`/kinect2/qhd/points`；动作偏差再调 `wpb_home.yaml` 的 `grab_*` 参数 |
| `PICK` 但机械臂没动 | 分拣逻辑已经发夹取 | 看 `/wpb_home/mani_ctrl`、终端 1 机械臂日志、电源和急停 |
| `DROP` 到点但不放或提前掉 | 放置安全高度、闭合保持值或释放等待不合适 | 看 `[DROP] travel safe pose`、`[DROP] release`，调 `drop_hold_gripper`、`drop_release_clearance`、`drop_safe_lift_height` |

这次遇到的典型问题是：相机识别到了物体，但主流程没有进入视觉 `ALIGN/APPROACH/PICK`，而是反复按 A 区位姿跑。修复后真机应当在锁定目标后走：

```text
SEARCH -> ALIGN -> APPROACH -> PICK
```

如果仍然没有这个状态流转，先确认运行的是新代码：

```bash
rosparam get /stack_sort_pipeline/gazebo_use_source_pick_targets
rostopic echo -n 3 /stack_sort/status
```

真机应该看到 `gazebo_use_source_pick_targets: false`，并且 `/stack_sort/status` 里的 `state` 会从 `SEARCH` 进入 `ALIGN`。

如果已经进入 `ALIGN/APPROACH`，但迟迟不 `PICK`：

```bash
rosparam get /stack_sort_pipeline/pick_stop_depth
rostopic echo /stack_sort/status
```

观察 `last_depth`。如果 `last_depth` 一直大于 `pick_stop_depth`，机器人会继续靠近或重新对准，不会夹取。这里的单位必须是米；正常应看到 `0.7`、`0.8`、`1.0` 这一类数。如果看到 `700`、`800`、`1000`，说明 Kinect 深度仍按毫米进入了流程，先确认 `depth_unit_auto_scale:=true` 并重启分拣。深度单位正常后，再检查目标是否在桌面 ROI 中心、深度图是否稳定、颜色特征是否需要重采。

如果进入 `PICK` 但机械臂没有动作，另开终端看机械臂控制 topic：

```bash
rostopic echo /wpb_home/mani_ctrl
```

同时看终端 1 是否有机械臂/串口报错。这个问题不属于视觉识别，优先查底层机械臂供电、急停、串口和 `wpb_home_core`。

如果已经进入 `PICK`，但没有抓取动作，先看日志：

```text
[PICK-WPB] grab object=...
```

如果没有 `grab object`，先确认 3D 点云和 WPB 物体检测：

```bash
rostopic hz /kinect2/qhd/points
rostopic echo /wpb_home/objects_3d
rostopic echo /wpb_home/grab_result
```

如果已经有 `grab object`，但夹爪碰桌或够不到物体，调 `wpb_home_bringup/config/wpb_home.yaml` 里的 `grab_y_offset`、`grab_lift_offset`、`grab_forward_offset`、`grab_gripper_value`，然后重启终端 3。

如果 `[DROP] failed to reach tabletop drop base target` 或 `[POSE] drop_base_* reached=False`，说明机器人没有到达 B/C 放置底盘点，程序现在会停在 `ERROR`，不会继续放置或记成功。通常是 B/C 点采得太贴桌、地图定位漂了，或者真实桌子位置变了；先重新采 B/C，让车离桌边再远 10-20cm，车头正对桌面中心。

## 10. 标定和识别常见问题

`no colored blob found in ROI`

只放一个样品到画面中心，查看 debug 图：

```bash
ls -lt $HOME/maps/debug_images | head
```

默认 ROI 已下移到桌面区域，默认参数是：

```bash
--feature-roi-x 0.30 --feature-roi-y 0.42 --feature-roi-width 0.35 --feature-roi-height 0.36 --feature-min-area 500
```

如果样品仍不在 ROI 中，按 debug 图继续微调。

夹取位置偏差或桌高不对，重新跑第 6 节，修改桌高或堆叠点距离：

```bash
--table-height 0.76
--stack-anchor-forward-offset 0.54
```

`/scan` 没数据：

```bash
ls -l /dev/rplidar
rosnode list | grep rplidar
```

`/amcl_pose` 没数据或 `/scan` 和地图不重合：

默认分拣命令不会再自动套用 A 点位姿。先在 RViz 里点 `2D Pose Estimate`，让红色 `/scan` 贴合黑色地图墙线；如果 AMCL 崩溃后自动重启，需要重新设置初始位姿。只有确认机器人真的停在采集过的 A/B/C 点且朝向一致时，才使用 `set_initial_pose_from_zone:=true initial_pose_zone:=A/B/C require_initial_pose_before_start:=false`。

`tf lookup map -> base_footprint failed`：

定位没成功。分拣 launch 默认会加载 `$HOME/maps/lab.yaml` 并启动 AMCL；如果仍没有 `/map`，至少重新启动：

```bash
roslaunch warehouse_tuning field_localization.launch \
  map_file:=$HOME/maps/lab.yaml \
  rviz:=true
```

然后在 RViz 点 `2D Pose Estimate`。

`RLException: field_robot_base.launch not found`：

```bash
cd ~/catkin_ws
source /opt/ros/noetic/setup.bash
source devel/setup.bash
rospack find warehouse_tuning
```

找不到就是包没放进 `~/catkin_ws/src` 或没有重新编译。

`No module named gazebo_msgs`：

用的是旧包。当前真机包不需要 Gazebo。重新解压最新 zip，并确认 `arm_grab_task/launch/stack_sort_field.launch` 里有：

```text
gazebo_enable_helper=false
```

`Resource not found ... kinect2_bridge`：

检查 `src/robot-tools-rec/iai_kinect2/CATKIN_IGNORE`。这个文件存在时，当前工作空间会跳过 Kinect2 包，`field_robot_base.launch start_kinect:=true` 就会在 `$(find kinect2_bridge)` 这里报错。只测底盘和导航时把 `start_kinect:=false`；如果要启用 Kinect，就删掉或改名这个 `CATKIN_IGNORE`，然后重新 `catkin_make`，白名单里必须包含 `kinect2_registration;kinect2_bridge`。

## 11. 停止顺序

先在终端 3 按 `Ctrl-C` 停分拣。再发一次零速度：

```bash
rostopic pub /cmd_vel geometry_msgs/Twist \
  '{linear: {x: 0.0, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.0}}' -1
```

然后按顺序关闭终端 2、终端 1。

## 12. 今天只需要记住

```text
1. 先开终端 1: field_robot_base.launch
2. 先测导航: field_nav_smoke.launch dry_run:=true，然后 sequence:=A,B,C,A
3. 再开分拣: stack_sort_field.launch
4. 出问题先看: rostopic echo -n 1 /stack_sort/status
5. 真机应看到: SEARCH -> ALIGN -> APPROACH -> PICK -> DROP
```

现场生成的地图、标定文件和调试记录应保存在机器人本地，不要提交到公开仓库。
