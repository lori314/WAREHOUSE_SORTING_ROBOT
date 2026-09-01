# Warehouse Tuning

现场调参包，用来生成实机部署前需要的地图、A/B/C 区域位姿和小方块视觉参数。

实验室真机优先看：

```text
warehouse_tuning/EASY_START.md
warehouse_tuning/REAL_ROBOT_START_HERE.md
```

`EASY_START.md` 是交接和演示用的短版流程。`REAL_ROBOT_START_HERE.md` 从编译、启动底层、建图、定位、标定到分拣给出完整命令。

实机流程不依赖 Gazebo。现场只运行：

```text
warehouse_tuning field_robot_base.launch
warehouse_tuning field_calibration_wizard.py
arm_grab_task stack_sort_field.launch
```

文件名带 `sim`、`demo`、`acceptance` 的脚本/launch 只用于本机仿真验收，不要在真机上启动。

## 推荐：一条命令标定

先按 `REAL_ROBOT_START_HERE.md` 启动终端 1 的真机底层：

```bash
roslaunch warehouse_tuning field_robot_base.launch \
  start_core:=true start_lidar:=true start_kinect:=true start_joy:=true
```

`field_robot_base.launch` 提供通用串口默认值。真机运行建议使用该设备的稳定 `by-id` 路径覆盖：

```text
底盘: core_port:=/dev/serial/by-id/<base-device>
雷达: rplidar_port:=/dev/serial/by-id/<lidar-device>
手柄: /dev/input/js0
```

先执行 `ls -l /dev/serial/by-id`，再用 `core_port:=...` 和 `rplidar_port:=...` 覆盖。`/dev/ttyUSB*` 编号可能在重启或重新插拔后变化。

手柄控制链路在终端 1 中应能看到：

```text
TeleopJoy publish /cmd_vel ...
[wpb_home_core] recv /cmd_vel ...
```

如果这两条都有但机器人仍不动，优先查底盘电源、急停、底盘使能、控制板和 FTDI 线。

分拣前必须先验证 A/B/C 标定位导航，用：

```bash
roslaunch warehouse_tuning field_nav_smoke.launch dry_run:=true
roslaunch warehouse_tuning field_nav_smoke.launch sequence:=A,B,C,A
```

这个测试不夹取、不识别物体，只检查地图定位、`map -> base_footprint`、A/B/C 标定点和 `/cmd_vel` 闭环运动。默认需要在 RViz 用 `2D Pose Estimate` 设置机器人真实位置和车头方向，脚本收到 `/initialpose` 后才会继续。导航移动默认使用类似手柄前进的 `drive_mode:=forward`，主要发 `linear.x`，少用横移。定位收敛后按回车，或另开终端执行 `rosservice call /field_nav_smoke/confirm_localized`。如果确认机器人就在采集过的 A 点且车头一致，可以启动时加 `localization_mode:=zone initial_pose_zone:=A`；如果机器人被挪走、不确定位置，可以加 `localization_mode:=global`，再按提示用手柄慢速移动帮助收敛；不想开 RViz 时加 `rviz:=false`。

如果只想专测 A->B->C，一条命令用这个 demo：

```bash
roslaunch warehouse_tuning field_nav_abc_demo.launch
```

它固定跑 `A,B,C`，启动后先在 RViz 点 `2D Pose Estimate`。默认不再等额外回车确认，点完位姿并收敛后会自动出发；如果你想保留人工确认，加 `confirm_before_cruise:=true`。
这一步是分拣前置门槛：只有 `/scan` 贴合地图、现实运动方向和 RViz 一致、机器人能稳定从 A 到 B 再到 C，才进入分拣。

终端 1 同一时间只能有一组。重复启动会让 Kinect v2 抢 USB，常见表现是 `/kinect2/sd/image_*` 掉到 `0Hz` 或 `LIBUSB_ERROR_BUSY`。

然后在另一个终端运行向导：

```bash
source ~/catkin_ws/devel/setup.bash
rosrun warehouse_tuning field_calibration_wizard.py \
  --manage-stack \
  --keep-managed-stack \
  --rviz \
  --map-prefix $HOME/maps/lab \
  --zone-file $HOME/maps/abc_zones.yaml \
  --feature-file $HOME/maps/cargo_features.yaml \
  --table-height 0.75
```

向导需要读取键盘输入，建议用 `rosrun` 启动，不要把它作为普通 `roslaunch` node 挂到后台。

向导会按顺序完成：

1. 建图：终端持续显示地图尺寸、覆盖率和占用点数量；RViz 地图稳定后按回车保存。
2. 定位：自动重载刚保存的地图并启动 AMCL；在 RViz 用 `2D Pose Estimate` 设置初始位姿，终端收到 `/amcl_pose` 后才进入下一步。
3. A/B/C 区域：把机器人开到对应桌子前，按回车采集；终端会显示采到的 `x/y/yaw`。
4. 小方块特征：每次只放一种颜色到 ROI 中心，按回车采集；终端会显示 HSV 范围、估计尺寸和 debug 图路径。

如果地图已经建好，只重新做定位和标定：

```bash
rosrun warehouse_tuning field_calibration_wizard.py \
  --manage-stack \
  --keep-managed-stack \
  --skip-mapping \
  --rviz \
  --map-prefix $HOME/maps/lab \
  --zone-file $HOME/maps/abc_zones.yaml \
  --feature-file $HOME/maps/cargo_features.yaml
```

向导状态也会发布到：

```text
/warehouse_tuning/field_calibration_status
/warehouse_tuning/mapping_status
/warehouse_tuning/abc_zone_status_A
/warehouse_tuning/abc_zone_status_B
/warehouse_tuning/abc_zone_status_C
/warehouse_tuning/cargo_feature_status_green
/warehouse_tuning/cargo_feature_status_red
```

标定完成后先做 A/B/C 导航验收；验收通过后，再用生成文件启动实机分拣：

```bash
roslaunch arm_grab_task stack_sort_field.launch \
  map_file:=$HOME/maps/lab.yaml \
  use_field_override:=true \
  field_override:=$HOME/maps/abc_zones.yaml \
  use_feature_override:=true \
  feature_override:=$HOME/maps/cargo_features.yaml \
  rviz:=true
```

默认分拣启动后会像导航测试一样停在 `LOCALIZING`：先等 `/map`、`/scan`、`/odom`，再等 RViz 的 `/initialpose`，然后等 `/amcl_pose` 和 TF 稳定。点完 `2D Pose Estimate` 不会自动开跑；确认红色 `/scan` 贴合黑色地图墙线后，在分拣终端按回车，或另开终端执行 `rosservice call /stack_sort_pipeline/confirm_start`。只有确认机器人就在采集过的 A/B/C 点且朝向一致时，才显式加 `set_initial_pose_from_zone:=true initial_pose_zone:=A require_initial_pose_before_start:=false`。
真机分拣默认 `search_spin_when_no_target:=false`，进入 `SEARCH` 后如果相机没有可选目标会停车并打印 `[SEARCH] no selectable target`，不会盲目原地转圈。
夹取阶段默认复用 WaterPlus `grab_demo` 的 `/wpb_home/grab_action` 行为；主流程只负责锁定颜色，收到 `/wpb_home/grab_result=done` 后再按颜色去 B/C。
放置阶段会先保持闭合并抬到安全高度，再到 B/C；到点后张开到最大，等待确认，然后保持张开后退离桌。
视觉默认只接受正方体块形状目标；启动时也会先复位机械臂到安全高度，默认夹取/放置都保留约 5cm 桌面间隙。
如果 AMCL 或 `/scan` 断档，主流程会触发 `[LOCALIZATION-FAULT]` 并回到手动定位确认，避免定位崩溃后继续发速度。

常用调整：

```bash
# 桌高
--table-height 0.76

# 堆叠点离机器人更远/更近
--stack-anchor-forward-offset 0.54

# 相机 ROI，比例坐标
--feature-roi-x 0.30 --feature-roi-y 0.42 --feature-roi-width 0.35 --feature-roi-height 0.36 --feature-min-area 500

# 如果相机 topic 不同
--rgb-topic /camera/color/image_raw --depth-topic /camera/depth/image_raw --camera-info-topic /camera/color/camera_info
```

交互按键：`回车` 确认，`r` 重做当前提示，`s` 跳过当前项，`q` 退出。

## 0. 仿真全链路验收

在实验室上机前，可以先跑完整流程：仿真建图、保存地图、重新加载地图定位、设置初始位姿、采集 A/B/C、采集小方块特征、加载生成配置并完成分拣。

真实仿真建图演示：机器人会发布 `/cmd_vel` 沿航点巡航，`slam_gmapping` 使用 `/scan` 和 `/odom` 更新 `/map`，再保存地图。`--box-layout jittered` 会按 seed 扰动 6 个箱子在 A 桌上的位置。

```bash
source ~/catkin_ws/devel/setup.bash
rosrun arm_grab_task run_stack_sort_field_tuning_acceptance.py \
  --mapping-mode gmapping \
  --mapping-route short_loop \
  --box-layout jittered \
  --box-seed 11 \
  --timeout 900 \
  --expected-per-color 3
```

复杂地图逻辑压测：这里使用 mock map 生成复杂占用栅格，用来压测地图保存、重载、定位、A/B/C 配置和分拣链路，不代表机器人真实绕场建图。

```bash
rosrun arm_grab_task run_stack_sort_field_tuning_acceptance.py \
  --mapping-mode mock \
  --map-profile complex_lab \
  --box-layout jittered \
  --box-seed 21 \
  --timeout 900 \
  --expected-per-color 3
```

通过后会输出 `PASS`，并生成：

```text
/tmp/warehouse_tuning_sim/lab.yaml
/tmp/warehouse_tuning_sim/abc_zones.yaml
/tmp/warehouse_tuning_sim/cargo_features.yaml
/tmp/warehouse_tuning_sim/debug_images/
```

定位验证看两行日志：`localization seed from current robot pose=(...)` 表示建图结束时机器人在地图中的初始估计，`initial pose verified` 表示 `/amcl_pose` 已接受该初始位姿。实机上这一步对应 RViz 的 `2D Pose Estimate`。

仿真采集小方块特征时，验收脚本会显式打开仿真回退参数。向导和手动服务的实机默认值都是 `allow_simulated_fallback:=false`，现场采集必须让样品进入相机 ROI。

## 手动排障流程

下面这些命令保留给排障使用。正常现场标定优先用上面的 `field_calibration_wizard.py`。

### 1. 建图

实机建图要让机器人实际走过 A/B/C 三张桌子的通道，不要只原地转。保存地图前确认 RViz 里地图边界、桌子/墙体轮廓已经稳定。

```bash
roslaunch warehouse_tuning mapping_session.launch map_prefix:=$HOME/maps/lab
rosservice call /warehouse_tuning/save_map
```

生成 `$HOME/maps/lab.yaml` 和 `$HOME/maps/lab.pgm`。如果已经手动启动 gmapping：

```bash
roslaunch warehouse_tuning mapping_session.launch start_gmapping:=false map_prefix:=$HOME/maps/lab
```

重新加载地图后，必须给初始位姿，否则机器人只知道地图，不知道自己在地图中的位置。实机如果用 AMCL，就在 RViz 用 `2D Pose Estimate`，或用等价的 `/initialpose` 发布工具。确认 `/amcl_pose` 已接近实际位置后，再采集 A/B/C。

### 2. 标定 A/B/C 区域

把机器人手动开到对应桌子前，车头朝向桌面中心。每次只开一个采集节点，然后调用服务。

```bash
roslaunch warehouse_tuning abc_zone_capture.launch zone_name:=A zone_role:=source table_height:=0.75 output_file:=$HOME/maps/abc_zones.yaml
rosservice call /warehouse_tuning/capture_abc_zone

roslaunch warehouse_tuning abc_zone_capture.launch zone_name:=B zone_role:=drop color:=green table_height:=0.75 output_file:=$HOME/maps/abc_zones.yaml
rosservice call /warehouse_tuning/capture_abc_zone

roslaunch warehouse_tuning abc_zone_capture.launch zone_name:=C zone_role:=drop color:=red table_height:=0.75 output_file:=$HOME/maps/abc_zones.yaml
rosservice call /warehouse_tuning/capture_abc_zone
```

生成文件里的 `stack_sort_pipeline.tabletop_return_base_target`、`tabletop_drop_base_targets`、`tabletop_stack_anchors` 可以作为实机覆盖参数。
如果堆叠位置不在桌面中心，调整 `stack_anchor_forward_offset` 后重新采 B/C。

### 3. 采集小方块特征

把一种小方块单独放到画面中心 ROI 内，分别采集：

```bash
roslaunch warehouse_tuning cargo_feature_capture.launch cargo_type:=green output_file:=$HOME/maps/cargo_features.yaml
rosservice call /warehouse_tuning/capture_cargo_features

roslaunch warehouse_tuning cargo_feature_capture.launch cargo_type:=red output_file:=$HOME/maps/cargo_features.yaml
rosservice call /warehouse_tuning/capture_cargo_features
```

生成文件会同时写入 `warehouse_sorting.cargo_types`、`stack_sort_pipeline.color_ranges` 和 `stack_sort_pipeline.field_dimensions.box_size`，可作为实机覆盖参数加载。
如果采集失败，启动时加 `save_debug_images:=true debug_output_dir:=$HOME/maps/debug_images`，先看截图里目标是否在 ROI 内，再调 ROI、灯光或 HSV padding。

### 4. 实机启动和观察

```bash
roslaunch arm_grab_task stack_sort_field.launch \
  map_file:=$HOME/maps/lab.yaml \
  use_field_override:=true \
  field_override:=$HOME/maps/abc_zones.yaml \
  use_feature_override:=true \
  feature_override:=$HOME/maps/cargo_features.yaml \
  rviz:=true
rostopic echo /stack_sort/status
```

启动后先在 RViz 点 `2D Pose Estimate`，确认 `/scan` 对齐地图。RViz 看 `/stack_sort/markers`，终端看 `/stack_sort/status`。日志重点看 `[CONFIG]`、`[STATE]`、`[PICK]`、`[PHYS-GRASP]`、`[DROP]`、`[METRICS]`。

也可以把 A/B/C 位姿和小方块特征的两段 `stack_sort_pipeline` 合并到同一个现场覆盖文件，只加载一次。

常改参数在 `config/lab_tuning.yaml` 和 `arm_grab_task/config/stack_sort_abc_tabletop_params.yaml`：相机 topic、ROI、HSV padding、桌高、箱体尺寸、夹爪开合、取放高度、堆叠层高。
