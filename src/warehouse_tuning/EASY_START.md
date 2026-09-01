# Easy Start：真机分拣一页启动

这份给老师或同学现场快速上手用。默认已经有这些文件：

```text
$HOME/maps/lab.yaml
$HOME/maps/abc_zones.yaml
$HOME/maps/cargo_features.yaml
```

详细标定、排障和参数说明看 `warehouse_tuning/REAL_ROBOT_START_HERE.md`。

不要在真机上运行 `sim_*`、`*_demo.launch`、`run_stack_sort_*acceptance.py`。

## 0. 每个新终端先执行

```bash
cd ~/catkin_ws
source /opt/ros/noetic/setup.bash
source devel/setup.bash
```

如果刚更新过代码，先编译一次：

```bash
catkin_make -DCATKIN_WHITELIST_PACKAGES="kinect2_registration;kinect2_bridge;wpb_home_bringup;wpb_home_behaviors;arm_grab_task;warehouse_sorting;warehouse_sorting_msgs;warehouse_tuning"
source devel/setup.bash
```

## 1. 终端 1：启动真机底层

自动导航或分拣演示时推荐关掉手柄 teleop，避免抢 `/cmd_vel`：

```bash
roslaunch warehouse_tuning field_robot_base.launch \
  start_core:=true \
  start_lidar:=true \
  start_kinect:=true \
  start_joy:=false
```

如果要建图或手柄遥控，把最后一行改成：

```bash
start_joy:=true
```

这个终端不要关。不要重复开第二个终端 1。

## 2. 终端 2：快速检查硬件

```bash
timeout 5 rostopic hz /odom
timeout 5 rostopic hz /scan
timeout 5 rostopic hz /kinect2/sd/image_color_rect
timeout 5 rostopic hz /kinect2/sd/image_depth_rect
```

自动程序还没启动前，`/cmd_vel` 不应该有持续频率：

```bash
timeout 3 rostopic hz /cmd_vel
```

如果这里已经有频率，说明有手柄节点或其他 demo 节点在发速度，先关掉它。

## 3. 终端 2：必做导航标定和巡航验收

这是启动分拣前的必要操作，不是可选测试。只有 A/B/C 导航验收通过，才能进入第 4 步分拣；如果这里失败，先修地图、定位、A/B/C 点或 `/cmd_vel` 抢占问题。

先做 dry run，不让机器人动：

```bash
roslaunch warehouse_tuning field_nav_smoke.launch dry_run:=true
```

启动后在 RViz 用 `2D Pose Estimate` 点机器人真实位置和车头方向。脚本收到 `/initialpose` 后才会继续。先确认 RViz 里的机器人、雷达扫描和现实方向一致。

通过后再让机器人真实走一遍：

```bash
roslaunch warehouse_tuning field_nav_smoke.launch sequence:=A,B,C,A
```

启动后 RViz 会显示 A/B/C 标签、计划路线和实际轨迹。再次用 `2D Pose Estimate` 对齐现实位置，定位确认后按回车开始；如果终端不能接收回车，另开终端执行：

```bash
rosservice call /field_nav_smoke/confirm_localized
```

如果 A/B/C 走得很顿，先看：

```bash
timeout 3 rostopic hz /cmd_vel
rosrun tf tf_echo map base_footprint
```

`/cmd_vel` 被其他节点抢，或者机器人静止时 `map -> base_footprint` 明显跳，都会导致顿挫。

如果确认机器人就在采集过的 A 点，且车头朝向和采 A 点时一致，可以用 A 点自动初始化：

```bash
roslaunch warehouse_tuning field_nav_smoke.launch \
  sequence:=A,B,C,A \
  localization_mode:=zone \
  initial_pose_zone:=A
```

只想单独测 A->B->C，就直接跑：

```bash
roslaunch warehouse_tuning field_nav_abc_demo.launch
```

通过标准：

```text
1. RViz 中红色 /scan 能贴住黑色地图墙线。
2. 机器人能从 A 稳定走到 B，再走到 C。
3. 现实运动方向和 RViz 方向一致，没有撞墙、反向或大幅漂移。
```

## 4. 终端 3：启动分拣

必须先完成第 3 步导航验收，再启动分拣。

把待分拣方块放在 A 桌，B/C 桌清空。确认桌边无人手，急停可触达。

```bash
roslaunch arm_grab_task stack_sort_field.launch \
  map_file:=$HOME/maps/lab.yaml \
  use_field_override:=true \
  field_override:=$HOME/maps/abc_zones.yaml \
  use_feature_override:=true \
  feature_override:=$HOME/maps/cargo_features.yaml \
  rviz:=true
```



启动后会和导航测试一样停在 `LOCALIZING`。脚本会先等 `/map`、`/scan`、`/odom`，然后等 RViz 的 `/initialpose`，再等 `/amcl_pose` 和 TF 稳定。点完 `2D Pose Estimate` 不会自动开跑；确认红色 `/scan` 和黑色地图墙线重合后，在分拣终端按回车。终端不能接收回车时，另开终端执行：

```bash
rosservice call /stack_sort_pipeline/confirm_start
```

红线不贴地图时立刻停分拣，重新启动后再点位姿。

正常状态流转应该是：

```text
LOCALIZING -> SEARCH -> PICK -> DROP -> SEARCH
```

现在主流程只用颜色视觉确认目标颜色，锁定颜色后直接进入 `PICK`，默认复用 WaterPlus 的 `grab_demo` 抓取行为，不再由主流程自己 `ALIGN/APPROACH` 靠近桌面。日志里应该看到 `[PICK-WPB] grab object=...`，然后 `/wpb_home/grab_result` 依次出现 `object x`、`hand up`、`forward`、可选的 `fine align`、`grab`、`object up`、`backward`、`done`。收到 `done` 后，主流程会根据锁定的 `target_color` 去 B 区或 C 区。

如果停在 `SEARCH` 不动，先看终端日志。真机默认不会再盲目原地转圈；日志里出现 `[SEARCH] no selectable target` 时，优先检查 A 点车头是否正对 A 桌、目标是否在相机画面中、颜色特征和 Kinect RGB/depth topic 是否正常。

视觉现在默认只接受“像正方体块”的目标：颜色检测会过滤长条/大片色块，WPB 点云检测也会按三维尺寸过滤非方块物体。启动日志里应能看到 `[VISION] square_filter ...` 和 `[objects_3d] cube_filter=...`。

现场默认不弹 OpenCV 相机调试窗口，避免图像窗口卡死拖住主流程。需要临时看 `stack_sort_debug` 窗口时，在分拣命令末尾加 `show_debug:=true`；正常测试优先看 RViz、`/stack_sort/status` 和 `/wpb_home/grab_result`。

如果日志里已经 `Target locked`，随后应该直接看到 `SEARCH -> PICK reason=wpb_direct_color_locked`。如果目标太近，程序会先打印 `[PICK-WPB] target too close before grab_demo ... backing up ...`，后退到更适合 WPB 识别桌面和物体的距离，再启动 WPB 抓取。

分拣启动时默认会同时启动 `localization_emergency_stop`。如果 `/amcl` 进程消失，会持续向 `/cmd_vel` 发零速度，并发布 `/warehouse_tuning/emergency_stop` 让主流程进入 `ERROR`。话题超时策略默认关闭，也就是 `/amcl_pose` 或 `/scan` 短暂没刷新不会直接拦停。触发急停后需要人工检查定位/雷达并重启分拣，不会自动恢复继续跑。

如果已经进入 `PICK` 但没有抓取动作，看 `[PICK-WPB]`、`/wpb_home/grab_result` 和底层终端里的 `[wpb_home_core] mani_ctrl ...`。没有 `[PICK-WPB] grab object=...` 通常是 `/kinect2/qhd/points` 没有频率，或者 `wpb_home_objects_3d` 没有从桌面点云里分割出物体；如果有 `grab object` 但动作不合适，再调 `wpb_home_bringup/config/wpb_home.yaml` 里的 `grab_y_offset`、`grab_lift_offset`、`grab_forward_offset`、`grab_close_target_x`、`grab_gripper_value`。现在闭爪前会短暂执行 `fine align`，用 `/wpb_home/objects_3d` 二次修正到夹爪中心附近；这个阶段默认留给视觉更长时间，点云短暂不新鲜时不会立刻丢掉最后一帧目标，超时后才会打印 `fine align no fresh object` 并按里程计方案闭爪。

夹得不紧时，调小 `grab_gripper_value`。现在默认是 `0.012`；数值越小夹得越紧，调太小会卡住或夹歪。

如果怀疑是夹爪本身或碰撞导致“不闭合”，可以先脱离抓取流程，只测夹爪开合。确认底层 `wpb_home_core` 已经启动时执行：

```bash
roslaunch wpb_home_bringup gripper_only_test.launch manual_step:=true cycles:=2 lift_value:=0.87
```

这条 demo 不发布 `/cmd_vel`，不会让底盘前进。为了避开底层驱动的折叠控制分支，默认会同时发布一个固定 `lift_value` 和夹爪开合值；把 `lift_value` 设成当前安全高度，日志里应看到 `[wpb_home_core] mani_ctrl lift=0.870 gripper=...`，不应再出现 `lift=-1.000`。若没有单独启动底层，可加 `start_core:=true`；但不要和已有 `wpb_home_core` 同时抢同一个串口。

每次启动分拣时，程序会先执行 `[ARM] vision stow ...`，把夹爪收窄并放到不遮挡相机的低位。主流程默认不再提前抬高夹爪或前进靠近桌面；进入 `PICK` 后，WPB 抓取行为会先移动到底盘安全距离，再 `hand up` 抬到物体高度，最后才前进夹取。WPB 的抬臂前安全距离由 `grab_target_x` 控制，当前是 `1.05m`。

进入 `DROP` 后，程序会先保持夹爪闭合并抬到安全高度，再去 B/C；到点后降到安全释放高度，先把夹爪张开到最大并保持，再开始后退离桌。回到下一轮取货前会无条件执行 `[ARM] vision stow reason=after_drop ...`，把机械臂收回低位窄爪状态。日志应出现 `[DROP] travel safe pose`、`[DROP] release`、`[DROP] open gripper to max before retreat`、`[DROP] release confirm wait`、`[DROP] retreat from tabletop`。

如果 AMCL 进程崩溃，急停节点会立刻停止底盘和抓取行为，并让主流程进入 `ERROR`。话题超时不再作为默认拦停条件；如果怀疑定位失效，先人工在 RViz 重新点 `2D Pose Estimate`，确认红线贴地图后再继续测试。

## 5. 终端 4：看状态

```bash
rostopic echo /stack_sort/status
```

重点看：

```text
state          当前阶段
target_color   当前目标颜色
detections     相机识别到的颜色、位置、深度
last_depth     目标深度，单位是米
last_align_error_px 目标中心和相机中心的像素误差
stack_count    已经放了几个
```

抓取几何参数：

```bash
rosparam get /stack_sort_pipeline/use_wpb_grab_action
rosparam get /wpb_home_grab_action/grab/grab_gripper_value
rosparam get /wpb_home_grab_action/grab/grab_target_x
rosparam get /wpb_home_grab_action/grab/grab_close_target_x
rosparam get /wpb_home_grab_action/grab/fine_align_enabled
rosparam get /wpb_home_grab_action/grab/fine_align_x_tolerance
rosparam get /wpb_home_grab_action/grab/fine_align_y_tolerance
rostopic echo /wpb_home/grab_result
rostopic hz /kinect2/qhd/points
rosparam get /stack_sort_pipeline/localization_watchdog_enabled
rosparam get /stack_sort_pipeline/scan_watchdog_timeout
rosparam get /stack_sort_pipeline/amcl_watchdog_timeout
rosparam get /stack_sort_pipeline/show_debug
rosparam get /stack_sort_pipeline/field_dimensions/table_height
rosparam get /stack_sort_pipeline/square_filter_enabled
rosparam get /wpb_home_objects_3d/cube_filter_enabled
rosparam get /wpb_home_objects_3d/cube_min_x
rosparam get /wpb_home_objects_3d/cube_max_x
rosparam get /wpb_home_objects_3d/table_height_min
rosparam get /wpb_home_objects_3d/table_height_max
rosparam get /stack_sort_pipeline/reset_arm_on_start
rosparam get /stack_sort_pipeline/startup_arm_lift_height
rosparam get /stack_sort_pipeline/vision_arm_stow_enabled
rosparam get /stack_sort_pipeline/vision_arm_lift_height
rosparam get /stack_sort_pipeline/vision_arm_gripper
rosparam get /stack_sort_pipeline/source_travel_lift_height
rosparam get /stack_sort_pipeline/approach_arm_guard_enabled
rosparam get /stack_sort_pipeline/drop_hold_gripper
rosparam get /stack_sort_pipeline/drop_open_gripper
rosparam get /stack_sort_pipeline/drop_release_clearance
rosparam get /stack_sort_pipeline/drop_safe_lift_height
rosparam get /stack_sort_pipeline/drop_release_confirm_seconds
rosparam get /stack_sort_pipeline/pick_stop_depth
rosparam get /stack_sort_pipeline/wpb_direct_pick_after_color_lock
rosparam get /stack_sort_pipeline/wpb_pregrab_distance_guard_enabled
rosparam get /stack_sort_pipeline/wpb_pregrab_min_depth
rosparam get /stack_sort_pipeline/wpb_pregrab_target_depth
rosparam get /stack_sort_pipeline/near_pick_align_deadband_px
```

## 6. 立刻停止

先在终端 3 按 `Ctrl-C` 停分拣，再发零速度：

```bash
rostopic pub /cmd_vel geometry_msgs/Twist \
  '{linear: {x: 0.0, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.0}}' -1
```

然后关闭终端 2、终端 1。

## 7. 常见现象

机器人不动：

```bash
rostopic hz /cmd_vel
rostopic hz /odom
```

如果 `/cmd_vel` 有速度但 `/odom` 不动，查底盘电源、急停、底盘使能和串口。

导航慢、嘎吱移动：

```bash
timeout 3 rostopic hz /cmd_vel
rosrun tf tf_echo map base_footprint
```

自动程序没启动前 `/cmd_vel` 不应有频率。机器人静止时 TF 明显跳，优先查地图、雷达、AMCL 初始位姿。

相机识别到了但不夹：

```bash
rostopic echo -n 1 /stack_sort/status
rostopic echo /wpb_home/grab_result
rostopic hz /kinect2/qhd/points
```

状态应从 `SEARCH` 进入 `ALIGN -> APPROACH -> PICK`。进入 `PICK` 后如果没有 `[PICK-WPB] grab object=...`，优先查 `/kinect2/qhd/points` 和 `wpb_home_objects_3d`。

Kinect 报 `kinect2_bridge` 找不到：

```bash
ls src/robot-tools-rec/iai_kinect2/CATKIN_IGNORE
```

这个文件存在时，当前工作空间会跳过 Kinect2 包。只测底盘和导航时，直接把 `start_kinect:=false`；如果要用 Kinect，就删掉或改名这个 `CATKIN_IGNORE`，然后重新 `catkin_make`，白名单里必须包含 `kinect2_registration;kinect2_bridge`。

## 8. 没有地图或点位时

重新建图、采 A/B/C、采颜色：

```bash
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

已经有地图，只重采 A/B/C 和颜色时加 `--skip-mapping`。
