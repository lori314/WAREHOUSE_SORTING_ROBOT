# 启智机器人桌面分拣堆叠

当前场景用于实机部署前的业务逻辑验收：A 桌取物，绿色放到 B 桌，蓝色放到 C 桌，每类 3 个并形成 3 层堆叠。

## 运行

```bash
cd ~/catkin_ws
catkin_make
source ~/catkin_ws/devel/setup.bash
roslaunch arm_grab_task stack_sort_abc_tabletop_demo.launch auto_start_pipeline:=true
```

带验收脚本运行：

```bash
source ~/catkin_ws/devel/setup.bash
rosrun arm_grab_task run_stack_sort_acceptance.py --timeout 900 --settle-seconds 1.5
```

最近一次完整验收已通过：

- 6 次取放成功，0 次失败，0 次重试
- 绿色 3 个、蓝色 3 个均完成真实夹取抬升验证：`[PHYS-GRASP] lifted`
- 未使用抓取吸附：`gazebo_attach_on_pick=false`
- 未使用放置瞬移稳定：`gazebo_stabilize_stack_on_release=false`
- 最终堆叠高度约为 `0.78 / 0.88 / 0.98`
- 报告输出：`/tmp/arm_grab_task_reports/stack_sort_abc_tabletop_20260601_235621.*`

## 仿真边界

当前仿真用于验证上层业务逻辑和验收效果，抓取必须靠夹爪物理接触完成。仿真默认不启用吸附或放置瞬移：

- `gazebo_attach_on_pick=false`：闭爪后不把方块绑定到夹爪。
- `gazebo_verify_physical_pick=true`：抬升后检查方块 z 轴增量，未真实抬起则判定抓取失败。
- `gazebo_stabilize_stack_on_release=false`：释放后不把方块移动到堆叠中心。
- `gazebo_initial_model_poses`：只在启动阶段把 6 个方块放回 A 桌，避免 Gazebo 并发生成时方块掉到地面；该步骤标记为 `[SIM-INIT]`，不参与抓取/堆叠验收。

仿真通过后再部署实机；实机部署保留同一套状态机和调参接口，Gazebo 专用参数不作为实机抓取依据。

## 现场调参入口

主配置文件：

```text
config/stack_sort_abc_tabletop_params.yaml
```

常用参数：

- 视觉启停：代码只在 `SEARCH/ALIGN/APPROACH/PICK/RECOVER_RETRY` 使用相机，`DROP/FINISH` 会关闭视觉处理。
- 方块颜色/面积：`min_box_area`、`max_box_area`，HSV 阈值在 `scripts/stack_sort_pipeline.py` 的 `ColorBoxPerception.color_ranges`。
- 取物停车/插入：`gazebo_source_pick_forward_offset`、`pose_drive_pick_dist_tolerance`、`pick_insert_distance`、`pick_insert_tolerance`。
- 机械臂速度/开合：`arm_open_seconds`、`arm_close_seconds`、`arm_lift_seconds`、`open_gripper`、`closed_gripper`。
- 目标桌位置：`tabletop_stack_anchors`、`tabletop_drop_base_targets`、`tabletop_return_base_target`。
- 仿真物理验证：`gazebo_grasp_max_xy_error`、`gazebo_grasp_max_z_error`、`gazebo_physical_pick_min_lift`。
- 初始桌面摆放：`gazebo_initial_reset_delay`、`gazebo_initial_model_poses`。

## 建图和实机前检查

建图、导航和现场参数建议单独放到上层的 `warehouse_tuning` 包维护。现场流程保持精简：

1. 建图并确认 A/B/C 三张桌子的地图位置。
2. 现场采集两类方块的颜色阈值、面积范围、桌面高度和夹爪开合参数。
3. 先跑 `run_stack_sort_acceptance.py` 做仿真逻辑验收。
4. 实机上关闭 Gazebo 专用辅助参数，只保留状态机、视觉检测、底盘目标点和机械臂动作参数。

## 机械臂夹取单元测试

只测试“识别桌面物体 -> 发送 WPB 抓取动作 -> 等待抓取结果”，不跑完整分拣：

```bash
roslaunch arm_grab_task wpb_pick_unit_test.launch dry_run:=true
roslaunch arm_grab_task wpb_pick_unit_test.launch
```

默认会启动 `/kinect2/qhd/points`、`wpb_home_objects_3d` 和 `wpb_home_grab_action`，但不启动底盘串口和 Kinect bridge；测试前先保持 `warehouse_tuning field_robot_base.launch` 的真机底层运行。第一次先用 `dry_run:=true` 确认 `/wpb_home/objects_3d` 能稳定识别到桌面方块，再去掉 dry run 做真实夹取。

常调参数可以直接写在 launch 命令后面：

```bash
roslaunch arm_grab_task wpb_pick_unit_test.launch \
  table_height_min:=0.70 \
  table_height_max:=0.80 \
  action_z_override:=0.75 \
  grab_y_offset:=-0.03 \
  grab_lift_offset:=0.05 \
  grab_forward_offset:=0.00 \
  grab_gripper_value:=0.012 \
  grab_target_x:=1.05 \
  grab_close_target_x:=0.65 \
  fine_align_timeout:=6.0 \
  wait_for_lift_before_forward:=true \
  lift_ready_tolerance:=0.02 \
  lift_wait_timeout:=35.0 \
  lift_command_to_joint_offset:=0.35
```

每次测试会输出 JSON 报告到 `/tmp/arm_grab_task_reports/wpb_pick_unit_*.json`，里面记录候选物体坐标、发送的抓取坐标和 `/wpb_home/grab_result` 阶段序列。当前默认按 75cm 桌面、10cm 方块设置：`action_z_override=0.75`，`grab_lift_offset=0.05`，闭爪高度约 0.80m；也已按“夹爪偏在物体左边约 3cm”设置 `grab_y_offset=-0.03` 做右向补偿。进入 `forward` 前会检查 `/joint_states` 里的 `mani_base`，并用 `lift_command_to_joint_offset=0.35` 换算升降命令和关节反馈的零点差，防止升降未到位时底盘前进。夹取不稳时优先按顺序调 `grab_y_offset`、`grab_lift_offset`、`grab_forward_offset`、`grab_gripper_value`；识别不到物体时先调 `cube_min_*`、`cube_max_*`、`table_height_min/max` 或检查 `/kinect2/qhd/points`。
