# Web 仪表盘 — 仓库分拣机器人实时监控

演示/展示用的实时仪表盘。浏览器打开即可看到相机画面（含颜色检测叠加）、地图+机器人位置+激光扫描、分拣进度和实时状态。

## 功能一览

- **相机画面**：Kinect 彩色图 MJPEG 实时流 + 检测框叠加（bounding box、颜色标签、深度）
- **地图视图**：OccupancyGrid 地图 + 机器人位姿箭头 + 激光扫描点 + A/B/C 区域标记 + 规划/实际路径
- **状态面板**：分拣进度条、各颜色堆叠数量、当前耗时时、深度、成功率、重试次数
- **地图交互**：鼠标滚轮缩放、拖拽平移
- **远程访问**：工控机以外的电脑/平板，同一局域网内浏览器即可查看

## 无 ROS 预览模式

需要展示界面但没有启动 ROS 时，可在仓库根目录运行：

```bash
python3 -m http.server 8000 -d src/arm_grab_task/web
```

然后访问 `http://localhost:8000/dashboard.html?demo=1`。该模式使用内置演示数据，只用于界面预览，不会连接或控制机器人。

## 依赖安装（一次性）

```bash
sudo apt-get install ros-noetic-rosbridge-suite ros-noetic-web-video-server
```

验证：

```bash
dpkg -l | grep ros-noetic-rosbridge-server
dpkg -l | grep ros-noetic-web-video-server
```

## 快速启动

### 方式 1：一条命令（推荐）

自动启动硬件底层 + 分拣 pipeline + web 桥接服务，并打开本地浏览器：

```bash
cd ~/catkin_ws
source devel/setup.bash

roslaunch arm_grab_task web_dashboard.launch \
  map_file:=$HOME/maps/lab.yaml \
  use_field_override:=true \
  field_override:=$HOME/maps/abc_zones.yaml \
  use_feature_override:=true \
  feature_override:=$HOME/maps/cargo_features.yaml
```

这一条命令等价于原来的 **终端 1（硬件）+ 终端 3（分拣）+ web 服务 + 浏览器**。

常用覆盖参数：

| 参数 | 默认 | 说明 |
|---|---|---|
| `start_robot_base` | `true` | 是否启动硬件底层（仿真时改 `false`） |
| `start_joy` | `true` | 是否启用手柄。默认和原流程一致，保留手柄移动能力 |
| `confirm_before_start` | `true` | 定位后等待网页按钮确认才开跑 |
| `open_browser` | `true` | 是否自动打开本地浏览器 |

示例——开 RViz 手动指定初始位姿，随后在网页点“确认位姿并开始”：

```bash
./start_dashboard.sh --rviz
```

示例——机器人确认在 A 区标定原点，自动设置初始位姿并跳过确认：

```bash
./start_dashboard.sh --auto
```

底层 roslaunch 方式：

```bash
roslaunch arm_grab_task web_dashboard.launch \
  map_file:=$HOME/maps/lab.yaml \
  use_field_override:=true field_override:=$HOME/maps/abc_zones.yaml \
  use_feature_override:=true feature_override:=$HOME/maps/cargo_features.yaml \
  rviz:=true
```

仿真示例（跳过硬件层）：

```bash
roslaunch arm_grab_task web_dashboard.launch start_robot_base:=false
```

### 方式 2：只启动桥接层

如果 pipeline 已通过其他方式运行，只启动 web 服务：

```bash
roslaunch arm_grab_task web_bridges_only.launch
```

### 方式 3：手动逐个启动

```bash
# 终端 1
roslaunch rosbridge_server rosbridge_websocket.launch

# 终端 2
rosrun web_video_server web_video_server

# 终端 3
python3 -m http.server 8000 -d $(rospack find arm_grab_task)/web
```

## 远程访问

工控机启动后，终端会打印远程地址：

```
━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━
  🌐 仪表盘已启动

  工控机本地:    file:///.../dashboard.html
  远程电脑访问:  http://ROBOT_IP:8000/dashboard.html

  端口占用:
    :9090  rosbridge (WebSocket)
    :8080  web_video_server (MJPEG)
    :8000  static file server (仪表盘 HTML)
━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━
```

**远程电脑**（笔记本/平板）只要和工控机在同一个局域网（WiFi），浏览器打开 `http://<工控机IP>:8000/dashboard.html` 即可。`dashboard.html` 会自动检测页面来源，将 WebSocket 和相机流指向工控机地址。

如果不知道工控机 IP：

```bash
# 在工控机上执行
hostname -I
```

## 各层启动内容

`web_dashboard.launch` 按顺序启动以下 6 个部分：

```
1. field_robot_base.launch    ← 底盘/雷达/Kinect/手柄 (start_robot_base:=false 跳过)
2. stack_sort_field.launch    ← 定位 + 分拣 pipeline
3. rosbridge_websocket        ← :9090 ROS 数据 → 浏览器 WebSocket
4. web_video_server           ← :8080 Kinect 图像 → MJPEG 流
5. serve_web.sh               ← :8000 静态文件服务器
6. open_dashboard.sh          ← 自动打开浏览器
```

## 数据流

| ROS Topic | 传输方式 | 仪表盘用途 |
|---|---|---|
| `/stack_sort/status` | WebSocket | 状态面板 + 检测框叠加 |
| `/kinect2/sd/image_color_rect` | HTTP MJPEG | 左侧实时画面 |
| `/amcl_pose` | WebSocket | 机器人三角箭头 |
| `/map` | WebSocket | 背景 OccupancyGrid |
| `/scan` | WebSocket | 激光扫描红点 |
| `/stack_sort/markers` | WebSocket | 叠放锚点/投放基线 |
| `/field_nav_smoke/markers` | WebSocket | A/B/C 区域标记 |
| `/field_nav_smoke/planned_path` | WebSocket | 规划路径线 |
| `/field_nav_smoke/actual_path` | WebSocket | 实际轨迹线 |

## 端口

| 服务 | 端口 | 说明 |
|---|---|---|
| rosbridge_websocket | 9090 | WebSocket JSON 协议 |
| web_video_server | 8080 | HTTP MJPEG 相机流 |
| serve_web.sh | 8000 | 静态文件服务器 |

端口冲突时通过命令行覆盖：

```bash
roslaunch arm_grab_task web_dashboard.launch \
  rosbridge_port:=9091 video_server_port:=8081 web_port:=8001 ...
```

## 浏览器兼容

- Chromium / Chrome：完全支持
- Firefox：完全支持
- 推荐分辨率：1440×900 以上
- 远程访问不要求浏览器安装任何插件

## 故障排查

**相机不显示**
```bash
rostopic hz /kinect2/sd/image_color_rect   # 确认 Kinect 有数据
rosnode list | grep web_video_server        # 确认视频服务器在运行
```

**检测框不出现**
- 确认 pipeline 状态不是 `LOCALIZING` 或 `FINISH`
- 检查 `/stack_sort/status` 中 `detections` 字段是否有数据

**地图空白**
```bash
rostopic echo /map/info | head              # 确认地图已加载
rosnode list | grep map_server              # 确认 map_server 在运行
```

**rosbridge 连不上**
```bash
rosnode list | grep rosbridge               # 确认节点在运行
curl -s http://localhost:9090               # 确认端口可达
```

**需要急停**

网页右侧“急停停止”按钮按下即触发，不做二次确认；它会连续发布 `/cmd_vel=0`，发布 `grab stop`、`place stop`、`object_detect stop`，并向 `/warehouse_tuning/emergency_stop` 写入停止原因。主流程收到后会进入 `ERROR`，需要人工检查并重启流程。

**流程异常**

主流程进入 `ERROR` 或状态消息带 `last_error` 时，网页会弹出红色异常框，同时补发一次停止命令。页面不会自动恢复继续跑。

**远程电脑能打开页面但无数据**
- 确认两台电脑在同一网络（能互相 ping 通）
- 检查工控机防火墙是否放开 9090、8080、8000 端口
- 浏览器 F12 → Console，看 WebSocket 连接是否成功
