# 仓储机器人视觉导航系统

这是一个基于 ROS2 Humble 的仓储机器人仿真项目，用于演示从传感器仿真、2D/3D 建图、地图保存、AMCL 定位到 Nav2 自主导航的完整流程。

项目包含 Gazebo 仓储场景、差速驱动机器人、LiDAR、RGB-D 相机、IMU、Cartographer、RTAB-Map、AMCL、Nav2 和多目标点导航脚本。

## 功能特性

- 仓储物流仿真场景，包含货架、通道、装卸区和动态叉车障碍
- 差速驱动机器人模型，搭载 2D LiDAR、RGB-D 相机、IMU 和里程计
- 基于 Cartographer 的 2D SLAM 建图
- 基于 RTAB-Map 的可选 3D 建图
- 基于 AMCL 的地图加载、定位和重定位
- 基于 Nav2 的路径规划、自主导航和动态避障
- 多目标点导航脚本
- `bringup.launch.py` 统一启动入口，支持 `sim`、`slam`、`nav` 三种模式
- 无 ROS 环境下可运行的静态回归测试

## 当前 macOS 可做的检查

当前 macOS 环境不能直接运行 ROS/Gazebo 仿真，但可以运行静态测试：

```bash
cd "/Users/xuzai/Desktop/大学/大三下/机器人传感/任务二/project"
python3 tests/static_checks.py
```

预期输出：

```text
PASS test_python_files_compile
PASS test_forklift_motion_stays_inside_bounds_and_reverses
PASS test_nav2_config_uses_humble_plugin_keys
PASS test_manifest_declares_runtime_dependencies
PASS test_spawn_launch_uses_world_launch_argument
PASS test_bringup_has_modes_and_map_validation
PASS test_nav_launch_checks_map_before_starting_nav2
PASS test_referenced_project_files_exist
PASS test_readme_documents_current_bringup_flow
```

## 实机运行环境

| 项目 | 要求 |
|---|---|
| 操作系统 | Ubuntu 22.04 |
| ROS 版本 | ROS2 Humble |
| 仿真器 | Gazebo Classic |
| Python | Python 3.10+ |
| 内存 | 2D 建图/导航建议 8 GB 以上；RTAB-Map 3D 建图建议 16 GB 以上 |

推荐工作空间结构：

```text
warehouse_ws/
└── src/
    └── project/
```

## 安装依赖

先安装 ROS2 Humble，然后安装主要运行依赖：

```bash
sudo apt update
sudo apt install -y \
  ros-humble-gazebo-ros-pkgs \
  ros-humble-xacro \
  ros-humble-robot-state-publisher \
  ros-humble-joint-state-publisher \
  ros-humble-teleop-twist-keyboard \
  ros-humble-cartographer \
  ros-humble-cartographer-ros \
  ros-humble-navigation2 \
  ros-humble-nav2-bringup \
  ros-humble-rtabmap-ros \
  ros-humble-rtabmap
```

也可以在工作空间根目录使用 `rosdep` 补齐包依赖：

```bash
cd ~/warehouse_ws
source /opt/ros/humble/setup.bash
rosdep install --from-paths src --ignore-src -r -y
```

## 编译项目

将本仓库放入 `~/warehouse_ws/src/project` 后执行：

```bash
cd ~/warehouse_ws
source /opt/ros/humble/setup.bash
colcon build --symlink-install
source install/setup.bash
```

如果使用 conda 并遇到 `GLIBCXX` 相关错误，先退出 conda：

```bash
conda deactivate
source /opt/ros/humble/setup.bash
source ~/warehouse_ws/install/setup.bash
```

## 推荐运行流程

推荐优先使用统一入口 `bringup.launch.py`。

### 1. 启动仿真模式

```bash
cd ~/warehouse_ws
source /opt/ros/humble/setup.bash
source install/setup.bash
ros2 launch project bringup.launch.py mode:=sim
```

`mode:=sim` 会启动 Gazebo、机器人、动态叉车和导航 RViz。

### 2. 启动键盘遥控

新终端执行：

```bash
cd ~/warehouse_ws
source /opt/ros/humble/setup.bash
source install/setup.bash
ros2 run teleop_twist_keyboard teleop_twist_keyboard
```

主要传感器和状态话题：

| 传感器 / 状态 | 话题 | 坐标系 |
|---|---|---|
| 2D LiDAR | `/scan` | `laser_link` |
| RGB 相机 | `/camera/rgb/image_raw` | `camera_rgbd_link` |
| 深度相机 | `/camera/depth/image_raw` | `camera_rgbd_link` |
| IMU | `/imu` | `imu_link` |
| 轮式里程计 | `/odom` | `odom -> base_footprint` |

### 3. 启动 2D SLAM 建图

```bash
cd ~/warehouse_ws
source /opt/ros/humble/setup.bash
source install/setup.bash
ros2 launch project bringup.launch.py mode:=slam
```

`mode:=slam` 会启动 Gazebo、机器人、Cartographer 和 SLAM RViz。使用键盘遥控机器人遍历仓储环境。

可观察地图输出：

```bash
ros2 topic echo /map --field header
ros2 topic echo /submap_list
```

### 4. 保存地图

建图完成后执行：

```bash
mkdir -p ~/warehouse_ws/src/project/maps
ros2 run nav2_map_server map_saver_cli -f ~/warehouse_ws/src/project/maps/warehouse_map
```

保存成功后应生成：

```text
maps/warehouse_map.yaml
maps/warehouse_map.pgm
```

### 5. 启动定位与导航

必须先完成地图保存，再启动导航：

```bash
cd ~/warehouse_ws
source /opt/ros/humble/setup.bash
source install/setup.bash
ros2 launch project bringup.launch.py mode:=nav map:=~/warehouse_ws/src/project/maps/warehouse_map.yaml
```

如果地图不存在，launch 会提示先建图并保存地图，然后退出。

也可以单独启动导航 launch：

```bash
ros2 launch project nav.launch.py map:=~/warehouse_ws/src/project/maps/warehouse_map.yaml
```

### 6. 运行多目标点导航

Nav2 启动并完成初始定位后，新终端执行：

```bash
cd ~/warehouse_ws
source /opt/ros/humble/setup.bash
source install/setup.bash
ros2 run project waypoint_nav.py
```

### 7. 可选 3D 建图

RTAB-Map 对内存要求较高，建议 16 GB 以上：

```bash
cd ~/warehouse_ws
source /opt/ros/humble/setup.bash
source install/setup.bash
ros2 launch project rtabmap.launch.py
```

## 一句话流程

```text
bringup.launch.py mode:=sim 启动仿真
-> teleop_twist_keyboard 遥控机器人
-> bringup.launch.py mode:=slam 建图
-> map_saver_cli 保存地图
-> bringup.launch.py mode:=nav 加载地图并导航
-> waypoint_nav.py 执行多目标点导航
```

## 项目结构

```text
project/
├── bringup.launch.py
├── CMakeLists.txt
├── package.xml
├── README.md
├── 运行要求.md
├── config/
│   ├── amcl.yaml
│   ├── cartographer.lua
│   ├── nav2.yaml
│   ├── nav.rviz
│   └── slam.rviz
├── description/
│   └── robot.xacro
├── launch/
│   ├── nav.launch.py
│   ├── rtabmap.launch.py
│   ├── slam.launch.py
│   └── spawn.launch.py
├── scripts/
│   ├── apriltag_detect.py
│   ├── forklift_controller.py
│   ├── setup_autodl.sh
│   └── waypoint_nav.py
├── tests/
│   └── static_checks.py
└── worlds/
    └── warehouse.world
```

## TF 关系

```text
map -> odom -> base_footprint -> base_link
                                ├── laser_link
                                ├── camera_rgbd_link
                                ├── camera_mono_link
                                └── imu_link
```

其中 `odom -> base_footprint` 由差速驱动仿真插件发布，`map -> odom` 由 Cartographer 或 AMCL 发布。

## 常见问题

| 问题 | 可能原因 | 解决方法 |
|---|---|---|
| `package 'project' not found` | 未加载工作空间环境 | 执行 `source ~/warehouse_ws/install/setup.bash` |
| `GLIBCXX_3.4.30 not found` | conda 的库路径与 ROS 冲突 | 执行 `conda deactivate` 后重新加载 ROS 和工作空间 |
| `mode:=nav` 直接退出 | 地图文件不存在 | 先运行 `mode:=slam` 建图，再用 `map_saver_cli` 保存地图 |
| Cartographer 启动后没有地图 | Gazebo 或 `/clock` 未就绪 | 等待 Gazebo 和机器人加载完成，再检查 `/scan`、`/odom` |
| 键盘遥控无反应 | 终端未获得焦点或 `/cmd_vel` 未发布 | 点击终端窗口，并用 `ros2 topic echo /cmd_vel` 检查 |
| RViz 或 Gazebo 在服务器上启动失败 | 缺少图形显示环境 | 使用 Xvfb、VNC、noVNC 或远程桌面 |

## 注意事项

- 各 launch 文件默认使用仿真时间。
- 不要一开始直接运行导航，必须先生成 `maps/warehouse_map.yaml`。
- `bringup.launch.py mode:=sim` 用于仿真检查。
- `bringup.launch.py mode:=slam` 用于 2D 建图。
- `bringup.launch.py mode:=nav` 用于地图加载、AMCL 定位和 Nav2 导航。
- `waypoint_nav.py` 需要在 Nav2 已经启动并可用后运行。
- 当前 macOS 只能用于静态测试，不能替代 Ubuntu ROS2 实机验证。

## 许可证

本项目采用 MIT License。
