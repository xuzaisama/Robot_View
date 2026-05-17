# 仓储机器人视觉导航系统

这是一个基于 ROS 2 的仓储机器人仿真项目，用于完成建图、定位、自主导航和动态避障等流程。项目集成 Gazebo、Cartographer、RTAB-Map、AMCL 和 Nav2，覆盖从传感器数据发布到多目标点导航的完整仿真闭环。

## 功能特性

- 仓储物流仿真场景，包含货架、通道和动态叉车障碍
- 差速驱动机器人模型，搭载 LiDAR、RGB-D 相机、IMU 和里程计
- 基于 Cartographer 的 2D SLAM 建图
- 基于 RTAB-Map 的可选 3D 建图
- 基于 AMCL 的地图加载与定位
- 基于 Nav2 的路径规划和自主导航
- 多目标点导航示例
- SLAM 和导航专用 RViz 配置

## 运行环境

- Ubuntu 22.04
- ROS 2 Humble
- Gazebo Classic
- Python 3.10+
- 推荐内存：2D 建图和导航建议 8 GB 以上，RTAB-Map 建议 16 GB 以上

建议将项目放在 ROS 2 工作空间中，例如：

```bash
warehouse_ws/
└── src/
    └── project/
```

## 安装依赖

请先安装 ROS 2 Humble，然后安装主要运行依赖：

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
  ros-humble-rtabmap-ros
```

也可以在工作空间根目录使用 `rosdep` 安装包依赖：

```bash
cd ~/warehouse_ws
rosdep install --from-paths src --ignore-src -r -y
```

## 编译项目

将本仓库克隆或复制到 ROS 2 工作空间的 `src` 目录：

```bash
mkdir -p ~/warehouse_ws/src
cd ~/warehouse_ws/src
git clone <repository-url> project

cd ~/warehouse_ws
colcon build --symlink-install
source install/setup.bash
```

如果使用 conda 并遇到 `GLIBCXX` 相关错误，请先退出 conda 环境：

```bash
conda deactivate
source /opt/ros/humble/setup.bash
source ~/warehouse_ws/install/setup.bash
```

## 快速开始

启动 Gazebo 仓储场景和机器人：

```bash
source ~/warehouse_ws/install/setup.bash
ros2 launch project spawn.launch.py
```

等待机器人成功加载后，打开新终端进行键盘遥控：

```bash
source ~/warehouse_ws/install/setup.bash
ros2 run teleop_twist_keyboard teleop_twist_keyboard
```

主要传感器和状态话题如下：

| 传感器 / 状态 | 话题 | 坐标系 |
| --- | --- | --- |
| 2D LiDAR | `/scan` | `laser_link` |
| RGB 相机 | `/camera/rgb/image_raw` | `camera_rgbd_link` |
| 深度相机 | `/camera/depth/image_raw` | `camera_rgbd_link` |
| IMU | `/imu` | `imu_link` |
| 轮式里程计 | `/odom` | `odom -> base_footprint` |

## 2D SLAM 建图

先启动仿真环境：

```bash
ros2 launch project spawn.launch.py
```

在第二个终端启动 Cartographer：

```bash
source ~/warehouse_ws/install/setup.bash
ros2 launch project slam.launch.py
```

在第三个终端遥控机器人运动并完成建图：

```bash
source ~/warehouse_ws/install/setup.bash
ros2 run teleop_twist_keyboard teleop_twist_keyboard
```

可通过以下命令观察地图输出：

```bash
ros2 topic echo /map --field header
ros2 topic echo /submap_list
```

建图完成后保存地图：

```bash
mkdir -p ~/warehouse_ws/src/project/maps
ros2 run nav2_map_server map_saver_cli -f ~/warehouse_ws/src/project/maps/warehouse_map
```

保存后会生成 `warehouse_map.yaml` 和 `warehouse_map.pgm`。

## 3D 建图

项目支持使用 RGB-D 相机运行 RTAB-Map：

```bash
source ~/warehouse_ws/install/setup.bash
ros2 launch project rtabmap.launch.py
```

RTAB-Map 对内存要求更高，建议在 16 GB 以上内存的环境中运行。

## 定位与自主导航

保存地图后，先启动仿真环境：

```bash
source ~/warehouse_ws/install/setup.bash
ros2 launch project spawn.launch.py
```

在另一个终端启动 AMCL 和 Nav2：

```bash
source ~/warehouse_ws/install/setup.bash
ros2 launch project nav.launch.py map:=~/warehouse_ws/src/project/maps/warehouse_map.yaml
```

如需触发全局重定位，可执行：

```bash
ros2 service call /reinitialize_global_localization std_srvs/srv/Empty
```

运行多目标点导航示例：

```bash
source ~/warehouse_ws/install/setup.bash
ros2 run project waypoint_nav.py
```

启动动态叉车障碍示例：

```bash
source ~/warehouse_ws/install/setup.bash
ros2 run project forklift_controller.py
```

## RViz 可视化

SLAM 可视化：

```bash
rviz2 -d ~/warehouse_ws/src/project/config/slam.rviz
```

导航可视化：

```bash
rviz2 -d ~/warehouse_ws/src/project/config/nav.rviz
```

如果在无显示器服务器上运行 Gazebo 或 RViz，可以使用 Xvfb、VNC、noVNC 或远程桌面环境提供图形显示。

## 项目结构

```text
project/
├── bringup.launch.py
├── CMakeLists.txt
├── package.xml
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
| --- | --- | --- |
| `package 'project' not found` | 未加载工作空间环境 | 执行 `source ~/warehouse_ws/install/setup.bash` |
| `GLIBCXX_3.4.30 not found` | conda 的库路径与 ROS 冲突 | 执行 `conda deactivate` 后重新加载 ROS 和工作空间 |
| Cartographer 启动后没有地图 | Gazebo 或 `/clock` 未就绪 | 先启动 Gazebo，并等待机器人加载完成 |
| 键盘遥控无反应 | 终端未获得焦点或 `/cmd_vel` 未发布 | 点击终端窗口，并用 `ros2 topic echo /cmd_vel` 检查 |
| RViz 或 Gazebo 在服务器上启动失败 | 缺少图形显示环境 | 使用 Xvfb、VNC、noVNC 或远程桌面 |

## 注意事项

- 各 launch 文件默认使用仿真时间。
- 启动 SLAM 或导航前，应先启动 Gazebo。
- 保存地图路径可通过 `map:=...` 参数传入。
- 当前 ROS 包名为 `project`；如需重命名，请同步修改 `package.xml`、`CMakeLists.txt` 和相关启动命令。

## 许可证

本项目采用 MIT License。
