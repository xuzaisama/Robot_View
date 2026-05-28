# Codex 接手说明

你正在接手一个 ROS2 Humble 仓储机器人视觉导航项目。目标环境是：

```text
Windows 设备
VirtualBox
Ubuntu 22.04 amd64
Gazebo Classic
ROS2 Humble
用户需要 GUI 和 RViz
默认优先稳定运行
```

## 1. 先读这些文件

按顺序阅读：

```text
README.md
运行要求.md
docs/验收清单.md
config/auto_mapping_route.yaml
```

不要先改代码。先确认环境、路径和用户当前卡在哪一步。

## 2. 当前项目设计决策

已根据用户选择完成以下设计：

```text
主目标：VirtualBox Ubuntu amd64 稳定运行
同步目标：自动建图
GUI：必须支持，默认 gui:=true
RViz：默认 rviz:=true
性能：默认 performance:=low
3D RTAB-Map：仅作为扩展，不进主演示路径
自动建图：mode:=slam_auto
自动巡航路线：safe / coverage / extended
默认路线：route:=safe
正式建图：route:=coverage
更精细地图：route:=extended
建图阶段动态叉车：dynamic_obstacles:=false
导航演示动态叉车：dynamic_obstacles:=true
地图保存：不自动保存，巡航结束后检查 RViz，再手动 map_saver_cli
```

## 3. 第一件事：确认项目静态自洽

在项目根目录运行：

```bash
cd ~/warehouse_ws/src/project
python3 tests/static_checks.py
```

预期 12 项全部 `PASS`：

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
PASS test_acceptance_checklist_documents_runtime_validation
PASS test_auto_mapping_configuration_is_installed_and_documented
PASS test_codex_handoff_documents_next_steps
```

如果失败，先修静态测试，不要急着跑 Gazebo。

## 4. 第二件事：安装依赖和编译

```bash
cd ~/warehouse_ws
source /opt/ros/humble/setup.bash
rosdep install --from-paths src --ignore-src -r -y
colcon build --symlink-install
source install/setup.bash
```

成功后检查：

```bash
ros2 pkg prefix project
```

如果 `package 'project' not found`，通常是没 `source install/setup.bash` 或项目没放在 `~/warehouse_ws/src/project`。

## 5. 第三件事：检查 GUI 和仿真

先运行：

```bash
ros2 launch project bringup.launch.py mode:=sim gui:=true rviz:=true performance:=low dynamic_obstacles:=true
```

预期：

```text
Gazebo GUI 正常显示
RViz 正常显示
机器人生成在 x=-8.0, y=0.0 附近
动态叉车移动
/scan /odom /imu /clock 有数据
```

检查命令：

```bash
ros2 topic echo /clock --once
ros2 topic echo /scan --once
ros2 topic echo /odom --once
ros2 topic echo /imu --once
```

如果 GUI 卡顿：

```bash
ros2 launch project bringup.launch.py mode:=sim gui:=false rviz:=true performance:=low dynamic_obstacles:=true
```

用户此前明确需要 GUI，所以 `gui:=false` 只作为排错兜底，不作为主演示方案。

## 6. 第四件事：自动建图

先跑短路线，只验证自动巡航能工作：

```bash
ros2 launch project bringup.launch.py mode:=slam_auto route:=safe gui:=true rviz:=true performance:=low dynamic_obstacles:=false
```

如果 `safe` 成功，再跑正式建图路线：

```bash
ros2 launch project bringup.launch.py mode:=slam_auto route:=coverage gui:=true rviz:=true performance:=low dynamic_obstacles:=false
```

如果地图不够完整，再跑：

```bash
ros2 launch project bringup.launch.py mode:=slam_auto route:=extended gui:=true rviz:=true performance:=low dynamic_obstacles:=false
```

自动巡航结束后：

```text
不要立刻保存
等待 10-30 秒
观察 RViz 地图是否闭环、是否有明显重影
再保存地图
```

保存地图：

```bash
mkdir -p ~/warehouse_ws/src/project/maps
ros2 run nav2_map_server map_saver_cli -f ~/warehouse_ws/src/project/maps/warehouse_map
```

## 7. 第五件事：导航和动态避障

地图保存成功后运行：

```bash
ros2 launch project bringup.launch.py mode:=nav gui:=true rviz:=true performance:=low dynamic_obstacles:=true map:=~/warehouse_ws/src/project/maps/warehouse_map.yaml
```

然后运行多目标点导航：

```bash
ros2 run project waypoint_nav.py
```

预期目标点：

```text
LoadingZone_A
ShelfCorridor_B
LoadingZone_C
Home
```

## 8. 如果用户问“现在我该干嘛”

按这个顺序指导：

```text
1. python3 tests/static_checks.py
2. colcon build --symlink-install
3. mode:=sim 检查 GUI/RViz/传感器
4. mode:=slam_auto route:=safe 验证自动建图
5. mode:=slam_auto route:=coverage 正式建图
6. map_saver_cli 保存地图
7. mode:=nav dynamic_obstacles:=true 启动导航
8. waypoint_nav.py 跑多目标点
```

## 9. 不要做的事

```text
不要一上来跑 route:=extended
不要建图阶段启动 dynamic_obstacles:=true
不要在地图刚结束巡航后立刻保存
不要把 RTAB-Map 作为主演示路径
不要在 GUI 卡顿时先改算法，先用 performance:=low / rviz / gui 参数定位瓶颈
```

## 10. 常见修复方向

如果 Gazebo 很卡：

```text
确认 performance:=low
确认 VirtualBox 3D Acceleration 已开启
确认显存 128 MB 以上
先保留 RViz，必要时临时 gui:=false 排错
```

如果自动巡航撞/卡：

```text
先 route:=safe
检查 /scan
降低 config/auto_mapping_route.yaml 中 linear_x
缩短贴近货架的动作 duration
```

如果地图质量不好：

```text
先 coverage
不够再 extended
巡航结束后等待 10-30 秒再保存
检查是否有回环重影
必要时降低速度并重新建图
```

如果导航失败：

```text
确认 maps/warehouse_map.yaml 存在
确认 /map /amcl_pose 有数据
确认 Nav2 lifecycle nodes active
确认 RViz 初始位姿已设置或 AMCL 已收敛
```
