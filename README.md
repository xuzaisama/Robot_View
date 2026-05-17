# 仓储物流机器人视觉导航系统

> ROS 2 Humble + Gazebo + Cartographer + RTAB-Map + Nav2

在 Gazebo 中构建仓储场景，机器人完成 2D/3D 建图、AMCL 定位、Nav2 自主导航与动态避障的完整闭环。

```
环境感知 → 2D/3D 建图 → 地图保存 → AMCL 定位 → 路径规划 → 自主导航 + 动态避障
```

## 环境搭建（一次性）

### 1. 本地 Mac 准备

```bash
# 生成 SSH 密钥（如已有可跳过）
ssh-keygen -t ed25519 -N "" -f ~/.ssh/id_ed25519

# VS Code SSH Config（~/.ssh/config）
Host autodl
    HostName connect.westb.seetacloud.com
    Port <你的端口>
    User root
    IdentityFile ~/.ssh/id_ed25519
    StrictHostKeyChecking no
```

### 2. 上传项目到 AutoDL

```bash
ssh-copy-id -p <端口> root@<实例IP>          # 或手动复制公钥到 ~/.ssh/authorized_keys
scp -r project/ autodl:~/warehouse_ws/src/   # 上传项目文件
```

### 3. 在 AutoDL 上一键安装（约 30-60 分钟）

```bash
ssh autodl
cd ~/warehouse_ws/src/project
bash scripts/setup_autodl.sh
```

> 脚本会自动：换清华镜像源 → 安装 ROS 2 Humble + Gazebo + Cartographer + RTAB-Map + Nav2 → 编译工作空间 → 配置 `ros2env` 环境函数

## 每次使用前

```bash
ros2env      # 每个新终端都要先执行（自动退出 conda + 加载 ROS + 加载工作空间）
```

> 忘记执行 `ros2env` 会报 `GLIBCXX_3.4.30 not found`，是 conda 的 libstdc++ 太旧导致的。

## 启动仿真环境

```bash
ros2 launch project spawn.launch.py
# 等待看到 "spawn_entity.py: process has finished cleanly" 表示 Gazebo + 机器人已就绪
```

| 传感器 | 话题 | 帧 |
|--------|------|-----|
| 2D LiDAR | `/scan` | `laser_link` |
| RGB 相机 | `/camera/rgb/image_raw` | `camera_rgbd_link` |
| 深度相机 | `/camera/depth/image_raw` | `camera_rgbd_link` |
| IMU | `/imu` | `imu_link` |
| 轮式里程计 | `/odom` | `odom` → `base_footprint` |

## 遥控机器人

```bash
ros2 run teleop_twist_keyboard teleop_twist_keyboard
```

```
u    i    o      i = 前进      , = 后退
j    k    l      j = 左转      l = 右转
m    ,    .      u = 左前      o = 右前
                 m = 左后      . = 右后
                 k = 停止

q/z : 增减最大速度 10%
w/x : 增减线速度 10%
e/c : 增减角速度 10%
```

**按键没反应？**

1. 点击终端窗口使其获得焦点（VS Code 中要点对应终端 Tab）
2. 另开终端执行 `ros2 topic echo /cmd_vel`，按 `i` 看是否有 `linear: x: 0.5` — 有数据 = 键盘正常
3. 确认 Gazebo 终端已看到 `spawn_entity.py: process has finished cleanly`
4. 直接用命令测试：`ros2 topic pub /cmd_vel geometry_msgs/msg/Twist "{linear: {x: 0.3}, angular: {z: 0.0}}" -r 10`

## SLAM 建图

**启动顺序**：必须先等 Gazebo 完全启动（看到 `spawn_entity finished cleanly`），再启动 SLAM。

```bash
# 终端1：仿真环境
ros2 launch project spawn.launch.py
# 等待 "spawn_entity.py: process has finished cleanly"

# 终端2：Cartographer 2D SLAM
ros2 launch project slam.launch.py
# 看到 "Inserted submap" 说明建图开始

# 终端3：遥控机器人边走边建图
ros2 run teleop_twist_keyboard teleop_twist_keyboard
```

**命令行监控建图进程：**

```bash
ros2 topic echo /map --field header 2>/dev/null | head -5    # 地图是否有更新
ros2 topic echo /submap_list                                  # 子图数量
```

## 保存地图

建图完成后，在 slam 终端 Ctrl+C 停止 Cartographer，然后：

```bash
ros2 run nav2_map_server map_saver_cli -f ~/warehouse_ws/src/project/maps/warehouse_map
```

会生成 `warehouse_map.yaml` + `warehouse_map.pgm`。

## 3D SLAM（RTAB-Map，可选）

需要 RGB-D 相机数据，内存峰值约 8-10GB：

```bash
ros2 launch project rtabmap.launch.py
```

建图完成后用 RTAB-Map 自带的数据库导出地图，或直接用 `map_saver_cli` 保存 `/rtabmap/grid_map`。

## 定位

需要有已保存的地图文件 `maps/warehouse_map.yaml`：

```bash
# 终端1：仿真环境
ros2 launch project spawn.launch.py

# 终端2：AMCL 定位 + 地图加载
ros2 launch project nav.launch.py
```

AMCL 使用 5000 个粒子，支持全局定位。可通过以下方式触发重定位：

```bash
ros2 service call /reinitialize_global_localization std_srvs/srv/Empty
```

## 自主导航 + 动态避障

```bash
# 终端1：仿真环境
ros2 launch project spawn.launch.py

# 终端2：导航栈（AMCL + Nav2）
ros2 launch project nav.launch.py

# 终端3：启动动态障碍（叉车在主通道来回移动）
ros2 run project forklift_controller.py

# 终端4：一键多目标点导航
ros2 run project waypoint_nav.py
```

**手动设置导航目标**（VNC 桌面中用 RViz）：用 "2D Goal Pose" 工具在地图上点击目标位置。

## 图形界面（VNC 远程桌面）

AutoDL 无显示器，需搭建 VNC 才能在 Mac 上看到 RViz / Gazebo 画面。

### 安装 VNC

```bash
apt install -y xfce4 xfce4-goodies tigervnc-standalone-server tigervnc-common
vncpasswd                   # 设置密码
```

### 启动 VNC

```bash
vncserver :1 -geometry 1920x1080 -depth 24 -localhost no
```

### 开放端口

AutoDL 网页控制台 → "自定义服务" → 添加端口 **5901**。

### Mac 连接

Finder → 前往 → 连接服务器 → `vnc://connect.westb.seetacloud.com:5901` → 输入密码

进入桌面后打开终端，`ros2env` 即可运行 `rviz2`。

### 关闭 VNC

```bash
vncserver -kill :1
```

> VNC 延迟大时可换备选方案：在 AutoDL 用 `map_saver_cli` 保存地图后 `scp` 到 Mac 本地查看。

## 项目结构

```
project/
├── bringup.launch.py              # 全系统一键启动
├── CMakeLists.txt                 # colcon 编译配置
├── package.xml                    # ROS 2 包描述
├── config/
│   ├── cartographer.lua           # Cartographer 2D（LiDAR + odometry）
│   ├── amcl.yaml                  # AMCL 定位参数
│   ├── nav2.yaml                  # Nav2（Smac Hybrid-A* + DWB）
│   ├── slam.rviz                  # SLAM RViz 配置
│   └── nav.rviz                   # 导航 RViz 配置
├── description/
│   └── robot.xacro                # 机器人 URDF
├── worlds/
│   └── warehouse.world            # 仓储场景（4排货架 + 叉车）
├── scripts/
│   ├── waypoint_nav.py            # 多目标点导航
│   ├── forklift_controller.py     # 叉车动态障碍
│   ├── apriltag_detect.py         # AprilTag 辅助定位（扩展）
│   └── setup_autodl.sh            # 一键安装脚本
├── launch/
│   ├── spawn.launch.py            # Gazebo + 机器人
│   ├── slam.launch.py             # Cartographer 2D SLAM
│   ├── rtabmap.launch.py          # RTAB-Map 3D SLAM
│   └── nav.launch.py              # Map Server + AMCL + Nav2
├── maps/                          # 保存的地图
└── video/                         # 演示视频
```

## TF 树

```
map → odom → base_footprint → base_link
              │                ├── laser_link
              │                ├── camera_rgbd_link
              │                ├── camera_mono_link
              │                └── imu_link
              │
              (Cartographer/AMCL 发布 map → odom)
              (diff_drive 插件 发布 odom → base_footprint)
```

## 参数速查

| 参数 | 值 | 说明 |
|------|-----|------|
| `max_vel_x` | 0.5 m/s | 线速度上限 |
| `inflation_radius` | 0.3 m | 障碍物膨胀半径 |
| `goal_tolerance` | 0.2 m / 0.2 rad | 目标到达容差 |
| AMCL 粒子数 | 5000 | 全局定位 |
| Cartographer 子图分辨率 | 0.05 m | 建图精度 |
| RTAB-Map point cloud voxel | 0.05 m | 降采样 |
| 回环检测 min_score | 0.65 | 防误闭合 |

## 故障排查

| 现象 | 原因 | 解决 |
|------|------|------|
| `ros2env: command not found` | 新终端未加载 bashrc | `source ~/.bashrc` |
| `GLIBCXX_3.4.30 not found` | conda libstdc++ 太旧 | 执行 `ros2env`（清除 conda 路径） |
| Cartographer 启动即崩溃 | 仿真时钟未就绪 | 等 Gazebo 完全启动后再开 SLAM |
| 按键遥控无反应 | 终端未获得焦点 | 鼠标点击终端窗口内部 |
| RViz 闪退 | 无头服务器缺 OGRE | 搭 VNC 后运行，或不用 RViz 纯命令行操作 |

## 注意事项

1. 所有 launch 文件默认 `use_sim_time: true`，依赖 Gazebo 的 `/clock` 话题
2. 必须先启动 Gazebo（等机器人 spawn 完成），再启动 SLAM / 导航
3. `ros2env` 会清除 conda 路径，解决 libstdc++ 冲突
4. RTAB-Map 3D 建图内存峰值 8-10GB，实例内存需 16GB+
5. 首次启动 Gazebo 会在线下载模型文件，后续会缓存
