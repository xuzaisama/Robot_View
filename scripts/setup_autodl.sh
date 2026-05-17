#!/bin/bash
# =============================================================
# AutoDL Setup — ROS 2 Humble + Gazebo + Cartographer + Nav2
# 镜像: PyTorch 2.1.0 / Python 3.10 / CUDA 12.1 / Ubuntu 22.04
# =============================================================
set -e

# 退出 conda 避免库路径污染
if [ -n "$CONDA_DEFAULT_ENV" ]; then
    echo "Deactivating conda env: $CONDA_DEFAULT_ENV..."
    conda deactivate
fi

export PATH="/usr/bin:$PATH"

# AutoDL 容器默认是 root
if [ "$(id -u)" -eq 0 ]; then
    SUDO=""
else
    SUDO="sudo"
fi

echo "============================================"
echo " Warehouse Robot Navigation - AutoDL Setup"
echo " Python: $(python3 --version 2>&1) | ROS 2 Humble"
echo "============================================"

# ---------- 0. 基础依赖 ----------
echo "[0/8] Installing base dependencies..."

$SUDO apt update
$SUDO apt install -y locales curl gnupg2 lsb-release software-properties-common
$SUDO locale-gen en_US en_US.UTF-8
$SUDO update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8
export LANG=en_US.UTF-8

# ---------- 1. ROS 2 Humble ----------
echo "[1/8] Installing ROS 2 Humble..."

if [ -d "/opt/ros/humble" ]; then
    echo "  ROS 2 Humble already installed, skipping."
else
    $SUDO add-apt-repository -y universe
    $SUDO curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key \
        -o /usr/share/keyrings/ros-archive-keyring.gpg

    echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] \
        https://mirrors.tuna.tsinghua.edu.cn/ros2/ubuntu $(. /etc/os-release && echo $UBUNTU_CODENAME) main" \
        | $SUDO tee /etc/apt/sources.list.d/ros2.list > /dev/null

    $SUDO apt update
    $SUDO apt install -y ros-humble-desktop
    echo "  ROS 2 Humble installed."
fi

# ---------- 2. Gazebo Classic ----------
echo "[2/8] Installing Gazebo + ROS-Gazebo integration..."

$SUDO apt install -y gazebo libgazebo-dev
$SUDO apt install -y ros-humble-gazebo-ros-pkgs ros-humble-gazebo-ros2-control

# ---------- 3. 开发工具 ----------
echo "[3/8] Installing dev tools..."

$SUDO apt install -y \
    python3-pip \
    python3-colcon-common-extensions \
    python3-rosdep \
    ros-humble-robot-state-publisher \
    ros-humble-joint-state-publisher \
    ros-humble-xacro \
    ros-humble-teleop-twist-keyboard \
    ros-humble-rviz2 \
    ros-humble-rviz-common \
    ros-humble-rqt-robot-steering

# ---------- 4. Cartographer ----------
echo "[4/8] Installing Cartographer..."

$SUDO apt install -y ros-humble-cartographer ros-humble-cartographer-ros

# ---------- 5. RTAB-Map ----------
echo "[5/8] Installing RTAB-Map..."

$SUDO apt install -y ros-humble-rtabmap-ros ros-humble-rtabmap

# ---------- 6. Nav2 ----------
echo "[6/8] Installing Nav2..."

$SUDO apt install -y \
    ros-humble-navigation2 \
    ros-humble-nav2-bringup \
    ros-humble-nav2-amcl \
    ros-humble-nav2-map-server \
    ros-humble-nav2-lifecycle-manager \
    ros-humble-nav2-planner \
    ros-humble-nav2-controller \
    ros-humble-nav2-behaviors \
    ros-humble-nav2-bt-navigator \
    ros-humble-nav2-waypoint-follower \
    ros-humble-nav2-velocity-smoother \
    ros-humble-nav2-costmap-2d \
    ros-humble-nav2-smac-planner

# ---------- 7. rosdep 初始化 ----------
echo "[7/8] Initializing rosdep..."

if [ ! -f "/etc/ros/rosdep/sources.list.d/20-default.list" ]; then
    $SUDO rosdep init
fi
rosdep update

# ---------- 8. 工作空间 ----------
echo "[8/8] Setting up workspace..."

WORKSPACE="$HOME/warehouse_ws"

# 编译
echo ""
echo "Building workspace..."
echo "  Python: $(which python3) ($(python3 --version 2>&1))"

source /opt/ros/humble/setup.bash

cd "$WORKSPACE"
colcon build --symlink-install --cmake-args -DPython3_EXECUTABLE=/usr/bin/python3

# ========== 环境配置 ==========
if ! grep -q "### ROS 2 Humble ###" ~/.bashrc 2>/dev/null; then
    cat >> ~/.bashrc << 'BASHRC_EOF'

### ROS 2 Humble ###
ros2env() {
    if [ -n "$CONDA_DEFAULT_ENV" ]; then
        conda deactivate
    fi
    source /opt/ros/humble/setup.bash
    source ~/warehouse_ws/install/setup.bash 2>/dev/null
    echo "ROS 2 Humble + warehouse_ws activated."
}
BASHRC_EOF
fi

echo ""
echo "============================================"
echo " Setup Complete!"
echo "============================================"
echo ""
echo " 每次新终端先执行:"
echo "   $ ros2env"
echo ""
echo " 各周命令:"
echo "   W1: ros2 launch project spawn.launch.py"
echo "   W2: ros2 launch project slam.launch.py"
echo "   W3-4: ros2 launch project nav.launch.py"
echo ""
echo "============================================"
