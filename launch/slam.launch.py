import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch_ros.actions import Node


def generate_launch_description():
    pkg_dir = get_package_share_directory("project")

    declare_use_sim_time = DeclareLaunchArgument(
        "use_sim_time", default_value="true"
    )

    cartographer_config = os.path.join(pkg_dir, "config", "cartographer.lua")

    cartographer_node = Node(
        package="cartographer_ros",
        executable="cartographer_node",
        name="cartographer_node",
        output="screen",
        parameters=[{"use_sim_time": True}],
        arguments=[
            "-configuration_directory", os.path.dirname(cartographer_config),
            "-configuration_basename", os.path.basename(cartographer_config),
        ],
        remappings=[
            ("scan", "/scan"),
            ("odom", "/odom"),
            ("imu", "/imu"),
        ],
    )

    cartographer_grid = Node(
        package="cartographer_ros",
        executable="cartographer_occupancy_grid_node",
        name="cartographer_occupancy_grid_node",
        output="screen",
        parameters=[{"use_sim_time": True}],
        arguments=["-resolution", "0.05", "-publish_period_sec", "1.0"],
    )

    # RViz disabled on headless server
    # Use: ros2 topic echo /map to verify mapping

    return LaunchDescription([
        declare_use_sim_time,
        cartographer_node,
        cartographer_grid,
    ])
