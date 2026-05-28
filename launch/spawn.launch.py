import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, TimerAction
from launch.substitutions import Command, LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    pkg_dir = get_package_share_directory("project")

    world_path = os.path.join(pkg_dir, "worlds", "warehouse.world")
    xacro_path = os.path.join(pkg_dir, "description", "robot.xacro")

    declare_use_sim_time = DeclareLaunchArgument(
        "use_sim_time", default_value="true"
    )
    declare_world = DeclareLaunchArgument(
        "world", default_value=world_path
    )
    world = LaunchConfiguration("world")

    gazebo = ExecuteProcess(
        cmd=["xvfb-run", "-s", "-screen 0 1280x1024x24", "gazebo", "--verbose", world,
             "-s", "libgazebo_ros_factory.so"],
        output="screen",
    )

    robot_description_content = Command(["xacro ", xacro_path])
    robot_description = {
        "robot_description": ParameterValue(
            robot_description_content,
            value_type=str
        )
    }
    robot_state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        parameters=[robot_description, {"use_sim_time": True}],
        output="screen",
    )

    spawn_robot = Node(
        package="gazebo_ros",
        executable="spawn_entity.py",
        arguments=[
            "-entity", "warehouse_robot",
            "-topic", "robot_description",
            "-x", "-8.0", "-y", "0.0", "-z", "0.1", "-Y", "0.0",
        ],
        output="screen",
    )

    joint_state_publisher = Node(
        package="joint_state_publisher",
        executable="joint_state_publisher",
        parameters=[{"use_sim_time": True}],
        output="screen",
    )

    return LaunchDescription([
        declare_use_sim_time,
        declare_world,
        gazebo,
        robot_state_publisher,
        joint_state_publisher,
        TimerAction(period=5.0, actions=[spawn_robot]),
    ])
