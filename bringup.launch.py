import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, TimerAction
from launch.substitutions import Command, LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    pkg_dir = get_package_share_directory("project")

    declare_use_sim_time = DeclareLaunchArgument("use_sim_time", default_value="true")

    world_path = os.path.join(pkg_dir, "worlds", "warehouse.world")
    xacro_path = os.path.join(pkg_dir, "description", "robot.xacro")
    map_file = os.path.join(pkg_dir, "maps", "warehouse_map.yaml")

    # ==================== SIMULATION ====================
    gazebo = ExecuteProcess(
        cmd=["xvfb-run", "-s", "-screen 0 1280x1024x24", "gazebo", "--verbose", world_path,
             "-s", "libgazebo_ros_factory.so"],
        output="screen",
    )

    robot_desc_content = Command(["xacro ", xacro_path])
    robot_desc = {
        "robot_description": ParameterValue(robot_desc_content, value_type=str)
    }
    robot_state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        parameters=[robot_desc, {"use_sim_time": True}],
        output="screen",
    )

    joint_state_publisher = Node(
        package="joint_state_publisher",
        executable="joint_state_publisher",
        parameters=[{"use_sim_time": True}],
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

    forklift_ctrl = Node(
        package="project",
        executable="forklift_controller.py",
        output="screen",
    )

    # ==================== SLAM ====================
    cartographer_config = os.path.join(pkg_dir, "config", "cartographer.lua")
    cartographer_node = Node(
        package="cartographer_ros",
        executable="cartographer_node",
        parameters=[{"use_sim_time": True}],
        arguments=[
            "-configuration_directory", os.path.dirname(cartographer_config),
            "-configuration_basename", os.path.basename(cartographer_config),
        ],
        remappings=[("scan", "/scan"), ("odom", "/odom"), ("imu", "/imu")],
        output="screen",
    )
    cartographer_grid = Node(
        package="cartographer_ros",
        executable="cartographer_occupancy_grid_node",
        parameters=[{"use_sim_time": True}],
        arguments=["-resolution", "0.05", "-publish_period_sec", "1.0"],
        output="screen",
    )

    # ==================== LOCALIZATION ====================
    map_server = Node(
        package="nav2_map_server",
        executable="map_server",
        parameters=[{"use_sim_time": True}, {"yaml_filename": map_file}],
        output="screen",
    )
    amcl = Node(
        package="nav2_amcl",
        executable="amcl",
        parameters=[os.path.join(pkg_dir, "config", "amcl.yaml")],
        remappings=[("scan", "/scan")],
        output="screen",
    )
    lifecycle_loc = Node(
        package="nav2_lifecycle_manager",
        executable="lifecycle_manager",
        name="lifecycle_manager_loc",
        parameters=[
            {"use_sim_time": True}, {"autostart": True},
            {"node_names": ["map_server", "amcl"]},
        ],
        output="screen",
    )

    # ==================== NAVIGATION ====================
    nav2_config = os.path.join(pkg_dir, "config", "nav2.yaml")
    controller_server = Node(
        package="nav2_controller", executable="controller_server",
        parameters=[nav2_config], output="screen",
    )
    planner_server = Node(
        package="nav2_planner", executable="planner_server",
        parameters=[nav2_config], output="screen",
    )
    behavior_server = Node(
        package="nav2_behaviors", executable="behavior_server",
        parameters=[nav2_config], output="screen",
    )
    bt_navigator = Node(
        package="nav2_bt_navigator", executable="bt_navigator",
        parameters=[nav2_config], output="screen",
    )
    waypoint_follower = Node(
        package="nav2_waypoint_follower", executable="waypoint_follower",
        parameters=[nav2_config], output="screen",
    )
    velocity_smoother = Node(
        package="nav2_velocity_smoother", executable="velocity_smoother",
        parameters=[nav2_config], output="screen",
    )
    lifecycle_nav = Node(
        package="nav2_lifecycle_manager",
        executable="lifecycle_manager",
        name="lifecycle_manager_nav",
        parameters=[
            {"use_sim_time": True}, {"autostart": True},
            {"node_names": [
                "controller_server", "planner_server",
                "behavior_server", "bt_navigator",
                "waypoint_follower", "velocity_smoother",
            ]},
        ],
        output="screen",
    )

    # ==================== RViz ====================
    rviz = Node(
        package="rviz2", executable="rviz2",
        arguments=["-d", os.path.join(pkg_dir, "config", "nav.rviz")],
        output="screen",
    )

    return LaunchDescription([
        declare_use_sim_time,
        # --- Simulation ---
        gazebo,
        robot_state_publisher,
        joint_state_publisher,
        TimerAction(period=5.0, actions=[spawn_robot]),
        TimerAction(period=8.0, actions=[forklift_ctrl]),

        # --- SLAM (W2: uncomment to map) ---
        # cartographer_node,
        # cartographer_grid,

        # --- Localization (W3+: uncomment after map saved) ---
        # map_server,
        # amcl,
        # lifecycle_loc,

        # --- Navigation (W4+: uncomment for nav) ---
        # controller_server,
        # planner_server,
        # behavior_server,
        # bt_navigator,
        # waypoint_follower,
        # velocity_smoother,
        # lifecycle_nav,

        # --- Visualization ---
        rviz,
    ])
