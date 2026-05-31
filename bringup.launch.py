import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, LogInfo, OpaqueFunction, Shutdown, TimerAction
from launch.conditions import IfCondition
from launch.substitutions import Command, LaunchConfiguration, PythonExpression
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def condition_expr(parts):
    return IfCondition(PythonExpression(parts))


def launch_equals(name, value):
    return condition_expr(["'", LaunchConfiguration(name), "' == '", value, "'"])


def launch_in(name, values):
    parts = ["("]
    for index, value in enumerate(values):
        if index:
            parts.append(" or ")
        parts.extend(["'", LaunchConfiguration(name), "' == '", value, "'"])
    parts.append(")")
    return condition_expr(parts)


def launch_bool(name):
    return condition_expr(["'", LaunchConfiguration(name), "' == 'true'"])


def launch_bool_and(name, extra_parts):
    return condition_expr(["'", LaunchConfiguration(name), "' == 'true' and (", *extra_parts, ")"])


def mode_expr(values):
    parts = []
    for index, value in enumerate(values):
        if index:
            parts.append(" or ")
        parts.extend(["'", LaunchConfiguration("mode"), "' == '", value, "'"])
    return parts


def validate_nav_map(context):
    mode = LaunchConfiguration("mode").perform(context)
    if mode != "nav":
        return []

    map_path = os.path.expanduser(LaunchConfiguration("map").perform(context))
    if os.path.exists(map_path):
        return []

    return [
        LogInfo(msg=[
            "Navigation map not found: ", map_path,
            ". Run slam_auto or slam first, then save a map with: ",
            "ros2 run nav2_map_server map_saver_cli -f ~/warehouse_ws/src/project/maps/warehouse_map",
        ]),
        Shutdown(reason="navigation map is missing"),
    ]


def generate_launch_description():
    pkg_dir = get_package_share_directory("project")

    declare_use_sim_time = DeclareLaunchArgument("use_sim_time", default_value="true")
    declare_mode = DeclareLaunchArgument(
        "mode",
        default_value="sim",
        choices=["sim", "slam", "slam_auto", "nav"],
        description="Bringup mode: sim, slam, slam_auto, or nav",
    )
    declare_gui = DeclareLaunchArgument(
        "gui",
        default_value="true",
        choices=["true", "false"],
        description="Start Gazebo with a visible GUI when true; use xvfb headless mode when false",
    )
    declare_rviz = DeclareLaunchArgument(
        "rviz",
        default_value="true",
        choices=["true", "false"],
        description="Start RViz when true",
    )
    declare_performance = DeclareLaunchArgument(
        "performance",
        default_value="low",
        choices=["low", "normal"],
        description="Sensor load profile for VirtualBox stability or full-resolution simulation",
    )
    declare_route = DeclareLaunchArgument(
        "route",
        default_value="safe",
        choices=["safe", "coverage", "extended"],
        description="Auto-mapping route used by mode:=slam_auto",
    )
    declare_dynamic_obstacles = DeclareLaunchArgument(
        "dynamic_obstacles",
        default_value="true",
        choices=["true", "false"],
        description="Move the dynamic forklift obstacle when true; keep false while mapping",
    )
    declare_world = DeclareLaunchArgument(
        "world",
        default_value=os.path.join(pkg_dir, "worlds", "warehouse.world"),
        description="Path to Gazebo world file",
    )
    declare_map = DeclareLaunchArgument(
        "map",
        default_value=os.path.join(pkg_dir, "maps", "warehouse_map.yaml"),
        description="Path to saved map YAML for nav mode",
    )

    world = LaunchConfiguration("world")
    map_file = LaunchConfiguration("map")
    route = LaunchConfiguration("route")
    route_file = os.path.join(pkg_dir, "config", "auto_mapping_route.yaml")
    xacro_path = os.path.join(pkg_dir, "description", "robot.xacro")

    lidar_samples = PythonExpression(["'1080' if '", LaunchConfiguration("performance"), "' == 'normal' else '540'"])
    lidar_update_rate = PythonExpression(["'10' if '", LaunchConfiguration("performance"), "' == 'normal' else '8'"])
    camera_width = PythonExpression(["'640' if '", LaunchConfiguration("performance"), "' == 'normal' else '320'"])
    camera_height = PythonExpression(["'480' if '", LaunchConfiguration("performance"), "' == 'normal' else '240'"])
    camera_update_rate = PythonExpression(["'15' if '", LaunchConfiguration("performance"), "' == 'normal' else '8'"])
    imu_update_rate = PythonExpression(["'100' if '", LaunchConfiguration("performance"), "' == 'normal' else '50'"])

    gazebo_gui = ExecuteProcess(
        cmd=["gazebo", "--verbose", world, "-s", "libgazebo_ros_factory.so"],
        condition=launch_bool("gui"),
        output="screen",
    )
    gazebo_headless = ExecuteProcess(
        cmd=["xvfb-run", "-s", "-screen 0 1280x1024x24", "gazebo", "--verbose", world,
             "-s", "libgazebo_ros_factory.so"],
        condition=condition_expr(["'", LaunchConfiguration("gui"), "' == 'false'"]),
        output="screen",
    )

    robot_desc_content = Command([
        "xacro ", xacro_path,
        " lidar_samples:=", lidar_samples,
        " lidar_update_rate:=", lidar_update_rate,
        " camera_width:=", camera_width,
        " camera_height:=", camera_height,
        " camera_update_rate:=", camera_update_rate,
        " imu_update_rate:=", imu_update_rate,
    ])
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
            "-x", "-8.0", "-y", "0.0", "-z", "0.0", "-Y", "0.0",
        ],
        output="screen",
    )

    forklift_ctrl = Node(
        package="project",
        executable="forklift_controller.py",
        condition=launch_bool_and("dynamic_obstacles", mode_expr(["sim", "nav"])),
        output="screen",
    )

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
        condition=launch_in("mode", ["slam", "slam_auto"]),
        output="screen",
    )
    cartographer_grid = Node(
        package="cartographer_ros",
        executable="cartographer_occupancy_grid_node",
        parameters=[{"use_sim_time": True}],
        arguments=["-resolution", "0.05", "-publish_period_sec", "1.0"],
        condition=launch_in("mode", ["slam", "slam_auto"]),
        output="screen",
    )
    auto_mapping_patrol = Node(
        package="project",
        executable="auto_mapping_patrol.py",
        parameters=[{
            "route_file": route_file,
            "route": route,
            "front_clearance": 0.6,
            "max_recovery_attempts": 5,
            "cmd_vel_topic": "/cmd_vel",
            "scan_topic": "/scan",
        }],
        condition=launch_equals("mode", "slam_auto"),
        output="screen",
    )

    map_server = Node(
        package="nav2_map_server",
        executable="map_server",
        parameters=[{"use_sim_time": True}, {"yaml_filename": map_file}],
        condition=launch_equals("mode", "nav"),
        output="screen",
    )
    amcl = Node(
        package="nav2_amcl",
        executable="amcl",
        parameters=[os.path.join(pkg_dir, "config", "amcl.yaml")],
        remappings=[("scan", "/scan")],
        condition=launch_equals("mode", "nav"),
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
        condition=launch_equals("mode", "nav"),
        output="screen",
    )

    nav2_config = os.path.join(pkg_dir, "config", "nav2.yaml")
    controller_server = Node(
        package="nav2_controller", executable="controller_server",
        parameters=[nav2_config], condition=launch_equals("mode", "nav"), output="screen",
    )
    planner_server = Node(
        package="nav2_planner", executable="planner_server",
        parameters=[nav2_config], condition=launch_equals("mode", "nav"), output="screen",
    )
    behavior_server = Node(
        package="nav2_behaviors", executable="behavior_server",
        parameters=[nav2_config], condition=launch_equals("mode", "nav"), output="screen",
    )
    bt_navigator = Node(
        package="nav2_bt_navigator", executable="bt_navigator",
        parameters=[nav2_config], condition=launch_equals("mode", "nav"), output="screen",
    )
    waypoint_follower = Node(
        package="nav2_waypoint_follower", executable="waypoint_follower",
        parameters=[nav2_config], condition=launch_equals("mode", "nav"), output="screen",
    )
    velocity_smoother = Node(
        package="nav2_velocity_smoother", executable="velocity_smoother",
        parameters=[nav2_config], condition=launch_equals("mode", "nav"), output="screen",
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
        condition=launch_equals("mode", "nav"),
        output="screen",
    )

    rviz_nav = Node(
        package="rviz2", executable="rviz2",
        arguments=["-d", os.path.join(pkg_dir, "config", "nav.rviz")],
        condition=launch_bool_and("rviz", mode_expr(["sim", "nav"])),
        output="screen",
    )
    rviz_slam = Node(
        package="rviz2", executable="rviz2",
        arguments=["-d", os.path.join(pkg_dir, "config", "slam.rviz")],
        condition=launch_bool_and("rviz", mode_expr(["slam", "slam_auto"])),
        output="screen",
    )

    return LaunchDescription([
        declare_use_sim_time,
        declare_mode,
        declare_gui,
        declare_rviz,
        declare_performance,
        declare_route,
        declare_dynamic_obstacles,
        declare_world,
        declare_map,
        OpaqueFunction(function=validate_nav_map),
        gazebo_gui,
        gazebo_headless,
        robot_state_publisher,
        joint_state_publisher,
        TimerAction(period=5.0, actions=[spawn_robot]),
        TimerAction(period=8.0, actions=[forklift_ctrl]),
        cartographer_node,
        cartographer_grid,
        TimerAction(period=12.0, actions=[auto_mapping_patrol]),
        map_server,
        amcl,
        lifecycle_loc,
        controller_server,
        planner_server,
        behavior_server,
        bt_navigator,
        waypoint_follower,
        velocity_smoother,
        lifecycle_nav,
        rviz_nav,
        rviz_slam,
    ])
