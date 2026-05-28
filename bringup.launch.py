import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, LogInfo, OpaqueFunction, Shutdown, TimerAction
from launch.conditions import IfCondition
from launch.substitutions import Command, LaunchConfiguration, PythonExpression
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def mode_is(mode):
    return IfCondition(PythonExpression(["'", LaunchConfiguration("mode"), "' == '", mode, "'"]))


def mode_in(modes):
    checks = ["'", LaunchConfiguration("mode"), "' == '", modes[0], "'"]
    for mode in modes[1:]:
        checks.extend([" or '", LaunchConfiguration("mode"), "' == '", mode, "'"])
    return IfCondition(PythonExpression(checks))


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
            ". Run slam.launch.py first, then save a map with: ",
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
        choices=["sim", "slam", "nav"],
        description="Bringup mode: sim, slam, or nav",
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

    xacro_path = os.path.join(pkg_dir, "description", "robot.xacro")
    world = LaunchConfiguration("world")
    map_file = LaunchConfiguration("map")

    # ==================== SIMULATION ====================
    gazebo = ExecuteProcess(
        cmd=["xvfb-run", "-s", "-screen 0 1280x1024x24", "gazebo", "--verbose", world,
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

    # ==================== LOCALIZATION ====================
    map_server = Node(
        package="nav2_map_server",
        executable="map_server",
        parameters=[{"use_sim_time": True}, {"yaml_filename": map_file}],
        condition=mode_is("nav"),
        output="screen",
    )
    amcl = Node(
        package="nav2_amcl",
        executable="amcl",
        parameters=[os.path.join(pkg_dir, "config", "amcl.yaml")],
        remappings=[("scan", "/scan")],
        condition=mode_is("nav"),
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
        condition=mode_is("nav"),
        output="screen",
    )

    # ==================== NAVIGATION ====================
    nav2_config = os.path.join(pkg_dir, "config", "nav2.yaml")
    controller_server = Node(
        package="nav2_controller", executable="controller_server",
        parameters=[nav2_config], condition=mode_is("nav"), output="screen",
    )
    planner_server = Node(
        package="nav2_planner", executable="planner_server",
        parameters=[nav2_config], condition=mode_is("nav"), output="screen",
    )
    behavior_server = Node(
        package="nav2_behaviors", executable="behavior_server",
        parameters=[nav2_config], condition=mode_is("nav"), output="screen",
    )
    bt_navigator = Node(
        package="nav2_bt_navigator", executable="bt_navigator",
        parameters=[nav2_config], condition=mode_is("nav"), output="screen",
    )
    waypoint_follower = Node(
        package="nav2_waypoint_follower", executable="waypoint_follower",
        parameters=[nav2_config], condition=mode_is("nav"), output="screen",
    )
    velocity_smoother = Node(
        package="nav2_velocity_smoother", executable="velocity_smoother",
        parameters=[nav2_config], condition=mode_is("nav"), output="screen",
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
        condition=mode_is("nav"),
        output="screen",
    )

    # ==================== RViz ====================
    rviz_nav = Node(
        package="rviz2", executable="rviz2",
        arguments=["-d", os.path.join(pkg_dir, "config", "nav.rviz")],
        condition=mode_in(["sim", "nav"]),
        output="screen",
    )
    rviz_slam = Node(
        package="rviz2", executable="rviz2",
        arguments=["-d", os.path.join(pkg_dir, "config", "slam.rviz")],
        condition=mode_is("slam"),
        output="screen",
    )

    return LaunchDescription([
        declare_use_sim_time,
        declare_mode,
        declare_world,
        declare_map,
        OpaqueFunction(function=validate_nav_map),
        gazebo,
        robot_state_publisher,
        joint_state_publisher,
        TimerAction(period=5.0, actions=[spawn_robot]),
        TimerAction(period=8.0, actions=[forklift_ctrl]),
        Node(
            package="cartographer_ros",
            executable="cartographer_node",
            parameters=[{"use_sim_time": True}],
            arguments=[
                "-configuration_directory", os.path.dirname(cartographer_config),
                "-configuration_basename", os.path.basename(cartographer_config),
            ],
            remappings=[("scan", "/scan"), ("odom", "/odom"), ("imu", "/imu")],
            condition=mode_is("slam"),
            output="screen",
        ),
        Node(
            package="cartographer_ros",
            executable="cartographer_occupancy_grid_node",
            parameters=[{"use_sim_time": True}],
            arguments=["-resolution", "0.05", "-publish_period_sec", "1.0"],
            condition=mode_is("slam"),
            output="screen",
        ),
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
