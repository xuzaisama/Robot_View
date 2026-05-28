import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, LogInfo, OpaqueFunction, Shutdown
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def validate_map_file(context):
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

    declare_use_sim_time = DeclareLaunchArgument(
        "use_sim_time", default_value="true"
    )
    declare_map = DeclareLaunchArgument(
        "map", default_value=os.path.join(pkg_dir, "maps", "warehouse_map.yaml"),
        description="Path to saved map YAML"
    )

    map_file = LaunchConfiguration("map")

    # ========== Map Server ==========
    map_server = Node(
        package="nav2_map_server",
        executable="map_server",
        name="map_server",
        parameters=[
            {"use_sim_time": True},
            {"yaml_filename": map_file},
        ],
        output="screen",
    )

    lifecycle_map = Node(
        package="nav2_lifecycle_manager",
        executable="lifecycle_manager",
        name="lifecycle_manager_map",
        parameters=[
            {"use_sim_time": True},
            {"autostart": True},
            {"node_names": ["map_server"]},
        ],
        output="screen",
    )

    # ========== AMCL Localization ==========
    amcl_config = os.path.join(pkg_dir, "config", "amcl.yaml")
    amcl = Node(
        package="nav2_amcl",
        executable="amcl",
        name="amcl",
        parameters=[amcl_config],
        remappings=[("scan", "/scan")],
        output="screen",
    )

    lifecycle_amcl = Node(
        package="nav2_lifecycle_manager",
        executable="lifecycle_manager",
        name="lifecycle_manager_amcl",
        parameters=[
            {"use_sim_time": True},
            {"autostart": True},
            {"node_names": ["amcl"]},
        ],
        output="screen",
    )

    # ========== Nav2 Stack ==========
    nav2_config = os.path.join(pkg_dir, "config", "nav2.yaml")

    controller_server = Node(
        package="nav2_controller",
        executable="controller_server",
        name="controller_server",
        parameters=[nav2_config],
        output="screen",
    )

    planner_server = Node(
        package="nav2_planner",
        executable="planner_server",
        name="planner_server",
        parameters=[nav2_config],
        output="screen",
    )

    behavior_server = Node(
        package="nav2_behaviors",
        executable="behavior_server",
        name="behavior_server",
        parameters=[nav2_config],
        output="screen",
    )

    bt_navigator = Node(
        package="nav2_bt_navigator",
        executable="bt_navigator",
        name="bt_navigator",
        parameters=[nav2_config],
        output="screen",
    )

    waypoint_follower = Node(
        package="nav2_waypoint_follower",
        executable="waypoint_follower",
        name="waypoint_follower",
        parameters=[nav2_config],
        output="screen",
    )

    velocity_smoother = Node(
        package="nav2_velocity_smoother",
        executable="velocity_smoother",
        name="velocity_smoother",
        parameters=[nav2_config],
        output="screen",
    )

    nav_nodes = [
        "controller_server",
        "planner_server",
        "behavior_server",
        "bt_navigator",
        "waypoint_follower",
        "velocity_smoother",
    ]

    lifecycle_nav = Node(
        package="nav2_lifecycle_manager",
        executable="lifecycle_manager",
        name="lifecycle_manager_nav",
        parameters=[
            {"use_sim_time": True},
            {"autostart": True},
            {"node_names": nav_nodes},
        ],
        output="screen",
    )

    # ========== RViz ==========
    rviz_config = os.path.join(pkg_dir, "config", "nav.rviz")
    rviz = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        arguments=["-d", rviz_config],
        output="screen",
        env={"QT_QPA_PLATFORM": "offscreen"},
    )

    return LaunchDescription([
        declare_use_sim_time,
        declare_map,
        OpaqueFunction(function=validate_map_file),
        map_server,
        lifecycle_map,
        amcl,
        lifecycle_amcl,
        controller_server,
        planner_server,
        behavior_server,
        bt_navigator,
        waypoint_follower,
        velocity_smoother,
        lifecycle_nav,
        rviz,
    ])
