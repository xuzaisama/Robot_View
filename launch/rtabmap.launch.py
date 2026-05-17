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

    rtabmap_node = Node(
        package="rtabmap_slam",
        executable="rtabmap",
        name="rtabmap",
        output="screen",
        parameters=[{
            "use_sim_time": True,
            "frame_id": "base_link",
            "subscribe_depth": True,
            "subscribe_rgb": True,
            "subscribe_scan": False,
            "wait_for_transform": 0.2,
            "approx_sync": True,
            "queue_size": 30,
            "odom_frame_id": "odom",
            "map_frame_id": "map",
            "publish_tf": True,
            "tf_delay": 0.05,
            "cloud_voxel_size": 0.05,
            "cloud_decimation": 4,
            "cloud_max_depth": 8.0,
            "cloud_min_depth": 0.2,
            "grid_size": 0.05,
            "grid_eroded": False,
            "grid_from_3d": True,
            "grid_3d": True,
            "mem/notLinkedNodesKept": False,
            "Rtabmap/DetectionRate": 1.0,
            "Rtabmap/TimeThr": 700,
            "Rtabmap/MemoryThr": 300,
            "Rtabmap/CreateOccupancyGrid": True,
            "RGBD/NeighborLinkRefining": True,
            "RGBD/AngularUpdate": 0.01,
            "RGBD/LinearUpdate": 0.01,
            "RGBD/ProximityBySpace": True,
            "Reg/Force3DoF": True,
            "Optimizer/GravitySigma": 0.3,
            "Kp/MaxFeatures": 400,
        }],
        remappings=[
            ("rgb/image", "/camera/rgb/image_raw"),
            ("depth/image", "/camera/depth/image_raw"),
            ("rgb/camera_info", "/camera/rgb/camera_info"),
            ("odom", "/odom"),
        ],
    )

    rtabmap_viz = Node(
        package="rtabmap_viz",
        executable="rtabmap_viz",
        name="rtabmap_viz",
        parameters=[{"use_sim_time": True}],
        output="screen",
    )

    rviz_config = os.path.join(pkg_dir, "config", "slam.rviz")
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
        rtabmap_node,
        rtabmap_viz,
        rviz,
    ])
