#!/usr/bin/env python3
"""
Optional: AprilTag Detection for Visual Localization
Activates when the mono camera plugin is enabled in robot.xacro.
Requires: pip install apriltag opencv-python
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, CameraInfo
from visualization_msgs.msg import Marker, MarkerArray
from geometry_msgs.msg import PoseStamped
import cv2
import numpy as np

try:
    from cv_bridge import CvBridge
    import apriltag
    HAS_APRILTAG = True
except ImportError:
    HAS_APRILTAG = False


class AprilTagDetector(Node):
    def __init__(self):
        super().__init__("apriltag_detector")

        if not HAS_APRILTAG:
            self.get_logger().error(
                "apriltag or cv_bridge not installed. "
                "Run: pip install apriltag opencv-python"
            )
            return

        self.bridge = CvBridge()

        # AprilTag detector (tag36h11 family)
        self.detector = apriltag.Detector(
            apriltag.DetectorOptions(families="tag36h11")
        )

        # Subscribers
        self.image_sub = self.create_subscription(
            Image, "/mono/image_raw", self.image_callback, 10
        )
        self.camera_info_sub = self.create_subscription(
            CameraInfo, "/mono/camera_info", self.camera_info_callback, 10
        )

        # Publishers
        self.marker_pub = self.create_publisher(MarkerArray, "/apriltag/markers", 10)
        self.pose_pub = self.create_publisher(PoseStamped, "/apriltag/pose", 10)

        self.intrinsics = None
        self.tag_size = 0.1  # meters (10cm tags)

        self.get_logger().info("AprilTag Detector initialized")

    def camera_info_callback(self, msg):
        if self.intrinsics is None:
            self.intrinsics = np.array(msg.k).reshape(3, 3)
            self.get_logger().info("Camera intrinsics received")

    def image_callback(self, msg):
        if self.intrinsics is None:
            return

        try:
            gray = self.bridge.imgmsg_to_cv2(msg, "mono8")
        except Exception as e:
            self.get_logger().error(f"Image conversion error: {e}")
            return

        tags = self.detector.detect(gray)

        if not tags:
            return

        marker_array = MarkerArray()
        for i, tag in enumerate(tags):
            # Estimate pose
            pose, e0, e1 = self.detector.detection_pose(
                tag, self.intrinsics, self.tag_size
            )

            marker = Marker()
            marker.header = msg.header
            marker.ns = "apriltag"
            marker.id = tag.tag_id
            marker.type = Marker.CUBE
            marker.action = Marker.ADD
            marker.pose.position.x = float(pose[0, 3])
            marker.pose.position.y = float(pose[1, 3])
            marker.pose.position.z = float(pose[2, 3])
            # Rotation matrix to quaternion
            marker.pose.orientation.w = (
                float(np.sqrt(1.0 + pose[0, 0] + pose[1, 1] + pose[2, 2])) / 2.0
            )
            marker.pose.orientation.x = (
                float(pose[2, 1] - pose[1, 2])
                / (4.0 * marker.pose.orientation.w)
            )
            marker.pose.orientation.y = (
                float(pose[0, 2] - pose[2, 0])
                / (4.0 * marker.pose.orientation.w)
            )
            marker.pose.orientation.z = (
                float(pose[1, 0] - pose[0, 1])
                / (4.0 * marker.pose.orientation.w)
            )
            marker.scale.x = self.tag_size
            marker.scale.y = self.tag_size
            marker.scale.z = 0.01
            marker.color.a = 1.0
            marker.color.r = 0.0
            marker.color.g = 1.0
            marker.color.b = 0.0
            marker_array.markers.append(marker)

            self.get_logger().debug(
                f"Tag {tag.tag_id}: x={marker.pose.position.x:.2f}, "
                f"y={marker.pose.position.y:.2f}"
            )

        self.marker_pub.publish(marker_array)


def main():
    rclpy.init()
    node = AprilTagDetector()
    rclpy.spin(node)


if __name__ == "__main__":
    main()
