#!/usr/bin/env python3
"""
Waypoint Navigation Script
Sends sequential navigation goals to Nav2 for multi-point delivery.
Usage: ros2 run project waypoint_nav.py
"""

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from nav2_msgs.action import NavigateToPose
from geometry_msgs.msg import PoseStamped, Point, Quaternion
import math
import time


class WaypointNavigator(Node):
    def __init__(self):
        super().__init__("waypoint_navigator")

        self.action_client = ActionClient(self, NavigateToPose, "navigate_to_pose")

        # Define waypoints: [LoadingZone A, Shelf B corridor, LoadingZone C]
        # Coordinates match the warehouse.world layout
        self.waypoints = [
            # Loading Zone A (right side, bottom)
            {"name": "LoadingZone_A", "x": 6.0, "y": -2.5, "yaw": math.pi},
            # Shelf corridor B (between rows 2 & 3, entrance)
            {"name": "ShelfCorridor_B", "x": -1.0, "y": 0.0, "yaw": 0.0},
            # Loading Zone C (right side, top)
            {"name": "LoadingZone_C", "x": 6.0, "y": 2.5, "yaw": math.pi},
            # Return to start
            {"name": "Home", "x": -8.0, "y": 0.0, "yaw": 0.0},
        ]

        self.current_index = 0
        self.goal_handle = None

        self.get_logger().info("Waypoint Navigator ready")
        self.get_logger().info(
            f"Loaded {len(self.waypoints)} waypoints: "
            + " -> ".join([w["name"] for w in self.waypoints])
        )

    def start(self):
        self.send_next_goal()

    def send_next_goal(self):
        if self.current_index >= len(self.waypoints):
            self.get_logger().info(" All waypoints completed!")
            rclpy.shutdown()
            return

        wp = self.waypoints[self.current_index]
        self.get_logger().info(f" Navigating to {wp['name']} ({wp['x']}, {wp['y']})")

        goal = NavigateToPose.Goal()
        goal.pose = PoseStamped()
        goal.pose.header.frame_id = "map"
        goal.pose.header.stamp = self.get_clock().now().to_msg()
        goal.pose.pose.position = Point(x=wp["x"], y=wp["y"], z=0.0)

        # Convert yaw to quaternion
        cy = math.cos(wp["yaw"] * 0.5)
        sy = math.sin(wp["yaw"] * 0.5)
        goal.pose.pose.orientation = Quaternion(x=0.0, y=0.0, z=sy, w=cy)

        self.get_logger().info(" Waiting for Nav2 action server...")
        self.action_client.wait_for_server()

        send_goal_future = self.action_client.send_goal_async(
            goal, feedback_callback=self.feedback_callback
        )
        send_goal_future.add_done_callback(self.goal_response_callback)

    def goal_response_callback(self, future):
        self.goal_handle = future.result()
        if not self.goal_handle.accepted:
            self.get_logger().error(" Goal rejected by Nav2!")
            self.current_index += 1
            self.send_next_goal()
            return

        self.get_logger().info(" Goal accepted, navigating...")
        self.goal_handle.get_result_async().add_done_callback(self.result_callback)

    def feedback_callback(self, feedback_msg):
        dist = feedback_msg.feedback.distance_remaining
        self.get_logger().debug(f"  Distance remaining: {dist:.2f}m")

    def result_callback(self, future):
        result = future.result()
        wp_name = self.waypoints[self.current_index]["name"]

        if result.status == 4:  # SUCCEEDED
            self.get_logger().info(f" Reached {wp_name}!")
        else:
            self.get_logger().warn(f" Failed to reach {wp_name} (status={result.status})")

        self.current_index += 1
        # Brief pause before next goal
        self.create_timer(1.0, self.send_next_goal)


def main():
    rclpy.init()
    navigator = WaypointNavigator()
    navigator.start()
    rclpy.spin(navigator)


if __name__ == "__main__":
    main()
