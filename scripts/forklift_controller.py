#!/usr/bin/env python3
"""
Forklift Controller - moves the forklift model via Gazebo set_model_state.
This creates a dynamic obstacle that moves back and forth in the main corridor.
"""

import rclpy
from rclpy.node import Node
from gazebo_msgs.srv import SetEntityState
from gazebo_msgs.msg import EntityState
from geometry_msgs.msg import Pose, Twist
import time


class ForkliftController(Node):
    def __init__(self):
        super().__init__("forklift_controller")

        self.cli = self.create_client(SetEntityState, "/set_entity_state")
        while not self.cli.wait_for_service(timeout_sec=5.0):
            self.get_logger().info("Waiting for /set_entity_state service...")

        self.timer = self.create_timer(0.05, self.timer_callback)
        self.start_time = self.get_clock().now()

        # Movement params
        self.y_min = 3.0   # top end
        self.y_max = -3.0   # bottom end (forklift moves along y in main corridor)
        self.speed = 0.6    # m/s
        self.direction = -1  # start moving toward y_min

        self.x = 5.0  # fixed x in main corridor
        self.current_y = 3.5  # start position

        self.get_logger().info("Forklift controller started - moving in main corridor")

    def timer_callback(self):
        now = self.get_clock().now()
        dt = (now - self.start_time).nanoseconds * 1e-9
        self.start_time = now

        # Update position
        self.current_y += self.direction * self.speed * 0.05  # 0.05s step

        # Reverse at bounds
        if self.current_y <= self.y_min:
            self.current_y = self.y_min
            self.direction = 1
        elif self.current_y >= self.y_max:
            self.current_y = self.y_max
            self.direction = -1

        # Send state to Gazebo
        req = SetEntityState.Request()
        req.state = EntityState()
        req.state.name = "forklift"
        req.state.pose = Pose()
        req.state.pose.position.x = self.x
        req.state.pose.position.y = self.current_y
        req.state.pose.position.z = 0.3
        req.state.pose.orientation.w = 1.0
        req.state.twist = Twist()
        req.state.twist.linear.y = self.direction * self.speed
        req.state.reference_frame = "world"

        self.cli.call_async(req)


def main():
    rclpy.init()
    node = ForkliftController()
    rclpy.spin(node)


if __name__ == "__main__":
    main()
