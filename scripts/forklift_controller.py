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


def advance_along_y(current_y, direction, speed, dt, y_min, y_max):
    next_y = current_y + direction * speed * dt
    next_direction = direction

    if next_y <= y_min:
        next_y = y_min
        next_direction = 1
    elif next_y >= y_max:
        next_y = y_max
        next_direction = -1

    return next_y, next_direction


class ForkliftController(Node):
    def __init__(self):
        super().__init__("forklift_controller")

        self.cli = self.create_client(SetEntityState, "/set_entity_state")
        while not self.cli.wait_for_service(timeout_sec=5.0):
            self.get_logger().info("Waiting for /set_entity_state service...")

        self.timer = self.create_timer(0.05, self.timer_callback)
        self.last_update = self.get_clock().now()

        # Movement params
        self.y_min = -3.0   # bottom end
        self.y_max = 3.0    # top end
        self.speed = 0.6    # m/s
        self.direction = -1  # start moving toward the bottom end

        self.x = 5.0  # fixed x in main corridor
        self.current_y = 3.0  # start at the top boundary

        self.get_logger().info("Forklift controller started - moving in main corridor")

    def timer_callback(self):
        now = self.get_clock().now()
        dt = (now - self.last_update).nanoseconds * 1e-9
        self.last_update = now

        self.current_y, self.direction = advance_along_y(
            self.current_y,
            self.direction,
            self.speed,
            dt,
            self.y_min,
            self.y_max,
        )

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
