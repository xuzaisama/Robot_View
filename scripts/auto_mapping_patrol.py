#!/usr/bin/env python3
"""
Automatic mapping patrol for Cartographer-based 2D mapping.

The node follows a configured cmd_vel route and uses LaserScan for simple
front-obstacle recovery. It does not save maps automatically; inspect RViz
after completion, wait for Cartographer to settle, then run map_saver_cli.
"""

import math
import os
import signal

import rclpy
from geometry_msgs.msg import Twist
from rclpy.duration import Duration
from rclpy.node import Node
from sensor_msgs.msg import LaserScan

try:
    import yaml
except ImportError:  # pragma: no cover - exercised in ROS environment
    yaml = None


class AutoMappingPatrol(Node):
    def __init__(self):
        super().__init__("auto_mapping_patrol")

        self.declare_parameter("route_file", "")
        self.declare_parameter("route", "safe")
        self.declare_parameter("front_clearance", 0.6)
        self.declare_parameter("max_recovery_attempts", 5)
        self.declare_parameter("cmd_vel_topic", "/cmd_vel")
        self.declare_parameter("scan_topic", "/scan")

        self.front_clearance = float(self.get_parameter("front_clearance").value)
        self.max_recovery_attempts = int(self.get_parameter("max_recovery_attempts").value)
        self.cmd_pub = self.create_publisher(
            Twist, str(self.get_parameter("cmd_vel_topic").value), 10
        )
        self.scan_sub = self.create_subscription(
            LaserScan, str(self.get_parameter("scan_topic").value), self.scan_callback, 10
        )

        route_file = str(self.get_parameter("route_file").value)
        route_name = str(self.get_parameter("route").value)
        self.route_config = self.load_route(route_file, route_name)
        self.steps = self.route_config["steps"]
        self.completion_hint = self.route_config.get("completion_hint", "")

        self.front_min = math.inf
        self.step_index = 0
        self.step_start = self.get_clock().now()
        self.pause_until = None
        self.recovery_actions = []
        self.recovery_index = 0
        self.recovery_start = None
        self.consecutive_recoveries = 0
        self.done = False

        self.timer = self.create_timer(0.1, self.timer_callback)
        signal.signal(signal.SIGINT, self.handle_shutdown_signal)
        signal.signal(signal.SIGTERM, self.handle_shutdown_signal)

        self.get_logger().info(
            f"Auto mapping route '{route_name}' loaded with {len(self.steps)} steps."
        )
        if route_name == "safe":
            self.get_logger().warn(
                "Safe route is for first-run validation only. "
                "After this passes, rerun with route:=coverage for full mapping."
            )
        elif route_name == "coverage":
            self.get_logger().info(
                "Coverage route selected. This slow route is designed for full warehouse mapping."
            )
        elif route_name == "extended":
            self.get_logger().info(
                "Extended route selected. Use this when coverage route misses corners or aisles."
            )

    def load_route(self, route_file, route_name):
        if yaml is None:
            raise RuntimeError("PyYAML is required. Install python3-yaml.")
        if not route_file or not os.path.exists(route_file):
            raise FileNotFoundError(f"Route file not found: {route_file}")

        with open(route_file, "r", encoding="utf-8") as stream:
            data = yaml.safe_load(stream)

        routes = data.get("routes", {})
        if route_name not in routes:
            raise ValueError(f"Route '{route_name}' not found in {route_file}")
        route = routes[route_name]
        steps = route.get("steps", [])
        if not steps:
            raise ValueError(f"Route '{route_name}' has no steps")
        return route

    def scan_callback(self, msg):
        values = []
        for index, distance in enumerate(msg.ranges):
            if not math.isfinite(distance):
                continue
            angle = msg.angle_min + index * msg.angle_increment
            if abs(angle) <= math.radians(30.0):
                values.append(distance)
        self.front_min = min(values) if values else math.inf

    def timer_callback(self):
        if self.done:
            return

        now = self.get_clock().now()

        if self.pause_until is not None:
            if now < self.pause_until:
                self.publish_velocity(0.0, 0.0)
                return
            self.pause_until = None
            self.step_start = now

        if self.recovery_actions:
            self.run_recovery(now)
            return

        if self.front_min < self.front_clearance:
            self.begin_recovery(now)
            return

        if self.step_index >= len(self.steps):
            self.finish()
            return

        step = self.steps[self.step_index]
        elapsed = (now - self.step_start).nanoseconds * 1e-9
        duration = float(step.get("duration", 0.0))

        if elapsed >= duration:
            self.publish_velocity(0.0, 0.0)
            self.get_logger().info(f"Completed step: {step.get('name', self.step_index)}")
            self.step_index += 1
            pause = float(step.get("pause", 0.5))
            self.pause_until = now + Duration(seconds=pause)
            return

        self.publish_velocity(
            float(step.get("linear_x", 0.0)),
            float(step.get("angular_z", 0.0)),
        )

    def begin_recovery(self, now):
        self.consecutive_recoveries += 1
        self.get_logger().warn(
            f"Front obstacle at {self.front_min:.2f}m. "
            f"Recovery {self.consecutive_recoveries}/{self.max_recovery_attempts}."
        )
        if self.consecutive_recoveries > self.max_recovery_attempts:
            self.get_logger().error(
                "Recovery failed too many times. Patrol stopped. "
                "Check Gazebo/RViz, teleop manually, or rerun with route:=safe."
            )
            self.finish()
            return

        self.recovery_actions = [
            {"name": "stop", "linear_x": 0.0, "angular_z": 0.0, "duration": 0.5},
            {"name": "back_up", "linear_x": -0.10, "angular_z": 0.0, "duration": 0.8},
            {"name": "turn_away", "linear_x": 0.0, "angular_z": 0.25, "duration": 1.2},
        ]
        self.recovery_index = 0
        self.recovery_start = now

    def run_recovery(self, now):
        if self.recovery_index >= len(self.recovery_actions):
            self.recovery_actions = []
            self.recovery_start = None
            self.step_start = now
            return

        action = self.recovery_actions[self.recovery_index]
        elapsed = (now - self.recovery_start).nanoseconds * 1e-9
        if elapsed >= float(action["duration"]):
            self.recovery_index += 1
            self.recovery_start = now
            return

        self.publish_velocity(float(action["linear_x"]), float(action["angular_z"]))

    def publish_velocity(self, linear_x, angular_z):
        msg = Twist()
        msg.linear.x = linear_x
        msg.angular.z = angular_z
        self.cmd_pub.publish(msg)

    def finish(self):
        if self.done:
            return
        self.done = True
        self.publish_velocity(0.0, 0.0)
        self.get_logger().info("Auto mapping patrol completed. Robot stopped.")
        if self.completion_hint:
            self.get_logger().info(self.completion_hint)
        self.get_logger().info(
            "Inspect RViz, wait 10-30 seconds for Cartographer optimization, then save with: "
            "ros2 run nav2_map_server map_saver_cli -f ~/warehouse_ws/src/project/maps/warehouse_map"
        )

    def handle_shutdown_signal(self, signum, frame):
        self.get_logger().warn("Shutdown requested. Stopping robot.")
        self.finish()
        rclpy.shutdown()


def main():
    rclpy.init()
    node = AutoMappingPatrol()
    try:
        rclpy.spin(node)
    finally:
        node.publish_velocity(0.0, 0.0)
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
