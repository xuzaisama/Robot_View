#!/usr/bin/env python3
"""
Spawn robot into Gazebo via /spawn_entity service.
Avoids lxml dependency issue in stock spawn_entity.py.
"""
import sys
import rclpy
from rclpy.node import Node
from gazebo_msgs.srv import SpawnEntity


def main():
    rclpy.init()
    node = Node("spawn_robot_client")

    cli = node.create_client(SpawnEntity, "/spawn_entity")
    while not cli.wait_for_service(timeout_sec=10.0):
        node.get_logger().info("Waiting for /spawn_entity service...")

    req = SpawnEntity.Request()
    req.name = "warehouse_robot"
    req.xml = ""
    req.robot_namespace = ""
    req.initial_pose.position.x = -8.0
    req.initial_pose.position.y = 0.0
    req.initial_pose.position.z = 0.1
    req.initial_pose.orientation.w = 1.0
    req.reference_frame = "world"

    node.get_logger().info("Spawning robot...")
    future = cli.call_async(req)
    rclpy.spin_until_future_complete(node, future, timeout_sec=15.0)

    if future.result() is not None and future.result().success:
        node.get_logger().info("Robot spawned successfully.")
    else:
        node.get_logger().error(f"Spawn failed: {future.result()}")

    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
