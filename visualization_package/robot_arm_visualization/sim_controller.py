#!/usr/bin/env python3
import rclpy
from rclpy.node import Node

class GazeboController(Node):
    def __init__(self):
        super().__init__("sim_controller_node")
        self.get_logger().info("Gazebo Sim Controller Node is Running")

    
def main(args=None):
    rclpy.init(args=args)

    node = GazeboController()
    rclpy.spin(node)

    rclpy.shutdown()
