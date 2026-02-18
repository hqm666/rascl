#!/usr/bin/env python3
"""send_task.py"""

import rclpy
from std_msgs.msg import String
import json

rclpy.init()
node = rclpy.create_node("sender")
pub = node.create_publisher(String, "/pick_and_place_task", 10)

import time

time.sleep(0.5)

# Your positions
positions = {
    "prepick": [-1.27, -0.65, 0.4, 0.0],
    "pick": [-1.27, -0.8, 0.4, 0.0],
    "grasped": [-1.27, -0.8, 0.4, -0.9],
    "safety": [0, -0.2, 0.4, -0.9],
    "preplace": [1.5, -0.65, 0.35, -0.9],
    "place": [1.5, -0.85, 0.25, -0.9],
    "unloaded": [1.5, -0.85, 0.25, 0.0],
    "postplace": [1.5, -0.65, 0.35, 0.0],
}

# Create message
msg = String()
msg.data = json.dumps({"positions": positions})

pub.publish(msg)
print("✓ Sent task")

rclpy.spin_once(node, timeout_sec=1.0)
node.destroy_node()
rclpy.shutdown()
