"""One-shot localization error check: Gazebo truth vs Cartographer.

Prints a single line. Exits non-zero if it cannot measure.
"""
import math
import sys
import time

import rclpy
from nav_msgs.msg import Odometry
from rclpy.node import Node
from tf2_ros import Buffer, TransformListener

rclpy.init()
node = Node("drift_check", parameter_overrides=[])
truth = {}
node.create_subscription(Odometry, "/odometry", lambda m: truth.update(
    x=m.pose.pose.position.x, y=m.pose.pose.position.y), 5)
buf = Buffer()
TransformListener(buf, node)

deadline = time.time() + 12
est = None
while time.time() < deadline:
    rclpy.spin_once(node, timeout_sec=0.2)
    if "x" in truth:
        try:
            t = buf.lookup_transform("map", "base_link", rclpy.time.Time())
            est = (t.transform.translation.x, t.transform.translation.y)
            break
        except Exception:
            pass

if est is None or "x" not in truth:
    print("DRIFT-CHECK unavailable")
    sys.exit(1)

err = math.hypot(est[0] - truth["x"], est[1] - truth["y"])
print(f"truth=({truth['x']:.2f},{truth['y']:.2f}) "
      f"slam=({est[0]:.2f},{est[1]:.2f}) error={err:.2f}m")
sys.exit(0)
