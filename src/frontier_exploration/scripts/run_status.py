#!/usr/bin/env python3
"""One-shot run status: mapped area, SLAM vs ground-truth error, altitude.

Prints a single line and exits, so it suits a polling loop as well as a
one-off check. A superset of drift_check.py, which reports the drift
alone.

Usage:  python3 run_status.py [name=x0,x1,y0,y1 ...]

Each optional argument adds a region whose free-space percentage is
reported separately, which is how you tell "the map is 90% done" from
"the map is 90% done and the corridor I care about is untouched". The
defaults are the two regions runs 70-75 kept missing.
"""
import math
import sys
import time

import numpy as np
import rclpy
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Odometry, OccupancyGrid
from rclpy.node import Node
from rclpy.qos import (
    QoSDurabilityPolicy,
    QoSHistoryPolicy,
    QoSProfile,
    QoSReliabilityPolicy,
)
from tf2_ros import Buffer, TransformListener

BEST_EFFORT = QoSProfile(
    reliability=QoSReliabilityPolicy.BEST_EFFORT,
    durability=QoSDurabilityPolicy.VOLATILE,
    history=QoSHistoryPolicy.KEEP_LAST,
    depth=5,
)

rclpy.init()
node = Node("run_status", parameter_overrides=[])
state = {}

node.create_subscription(
    Odometry, "/odometry",
    lambda m: state.update(tx=m.pose.pose.position.x, ty=m.pose.pose.position.y,
                           tz=m.pose.pose.position.z), 5)
node.create_subscription(
    PoseStamped, "/ap/v1/pose/filtered",
    lambda m: state.update(alt=m.pose.position.z), BEST_EFFORT)


# The two regions run 70 left unmapped, in world coords (x0,x1,y0,y1).
REGIONS = {
    "west": (-10.4, -7.3, -7.0, 1.5),
    "SEcorr": (-1.2, 9.8, -3.4, -0.9),
}
for _arg in sys.argv[1:]:
    if "=" in _arg:
        _name, _box = _arg.split("=", 1)
        REGIONS[_name] = tuple(float(v) for v in _box.split(","))


def on_map(m):
    g = np.asarray(m.data, dtype=np.int16)
    free = int(((g >= 0) & (g <= 25)).sum())
    state["area"] = free * m.info.resolution ** 2
    state["unknown"] = int((g < 0).sum())

    res = m.info.resolution
    ox, oy = m.info.origin.position.x, m.info.origin.position.y
    grid = g.reshape(m.info.height, m.info.width)
    covered = {}
    for name, (x0, x1, y0, y1) in REGIONS.items():
        c0, c1 = int((x0 - ox) / res), int((x1 - ox) / res)
        r0, r1 = int((y0 - oy) / res), int((y1 - oy) / res)
        sub = grid[max(r0, 0):max(r1, 0), max(c0, 0):max(c1, 0)]
        if sub.size:
            covered[name] = 100.0 * ((sub >= 0) & (sub <= 25)).sum() / sub.size
    state["regions"] = covered


node.create_subscription(OccupancyGrid, "/map", on_map, 5)

buf = Buffer()
TransformListener(buf, node)

deadline = time.time() + 12
while time.time() < deadline:
    rclpy.spin_once(node, timeout_sec=0.2)
    if "slam" not in state:
        try:
            t = buf.lookup_transform("map", "base_link", rclpy.time.Time())
            state["slam"] = (t.transform.translation.x, t.transform.translation.y)
        except Exception:
            pass
    if {"tx", "area", "slam"} <= state.keys():
        break

bits = []
if "area" in state:
    bits.append(f"mapped={state['area']:.1f}m2")
if state.get("regions"):
    bits.append(
        "[" + " ".join(f"{k}={v:.0f}%free" for k, v in state["regions"].items()) + "]"
    )
if "alt" in state:
    bits.append(f"alt={state['alt']:.2f}m")
if "slam" in state and "tx" in state:
    err = math.hypot(state["slam"][0] - state["tx"], state["slam"][1] - state["ty"])
    bits.append(
        f"truth=({state['tx']:.1f},{state['ty']:.1f}) "
        f"slam=({state['slam'][0]:.1f},{state['slam'][1]:.1f}) drift={err:.2f}m"
    )
elif "tx" in state:
    bits.append(f"truth=({state['tx']:.1f},{state['ty']:.1f}) slam=UNAVAILABLE")

print(" ".join(bits) if bits else "no data")
sys.exit(0 if bits else 1)
