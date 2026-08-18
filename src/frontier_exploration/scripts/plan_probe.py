#!/usr/bin/env python3
"""What is the live /plan actually crossing?

"It's routing through walls" is the most misleading symptom in this
project. It was blamed on the lethal threshold (integration note 25)
and on the inert obstacle layer (note 26), and was neither: sampled
against the map, *no* pose crossed a known wall, while 30-41% crossed
unmapped space that the costmap was reporting as free because
`track_unknown_space` defaulted to false (note 28). The paths were
straight lines drawn confidently across regions never seen, and where
a wall happened to stand in one, it crossed.

That question is answerable in about forty seconds and it cost four
runs to ask. Run this before changing any threshold in response to a
path that looks wrong.

Reads one /plan and classifies every pose against the current /map:

    plan: 1306 poses
      KNOWN WALL (>=90)          0     0.0%
      UNKNOWN (-1)             392    30.0%
      free (0-25)              716    54.8%

Non-zero KNOWN WALL means the planner really is crossing geometry it
can see, and the costmap thresholds are worth looking at. A large
UNKNOWN share means it is planning through fog instead, which is a
different bug with a different fix. Late in a run a rising UNKNOWN
share is expected and correct -- the frontiers that remain are in
barely-mapped ground, so unknown is the only way to reach them.

Usage:  python3 plan_probe.py [timeout_s]
"""
import sys
import time

import numpy as np
import rclpy
from nav_msgs.msg import OccupancyGrid, Path
from rclpy.node import Node
from rclpy.qos import (
    QoSDurabilityPolicy,
    QoSHistoryPolicy,
    QoSProfile,
    QoSReliabilityPolicy,
)

# /map is latched: transient-local, or the subscriber gets nothing until
# the next 1 Hz publish.
MAP_QOS = QoSProfile(
    reliability=QoSReliabilityPolicy.RELIABLE,
    durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
    history=QoSHistoryPolicy.KEEP_LAST,
    depth=1,
)

# Matches global_costmap's occupied_min: see integration note 27 for why
# no threshold cleanly separates fog from wall, and why 90 is where this
# one sits anyway.
WALL_MIN = 90


class PlanProbe(Node):
    def __init__(self):
        super().__init__("plan_probe")
        self.grid = None
        self.report = None
        self.create_subscription(OccupancyGrid, "/map", self._on_map, MAP_QOS)
        self.create_subscription(Path, "/plan", self._on_plan, 10)

    def _on_map(self, msg):
        self.grid = msg

    def _on_plan(self, msg):
        if self.grid is None or self.report is not None or not msg.poses:
            return
        info = self.grid.info
        data = np.asarray(self.grid.data, dtype=np.int16).reshape(
            info.height, info.width
        )
        buckets = {
            f"KNOWN WALL (>={WALL_MIN})": 0,
            "fog (50-89)": 0,
            "fog (26-49)": 0,
            "UNKNOWN (-1)": 0,
            "free (0-25)": 0,
            "off map": 0,
        }
        for ps in msg.poses:
            col = int((ps.pose.position.x - info.origin.position.x) / info.resolution)
            row = int((ps.pose.position.y - info.origin.position.y) / info.resolution)
            if not (0 <= row < info.height and 0 <= col < info.width):
                buckets["off map"] += 1
                continue
            v = data[row, col]
            if v < 0:
                buckets["UNKNOWN (-1)"] += 1
            elif v >= WALL_MIN:
                buckets[f"KNOWN WALL (>={WALL_MIN})"] += 1
            elif v >= 50:
                buckets["fog (50-89)"] += 1
            elif v >= 26:
                buckets["fog (26-49)"] += 1
            else:
                buckets["free (0-25)"] += 1
        self.report = (len(msg.poses), info, buckets)


def main():
    timeout = float(sys.argv[1]) if len(sys.argv) > 1 else 40.0
    rclpy.init()
    node = PlanProbe()
    start = time.time()
    while rclpy.ok() and node.report is None and time.time() - start < timeout:
        rclpy.spin_once(node, timeout_sec=0.5)

    if node.report is None:
        # Which one is missing localises the problem: no /map means
        # Cartographer, no /plan means Nav2 has no active goal.
        print("no /map received" if node.grid is None else "no /plan received")
        rc = 1
    else:
        n, info, buckets = node.report
        print(f"plan: {n} poses   (map {info.width}x{info.height} @ {info.resolution:.3f} m)")
        for name, count in buckets.items():
            print(f"  {name:22s} {count:5d}   {100.0 * count / n:5.1f}%")
        rc = 0
    node.destroy_node()
    rclpy.shutdown()
    return rc


if __name__ == "__main__":
    sys.exit(main())
