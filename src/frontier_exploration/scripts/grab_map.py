#!/usr/bin/env python3
"""Snapshot /map to .npy + metadata, and render a PNG for inspection.

Usage:  python3 grab_map.py OUT_PREFIX

Writes OUT_PREFIX.npy (int8 grid), OUT_PREFIX.json (resolution and
origin) and OUT_PREFIX.png. This is the grabber that produced
test/fixtures -- capture a grid whenever a run fails in a way you had
to be watching to notice, since replaying one costs a second and
reflying the hypothesis costs fifteen minutes.
"""
import json
import sys
import time

import numpy as np
import rclpy
from nav_msgs.msg import OccupancyGrid
from rclpy.node import Node

OUT = sys.argv[1] if len(sys.argv) > 1 else "/tmp/map"

rclpy.init()
node = Node("map_grab", parameter_overrides=[])
got = {}


def on_map(m):
    got["grid"] = np.asarray(m.data, dtype=np.int8).reshape(
        m.info.height, m.info.width)
    got["info"] = dict(
        resolution=m.info.resolution,
        width=m.info.width,
        height=m.info.height,
        origin_x=m.info.origin.position.x,
        origin_y=m.info.origin.position.y,
    )


node.create_subscription(OccupancyGrid, "/map", on_map, 5)
deadline = time.time() + 20
while time.time() < deadline and "grid" not in got:
    rclpy.spin_once(node, timeout_sec=0.2)

if "grid" not in got:
    print("no /map received")
    sys.exit(1)

g, info = got["grid"], got["info"]
np.save(OUT + ".npy", g)
json.dump(info, open(OUT + ".json", "w"), indent=2)

res = info["resolution"]
free = int(((g >= 0) & (g <= 25)).sum())
occ = int((g >= 65).sum())
unk = int((g < 0).sum())
print(f"grid {g.shape} res={res} origin=({info['origin_x']:.2f},{info['origin_y']:.2f})")
print(f"free={free} ({free * res**2:.1f} m2)  occupied={occ}  unknown={unk}")

# Render: unknown grey, free white, occupied black, fog (26-64) light blue.
try:
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt

    img = np.zeros(g.shape + (3,), dtype=np.uint8)
    img[...] = (128, 128, 128)                      # unknown
    img[(g >= 0) & (g <= 25)] = (255, 255, 255)     # free
    img[(g > 25) & (g < 65)] = (150, 190, 230)      # fog / intermediate
    img[g >= 65] = (0, 0, 0)                        # occupied

    ox, oy = info["origin_x"], info["origin_y"]
    extent = [ox, ox + info["width"] * res, oy, oy + info["height"] * res]
    fig, ax = plt.subplots(figsize=(11, 11), dpi=110)
    ax.imshow(img, origin="lower", extent=extent, interpolation="nearest")
    ax.set_xticks(np.arange(-10, 11, 1))
    ax.set_yticks(np.arange(-10, 11, 1))
    ax.grid(color="red", alpha=0.35, linewidth=0.5)
    ax.set_xlabel("x (m, map frame)")
    ax.set_ylabel("y (m, map frame)")
    ax.set_title("Cartographer /map at end of run")
    fig.tight_layout()
    fig.savefig(OUT + ".png")
    print("wrote", OUT + ".png")
except Exception as exc:  # rendering is a convenience, not the point
    print("render skipped:", exc)
