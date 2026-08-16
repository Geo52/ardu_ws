import os
from glob import glob

from setuptools import setup

package_name = "frontier_exploration"

setup(
    name=package_name,
    version="0.1.0",
    packages=[package_name],
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        (os.path.join("share", package_name, "launch"), glob("launch/*.launch.py")),
        (os.path.join("share", package_name, "config"), glob("config/*")),
        (os.path.join("share", package_name, "worlds"), glob("worlds/*.sdf")),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="Jorge Rios",
    maintainer_email="jorgeluisrios47@gmail.com",
    description="Frontier-based autonomous exploration for a GPS-denied UAV.",
    license="MIT",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            "explorer = frontier_exploration.explorer_node:main",
            "pose_relay = frontier_exploration.pose_relay:main",
            "odom_sanitizer = frontier_exploration.odom_sanitizer:main",
            "cmd_vel_altitude_hold = frontier_exploration.altitude_hold_relay:main",
        ],
    },
)
