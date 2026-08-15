"""Bringup for frontier-based autonomous exploration in the maze world.

Starts, in order:
  * Gazebo maze world + Iris quadcopter + ArduPilot SITL + DDS agent
    (ardupilot_gz_bringup iris_maze.launch.py), with EKF3 configured for
    ExternalNav and GPS removed,
  * Cartographer SLAM,
  * Nav2 (with the twist stamper feeding /ap/cmd_vel),
  * the Cartographer -> ArduPilot pose relay,
  * the frontier explorer.

Usage:
    ros2 launch frontier_exploration explore.launch.py
    ros2 launch frontier_exploration explore.launch.py rviz:=false gui:=false
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, TimerAction
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    pkg_sitl = get_package_share_directory("ardupilot_sitl")
    pkg_gz_bringup = get_package_share_directory("ardupilot_gz_bringup")
    pkg_cartographer = get_package_share_directory("ardupilot_cartographer")
    pkg_self = get_package_share_directory("frontier_exploration")

    sitl_params = os.path.join(pkg_sitl, "config", "default_params")
    defaults = ",".join(
        [
            os.path.join(sitl_params, "copter.parm"),
            os.path.join(sitl_params, "gazebo-iris.parm"),
            os.path.join(sitl_params, "dds_udp.parm"),
            os.path.join(sitl_params, "dds_use_ns.parm"),
            os.path.join(pkg_self, "config", "frontier_ekf3.parm"),
        ]
    )

    # Gazebo + SITL + DDS agent. Gazebo ground-truth TF is disabled:
    # odom->base_link must come from Cartographer alone, both for Nav2
    # and for the ExternalNav feedback to EKF3.
    sim = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_gz_bringup, "launch", "iris_maze.launch.py")
        ),
        launch_arguments={
            "rviz": "false",
            "use_gz_tf": "false",
            "use_gz_sim_gui": LaunchConfiguration("gui"),
            "defaults": defaults,
        }.items(),
    )

    # Cartographer is launched directly (not via ardupilot_cartographer)
    # so it picks up this package's config: scan-matching only, no
    # ground-truth odometry prior from the simulator.
    cartographer = TimerAction(
        period=4.0,
        actions=[
            Node(
                package="frontier_exploration",
                executable="odom_sanitizer",
                output="screen",
                parameters=[{"use_sim_time": True}],
            ),
            Node(
                package="cartographer_ros",
                executable="cartographer_node",
                output="screen",
                parameters=[{"use_sim_time": True}],
                arguments=[
                    "-configuration_directory",
                    os.path.join(pkg_self, "config"),
                    "-configuration_basename",
                    "cartographer.lua",
                ],
                remappings=[("odom", "/odometry/filtered")],
            ),
            Node(
                package="cartographer_ros",
                executable="cartographer_occupancy_grid_node",
                parameters=[{"use_sim_time": True}, {"resolution": 0.05}],
            ),
        ],
    )

    # Nav2 with this package's params (larger inflation radius and a
    # modest speed cap keep the copter clear of the maze walls), plus
    # the twist stamper feeding ArduPilot velocity control.
    pkg_nav2_bringup = get_package_share_directory("nav2_bringup")
    navigation = TimerAction(
        period=8.0,
        actions=[
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    os.path.join(pkg_nav2_bringup, "launch", "navigation_launch.py")
                ),
                launch_arguments={
                    "use_sim_time": "true",
                    "params_file": os.path.join(
                        pkg_self, "config", "navigation.yaml"
                    ),
                }.items(),
            ),
            Node(
                package="twist_stamper",
                executable="twist_stamper",
                parameters=[{"use_sim_time": True, "frame_id": "base_link"}],
                remappings=[
                    ("cmd_vel_in", "cmd_vel"),
                    ("cmd_vel_out", "ap/cmd_vel"),
                ],
            ),
            Node(
                package="rviz2",
                executable="rviz2",
                arguments=[
                    "-d",
                    os.path.join(pkg_cartographer, "rviz", "navigation.rviz"),
                ],
                condition=IfCondition(LaunchConfiguration("rviz")),
            ),
        ],
    )

    pose_relay = Node(
        package="frontier_exploration",
        executable="pose_relay",
        output="screen",
        parameters=[{"use_sim_time": True}],
    )

    # The stock twist_stamper (ardupilot_cartographer) publishes velocity
    # commands on /ap/cmd_vel, but AP_DDS subscribes under /ap/v1.
    cmd_vel_relay = Node(
        package="topic_tools",
        executable="relay",
        arguments=["/ap/cmd_vel", "/ap/v1/cmd_vel"],
        output="screen",
        parameters=[{"use_sim_time": True}],
    )

    explorer = TimerAction(
        period=10.0,
        actions=[
            Node(
                package="frontier_exploration",
                executable="explorer",
                output="screen",
                parameters=[
                    {
                        "use_sim_time": True,
                        "takeoff_alt": ParameterValue(
                            LaunchConfiguration("takeoff_alt"), value_type=float
                        ),
                    }
                ],
            )
        ],
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "rviz", default_value="true", description="Open RViz (Nav2 view)."
            ),
            DeclareLaunchArgument(
                "gui", default_value="true", description="Run the Gazebo GUI."
            ),
            DeclareLaunchArgument(
                "takeoff_alt",
                default_value="2.0",
                description="Exploration altitude in meters.",
            ),
            sim,
            cartographer,
            navigation,
            pose_relay,
            cmd_vel_relay,
            explorer,
        ]
    )
