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

    # Gazebo is started directly rather than through
    # ardupilot_gz_bringup's iris_maze.launch.py, which hardcodes the
    # stock maze world. That world's east wall is 3 m short, and
    # autonomous exploration reliably escapes through the gap; outside
    # the walls a 2D lidar has nothing to scan-match against, so
    # Cartographer diverges and takes the map, the EKF pose and the
    # boundary check down with it. worlds/maze_closed.sdf seals it.
    # The world is still named "maze" so the ros_gz bridge topic
    # templates ({{ world_name }}) resolve unchanged.
    pkg_ros_gz_sim = get_package_share_directory("ros_gz_sim")
    world = os.path.join(pkg_self, "worlds", "maze_closed.sdf")

    gz_server = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_ros_gz_sim, "launch", "gz_sim.launch.py")
        ),
        launch_arguments={"gz_args": f"-v4 -s -r {world}"}.items(),
    )

    gz_gui = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_ros_gz_sim, "launch", "gz_sim.launch.py")
        ),
        launch_arguments={"gz_args": "-v4 -g"}.items(),
        condition=IfCondition(LaunchConfiguration("gui")),
    )

    # Iris + SITL + DDS agent. Gazebo ground-truth TF is disabled:
    # odom->base_link must come from Cartographer alone, both for Nav2
    # and for the ExternalNav feedback to EKF3.
    robot = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                pkg_gz_bringup, "launch", "robots", "iris_lidar.launch.py"
            )
        ),
        launch_arguments={
            "use_gz_tf": "false",
            "world_name": "maze",
            "defaults": defaults,
        }.items(),
    )

    # Cartographer is launched directly (not via ardupilot_cartographer)
    # so it picks up this package's config.
    cartographer = TimerAction(
        period=4.0,
        actions=[
            Node(
                package="frontier_exploration",
                executable="odom_sanitizer",
                output="screen",
                # No sim time: this node only compares stamps carried by
                # the messages themselves. Subscribing to /clock, which
                # Gazebo publishes at 1 kHz, costs ~half a core in rclpy
                # and starves Nav2's control loop.
                parameters=[{"use_sim_time": False}],
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
            # This package's own RViz config, not ardupilot_cartographer's:
            # the stock one knows nothing about frontier markers, so the
            # display had to be added by hand on every launch. This is the
            # same config plus a MarkerArray on
            # /frontier_explorer/frontiers -- cyan frontier cells, orange
            # candidate goals, a green cylinder on the active goal. Seeing
            # what the policy is considering, rather than only where the
            # vehicle went, is most of the debugging.
            #
            # use_sim_time matters here and the stock invocation omitted
            # it. Gazebo's clock starts at zero while the system clock is
            # at the epoch, so an RViz on system time finds every
            # transform hopelessly stale and draws nothing.
            Node(
                package="rviz2",
                executable="rviz2",
                parameters=[{"use_sim_time": True}],
                arguments=[
                    "-d",
                    os.path.join(pkg_self, "rviz", "frontiers.rviz"),
                ],
                condition=IfCondition(LaunchConfiguration("rviz")),
            ),
        ],
    )

    pose_relay = Node(
        package="frontier_exploration",
        executable="pose_relay",
        output="screen",
        # No sim time: transforms are forwarded with their original
        # stamps and the clock is only read for rate limiting. See the
        # odom_sanitizer note on /clock at 1 kHz.
        parameters=[{"use_sim_time": False}],
    )

    # The stock twist_stamper (ardupilot_cartographer) publishes velocity
    # commands on /ap/cmd_vel, but AP_DDS subscribes under /ap/v1.
    # Forwards /ap/cmd_vel into the /ap/v1 namespace this ArduPilot
    # build uses, and fills in the vertical velocity Nav2 never sets.
    # Nav2 is 2D: it leaves linear.z at 0 and nothing else controls
    # height, so the vehicle sagged out of the air repeatedly.
    cmd_vel_relay = Node(
        package="frontier_exploration",
        executable="cmd_vel_altitude_hold",
        output="screen",
        parameters=[
            {
                "use_sim_time": False,
                "target_alt": ParameterValue(
                    LaunchConfiguration("takeoff_alt"), value_type=float
                ),
            }
        ],
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
            gz_server,
            gz_gui,
            robot,
            cartographer,
            navigation,
            pose_relay,
            cmd_vel_relay,
            explorer,
        ]
    )
