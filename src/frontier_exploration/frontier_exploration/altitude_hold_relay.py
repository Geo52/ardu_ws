"""Forward Nav2 velocity commands to ArduPilot, adding altitude hold.

Nav2 is a 2D navigation stack: it commands x, y and yaw and leaves the
vertical channel at zero. Nothing else in the pipeline controls height
either — ArduPilot's GUIDED velocity mode is simply asked to hold, and
in practice the vehicle sags, losing more than a metre over a few
minutes of manoeuvring. Once it drops far enough the explorer mistakes
it for a crash, and re-issuing a takeoff to an already-airborne copter
does not climb like a fresh one, so the mission can hang outright.

This closes that loop: a proportional controller on altitude error sets
the vertical velocity of each command as it passes through. It also
does the /ap/cmd_vel -> /ap/v1/cmd_vel forwarding this ArduPilot build's
namespace requires, replacing a plain topic relay.
"""

import rclpy
from geometry_msgs.msg import PoseStamped, TwistStamped
from rclpy.node import Node
from rclpy.qos import (
    QoSDurabilityPolicy,
    QoSHistoryPolicy,
    QoSProfile,
    QoSReliabilityPolicy,
)

BEST_EFFORT = QoSProfile(
    reliability=QoSReliabilityPolicy.BEST_EFFORT,
    durability=QoSDurabilityPolicy.VOLATILE,
    history=QoSHistoryPolicy.KEEP_LAST,
    depth=10,
)


class AltitudeHoldRelay(Node):
    def __init__(self):
        super().__init__("cmd_vel_altitude_hold")
        self.declare_parameter("target_alt", 2.0)
        self.declare_parameter("kp", 0.9)
        self.declare_parameter("max_climb", 0.6)
        self.declare_parameter("deadband", 0.05)
        self.declare_parameter("input_topic", "/ap/cmd_vel")
        self.declare_parameter("output_topic", "/ap/v1/cmd_vel")
        self.declare_parameter("pose_topic", "/ap/v1/pose/filtered")

        self._target = self.get_parameter("target_alt").value
        self._kp = self.get_parameter("kp").value
        self._max_climb = self.get_parameter("max_climb").value
        self._deadband = self.get_parameter("deadband").value
        self._alt = None
        self._warned = False

        self.create_subscription(
            PoseStamped,
            self.get_parameter("pose_topic").value,
            self._on_pose,
            BEST_EFFORT,
        )
        self._pub = self.create_publisher(
            TwistStamped, self.get_parameter("output_topic").value, 10
        )
        self.create_subscription(
            TwistStamped,
            self.get_parameter("input_topic").value,
            self._on_cmd,
            10,
        )
        self.get_logger().info(
            f"Holding {self._target} m on forwarded velocity commands"
        )

    def _on_pose(self, msg: PoseStamped):
        self._alt = msg.pose.position.z

    def _on_cmd(self, msg: TwistStamped):
        if self._alt is not None:
            error = self._target - self._alt
            if abs(error) < self._deadband:
                vz = 0.0
            else:
                vz = max(-self._max_climb, min(self._max_climb, self._kp * error))
            msg.twist.linear.z = vz
            if not self._warned and abs(error) > 0.5:
                self.get_logger().info(
                    f"Correcting altitude: {self._alt:.2f} m -> {self._target} m"
                )
                self._warned = True
        self._pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = AltitudeHoldRelay()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass


if __name__ == "__main__":
    main()
