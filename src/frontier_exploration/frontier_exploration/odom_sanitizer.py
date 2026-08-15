"""Drop odometry messages with non-increasing timestamps.

The ros_gz odometry bridge occasionally delivers consecutive messages
with identical stamps, which trips a fatal CHECK inside Cartographer
(map_by_time.h: "data.time > prev"). This node republishes /odometry as
/odometry/filtered with strictly increasing stamps.
"""

import rclpy
from nav_msgs.msg import Odometry
from rclpy.node import Node


class OdomSanitizer(Node):
    def __init__(self):
        super().__init__("odom_sanitizer")
        self._last_stamp = None
        self._dropped = 0
        self._pub = self.create_publisher(Odometry, "/odometry/filtered", 50)
        self._sub = self.create_subscription(
            Odometry, "/odometry", self._on_odom, 50
        )
        self.get_logger().info(
            "Sanitizing /odometry -> /odometry/filtered (strictly increasing stamps)"
        )

    def _on_odom(self, msg: Odometry):
        stamp = (msg.header.stamp.sec, msg.header.stamp.nanosec)
        if self._last_stamp is not None and stamp <= self._last_stamp:
            self._dropped += 1
            if self._dropped % 500 == 1:
                self.get_logger().info(
                    f"Dropped {self._dropped} non-increasing odometry stamps"
                )
            return
        self._last_stamp = stamp
        self._pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = OdomSanitizer()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass


if __name__ == "__main__":
    main()
