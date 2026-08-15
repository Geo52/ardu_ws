"""Relay Cartographer's odom->base_link transform to ArduPilot.

AP_DDS subscribes to /ap/tf (tf2_msgs/TFMessage) and feeds any transform
with frame_id "odom" and child_frame_id "base_link" into AP_VisualOdom,
which EKF3 fuses as ExternalNav. Cartographer publishes that transform on
/tf at ~200 Hz; this node filters and throttles it.
"""

import rclpy
from rclpy.node import Node
from tf2_msgs.msg import TFMessage


class PoseRelay(Node):
    def __init__(self):
        super().__init__("ap_pose_relay")
        self.declare_parameter("parent_frame", "odom")
        self.declare_parameter("child_frame", "base_link")
        # Throttling adds staleness on top of the pipeline delay that
        # EKF3 already has to compensate for, so keep this generous.
        self.declare_parameter("max_rate_hz", 50.0)
        self.declare_parameter("ap_tf_topic", "/ap/v1/tf")

        self._parent = self.get_parameter("parent_frame").value
        self._child = self.get_parameter("child_frame").value
        max_rate = self.get_parameter("max_rate_hz").value
        self._min_period_ns = int(1e9 / max_rate) if max_rate > 0.0 else 0
        self._last_pub_ns = 0
        self._relayed = 0

        ap_tf_topic = self.get_parameter("ap_tf_topic").value
        self._pub = self.create_publisher(TFMessage, ap_tf_topic, 10)
        self._sub = self.create_subscription(TFMessage, "/tf", self._on_tf, 50)
        self.get_logger().info(
            f"Relaying {self._parent}->{self._child} from /tf to {ap_tf_topic}"
        )

    def _on_tf(self, msg: TFMessage):
        for transform in msg.transforms:
            if (
                transform.header.frame_id == self._parent
                and transform.child_frame_id == self._child
            ):
                now_ns = self.get_clock().now().nanoseconds
                if now_ns - self._last_pub_ns < self._min_period_ns:
                    return
                self._last_pub_ns = now_ns
                out = TFMessage()
                out.transforms = [transform]
                self._pub.publish(out)
                self._relayed += 1
                if self._relayed == 1:
                    self.get_logger().info("First pose relayed to ArduPilot")
                return


def main(args=None):
    rclpy.init(args=args)
    node = PoseRelay()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass


if __name__ == "__main__":
    main()
