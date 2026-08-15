"""Frontier-based autonomous exploration for a GPS-denied UAV.

Flight sequence (all via the ArduPilot DDS interface):

  1. Set the EKF origin over MAVLink (no GPS, so EKF3 has no global
     reference until one is provided).
  2. Switch to GUIDED, arm, take off to the exploration altitude.
  3. Explore: detect frontiers (free/unknown boundaries) on the
     Cartographer occupancy grid, cluster them, and dispatch the nearest
     cluster's centroid goal to Nav2 (NavigateToPose). The frontier set
     is re-evaluated as the map updates; a goal whose frontier has been
     mapped away is preempted, and unreachable goals are blacklisted.
  4. When no reachable frontiers remain, switch to LAND and wait for
     disarm.
"""

import math
import threading
from enum import Enum, auto

import numpy as np
import rclpy
from action_msgs.msg import GoalStatus
from ardupilot_msgs.msg import Status
from ardupilot_msgs.srv import ArmMotors, ModeSwitch, Takeoff
from geometry_msgs.msg import PoseStamped
from nav2_msgs.action import NavigateToPose
from nav_msgs.msg import OccupancyGrid
from rclpy.action import ActionClient
from rclpy.duration import Duration
from rclpy.node import Node
from rclpy.qos import (
    QoSDurabilityPolicy,
    QoSHistoryPolicy,
    QoSProfile,
    QoSReliabilityPolicy,
)
from tf2_ros import Buffer, TransformListener
from visualization_msgs.msg import Marker, MarkerArray

from frontier_exploration.frontier_search import cell_to_world, find_frontiers

COPTER_MODE_GUIDED = 4
COPTER_MODE_LAND = 9

BEST_EFFORT = QoSProfile(
    reliability=QoSReliabilityPolicy.BEST_EFFORT,
    durability=QoSDurabilityPolicy.VOLATILE,
    history=QoSHistoryPolicy.KEEP_LAST,
    depth=10,
)


class State(Enum):
    WAIT_INTERFACES = auto()
    SET_MODE = auto()
    ARM = auto()
    TAKEOFF = auto()
    CLIMB = auto()
    EXPLORE = auto()
    LAND = auto()
    DONE = auto()


class Explorer(Node):
    def __init__(self):
        super().__init__("frontier_explorer")

        self.declare_parameter("takeoff_alt", 2.0)
        self.declare_parameter("free_max", 25)
        self.declare_parameter("min_frontier_size", 10)
        # Cartographer grids have a ~2-cell intermediate-probability rim
        # between free and unknown space; see frontier_search. Must stay
        # below the thinnest wall thickness in cells (4 in the maze).
        self.declare_parameter("unknown_dilation", 3)
        # Exploration boundary (map frame). The maze world has an
        # entrance in its outer wall; without a boundary the vehicle
        # happily explores the unbounded outside world forever.
        self.declare_parameter("bound_min_x", -11.0)
        self.declare_parameter("bound_max_x", 11.0)
        self.declare_parameter("bound_min_y", -11.0)
        self.declare_parameter("bound_max_y", 11.0)
        self.declare_parameter("eval_period", 2.0)
        self.declare_parameter("blacklist_radius", 0.8)
        self.declare_parameter("goal_timeout", 90.0)
        # A goal is preempted when no frontier cell remains within this
        # distance of it (the area has been mapped while in transit).
        self.declare_parameter("goal_invalidate_dist", 1.0)
        self.declare_parameter("empty_evals_before_land", 5)
        # MAVLink endpoint used once at startup to set the EKF origin.
        self.declare_parameter("mavlink_url", "udp:127.0.0.1:14550")
        self.declare_parameter("origin_lat", -35.363262)
        self.declare_parameter("origin_lon", 149.165237)
        self.declare_parameter("origin_alt", 584.0)

        self._takeoff_alt = self.get_parameter("takeoff_alt").value
        self._free_max = self.get_parameter("free_max").value
        self._min_frontier_size = self.get_parameter("min_frontier_size").value
        self._unknown_dilation = self.get_parameter("unknown_dilation").value
        self._bounds = (
            self.get_parameter("bound_min_x").value,
            self.get_parameter("bound_max_x").value,
            self.get_parameter("bound_min_y").value,
            self.get_parameter("bound_max_y").value,
        )
        self._blacklist_radius = self.get_parameter("blacklist_radius").value
        self._goal_timeout = self.get_parameter("goal_timeout").value
        self._goal_invalidate_dist = self.get_parameter("goal_invalidate_dist").value
        self._empty_evals_before_land = self.get_parameter(
            "empty_evals_before_land"
        ).value

        # Interfaces to ArduPilot (AP_DDS exposes them under /ap/v1).
        self.declare_parameter("ap_ns", "/ap/v1")
        ap_ns = self.get_parameter("ap_ns").value
        self._arm_client = self.create_client(ArmMotors, f"{ap_ns}/arm_motors")
        self._mode_client = self.create_client(ModeSwitch, f"{ap_ns}/mode_switch")
        self._takeoff_client = self.create_client(
            Takeoff, f"{ap_ns}/experimental/takeoff"
        )
        self.create_subscription(
            PoseStamped, f"{ap_ns}/pose/filtered", self._on_ap_pose, BEST_EFFORT
        )
        self.create_subscription(
            Status, f"{ap_ns}/status", self._on_status, BEST_EFFORT
        )

        # Map and navigation.
        self.create_subscription(OccupancyGrid, "/map", self._on_map, 5)
        self._nav_client = ActionClient(self, NavigateToPose, "navigate_to_pose")
        self._tf_buffer = Buffer()
        self._tf_listener = TransformListener(self._tf_buffer, self)

        self._marker_pub = self.create_publisher(MarkerArray, "~/frontiers", 5)

        # State.
        self._state = State.WAIT_INTERFACES
        self._map = None
        self._ap_alt = None
        self._status = None
        self._pending_srv = None
        self._origin_set = threading.Event()
        self._blacklist = []
        self._blacklist_clears_left = 2
        self._goal_handle = None
        self._result_future = None
        self._current_goal_xy = None
        self._goal_sent_time = None
        self._goal_seq = 0
        self._reached_goal_xy = None
        self._continue_from_xy = None
        # How far a successor frontier may be from an invalidated goal
        # and still count as "the same direction".
        self._continue_radius = 3.0
        self._goals_succeeded = 0
        self._goals_failed = 0
        self._empty_evals = 0
        self._land_requested = False
        self._retry_at = None

        threading.Thread(target=self._set_ekf_origin, daemon=True).start()

        self._tick_timer = self.create_timer(1.0, self._tick)
        self._eval_timer = self.create_timer(
            self.get_parameter("eval_period").value, self._evaluate
        )

        self.get_logger().info("Frontier explorer started")

    # ------------------------------------------------------------------
    # EKF origin (MAVLink, once at startup)
    # ------------------------------------------------------------------
    def _set_ekf_origin(self):
        """Send SET_GPS_GLOBAL_ORIGIN until ArduPilot confirms it.

        With GPS disabled EKF3 has no global origin, and it will not
        accept ExternalNav data (nor allow arming) until one is set.
        """
        from pymavlink import mavutil

        url = self.get_parameter("mavlink_url").value
        lat = int(self.get_parameter("origin_lat").value * 1e7)
        lon = int(self.get_parameter("origin_lon").value * 1e7)
        alt = int(self.get_parameter("origin_alt").value * 1e3)

        try:
            conn = mavutil.mavlink_connection(url)
            self.get_logger().info(f"MAVLink: waiting for heartbeat on {url}")
            conn.wait_heartbeat(timeout=120)
            # If the origin is already set (e.g. explorer restart mid
            # flight), accept it instead of waiting for a confirmation
            # that ArduPilot only broadcasts on the first set.
            conn.mav.command_long_send(
                conn.target_system, conn.target_component,
                mavutil.mavlink.MAV_CMD_REQUEST_MESSAGE, 0,
                mavutil.mavlink.MAVLINK_MSG_ID_GPS_GLOBAL_ORIGIN,
                0, 0, 0, 0, 0, 0,
            )
            msg = conn.recv_match(type="GPS_GLOBAL_ORIGIN", blocking=True, timeout=3)
            if msg is not None:
                self.get_logger().info("EKF origin already set")
                self._origin_set.set()
                conn.close()
                return
            for attempt in range(30):
                conn.mav.set_gps_global_origin_send(conn.target_system, lat, lon, alt)
                msg = conn.recv_match(
                    type="GPS_GLOBAL_ORIGIN", blocking=True, timeout=2
                )
                if msg is not None:
                    self.get_logger().info(
                        f"EKF origin set: {msg.latitude / 1e7:.6f}, "
                        f"{msg.longitude / 1e7:.6f}"
                    )
                    # Home must also be set explicitly: with no GPS there
                    # is no fix to set it from, and Copter refuses to arm
                    # ("AHRS: waiting for home") without one.
                    for _ in range(10):
                        conn.mav.command_long_send(
                            conn.target_system,
                            conn.target_component,
                            mavutil.mavlink.MAV_CMD_DO_SET_HOME,
                            0,
                            0,  # use specified location, not current
                            0, 0, 0,
                            lat / 1e7,
                            lon / 1e7,
                            alt / 1e3,
                        )
                        ack = conn.recv_match(
                            type="COMMAND_ACK", blocking=True, timeout=2
                        )
                        if (
                            ack is not None
                            and ack.command == mavutil.mavlink.MAV_CMD_DO_SET_HOME
                            and ack.result == 0
                        ):
                            self.get_logger().info("Home position set")
                            break
                    else:
                        self.get_logger().warning(
                            "Home position not acknowledged; arming may fail"
                        )
                    self._origin_set.set()
                    conn.close()
                    return
            self.get_logger().error("EKF origin: no confirmation received")
        except Exception as exc:  # noqa: BLE001
            self.get_logger().error(f"EKF origin setup failed: {exc}")

    # ------------------------------------------------------------------
    # Subscriptions
    # ------------------------------------------------------------------
    def _on_map(self, msg: OccupancyGrid):
        self._map = msg

    def _on_ap_pose(self, msg: PoseStamped):
        self._ap_alt = msg.pose.position.z

    def _on_status(self, msg: Status):
        self._status = msg

    # ------------------------------------------------------------------
    # Flight state machine (1 Hz)
    # ------------------------------------------------------------------
    def _tick(self):
        if self._retry_at is not None:
            if self.get_clock().now() < self._retry_at:
                return
            self._retry_at = None

        # If the vehicle disarmed unexpectedly (e.g. auto-disarm after a
        # takeoff that could not proceed), restart the flight sequence.
        if (
            self._state in (State.CLIMB, State.EXPLORE)
            and self._status is not None
            and not self._status.armed
        ):
            self.get_logger().warning("Vehicle disarmed unexpectedly, re-arming")
            self._cancel_current_goal()
            self._clear_goal()
            self._pending_srv = None
            self._state = State.SET_MODE
            return

        if self._state == State.WAIT_INTERFACES:
            ready = (
                self._arm_client.service_is_ready()
                and self._mode_client.service_is_ready()
                and self._takeoff_client.service_is_ready()
                and self._nav_client.server_is_ready()
                and self._map is not None
                and self._origin_set.is_set()
            )
            if ready:
                self.get_logger().info("All interfaces ready")
                self._state = State.SET_MODE
            return

        if self._state == State.SET_MODE:
            self._call_and_advance(
                self._mode_client,
                ModeSwitch.Request(mode=COPTER_MODE_GUIDED),
                lambda r: r.status or r.curr_mode == COPTER_MODE_GUIDED,
                State.ARM,
                "set mode GUIDED",
            )
            return

        if self._state == State.ARM:
            self._call_and_advance(
                self._arm_client,
                ArmMotors.Request(arm=True),
                lambda r: r.result,
                State.TAKEOFF,
                "arm motors",
                retry_delay=5.0,
            )
            return

        if self._state == State.TAKEOFF:
            # Already at altitude (e.g. explorer restart mid-flight):
            # commanding another takeoff would climb higher instead.
            if self._ap_alt is not None and self._ap_alt >= 0.9 * self._takeoff_alt:
                self.get_logger().info("Already at altitude, skipping takeoff")
                self._state = State.EXPLORE
                return
            self._call_and_advance(
                self._takeoff_client,
                Takeoff.Request(alt=float(self._takeoff_alt)),
                lambda r: r.status,
                State.CLIMB,
                f"takeoff to {self._takeoff_alt} m",
            )
            return

        if self._state == State.CLIMB:
            if self._ap_alt is not None and self._ap_alt >= 0.9 * self._takeoff_alt:
                self.get_logger().info(
                    f"Reached {self._ap_alt:.2f} m, starting exploration"
                )
                self._state = State.EXPLORE
            return

        if self._state == State.LAND:
            if not self._land_requested:
                self._cancel_current_goal()
                self._call_and_advance(
                    self._mode_client,
                    ModeSwitch.Request(mode=COPTER_MODE_LAND),
                    lambda r: r.status or r.curr_mode == COPTER_MODE_LAND,
                    State.LAND,
                    "set mode LAND",
                )
                self._land_requested = True
                return
            if self._status is not None and not self._status.armed:
                self.get_logger().info(
                    "Exploration complete: landed and disarmed "
                    f"({self._goals_succeeded} goals reached, "
                    f"{self._goals_failed} blacklisted)"
                )
                self._state = State.DONE
            return

    def _call_and_advance(
        self, client, request, ok, next_state, label, retry_delay=2.0
    ):
        """Drive an async service call from the timer without blocking."""
        if self._pending_srv is None:
            self.get_logger().info(f"Requesting: {label}")
            self._pending_srv = client.call_async(request)
            return
        if not self._pending_srv.done():
            return
        try:
            result = self._pending_srv.result()
            success = ok(result)
        except Exception as exc:  # noqa: BLE001
            self.get_logger().warning(f"{label} failed: {exc}")
            success = False
        self._pending_srv = None
        if success:
            self.get_logger().info(f"OK: {label}")
            self._state = next_state
        else:
            self.get_logger().warning(f"{label} rejected, retrying")
            self._retry_at = self.get_clock().now() + Duration(
                seconds=retry_delay
            )

    # ------------------------------------------------------------------
    # Exploration
    # ------------------------------------------------------------------
    def _evaluate(self):
        if self._state != State.EXPLORE or self._map is None:
            return

        grid_msg = self._map
        info = grid_msg.info
        grid = np.array(grid_msg.data, dtype=np.int8).reshape(
            info.height, info.width
        )
        frontiers = find_frontiers(
            grid,
            free_max=self._free_max,
            min_size=self._min_frontier_size,
            unknown_dilation=self._unknown_dilation,
            require_line_of_sight=True,
        )

        # World-frame goal for each cluster, minus blacklisted ones and
        # anything outside the exploration boundary.
        min_x, max_x, min_y, max_y = self._bounds
        candidates = []
        for f in frontiers:
            x, y = cell_to_world(
                f.goal_cell, info.origin.position.x, info.origin.position.y,
                info.resolution,
            )
            if not (min_x <= x <= max_x and min_y <= y <= max_y):
                continue
            if any(
                math.hypot(x - bx, y - by) < self._blacklist_radius
                for bx, by in self._blacklist
            ):
                continue
            candidates.append((x, y, f))

        self._publish_markers(candidates, info)

        # A frontier that survived our arrival cannot be mapped from
        # here: blacklist it so the policy moves on.
        if self._reached_goal_xy is not None:
            rx, ry = self._reached_goal_xy
            if self._frontier_near(frontiers, info, rx, ry):
                self.get_logger().info(
                    "Frontier persists after arrival, blacklisting"
                )
                self._blacklist.append(self._reached_goal_xy)
            self._reached_goal_xy = None

        # Check the in-flight goal.
        if self._current_goal_xy is not None:
            self._check_active_goal(frontiers, info)

        if self._current_goal_xy is not None:
            return  # still navigating

        if not candidates:
            self._empty_evals += 1
            if self._empty_evals >= self._empty_evals_before_land:
                # Give blacklisted frontiers another chance before
                # declaring the map complete: a goal may have failed
                # transiently while the map was still young.
                if self._blacklist and self._blacklist_clears_left > 0:
                    self._blacklist_clears_left -= 1
                    self._blacklist.clear()
                    self._empty_evals = 0
                    self.get_logger().info(
                        "Only blacklisted frontiers remain; clearing "
                        "blacklist for a retry"
                    )
                    return
                self.get_logger().info("No reachable frontiers remain: landing")
                self._state = State.LAND
            return
        self._empty_evals = 0

        pose = self._robot_xy()
        if pose is None:
            return
        rx, ry = pose

        # Momentum: continue toward the cluster nearest the goal that
        # was just invalidated, if one is close enough to it.
        if self._continue_from_xy is not None:
            cx, cy = self._continue_from_xy
            self._continue_from_xy = None
            near_old = min(
                candidates,
                key=lambda c: math.hypot(c[0] - cx, c[1] - cy),
            )
            if math.hypot(near_old[0] - cx, near_old[1] - cy) < self._continue_radius:
                self._send_goal(near_old[0], near_old[1], rx, ry)
                return

        candidates.sort(key=lambda c: math.hypot(c[0] - rx, c[1] - ry))
        x, y, f = candidates[0]
        self._send_goal(x, y, rx, ry)

    def _robot_xy(self):
        try:
            t = self._tf_buffer.lookup_transform(
                "map", "base_link", rclpy.time.Time()
            )
            return t.transform.translation.x, t.transform.translation.y
        except Exception:  # noqa: BLE001
            self.get_logger().warning("map->base_link transform unavailable")
            return None

    def _send_goal(self, x, y, rx, ry):
        yaw = math.atan2(y - ry, x - rx)
        goal = NavigateToPose.Goal()
        goal.pose.header.frame_id = "map"
        goal.pose.header.stamp = self.get_clock().now().to_msg()
        goal.pose.pose.position.x = x
        goal.pose.pose.position.y = y
        goal.pose.pose.orientation.z = math.sin(yaw / 2.0)
        goal.pose.pose.orientation.w = math.cos(yaw / 2.0)

        self._goal_seq += 1
        seq = self._goal_seq
        self._current_goal_xy = (x, y)
        self._goal_sent_time = self.get_clock().now()
        self.get_logger().info(f"New frontier goal #{seq}: ({x:.2f}, {y:.2f})")

        send_future = self._nav_client.send_goal_async(goal)
        send_future.add_done_callback(
            lambda future: self._on_goal_response(future, seq)
        )

    def _on_goal_response(self, future, seq):
        handle = future.result()
        if seq != self._goal_seq:
            return  # response for a goal we have already moved past
        if not handle.accepted:
            self.get_logger().warning("Nav2 rejected goal, blacklisting")
            self._fail_current_goal()
            return
        self._goal_handle = handle
        self._result_future = handle.get_result_async()
        self._result_future.add_done_callback(
            lambda future: self._on_goal_result(future, seq)
        )

    def _on_goal_result(self, future, seq):
        try:
            status = future.result().status
        except Exception:  # noqa: BLE001
            status = GoalStatus.STATUS_UNKNOWN
        if seq != self._goal_seq or self._current_goal_xy is None:
            return  # stale result from a preempted/superseded goal
        if status == GoalStatus.STATUS_SUCCEEDED:
            self._goals_succeeded += 1
            self.get_logger().info("Frontier reached")
            # Remember where we arrived: if the frontier there survives
            # the next map evaluation, being at it clearly does not map
            # it, and re-sending goals to it would loop forever.
            self._reached_goal_xy = self._current_goal_xy
            self._clear_goal()
        elif status == GoalStatus.STATUS_CANCELED:
            self._clear_goal()
        else:
            self.get_logger().warning(f"Goal ended with status {status}, blacklisting")
            self._fail_current_goal()

    def _frontier_near(self, frontiers, info, x, y):
        """True if any frontier cell lies within goal_invalidate_dist."""
        for f in frontiers:
            cells_x = info.origin.position.x + (f.cells[:, 1] + 0.5) * info.resolution
            cells_y = info.origin.position.y + (f.cells[:, 0] + 0.5) * info.resolution
            d = np.hypot(cells_x - x, cells_y - y)
            if (d < self._goal_invalidate_dist).any():
                return True
        return False

    def _check_active_goal(self, frontiers, info):
        """Preempt the goal if its frontier vanished or it timed out."""
        gx, gy = self._current_goal_xy
        elapsed = (self.get_clock().now() - self._goal_sent_time).nanoseconds / 1e9
        if elapsed > self._goal_timeout:
            self.get_logger().warning("Goal timed out, blacklisting")
            self._cancel_current_goal()
            self._fail_current_goal()
            return

        if not self._frontier_near(frontiers, info, gx, gy):
            self.get_logger().info("Goal area already mapped, re-planning")
            self._cancel_current_goal()
            self._clear_goal()
            # Keep momentum: the frontier usually recedes outward as we
            # approach, so prefer continuing toward the cluster nearest
            # the old goal over flapping to the nearest-to-robot one.
            self._continue_from_xy = (gx, gy)

    def _fail_current_goal(self):
        if self._current_goal_xy is not None:
            self._blacklist.append(self._current_goal_xy)
            self._goals_failed += 1
        self._clear_goal()

    def _clear_goal(self):
        self._current_goal_xy = None
        self._goal_handle = None
        self._result_future = None
        self._goal_sent_time = None

    def _cancel_current_goal(self):
        if self._goal_handle is not None:
            try:
                self._goal_handle.cancel_goal_async()
            except Exception:  # noqa: BLE001
                pass

    # ------------------------------------------------------------------
    # Visualization
    # ------------------------------------------------------------------
    def _publish_markers(self, candidates, info):
        markers = MarkerArray()

        clear = Marker()
        clear.action = Marker.DELETEALL
        markers.markers.append(clear)

        cells = Marker()
        cells.header.frame_id = "map"
        cells.header.stamp = self.get_clock().now().to_msg()
        cells.ns = "frontier_cells"
        cells.id = 0
        cells.type = Marker.POINTS
        cells.scale.x = cells.scale.y = info.resolution
        cells.color.r = 0.0
        cells.color.g = 0.9
        cells.color.b = 1.0
        cells.color.a = 0.8
        for x, y, f in candidates:
            for row, col in f.cells:
                px, py = cell_to_world(
                    (row, col), info.origin.position.x, info.origin.position.y,
                    info.resolution,
                )
                cells.points.append(_point(px, py, 0.05))
        markers.markers.append(cells)

        goals = Marker()
        goals.header.frame_id = "map"
        goals.header.stamp = cells.header.stamp
        goals.ns = "frontier_goals"
        goals.id = 1
        goals.type = Marker.SPHERE_LIST
        goals.scale.x = goals.scale.y = goals.scale.z = 0.3
        goals.color.r = 1.0
        goals.color.g = 0.6
        goals.color.a = 0.9
        for x, y, _ in candidates:
            goals.points.append(_point(x, y, 0.1))
        markers.markers.append(goals)

        if self._current_goal_xy is not None:
            active = Marker()
            active.header.frame_id = "map"
            active.header.stamp = cells.header.stamp
            active.ns = "active_goal"
            active.id = 2
            active.type = Marker.CYLINDER
            active.scale.x = active.scale.y = 0.4
            active.scale.z = 1.5
            active.color.g = 1.0
            active.color.a = 0.6
            active.pose.position.x = self._current_goal_xy[0]
            active.pose.position.y = self._current_goal_xy[1]
            active.pose.position.z = 0.75
            markers.markers.append(active)

        self._marker_pub.publish(markers)


def _point(x, y, z):
    from geometry_msgs.msg import Point

    p = Point()
    p.x, p.y, p.z = float(x), float(y), float(z)
    return p


def main(args=None):
    rclpy.init(args=args)
    node = Explorer()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass


if __name__ == "__main__":
    main()
