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
from rclpy.time import Time as RclTime
from rclpy.qos import (
    QoSDurabilityPolicy,
    QoSHistoryPolicy,
    QoSProfile,
    QoSReliabilityPolicy,
)
from tf2_ros import Buffer, TransformListener
from visualization_msgs.msg import Marker, MarkerArray

from frontier_exploration.frontier_search import (
    cell_to_world,
    find_frontiers,
    travel_distances,
    unknown_gain,
)

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
        # Exploration boundary (map frame). The maze world spans +/-10 m
        # and has an opening in its outer wall, so these must sit
        # INSIDE that wall: at +/-11 the vehicle was allowed to chase
        # goals a metre outside the maze, flew out through the opening,
        # and once in open ground the lidar had no walls to scan-match
        # against. Cartographer then diverged by >10 m, which silently
        # invalidates the map, the EKF pose and this bounds check itself.
        self.declare_parameter("bound_min_x", -9.5)
        self.declare_parameter("bound_max_x", 9.5)
        self.declare_parameter("bound_min_y", -9.5)
        self.declare_parameter("bound_max_y", 9.5)
        self.declare_parameter("eval_period", 2.0)
        self.declare_parameter("marker_cell_budget", 800)
        # Radius (m) over which unknown area is counted when scoring a
        # frontier's information gain. Roughly the useful sensing
        # footprint in a maze, where walls curtail the 30 m lidar.
        self.declare_parameter("gain_radius", 6.0)
        # Keep goals this far (m) from walls. Below the costmap's
        # inscribed radius (robot_radius, 0.35) the planner cannot put
        # the vehicle at the goal at all.
        self.declare_parameter("min_goal_clearance", 0.5)
        # How long to wait for a post-arrival map before giving up on
        # the persistence check rather than blocking exploration.
        self.declare_parameter("arrival_settle_timeout", 8.0)
        self.declare_parameter("blacklist_radius", 0.8)
        self.declare_parameter("goal_timeout", 90.0)
        # A goal is preempted when no frontier cell remains within this
        # distance of it (the area has been mapped while in transit).
        # Kept in step with Nav2's xy_goal_tolerance: the vehicle now
        # counts as arrived 1.5 m out, so "did the frontier survive our
        # arrival" has to be asked over the same radius, or frontiers
        # get blacklisted for persisting near a point the vehicle was
        # never required to reach.
        self.declare_parameter("goal_invalidate_dist", 1.5)
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
        self._marker_cell_budget = self.get_parameter("marker_cell_budget").value
        self._gain_radius = self.get_parameter("gain_radius").value
        self._min_goal_clearance = self.get_parameter("min_goal_clearance").value
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
        self._arrival_settle_timeout = self.get_parameter(
            "arrival_settle_timeout"
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
        self._reached_at = None
        self._goals_succeeded = 0
        self._goals_failed = 0
        self._empty_evals = 0
        self._land_requested = False
        self._retry_at = None
        self._ap_orientation = None
        self._grounded_ticks = 0
        # Tick runs at 1 Hz; require a few consecutive readings so a
        # transient altitude dip does not trigger a takeoff.
        self._grounded_ticks_before_recover = 4
        self._recoveries_left = 1

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
        self._ap_orientation = msg.pose.orientation

    def _tilt_deg(self):
        """Angle between the vehicle's z axis and vertical, in degrees."""
        q = self._ap_orientation
        if q is None:
            return 0.0
        # z-axis of the body frame expressed in world coordinates.
        zz = 1.0 - 2.0 * (q.x * q.x + q.y * q.y)
        return math.degrees(math.acos(max(-1.0, min(1.0, zz))))

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

        # A wall strike can put the vehicle on the ground without
        # ArduPilot declaring a crash: it stays level, so the crash
        # detector's angle test never trips, and it reports armed,
        # GUIDED and "flying" while sitting at zero altitude. Nav2 then
        # keeps commanding velocity at a grounded vehicle forever.
        # Horizontal velocity will not lift it, so re-take-off.
        if (
            self._state == State.EXPLORE
            and self._ap_alt is not None
            and self._ap_alt < 0.5 * self._takeoff_alt
        ):
            self._grounded_ticks += 1
            if self._grounded_ticks >= self._grounded_ticks_before_recover:
                self._cancel_current_goal()
                self._clear_goal()
                self._pending_srv = None
                self._grounded_ticks = 0
                # Only attempt a climb if the vehicle is sitting level.
                # Commanding takeoff while it is lodged against a wall
                # tilts it until ArduPilot's crash detector trips —
                # that turned a recoverable grounding into a crash.
                # Give up rather than repeat: two failed recoveries in
                # a row means something the explorer cannot fix.
                if self._tilt_deg() < 15.0 and self._recoveries_left > 0:
                    self._recoveries_left -= 1
                    self.get_logger().warning(
                        f"On the ground at {self._ap_alt:.2f} m and level; "
                        "re-taking off"
                    )
                    self._state = State.TAKEOFF
                else:
                    self.get_logger().error(
                        f"Grounded at {self._ap_alt:.2f} m, tilt "
                        f"{self._tilt_deg():.0f} deg; ending run"
                    )
                    self._state = State.LAND
                return
        else:
            self._grounded_ticks = 0

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
            min_goal_clearance=self._min_goal_clearance / info.resolution,
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
        # here: blacklist it so the policy moves on. This may only be
        # judged against a map built *after* we arrived — /map is
        # published at 1 Hz, so the next evaluation still carries the
        # pre-arrival map and would condemn every frontier we reach.
        if self._reached_goal_xy is not None:
            map_time = RclTime.from_msg(grid_msg.header.stamp)
            waited = self.get_clock().now() - self._reached_at
            if map_time > self._reached_at:
                rx, ry = self._reached_goal_xy
                if self._frontier_near(frontiers, info, rx, ry):
                    self.get_logger().info(
                        "Frontier persists after arrival, blacklisting"
                    )
                    self._blacklist.append(self._reached_goal_xy)
                self._reached_goal_xy = None
            elif waited > Duration(seconds=self._arrival_settle_timeout):
                self.get_logger().warning("No fresh map after arrival; skipping check")
                self._reached_goal_xy = None

        # Check the in-flight goal.
        if self._current_goal_xy is not None:
            self._check_active_goal(frontiers, info)

        if self._current_goal_xy is not None:
            return  # still navigating

        # Rank by how far the vehicle must actually fly, not by straight
        # line. In a maze the two differ wildly: measured on a real map,
        # the nearest candidate by straight line needed 33 m of travel
        # while another at the same apparent distance needed 18 m.
        # Euclidean ranking sends the vehicle back and forth across
        # ground it has already covered. Unreachable candidates are
        # dropped here rather than discovered by flying at them.
        pose = self._robot_xy()
        if pose is not None and candidates:
            rx, ry = pose
            robot_cell = (
                int((ry - info.origin.position.y) / info.resolution),
                int((rx - info.origin.position.x) / info.resolution),
            )
            travel = travel_distances(
                grid, robot_cell, free_max=self._free_max
            )
            # Utility, not just proximity: a frontier opening into a
            # large unmapped region is worth flying past several small
            # ones. Score is unknown area revealed per metre flown, so
            # there is no weight to hand-tune between the two terms.
            # Measured on a real map: cost-only picks a frontier 18 m
            # away revealing 15 m2, this picks one 32 m away revealing
            # 56 m2.
            gain = unknown_gain(
                grid, int(self._gain_radius / info.resolution)
            )
            cell_area = info.resolution ** 2
            # The travel estimate is a coarse approximation, so it ranks
            # but never vetoes: Nav2 plans at full resolution and finds
            # routes this misses. Discarding "unreachable" candidates
            # instead threw away 3 of 4 real frontiers and landed the
            # vehicle on a map only 90 m2 explored. Unreachable ones go
            # last, and are still tried once everything else is done.
            ranked = []
            unreachable = 0
            for x, y, f in candidates:
                cost = travel[f.goal_cell] * info.resolution
                reveals = gain[f.goal_cell] * cell_area
                if math.isfinite(cost):
                    score = reveals / (cost + 2.0)
                else:
                    unreachable += 1
                    score = -1.0  # sorts below every reachable candidate
                ranked.append((score, x, y, f))
            if unreachable:
                self.get_logger().info(
                    f"{unreachable} frontier(s) with no coarse route, "
                    "deprioritised"
                )
            ranked.sort(key=lambda r: -r[0])
            candidates = [(x, y, f) for _, x, y, f in ranked]

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

        # Ordered above by utility: unknown area revealed per metre of
        # travel, with travel measured around walls rather than through
        # them.
        #
        # A "momentum" rule used to run ahead of this, continuing toward
        # whichever frontier was nearest the goal just invalidated. It
        # was a patch for the zigzagging of Euclidean nearest-first, and
        # the cost function replaced the need for it — but it stayed,
        # and since most goals end as "already mapped" it was overriding
        # the utility ranking on the majority of decisions with plain
        # straight-line proximity. Removed: one selection rule, the
        # measured one.
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
            self._reached_at = self.get_clock().now()
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
            # Deliberately no cancel: sending the next goal preempts this
            # one at the action server, whereas cancelling halts the
            # controller and the copter must stop and re-accelerate.
            # Most goals end here (the 30 m lidar outranges the flight),
            # so that stop-go cost dominates the run.
            self._clear_goal()

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
        # Frontier bands run to thousands of cells; building a Point per
        # cell every evaluation costs enough CPU to starve Nav2's
        # control loop, so subsample to a budget for visualisation.
        total = sum(len(f.cells) for _, _, f in candidates)
        stride = max(1, total // self._marker_cell_budget)
        for x, y, f in candidates:
            for row, col in f.cells[::stride]:
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
