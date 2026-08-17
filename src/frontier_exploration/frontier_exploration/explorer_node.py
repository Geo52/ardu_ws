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
from dataclasses import replace
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
    rank_candidates,
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
        # How far the unknown region is grown toward free space when
        # looking for frontiers. Masked by walls (see frontier_search),
        # so it can be generous without leaking through them — and it
        # must be, because the fog band between mapped and unmapped
        # space is far wider than a few cells wherever an area was only
        # glimpsed from a distance. At 3 the detector offered just 4
        # frontiers while 75 m2 of the maze remained unexplored.
        # Measured on a stalled map: reach 3 found 3 frontiers (whole
        # corridors invisible), reach 8 found 7 but four of them were
        # false — fog along a wall face is ambiguous between open
        # passage and unconfirmed wall, and too long a reach pushes
        # through solid geometry to manufacture frontiers that can
        # never be cleared. Reach 6 gave 7 genuine and only 2 false.
        #
        # Lowered back to 3 after run 70. That run landed with 25.1 m2
        # of the south-east corridor (91% of it) unmapped, having spent
        # 12+ goals attacking it from the wrong side: at reach 6 the
        # dilation crosses the 0.2 m wall at y = -3.5 (4 cells at 0.05
        # m/cell) and manufactures a large frontier at (7.76, -3.75)
        # that sits in already-mapped corridor, while the corridor's
        # real opening at its west end registers as only 21 cells and
        # never outranks it. Replaying run 70's final map:
        #
        #   dilation  clusters  pointing into the unmapped regions
        #          2         3      3   (under-detects overall)
        #          3        12      4
        #          4         8      2
        #          6        12      1
        #
        # Reach 3 offers the most distinct ways into the ground that
        # was actually missed. Note the leak is not unique to 6 -- goals
        # hard against a wall face appear at 3 as well -- so this
        # narrows the false attractor rather than eliminating it.
        self.declare_parameter("unknown_dilation", 3)
        # Exploration boundary (map frame), set just outside the maze's
        # outer wall.
        #
        # This check exists to catch a diverged pose, not to keep the
        # vehicle in the maze -- worlds/maze_closed.sdf seals the 3 m
        # gap it used to escape through, so geometry handles that now.
        # It was originally +/-11, a metre outside the wall, which let
        # the vehicle chase goals into open ground where a 2D lidar has
        # nothing to scan-match against; Cartographer diverged by >10 m
        # and silently invalidated the map, the EKF pose and this check
        # itself.
        #
        # Tightening it to +/-9.5 overcorrected. Measured against the
        # walls on run 67 (x [-10.05, 9.85], y [-7.02, 9.98]) a +/-9.5
        # box discards 14.0 m2 of already-mapped free space as goal
        # candidates, including the whole strip the west corridor's
        # 1021-cell frontier sits in at x = -9.80 -- the largest
        # frontier on the map, silently dropped every cycle, which is
        # why three consecutive runs left that corridor unexplored.
        #
        # The maze is also not square: it runs to y = -7, not -9.5, so
        # a symmetric box was never the right shape. These follow the
        # wall extent with a small margin, which still traps any pose
        # that has genuinely diverged.
        #
        # Take the extent from the world file, never from a map. The
        # maze's outer wall sits at +/-10 in both axes (39 poses in
        # worlds/maze_closed.sdf, x and y both spanning -10..10), so
        # +/-9.9 is just inside it.
        #
        # Measuring it from a map instead produced two wrong answers in
        # a row. Run 67's occupancy grid had no wall below y = -7.02,
        # so a bound of -7.0 looked correct -- but the grid stopped
        # there only because no run had ever entered the region below,
        # and that bound would have sealed off the entire southern
        # third of the maze permanently. A map cannot tell you how big
        # the world is when the unexplored part is what you are
        # measuring.
        self.declare_parameter("bound_min_x", -9.9)
        self.declare_parameter("bound_max_x", 9.9)
        self.declare_parameter("bound_min_y", -9.9)
        self.declare_parameter("bound_max_y", 9.9)
        self.declare_parameter("eval_period", 2.0)
        self.declare_parameter("marker_cell_budget", 800)
        # Radius (m) over which unknown area is counted when scoring a
        # frontier's information gain. Roughly the useful sensing
        # footprint in a maze, where walls curtail the 30 m lidar.
        self.declare_parameter("gain_radius", 6.0)
        # Two frontier observations closer than this are treated as the
        # same frontier across evaluation cycles, so it keeps its
        # discovery order as its centroid drifts.
        self.declare_parameter("frontier_match_radius", 2.0)
        # A frontier within this distance (m) of anywhere the vehicle
        # has already flown counts as somewhere it has been, and is
        # ranked behind every frontier in new ground.
        self.declare_parameter("visited_radius", 2.5)
        self.declare_parameter("visited_spacing", 0.5)
        # Keep goals this far (m) from walls. Below the costmap's
        # inscribed radius (robot_radius, 0.35) the planner cannot put
        # the vehicle at the goal at all.
        self.declare_parameter("min_goal_clearance", 0.5)
        # Among a cluster's cells, prefer the one with the most unknown
        # within this radius (m), so the route ends against the largest
        # patch of unexplored ground rather than the middle of the
        # frontier. The goal stays on a known-free cell: moving it into
        # the unknown outright plants it inside unmapped wall.
        self.declare_parameter("face_unknown_radius", 2.5)
        # A frontier revealing less unknown than this (m2, counted
        # within face_unknown_radius of the goal) is demoted behind
        # everything that reveals more -- never dropped, so it is still
        # collected at the end.
        #
        # Chosen from the data, not by feel. Across two captured maps
        # the frontiers worth flying to revealed 5.5-15.8 m2 while fog
        # banded along the outer wall revealed 0.5-2.1 m2, with nothing
        # in between; 3.0 sits in that gap on both.
        self.declare_parameter("min_unknown_gain_m2", 3.0)
        # How long to wait for a post-arrival map before giving up on
        # the persistence check rather than blocking exploration.
        self.declare_parameter("arrival_settle_timeout", 8.0)
        # A frontier counts as unmappable-from-here only after surviving
        # this many arrivals within persist_same_spot metres of each
        # other; one survival just means it receded as we mapped.
        self.declare_parameter("persist_before_blacklist", 3)
        self.declare_parameter("persist_same_spot", 1.5)
        # Radius around a blacklisted point that is also excluded.
        # Frontiers arrive in lines, not singly: an unconfirmed fog
        # band along a wall face produces a dozen near-identical
        # frontiers a metre apart, none of which can ever be cleared
        # because the barrier behind them is solid. At 0.8 m each had
        # to be visited three times to dismiss it, which is upwards of
        # thirty wasted trips. Dismissing a neighbourhood at a time
        # costs a little genuine frontier at the edges and saves the
        # endgame from grinding.
        self.declare_parameter("blacklist_radius", 2.5)
        # ...but that radius is calibrated for fog bands, and applying it
        # to a large frontier seals the way into whatever it opens onto.
        #
        # Run 70 landed with 14.4 m2 of the west strip unmapped while the
        # detector was still offering it as a 1949-cell frontier -- the
        # largest on the map by a factor of three. One Nav2 abort at
        # (-9.59, -4.80) blacklisted a 2.5 m neighbourhood, which is the
        # entire entrance; three separate attempts over the run each died
        # the same way, and the two blacklist-clear retries were spent
        # elsewhere. Nothing was wrong with the frontier: `Failed to make
        # progress` never appears in that run, so the controller was not
        # stalling, the planner simply refused that one goal cell.
        #
        # A cluster that large has hundreds of alternative goal cells, so
        # excluding just the cell that failed lets the next evaluation
        # approach the same region from somewhere the planner accepts,
        # while small frontiers keep the wide radius that stops the
        # endgame grinding through a dozen near-identical fog goals.
        self.declare_parameter("large_frontier_cells", 400)
        self.declare_parameter("large_frontier_blacklist_radius", 0.6)
        # Bound on that leniency. Each failed goal costs ~20 s, and a
        # 1949-cell cluster holds enough alternative cells to spend ten
        # minutes discovering the planner refuses all of them. After
        # this many narrow exclusions in one neighbourhood, treat the
        # region as genuinely unreachable and apply the full radius.
        self.declare_parameter("large_frontier_attempts", 3)
        self.declare_parameter("goal_timeout", 90.0)
        # How far from an abandoned goal a frontier still counts as
        # "onward" when the goal was dropped because its area got
        # mapped from a distance. Wide enough to cover a frontier that
        # has receded a corridor's length deeper, narrow enough that it
        # does not simply re-select the whole map.
        self.declare_parameter("commit_radius", 5.0)
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
        self._frontier_match_radius = self.get_parameter(
            "frontier_match_radius"
        ).value
        self._visited_radius = self.get_parameter("visited_radius").value
        self._visited_spacing = self.get_parameter("visited_spacing").value
        self._min_goal_clearance = self.get_parameter("min_goal_clearance").value
        self._face_unknown_radius = self.get_parameter(
            "face_unknown_radius"
        ).value
        self._bounds = (
            self.get_parameter("bound_min_x").value,
            self.get_parameter("bound_max_x").value,
            self.get_parameter("bound_min_y").value,
            self.get_parameter("bound_max_y").value,
        )
        self._blacklist_radius = self.get_parameter("blacklist_radius").value
        self._large_frontier_cells = self.get_parameter(
            "large_frontier_cells"
        ).value
        self._large_frontier_blacklist_radius = self.get_parameter(
            "large_frontier_blacklist_radius"
        ).value
        self._large_frontier_attempts = self.get_parameter(
            "large_frontier_attempts"
        ).value
        self._goal_timeout = self.get_parameter("goal_timeout").value
        self._goal_invalidate_dist = self.get_parameter("goal_invalidate_dist").value
        self._commit_radius = self.get_parameter("commit_radius").value
        self._min_unknown_gain_m2 = self.get_parameter(
            "min_unknown_gain_m2"
        ).value
        self._min_unknown_gain = 0  # in cells; set per map from resolution
        self._empty_evals_before_land = self.get_parameter(
            "empty_evals_before_land"
        ).value
        self._arrival_settle_timeout = self.get_parameter(
            "arrival_settle_timeout"
        ).value
        self._persist_before_blacklist = self.get_parameter(
            "persist_before_blacklist"
        ).value
        self._persist_same_spot = self.get_parameter("persist_same_spot").value

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
        self._blacklist = []  # (x, y, exclusion radius in metres)
        self._blacklist_clears_left = 2
        # Cell count of the frontier the active goal came from, so a
        # failure can size its own exclusion.
        self._current_goal_size = 0
        self._goal_handle = None
        self._result_future = None
        self._current_goal_xy = None
        # Set when a goal is dropped because its area was mapped from a
        # distance; biases the next selection to keep the same heading.
        self._preempted_goal_xy = None
        self._goal_sent_time = None
        self._goal_seq = 0
        self._reached_goal_xy = None
        self._reached_at = None
        self._last_persist_xy = None
        self._persist_count = 0
        self._climb_ticks = 0
        self._climb_timeout_ticks = 25
        self._visited = []           # sampled flight path
        self._known_frontiers = []   # (x, y, discovery sequence)
        self._frontier_seq = 0
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
            # Report what is missing, not just that something is. A
            # startup race here looks identical from the outside to a
            # vehicle that refuses to take off, and without this the
            # only way to tell which interface never came up was to
            # interrogate the graph by hand while the run sat idle.
            missing = [
                name
                for name, ok in (
                    ("arm service", self._arm_client.service_is_ready()),
                    ("mode service", self._mode_client.service_is_ready()),
                    ("takeoff service", self._takeoff_client.service_is_ready()),
                    ("nav2 action server", self._nav_client.server_is_ready()),
                    ("/map", self._map is not None),
                    ("EKF origin", self._origin_set.is_set()),
                )
                if not ok
            ]
            if not missing:
                self.get_logger().info("All interfaces ready")
                self._state = State.SET_MODE
                return
            self._wait_ticks = getattr(self, "_wait_ticks", 0) + 1
            if self._wait_ticks % 10 == 0:
                self.get_logger().warning(
                    f"Still waiting on: {', '.join(missing)}"
                )
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
            self._climb_ticks += 1
            if self._ap_alt is not None and self._ap_alt >= 0.9 * self._takeoff_alt:
                self.get_logger().info(
                    f"Reached {self._ap_alt:.2f} m, starting exploration"
                )
                self._climb_ticks = 0
                self._state = State.EXPLORE
            elif self._climb_ticks >= self._climb_timeout_ticks:
                # Never block the mission on an exact altitude. A
                # takeoff issued to an already-airborne copter does not
                # climb like a fresh one, so after an altitude-sag
                # recovery the vehicle settled at 1.4 m and this state
                # waited forever. Any height that clears the ground is
                # enough to map from; the walls are 3.25 m.
                self.get_logger().warning(
                    f"Still at {self._ap_alt:.2f} m after {self._climb_ticks}s; "
                    "exploring anyway"
                )
                self._climb_ticks = 0
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
            face_unknown_radius=int(
                self._face_unknown_radius / info.resolution
            ),
        )

        # World-frame goal for each cluster, minus blacklisted ones and
        # anything outside the exploration boundary.
        min_x, max_x, min_y, max_y = self._bounds
        candidates = []
        out_of_bounds = []
        for f in frontiers:
            x, y = cell_to_world(
                f.goal_cell, info.origin.position.x, info.origin.position.y,
                info.resolution,
            )
            if not (min_x <= x <= max_x and min_y <= y <= max_y):
                # Try another cell before giving up on the cluster.
                #
                # A frontier is represented by one cell chosen for how
                # much unknown it faces, which pushes it hard against
                # whatever wall the unknown lies behind. Discarding the
                # whole cluster when that single cell falls outside the
                # boundary throws away everything it borders: run 69
                # rejected a 734-cell frontier because its
                # representative landed at x = -9.93, 3 cm inside the
                # west wall's footprint, while hundreds of its cells
                # sat in open space. Widening the boundary would not
                # help -- the planner cannot route into a wall either
                # -- so re-pick the nearest cell that is genuinely in
                # bounds.
                alt = self._nearest_in_bounds(f, info)
                if alt is None:
                    out_of_bounds.append((x, y, f.size))
                    continue
                x, y, f = alt
            if any(
                math.hypot(x - bx, y - by) < br
                for bx, by, br in self._blacklist
            ):
                continue
            candidates.append((x, y, f))

        # Say so when the boundary rejects something substantial.
        #
        # This filter is the only place a frontier leaves the pipeline
        # without a word, and that silence cost three runs: at +/-9.5
        # the largest frontier on the map -- 1021 cells opening into
        # the unexplored west corridor at x = -9.80 -- was discarded
        # every single cycle, and the logs showed only that it was
        # never chosen. A rejection bigger than anything still in the
        # running is the signature of a boundary cutting into the map
        # rather than guarding its edge, so it is worth a warning.
        if out_of_bounds:
            biggest = max(out_of_bounds, key=lambda c: c[2])
            largest_kept = max((f.size for _, _, f in candidates), default=0)
            if biggest[2] >= max(largest_kept, 200):
                self._oob_warned = getattr(self, "_oob_warned", 0) + 1
                if self._oob_warned % 10 == 1:
                    self.get_logger().warning(
                        f"Boundary rejected a {biggest[2]}-cell frontier at "
                        f"({biggest[0]:.2f}, {biggest[1]:.2f}); largest kept "
                        f"is {largest_kept}. Bounds may be cutting into "
                        f"the map."
                    )

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
                    # Surviving one arrival is normal, and is how a
                    # corridor gets explored: arrive, map the near part,
                    # the frontier recedes deeper, go again. Blacklisting
                    # on the first survival takes one bite of a corridor
                    # and abandons it — especially since "arrived" means
                    # within xy_goal_tolerance, which need not be close
                    # enough to clear it.
                    #
                    # Only a frontier that survives repeated arrivals at
                    # the same place is genuinely unmappable from there.
                    # A goal that has moved on means progress, so the
                    # count restarts.
                    if (
                        self._last_persist_xy is not None
                        and math.hypot(
                            rx - self._last_persist_xy[0],
                            ry - self._last_persist_xy[1],
                        )
                        < self._persist_same_spot
                    ):
                        self._persist_count += 1
                    else:
                        self._persist_count = 1
                    self._last_persist_xy = (rx, ry)

                    if self._persist_count >= self._persist_before_blacklist:
                        self.get_logger().info(
                            f"Frontier unchanged after {self._persist_count} "
                            "arrivals, blacklisting"
                        )
                        # Full radius: a frontier that survived repeated
                        # arrivals is unmappable from here, and its
                        # neighbours almost always are too.
                        self._blacklist.append(
                            (*self._reached_goal_xy, self._blacklist_radius)
                        )
                        self._persist_count = 0
                        self._last_persist_xy = None
                    else:
                        self.get_logger().info(
                            "Frontier receded rather than cleared; "
                            "continuing into it"
                        )
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
        if pose is not None:
            self._record_visited(*pose)
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
            # Depth-first: go to the most recently discovered frontier.
            #
            # Frontiers are recomputed from scratch each cycle, so a
            # policy that only looks at the current set has no memory of
            # which opening it uncovered last. Two corridors of similar
            # distance then trade the lead every time one is partly
            # mapped, and neither gets finished.
            #
            # Tracking when each frontier was first seen gives the stack
            # discipline DFS needs. New frontiers appear where the
            # vehicle is currently revealing space, so preferring the
            # newest drives it deeper down one branch; when that branch
            # is exhausted the newest surviving frontier is whatever was
            # deferred most recently, which is the backtrack. Distance
            # only breaks ties among equally-recent frontiers.
            # How much unknown each goal actually opens up. Cluster
            # size is not a usable proxy: fog banded along an outer
            # wall makes a large cluster that reveals almost nothing,
            # and that is what the vehicle kept flying back to.
            gain_field = unknown_gain(
                grid, int(self._face_unknown_radius / info.resolution)
            )
            self._min_unknown_gain = int(
                self._min_unknown_gain_m2 / (info.resolution ** 2)
            )
            unreachable = 0
            tagged = []
            for x, y, f in candidates:
                cost = travel[f.goal_cell] * info.resolution
                if not math.isfinite(cost):
                    unreachable += 1
                    cost = 1e6  # tried last, never vetoed
                tagged.append(
                    (self._been_there(x, y), self._frontier_age(x, y),
                     cost, x, y, f, int(gain_field[f.goal_cell]))
                )
            if unreachable:
                self.get_logger().info(
                    f"{unreachable} frontier(s) with no coarse route, "
                    "deprioritised"
                )
            self._forget_stale_frontiers(candidates)
            # Somewhere new first; then newest-discovered (depth-first);
            # then nearest. A frontier inside ground the vehicle has
            # already flown is real — residual fog is left along the
            # path itself, behind the vehicle and at grazing angles —
            # but going back for it while anywhere new remains is what
            # makes the exploration look like it is retracing itself.
            # Ranked last rather than discarded, so they are still
            # collected once the unvisited frontiers are gone.
            fresh = sum(1 for t in tagged if t[0] == 0)
            tagged = rank_candidates(tagged, min_gain=self._min_unknown_gain)

            # Honour the heading we were already committed to.
            #
            # If the last goal was dropped because its area got mapped
            # from a distance, the exploration has not failed -- it has
            # advanced, and the frontier has receded deeper into the
            # space just revealed. Prefer a candidate near where we
            # were going, so the vehicle carries on into the unmapped
            # region instead of turning round for closer fog. Only
            # applies for one selection, and only if something is
            # actually out that way.
            if self._preempted_goal_xy is not None:
                px, py = self._preempted_goal_xy
                # Partition by index: these tuples hold Frontier
                # objects whose numpy `cells` make `in` / `==`
                # ambiguous.
                near_idx, far_idx = [], []
                for i, t in enumerate(tagged):
                    d = math.hypot(t[3] - px, t[4] - py)
                    (near_idx if d <= self._commit_radius else far_idx).append(i)
                if near_idx:
                    self.get_logger().info(
                        f"Continuing toward ({px:.2f}, {py:.2f}): "
                        f"{len(near_idx)} frontier(s) still onward"
                    )
                    tagged = [tagged[i] for i in near_idx + far_idx]
                self._preempted_goal_xy = None
            if fresh == 0 and tagged:
                self.get_logger().info(
                    "Only already-visited frontiers remain; "
                    "going back for the most informative first"
                )
            candidates = [(t[3], t[4], t[5]) for t in tagged]

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
        self._current_goal_size = int(f.size)
        self._send_goal(x, y, rx, ry)


    # ------------------------------------------------------------------
    # Frontier discovery order (depth-first bookkeeping)
    # ------------------------------------------------------------------
    def _nearest_in_bounds(self, f, info):
        """Re-pick a cluster's goal to the closest in-bounds cell.

        Returns ``(x, y, frontier)`` with the frontier's ``goal_cell``
        moved, or None if the whole cluster lies outside the boundary
        -- in which case it really should be dropped.
        """
        min_x, max_x, min_y, max_y = self._bounds
        ox, oy, res = (
            info.origin.position.x, info.origin.position.y, info.resolution,
        )
        xs = ox + (f.cells[:, 1] + 0.5) * res
        ys = oy + (f.cells[:, 0] + 0.5) * res
        ok = (xs >= min_x) & (xs <= max_x) & (ys >= min_y) & (ys <= max_y)
        idx = np.flatnonzero(ok)
        if idx.size == 0:
            return None
        # Aim at the cluster's body, not at the cell nearest the one
        # that was rejected. The rejected cell is against a wall by
        # construction -- it was chosen for facing the most unknown --
        # so its nearest in-bounds neighbour hugs the same wall and
        # lands inside the costmap's inflated zone, where the planner
        # aborts with "compute_path_to_pose" and Nav2 burns a
        # backup-and-spin recovery. Run 69 stalled for five minutes in
        # the south-west corner doing exactly that. The cell closest to
        # the centroid sits in open frontier instead.
        cr, cc = f.centroid
        d = (f.cells[idx, 0] - cr) ** 2 + (f.cells[idx, 1] - cc) ** 2
        best = idx[int(np.argmin(d))]
        cell = (int(f.cells[best, 0]), int(f.cells[best, 1]))
        return float(xs[best]), float(ys[best]), replace(f, goal_cell=cell)

    def _frontier_age(self, x, y):
        """Sequence number of when this frontier was first seen.

        Frontier clusters are recomputed every cycle and their centroids
        shift as the map fills in, so identity is by proximity: a
        frontier within `frontier_match_radius` of a known one is the
        same frontier, and keeps its original sequence number. Anything
        else is newly discovered and gets the next number.
        """
        for i, (px, py, seq) in enumerate(self._known_frontiers):
            if math.hypot(x - px, y - py) < self._frontier_match_radius:
                # Track its latest position so it can drift with the map.
                self._known_frontiers[i] = (x, y, seq)
                return seq
        self._frontier_seq += 1
        self._known_frontiers.append((x, y, self._frontier_seq))
        return self._frontier_seq

    def _forget_stale_frontiers(self, candidates):
        """Drop remembered frontiers that no longer exist.

        Without this the list grows without bound and, worse, a mapped
        away frontier could keep its old sequence number if a new one
        later appeared nearby.
        """
        if not candidates:
            self._known_frontiers = []
            return
        live = []
        for px, py, seq in self._known_frontiers:
            if any(
                math.hypot(px - x, py - y) < self._frontier_match_radius
                for x, y, _ in candidates
            ):
                live.append((px, py, seq))
        self._known_frontiers = live


    def _record_visited(self, x, y):
        """Remember roughly where the vehicle has flown."""
        if self._visited and math.hypot(
            x - self._visited[-1][0], y - self._visited[-1][1]
        ) < self._visited_spacing:
            return
        self._visited.append((x, y))

    def _been_there(self, x, y):
        """1 if this goal sits in ground the vehicle has already flown."""
        for vx, vy in self._visited:
            if math.hypot(x - vx, y - vy) < self._visited_radius:
                return 1
        return 0

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
            # Remember where we were heading. The lidar reaches ~30 m
            # across a 20 m maze, so the sensor outranges the flight:
            # the region we set out for is usually mapped from a
            # distance before we arrive, and this branch fires. Most
            # goals end here.
            #
            # Re-ranking from scratch at that moment throws away the
            # commitment. The frontier we were chasing has receded
            # deeper into the space it just revealed, but that new
            # frontier is now further from the vehicle than the
            # residual fog behind it, so the score sends us backwards
            # -- the vehicle turns around halfway to an unmapped
            # region and returns to ground it has already covered.
            # Keeping the heading lets the next selection prefer
            # whatever lies onward before reconsidering the whole map.
            self._preempted_goal_xy = (gx, gy)
            # Deliberately no cancel: sending the next goal preempts this
            # one at the action server, whereas cancelling halts the
            # controller and the copter must stop and re-accelerate.
            self._clear_goal()

    def _fail_current_goal(self):
        if self._current_goal_xy is not None:
            gx, gy = self._current_goal_xy
            # How many narrow exclusions this neighbourhood has already
            # been granted.
            spent = sum(
                1
                for bx, by, br in self._blacklist
                if br < self._blacklist_radius
                and math.hypot(gx - bx, gy - by) < self._blacklist_radius
            )
            if (
                self._current_goal_size >= self._large_frontier_cells
                and spent < self._large_frontier_attempts
            ):
                # Exclude the cell that failed, not the region it opens
                # onto: a cluster this size has hundreds of other goal
                # cells and the planner may well accept one of them.
                radius = self._large_frontier_blacklist_radius
                self.get_logger().info(
                    f"Large frontier ({self._current_goal_size} cells), "
                    f"attempt {spent + 1}/{self._large_frontier_attempts}: "
                    f"excluding {radius} m around the failed goal only"
                )
            else:
                radius = self._blacklist_radius
            self._blacklist.append((*self._current_goal_xy, radius))
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
