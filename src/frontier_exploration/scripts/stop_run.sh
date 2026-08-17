#!/usr/bin/env bash
# Tear down every process a run starts.
#
# `ros2 launch` killed via its parent does not reliably take its
# children with it, and a survivor is not harmless: an orphaned rviz2
# stays subscribed to /map and competes for the CPU the next run's
# control loop needs, while an orphaned gz sim holds the simulation
# port so the next launch comes up half-broken. Both have happened.
# Kill by name, verify, then escalate.
set -u

PATTERNS=(
    "explore.launch.py"
    "gz sim"
    "ruby.*gz"
    "arducopter"
    "cartographer"
    "micro_ros_agent"
    "rviz2"
    "twist_stamper"
    # Match this package's nodes by executable name, never by package
    # name: a pattern of "frontier_exploration" also matches this
    # script's own path, so pkill -f killed the teardown mid-run and
    # left 46 processes up.
    "lib/frontier_exploration/explorer"
    "lib/frontier_exploration/pose_relay"
    "lib/frontier_exploration/odom_sanitizer"
    "lib/frontier_exploration/cmd_vel_altitude_hold"
    "controller_server"
    "planner_server"
    "bt_navigator"
    "behavior_server"
    "smoother_server"
    "velocity_smoother"
    "waypoint_follower"
    "lifecycle_manager"
    "nav2_container"
    "component_container"
    "parameter_bridge"
    "robot_state_publisher"
)

SELF=$$

alive() {
    # pgrep -c prints 0 AND exits non-zero when nothing matches, so a
    # `|| echo 0` fallback would emit two lines and break the sum.
    # Never count this script or its own subshells.
    local n=0 c
    for p in "${PATTERNS[@]}"; do
        c=$(pgrep -f "$p" 2>/dev/null | grep -cvE "^($SELF|$PPID)$")
        n=$((n + ${c:-0}))
    done
    echo "$n"
}

# pkill -f matches this script's own command line via its arguments, so
# kill by PID with self excluded rather than by pattern.
sweep() {
    local sig=$1 p pid
    for p in "${PATTERNS[@]}"; do
        for pid in $(pgrep -f "$p" 2>/dev/null); do
            [ "$pid" = "$SELF" ] && continue
            [ "$pid" = "$PPID" ] && continue
            kill "$sig" "$pid" 2>/dev/null
        done
    done
}

echo "before: $(alive) process(es)"

sweep -TERM
sleep 4

if [ "$(alive)" -gt 0 ]; then
    echo "escalating to SIGKILL"
    sweep -KILL
    sleep 3
fi

remaining=$(alive)
echo "after: $remaining process(es)"
if [ "$remaining" -gt 0 ]; then
    echo "STILL RUNNING — do not start another run until these are gone:"
    for p in "${PATTERNS[@]}"; do
        pgrep -af "$p" 2>/dev/null | grep -vE "^($SELF|$PPID) "
    done
    exit 1
fi
echo "clean"
