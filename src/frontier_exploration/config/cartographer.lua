include "map_builder.lua"
include "trajectory_builder.lua"

options = {
  map_builder = MAP_BUILDER,
  trajectory_builder = TRAJECTORY_BUILDER,
  map_frame = "map",
  tracking_frame = "imu_link",
  published_frame = "base_link",
  odom_frame = "odom",
  provide_odom_frame = true,
  publish_frame_projected_to_2d = false,
  -- Odometry keeps the pose extrapolator (and therefore the tf stream
  -- feeding EKF3 ExternalNav) smooth between scan matches. The topic
  -- must have strictly increasing stamps: the raw gz bridge output does
  -- not guarantee that (fatal CHECK in MapByTime), so the launch pipes
  -- it through the odom_sanitizer node.
  use_odometry = true,
  use_nav_sat = false,
  use_landmarks = false,
  num_laser_scans = 1,
  num_multi_echo_laser_scans = 0,
  num_subdivisions_per_laser_scan = 1,
  num_point_clouds = 0,
  lookup_transform_timeout_sec = 0.2,
  submap_publish_period_sec = 0.3,
  -- 200 Hz (5e-3) is wasted work: the ExternalNav relay throttles to
  -- 50 Hz anyway, and the surplus /tf traffic costs enough CPU in the
  -- Python relay to starve Nav2's 20 Hz control loop.
  pose_publish_period_sec = 0.02,
  trajectory_publish_period_sec = 30e-3,
  rangefinder_sampling_ratio = 1.,
  odometry_sampling_ratio = 1.,
  fixed_frame_pose_sampling_ratio = 1.,
  imu_sampling_ratio = 1.,
  landmarks_sampling_ratio = 1.,
}

MAP_BUILDER.use_trajectory_builder_2d = true

TRAJECTORY_BUILDER_2D.min_range = 0.05
TRAJECTORY_BUILDER_2D.max_range = 30
TRAJECTORY_BUILDER_2D.missing_data_ray_length = 8.5
TRAJECTORY_BUILDER_2D.use_imu_data = false
-- These weights anchor the scan-match solution to the motion prior.
-- The upstream ardupilot_cartographer values (0.2 / 5) are ~50x below
-- Cartographer's defaults, which leaves the solution free to slide
-- along a corridor: scan matching is geometrically degenerate along
-- the corridor axis, so with a near-zero prior weight the estimate
-- drifted >6 m in x while y stayed accurate to centimetres, silently
-- corrupting the map and the ExternalNav pose fed to EKF3.
TRAJECTORY_BUILDER_2D.ceres_scan_matcher.translation_weight = 10
TRAJECTORY_BUILDER_2D.ceres_scan_matcher.rotation_weight = 40
TRAJECTORY_BUILDER_2D.use_online_correlative_scan_matching = true
TRAJECTORY_BUILDER_2D.real_time_correlative_scan_matcher.linear_search_window = 0.1
TRAJECTORY_BUILDER_2D.real_time_correlative_scan_matcher.translation_delta_cost_weight = 1.
TRAJECTORY_BUILDER_2D.real_time_correlative_scan_matcher.rotation_delta_cost_weight = 10
TRAJECTORY_BUILDER_2D.motion_filter.max_angle_radians = math.rad(0.2)
-- for current lidar only 1 is good value
TRAJECTORY_BUILDER_2D.num_accumulated_range_data = 1

TRAJECTORY_BUILDER_2D.min_z = -0.5
TRAJECTORY_BUILDER_2D.max_z = 0.5

-- Global loop closure is DISABLED in this environment.
--
-- A maze is adversarial for it: every corridor looks like every other
-- corridor, so the matcher finds convincing but wrong correspondences
-- and each accepted one yanks the whole pose graph. Raising the
-- acceptance threshold from 0.65 to 0.80 reduced but did not stop it
-- -- a later run still diverged to 7.4 m in both axes, and the
-- resulting map double-drew the maze, reporting 333 m2 of free space
-- in a world with roughly 260 m2 of it.
--
-- Setting optimize_every_n_nodes = 0 turns off the pose-graph
-- optimisation entirely, leaving local scan matching against the
-- odometry prior. That is a deliberate trade, and a reasonable one
-- here: the maze is 20 m across and bounded, so accumulated drift has
-- little distance in which to grow, whereas one false closure
-- corrupts the whole map irrecoverably. In a large or looping
-- environment this would be the wrong choice.
POSE_GRAPH.constraint_builder.min_score = 0.80
POSE_GRAPH.constraint_builder.global_localization_min_score = 0.75
POSE_GRAPH.optimization_problem.huber_scale = 1e2
POSE_GRAPH.optimize_every_n_nodes = 0

return options
