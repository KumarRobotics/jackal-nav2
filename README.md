# jackal_nav2

Standalone ROS 2 Jazzy sensor and Nav2 autonomy bringup for the DCIST Clearpath
Jackal.

## Runtime layout

Run sensors and navigation in separate terminals so either side can be restarted
and inspected independently:

    ros2 launch jackal_nav2 jackal_sensors.launch.py
    ros2 launch jackal_nav2 jackal_navigation.launch.py

The sensor launch starts Ouster, ZED, and DLIO by default. The navigation launch
starts GroundGrid, the map/odometry static transforms, the tested Nav2 server set,
and the physical-velocity-to-Joy bridge. The Jackal platform driver that consumes
/joy must already be running.

Send a waypoint file with:

    ros2 run jackal_nav2 goto_nav2 /path/to/waypoints.yaml

The waypoint format contains a frame_id and a waypoints list. Each waypoint accepts
x, y, and an optional yaw value.

Record a navigation-focused diagnostic bag with:

    ros2 launch jackal_nav2 record_jackal.launch.py profile:=navigation

Available profiles are navigation, lidar, semantic, and full. Use
```
extra_topics:=/topic/one,/topic/two to append experiment-specific topics.
```

## Workspace dependencies

The surrounding workspace or robot image is responsible for supplying:

- ROS 2 Jazzy and Navigation2, including MPPI, rotation shim, SmacPlanner2D,
  velocity smoother, and SpatioTemporalVoxelLayer.
- ouster_ros
- zed_wrapper
- direct_lidar_inertial_odometry
- groundgrid

From the workspace root:

    (This needs to be tested!!) rosdep install --from-paths src --ignore-src -r -y
    colcon build --symlink-install --packages-up-to jackal_nav2

## Motion analysis

The optional `motion_stats` node compares the raw Nav2 command, velocity-smoother
output, and measured odometry. Enable it with:

    ros2 launch jackal_nav2 jackal_navigation.launch.py \
      start_motion_analysis:=true

It updates the current run's artifacts every 30 seconds and once more on clean
shutdown. Each run is stored under `plots/run_<timestamp>/` in the source package
(or `~/.ros/jackal_nav2/plots` when no source workspace can be found):

- `velocity_and_acceleration.png` overlays command-only linear/angular velocities
  and filtered accelerations.
- `jerk_profile.png` overlays linear and angular jerk for the raw Nav2 and smoothed
  commands. Jerk is the finite difference of the filtered acceleration. Each panel
  uses the 92nd percentile of absolute jerk as a symmetric display limit and
  annotates how many larger spikes were omitted.
- `velocity_tracking_error.png` plots measured odometry minus the latest smoothed
  command.
- `summary.json` includes velocity, acceleration, jerk, stationary-time, distance,
  rotation, sample-rate, and tracking-error statistics. Jerk statistics are
  command-only.
- The two CSV files retain raw, filtered, and jerk samples for custom analysis.

Useful launch overrides are:

    motion_analysis_output_directory:=/path/to/output
    motion_analysis_report_interval:=30.0
    motion_analysis_acceleration_filter_alpha:=0.25

The filter alpha is in `(0, 1]`; `1.0` gives unfiltered finite differences. Topic
names follow the existing `nav_cmd_vel_topic`, `smoothed_cmd_vel_topic`, and
`odom_topic` launch arguments. Acceleration samples outside `[-1, 2]` are retained
in the CSV and summary statistics but omitted from both acceleration plot panels.
Odometry is retained in CSV/JSON statistics and the tracking-error plot, but it is
omitted from all motion-profile panels. Odometry jerk is neither calculated nor
plotted.
Jerk outliers remain in `velocity_samples.csv` and `summary.json`; only their plot
points are omitted.
