# jackal_nav2

Standalone ROS 2 Jazzy sensor and Nav2 autonomy bringup for the DCIST Clearpath
Jackal.

This repository is packaged as a sub-module for the main [jackal autonomy image](https://github.com/KumarRobotics/jackal-autonomy) repository

## Runtime layout

Run sensors and navigation in separate terminals so either side can be restarted
and inspected independently:

    ros2 launch jackal_nav2 jackal_sensors.launch.py
    ros2 launch jackal_nav2 jackal_navigation.launch.py

The sensor launch starts Ouster, ZED, and DLIO by default. The navigation launch
starts GroundGrid, the map/odometry static transforms, the tested Nav2 server set,
and the physical-velocity-to-Joy bridge. The Jackal platform driver that consumes
/joy must already be running.

The sensor launch also publishes `os_lidar -> zed_camera_link` when ZED is enabled.
`config/zed_mount.yaml` defaults to `mode: calibration`: it composes the retained
LiDAR-to-left-optical calibration with the inverse of the camera root-to-optical
transform from the ZED description. DLIO owns `odom -> base_link -> os_lidar`;
the ZED description owns all internal camera frames. This keeps TF consistent
with fusion without changing the calibration, DLIO or scan-time projection code.

The profile specifies the calibration file, parent/child frames, ZED description
package/path, camera name/model and custom baseline. These must match the running
camera. The historical `x_m`, `y_m`, `z_m`, `roll_rad`, `pitch_rad`, `yaw_rad`
values apply only to explicit `mode: manual`; set its parent accordingly. They are
not used as a fallback when calibration loading fails. The current DLIO base
origin convention is preserved, not physically recalibrated.

To publish only the mounting edge:

```bash
ros2 launch jackal_nav2 zed_mount.launch.py
ros2 run tf2_ros tf2_echo base_link zed_left_camera_optical_frame
```

The complete lookup also requires DLIO and the ZED internal TF publisher. Use one
owner for the mounting edge. If the standalone publisher is already running, use
`publish_zed_mount_tf:=false` with `jackal_sensors.launch.py`, or stop it before
restarting sensors. Navigation's `publish_zed_static_tf` defaults to false; if
explicitly enabled it uses the same profile. All three launch paths accept
`zed_mount_config:=/absolute/path/to/config.yaml`. Calibration mode forbids
relabeling its LiDAR parent as a body frame. Restart consumers when replacing an
old base-to-camera publisher so their TF caches do not retain the old parent.

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

## Motion analysis

The optional `motion_stats` node compares the raw Nav2 command, velocity-smoother
output, and measured odometry. Enable it with:

    ros2 launch jackal_nav2 jackal_navigation.launch.py \
      start_motion_analysis:=true

It updates the current run's artifacts every 30 seconds and once more on clean
shutdown. Each run is stored under `plots/run_<timestamp>/` in the source package
(or `~/.ros/jackal_nav2/plots` when no source workspace can be found):
