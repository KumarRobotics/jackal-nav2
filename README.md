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
