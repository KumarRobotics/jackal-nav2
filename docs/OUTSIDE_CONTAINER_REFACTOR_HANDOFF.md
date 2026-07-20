# Outside-container refactoring handoff

Date: 2026-07-20

This document is the handoff for the later Docker and `jackal_serial`
integration work. The ROS refactoring described here was performed inside the
existing robot-development container. No Dockerfile, image build, or container
runtime test was performed in this phase.

## Result of this phase

Jackal sensor and Nav2 autonomy are now owned by a standalone ROS 2 package and
sibling Git repository:

    /home/dcist/dcist_ws/src/jackal_nav2

The repository was initialized locally on branch `main`; it does not yet have a
remote or an initial commit. The package version is `0.1.0`.

The package has no dependency on `spine_multi_ros`. SPINE instead depends on
`jackal_nav2` and provides thin compatibility wrappers plus a separate
robot-side adapter. Sensors and navigation deliberately remain separate launch
operations; there is no aggregate bringup launch in the new package.

## Runtime architecture

The intended robot-side processes are:

1. `jackal_nav2/jackal_sensors.launch.py`
   - Ouster driver and component container
   - ZED driver, included now as a first-class dependency for future semantics
   - DLIO loaded into the Ouster component container
2. `jackal_nav2/jackal_navigation.launch.py`
   - GroundGrid as a standalone process
   - map/odometry and optional ZED static transforms
   - the tested Nav2 server subset and lifecycle manager
   - stamped velocity-to-Joy bridge for the current Jackal platform interface
3. The Jackal hardware/serial driver, supplied later by `jackal_serial`
4. Only when SPINE is required,
   `spine_multi_ros/jackal_spine_adapter.launch.py`
   - `jackal_autonomy_server.py`
   - throttled costmap publication
   - optional robot-side vision launch
5. MOCHA/Zenoh or the selected communications transport is managed separately.
   SPINE planning and task dispatch remain on the basestation.

This confirms that `jackal_autonomy_server.py` is the essential SPINE-specific
robot adapter. Its transitive requirements still include `teaming_msgs` and the
selected communications path. Vision services are optional at launch time.

## Standalone commands

Build from the workspace root:

    rosdep install --from-paths src --ignore-src -r -y
    colcon build --symlink-install --packages-up-to jackal_nav2
    source install/setup.bash

Run sensors and navigation in separate terminals:

    ros2 launch jackal_nav2 jackal_sensors.launch.py
    ros2 launch jackal_nav2 jackal_navigation.launch.py

Send YAML waypoints:

    ros2 run jackal_nav2 goto_nav2 /path/to/waypoints.yaml

Record a diagnostic bag:

    ros2 launch jackal_nav2 record_jackal.launch.py profile:=navigation

Recording profiles are `navigation`, `lidar`, `semantic`, and `full`.
`extra_topics` accepts a comma-separated list for experiment-specific topics.

Add SPINE without relaunching autonomy:

    ros2 launch spine_multi_ros jackal_spine_adapter.launch.py

The old SPINE Jackal sensor, navigation, static-transform, recording, helper,
and Joy-bridge entry points remain as deprecated wrappers so existing commands
can be migrated gradually.

## Topic and frame contract

| Producer | Default output | Consumer |
| --- | --- | --- |
| Ouster | `ouster/points` | DLIO and GroundGrid |
| Ouster | `ouster/imu` | DLIO |
| DLIO | `dlio/odom_node/odom` | GroundGrid, Nav2, recording |
| GroundGrid | `groundgrid/obstacle_cloud` | local/global Nav2 costmaps |
| Nav2 controller | `platform/cmd_vel` | Nav2 velocity smoother |
| Velocity smoother | `cmd_vel_smoothed` | `cmd_vel_to_joy` |
| Joy bridge | `joy` | current Jackal platform/serial interface |
| Nav2 BT navigator | `navigate_to_pose` | waypoint helper and SPINE adapter |
| SPINE adapter | `behavior_request` / `behavior_result` | basestation transport |

All topic names are launch arguments and are relative by default so a robot
namespace can be applied. Frames are also arguments; defaults are `map`,
`odom`, `base_link`, `os_lidar`, and `zed_camera_link`.

The navigation launch publishes `map -> odom` directly when
`robot_map_frame` is empty. For legacy multi-robot operation it publishes
`map -> <robot_map_frame> -> odom`. DLIO/GroundGrid provide or consume the
`odom -> base_link` and lidar-frame portions of the chain.

## Files and repositories changed

New `jackal_nav2` repository:

- `package.xml`, `setup.py`, `setup.cfg`, `LICENSE`, package resource marker
- `launch/jackal_sensors.launch.py`
- `launch/jackal_navigation.launch.py`
- `launch/nav2_servers.launch.py`
- `launch/jackal_static_transforms.launch.py`
- `launch/record_jackal.launch.py`
- `jackal_nav2/cmd_vel_to_joy.py`
- `jackal_nav2/goto_nav2.py`
- three migrated Jackal Nav2 YAML profiles under `config/`
- three migrated Jackal/Nav2 RViz profiles under `rviz/`
- unit tests for waypoint parsing and velocity conversion

`spine-multi` changes:

- Jackal-specific Nav2 configs and RViz files were removed after migration.
- Jackal sensor, aggregate navigation, static-transform, and recording launches
  are now thin wrappers around `jackal_nav2`.
- `goto_nav2.py` and `convert_vel_to_joy.py` are import shims.
- `jackal_spine_adapter.launch.py` launches only SPINE robot-side integration.
- `jackal_autonomy_server.py` now honors configurable Nav2 action, parameter
  server, goal frame, world frame, and robot base frame.
- `package.xml` declares the runtime packages used by the adapter and scripts.
- The shared `navigation_launch.py` was intentionally left unchanged because
  Husky and simulation launch files also consume it.

`groundgrid` changes:

- The ROS 2 component now declares topic, frame, transform-timeout, and height
  threshold parameters instead of hard-coding Jackal names.
- Its launch file starts a standalone node, removing the hidden requirement on
  the Ouster component container.

`DLIO` change:

- `dlio.launch.py` now actually returns its previously defined `namespace`
  launch argument, allowing the standalone sensor launch to pass it legally.

## Verification completed inside this container

- Python launch and script syntax compilation passed.
- All migrated YAML files and both edited package manifests parsed.
- Targeted `colcon build --symlink-install` passed for DLIO, GroundGrid,
  `jackal_nav2`, and `spine_multi_ros`.
- GroundGrid C++ rebuilt successfully with the new parameters.
- `colcon test --packages-select jackal_nav2` discovered and passed 6 tests.
- `ros2 launch ... --show-args` passed for all standalone launches, both main
  SPINE compatibility wrappers, and the SPINE adapter.
- A hardware-disabled Nav2 smoke test started all seven navigation servers and
  the lifecycle manager using the migrated/timestamp-rewritten parameter file.
  It was intentionally stopped after ten seconds.
- `rosdep check` reported that installed system dependencies were satisfied,
  but this container's rosdep database had no definition for the standard
  `ament_python` key. The package nevertheless builds with the installed Jazzy
  toolchain. Ensure the external image provides ROS Jazzy's Python ament tools.

No sensors, motors, serial device, live navigation goal, bag playback, Docker
build, or Docker runtime was tested in this phase.

## Next phase: unified Docker image

The outside-container agent should:

1. Give `jackal_nav2` a Git remote and pin a revision in the image's repository
   import mechanism alongside `spine-multi`, `jackal_serial`, GroundGrid, DLIO,
   Ouster ROS, and ZED ROS 2 wrapper.
2. Preserve the modular launch model. The image entrypoint should source ROS and
   the workspace; it should not automatically combine sensors, navigation,
   SPINE, and serial into one opaque process.
3. Install ROS dependencies from package manifests. Explicitly account for the
   ZED SDK/CUDA compatibility matrix, Ouster dependencies, Nav2 plugins,
   SpatioTemporalVoxelLayer, and rosbag2 CLI/storage plugins.
4. Integrate `jackal_serial` as the hardware layer. Verify its expected Joy
   topic, QoS, axis mapping, enable/deadman semantics, device path, baud rate,
   udev rules, and Linux group permissions against `cmd_vel_to_joy` before
   changing either interface.
5. Add runtime access for lidar, ZED, serial, GPU, host networking/DDS, and the
   bag output volume. Do not bake site-specific device paths into ROS code.
6. Keep SPINE optional. The base autonomy image should run the two standalone
   launches plus `jackal_serial`; SPINE-enabled deployment adds the adapter and
   communications configuration.
7. Pin repository revisions after the robot acceptance test so the image is
   reproducible.

## Hardware/image acceptance checklist

- Ouster point cloud and IMU publish at expected rates.
- ZED image, camera-info, and depth topics are present and timestamped.
