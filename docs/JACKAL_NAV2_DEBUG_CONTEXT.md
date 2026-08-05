# Jackal Nav2 / MPPI Debugging Context

Last updated: 2026-08-04

## Purpose

This file is the durable context primer for continued debugging of the Clearpath
Jackal Nav2 controller, MPPI behavior, rotation shim, and `cmd_vel`-to-Joy bridge.
Give this file to a new chat before making additional changes.

The immediate objective is reliable autonomous waypoint following at selectable
speeds up to approximately 1.0 m/s, with smooth straight-line motion and reliable
180-degree turns. Velocity scaling must happen through physically meaningful Nav2
limits and bridge calibration, not arbitrary multipliers applied after Nav2.

## Current status at a glance

- The DLIO odometry twist-frame correction is now **applied and robot-verified**.
  The direction-specific return-leg start/stop motion disappeared after loading
  the rebuilt DLIO node.
- SmacPlanner2D and the replan-on-invalid BT remain active and produced satisfactory,
  stable global paths during the preceding test series.
- The position-only one-shot-shim architecture is now **applied and robot-verified**.
  Initial heading changes are handled by an in-place shim turn, ordinary endpoint
  yaw is ignored, and the resulting Jackal motion is considered reasonable.
- A 0.5 m XY tolerance still produced occasional overshoot in the latest physical
  test. The user increased `PositionGoalChecker.xy_goal_tolerance` to 1.0 m; four
  consecutive shuttle legs then terminated successfully before a fifth was manually
  canceled. This 1.0 m outdoor waypoint acceptance radius is intentional and should
  be retained.
- The non-terminating return had a different requirement: the mission sent yaw zero
  at every endpoint, so the Jackal arrived at `(0, 0)` facing approximately pi away
  from the requested yaw. Nav2 correctly continued rather than reporting success.
- The direct-MPPI goal-yaw fix is **applied and tested; unsuccessful as a complete
  solution**. The Jackal still drove forward arcs while trying to turn, repeatedly
  aborted MPPI, and depended on recovery spins before goal completion.
- ~~The remaining motion-quality issue is a mild straight-line jerk/pulse resembling
  gentle start/stop behavior.~~ **Resolved in the 2026-07-28 physical test.** The
  separate `/platform/cmd_vel` publisher collision caused the severe multi-second
  start/stop behavior, and restoring private `/cmd_vel_nav` command flow removed it.
  Reducing the velocity smoother's linear acceleration/deceleration limits from
  `0.75 / -1.0` to `0.5 / -0.5 m/s²` then removed the remaining super-jerky motion.
  That July motion-quality problem is closed. The current smoother values have since
  changed to `0.3 / -1.0 m/s²` and require their own safety, stopping-distance, and
  goal-overshoot validation before being treated as production-final.
- The missing local/global obstacle maps are **resolved in a stationary live-sensor
  test**. Fully qualified STVL topics restored obstacle marking and raw-scan clearing;
  moving obstacle-avoidance behavior still requires a controlled field test.
- The standalone-image waypoint regression is **resolved in a robot test**. The old
  image contains Nav2 1.3.7 and the new image contains Nav2 1.3.12. An upstream MPPI
  acceleration-constraint change made the asymmetric rollout limits reverse-biased,
  causing 9-12 seconds of start/stop indecision and sometimes full-leg reverse travel.
- The successful Nav2 1.3.12 workaround uses symmetric MPPI rollout acceleration
  limits of `6.0 / -6.0 m/s²`, `vx_min: 0.0`, and `PathAngleCritic.mode: 0`.
  Controller-selected reverse is intentionally disabled; explicit behavior-tree
  `BackUp` remains available through the velocity smoother. This is separate from
  the 2026-07-28 command-topic collision and physical-smoother jerk regression.
- The slow ZED/Ouster/DLIO launch is **resolved and robot-verified**. With
  `ROS_AUTOMATIC_DISCOVERY_RANGE=SUBNET`, Fast DDS discovered another robot using
  the same ROS domain and spent tens of seconds processing the foreign graph while
  ZED created its publishers. Setting the range to `LOCALHOST` in
  `autonomy_ws/bashrc` restored normal sensor startup. This deliberately prevents
  discovery of off-machine ROS nodes; use a unique domain ID instead if distributed
  robot/base-station communication is required.

- Explicit inspection-yaw behavior in `jackal_autonomy_server.py` remains deliberately
  deferred.
- A direction-specific odometry defect was found in DLIO: the message declares
  `child_frame_id: base_link` but published `state.v.lin.w`, its world-frame linear
  velocity. Nav2 consumes that value without rotating it. Thus forward motion toward
  negative odom X was reported to MPPI as negative base-frame `vx`, conflicting with
  forward-only `vx_min: 0.0` only on return legs.
- DLIO now rotates world velocity into base_link before publishing Odometry twist.
  This correction builds successfully and its robot test fixed the original
  direction-dependent oscillation.
- Occasional message-filter drops remain deferred unless they persist after the
  odometry correction.
- The previous rotation-shim configuration remains preserved.

## System and command path

The relevant command path is:

1. The rotation shim emits zero-linear-velocity turns for large initial path-heading
   errors; otherwise its internal MPPI controller generates the Twist.
2. Nav2 publishes `/cmd_vel_nav`.
3. `nav2_velocity_smoother` publishes `/cmd_vel_smoothed`.
4. `convert_vel_to_joy.py` converts physical Twist commands to normalized Joy axes.
5. `jackal-serial`, running in a different container, converts Joy/controller data
   into Jackal drive commands.
6. DLIO publishes odometry on `/dlio/odom_node/odom` and dynamic odom-to-base TF.
   Odometry pose is in `odom`; twist is now correctly expressed in `base_link`.

Manual teleoperation produces `/jackal_velocity_controller/cmd_vel` in the
`jackal-serial` container. For autonomous Joy-bridge calibration, compare the
autonomous physical command entering the bridge with steady-state measured DLIO
odometry. Manual teleoperation is useful for independently finding the robot's
full-stick physical speed, but is not a substitute for checking the autonomous
end-to-end command path.

## Main files

- `ros/spine_multi_ros/config/nav2_jackal_stvox_mppi_tuning.yaml`
  - Active experimental Nav2 configuration: a one-shot initial-heading rotation
    shim wraps MPPI; ordinary goal completion is position-only.
- `ros/spine_multi_ros/config/nav2_jackal_stvox.yaml`
  - Preserved previous rotation-shim configuration for later A/B testing.
- `ros/spine_multi_ros/scripts/convert_vel_to_joy.py`
  - Converts physical Twist velocities to normalized Joy axes.
- `ros/spine_multi_ros/launch/jackal_launch_all.launch.py`
  - Launches the Nav2 stack and passes the Nav2 configuration to the Joy bridge.
- `../../../groundgrid/src/GroundGridNode.cpp`
  - Produces `/groundgrid/obstacle_cloud`; now uses timestamp-matched transforms
    before publishing its odom-frame clouds.
- `ros/spine_multi_ros/data/perch_waypoints.yaml`
  - Current straight-line shuttle mission and waypoint headings.
- `../DLIO/src/dlio/odom.cc` and `../DLIO/include/dlio/odom.h`
  - DLIO odometry and TF publication changes. From the workspace root these are
    `DLIO/src/dlio/odom.cc` and `DLIO/include/dlio/odom.h`.

There is also a modified `ros/spine_multi_ros/launch/record_jackal.launch.py` and a
modified DLIO `cfg/params.yaml`. Those are not known to be part of this controller
fix and must be treated as pre-existing/user-owned changes unless confirmed
otherwise.

## Current active Nav2 configuration

The following summarizes the important values currently present in
`nav2_jackal_stvox_mppi_tuning.yaml`, which is selected by the Jackal aggregate
launch file. Always re-read the file before relying on this summary.

### Controller server and goal checker

- Controller frequency: 20 Hz.
- Odometry topic: `/dlio/odom_node/odom`.
- Odom duration: 0.3 s.
- Ordinary goals load `nav2_controller::PositionGoalChecker`.
- XY tolerance: 1.0 m, intentionally selected for outdoor long-distance waypoints.
- Goal orientation is intentionally ignored; no yaw tolerance applies in this mode.
- `stateful: false`; success reflects the current position measurement.

### Controller selection

- `FollowPath` loads `nav2_rotation_shim_controller::RotationShimController`, whose
  primary controller is `nav2_mppi_controller::MPPIController`.
- The shim engages only for a changed goal whose sampled initial path heading differs
  by more than 0.785 rad, emits zero linear velocity, and disengages below 0.15 rad.
- `rotate_to_heading_once: true` prevents same-goal replans from restarting rotation.
- `rotate_to_goal_heading: false` prevents placeholder yaw from causing endpoint turns.
- `closed_loop: false` retains the robot-tested open-loop ramp to 0.6 rad/s.
- `use_path_orientations: false` derives heading from SmacPlanner2D path geometry.

### MPPI

- Time steps: 56 at 0.05 s, for a 2.8 s prediction horizon.
- Batch size: 2000.
- `open_loop: true`.
- Linear velocity limits: `vx_min: 0.0`, `vx_max: 1.0 m/s`.
- Angular velocity maximum: 0.75 rad/s.
- Linear rollout acceleration maximum/minimum: `6.0 / -6.0 m/s²`.
- MPPI angular rollout acceleration: 6.0 rad/s², allowing candidates to reach the
  proven breakaway velocity while odometry still reads zero.
- Angular sampling standard deviation: 0.4 rad/s.
- Temperature: 0.4; gamma: 0.015.
- Motion model: differential drive.
- MPPI retry attempt limit: 1.
- `GoalAngleCritic` is removed because ordinary goal yaw is not a task requirement.
- `PathAngleCritic` keeps weight 8.0 and uses mode 0, enforcing forward path
  alignment for ordinary MPPI tracking.
- A `VelocityDeadbandCritic` remains enabled with deadbands `[0.0, 0.0, 0.04]`
  and weight 35. Only angular breakaway is encouraged; zero linear velocity is no
  longer penalized during a stationary turn.

`vx_min: 0.0` and `PathAngleCritic.mode: 0` deliberately prevent MPPI from selecting
reverse trajectories in Nav2 1.3.12. `PreferForwardCritic` remains as a soft forward
preference, while the hard velocity bound and path-angle mode remove the ambiguous
reverse solution. Controller `failure_tolerance` remains 0.0 so all-trajectories-
collide exits immediately into behavior-tree recovery instead of publishing patience-
window zeros. Explicit behavior-tree `BackUp` commands can still pass through the
velocity smoother's negative X bound.

### Velocity smoother

- Frequency: 50 Hz.
- Feedback mode: open loop.
- Maximum velocities: `[1.0, 0.0, 0.75]`.
- Minimum velocities: `[-0.5, 0.0, -0.75]`; explicit backup-recovery commands can
  pass through even though ordinary MPPI reverse is disabled by `vx_min: 0.0`.
- Maximum accelerations: `[0.3, 0.0, 1.5]`.
- Maximum decelerations: `[-1.0, 0.0, -1.5]`.
- Odometry topic: `/dlio/odom_node/odom`.

At the 50 Hz smoother rate, these linear limits constrain each output update to
approximately `+0.006 m/s` while accelerating and `-0.020 m/s` while decelerating.
These are the physical output slew limits; the MPPI `6.0 / -6.0 m/s²` values only
bound optimizer rollouts and are the confirmed Nav2 1.3.12 workaround.

The optimizer and smoother acceleration envelopes intentionally do not currently
match. Preserve the robot-tested MPPI workaround, validate stopping and obstacle
avoidance with the physical `0.3 / -1.0 m/s²` smoother limits, and then test whether
a smaller symmetric MPPI pair or a corrected Nav2 release retains the good behavior.

### Costmaps

- Local and global footprints now match at a 0.25 m half-width square.
- The rolling local costmap now uses `odom`, avoiding the map-to-odom transform in
  the controller-critical observation path. Its previous 10-second transform
  tolerance was reduced to 0.3 seconds.
- Local and global inflation radii remain 0.5 m.
- Both STVL instances mark from the absolute `/groundgrid/obstacle_cloud` topic and
  clear free space from the absolute raw `/ouster/points` topic. The corrected live
  graph shows one marking subscription from each costmap and no private namespaced
  ghost topics. Footprint clearing remains explicit.
- The local voxel decay is now 1.0 second to remove transient/self observations
  quickly; the global layer remains 5.0 seconds.
- Global costmap width and height remain 100 m. At 0.25 m resolution it is about
  400 by 400 cells. A 30-40 m workspace may still be sufficient for this test.

### Global planning and replanning

- `GridBased` now loads `nav2_smac_planner::SmacPlanner2D` instead of NavFn.
- Smac uses full-resolution cost-aware A*, a cost travel multiplier of 2.0, and
  its path smoother. This favors corridor clearance without shrinking inflation.
- NavigateToPose now uses Nav2's recovery BT that replans only when the goal changes
  or `IsPathValid` rejects the current path. It no longer replaces a valid path at
  1 Hz.
- Inflation remains 0.5 m. The configured square footprint has an approximately
  0.354 m circumscribed radius, leaving only about 0.146 m of additional inflation;
  reducing it is not the selected fix.

### Joy bridge

`convert_vel_to_joy.py` no longer applies arbitrary multipliers to the incoming
linear and angular commands. It now:

- Accepts a configurable Twist input topic, currently `/cmd_vel_smoothed`.
- Uses `linear_speed_at_full_axis: 2.0 m/s`.
- Uses `angular_speed_at_full_axis: 0.75 rad/s`.
- Divides the physical command by those calibrated full-axis values.
- Clamps each Joy axis to `[-1, 1]`.
- Publishes at 50 Hz.

These two full-axis values came from the `jackal-serial` controller mapping and are
not yet confirmed by steady-state odometry measurements on this particular robot.
The Joy mapping also has a roughly 0.04 normalized deadzone, corresponding to about
0.08 m/s linear with the current calibration and 0.03 rad/s angular.

`jackal_launch_all.launch.py` passes the selected Nav2 YAML file to the Joy bridge.
Some vision/autonomy launch entries are also currently commented out; do not assume
that those comments were made as part of MPPI tuning.

### Waypoints

`perch_waypoints.yaml` now uses yaw zero at every endpoint. In the ordinary navigation
mode this is explicitly an ignored placeholder. Each new path's geometry, rather than
the preceding goal quaternion, determines whether the shim performs an initial turn.

## Experiment history

### 2026-08-04 slow sensor launch — applied and tested

Status: **Applied and tested; confirmed fix on the robot**.

- Symptom: ZED, Ouster, and DLIO took much longer to launch in the standalone image.
  ZED was the most visible delay and was still creating/activating publishers after
  Ouster and DLIO had begun running.
- Controlled image tests showed that the physical sensor initialization was not the
  cause. ZED camera opening remained approximately 2.9 seconds in both the old and
  new images, and Ouster initialization remained approximately 12.6-14 seconds.
- Fast DDS with `ROS_DOMAIN_ID=2` and subnet discovery produced repeated roughly
  3.07-second pauses while ZED declared parameters, services, and image publishers.
  The same image started normally on an isolated domain, and a Cyclone DDS A/B test
  also started normally on domain 2. Restricting ZED to raw image transport reduced
  publisher count but did not remove the pauses, so image transports amplified the
  problem rather than caused it.
- The decisive field observation was that topics and nodes remained visible with no
  local stack running. Another robot on the subnet was using the same domain ID.
  Fast DDS therefore discovered and processed that robot's graph during local sensor
  startup.
- `autonomy_ws/bashrc` now sets:

  ```bash
  export ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST
  ```

  Sensor startup returned to normal after this change, confirming foreign subnet
  discovery as the operational root cause.
- `LOCALHOST` is appropriate while every required ROS process runs on this computer,
  including processes in host-networked containers. It intentionally hides ROS nodes
  on other machines. If a base station or another computer must communicate with the
  robot, assign this robot a unique `ROS_DOMAIN_ID` and configure the intended peers
  to use it before restoring `SUBNET` discovery or defining explicit static peers.
- The sensor launch file required no permanent change. Its actions remain concurrent,
  so DLIO can still log activity before ZED finishes, but the pathological delay is
  removed.

### Initial behavior: commands too small

- `/cmd_vel_smoothed` initially showed about 0.0133 m/s linear and 0.0082 rad/s
  angular. The robot did not move.
- A later test showed about 0.044 m/s on both `/cmd_vel_nav` and
  `/cmd_vel_smoothed`, still below the effective drive/deadband threshold.
- After controller and bridge changes, initial linear speed was about 0.14 m/s and
  ramped to 0.6 m/s in approximately 2-3 seconds. The robot moved.

### Straight-line jerk/pulsing

Status: **Resolved in the 2026-07-28 physical test; retained as history.**

- During the 10 m straight, the wheels periodically appeared to stop after a short
  acceleration and start again. Motor sound also suggested periodic loading.
- The inspected Nav2 command topics were identical and did not show corresponding
  drops, so the pulsing may occur downstream of Nav2.
- In `KumarRobotics/jackal-serial`, the controller manager was found configured at
  10 Hz. The velocity controller's `publish_rate: 50` concerns odometry/TF output;
  it does not necessarily make the serial drive command update at 50 Hz. The serial
  node appears to write a drive packet on joint-state callbacks.
- The user later changed the deployed controller-manager update rate to 50 Hz. This
  is **applied but unverified**: the next serial-container session must confirm the
  running parameter, measured `/joint_states` rate, and callback-driven serial-write
  cadence are actually near 50 Hz.
- Later testing separated two symptoms. The severe multi-second stop/start motion was
  caused by Nav2 and Clearpath teleop both writing the post-mux `/platform/cmd_vel`
  topic; restoring `/cmd_vel_nav` as Nav2's private pre-smoother topic fixed that
  command collision. With command ownership corrected, lowering the smoother's linear
  acceleration/deceleration limits to `0.5 / -0.5 m/s²` removed the remaining
  super-jerky motion. The prior serial-rate and TF hypotheses were reasonable from
  the evidence then available, but are no longer required to explain this symptom.

### First rotation-shim test: TF and missed-loop failures

The first shim attempt did not rotate. Logs showed TF lookup into the future and
controller-loop rates near 9-11 Hz instead of the desired 20 Hz. The TF timestamp
gap was small, but lookup waits consumed enough time to miss controller deadlines.

DLIO was changed so dynamic odom-to-base TF is published from the 100 Hz pose timer
using the IMU timestamp, rather than only during slower LiDAR-scan publication.
The package was rebuilt successfully. The next robot test no longer showed the TF
extrapolation or controller-loop-rate warnings. This is a confirmed fix.

### Second rotation-shim test: command stuck at 0.075 rad/s

- Linear command was zero, showing the shim was actively requesting in-place turn.
- Angular command remained approximately -0.075 rad/s and did not ramp to -0.6.
- Cause: closed-loop acceleration ramping repeatedly started from near-zero measured
  odometry because the command could not overcome static friction.
- Changing `closed_loop` to `false` allowed the angular command to ramp and the
  robot successfully completed an in-place rotation. This is a confirmed fix.

### Subsequent mission: large oscillations and planning failures

After the first successful 180-degree turn:

- The return leg developed severe forward/reverse oscillation, described as about
  2 m forward followed by 1 m backward.
- The robot reached the start but could not complete the next 180-degree turn. It
  oscillated between small right and left rotations and/or linear motions.
- The controller logged two optimizer resets followed by
  `Optimizer fail to compute path`.
- NavFn logged `Failed to create a plan from potential when a legal potential was
  found`, failing from approximately `(6.93, -0.12)` to `(0.00, 0.00)` with 0.5 m
  tolerance.
- The behavior tree cleared the entire global costmap after the planning failure.
- The planner loop marginally missed 1 Hz, running at approximately 0.927 Hz.

Applied responses included:

- Setting MPPI `vx_min` to 0.0.
- Enabling `rotate_to_heading_once` to avoid restarting a heading shim for every
  frequent replan.
- Reducing the global costmap from 1000 m to 100 m.
- Setting global inflation radius to 0.5 m.
- Using alternating waypoint yaw values.
- Keeping the XY goal tolerance at 0.5 m.

The latest test with these changes still did not work. No exact new logs or command
traces were supplied, so the next session must collect evidence before making more
gain changes.

### 2026-07-14 direct-MPPI robot test — applied and tested

Status: **Applied and tested; turning succeeded, mission stability failed**.

- The Jackal reached the first waypoint and completed the turn with MPPI alone.
  This validates the current 0.6 rad/s angular limit, 0.4 rad/s sampling standard
  deviation, 6.0 rad/s² rollout acceleration, and stronger angle critics well
  enough to retain them for the next test.
- After turning, the Jackal again oscillated forward/reverse. Setting only MPPI
  `vx_min` to zero converted the behavior to stop/forward pulsing but did not fix it.
- Changing waypoint goal yaw had no effect.
- The repeated core log sequence was:
  `Optimizer reset`, `Optimizer reset`, `Optimizer fail to compute path`. New paths
  were passed to the controller around 1 Hz between failures.
- Local and global costmap filters also reported an `odom` cloud stamp earlier than
  all data in the TF cache.
- Exact Nav2 1.3.7 source inspection showed that this MPPI failure path is reached
  when CostCritic marks all trajectories as colliding. Controller-server source
  also showed that `failure_tolerance: 0.3` publishes zero command during brief
  failures, explaining valid-motion/zero-command pulsing.
- Conclusion: **critic gain and goal-yaw tuning are rejected as the next fix**. The
  local collision data must be corrected first.

### 2026-07-14 collision-data fix — applied and tested, unsuccessful

Status: **Applied and tested; did not fix stop/forward motion**.

- The robot still pulsed only on 10-to-0 return legs. Smoothed linear X rose to
  approximately 0.23 m/s and repeatedly returned to zero instead of reaching the
  prior approximately 0.7 m/s.
- The local/global costmaps appeared to contain usable free-space routes.
- Raw `/ouster/points` observations were occasionally dropped at about 0.6 seconds
  old, but the sparse drops did not correlate with the direction-only pattern.
- NavFn failed to extract a path despite finding legal potential immediately after
  the first endpoint, cleared the global costmap, and then produced paths at 1 Hz.
- In the longer mission MPPI later failed all trajectories, causing FollowPath to
  abort and the BT to run spin, local-costmap clearing, and wait recoveries.
- Conclusion: the sensor/costmap changes remain reasonable but are **rejected as a
  complete solution**. Global path generation and churn are now the primary test.

### 2026-07-14 return-path stability fix — applied and tested, partial

Status: **Applied and tested; planning improved, motion failure remained**.

- SmacPlanner2D started and generated satisfactory paths. The NavFn extraction
  error disappeared.
- Replan-on-invalid behavior stopped continuous path replacement; the first return
  received two early path updates and then no 1 Hz churn.
- Both Nav2 command topics still zeroed during return-leg stops.
- MPPI failed all trajectories at approximately 15 and 31 seconds into the first
  return, causing controller aborts and costmap recovery.
- The return eventually succeeded after roughly 97 seconds; the adjacent outbound
  legs completed in roughly 18 seconds.
- Endpoint yaw set/unset tests behaved the same.
- Conclusion: Smac and conditional replanning remain as improvements, but they are
  **rejected as the complete stop/start fix**.

### 2026-07-14 DLIO odometry twist-frame fix — applied and tested

Status: **Applied and tested; confirmed fix on the robot**.

- Confirmed `nav_msgs/Odometry` requires twist in `child_frame_id`. DLIO sets that
  frame to `base_link` but assigned linear twist from `state.v.lin.w`, explicitly
  its world-frame velocity. Angular velocity was already correctly body-frame.
- Confirmed Nav2 OdomSmoother averages and forwards the twist components without a
  frame transform, and MPPI seeds every rollout from that measured linear X.
- Therefore outbound forward travel along positive odom X looked valid, while
  return forward travel along negative odom X looked like negative base-frame `vx`.
  This exactly matches the direction-only failure and conflicts with `vx_min: 0.0`.
- `publishPose()` now rotates `state.v.lin.w` through the inverse current orientation
  and publishes the resulting body-frame X/Y/Z components.
- `colcon build --packages-select direct_lidar_inertial_odometry --symlink-install`
  completed successfully.
- The subsequent robot test eliminated the direction-specific start/stop behavior
  on the return journey. This confirms the incorrectly framed twist was the root
  cause of that failure.
- A distinct goal-overshoot issue is now visible: the robot passes a waypoint by
  approximately 2-3 m and then returns toward it before completion.

### 2026-07-14 goal-termination diagnosis and fix — applied and tested, unsuccessful

Status: **Applied and tested; unsuccessful as a complete fix**.

- The post-odometry-fix log shows the first action started at `(0.00, 0.00)`, reached
  `(10.00, 0.00)`, and logged `Reached the goal!`. The immediately following action
  began from `(9.72, -0.39)`, whose 0.48 m radial error is inside the configured
  0.5 m tolerance. This directly disproves an undersized XY tolerance on that leg.
- Every active waypoint specified yaw zero. That matches the positive-X arrival but
  differs by approximately pi from the negative-X arrival at `(0, 0)`. With
  `yaw_goal_tolerance: 0.2`, the return action must not succeed on position alone.
- Nav2 1.3.7 source confirms `SimpleGoalChecker.stateful: true` permanently disables
  its XY recheck after first entering tolerance while it waits for yaw. It also
  confirms `VelocityDeadbandCritic` is active everywhere; the former 0.09 m/s X
  deadband and weight 35 penalized every zero-linear-speed rollout during endpoint
  rotation by about `0.09 * 2.8 * 35 = 8.82` cost units.
- Changed the active goal checker to `stateful: false`, retained the actual 0.5 m XY
  and 0.2 rad yaw requirements, removed only the linear deadband penalty, and kept
  the 0.04 rad/s angular breakaway penalty.
- Restored semantically correct shuttle headings: pi at `x=10`, zero at `x=0`.
  `goto_nav2.py` always sends a quaternion, so a missing waypoint yaw previously
  defaulted to zero; it did not mean orientation was ignored.
- The same log contains repeated `Optimizer fail to compute path`, control-loop-rate
  warnings, and recovery actions during the return. Those events began immediately
  after submitting a path behind the yaw-zero robot. The new final headings should
  remove that transition, but this remains a robot-test assertion, not a confirmed
  fix.

### 2026-07-14 direct-MPPI endpoint-yaw retest — applied and tested, unsuccessful

Status: **Applied and tested; rejected as the general goal-handling architecture**.

- The first `(0, 0) -> (10, 0)` goal began at `1784055218.920`, encountered four
  `Optimizer fail to compute path` aborts near the endpoint, used a successful
  recovery spin, and reached the goal at `1784055271.565`. The next action began at
  `(10.45, -0.06)`, confirming about 0.45 m X overshoot and a 52.6-second action.
- The return began at `1784055271.644`, repeatedly aborted MPPI, failed one spin for
  collision, failed backup after a TF extrapolation/current-footprint error, failed
  another spin by timeout, and finally reached the goal at `1784055383.759`. The
  next action began at `(-0.29, 0.27)`, after approximately 112.1 seconds.
- Physical observation confirmed MPPI tried to achieve the 180-degree goal-yaw change
  through forward arcs rather than a stationary turn. More GoalAngleCritic gain is
  rejected as the primary fix because controller role separation is more general.
- Proposed next configuration, not yet applied: transplant the previously verified
  rotation shim into the current active tuning file; use it only for one-shot initial
  path alignment; make ordinary goals position-only; and restore bounded reverse
  motion in MPPI and the velocity smoother. Do not switch wholesale to the stale
  preserved shim YAML because it lacks later costmap, Smac, BT, and DLIO-era fixes.

### 2026-07-14 position-only one-shot-shim architecture — applied and tested

Status: **Applied and robot-tested; retained as the ordinary-navigation design**.

- Replaced the active `SimpleGoalChecker` with non-stateful `PositionGoalChecker`.
  Ordinary waypoint quaternions no longer affect success.
- Wrapped the existing active MPPI configuration with `RotationShimController`; did
  not switch to the stale preserved shim YAML, so all newer costmap, Smac, BT, bridge,
  and DLIO-era settings remain intact.
- The shim uses the previously proven 0.6 rad/s open-loop rotation, 0.785 rad engage
  threshold, 0.15 rad disengage threshold, 0.5 m path sample, one-shot behavior, and
  no goal-heading rotation. Its rotation command has zero linear velocity.
- Restored MPPI `vx_min: -0.5`, smoother minimum X `-0.5`, and PathAngleCritic mode 1.
  PreferForwardCritic remains enabled so reverse is permitted but not gratuitous.
- Removed GoalAngleCritic from the active critic list and returned all shuttle yaw
  values to zero.
- The Jackal now performs the necessary initial in-place turn and otherwise moves
  reasonably. No random mid-straight shim rotation was reported.
- Per user direction, no explicit inspection-yaw implementation or change to
  `jackal_autonomy_server.py` was made; that behavior is deferred.

### 2026-07-14 1.0 m goal-tolerance shuttle — applied and tested

Status: **Applied and tested; successful and intentionally retained**.

- At 0.5 m XY tolerance, the robot occasionally overshot before it could enter the
  acceptance region. The user changed the active tolerance to 1.0 m for outdoor,
  long-distance waypoints where plus or minus 1 m is acceptable.
- The supplied log shows five action starts. The first four completed normally, and
  the fifth was deliberately canceled by Ctrl-C after 5.3 seconds. Each following
  leg began at `(9.07, -0.17)`, `(0.76, 0.61)`, `(9.09, 0.32)`, and `(0.89, 0.34)`
  for alternating `(10, 0)` and `(0, 0)` goals.
- Each completed leg emitted `Reached the goal!`, `Optimizer reset`, and
  `Goal succeeded`; there was no goal-termination failure in this run.
- Two return legs logged future TF extrapolation followed by missed 20 Hz controller
  cycles at 9.3765 Hz and 9.0111 Hz. There were also isolated old-stamp message-filter
  drops for `os_lidar` and `odom`. These did not prevent completion but are relevant
  evidence for the remaining mild jerk investigation.
- This outcome does not prove that 0.5 m was fundamentally impossible; it establishes
  that 1.0 m provides the desired outdoor acceptance behavior with the current MPPI
  tracking and is a deliberate mission requirement, not a hidden yaw workaround.

### 2026-07-28 command isolation and velocity-smoother ramp tuning — applied and tested

Status: **Applied and robot-tested; the observed stop/start and super-jerky motion are
resolved. The braking envelope remains to be validated before production use.**

- Changed the normal standalone launch's raw Nav2 topic from the Clearpath post-mux
  `/platform/cmd_vel` output to private `/cmd_vel_nav`. This stopped continuous
  `teleop_twist_joy` zeros from entering the velocity smoother and eliminated the
  severe multi-second start/stop behavior.
- Reduced the smoother's linear acceleration/deceleration limits from
  `0.75 / -1.0` to `0.5 / -0.5 m/s²`. At 50 Hz this reduces the largest per-cycle
  linear changes from approximately `+0.015 / -0.020 m/s` to
  `+0.010 / -0.010 m/s`.
- The subsequent physical test no longer exhibited the remaining super-jerky motion.
  This outcome is mechanically consistent with the velocity smoother's purpose as a
  slew-rate limiter, so the motion-quality symptom is considered resolved.
- The user subsequently restored both MPPI and smoother maximum X velocity to
  1.0 m/s and selected symmetric `0.5 / -0.5 m/s²` linear slew limits. Attribution
  of the smoother motion improvement still relies on the user’s reported test sequence.
- The current MPPI rollout limits remain `ax_max: 2.0` and `ax_min: -1.0 m/s²`, which
  do not model the downstream `0.5 / -0.5 m/s²` slew limits. Validate controlled
  stops, goal approaches, reverse transitions, and obstacle avoidance before calling
  these exact values production-final.

### 2026-07-28 missing-costmap regression — applied and stationary-tested

Status: **Applied and tested with live stationary sensor data; moving obstacle
avoidance remains to be field-tested.**

- GroundGrid correctly published `/groundgrid/obstacle_cloud`, and RViz displayed
  that absolute topic, but STVL 2.5.5 resolved the relative configured source under
  each private costmap namespace. The resulting
  `/local_costmap/groundgrid/obstacle_cloud` and
  `/global_costmap/groundgrid/obstacle_cloud` topics each had two subscribers and
  zero publishers. `expected_update_rate: 0.0` explains why startup stayed quiet.
- Before the fix, one GroundGrid cloud contained 20,617 points, while both published
  STVL voxel grids contained zero points. The 80 x 80 local costmap had `{0: 6400}`
  only; the 400 x 400 global map had `{-1: 159984, 0: 16}`. The 16 global free cells
  were only the footprint-cleared area.
- Both launch entry points now default marking to absolute
  `/groundgrid/obstacle_cloud` and raw clearing to absolute `/ouster/points`. Both
  YAML clearing sources were restored to `__POINTCLOUD_TOPIC__`. The repository RViz
  voxel displays now use the real `voxel_grid` topics.
- After restarting the stack, the live graph showed two STVL subscribers on the real
  GroundGrid topic and one local plus one global clearing subscriber on raw Ouster.
  The private ghost topics disappeared. With a 19,367-point GroundGrid cloud, local
  and global STVL published 1,621 and 1,983 voxels. The local map contained 568
  lethal cells plus 1,182 inflated cells; the global map contained 700 lethal cells
  plus 860 inflated cells.
- `voxel_min_points: 2` is retained because the repaired path produces dense obstacle
  markings. The supplied startup log had one 8.104 ms future-TF extrapolation drop;
  it is a secondary timing concern, not the cause of continuous empty costmaps.
- Build and overlay checks found source/build/install files identical through the
  symlink install, matching GroundGrid binaries, and successful latest builds. This
  was a topic-resolution configuration regression, not a stale build or semantic-
  branch artifact.

### 2026-08-04 Nav2 1.3.12 MPPI start/reverse regression — applied and tested

Status: **Applied and robot-tested; successful.**

- The same `perch_waypoints.yaml` shuttle and the same pre-fix Nav2 configuration
  behaved normally in `kumarrobotics/spine-nvda:nav2_test_restored` but hesitated for
  approximately 9-12 seconds at some waypoint starts in
  `kumarrobotics/jackal_autonomy:jazzy-20260730-r4`. The new image also sometimes
  drove an entire return leg in reverse before the rotation shim turned it for the
  next forward leg.
- The supplied pre-fix logs are `data/logs/ros2_spine_docker_navigation.log` for the
  old image and `data/logs/ros2_autonomy_docker_nav2.log` for the new image. The new
  image's first `(0,0) -> (10,0)` leg took 23.48 seconds from navigation start to
  success versus 15.20 seconds in the old image, an 8.28-second excess consistent
  with the observed initial hesitation. Completed legs did not log a BT spin,
  backup, or recovery transition, so the reverse leg was an MPPI trajectory choice,
  not `SmacRotation`, `BackUp`, or another recovery behavior.
- Package inspection found coherent Nav2 installations rather than a mixed-library
  or overlay problem: the old container has Nav2 1.3.7 and the standalone container
  has Nav2 1.3.12, both loaded from `/opt/ros/jazzy`. The checked-out Jackal Nav2
  source, launch, BT XML, and pre-fix parameter file were identical between the two
  containers. The Dockerfile's mutable `latest` base and unversioned apt installs
  allowed the Nav2 patch release to change even though the application parameters did
  not.
- Nav2 PR [#5260](https://github.com/ros-navigation/navigation2/pull/5260), backported
  as commit
  [`b47bcfa5`](https://github.com/ros-navigation/navigation2/commit/b47bcfa5e633e4c06b7def8a615e16cfdbec397c),
  changed MPPI's bidirectional acceleration constraints between these releases. With
  `model_dt: 0.05`, pre-fix `ax_max: 2.5`, `ax_min: -1.0`, and zero initial velocity,
  the feasible first-step X-velocity interval changed from approximately
  `[-0.05, +0.125] m/s` to `[-0.125, +0.05] m/s`. The same nominal limits therefore
  became reverse-biased. The later upstream discussion in Nav2 PR
  [#6248](https://github.com/ros-navigation/navigation2/pull/6248) independently
  reports asymmetric limits producing oscillation or failure to start.
- The pre-fix combination of `vx_min: -0.5`, `PathAngleCritic.mode: 1`, and the soft
  `PreferForwardCritic` allowed either travel direction. Once the acceleration window
  favored negative velocity, MPPI could alternate around zero at startup or settle on
  a locally valid reverse trajectory for the whole leg. The rotation shim was not the
  cause; it only acted later when a sufficiently large initial heading error existed.
- The successful configuration change was:

  ```yaml
  ax_max: 6.0
  ax_min: -6.0
  vx_min: 0.0
  PathAngleCritic:
    mode: 0
  ```

- The symmetric rollout bounds remove the sign-dependent magnitude bias. The hard
  nonnegative X bound and forward-only path-angle mode prevent ordinary MPPI tracking
  from selecting a full reverse leg. The user repeated the robot mission and confirmed
  that the reported start/stop delay and unintended full-leg reverse behavior were
  gone.
- `6.0 / -6.0 m/s²` is an optimizer-rollout compatibility workaround, not permission
  for that physical acceleration. The downstream velocity smoother still enforces
  `0.3 / -1.0 m/s²`. Before production freeze, validate stopping and obstacle
  avoidance, test a smaller symmetric rollout pair if desired, and pin the base image
  and Nav2 package versions so a rebuild cannot silently change controller semantics.

## What is fixed

1. Arbitrary post-Nav2 scalar velocity multiplication was removed from the bridge.
2. Physical Twist-to-Joy conversion and axis clamping are implemented.
3. Nav2 can command useful forward speeds and has reached about 0.6 m/s.
4. DLIO dynamic TF publication was moved to the 100 Hz pose timer and removed the
   earlier persistent starvation. The latest log still has two isolated future-TF
   errors and 9 Hz controller cycles, now explicitly tracked as a remaining issue.
5. MPPI alone can complete the first waypoint turn with the current angular tuning;
   the rotation shim is not required for that turn.
6. GroundGrid now generates odom-frame clouds using transforms from the same stamp
   carried by the output cloud. The former `frame odom` drop was not in the latest
   logs; occasional drops now concern the separate raw `os_lidar` clearing source.
7. Publishing DLIO linear twist in `base_link` fixed the direction-specific return-leg
   start/stop motion; this is confirmed by the latest robot test.
8. The one-shot rotation shim handles large changed-goal heading errors with an
   in-place turn without reported random rotations during straight tracking.
9. Position-only goal completion at the intentionally selected 1.0 m outdoor XY
   tolerance completed four consecutive shuttle legs in the latest test.
10. Private `/cmd_vel_nav` command ownership removes the external teleop-zero race
    from the velocity smoother and fixed the severe multi-second start/stop behavior.
11. The robot-tested `0.5 / -0.5 m/s²` smoother limits eliminated the remaining
    super-jerky straight-line motion.
12. Absolute STVL marking and clearing topics restored non-empty local/global voxel
    layers and lethal/inflated costmap cells in the stationary live-sensor test.
13. The Nav2 1.3.12 MPPI startup hesitation and unintended full-leg reverse behavior
    are robot-verified fixed with symmetric `6.0 / -6.0 m/s²` rollout bounds,
    `vx_min: 0.0`, and `PathAngleCritic.mode: 0`.
14. Slow sensor-stack startup is fixed by restricting automatic ROS discovery to
    `LOCALHOST`. Subnet discovery had found another robot using the same domain ID,
    causing Fast DDS graph interference during ZED publisher creation.

## What remains unresolved

1. ~~Isolate the mild straight-line jerk/pulsing by recording every command stage and
   wheel feedback with synchronized timestamps.~~ **Resolved by the 2026-07-28
   velocity-smoother tuning test.**
2. ~~Test whether another publisher sends zero velocity alongside Nav2.~~ **Confirmed
   at `/platform/cmd_vel` and fixed for the normal standalone launch by restoring the
   private `/cmd_vel_nav` pre-smoother topic.**
3. Validate whether the current smoother's `-1.0 m/s²` deceleration provides an
   acceptable stopping distance at every allowed speed, especially at 1.0 m/s.
4. The robot-tested Nav2 1.3.12 workaround uses MPPI `ax_max / ax_min` of
   `6.0 / -6.0 m/s²`, while the physical smoother uses `0.3 / -1.0 m/s²`. Validate
   stopping and obstacle avoidance, then determine whether a smaller symmetric MPPI
   pair or a corrected/pinned Nav2 release can retain the fix with a closer model.
5. ~~Change the inner `nav2_servers.launch.py` default from `platform/cmd_vel` to
   `cmd_vel_nav`.~~ **Resolved on 2026-07-28; both launch entry points now default
   to the private pre-smoother command topic.**
6. Verify the already-applied 50 Hz serial-container controller-manager setting when
   low-level cadence is next inspected. It is no longer required to explain the
   resolved jerk symptom, but remains useful end-to-end platform validation.
7. Treat the two historical 9 Hz controller cycles and TF extrapolation errors as a
   timing follow-up only if they recur; they are no longer needed to explain the jerk.
8. MPPI reverse is intentionally disabled in the current Nav2 1.3.12 workaround.
   If controller-selected reverse is required later, reintroduce it only after testing
   a corrected Nav2 version or constraints that do not recreate the startup bias.
   Explicit behavior-tree `BackUp` remains available through the smoother.
9. Replace the square footprint with measured Jackal geometry before treating tight-
   space rotational collision checks as authoritative.
10. Explicit inspection-yaw handling in `jackal_autonomy_server.py` remains deferred.
11. Joy calibration and selectable speed profiles remain follow-up work.
12. Run a controlled physical obstacle-avoidance test with the repaired costmaps; the
    stationary test proves perception fusion but not the complete moving behavior.

## Why the odometry correction fixed the former oscillation

The former failure persisted without the shim, with stable Smac paths, with yaw set
or unset, and with both Nav2 command topics zeroing together. The only consistent
physical discriminator was travel direction in the odom/world frame.

ROS Odometry declares pose in `header.frame_id` and twist in `child_frame_id`. DLIO
was mixing these contracts: world-frame linear velocity was labeled as base_link
twist. Nav2 OdomSmoother does not rotate it, and MPPI places its X value into the
first velocity of all 2000 rollouts. A Jackal facing pi and driving forward home
therefore entered each rollout with negative `vx`; outbound forward travel entered
with positive `vx`. The corrected publication makes body-forward X positive in both
directions.

## Goal-checker note

Ordinary navigation now uses `PositionGoalChecker`: only the current XY error is
checked, with a 1.0 m tolerance, and the goal quaternion is ignored. This is explicit
position-only semantics rather than a large yaw tolerance pretending to ignore yaw.

`goto_nav2.py` may therefore continue sending yaw zero for ordinary goals. Explicit
inspection-yaw behavior will later require a separately selected pose-goal mode; per
user direction, `jackal_autonomy_server.py` is not changed in this implementation.

## Historical cross-container straight-line jerk handoff (superseded)

Status: **Superseded by the 2026-07-28 robot-tested fixes.** Retain this section as
historical diagnostic guidance if similar pulsing ever recurs; it is no longer the
active next task.

The earlier proposed debugging chat would have run from the deployed
`KumarRobotics/jackal-serial` checkout/container and inspected that checkout's branch,
commit, configuration, launch remappings, and local diff before changing anything.
The verified upstream facts below refer to `main` commit
`b75e3f9044da2181b6f281105e5a48c62f625ea9`; the robot may run a fork or older image.

### Symptom boundary

- The old failure was aggressive start/stop motion specifically on the return leg.
  It was caused by DLIO publishing world-frame linear velocity while labeling it
  `base_link`; that correction is built and robot-verified. Preserve it.
- ~~The current symptom is much milder jerk/pulsing while traveling down a straight.~~
  **Resolved in the 2026-07-28 physical test by gentler velocity-smoother linear
  acceleration/deceleration limits after command-topic isolation.**
- The older 1.0 m-tolerance shuttle log contained two future-TF lookup failures and
  controller-loop rates of 9.3765 and 9.0111 Hz instead of 20 Hz. These remain useful
  historical timing evidence, but they are no longer required to explain the resolved
  jerk symptom.

### Verified upstream behavior and deployed 50 Hz change

At the upstream commit named above:

- `jackal-control/config/control.yaml` sets `controller_manager.update_rate: 10` Hz.
- The same file sets `jackal_velocity_controller.publish_rate: 50.0`; that output
  publish setting does not by itself prove a 50 Hz controller or serial-write rate.
- `jackal-serial/ros/jackal_serial_node.cpp` subscribes to `/joint_states`; its
  callback converts wheel velocities to a Drive message and immediately calls
  `writeDriveMsg()`. Thus serial drive cadence is coupled to actual joint-state
  callback cadence in upstream `main`.
- `jackal-teleop/src/jackal_teleop.cpp` publishes TwistStamped directly to
  `/jackal_velocity_controller/cmd_vel`. An autonomous Joy message and a physical
  receiver/teleop node can therefore share the downstream command path. Whether
  both are active in the deployed launch must be measured, not guessed.

The user has **already changed the deployed serial-container controller-manager
update rate from 10 Hz to 50 Hz** based on an earlier debugging recommendation. Treat
that edit as **applied but unverified**, not as a proposed next fix. The next chat must
locate its exact file/diff, verify the launched parameter resolves to 50 Hz, measure
actual `/joint_states` cadence, and confirm the callback-driven serial writes also
run near 50 Hz. A wrong config path, stale image/build, launch override, missed loop,
or slower broadcaster could make the source edit ineffective.

The user's hypothesis that this stack publishes zero `linear.x` commands alongside
Nav2 is plausible but **unconfirmed**. Even if the 50 Hz change is active, multiple
publishers or sporadic timing can still explain the mild pulse.

### First inspection in the serial container

Start by discovering exact deployed names and command ownership:

```bash
ros2 topic list | sort
ros2 node list | sort
ros2 topic info -v /jackal_velocity_controller/cmd_vel
ros2 topic info -v /joint_states
ros2 topic hz /jackal_velocity_controller/cmd_vel
ros2 topic hz /joint_states
ros2 param get /controller_manager update_rate
ros2 control list_controllers
ros2 control list_hardware_interfaces
```

Also run `ros2 topic info -v` for `/joy`, the actual autonomous Joy topic, and any
`cmd_vel`, mux, or controller-input topics discovered by `ros2 topic list`. Record
every publisher node name, namespace, message type, QoS, and publisher count. Topic
names may differ because of container remappings; use discovered names thereafter.

Inspect the deployed `control.yaml`, launch parameter path, built/install-space copy,
container image, and repository diff. Reconcile those with the runtime parameter. If
the runtime value is 50 Hz but `/joint_states` remains near 10 Hz, inspect controller
update diagnostics, missed cycles, broadcaster settings, CPU scheduling, and whether
the serial node subscribes to the topic being measured.

If `/jackal_velocity_controller/cmd_vel` has more than one publisher, stop or disable
one suspected publisher at a time in a clear test area. Do not infer which publisher
created a zero from a merged `ros2 topic echo`; publisher identity and synchronized
recording are required. Inspect launch files for teleop, safety, mux, bridge, and
watchdog nodes that can legally emit zero commands.

### Synchronized recording

Prefer one ROS bag from a container that can see all DDS topics. Otherwise record
bags in both containers while their system/ROS clocks are synchronized and note the
start time. Before driving, verify exact topic types and names. At minimum record:

Nav2 container:

- `/cmd_vel_nav`
- `/cmd_vel_smoothed`
- the Joy output of `convert_vel_to_joy.py`
- `/dlio/odom_node/odom`
- `/tf`, `/tf_static`, `/rosout`, and `/diagnostics`

Serial container:

- `/joy` and the actual controller input command
- `/jackal_velocity_controller/cmd_vel`
- `/joint_states`
- controller odometry/state topics
- `/rosout`, `/diagnostics`, and any safety/mux output

Run one constant-speed straight goal in a clear area and verbally or electronically
mark each physical jerk. Measure message intervals, zeros, source publisher, wheel
velocity response, and whether a zero persists long enough to approach the 0.25 s
upstream controller timeout. `ros2 topic hz` is a useful live check, but the bag is
the evidence for individual gaps and cross-topic timing.

### Attribution decision tree

1. If `/cmd_vel_nav.linear.x` dips or becomes zero at each jerk, investigate MPPI,
   obstacle/costmap changes, TF lookup failures, and missed controller cycles.
2. If Nav2 is continuous but `/cmd_vel_smoothed` dips, investigate velocity-smoother
   input freshness, timeout, odometry feedback, and lifecycle state.
3. If smoothed Twist is continuous but the bridge Joy output dips, inspect
   `convert_vel_to_joy.py`, its input subscription, timer execution, and process load.
4. If autonomous Joy is continuous but the base command contains interleaved zeros,
   identify multiple publishers, teleop/safety/mux behavior, and input timeouts.
5. If the base command is continuous but `/joint_states` or serial writes remain
   sparse, debug why the already-applied 50 Hz controller-manager change is not
   producing a 50 Hz end-to-end hardware command cadence.
6. If commands and serial cadence are continuous but wheel velocity or physical motion
   pulses, investigate motor-controller watchdog/deadband, serial latency/errors,
   hardware diagnostics, power, and wheel feedback.

Change one variable at a time and preserve a baseline bag. Do not tune MPPI gains,
remove the shim, or revert the DLIO correction until the discontinuity is attributed.
Use a clear outdoor area or safely support the wheels for cadence-only tests.

### Follow-up after continuity is fixed

- Confirm reverse commands from MPPI and `BackUp` traverse the whole path when needed.
- Replace the approximate square footprint with measured geometry for tight spaces.
- Calibrate `linear_speed_at_full_axis` and `angular_speed_at_full_axis` from steady
  commands and measured motion, then add selectable speed profiles.
- Keep the 1.0 m XY tolerance for the intended outdoor waypoint mode. A future indoor
  precision mode can select a smaller tolerance separately rather than silently
  changing the tested outdoor profile.
- Continue deferring explicit inspection-yaw changes to `jackal_autonomy_server.py`.

## High-level implementation changes by file

### `nav2_jackal_stvox_mppi_tuning.yaml`

- Active experiment wraps MPPI with a one-shot initial-heading rotation shim.
- Retains the robot-tested shim turn at 0.6 rad/s with open-loop acceleration ramping,
  plus MPPI's 0.75 rad/s maximum, 0.4 rad/s sampling deviation, and 6.0 rad/s² rollout
  acceleration.
- Sets MPPI minimum X velocity to 0.0 m/s to prohibit controller-selected
  reverse; the smoother retains -0.5 m/s so explicit `BackUp` commands can pass.
- Sets controller failure tolerance to zero to avoid patience-window stop/go pulses.
- Uses an odom-frame rolling local costmap with a 0.3-second transform tolerance.
- Makes local/global footprints identical.
- Marks STVL obstacles from `/groundgrid/obstacle_cloud` and performs frustum
  clearing from the full raw `/ouster/points` scan; both topic defaults are absolute.
- Explicitly clears the footprint and uses a 1-second local voxel decay.
- Replaces failing NavFn with cost-aware SmacPlanner2D and its path smoother.
- Selects the official recovery BT that replans only for a new or invalid path,
  eliminating unconditional 1 Hz path replacement.
- Uses non-stateful `PositionGoalChecker` with the tested 1.0 m outdoor XY tolerance.
- Removes GoalAngleCritic and keeps only the angular VelocityDeadbandCritic penalty.
- Configures PathAngleCritic mode 0 for forward path alignment while retaining
  PreferForwardCritic.
- Sets MPPI and smoother maximum X velocity to 1.0 m/s.
- Uses symmetric MPPI rollout acceleration/deceleration limits of `6.0 / -6.0 m/s²`
  as the robot-tested Nav2 1.3.12 compatibility workaround.
- The current velocity-smoother physical linear acceleration/deceleration limits are
  `0.3 / -1.0 m/s²`; these still require braking-distance and obstacle validation.

### `jackal_navigation.launch.py`

- Changes the normal standalone raw Nav2 command topic from the Clearpath post-mux
  `/platform/cmd_vel` output to private `/cmd_vel_nav`, eliminating the confirmed
  teleop-zero collision at the velocity-smoother input.
- Defaults STVL marking and raw clearing topics to fully qualified root topics so
  costmap-private namespaces cannot silently redirect the subscriptions.
- The nested `nav2_servers.launch.py` default now also uses `cmd_vel_nav`, so direct
  launches do not recreate the post-mux feedback collision.

### GroundGrid `src/GroundGridNode.cpp`

- Looks up odom-to-base, odom-to-sensor, and odom-to-cloud transforms at the
  incoming cloud timestamp with a 0.5-second wait.
- Drops a cloud if timestamp-matched transforms are unavailable instead of
  publishing coordinates generated from latest TF with an old stamp.
- Corrects the input point frame label before transforming points into odom.
- Passes the timestamp-matched base transform into ground segmentation.

### `nav2_jackal_stvox.yaml`

- Preserved as the previous shim experiment; it is no longer selected by the
  default Jackal aggregate launch.
- Added explicit odometry and speed-limit topics to the controller server.
- Wrapped MPPI with the rotation shim and tuned its heading thresholds, angular
  speed, acceleration, goal-heading behavior, one-shot behavior, and open-loop
  ramping.
- Raised useful MPPI velocity/acceleration ranges and added a velocity deadband
  critic.
- Set MPPI minimum linear velocity to zero to discourage controller-selected
  reverse motion.
- Tuned goal tolerances, velocity smoother limits, and inflation radii.
- Reduced an extremely large global costmap, although it remains oversized.

### `.gitignore`

- Whitelists both the MPPI tuning YAML and active `data/perch_waypoints.yaml` so
  their experiment settings are visible despite the repository-wide `*yaml` rule.

### `convert_vel_to_joy.py`

- Replaced arbitrary output multipliers with physical full-axis calibration
  parameters.
- Made the input command topic configurable.
- Added validation, normalized conversion, and clamping.
- Publishes Joy commands on a fixed 50 Hz timer.

### `jackal_launch_all.launch.py`

- Selects `nav2_jackal_stvox_mppi_tuning.yaml` for the current test.
- Passes the selected Nav2 configuration file into the Joy bridge so the bridge
  uses the same calibrated parameters.
- Other commented launch entries may be unrelated user changes.

### DLIO `odom.h` and `odom.cc`

- Added a dedicated odometry-transform publication function.
- Moved dynamic odom-to-base TF publication to the 100 Hz pose timer.
- Uses the current IMU timestamp for the high-rate transform.
- Removed dependence on the slower LiDAR publication path for that transform.
- Corrected Odometry linear twist from world-frame `state.v.lin.w` to base-frame
  velocity using the current orientation. This satisfies the `child_frame_id`
  contract and makes body-forward X positive on both outbound and return legs.
- The latest robot test verified that this correction eliminates the return-leg
  start/stop motion.

### `perch_waypoints.yaml`

- The active shuttle mission is trackable instead of silently ignored.
- All endpoints now use yaw zero as an explicitly ignored ordinary-navigation
  placeholder; new-path geometry drives initial shim alignment.

## Rules for updating this context file

Every future debugging chat should update this file before ending the session.
Follow these rules:

1. Re-read the actual files and inspect `git diff`/`git status` in both
   `spine-multi` and `DLIO` before updating this document. The filesystem is the
   source of truth; correct this primer if it has drifted.
2. Add a dated experiment entry containing the exact test, observed command and
   odometry values, relevant logs, configuration changes, and outcome.
3. Explicitly label each idea as one of:
   - **Applied and tested**
   - **Applied but unverified**
   - **Proposed, not applied**
   - **Rejected or reverted**
4. Record all important current problems, attempted solutions, high-level file
   changes, confirmed fixes, remaining failures, and proposed next actions.
5. Preserve the history of failed attempts. Do not rewrite a failed experiment as
   successful just because a later hypothesis sounds plausible.
6. Keep the “Current status,” “Current active configuration,” “What is fixed,” and
   “What remains unresolved” sections synchronized with the latest test.
7. Include exact log excerpts and measurements when they materially distinguish
   the command source or failure mode, but do not paste full source diffs.
8. Mention unrelated modified files so they are not accidentally overwritten, but
   do not attribute those changes to this work without evidence.
9. If a proposed fix did not work and there is no new diagnostic evidence, say so
   plainly and make data collection the next step instead of stacking more tuning
   changes.
10. Maintain the core goal: smooth, stable, physically calibrated speed control up
    to 1.0 m/s, plus repeatable in-place turns and waypoint tracking.

