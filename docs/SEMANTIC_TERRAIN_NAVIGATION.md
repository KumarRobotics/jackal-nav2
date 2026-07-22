# SAM3 semantic terrain navigation

This feature adds an opt-in SAM3 RGB-D terrain pipeline to `jackal_nav2` and
fuses its output into both Nav2 costmaps. It does not import or launch
`vision_ros2` or `opennav_amd_semantic_navigation`. The only custom ROS package
dependency is the Jazzy `semantic_segmentation_layer` checkout beside this
package.

## Data flow

`semantic_terrain` approximately synchronizes the ZED left rectified color,
registered depth, and depth `CameraInfo`. SAM3 runs on the source image. The
class mask, confidence, and organized XYZ cloud are sampled together to
320 x 180, retain one common timestamp, and are published as:

- `semantic_terrain/label_mask` (`sensor_msgs/Image`, `mono8`)
- `semantic_terrain/confidence` (`sensor_msgs/Image`, `mono8`)
- `semantic_terrain/label_info` (`vision_msgs/LabelInfo`, reliable and transient)
- `semantic_terrain/points` (organized `sensor_msgs/PointCloud2`, XYZ float32)
- optional `semantic_terrain/overlay` and `semantic_terrain/diagnostics`

The startup-static mapping is:

| ID | Costmap class | Example prompts | Base / maximum cost |
| --- | --- | --- | --- |
| 0 | Unlabeled | no accepted SAM3 mask | no semantic effect |
| 1 | `preferred_surface` | paved road, compact gravel, firm dirt | 0 / 0 |
| 2 | `caution_surface` | short grass, loose gravel | 60 / 100 |
| 3 | `high_risk_surface` | tall vegetation, mud, deep sand, curb | 150 / 220 |
| 4 | `water_or_dropoff` | water, ditch, drop-off | 200 / 252 |

The arrays `prompts`, `prompt_class_ids`, and `prompt_class_names` must remain
index-aligned. They are read-only after startup; change the YAML and restart the
terrain node and both costmaps to change the mapping.

## Calibration gate

The packaged `config/jackal_sensor_extrinsics.yaml` contains only provisional
values for `base_link -> zed_camera_link`:

```yaml
zed:
  x: 0.25
  y: 0.0
  z: 0.50
  roll: 0.0
  pitch: 0.0
  yaw: 0.0
  calibrated: false
```

ROS axes are x forward, y left, z up; roll, pitch, and yaw are radians using the
right-hand rule. Measure the actual transform, place it in either the packaged
file or an external YAML with the same schema, and set `calibrated: true` only
after validation. Semantic Nav2 startup is blocked while it is false.

Use an external calibration without editing the package:

```bash
ros2 launch jackal_nav2 jackal_navigation.launch.py \
  semantic_navigation:=true \
  zed_extrinsics_file:=/absolute/path/zed_extrinsics.yaml
```

If the URDF or another process already publishes this transform, also pass
`publish_zed_static_tf:=false`; exactly one TF authority may publish it. The
external YAML is still used as the semantic-navigation calibration assertion.
After a change, restart the static publisher and costmaps, inspect it with:

```bash
ros2 run tf2_ros tf2_echo base_link zed_camera_link
```

Then validate stationary projection against measured landmarks before allowing
motion. A direct camera-to-LiDAR calibration is not required when each sensor has
an accurate transform to `base_link`, but both transforms must be physically
correct.

## Build and run

Always use the Jazzy dependency branch:

```bash
cd /home/dcist/dcist_ws/src/semantic_segmentation_layer
git switch jazzy
cd /home/dcist/dcist_ws
source /opt/ros/jazzy/setup.bash
source /home/dcist/ros_venv/bin/activate
colcon build --symlink-install \
  --packages-select semantic_segmentation_layer jackal_nav2
source install/setup.bash
```

The default model path is `/home/dcist/data/weights/sam3.pt`, device `cuda:0`,
inference size 640, and inference rate is uncapped (`0.0`). Override
`semantic_terrain_params_file` with another complete parameter file when needed.
No Python package changes are required by this implementation; the existing
`ros_venv` supplies NumPy 1.26.4, Torch, Ultralytics, and OpenCV.

Start the normal bringup with semantic fusion enabled using the launch command
above. The semantic overlay is layered on the existing tuned Nav2 file only in
this mode. It:

- loads `voxel_layer -> semantic_layer -> inflation_layer` in both costmaps;
- uses maximum fusion, so semantics cannot clear geometric obstacles;
- limits semantic costs to 252, below Nav2 lethal cost 254;
- decays semantic observations after 3 seconds;
- replans the global path periodically at 1 Hz; and
- caps initial forward and reverse motion at 0.3 m/s.

For perception-only stationary testing, run the node without starting semantic
Nav2:

```bash
ros2 run jackal_nav2 semantic_terrain --ros-args \
  --params-file /home/dcist/dcist_ws/install/jackal_nav2/share/jackal_nav2/config/semantic_terrain.yaml
```

The service `semantic_terrain/enable` (`std_srvs/SetBool`) pauses and resumes
inference. If uncapped inference affects Nav2 deadlines, set
`max_inference_rate_hz` to successively lower values and retain the highest
stable rate.

## First motion test

Do not perform the first live test until the calibrated flag is true, the ZED
and LiDAR transforms to the base are verified, registered depth aligns with the
left image, stationary cost projection is correct in both costmaps, and an
operator has an immediate manual stop. Keep the 0.3 m/s profile for that test.
