# Semantic Terrain Navigation Implementation Context

## Objective

Add a self-contained SAM3 semantic traversability frontend to `jackal_nav2` and fuse its results into Nav2 through the Jazzy `semantic_segmentation_layer`. The runtime implementation must not depend on `vision_ros2` or `opennav_amd_semantic_navigation`; those repositories were used only as references. The only external runtime package introduced for semantic costmap fusion is the standalone `semantic_segmentation_layer`.

## Implemented solution

### Semantic perception frontend in `jackal_nav2`

- `jackal_nav2/sam3_backend.py` wraps Ultralytics `SAM3SemanticPredictor` behind a small, injectable backend interface. Imports and model construction are lazy so ordinary navigation does not load SAM3, Torch, or the model weights.
- `jackal_nav2/semantic_terrain.py` implements prompt-to-class mapping, deterministic overlap resolution, registered-depth deprojection, aligned label/confidence products, organized point-cloud construction, and a latest-frame mailbox.
- `jackal_nav2/semantic_terrain_sampling.py` contains deterministic nearest-neighbor sampling shared by image/depth products and depth-range invalidation.
- `jackal_nav2/semantic_terrain_node.py` synchronizes ZED RGB, registered depth, and `CameraInfo`; runs inference in a latest-frame worker; and publishes label mask, confidence, label metadata, organized XYZ cloud, optional overlay, and diagnostics. It also exposes an enable/disable service.
- Class mappings and cost policies are startup-static. A running costmap rejects changes to those parameters so one numeric label cannot silently acquire a different meaning while old observations remain buffered.
- Node shutdown disables the publication gate before its bounded worker join, so a long-running SAM inference cannot publish through destroyed ROS entities.
- Inference defaults to 640 pixels and is uncapped (`max_inference_rate_hz: 0.0`). Throughput is bounded by available compute and the latest-frame mailbox drops stale input rather than building latency.

### Nav2 integration

- `config/semantic_terrain.yaml` defines SAM3 settings, ZED topics, output geometry, depth limits, and four traversability classes.
- `config/nav2_semantic_terrain_overlay.yaml` adds `SemanticSegmentationLayer` between the voxel and inflation layers for both costmaps. Preferred, caution, high-risk, and water/drop-off classes receive increasing non-lethal semantic costs; geometric lethal obstacles remain authoritative.
- `launch/nav2_servers.launch.py` merges the semantic overlay only when `semantic_navigation:=true`. Normal Nav2 startup therefore neither creates nor requires the semantic plugin.
- `launch/jackal_navigation.launch.py` conditionally starts semantic perception and passes the semantic overlay into Nav2. It enforces calibrated camera extrinsics before semantic motion, while allowing perception-only startup with `start_nav2:=false` for calibration and visualization checks.
- No RViz configuration or bag-recording launch files were changed, per the requested scope.

### Camera extrinsics

- `config/jackal_sensor_extrinsics.yaml` contains an explicit provisional `base_link -> zed_camera_link` transform: `x: 0.25`, `y: 0.0`, `z: 0.50`, zero roll/pitch/yaw, and `calibrated: false`.
- `jackal_nav2/sensor_extrinsics.py` validates all six finite numeric fields and the boolean calibration flag.
- `launch/jackal_static_transforms.launch.py` loads the YAML and publishes the ZED transform independently of the optional map-to-odom transform.
- A measured file can later be selected with `zed_extrinsics_file:=/absolute/path/to/file.yaml`. Semantic robot motion remains intentionally blocked until its `calibrated` value is explicitly set to `true`.

### External Jazzy semantic layer hardening

The standalone `semantic_segmentation_layer` checkout is on its `jazzy` branch. Its buffer and costmap plugin were hardened to:

- initialize a stable class map before any `LabelInfo` message;
- reject invalid class IDs and malformed mask, confidence, and organized-cloud messages;
- expire and clear stale semantic cells without leaving ghost costs;
- preserve lethal geometric costs under maximum fusion;
- lock class mappings and cost policies after startup while retaining safe runtime controls;
- cover those behaviors with seven focused C++ regression tests.

An empty `opennav_amd_semantic_navigation/COLCON_IGNORE` prevents the AMD ROCm/MIGraphX packages from entering this workspace build. No runtime code imports that repository.

## Files changed

### `jackal_nav2`

- Modified: `setup.py`, `package.xml`
- Modified launches: `launch/jackal_navigation.launch.py`, `launch/jackal_static_transforms.launch.py`, `launch/nav2_servers.launch.py`
- Added implementation: `jackal_nav2/sam3_backend.py`, `jackal_nav2/semantic_terrain.py`, `jackal_nav2/semantic_terrain_sampling.py`, `jackal_nav2/semantic_terrain_node.py`, `jackal_nav2/sensor_extrinsics.py`
- Added configuration: `config/semantic_terrain.yaml`, `config/nav2_semantic_terrain_overlay.yaml`, `config/jackal_sensor_extrinsics.yaml`
- Added documentation: `docs/SEMANTIC_TERRAIN_NAVIGATION.md` and this context file
- Added tests: `test/test_semantic_terrain.py`, `test/test_semantic_terrain_sampling.py`, `test/test_semantic_navigation_config.py`, `test/test_sensor_extrinsics.py`

The pre-existing user edit in `config/nav2_jackal_stvox_mppi_tuning.yaml` was preserved and was not part of this implementation.

### `semantic_segmentation_layer`

- Modified: `CMakeLists.txt`, `package.xml`
- Modified headers: `include/semantic_segmentation_layer/segmentation_buffer.hpp`, `include/semantic_segmentation_layer/semantic_segmentation_layer.hpp`
- Modified sources: `src/segmentation_buffer.cpp`, `src/semantic_segmentation_layer.cpp`
- Added: `test/test_semantic_segmentation_layer.cpp`

### `opennav_amd_semantic_navigation`

- Added: `COLCON_IGNORE`

## Verification completed

- Both packages build together under ROS 2 Jazzy with `colcon build --symlink-install --packages-select semantic_segmentation_layer jackal_nav2`.
- All 39 `jackal_nav2` tests pass.
- All seven focused semantic-layer C++ regression tests pass.
- All modified launch descriptions import and instantiate.
- The installed `jackal_nav2 semantic_terrain` executable and semantic plugin registration are discoverable.
- The default semantic-motion launch fails immediately and deliberately while extrinsics are uncalibrated.
- Perception-only mode bypasses the motion calibration gate.
- A static-TF smoke test publishes the provisional translation from `base_link` to `zed_camera_link` with map-to-odom publication disabled.
- A fake-backend node smoke test confirms shutdown immediately closes the inference/publication gate without loading SAM3.
- No Python packages were installed or upgraded. In particular, the existing `ros_venv` NumPy version remains `1.26.4`.

The full upstream semantic-layer lint suite still reports inherited formatting and license-header debt. This is separate from the new focused functional tests, which pass.

## Remaining work and proposed next steps

1. Measure the physical six-degree-of-freedom ZED extrinsic, update a copy of `jackal_sensor_extrinsics.yaml`, and set `calibrated: true` only after TF direction and values have been checked.
2. Run perception-only mode against the live ZED. Verify registered depth is pixel-aligned with RGB, all output headers share the RGB timestamp, the organized cloud dimensions match the semantic masks, and terrain classes are stable outdoors.
3. Benchmark SAM3 on the actual robot computer. Leave inference uncapped initially, then set `max_inference_rate_hz` only if thermal, compute, or navigation contention warrants an explicit cap.
4. Tune prompt wording, thresholds, class costs, observation persistence, and range limits using stationary and teleoperated tests before enabling autonomous motion.
5. Perform the first motion test at low speed with an emergency stop and validate that geometric obstacles remain lethal even when semantic predictions disagree.
6. Optionally clean the upstream semantic-layer lint debt in a separate change; it is not required for functional bring-up.

No live camera, full SAM3 model inference, or robot-motion test has been performed yet. Those require the physical ZED stream, measured calibration, and robot hardware.
