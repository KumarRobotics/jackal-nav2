# Semantic Navigation and Exploration Workshop

Last updated: 2026-07-22

Status: design notes and proposed experiments only. None of the systems described
below are implemented merely by the presence of this document.

## Executive summary

The recommended architecture is a hybrid rather than a replacement for Nav2:

- Semantics decide **where to search**, **which reachable subgoal is promising**,
  and **which terrain should be preferred**.
- Nav2 decides whether a subgoal is geometrically feasible and how to reach it.
- A semantic terrain layer biases paths toward roads and away from undesirable
  terrain.
- LiDAR, GroundGrid, and the geometric costmap remain the hard collision and
  traversability authority.

WildOS is the strongest long-term architectural reference for this outdoor Jackal
because it combines persistent graph memory, reachability-gated visual cues,
far-target triangulation, and short-goal handoff to Nav2. OpenFrontier provides a
simpler decision pattern to test first: mark several candidate directions, have a
VLM rank them, and send only the winning safe pose to Nav2.

The first practical work should be two independent proof-of-concept experiments:

1. Project SAM3 terrain masks into an offline semantic costmap and determine
   whether paths can reliably prefer a road over grass or other terrain.
2. Present a VLM with two to four marked, Nav2-reachable directions at T/Y
   junctions and evaluate whether it selects the direction most likely to lead to
   a car.

Only after these pass offline and shadow-mode evaluation should semantic decisions
be allowed to command low-speed robot motion.

## Target capabilities

The intended system should eventually support:

- Search for a target such as a car using a rough spatial prior.
- Search for a target without a prior map by building memory online.
- Use contextual evidence such as roads, parking areas, loading areas, garages,
  curbs, and traffic cones before the target itself becomes visible.
- Prefer roads and benign terrain while avoiding vegetation, mud, water, ditches,
  drop-offs, and terrain that is unsuitable for the Jackal.
- Recover from dead ends without repeatedly selecting the same failed frontier.
- Approach a detected target to a safe, observable standoff pose rather than
  driving toward a raw image or object centroid.

## Recommended system architecture

```text
Natural-language goal + optional rough prior
                     |
              Semantic executive
       +-------------+----------------+
       |             |                |
 candidate views   target belief   semantic memory
 / frontiers       car + context   visited/failed places
       +-------------+----------------+
                     |
      semantic + information + prior utility
                     |
           hard Nav2 feasibility gate
                     |
          short NavigateToPose / Spin goal
                     |
       Smac + MPPI + GroundGrid + STVL
                     |
                   Jackal
```

The semantic executive should operate above Nav2. It may propose or cancel metric
subgoals, but it should never generate velocity commands. A semantic costmap may
add preference or risk costs below the executive, but it must not erase a geometric
obstacle.

### Suggested component boundaries

#### Semantic observation adapter

Convert SAM3 and RGB-D observations into navigation-grade measurements containing:

- Timestamp and camera frame.
- Label and confidence.
- Segmentation mask or polygon.
- Timestamped camera pose.
- Observation origin and bearing.
- Depth-supported centroid and covariance, when available.
- Appearance embedding or keyframe identifier.
- Validity and expiry information.

This adapter is a prerequisite for object goals, semantic terrain projection, and
multi-view triangulation.

#### Candidate-view generator

Produce multiple candidate poses instead of asking an LLM to invent coordinates.
Each candidate should include:

- Stable candidate ID.
- Pose and desired viewing direction.
- Nav2 reachability and path cost.
- Clearance or hazard score.
- Expected information gain.
- Associated image bearing or marked image location.
- Visit, failure, and retirement history.

Candidates can initially be short free-space headings or corridor endpoints. Later
they can be conventional map frontiers, learned visual frontiers, graph frontiers,
or target-inspection viewpoints.

#### Semantic selector

Rank the fixed candidate set using VLM reasoning, embedding similarity, target
belief, rough-prior compatibility, information gain, terrain preference, and search
history. The output should identify a candidate ID and structured scores, not a raw
coordinate or free-form navigation command.

#### Persistent semantic memory

Maintain the information that rolling costmaps cannot:

- Places and traversable connections.
- Views already inspected.
- Objects and contextual observations.
- Positive and negative target evidence.
- Failed goals and dead ends.
- Active, visited, and retired frontiers.

## Fit with the current repository

The current stack already contains most of the low-level pieces, but several gaps
must be addressed before semantic output can safely influence navigation.

### Existing strengths

- Nav2 exposes `NavigateToPose` and uses a cost-aware SmacPlanner2D with MPPI.
- GroundGrid supplies a geometric obstacle cloud for the rolling STVL costmaps.
- ZED RGB and registered depth are available.
- SAM3 accepts runtime-configurable prompts and already computes object masks.
- A VLM service can query the current front-camera image.
- SPINE contains graph, region, object, navigation, and VLM plumbing that could be
  reused for semantic memory and mission-level execution.
- Recording profiles already capture navigation and semantic sensor data.

### Current gaps

1. **SAM3 masks are discarded.**

   `vision_ros2/vision_ros2/sam3.py` computes masks, but its public prediction
   result currently returns boxes, labels, and confidences without preserving the
   masks. Pixel-level grounding is more useful than a bounding box for both terrain
   projection and object depth estimation.

2. **Perception inputs are not synchronized for metric grounding.**

   The detector combines the latest RGB, depth, intrinsics, and odometry values.
   Deprojection uses a bounding-box depth region rather than masked depth, and
   placement does not use a timestamped camera TF including translation. This is
   adequate for approximate observations but not yet for autonomous object approach.

3. **There is no durable exploration map.**

   The local and global costmaps are rolling STVL maps whose observations decay.
   They support collision avoidance, but they do not provide durable known/free/
   unknown state or search history. Conventional global frontier exploration needs
   either an online occupancy map or a separate persistent graph.

4. **GroundGrid information is underused.**

   GroundGrid maintains ground height, height variation, confidence, and related
   grid layers, while Nav2 currently consumes only the obstacle cloud. These layers
   are a natural basis for graded slope, roughness, drop-off, and uncertain-ground
   costs.

5. **The VLM API is EQA-oriented.**

   The current service operates on the latest front image and returns free-form
   text. Navigation needs candidate IDs, structured rankings, uncertainty, and
   decision expiry. SAM3 and the VLM also share GPU resources, so reasoning should
   be event-driven and batched rather than placed in the controller loop.

6. **Existing SPINE frontier logic is not frontier exploration.**

   The current implementation repairs one externally proposed coordinate into a
   feasible location. It does not extract multiple free/unknown boundaries or
   visual frontiers. The graph and action infrastructure are reusable, but the
   frontier generator should be replaced or extended.

7. **Inspection orientation needs explicit handling.**

   The current position goal checker intentionally ignores final yaw. An active
   perception goal therefore needs an explicit Nav2 `Spin`/look behavior or a
   dedicated inspection behavior after reaching the position.

8. **Long-duration memory will inherit odometry drift.**

   `map -> odom` is currently static and DLIO provides locally consistent odometry.
   Small proof-of-concept courses can use this frame directly, but larger loops will
   eventually need loop closure, localization, GNSS fusion, or graph correction.

## Reference approaches

### WildOS

[WildOS](https://arxiv.org/abs/2602.19308) combines:

- A persistent sparse graph generated from local geometric traversability.
- Learned dense visual traversability and visual-frontier predictions.
- Open-vocabulary target similarity from dense image features.
- Multi-view bearing/particle triangulation for targets beyond reliable depth.
- Graph planning followed by short local-goal handoff to Nav2.

WildOS is not a deliberative generative-VLM controller. Its visual module is a
dense perception model with learned frontier and traversability heads plus
open-vocabulary feature similarity. The persistent graph is what allows it to
remember inspected space and return from dead ends.

The authors provide an Apache-licensed, ROS 2 Jazzy-tested
[implementation](https://github.com/nasa-jpl/nebula2-wildos) with graph mapping,
planning, visual inference, and triangulation components. This makes it a strong
candidate for selective reuse after the simpler experiments establish value.

WildOS does not by itself enforce road adherence. Its visual traversability labels
treat multiple off-road surfaces as traversable. Road preference still requires a
separate preference score or semantic cost layer. It also uses an approximate prior
before it can triangulate a target, so completely prior-free search requires an
additional exploration objective.

### OpenFrontier

[OpenFrontier](https://arxiv.org/abs/2603.05377) uses a learned visual-frontier
detector, overlays identifiers such as A/B/C on candidate directions, and asks a
generative VLM how likely each direction is to lead to the language target. It
combines semantic relevance with expected information gain and distance. SAM3 is
used for target detection and verification rather than frontier generation.

Its map-free claim means no prior global map or dense semantic reconstruction; it
still requires camera calibration, pose, depth or local geometry, and sparse state.
The [public implementation](https://github.com/cvg/OpenFrontier) is not a turnkey
ROS 2/Nav2 package, and its learned frontier detector is primarily evaluated
indoors. The candidate-marking and structured VLM-ranking interface is the most
useful part to reproduce first.

## Idea 1: semantic road and terrain costmaps

This is the fastest concrete path toward road adherence.

The Nav2 ecosystem includes a
[SAM3 semantic-navigation tutorial](https://docs.nav2.org/tutorials/docs/navigation2_with_sam3_semantic_segmentation.html)
and an Apache-licensed
[semantic segmentation layer](https://github.com/kiwicampus/semantic_segmentation_layer).
The layer consumes a class mask, label metadata, and a pixel-aligned registered
point cloud, then projects class-specific costs into the navigation costmap. The
tutorial targets ROS 2 Jazzy or newer; distribution compatibility should be checked
before selecting the integration route.

Candidate terrain policy:

| Terrain | Initial navigation treatment |
| --- | --- |
| Paved road, compact trail | Low preference cost |
| Gravel road, firm dirt | Low to moderate cost |
| Short grass, loose gravel | Moderate cost |
| Tall vegetation, mud, deep sand, curb | High cost |
| Water, ditch, drop-off | Lethal only with reliable geometric support |

### Proof-of-concept plan

1. Preserve the masks already generated by SAM3.
2. Synchronize mask, registered depth/cloud, camera information, and timestamped TF.
3. Replay a recorded bag and project terrain labels into a temporary BEV or
   semantic costmap.
4. Display the semantic and geometric layers independently in RViz.
5. Compare paths between two routes, such as a short grass route and a slightly
   longer road, with semantic costs disabled and enabled.
6. Convert GroundGrid height variation, ground confidence, slope, and potential
   drop-off evidence into a separate graded geometric risk.
7. Once projection is reliable, run a low-speed closed course.

### Evaluation

- Fraction of the planned and executed path lying on the road.
- Intrusions into high-risk classes.
- Path-length increase over geometry-only Nav2.
- Semantic projection error relative to surveyed landmarks.
- False-cost persistence, inference latency, and TF failures.
- Human safety interventions.

### Safety contract

Semantic perception may add cost or preference, but it must never clear a
GroundGrid/STVL obstacle. Road preference should begin as a soft bias. A semantic
class should become lethal only when the false-positive consequences and geometric
corroboration are understood.

## Idea 2: OpenFrontier-lite candidate reranking

Do not begin by porting FrontierNet. First test whether the current VLM can choose
useful branches from a bounded set of safe candidates.

Initial candidate sources can include:

- Short free-space headings sampled around the robot.
- Endpoints of locally visible corridors.
- Branch endpoints at a T/Y junction.
- Later, conventional occupancy frontiers, learned visual frontiers, or sparse graph
  frontiers.

### Proof-of-concept plan

1. At a decision point, use an explicit `Spin` behavior to collect a short panorama.
2. Generate several short candidate poses in visible free corridors.
3. Call Nav2 planning for every candidate, rejecting unreachable poses and retaining
   actual path cost and clearance.
4. Associate each candidate with its corresponding image direction and overlay
   labels such as A/B/C.
5. Ask the VLM for structured per-candidate relevance and a short rationale.
6. Run in shadow mode and log the selected branch without issuing robot commands.
7. Compare against random, nearest-candidate, and geometry/information-only
   baselines.
8. After the selector is stable, issue only short, reachable `NavigateToPose` goals,
   inspect again, and repeat.

Example structured VLM result:

```json
{
  "A": {"relevance": 0.75, "reason": "road leads toward parking area"},
  "B": {"relevance": 0.15, "reason": "dense vegetation"},
  "C": {"relevance": 0.45, "reason": "possible loading area"}
}
```

One possible bounded utility is:

```text
U(f) = safe_and_reachable(f) *
       (w_semantic * semantic_relevance
        + w_info * information_gain
        + w_prior * rough_prior_compatibility
        + w_road * road_preference
        - w_cost * nav2_path_cost
        - w_visit * revisit_penalty
        - w_hazard * hazard_uncertainty)
```

All input terms should be normalized. The VLM's self-reported number should be
treated as a ranking signal, not as a calibrated probability. Selection also needs
hysteresis and a minimum commitment period to prevent oscillation between similarly
scored candidates.

### Evaluation

- Top-one branch agreement with a human reference.
- Search success, time, and distance.
- Number of frontiers visited and revisited.
- Unsafe or Nav2-rejected top choices.
- Candidate reversals and oscillations.
- VLM latency, parsing failures, and score stability.

## Idea 3: active object-search belief

For a mission such as "find the car," target location should be represented as an
evolving belief rather than a single detection or VLM answer.

### Proof-of-concept plan

1. Initialize the belief:
   - With a rough prior, use a broad spatial distribution rather than one hard point.
   - Without a prior, distribute belief over unexplored candidate regions.
2. Prompt SAM3 for the target and contextual cues such as vehicle, road, parking
   area, loading area, garage, driveway, curb, and traffic cones.
3. Store each target observation as a mask, confidence, timestamped pose, bearing,
   depth estimate, uncertainty, and best appearance keyframe.
4. When masked depth is reliable, fuse a direct 3D target estimate.
5. When the target is visible beyond reliable depth, accumulate bearing rays from
   multiple poses and triangulate a particle cloud as in WildOS.
6. Reduce belief in deliberately inspected regions where no target was observed.
7. Score candidate views by expected target visibility, contextual evidence,
   information gain, and path cost.
8. Require multi-frame or multi-view confirmation before switching from exploration
   to target approach.
9. Generate a collision-free standoff pose facing the object. Verify the target from
   that viewpoint before declaring success or making a final approach.

### Toy course

Use a T/Y junction with the car hidden down one branch. Put road, parking, or loading
cues on the correct branch and misleading or irrelevant cues on another. Repeat with:

- A correct rough prior.
- A deliberately inaccurate prior.
- No prior.
- A visible and an occluded car.
- No car present.

### Evaluation

- Target-search success rate.
- Search distance and time.
- False target switches and false-positive stops.
- Bearing-triangulation error against surveyed ground truth.
- Number of revisits.
- Standoff-pose position and viewing-angle error.

## Idea 4: sparse semantic topological memory

Persistent memory is the main enabler for long-range no-prior exploration and
dead-end recovery.

Each place node should retain:

- Pose, timestamp, and best camera keyframe or embedding.
- Scene tags and observed objects.
- Free and explored radius or equivalent coverage information.
- Visited, inspected, failed, and retired states.
- Negative evidence such as "searched here; no car observed."
- Edges annotated with Nav2 feasibility, path cost, and traversability.

### Proof-of-concept plan

1. Teleoperate one loop and create nodes at meaningful motion intervals and
   junctions.
2. Attach current SAM3 observations and scene descriptions to the nearest place
   node.
3. Store observation viewpoint and bearing rather than only robot position.
4. Test retrieval queries such as "car near traffic cones" and "road leading toward
   a parking area."
5. Ask the robot to revisit a retrieved node and produce an inspection/standoff goal.
6. Add candidate frontier nodes and retire them after inspection.
7. Test a dead end and verify that the selector returns through the graph instead of
   repeatedly choosing the same branch.
8. If this representation proves useful, compare extending the SPINE graph with
   adopting the released WildOS graph mapper and planner.

## Idea 5: persistent semantic value map

A [VLFM](https://arxiv.org/abs/2312.03275)-style value map is an alternative when a
generative VLM is too slow or unstable. It projects image/text embedding similarity
into a persistent spatial field and uses that field to rank ordinary frontiers.

Example text query:

> An outdoor area likely to contain a parked vehicle.

### Proof-of-concept plan

1. Select posed keyframes from a recorded bag.
2. Compute CLIP/SigLIP similarity for the target and contextual prompts.
3. Project each score into the visible ground region or attach it to a sparse place
   node.
4. Accumulate evidence using confidence, range, and camera viewing angle.
5. Score local candidates from the persistent likelihood field.
6. Compare selection accuracy and latency against generative set-of-marks ranking.

This approach provides spatial persistence without running a large generative VLM
at every decision. A later hybrid could use local embeddings continuously and invoke
the generative VLM only at ambiguous junctions.

## Idea 6: learned Jackal-specific traversability

Hand-authored terrain classes will eventually become brittle. A longer-term system
can learn what this particular Jackal can traverse from its own driving history.

### Proof-of-concept plan

1. Record RGB, registered depth/LiDAR, GroundGrid layers, odometry, commanded and
   achieved speed, IMU vibration, slip estimates, and operator interventions.
2. Project the Jackal's future driven footprint into earlier camera frames to create
   automatic positive traversability labels.
3. Treat excessive slip, getting stuck, high vibration, and operator intervention
   as risky outcomes.
4. Begin with frozen DINO/RADIO features and a small classifier, reconstruction
   model, or nearest-neighbor method.
5. Evaluate entirely offline on held-out environments.
6. Publish risk and uncertainty in shadow mode.
7. Add the learned signal as a soft cost only after it consistently ranks roads
   below mud, vegetation, problematic slopes, and other risky terrain.

The advantage is an embodiment-specific affordance estimate: not merely "this is
grass," but "this appearance and slope have historically been unreliable for this
Jackal."

## Recommended experimental sequence

### Phase 0: measurement foundation

- Preserve SAM3 masks and confidence.
- Synchronize RGB, depth/cloud, camera information, and TF.
- Verify the physical ZED-to-base transform.
- Define semantic observation and candidate-view interfaces.
- Record all observations, candidates, scores, selections, goal outcomes, and
  intervention events.

### Phase 1: offline independent tests

Run two experiments without commanding motion:

1. **Road-cost test:** compare geometry-only and semantic-cost paths on recorded
   road/grass/ditch scenes.
2. **Junction-ranking test:** collect approximately 20-50 decision views, mark two
   to four safe directions, and compare VLM choices with human, nearest, random, and
   information-only baselines.

These isolate semantic projection quality from semantic reasoning quality.

### Phase 2: shadow-mode integration

- Run the candidate selector beside the live navigation stack.
- Publish recommendations and RViz markers only.
- Maintain a small table of visited, failed, and retired candidates.
- Exercise roads, forks, fences, blocked routes, and dead ends.
- Add timeouts and deterministic fallback behavior for perception or VLM failure.

### Phase 3: controlled closed-loop course

- Enable only short, Nav2-reachable goals.
- Run at low speed with manual stop capability.
- Use geometric costmaps as the final safety authority.
- Compare geometry-only, semantic-cost-only, frontier-ranking-only, and combined
  configurations.
- Add confirmed target standoff and explicit look/verification behaviors.

### Phase 4: persistent exploration

- Add a sparse semantic/topological graph.
- Store positive and negative evidence and prevent repeated dead-end selection.
- Add target belief and multi-view triangulation.
- Test correct, incorrect, and absent rough priors.
- Increase course scale only after drift and memory consistency are understood.

### Phase 5: learned visual frontiers and traversability

- Evaluate released WildOS/ExploRFM checkpoints on recorded outdoor Jackal data.
- Compare learned visual frontiers with geometric candidates using the same
  downstream selector and metrics.
- Train or adapt lightweight heads only if the released predictions do not transfer.
- Add Jackal-specific learned terrain risk in shadow mode before costmap fusion.

## Evaluation matrix

| Capability | Baselines | Primary metrics |
| --- | --- | --- |
| Road adherence | Geometry-only path | Road fraction, hazard intrusion, excess path length |
| Semantic branch choice | Random, nearest, maximum information | Top-one accuracy, stability, rejected candidates |
| Object search | Geometric exploration, rough-prior-only | Success, SPL-like efficiency, time, false stops |
| Search memory | No memory, simple visited blacklist | Revisits, dead-end recovery, oscillations |
| Target localization | Bounding-box depth | Mask-depth error, triangulation error, false switches |
| Learned traversability | Class costs, GroundGrid-only | Risk ranking, intervention prediction, uncertainty calibration |

Every physical experiment should additionally report human interventions, minimum
obstacle clearance, localization/TF failures, model latency, stale-observation rate,
and fallback activations.

## Design rules and anticipated failure modes

1. **Never use the VLM as a controller.** It ranks bounded candidate IDs at decision
   events; Nav2 controls the robot.
2. **Geometry is a hard gate.** An attractive semantic candidate is discarded when
   Nav2 cannot produce a safe path.
3. **Semantic costs do not clear obstacles.** False-positive road pixels must not
   make a geometric obstacle traversable.
4. **Use short receding-horizon goals.** Re-evaluate after meaningful motion, a
   changed frontier set, goal success/failure, or new target evidence.
5. **Add hysteresis and commitment.** Frame-to-frame score noise must not repeatedly
   cancel goals or switch distant frontiers.
6. **Use explicit scan and inspect actions.** The front-only camera and position-only
   goal checker otherwise leave important areas unseen.
7. **Represent negative evidence.** "Looked here and did not see the car" is essential
   for efficient search.
8. **Handle uncertainty explicitly.** Confidence, timestamp, covariance, and expiry
   should travel with every semantic observation and decision.
9. **Provide deterministic fallbacks.** Low confidence, stale data, malformed VLM
   output, or GPU timeout should fall back to geometric exploration or stop safely.
10. **Avoid semantic replanning assumptions.** The current behavior tree replans
    when the goal changes or the path becomes invalid. Changes to soft semantic cost
    alone may not trigger a new path, so semantic experiments may need an explicit
    event-driven replan policy or short successive goals.

The current server launch also does not start Nav2 Collision Monitor. Initial
closed-loop semantic experiments should retain low speed, geometric obstacle layers,
manual stop capability, and the existing platform safety mechanisms.

## Immediate combined demonstration

A useful first end-to-end toy demonstration is:

```text
T/Y junction with car hidden beyond one branch
             |
VLM ranks marked, Nav2-reachable branches
             |
semantic costmap keeps the selected path on the road
             |
SAM3 creates a synchronized target hypothesis
             |
Nav2 reaches a collision-free inspection pose
             |
multi-frame detection or VLM verifies the car
             |
Nav2 reaches a safe standoff pose
```

Vary the experiment with valid and misleading context, correct and incorrect rough
priors, a directly visible and an occluded car, unequal route lengths, and no car at
all. This single course tests semantic reasoning, terrain preference, exploration
memory, target grounding, and safe termination while remaining small enough to
debug systematically.

## Relevant repository files

- `config/nav2_jackal_stvox_mppi_tuning.yaml`: active rolling STVL, Smac, MPPI,
  behavior, and goal-checker configuration.
- `launch/nav2_servers.launch.py`: launched Nav2 servers and lifecycle nodes.
- `launch/jackal_static_transforms.launch.py`: current map/odom and camera transforms.
- `launch/record_jackal.launch.py`: navigation and semantic recording profiles.
- `../vision_ros2/vision_ros2/sam3.py`: SAM3 masks and current prediction output.
- `../vision_ros2/vision_ros2/detector.py`: observation synchronization,
  deprojection, and tracking.
- `../vision_ros2/vision_ros2/vlm_node.py`: current free-form scene-query service.
- `../groundgrid/src/GroundGridNode.cpp`: GroundGrid outputs supplied to Nav2.
- `../spine-multi/src/spine_multi/spine/mapping/frontiers.py`: current SPINE
  coordinate-feasibility logic.
- `../spine-multi/ros/spine_multi_ros/spine_multi_ros/tracker_client.py`: attachment
  of tracked objects to graph regions.

## External references

- WildOS paper: <https://arxiv.org/abs/2602.19308>
- WildOS project: <https://leggedrobotics.github.io/wildos/>
- WildOS ROS 2 implementation: <https://github.com/nasa-jpl/nebula2-wildos>
- OpenFrontier paper: <https://arxiv.org/abs/2603.05377>
- OpenFrontier project: <https://boysun045.github.io/OpenFrontier-Project/>
- OpenFrontier implementation: <https://github.com/cvg/OpenFrontier>
- Nav2 SAM3 semantic navigation tutorial:
  <https://docs.nav2.org/tutorials/docs/navigation2_with_sam3_semantic_segmentation.html>
- Semantic segmentation costmap layer:
  <https://github.com/kiwicampus/semantic_segmentation_layer>
- Nav2 Ground Consistency Layer tutorial:
  <https://docs.nav2.org/tutorials/docs/navigation2_with_ground_consistency_layer.html>
- VLFM: <https://arxiv.org/abs/2312.03275>
- ConceptGraphs: <https://arxiv.org/abs/2309.16650>
- RoadRunner: <https://arxiv.org/abs/2402.19341>
