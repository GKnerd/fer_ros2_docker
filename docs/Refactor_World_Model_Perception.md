# Perception → World Model → Behavior Tree: object-centric platform plan

## Context

`chained_pick_place.xml` hardcodes `object_id="cylinder_1"`. That name only works
because `mock_camera` happens to publish a CollisionObject with exactly that id.
On the real robot nothing populates the scene, so every pick fails with
`OBJECT_NOT_IN_SCENE`.

The root problem is architectural, not the literal:

| Today | Consequence |
|---|---|
| Perception (`mock_camera`) writes MoveIt `CollisionObject`s to `/planning_scene` directly | Every perception node must know MoveIt, invent ids, keep them stable |
| World model is a read-only mirror of `/monitored_planning_scene` | World model is MoveIt-coupled; GRASPED guard only flags *after* the scene is already overwritten |
| Objects carry only id + geometry | No class, score, source — a BT cannot ask "give me the cylinders" |
| Query is `std_srvs/Trigger` returning JSON | Untyped; BT never uses it |
| BT trees carry object ids as literals | Tree only runs against one hand-written scene |

**Goal:** any detector (mock now, RealSense D405 + detector later) plugs in by publishing
one standard message. The world model is the planner-independent source of truth.
Trees ask the world model for objects and pass *ids* through the blackboard.
Replacing MoveIt means replacing one bridge node, not the world model or the BT.

## Decisions (agreed)

1. **World model is MoveIt-independent.** It owns object identity and state; a separate
   bridge writes its view into whatever planner is in use (MoveIt today).
2. **Perception contract = `vision_msgs/Detection3DArray`** (class, score, pose, bbox),
   stamped at capture time, in the camera frame. Nothing else is required from a detector.
3. **Detection mode = snapshot + refine.** One global snapshot from a viewing pose; one
   close-up snapshot at pre-grasp before each pick. Cost ≈ 1–3 s per pick (settle +
   detect + MTC replan), ~5–15 % of a cycle; buys D405 close-range accuracy (7–50 cm
   sweet spot) and avoids missed grasps from global-view error. The world model is built
   stream-capable (association, gating) so continuous mode is a later gate relaxation,
   with no perception change.
4. **Grasp state layering.** Skill provides *identity* ("Pick succeeded on X" → X GRASPED).
   A separate grasp monitor provides *evidence* (gripper width + external-wrench residual).
   The world model flags disagreement (phantom grasp / drop). Residual export belongs in
   the driver as `geometry_msgs/WrenchStamped` (consolidation plan §11); the monitor is its
   own package, sim + real.
5. **Blackboard holds references, world model holds truth.** Goal payload → global
   blackboard (task arguments, e.g. `target_class`, `place_zone`). Query results (object
   ids) → tree blackboard. The world is never copied wholesale into the blackboard
   (that would be a second, stale world model).

## Target data flow

```
 detector (mock | D405+detector)            ── publishes ──►  /perception/detections
   [only depends on vision_msgs]                               vision_msgs/Detection3DArray

 fer_world_model  (no MoveIt, no franka)
   ├─ ~/detect_objects   action   snapshot: take next detections after request,
   │                              transform to base (TF at stamp), associate, commit
   ├─ ~/query_objects    srv      filter by class / status / region → WorldObject[]
   ├─ ~/set_object_status srv     skills report GRASPED/FREE (+ held_by)
   ├─ grasp evidence sub          from grasp monitor → conflict flags
   └─ ~/objects          topic    latched WorldObjectArray (full view)

 fer_world_model_moveit (bridge)  ~/objects ──► /planning_scene diffs (FREE objects only;
                                   attached objects stay MTC's)            [swap for another planner]

 fer_behavior_trees
   payload {"target_class": "cylinder", ...} ──► global blackboard {@target_class}
   DetectObjects → QueryObjects(class={@target_class}) → {object_ids}
   LoopString over {object_ids} as {object_id}:
       GetObjectPose → MoveToPose(pre-grasp) → DetectObjects(refine) → PickObject → PlaceObject → GoHome
```

## Phases

### Phase 1 — Contracts (`fer_world_model_interfaces`, new package, public API per §2.6)
- `msg/WorldObject.msg`: `id`, `class_id`, `score`, `geometry_msgs/PoseStamped pose`,
  `shape_msgs/SolidPrimitive shape` (+ optional mesh resource), `status`, `held_by`,
  `last_observed`, `conflict`.
- `msg/WorldObjectArray.msg`.
- `srv/QueryObjects.srv`: filters (class_id, status, optional region), sort (distance to
  frame / score), max count → `WorldObject[]`.
- `srv/SetObjectStatus.srv`: id, status, held_by.
- `action/DetectObjects.action`: optional class filter, timeout → added/updated/missing ids.
- Depends only on `std_msgs`, `geometry_msgs`, `shape_msgs`, `builtin_interfaces`.
- Add `ros-jazzy-vision-msgs` to the image (`docker/Dockerfile` / rosdep).

### Phase 2 — World model core (pure Python, `fer_world_model/core/`)
Reuse and extend what exists (`planning_scene_object.py`, `planning_scene_world_model.py`,
`result.py` — already ROS-free and returning `WMResult`):
- Add `class_id`, `score`, `source` to the object schema; keep FREE/GRASPED lifecycle and the
  GRASPED guard (it now guards *before* anything reaches the planner).
- New `association.py`: match detections to existing FREE objects by class + gating distance;
  unmatched → new id `<class>_<n>` (world model owns ids); GRASPED objects never updated by
  detections (gripper occlusion).
- New `catalog.py` + `config/object_catalog.yaml`: `class_id → shape` (primitive dims or mesh),
  grasp hints later. Fallback: box from `Detection3D.bbox.size`.
- Commit gating hooks (movement deadband, locked target) — trivial in snapshot mode, the
  switch for continuous mode later.
- Unit tests (pytest) for association, lifecycle, guard, catalog.

### Phase 3 — World model node (MoveIt removed)
- `planning_scene_world_model_server.py` → rewritten against Phase 1 interfaces: subscribes
  `/perception/detections`, serves `detect_objects` / `query_objects` / `set_object_status`,
  publishes `~/objects`. Drops `/get_planning_scene`, `/monitored_planning_scene`, `moveit_msgs`.
- TF lookup at detection stamp into `base`. Snapshot = first detection message stamped after
  the request, taken with the arm stationary (sidesteps the 30 Hz `joint_state_publisher`
  rate in `fer_real_ros2_control.launch.py`).
- `planning_scene_adapter.py` moves to the bridge (Phase 4).

### Phase 4 — MoveIt bridge (`fer_world_model_moveit`, new package)
- Subscribes `~/objects`, publishes `/planning_scene` diffs: ADD/MOVE/REMOVE for FREE objects.
- Never touches attached objects (MTC owns attach/detach + ACM).
- Holds updates while a trajectory executes (avoids MoveIt aborting on scene jitter).
- This is the only package to replace when MoveIt is swapped out.

### Phase 5 — Mock perception
- Replace `mock_camera_node.py` with `mock_perception`: reads YAML (class, pose, size — **no ids**),
  publishes `Detection3DArray` on `/perception/detections` in `base` (or a fake camera frame).
- Same contract as the D405 node will use; proves the plug-in path.

### Phase 6 — Skills report grasp state
- `fer_skills` Pick/Place call `set_object_status` on success (GRASPED/FREE, `held_by`).
- Skills depend on `fer_world_model_interfaces` only.
- MoveIt-side checks (object in planning scene) stay inside the MoveIt skill backend.

### Phase 7 — BT nodes and trees (`fer_behavior_trees`)
- `onGoalReceived`: JSON payload → global blackboard; unset previous goal's keys; reject
  malformed payload.
- New nodes: `DetectObjects` (RosActionNode), `QueryObjects` (RosServiceNode → output
  `std::vector<std::string> object_ids`), `GetObjectPose` (RosServiceNode → PoseStamped +
  pre-grasp offset for `MoveToPose`).
- Iterate with BT.CPP 4.10 `LoopString` (accepts `std::vector<std::string>`).
- Rewrite `chained_pick_place.xml`; `{@target_class}` from payload; Fallback on Pick failure →
  re-detect (robotics_stack.md task 6a/3a).
- Place targets stay literal for now (named locations = later phase).

### Phase 8 — Launch integration (`fer_ros2_bringup`)
- `perception:=mock|none` arg on `fer_moveit_skills_bt.launch.py`; launch world model +
  bridge + mock perception.

### Later (out of this plan's implementation scope, interfaces already fit)
- Driver: export external wrench as `WrenchStamped` (§11); `fer_grasp_monitor` package
  (gripper width + residual, `set_load()` handling, drop detection) → evidence topic consumed
  by world model.
- D405 perception node in its own container; hand-eye TF published by the perception host.
- Named place locations/zones.
- Continuous mode: relax world-model gates, add frustum/occlusion-aware "missing".
- Rename `fer_world_model` → scene manager (robotics_stack.md note).

## Critical files
- `fer_planning_world_model/fer_world_model/core/*.py` — extend (reuse lifecycle, guard, `WMResult`)
- `fer_planning_world_model/fer_world_model/planning_scene_world_model_server.py` — rewrite
- `fer_planning_world_model/fer_world_model/planning_scene_adapter.py` — move to bridge
- `fer_planning_world_model/fer_world_model/mock_camera_node.py` → `mock_perception`
- `fer_planning_world_model/config/scene_objects.yaml` → detections fixture (no ids) + `object_catalog.yaml`
- `fer_skills/src/skills/pick_object.cpp`, `place_object.cpp` — report status
- `fer_behavior_trees/src/bt_server_node.cpp` (+ hpp) — payload → global blackboard; register new nodes
- `fer_behavior_trees/trees/chained_pick_place.xml`, `mock_pick_place.xml`
- `fer_ros2_bringup/launch/fer_moveit_skills_bt.launch.py`
- `docker/Dockerfile` — vision_msgs

## Verification
- Phase 2: `colcon test --packages-select fer_world_model` — association, guard, catalog unit tests.
- Phase 3–5 (sim): mock perception up → `ros2 action send_goal .../detect_objects` →
  `ros2 service call .../query_objects "{class_id: cylinder}"` returns world-model ids;
  RViz planning scene shows the bridge-written objects.
- Phase 6: pick in sim → `query_objects` shows GRASPED/held_by; place → FREE.
- Phase 7: `ros2 action send_goal /execute_tree ... "{target_tree: ChainedPickPlace, payload: '{\"target_class\": \"cylinder\"}'}"`
  runs without any id literal in XML; missing payload → tree fails at first node, arm never moves;
  second goal without payload does not reuse the first goal's keys.
- Real robot: mock perception with objects placed at YAML poses; then the same tree unchanged.
- MoveIt independence check: `fer_world_model` and `fer_world_model_interfaces` package.xml contain
  no `moveit_*` dependency.
