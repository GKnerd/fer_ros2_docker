# Status

Updated: 2026-10-03

## Current step

Phase 8.5 — behavior trees (`fer_bt_server`). First full PickPlace SUCCESS in MuJoCo on
2026-10-02. Place candidates (8.4) are implemented and tested.

## Phase 8

| Step | State |
|---|---|
| 8.0 Groundwork | done |
| 8.1 `fer_world_model` | done, committed |
| 8.2 `fer_gripper_server` | done, committed |
| 8.3 `fer_moveit_motion_server` | done, committed; real-robot checks open |
| 8.4 `fer_grasp_planner` | grasp candidates done; place candidates done, uncommitted |
| 8.5 `fer_behavior_trees` | implemented, PickPlace runs in MuJoCo, uncommitted |
| 8.6 Bringup | not started (`launch/fer_manipulation.launch.py` exists, untracked) |
| 8.7 Removal | not started |

## Uncommitted work

| Repo | Changes |
|---|---|
| `fer_behavior_trees` | server, launch, tests, `pick_place.xml` |
| `fer_grasp_planner` | place server, catalog, config, README |
| `fer_gripper_server` | `config/gripper_mujoco.yaml` |
| `fer_interfaces` | `PlaceCandidate.msg`, `GetPlaceCandidates.srv` (new), `CheckReachable.srv` |
| `fer_moveit_config` | lift fix in `MoveGroupClient`, `fer_moveit_launch.py`, tests |
| `fer_ros2_bringup` | controller gains, `package.xml`, `fer_manipulation.launch.py`, `scripts/` (new) |
| `fer_ros2` | `Repo_Hotfixes.md` (new) |
| `fer_ros2_docker` | plan doc, `run_container.sh` |

## Next

1. Commit 8.4 and 8.5 work.
2. Decide the open MuJoCo items below (duration monitoring, box force).
3. 8.6 bringup.

## Open items

- **Execution duration monitoring is off** in `fer_moveit_launch.py` (joint 7 lags its plan by
  up to 0.28 rad). The motion server watchdog (2×T + 7 s) is the only time limit. Options:
  re-enable with scaling 1.5; `FindReachableGrasp` prefers the least joint-7 travel.
- **Catalog `box: {force: 5.0}`** is shared with the real robot. Alternative: stiffer object
  contacts in `pick_place_world.xml` (solref 0.004 1, solimp 0.95 0.99 0.001), back to 15 N.
- Grasp closing speed is tied to force (finger trajectory controller later).
- A failed lift leaves the box attached (no release fallback).
- A failed Release makes the watch mark the object LOST.
- World model can disagree with the physical grasp if `SetObjectStatus` fails or a Grasp is
  cancelled after closing.
- `Gripper` in `core/grasp.py` is a `typing.Protocol`; switching to an ABC is undecided.
- `MultiThreadedExecutor(num_threads=…)` fixed in both servers; named constant for the
  0.05 s / 0.1 s stop-poll intervals.
- Old stack (`fer_skills`) not yet checked against the changed `fer_moveit_launch.py`.
- Real robot: 8.3 checks, `is_grasped`.
- magic numbers on the polling interval in `fer_gripper_server` and in `fer_world_model`. 
- Protocol used for the Gripper instead of ABC, check if is worth to change it up.
- Fixed executor thread count
- grasp-versus-world-model mismatch - needs to be watched at runtime
- MuJoCo scene objects and how they are added to the world_model when sim is used

## Known limits

- `move_group` in plan-only mode replaces planner error codes with FAILURE, so every planning
  failure reports NO_PATH. Trees never branch on UNREACHABLE.
- Objects written to `move_group`'s scene survive a motion server restart.
- MuJoCo gripper needs `hand_control_type:=effort`; closed on nothing, `fer_finger_joint1` sits
  ~0.85 mm below its lower limit.

## Decisions

| Date | Decision | Reason |
|---|---|---|
| 2026-09-26 | Skip Phase 2 (all sim command interfaces) | Servers pick their controller at startup; no runtime switching needed |
| 2026-09-28 | Servers never switch controllers | Activation stays explicit, not hidden in adapters |
| 2026-09-29 | Motion server is a `move_group` client | Keeps the MotionPlanning panel, one parameter set, tested MoveIt execution |
| 2026-09-30 | Pick retried `pick_attempts`; failed Place ends the tree; SUCCESS only if all placed | Simple first policy, refine later |
| 2026-09-30 | Place target orientation = object's full orientation; `place_clearance` 0.005 | Literal contract; must be >0 (attached box vs table) |
| 2026-10-02 | Robot not raised in the MuJoCo scene | Would need a description change or a MuJoCo-only offset |
| 2026-10-02 | MuJoCo damping unchanged | Same dynamics worked with the old stack |
