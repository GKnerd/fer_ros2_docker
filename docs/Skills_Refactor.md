# fer_skills refactor: split skills from MoveIt, gripper as its own actuator

## Context

`ControlGripper` fails on the real robot for two reasons:
1. `src/skills/control_gripper.cpp:40` hardcodes the named target `"ready"`, which `fer_hand` does not have (SRDF has only `open`/`close`), and ignores the goal's `position`/`max_effort`.
2. `franka_gripper` publishes closed fingers at −2.6e‑6. MoveIt 2.12.4's `CheckStartStateBounds` has no tolerance and rejects every hand plan. The `start_state_max_bounds_error: 0.1` in both `fer_moveit_config/launch/fer_moveit_launch.py:167` and `fer_skills/launch/fer_skills.launch.py` is not a parameter in 2.12.4 (verified in the installed `.so`); the real knob is `fix_start_state`.

The deeper problem: `SkillServer` is a god class. It owns both MoveGroupInterfaces, MTC planners, pick/place, config loading and five action servers, so every skill depends on MoveIt. In Pick/Place the gripper is a *segment of an MTC trajectory* executed by move_group. That means a fixed effort, no grasp feedback, the object attached on schedule rather than on evidence, and `close`=0.0 makes franka's `grasp()` report failure while holding an object.

Target, following the common pattern outside MoveIt (cuRobo `plan_grasp`, Drake, Tesseract):
- **Planners produce arm motion only.** MTC still plans the whole task, with the hand modelled.
- **The gripper is its own actuator with its own result.**
- **World-model attach/detach happens only after gripper feedback confirms it.**
- **Hardware vs. simulation differ only in launch/config.** Both expose `control_msgs/GripperCommand` + `FollowJointTrajectory`.

## Target architecture

```
fer_skill_interfaces   actions only (moved from fer_skills) ← fer_behavior_trees depends on this only
fer_skills
  core/  (no MoveIt includes)
    ports/  ArmMotion, Gripper, Manipulation, WorldModel      (abstract C++ interfaces)
    step_plan.hpp   Step = variant<ArmTrajectory, GripperAction, SceneChange>
    step_executor   runs a StepPlan against the ports; gates attach/detach on gripper result
    skills/         GoHome, MoveToPose, ControlGripper, PickObject, PlaceObject (thin, port-only)
    skill_action_server.hpp  template: goal/cancel/accepted/thread boilerplate (today copy-pasted 5×)
  backends/
    moveit_arm_motion        MoveGroupInterface (named/pose targets, execute JointTrajectory)
    mtc_manipulation         MTC task build + plan → StepPlan (never calls task.execute)
    moveit_world_model       attach/detach + ACM allow/forbid via PlanningScene diffs
    gripper_action_client    rclcpp_action client for control_msgs/GripperCommand (no MoveIt)
  skill_server_main.cpp      composition root: builds backends from params, wires skills
```

Port sketch (C++17, ROS message types only, no MoveIt types):
- `ArmMotion`: `moveToNamed(name, scaling)`, `moveToPose(PoseStamped, scaling)`, `execute(trajectory_msgs::JointTrajectory)`. Each returns `MotionResult{code, message}`.
- `Gripper`: `command(position, max_effort)` returns `GripperResult{ok, holding, width, message}`. `open()` and `grasp(width, force)` are convenience wrappers.
- `Manipulation`: `planPick(PickRequest)` and `planPlace(PlaceRequest)` return `PlanOutcome{code, message, StepPlan}`.
- `WorldModel`: `attach(object, link, touch_links)`, `detach(object)`, `setCollision(object, links, allow)`, `hasObject(id)`.

One `SkillCode` enum maps to the existing `message` strings: `OK, PLANNING_FAILED, EXECUTION_FAILED, CANCELLED, INVALID_GOAL, OBJECT_NOT_IN_SCENE` plus new `GRASP_FAILED, RELEASE_FAILED, GRIPPER_FAILED`. Action result fields stay `success`/`message`, so BT client nodes need no logic change.

## Phases (each builds and runs on its own)

### Phase 0: MoveIt config (fixes the planning abort)
- In both pipeline blocks (`move_group` in `fer_moveit_launch.py`, `ompl` in `fer_skills.launch.py`), replace `start_state_max_bounds_error` with `fix_start_state: True`. The skill node's block matters because MTC's `PipelinePlanner` plans locally in `fer_skill_server`.
- Real bringup: set the `franka_gripper` params `default_grasp_epsilon.inner/outer` (e.g. outer 0.04) so closing on an object of any width inside the stroke counts as a successful grasp. Find where the gripper node is launched in `fer_ros2_bringup/launch/fer_real_ros2_control.launch.py` / `fer_ros2/franka_gripper` launch.

### Phase 1: Gripper port + ControlGripper (fixes the reported bug)
- `core/ports/gripper.hpp`, `backends/gripper_action_client.{hpp,cpp}`: one async `GripperCommand` client whose action name is a parameter.
  - Normalize results: `holding = (reached_goal || stalled) && width > gripper.min_hold_width`.
  - `width` comes from `result.position`, falling back to the latest finger value on `/joint_states`. franka's `onExecuteGripperCommand` doesn't always fill `result.position`, so check this during implementation.
- Parameter `gripper.action_name`, set per hardware in `fer_skills.launch.py`:
  - real: `/fer_gripper/gripper_action`;
  - mujoco: `/gripper_<hand_control_type>_controller/gripper_cmd`.
  - Forward `hardware` + `hand_control_type` from `fer_ros2_bringup/launch/fer_moveit_skills.launch.py` into `fer_skills.launch.py`; today they aren't passed.
- Rewrite `ControlGripper` against `Gripper`, honouring goal `position` and `max_effort`. Remove the `hand_` MoveGroupInterface use from it.

### Phase 2: interfaces package + server skeleton
- New `fer_skill_interfaces` (ament_cmake + rosidl) holding the 5 `.action` files. `fer_skills` stops generating interfaces.
- `fer_behavior_trees`: depend on `fer_skill_interfaces`, change includes/aliases from `fer_skills/action/...` to `fer_skill_interfaces/action/...` in the 5 client headers. This removes MoveIt/MTC from the BT build graph.
- `skill_action_server.hpp`: a template that owns the server plus the accept/cancel/detached-thread boilerplate. Each skill implements `run(goal, handle) -> SkillOutcome`. Replace `SkillServer`/`skill_server_node.{hpp,cpp}` with `skill_server_main.cpp` doing the composition.
- Update `DESIGN.md`: the layer diagram and the "Gripper API surface" decision.

### Phase 3: ArmMotion
- `backends/moveit_arm_motion` wraps the existing `arm_` MoveGroupInterface logic from `go_home.cpp`/`move_to_pose.cpp`.
- GoHome/MoveToPose become port-only skills. `execute(JointTrajectory)` wraps into `moveit_msgs::RobotTrajectory` + `MoveGroupInterface::execute`, which routes to the active arm controller.

### Phase 4: WorldModel
- `backends/moveit_world_model`:
  - attach/detach via `AttachedCollisionObject` diffs through `PlanningSceneInterface::applyPlanningScene`;
  - ACM allow/forbid via `/get_planning_scene` (ALLOWED_COLLISION_MATRIX), edit with `collision_detection::AllowedCollisionMatrix`, apply as a diff.
- Move the `OBJECT_NOT_IN_SCENE` pre-check from `pick_object.cpp` into `WorldModel::hasObject`, and drop the diagnostic dump.
- `fer_planning_world_model` keeps mirroring `/planning_scene` and needs no change. Check that it still sees attach/detach from the diffs (it already folds `attached_collision_objects`).

### Phase 5: Manipulation (MTC plans, core executes)
- `backends/mtc_manipulation` absorbs `mtc_pick_object`, `mtc_place_object`, `mtc_common`, `mtc_planners`. Stage construction is kept; `execute_task()` is removed.
- Plan "close hand" to a realistic width (`gripper.default_grasp_width` / goal) with a joint-value `MoveTo` instead of the named `close`. The planned finger state then matches reality.
- After `task.plan()`, convert the front solution (`toMsg`) and walk `sub_trajectory[]` into a `StepPlan`:
  - non-empty trajectory whose `joint_names` ⊂ arm group → `ArmTrajectory` (arm joints only);
  - non-empty hand trajectory → `GripperAction{Open|Grasp}`. Open vs. grasp is decided by comparing the end with the start width. Width and force come from config/goal, not from the planned value;
  - empty trajectory → `SceneChange`, classified from the backend's own stage names (`info.stage_id` → stage it created) into attach/detach/allow/forbid;
  - `SceneChange{attach}` after a grasp and `{detach}` after an open are marked `gated`.
- `core/step_executor`: runs the steps in order and checks cancel before each.
  - Publishes per-step feedback (`current_phase` = step label, progress = i/n).
  - A failed gated gripper step → skip the gated change, run recovery (Pick: open + abort `GRASP_FAILED`; Place: abort `RELEASE_FAILED`, object stays attached), and stop. No further segments run.
- PickObject/PlaceObject become: `hasObject` → `planPick/planPlace` → `StepExecutor::run`.

### Phase 6: cleanup + tests
- Remove `moveit_*` from `find_package` for anything under `core/`. Build `core` as its own library target without MoveIt includes, so the compiler enforces the boundary.
- gtest with fake ports (`test/fakes/`):
  - executor gating: grasp fails → no attach, `GRASP_FAILED`;
  - cancel between steps;
  - ControlGripper result mapping.
- A shared BT (`fer_behavior_trees/trees/test_tree.xml`, extended with Pick/Place) run on both `hardware:=mujoco` and `hardware:=real`.

## Critical files
- `fer_skills/src/skills/*.cpp`, `fer_skills/src/skill_server_node.cpp`, `fer_skills/include/fer_skills/skill_server_node.hpp`: split into core skills + main.
- `fer_skills/src/mtc_*.cpp`, `include/fer_skills/mtc_*.hpp`: move to `backends/mtc_manipulation`. Reuse `make_move_to_named`, `make_connect`, `make_relative`, `make_modify_collisions`, `make_attach`, `plan_task`, `load_*_config`.
- `fer_skills/CMakeLists.txt`, `package.xml`, `launch/fer_skills.launch.py`, `config/skill_server_params.yaml` (add the `gripper.*` params: `action_name`, `min_hold_width`, `open_width`).
- `fer_behavior_trees/package.xml`, `CMakeLists.txt`, `include/fer_behavior_trees/*_client_node.hpp`.
- `fer_moveit_config/launch/fer_moveit_launch.py`, `fer_ros2_bringup/launch/fer_moveit_skills.launch.py`, real gripper launch params.

## Verification
- Phases 0–1:
  - With the gripper physically closed on the real robot, run `test_tree.xml`: no `CheckStartStateBounds` error.
  - `ControlGripper position=0.04` opens; `position=0.0 max_effort=20` on an object returns success.
  - `ros2 action send_goal /control_gripper ...` gives the same result on mujoco.
- Phase 2: `colcon build --packages-up-to fer_behavior_trees` doesn't pull MoveIt/MTC (check `colcon graph`); the BT still runs GoHome.
- Phases 3–5:
  - Pick on mujoco and real, with feedback showing each step.
  - Pick with no object (the gripper closes empty) → `GRASP_FAILED`, the scene shows no attached object, the gripper reopens.
  - Place → detach only after the open succeeds.
  - `fer_world_model` state matches.
- Phase 6: `colcon test --packages-select fer_skills` passes the fake-port unit tests.
