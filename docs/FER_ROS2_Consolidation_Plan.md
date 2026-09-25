# FER ROS 2 — Sim/Real Consolidation and Platform Refactor Plan

**Date:** 2026-09-25
**Supersedes:** items #2 and #3 of `FER_ROS2_Review_2026-08-24.md`;
`legacy/Skills_Refactor.md`; `legacy/Refactor_World_Model_Perception.md`

---

## 0. Purpose and scope

Collapse the two parallel stacks (`fer_ros2_docker`, `fer_ros2_mjc_docker`) into one
platform where **simulation vs. reality is a launch argument, not a package
identity**. The per-package repos and the `.repos` workflow are kept;
`fer_ros2_docker` becomes the single meta-repo and `fer_ros2_simulation` is archived.

**In scope:** meta-repo consolidation, `.repos` profiles, description unification,
controller naming, the sim hardware interface, the gripper seam, launch topology,
CI, multi-host deployment layout.

**Out of scope:** SSM loop closure.

**Follow-up after the consolidation:** platform refactor — `fer_interfaces`, world
model without MoveIt, gripper server, swappable motion backend, pick and place as
behavior trees (Phase 8).

**Deferred:** Cartesian impedance control, the wrench path (§11), a mock hardware
backend (Phase 6). Before the impedance controller is written, decide where its
dynamics model comes from (§10).

---

## 1. Starting state

Verified by direct inspection on 2026-09-16.

### 1.1 Repository inventory (before)

| Repo | Branch | Role |
|---|---|---|
| `GKnerd/fer_ros2_docker` | `jazzy` | real meta-repo |
| `GKnerd/fer_ros2_simulation` | `main` | sim meta-repo |
| `GKnerd/fer_ros2` | `jazzy` | real driver port (multipanda→Jazzy), incl. `fer_bringup` |
| `GKnerd/fer_ros2_mjc_bringup` | `main` | sim bringup |
| `GKnerd/fer_moveit_config` | `jazzy-devel` | shared |
| `GKnerd/fer_skills` | `main` | shared |
| `GKnerd/fer_behavior_trees` | `main` | shared |
| `GKnerd/fer_planning_world_model` | `main` | sim only (package name `fer_world_model`) |
| `GKnerd/speed_and_separation_monitoring` | `main` | sim only |
| `GKnerd/fer_speed_and_separation_monitoring` | `main` | empty — README only |
| `GKnerd/fer_ws` | `main` | trial monorepo (subtrees of five repos) |

Externals: `frankarobotics/franka_description` @ `72baf5b`,
`frankarobotics/libfranka` @ `0.9.2`, `BehaviorTree/BehaviorTree.ROS2` @ `6c6aa07`,
`ros-controls/mujoco_ros2_control` @ `375a5db`, `pal-robotics/mujoco_vendor` @ `26187d6`.

`fer_custom_interfaces` is an empty `ros2 pkg create` skeleton committed inside
`fer_ros2_simulation`.

### 1.2 The dependency fact that drives the design

`fer_skills`, `fer_behavior_trees`, `fer_moveit_config`, `fer_world_model` and
`speed_and_separation_monitoring` contain zero references to `franka_msgs`,
`franka_hardware`, `franka_gripper`, `franka_semantic_components` or
`franka_robot_state_broadcaster`. Everything above the driver reaches the hardware
only through ros2_control.

### 1.3 Confirmed drift

- Four independent `robot_description` producers: `fer_ros2/fer_bringup/urdf/fer.xacro`,
  `fer_ros2_mjc_bringup/urdf/fer_mujoco.urdf.xacro`,
  `fer_moveit_config/launch/fer_moveit_launch.py:91`,
  `fer_skills/launch/fer_skills.launch.py:53`.
- Controller names: real has `effort_joint_trajectory_controller` /
  `vel_joint_trajectory_controller`; sim and MoveIt have
  `joint_effort_traj_controller` / `joint_pos_traj_controller`. No overlap.
- `use_sim_time` defaults to `true` in `fer_moveit_launch.py:263` and
  `fer_skills.launch.py`; nothing on the real side overrides it.
- `fer_ros2_mjc_bringup/CMakeLists.txt:10-14` calls `find_package(... REQUIRED)` on
  five packages while compiling nothing — the sole build-time coupling to MuJoCo.
- Dead config: `cartesian_compliance_controller` and `motion_control_handle` in
  `franka_mujoco_controllers.yaml`.
- `fer_ros2_mjc.repos` omits `fer_world_model`, `speed_and_separation_monitoring`
  and `fer_custom_interfaces`, all of which are in `src/`.
- Branch conventions differ across repos (`jazzy`, `jazzy-devel`, `main`).

### 1.4 Both stacks are working references

The driver runs clean on the real Franka, and the sim stack is demonstrated working.
Every phase is validated against both environments.

---

## 2. Target repository topology

### 2.1 Principles

1. **One meta-repo assembles the platform.** It holds Docker, env, config, profile
   manifests, integration CI, scripts and all docs. It contains no ROS packages.
2. **Component repos stay small and focused.** The existing per-package repos are
   kept.
3. **Hardware coupling is confined to the driver repo.**
4. **Package dependencies point one way; no cycles.**

### 2.2 Repos

| Repo | Role | Packages |
|---|---|---|
| `fer_ros2_docker` | meta-repo (promoted) | none |
| `fer_ros2_driver` | driver (renamed from `fer_ros2`) | `franka_hardware`, `franka_gripper`, `franka_msgs`, `franka_semantic_components`, `franka_robot_state_broadcaster`, `franka_bringup` (driver-only launch) |
| `fer_ros2_bringup` | robot bringup for real and MuJoCo (new repo, grown from `fer_ros2/fer_bringup`) | `fer_ros2_bringup` |
| `fer_moveit_config` | MoveIt configuration | `fer_moveit_config` |
| `fer_interfaces` | contract: msgs, services, actions (Phase 8) | `fer_interfaces` |
| `fer_motion_moveit` | motion backend, MoveIt (Phase 8) | `fer_motion_moveit` |
| `fer_gripper_server` | gripper server (Phase 8) | `fer_gripper_server` |
| `fer_grasp_planner` | grasp candidates (Phase 8) | `fer_grasp_planner` |
| `fer_behavior_trees` | behavior | `fer_behavior_trees` |
| `fer_planning_world_model` | world model | `fer_world_model` |
| `speed_and_separation_monitoring` | safety prototype | `speed_and_separation_monitoring` |
| `fer_perception` | perception (future) | — |

### 2.3 Package dependency direction

```
fer_behavior_trees ──┐
fer_motion_moveit  ──┤
fer_gripper_server ──┼──> fer_interfaces ──> standard message packages only
fer_world_model    ──┤
fer_grasp_planner  ──┘
fer_motion_moveit ──> fer_moveit_config ──> franka_description (upstream)
fer_world_model   ──> vision_msgs

fer_ros2_bringup (launch levels) ──> franka_description, fer_moveit_config,
                                     fer_motion_moveit, fer_gripper_server,
                                     fer_world_model, fer_grasp_planner, fer_behavior_trees
fer_ros2_bringup (hardware)      ──runtime──> franka_hardware | mujoco_ros2_control
franka_hardware ──> libfranka
```

The robot description is upstream `franka_description`, used unmodified. Every
consumer builds from `franka_description/robots/fer/fer.urdf.xacro`.
`fer_moveit_config` and `fer_motion_moveit` use it directly; `fer_ros2_bringup`
includes it and adds only hardware overlays (§3.2). MoveIt and the motion backend
therefore never depend on the bringup. `fer_motion_moveit` is the only package above
ros2_control that depends on MoveIt.

`fer_ros2_bringup` is the integration package: it sits at the top and depends on
everything else. Nothing depends on it.

Perception publishes standard message types or depends on a pinned interface
package; it never depends on application internals.

### 2.4 Repo fates

| Repo | Fate |
|---|---|
| `GKnerd/fer_ros2_docker` | promoted to the platform meta-repo; absorbs the infra and docs of `fer_ros2_simulation` |
| `GKnerd/fer_ros2` | renamed → `fer_ros2_driver`; loses `fer_bringup` (Phase 1) |
| `GKnerd/fer_ros2_bringup` | new; `fer_bringup` extended to both backends (Phase 1) |
| `GKnerd/fer_ros2_mjc_bringup` | contents merged into `fer_ros2_bringup`, then archived |
| `GKnerd/fer_moveit_config` | one controller mapping per hardware (Phase 1) |
| `GKnerd/fer_skills` | archived after Phase 8 |
| `GKnerd/fer_behavior_trees` | rewritten against `fer_interfaces` (Phase 8) |
| `GKnerd/fer_planning_world_model` | MoveIt removed (Phase 8) |
| `GKnerd/fer_interfaces` | new (Phase 8) |
| `GKnerd/fer_motion_moveit` | new (Phase 8) |
| `GKnerd/fer_gripper_server` | new (Phase 8) |
| `GKnerd/fer_grasp_planner` | new (Phase 8) |
| `GKnerd/speed_and_separation_monitoring` | unchanged |
| `GKnerd/fer_ros2_simulation` | infra and docs copied into `fer_ros2_docker`, then archived |
| `GKnerd/fer_ws` | archived (trial monorepo, superseded) |
| `GKnerd/fer_speed_and_separation_monitoring` | deleted — empty duplicate |

### 2.5 Manifests

**Platform profiles — in `fer_ros2_docker`.** Peer profiles, none privileged. Paths
are relative to the platform repo root; every file is imported with a plain
`vcs import .`.

```
fer_core.repos    ros2_ws/src/franka_description
                  ros2_ws/src/BehaviorTree.ROS2
                  ros2_ws/src/fer_ros2_bringup
                  ros2_ws/src/fer_moveit_config
                  ros2_ws/src/fer_interfaces
                  ros2_ws/src/fer_motion_moveit
                  ros2_ws/src/fer_gripper_server
                  ros2_ws/src/fer_grasp_planner
                  ros2_ws/src/fer_behavior_trees
                  ros2_ws/src/fer_planning_world_model
                  ros2_ws/src/speed_and_separation_monitoring

fer_real.repos    deps/libfranka
                  ros2_ws/src/fer_ros2            (fer_ros2_driver after the rename)

fer_sim.repos     ros2_ws/src/mujoco_ros2_control
                  ros2_ws/src/mujoco_vendor
```

`fer_ros2_bringup` is in the core profile: it compiles nothing and needs no
`find_package` beyond `ament_cmake`, so it builds with either backend absent.

```bash
cd ~/Projects/fer_ros2_ws
git clone git@github.com:GKnerd/fer_ros2_docker.git && cd fer_ros2_docker

vcs import . < fer_core.repos     # always
vcs import . < fer_real.repos     # real robot
vcs import . < fer_sim.repos      # simulation
```

| Import | Host |
|---|---|
| core + sim | x86_64 |
| core + real | x86_64, Orin |
| core + real + sim | x86_64 |

Sim-only never clones libfranka or the driver; real-only never clones MuJoCo. The
Orin never imports the simulation profile.

**One image for every combination.** `docker/Dockerfile` serves x86_64 and aarch64;
the image tag carries the host architecture (`jazzy_x86_64`, `jazzy_aarch64`).
libfranka is an image-level dependency built from `deps/libfranka`, guarded by its
presence. colcon builds whatever the imported profiles placed in `ros2_ws/src`, so
no package lists are kept in the image.

**Pin ownership.** Each external has exactly one pin: the platform profile.
Component repos carry no `.repos` files; component CI that needs an external
(e.g. `franka_description` for `fer_ros2_bringup`) reads its pin from
`fer_ros2_docker/fer_core.repos`.

**Dev vs release.** Profiles track branches for development. A known-good state is
frozen with `vcs export --exact` into `releases/<date-or-tag>.repos`.

**Dependency declaration.** `fer_ros2_bringup` declares `mujoco_ros2_control` and
`franka_gripper` as plain `<exec_depend>`. The Dockerfile runs
`rosdep install --from-paths src --ignore-src -r -y`; `-r` continues past the absent
profile's keys with a warning.

### 2.6 Conventions

- **Branches:** `jazzy_devel` for development in every repo; `jazzy-<version>` tags for
  releases.
- **Versioning:** semver tags and `CHANGELOG.md` per repo.
- **Contracts are public APIs.** Action/msg definitions, the `hardware:=` backends,
  the BT node library and the cross-host topic/frame list (§9) are what other code
  builds on. Changing them is an explicit breaking change.
- **Package naming:** no package name exists in two repos.

---

## 3. Target layouts

### 3.1 Workspace

```
~/Projects/fer_ros2_ws/
└── fer_ros2_docker/                     ← meta-repo
    ├── docker/
    │   ├── Dockerfile                   ← x86_64 and aarch64
    │   ├── build_image.sh               ← tag fer_ros2_docker/ros:jazzy_<arch>
    │   └── run_container.sh             ← real-time limits on every host
    ├── compose/                         ← §9, added with the second host
    ├── env/          cyclone_dds.xml, cyclone_<host>.xml (§9), dds_profile.xml
    ├── config/       nic_orin_config.sh
    ├── docs/         all project docs: this plan, the review, the handoff,
    │                 robotics_stack.md, DDS_Profiles.md,
    │                 cross_host_contract.md (§9),
    │                 legacy/ (former READMEs and superseded plans)
    ├── releases/                        ← vcs export --exact snapshots
    ├── .github/workflows/               ← Phase 6
    ├── fer_core.repos  fer_real.repos  fer_sim.repos
    ├── .dockerignore  .gitignore  README.md  LICENSE
    ├── deps/         ← ignored; libfranka
    └── ros2_ws/      ← ignored; all component repos
```

`Repo_Hotfixes.md` and `Fer_Real_Time_Control_Manual.md` stay untracked at the repo
root.

### 3.2 `fer_ros2_bringup` repo

```
fer_ros2_bringup/                        ← repo = package
├── urdf/
│   ├── fer.urdf.xacro                       ← includes upstream fer.urdf.xacro,
│   │                                           hardware:=real|mujoco|none adds one overlay
│   ├── control/fer_real.ros2_control.xacro  ← FrankaMultiHardwareInterface
│   ├── control/fer_mujoco.ros2_control.xacro← MujocoSystemInterface
│   └── mujoco/fer_mujoco_inputs.xacro       ← actuators, joint dynamics, gravcomp
├── config/
│   ├── fer_controllers.yaml                 ← all backends
│   ├── fer_controllers_real.yaml            ← franka_robot_state_broadcaster
│   └── fer_controllers_gripper.yaml         ← gripper controllers (mujoco)
├── launch/
│   ├── fer_real_ros2_control.launch.py
│   ├── fer_mujoco_ros2_control.launch.py
│   ├── fer_moveit.launch.py                 ← manual planning in RViz only
│   ├── fer_manipulation.launch.py           ← Phase 8.6
│   └── fer_manipulation_bt.launch.py        ← Phase 8.6
├── scenes/base_world.xml
├── rviz/fer_real.rviz  rviz/fer_mujoco.rviz
├── test/test_description.py
├── .github/workflows/ci.yml
└── package.xml  CMakeLists.txt  README.md  LICENSE
```

`fer.urdf.xacro` contains no links, joints, meshes or inertials of its own. All
robot geometry, kinematics and dynamics come from upstream `franka_description`.
Both overlays name their ros2_control component `fer_hardware`.

---

## 4. Change plan

Each phase is independently reviewable, leaves every repo working, and is validated
against both environments.

---

### Phase 0 — One meta-repo

`fer_ros2_docker` (`jazzy` @ `9940f7a`) is promoted to the platform meta-repo. No
ROS package changes.

1. **One Dockerfile.** `docker/Dockerfile` replaces `Dockerfile.x86` and
   `Dockerfile.orin`:
   - based on `Dockerfile.x86`;
   - the libfranka block runs only if `deps/libfranka` exists;
   - the package skip list of `9940f7a` (`multi_mode_controller`,
     `multi_mode_controller_impl`, `panda_motion_generators`) applies on both
     architectures.
2. **One build and one run script.** `build_image.sh` and `run_container.sh` replace
   the `*_orin.sh` pair:
   - image tag `fer_ros2_docker/ros:jazzy_$(uname -m)`;
   - `build_image.sh` creates `deps/` so a simulation-only import builds;
   - `run_container.sh` always sets `--cap-add=sys_nice`, `--ulimit rtprio=99`,
     `--ulimit memlock=-1`, and mounts `~/.Xauthority` only where it exists.
3. **Profiles.** `fer_ros2.repos` is replaced by `fer_core.repos`, `fer_real.repos`,
   `fer_sim.repos` (§2.5).
4. **Infra and docs from `fer_ros2_simulation`** (`main` @ `5927830`):
   - `.dockerignore` (adapted to the combined layout), `env/dds_profile.xml`;
   - `DDS_Profiles.md` → `docs/`;
   - `THIRDPARTY.md` → `NOTICE.md` at the repo root, updated for the combined
     platform (libfranka added);
   - `README.md` → `docs/legacy/README_sim.md`.

   `env/cyclone_dds.xml` and `LICENSE` are identical in both repos and kept once.
5. **Workspace docs** → `docs/`: this plan, `FER_ROS2_Review_2026-08-24.md`,
   `FER_ROS2_Handoff.md`, `robotics_stack.md`. The previous `README.md` →
   `docs/legacy/README_real.md`.
6. **`README.md`** rewritten around the profile workflow.
7. **Rename GitHub repo** `fer_ros2` → `fer_ros2_driver`; update the profile URL
   and path and the local remote.
8. **Archive** `GKnerd/fer_ros2_simulation` and `GKnerd/fer_ws`.

**Verification:** from a clean clone of `fer_ros2_docker` on x86_64, import and build
core+sim, core+real and core+real+sim; on the Orin, import and build core+real.
Every combination builds with the unchanged `docker/Dockerfile`.

---

### Phase 1 — One bringup for the real robot and MuJoCo

`fer_ros2/fer_bringup` becomes the repo `fer_ros2_bringup` and absorbs the
simulation parts of `fer_ros2_mjc_bringup`. The hardware is a launch argument,
`hardware:=real|mujoco`; `fer_ros2_mjc_bringup` is archived afterwards and
`fer_ros2_driver` loses `fer_bringup/`.

**Description.** Upstream `franka_description/robots/fer/fer.urdf.xacro` is the
only description source.
- `urdf/fer.urdf.xacro` includes it unmodified; all upstream arguments pass
  through, upstream `ros2_control` stays `false` (enforced), and
  `hardware:=real|mujoco|none` adds exactly one overlay.
- `urdf/control/fer_real.ros2_control.xacro` — the former
  `fer_bringup/urdf/fer_ros2_control.xacro` (`FrankaMultiHardwareInterface`).
- `urdf/control/fer_mujoco.ros2_control.xacro` and
  `urdf/mujoco/fer_mujoco_inputs.xacro` — from `fer_ros2_mjc_bringup`; the
  duplicate finger actuator and equality of `franka_hand_mujoco.urdf.xacro` are
  dropped, and the unused `version` / `robot_type` / `prefix` hardware
  parameters are removed.
- Both overlays name the component `fer_hardware`.
- The real model is unchanged: apart from comments and the component name, the
  new `hardware:=real` URDF equals the former `fer_bringup` URDF.
- `fer_moveit_config` and `fer_skills` keep building from upstream
  `fer.urdf.xacro` with `ros2_control:=false`.

**Controllers.** Naming follows the ros2_control convention
`<command_interface>_<type>_controller`.

| Old (real) | Old (sim) | New |
|---|---|---|
| `effort_joint_trajectory_controller` | `joint_effort_traj_controller` | `effort_trajectory_controller` |
| `vel_joint_trajectory_controller` | — | `velocity_trajectory_controller` |
| — | `joint_pos_traj_controller` | `position_trajectory_controller` |
| — | `gripper_position_controller` | `gripper_position_controller` |
| — | `gripper_effort_controller` | `gripper_effort_controller` |
| `joint_state_broadcaster` | `joint_state_broadcaster` | unchanged |
| `franka_robot_state_broadcaster` | — | unchanged |
| — | `cartesian_compliance_controller`, `motion_control_handle` | removed |
| (future) | (future) | `cartesian_impedance_controller` |

- `config/fer_controllers.yaml` — update rate, all controller types, the three
  JTCs with gains (`antiwindup_strategy: none`, no `i_clamp`).
- `config/fer_controllers_real.yaml` — `franka_robot_state_broadcaster`.
- `config/fer_controllers_gripper.yaml` — the two gripper controllers (MuJoCo).
- `ros2_control_node` and every spawner receive the file list for the selected
  hardware; later files override earlier ones.
- Only the broadcasters start active. Every motion and gripper controller is
  loaded inactive and switched on with `ros2 control switch_controllers`.

**Gripper seam.** The real gripper is the `franka_gripper` node `fer_gripper`
serving `/fer_gripper/gripper_action`; `moveit_simple_controller_manager`
resolves `<controller_name>/<action_ns>`, so it is routable by configuration.
- `fer_moveit_config/config/moveit_controllers_real.yaml` — three JTCs and
  `fer_gripper` (`action_ns: gripper_action`).
- `fer_moveit_config/config/moveit_controllers_mujoco.yaml` — effort and position
  JTCs, both gripper controllers (`action_ns: gripper_cmd`).
- `fer_moveit_launch.py` takes `hardware` and loads the matching file;
  `arm_control_type` / `hand_control_type` set `default: true` on
  `<type>_trajectory_controller` / `gripper_<type>_controller`, and a type the
  hardware does not offer aborts the launch. `controller_selection.yaml` is
  deleted.
- `MoveItSimpleControllerManager` reports every listed controller as available,
  so a dead `fer_gripper` goes unnoticed by MoveIt; the real hardware launch
  shuts the whole bringup down when the gripper node exits.

**Launch levels.** Each level includes the one below and adds one part.

| File | Adds |
|---|---|
| `fer_real_ros2_control.launch.py` / `fer_mujoco_ros2_control.launch.py` | description, RSP, ros2_control, controllers, gripper |
| `fer_moveit.launch.py` | MoveIt |
| `fer_moveit_skills.launch.py` | skill server |
| `fer_moveit_skills_bt.launch.py` | BT server, world model |

- The upper three take `hardware:=real|mujoco` (default `mujoco`) and include
  `fer_<hardware>_ros2_control.launch.py`. `use_sim_time` is derived from
  `hardware`; only the top level started opens RViz.
- Real: `robot_ip` is required; the `joint_state_broadcaster` remap to
  `fer/joint_states` is passed with `--controller-ros-args`; `joint_state_publisher`
  merges arm and gripper states; `robot_description` comes from the topic.
- MuJoCo: `scene:=` selects the MJCF scene, default `scenes/base_world.xml`.
- Deleted: `fer_bringup.launch.py`, `empty_world.launch.py`,
  `fer_mujoco_moveit`, `fer_mjc_moveit_skills`, `fer_mjc_moveit_skills_bt`.

**CI.** `test/test_description.py` (`description-invariant`, §Phase 6) checks:
- the model equals upstream for `hardware:=none|real|mujoco`, with and without hand;
- exactly one overlay, named `fer_hardware`, with the right plugin;
- every overlay joint exists; the real overlay's parameter and interface contract;
- no duplicate MuJoCo actuators for any control type;
- invalid `hardware` and `ros2_control:=true` are rejected;
- every joint in the controller YAMLs exists, and every configured controller has a type.

**Profiles.** `fer_ros2_bringup` joins `fer_core.repos`;
`fer_ros2_mjc_bringup` leaves `fer_sim.repos`.

**Verification.**
- `description-invariant` green.
- core+sim, core+real and core+real+sim build.
- MuJoCo: all four levels start; after switching on the controllers, pick, place
  and the BT run as before.
- Real (Orin): hardware level with `robot_ip`, broadcasters active, gripper up;
  MoveIt level plans and executes after `switch_controllers`; then skills and BT.

---

### Phase 2 — Expose all command interfaces in simulation

Removes `mujoco_control_type` / `hand_control_type` from the description.

Supporting behaviour in `mujoco_system_interface.cpp`:
`export_command_interfaces()` (1221-1253) exports one interface per declared
`<command_interface>`; `perform_command_mode_switch()` (1320-1392) disables all
modes then enables the requested one.

The MJCF actuator type gates which interfaces are usable:

| Command interface | `<motor>` | `<position>` | `<velocity>` |
|---|---|---|---|
| effort | direct | error, skipped (2063-2068) | error, skipped |
| position | needs position PID (1994-2016) | direct | needs PID |
| velocity | needs velocity PID (2032-2055) | error (2023-2025) | direct |

If no interface ends up enabled, `register_urdf_joints` throws (2082-2088).

**`fer_ros2_bringup`**
- `urdf/mujoco/fer_mujoco_inputs.xacro` — `<motor>` actuators unconditionally for
  all 7 arm joints and the finger. Motor matches the real FER, which is
  torque-controlled.
- `urdf/control/fer_mujoco.ros2_control.xacro` — `effort`, `position` and
  `velocity` command interfaces on every joint, matching the real overlay.
- `config/fer_controllers_mujoco.yaml` — position and velocity PID gains keyed by
  actuator name, read via
  `control_toolbox::PidROS::initialize_from_ros_parameters()` (1731, 1747).
  `gripper_position_controller` requires a position PID on the finger.
- `velocity_trajectory_controller` is spawned in MuJoCo as well.

**`fer_moveit_config`**
- `moveit_controllers_mujoco.yaml` gains `velocity_trajectory_controller`.
- `hand_control_type` remains the only selection that differs by hardware.

Afterwards the sim and real `<ros2_control>` blocks have the same shape, and mode
selection is `ros2 control switch_controllers` in both environments.

---

### Phases 3–5

Controller naming (3), the gripper seam (4) and the launch levels (5) are part of
Phase 1.

One gripper interface for both environments, with width, tolerance and force, is
`fer_gripper_server` (Phase 8.2).

---

### Phase 6 — CI

Two levels. Component CI never relies on anything the platform imports.

**Component CI — every component repo**
- Externals it needs are taken at the pins in `fer_ros2_docker/fer_core.repos`.
- `colcon build`, `colcon test`.
- `lint` — `ament_flake8`, `ament_pep257`, `ament_uncrustify --reformat`,
  `ament_cpplint`.
- `fer_ros2_bringup` additionally: `description-invariant` (the Phase 1 diff against
  upstream).
- `fer_ros2_driver` additionally: libfranka build.

**Platform CI — `fer_ros2_docker`**
- `build-core-sim`, `build-core-real`, `build-core-both` — fresh import per
  combination, image build.
- `launch-mock` — adds `hardware:=mock` to `fer_ros2_bringup`
  (`urdf/control/fer_mock.ros2_control.xacro`, `mock_components/GenericSystem`,
  `fer_mock_ros2_control.launch.py`). Core only; bring up
  `fer_manipulation.launch.py hardware:=mock` with
  `arm_controller:=position_trajectory_controller`, assert `/joint_states` publishing
  and a `MoveToJoints` goal executed. `GenericSystem` does not simulate
  effort, so effort behaviour stays covered by sim and hardware validation.
- `unique-package-names` — no package name appears twice across imported repos.
- `dependency-direction` — package dependencies match §2.3.
- `stale-names` — `grep` for retired controller names.
- `images` — Docker images built from a fresh import, never from a local tree.

Branch protection on `jazzy_devel` in every repo requires its CI.

Hardware behaviour — real-time, reflexes, error recovery — is not covered by CI and
stays a manual validation step. Covering it requires a self-hosted runner on the
Orin with the robot attached.

---

### Phase 7 — Cleanup and decommissioning

After Phases 1–6 are green and both robots are re-validated.

- Delete dead multi-arm configs and launch files in
  `fer_ros2_driver/franka_bringup/{config,launch}/{mixed,real}/` (`dual_*`, `mixed_*`).
- Align default branches to `jazzy_devel` (§2.6).
- Delete `GKnerd/fer_speed_and_separation_monitoring`.
- `fer_ros2_simulation` README note before archiving:
  *"Superseded by GKnerd/fer_ros2_docker. Archived."*

---

### Phase 8 — Platform refactor: interfaces, world model, gripper, motion, trees

`fer_skills` fuses contract, server and MoveIt/MTC backend, so MoveIt leaks into every
caller. The world model mirrors MoveIt's planning scene, and the gripper is a segment
of an MTC trajectory. Phase 8 replaces the skill server with small servers behind one
interface package, makes the world model the planner-independent source of truth, and
moves task composition into the behavior trees.

**Current issues**

| Issue | Effect |
|---|---|
| Action definitions live in the MTC package | `fer_behavior_trees` depends on MoveIt and MTC to obtain message types |
| Pick and Place are one MTC plan executed by `move_group` | No step between approach and grasp can be changed from a tree; the gripper has no result of its own |
| Gripper driven through SRDF named states | Width, tolerance and force cannot be expressed; `close` = 0.0 makes franka `grasp()` report failure while holding an object |
| Server and backend are one object (`SkillServer` holds `picker_` / `placer_` MTC tasks) | No backend swap; concurrent goals collide |
| Perception writes `CollisionObject`s to `/planning_scene`; the world model mirrors `/monitored_planning_scene` | Every detector must know MoveIt; trees carry object ids as literals |

**Target**

```
fer_behavior_trees        order, choices, retries, reactions to world changes
  │  fer_interfaces
  ├─► motion backend      /motion/*        plans and executes arm motion; one package per planner
  ├─► fer_gripper_server  /gripper/*       gripper commands, grasp evidence → object status
  ├─► fer_world_model     /world_model/*   objects: identity, pose, shape, status
  └─► fer_grasp_planner   /grasp/*         grasp candidates for an object
ros2_control              controllers, loaded inactive, activated by the server that needs them
```

- The tree never sees a trajectory, a planner name or a controller name.
- Pick and Place are behavior-tree SubTrees built from these actions. No server holds a
  pick sequence.
- Another motion backend (e.g. MPC) is another package serving the same names; launch
  selects one.

**Contract — `fer_interfaces`** (repo `fer_interfaces`, branch `jazzy_devel`), a public
contract per §2.6. Depends only on `action_msgs`, `builtin_interfaces`,
`geometry_msgs`, `shape_msgs`, `std_msgs`.

| Interface | Kind | Name | Served by | Called by |
|---|---|---|---|---|
| `MoveToPose` | action | `/motion/move_to_pose` | motion backend | BT |
| `MoveToJoints` | action | `/motion/move_to_joints` | motion backend | BT |
| `CheckReachable` | service | `/motion/check_reachable` | motion backend | BT |
| `MoveGripper` | action | `/gripper/move` | `fer_gripper_server` | BT |
| `Grasp` | action | `/gripper/grasp` | `fer_gripper_server` | BT |
| `Release` | action | `/gripper/release` | `fer_gripper_server` | BT |
| `DetectObjects` | action | `/world_model/detect_objects` | `fer_world_model` | BT |
| `RefineObject` | action | `/world_model/refine_object` | `fer_world_model` | BT |
| `QueryObjects` | service | `/world_model/query_objects` | `fer_world_model` | BT, motion backend, gripper server, grasp planner |
| `SetObjectStatus` | service | `/world_model/set_object_status` | `fer_world_model` | `fer_gripper_server` |
| `GetGraspCandidates` | service | `/grasp/candidates` | `fer_grasp_planner` | BT |
| `WorldObjectArray` | topic, latched | `/world_model/objects` | `fer_world_model` | RViz, monitoring |

Rules:
- SI units. Gripper width is the full opening between the fingers.
- Poses are `PoseStamped` in any TF frame. A server converts a pose to `base` once,
  when it accepts the goal; a pose in `fer_hand_tcp` is relative to the hand at that
  moment.
- An object pose is the center of its shape (`shape_msgs/SolidPrimitive` convention).
- Every result starts with `Outcome`. Trees branch on `outcome.code`, never on
  `message`.
- A new goal on a server replaces the running one; the old goal ends `CANCELLED`.
- Only detection and `fer_gripper_server` write to the world model. Trees read.
- The blackboard holds object ids, never copies of objects.

**Enforcement**, in this order: message types, server validation, contract tests.
- The types exclude two targets in one goal (`MoveToPose` / `MoveToJoints`), a straight
  path to a joint target, partial joint lists (`float64[7]`), and detect/refine
  combinations (`DetectObjects` / `RefineObject`).
- Servers reject the rest:

| Rule | Checked by | Outcome |
|---|---|---|
| `speed_scaling` in (0, 1] | motion backend | `INVALID_GOAL` |
| gripper width in [0, 0.08] m | gripper server | `INVALID_GOAL` |
| pose frame unknown to TF | motion backend | `INVALID_GOAL` |
| unknown object id (`may_touch`, `object_id`) | motion backend, gripper server, world model | `NOT_FOUND` |
| `Grasp` on a non-FREE object, `Release` on a non-GRASPED object, `RefineObject` on a GRASPED object | gripper server, world model | `INVALID_STATE` |

- Each server package has a contract test that sends valid and invalid goals and checks
  the outcomes. Every implementation of an interface — MoveIt or MPC backend, real or
  sim gripper — passes the same test.

**Testing** — every Phase 8 package.
- Package layout: `core/` (no ROS imports), `adapters/` (hardware or other nodes, behind
  a small interface), a thin node.
- Unit tests on `core/`: pytest, gtest. Node tests: the node plus fake neighbours in one
  process. Integration: `launch_testing` on `hardware:=mock` (Phase 6) or MuJoCo.
- Tests never share the robot's DDS domain: C++ uses `ament_add_ros_isolated_gtest`
  (`ament_cmake_ros`); Python uses a `conftest.py` that sets a unique `ROS_DOMAIN_ID`
  and `ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST` before `rclpy.init`.
- A sub-phase is done when `colcon test` passes for its packages. Lint per Phase 6.

Old and new stack run side by side until 8.6.

#### 8.0 Groundwork

- `fer_interfaces` joins `fer_core.repos` (`jazzy_devel`).
- `fer_interfaces/README.md`: the interface table above.
- `docker/Dockerfile`: `ros-jazzy-vision-msgs`.
- Test isolation: the first `conftest.py` is written in `fer_planning_world_model`
  (8.1); later Python packages copy it. C++ packages use
  `ament_add_ros_isolated_gtest`.

**Verification:** fresh import and image build; `fer_interfaces` builds in the
container.

#### 8.1 World model without MoveIt — `fer_planning_world_model` (Python)

- `core/`: the object schema gains `class_id`, `score`, `source`, `fixed` and status
  LOST. New `association.py`: a detection matches a FREE object of the same class within
  a gating distance, otherwise gets a new id `<class>_<n>`; the world model owns ids.
  GRASPED objects are never changed by detections. An object not seen is never removed
  or changed; it is reported in `not_seen`.
- `planning_scene_world_model_server.py` rewritten against `fer_interfaces`:
  `DetectObjects`, `RefineObject`, `QueryObjects`, `SetObjectStatus`,
  `/world_model/objects` with `revision`. Subscribes `/perception/detections`
  (`vision_msgs/Detection3DArray`, stamped at capture, camera frame) and transforms with
  TF at the detection stamp into `base`. Fixed objects (table) from
  `config/fixtures.yaml`. No `moveit_msgs`.
- Snapshot: the first detection message stamped after the request, taken with the arm
  at rest. The real `joint_state_publisher` runs at 30 Hz
  (`fer_real_ros2_control.launch.py`), so TF at the stamp is inaccurate while the arm
  moves. `RefineObject` is taken from a viewing pose within the D405 range (7–50 cm).
- New node `mock_perception`: publishes `Detection3DArray` from YAML (class, center
  pose, size; no ids).
- `mock_camera_node.py` and `planning_scene_adapter.py` stay for the old stack until
  8.7.

**Verification:** pytest for association, status rules and not-seen handling; CLI
detect → query → set status → refine; `package.xml` contains no `moveit_*`.

#### 8.2 Gripper — `fer_gripper_server` (new repo, Python)

- Serves `MoveGripper`, `Grasp`, `Release`. One adapter per hardware, selected by
  parameter:
  - real: `/fer_gripper/move`, `/fer_gripper/grasp` (width, epsilon = `tolerance`,
    speed from config, force); width from `/fer_gripper/joint_states`;
  - mujoco: `control_msgs/GripperCommand` on `gripper_effort_controller` (position =
    width / 2, `max_effort` = force); the node checks width and tolerance itself and
    activates `gripper_effort_controller` at startup.
- `Grasp`: object must be FREE. Success when the measured width is within `tolerance`
  of `width` and above `min_hold_width` → `SetObjectStatus` GRASPED, `held_by`
  `fer_hand_tcp`, pose relative to the hand. Failure → reopen to the width before
  closing, `GRASP_FAILED`, world model unchanged.
- `Release`: object must be GRASPED. Open, confirm the width, then `SetObjectStatus`
  FREE at the hand pose combined with the stored offset, `source` `release_estimate`.

**Verification:** contract test; `Grasp` on nothing → `GRASP_FAILED`, gripper reopened,
world model unchanged; `Grasp` on an object → GRASPED; `Release` → FREE. Sim and real.

#### 8.3 Motion — `fer_motion_moveit` (new repo, C++)

- Serves `MoveToPose`, `MoveToJoints`, `CheckReachable`. MoveIt runs in-process through
  MoveItCpp; no `move_group`.
- Per request: `QueryObjects` snapshot → collision scene (FREE and fixed objects as
  primitives, GRASPED objects attached to `held_by`, `may_touch` allows hand-link
  contact with the listed objects for this request only) → plan with OMPL for
  `PATH_FREE` and Pilz LIN for `PATH_STRAIGHT` → time parameterization with
  `speed_scaling` → start state checked against `/joint_states` →
  `FollowJointTrajectory` to the configured arm controller.
- `CheckReachable` plans the chained targets without executing and returns the
  configuration at each target.
- Activates `arm_controller` (parameter: `effort_trajectory_controller` |
  `position_trajectory_controller`) through `/controller_manager/switch_controller` at
  startup.
- Cancel → cancel the controller goal; `CANCELLED` once the arm is at rest.
- `ROBOT_ERROR` from the robot mode reported by `franka_robot_state_broadcaster`
  (real).
- The planning pipeline keeps a start-state fix: the closed real gripper reports
  −2.6e‑6, below its joint limit.
- Publishes the planning scene for RViz. Robot description, SRDF, kinematics and
  pipeline parameters from `fer_moveit_config`.
- New `fer_moveit_config/config/moveit_cpp.yaml`: planning scene monitor options and
  the planning pipelines (OMPL, Pilz) for MoveItCpp. `move_group` has no equivalent
  file.
- To check: how the JTC stops a cancelled goal at speed (hold or deceleration).

**Verification:** contract test; unit tests with the robot model and a fake
`FollowJointTrajectory` server; sim: joints → home, pose with free path, pose with
straight path, `CheckReachable` returns 7 values per target, cancel mid-motion; real at
low speed; `launch-mock` once Phase 6 exists.

#### 8.4 Grasp candidates — `fer_grasp_planner` (new repo, Python)

- Serves `GetGraspCandidates`. Top-down candidates from the object's box: width from
  the box, pre-grasp above the grasp along the approach, lift above the grasp, force
  per class from `config/object_catalog.yaml`.
- A grasp network later replaces the computation behind the same service. It runs on
  the perception host (§9), crops the newest point cloud to the object's box, and
  converts its gripper frame to `fer_hand_tcp` (0.1034 m offset).

**Verification:** pytest for the geometry; candidates for a mock object shown in RViz;
`CheckReachable` passes for at least one.

#### 8.5 Behavior trees — `fer_behavior_trees`

- One node per interface: `MoveToPose`, `MoveToJoints`, `MoveGripper`, `Grasp`,
  `Release`, `DetectObjects`, `RefineObject`, `QueryObjects`, `GetGraspCandidates`;
  each writes `outcome.code` to an output port. `FindReachableGrasp` calls
  `CheckReachable` per candidate (pre-grasp free → grasp straight → lift straight) and
  outputs the first passing candidate with its pre-grasp configuration.
- `Pick` and `Place` SubTrees; `pick_place.xml` loops over `QueryObjects` results with
  `LoopString`.
- `Pick` sequence: `MoveToJoints` to the view pose → `RefineObject` →
  `GetGraspCandidates` → `FindReachableGrasp` → `MoveGripper` open → `MoveToJoints` to
  the pre-grasp configuration → `MoveToPose` straight to the grasp with `may_touch` →
  `Grasp` (on failure: `MoveToPose` straight back to the pre-grasp, then fail) →
  `MoveToPose` straight to the lift pose.
- The pre-grasp is reached with `MoveToJoints`, not `MoveToPose`: the arm has 7
  joints, so a pose alone can end in a different arm configuration than the one
  `CheckReachable` tested, from which the straight approach may fail.
- Goal payload (JSON) → global blackboard (e.g. `target_class`). Keys of the previous
  goal are cleared; a malformed payload is rejected.
- The blackboard holds ids only. Place targets are fixed poses in the tree. Named poses
  (`home`, `view`) come from `config/poses.yaml`.
- Depends on `fer_interfaces` only.

**Verification:** tree tests with fake nodes (`Grasp` fails → `Pick` backs out and
fails); sim pick-and-place with mock perception and no object id in XML; one failed
grasp recovers.

#### 8.6 Bringup — `fer_ros2_bringup`

| File | Adds |
|---|---|
| `fer_real_ros2_control.launch.py` / `fer_mujoco_ros2_control.launch.py` | description, RSP, ros2_control, controllers (inactive), gripper |
| `fer_manipulation.launch.py` | world model, `perception:=mock\|none`, grasp planner, gripper server, motion backend |
| `fer_manipulation_bt.launch.py` | BT server |

- `hardware` reaches every node that differs by hardware.
- `move_group` and `fer_skills` are not started.

**Verification:** the same tree on `hardware:=mujoco` and `hardware:=real`.

#### 8.7 Removal

- `fer_skills` leaves `fer_core.repos`; the repo is archived.
- `fer_behavior_trees`: old client nodes and trees removed.
- `fer_planning_world_model`: `mock_camera_node.py`, `planning_scene_adapter.py`
  removed.
- `fer_ros2_bringup`: `fer_moveit_skills.launch.py`, `fer_moveit_skills_bt.launch.py`
  removed; `fer_moveit.launch.py` stays for manual planning in RViz.
- `robotics_stack.md` updated.

**Deferred** — the interfaces already allow them: grasp network, D405 perception node,
grasp monitor (§11 wrench, drops → LOST), continuous perception, handover and a
streaming motion contract (§10), MPC backend, compliant control.

---

## 5. Sequencing

| Phase | Content | Effort | Gate |
|---|---|---|---|
| 0 | promote `fer_ros2_docker`, profiles, renames | ~1 d | clean-clone builds, all three combinations |
| 1 | one bringup: description, controllers, gripper seam, launch levels, `description-invariant` | ~2 d | CI green; every level starts in both environments; real robot moves |
| 2 | sim all-interfaces | ~1 d | runtime controller switch works |
| 6 | mock backend, `launch-mock`, platform CI | ~2 d | branch protection on |
| 7 | cleanup, archive | ~0.5 d | — |
| 8 | platform refactor (8.0–8.7) | ~3 wk | pick and place runs on sim and real through `fer_interfaces`; `colcon test` green in every Phase 8 package; `fer_skills` archived |

---

## 6. Risks

| Risk | Mitigation |
|---|---|
| Phase 1 changes kinematics unnoticed | Blocking diff against upstream, permanent CI job |
| Real robot regresses | Driver source untouched; real URDF compared against the former `fer_bringup` output; hardware re-validation after Phases 1 and 2 |
| Phase 1 lands in one repo but not the others | Changes to `fer_ros2_bringup`, `fer_moveit_config`, `fer_ros2_driver` and the profiles merged together; `unique-package-names` CI |
| Consumers pass different arguments to the upstream xacro | Arguments aligned in Phase 1; `description-invariant` CI compares against upstream |
| Package-level dependency cycle | MoveIt config and motion backend depend on upstream `franka_description`, never on the bringup; `dependency-direction` CI |
| Position/velocity PID tuning poor | Every motion controller starts inactive; modes are switched on deliberately |
| Retired controller name missed | `stale-names` CI |
| Repo renames break clones and manifests | Profiles updated in the same step; GitHub redirects as a fallback only |
| rosdep warnings mistaken for errors | Documented in the platform README; both profile builds in CI |
| A test goal reaches the real robot (host network, one DDS domain) | Every test runs in an isolated DDS domain (Phase 8, Testing) |
| Old and new stack diverge during Phase 8 | Both run side by side until 8.6; the old stack is removed only after the same tree passes on sim and real |

---

## 7. Failure modes of the multi-repo layout and their guards

| # | Failure mode | Guard |
|---|---|---|
| 1 | Floating `main` pins make the platform unreproducible | Dev profiles track branches; `vcs export --exact` into `releases/` |
| 2 | Cross-repo changes merged out of order | Additive provider changes first; provider merges before consumer; identical feature-branch names, honoured by platform CI |
| 3 | Component CI green only because a developer workspace supplies a dependency | Component CI starts from an empty runner and imports only the pins it needs from `fer_core.repos` |
| 4 | Two repos pin the same external differently | Only the platform profiles carry pins; component repos have no `.repos` files |
| 5 | Duplicate package names across repos | `unique-package-names` CI |
| 6 | Interfaces in the wrong package create upward dependencies or cycles | Interfaces in the lowest layer that needs them; `dependency-direction` CI |
| 7 | Unpushed or stale work hidden in ignored checkouts | `vcs status` and `vcs custom --git --args log --oneline @{u}..` before pushing |
| 8 | Docker images built from whatever is on disk | CI builds images from a fresh import |
| 9 | Meta-repo accumulates untested code | No ROS packages in `fer_ros2_docker` |
| 10 | Most changes span several repos | Signal to merge those repos (`git subtree`) |
| 11 | Contract changes break other components silently | Semver tags, changelogs, explicit breaking changes |
| 12 | Inconsistent branch names | `main` everywhere, `jazzy-<version>` tags |

Guards 1, 3, 5 and 8 are in place from Phase 0 / Phase 6 onward because those
failures are silent.

---

## 8. Invariants

1. With overlays removed, the bringup's URDF equals upstream `franka_description`
   for every `hardware` value (`description-invariant`).
2. The platform builds with core+sim and with core+real; neither profile requires
   the other.
3. The platform launches with core only (`hardware:=mock`, from Phase 6).
4. Every external has exactly one pin, in the platform profiles.
5. One controller name set, one gain table, one description source (upstream), one
   set of launch levels.
6. No package name exists twice; package dependencies follow §2.3.

---

## 9. Multi-host deployment

Each machine runs its own containers from its own image. Docker Compose manages
containers on one host only; the machines are connected by DDS over the network,
not by Docker.

### 9.1 Per-host compose files

`fer_ros2_docker/compose/` holds one file per host role: `control.yaml` (Orin),
`perception.yaml` (perception PC), `workstation.yaml` (sim, RViz, Groot). Every
service sets:

```yaml
network_mode: host          # DDS discovery; bridge networks break it
ipc: host                   # shared-memory transport between containers on one host
environment:
  ROS_DOMAIN_ID: "<id>"                               # same on every host
  RMW_IMPLEMENTATION: rmw_cyclonedds_cpp              # same on every host
  CYCLONEDDS_URI: file:///config/cyclone_<host>.xml   # per-host interface selection
```

Remote hosts are driven from one terminal with Docker contexts:

```bash
docker context create perception --docker "host=ssh://<user>@<perception-pc>"
docker --context perception compose -f compose/perception.yaml up -d
docker compose -f compose/control.yaml up -d
```

Each host runs its own top-level launch; `ros2 launch` does not start nodes on
other hosts.

### 9.2 Requirements across hosts

- Same ROS distro (Jazzy) on every host.
- Identical message definitions: cross-host topics use standard message types, or an
  interface package pinned to the same version in every image.
- Same `ROS_DOMAIN_ID` and RMW.
- Clock synchronisation (chrony or PTP) between hosts.

### 9.3 Network

- On the Orin, `cyclone_orin.xml` binds DDS to the ROS network interface, never to
  the FCI interface to the robot.
- Where multicast is blocked, the Cyclone configs list peers explicitly.
- Heavy data stays on the host that produces it: perception sends results (object
  poses), not raw point clouds. `fer_grasp_planner` therefore runs on the perception
  host; only grasp candidates cross the network. Images crossing the network use `image_transport`
  compression; DDS socket buffers per `docs/DDS_Profiles.md`.
- `rmw_zenoh` is the fallback if DDS discovery across the network proves unreliable.

### 9.4 Cross-host contract

`docs/cross_host_contract.md` lists, per host: published and subscribed topics with
message types and QoS, and owned TF frames. Each transform has exactly one
publisher; the hand-eye calibration transform is published by the perception host.
The document is a public contract per §2.6.

---

## 10. Known limits

- **Sim/real parity ends at joint level.** The real driver also exports Cartesian
  command interfaces and the `robot_state` / `robot_model` interfaces; MuJoCo does
  not. A controller reading `franka_model` runs on the real robot only. Before
  `cartesian_impedance` is written, its dynamics source is decided: a URDF-based
  model (Pinocchio / KDL) usable in both environments, or a sim-side model
  interface.
- **CI does not cover hardware behaviour.** Mock hardware has no dynamics; real-time
  behaviour needs the robot.
- **Everything above the driver is FER-specific in configuration.** A second robot
  needs its own upstream description, hardware overlays, bringup and MoveIt config; FR3 additionally needs a
  different libfranka and cannot share a workspace with the Panda driver.
- **The motion contract is goal-terminated.** Condition-terminated and reactive
  behaviour (tracking, handover, SSM speed scaling, MoveIt Servo) needs a streaming
  contract and a speed-scaling hook in the control path.
- **The driver is frozen on libfranka 0.9.2.** Distro and toolchain upgrades may
  require further patches; libfranka patches are kept as separate commits.
- **SSM is not a certified safety function.** Protective stops go through the
  Franka safety system and E-stop.

---

## 11. The wrench contract

The sim/real contract is a standard ROS message, never a vendor message. Consumers —
SSM, contact detection, the grasp monitor — depend on `geometry_msgs/WrenchStamped`, never on
`franka_msgs`, which stays in `fer_ros2_driver`.

**Sim.** `register_sensors()` (`mujoco_system_interface.cpp:2334`) reads `<sensor>`
entries from the ros2_control block:

```xml
<sensor name="fer_ft">
  <param name="mujoco_type">fts</param>
  <state_interface name="force.x"/>  <!-- … force.y/z, torque.x/y/z -->
</sensor>
```

It binds to MJCF sensors `fer_ft_force` / `fer_ft_torque` (suffixes configurable via
`force_mjcf_suffix` / `torque_mjcf_suffix`) and exports six state interfaces
(1103-1136), consumed by the stock `force_torque_sensor_broadcaster`.

**Real.** `FrankaMultiHardwareInterface::export_state_interfaces()` (143-180) exports
joint position/velocity/effort, `<arm>_ee_cartesian_position` and
`_ee_cartesian_velocity`, and the `robot_state` / `robot_model` pointers — no
force/torque interface. The approach is to add a `<sensor>` block and six state
interfaces sourced from `k_f_ext_hat_k` / `o_f_ext_hat_k` in `FrankaState`, so both
environments use the same `force_torque_sensor_broadcaster`, topic and message.
Fallback: a node republishing `FrankaState` as `WrenchStamped`.

`set_load()` must be called after attaching a payload to keep the external-force
estimate correct.
