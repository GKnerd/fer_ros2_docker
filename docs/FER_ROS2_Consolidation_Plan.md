# FER ROS 2 — Sim/Real Consolidation Plan

**Date:** 2026-09-16
**Supersedes:** items #2 and #3 of `FER_ROS2_Review_2026-08-24.md`

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

**Follow-up after the consolidation:** skills redesign incl. arbitration (Phase 8).

**Deferred:** Cartesian impedance control, the wrench path (§11), the gripper relay
controller (Phase 4). Before the impedance controller is written, decide where its
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
| `fer_ros2_bringup` | robot bringup (renamed from `fer_ros2_mjc_bringup`; simulation profile until Phase 3, core afterwards) | `fer_ros2_bringup` |
| `fer_moveit_config` | MoveIt configuration | `fer_moveit_config` |
| `fer_skills` | skills | `fer_skills` (Phase 8: + `fer_skill_interfaces`, split server/backend) |
| `fer_behavior_trees` | behavior | `fer_behavior_trees` |
| `fer_planning_world_model` | world model | `fer_world_model` |
| `speed_and_separation_monitoring` | safety prototype | `speed_and_separation_monitoring` |
| `fer_perception` | perception (future) | — |

### 2.3 Package dependency direction

```
fer_behavior_trees ──> fer_skills ──> fer_moveit_config ──> franka_description (upstream)
                           └─────────────────────────────> franka_description (upstream)
fer_world_model ──> (moveit_msgs only)

fer_ros2_bringup (launch ladder) ──> franka_description, fer_moveit_config,
                                     fer_skills, fer_behavior_trees, fer_world_model
fer_ros2_bringup (hardware)      ──runtime──> franka_hardware | mujoco_ros2_control | mock
franka_hardware ──> libfranka
```

The robot description is upstream `franka_description`, used unmodified. Every
consumer builds from `franka_description/robots/fer/fer.urdf.xacro`.
`fer_moveit_config` and `fer_skills` use it directly; `fer_ros2_bringup` includes it
and adds only hardware overlays (§3.2). MoveIt and the skills therefore never depend
on the bringup.

`fer_ros2_bringup` is the integration package: it sits at the top and depends on
everything else. Nothing depends on it.

Perception publishes standard message types or depends on a pinned interface
package; it never depends on application internals.

### 2.4 Repo fates

| Repo | Fate |
|---|---|
| `GKnerd/fer_ros2_docker` | promoted to the platform meta-repo; absorbs the infra and docs of `fer_ros2_simulation` |
| `GKnerd/fer_ros2` | renamed → `fer_ros2_driver`; loses `fer_bringup` (Phase 1) |
| `GKnerd/fer_ros2_mjc_bringup` | renamed → `fer_ros2_bringup`; becomes the consolidated bringup |
| `GKnerd/fer_moveit_config` | unchanged |
| `GKnerd/fer_skills` | unchanged until Phase 8 |
| `GKnerd/fer_behavior_trees` | unchanged |
| `GKnerd/fer_planning_world_model` | unchanged |
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
                  ros2_ws/src/fer_moveit_config
                  ros2_ws/src/fer_skills
                  ros2_ws/src/fer_behavior_trees
                  ros2_ws/src/fer_planning_world_model
                  ros2_ws/src/speed_and_separation_monitoring

fer_real.repos    deps/libfranka
                  ros2_ws/src/fer_ros2            (fer_ros2_driver after the rename)

fer_sim.repos     ros2_ws/src/mujoco_ros2_control
                  ros2_ws/src/mujoco_vendor
                  ros2_ws/src/fer_ros2_mjc_bringup (fer_ros2_bringup after the rename)
```

The bringup sits in the simulation profile while its `CMakeLists.txt` requires the
MuJoCo packages; it moves to the core profile in Phase 3.

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

**Per-repo dependencies.** A component repo whose dependencies are not
rosdep-resolvable carries a `dependencies.repos` so it can be used without the
platform: `fer_ros2_driver` (libfranka), `fer_ros2_bringup` (`franka_description`),
`fer_behavior_trees` (`BehaviorTree.ROS2`).

**Pin ownership.** Each external has one authoritative pin: the platform profile.
Platform CI asserts that every `dependencies.repos` agrees with it.

**Dev vs release.** Profiles track branches for development. A known-good state is
frozen with `vcs export --exact` into `releases/<date-or-tag>.repos`.

**Dependency declaration.** `fer_ros2_bringup` declares `mujoco_ros2_control` and
`franka_gripper` as plain `<exec_depend>`. The Dockerfile runs
`rosdep install --from-paths src --ignore-src -r -y`; `-r` continues past the absent
profile's keys with a warning. The package builds either way because it compiles
nothing and has no `find_package` beyond `ament_cmake` (Phase 3).

### 2.6 Conventions

- **Branches:** `main` for development in every repo; `jazzy-<version>` tags for
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
    │                 cross_host_contract.md (§9), legacy/README_{real,sim}.md
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
│   ├── fer_hw.urdf.xacro                    ← includes upstream fer.urdf.xacro,
│   │                                           adds one hardware overlay
│   ├── control/fer_real.ros2_control.xacro
│   ├── control/fer_mujoco.ros2_control.xacro
│   ├── control/fer_mock.ros2_control.xacro
│   └── mujoco/{franka_mujoco.xacro, franka_hand_mujoco.xacro}
├── config/
│   ├── controllers_common.yaml  controllers_real.yaml  controllers_sim.yaml
│   └── skill_server_fer.yaml                ← FER values for the skill server (Phase 8)
├── launch/                                  ← the ladder, Phase 5
├── scenes/  rviz/  test/
├── dependencies.repos
├── .github/workflows/
└── package.xml  CMakeLists.txt  CHANGELOG.md  README.md  LICENSE
```

`fer_hw.urdf.xacro` contains no links, joints, meshes or inertials of its own. All
robot geometry, kinematics and dynamics come from upstream `franka_description`.

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
7. **Rename GitHub repos:** `fer_ros2` → `fer_ros2_driver`,
   `fer_ros2_mjc_bringup` → `fer_ros2_bringup`; update the profile URLs and paths
   and the local remotes.
8. **Archive** `GKnerd/fer_ros2_simulation` and `GKnerd/fer_ws`.

**Verification:** from a clean clone of `fer_ros2_docker` on x86_64, import and build
core+sim, core+real and core+real+sim; on the Orin, import and build core+real.
Every combination builds with the unchanged `docker/Dockerfile`.

---

### Phase 1 — One description source, hardware as an overlay

The root-cause fix. Upstream `franka_description/robots/fer/fer.urdf.xacro` is the
only description source; the bringup adds hardware overlays on top of it.

**`fer_ros2_bringup`**
- Rename the package `fer_ros2_mjc_bringup` → `fer_ros2_bringup` (`package.xml`,
  `CMakeLists.txt`, every `FindPackageShare`).
- `urdf/fer_hw.urdf.xacro` — replaces `urdf/fer_mujoco.urdf.xacro`. Includes
  upstream `robots/fer/fer.urdf.xacro` (as the current sim wrapper already does)
  and forwards its arguments unchanged. Adds an argument `hardware`
  (`real`|`mujoco`|`mock`) and includes exactly one overlay under `xacro:if`, gated
  additionally by `ros2_control`:
  - `urdf/control/fer_real.ros2_control.xacro` — copied from
    `fer_ros2_driver/fer_bringup/urdf/fer_ros2_control.xacro` (source SHA in the
    commit message).
  - `urdf/control/fer_mujoco.ros2_control.xacro` plus `urdf/mujoco/*.xacro` —
    moved from `urdf/fer/` and `urdf/end_effector/`.
  - `urdf/control/fer_mock.ros2_control.xacro` — new, `mock_components/GenericSystem`.

**`fer_moveit_config`, `fer_skills`**
- Keep building from upstream `fer.urdf.xacro` with `ros2_control:=false`.
- Align the forwarded arguments (`hand`, `ee_id`, geometry) with those the bringup
  passes, so all consumers produce the same model.

**`fer_ros2_driver`, landing together with the above**
- Delete `fer_bringup/`, including `urdf/fer.xacro`, which called
  `franka_robot.xacro` directly instead of the upstream `fer.urdf.xacro`.
- Keep `franka_bringup` as a minimal driver-only launch so the driver stays
  startable standalone.

**Verification gate — blocking:** with overlays disabled, the bringup wrapper must
reproduce upstream exactly, for every hardware value.
```bash
UP=$(ros2 pkg prefix franka_description)/share/franka_description/robots/fer/fer.urdf.xacro
HW=$(ros2 pkg prefix fer_ros2_bringup)/share/fer_ros2_bringup/urdf/fer_hw.urdf.xacro
xacro $UP ros2_control:=false                   > /tmp/upstream.urdf
for h in real mujoco mock; do
  xacro $HW hardware:=$h ros2_control:=false    > /tmp/$h.urdf
  diff /tmp/upstream.urdf /tmp/$h.urdf          # must be empty
done
```
Plus: the real robot still moves under `effort_jtc`.

---

### Phase 2 — Expose all command interfaces in simulation

Removes the cause of `controller_selection.yaml`.

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
- `urdf/mujoco/franka_mujoco.xacro` — `<motor>` actuators unconditionally for all 7
  arm joints and the finger (delete the `control_type` branches). Motor matches the
  real FER, which is torque-controlled.
- `urdf/control/fer_mujoco.ros2_control.xacro` — `effort`, `position` and
  `velocity` command interfaces on every joint, matching the real overlay.
- `config/controllers_sim.yaml` — position and velocity PID gains keyed by actuator
  name, read via `control_toolbox::PidROS::initialize_from_ros_parameters()`
  (1731, 1747). `pos_gripper` requires a position PID on the finger.

**`fer_moveit_config`**
- Delete `config/controller_selection.yaml`, `select_default_controllers()` and its
  two launch arguments (`fer_moveit_launch.py:51-66, 310-321`).

Afterwards the sim and real `<ros2_control>` blocks have the same shape, and mode
selection is `ros2 control switch_controllers` in both environments.

---

### Phase 3 — One controller name set, one gain table

Naming scheme: `<command_interface>_<controller_type>`.

| Old (real) | Old (sim) | New |
|---|---|---|
| `effort_joint_trajectory_controller` | `joint_effort_traj_controller` | `effort_jtc` |
| `vel_joint_trajectory_controller` | — (added) | `vel_jtc` |
| — | `joint_pos_traj_controller` | `pos_jtc` |
| — | `gripper_position_controller` | `pos_gripper` |
| — | `gripper_effort_controller` | `effort_gripper` |
| `joint_state_broadcaster` | `joint_state_broadcaster` | unchanged |
| `franka_robot_state_broadcaster` | — | unchanged |
| (future) | (future) | `cartesian_impedance` |

Broadcasters keep upstream-conventional names. `fer_gripper` is a node name, not a
controller. `cartesian_impedance` is a deliberate exception to interface-first
naming: impedance control is unambiguously effort-based.

**`fer_ros2_bringup`**
- `config/controllers_common.yaml` — update rate, joint lists, JTC types, the effort
  gain table, `joint_state_broadcaster`.
- `config/controllers_real.yaml` — `franka_robot_state_broadcaster`, `arm_id: fer`.
- `config/controllers_sim.yaml` — gripper controllers, PID gains.
- `CMakeLists.txt` — remove every `find_package(... REQUIRED)` except `ament_cmake`.
  The bringup then builds without MuJoCo and moves from `fer_sim.repos` to
  `fer_core.repos`.
- Delete `config/franka_mujoco_controllers.yaml`, including the dead
  `cartesian_compliance_controller` and `motion_control_handle` blocks.

`ros2_control_node` merges the list of param files.

**`fer_moveit_config`**
- `config/moveit_controllers.yaml` — new names; `vel_jtc` entry added.

---

### Phase 4 — The gripper seam

The real gripper is not a ros2_control controller. `franka_gripper_node` launches as
`fer_gripper` and serves `/fer_gripper/gripper_action`.
`moveit_simple_controller_manager` resolves `<controller_name>/<action_ns>`, so the
real gripper is routable by configuration alone.

**`fer_moveit_config`**
- `config/moveit_controllers_real.yaml`:
  ```yaml
  fer_gripper: { type: GripperCommand, action_ns: gripper_action,
                 joints: [fer_finger_joint1] }
  ```
- `config/moveit_controllers_sim.yaml` — `pos_gripper` / `effort_gripper`.
- `launch/fer_moveit_launch.py` — select the fragment by `hardware`.

`MoveItSimpleControllerManager` reports all listed controllers as ACTIVE, so a dead
`fer_gripper` goes unnoticed; the real hardware fragment adds a liveness check.

A gripper relay controller presenting one interface in both environments is the full
fix and enables `width` / `epsilon` / `force` grasping. Deferred.

---

### Phase 5 — Composable launch ladder

The sim stack's laddering is kept and generalised: `hardware` becomes a parameter.

Rule: a launch file starting nodes from one package lives in that package; a launch
file composing several packages is a scenario and lives in `fer_ros2_bringup`.

**Layer launches:**

| File | Package | Starts |
|---|---|---|
| `moveit.launch.py` | `fer_moveit_config` | `move_group` and its RViz |
| `fer_skills.launch.py` | `fer_skills` | skill server |
| `bt_server.launch.py` | `fer_behavior_trees` | BT server |
| `world_model.launch.py` | `fer_world_model` | planning-scene observer |
| `hardware.launch.py` | `fer_ros2_bringup` | RSP, ros2_control, controllers, gripper |

`hardware.launch.py` takes `hardware:=real|mujoco|mock` and delegates to
`_hardware_real.launch.py`, `_hardware_mujoco.launch.py` or
`_hardware_mock.launch.py`. Only the MuJoCo fragment references
`mujoco_ros2_control`.

**Scenario ladder in `fer_ros2_bringup`:**

| File | Includes | Use |
|---|---|---|
| `fer_hardware.launch.py` | hardware | driver / sim bring-up, controller tuning |
| `fer_moveit.launch.py` | + moveit | planning |
| `fer_skills.launch.py` | + skills | skill-level testing |
| `fer_full.launch.py` | + world_model + bt | full autonomy |

Every rung takes `hardware:=`, passes it down, owns its own RViz and passes
`use_rviz:=false` below. `use_sim_time` is derived from `hardware` and never
defaulted.

**Deleted:** `empty_world`, `fer_mujoco_ros2_control`, `fer_mujoco_moveit`,
`fer_mjc_moveit_skills`, `fer_mjc_moveit_skills_bt` launch files.

The `/joint_states` asymmetry is kept deliberate inside the hardware fragments: real
aggregates `fer/joint_states` + `fer_gripper/joint_states` via
`joint_state_publisher`; sim publishes directly from `joint_state_broadcaster`.

---

### Phase 6 — CI

Two levels. Component CI never relies on anything the platform imports.

**Component CI — every component repo**
- Imports only its own `dependencies.repos` (where present).
- `colcon build`, `colcon test`.
- `lint` — `ament_flake8`, `ament_pep257`, `ament_uncrustify --reformat`,
  `ament_cpplint`.
- `fer_ros2_bringup` additionally: `description-invariant` (the Phase 1 diff against
  upstream).
- `fer_ros2_driver` additionally: libfranka build.

**Platform CI — `fer_ros2_docker`**
- `build-core-sim`, `build-core-real`, `build-core-both` — fresh import per
  combination, image build.
- `launch-mock` — core only; bring up `fer_moveit.launch.py hardware:=mock`, assert
  controller_manager is up, controllers loaded, `/joint_states` publishing,
  `move_group` active. `mock_components/GenericSystem` does not integrate effort, so
  this test uses `pos_jtc`; effort behaviour is covered by sim and hardware
  validation.
- `manifest-consistency` — every `dependencies.repos` agrees with the platform pins.
- `unique-package-names` — no package name appears twice across imported repos.
- `dependency-direction` — package dependencies match §2.3.
- `stale-names` — `grep` for retired controller names.
- `images` — Docker images built from a fresh import, never from a local tree.

Branch protection on `main` in every repo requires its CI.

Hardware behaviour — real-time, reflexes, error recovery — is not covered by CI and
stays a manual validation step. Covering it requires a self-hosted runner on the
Orin with the robot attached.

---

### Phase 7 — Cleanup and decommissioning

After Phases 1–6 are green and both robots are re-validated.

- Delete dead multi-arm configs and launch files in
  `fer_ros2_driver/franka_bringup/{config,launch}/{mixed,real}/` (`dual_*`, `mixed_*`).
- Align default branches to `main` (§2.6).
- Delete `GKnerd/fer_speed_and_separation_monitoring`.
- `fer_ros2_simulation` README note before archiving:
  *"Superseded by GKnerd/fer_ros2_docker on `<date>`. Archived."*

---

### Phase 8 — Skills redesign (follow-up)

`fer_skills` fuses three layers — contract, server, backend — so the MoveIt/MTC
coupling of the backend leaks into every caller.

**Current issues**

| Issue | Effect |
|---|---|
| Action definitions live in the MTC package | `fer_behavior_trees` depends on MoveIt and MTC to obtain message types |
| Robot values in the skills config (`fer_arm`, `fer_hand`, named states `"open"` / `"close"`) | Every robot change touches `fer_skills` |
| Gripper driven through SRDF named states | The contract cannot express width, epsilon or force |
| Server and backend are one object (`SkillServer` holds `picker_` / `placer_` MTC tasks) | No arbitration, no backend swap; concurrent goals collide |
| `hand_group` passed where a link name is expected (`mtc_pick_object.cpp:80`) | Planning group and link conflated; works only because the names coincide |

**Target design — in the `fer_skills` repo**

```
fer_skill_interfaces   neutral contract: frames, poses, object IDs, gripper width/force;
                       no group names, no named states, no MoveIt types
fer_skill_server       goal handling, validation, arbitration (one motion at a time),
                       dispatch to a backend through a small C++ interface
fer_skills_moveit      MTC / MoveGroup backend: maps the contract onto groups, links,
                       MTC stages; robot values arrive as parameters
```

- `fer_behavior_trees` depends on `fer_skill_interfaces` only and becomes
  independent of MoveIt and of the robot.
- `fer_skills_moveit` remains MoveIt-coupled by design; a second planning framework
  is a second backend package.
- The FER-specific parameters (group names, TCP link, gripper limits) move to
  `fer_ros2_bringup/config/skill_server_fer.yaml` and are passed by the skills rung.
- `fer_world_model` stays MoveIt-coupled; it observes `/monitored_planning_scene`.
- A pluginlib backend interface is introduced only when a second backend exists.
- `fer_skill_interfaces` is a public contract per §2.6.

---

## 5. Sequencing

| Phase | Content | Effort | Gate |
|---|---|---|---|
| 0 | promote `fer_ros2_docker`, profiles, renames | ~1 d | clean-clone builds, all three combinations |
| 1 | upstream description + hardware overlays | ~1 d | diff against upstream empty, real robot moves |
| 2 | sim all-interfaces | ~1 d | runtime controller switch works |
| 6a | component CI + `launch-mock` | ~0.5 d | CI green |
| 3 | controller merge | ~0.5 d | both environments unchanged |
| 4 | gripper seam | ~0.5 d | MoveIt gripper works in both |
| 5 | launch ladder | ~1 d | every rung standalone, both environments |
| 6b | remaining platform CI | ~1.5 d | branch protection on |
| 7 | cleanup, archive | ~0.5 d | — |
| 8 | skills redesign | ~3–4 d | BTs build without MoveIt; concurrent goals rejected |

---

## 6. Risks

| Risk | Mitigation |
|---|---|
| Phase 1 changes kinematics unnoticed | Blocking diff against upstream, permanent CI job |
| Real robot regresses | Driver source untouched; hardware re-validation after Phases 1, 3, 5 |
| Phase 1 lands in one repo but not the others | Changes to `fer_ros2_bringup`, `fer_moveit_config`, `fer_skills` and `fer_ros2_driver` merged together; `unique-package-names` CI |
| Consumers pass different arguments to the upstream xacro | Arguments aligned in Phase 1; `description-invariant` CI compares against upstream |
| Package-level dependency cycle | MoveIt and skills depend on upstream `franka_description`, never on the bringup; `dependency-direction` CI |
| Position/velocity PID tuning poor | Effort mode is default; other modes inactive until tuned |
| Retired controller name missed | `stale-names` CI |
| Repo renames break clones and manifests | Profiles updated in the same step; GitHub redirects as a fallback only |
| rosdep warnings mistaken for errors | Documented in the platform README; both profile builds in CI |

---

## 7. Failure modes of the multi-repo layout and their guards

| # | Failure mode | Guard |
|---|---|---|
| 1 | Floating `main` pins make the platform unreproducible | Dev profiles track branches; `vcs export --exact` into `releases/` |
| 2 | Cross-repo changes merged out of order | Additive provider changes first; provider merges before consumer; identical feature-branch names, honoured by platform CI |
| 3 | Component CI green only because the platform supplies a dependency | Component CI imports only its own `dependencies.repos` |
| 4 | Two repos pin the same external differently | Platform owns every pin; `manifest-consistency` CI |
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

1. With `ros2_control:=false`, the bringup wrapper's URDF is byte-identical to
   upstream `franka_description` for all `hardware` values.
2. The platform builds with core+sim and with core+real; neither profile requires
   the other.
3. The platform launches with core only (`hardware:=mock`).
4. `fer_ros2_driver` builds from its own `dependencies.repos`.
5. One controller name set, one gain table, one description source (upstream), one
   launch ladder.
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
  poses), not raw point clouds. Images crossing the network use `image_transport`
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
- **The skill contract is goal-terminated.** Condition-terminated and reactive
  behaviour (tracking, handover, SSM speed scaling, MoveIt Servo) needs a streaming
  contract and a speed-scaling hook in the control path.
- **The driver is frozen on libfranka 0.9.2.** Distro and toolchain upgrades may
  require further patches; libfranka patches are kept as separate commits.
- **SSM is not a certified safety function.** Protective stops go through the
  Franka safety system and E-stop.

---

## 11. The wrench contract

The sim/real contract is a standard ROS message, never a vendor message. Consumers —
SSM, contact detection, skills — depend on `geometry_msgs/WrenchStamped`, never on
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
