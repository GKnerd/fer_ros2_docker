# fer_ros2_docker

ROS 2 Jazzy platform for the Franka Emika Robot (FER / Panda, libfranka 0.9.2): control,
planning, behavior trees, perception. Real robot on the Jetson Orin, MuJoCo simulation on
x86_64.

## Start of every session

- Read `docs/STATUS.md`: current step, open items, recent decisions.
- The plan is `docs/FER_ROS2_Consolidation_Plan.md`. Copies outside `docs/` are stale.

## Layout

- This repo is the meta-repo: Docker image, DDS config, `.repos` profiles, docs. It holds no
  ROS packages.
- `ros2_ws/src/*` are separate git repos imported with `vcs import . < fer_<core|real|sim>.repos`.
  Check git state per package repo, not only here.
- `deps/libfranka` is built only when the real profile is imported.

## Build and test

- Image: `./docker/build_image.sh`; container: `./docker/run_container.sh`.
- The user runs `colcon build` / `colcon test` in the dev container. Do not run them unasked.
- When asked to test: use a throwaway container (`--network none`, Fast DDS, own
  `ROS_DOMAIN_ID`, `--merge-install` overlay, `local_setup.bash`). Never touch the host
  `ros2_ws/build`, `install` or `log`.
- A sub-phase is done when `colcon test` passes for its packages.

## Fixed decisions — do not reopen

- Per-package repos plus `.repos` profiles; one meta-repo; one Dockerfile for both
  architectures; one bringup (`fer_ros2_bringup`, `hardware:=real|mujoco`).
- Robot description is upstream `franka_description`, never an own package.
- Renames, removals and archiving (`fer_skills`, `fer_ros2_mjc_bringup`, `fer_ros2` →
  `fer_ros2_driver`, …) happen only in 8.7, after the migration. Never propose them earlier.
- Servers never switch controllers. The user activates them (launch or
  `ros2 control switch_controllers`); a server waits for its controller and fails with a hint.
- Motion backend is a client of `move_group` (`fer_moveit_motion_server` in
  `fer_moveit_config`), not MoveItCpp; straight paths through Pilz LIN.
- Servers depend only on `fer_interfaces`. World-model clients are written per package.
- Objects are boxes only.
- Controller names: `<interface>_<type>_controller`.
- SSM (`speed_and_separation_monitoring`) stays separate.

## Gotchas

- MuJoCo: freejoints must be named (`MujocoSystemInterface` refuses unnamed joints).

## End of a session

When asked, update `docs/STATUS.md`: current step, what changed, decisions with their reason,
next step.
