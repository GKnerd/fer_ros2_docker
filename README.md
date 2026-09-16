# Franka Emika Robot (FER / Panda) — ROS 2 Jazzy platform

Meta-repository for the FER stack: Docker image, DDS configuration, workspace
profiles and project documentation. It contains no ROS packages; those are
imported into `ros2_ws/src` (and libfranka into `deps/`) from the profile
manifests.

Supported hosts: x86_64 workstations and the Jetson Orin AGX (aarch64).

## Profiles

| Manifest | Contents |
|---|---|
| `fer_core.repos` | `franka_description`, `BehaviorTree.ROS2`, `fer_ros2_bringup`, `fer_moveit_config`, `fer_skills`, `fer_behavior_trees`, `fer_planning_world_model`, `speed_and_separation_monitoring` |
| `fer_real.repos` | `libfranka` 0.9.2 (`deps/`), `fer_ros2` (driver) |
| `fer_sim.repos` | `mujoco_ros2_control`, `mujoco_vendor` |

`fer_core.repos` is always imported. Add the real profile, the simulation
profile, or both:

```bash
git clone git@github.com:GKnerd/fer_ros2_docker.git && cd fer_ros2_docker

vcs import . < fer_core.repos     # always
vcs import . < fer_real.repos     # real robot
vcs import . < fer_sim.repos      # simulation
```

| Import | Host | Result |
|---|---|---|
| core + sim | x86_64 | MuJoCo simulation |
| core + real | x86_64, Orin | real robot |
| core + real + sim | x86_64 | both |

The Orin runs the real robot only; do not import the simulation profile there.

`fer_ros2_bringup` serves both backends and is selected at launch time:

```bash
ros2 launch fer_ros2_bringup fer_moveit_skills_bt.launch.py hardware:=mujoco
ros2 launch fer_ros2_bringup fer_moveit_skills_bt.launch.py hardware:=real robot_ip:=<FCI address>
```

## Build and run

```bash
./docker/build_image.sh
./docker/run_container.sh
```

One Dockerfile (`docker/Dockerfile`) serves both architectures. The image tag
is `fer_ros2_docker/ros:jazzy_<arch>` (`x86_64` or `aarch64`).

- libfranka is built from `deps/libfranka` when the real profile is imported
  and skipped otherwise.
- colcon builds whatever the imported profiles placed in `ros2_ws/src`.
- The container runs with real-time limits (`rtprio`, `memlock`) on every host;
  they only take effect on the real robot.
- `env/` is mounted into the container; `CYCLONEDDS_URI` points at
  `env/cyclone_dds.xml`.

## Jetson Orin AGX

Tune the interrupt coalescing of the NIC that carries FCI traffic before a
control session:

```bash
sudo ./config/nic_orin_config.sh -i <iface>
```

The settings are volatile and are lost on reboot or driver reload.

## Layout

```
docker/     Dockerfile, build_image.sh, run_container.sh
env/        cyclone_dds.xml, dds_profile.xml
config/     nic_orin_config.sh
docs/       consolidation plan, review, handoff, robotics stack,
            DDS profiles, legacy READMEs
deps/       ignored; libfranka
ros2_ws/    ignored; imported component repos
```

## License

See `LICENSE` and `NOTICE.md`.
