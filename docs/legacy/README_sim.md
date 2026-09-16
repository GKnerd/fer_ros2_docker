# fer_ros2_mjc_docker — ROS 2 MuJoCo Simulation for the Franka Emika Robot (FER)

A **meta-repository** that assembles a complete ROS 2 Jazzy simulation
environment for the Franka Emika Robot (FER) in MuJoCo. This repository
does not contain ROS 2 source packages itself — it bundles the glue
needed to stand them up reproducibly:

* A **Docker image** preloaded with ROS 2 Jazzy, MoveIt, MuJoCo, and
  CycloneDDS, plus helper scripts to build and run it.
* A **vcstool manifest** (`fer_ros2_mjc.repos`) that pins exact upstream
  and first-party package commits.
* A **CycloneDDS profile** (`env/cyclone_dds.xml`) tuned for trajectory
  traffic to avoid dropped packets on large MoveIt plans.
* Top-level **third-party attribution** for all bundled components.

The actual robotics code lives in the packages it imports. First-party
packages each live in their own GitHub repo and are version-pinned in
`fer_ros2_mjc.repos` alongside upstream third-party packages — see
[Workspace packages](#workspace-packages-imported-via-fer_ros2_mjcrepos)
below.

## Key Features
* **Modular URDF Wrapper**: A top-level Xacro wrapper injects MuJoCo
  physics into the upstream Franka URDF without modifying upstream
  Franka packages.
* **Dockerized Workflow**: Consistent development environment with all
  dependencies (MuJoCo, MoveIt, ros2_control, CycloneDDS) preinstalled.
* **Pinned, reproducible workspace**: All third-party sources and
  first-party packages are pinned to specific commits in
  `fer_ros2_mjc.repos`, so a clean re-import always reconstructs the
  same workspace.

---

## Prerequisites

Ensure the following are installed on your host machine:
* **Docker** (latest version)
* **vcstool**: To manage repository dependencies.
  ```bash
  pip install vcstool
  ```

## Installation & Setup

1. Clone the repository:
```bash
git clone https://github.com/GKnerd/fer_ros2_mjc_docker.git
cd fer_ros2_mjc_docker
```

2. Create the source directory and import dependencies:
```bash
mkdir -p ros2_ws/src
vcs import ros2_ws/src < fer_ros2_mjc.repos
```

3. Build the Docker image:
```bash
./docker/build_image.sh
```
This builds the image, compiles the full ROS 2 workspace inside the container, and copies the built workspace back to `ros2_ws/` on your host.

## Running the Simulation

0. Prelaunch setup:
**Crucial**
CycloneDDS requires a massive receive buffer to reliably handle large trajectory messages without dropping packets. You must configure your Host OS kernel to allow this, or the Docker container will crash on boot.
```bash
sudo sysctl -w net.core.rmem_max=2147483647
```
>Note 1: To make this persist across reboots, add `net.core.rmem_max=2147483647` to your `/etc/sysctl.conf file`.
>Note 2: You don´t have to specify 2GBs as here, this is use case specific. The value was chosen arbitratily in this case. 

1. Launch the container:
```bash
./docker/run_container.sh
```

2. Inside the container, run the main launch file:
```bash
ros2 launch fer_ros2_mjc_bringup fer_mujoco_ros2_control.launch.py
```
This opens the MuJoCo simulator and activates the effort controller for the arm and an effort controller for the hand.

If you wish to use the Mujoco simulator with MoveIt and an activated effort controller for the hand and arm you can use this launch file instead.

```bash
ros2 launch fer_ros2_mjc_bringup fer_mujoco_moveit.launch.py
```

## Repository Structure

```
fer_ros2_mjc_docker/
├── docker/
│   ├── Dockerfile              # Image definition (ROS 2 Jazzy + MoveIt + CycloneDDS)
│   ├── build_image.sh          # Builds the Docker image and extracts the built workspace
│   └── run_container.sh        # Runs the container with GPU/display/volume mounts
├── env/
│   └── cyclone_dds.xml         # CycloneDDS middleware profile
├── ros2_ws/
│   └── src/                    # Populated by vcs import (see Installation)
├── fer_ros2_mjc.repos          # VCS dependency manifest
├── DDS_Profiles.md             # Reference guide for CycloneDDS configuration and some additional experimental tips which are not on main
├── THIRDPARTY.md               # Third-party attribution notes
└── LICENSE
```

### Workspace packages (imported via `fer_ros2_mjc.repos`)

**First-party packages** (developed for this project, hosted under
[github.com/GKnerd](https://github.com/GKnerd)):

| Package | Description |
|---|---|
| [`fer_ros2_mjc_bringup`](https://github.com/GKnerd/fer_ros2_mjc_bringup) | Project-glue bringup: Xacro wrapper, controller configs, MuJoCo scenes, and the top-level launch files |
| [`fer_moveit_config`](https://github.com/GKnerd/fer_moveit_config) | MoveIt configuration for the FER |
| [`fer_skills`](https://github.com/GKnerd/fer_skills) | Reusable manipulation skills built on MoveIt |

**Upstream third-party packages** (pinned to specific commits, see
[`THIRDPARTY.md`](THIRDPARTY.md) for full attribution):

| Package | Source | Purpose |
|---|---|---|
| `franka_description` | [frankarobotics](https://github.com/frankarobotics/franka_description) | Official Franka URDF and meshes |
| `mujoco_ros2_control` | [ros-controls](https://github.com/ros-controls/mujoco_ros2_control) | MuJoCo hardware interface for `ros2_control` |
| `mujoco_vendor` | [pal-robotics](https://github.com/pal-robotics/mujoco_vendor) | CMake vendor package that integrates the MuJoCo binary |
| `BehaviorTree.ROS2` | [BehaviorTree](https://github.com/BehaviorTree/BehaviorTree.ROS2) | ROS 2 bindings for BehaviorTree.CPP (pinned past 0.3.0 for a Jazzy crash fix) |

**Apt-provided dependencies** (installed by the Dockerfile, not via
`.repos`): MoveIt, `ros2_control`, `ros2_controllers`, CycloneDDS,
RViz2, MoveIt Task Constructor, **BehaviorTree.CPP**
(`ros-jazzy-behaviortree-cpp`).

---

## Contribution Guidelines

This repository is currently a work in progress.

* **No direct commits to `main`**: All development must happen in feature branches.
* Create a new branch for your task:
  ```bash
  git checkout -b feature/your-feature-name
  ```
* Run a clean `colcon build` before submitting a Pull Request.
