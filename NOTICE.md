# Third-Party Software Notices

This repository builds the Docker image for the Franka Emika Robot (FER) ROS 2
Jazzy platform and pins the external components imported through
`fer_core.repos`, `fer_real.repos` and `fer_sim.repos`. We gratefully
acknowledge the developers and contributors of the following projects. Each
component is provided under the license indicated; the full license texts are
available at the linked sources.

-------------------------------------------------------------------------
1. ROS 2 (Robot Operating System), `ros:jazzy-ros-base` base image
-------------------------------------------------------------------------
Copyright (c) Open Source Robotics Foundation (OSRF) / Open Robotics
and contributors.
License: Apache License 2.0
Source: https://github.com/ros2
Base image: https://hub.docker.com/_/ros (`ros:jazzy-ros-base`)

ROS 2 is an open-source middleware suite for robot software development.
This project targets the Jazzy Jalisco distribution and uses the
official OSRF-maintained Docker base image on x86_64 and aarch64.

-------------------------------------------------------------------------
2. libfranka
-------------------------------------------------------------------------
Copyright (c) Franka Robotics GmbH.
License: Apache License 2.0
Source: https://github.com/frankarobotics/libfranka (version 0.9.2)
Submodule: https://github.com/frankaemika/libfranka-common

C++ client library for the Franka Control Interface (FCI). Imported into
`deps/libfranka` by `fer_real.repos` and built inside the image when present.

-------------------------------------------------------------------------
3. franka_description
-------------------------------------------------------------------------
Copyright (c) Franka Robotics GmbH.
License: Apache License 2.0
Source: https://github.com/frankarobotics/franka_description

Official URDF/xacro descriptions, meshes and model files for Franka
robots, used unmodified as the robot description of the FER. Imported by
`fer_core.repos`.

-------------------------------------------------------------------------
4. BehaviorTree.ROS2
-------------------------------------------------------------------------
Copyright (c) Davide Faconti and contributors.
License: see upstream — the repository's top-level `LICENSE` file is
Apache License 2.0, while individual `package.xml` files declare MIT.
Both notices are preserved in the imported source tree.
Source: https://github.com/BehaviorTree/BehaviorTree.ROS2

ROS 2 bindings and node/action server templates for BehaviorTree.CPP.
Imported by `fer_core.repos`, pinned to a commit that fixes a thread
crash on ROS 2 Jazzy.

-------------------------------------------------------------------------
5. MuJoCo (Multi-Joint dynamics with Contact)
-------------------------------------------------------------------------
Copyright (c) DeepMind Technologies Limited.
License: Apache License 2.0
Source: https://github.com/google-deepmind/mujoco

Open-source physics engine used as the simulation back end. Obtained
through `mujoco_vendor`.

-------------------------------------------------------------------------
6. mujoco_vendor
-------------------------------------------------------------------------
Copyright (c) PAL Robotics.
License: Apache License 2.0
Source: https://github.com/pal-robotics/mujoco_vendor

CMake vendor package that integrates the MuJoCo binary into the ROS 2
build. Imported by `fer_sim.repos`.

-------------------------------------------------------------------------
7. mujoco_ros2_control
-------------------------------------------------------------------------
Copyright (c) ros-controls working group and contributors.
License: Apache License 2.0
Source: https://github.com/ros-controls/mujoco_ros2_control

Hardware interface that bridges MuJoCo with the `ros2_control`
framework. Imported by `fer_sim.repos`.

-------------------------------------------------------------------------
8. Docker tooling — `docker/Dockerfile`, `docker/build_image.sh`,
   `docker/run_container.sh`
-------------------------------------------------------------------------
Copyright (c) 2025 Proximity Robotics & Automation GmbH.
Modifications copyright (c) 2026 Georgios Katranis.
License: Apache License 2.0
Source: <internal — no public repository>

The Dockerfile layering pattern and the `build_image.sh` /
`run_container.sh` shell scripts are derived from internal tooling
developed at Proximity Robotics & Automation GmbH. They have been adapted
for the FER platform: dependency set, user-creation block, CycloneDDS
configuration, the optional libfranka build, the colcon build stage, a
single image for x86_64 and aarch64, and real-time container limits. Each
file retains the original copyright notice and carries a modification
notice per Apache 2.0 § 4(b).

-------------------------------------------------------------------------

Unless otherwise noted above, the listed software is licensed under the
Apache License, Version 2.0 (the "License"); you may not use these files
except in compliance with the License. You may obtain a copy of the
License at:

    http://www.apache.org/licenses/LICENSE-2.0

Unless required by applicable law or agreed to in writing, software
distributed under the License is distributed on an "AS IS" BASIS,
WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or
implied. See the License for the specific language governing permissions
and limitations under the License.
