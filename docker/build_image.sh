#!/bin/bash

#    Copyright 2025 Proximity Robotics & Automation GmbH
#    Modifications copyright 2026 Georgios Katranis
#
#    This script is derived from internal Docker tooling developed at
#    Proximity Robotics & Automation GmbH. It has been adapted for the
#    Franka Emika Robot (FER) platform: single Dockerfile for x86_64 and
#    aarch64, architecture-specific image tag, and the deps directory.

#    Licensed under the Apache License, Version 2.0 (the "License");
#    you may not use this file except in compliance with the License.
#    You may obtain a copy of the License at

#        http://www.apache.org/licenses/LICENSE-2.0

#    Unless required by applicable law or agreed to in writing, software
#    distributed under the License is distributed on an "AS IS" BASIS,
#    WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
#    See the License for the specific language governing permissions and
#    limitations under the License.

uid=$(eval "id -u")
gid=$(eval "id -g")

# ANSI escape codes
RED_BOLD="\033[1;31m"
YELLOW_BOLD="\033[1;33m"
GREEN_BOLD="\033[1;32m"
RESET="\033[0m"

PACKAGE_NAME="fer_ros2_docker"
CONTAINER_USER="fer_ros2"
IMAGE="$PACKAGE_NAME/ros:jazzy_$(uname -m)"

# Set Package root
if [[ "$(pwd)" == *"/$PACKAGE_NAME/"* ]]; then
    # Case A: Inside a subdirectory
    echo -e "${YELLOW_BOLD}Inside subdirectory. Navigating to root...${RESET}"
    # Strip everything after the package name to find the root
    _cwd="$(pwd)"
    PACKAGE_ROOT="${_cwd%%/$PACKAGE_NAME/*}/$PACKAGE_NAME"
    cd "$PACKAGE_ROOT" || exit 1
elif [[ "$(pwd)" == *"/$PACKAGE_NAME" ]]; then
    # Case B: Already at the root
    echo -e "${GREEN_BOLD}Already at package root.${RESET}"
    PACKAGE_ROOT="$(pwd)"
else
    # Case C: Not in the package at all
    echo -e "${RED_BOLD}Error: You are not inside the directory '$PACKAGE_NAME'.${RESET}"
    echo "Current path: $(pwd)"
    exit 1
fi

for FOLDER in ros2_ws/src deps env log data; do
    HOST_PATH="$PACKAGE_ROOT/$FOLDER"
    if [ ! -d "$HOST_PATH" ]; then
        echo -e "${YELLOW_BOLD}Warning: $HOST_PATH does not exist. Creating it...${RESET}"
        mkdir -p "$HOST_PATH"
    fi
done

docker build \
    --build-arg UID="$uid" \
    --build-arg GID="$gid" \
    --network=host \
    -t $IMAGE . \
    -f $PACKAGE_ROOT/docker/Dockerfile \
    && docker create --name temp-container $IMAGE \
    && docker cp temp-container:/home/${CONTAINER_USER}/ros2_ws/. $PACKAGE_ROOT/ros2_ws/. \
    && docker rm temp-container
