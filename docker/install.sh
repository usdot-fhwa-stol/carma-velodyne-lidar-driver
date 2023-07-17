#!/bin/bash

#  Copyright (C) 2018-2021 LEIDOS.
# 
#  Licensed under the Apache License, Version 2.0 (the "License"); you may not
#  use this file except in compliance with the License. You may obtain a copy of
#  the License at
# 
#  http://www.apache.org/licenses/LICENSE-2.0
# 
#  Unless required by applicable law or agreed to in writing, software
#  distributed under the License is distributed on an "AS IS" BASIS, WITHOUT
#  WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied. See the
#  License for the specific language governing permissions and limitations under
#  the License.

# Source ros2
if [[ ! -z "$ROS2_PACKAGES" ]]; then
    echo "Sourcing previous build for incremental build start point..."
    source /opt/carma/install/setup.bash
else
    echo "Sourcing base image for full build..."
    source /opt/ros/foxy/setup.bash
fi

# Get driver
sudo apt-get update && sudo apt install curl gnupg2 lsb-release
sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key  -o /usr/share/keyrings/ros-archive-keyring.gpg
echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(lsb_release -cs) main" | sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null

# Install the driver and pointcloud conversion package
sudo apt install ros-foxy-velodyne-driver -y

# Don't proceed in Continuous Integration environment
if [[ "$CI" == "true" ]]; then
    exit
fi

# Build wrapper
cd ~
if [[ ! -z "$ROS2_PACKAGES" ]]; then
    colcon build --cmake-args -DCMAKE_BUILD_TYPE=Release --packages-above $ROS2_PACKAGES
else
    colcon build --cmake-args -DCMAKE_BUILD_TYPE=Release --packages-up-to velodyne_lidar_driver_wrapper driver_shutdown_ros2
fi
