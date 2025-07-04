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

if [[ ! -z "$PACKAGES" ]]; then
    # No need to install and build dependencies as they are already present in /opt/carma/install/setup.bash
    echo "Sourcing previous build for incremental build start point..."
    source /opt/carma/install/setup.bash
else
    # Get driver
    sudo apt-get update && sudo apt install curl

    # Before installing driver and the wrapper, temporary workaround until Eigen3 and PCL are properly exported by velodyne_pointcloud
    # open issue: https://github.com/ros-drivers/velodyne/issues/550, workaround by: @hect95
    sudo sed -i 's/\beigen\b/Eigen3/g' /opt/ros/humble/share/velodyne_pointcloud/cmake/ament_cmake_export_dependencies-extras.cmake
    sudo sed -i 's/\bpcl\b/PCL/g' /opt/ros/humble/share/velodyne_pointcloud/cmake/ament_cmake_export_dependencies-extras.cmake

    # Install the driver and pointcloud conversion package
    sudo apt install ros-humble-velodyne-driver -y

    echo "Sourcing base image for a full build..."
    source /opt/ros/humble/setup.bash
    source /opt/autoware.ai/ros/install/setup.bash
fi

# Don't proceed in Continuous Integration environment
if [[ "$CI" == "true" ]]; then
    exit
fi

# Build wrapper
cd ~
if [[ ! -z "$PACKAGES" ]]; then
    colcon build --cmake-args -DCMAKE_BUILD_TYPE=Release --packages-above $PACKAGES
else
    colcon build --cmake-args -DCMAKE_BUILD_TYPE=Release --packages-up-to velodyne_lidar_driver_wrapper driver_shutdown_ros2
fi
