# Copyright (C) 2021 LEIDOS.
#
# Licensed under the Apache License, Version 2.0 (the "License"); you may not
# use this file except in compliance with the License. You may obtain a copy of
# the License at
#
# http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS, WITHOUT
# WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied. See the
# License for the specific language governing permissions and limitations under
# the License.

cmake_minimum_required(VERSION 3.8)
project(velodyne_lidar_driver_wrapper)

find_package(carma_cmake_common REQUIRED)
carma_check_ros_version(2)
carma_package()

# Use C++17
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
  set(CMAKE_CXX_STANDARD_REQUIRED ON)
endif()

## Find dependencies using ament auto
find_package(ament_cmake_auto REQUIRED)
find_package(angles REQUIRED)  # Add this line explicitly otherwise won't build
ament_auto_find_build_dependencies()

# Name build targets
set(node_exec velodyne_lidar_driver_wrapper_node_exec)
set(node_lib velodyne_lidar_driver_wrapper_node)

# Includes

include_directories(
  include
)

include_directories(SYSTEM
  ${EIGEN3_INCLUDE_DIR}
)

# Build
ament_auto_add_library(${node_lib} SHARED
  src/velodyne_lidar_driver_wrapper.cpp
)

ament_auto_add_executable(${node_exec} 
        src/main.cpp)

# Register component
rclcpp_components_register_nodes(${node_lib} "velodyne_lidar_driver_wrapper::Node")

target_link_libraries( ${node_exec} 
          ${node_lib}
)

#Install
ament_auto_package(
  INSTALL_TO_SHARE config launch
)