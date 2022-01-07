#  Copyright (C) 2021 LEIDOS.

#  Licensed under the Apache License, Version 2.0 (the "License"); you may not
#  use this file except in compliance with the License. You may obtain a copy of
#  the License at

#  http://www.apache.org/licenses/LICENSE-2.0

#  Unless required by applicable law or agreed to in writing, software
#  distributed under the License is distributed on an "AS IS" BASIS, WITHOUT
#  WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied. See the
#  License for the specific language governing permissions and limitations under
#  the License.

from ament_index_python import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import ComposableNodeContainer
from launch_ros.descriptions import ComposableNode
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from carma_ros2_utils.launch.get_current_namespace import GetCurrentNamespace

from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource

import os

def generate_launch_description():

    # Declare the launch arguments
    log_level = LaunchConfiguration('log_level')
    declare_log_level_arg = DeclareLaunchArgument(
        name ='log_level', default_value = 'DEBUG', description="Log level to print.", choices=["DEBUG","INFO","WARN","ERROR","FATAL"])


    # # Define Velodyne Driver Node and pointcloud converter
    velodyne_lidar_driver_wrapper_pkg = get_package_share_directory('velodyne_lidar_driver_wrapper')
    velodyne_driver_node = IncludeLaunchDescription(
                PythonLaunchDescriptionSource(['/', velodyne_lidar_driver_wrapper_pkg, '/launch', '/velodyne_lidar_driver_launch.py']),
    )

    #  Get parameter file path
    param_file_path = os.path.join(
        get_package_share_directory('velodyne_lidar_driver_wrapper'), 'config/parameters.yaml')

    # Launch node(s) in a carma container to allow logging to be configured
    velodyne_lidar_wrapper_container = ComposableNodeContainer(
        package='carma_ros2_utils',
        name='velodyne_lidar_driver_wrapper_container',
        namespace=GetCurrentNamespace(),
        executable='carma_component_container_mt',
        composable_node_descriptions=[
            
            # Launch the core node(s)
            ComposableNode(
                    package='velodyne_lidar_driver_wrapper',
                    plugin='velodyne_lidar_driver_wrapper::Node',
                    name='velodyne_lidar_driver_wrapper_node',
                    namespace=GetCurrentNamespace(),
                    extra_arguments=[
                        {'use_intra_process_comms': True},
                        {'--log-level' : log_level }
                    ],
                    parameters=[ param_file_path ]
            ),
        ]
    )
    return LaunchDescription([
        # Specify Args
        declare_log_level_arg,
        velodyne_driver_node,
        velodyne_lidar_wrapper_container
    ])