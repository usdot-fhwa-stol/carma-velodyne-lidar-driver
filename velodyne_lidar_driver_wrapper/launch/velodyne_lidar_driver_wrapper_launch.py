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
import launch
from launch_ros.actions import ComposableNodeContainer
from launch_ros.descriptions import ComposableNode
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration

from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource

import os

def generate_launch_description():

    # Declare the launch arguments
    log_level = LaunchConfiguration('log_level')
    declare_log_level_arg = DeclareLaunchArgument(
        name ='log_level', default_value = 'WARN', description="Log level to print.", choices=["DEBUG","INFO","WARN","ERROR","FATAL"])

    frame_id = LaunchConfiguration('frame_id')
    declare_frame_id = DeclareLaunchArgument(name = 'frame_id', default_value = "velodyne", description="The frame id to use for the scan data")

    device_ip = LaunchConfiguration('device_ip')
    declare_device_ip = DeclareLaunchArgument(name = 'device_ip', default_value='192.168.1.201', description="Ip address of velodyne device")

    max_range = LaunchConfiguration('max_range')
    declare_max_range = DeclareLaunchArgument(name = 'max_range', default_value='200', description="Maximum lidar range in meters")

    port = LaunchConfiguration('port')
    declare_port = DeclareLaunchArgument(name = 'port', default_value='2368', description="Communication port of lidar")

    # Define Velodyne Driver Node
    velodyne_driver_pkg = get_package_share_directory('velodyne_driver')
    velodyne_driver_node = IncludeLaunchDescription(
                PythonLaunchDescriptionSource(['/', velodyne_driver_pkg, '/launch', '/velodyne_driver_node-VLP32C-launch.py']),
                launch_arguments={'frame_id': frame_id, 'device_ip':device_ip, 'max_range':max_range, 'port': port}.items(),
                
    )
    # Define Velodyne ROS2 raw to pointcloud converter
    # This nodes subscribes to the velodyne packets published by velodyne_driver_node and publishes the pointcloud
    velodyne_pointcloud_pkg = get_package_share_directory('velodyne_pointcloud')
    velodyne_pointcloud_node = IncludeLaunchDescription(
                PythonLaunchDescriptionSource(['/', velodyne_pointcloud_pkg, '/launch', '/velodyne_transform_node-VLP32C-launch.py'])
    )

    #  Get parameter file path
    param_file_path = os.path.join(
        get_package_share_directory('velodyne_lidar_driver_wrapper'), 'config/parameters.yaml')

    # Launch node(s) in a carma container to allow logging to be configured
    velodyne_lidar_wrapper_container = ComposableNodeContainer(
        package='carma_ros2_utils',
        name='velodyne_lidar_driver_wrapper_container',
        namespace='/',
        executable='carma_component_container_mt',
        composable_node_descriptions=[
            
            # Launch the core node(s)
            ComposableNode(
                    package='velodyne_lidar_driver_wrapper',
                    plugin='velodyne_lidar_driver_wrapper::Node',
                    name='velodyne_lidar_driver_wrapper_node',
                    namespace="/",
                    extra_arguments=[
                        {'use_intra_process_comms': True},
                        {'--log-level' : log_level }
                    ],
                    parameters=[ param_file_path ]
            ),
        ]
    )
    return LaunchDescription([
        declare_log_level_arg,
        declare_frame_id,
        declare_device_ip,
        declare_max_range,
        declare_port,
        velodyne_driver_node,
        velodyne_pointcloud_node,
        velodyne_lidar_wrapper_container
    ])