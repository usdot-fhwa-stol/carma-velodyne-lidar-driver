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
from launch_ros.actions import Node
from launch_ros.actions import ComposableNodeContainer
from launch_ros.descriptions import ComposableNode
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, launch_configuration
from carma_ros2_utils.launch.get_current_namespace import GetCurrentNamespace
from launch.actions import Shutdown
from launch.actions import GroupAction
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import set_remap
from launch_ros.actions import set_parameter

import os

def generate_launch_description():

    # Declare the launch arguments
    log_level = LaunchConfiguration('log_level')
    declare_log_level_arg = DeclareLaunchArgument(
        name ='log_level', default_value = 'DEBUG', description="Log level to print.", choices=["DEBUG","INFO","WARN","ERROR","FATAL"])


    # Args for driver
    frame_id = LaunchConfiguration('frame_id')
    declare_frame_id = DeclareLaunchArgument(name = 'frame_id', default_value = "velodyne", description="The frame id to use for the scan data")

    device_ip = LaunchConfiguration('device_ip')
    declare_device_ip = DeclareLaunchArgument(name = 'device_ip', default_value='192.168.1.201', description="Ip address of velodyne device")

    max_range = LaunchConfiguration('max_range')
    declare_max_range = DeclareLaunchArgument(name = 'max_range', default_value='200', description="Maximum lidar range in meters")

    port = LaunchConfiguration('port')
    declare_port = DeclareLaunchArgument(name = 'port', default_value='2368', description="Communication port of lidar")

    model = LaunchConfiguration('model')
    declare_model = DeclareLaunchArgument(name = 'model', default_value = '32C')

    cut_angle = LaunchConfiguration('cut_angle')
    declare_cut_angle = DeclareLaunchArgument(name = 'cut_angle', default_value = '-0.01')

    # Args for Pointcloud
    velodyne_pointcloud_pkg = get_package_share_directory('velodyne_pointcloud')

    organize_cloud = LaunchConfiguration('organize_cloud')
    declare_organize_cloud = DeclareLaunchArgument(name ='organize_cloud', default_value = 'False')

    # Define Velodyne Driver Node
    velodyne_driver_pkg = get_package_share_directory('velodyne_driver')

    # Define Velodyne ROS2 driver node along with pointcloud converter
    velodyne_driver_group = GroupAction(
                    actions = [
                        #IncludeLaunchDescription(
                        #    PythonLaunchDescriptionSource(['/', velodyne_driver_pkg, '/launch', '/velodyne_driver_node-VLP32C-launch.py']),
                        #    launch_arguments={
                        #        'frame_id': frame_id, 
                        #        'device_ip':device_ip, 
                        #        'max_range':max_range, 
                        #        'port': port, 
                        #        'model' : model, 
                        #        'cut_angle' : cut_angle
                        #    }.items()
                        #),
                        set_remap.SetRemap('velodyne_points','lidar/points_raw'),
                        IncludeLaunchDescription(
                            PythonLaunchDescriptionSource(['/', velodyne_pointcloud_pkg, '/launch', '/velodyne_convert_node-VLP32C-launch.py']),
                            launch_arguments={'organize_cloud':organize_cloud}.items()
                        ),
                    ]
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
            ComposableNode(
                package='velodyne_driver',
                plugin='VelodyneDriver::VelodyneDriver',
                name='velodyne_driver_node',
                output='both',
                #parameters=[ velodyne_driver_params ], # Need to get this param file
                remappings = ['velodyne_points', 'lidar/points_raw'],
                extra_arguments = {
                    'frame_id': frame_id, 
                    'device_ip':device_ip, 
                    'max_range':max_range, 
                    'port': port, 
                    'model' : model, 
                    'cut_angle' : cut_angle                
                }.items(), # Need to list out all of these
                on_exit = Shutdown() # Remove?
            )
        ]
    )

    return LaunchDescription([
        # Specify Args
        declare_log_level_arg,
        declare_frame_id,
        declare_device_ip,
        declare_max_range,
        declare_port,
        declare_model,
        declare_cut_angle,
        # Pointcloud args
        declare_organize_cloud,
        # Specify Nodes
        velodyne_driver_group,
        velodyne_driver,
        velodyne_lidar_wrapper_container
    ])