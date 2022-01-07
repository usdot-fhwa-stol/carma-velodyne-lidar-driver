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


import os

import ament_index_python.packages
import launch
import launch_ros.actions
from launch.actions import DeclareLaunchArgument
from ament_index_python import get_package_share_directory

from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration

import yaml


def generate_launch_description():
     # Args for driver
    frame_id = LaunchConfiguration('frame_id')
    declare_frame_id = DeclareLaunchArgument(name = 'frame_id', default_value = "velodyne", description="The frame id to use for the scan data")

    device_ip = LaunchConfiguration('device_ip')
    declare_device_ip = DeclareLaunchArgument(name = 'device_ip', default_value='192.168.1.201', description="Ip address of velodyne device")

    max_range = LaunchConfiguration('max_range')
    declare_max_range = DeclareLaunchArgument(name = 'max_range', default_value='200', description="Maximum lidar range in meters")

    port = LaunchConfiguration('port')
    declare_port = DeclareLaunchArgument(name = 'port', default_value='2368', description="Communication port of lidar")

    cut_angle = LaunchConfiguration('cut_angle')
    declare_cut_angle = DeclareLaunchArgument(name = 'cut_angle', default_value = '-0.01')

    # Args for Pointcloud
    organize_cloud = LaunchConfiguration('organize_cloud')
    declare_organize_cloud = DeclareLaunchArgument(name ='organize_cloud', default_value = 'False')

    # Define Nodes
    driver_share_dir = get_package_share_directory('velodyne_driver')
    driver_params_file = os.path.join(driver_share_dir, 'config', 'VLP32C-velodyne_driver_node-params.yaml')
    velodyne_driver_node = launch_ros.actions.Node(package='velodyne_driver',
                                                   executable='velodyne_driver_node',
                                                   output='both',
                                                   parameters=[driver_params_file,
                                                        {'frame_id':frame_id}, {'device_ip': device_ip}, {'max_range': max_range}, 
                                                        {'port': port}, {'cut_angle': cut_angle}]
                                                    )

    convert_share_dir = get_package_share_directory('velodyne_pointcloud')
    convert_params_file = os.path.join(convert_share_dir, 'config', 'VLP32C-velodyne_convert_node-params.yaml')
    with open(convert_params_file, 'r') as f:
        convert_params = yaml.safe_load(f)['velodyne_convert_node']['ros__parameters']
    convert_params['calibration'] = os.path.join(convert_share_dir, 'params', 'VeloView-VLP-32C.yaml')
    velodyne_convert_node = launch_ros.actions.Node(package='velodyne_pointcloud',
                                                    executable='velodyne_convert_node',
                                                    output='both',
                                                    parameters=[convert_params,
                                                        {"organize_cloud": organize_cloud}],
                                                    remappings = [('velodyne_points', 'lidar/points_raw')]
                                                    )

    return launch.LaunchDescription([
                                    # Args
                                    declare_frame_id,
                                    declare_device_ip,
                                    declare_max_range,
                                    declare_port,
                                    declare_cut_angle,
                                    declare_organize_cloud,
                                    # Nodes
                                    velodyne_driver_node,
                                    velodyne_convert_node,

                                     launch.actions.RegisterEventHandler(
                                         event_handler=launch.event_handlers.OnProcessExit(
                                             target_action=velodyne_driver_node,
                                             on_exit=[launch.actions.EmitEvent(
                                                 event=launch.events.Shutdown())],
                                         )),
                                     ])
