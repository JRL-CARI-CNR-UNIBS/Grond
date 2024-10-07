# Copyright 2024 STIIMA
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

import os
import yaml

from ament_index_python.packages import get_package_share_directory

from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch import LaunchDescription
from launch_ros.actions import Node

# def trasform_publisher_with_target_frame(context):
#     point_to_laser_config_path = LaunchConfiguration('point_to_laser_config_path').perform(context)
    
#     target_frame = 'lidar'
#     try:
#         with open(point_to_laser_config_path, 'r') as file:
#             config_file = yaml.safe_load(file)
#         target_frame = config_file.get('pointcloud_to_laserscan_node', {}).get(
#             'ros__parameters', {}).get('target_frame', 'lidar')
#     except FileNotFoundError:
#         print(f"Configuration file {point_to_laser_config_path} not found. Using default target frame: 'lidar'.")
#     except yaml.YAMLError as exc:
#         print(f"Error parsing YAML file: {exc}. Using default target frame: 'lidar'.")
    
#     transform_pub_cmd = Node(
#         package='tf2_ros',
#         executable='static_transform_publisher',
#         name='static_transform_publisher',
#         arguments=[
#             '--x', '0', '--y', '0', '--z', '0',
#             '--qx', '0', '--qy', '0', '--qz', '0', '--qw', '1',
#             '--frame-id', 'map', '--child-frame-id', target_frame
#         ]
#     )

#     return [transform_pub_cmd]


def generate_launch_description():
    package_dir = get_package_share_directory('pdor_bringup')

    namespace_scanner_cmd = DeclareLaunchArgument(
        name='scanner', default_value='scanner',
        description='Namespace for sample topics'
    )

    default_point_to_laser_config_path = os.path.join(
        package_dir,
        'config',
        'blickfeld_configuration.yaml'
    )

    point_to_laser_config_cmd = DeclareLaunchArgument(
        name='point_to_laser_config_path', 
        default_value=default_point_to_laser_config_path,
        description='Path to the config file for the point to laser node'
    )
    
    point_to_laser_cmd = Node(
            package='pointcloud_to_laserscan', executable='pointcloud_to_laserscan_node',
            remappings=[('cloud_in', '/blickfeld_cube1_front/points_raw'),
                        ('scan', [LaunchConfiguration(variable_name='scanner'), '/scan'])],
            parameters=[LaunchConfiguration('point_to_laser_config_path')],
            name='pointcloud_to_laserscan'
        )
    
    # trasform_publisher = OpaqueFunction(function = trasform_publisher_with_target_frame)

    ld = LaunchDescription()
    ld.add_action(namespace_scanner_cmd)
    ld.add_action(point_to_laser_config_cmd)
    ld.add_action(point_to_laser_cmd)
    # ld.add_action(trasform_publisher)

    return ld