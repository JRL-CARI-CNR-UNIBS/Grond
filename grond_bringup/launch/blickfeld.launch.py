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
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration

def generate_launch_description():
    package_dir = get_package_share_directory('grond_bringup')
    # Dichiarazione degli argomenti

    config_arg = DeclareLaunchArgument(
        'config', 
        default_value=os.path.join(
            package_dir, 
            'config', 'blickfeld_configuration.yaml'),
        description='Path to the config file for the point to laser node and the blickfeld driver'
    )

    # Launch file for pointcloud_to_laserscan
    pointcloud_to_laserscan_launch_file = os.path.join(
        package_dir,
        'launch',
        'pointcloud_to_laserscan.launch.py'
    )

    # Launch file for blickfeld driver
    blickfeld_driver_launch_file = os.path.join(
        package_dir,
        'launch',
        'blickfeld_driver.launch.py'  # Assicurati di avere il nome corretto del file
    )
    
    blickfeld_driver_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(blickfeld_driver_launch_file),
        launch_arguments={
            'config': LaunchConfiguration('config')
        }.items()
    )

    point_to_laserscan_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(pointcloud_to_laserscan_launch_file),
        launch_arguments={
            'config' : LaunchConfiguration('config')
        }.items()
    )

    ld = LaunchDescription()
    ld.add_action(scanner_arg)
    ld.add_action(config_arg)
    ld.add_action(blickfeld_driver_cmd)
    ld.add_action(point_to_laserscan_cmd)
    return ld