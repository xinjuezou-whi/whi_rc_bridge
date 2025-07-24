# Copyright 2025 WheelHub Intelligent
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

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
import os

from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    # Input parameters declaration
    robot_name = LaunchConfiguration('robot_name')

    # Declare arguments
    declare_robot_name_arg = DeclareLaunchArgument(
        'robot_name', default_value='',
        description='Robot name'
    )

    # Path to config file
    config_file = os.path.join(
        get_package_share_directory('whi_rc_bridge'),
        'config',
        'config.yaml'
    )

    # Node definition
    start_whi_rc_bridge_node = Node(
        package='whi_rc_bridge',
        executable='whi_rc_bridge_node',
        name='whi_rc_bridge',
        output='screen',
        parameters=[config_file]
    )

    return LaunchDescription([
        declare_robot_name_arg,
        start_whi_rc_bridge_node
    ])
