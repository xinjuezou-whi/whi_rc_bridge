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
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare
from launch_ros.actions import Node

from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    # Input parameters declaration
    namespace = LaunchConfiguration('namespace')
    use_stamped_vel = LaunchConfiguration('use_stamped_vel')

    # Declare arguments
    declare_namespace_arg = DeclareLaunchArgument(
        'namespace', default_value='',
        description='Top-level namespace'
    )
    declare_use_stamped_vel_arg = DeclareLaunchArgument(
        'use_stamped_vel', default_value='false',
        description='Flag of use stamped twist'
    )

    # Path to config file
    config_file = PathJoinSubstitution([
        FindPackageShare('whi_rc_bridge'),
        'config',
        'config.yaml'
    ])

    # Node definition
    start_whi_rc_bridge_node = Node(
        package='whi_rc_bridge',
        executable='whi_rc_bridge_node',
        name='whi_rc_bridge',
        namespace=namespace,
        output='screen',
        parameters=[
            config_file,
            {'use_stamped_vel': LaunchConfiguration('use_stamped_vel')} # do not define in yaml if it is dynamic through argument
        ]
    )

    return LaunchDescription([
        declare_namespace_arg,
        declare_use_stamped_vel_arg,
        start_whi_rc_bridge_node
    ])
