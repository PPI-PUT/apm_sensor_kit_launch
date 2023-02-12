# Copyright 2023 Perception for Physical Interaction Laboratory at Poznan University of Technology
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
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import ComposableNodeContainer, Node
from launch_ros.descriptions import ComposableNode
from launch_ros.substitutions import FindPackageShare


def launch_setup(context, *args, **kwargs):
    pkg_prefix = FindPackageShare(LaunchConfiguration('param_file_pkg'))
    gnss_config = PathJoinSubstitution([pkg_prefix, LaunchConfiguration('gnss_param_file')])
    ntrip_config = PathJoinSubstitution([pkg_prefix, LaunchConfiguration('ntrip_param_file')])

    container = ComposableNodeContainer(
        name='ublox_gps_container',
        namespace='',
        package='rclcpp_components',
        executable='component_container',
        composable_node_descriptions=[
                ComposableNode(
                    package='ublox_gps',
                    namespace='ublox',
                    plugin='ublox_node::UbloxNode',
                    name='ublox_gps_node',
                    parameters=[
                        gnss_config
                    ]),
        ],
        output='both',
    )

    ntrip_node = Node(
        name='ntrip_client_node',
        namespace='',
        package='ntrip_client',
        executable='ntrip_ros.py',
        parameters=[
            {
                ntrip_config
            }
        ],
    )

    return [
        container,
        ntrip_node
    ]


def generate_launch_description():
    declared_arguments = []

    declared_arguments.append(
        DeclareLaunchArgument(
            'param_file_pkg',
            default_value='apm_sensor_kit_launch',
            description="Package name which contains param file."
        )
    )

    declared_arguments.append(
        DeclareLaunchArgument(
            'gnss_param_file',
            default_value='config/gnss/gnss.param.yaml',
            description="Param file (relative path)."
        )
    )

    declared_arguments.append(
        DeclareLaunchArgument(
            'ntrip_param_file',
            default_value='config/gnss/ntrip.param.yaml',
            description="Param file (relative path)."
        )
    )

    return LaunchDescription([
        *declared_arguments,
        OpaqueFunction(function=launch_setup)
    ])
