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
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def launch_setup(context, *args, **kwargs):
    pkg_prefix = FindPackageShare(LaunchConfiguration('param_file_pkg'))
    imu_config = PathJoinSubstitution([pkg_prefix, LaunchConfiguration('imu_param_file')])
    imu_corrector_config = PathJoinSubstitution([pkg_prefix, LaunchConfiguration('imu_corrector_param_file')])

    xsens_mti_node = Node(
        name='xsens_mti_node',
        namespace='',
        package='bluespace_ai_xsens_mti_driver',
        executable='xsens_mti_node',
        parameters=[
                imu_config
        ],
        # remappings=[
        #         ("imu/data", "imu/imu_data")
        # ],
        output='screen',
        arguments=['--ros-args', '--log-level', 'info', '--enable-stdout-logs'],
        emulate_tty=True
    )
    
    imu_corrector_node = Node(
        name='imu_corrector',
        namespace='',
        package='imu_corrector',
        executable='imu_corrector',
        parameters=[
                imu_corrector_config
        ],
        remappings=[
                ("input", "imu/data"),
                ("output", "imu/imu_data")
        ],
        output='screen',
        arguments=['--ros-args', '--log-level', 'info', '--enable-stdout-logs'],
        emulate_tty=True
    )

    return [
        xsens_mti_node,
        imu_corrector_node
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
            'imu_param_file',
            default_value='config/imu/imu.param.yaml',
            description="Param file (relative path)."
        )
    )

    declared_arguments.append(
        DeclareLaunchArgument(
            'imu_corrector_param_file',
            default_value='config/imu/imu_corrector.param.yaml',
            description="Param file (relative path)."
        )
    )

    return LaunchDescription([
        *declared_arguments,
        OpaqueFunction(function=launch_setup)
    ])
