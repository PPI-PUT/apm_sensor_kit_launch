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
from launch.actions import DeclareLaunchArgument, OpaqueFunction, IncludeLaunchDescription
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch.launch_description_sources import PythonLaunchDescriptionSource, FrontendLaunchDescriptionSource
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def launch_setup(context, *args, **kwargs):
    urdf_pkg_prefix = FindPackageShare(LaunchConfiguration('urdf_pkg'))
    # urdf_config = PathJoinSubstitution([urdf_pkg_prefix, 'urdf/sensors.xacro'])

    # Lidar
    lidar_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            launch_file_path=PathJoinSubstitution([
                FindPackageShare('apm_sensor_kit_launch'), 'launch', 'lidar.launch.py'
            ]),
        )
    )

    # IMU
    imu_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            launch_file_path=PathJoinSubstitution([
                FindPackageShare('apm_sensor_kit_launch'), 'launch', 'imu.launch.py'
            ]),
        )
    )

    # GNSS
    gnss_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            launch_file_path=PathJoinSubstitution([
                FindPackageShare('apm_sensor_kit_launch'), 'launch', 'gnss.launch.py'
            ]),
        )
    )
    
    # Vehicle Velocity Converter
    vehicle_velocity_converter_launch = IncludeLaunchDescription(
        FrontendLaunchDescriptionSource(
            launch_file_path=PathJoinSubstitution([
                FindPackageShare('vehicle_velocity_converter'), 'launch', 'vehicle_velocity_converter.launch.xml'
            ]),
        ),
        launch_arguments={
            'input_vehicle_velocity_topic': "/vehicle/status/velocity_status",
            'output_twist_with_covariance': "/sensing/vehicle_velocity_converter/twist_with_covariance",
        }.items()
    )
    
    # Remote controller
    remote_controller_launch = IncludeLaunchDescription(
        FrontendLaunchDescriptionSource(
            launch_file_path=PathJoinSubstitution([
                FindPackageShare('joy_controller'), 'launch', 'joy_controller.launch.xml'
            ]),
        ),
        launch_arguments={
            'config_file': PathJoinSubstitution([
                FindPackageShare('apm_sensor_kit_launch'), 'config', 'joy/joy_controller.param.yaml'
            ]),
            'input_joy': "/sensing/joy"
        }.items()
    )

    # # do not use, just for tf validation
    # state_publisher_node = Node(
    #     package='robot_state_publisher',
    #     executable='robot_state_publisher',
    #     name='sensors_state_publisher',
    #     namespace='sensors',
    #     output='screen',
    #     parameters=[{
    #         'use_sim_time': True,
    #         'robot_description': Command(['xacro', ' ', urdf_config])
    #     }]
    # )
    
    return [
        lidar_launch,
        imu_launch,
        gnss_launch,
        vehicle_velocity_converter_launch,
        remote_controller_launch,
        # state_publisher_node
    ]


def generate_launch_description():
    declared_arguments = []

    declared_arguments.append(
        DeclareLaunchArgument(
            'urdf_pkg',
            default_value='apm_sensor_kit_description',
            description="Package name which contains urdf file."
        )
    )

    return LaunchDescription([
        *declared_arguments,
        OpaqueFunction(function=launch_setup)
    ])
