# Copyright 2020 Tier IV, Inc. All rights reserved.
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

import yaml

import launch
from launch.actions import DeclareLaunchArgument
from launch.actions import OpaqueFunction
from launch.actions import SetLaunchConfiguration
from launch.conditions import IfCondition
from launch.conditions import UnlessCondition
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import ComposableNodeContainer
from launch_ros.actions import LoadComposableNodes
from launch_ros.actions import Node
from launch_ros.descriptions import ComposableNode
from launch_ros.substitutions import FindPackageShare


def get_vehicle_info(vehicle_params):
    crop_params = {}
    crop_params["vehicle_length"] = vehicle_params["front_overhang"] + \
        vehicle_params["wheel_base"] + vehicle_params["rear_overhang"]
    crop_params["vehicle_width"] = vehicle_params["wheel_tread"] + \
        vehicle_params["left_overhang"] + vehicle_params["right_overhang"]
    crop_params["min_longitudinal_offset"] = -vehicle_params["rear_overhang"]
    crop_params["max_longitudinal_offset"] = vehicle_params["front_overhang"] + vehicle_params["wheel_base"]
    crop_params["min_lateral_offset"] = -(vehicle_params["wheel_tread"] / 2.0 +
                                          vehicle_params["right_overhang"])
    crop_params["max_lateral_offset"] = vehicle_params["wheel_tread"] / 2.0 + vehicle_params["left_overhang"]
    crop_params["min_height_offset"] = 0.3
    crop_params["max_height_offset"] = vehicle_params["vehicle_height"]
    return crop_params


def get_config(context, param):
    path = LaunchConfiguration(param).perform(context)
    with open(path, "r") as f:
        p = yaml.safe_load(f)["/**"]["ros__parameters"]
    return p


def launch_setup(context, *args, **kwargs):
    def create_parameter_dict(*args):
        result = {}
        for x in args:
            result[x] = LaunchConfiguration(x)
        return result

    lidar_params = get_config(context, 'lidar_params')
    vehicle_params = get_config(context, 'vehicle_params')
    vehicle_info = get_vehicle_info(vehicle_params)

    cropbox_parameters = create_parameter_dict("input_frame", "output_frame")
    cropbox_parameters["negative"] = True
    cropbox_parameters["min_x"] = vehicle_info["min_longitudinal_offset"]
    cropbox_parameters["max_x"] = vehicle_info["max_longitudinal_offset"]
    cropbox_parameters["min_y"] = vehicle_info["min_lateral_offset"]
    cropbox_parameters["max_y"] = vehicle_info["max_lateral_offset"]
    cropbox_parameters["min_z"] = vehicle_info["min_height_offset"]
    cropbox_parameters["max_z"] = vehicle_info["max_height_offset"]

    crop_box_vehicle_component = ComposableNode(
        package="pointcloud_preprocessor",
        plugin="pointcloud_preprocessor::CropBoxFilterComponent",
        name="crop_box_filter_self",
        namespace="lidar",
        remappings=[
                ("input", "points"),
                # ("output", "concatenated/pointcloud")
                ("output", "self_cropped/pointcloud_ex")
        ],
        parameters=[cropbox_parameters],
        extra_arguments=[{"use_intra_process_comms": LaunchConfiguration("use_intra_process")}],
    )

    mirror_info = get_config(context, 'vehicle_mirror_param_file')
    cropbox_parameters["min_x"] = mirror_info["min_longitudinal_offset"]
    cropbox_parameters["max_x"] = mirror_info["max_longitudinal_offset"]
    cropbox_parameters["min_y"] = mirror_info["min_lateral_offset"]
    cropbox_parameters["max_y"] = mirror_info["max_lateral_offset"]
    cropbox_parameters["min_z"] = mirror_info["min_height_offset"]
    cropbox_parameters["max_z"] = mirror_info["max_height_offset"]

    crop_box_mirror_component = ComposableNode(
        package="pointcloud_preprocessor",
        plugin="pointcloud_preprocessor::CropBoxFilterComponent",
        name="crop_box_filter_mirror",
        namespace="lidar",
        remappings=[
                ("input", "self_cropped/pointcloud_ex"),
                ("output", "mirror_cropped/pointcloud_ex"),
        ],
        parameters=[cropbox_parameters],
        extra_arguments=[{"use_intra_process_comms": LaunchConfiguration("use_intra_process")}],
    )

    distortion_corrector_component = ComposableNode(
        package="pointcloud_preprocessor",
        plugin="pointcloud_preprocessor::DistortionCorrectorComponent",
        name="distortion_corrector_node",
        namespace="lidar",
        remappings=[
            ("~/input/twist", "/sensing/vehicle_velocity_converter/twist_with_covariance"),
            ("~/input/imu", "/sensing/imu/imu_data"),
            ("~/input/pointcloud", "mirror_cropped/pointcloud_ex"),
            ("~/output/pointcloud", "rectified/pointcloud_ex"),
        ],
        extra_arguments=[{"use_intra_process_comms": LaunchConfiguration("use_intra_process")}],
    )

    ring_outlier_filter_component = ComposableNode(
        package="pointcloud_preprocessor",
        plugin="pointcloud_preprocessor::RingOutlierFilterComponent",
        name="ring_outlier_filter",
        namespace="lidar",
        remappings=[
                ("input", "rectified/pointcloud_ex"),
                ("output", "concatenated/pointcloud"),
        ],
        extra_arguments=[{"use_intra_process_comms": LaunchConfiguration("use_intra_process")}],
    )

    driver_component = ComposableNode(
        package="ros2_ouster",
        plugin="ros2_ouster::Driver",
        name="ouster_driver",
        namespace="lidar",
        parameters=[lidar_params],
        extra_arguments=[{"use_intra_process_comms": LaunchConfiguration("use_intra_process")}],
    )

    container = ComposableNodeContainer(
        name=LaunchConfiguration("pointcloud_container_name"),
        namespace="",
        package="rclcpp_components",
        executable=LaunchConfiguration("container_executable"),
        composable_node_descriptions=[],
        condition=UnlessCondition(LaunchConfiguration("use_pointcloud_container")),
        output="screen",
    )

    target_container = (
        container
        if UnlessCondition(LaunchConfiguration("use_pointcloud_container")).evaluate(context)
        else LaunchConfiguration("pointcloud_container_name")
    )

    loader = LoadComposableNodes(
        composable_node_descriptions=[driver_component,
                                      crop_box_vehicle_component,
                                      crop_box_mirror_component,
                                      distortion_corrector_component,
                                      ring_outlier_filter_component
                                      ],
        target_container=target_container,
        condition=IfCondition(LaunchConfiguration("launch_driver")),
    )

    ouster_script = Node(
        name='init_ouster',
        package='apm_sensor_kit_launch',
        executable='init_ouster.sh',
    )

    return [container, loader, ouster_script]


def generate_launch_description():
    sensor_kit_dir = FindPackageShare("apm_sensor_kit_launch")
    vehicle_dir = FindPackageShare("apm_vehicle_description")

    launch_arguments = []

    def add_launch_arg(name: str, default_value=None):
        launch_arguments.append(DeclareLaunchArgument(name, default_value=default_value))

    add_launch_arg("launch_driver", "True")
    add_launch_arg("vehicle_mirror_param_file", default_value=PathJoinSubstitution(
        [vehicle_dir, 'config/mirror.param.yaml']))
    add_launch_arg("use_pointcloud_container", "False")
    add_launch_arg("pointcloud_container_name", "pointcloud_container")

    add_launch_arg("base_frame", "base_link")
    add_launch_arg("use_multithread", "False")
    add_launch_arg("use_intra_process", "False")
    add_launch_arg("vehicle_id", "apm_vehicle")
    add_launch_arg("lidar_params", default_value=PathJoinSubstitution(
        [sensor_kit_dir, 'config/lidar/lidar.param.yaml']))
    add_launch_arg("vehicle_params", default_value=PathJoinSubstitution(
        [vehicle_dir, 'config/vehicle_info.param.yaml']))
    add_launch_arg("input_frame", "lidar_laser_link")
    add_launch_arg("output_frame", LaunchConfiguration("base_frame"))

    set_container_executable = SetLaunchConfiguration(
        "container_executable",
        "component_container",
        condition=UnlessCondition(LaunchConfiguration("use_multithread")),
    )

    set_container_mt_executable = SetLaunchConfiguration(
        "container_executable",
        "component_container_mt",
        condition=IfCondition(LaunchConfiguration("use_multithread")),
    )

    return launch.LaunchDescription(
        launch_arguments
        + [set_container_executable, set_container_mt_executable]
        + [OpaqueFunction(function=launch_setup)]
    )
