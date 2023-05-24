# Copyright (c) 2023 PAL Robotics S.L. All rights reserved.
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


def generate_launch_description():

    marker_id_arg = DeclareLaunchArgument(
        'marker_id', default_value='584',
        description='Marker ID. '
    )

    marker_size_arg = DeclareLaunchArgument(
        'marker_size', default_value='0.10',
        description='Marker size in m. '
    )

    aruco_single_params = {
        'image_is_rectified': False,
        'marker_size': LaunchConfiguration('marker_size'),
        'marker_id': LaunchConfiguration('marker_id'),
        'reference_frame': 'base_footprint',
        'camera_frame': 'head_front_camera_rgb_optical_frame',
        'marker_frame': 'aruco_frame',
        'corner_refinement': 'SUBPIX',
    }

    aruco_single = Node(
        package='aruco_ros',
        executable='single',
        parameters=[aruco_single_params],
        remappings=[('/camera_info', '/head_front_camera/rgb/camera_info'),
                    ('/image', '/head_front_camera/rgb/image_raw')],
        name="aruco_single",
    )

    look_to_aruco = Node(
        package='look_to_aruco_demo',
        executable='look_to_aruco',
    )

    head_action_params = {
        'pan_link': 'head_1_link',
        'default_pointing_frame': 'head_2_link',
        'success_angle_threshold': 0.01,
    }

    head_action = Node(
            package='head_action',
            executable='head_action',
            namespace="head_controller",
            output="both",
            parameters=[head_action_params],)

    # Create the launch description and populate
    ld = LaunchDescription()

    ld.add_action(marker_id_arg)
    ld.add_action(marker_size_arg)

    ld.add_action(aruco_single)

    ld.add_action(head_action)

    ld.add_action(look_to_aruco)

    return ld
