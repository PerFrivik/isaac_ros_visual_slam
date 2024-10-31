# SPDX-License-Identifier: Apache-2.0

import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import Command
from launch_ros.actions import Node
from ament_index_python import get_package_share_directory


def generate_launch_description():
    # Define paths to configuration files
    zed_wrapper_path = get_package_share_directory('zed_wrapper')
    isaac_ros_zed_path = get_package_share_directory('isaac_ros_zed')
    
    xacro_path = os.path.join(zed_wrapper_path, 'urdf', 'zed_descr.urdf.xacro')
    config_common = os.path.join(isaac_ros_zed_path, 'config', 'zed.yaml')
    config_camera = os.path.join(zed_wrapper_path, 'config', 'zedx.yaml')
    config_ffmpeg = os.path.join(zed_wrapper_path, 'config', 'ffmpeg.yaml')
    
    # Define ZED camera model
    camera_model = 'zedx'

    # Robot State Publisher node for the camera URDF
    rsp_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='zed_state_publisher',
        output='screen',
        parameters=[{
            'robot_description': Command(
                [
                    'xacro', ' ', xacro_path, ' ',
                    'camera_name:=', camera_model, ' ',
                    'camera_model:=', camera_model
                ])
        }]
    )

    # ZED Wrapper node for managing the ZED camera
    zed_node = Node(
        package='zed_wrapper',
        executable='zed_wrapper',
        output='screen',
        parameters=[
            config_common,  # Common parameters
            config_camera,  # Camera-specific parameters
            config_ffmpeg,  # FFMPEG parameters
            {
                'pos_tracking.publish_tf': True,
                'pos_tracking.publish_map_tf': True,
                'sensors.publish_imu_tf': True,
                'use_sim_time': False,  # Set to true if running in simulation
                'general.camera_name': camera_model,
                'general.camera_model': camera_model,
            }
        ],
        arguments=[
            '--ros-args',
            '--remap', 'zed_node/left/camera_info:=/left/camera_info_rect',
            '--remap', 'zed_node/right/camera_info:=/right/camera_info_rect',
        ]
    )

    return LaunchDescription([
        rsp_node,
        zed_node
    ])
