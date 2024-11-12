# SPDX-License-Identifier: Apache-2.0

from launch import LaunchDescription
from launch_ros.actions import ComposableNodeContainer
from launch_ros.descriptions import ComposableNode


def generate_launch_description():
    # Define composable nodes directly here
    composable_nodes = [
        ComposableNode(
            package='isaac_ros_image_proc',
            plugin='nvidia::isaac_ros::image_proc::ImageFormatConverterNode',
            name='image_format_converter_node_left',
            parameters=[
                {'encoding_desired': 'rgb8'},
                {'image_width': 960},
                {'image_height': 600}
            ],
            remappings=[
                ('image_raw', 'zed_node/left/image_rect_color'),
                ('image', 'left/image_rect')
            ]
        ),
        ComposableNode(
            package='isaac_ros_image_proc',
            plugin='nvidia::isaac_ros::image_proc::ImageFormatConverterNode',
            name='image_format_converter_node_right',
            parameters=[
                {'encoding_desired': 'rgb8'},
                {'image_width': 960},
                {'image_height': 600}
            ],
            remappings=[
                ('image_raw', 'zed_node/right/image_rect_color'),
                ('image', 'right/image_rect')
            ]
        ),
        ComposableNode(
            package='isaac_ros_visual_slam',
            plugin='nvidia::isaac_ros::visual_slam::VisualSlamNode',
            name='visual_slam_node',
            parameters=[
                {'enable_image_denoising': False},
                {'rectified_images': True},
                {'enable_slam_visualization': True},
                {'enable_landmarks_view': True},
                {'enable_observations_view': False},
                {'camera_optical_frames': ['zedx_left_camera_optical_frame', 'zedx_right_camera_optical_frame']},
                {'base_frame': 'zedx_camera_center'},
                {'num_cameras': 2},
                {'min_num_images': 2},
                {'enable_imu_fusion': True},
                {'use_sim_time': False},
                {'imu_frame': 'zedx_imu_link'},
                {'gyro_noise_density': 0.000244},
                {'gyro_random_walk': 0.000019393},
                {'accel_noise_density': 0.001862},
                {'accel_random_walk': 0.003},
                {'calibration_frequency': 200.0},
                {'image_jitter_threshold_ms': 50.0},
                {'imu_jitter_threshold_ms': 5.0},
                {'sync_matching_threshold_ms': 30.0},
                {'image_buffer_size': 5},
                {'imu_buffer_size': 100},
                {'enable_localization_n_mapping': True},
                {'enable_planar_mode': False},
            ],
            remappings=[
                ('/visual_slam/image_0', 'left/image_rect'),
                ('/visual_slam/camera_info_0', 'left/camera_info_rect'),
                ('/visual_slam/image_1', 'right/image_rect'),
                ('/visual_slam/camera_info_1', 'right/camera_info_rect'),
                ('visual_slam/imu', 'zed_node/imu/data')
            ]
        )
    ]

    # Create a container for all composable nodes
    container = ComposableNodeContainer(
        name='isaac_ros_container',
        namespace='isaac_ros_examples',
        package='rclcpp_components',
        executable='component_container_mt',
        composable_node_descriptions=composable_nodes,
        output='screen'
    )

    # Launch description to start the container
    return LaunchDescription([container])
