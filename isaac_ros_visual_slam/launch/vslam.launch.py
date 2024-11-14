# SPDX-License-Identifier: Apache-2.0

from launch import LaunchDescription
from launch_ros.actions import ComposableNodeContainer
from launch_ros.descriptions import ComposableNode
from launch_ros.actions import Node


def generate_launch_description():
    # Define composable nodes directly here
    composable_nodes = [
        # ComposableNode(
        #     package="tf2_ros",
        #     executable="static_transform_publisher",
        #     output="screen",
        #     arguments=["0", "0", "1", "0", "0", "0", "odom", "sw_base_link"]
        # ),
        # ComposableNode(
        #     package="tf2_ros",
        #     executable="static_transform_publisher",
        #     output="screen",
        #     arguments=["0", "0.387", "-0.201", "0", "0", "0", "sw_base_link", "zedx_camera_center"]
        # ),
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
                {'base_frame': 'sw_base_link'},
                {'num_cameras': 2},
                {'min_num_images': 2},
                {'enable_imu_fusion': True},
                {'use_sim_time': False},
                {'imu_frame': 'zedx_imu_link'},
                {'gyro_noise_density': 0.004720999859273434},
                {'gyro_random_walk': 0.003800000064074993},
                {'accel_noise_density': 0.0013500000350177288},
                {'accel_random_walk': 0.00419999985024333},
                {'calibration_frequency': 200.0},
                {'image_jitter_threshold_ms': 34.0},
                {'imu_jitter_threshold_ms': 5.0},
                {'sync_matching_threshold_ms': 10.0},
                {'image_buffer_size': 5},
                {'imu_buffer_size': 100},
                {'enable_localization_n_mapping': True},
                {'enable_planar_mode': True},
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
    
    # static_tf = ComposableNode(package = "tf2_ros", 
    #                    executable = "static_transform_publisher",
    #                    arguments = ["0 0 0 0 0 0 odom laser"])
    
    # static_tf_node = ComposableNode(
    #         package="tf2_ros",
    #         executable="static_transform_publisher",
    #         output="screen" ,
    #         arguments=["0", "0", "0", "0", "0", "0", "odom", "laser"]
    # )

    # Create a container for all composable nodes
    container = ComposableNodeContainer(
        name='isaac_ros_container',
        namespace='isaac_ros_examples',
        package='rclcpp_components',
        executable='component_container_mt',
        composable_node_descriptions=composable_nodes,
        output='screen'
    )
    
    static_tf_node = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        output="screen",
        arguments=["0.387", "0.0", "0.201", "0", "0", "0", "sw_base_link", "zedx_camera_link"]
    )

    # Launch description to start the container
    # return LaunchDescription([container])
    return LaunchDescription([
        static_tf_node, 
        container
    ])