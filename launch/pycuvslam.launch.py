from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        # Topic Remapping
        DeclareLaunchArgument('rgb_topic', default_value='/camera/color/image_raw'),
        DeclareLaunchArgument('depth_topic', default_value='/camera/depth/image_raw'),
        DeclareLaunchArgument('camera_info_topic', default_value='/camera/color/camera_info'),
        
        # Frames
        DeclareLaunchArgument('odom_frame', default_value='odom'),
        DeclareLaunchArgument('base_frame', default_value='camera_link'),
        DeclareLaunchArgument('publish_tf', default_value='true'),
        
        # Tuning
        DeclareLaunchArgument('use_gpu', default_value='true'),
        DeclareLaunchArgument('async_sba', default_value='true'),
        DeclareLaunchArgument('use_motion_model', default_value='true'),
        DeclareLaunchArgument('enable_depth_denoising', default_value='true'),
        DeclareLaunchArgument('fast_depth', default_value='true'),
        
        # Visualization
        DeclareLaunchArgument('use_rerun', default_value='false'),
        
        Node(
            package='pycuvslam_ros2',
            executable='node',
            name='pycuvslam_node',
            output='screen',
            parameters=[{
                'rgb_topic': LaunchConfiguration('rgb_topic'),
                'depth_topic': LaunchConfiguration('depth_topic'),
                'camera_info_topic': LaunchConfiguration('camera_info_topic'),
                
                'odom_frame_id': LaunchConfiguration('odom_frame'),
                'base_frame_id': LaunchConfiguration('base_frame'),
                'publish_tf': LaunchConfiguration('publish_tf'),
                
                'use_gpu': LaunchConfiguration('use_gpu'),
                'async_sba': LaunchConfiguration('async_sba'),
                'use_motion_model': LaunchConfiguration('use_motion_model'),
                
                'enable_depth_denoising': LaunchConfiguration('enable_depth_denoising'),
                'fast_depth': LaunchConfiguration('fast_depth'),
                
                'use_rerun': LaunchConfiguration('use_rerun'),
            }]
        )
    ])
