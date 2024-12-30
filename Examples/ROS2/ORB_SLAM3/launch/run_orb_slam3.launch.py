from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from ament_index_python.packages import get_package_share_directory
import os
import yaml

def generate_launch_description():
    # Get the package share directory
    package_share_dir = get_package_share_directory('orb_slam3_ros2')
    
    # Path to your config file
    config_file = DeclareLaunchArgument(
        'config_file',
        default_value = os.path.join(package_share_dir, "config", "Zed2.yaml"),
        description= "path to ros2 interface setting file"
    )
    # Declare launch arguments
    vocabulary_path = DeclareLaunchArgument(
        'vocabulary_path',
        default_value=os.path.join(package_share_dir, 'vocabulary', 'ORBvoc.txt'),
        description='Path to ORB vocabulary file'
    )
    
    settings_path = DeclareLaunchArgument(
        'settings_path',
        default_value=os.path.join(package_share_dir, 'config', 'settings.yaml'),
        description='Path to settings file'
    )
    
    # Add boolean arguments
    enable_rectify = DeclareLaunchArgument(
        'enable_rectify',
        default_value='false',
        description='Enable rectification'
    )
    
    enable_equalize = DeclareLaunchArgument(
        'enable_equalize',
        default_value='false',  # String 'true' or 'false'
        description='Enable visualization'
    )

    # Create node with parameters
    slam_node = Node(
        package='orb_slam3_ros2',
        executable='stereo_inertial',
        name='slam_node',
        output='screen',
        parameters=[{
            'config_file': LaunchConfiguration('config_file'),
        }],
        arguments=[
            LaunchConfiguration('vocabulary_path'),
            LaunchConfiguration('settings_path'),
            LaunchConfiguration('enable_rectify'),
            LaunchConfiguration('enable_equalize')
        ]
    )

    # Return launch description with all components
    return LaunchDescription([
        config_file,
        vocabulary_path,
        settings_path,
        enable_rectify,
        enable_equalize,
        slam_node
    ])