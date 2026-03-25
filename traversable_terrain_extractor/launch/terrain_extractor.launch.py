import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    pkg_dir = get_package_share_directory('traversable_terrain_extractor')
    config_file = os.path.join(pkg_dir, 'config', 'terrain_params.yaml')

    return LaunchDescription([
        DeclareLaunchArgument(
            'params_file',
            default_value=config_file,
            description='Full path to the parameter YAML file'
        ),
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='false',
            description='Use simulation (bag) clock'
        ),

        Node(
            package='traversable_terrain_extractor',
            executable='terrain_extractor_node',
            name='terrain_extractor',
            output='screen',
            parameters=[
                LaunchConfiguration('params_file'),
                {'use_sim_time': LaunchConfiguration('use_sim_time')}
            ],
            remappings=[
                # Remap if your LIO-SAM topics differ
                # ('/lio_sam/mapping/cloud_registered', '/your_topic'),
            ]
        ),
    ])
