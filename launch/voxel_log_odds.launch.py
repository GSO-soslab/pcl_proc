import os
import yaml
import pathlib
from launch import LaunchDescription
import launch.actions
from ament_index_python.packages import get_package_share_directory
from launch_ros.actions import Node
from launch.substitutions import EnvironmentVariable
from launch.actions import DeclareLaunchArgument

def generate_launch_description():

    ld = LaunchDescription()

    param_config = os.path.join(
        get_package_share_directory('iceberg_nav'),
        'config',
        'voxel_log_odds.yaml'
    )
    
    node = Node(
        package='iceberg_nav',
        executable='voxel_log_odds_visualizer',
        name='voxel_log_odds_visualizer',
        namespace="alpha_rise",
        output='screen',
        parameters=[param_config]
    )

    ld.add_action(node)

    return ld