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
        get_package_share_directory('pcl_proc'),
        'config',
        'wp_admin.yaml'
    )
    
    node = Node(
        package='pcl_proc',
        executable='wp_admin.py',
        name='waypoint_admin',
        namespace="alpha_rise",
        output='screen',
        parameters=[param_config]
    )

    ld.add_action(node)

    return ld