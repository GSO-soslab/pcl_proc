import os
from launch import LaunchDescription
from ament_index_python.packages import get_package_share_directory
from launch_ros.actions import Node


def generate_launch_description():

    ld = LaunchDescription()

    param_config = os.path.join(
        get_package_share_directory('iceberg_nav'),
        'config',
        'msis_voxels.yaml'
    )

    msis_voxels_node = Node(
        package='iceberg_nav',
        executable='msis_voxels',
        name='msis_voxel_node',
        namespace='alpha_rise',
        output='screen',
        parameters=[param_config]
    )

    msis_prob_clouds_node = Node(
        package='iceberg_nav',
        executable='msis_prob_clouds.py',
        name='msis_prob_clouds',
        namespace='alpha_rise',
        output='screen',
        parameters=[param_config]
    )

    ld.add_action(msis_voxels_node)
    ld.add_action(msis_prob_clouds_node)

    return ld
