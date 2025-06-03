from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
        ld = LaunchDescription()
        
        param_config = os.path.join(
        get_package_share_directory('pcl_proc'),    
        'config',
        'costmap_params.yaml'
        )

        node = Node(
            package='nav2_costmap_2d',
            executable='nav2_costmap_2d',
            name='costmap',
            namespace="alpha_rise",
            output='screen',
            parameters=[param_config],
        )
        
    
        ld.add_action(node)
        
        return ld
