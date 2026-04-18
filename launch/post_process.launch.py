import os
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, ExecuteProcess
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory
from launch_ros.actions import Node  # used by rviz

def generate_launch_description():

    ld = LaunchDescription()

    inv_models = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory('alpha_rise_bringup'),
                'launch',
                'bringup_inv_models.launch.py'
            )
        ),
        launch_arguments={'use_sim_time': 'true'}.items()
    )


    path = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([os.path.join(get_package_share_directory('alpha_rise_bringup'), 'launch','bringup_path.launch.py')]),
        launch_arguments={
            'robot_name': 'alpha_rise',
            'description_delay': '0.0',
            'use_sim_time': 'true'
        }.items()   
    )

    # Vehicle description
    description = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            os.path.join(get_package_share_directory('alpha_rise_bringup'), 
            'launch/include/description.launch.py')]),
        launch_arguments={
            'robot_name': 'alpha_rise',
            'description_delay': '0.0',
            'use_sim_time': 'true'
        }.items()  
    )

    octomap = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory('fls_ism'),
                'launch',
                'octomap_mapping.launch.py'
            )
        ),
        launch_arguments={'use_sim_time': 'true'}.items()
    )

    rviz_config_dir = os.path.join( get_package_share_directory('alpha_rise_description'), 'rviz', 'config_post.rviz' )

    rviz = Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            arguments=['-d', [rviz_config_dir]],
            parameters=[{'use_sim_time': True}],
            additional_env={
            "LD_PRELOAD": "/usr/lib/x86_64-linux-gnu/liboctomap.so",
    },
        )
    
        #Foxglove Bridge
    foxglove = Node(
            package='foxglove_bridge',
            executable='foxglove_bridge',
            name='foxglove_bridge',
            output='screen')

        # Bag file path (change this to the full path or make it configurable)
    # bag_file_path = '/home/tony/bags/whale_rock_10_10/rosbag2_2025_10_10-18_09_27/rosbag2_2025_10_10-18_09_28'
    # bag_file_path = '/home/tony/bags/whale_rock_10_10/rosbag2_2025_10_10-17_46_01/rosbag2_2025_10_10-17_46_03/'
    # bag_file_path = '/home/tony/bags/whale_rock_10_10/rosbag2_2025_10_10-17_25_13/rosbag2_2025_10_10-17_25_14/'


    ## MSIS bags
    # bag_file_path = '/home/tony/auv_ws/bags/msis/rosbag2_2025_11_24-15_44_37/rosbag2_2025_11_24-15_44_38/'

                            # rosbag2_2025_12_05-15_29_38: gain = 2
                            # rosbag2_2025_12_05-15_57_34: gain = 2
                            # rosbag2_2025_12_05-16_09_45: gain = 2
                            # rosbag2_2025_12_05-16_26_08: gain = 2
                            # rosbag2_2025_12_05-16_43_41: gain = 1
                            # rosbag2_2025_12_05-16_54_58: gain = 1
                            # rosbag2_2025_12_05-17_09_41: gain = 2
                            # rosbag2_2025_12_05-17_22_29: gain = 2
                            # rosbag2_2025_12_05-17_35_12: gain = 2
    # bag_file_path = '/home/tony/auv_ws/bags/whale_rock/whale_rock_10_10/rosbag2_2025_10_10-18_09_27/rosbag2_2025_10_10-18_09_28'
    
    #Whale Rock 12/5
    # bag_file_path = '/home/tony/auv_ws/bags/whale_rock/whale_rock_12_05_25/rosbag2_2025_12_05-15_29_36/rosbag2_2025_12_05-15_29_38'
    # bag_file_path = '/home/tony/auv_ws/bags/whale_rock/whale_rock_12_05_25/rosbag2_2025_12_05-15_57_34/rosbag2_2025_12_05-15_57_35'
    # bag_file_path = '/home/tony/auv_ws/bags/whale_rock/whale_rock_12_05_25/rosbag2_2025_12_05-16_09_45/rosbag2_2025_12_05-16_09_47'
    # bag_file_path = '/home/tony/auv_ws/bags/whale_rock/whale_rock_12_05_25/rosbag2_2025_12_05-16_26_08/rosbag2_2025_12_05-16_26_09'
    # bag_file_path = '/home/tony/auv_ws/bags/whale_rock/whale_rock_12_05_25/rosbag2_2025_12_05-16_43_41/rosbag2_2025_12_05-16_43_42'
    # bag_file_path = '/home/tony/auv_ws/bags/whale_rock/whale_rock_12_05_25/rosbag2_2025_12_05-16_54_58/rosbag2_2025_12_05-16_55_00'
    # bag_file_path = '/home/tony/auv_ws/bags/whale_rock/whale_rock_12_05_25/rosbag2_2025_12_05-17_09_41/rosbag2_2025_12_05-17_09_42'
    # bag_file_path = '/home/tony/auv_ws/bags/whale_rock/whale_rock_12_05_25/rosbag2_2025_12_05-17_22_29/rosbag2_2025_12_05-17_22_30'
    # bag_file_path = '/home/tony/auv_ws/bags/whale_rock/whale_rock_12_05_25/rosbag2_2025_12_05-17_35_12/rosbag2_2025_12_05-17_35_13'
    
    # bag_file_path = '/home/tony/auv_ws/bags/mbes_testing_allen_harbor_3_13_26/rosbag2_2026c_03_13-17_21_16/rosbag2_2026_03_13-17_21_16'
    # bag_file_path = '/home/tony/auv_ws/bags/mbes_testing_allen_harbor_3_13_26/rosbag2_2026_03_13-17_34_47/rosbag2_2026_03_13-17_34_48'

    bag_file_path = '/media/tony/Vault/2026_04_09_Whale_rock/rosbag2_2026_04_09-18_31_27/rosbag2_2026_04_09-18_31_28'

    bag_play = ExecuteProcess(
        cmd=[
            'ros2', 'bag', 'play', bag_file_path,
            # '--start-offset', '70.0',
            '--rate', '1.0',
            '--clock',
        ],
        output='screen'
    )

    ld.add_action(bag_play)

    ld.add_action(description)
    ld.add_action(inv_models)
    ld.add_action(rviz)
    ld.add_action(path)
    # ld.add_action(octomap)
    # ld.add_action(foxglove)



    return ld