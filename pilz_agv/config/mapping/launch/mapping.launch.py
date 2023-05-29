import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    use_sim_time = LaunchConfiguration('use_sim_time', default='false')

    param_dir = LaunchConfiguration('parameters', default=os.path.join(
        get_package_share_directory('pilz_agv'),'config/mapping/mapping.yaml'))
    
    rviz_config_file_path = os.path.join(
        get_package_share_directory('pilz_agv'), 'config/rviz/mapping_default_view.rviz')

    return LaunchDescription([
        DeclareLaunchArgument(
            'parameters',
            default_value=param_dir,
            description='Full path to param file to load'),

        Node(
            package='slam_toolbox', 
            executable='sync_slam_toolbox_node', 
            output='screen',
            name='slam_toolbox', 
            parameters = [param_dir]),

        Node(
            package='rviz2',
            executable='rviz2',
            output='screen',
            name='rviz2_node',
            parameters=[{'use_sim_time': use_sim_time}],
            arguments=['-d', rviz_config_file_path]
        ),
    ])
