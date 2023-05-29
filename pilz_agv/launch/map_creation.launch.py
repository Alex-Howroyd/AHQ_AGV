import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource


def generate_launch_description():
    use_sim_time = LaunchConfiguration('use_sim_time', default='false')

    urdf = os.path.join(
        get_package_share_directory('pilz_agv'),'robot/pilz_agv.urdf.xml')

    mapping_launch_file = os.path.join(
        get_package_share_directory('pilz_agv'), 'config/mapping/launch')
    
    return LaunchDescription([
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            output='screen',
            parameters=[{'use_sim_time': use_sim_time, 'rate': 200.0}],
            arguments=[urdf]
        ),

        Node(
            package='joint_state_publisher',
            executable='joint_state_publisher',
            name='joint_state_publisher',
            parameters=[{'source_list': ['fixed_wheel_right_joint','fixed_wheel_left_joint'],'use_sim_time': use_sim_time, 'publish_rate': 200.0}],
            output='screen'
        ),

        Node(
            package='teleop_twist_keyboard',
            executable="teleop_twist_keyboard",
            output='screen',
            prefix = 'xterm -e',
            name='teleop'
        ),

        Node(
            package="pilz_agv",
            executable="wheels_vel.py",
            name='wheels_vel',
        ),

        Node(
            package="pilz_agv",
            executable="odom.py",
            name='odom'
        ),

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([mapping_launch_file, '/mapping.launch.py']))
        
    ])
