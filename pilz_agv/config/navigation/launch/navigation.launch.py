import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():

    use_sim_time = LaunchConfiguration('use_sim_time', default='false')

    controller_yaml = os.path.join(
        get_package_share_directory('pilz_agv'), 'config/navigation/controller.yaml')
    
    default_bt_xml_path = os.path.join(
        get_package_share_directory('pilz_agv'), 'config/navigation/behavior.xml')

    planner_yaml = os.path.join(
        get_package_share_directory('pilz_agv'), 'config/navigation/planner_server.yaml')
    
    recovery_yaml = os.path.join(
        get_package_share_directory('pilz_agv'), 'config/navigation/recovery.yaml')
    
    bt_navigator_yaml = os.path.join(
        get_package_share_directory('pilz_agv'), 'config/navigation/bt_navigator.yaml')
    
    waypoint_follower = os.path.join(
        get_package_share_directory('pilz_agv'), 'config/navigation/waypoint_follower.yaml')
    
    nav2_yaml = os.path.join(
        get_package_share_directory('pilz_agv'), 'config/amcl/amcl_config.yaml')
    
    map_file = os.path.join(
        get_package_share_directory('pilz_agv'), 'maps', 'my_map_3.yaml')
    
    rviz_config_file_path = os.path.join(
        get_package_share_directory('pilz_agv'), 'config/rviz/nav2_default_view.rviz')

    remappings = [('/tf', 'tf'),
                  ('/tf_static', 'tf_static')]

    return LaunchDescription([
        Node(
            package='nav2_map_server',
            executable='map_server',
            name='map_server',
            output='screen',
            parameters=[{'use_sim_time': use_sim_time},
                        {'yaml_filename': map_file}],
            remappings=remappings
        ),

        Node(
            package='nav2_amcl',
            executable='amcl',
            name='amcl',
            output='screen',
            parameters=[nav2_yaml],
            remappings=remappings
        ),

        Node(
            package='nav2_controller',
            executable='controller_server',
            name='controller_server',
            output='screen',
            parameters=[controller_yaml],
            remappings=remappings
        ),
            
        Node(
            package='nav2_planner',
            executable='planner_server',
            name='planner_server',
            output='screen',
            parameters=[planner_yaml],
            remappings=remappings    
        ),

        Node(
            package='nav2_recoveries',
            executable='recoveries_server',
            name='recoveries_server',
            output='screen',
            parameters=[recovery_yaml],
            remappings=remappings    
        ),

        Node(
            package='nav2_bt_navigator',
            executable='bt_navigator',
            name='bt_navigator',
            output='screen',
            parameters=[bt_navigator_yaml, {'default_bt_xml_filename': default_bt_xml_path}],
            remappings=remappings    
        ),
    
        Node(
            package='nav2_waypoint_follower',
            executable='waypoint_follower',
            name='waypoint_follower',
            output='screen',
            parameters=[waypoint_follower ],
            remappings=remappings
        ),

        Node(
            package='rviz2',
            executable='rviz2',
            output='screen',
            name='rviz2_node',
            parameters=[{'use_sim_time': use_sim_time}],
            arguments=['-d', rviz_config_file_path]
        ),

        Node(
            package='nav2_lifecycle_manager',
            executable='lifecycle_manager',
            name='lifecycle_manager',
            output='screen',
            parameters=[{'autostart': True},
                        {'node_names': ['map_server',
                                        'amcl',
                                        'controller_server',
                                        'planner_server',
                                        'recoveries_server',
                                        'bt_navigator']}]
        ),

        Node(
            package="tf2_ros",
            executable="static_transform_publisher",
            output="screen" ,
            arguments=["0", "0", "0", "0", "0", "0", "map", "odom"]
        ),

        Node(
            package="tf2_ros",
            executable="static_transform_publisher",
            output="screen" ,
            arguments=["0", "0", "0", "0", "0", "0", "odom", "base_footprint"]
        )
    ])
