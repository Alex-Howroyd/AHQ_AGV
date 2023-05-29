import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription
from launch_xml.launch_description_sources import XMLLaunchDescriptionSource

def generate_launch_description():
    psen_scan_v2_launch_file = os.path.join(
        get_package_share_directory('psen_scan_v2'),'launch/bringup.launch.xml')
    
    return LaunchDescription([
        Node(
            package="agv_pilz",
            executable="md49_driver.py",
            name='md49_driver'
        ),
        IncludeLaunchDescription(
            XMLLaunchDescriptionSource([psen_scan_v2_launch_file]))
    ])
     
