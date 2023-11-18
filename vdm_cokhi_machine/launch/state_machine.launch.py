import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from launch_ros.actions import Node
from launch_xml.launch_description_sources import XMLLaunchDescriptionSource

def generate_launch_description():
    
    launch_dir = os.path.join(get_package_share_directory('rosbridge_server'),'launch')
    
    return LaunchDescription([
        
        # Launch websocket_server
    	IncludeLaunchDescription(
    	    XMLLaunchDescriptionSource(os.path.join(launch_dir, 'rosbridge_websocket_launch.xml'))),

        # Run node
        Node(
            package='vdm_cokhi_machine',
            namespace='',
            executable='plc_service_ros',
            name='plc_service_ros',
            output="screen",
        ),
    ])
