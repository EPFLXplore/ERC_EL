import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
def generate_launch_description():
    ld = LaunchDescription()
    config = os.path.join(
        get_package_share_directory('roco-ros-interface'),
        'config',
        'node_ids_params.yaml'
        )
        
    node=Node(
        package='roco-ros-interface',
        executable='roco_interface',
        namespace='can0',
        parameters = [config]
    )
    ld.add_action(node)
    return ld