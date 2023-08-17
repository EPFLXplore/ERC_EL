import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    ld = LaunchDescription()

    ld.add_action(DeclareLaunchArgument(
        'bus',
        default_value='can0',
        description='CAN bus name'
    ))

    config = os.path.join(
        get_package_share_directory('roco-ros-interface'),
        'config',
        'node_ids_params.yaml'
    )

    # Launch first node with 'can0' namespace and 'bus' parameter set to 'can0'
    node_can0 = Node(
        package='roco-ros-interface',
        executable='roco_interface',
        namespace='can0',
        parameters=[config],
        arguments=['--ros-args', '--param', 'bus:=can0']
    )
    ld.add_action(node_can0)

    # # Launch second node with 'can1' namespace and 'bus' parameter set to 'can1'
    # node_can1 = Node(
    #     package='roco-ros-interface',
    #     executable='roco_interface',
    #     namespace='can1',
    #     parameters=[config],
    #     arguments=['--ros-args', '--param', 'bus:=can1']
    # )
    # ld.add_action(node_can1)

    return ld