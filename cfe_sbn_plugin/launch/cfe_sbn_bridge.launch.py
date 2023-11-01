import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    ld = LaunchDescription()
    config = os.path.join(
        get_package_share_directory('cfe_sbn_plugin'),
        'config',
        'cfe_sbn_config.yaml'
        )

    sbn_config = os.path.join(
        get_package_share_directory('cfe_sbn_plugin'),
        'config',
        'cfe_plugin_config.yaml'
        )

    node = Node(
        package='fsw_ros2_bridge',
        name='cfe_sbn_bridge',
        executable='fsw_ros2_bridge',
        parameters=[config, sbn_config]
    )
    ld.add_action(node)
    return ld
