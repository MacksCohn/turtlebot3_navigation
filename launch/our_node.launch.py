import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():

    nav2_yaml = os.path.join(get_package_share_directory(
        'turtlebot3_navigation'), 'config', 'amcl_config.yaml')

    return LaunchDescription([
         Node(
            package='nav2_amcl',
            executable='amcl',
            name='amcl',
            output='screen',
            parameters=[nav2_yaml]
        ),
        Node(
            package='turtlebot3_navigation',
            name='our_node',
            executable='example_node'
        ),
        Node(
            package='nav2_lifecycle_manager',
            executable='lifecycle_manager',
            name='lifecycle_manager',
            output='screen',
            parameters=[{'autostart': True},
                        {'node_names': [
                                        'amcl',
                                        'example_node',
                                        ]}])
    ])