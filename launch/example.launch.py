import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    
    map_file = os.path.join(get_package_share_directory('turtlebot3_navigation'), 'config', 'map.yaml')
    controller_yaml = os.path.join(get_package_share_directory('turtlebot3_navigation'), 'config', 'controller.yaml')
    return LaunchDescription([
         Node(
            package='nav2_map_server',
            executable='map_server',
            name='map_server',
            output='screen',
            parameters=[{'use_sim_time': True}, 
                        {'yaml_filename':map_file} 
                       ]),
        Node(
            package='nav2_planner',
            executable='planner_server',
            name='planner_server',
            output='screen'),
        Node(
            package='nav2_bt_navigator',
            executable='bt_navigator',
            name='bt_navigator',
            output='screen'),
        Node(
            package='nav2_controller',
            executable='controller_server',
            name='controller_server',
            output='screen',
            parameters=[{'yaml_filename':controller_yaml}]),
        Node(
            package='nav2_lifecycle_manager',
            executable='lifecycle_manager',
            name='lifecycle_manager_mapper',
            output='screen',
            parameters=[{'use_sim_time': True},
                        {'autostart': True},
                        {'node_names': ['map_server',
                                        'bt_navigator',
                                        'planner_server',
                                        'controller_server',
                                        ]}]),
        Node(
            package='turtlebot3_navigation',
            executable='example_node',
            output='screen',
            ),        
        ])
