import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('grid_size', default_value='10', description='Size of the grid'),
        DeclareLaunchArgument('pickups', default_value='5', description='Number of pickups'),

        Node(
            package='dustbot_package',
            executable='world_node',
            name='world_node',
            output='screen',
            parameters=[{
                'grid_size': os.environ.get('grid_size', 10),
                'pickups': os.environ.get('pickups', 5)
            }]
        ),

        Node(
            package='dustbot_package',
            executable='robot_node',
            name='robot_node',
            output='screen'
        )
    ])
