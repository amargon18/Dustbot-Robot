import launch
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration

def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('N', default_value='5', description='Size of the grid'),
        DeclareLaunchArgument('P', default_value='3', description='Number of garbage pick-ups'),

        Node(
            package='dustbot',
            executable='world_node',
            name='world_node',
            output='screen',
            parameters=[{'N': LaunchConfiguration('N'), 'P': LaunchConfiguration('P')}]
        ),

        Node(
            package='dustbot',
            executable='robot_node',
            name='robot_node',
            output='screen',
            parameters=[{'N': LaunchConfiguration('N'), 'P': LaunchConfiguration('P')}]
        ),
    ])
