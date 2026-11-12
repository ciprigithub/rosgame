from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        # Bridge Node (Comms with CoppeliaSim)
        Node(
            package='rosgame_bridge',
            executable='bridge',
            name='rosgame_bridge',
            output='screen'
        ),
        
        # Player Node (Bot Royale)
        Node(
            package='team_teachers',
            executable='warrior',
            name='warrior',
            output='screen'
        ),
        Node(
            package='team_teachers',
            executable='warrior',
            name='warrior2',
            output='screen'
        ),
        Node(
            package='team_teachers',
            executable='warrior',
            name='warrior3',
            output='screen'
        ),
        Node(
            package='team_teachers',
            executable='warrior',
            name='warrior4',
            output='screen'
        )
    ])