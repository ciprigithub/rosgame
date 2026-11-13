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
            name='team_teachers_warrior',
            output='screen'
        ),
        Node(
            package='team_xx',
            executable='warrior',
            name='team_xx_warrior',
            output='screen'
        ),
        Node(
            package='team_teachers',
            executable='referee',
            name='match_referee',
            output='screen'
        )
    ])