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
            package='team_xx',
            executable='warrior',
            name='warrior',
            output='screen'
        )
    ])