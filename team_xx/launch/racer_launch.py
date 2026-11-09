from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        # Bridge Node (Comms with CoppeliaSim)
        Node(
            package='rosgame_bridge',
            executable='rosgame_bridge_node',
            name='rosgame_bridge',
            output='screen'
        ),
        
        # Player Node (Crono Bot)
        Node(
            package='team_xx',
            executable='racer',
            name='racer',
            output='screen'
        )
    ])
