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

        # Referee
        Node(
            package='team_teachers',
            executable='referee',
            name='match_referee',
            output='screen'
        ),
        
        # Player Nodes
        Node(
            package='team_teachers',
            executable='warrior',
            name='team_teachers_warrior1',
            output='screen'
        ),
        Node(
            package='team_teachers',
            executable='warrior',
            name='team_teachers_warrior2',
            output='screen'
        ),
        Node(
            package='team_teachers',
            executable='warrior',
            name='team_teachers_warrior3',
            output='screen'
        ),
        Node(
            package='team_teachers',
            executable='warrior',
            name='team_teachers_warrior4',
            output='screen'
        ),
        
    ])