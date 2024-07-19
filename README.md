# rosgame
This package implements a game manager in which players are robots that must survive in a tournament arena. 

The game is played in a simulated world using CoppeliaSim. Players, ROS2 nodes, get world information like laser data, odometry and information about near items. Available items are battery recharge, shield, hammer, and automatic navigation.

To test the package, three different types of players are implemented:

- Pacific players that will navigate searching for battery recharge points and are not interested in having items
- War players that will search hammers and oponents
- Elusive players that are only interested in avoiding obstacles.

How to test it?

1.- Launch tfg_scene in Coppelia (TODO change the name :))
2.- Run bridge node from rosgame_bridge package
3.- Launch one of the launch files from the rosgame_players package
