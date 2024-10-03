# rosgame
A ROS2 workspace to learn and practice the ROS2 concepts while playing a "video-game".
It is designed as part of a university course in "robotic control and programming" from University of Málaga.
The "game" is designed within the CoppeliaSim robotic simulator (as a scene), implementing all the game dynamics as well as offering a ROS2 API to allow "players" to  intereact with one robot within the game. Therefore, all players must be implemented as ROS2 nodes (c++ or python).

The code is organized into four pkgs:
1. rosgame_bridge: Pkg that reads/exports data to/from the game (that is, coppeliaSim), allowing communication with ROS2 nodes (players). It controlls the data flow to implement some "security" like avoiding players to read data from other robots (which is considered cheating XD), or commanding a robot that is not under their control.
2. rosgame_msgs: Definition package to host specific messages used in the game.
3. rosgame_players: Some implementation examples of "players", that is, ROS2 nodes that will command a robot through the game based on its senor readings. Those examples are provided to guide the creation of more advances players.
4. rosgame_scene: Though strictly speaking not a pkg, its the folder containing the CoppliaSim scene where the "video-game" its implemented.


# How To Use:
1. Run CoppeliaSim and launch the "tfg_scene" to start the game (TODO change the name)
2. Run one single instance of the "bridge" node from rosgame_bridge package
3. Run one or multiple players. You can launch your own implementation of a player or use an example from the ones contained in the rosgame_players package.
4. Enjoy!

# Credits
University of Málaga
MAPIR Research Group - Machine Perception and Intelligent Robotics

# To Do List
- Remove Robot from Scene upon node kill (CoppeliSim)
