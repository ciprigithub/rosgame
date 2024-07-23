# rosgame
A ROS2 repository to learn and practice the ROS2 concepts while playing a "video-game".
It is designed as part of a university course in "robotic control and programming" from University of Málaga.
The "game" is designed within the CoppeliaSim robotic simulator, but must be intereacted from ROS2 nodes (c++ or python).

It is composed of four folders, containing:
1. Bridge: Code that reads/exports data to/from the game, allowing communication with ROS2 nodes (players). It controlls the data flow to implement some "security" like avoiding players to read data from other robots, or commanding a robot that is not under control.
2. Msgs: Definition package to host specific messages used in the game.
3. Players: Some implementation examples of "players", that is nodes that will command a robot through the game based on its readings.
4. Scene: The CoppliaSim scene contaning the "vide-game".


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
