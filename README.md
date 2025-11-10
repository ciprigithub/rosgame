# Game-Based Learning in Mobile Robotics. Teaching ROS2 Through a Competitive Game

A ROS2 workspace and CoppeliaSim scenes to learn and practice the ROS2 concepts while playing different"video-games". 

It is designed as part of a undergraduate course in "robotic control and programming" from University of Málaga.

The game baptized as "rosgame"i s designed within the CoppeliaSim robotic simulator (as a scene), implementing all the game dynamics as well as offering a ROS2 API to allow "players" to  intereact with one robot within the game. Therefore, all players must be implemented as ROS2 nodes (c++ or python).

The code is organized into four pkgs:
1. **rosgame_bridge**: Pkg that reads/exports data to/from the game (that is, coppeliaSim), allowing communication with ROS2 nodes (players). It controlls the data flow to implement some "security" like avoiding players to read data from other robots (which is considered cheating XD), or commanding a robot that is not under their control.
2. **rosgame_msgs**: Definition package to host specific messages used in the game.
3. **rosgame_players**: Some implementation examples of "players", that is, ROS2 nodes that will command a robot through the game based on its senor readings. Those examples are provided to guide the creation of more advances players.
4. **rosgame_scenes**: Though strictly speaking not a pkg, thise folder contais the CoppliaSim scenes where the "video-game" its implemented. Different scenes implement different games:
    - **survival_square.ttt** The game dynamics implemented in this scene are those of a survival game, where the goal is to be the last robot alive. 
        - Battery: Players must take care of their battery levels. There are randomly placed rechargin-stations that can be used for recharging. If a robot gets on low battery, the movement speed is reduced considerably (making it an easy prey -_-).
        - Health: All robots have a life/health metter. This indicador decreases when fighting other robots equipped with a hammer. Be careful, reaching 0 means Game Over.
        - Ability blocks: The game offers a set of ability blocks randomly placed withing the scenario. Players can get those blocks to gain abilities like "shield", "hammer" or "autopilot"


# How To Use:
1. Run CoppeliaSim and launch one game. For example "survival_square.ttt" to start the survival game. You should see how the game is poppulated with different obstacles, points to recharge the battery and ability blocks.
2. Run one single instance of the "bridge" node from the "rosgame_bridge" package. You can use the "rosgame_bridge_launch.xml" launch file to do so.
3. Finally, run one or multiple players. You can launch your own implementation of a player, or use an example from the ones contained in the rosgame_players package. You can use the launch files in the "rosgame_players" pkg.
4. Enjoy!

# Credits
University of Málaga
MAPIR Research Group - Machine Perception and Intelligent Robotics
.
