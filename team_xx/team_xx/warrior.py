#!/usr/bin/env python3
"""
===============================================================================
ROSGAME - Bot Royale Match Player Template (team_xx_warrior)
Machine Perception and Intelligent Robotics (MAPIR) - University of Malaga
===============================================================================
"""
import rclpy
import json
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from std_msgs.msg import String
from rosgame_msgs.msg import RosgameTwist
from rosgame_bridge.srv import RosgameRegister

# ==============================================================================
#  TEAM CONFIGURATION (EDIT ONLY THIS LINE WITH YOUR ASSIGNED TEAM NUMBER!)
# ==============================================================================
TEAM_ID = "team_xx"   # Example: "team_01", "team_02", ...


class WarriorNode(Node):
    def __init__(self):
        super().__init__(f"{TEAM_ID}_warrior")
        
        # Robot state vars (to complete)
        self.health = 100
        self.battery = 100.0
        self.pos_x = 0.0
        self.pos_y = 0.0
        self.gamma = 0.0
        self.hammer_enabled = False
        self.shield_enabled = False

        # Field-of-View (FOV) detections
        self.skill_pos_list = []
        self.charger_pos_list = []
        self.player_pos_list = []

        # Connect to the game registration service
        self.cli = self.create_client(RosgameRegister, 'register_service')
        while not self.cli.wait_for_service(timeout_sec=2.0):
            self.get_logger().info('Register service not available, waiting...')

        # Register our robot
        self.nick = self.get_name()     # read the launch.name team_xx_warrior
        self.register_player()


    
    def register_player(self):
        """Registers a new player within the game."""
        req = RosgameRegister.Request()
        req.username = self.nick
        self.code = "-1"
        self.cont = 0

        while self.code == "-1" and rclpy.ok():
            future = self.cli.call_async(req)
            rclpy.spin_until_future_complete(self, future)
            res = future.result()
            
            if res is not None:
                self.code = res.code
                if self.code == "-1":
                    self.get_logger().warning("Username taken. Retrying with incremented nick...")
                    self.nick = f"{TEAM_ID}_warrior_{self.cont}"
                    req.username = self.nick
                    self.cont += 1
                else:
                    # Registration successful
                    self.init_subscribers_and_publishers()
            else:
                self.get_logger().error("Failed to connect to registration service.")


    
    def init_subscribers_and_publishers(self):
        """ Initializes the necessary subscribers and publishers for the robot. """
        # Publishers
        self.pub_cmdvel = self.create_publisher(RosgameTwist, f'/{self.code}/cmd_vel', 1)
        
        # Subscribers
        self.sub_laserscan = self.create_subscription(LaserScan, f'/{self.code}/laser_scan', self.process_laser_info, 1)
        self.sub_sceneinfo = self.create_subscription(String, f'/{self.code}/scene_info', self.process_scene_info, 1)
        
        self.get_logger().info(f"== [CyPR] Warrior Node '{self.nick}' registered! Implement your strategy and good luck! ==")

    

    def process_laser_info(self, msg: LaserScan):
        # TODO: Implement LiDAR processing logic here
        pass



    def process_scene_info(self, msg: String):
        # Scene info processing: Robot Pose, Battery lvl, HP points, Active Skills, FOV (chargers, powerups, enemies)
        # TODO: Adapt this to your strategy and data handling needs
        try:
            scene_data = json.loads(msg.data)

            self.health = int(scene_data.get("Health", 100))
            self.battery = float(scene_data.get("Battery_Level", 100.0))
            
            pose = scene_data.get("Robot_Pose", {})
            self.pos_x = float(pose.get("x", 0.0))
            self.pos_y = float(pose.get("y", 0.0))
            self.gamma = float(pose.get("gamma", 0.0))
            
            skills = scene_data.get("Skills", {})
            self.hammer_enabled = bool(skills.get("Hammer", False))
            self.shield_enabled = bool(skills.get("Shield", False))
            
            fov = scene_data.get("FOV", {})
            self.charger_pos_list = fov.get("Chargers_Positions", [])
            self.skill_pos_list = fov.get("Skills_Positions", [])
            self.player_pos_list = fov.get("Players_Positions", [])

            # TODO: Implement your decision-making!!
                    
        except Exception as e:
            self.get_logger().error(f"[WARRIOR] Error parsing JSON payload: {e}")


# ==============================
#             MAIN
# ==============================
def main(args=None):
    rclpy.init(args=args)
    node = WarriorNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()