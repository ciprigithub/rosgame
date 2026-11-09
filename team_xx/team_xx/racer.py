import rclpy
from rclpy.node import Node
import json
from sensor_msgs.msg import LaserScan
from std_msgs.msg import String
from rosgame_msgs.msg import RosgameTwist, RosgamePoint
from rosgame_bridge.srv import RosgameRegister

class RacerNode(Node):
    def __init__(self):
        super().__init__('racer_node')
                      
        # Robot state vars (to complete)
        self.battery = 100.0
        self.points = 0
        self.pos_x = 0.0
        self.pos_y = 0.0
        self.gamma = 0.0
        self.list_coin_pos = []
        self.list_charger_pos = []

        # We need to call a Game Srv to register
        self.cli = self.create_client(RosgameRegister, 'register_service')
        while not self.cli.wait_for_service(timeout_sec=2.0):
            self.get_logger().info('Register service not available, waiting...')
        
        # Register our robot
        self.nick = "team_xx_racer"  # set your team here!
        self.register_player()

    """
    Registers a new player within the game.
    """
    def register_player(self):
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
                    self.get_logger().warn("Username taken. Retrying with incremented nick...")
                    self.nick = f"team_xx_racer_{self.cont}"
                    req.username = self.nick
                    self.cont += 1
                else:
                    # Registration successful
                    self.init_subscribers_and_publishers()
            else:
                self.get_logger().error("Failed to connect to registration service.")

    """
    Initializes the necessary subscribers and publishers for the robot.
    """
    def init_subscribers_and_publishers(self):
        # Publishers
        self.pub_cmdvel = self.create_publisher(RosgameTwist, f'/{self.code}/cmd_vel', 1)
        
        # Subscribers
        self.sub_laserscan = self.create_subscription(LaserScan, f'/{self.code}/laser_scan', self.process_laser_info, 1)
        self.sub_sceneinfo = self.create_subscription(String, f'/{self.code}/scene_info', self.process_scene_info, 1)
        
        # Log registration success
        self.get_logger().info(f"== [CyPR] Racer Node '{self.nick}' registered! Implement your strategy and good luck! ==")


    def process_laser_info(self, msg: LaserScan):        
        # TODO: Implement LiDAR processing logic here
        pass


    def process_scene_info(self, msg: String):
        # Scene info processing: Robot Pose, Battery lvl, Points, FOV (chargers, coins)
        # TODO: Adapt this to your strategy and data handling needs
        try:
            # Data format is JSON
            scene_data = json.loads(msg.data)

            # Get current battery level
            self.battery = float(scene_data.get("Battery_Level", 100.0))
            
            # Get current number of points (+coins-collisions)
            self.points = int(scene_data.get("Points", 0))

            # Get current robot pose
            pose = scene_data.get("Robot_Pose", {})
            self.pos_x = float(pose.get("x", 0.0))
            self.pos_y = float(pose.get("y", 0.0))
            self.gamma = float(pose.get("gamma", 0.0))
            
            # Get current field of view (FOV) information
            fov = scene_data.get("FOV", {})
            
            # Charger Positions
            all_chargers = fov.get("Chargers_Positions", [])
            # TODO: what to do with this info

            # Coins
            all_coins = fov.get("Coins_Positions", [])
            # TODO: what to do with this info
                    
        except Exception as e:
            self.get_logger().error(f"[RACER] Error parsing JSON payload: {e}")


# ==============================
#             MAIN
# ==============================
def main(args=None):
    rclpy.init(args=args)
    node = RacerNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()