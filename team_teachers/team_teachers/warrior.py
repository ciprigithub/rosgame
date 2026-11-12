import math
import json
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from std_msgs.msg import String
from rosgame_msgs.msg import RosgameTwist
from rosgame_bridge.srv import RosgameRegister


class WarriorNode(Node):
    def __init__(self):
        super().__init__('warrior_teachers_node')
        
        # Robot state vars (to complete)
        self.health = 100
        self.battery = 100.0
        self.pos_x = 0.0
        self.pos_y = 0.0
        self.gamma = 0.0
        self.hammer_enabled = False
        self.shield_enabled = False
        self.scene_received = False

        # FOV items
        self.skill_pos_list = []
        self.charger_pos_list = []
        self.player_pos_list = []

        # LiDAR distances (by sector)
        self.front_min_dist = float('inf')
        self.left_min_dist = float('inf')
        self.right_min_dist = float('inf')

        # We need to call a Game Srv to register
        self.cli = self.create_client(RosgameRegister, 'register_service')
        while not self.cli.wait_for_service(timeout_sec=2.0):
            self.get_logger().info('Register service not available, waiting...')

        # Register our robot
        self.nick = "team_teachers_warrior"  # set your team here!
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
                    self.get_logger().warning("Username taken. Retrying with incremented nick...")
                    self.nick = f"team_teachers_warrior_{self.cont}"
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
        
        self.get_logger().info(f"== [CyPR] Warrior Node '{self.nick}' registered! Implement your strategy and good luck! ==")

        # Timers: 10 Hz navigation loop & 2 Hz debug screen output
        self.timer_control = self.create_timer(0.1, self.control_loop)
        self.timer_debug = self.create_timer(0.5, self.print_debug_screen)


    # ==================================================================
    # 1. SENSOR & SCENE CALLBACKS
    # ==================================================================
    def process_laser_info(self, msg: LaserScan):
        """Splits the LaserScan into Right, Front, and Left sectors."""
        ranges = msg.ranges
        n = len(ranges)
        if n == 0:
            return

        # Consider 3 segments
        third = n // 3
        right_rays = [r for r in ranges[:third] if msg.range_min < r < msg.range_max]
        front_rays = [r for r in ranges[third:2 * third] if msg.range_min < r < msg.range_max]
        left_rays = [r for r in ranges[2 * third:] if msg.range_min < r < msg.range_max]

        # min distance on each segment
        self.right_min_dist = min(right_rays) if right_rays else float('inf')
        self.front_min_dist = min(front_rays) if front_rays else float('inf')
        self.left_min_dist = min(left_rays) if left_rays else float('inf')


    def process_scene_info(self, msg: String):
        # Scene info processing: Robot Pose, Battery lvl, Active Skills, FOV (chargers, powerups, enemies)
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
            self.skill_pos_list = fov.get("Skills_Positions", [])
            self.charger_pos_list = fov.get("Chargers_Positions", [])
            self.player_pos_list = fov.get("Players_Positions", [])

            self.scene_received = True

        except Exception as e:
            self.get_logger().error(f"[WARRIOR] Error parsing JSON payload: {e}")


    # ==================================================================
    # 2. DEBUG 
    # ==================================================================
    def print_debug_screen(self):
        """Prints current pose, battery, skills, items, and enemy status."""
        if not self.scene_received:
            return

        # Show scene_info on screen (organized and clear)
        chargers_str = [(round(c[0], 2), round(c[1], 2)) for c in self.charger_pos_list]
        skills_str = [(round(s[0], 2), round(s[1], 2)) for s in self.skill_pos_list]

        lines = [
            f"\n==================== [{self.nick} | {self.code}] ====================",
            f"Pose: (x={self.pos_x:.2f}, y={self.pos_y:.2f}, yaw={self.gamma:.2f} rad)",
            f"Status: Health={self.health}/100 | Battery={self.battery:.1f}%",
            f"My Skills: Hammer={self.hammer_enabled} | Shield={self.shield_enabled}",
            f"Chargers ({len(self.charger_pos_list)}): {chargers_str}",
            f"Skill Blocks ({len(self.skill_pos_list)}): {skills_str}",
            f"Enemies ({len(self.player_pos_list)}):"
        ]

        # Show Enemies
        if not self.player_pos_list:
            lines.append("  - None visible")
        else:
            for idx, enemy in enumerate(self.player_pos_list, start=1):
                if isinstance(enemy, dict):
                    epos = enemy.get("pos", [0.0, 0.0, 0.0])
                    ehammer = enemy.get("Hammer", False)
                    eshield = enemy.get("Shield", False)
                    lines.append(
                        f"  - Enemy {idx}: pos=({epos[0]:.2f}, {epos[1]:.2f}) | "
                        f"Hammer={ehammer} | Shield={eshield}"
                    )
                else:
                    lines.append(f"  - Enemy {idx}: pos=({enemy[0]:.2f}, {enemy[1]:.2f})")

        lines.append("=================================================================")
        self.get_logger().info("\n".join(lines))



    # ==================================================================
    # 3. NAVIGATION CONTROLLER (basic)
    # ==================================================================
    def get_closest_point(self, points):
        """Returns (x, y) of the closest 2D point to our robot."""
        best_pt = None
        best_dist_sq = float('inf')
        for pt in points:
            dx = pt[0] - self.pos_x
            dy = pt[1] - self.pos_y
            d_sq = dx * dx + dy * dy
            if d_sq < best_dist_sq:
                best_dist_sq = d_sq
                best_pt = (pt[0], pt[1])
        return best_pt

    def select_target(self):
        """Chooses the next (x, y) waypoint based on battery, skills, and enemies."""
        # 1. Low/medium battery -> prioritize nearest charger
        if self.battery < 45.0 and self.charger_pos_list:
            return self.get_closest_point(self.charger_pos_list)

        # 2. If we have Hammer -> hunt enemies that DO NOT have a Hammer (if any)
        if self.hammer_enabled and self.player_pos_list:
            unarmed_enemies = []
            for enemy in self.player_pos_list:
                if isinstance(enemy, dict):
                    if not enemy.get("Hammer", False):
                        unarmed_enemies.append(enemy["pos"])
                else:
                    unarmed_enemies.append(enemy)

            if unarmed_enemies:
                return self.get_closest_point(unarmed_enemies)

        # 3. Gather skill blocks on sight (to get Hammer / Shield)
        # It may be empty
        if self.skill_pos_list:
            return self.get_closest_point(self.skill_pos_list)

        # 4. Top up battery if below 85%
        if self.battery < 85.0 and self.charger_pos_list:
            return self.get_closest_point(self.charger_pos_list)

        # Fallback: "wander" or move toward center of the arena
        return (0.0, 0.0)
    

    def send_cmd_vel(self, linear_x: float, angular_z: float):
        """Publishes a RosgameTwist velocity command."""
        msg = RosgameTwist()
        msg.code = self.code
        msg.vel.linear.x = float(linear_x)
        msg.vel.angular.z = float(angular_z)
        self.pub_cmdvel.publish(msg)


    def control_loop(self):
        """Called by timer: navigation + obstacle avoidance"""
        if not self.scene_received:
            return

        # Stop robot if eliminated
        if self.health <= 0:
            self.send_cmd_vel(0.0, 0.0)
            return

        # Where to move? skill / battery / enemy?
        target_x, target_y = self.select_target()

        # Reactive LiDAR obstacle avoidance
        # We have a goal, but lets avoid hitting obstacles
        if self.front_min_dist < 0.45:
            # Turn toward whichever side has more clearance
            turn_dir = 1.2 if self.left_min_dist > self.right_min_dist else -1.2
            self.send_cmd_vel(0.05, turn_dir)
            return

        # Proportional go-to-goal steering
        dx = target_x - self.pos_x
        dy = target_y - self.pos_y
        dist = math.hypot(dx, dy)
        desired_yaw = math.atan2(dy, dx)

        # Normalize angular error to [-pi, pi]
        angle_err = (desired_yaw - self.gamma + math.pi) % (2.0 * math.pi) - math.pi

        angular_z = max(-1.5, min(1.5, 2.0 * angle_err))
        if abs(angle_err) < 0.6:
            linear_x = min(0.45, 0.20 + 0.5 * dist)
        else:
            linear_x = 0.10

        self.send_cmd_vel(linear_x, angular_z)


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
    finally:
        if hasattr(node, 'pub_cmdvel'):
            node.send_cmd_vel(0.0, 0.0)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()