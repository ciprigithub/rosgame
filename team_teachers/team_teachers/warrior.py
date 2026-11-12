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
        #self.timer_debug = self.create_timer(1, self.print_debug_screen)


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

        # Display number of beams and min distances
        #self.get_logger().info(f"== [Laser] Warrior '{self.nick}': beams={n}, min_dist=({self.left_min_dist,self.front_min_dist,self.right_min_dist})  ==")


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
                    ehp = enemy.get("hp", 100)
                    ehammer = enemy.get("Hammer", False)
                    eshield = enemy.get("Shield", False)
                    lines.append(
                        f"  - Enemy {idx}: pos=({epos[0]:.2f}, {epos[1]:.2f}) | "
                        f"HP={ehp}/100 | Hammer={ehammer} | Shield={eshield}"
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


    def score_enemy(self, enemy: dict) -> float:
        """
        Computes a priority cost for an enemy (lower = better target).
        Balances distance (time efficiency) against Shield, Hammer, and HP.
        """
        epos = enemy.get("pos", [0.0, 0.0, 0.0])
        dist = math.hypot(epos[0] - self.pos_x, epos[1] - self.pos_y)

        ehp = enemy.get("hp", 100)
        eshield = enemy.get("Shield", False)
        ehammer = enemy.get("Hammer", False)

        # Effective hits/HP factor (scaled so 50 HP ~= 2.0m of travel distance)
        hp_cost = 0.04 * ehp

        # Shield halves our hammer damage (50 -> 25), making kills much slower
        shield_cost = 3.0 if eshield else 0.0

        # If we have Shield + Hammer, we can trade safely even against Hammer users;
        # without Shield, strongly avoid enemy Hammers unless they are critical HP.
        if self.hammer_enabled and self.shield_enabled:
            hammer_cost = 1.5 if ehammer else 0.0
        else:
            hammer_cost = 6.0 if ehammer else 0.0

        return dist + hp_cost + shield_cost + hammer_cost
    

    def select_target(self):
        """Chooses the next (x, y) waypoint based on battery, skills, and enemies."""

        # God Mode!
        juggernaut = self.hammer_enabled and self.shield_enabled

        # 1. Critical battery check (ignore normal 45% threshold if in Juggernaut mode!)
        bat_threshold = 20.0 if juggernaut else 45.0
        if self.battery < bat_threshold and self.charger_pos_list:
            return self.get_closest_point(self.charger_pos_list), False

        # 2. FULL AGGRESSION: Hammer + Shield active -> hunt weakest/closest enemy immediately!
        if juggernaut and self.player_pos_list:
            enemies = [e for e in self.player_pos_list if isinstance(e, dict)]
            if enemies:
                best = min(enemies, key=self.score_enemy)
                return (best["pos"][0], best["pos"][1]), True

        # 3. HAMMER ACTIVE (no Shield): hunt weakest unarmed/unshielded enemy
        if self.hammer_enabled and self.player_pos_list:
            enemies = [e for e in self.player_pos_list if isinstance(e, dict)]
            if enemies:
                best = min(enemies, key=self.score_enemy)
                # Only engage a hammer-wielder without shield if they are nearly dead (<= 25 HP)
                if not best.get("Hammer", False) or best.get("Health", 100) <= 25:
                    return (best["pos"][0], best["pos"][1]), True

        # 4. OPPORTUNISTIC RAM KILL: Finish off any very weak unarmed enemy (<= 18 HP) for full heal!
        if self.player_pos_list:
            weak_enemies = [
                e for e in self.player_pos_list
                if isinstance(e, dict)
                and not e.get("Hammer", False)
                and e.get("Health", 100) <= 18
            ]
            if weak_enemies:
                best = min(weak_enemies, key=self.score_enemy)
                return (best["pos"][0], best["pos"][1]), True

        # 5. Gather Skill Blocks to gear up (Hammer / Shield)
        if self.skill_pos_list:
            return self.get_closest_point(self.skill_pos_list), False

        # 6. Top up battery if below 85%
        if self.battery < 85.0 and self.charger_pos_list:
            return self.get_closest_point(self.charger_pos_list), False

        # Fallback: patrol arena center to find targets
        return (0.0, 0.0), False
    

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
        (target_x, target_y), combat_rush = self.select_target()

        dx = target_x - self.pos_x
        dy = target_y - self.pos_y
        dist = math.hypot(dx, dy)
        desired_yaw = math.atan2(dy, dx)
        angle_err = (desired_yaw - self.gamma + math.pi) % (2.0 * math.pi) - math.pi

        # Tighten obstacle avoidance radius when lunging at a target (< 0.8m away)
        # so LiDAR doesn't shy away from the enemy robot right before impact!
        obs_threshold = 0.25 if (combat_rush and dist < 0.85) else 0.45

        if self.front_min_dist < obs_threshold:
            turn_dir = 1.5 if self.left_min_dist > self.right_min_dist else -1.5
            self.send_cmd_vel(0.08, turn_dir)
            return

        # Aggressive steering & higher top speed during combat rush
        kp_ang = 2.6 if combat_rush else 2.0
        max_v = 0.65 if combat_rush else 0.45

        angular_z = max(-2.0, min(2.0, kp_ang * angle_err))
        if abs(angle_err) < 0.7:
            linear_x = min(max_v, 0.25 + 0.6 * dist)
        else:
            linear_x = 0.12

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