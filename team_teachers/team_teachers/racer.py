#!/usr/bin/env python3
import json
import math
import time
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from std_msgs.msg import String
from rosgame_msgs.msg import RosgameTwist
from rosgame_bridge.srv import RosgameRegister

# ==============================================================================
#  TEAM CONFIGURATION (EDIT ONLY THIS LINE WITH YOUR ASSIGNED TEAM NUMBER!)
# ==============================================================================
TEAM_ID = "team_xx"   # Example: "team_01", "team_02", ...


def normalize_angle(angle: float) -> float:
    """Wraps an angle in radians to [-pi, pi]."""
    return math.atan2(math.sin(angle), math.cos(angle))


class RacerNode(Node):
    def __init__(self):
        super().__init__(f"{TEAM_ID}_racer")

        # Robot state variables
        self.battery = 100.0
        self.points = 0
        self.pos_x = 0.0
        self.pos_y = 0.0
        self.gamma = 0.0
        self.pose_received = False

        # Global arena positions (FOV disabled -> full arena coin & charger list)
        self.list_coin_pos = []
        self.list_charger_pos = []
        self.is_charging = False

        # Short-range LiDAR sector minimum distances (meters)
        self.lidar_max_range = 0.0
        self.front_dist = 1.5
        self.front_left_dist = 1.5
        self.front_right_dist = 1.5
        self.left_dist = 1.5
        self.right_dist = 1.5
        self.lidar_ready = False

        # Control smoothing & avoidance hysteresis state
        self.prev_heading_err = 0.0
        self.prev_control_time = time.time()
        self.avoid_dir = None          # Locks +1 (left) or -1 (right) during an avoidance maneuver

        # Stuck detection & escape state
        self.last_check_time = time.time()
        self.last_check_pos = None
        self.recover_until = 0.0
        self.recover_turn_dir = 1.0

        # Debug telemetry snapshot
        self.debug_state = "INIT"
        self.debug_target_type = "NONE"
        self.debug_target = None       # (tx, ty) or None
        self.debug_heading_err = 0.0
        self.debug_cmd = (0.0, 0.0)    # (linear_x, angular_z)

        # Connect to the game registration service
        self.cli = self.create_client(RosgameRegister, 'register_service')
        while not self.cli.wait_for_service(timeout_sec=2.0):
            self.get_logger().info('Register service not available, waiting...')

        # Register our robot using the active node name
        self.nick = self.get_name()
        self.register_player()

    def register_player(self):
        """Registers a new player within the game."""
        req = RosgameRegister.Request()
        req.username = self.nick
        self.code = "-1"
        self.cont = 1

        while self.code == "-1" and rclpy.ok():
            future = self.cli.call_async(req)
            rclpy.spin_until_future_complete(self, future)
            res = future.result()

            if res is not None:
                self.code = res.code
                if self.code == "-1":
                    self.get_logger().warning("Username taken. Retrying with incremented nick...")
                    self.nick = f"{self.get_name()}_{self.cont}"
                    req.username = self.nick
                    self.cont += 1
                else:
                    self.init_subscribers_and_publishers()
            else:
                self.get_logger().error("Failed to connect to registration service.")

    def init_subscribers_and_publishers(self):
        """Initializes the necessary subscribers and publishers for the robot."""
        self.pub_cmdvel = self.create_publisher(RosgameTwist, f'/{self.code}/cmd_vel', 1)

        self.sub_laserscan = self.create_subscription(
            LaserScan, f'/{self.code}/laser_scan', self.process_laser_info, 1
        )
        self.sub_sceneinfo = self.create_subscription(
            String, f'/{self.code}/scene_info', self.process_scene_info, 1
        )

        self.get_logger().info(
            f"== [CyPR] Racer Node '{self.nick}' registered! Global coin planner active =="
        )

        # 25 Hz control loop + 1 Hz debug telemetry report
        self.cmd_timer = self.create_timer(0.04, self.control_loop)
        self.debug_timer = self.create_timer(1.0, self.debug_status)

    # --------------------------------------------------------------------------
    #  DEBUG TELEMETRY FUNCTION
    # --------------------------------------------------------------------------
    def debug_status(self):
        """Logs current points, battery, active state, next target, LiDAR, and velocity."""
        if not self.pose_received:
            self.get_logger().info(f"[{self.nick}] Waiting for scene_info...")
            return

        v, w = self.debug_cmd
        err_deg = math.degrees(self.debug_heading_err)

        if self.debug_target is not None:
            tx, ty = self.debug_target
            dist = math.hypot(tx - self.pos_x, ty - self.pos_y)
            target_info = f"{self.debug_target_type} ({tx:+.2f}, {ty:+.2f}) @ {dist:.2f}m"
        else:
            target_info = "NONE"

        self.get_logger().info(
            f"[{self.nick}] PTS={self.points:3d} | BAT={self.battery:5.1f}% | "
            f"STATE={self.debug_state:<14} | NEXT={target_info:<28} | "
            f"ERR={err_deg:+5.1f}deg | LIDAR(L/F/R)={self.left_dist:.2f}/{self.front_dist:.2f}/{self.right_dist:.2f}m | "
            f"CMD=(v={v:+.2f}, w={w:+.2f})"
        )

    # --------------------------------------------------------------------------
    #  SENSOR & TELEMETRY CALLBACKS
    # --------------------------------------------------------------------------
    def process_laser_info(self, msg: LaserScan):
        """Splits short-range LiDAR into 5 fine sectors for tight wall/pipe negotiation."""
        max_r = msg.range_max if math.isfinite(msg.range_max) and msg.range_max > 0.1 else 1.5
        self.lidar_max_range = max_r

        front = max_r
        f_left = max_r
        f_right = max_r
        left = max_r
        right = max_r

        angle = msg.angle_min
        for r in msg.ranges:
            if math.isfinite(r) and msg.range_min < r < max_r:
                deg = math.degrees(angle)
                if -18.0 <= deg <= 18.0:
                    if r < front:
                        front = r
                elif 18.0 < deg <= 52.0:
                    if r < f_left:
                        f_left = r
                elif -52.0 <= deg < -18.0:
                    if r < f_right:
                        f_right = r
                elif 52.0 < deg <= 95.0:
                    if r < left:
                        left = r
                elif -95.0 <= deg < -52.0:
                    if r < right:
                        right = r
            angle += msg.angle_increment

        self.front_dist = front
        self.front_left_dist = f_left
        self.front_right_dist = f_right
        self.left_dist = left
        self.right_dist = right
        self.lidar_ready = True

    def process_scene_info(self, msg: String):
        """Parses Robot Pose, Battery level, Points, and global arena Coins/Chargers."""
        try:
            scene_data = json.loads(msg.data)

            self.battery = float(scene_data.get("Battery_Level", 100.0))
            self.points = int(scene_data.get("Points", 0))

            pose = scene_data.get("Robot_Pose", {})
            self.pos_x = float(pose.get("x", 0.0))
            self.pos_y = float(pose.get("y", 0.0))
            self.gamma = float(pose.get("gamma", 0.0))
            self.pose_received = True

            fov = scene_data.get("FOV", scene_data)
            self.list_charger_pos = self._parse_points(fov.get("Chargers_Positions", []))
            self.list_coin_pos = self._parse_points(fov.get("Coins_Positions", []))

        except Exception as e:
            self.get_logger().error(f"[RACER] Error parsing JSON payload: {e}")

    @staticmethod
    def _parse_points(raw_list) -> list:
        pts = []
        if not isinstance(raw_list, list):
            return pts
        for item in raw_list:
            if isinstance(item, (list, tuple)) and len(item) >= 2:
                pts.append((float(item[0]), float(item[1])))
            elif isinstance(item, dict) and "x" in item and "y" in item:
                pts.append((float(item["x"]), float(item["y"])))
        return pts

    # --------------------------------------------------------------------------
    #  2-STEP LOOKAHEAD TARGET PLANNER
    # --------------------------------------------------------------------------
    def pick_best_coin(self):
        """Chooses the coin minimizing d(robot -> coin) + 0.65 * d(coin -> next_coin)."""
        if not self.list_coin_pos:
            return None

        if len(self.list_coin_pos) == 1:
            return self.list_coin_pos[0]

        best_coin = None
        best_cost = float("inf")

        for i, (cx, cy) in enumerate(self.list_coin_pos):
            d1 = math.hypot(cx - self.pos_x, cy - self.pos_y)
            desired_yaw = math.atan2(cy - self.pos_y, cx - self.pos_x)
            turn_cost = 0.25 * abs(normalize_angle(desired_yaw - self.gamma))

            d2 = min(
                math.hypot(ox - cx, oy - cy)
                for j, (ox, oy) in enumerate(self.list_coin_pos)
                if j != i
            )

            cost = d1 + 0.65 * d2 + turn_cost
            if cost < best_cost:
                best_cost = cost
                best_coin = (cx, cy)

        return best_coin

    def select_target(self):
        """Returns ((tx, ty), hold_still, state_label, target_type)."""
        if self.battery <= 32.0:
            self.is_charging = True
        elif self.battery >= 92.0:
            self.is_charging = False

        # Opportunistic top-up if passing right beside a charger while below 55%
        if not self.is_charging and self.battery < 55.0 and self.list_charger_pos:
            nearest_c = min(
                self.list_charger_pos,
                key=lambda c: math.hypot(c[0] - self.pos_x, c[1] - self.pos_y)
            )
            if math.hypot(nearest_c[0] - self.pos_x, nearest_c[1] - self.pos_y) < 1.1:
                self.is_charging = True

        if self.is_charging and self.list_charger_pos:
            best_coin = self.pick_best_coin()
            cx, cy = min(
                self.list_charger_pos,
                key=lambda c: math.hypot(c[0] - self.pos_x, c[1] - self.pos_y)
                + (0.35 * math.hypot(best_coin[0] - c[0], best_coin[1] - c[1]) if best_coin else 0.0)
            )
            if math.hypot(cx - self.pos_x, cy - self.pos_y) < 0.28:
                return (cx, cy), True, "CHARGING", "CHARGER"
            return (cx, cy), False, "SEEK_CHARGER", "CHARGER"

        coin = self.pick_best_coin()
        if coin is not None:
            return coin, False, "COIN_PURSUIT", "COIN"

        return (0.0, 0.0), math.hypot(self.pos_x, self.pos_y) < 0.4, "CENTER_IDLE", "CENTER"

    # --------------------------------------------------------------------------
    #  SMOOTHED SHORT-RANGE LIDAR AVOIDANCE & MOTION CONTROL
    # --------------------------------------------------------------------------
    def send_velocity_command(self, linear_x: float, angular_z: float):
        msg = RosgameTwist()
        msg.code = self.code
        msg.vel.linear.x = float(linear_x)
        msg.vel.angular.z = float(angular_z)/2.0
        self.debug_cmd = (msg.vel.linear.x, msg.vel.angular.z)
        self.pub_cmdvel.publish(msg)

    def control_loop(self):
        """25 Hz damped goal-seeking + hysteresis obstacle avoidance."""
        if not self.pose_received or not self.lidar_ready:
            return

        now = time.time()
        dt = max(0.01, now - self.prev_control_time)
        self.prev_control_time = now

        # 1. Stuck recovery
        if self.last_check_pos is None:
            self.last_check_pos = (self.pos_x, self.pos_y)
            self.last_check_time = now
        elif now - self.last_check_time >= 2.0:
            moved = math.hypot(self.pos_x - self.last_check_pos[0], self.pos_y - self.last_check_pos[1])
            if moved < 0.12 and not self.is_charging:
                self.recover_until = now + 0.9
                left_space = self.front_left_dist + self.left_dist
                right_space = self.front_right_dist + self.right_dist
                self.recover_turn_dir = 1.0 if left_space >= right_space else -1.0
            self.last_check_pos = (self.pos_x, self.pos_y)
            self.last_check_time = now

        if now < self.recover_until:
            self.debug_state = "STUCK_RECOVERY"
            self.send_velocity_command(-0.22, 1.1 * self.recover_turn_dir)
            return

        # 2. Target selection
        (tx, ty), hold_still, goal_state, target_type = self.select_target()
        self.debug_target = (tx, ty)
        self.debug_target_type = target_type

        if hold_still:
            self.debug_state = goal_state
            self.debug_heading_err = 0.0
            self.avoid_dir = None
            self.send_velocity_command(0.0, 0.0)
            return

        dx = tx - self.pos_x
        dy = ty - self.pos_y
        dist_to_target = math.hypot(dx, dy)
        desired_yaw = math.atan2(dy, dx)
        heading_err = normalize_angle(desired_yaw - self.gamma)
        self.debug_heading_err = heading_err

        # 3. Hysteresis short-range LiDAR collision avoidance (prevents left/right thrashing)
        min_front_arc = min(self.front_dist, self.front_left_dist, self.front_right_dist)

        if min_front_arc < 0.44:
            # Lock turn direction once when entering avoidance until front arc clears > 0.50m
            if self.avoid_dir is None:
                left_clearance = self.front_left_dist + 0.7 * self.left_dist
                right_clearance = self.front_right_dist + 0.7 * self.right_dist
                self.avoid_dir = 1.0 if left_clearance >= right_clearance else -1.0

            if min_front_arc < 0.24:
                self.debug_state = "EMERGENCY_REV"
                self.send_velocity_command(-0.20, 1.15 * self.avoid_dir)
                return

            self.debug_state = "OBS_AVOIDANCE"
            severity = (0.44 - min_front_arc) / 0.20
            v = max(0.10, 0.34 * (1.0 - 0.65 * severity))
            w = self.avoid_dir * (0.85 + 0.50 * severity)
            self.prev_heading_err = heading_err
            self.send_velocity_command(v, w)
            return
        elif min_front_arc > 0.50:
            # Release locked avoidance direction only after clearing the obstacle
            self.avoid_dir = None

        # 4. Damped PD goal-seeking steering + gentle side-wall guard
        self.debug_state = goal_state
        d_err = normalize_angle(heading_err - self.prev_heading_err) / dt
        self.prev_heading_err = heading_err

        # Deadband of ~3 degrees (0.05 rad) prevents micro-wobble on straight runs
        if abs(heading_err) < 0.05:
            w = 0.0
        else:
            w = 1.45 * heading_err - 0.18 * d_err

        # Gentle side-wall repulsion
        if self.left_dist < 0.30:
            w -= 0.95 * (0.30 - self.left_dist) / 0.30
        if self.right_dist < 0.30:
            w += 0.95 * (0.30 - self.right_dist) / 0.30

        # Cap angular velocity to prevent overshoot oscillations
        w = max(-1.35, min(1.35, w))

        # Smooth linear velocity: pivot cleanly when angle error is large (> 45 deg)
        if abs(heading_err) > 0.80:
            v = 0.12
        else:
            alignment = max(0.0, math.cos(heading_err))
            v = 0.85 * (0.25 + 0.75 * alignment)
            if dist_to_target < 0.45 and abs(heading_err) > 0.35:
                v *= 0.45

        self.send_velocity_command(v, w)


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