#!/usr/bin/env python3
"""
===============================================================================
ROSGAME - Bot Royale Match Referee & Telemetry Evaluator (referee.py)
Machine Perception and Intelligent Robotics (MAPIR) - University of Malaga
===============================================================================

Goal:
    Acts as an impartial supervisor node for the teachers during multi-robot
    Bot Royale matches (4 to 6+ student teams competing simultaneously). It
    aggregates real-time gameplay events and robot telemetry without leaking
    internal simulation data to student nodes, providing live match standings
    and exporting post-match grading reports.

Subscribed Topics:
    - /game_events (std_msgs/msg/String):
        Discrete pipe-delimited events emitted by Bridge and CoppeliaSim's
        Event_Handler script:
          * REGISTER|<RobotN>|<username>|<color>
          * SKILL|<RobotN>|<Hammer|Shield>
          * HIT|<Attacker>|<Victim>|<HAMMER|BUMP>|<damage>
          * KILL|<Killer>|<Victim>|<HAMMER|BUMP>
          * BATT|<RobotN>|CRITICAL
    - /RobotN/scene_info (std_msgs/msg/String):
        Ground-truth telemetry per robot used to integrate distance travelled
        (meters), track remaining HP/battery, and measure survival duration.

Outputs:
    - Live ranked leaderboard printed to the console every 5 seconds.
    - Timestamped CSV & JSON grading reports saved on shutdown (Ctrl+C) to:
        ~/bot_royale_results/match_YYYYMMDD_HHMMSS.{csv,json}
===============================================================================
"""

import csv
import json
import math
import os
import time
from datetime import datetime

import rclpy
from rclpy.node import Node
from std_msgs.msg import String


class MatchRefereeNode(Node):
    def __init__(self):
        super().__init__('match_referee')

        # PARAMS
        self.declare_parameter('max_robots', 8)
        self.declare_parameter('output_dir', os.path.expanduser('~/bot_royale_results'))
        self.max_robots = self.get_parameter('max_robots').value
        self.output_dir = self.get_parameter('output_dir').value
        os.makedirs(self.output_dir, exist_ok=True)

        self.stats = {}
        self.scene_subs = []


        # Listen to game events
        self.event_sub = self.create_subscription(
            String, '/game_events', self.on_game_event, 50
        )

        # Telemetry streams from Scene_Manager for distance travelled & survival time
        for idx in range(1, self.max_robots + 1):
            alias = f"Robot{idx}"
            sub = self.create_subscription(
                String,
                f'/{alias}/scene_info',
                lambda msg, a=alias: self.on_scene_info(msg, a),
                10
            )
            self.scene_subs.append(sub)

        # TIMERS
        self.create_timer(5.0, self.print_leaderboard)
        self.get_logger().info(f"Match Referee ready! Reports will be saved in: {self.output_dir}")



    def ensure_player(self, alias: str) -> dict:
        """ If not registered, creaty temporally one """
        if alias not in self.stats:
            self.stats[alias] = {
                "robot_alias": alias,
                "username": alias,
                "color": "UNKNOWN",
                "registered": False,
                "skills_total": 0,
                "hammers_picked": 0,
                "shields_picked": 0,
                "hits_total": 0,
                "hammer_hits": 0,
                "bump_hits": 0,
                "damage_dealt": 0,
                "kills": 0,
                "alive": True,
                "health": 100,
                "battery": 100.0,
                "distance_m": 0.0,
                "survival_sec": 0.0,
                "last_x": None,
                "last_y": None,
                "spawn_time": None,
                "death_time": None,
            }
        return self.stats[alias]



    # ------------------------------------------------------------------
    # EVENT & TELEMETRY HANDLERS
    # ------------------------------------------------------------------
    def on_game_event(self, msg: String):
        parts = msg.data.strip().split('|')
        if not parts:
            return

        # Event types: REGISTER, SKILL, HIT, KILL, BATT
        etype = parts[0]

        if etype == "REGISTER" and len(parts) >= 4:
            _, alias, username, color = parts[:4]
            p = self.ensure_player(alias)
            first_time = not p["registered"]
            p["username"] = username
            p["color"] = color
            p["registered"] = True
            if p["spawn_time"] is None:
                p["spawn_time"] = time.time()
            if first_time:
                self.get_logger().info(f"[REGISTERED] {username} -> {alias} ({color})")

        elif etype == "SKILL" and len(parts) >= 3:
            _, alias, skill_name = parts[:3]
            p = self.ensure_player(alias)
            p["skills_total"] += 1
            if skill_name == "Hammer":
                p["hammers_picked"] += 1
            elif skill_name == "Shield":
                p["shields_picked"] += 1
            self.get_logger().info(f"[SKILL] {p['username']} ({p['color']}) captured {skill_name}")

        elif etype == "HIT" and len(parts) >= 5:
            _, attacker, victim, hit_type, dmg_str = parts[:5]
            p_att = self.ensure_player(attacker)
            p_vic = self.ensure_player(victim)
            dmg = int(float(dmg_str))

            p_att["hits_total"] += 1
            p_att["damage_dealt"] += dmg
            if hit_type == "HAMMER":
                p_att["hammer_hits"] += 1
            else:
                p_att["bump_hits"] += 1

            self.get_logger().info(
                f"[HIT] {p_att['username']} ({hit_type}, -{dmg} HP) -> {p_vic['username']}"
            )

        elif etype == "KILL" and len(parts) >= 4:
            _, killer, victim, hit_type = parts[:4]
            p_kill = self.ensure_player(killer)
            p_vic = self.ensure_player(victim)
            p_kill["kills"] += 1
            p_vic["alive"] = False
            if p_vic["death_time"] is None:
                p_vic["death_time"] = time.time()

            self.get_logger().warning(
                f"[KILL] {p_kill['username']} ({p_kill['color']}) eliminated "
                f"{p_vic['username']} ({p_vic['color']}) via {hit_type}!"
            )

    def on_scene_info(self, msg: String, alias: str):
        try:
            data = json.loads(msg.data)
        except json.JSONDecodeError:
            return

        p = self.ensure_player(alias)
        if p["spawn_time"] is None:
            p["spawn_time"] = time.time()

        hp = int(data.get("Health", 100))
        bat = float(data.get("Battery_Level", 100.0))
        pose = data.get("Robot_Pose", {})
        x = float(pose.get("x", 0.0))
        y = float(pose.get("y", 0.0))

        # Accumulate distance travelled while alive (ignore spawn jumps > 1.5m)
        if p["last_x"] is not None and hp > 0:
            step_dist = math.hypot(x - p["last_x"], y - p["last_y"])
            if step_dist < 1.5:
                p["distance_m"] += step_dist

        p["last_x"] = x
        p["last_y"] = y
        p["health"] = hp
        p["battery"] = bat

        if hp <= 0 and p["alive"]:
            p["alive"] = False
            if p["death_time"] is None:
                p["death_time"] = time.time()

        end_t = p["death_time"] if p["death_time"] is not None else time.time()
        p["survival_sec"] = max(0.0, end_t - p["spawn_time"])

    # ------------------------------------------------------------------
    # SCORING & REPORT EXPORT
    # ------------------------------------------------------------------
    def compute_score(self, p: dict) -> float:
        """
        Grading rubric weights (adjust to match your course rubric):
          +100 pts per elimination (Kill)
          +15  pts per hit landed
          +10  pts per skill captured
          +2   pts per meter travelled (capped at 50m -> 100 pts)
          +50  pts survivor bonus if alive at match end
        """
        dist_pts = min(100.0, p["distance_m"] * 2.0)
        alive_bonus = 50.0 if p["alive"] else 0.0
        return round(
            p["kills"] * 100.0
            + p["hits_total"] * 15.0
            + p["skills_total"] * 10.0
            + dist_pts
            + alive_bonus,
            1
        )

    def get_ranked_players(self) -> list:
        active = [
            p for p in self.stats.values()
            if p["registered"] or p["distance_m"] > 0.05 or p["skills_total"] > 0
        ]
        for p in active:
            p["score"] = self.compute_score(p)
        return sorted(
            active,
            key=lambda p: (p["score"], p["kills"], p["damage_dealt"], p["survival_sec"]),
            reverse=True
        )

    def print_leaderboard(self):
        ranked = self.get_ranked_players()
        if not ranked:
            return

        header = (
            f"\n{'RANK':<5} {'TEAM / USER':<22} {'COLOR':<8} {'STATUS':<7} "
            f"{'KILLS':<6} {'HITS':<6} {'SKILLS':<7} {'DIST(m)':<8} {'SCORE':<7}"
        )
        sep = "-" * len(header)
        rows = [header, sep]

        for idx, p in enumerate(ranked, start=1):
            status = f"{p['health']}HP" if p["alive"] else "DEAD"
            rows.append(
                f"#{idx:<4} {p['username']:<22} {p['color']:<8} {status:<7} "
                f"{p['kills']:<6} {p['hits_total']:<6} {p['skills_total']:<7} "
                f"{p['distance_m']:<8.2f} {p['score']:<7.1f}"
            )
        rows.append(sep)
        self.get_logger().info("\n".join(rows))



    def save_reports(self):
        ranked = self.get_ranked_players()
        if not ranked:
            print("No player telemetry recorded; skipping report export.")
            return

        timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
        csv_path = os.path.join(self.output_dir, f"match_{timestamp}.csv")
        json_path = os.path.join(self.output_dir, f"match_{timestamp}.json")

        fields = [
            "rank", "username", "robot_alias", "color", "score", "alive", "health",
            "kills", "hits_total", "hammer_hits", "bump_hits", "damage_dealt",
            "skills_total", "hammers_picked", "shields_picked",
            "distance_m", "survival_sec"
        ]

        with open(csv_path, "w", newline="", encoding="utf-8") as f:
            writer = csv.DictWriter(f, fieldnames=fields, extrasaction="ignore")
            writer.writeheader()
            for idx, p in enumerate(ranked, start=1):
                row = dict(p)
                row["rank"] = idx
                row["distance_m"] = round(row["distance_m"], 2)
                row["survival_sec"] = round(row["survival_sec"], 1)
                writer.writerow(row)

        with open(json_path, "w", encoding="utf-8") as f:
            json.dump(ranked, f, indent=2)

        print(f"Final CSV saved to:  {csv_path}")
        print(f"Final JSON saved to: {json_path}")


def main(args=None):
    rclpy.init(args=args)
    node = MatchRefereeNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.print_leaderboard()
        node.save_reports()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()