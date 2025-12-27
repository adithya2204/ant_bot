#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from sensor_msgs.msg import Imu
from transforms3d.euler import quat2euler
import math
import random

class AntNavigator(Node):

    def __init__(self):
        super().__init__('ant_navigator')

        # ---------------- PUBS/SUBS ----------------
        self.cmd_pub = self.create_publisher(String, '/ant_bot/gait_mode', 10)
        self.imu_sub = self.create_subscription(Imu, '/ant_bot/imu', self.imu_callback, 10)

        # ---------------- PATH INTEGRATION STATE ----------------
        self.current_yaw = 0.0
        self.position_x = 0.0
        self.position_y = 0.0
        
        # SPEED CALIBRATION (0.11 m/s)
        self.ROBOT_SPEED = 0.11  

        # ---------------- MISSION PARAMETERS ----------------
        self.state = "IDLE" 
        self.target_yaw = 0.0
        
        self.mission_start_time = None
        self.segment_start_time = 0.0
        self.segment_duration = 0.0
        
        # 90 Seconds Total Mission
        self.SEARCH_TIME_LIMIT = 90.0 

        # RE-CHECK TIMER
        self.last_recheck_time = 0.0
        self.RECHECK_INTERVAL = 4.0 

        # ---------------- LOGIC LOOP (10Hz) ----------------
        self.timer = self.create_timer(0.1, self.game_loop)
        self.last_time = self.get_clock().now()

        self.get_logger().info("Ant Navigator Loaded. Waiting for mission start...")

    def imu_callback(self, msg):
        q = msg.orientation
        _, _, yaw = quat2euler([q.w, q.x, q.y, q.z])
        self.current_yaw = yaw

    def game_loop(self):
        now = self.get_clock().now()
        dt = (now - self.last_time).nanoseconds / 1e9
        self.last_time = now
        now_sec = now.nanoseconds / 1e9

        # 1. PATH INTEGRATION (Odometer)
        if self.state in ["WALK_RANDOM", "WALK_HOME"]:
            self.position_x += self.ROBOT_SPEED * math.cos(self.current_yaw) * dt
            self.position_y += self.ROBOT_SPEED * math.sin(self.current_yaw) * dt

        # 2. STATE MACHINE
        if self.state == "IDLE":
            if now_sec > 5.0: 
                self.get_logger().info("Mission Start: Foraging for 90 seconds...")
                self.mission_start_time = now
                self.start_random_exploration()

        elif self.state == "TURN_RANDOM":
            self.monitor_turn("WALK_RANDOM")

        elif self.state == "WALK_RANDOM":
            time_walking = now_sec - self.segment_start_time
            
            if time_walking > self.segment_duration:
                total_elapsed = (now - self.mission_start_time).nanoseconds / 1e9
                if total_elapsed > self.SEARCH_TIME_LIMIT:
                    self.get_logger().info(f"FOOD FOUND! Homing...")
                    self.start_homing()
                else:
                    self.start_random_exploration()

        elif self.state == "TURN_HOME":
            self.monitor_turn("WALK_HOME")

        elif self.state == "WALK_HOME":
            dist = math.sqrt(self.position_x**2 + self.position_y**2)
            
            # A. RE-CHECK ALIGNMENT (Every 4 seconds)
            if (now_sec - self.last_recheck_time) > self.RECHECK_INTERVAL:
                self.last_recheck_time = now_sec
                correct_yaw = math.atan2(-self.position_y, -self.position_x)
                
                # Check error magnitude
                error = abs(correct_yaw - self.current_yaw)
                if error > math.pi: error = 2*math.pi - error
                
                # If error > 10 degrees, force a stop and turn
                if error > 0.17:
                    self.get_logger().info(f"Drift Detected ({error:.2f} rad). Re-aligning...")
                    self.target_yaw = correct_yaw
                    self.state = "TURN_HOME"
                    # NOTE: We don't call decide_turn_direction here anymore.
                    # monitor_turn handles it dynamically now.
                    return 

            # B. LOGGING
            if int(now_sec) % 2 == 0:
                self.get_logger().info(f"Homing... Dist: {dist:.2f}m")

            # C. STOP CONDITION
            if dist < 0.05:  
                self.cmd_pub.publish(String(data="STAND"))
                self.state = "FINISHED"
                self.get_logger().info("ARRIVED AT NEST. Mission Complete.")

    # ---------------- BEHAVIORS ----------------

    def start_random_exploration(self):
        self.target_yaw = random.uniform(-math.pi, math.pi)
        self.segment_duration = random.uniform(20.0, 30.0) 
        self.state = "TURN_RANDOM"
        self.get_logger().info(f"Searching... New Heading: {self.target_yaw:.2f} rad")

    def start_homing(self):
        self.get_logger().info(f"CALCULATING HOME VECTOR from ({self.position_x:.2f}, {self.position_y:.2f})")
        self.target_yaw = math.atan2(-self.position_y, -self.position_x)
        self.state = "TURN_HOME"
        self.last_recheck_time = self.get_clock().now().nanoseconds / 1e9

    def monitor_turn(self, next_state):
        """
        Active Steering: Calculates the shortest turn direction continuously.
        This ensures both LEFT and RIGHT re-alignment work correctly.
        """
        # 1. Calculate Error (Signed)
        diff = self.target_yaw - self.current_yaw
        
        # 2. Normalize to -pi to +pi
        while diff > math.pi: diff -= 2*math.pi
        while diff < -math.pi: diff += 2*math.pi
        
        # 3. Check Magnitude (Are we there yet?)
        error_mag = abs(diff)
        
        if error_mag < 0.08: # ~4.5 degrees tolerance
            self.cmd_pub.publish(String(data="FORWARD"))
            self.segment_start_time = self.get_clock().now().nanoseconds / 1e9
            self.state = next_state
            self.get_logger().info(f"Heading Locked. Walking {next_state}...")
        else:
            # 4. ACTIVE SWITCHING (The Fix)
            # If diff is positive, Target is to the LEFT.
            # If diff is negative, Target is to the RIGHT.
            if diff > 0:
                self.cmd_pub.publish(String(data="LEFT"))
            else:
                self.cmd_pub.publish(String(data="RIGHT"))

def main(args=None):
    rclpy.init(args=args)
    node = AntNavigator()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()