#!/usr/bin/env python3
import math
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from std_msgs.msg import Float64MultiArray
from sensor_msgs.msg import Imu
from transforms3d.euler import quat2euler

class GaitController(Node):

    def __init__(self):
        super().__init__('gait_controller')

        # ---------------- ROS ----------------
        self.cmd_sub = self.create_subscription(
            String, '/ant_bot/gait_mode', self.cmd_callback, 10
        )

        self.imu_sub = self.create_subscription(
            Imu, '/ant_bot/imu', self.imu_callback, 10
        )

        self.cmd_pub = self.create_publisher(
            Float64MultiArray, '/forward_position_controller/commands', 10
        )

        # 100Hz control loop
        self.control_period = 0.01
        self.timer = self.create_timer(self.control_period, self.update)

        # ---------------- Gait State ----------------
        self.current_mode = "STAND"
        self.support_state = "A"
        self.swing_phase = 0.0

        # ---------------- SPIN 360 LOGIC ----------------
        self.return_mode = "STAND"
        self.total_turned = 0.0        
        self.last_imu_yaw = 0.0        
        # Target ~358 degrees (6.25 rad). 
        # Since we fixed the "vibration bug", this will now be accurate.
        self.TARGET_SPIN_RAD = 6.25    

        # ---------------- SOFT START ----------------
        self.transition_start_time = None
        self.RAMP_DURATION = 1.0

        # ---------------- YAW CONTROL (PID) ----------------
        self.current_yaw = 0.0
        self.target_yaw = 0.0
        self.yaw_initialized = False
        
        # PID: Stiff & Responsive
        self.YAW_KP = 2.0   
        self.YAW_KD = 0.15   
        self.YAW_KI = 0.0   
        
        self.previous_yaw_error = 0.0
        self.yaw_error_integral = 0.0

        # ---------------- MOTION PARAMETERS ----------------
        self.SWEEP_AMP = 0.45      
        self.TURN_AMP = 0.55       
        self.SWING_TIME = 0.22     
        self.STANCE_PUSH_FACTOR = 1.6
        
        self.GROUND_FEMUR = -1.45 
        self.GROUND_TIBIA = -0.7 
        self.LIFT_FEMUR = -1.0
        self.LIFT_TIBIA = -0.35
        self.MAX_LIFT_HEIGHT = 0.15 

        self.LEFT_FORCE_SCALE = 1.0   
        self.RIGHT_FORCE_SCALE = 1.0

        self.get_logger().info("Gait Controller: SPIN360 (Vibration Fix Applied)")

    def cmd_callback(self, msg):
        new_mode = msg.data
        
        # Handle SPIN360 trigger
        if new_mode == "SPIN360":
            if self.current_mode != "SPIN360":
                if self.current_mode in ["FORWARD", "BACKWARD"]:
                    self.return_mode = self.current_mode
                else:
                    self.return_mode = "FORWARD"
                
                # Reset Turn Accumulators
                self.total_turned = 0.0
                self.last_imu_yaw = self.current_yaw
                
                self.current_mode = "SPIN360"
                self.transition_start_time = self.get_clock().now()
                self.get_logger().info(f"Spinning! Will return to: {self.return_mode}")
                
        elif new_mode != self.current_mode:
            self.current_mode = new_mode
            self.transition_start_time = self.get_clock().now()
            
            if self.current_mode in ["FORWARD", "BACKWARD"] and self.yaw_initialized:
                self.target_yaw = self.current_yaw
                self.yaw_error_integral = 0.0
                self.get_logger().info(f"Heading Locked: {self.target_yaw:.3f}")

    def imu_callback(self, msg):
        q = msg.orientation
        roll, pitch, yaw = quat2euler([q.w, q.x, q.y, q.z])
        self.current_yaw = yaw
        
        if not self.yaw_initialized:
            self.target_yaw = yaw
            self.last_imu_yaw = yaw
            self.yaw_initialized = True

    def check_spin_progress(self):
        """Calculates true net rotation, ignoring vibration"""
        if self.current_mode != "SPIN360":
            return

        # Calculate smallest difference (handling wrap-around)
        diff = self.current_yaw - self.last_imu_yaw
        if diff > math.pi:
            diff -= 2 * math.pi
        elif diff < -math.pi:
            diff += 2 * math.pi
            
        # FIX: Sum the SIGNED difference (diff), not absolute (abs(diff)).
        # This allows backward vibrations to cancel out forward vibrations.
        self.total_turned += diff
        self.last_imu_yaw = self.current_yaw

        # Check completion using absolute value of the total sum
        if abs(self.total_turned) >= self.TARGET_SPIN_RAD:
            self.get_logger().info("360 Spin Complete! Resuming motion.")
            
            self.current_mode = self.return_mode
            self.transition_start_time = self.get_clock().now()
            self.target_yaw = self.current_yaw 
            self.yaw_error_integral = 0.0

    def update(self):
        self.check_spin_progress()

        if self.current_mode == "STAND":
            joints = self.stand_pose()
        else:
            joints = self.tripod_gait()

        self.cmd_pub.publish(Float64MultiArray(data=joints))

    def stand_pose(self):
        joints = []
        for _ in range(6):
            joints += [0.0, self.GROUND_FEMUR, self.GROUND_TIBIA]
        return joints

    def calculate_yaw_correction(self):
        if not self.yaw_initialized: return 0.0
        
        if self.current_mode not in ["FORWARD", "BACKWARD"]:
            return 0.0
        
        yaw_error = self.target_yaw - self.current_yaw
        while yaw_error > math.pi: yaw_error -= 2 * math.pi
        while yaw_error < -math.pi: yaw_error += 2 * math.pi
        
        yaw_correction = (
            self.YAW_KP * yaw_error +
            self.YAW_KD * (yaw_error - self.previous_yaw_error) / self.control_period
        )
        self.previous_yaw_error = yaw_error
        
        return max(-0.8, min(0.8, yaw_correction))

    def tripod_gait(self):
        joints = []

        # RAMP
        ramp = 1.0
        if self.transition_start_time is not None:
            now = self.get_clock().now()
            elapsed = (now - self.transition_start_time).nanoseconds / 1e9
            if elapsed < self.RAMP_DURATION:
                ramp = elapsed / self.RAMP_DURATION
            else:
                ramp = 1.0
        
        # TARGETS
        drive_target = 0.0
        turn_target = 0.0

        if self.current_mode == "FORWARD":
            drive_target = self.SWEEP_AMP
        elif self.current_mode == "BACKWARD":
            drive_target = -self.SWEEP_AMP
        elif self.current_mode == "LEFT":
            turn_target = self.TURN_AMP  
        elif self.current_mode == "RIGHT":
            turn_target = -self.TURN_AMP 
        elif self.current_mode == "SPIN360":
            turn_target = self.TURN_AMP # Spin Left

        current_drive = drive_target * ramp
        current_turn = turn_target * ramp
        current_lift = self.MAX_LIFT_HEIGHT * ramp

        # PHASE
        self.swing_phase += self.control_period / self.SWING_TIME
        if self.swing_phase >= 1.0:
            self.swing_phase = 0.0
            self.support_state = "B" if self.support_state == "A" else "A"

        # PID
        yaw_pid_val = self.calculate_yaw_correction()

        tripod_A = [0, 2, 4]
        tripod_B = [1, 3, 5]

        for leg in range(6):
            is_stance = (
                (self.support_state == "A" and leg in tripod_A) or
                (self.support_state == "B" and leg in tripod_B)
            )

            t = self.swing_phase
            is_left_side = (leg < 3)

            if is_stance:
                t_eased = self.ease_in_out_cubic(t)
                wave_val = (0.5 - t_eased) * 2.0 * self.STANCE_PUSH_FACTOR
                
                drive_component = current_drive * wave_val
                turn_component = current_turn * wave_val
                
                total_turn = turn_component + (yaw_pid_val * abs(current_drive))

                if is_left_side:
                    sweep = drive_component - total_turn
                else:
                    sweep = -drive_component - total_turn

                sweep *= (self.LEFT_FORCE_SCALE if is_left_side else self.RIGHT_FORCE_SCALE)
                
                femur = self.GROUND_FEMUR
                tibia = self.GROUND_TIBIA
                
            else:
                if t < 0.35:
                    lift = self.ease_out_quad(t / 0.35) * current_lift
                elif t < 0.75:
                    lift = current_lift * (1.0 - 0.3 * (t - 0.35) / 0.4)
                else:
                    lift = self.ease_in_quad((1 - t) / 0.25) * current_lift * 0.7
                
                t_forward = self.ease_in_out_cubic(t)
                wave_val = -(0.5 - t_forward) * 2.0
                
                drive_component = current_drive * wave_val
                turn_component = current_turn * wave_val
                total_turn = turn_component 
                
                if is_left_side:
                    sweep = drive_component - total_turn
                else:
                    sweep = -drive_component - total_turn
                
                sweep *= (self.LEFT_FORCE_SCALE if is_left_side else self.RIGHT_FORCE_SCALE)
                
                if current_lift > 0.001:
                    lift_norm = lift / current_lift
                else:
                    lift_norm = 0.0

                femur = self.GROUND_FEMUR + lift_norm * (self.LIFT_FEMUR - self.GROUND_FEMUR)
                tibia = self.GROUND_TIBIA + lift_norm * (self.LIFT_TIBIA - self.GROUND_TIBIA)

            joints += [sweep, femur, tibia]

        return joints

    def ease_in_out_cubic(self, t):
        if t < 0.5: return 4 * t * t * t
        else: return 1 - pow(-2 * t + 2, 3) / 2

    def ease_out_quad(self, t): return 1 - (1 - t) * (1 - t)
    def ease_in_quad(self, t): return t * t

def main():
    rclpy.init()
    node = GaitController()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()