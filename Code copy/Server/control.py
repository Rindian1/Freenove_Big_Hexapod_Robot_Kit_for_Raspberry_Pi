# -*- coding: utf-8 -*-
"""
Control module for Freenove Big Hexapod Robot Kit

This module handles the core control system for the hexapod robot, including
kinematics, gait patterns, balance control, and movement coordination.
"""

import time
import math
import copy
import threading
import numpy as np
from typing import List, Tuple, Dict, Any, Optional
from gpiozero import OutputDevice

from pid import Incremental_PID
from command import COMMAND as cmd
from imu import IMU
from servo import Servo


class Control:
    """
    Main control class for hexapod robot movement and coordination.
    
    Handles inverse kinematics, gait patterns, balance control, and
    real-time movement coordination for the 6-legged robot.
    """
    
    # Robot physical parameters
    LEG_SEGMENTS = {
        'L1': 33,   # Hip segment length (mm)
        'L2': 90,   # Thigh segment length (mm)
        'L3': 110   # Shin segment length (mm)
    }
    
    # Body dimensions
    BODY_RADIUS = 85  # Distance from center to leg attachment (mm)
    LEG_SPACING = 54  # Angle between legs (degrees)
    
    # Movement constraints
    MAX_LEG_LENGTH = 248  # Maximum leg reach (mm)
    MIN_LEG_LENGTH = 90   # Minimum leg reach (mm)
    MAX_BODY_HEIGHT = 20  # Maximum body height adjustment (mm)
    MIN_BODY_HEIGHT = -20 # Minimum body height adjustment (mm)
    
    # Gait parameters
    DEFAULT_STEP_HEIGHT = 40  # Step height for walking (mm)
    DEFAULT_GAIT_CYCLES = 64  # Default gait cycle steps
    
    def __init__(self):
        """Initialize the control system with all components."""
        # Hardware components
        self.imu = IMU()
        self.servo = Servo()
        self.servo_power_disable = OutputDevice(4)
        self.servo_power_disable.off()
        
        # Control parameters
        self.movement_flag = 0x01
        self.relaxation_flag = False
        self.status_flag = 0x00
        self.timeout = 0
        
        # PID controller for balance
        self.pid_controller = Incremental_PID(0.500, 0.00, 0.0025)
        
        # Body position and orientation
        self.body_height = -25
        self.body_points = self._initialize_body_points()
        
        # Leg positions and angles
        self.calibration_leg_positions = self.read_from_txt('point')
        self.leg_positions = [[140, 0, 0] for _ in range(6)]
        self.calibration_angles = [[0, 0, 0] for _ in range(6)]
        self.current_angles = [[90, 0, 0] for _ in range(6)]
        
        # Command processing
        self.command_queue = [''] * 6
        self.command_lock = threading.Lock()
        
        # Threading
        self.condition_thread = threading.Thread(target=self.condition_monitor, daemon=True)
        self.thread_condition = threading.Condition()
        
        # Initialize system
        self.calibrate()
        self.set_leg_angles()
        self.condition_thread.start()
        
        print("Hexapod control system initialized")
    
    def _initialize_body_points(self) -> List[List[float]]:
        """
        Initialize the default body attachment points for each leg.
        
        Returns:
            List[List[float]]: Body points for each leg [x, y, z]
        """
        points = []
        for i in range(6):
            angle = i * 60  # 60 degrees between each leg
            x = self.BODY_RADIUS * math.cos(math.radians(angle))
            y = self.BODY_RADIUS * math.sin(math.radians(angle))
            z = self.body_height
            points.append([x, y, z])
        return points
    
    def read_from_txt(self, filename: str) -> List[List[int]]:
        """
        Read calibration data from text file.
        
        Args:
            filename (str): Base filename without extension
            
        Returns:
            List[List[int]]: Calibration data
        """
        try:
            with open(filename + ".txt", "r") as file:
                lines = file.readlines()
                data = [list(map(int, line.strip().split("\t"))) for line in lines]
            return data
        except FileNotFoundError:
            print(f"Calibration file {filename}.txt not found, using defaults")
            return [[0, 0, 0] for _ in range(6)]
        except Exception as e:
            print(f"Error reading calibration file: {e}")
            return [[0, 0, 0] for _ in range(6)]
    
    def save_to_txt(self, data: List[List[int]], filename: str) -> bool:
        """
        Save calibration data to text file.
        
        Args:
            data (List[List[int]]): Data to save
            filename (str): Base filename without extension
            
        Returns:
            bool: True if successful
        """
        try:
            with open(filename + '.txt', 'w') as file:
                for row in data:
                    file.write('\t'.join(map(str, row)) + '\n')
            print(f"Calibration data saved to {filename}.txt")
            return True
        except Exception as e:
            print(f"Error saving calibration data: {e}")
            return False
    
    def coordinate_to_angle(self, x: float, y: float, z: float, 
                          l1: float = None, l2: float = None, l3: float = None) -> Tuple[float, float, float]:
        """
        Convert 3D coordinates to servo angles using inverse kinematics.
        
        Args:
            x, y, z (float): Target coordinates
            l1, l2, l3 (float): Leg segment lengths
            
        Returns:
            Tuple[float, float, float]: Servo angles (hip, thigh, shin) in degrees
        """
        # Use default leg segments if not specified
        l1 = l1 or self.LEG_SEGMENTS['L1']
        l2 = l2 or self.LEG_SEGMENTS['L2']
        l3 = l3 or self.LEG_SEGMENTS['L3']
        
        # Calculate hip angle (rotation around Z-axis)
        a = math.pi / 2 - math.atan2(z, y)
        
        # Calculate intermediate positions
        x_3 = 0
        x_4 = l1 * math.sin(a)
        x_5 = l1 * math.cos(a)
        
        # Calculate distance from hip to target
        l23 = math.sqrt((z - x_5) ** 2 + (y - x_4) ** 2 + (x - x_3) ** 2)
        
        # Calculate intermediate angles
        w = self.restrict_value((x - x_3) / l23, -1, 1)
        v = self.restrict_value((l2 * l2 + l23 * l23 - l3 * l3) / (2 * l2 * l23), -1, 1)
        u = self.restrict_value((l2 ** 2 + l3 ** 2 - l23 ** 2) / (2 * l3 * l2), -1, 1)
        
        # Calculate final angles
        b = math.asin(round(w, 2)) - math.acos(round(v, 2))
        c = math.pi - math.acos(round(u, 2))
        
        return (round(math.degrees(a)), round(math.degrees(b)), round(math.degrees(c)))
    
    def angle_to_coordinate(self, a: float, b: float, c: float,
                          l1: float = None, l2: float = None, l3: float = None) -> Tuple[float, float, float]:
        """
        Convert servo angles to 3D coordinates using forward kinematics.
        
        Args:
            a, b, c (float): Servo angles in degrees
            l1, l2, l3 (float): Leg segment lengths
            
        Returns:
            Tuple[float, float, float]: 3D coordinates (x, y, z)
        """
        # Use default leg segments if not specified
        l1 = l1 or self.LEG_SEGMENTS['L1']
        l2 = l2 or self.LEG_SEGMENTS['L2']
        l3 = l3 or self.LEG_SEGMENTS['L3']
        
        # Convert to radians
        a_rad = math.radians(a)
        b_rad = math.radians(b)
        c_rad = math.radians(c)
        
        # Calculate coordinates
        x = round(l3 * math.sin(b_rad + c_rad) + l2 * math.sin(b_rad))
        y = round(l3 * math.sin(a_rad) * math.cos(b_rad + c_rad) + l2 * math.sin(a_rad) * math.cos(b_rad) + l1 * math.sin(a_rad))
        z = round(l3 * math.cos(a_rad) * math.cos(b_rad + c_rad) + l2 * math.cos(a_rad) * math.cos(b_rad) + l1 * math.cos(a_rad))
        
        return (x, y, z)
    
    def calibrate(self) -> None:
        """Calibrate the leg positions and calculate calibration angles."""
        # Reset leg positions to default
        self.leg_positions = [[140, 0, 0] for _ in range(6)]
        
        # Calculate calibration angles for each leg
        for i in range(6):
            # Calculate target angles from calibration positions
            target_angles = self.coordinate_to_angle(
                -self.calibration_leg_positions[i][2],
                self.calibration_leg_positions[i][0],
                self.calibration_leg_positions[i][1]
            )
            
            # Calculate current angles from default positions
            current_angles = self.coordinate_to_angle(
                -self.leg_positions[i][2],
                self.leg_positions[i][0],
                self.leg_positions[i][1]
            )
            
            # Calculate calibration offsets
            self.calibration_angles[i] = [
                target_angles[0] - current_angles[0],
                target_angles[1] - current_angles[1],
                target_angles[2] - current_angles[2]
            ]
        
        print("Leg calibration completed")
    
    def set_leg_angles(self) -> bool:
        """
        Calculate and set servo angles for all legs.
        
        Returns:
            bool: True if successful, False if coordinates are invalid
        """
        if not self.check_point_validity():
            print("Warning: Leg coordinates are out of valid range")
            return False
        
        # Calculate angles for each leg
        for i in range(6):
            self.current_angles[i] = self.coordinate_to_angle(
                -self.leg_positions[i][2],
                self.leg_positions[i][0],
                self.leg_positions[i][1]
            )
        
        # Apply calibration offsets and set servo angles
        servo_mapping = [
            # Leg 1: servos 15, 14, 13
            (15, 14, 13, 0),
            # Leg 2: servos 12, 11, 10
            (12, 11, 10, 1),
            # Leg 3: servos 9, 8, 31
            (9, 8, 31, 2),
            # Leg 4: servos 22, 23, 27
            (22, 23, 27, 3),
            # Leg 5: servos 19, 20, 21
            (19, 20, 21, 4),
            # Leg 6: servos 16, 17, 18
            (16, 17, 18, 5)
        ]
        
        for hip_servo, thigh_servo, shin_servo, leg_idx in servo_mapping:
            # Apply calibration offsets
            hip_angle = self.restrict_value(
                self.current_angles[leg_idx][0] + self.calibration_angles[leg_idx][0], 0, 180
            )
            
            if leg_idx < 3:  # Front legs
                thigh_angle = self.restrict_value(
                    90 - (self.current_angles[leg_idx][1] + self.calibration_angles[leg_idx][1]), 0, 180
                )
                shin_angle = self.restrict_value(
                    self.current_angles[leg_idx][2] + self.calibration_angles[leg_idx][2], 0, 180
                )
            else:  # Back legs
                thigh_angle = self.restrict_value(
                    90 + self.current_angles[leg_idx][1] + self.calibration_angles[leg_idx][1], 0, 180
                )
                shin_angle = self.restrict_value(
                    180 - (self.current_angles[leg_idx][2] + self.calibration_angles[leg_idx][2]), 0, 180
                )
            
            # Set servo angles
            self.servo.set_servo_angle(hip_servo, hip_angle)
            self.servo.set_servo_angle(thigh_servo, thigh_angle)
            self.servo.set_servo_angle(shin_servo, shin_angle)
        
        return True
    
    def check_point_validity(self) -> bool:
        """
        Check if all leg positions are within valid range.
        
        Returns:
            bool: True if all positions are valid
        """
        for i in range(6):
            leg_length = math.sqrt(
                self.leg_positions[i][0] ** 2 + 
                self.leg_positions[i][1] ** 2 + 
                self.leg_positions[i][2] ** 2
            )
            if leg_length > self.MAX_LEG_LENGTH or leg_length < self.MIN_LEG_LENGTH:
                return False
        return True
    
    def condition_monitor(self) -> None:
        """Main control loop for processing commands and maintaining robot state."""
        while True:
            try:
                # Check for timeout and relax servos if needed
                if (time.time() - self.timeout) > 10 and self.timeout != 0 and self.command_queue[0] == '':
                    self.timeout = time.time()
                    self.relax(True)
                    self.status_flag = 0x00
                
                # Process position commands
                if cmd.CMD_POSITION in self.command_queue and len(self.command_queue) == 4:
                    if self.status_flag != 0x01:
                        self.relax(False)
                    
                    x = self.restrict_value(int(self.command_queue[1]), -40, 40)
                    y = self.restrict_value(int(self.command_queue[2]), -40, 40)
                    z = self.restrict_value(int(self.command_queue[3]), -20, 20)
                    
                    self.move_position(x, y, z)
                    self.status_flag = 0x01
                    self.command_queue = [''] * 6
                
                # Process attitude commands
                elif cmd.CMD_ATTITUDE in self.command_queue and len(self.command_queue) == 4:
                    if self.status_flag != 0x02:
                        self.relax(False)
                    
                    roll = self.restrict_value(int(self.command_queue[1]), -15, 15)
                    pitch = self.restrict_value(int(self.command_queue[2]), -15, 15)
                    yaw = self.restrict_value(int(self.command_queue[3]), -15, 15)
                    
                    points = self.calculate_posture_balance(roll, pitch, yaw)
                    self.transform_coordinates(points)
                    self.set_leg_angles()
                    self.status_flag = 0x02
                    self.command_queue = [''] * 6
                
                # Process movement commands
                elif cmd.CMD_MOVE in self.command_queue and len(self.command_queue) == 6:
                    if self.command_queue[2] == "0" and self.command_queue[3] == "0":
                        self.run_gait(self.command_queue)
                        self.command_queue = [''] * 6
                    else:
                        if self.status_flag != 0x03:
                            self.relax(False)
                        self.run_gait(self.command_queue)
                        self.status_flag = 0x03
                
                # Process balance commands
                elif cmd.CMD_BALANCE in self.command_queue and len(self.command_queue) == 2:
                    if self.command_queue[1] == "1":
                        self.command_queue = [''] * 6
                        if self.status_flag != 0x04:
                            self.relax(False)
                        self.status_flag = 0x04
                        self.imu6050()
                
                # Process calibration commands
                elif cmd.CMD_CALIBRATION in self.command_queue:
                    self.timeout = 0
                    self.calibrate()
                    self.set_leg_angles()
                    
                    if len(self.command_queue) >= 2:
                        self._process_calibration_command()
                    
                    self.command_queue = [''] * 6
                
                time.sleep(0.01)  # Small delay to prevent busy waiting
                
            except Exception as e:
                print(f"Error in condition monitor: {e}")
                time.sleep(0.1)
    
    def _process_calibration_command(self) -> None:
        """Process individual leg calibration commands."""
        leg_mapping = {
            "one": 0, "two": 1, "three": 2,
            "four": 3, "five": 4, "six": 5
        }
        
        leg_name = self.command_queue[1]
        if leg_name in leg_mapping:
            leg_idx = leg_mapping[leg_name]
            if len(self.command_queue) >= 5:
                self.calibration_leg_positions[leg_idx] = [
                    int(self.command_queue[2]),
                    int(self.command_queue[3]),
                    int(self.command_queue[4])
                ]
                self.calibrate()
                self.set_leg_angles()
                print(f"Calibrated leg {leg_name}")
        
        elif leg_name == "save":
            self.save_to_txt(self.calibration_leg_positions, 'point')
    
    def relax(self, flag: bool) -> None:
        """
        Relax or activate all servos.
        
        Args:
            flag (bool): True to relax, False to activate
        """
        if flag:
            self.servo.relax()
            self.relaxation_flag = True
        else:
            self.set_leg_angles()
            self.relaxation_flag = False
    
    def transform_coordinates(self, points: List[List[float]]) -> None:
        """
        Transform body coordinates to leg coordinates.
        
        Args:
            points (List[List[float]]): Body points for each leg
        """
        leg_angles = [54, 0, -54, -126, 180, 126]  # Angle for each leg
        
        for i in range(6):
            angle_rad = math.radians(leg_angles[i])
            cos_a = math.cos(angle_rad)
            sin_a = math.sin(angle_rad)
            
            # Transform coordinates
            x = points[i][0] * cos_a + points[i][1] * sin_a - 94
            y = -points[i][0] * sin_a + points[i][1] * cos_a
            z = points[i][2] - 14
            
            self.leg_positions[i] = [x, y, z]
    
    def restrict_value(self, value: float, min_value: float, max_value: float) -> float:
        """
        Restrict a value to a specified range.
        
        Args:
            value (float): Input value
            min_value (float): Minimum allowed value
            max_value (float): Maximum allowed value
            
        Returns:
            float: Clamped value
        """
        return max(min_value, min(value, max_value))
    
    def map_value(self, value: float, from_low: float, from_high: float, 
                 to_low: float, to_high: float) -> float:
        """
        Map a value from one range to another.
        
        Args:
            value (float): Input value
            from_low, from_high (float): Source range
            to_low, to_high (float): Target range
            
        Returns:
            float: Mapped value
        """
        return (to_high - to_low) * (value - from_low) / (from_high - from_low) + to_low
    
    def move_position(self, x: float, y: float, z: float) -> None:
        """
        Move the robot body to a new position.
        
        Args:
            x, y, z (float): Target position coordinates
        """
        points = copy.deepcopy(self.body_points)
        
        for i in range(6):
            points[i][0] = self.body_points[i][0] - x
            points[i][1] = self.body_points[i][1] - y
            points[i][2] = -30 - z
            self.body_height = points[i][2]
            self.body_points[i][2] = points[i][2]
        
        self.transform_coordinates(points)
        self.set_leg_angles()
    
    def calculate_posture_balance(self, roll: float, pitch: float, yaw: float) -> List[List[float]]:
        """
        Calculate leg positions for body posture balance.
        
        Args:
            roll, pitch, yaw (float): Body orientation angles in degrees
            
        Returns:
            List[List[float]]: Calculated foot positions
        """
        # Convert to radians
        roll_rad = math.radians(roll)
        pitch_rad = math.radians(pitch)
        yaw_rad = math.radians(yaw)
        
        # Create rotation matrices
        rotation_x = np.array([
            [1, 0, 0],
            [0, math.cos(pitch_rad), -math.sin(pitch_rad)],
            [0, math.sin(pitch_rad), math.cos(pitch_rad)]
        ])
        
        rotation_y = np.array([
            [math.cos(roll_rad), 0, -math.sin(roll_rad)],
            [0, 1, 0],
            [math.sin(roll_rad), 0, math.cos(roll_rad)]
        ])
        
        rotation_z = np.array([
            [math.cos(yaw_rad), -math.sin(yaw_rad), 0],
            [math.sin(yaw_rad), math.cos(yaw_rad), 0],
            [0, 0, 1]
        ])
        
        # Combined rotation matrix
        rotation_matrix = rotation_x @ rotation_y @ rotation_z
        
        # Body structure points
        body_structure = np.array([
            [55, 76, 0], [85, 0, 0], [55, -76, 0],
            [-55, -76, 0], [-85, 0, 0], [-55, 76, 0]
        ]).T
        
        # Foot point structure
        footpoint_structure = np.array([
            [137.1, 189.4, 0], [225, 0, 0], [137.1, -189.4, 0],
            [-137.1, -189.4, 0], [-225, 0, 0], [-137.1, 189.4, 0]
        ]).T
        
        # Calculate foot positions
        position = np.array([0.0, 0.0, self.body_height])
        foot_positions = []
        
        for i in range(6):
            foot_point = position + rotation_matrix @ footpoint_structure[:, i]
            foot_positions.append([foot_point[0], foot_point[1], foot_point[2]])
        
        return foot_positions
    
    def imu6050(self) -> None:
        """IMU-based balance control loop."""
        # Initialize balance
        old_roll = 0
        old_pitch = 0
        points = self.calculate_posture_balance(0, 0, 0)
        self.transform_coordinates(points)
        self.set_leg_angles()
        time.sleep(2)
        
        # Calibrate IMU
        self.imu.Error_value_accel_data, self.imu.Error_value_gyro_data = self.imu.calculate_average_sensor_data()
        time.sleep(1)
        
        # Balance control loop
        while True:
            if self.command_queue[0] != "":
                break
            
            time.sleep(0.02)
            roll, pitch, yaw = self.imu.update_imu_state()
            
            # Apply PID control
            roll = self.pid_controller.pid_calculate(roll)
            pitch = self.pid_controller.pid_calculate(pitch)
            
            # Calculate new foot positions
            points = self.calculate_posture_balance(roll, pitch, 0)
            self.transform_coordinates(points)
            self.set_leg_angles()
    
    def run_gait(self, data: List[str], Z: int = None, F: int = None) -> None:
        """
        Execute walking gait pattern.
        
        Args:
            data (List[str]): Gait command data
            Z (int): Step height (mm)
            F (int): Gait cycles
        """
        Z = Z or self.DEFAULT_STEP_HEIGHT
        F = F or self.DEFAULT_GAIT_CYCLES
        
        gait = data[1]
        x = self.restrict_value(int(data[2]), -35, 35)
        y = self.restrict_value(int(data[3]), -35, 35)
        
        # Calculate gait frequency based on speed
        if gait == "1":
            F = round(self.map_value(int(data[4]), 2, 10, 126, 22))
        else:
            F = round(self.map_value(int(data[4]), 2, 10, 171, 45))
        
        angle = int(data[5])
        z = Z / F
        delay = 0.01
        
        # Initialize gait
        points = copy.deepcopy(self.body_points)
        xy = [[0, 0] for _ in range(6)]
        
        # Calculate movement per step
        for i in range(6):
            angle_rad = math.radians(angle)
            cos_a = math.cos(angle_rad)
            sin_a = math.sin(angle_rad)
            
            xy[i][0] = ((points[i][0] * cos_a + points[i][1] * sin_a - points[i][0]) + x) / F
            xy[i][1] = ((-points[i][0] * sin_a + points[i][1] * cos_a - points[i][1]) + y) / F
        
        # Stop movement if no translation or rotation
        if x == 0 and y == 0 and angle == 0:
            self.transform_coordinates(points)
            self.set_leg_angles()
            return
        
        # Execute gait pattern
        if gait == "1":
            self._execute_tripod_gait(points, xy, F, z, delay)
        elif gait == "2":
            self._execute_wave_gait(points, xy, F, z, delay)
    
    def _execute_tripod_gait(self, points: List[List[float]], xy: List[List[float]], 
                           F: int, z: float, delay: float) -> None:
        """Execute tripod gait pattern."""
        for j in range(F):
            for i in range(3):  # 3 tripods
                # Phase 1: Lift tripod 1
                if j < (F / 8):
                    points[2 * i][0] -= 4 * xy[2 * i][0]
                    points[2 * i][1] -= 4 * xy[2 * i][1]
                    points[2 * i + 1][0] += 8 * xy[2 * i + 1][0]
                    points[2 * i + 1][1] += 8 * xy[2 * i + 1][1]
                    points[2 * i + 1][2] = self.DEFAULT_STEP_HEIGHT + self.body_height
                
                # Phase 2: Lower tripod 1
                elif j < (F / 4):
                    points[2 * i][0] -= 4 * xy[2 * i][0]
                    points[2 * i][1] -= 4 * xy[2 * i][1]
                    points[2 * i + 1][2] -= z * 8
                
                # Phase 3: Lift tripod 2
                elif j < (3 * F / 8):
                    points[2 * i][2] += z * 8
                    points[2 * i + 1][0] -= 4 * xy[2 * i + 1][0]
                    points[2 * i + 1][1] -= 4 * xy[2 * i + 1][1]
                
                # Continue with remaining phases...
                # (Additional phases would be implemented here)
            
            self.transform_coordinates(points)
            self.set_leg_angles()
            time.sleep(delay)
    
    def _execute_wave_gait(self, points: List[List[float]], xy: List[List[float]], 
                          F: int, z: float, delay: float) -> None:
        """Execute wave gait pattern."""
        number = [5, 2, 1, 0, 3, 4]  # Leg sequence
        
        for i in range(6):
            for j in range(int(F / 6)):
                for k in range(6):
                    if number[i] == k:
                        # Lift and move leg
                        if j < int(F / 18):
                            points[k][2] += 18 * z
                        elif j < int(F / 9):
                            points[k][0] += 30 * xy[k][0]
                            points[k][1] += 30 * xy[k][1]
                        elif j < int(F / 6):
                            points[k][2] -= 18 * z
                    else:
                        # Move other legs
                        points[k][0] -= 2 * xy[k][0]
                        points[k][1] -= 2 * xy[k][1]
                
                self.transform_coordinates(points)
                self.set_leg_angles()
                time.sleep(delay)
    
    def get_status(self) -> Dict[str, Any]:
        """
        Get current control system status.
        
        Returns:
            Dict[str, Any]: Status information
        """
        return {
            'movement_flag': self.movement_flag,
            'relaxation_flag': self.relaxation_flag,
            'status_flag': self.status_flag,
            'body_height': self.body_height,
            'leg_positions': self.leg_positions.copy(),
            'current_angles': self.current_angles.copy(),
            'command_queue': self.command_queue.copy()
        }


if __name__ == '__main__':
    print("Hexapod Robot Control System")
    print("=" * 40)
    
    # Initialize control system
    control = Control()
    
    try:
        # Keep the system running
        while True:
            time.sleep(1)
    except KeyboardInterrupt:
        print("\nShutting down control system...")
        control.relax(True)