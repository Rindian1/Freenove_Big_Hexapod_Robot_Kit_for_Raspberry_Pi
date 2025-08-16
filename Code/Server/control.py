# -*- coding: utf-8 -*-
# Standard library imports
import time
import math
import copy
import threading

# Third-party imports
import numpy as np
from gpiozero import OutputDevice

# Local application imports
from pid import Incremental_PID  # PID controller for balance
from command import COMMAND as cmd  # Command definitions
from imu import IMU  # Inertial Measurement Unit for orientation sensing
from servo import Servo  # Servo motor control

class Control:
    """
    Main control class for the hexapod robot.
    Handles movement, balance, and servo control for all six legs.
    """
    
    def __init__(self):  # type: () -> None
        """
        Initialize the Control class with default parameters and hardware interfaces.
        Sets up servos, IMU, and initial robot state.
        """
        # Initialize IMU for orientation sensing
        self.imu = IMU()
        
        # Initialize servo controller
        self.servo = Servo()
        
        # Movement control flag (0x01 = normal movement)
        self.movement_flag = 0x01 
        
        # Flag to track if servos are in relaxed state
        self.relaxation_flag = False
        
        # PID controller for balance (Proportional = 0.5, Integral = 0, Derivative = 0.0025)
        self.pid_controller = Incremental_PID(0.500, 0.00, 0.0025)
        
        # GPIO control for servo power (pin 4)
        self.servo_power_disable = OutputDevice(4)
        self.servo_power_disable.off()  # Turn on servo power (active low)
        
        # Robot status flag (see below for bit meanings)
        self.status_flag = 0x00 
        # indicates Status of the robot ie.  
            # whether they should be relaxed (0x00) 
            # whether the servos should be used to move and maintain specific positions of the entire hexapod(0x01)  
            # whether the servos should be used to move and maintain specific attitudes of the entire hexapod (0x02) 
            # whether the servos should be used to move and maintain specific positions and attitudes of the entire hexapod simultaneously(0x03)
            # whether it should be balancing (0x04)
        self.timeout = 0
        
        # Default body height (negative Z is up in robot's coordinate system)
        self.body_height = -25
        
        # Default positions for each leg's attachment point on the body (in mm)
        # Format: [ [x1,y1,z1], [x2,y2,z2], ... ] for legs 1-6
        self.body_points = [
            [137.1, 189.4, self.body_height],  # Leg 1 (front right)
            [225, 0, self.body_height],        # Leg 2 (middle right)
            [137.1, -189.4, self.body_height], # Leg 3 (rear right)
            [-137.1, -189.4, self.body_height],# Leg 4 (rear left)
            [-225, 0, self.body_height],       # Leg 5 (middle left)
            [-137.1, 189.4, self.body_height]  # Leg 6 (front left)
        ]
        
        # Load leg calibration data from file
        self.calibration_leg_positions = self.read_from_txt('point')

        # Current target positions for each leg's end effector [x,y,z] in mm
        self.leg_positions = [[140, 0, 0], [140, 0, 0], [140, 0, 0], [140, 0, 0], [140, 0, 0], [140, 0, 0]]
        self.calibration_angles = [[0, 0, 0], [0, 0, 0], [0, 0, 0], [0, 0, 0], [0, 0, 0], [0, 0, 0]]
        self.current_angles = [[90, 0, 0], [90, 0, 0], [90, 0, 0], [90, 0, 0], [90, 0, 0], [90, 0, 0]]
        self.command_queue = ['', '', '', '', '', '']
        
        #~self.leg_positions = [[140, 0, 0] for _ in range(6)]
        #~self.calibration_angles = [[0, 0, 0] for _ in range(6)]
        #~self.current_angles = [[90, 0, 0] for _ in range(6)] 
        #~self.command_queue = [''] * 6
        
        # Perform initial calibration and set default leg positions
        self.calibrate()
        self.set_leg_angles()
        
        # Start the condition monitoring thread
        self.condition_thread = threading.Thread(target=self.condition_monitor)
        self.condition_thread.daemon = True  # Allow program to exit while thread is running
        self.condition_thread.start()
        
        # Thread synchronization primitive
        self.Thread_conditiona = threading.Condition()

    def read_from_txt(self, filename):  # type: (str) -> List[List[int]]
        """
        Read data from a text file and return it as a list of lists.
        Each inner list represents a line in the file, with values separated by tabs.
        """
        with open(filename + ".txt", "r") as file:
            lines = file.readlines()
            data = [list(map(int, line.strip().split("\t"))) for line in lines]
        return data

    def save_to_txt(self, data, filename):  # type: (List[List[Union[int, float]]], str) -> None
        with open(filename + '.txt', 'w') as file:
            for row in data:
                file.write('\t'.join(map(str, row)) + '\n')

    def coordinate_to_angle(self, x, y, z, l1=33, l2=90, l3=110):  # type: (float, float, float, float, float, float) -> Tuple[float, float, float]
        """
        Convert 3D coordinates to servo angles using inverse kinematics.
        
        This method calculates the required servo angles to position the end effector
        (foot) at the specified (x,y,z) coordinates relative to the leg's base.
        
        Args:
            x: X-coordinate of target position (forward/backward)
            y: Y-coordinate of target position (left/right)
            z: Z-coordinate of target position (up/down)
            l1: Length of first limb segment (shoulder to elbow) in mm
            l2: Length of second limb segment (elbow to wrist) in mm
            l3: Length of third limb segment (wrist to foot) in mm
            
        Returns:
            Tuple of (base_angle, middle_angle, end_angle) in degrees
            
        The coordinate system is right-handed with:
        - X: Forward (positive) / Backward (negative)
        - Y: Left (positive) / Right (negative)
        - Z: Up (negative) / Down (positive)
        """
        # Calculate base angle (rotation around vertical axis)
        # This determines the yaw of the entire leg
        a = math.pi / 2 - math.atan2(z, y)
        
        # Project the target point onto the leg's plane of movement
        # x_3 is the x-coordinate of the first joint (always 0 in local coordinates)
        x_3 = 0
        
        # Calculate the position of the first joint relative to the base
        # These represent the projection of the first link (l1) in 3D space
        x_4 = l1 * math.sin(a)  # y-component of first link
        x_5 = l1 * math.cos(a)  # z-component of first link
        
        # Calculate the distance from the second joint to the end effector
        # This forms a triangle with sides l2, l3, and l23
        l23 = math.sqrt((z - x_5) ** 2 + (y - x_4) ** 2 + (x - x_3) ** 2)
        
        # Intermediate values for the Law of Cosines calculations
        # These are clamped to [-1, 1] to avoid math domain errors from floating point inaccuracies
        w = self.restrict_value((x - x_3) / l23, -1, 1)  # x-component ratio
        v = self.restrict_value((l2**2 + l23**2 - l3**2) / (2 * l2 * l23), -1, 1)  # cos(angle between l2 and l23)
        u = self.restrict_value((l2**2 + l3**2 - l23**2) / (2 * l3 * l2), -1, 1)  # cos(angle between l2 and l3)
        
        # Calculate the second angle (b) - middle joint angle
        # This is the angle of the second servo (middle joint)
        b = math.asin(round(w, 2)) - math.acos(round(v, 2))
        
        # Calculate the third angle (c) - end joint angle
        # This is the angle of the third servo (end effector joint)
        c = math.pi - math.acos(round(u, 2))
        
        # Convert all angles from radians to degrees and round to nearest integer
        return (
            round(math.degrees(a)),
            round(math.degrees(b)),
            round(math.degrees(c))
        )

    def angle_to_coordinate(self, a, b, c, l1=33, l2=90, l3=110):  # type: (float, float, float, float, float, float) -> Tuple[float, float, float]
        """
        Convert servo angles to 3D coordinates using forward kinematics.
        
        This is the inverse of coordinate_to_angle, calculating the end effector
        position from the given joint angles.
        
        Args:
            a: Base angle in degrees (rotation around vertical axis)
            b: Middle angle in degrees (shoulder joint)
            c: End angle in degrees (elbow joint)
            l1: Length of first limb segment in mm
            l2: Length of second limb segment in mm
            l3: Length of third limb segment in mm
            
        Returns:
            Tuple of (x, y, z) coordinates in mm
        """
        # Convert angles from degrees to radians
        a = math.pi / 180 * a
        b = math.pi / 180 * b
        c = math.pi / 180 * c
        
        # Calculate end effector position using forward kinematics
        x = round(l3 * math.sin(b + c) + l2 * math.sin(b))
        y = round(l3 * math.sin(a) * math.cos(b + c) + 
                 l2 * math.sin(a) * math.cos(b) + 
                 l1 * math.sin(a))
        z = round(l3 * math.cos(a) * math.cos(b + c) + 
                 l2 * math.cos(a) * math.cos(b) + 
                 l1 * math.cos(a))
        
        return x, y, z

    def calibrate(self):  # type: () -> None
        """
        Calculate calibration offsets for all servos based on stored calibration points.
        
        This method:
        1. Resets leg positions to default (straight down)
        2. Calculates the theoretical angles for calibration points
        3. Calculates the difference between current and target angles
        4. Stores these differences as calibration offsets
        
        The calibration process ensures that when the robot is commanded to move to a 
        specific position, the servos account for any mechanical misalignments.
        """
        # Reset all leg positions to default (straight down)
        self.leg_positions = [[140, 0, 0], [140, 0, 0], [140, 0, 0], [140, 0, 0], [140, 0, 0], [140, 0, 0]]
        for i in range(6):
            self.calibration_angles[i][0], self.calibration_angles[i][1], self.calibration_angles[i][2] = self.coordinate_to_angle(
                -self.calibration_leg_positions[i][2], self.calibration_leg_positions[i][0], self.calibration_leg_positions[i][1])
        for i in range(6):
            self.current_angles[i][0], self.current_angles[i][1], self.current_angles[i][2] = self.coordinate_to_angle(
                -self.leg_positions[i][2], self.leg_positions[i][0], self.leg_positions[i][1])
        for i in range(6):
            self.calibration_angles[i][0] = self.calibration_angles[i][0] - self.current_angles[i][0]  # Base
            self.calibration_angles[i][1] = self.calibration_angles[i][1] - self.current_angles[i][1]  # Middle
            self.calibration_angles[i][2] = self.calibration_angles[i][2] - self.current_angles[i][2]  # Tip

    def set_leg_angles(self):  # type: () -> None
        """
        Calculate and set all servo angles based on current leg positions.
        
        This method:
        1. Checks if target positions are valid
        2. Converts 3D positions to servo angles
        3. Applies calibration offsets
        4. Sends angle commands to servos
        
        The method handles all six legs and their three servos each.
        Left and right legs have mirrored kinematics, which is accounted for.
        """
        if self.check_point_validity(): #~ IMPROVE THIS MESS
            # Convert current leg positions to angles for all legs
            for i in range(6):
                self.current_angles[i][0], self.current_angles[i][1], self.current_angles[i][2] = self.coordinate_to_angle(
                    -self.leg_positions[i][2], self.leg_positions[i][0], self.leg_positions[i][1])
            for i in range(3):
                self.current_angles[i][0] = self.restrict_value(self.current_angles[i][0] + self.calibration_angles[i][0], 0, 180)
                self.current_angles[i][1] = self.restrict_value(90 - (self.current_angles[i][1] + self.calibration_angles[i][1]), 0, 180)
                self.current_angles[i][2] = self.restrict_value(self.current_angles[i][2] + self.calibration_angles[i][2], 0, 180)
                self.current_angles[i + 3][0] = self.restrict_value(self.current_angles[i + 3][0] + self.calibration_angles[i + 3][0], 0, 180)
                self.current_angles[i + 3][1] = self.restrict_value(90 + self.current_angles[i + 3][1] + self.calibration_angles[i + 3][1], 0, 180)
                self.current_angles[i + 3][2] = self.restrict_value(180 - (self.current_angles[i + 3][2] + self.calibration_angles[i + 3][2]), 0, 180)
            # Leg 1
            self.servo.set_servo_angle(15, self.current_angles[0][0])
            self.servo.set_servo_angle(14, self.current_angles[0][1])
            self.servo.set_servo_angle(13, self.current_angles[0][2])
            # Leg 2
            self.servo.set_servo_angle(12, self.current_angles[1][0])
            self.servo.set_servo_angle(11, self.current_angles[1][1])
            self.servo.set_servo_angle(10, self.current_angles[1][2])
            
            # Leg 3 (Rear Right)
            self.servo.set_servo_angle(9, self.current_angles[2][0])
            self.servo.set_servo_angle(8, self.current_angles[2][1])
            self.servo.set_servo_angle(31, self.current_angles[2][2])
            
            # Leg 6 (Front Left) - Note: Leg numbering wraps around
            self.servo.set_servo_angle(16, self.current_angles[5][0])
            self.servo.set_servo_angle(17, self.current_angles[5][1])
            self.servo.set_servo_angle(18, self.current_angles[5][2])
            
            # Leg 5 (Middle Left)
            self.servo.set_servo_angle(19, self.current_angles[4][0])
            self.servo.set_servo_angle(20, self.current_angles[4][1])
            self.servo.set_servo_angle(21, self.current_angles[4][2])
            
            # Leg 4 (Rear Left)
            self.servo.set_servo_angle(22, self.current_angles[3][0])
            self.servo.set_servo_angle(23, self.current_angles[3][1])
            self.servo.set_servo_angle(27, self.current_angles[3][2])
        else:
            print("This coordinate point is out of the active range")

    def check_point_validity(self):  # type: () -> bool
        """
        Verify that all leg positions are within the robot's working range.
        
        Returns:
            bool: True if all leg positions are valid, False otherwise
            
        This prevents the robot from attempting to move to physically impossible
        positions that could damage the servos or mechanical structure.
        """
        is_valid = True
        leg_lengths = [0] * 6
        
        # Calculate the 3D vector length for each leg position
        for i in range(6):
            leg_lengths[i] = math.sqrt(
                self.leg_positions[i][0] ** 2 +  # x²
                self.leg_positions[i][1] ** 2 +  # y²
                self.leg_positions[i][2] ** 2    # z²
            )
        
        # Check each leg's length against physical constraints
        # Minimum length: 90mm (fully retracted)
        # Maximum length: 248mm (fully extended)
        for length in leg_lengths:
            if length > 248 or length < 90:
                is_valid = False
        return is_valid

    def condition_monitor(self):  # type: () -> None
        """
        Main control loop that processes incoming commands and manages robot states.
        
        This method runs in a separate thread and continuously checks for:
        1. Timeout conditions (inactivity)
        2. New commands in the command queue
        3. Current robot status and mode changes
        
        The method handles several command types:
        - CMD_POSITION: Move to specific XYZ coordinates
        - CMD_ATTITUDE: Set body orientation (roll, pitch, yaw)
        - CMD_MOVE: Execute a walking gait
        - CMD_BALANCE: Enable/disable balance mode
        - CMD_CALIBRATION: Calibrate leg positions
        
        The robot will automatically relax (disable servos) after 10 seconds of inactivity.
        """
        while True:
            # if 
                # 1. No commands have been received for more than 10 seconds (timeout check)
                # 2. The timeout has been initialized (self.timeout != 0)
                # 3. The command queue is empty (self.command_queue[0] == '')
            # then 
            # 1. Reset the timeout 
            # 2. Put the robot in a relaxed state 
            # 3. Set the status flag to 0x00  (servos disengaged)
            if (time.time() - self.timeout) > 10 and self.timeout != 0 and self.command_queue[0] == '':
                self.timeout = time.time()  # Reset timeout
                self.relax(True)  # Relax all servos
                self.status_flag = 0x00  # Set status to relaxed
            
            # Process POSITION command (move body to specific x,y,z)
            if cmd.CMD_POSITION in self.command_queue and len(self.command_queue) == 4:
                if self.status_flag != 0x01:  # If not already in position mode
                    self.relax(False)  # Wake up servos
                # Parse and restrict position values
                x = self.restrict_value(int(self.command_queue[1]), -40, 40)
                y = self.restrict_value(int(self.command_queue[2]), -40, 40)
                z = self.restrict_value(int(self.command_queue[3]), -20, 20)
                self.move_position(x, y, z)  # Move to new position
                self.status_flag = 0x01  # Set status to position control
                self.command_queue = [''] * 6  # Clear command queue
            
            # Process ATTITUDE command (set body orientation)
            elif cmd.CMD_ATTITUDE in self.command_queue and len(self.command_queue) == 4:
                if self.status_flag != 0x02:  # If not already in attitude mode
                    self.relax(False)  # Wake up servos
                # Parse and restrict orientation values
                roll = self.restrict_value(int(self.command_queue[1]), -15, 15)
                pitch = self.restrict_value(int(self.command_queue[2]), -15, 15)
                yaw = self.restrict_value(int(self.command_queue[3]), -15, 15)
                # Calculate and apply new leg positions for desired orientation
                points = self.calculate_posture_balance(roll, pitch, yaw)
                self.transform_coordinates(points)
                self.set_leg_angles()
                self.status_flag = 0x02  # Set status to attitude control
                self.command_queue = [''] * 6  # Clear command queue
            
            # Process MOVE command (walking)
            elif cmd.CMD_MOVE in self.command_queue and len(self.command_queue) == 6:
                if self.command_queue[2] == "0" and self.command_queue[3] == "0":
                    # Special case: Stop movement
                    self.run_gait(self.command_queue)
                    self.command_queue = [''] * 6
                else:
                    if self.status_flag != 0x03:  # If not already in movement mode
                        self.relax(False)  # Wake up servos
                    self.run_gait(self.command_queue)  # Execute walking gait
                    self.status_flag = 0x03  # Set status to movement
            
            # Process BALANCE command
            elif cmd.CMD_BALANCE in self.command_queue and len(self.command_queue) == 2:
                if self.command_queue[1] == "1":  # Enable balance
                    self.command_queue = [''] * 6  # Clear command queue
                    if self.status_flag != 0x04:  # If not already in balance mode
                        self.relax(False)  # Wake up servos
                    self.status_flag = 0x04  # Set status to balance mode
                    self.imu6050()  # Start IMU-based balance control
            
            # Process CALIBRATION command
            elif cmd.CMD_CALIBRATION in self.command_queue:
                self.timeout = 0  # Disable timeout during calibration
                self.calibrate()  # Run calibration
                self.set_leg_angles()  # Apply calibration
                
                # Handle different calibration sub-commands
                if len(self.command_queue) >= 2:
                    if self.command_queue[1] == "one":
                        self.calibration_leg_positions[0][0] = int(self.command_queue[2])
                        self.calibration_leg_positions[0][1] = int(self.command_queue[3])
                        self.calibration_leg_positions[0][2] = int(self.command_queue[4])
                        self.calibrate()
                        self.set_leg_angles()
                    elif self.command_queue[1] == "two":
                        self.calibration_leg_positions[1][0] = int(self.command_queue[2])
                        self.calibration_leg_positions[1][1] = int(self.command_queue[3])
                        self.calibration_leg_positions[1][2] = int(self.command_queue[4])
                        self.calibrate()
                        self.set_leg_angles()
                    elif self.command_queue[1] == "three":
                        self.calibration_leg_positions[2][0] = int(self.command_queue[2])
                        self.calibration_leg_positions[2][1] = int(self.command_queue[3])
                        self.calibration_leg_positions[2][2] = int(self.command_queue[4])
                        self.calibrate()
                        self.set_leg_angles()
                    elif self.command_queue[1] == "four":
                        self.calibration_leg_positions[3][0] = int(self.command_queue[2])
                        self.calibration_leg_positions[3][1] = int(self.command_queue[3])
                        self.calibration_leg_positions[3][2] = int(self.command_queue[4])
                        self.calibrate()
                        self.set_leg_angles()
                    elif self.command_queue[1] == "five":
                        self.calibration_leg_positions[4][0] = int(self.command_queue[2])
                        self.calibration_leg_positions[4][1] = int(self.command_queue[3])
                        self.calibration_leg_positions[4][2] = int(self.command_queue[4])
                        self.calibrate()
                        self.set_leg_angles()
                    elif self.command_queue[1] == "six":
                        self.calibration_leg_positions[5][0] = int(self.command_queue[2])
                        self.calibration_leg_positions[5][1] = int(self.command_queue[3])
                        self.calibration_leg_positions[5][2] = int(self.command_queue[4])
                        self.calibrate()
                        self.set_leg_angles()
                    elif self.command_queue[1] == "save":
                        self.save_to_txt(self.calibration_leg_positions, 'point')
                self.command_queue = ['', '', '', '', '', '']

    def relax(self, flag):  # type: (bool) -> None
        """
        Enable or disable servo power.
        
        Args:
            flag: If True, relax all servos (disable torque).
                  If False, enable servos and set to current angles.
                  
        When relaxed, servos can be moved manually, which is useful for:
        - Initial setup and calibration
        - Preventing damage during collisions
        - Conserving power when not in use
        """
        if flag:
            self.servo.relax()  # Disable all servos
        else:
            self.set_leg_angles()  # Enable servos and set to current angles

    def transform_coordinates(self, points):  # type: (List[List[float]]) -> None
        # Leg 1
        self.leg_positions[0][0] = points[0][0] * math.cos(54 / 180 * math.pi) + points[0][1] * math.sin(54 / 180 * math.pi) - 94
        self.leg_positions[0][1] = -points[0][0] * math.sin(54 / 180 * math.pi) + points[0][1] * math.cos(54 / 180 * math.pi)
        self.leg_positions[0][2] = points[0][2] - 14
        # Leg 2
        self.leg_positions[1][0] = points[1][0] * math.cos(0 / 180 * math.pi) + points[1][1] * math.sin(0 / 180 * math.pi) - 85
        self.leg_positions[1][1] = -points[1][0] * math.sin(0 / 180 * math.pi) + points[1][1] * math.cos(0 / 180 * math.pi)
        self.leg_positions[1][2] = points[1][2] - 14
        # Leg 3
        self.leg_positions[2][0] = points[2][0] * math.cos(-54 / 180 * math.pi) + points[2][1] * math.sin(-54 / 180 * math.pi) - 94
        self.leg_positions[2][1] = -points[2][0] * math.sin(-54 / 180 * math.pi) + points[2][1] * math.cos(-54 / 180 * math.pi)
        self.leg_positions[2][2] = points[2][2] - 14
        # Leg 4
        self.leg_positions[3][0] = points[3][0] * math.cos(-126 / 180 * math.pi) + points[3][1] * math.sin(-126 / 180 * math.pi) - 94
        self.leg_positions[3][1] = -points[3][0] * math.sin(-126 / 180 * math.pi) + points[3][1] * math.cos(-126 / 180 * math.pi)
        self.leg_positions[3][2] = points[3][2] - 14
        # Leg 5
        self.leg_positions[4][0] = points[4][0] * math.cos(180 / 180 * math.pi) + points[4][1] * math.sin(180 / 180 * math.pi) - 85
        self.leg_positions[4][1] = -points[4][0] * math.sin(180 / 180 * math.pi) + points[4][1] * math.cos(180 / 180 * math.pi)
        self.leg_positions[4][2] = points[4][2] - 14
        # Leg 6
        self.leg_positions[5][0] = points[5][0] * math.cos(126 / 180 * math.pi) + points[5][1] * math.sin(126 / 180 * math.pi) - 94
        self.leg_positions[5][1] = -points[5][0] * math.sin(126 / 180 * math.pi) + points[5][1] * math.cos(126 / 180 * math.pi)
        self.leg_positions[5][2] = points[5][2] - 14

    def restrict_value(self, value, min_value, max_value):  # type: (float, float, float) -> float
        if value < min_value:
            return min_value
        elif value > max_value:
            return max_value
        else:
            return value

    def map_value(self, value, from_low, from_high, to_low, to_high):  # type: (float, float, float, float, float) -> float
        return (to_high - to_low) * (value - from_low) / (from_high - from_low) + to_low

    def move_position(self, x, y, z):  # type: (int, int, int) -> None
        """
        Move the robot's body to a new position.
        
        This method updates the target position of all legs to move the body
        to the specified coordinates relative to the current position.
        
        Args:
            x: Movement in X direction (forward/backward) in mm
            y: Movement in Y direction (left/right) in mm
            z: Movement in Z direction (up/down) in mm
            
        The movement is performed by adjusting all leg positions simultaneously
        to maintain the body's orientation while moving it in space.
        """
        # Create a copy of the current body points
        points = copy.deepcopy(self.body_points)
        
        # Update each leg's target position
        for i in range(6):
            # Apply translation to each leg's attachment point
            points[i][0] = self.body_points[i][0] - x  # X movement
            points[i][1] = self.body_points[i][1] - y  # Y movement
            points[i][2] = -30 - z  # Z movement (inverted: negative is up)
            
            # Update body height tracking
            self.body_height = points[i][2]
            self.body_points[i][2] = points[i][2]
        
        # Transform coordinates and update servos
        self.transform_coordinates(points)
        self.set_leg_angles()

    def calculate_posture_balance(self, roll, pitch, yaw):  # type: (int, int, int) -> List[List[float]]
        position = np.mat([0.0, 0.0, self.body_height])
        rpy = np.array([roll, pitch, yaw]) * math.pi / 180
        roll_angle, pitch_angle, yaw_angle = rpy[0], rpy[1], rpy[2]
        rotation_x = np.mat([[1, 0, 0],
                             [0, math.cos(pitch_angle), -math.sin(pitch_angle)],
                             [0, math.sin(pitch_angle), math.cos(pitch_angle)]])
        rotation_y = np.mat([[math.cos(roll_angle), 0, -math.sin(roll_angle)],
                             [0, 1, 0],
                             [math.sin(roll_angle), 0, math.cos(roll_angle)]])
        rotation_z = np.mat([[math.cos(yaw_angle), -math.sin(yaw_angle), 0],
                             [math.sin(yaw_angle), math.cos(yaw_angle), 0],
                             [0, 0, 1]])
        rotation_matrix = rotation_x * rotation_y * rotation_z
        body_structure = np.mat([[55, 76, 0],
                                [85, 0, 0],
                                [55, -76, 0],
                                [-55, -76, 0],
                                [-85, 0, 0],
                                [-55, 76, 0]]).T
        footpoint_structure = np.mat([[137.1, 189.4, 0],
                                     [225, 0, 0],
                                     [137.1, -189.4, 0],
                                     [-137.1, -189.4, 0],
                                     [-225, 0, 0],
                                     [-137.1, 189.4, 0]]).T
        ab = np.mat(np.zeros((3, 6)))
        foot_positions = [[0, 0, 0], [0, 0, 0], [0, 0, 0], [0, 0, 0], [0, 0, 0], [0, 0, 0]]
        for i in range(6):
            ab[:, i] = position + rotation_matrix * footpoint_structure[:, i]
            foot_positions[i][0] = ab[0, i]
            foot_positions[i][1] = ab[1, i]
            foot_positions[i][2] = ab[2, i]
        return foot_positions

    def imu6050(self):  # type: () -> None
        old_roll = 0
        old_pitch = 0
        points = self.calculate_posture_balance(0, 0, 0)
        self.transform_coordinates(points)
        self.set_leg_angles()
        time.sleep(2)
        self.imu.Error_value_accel_data, self.imu.Error_value_gyro_data = self.imu.calculate_average_sensor_data()
        time.sleep(1)
        while True:
            if self.command_queue[0] != "":
                break
            time.sleep(0.02)
            roll, pitch, yaw = self.imu.update_imu_state()
            roll = self.pid_controller.pid_calculate(roll)
            pitch = self.pid_controller.pid_calculate(pitch)
            points = self.calculate_posture_balance(roll, pitch, 0)
            self.transform_coordinates(points)
            self.set_leg_angles()
                
    def run_gait(self, data, Z=40, F=64):  # type: (List[str], int, int) -> None  # Example: data=['CMD_MOVE', '1', '0', '25', '10', '0']
        gait = data[1]
        x = self.restrict_value(int(data[2]), -35, 35)
        y = self.restrict_value(int(data[3]), -35, 35)
        if gait == "1":
            F = round(self.map_value(int(data[4]), 2, 10, 126, 22))
        else:
            F = round(self.map_value(int(data[4]), 2, 10, 171, 45))
        angle = int(data[5])
        z = Z / F
        delay = 0.01
        points = copy.deepcopy(self.body_points)
        xy = [[0, 0], [0, 0], [0, 0], [0, 0], [0, 0], [0, 0]]
        for i in range(6):
            xy[i][0] = ((points[i][0] * math.cos(angle / 180 * math.pi) + points[i][1] * math.sin(angle / 180 * math.pi) - points[i][0]) + x) / F
            xy[i][1] = ((-points[i][0] * math.sin(angle / 180 * math.pi) + points[i][1] * math.cos(angle / 180 * math.pi) - points[i][1]) + y) / F
        if x == 0 and y == 0 and angle == 0:
            self.transform_coordinates(points)
            self.set_leg_angles()
        elif gait == "1":
            for j in range(F):
                for i in range(3):
                    if j < (F / 8):
                        points[2 * i][0] = points[2 * i][0] - 4 * xy[2 * i][0]
                        points[2 * i][1] = points[2 * i][1] - 4 * xy[2 * i][1]
                        points[2 * i + 1][0] = points[2 * i + 1][0] + 8 * xy[2 * i + 1][0]
                        points[2 * i + 1][1] = points[2 * i + 1][1] + 8 * xy[2 * i + 1][1]
                        points[2 * i + 1][2] = Z + self.body_height
                    elif j < (F / 4):
                        points[2 * i][0] = points[2 * i][0] - 4 * xy[2 * i][0]
                        points[2 * i][1] = points[2 * i][1] - 4 * xy[2 * i][1]
                        points[2 * i + 1][2] = points[2 * i + 1][2] - z * 8
                    elif j < (3 * F / 8):
                        points[2 * i][2] = points[2 * i][2] + z * 8
                        points[2 * i + 1][0] = points[2 * i + 1][0] - 4 * xy[2 * i + 1][0]
                        points[2 * i + 1][1] = points[2 * i + 1][1] - 4 * xy[2 * i + 1][1]
                    elif j < (5 * F / 8):
                        points[2 * i][0] = points[2 * i][0] + 8 * xy[2 * i][0]
                        points[2 * i][1] = points[2 * i][1] + 8 * xy[2 * i][1]
                        points[2 * i + 1][0] = points[2 * i + 1][0] - 4 * xy[2 * i + 1][0]
                        points[2 * i + 1][1] = points[2 * i + 1][1] - 4 * xy[2 * i + 1][1]
                    elif j < (3 * F / 4):
                        points[2 * i][2] = points[2 * i][2] - z * 8
                        points[2 * i + 1][0] = points[2 * i + 1][0] - 4 * xy[2 * i + 1][0]
                        points[2 * i + 1][1] = points[2 * i + 1][1] - 4 * xy[2 * i + 1][1]
                    elif j < (7 * F / 8):
                        points[2 * i][0] = points[2 * i][0] - 4 * xy[2 * i][0]
                        points[2 * i][1] = points[2 * i][1] - 4 * xy[2 * i][1]
                        points[2 * i + 1][2] = points[2 * i + 1][2] + z * 8
                    elif j < (F):
                        points[2 * i][0] = points[2 * i][0] - 4 * xy[2 * i][0]
                        points[2 * i][1] = points[2 * i][1] - 4 * xy[2 * i][1]
                        points[2 * i + 1][0] = points[2 * i + 1][0] + 8 * xy[2 * i + 1][0]
                        points[2 * i + 1][1] = points[2 * i + 1][1] + 8 * xy[2 * i + 1][1]
                self.transform_coordinates(points)
                self.set_leg_angles()
            time.sleep(delay)
        elif gait == "2":
            # Gait pattern 2: Tripod gait (3 legs move at a time)
            number = [5, 2, 1, 0, 3, 4]  # Leg movement order for this gait
            for i in range(6):
                for j in range(int(F / 6)):
                    for k in range(6):
                        if number[i] == k:
                            if j < int(F / 18):
                                points[k][2] += 18 * z
                            elif j < int(F / 9):
                                points[k][0] += 30 * xy[k][0]
                                points[k][1] += 30 * xy[k][1]
                            elif j < int(F / 6):
                                points[k][2] -= 18 * z
                        else:
                            points[k][0] -= 2 * xy[k][0]
                            points[k][1] -= 2 * xy[k][1]
                    self.transform_coordinates(points)
                    self.set_leg_angles()
                    time.sleep(delay)

if __name__ == '__main__':
    pass