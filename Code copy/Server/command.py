# -*- coding: utf-8 -*-
"""
Command definitions for Freenove Big Hexapod Robot Kit Server

This module defines all the command constants used for communication
between the client and server components of the hexapod robot system.

Commands follow the format: CMD_TYPE#PARAM1#PARAM2#...\n
"""

from enum import Enum
from typing import Dict, List, Any, Optional


class COMMAND:
    """
    Command constants for hexapod robot server communication protocol.
    
    All commands are received as strings with parameters separated by '#' 
    and terminated with '\n' (newline).
    """
    
    # ===== MOVEMENT COMMANDS =====
    CMD_MOVE = "CMD_MOVE"                    # Movement control: CMD_MOVE#DIRECTION#X#Y#ROTATION#SPEED
    CMD_POSITION = "CMD_POSITION"            # Body positioning: CMD_POSITION#X#Y#Z
    CMD_ATTITUDE = "CMD_ATTITUDE"            # Body orientation: CMD_ATTITUDE#ROLL#PITCH#YAW
    CMD_BALANCE = "CMD_BALANCE"              # Balance control: CMD_BALANCE#ENABLE
    
    # ===== SENSOR COMMANDS =====
    CMD_SONIC = "CMD_SONIC"                  # Distance measurement: CMD_SONIC
    CMD_POWER = "CMD_POWER"                  # Battery status: CMD_POWER
    
    # ===== ACTUATOR COMMANDS =====
    CMD_HEAD = "CMD_HEAD"                    # Head servo control: CMD_HEAD#SERVO_ID#ANGLE
    CMD_CAMERA = "CMD_CAMERA"                # Camera servo control: CMD_CAMERA#X#Y
    CMD_BUZZER = "CMD_BUZZER"                # Buzzer control: CMD_BUZZER#STATE
    CMD_SERVOPOWER = "CMD_SERVOPOWER"        # Servo power control: CMD_SERVOPOWER#STATE
    
    # ===== LIGHTING COMMANDS =====
    CMD_LED = "CMD_LED"                      # LED control: CMD_LED#MODE#R#G#B
    CMD_LED_MOD = "CMD_LED_MOD"              # LED mode control: CMD_LED_MOD#MODE#R#G#B
    CMD_PROXIMITY = "CMD_PROXIMITY"          # Proximity control: CMD_PROXIMITY#ACTION#PARAMS
    
    # ===== SYSTEM COMMANDS =====
    CMD_RELAX = "CMD_RELAX"                  # Servo relaxation: CMD_RELAX
    CMD_CALIBRATION = "CMD_CALIBRATION"      # Calibration: CMD_CALIBRATION#LEG#X#Y#Z
    
    # ===== COMMAND DESCRIPTIONS =====
    COMMAND_DESCRIPTIONS = {
        CMD_MOVE: "Control robot movement (direction, X, Y, rotation, speed)",
        CMD_POSITION: "Set robot body position (X, Y, Z coordinates)",
        CMD_ATTITUDE: "Set robot body attitude (roll, pitch, yaw angles)",
        CMD_BALANCE: "Enable/disable IMU-based balance control",
        CMD_SONIC: "Get ultrasonic distance measurement",
        CMD_POWER: "Get battery voltage status",
        CMD_HEAD: "Control head servos (servo ID, angle)",
        CMD_CAMERA: "Control camera servos (X, Y angles)",
        CMD_BUZZER: "Control buzzer (on/off state)",
        CMD_SERVOPOWER: "Control servo power supply",
        CMD_LED: "Control RGB LED strip (mode, color)",
        CMD_LED_MOD: "Set LED animation mode (mode, color)",
        CMD_PROXIMITY: "Control proximity light system (start/stop/status/config)",
        CMD_RELAX: "Relax all servos to save power",
        CMD_CALIBRATION: "Calibrate leg positions (leg, X, Y, Z)"
    }
    
    # ===== PARAMETER RANGES =====
    PARAMETER_RANGES = {
        CMD_MOVE: {
            "direction": ["forward", "backward", "left", "right", "turn_left", "turn_right", "stop"],
            "X": [-100, 100],
            "Y": [-100, 100], 
            "rotation": [-180, 180],
            "speed": [1, 10]
        },
        CMD_POSITION: {
            "X": [-40, 40],
            "Y": [-40, 40],
            "Z": [-20, 20]
        },
        CMD_ATTITUDE: {
            "roll": [-15, 15],
            "pitch": [-15, 15],
            "yaw": [-15, 15]
        },
        CMD_HEAD: {
            "servo_id": [0, 1],
            "angle": [0, 180]
        },
        CMD_CAMERA: {
            "X": [50, 180],
            "Y": [0, 180]
        },
        CMD_LED: {
            "mode": ["1", "2", "3", "4", "5", "6", "7", "8"],
            "R": [0, 255],
            "G": [0, 255],
            "B": [0, 255]
        },
        CMD_BUZZER: {
            "state": ["0", "1"]
        },
        CMD_SERVOPOWER: {
            "state": ["0", "1"]
        },
        CMD_BALANCE: {
            "enable": ["0", "1"]
        }
    }
    
    # ===== LED MODES =====
    LED_MODES = {
        "1": "Solid Color",
        "2": "Rainbow",
        "3": "Rainbow Cycle", 
        "4": "Theater Chase",
        "5": "Color Wipe",
        "6": "Breathing",
        "7": "Strobe",
        "8": "Off"
    }
    
    # ===== MOVEMENT DIRECTIONS =====
    MOVEMENT_DIRECTIONS = {
        "forward": "Move forward",
        "backward": "Move backward", 
        "left": "Move left",
        "right": "Move right",
        "turn_left": "Turn left",
        "turn_right": "Turn right",
        "stop": "Stop movement"
    }
    
    # ===== CALIBRATION LEGS =====
    CALIBRATION_LEGS = {
        "one": 0,
        "two": 1,
        "three": 2,
        "four": 3,
        "five": 4,
        "six": 5
    }
    
    # ===== SERVO MAPPING =====
    SERVO_MAPPING = {
        # Head servos
        "head_x": 0,      # Horizontal head movement
        "head_y": 1,      # Vertical head movement
        
        # Camera servos
        "camera_x": 0,    # Camera horizontal pan
        "camera_y": 1,    # Camera vertical tilt
        
        # Leg servos (18 total: 3 per leg × 6 legs)
        "leg1_hip": 15,   "leg1_thigh": 14,   "leg1_shin": 13,
        "leg2_hip": 12,   "leg2_thigh": 11,   "leg2_shin": 10,
        "leg3_hip": 9,    "leg3_thigh": 8,    "leg3_shin": 31,
        "leg4_hip": 22,   "leg4_thigh": 23,   "leg4_shin": 27,
        "leg5_hip": 19,   "leg5_thigh": 20,   "leg5_shin": 21,
        "leg6_hip": 16,   "leg6_thigh": 17,   "leg6_shin": 18
    }
    
    # ===== RESPONSE FORMATS =====
    RESPONSE_FORMATS = {
        CMD_SONIC: "CMD_SONIC#{distance}",
        CMD_POWER: "CMD_POWER#{battery1}#{battery2}",
        CMD_BUZZER: "CMD_BUZZER#{status}",
        CMD_LED: "CMD_LED#{status}",
        CMD_RELAX: "CMD_RELAX#{status}",
        CMD_CALIBRATION: "CMD_CALIBRATION#{status}"
    }
    
    # ===== ERROR CODES =====
    ERROR_CODES = {
        "INVALID_COMMAND": "E001",
        "INVALID_PARAMETERS": "E002",
        "OUT_OF_RANGE": "E003",
        "HARDWARE_ERROR": "E004",
        "NOT_INITIALIZED": "E005",
        "BUSY": "E006",
        "TIMEOUT": "E007"
    }
    
    # ===== STATUS CODES =====
    STATUS_CODES = {
        "SUCCESS": "OK",
        "FAILED": "FAIL",
        "BUSY": "BUSY",
        "READY": "READY",
        "ERROR": "ERROR"
    }
    
    def __init__(self):
        """Initialize the COMMAND class."""
        pass
    
    @classmethod
    def get_command_description(cls, command: str) -> str:
        """
        Get the description for a specific command.
        
        Args:
            command (str): The command constant
            
        Returns:
            str: Command description or "Unknown command"
        """
        return cls.COMMAND_DESCRIPTIONS.get(command, "Unknown command")
    
    @classmethod
    def get_parameter_range(cls, command: str, parameter: str) -> Optional[List]:
        """
        Get the valid range for a command parameter.
        
        Args:
            command (str): The command constant
            parameter (str): The parameter name
            
        Returns:
            list: [min_value, max_value] or None if not found
        """
        if command in cls.PARAMETER_RANGES:
            return cls.PARAMETER_RANGES[command].get(parameter)
        return None
    
    @classmethod
    def validate_command(cls, command: str, parameters: List[str]) -> Dict[str, Any]:
        """
        Validate command parameters against defined ranges.
        
        Args:
            command (str): The command constant
            parameters (list): List of parameter values
            
        Returns:
            dict: Validation result with status and details
        """
        result = {
            "valid": True,
            "errors": [],
            "warnings": []
        }
        
        if command not in cls.PARAMETER_RANGES:
            return result  # No validation rules defined
        
        param_ranges = cls.PARAMETER_RANGES[command]
        param_names = list(param_ranges.keys())
        
        for i, param_name in enumerate(param_names):
            if i >= len(parameters):
                continue
                
            try:
                param_value = parameters[i]
                param_range = param_ranges[param_name]
                
                # Handle numeric parameters
                if isinstance(param_range, list) and len(param_range) == 2:
                    try:
                        num_value = float(param_value)
                        if num_value < param_range[0] or num_value > param_range[1]:
                            result["valid"] = False
                            result["errors"].append(
                                f"Parameter {param_name}={param_value} out of range [{param_range[0]}, {param_range[1]}]"
                            )
                    except ValueError:
                        result["valid"] = False
                        result["errors"].append(f"Parameter {param_name}={param_value} is not numeric")
                
                # Handle enumeration parameters
                elif isinstance(param_range, list):
                    if param_value not in param_range:
                        result["valid"] = False
                        result["errors"].append(
                            f"Parameter {param_name}={param_value} not in allowed values {param_range}"
                        )
                        
            except Exception as e:
                result["valid"] = False
                result["errors"].append(f"Error validating {param_name}: {str(e)}")
                
        return result
    
    @classmethod
    def format_response(cls, command: str, *parameters) -> str:
        """
        Format a command response.
        
        Args:
            command (str): The command constant
            *parameters: Variable number of parameters
            
        Returns:
            str: Formatted response string
        """
        if not parameters:
            return f"{command}\n"
        
        param_str = "#".join(str(param) for param in parameters)
        return f"{command}#{param_str}\n"
    
    @classmethod
    def format_error_response(cls, command: str, error_code: str, message: str = "") -> str:
        """
        Format an error response.
        
        Args:
            command (str): The command constant
            error_code (str): Error code
            message (str): Error message
            
        Returns:
            str: Formatted error response
        """
        if message:
            return f"{command}#ERROR#{error_code}#{message}\n"
        else:
            return f"{command}#ERROR#{error_code}\n"
    
    @classmethod
    def parse_command(cls, command_string: str) -> tuple:
        """
        Parse a command string into command and parameters.
        
        Args:
            command_string (str): The command string to parse
            
        Returns:
            tuple: (command, parameters_list) or (None, []) if invalid
        """
        try:
            command_string = command_string.strip()
            if not command_string:
                return None, []
                
            parts = command_string.split('#')
            command = parts[0]
            parameters = parts[1:] if len(parts) > 1 else []
            
            return command, parameters
            
        except Exception:
            return None, []
    
    @classmethod
    def get_servo_id(cls, servo_name: str) -> Optional[int]:
        """
        Get servo ID by name.
        
        Args:
            servo_name (str): Servo name (e.g., "head_x", "leg1_hip")
            
        Returns:
            int: Servo ID or None if not found
        """
        return cls.SERVO_MAPPING.get(servo_name)
    
    @classmethod
    def get_leg_number(cls, leg_name: str) -> Optional[int]:
        """
        Get leg number by name.
        
        Args:
            leg_name (str): Leg name (e.g., "one", "two", "three")
            
        Returns:
            int: Leg number (0-5) or None if not found
        """
        return cls.CALIBRATION_LEGS.get(leg_name.lower())
    
    @classmethod
    def is_movement_command(cls, command: str) -> bool:
        """
        Check if command is a movement command.
        
        Args:
            command (str): Command to check
            
        Returns:
            bool: True if movement command
        """
        movement_commands = [cls.CMD_MOVE, cls.CMD_POSITION, cls.CMD_ATTITUDE, cls.CMD_BALANCE]
        return command in movement_commands
    
    @classmethod
    def is_sensor_command(cls, command: str) -> bool:
        """
        Check if command is a sensor command.
        
        Args:
            command (str): Command to check
            
        Returns:
            bool: True if sensor command
        """
        sensor_commands = [cls.CMD_SONIC, cls.CMD_POWER]
        return command in sensor_commands
    
    @classmethod
    def is_actuator_command(cls, command: str) -> bool:
        """
        Check if command is an actuator command.
        
        Args:
            command (str): Command to check
            
        Returns:
            bool: True if actuator command
        """
        actuator_commands = [cls.CMD_HEAD, cls.CMD_CAMERA, cls.CMD_BUZZER, cls.CMD_SERVOPOWER]
        return command in actuator_commands
    
    @classmethod
    def get_command_category(cls, command: str) -> str:
        """
        Get the category of a command.
        
        Args:
            command (str): Command to categorize
            
        Returns:
            str: Command category
        """
        if cls.is_movement_command(command):
            return "movement"
        elif cls.is_sensor_command(command):
            return "sensor"
        elif cls.is_actuator_command(command):
            return "actuator"
        elif command in [cls.CMD_LED, cls.CMD_LED_MOD]:
            return "lighting"
        elif command in [cls.CMD_RELAX, cls.CMD_CALIBRATION]:
            return "system"
        else:
            return "unknown"


# Create a global instance for easy access
CMD = COMMAND()

if __name__ == '__main__':
    # Test the command system
    print("Hexapod Robot Server Command System")
    print("=" * 50)
    
    # Test command validation
    test_commands = [
        ("CMD_MOVE", ["forward", "50", "0", "0", "5"]),
        ("CMD_POSITION", ["20", "15", "-10"]),
        ("CMD_HEAD", ["0", "90"]),
        ("CMD_SONIC", []),
        ("CMD_INVALID", ["param1", "param2"])
    ]
    
    for command, params in test_commands:
        print(f"\nTesting command: {command}")
        print(f"Parameters: {params}")
        
        # Validate command
        validation = CMD.validate_command(command, params)
        print(f"Valid: {validation['valid']}")
        if validation['errors']:
            print(f"Errors: {validation['errors']}")
        
        # Get description
        desc = CMD.get_command_description(command)
        print(f"Description: {desc}")
        
        # Get category
        category = CMD.get_command_category(command)
        print(f"Category: {category}")
    
    # Test servo mapping
    print(f"\nServo mappings:")
    for name, servo_id in CMD.SERVO_MAPPING.items():
        print(f"  {name}: {servo_id}")
    
    # Test error formatting
    print(f"\nError response example:")
    error_response = CMD.format_error_response("CMD_MOVE", "E002", "Invalid parameters")
    print(f"  {error_response.strip()}")
    
    print("\n✓ Server command system test completed")
