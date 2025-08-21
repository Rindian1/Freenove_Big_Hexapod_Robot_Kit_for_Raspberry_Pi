# -*- coding: utf-8 -*-
"""
Command definitions for Freenove Big Hexapod Robot Kit

This module defines all the command constants used for communication
between the client and server components of the hexapod robot system.

Commands follow the format: CMD_TYPE#PARAM1#PARAM2#...\n
"""

class COMMAND:
    """
    Command constants for hexapod robot communication protocol.
    
    All commands are sent as strings with parameters separated by '#' 
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
            "direction": ["forward", "backward", "left", "right", "turn_left", "turn_right"],
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
    
    def __init__(self):
        """Initialize the COMMAND class."""
        pass
    
    @classmethod
    def get_command_description(cls, command):
        """
        Get the description for a specific command.
        
        Args:
            command (str): The command constant
            
        Returns:
            str: Command description or "Unknown command"
        """
        return cls.COMMAND_DESCRIPTIONS.get(command, "Unknown command")
    
    @classmethod
    def get_parameter_range(cls, command, parameter):
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
    def validate_command(cls, command, parameters):
        """
        Validate command parameters against defined ranges.
        
        Args:
            command (str): The command constant
            parameters (list): List of parameter values
            
        Returns:
            bool: True if valid, False otherwise
        """
        if command not in cls.PARAMETER_RANGES:
            return True  # No validation rules defined
        
        param_ranges = cls.PARAMETER_RANGES[command]
        param_names = list(param_ranges.keys())
        
        for i, param_name in enumerate(param_names):
            if i >= len(parameters):
                continue
                
            try:
                param_value = float(parameters[i])
                param_range = param_ranges[param_name]
                
                if isinstance(param_range, list) and len(param_range) == 2:
                    if param_value < param_range[0] or param_value > param_range[1]:
                        return False
                elif isinstance(param_range, list):
                    if param_value not in param_range:
                        return False
                        
            except (ValueError, TypeError):
                return False
                
        return True
    
    @classmethod
    def format_command(cls, command, *parameters):
        """
        Format a command with parameters.
        
        Args:
            command (str): The command constant
            *parameters: Variable number of parameters
            
        Returns:
            str: Formatted command string
        """
        if not parameters:
            return f"{command}\n"
        
        param_str = "#".join(str(param) for param in parameters)
        return f"{command}#{param_str}\n"
    
    @classmethod
    def parse_command(cls, command_string):
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


# Create a global instance for easy access
CMD = COMMAND()

if __name__ == '__main__':
    # Test the command system
    print("Hexapod Robot Command System")
    print("=" * 40)
    
    # Test command formatting
    move_cmd = CMD.format_command(CMD.CMD_MOVE, "forward", 50, 0, 0, 5)
    print(f"Formatted command: {move_cmd.strip()}")
    
    # Test command parsing
    parsed_cmd, params = CMD.parse_command(move_cmd)
    print(f"Parsed command: {parsed_cmd}")
    print(f"Parameters: {params}")
    
    # Test validation
    is_valid = CMD.validate_command(CMD.CMD_MOVE, ["forward", 50, 0, 0, 5])
    print(f"Command valid: {is_valid}")
    
    # Show available commands
    print("\nAvailable Commands:")
    for cmd, desc in CMD.COMMAND_DESCRIPTIONS.items():
        print(f"  {cmd}: {desc}") 