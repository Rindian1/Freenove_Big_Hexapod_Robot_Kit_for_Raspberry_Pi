"""
Movement Library for Hexapod Robot

This module provides a system for defining and executing complex movement sequences
and gait patterns for a hexapod robot.
"""

import json
import logging
import os
import asyncio
from typing import Dict, List, Any, Optional, Callable, Tuple
from dataclasses import dataclass
from enum import Enum

# Configure logging
logger = logging.getLogger(__name__)

class MovementType(Enum):
    """Types of movements supported by the movement library."""
    POSE = "pose"
    GAIT = "gait"
    SEQUENCE = "sequence"
    TRAJECTORY = "trajectory"

class GaitType(Enum):
    """Supported gait patterns."""
    TRIPOD = "tripod"
    WAVE = "wave"
    RIPPLE = "ripple"
    METACHROMATIC = "metachromatic"

@dataclass
class MovementCommand:
    """Represents a single movement command with parameters."""
    command: str
    params: Dict[str, Any]
    duration: float  # in seconds

class MovementLibrary:
    """Manages a library of movement patterns and sequences."""
    
    def __init__(self, hexapod_controller, config_dir: str = 'config'):
        """Initialize the movement library.
        
        Args:
            hexapod_controller: Reference to the main hexapod controller
            config_dir: Directory containing movement configuration files
        """
        self.hexapod = hexapod_controller
        self.config_dir = config_dir
        self.movements: Dict[str, Dict] = {}
        self.default_params = {
            'speed': 100,
            'step_length': 50,
            'step_height': 30,
            'duration': 1.0,
            'smoothing': 0.2
        }
        
        # Create config directory if it doesn't exist
        os.makedirs(self.config_dir, exist_ok=True)
        
        # Load default movements
        self._load_default_movements()
        
        # Load movements from config files
        self.load_movements()
    
    def _load_default_movements(self) -> None:
        """Load default movement definitions."""
        self.movements = {
            # Basic poses
            'stand': {
                'type': MovementType.POSE,
                'height': 120,
                'duration': 2.0,
                'description': 'Stand at the specified height'
            },
            'crouch': {
                'type': MovementType.POSE,
                'height': 60,
                'duration': 2.0,
                'description': 'Lower the body to a crouched position'
            },
            
            # Basic gaits
            'walk_forward': {
                'type': MovementType.GAIT,
                'gait': GaitType.TRIPOD,
                'step_length': 50,
                'step_height': 30,
                'speed': 100,
                'description': 'Walk forward using tripod gait'
            },
            'walk_backward': {
                'type': MovementType.GAIT,
                'gait': GaitType.TRIPOD,
                'step_length': -50,
                'step_height': 30,
                'speed': 100,
                'description': 'Walk backward using tripod gait'
            },
            'turn_left': {
                'type': MovementType.GAIT,
                'gait': GaitType.TRIPOD,
                'rotation': 30,  # degrees per step
                'step_length': 0,
                'step_height': 25,
                'speed': 80,
                'description': 'Turn in place to the left'
            },
            'turn_right': {
                'type': MovementType.GAIT,
                'gait': GaitType.TRIPOD,
                'rotation': -30,  # degrees per step
                'step_length': 0,
                'step_height': 25,
                'speed': 80,
                'description': 'Turn in place to the right'
            },
            
            # Sequences
            'dance': {
                'type': MovementType.SEQUENCE,
                'steps': [
                    {'command': 'stand', 'height': 120, 'duration': 1.0},
                    {'command': 'wave_leg', 'leg': 'front_right', 'cycles': 2},
                    {'command': 'wave_leg', 'leg': 'front_left', 'cycles': 2},
                    {'command': 'turn_left', 'steps': 3, 'speed': 120},
                    {'command': 'turn_right', 'steps': 3, 'speed': 120},
                    {'command': 'crouch', 'duration': 0.5}
                ],
                'description': 'A fun dance sequence'
            }
        }
    
    def load_movements(self, filename: str = 'movement_library.json') -> bool:
        """Load movements from a JSON configuration file.
        
        Args:
            filename: Name of the configuration file
            
        Returns:
            bool: True if movements were loaded successfully, False otherwise
        """
        config_path = os.path.join(self.config_dir, filename)
        
        try:
            if os.path.exists(config_path):
                with open(config_path, 'r') as f:
                    custom_movements = json.load(f)
                    
                # Validate and merge custom movements
                for name, movement in custom_movements.items():
                    if self._validate_movement(movement):
                        self.movements[name] = movement
                    else:
                        logger.warning(f"Skipping invalid movement: {name}")
                
                logger.info(f"Loaded {len(custom_movements)} movements from {filename}")
                return True
            else:
                # Create default config if it doesn't exist
                self._save_movements()
                return False
                
        except Exception as e:
            logger.error(f"Failed to load movements from {filename}: {e}")
            return False
    
    def _save_movements(self, filename: str = 'movement_library.json') -> bool:
        """Save the current movement library to a file.
        
        Args:
            filename: Name of the file to save to
            
        Returns:
            bool: True if save was successful, False otherwise
        """
        config_path = os.path.join(self.config_dir, filename)
        
        try:
            with open(config_path, 'w') as f:
                json.dump(self.movements, f, indent=2, default=str)
            return True
        except Exception as e:
            logger.error(f"Failed to save movements to {filename}: {e}")
            return False
    
    def _validate_movement(self, movement: Dict) -> bool:
        """Validate a movement definition.
        
        Args:
            movement: Movement definition to validate
            
        Returns:
            bool: True if movement is valid, False otherwise
        """
        try:
            # Check required fields
            if 'type' not in movement:
                return False
                
            # Validate based on movement type
            move_type = MovementType(movement['type'])
            
            if move_type == MovementType.POSE:
                required = ['height', 'duration']
            elif move_type == MovementType.GAIT:
                required = ['gait', 'step_length', 'step_height', 'speed']
                if not GaitType(movement['gait']):
                    return False
            elif move_type == MovementType.SEQUENCE:
                required = ['steps']
                if not isinstance(movement.get('steps', []), list):
                    return False
            else:
                # Unknown movement type
                return False
            
            # Check required fields are present
            return all(field in movement for field in required)
            
        except (ValueError, TypeError):
            return False
    
    def get_movement(self, name: str) -> Optional[Dict]:
        """Get a movement definition by name.
        
        Args:
            name: Name of the movement to retrieve
            
        Returns:
            Optional[Dict]: The movement definition, or None if not found
        """
        return self.movements.get(name)
    
    def list_movements(self) -> List[str]:
        """Get a list of all available movement names.
        
        Returns:
            List[str]: List of movement names
        """
        return list(self.movements.keys())
    
    async def execute_movement(self, name: str, **params) -> bool:
        """Execute a movement by name with optional parameters.
        
        Args:
            name: Name of the movement to execute
            **params: Additional parameters to override defaults
            
        Returns:
            bool: True if movement completed successfully, False otherwise
        """
        movement = self.get_movement(name)
        if not movement:
            logger.error(f"Movement not found: {name}")
            return False
        
        try:
            move_type = MovementType(movement['type'])
            
            if move_type == MovementType.POSE:
                return await self._execute_pose(movement, **params)
            elif move_type == MovementType.GAIT:
                return await self._execute_gait(movement, **params)
            elif move_type == MovementType.SEQUENCE:
                return await self._execute_sequence(movement, **params)
            else:
                logger.warning(f"Unsupported movement type: {move_type}")
                return False
                
        except Exception as e:
            logger.error(f"Error executing movement '{name}': {e}")
            return False
    
    async def _execute_pose(self, pose: Dict, **params) -> bool:
        """Execute a pose movement.
        
        Args:
            pose: Pose definition
            **params: Override parameters
            
        Returns:
            bool: True if pose was set successfully, False otherwise
        """
        # Merge default params with overrides
        pose_params = {
            'height': pose.get('height', 120),
            'duration': pose.get('duration', 2.0),
            **params
        }
        
        if hasattr(self.hexapod, 'set_pose'):
            return await self.hexapod.set_pose(**pose_params)
        elif hasattr(self.hexapod, 'stand'):
            return await self.hexapod.stand(
                height=pose_params['height'],
                duration=pose_params['duration']
            )
        else:
            logger.warning("Hexapod controller does not support pose movements")
            return False
    
    async def _execute_gait(self, gait: Dict, **params) -> bool:
        """Execute a gait movement.
        
        Args:
            gait: Gait definition
            **params: Override parameters
            
        Returns:
            bool: True if gait was executed successfully, False otherwise
        """
        # Merge default params with overrides
        gait_params = {
            'gait': GaitType(gait['gait']),
            'step_length': gait.get('step_length', 50),
            'step_height': gait.get('step_height', 30),
            'speed': gait.get('speed', 100),
            'steps': params.pop('steps', 1),  # Default to 1 step if not specified
            **params
        }
        
        if hasattr(self.hexapod, 'execute_gait'):
            return await self.hexapod.execute_gait(**gait_params)
        else:
            logger.warning("Hexapod controller does not support direct gait execution")
            return False
    
    async def _execute_sequence(self, sequence: Dict, **params) -> bool:
        """Execute a sequence of movements.
        
        Args:
            sequence: Sequence definition
            **params: Parameters to pass to child movements
            
        Returns:
            bool: True if all movements completed successfully, False otherwise
        """
        steps = sequence.get('steps', [])
        if not steps:
            logger.warning("Empty movement sequence")
            return False
        
        for step in steps:
            if 'command' not in step:
                logger.warning("Step missing 'command' field")
                return False
            
            # Execute each step in sequence
            command = step['command']
            step_params = {**step, **params}  # Merge step params with overrides
            
            # Remove the command from params to avoid recursion
            step_params.pop('command', None)
            
            if not await self.execute_movement(command, **step_params):
                logger.error(f"Sequence failed at step: {command}")
                return False
        
        return True
