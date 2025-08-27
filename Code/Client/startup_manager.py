"""
Startup Sequence Manager for Hexapod Robot

This module handles the pre-movement startup sequence, including calibration,
self-tests, and initial movement preparation.
"""

import json
import logging
import os
import time
from typing import Dict, List, Optional, Any, Callable
from enum import Enum, auto

# Configure logging
logging.basicConfig(level=logging.INFO)
logger = logging.getLogger(__name__)

class StartupState(Enum):
    """Represents the current state of the startup sequence."""
    IDLE = auto()
    INITIALIZING = auto()
    CALIBRATING = auto()
    TESTING = auto()
    READY = auto()
    ERROR = auto()

class StartupError(Exception):
    """Custom exception for startup sequence errors."""
    def __init__(self, code: str, message: str):
        self.code = code
        self.message = message
        super().__init__(f"[{code}] {message}")

class StartupSequenceManager:
    """Manages the hexapod's startup sequence and pre-movement routines."""
    
    def __init__(self, hexapod_controller, config_path: str = 'config/startup_config.json'):
        """Initialize the startup sequence manager.
        
        Args:
            hexapod_controller: Reference to the main hexapod controller
            config_path: Path to the startup configuration file
        """
        self.hexapod = hexapod_controller
        self.config_path = config_path
        self.state = StartupState.IDLE
        self.calibration_data = {}
        self.startup_sequence = []
        self.current_step = 0
        self.callbacks = {
            'on_state_change': [],
            'on_error': [],
            'on_complete': []
        }
        
        # Load default configuration
        self._load_config()
    
    def _load_config(self) -> None:
        """Load the startup configuration from JSON file."""
        try:
            # Create config directory if it doesn't exist
            os.makedirs(os.path.dirname(self.config_path), exist_ok=True)
            
            if os.path.exists(self.config_path):
                with open(self.config_path, 'r') as f:
                    config = json.load(f)
                    self.startup_sequence = config.get('startup_sequence', [])
                    logger.info(f"Loaded startup sequence with {len(self.startup_sequence)} steps")
            else:
                # Create a default configuration if none exists
                self._create_default_config()
                
        except Exception as e:
            logger.error(f"Failed to load startup config: {e}")
            self._create_default_config()
    
    def _create_default_config(self) -> None:
        """Create a default startup configuration."""
        self.startup_sequence = [
            {"command": "power_on_self_test", "timeout": 2000},
            {"command": "calibrate_servos", "timeout": 5000},
            {"command": "center_all_servos", "timeout": 2000},
            {"command": "stand", "height": 100, "duration": 2000},
            {"command": "verify_movement_range", "timeout": 3000}
        ]
        
        try:
            with open(self.config_path, 'w') as f:
                json.dump({"startup_sequence": self.startup_sequence}, f, indent=2)
            logger.info("Created default startup configuration")
        except Exception as e:
            logger.error(f"Failed to save default config: {e}")
    
    def register_callback(self, event: str, callback: Callable) -> None:
        """Register a callback for startup events.
        
        Args:
            event: Event name ('on_state_change', 'on_error', 'on_complete')
            callback: Callback function to register
        """
        if event in self.callbacks:
            self.callbacks[event].append(callback)
    
    def _trigger_event(self, event: str, *args, **kwargs) -> None:
        """Trigger all callbacks for the specified event."""
        for callback in self.callbacks.get(event, []):
            try:
                callback(*args, **kwargs)
            except Exception as e:
                logger.error(f"Error in {event} callback: {e}")
    
    def _set_state(self, new_state: StartupState) -> None:
        """Update the current state and notify listeners."""
        old_state = self.state
        self.state = new_state
        logger.info(f"Startup state changed: {old_state.name} -> {new_state.name}")
        self._trigger_event('on_state_change', old_state, new_state)
    
    async def execute_startup_sequence(self) -> bool:
        """Execute the pre-defined startup sequence.
        
        Returns:
            bool: True if startup completed successfully, False otherwise
        """
        if self.state != StartupState.IDLE:
            logger.warning("Startup sequence already in progress")
            return False
        
        self._set_state(StartupState.INITIALIZING)
        self.current_step = 0
        
        try:
            # Execute each step in the startup sequence
            for step in self.startup_sequence:
                if not await self._execute_step(step):
                    raise StartupError(
                        "E004", 
                        f"Failed to execute step: {step.get('command', 'unknown')}"
                    )
                self.current_step += 1
            
            self._set_state(StartupState.READY)
            self._trigger_event('on_complete', True)
            return True
            
        except Exception as e:
            error_code = getattr(e, 'code', 'E000')
            error_msg = str(e)
            logger.error(f"Startup failed at step {self.current_step}: {error_msg}")
            self._set_state(StartupState.ERROR)
            self._trigger_event('on_error', error_code, error_msg)
            return False
    
    async def _execute_step(self, step: Dict[str, Any]) -> bool:
        """Execute a single step in the startup sequence.
        
        Args:
            step: Step configuration dictionary
            
        Returns:
            bool: True if step completed successfully, False otherwise
        """
        command = step.get('command', '')
        timeout = step.get('timeout', 1000) / 1000  # Convert ms to seconds
        
        logger.info(f"Executing startup step: {command}")
        
        try:
            if command == 'power_on_self_test':
                return await self.power_on_self_test(timeout)
            elif command == 'calibrate_servos':
                return await self.calibrate_servos(timeout)
            elif command == 'center_all_servos':
                return await self.center_all_servos(timeout)
            elif command == 'stand':
                height = step.get('height', 120)
                duration = step.get('duration', 2000) / 1000
                return await self.stand(height, duration)
            elif command == 'verify_movement_range':
                return await self.verify_movement_range(timeout)
            else:
                logger.warning(f"Unknown startup command: {command}")
                return False
                
        except Exception as e:
            logger.error(f"Error executing step '{command}': {e}")
            raise
    
    async def power_on_self_test(self, timeout: float = 2.0) -> bool:
        """Perform power-on self-test (POST).
        
        Args:
            timeout: Maximum time to wait for POST to complete (seconds)
            
        Returns:
            bool: True if POST passed, False otherwise
        """
        self._set_state(StartupState.TESTING)
        
        # Check battery level
        if hasattr(self.hexapod, 'get_battery_voltage'):
            voltage = self.hexapod.get_battery_voltage()
            if voltage < 10.5:  # Minimum safe voltage (adjust as needed)
                raise StartupError("E002", f"Battery voltage too low: {voltage:.1f}V")
        
        # Check servo communication
        if hasattr(self.hexapod, 'check_servo_communication'):
            if not self.hexapod.check_servo_communication():
                raise StartupError("E001", "Servo communication failure")
        
        # Add more tests as needed
        
        await asyncio.sleep(0.5)  # Simulate test time
        return True
    
    async def calibrate_servos(self, timeout: float = 5.0) -> bool:
        """Calibrate all servos.
        
        Args:
            timeout: Maximum time to wait for calibration (seconds)
            
        Returns:
            bool: True if calibration succeeded, False otherwise
        """
        self._set_state(StartupState.CALIBRATING)
        
        if hasattr(self.hexapod, 'calibrate_servos'):
            success = await self.hexapod.calibrate_servos()
            if not success:
                raise StartupError("E003", "Servo calibration failed")
        else:
            logger.warning("Hexapod controller does not support direct servo calibration")
            # Skip calibration if not supported
            await asyncio.sleep(1.0)
        
        return True
    
    async def center_all_servos(self, timeout: float = 2.0) -> bool:
        """Center all servos to their neutral positions.
        
        Args:
            timeout: Maximum time to wait for movement to complete (seconds)
            
        Returns:
            bool: True if movement completed successfully, False otherwise
        """
        if hasattr(self.hexapod, 'center_all_servos'):
            return await self.hexapod.center_all_servos()
        return False
    
    async def stand(self, height: int = 120, duration: float = 2.0) -> bool:
        """Make the hexapod stand up.
        
        Args:
            height: Target height in mm
            duration: Time to complete the movement (seconds)
            
        Returns:
            bool: True if movement completed successfully, False otherwise
        """
        if hasattr(self.hexapod, 'stand'):
            return await self.hexapod.stand(height, duration)
        return False
    
    async def verify_movement_range(self, timeout: float = 3.0) -> bool:
        """Verify that all servos can move through their full range.
        
        Args:
            timeout: Maximum time to wait for verification (seconds)
            
        Returns:
            bool: True if all servos passed verification, False otherwise
        """
        if hasattr(self.hexapod, 'verify_servo_range'):
            return await self.hexapod.verify_servo_range()
        return True  # Skip if not supported
    
    def get_status(self) -> Dict[str, Any]:
        """Get the current status of the startup sequence."""
        return {
            'state': self.state.name,
            'current_step': self.current_step,
            'total_steps': len(self.startup_sequence),
            'calibration_data': self.calibration_data
        }
