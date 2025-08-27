# -*- coding: utf-8 -*-
"""
Main application for Freenove Hexapod Robot Controller

This module provides the main GUI interface for controlling the hexapod robot,
including video streaming, movement controls, and system monitoring.
"""

import sys
import math
import logging
import time
import threading
import json
import os
import asyncio
from typing import Optional, Tuple, List, Any, Dict, Callable, Union, Coroutine
from enum import Enum, auto

from PyQt5 import QtCore, QtGui, QtWidgets
from PyQt5.QtCore import QTimer, QCoreApplication, QEvent, QObject, pyqtSignal, QThread, pyqtSlot, QMutex, QMutexLocker
from PyQt5.QtWidgets import (QMainWindow, QApplication, QMessageBox, QFileDialog,
                           QVBoxLayout, QHBoxLayout, QLabel, QPushButton, QSlider,
                           QProgressBar, QCheckBox, QGroupBox, QComboBox, QSpinBox,
                           QDoubleSpinBox, QStatusBar, QTabWidget, QWidget, QFrame,
                           QProgressDialog)
from PyQt5.QtGui import QIcon, QPixmap, QImage, QPainter, QPen, QColor, QCloseEvent

# Local imports
from ui_led import Ui_led
from ui_face import Ui_Face
from ui_client import Ui_client
from Client import Client
from Calibration import *
from camera_recording import CameraRecorder
from Command import COMMAND as cmd
from PID import Incremental_PID
from Thread import SafeThread, ThreadManager, stop_thread

# Configuration and movement system imports
from config_manager import ConfigManager, get_config_manager
from startup_manager import StartupSequenceManager, StartupState, StartupError
from movement_library import MovementLibrary, MovementType, GaitType
from gait_patterns import GaitController, LegPosition, GaitState

# Configure logging
logging.basicConfig(
    level=logging.INFO,
    format='%(asctime)s - %(name)s - %(levelname)s - %(message)s',
    handlers=[
        logging.StreamHandler(),
        logging.FileHandler('hexapod_controller.log')
    ]
)
logger = logging.getLogger(__name__)

class HexapodController(QMainWindow, Ui_client):
    """
    Main controller class for the Hexapod Robot GUI.
    
    Handles the user interface, command processing, and communication with the robot.
    """
    
    # Signal for updating the UI from other threads
    update_signal = pyqtSignal(str, object)
    
    # Startup sequence signals
    startup_progress = pyqtSignal(int, str)  # progress percentage, status message
    startup_complete = pyqtSignal(bool)  # True if completed successfully, False if failed
    
    # Movement control signals
    movement_complete = pyqtSignal(str, bool)  # movement name, success
    
    # Thread management
    _thread_manager = ThreadManager()
    _thread_lock = QMutex()
    
    def __init__(self, parent=None):
        """Initialize the Hexapod Controller application."""
        super(HexapodController, self).__init__(parent)
        self.setupUi(self)
        
        # Initialize configuration
        self.config = get_config_manager()
        self._load_configurations()
        
        # Initialize UI components
        self._setup_ui()
        
        # Initialize robot client with config
        self.client = Client()
        
        # Initialize PID controllers for smooth movements
        self._init_pid_controllers()
        
        # Initialize movement system with config
        self._init_movement_system()
        
        # Initialize startup sequence manager with config
        self._init_startup_sequence()
        
        # Connect signals
        self._connect_signals()
        
        # Initialize timers with config
        self._init_timers()
        
        # Load settings
        self._load_settings()
        
        # Initialize state
        self._movement_queue = asyncio.Queue()
        self._current_movement = None
        self._is_moving = False
        
        # Start the movement processing task
        self._movement_task = asyncio.create_task(self._process_movement_queue())
        
        logger.info("Hexapod Controller initialized with configuration")
        
    def _load_configurations(self):
        """Load all configuration files."""
        try:
            # Load main configuration
            self.robot_config = self.config.load_config("hexapod_config")
            
            # Load movement configurations
            self.movement_config = self.config.load_config("movements")
            
            # Load startup sequence
            self.startup_config = self.config.load_config("startup_sequence")
            
            # Update UI based on config
            self._update_ui_from_config()
            
        except Exception as e:
            logger.error(f"Failed to load configurations: {e}")
            # Fall back to default settings if config loading fails
            self._load_default_config()
            
    def _load_default_config(self):
        """Load default configuration values."""
        self.robot_config = {
            "robot": {
                "name": "Freenove Hexapod",
                "version": "2.0",
                "default_leg_angles": {
                    "coxa": 0,
                    "femur": 45,
                    "tibia": -90
                },
                "servo_limits": {
                    "coxa": {"min": 0, "max": 180, "default": 90},
                    "femur": {"min": 0, "max": 180, "default": 90},
                    "tibia": {"min": 0, "max": 180, "default": 90}
                }
            },
            "movement": {
                "default_speed": 50,
                "max_speed": 100,
                "min_speed": 10,
                "step_height": 30,
                "step_length": 50,
                "body_height": 100,
                "gait_cycle_time": 1000
            }
        }
        
    def _update_ui_from_config(self):
        """Update UI elements based on loaded configuration."""
        try:
            # Update window title with robot name
            robot_name = self.robot_config.get("robot", {}).get("name", "Hexapod Robot")
            self.setWindowTitle(f"{robot_name} Controller")
            
            # Update movement parameters
            movement_cfg = self.robot_config.get("movement", {})
            if "default_speed" in movement_cfg:
                self.speedSlider.setValue(movement_cfg["default_speed"])
                
            # Update other UI elements as needed
            
        except Exception as e:
            logger.error(f"Failed to update UI from config: {e}")
    
    def _init_movement_system(self) -> None:
        """Initialize the movement system components with configuration."""
        try:
            # Get movement configuration
            movement_cfg = self.robot_config.get("movement", {})
            
            # Initialize movement library with config
            self.movement_library = MovementLibrary(self)
            
            # Load movements from config if available
            if hasattr(self, 'movement_config') and 'poses' in self.movement_config:
                for name, pose in self.movement_config['poses'].items():
                    self.movement_library.add_movement('pose', name, pose)
                    
                for name, gait in self.movement_config.get('gaits', {}).items():
                    self.movement_library.add_movement('gait', name, gait)
                    
                for name, sequence in self.movement_config.get('sequences', {}).items():
                    self.movement_library.add_movement('sequence', name, sequence)
            
            # Initialize gait controller with config
            gait_config = {
                'step_height': movement_cfg.get('step_height', 30),
                'step_length': movement_cfg.get('step_length', 50),
                'body_height': movement_cfg.get('body_height', 100),
                'cycle_time': movement_cfg.get('gait_cycle_time', 1000)
            }
            self.gait_controller = GaitController(self, **gait_config)
            
            logger.info("Movement system initialized with configuration")
            
        except Exception as e:
            logger.error(f"Failed to initialize movement system: {e}")
            # Fall back to defaults
            self.movement_library = MovementLibrary(self)
            self.gait_controller = GaitController(self)
        
        # Initialize movement queue
        self._movement_queue = asyncio.Queue()
        self._current_movement = None
        self._is_moving = False
        
        # Movement state
        self._movement_task = None
        self._stop_movement_flag = False
        
        logger.info("Movement system initialized")
    
    def _setup_ui(self) -> None:
        """Set up the user interface components."""
        # Set window icon and title
        self.setWindowIcon(QIcon('Picture/logo_Mini.png'))
        self.setWindowTitle("Hexapod Robot Controller")
        
        # Set video display properties
        self.Video.setScaledContents(True)
        self.Video.setPixmap(QPixmap('Picture/Spider_client.png'))
        
        # Initialize camera recorder
        self.camera_recorder = CameraRecorder(
            output_dir='Captures', 
            video_label=self.Video
        )
        
        # Initialize movement controls
        self._init_movement_ui()
        
        # Initialize status bar
        self.statusBar().showMessage("Ready")
        
        # Create config directory if it doesn't exist
        os.makedirs('config', exist_ok=True)
    
    def _init_movement_ui(self) -> None:
        """Initialize movement-related UI components."""
        # Movement controls
        self.Button_Startup = QPushButton("Startup Sequence", self)
        self.Button_Startup.clicked.connect(self.start_startup_sequence)
        
        self.Button_Stop = QPushButton("Stop", self)
        self.Button_Stop.clicked.connect(self.stop_movement)
        
        # Add buttons to layout (adjust based on your UI layout)
        # This is an example - modify according to your actual UI structure
        if hasattr(self, 'horizontalLayout_Controls'):
            self.horizontalLayout_Controls.addWidget(self.Button_Startup)
            self.horizontalLayout_Controls.addWidget(self.Button_Stop)
    
    def _init_startup_sequence(self):
        """Initialize the startup sequence manager with configuration."""
        try:
            # Initialize with config if available, otherwise use defaults
            if hasattr(self, 'startup_config') and 'sequence' in self.startup_config:
                self.startup_manager = StartupSequenceManager(self.startup_config)
                logger.info("Loaded startup sequence from config")
            else:
                # Fallback to default sequence
                default_sequence = [
                    {"name": "Initializing", "duration": 1000},
                    {"name": "Checking Motors", "duration": 2000},
                    {"name": "Calibrating Sensors", "duration": 1500},
                    {"name": "Loading Configuration", "duration": 1000},
                    {"name": "Ready", "duration": 500}
                ]
                self.startup_manager = StartupSequenceManager({"sequence": default_sequence})
                logger.warning("Using default startup sequence")
            
            # Connect signals
            self.startup_manager.progress_updated.connect(self._on_startup_progress)
            self.startup_manager.sequence_complete.connect(self._on_startup_complete)
            self.startup_manager.state_changed.connect(self._on_startup_state_change)
            self.startup_manager.error_occurred.connect(self._on_startup_error)
            
            # Add custom actions if defined in config
            if hasattr(self, 'startup_config') and 'custom_actions' in self.startup_config:
                for action in self.startup_config['custom_actions']:
                    self.startup_manager.add_custom_action(
                        action['name'],
                        getattr(self, action['method']),
                        action.get('params', {})
                    )
            
            logger.info("Startup sequence initialized with configuration")
            
        except Exception as e:
            logger.error(f"Failed to initialize startup sequence: {e}")
            # Fall back to minimal working configuration
            self.startup_manager = StartupSequenceManager({
                "sequence": [{"name": "Ready", "duration": 500}]
            })
    
    def _setup_connections(self) -> None:
        """Set up signal-slot connections."""
        # Button connections
        self.Button_Connect.clicked.connect(self.connect_to_robot)
        self.Button_Video.clicked.connect(self.toggle_video_stream)
        self.Button_Calibration.clicked.connect(self.show_calibration_window)
        self.Button_LED.clicked.connect(self.show_led_window)
        self.Button_Face_ID.clicked.connect(self.show_face_window)
        self.Button_Take_Photo.clicked.connect(self.take_photo)
        self.Button_Relax.clicked.connect(self.relax_servos)
        self.Button_Buzzer.pressed.connect(lambda: self.control_buzzer(True))
        self.Button_Buzzer.released.connect(lambda: self.control_buzzer(False))
        
        # Startup sequence controls
        if hasattr(self, 'checkBox_StartupMovements'):
            self.checkBox_StartupMovements.setChecked(True)
            self.checkBox_StartupMovements.toggled.connect(self.toggle_startup_movements)
        if hasattr(self, 'Button_StartupSequence'):
            self.Button_StartupSequence.clicked.connect(self.start_startup_sequence)
        
        # Slider connections
        self.slider_head.valueChanged.connect(self.update_head_position)
        self.slider_head_1.valueChanged.connect(self.update_head_rotation)
        self.slider_speed.valueChanged.connect(self.update_movement_speed)
        self.slider_roll.valueChanged.connect(self.update_roll)
        self.slider_Z.valueChanged.connect(self.update_height)
        
        # Checkbox connections
        self.ButtonActionMode1.toggled.connect(self.set_action_mode)
        self.ButtonActionMode2.toggled.connect(self.set_action_mode)
        self.ButtonGaitMode1.toggled.connect(self.set_gait_mode)
        self.ButtonGaitMode2.toggled.connect(self.set_gait_mode)
        
        # Update signal connection
        self.update_signal.connect(self.update_ui)
    
    def _init_timers(self) -> None:
        """Initialize all timers used in the application with configuration."""
        try:
            # Get timer intervals from config or use defaults
            timer_config = self.robot_config.get('timers', {})
            
            # Video refresh timer (default: 30 FPS)
            self.video_timer = QTimer(self)
            self.video_timer.timeout.connect(self.refresh_video)
            video_interval = timer_config.get('video_interval', 33)  # ms
            self.video_timer.setInterval(video_interval)
            
            # Power monitoring timer (default: 1 second)
            self.power_timer = QTimer(self)
            self.power_timer.timeout.connect(self.update_power_status)
            power_interval = timer_config.get('power_interval', 1000)  # ms
            self.power_timer.setInterval(power_interval)
            
            # Sonic sensor timer (default: 100ms)
            self.sonic_timer = QTimer(self)
            self.sonic_timer.timeout.connect(self.update_sonic_reading)
            sonic_interval = timer_config.get('sonic_interval', 100)  # ms
            self.sonic_timer.setInterval(sonic_interval)
            
            # Movement status update timer (default: 100ms)
            self.movement_timer = QTimer(self)
            self.movement_timer.timeout.connect(self._update_movement_status)
            movement_interval = timer_config.get('movement_interval', 100)  # ms
            self.movement_timer.setInterval(movement_interval)
            
            logger.info(f"Timers initialized with intervals - Video: {video_interval}ms, "
                      f"Power: {power_interval}ms, Sonic: {sonic_interval}ms, "
                      f"Movement: {movement_interval}ms")
            
        except Exception as e:
            logger.error(f"Failed to initialize timers: {e}")
            # Fall back to default intervals
            self.video_timer = QTimer(self)
            self.video_timer.timeout.connect(self.refresh_video)
            self.video_timer.setInterval(33)
            
            self.power_timer = QTimer(self)
            self.power_timer.timeout.connect(self.update_power_status)
            self.power_timer.setInterval(1000)
            
            self.sonic_timer = QTimer(self)
            self.sonic_timer.timeout.connect(self.update_sonic_reading)
            self.sonic_timer.setInterval(100)
            
            self.movement_timer = QTimer(self)
            self.movement_timer.timeout.connect(self._update_movement_status)
            self.movement_timer.setInterval(100)  # Update every 100ms
    
    def _init_pid_controllers(self) -> None:
        """Initialize PID controllers with configuration values."""
        try:
            # Get PID configuration or use defaults
            pid_config = self.robot_config.get('pid', {})
            
            # Position control PID (X, Y, Z)
            pos_pid_cfg = pid_config.get('position', {'P': 1.0, 'I': 0.05, 'D': 0.2, 'limit': 100, 'rate_limit': 50})
            self.position_pid = Incremental_PID(
                P=pos_pid_cfg.get('P', 1.0),
                I=pos_pid_cfg.get('I', 0.05),
                D=pos_pid_cfg.get('D', 0.2),
                setpoint=0.0
            )
            self.position_pid.set_output_limits(-pos_pid_cfg.get('limit', 100), pos_pid_cfg.get('limit', 100))
            self.position_pid.set_rate_limit(pos_pid_cfg.get('rate_limit', 50))
            
            # Attitude control PID (roll, pitch, yaw)
            att_pid_cfg = pid_config.get('attitude', {'P': 0.8, 'I': 0.02, 'D': 0.15, 'limit': 100, 'rate_limit': 30})
            self.attitude_pid = Incremental_PID(
                P=att_pid_cfg.get('P', 0.8),
                I=att_pid_cfg.get('I', 0.02),
                D=att_pid_cfg.get('D', 0.15),
                setpoint=0.0
            )
            self.attitude_pid.set_output_limits(-att_pid_cfg.get('limit', 100), att_pid_cfg.get('limit', 100))
            self.attitude_pid.set_rate_limit(att_pid_cfg.get('rate_limit', 30))
            
            # Leg movement PID
            leg_pid_cfg = pid_config.get('leg', {'P': 1.2, 'I': 0.1, 'D': 0.25, 'limit': 100, 'rate_limit': 100})
            self.leg_pid = Incremental_PID(
                P=leg_pid_cfg.get('P', 1.2),
                I=leg_pid_cfg.get('I', 0.1),
                D=leg_pid_cfg.get('D', 0.25),
                setpoint=0.0
            )
            self.leg_pid.set_output_limits(-leg_pid_cfg.get('limit', 100), leg_pid_cfg.get('limit', 100))
            self.leg_pid.set_rate_limit(leg_pid_cfg.get('rate_limit', 100))
            
            logger.info("PID controllers initialized with configuration")
            
        except Exception as e:
            logger.error(f"Failed to initialize PID controllers: {e}")
            # Fall back to default values
            self.position_pid = Incremental_PID(P=1.0, I=0.05, D=0.2, setpoint=0.0)
            self.position_pid.set_output_limits(-100, 100)
            self.position_pid.set_rate_limit(50)
            
            self.attitude_pid = Incremental_PID(P=0.8, I=0.02, D=0.15, setpoint=0.0)
            self.attitude_pid.set_output_limits(-100, 100)
            self.attitude_pid.set_rate_limit(30)
            
            self.leg_pid = Incremental_PID(P=1.2, I=0.1, D=0.25, setpoint=0.0)
            self.leg_pid.set_output_limits(-100, 100)
            self.leg_pid.set_rate_limit(100)
    
    def _get_startup_movements(self) -> List[Dict[str, Any]]:
        """Define the sequence of movements for the startup routine."""
        return [
            {
                'name': 'servo_power_on',
                'command': cmd.CMD_SERVOPOWER,
                'params': ['1'],  # Turn on servos
                'delay': 1000,    # 1 second delay
                'use_pid': False,
                'description': 'Powering on servos...'
            },
            {
                'name': 'calibration_check',
                'command': cmd.CMD_CALIBRATION,
                'params': [],
                'delay': 500,
                'use_pid': False,
                'description': 'Checking calibration...'
            },
            {
                'name': 'reset_attitude',
                'command': cmd.CMD_ATTITUDE,
                'params': ['0', '0', '0'],  # Roll, pitch, yaw
                'delay': 800,
                'use_pid': True,
                'pid_target': [0, 0, 0],
                'description': 'Resetting attitude...'
            },
            {
                'name': 'reset_position',
                'command': cmd.CMD_POSITION,
                'params': ['0', '0', '0'],  # X, Y, Z
                'delay': 800,
                'use_pid': True,
                'pid_target': [0, 0, 0],
                'description': 'Centering position...'
            },
            {
                'name': 'leg_stretch_test',
                'command': cmd.CMD_POSITION,
                'params': ['0', '0', '5'],  # Slight lift
                'delay': 1000,
                'use_pid': True,
                'pid_target': [0, 0, 5],
                'description': 'Testing leg movement...'
            },
            {
                'name': 'return_to_neutral',
                'command': cmd.CMD_POSITION,
                'params': ['0', '0', '0'],
                'delay': 1000,
                'use_pid': True,
                'pid_target': [0, 0, 0],
                'description': 'Returning to neutral...'
            },
            {
                'name': 'head_center',
                'command': cmd.CMD_HEAD,
                'params': ['0', '90', '1', '90'],  # Pan, tilt
                'delay': 500,
                'use_pid': False,
                'description': 'Centering head...'
            },
            {
                'name': 'ready_signal',
                'command': cmd.CMD_LED_MOD,
                'params': ['2'],  # Ready pattern
                'delay': 1000,
                'use_pid': False,
                'description': 'Initialization complete!'
            }
        ]
    
    def _init_startup_sequence(self) -> None:
        """Initialize the startup sequence state."""
        self._startup_sequence_running = False
        self._startup_movement_index = 0
        self._startup_timer = QTimer(self)
        self._startup_timer.timeout.connect(self._execute_startup_movement)
        self._startup_timer.setSingleShot(True)
    
    def start_startup_sequence(self) -> None:
        """Start the startup sequence in a separate thread."""
        if not self.client.tcp_flag:
            logger.warning("Cannot start sequence: Not connected to robot")
            self.statusBar().showMessage("Error: Not connected to robot")
            return
            
        if self._startup_sequence_running:
            logger.info("Startup sequence already running")
            return
            
        logger.info("Starting hexapod startup sequence...")
        self._startup_sequence_running = True
        self._startup_movement_index = 0
        
        # Disable UI controls during startup
        self.set_controls_enabled(False)
        
        # Start the sequence in a separate thread
        self._startup_thread = SafeThread(
            target=self._run_startup_sequence,
            name="StartupSequence"
        )
        self._thread_manager.add_thread(self._startup_thread)
        self._startup_thread.start()
        
        # Update UI
        self.startup_progress.emit(0, "Starting up...")
        self.statusBar().showMessage("Startup sequence started")
    
    def stop_startup_sequence(self) -> None:
        """Stop the currently running startup sequence."""
        if not self._startup_sequence_running:
            return
            
        logger.info("Stopping startup sequence...")
        self._startup_sequence_running = False
        
        # Stop the timer if it's active
        if self._startup_timer.isActive():
            self._startup_timer.stop()
        
        # Re-enable controls
        self.set_controls_enabled(True)
        
        # Update UI
        self.startup_progress.emit(0, "Startup sequence stopped")
        self.statusBar().showMessage("Startup sequence stopped")
    
    def _run_startup_sequence(self) -> None:
        """Execute the startup sequence in a thread."""
        try:
            total_movements = len(self._startup_movements)
            
            while (self._startup_sequence_running and 
                   self._startup_movement_index < total_movements):
                
                movement = self._startup_movements[self._startup_movement_index]
                
                # Update progress
                progress = int((self._startup_movement_index / total_movements) * 100)
                self.startup_progress.emit(progress, movement['description'])
                
                # Execute the movement
                self._execute_movement(movement)
                
                # Wait for the movement to complete
                time.sleep(movement['delay'] / 1000.0)
                
                # Move to next movement
                self._startup_movement_index += 1
            
            # Sequence completed
            self._startup_sequence_running = False
            success = self._startup_movement_index >= total_movements
            self.startup_complete.emit(success)
            
        except Exception as e:
            logger.error(f"Error in startup sequence: {e}", exc_info=True)
            self._startup_sequence_running = False
            self.startup_complete.emit(False)
    
    def _execute_movement(self, movement: Dict[str, Any]) -> None:
        """Execute a single movement from the startup sequence."""
        try:
            if movement.get('use_pid', False):
                self._execute_pid_controlled_movement(movement)
            else:
                self._execute_simple_movement(movement)
                
        except Exception as e:
            logger.error(f"Error executing movement {movement.get('name', 'unknown')}: {e}")
            raise
    
    def _execute_pid_controlled_movement(self, movement: Dict[str, Any]) -> None:
        """Execute a movement with PID control for smooth operation."""
        command = movement['command']
        target_params = [float(p) for p in movement['params']]
        steps = 10  # Number of intermediate steps
        
        for step in range(1, steps + 1):
            if not self._startup_sequence_running:
                break
                
            # Calculate progress (0.0 to 1.0)
            progress = step / steps
            
            # Interpolate each parameter
            current_params = []
            for i, target in enumerate(target_params):
                # Simple linear interpolation
                current = target * progress
                current_params.append(str(int(current)))
            
            # Build and send command
            cmd_str = f"{command}#{'#'.join(current_params)}\n"
            self.client.send_data(cmd_str)
            
            # Small delay between steps
            time.sleep(0.05)
    
    def _execute_simple_movement(self, movement: Dict[str, Any]) -> None:
        """Execute a simple movement command."""
        command = movement['command']
        params = movement['params']
        
        # Build command string
        if command == cmd.CMD_SERVOPOWER:
            cmd_str = f"{command}#{params[0]}\n"
        elif command == cmd.CMD_HEAD:
            cmd_str = f"{command}#{params[0]}#{params[1]}\n"
        elif command == cmd.CMD_LED_MOD:
            cmd_str = f"{command}#{params[0]}\n"
        elif command == cmd.CMD_CALIBRATION:
            cmd_str = f"{command}\n"
        else:
            cmd_str = f"{command}#{'#'.join(params)}\n"
        # Send the command
        self.client.send_data(cmd_str)
    
    def _update_startup_progress(self, progress: int, message: str) -> None:
        """Update the UI with startup progress."""
        self.startupProgressBar.setValue(progress)
        self.startupStatusLabel.setText(message)
    
    def _on_startup_complete(self, success: bool) -> None:
        """Handle completion of the startup sequence."""
        self._startup_sequence_running = False
        self.set_controls_enabled(True)
        
        if success:
            logger.info("Startup sequence completed successfully")
            self.statusBar().showMessage("Startup sequence completed")
            self.startup_progress.emit(100, "Ready")
        else:
            logger.warning("Startup sequence was interrupted or failed")
            self.statusBar().showMessage("Startup sequence failed")
            self.startup_progress.emit(0, "Startup failed")
    
    def set_controls_enabled(self, enabled: bool) -> None:
        """Enable or disable UI controls."""
        # Example controls - adjust based on your actual UI
        for widget in [
            self.Button_Connect,
            self.Button_Video,
            self.Button_Calibration,
            self.Button_LED,
            self.Button_Face_ID,
            self.Button_Take_Photo,
            self.Button_Relax,
            self.Button_Buzzer,
            self.slider_head,
            self.slider_head_1,
            self.slider_speed,
            self.slider_roll,
            self.slider_Z
        ]:
            widget.setEnabled(enabled)
    
    def _stop_threads(self) -> None:
        """Safely stop all running threads."""
        logger.info("Stopping all threads...")
        
        # List of thread attributes to check and stop
        thread_attrs = [
            '_video_thread', '_command_thread', '_power_thread',
            '_sonic_thread', '_startup_thread', 'receive_thread'
        ]
        
        for thread_attr in thread_attrs:
            thread = getattr(self, thread_attr, None)
            if thread and isinstance(thread, (threading.Thread, QThread)):
                try:
                    # For SafeThread instances
                    if hasattr(thread, 'stop'):
                        thread.stop()
                    # For QThreads
                    elif hasattr(thread, 'requestInterruption'):
                        thread.requestInterruption()
                        if thread.isRunning():
                            thread.quit()
                            if not thread.wait(1000):  # Wait up to 1 second
                                thread.terminate()
                    # For regular Python threads
                    elif hasattr(thread, 'is_alive') and thread.is_alive():
                        # Set daemon to True to allow program exit even if thread is still running
                        thread.daemon = True
                    
                    logger.debug(f"Stopped thread: {thread_attr}")
                    
                except Exception as e:
                    logger.error(f"Error stopping thread {thread_attr}: {e}")
    
    def _load_settings(self) -> None:
        """Load application settings from file."""
        try:
            with open('IP.txt', 'r') as f:
                ip_address = f.readline().strip()
                self.lineEdit_IP_Adress.setText(ip_address)
                logger.info(f"Loaded IP address: {ip_address}")
        except FileNotFoundError:
            logger.warning("IP.txt not found, using default settings")
            self.lineEdit_IP_Adress.setText("192.168.1.1")  # Default IP
        except Exception as e:
            logger.error(f"Error loading settings: {e}")
    
    def connect_to_robot(self) -> None:
        """Establish connection to the robot."""
        ip_address = self.lineEdit_IP_Adress.text().strip()
        if not ip_address:
            self.show_error("Error", "Please enter a valid IP address")
            return
            
        try:
            self.client.turn_on_client(ip_address)
            # Start communication threads
            self.start_communication_threads()
            self.Button_Connect.setText("Disconnect")
            logger.info(f"Connected to robot at {ip_address}")
            
            # Start startup sequence if enabled
            if hasattr(self, 'startup_movements_enabled') and self.startup_movements_enabled:
                QTimer.singleShot(2000, self.start_startup_sequence)  # Wait 2s for connection
        except Exception as e:
            self.show_error("Connection Error", f"Failed to connect: {str(e)}")
            logger.error(f"Connection failed: {e}")
    
    def start_communication_threads(self) -> None:
        """Start all communication threads with the robot."""
        # Start video thread
        self.video_thread = threading.Thread(
            target=self.client.receiving_video,
            args=(self.lineEdit_IP_Adress.text().strip(),),
            daemon=True
        )
        self.video_thread.start()
        
        # Start command thread
        self.command_thread = threading.Thread(
            target=self.receive_commands,
            daemon=True
        )
        self.command_thread.start()
        
        # Start timers
        self.video_timer.start(30)  # ~33 FPS
        self.power_timer.start(1000)  # Update power every second
        self.sonic_timer.start(500)   # Update sonic sensor every 500ms
    
    def toggle_video_stream(self) -> None:
        """Toggle video streaming on/off."""
        try:
            if self.Button_Video.text() == 'Open Video':
                if not self.video_timer.isActive():
                    self.video_timer.start(30)
                    self.Button_Video.setText('Close Video')
                    logger.info("Video stream started")
            else:
                if self.video_timer.isActive():
                    self.video_timer.stop()
                self.Button_Video.setText('Open Video')
                logger.info("Video stream stopped")
        except Exception as e:
            self.show_error("Video Error", f"Failed to toggle video: {str(e)}")
            logger.error(f"Video toggle error: {e}")
    
    def update_power_status(self) -> None:
        """Request and update the power status from the robot."""
        if not hasattr(self, 'client') or not self.client.tcp_flag:
            return
            
        try:
            command = f"{cmd.CMD_POWER}\n"
            self.client.send_data(command)
            
            if hasattr(self, 'power_value') and self.power_value:
                try:
                    # Update battery 1
                    voltage1 = float(self.power_value[0])
                    self.progress_Power1.setFormat(f"{voltage1:.1f}V")
                    self.progress_Power1.setValue(
                        self._calculate_battery_percentage(voltage1, 5.0, 8.4)  # LiPo 2S range
                    )
                    
                    # Update battery 2 if available
                    if len(self.power_value) > 1:
                        voltage2 = float(self.power_value[1])
                        self.progress_Power2.setFormat(f"{voltage2:.1f}V")
                        self.progress_Power2.setValue(
                            self._calculate_battery_percentage(voltage2, 7.0, 8.4)  # LiPo 2S range
                        )
                    
                    # Visual indicators for low battery
                    if voltage1 < 6.5:  # Warning threshold for 2S LiPo
                        self.progress_Power1.setStyleSheet(
                            "QProgressBar::chunk { background-color: red; }"
                        )
                    else:
                        self.progress_Power1.setStyleSheet("")
                        
                except (ValueError, IndexError) as e:
                    logger.error(f"Invalid power value format: {self.power_value}")
                    
        except Exception as e:
            logger.error(f"Error updating power status: {e}")
    
    def _calculate_battery_percentage(self, voltage: float, min_v: float, max_v: float) -> int:
        """Calculate battery percentage based on voltage."""
        return min(100, max(0, int((voltage - min_v) / (max_v - min_v) * 100)))
    
    def update_sonic_reading(self) -> None:
        """Update the ultrasonic sensor reading."""
        if not hasattr(self, 'client') or not self.client.tcp_flag:
            return
            
        try:
            command = f"{cmd.CMD_SONIC}\n"
            self.client.send_data(command)
        except Exception as e:
            logger.error(f"Error updating sonic reading: {e}")
    
    def refresh_video(self) -> None:
        """Refresh the video display."""
        try:
            if hasattr(self.client, 'image') and self.client.image is not None:
                # Convert the image to QPixmap and display it
                height, width, channel = self.client.image.shape
                bytes_per_line = 3 * width
                q_img = QImage(
                    self.client.image.data, 
                    width, 
                    height, 
                    bytes_per_line, 
                    QImage.Format_RGB888
                ).rgbSwapped()
                
                self.Video.setPixmap(QPixmap.fromImage(q_img))
        except Exception as e:
            logger.error(f"Error refreshing video: {e}")
    
    def receive_commands(self) -> None:
        """Thread function to receive and process commands from the robot."""
        while getattr(self, 'client', None) and hasattr(self.client, 'tcp_flag') and self.client.tcp_flag:
            try:
                data = self.client.receive_data()
                if data:
                    self.process_received_data(data)
            except Exception as e:
                logger.error(f"Error receiving data: {e}")
                time.sleep(0.1)  # Prevent tight loop on error
    
    def process_received_data(self, data: str) -> None:
        """Process data received from the robot."""
        try:
            if data.startswith(cmd.CMD_POWER):
                # Format: CMD_POWER#voltage1#voltage2
                parts = data.strip().split('#')
                if len(parts) >= 2:
                    self.power_value = parts[1:]
                    self.update_signal.emit('power', self.power_value)
                    
            elif data.startswith(cmd.CMD_SONIC):
                # Format: CMD_SONIC#distance
                parts = data.strip().split('#')
                if len(parts) >= 2:
                    distance = float(parts[1])
                    self.update_signal.emit('sonic', distance)
                    
            # Add more command handlers as needed
            
        except Exception as e:
            logger.error(f"Error processing received data: {e}")
    
    @pyqtSlot(str, object)
    def update_ui(self, data_type: str, value: Any) -> None:
        """Update the UI from a non-GUI thread."""
        if data_type == 'power':
            # Update power display
            pass  # Handled in update_power_status
        elif data_type == 'sonic':
            self.label_Sonic.setText(f"{value:.1f} cm")
    
    # Movement control methods
    
    async def execute_movement(self, movement_name: str, **params) -> bool:
        """Execute a movement by name with optional parameters.
        
        Args:
            movement_name: Name of the movement to execute
            **params: Additional parameters for the movement
            
        Returns:
            bool: True if movement was queued successfully, False otherwise
        """
        try:
            # Add movement to the queue
            await self._movement_queue.put((movement_name, params))
            return True
        except Exception as e:
            logger.error(f"Failed to queue movement '{movement_name}': {e}")
            return False
    
    async def _process_movement_queue(self) -> None:
        """Process the movement queue in the background."""
        while True:
            try:
                # Get the next movement from the queue
                movement_name, params = await self._movement_queue.get()
                
                # Skip if already moving and movement is not queuable
                if self._is_moving and not params.get('queuable', False):
                    logger.warning(f"Skipping movement '{movement_name}' - another movement is in progress")
                    continue
                
                # Execute the movement
                self._current_movement = movement_name
                self._is_moving = True
                
                try:
                    # Execute the movement
                    success = await self._execute_movement(movement_name, **params)
                    
                    # Emit completion signal
                    self.movement_complete.emit(movement_name, success)
                    
                except Exception as e:
                    logger.error(f"Error executing movement '{movement_name}': {e}")
                    self.movement_complete.emit(movement_name, False)
                
                # Mark task as done
                self._movement_queue.task_done()
                
            except asyncio.CancelledError:
                # Handle task cancellation
                logger.info("Movement processing task cancelled")
                break
            except Exception as e:
                logger.error(f"Error in movement processing task: {e}")
                await asyncio.sleep(1)  # Prevent tight loop on errors
    
    async def _execute_movement(self, movement_name: str, **params) -> bool:
        """Execute a movement from the movement library.
        
        Args:
            movement_name: Name of the movement to execute
            **params: Additional parameters for the movement
            
        Returns:
            bool: True if movement completed successfully, False otherwise
        """
        logger.info(f"Executing movement: {movement_name} with params: {params}")
        
        try:
            # Check if movement exists in the library
            if not self.movement_library.has_movement(movement_name):
                logger.error(f"Movement not found: {movement_name}")
                return False
            
            # Get the movement definition
            movement = self.movement_library.get_movement(movement_name)
            
            # Execute based on movement type
            if movement['type'] == 'pose':
                return await self._execute_pose_movement(movement, **params)
            elif movement['type'] == 'gait':
                return await self._execute_gait_movement(movement, **params)
            elif movement['type'] == 'sequence':
                return await self._execute_sequence_movement(movement, **params)
            else:
                logger.error(f"Unsupported movement type: {movement['type']}")
                return False
                
        except Exception as e:
            logger.error(f"Error executing movement '{movement_name}': {e}")
            return False
    
    async def _execute_pose_movement(self, movement: Dict, **params) -> bool:
        """Execute a pose movement.
        
        Args:
            movement: Movement definition
            **params: Override parameters
            
        Returns:
            bool: True if movement completed successfully, False otherwise
        """
        # Merge movement params with overrides
        pose_params = {
            'height': movement.get('height', 120),
            'duration': movement.get('duration', 2.0),
            **params
        }
        
        # Execute the pose
        return await self.stand(pose_params['height'], pose_params['duration'])
    
    async def _execute_gait_movement(self, movement: Dict, **params) -> bool:
        """Execute a gait movement.
        
        Args:
            movement: Movement definition
            **params: Override parameters
            
        Returns:
            bool: True if movement completed successfully, False otherwise
        """
        # Merge movement params with overrides
        gait_params = {
            'gait': movement.get('gait', 'tripod'),
            'step_length': movement.get('step_length', 50),
            'step_height': movement.get('step_height', 30),
            'speed': movement.get('speed', 100),
            'steps': params.get('steps', 1),
            **params
        }
        
        # Configure the gait
        gait_type = gait_params['gait'].lower()
        speed = max(1, min(100, gait_params['speed']))
        step_length = gait_params['step_length']
        step_height = gait_params['step_height']
        steps = max(1, int(gait_params.get('steps', 1)))
        
        # Set the appropriate gait
        if gait_type == 'tripod':
            await self.gait_controller.set_tripod_gait(
                speed=speed,
                step_length=step_length,
                step_height=step_height
            )
        elif gait_type == 'wave':
            await self.gait_controller.set_wave_gait(
                speed=speed,
                step_length=step_length,
                step_height=step_height
            )
        elif gait_type == 'ripple':
            await self.gait_controller.set_ripple_gait(
                speed=speed,
                step_length=step_length,
                step_height=step_height
            )
        else:
            logger.error(f"Unsupported gait type: {gait_type}")
            return False
        
        # Start the gait
        await self.gait_controller.start()
        
        # Wait for the specified number of steps
        step_duration = self.gait_controller.cycle_time / 2  # Approximate time per step
        await asyncio.sleep(step_duration * steps)
        
        # Stop the gait
        await self.gait_controller.stop()
        
        return True
    
    async def _execute_sequence_movement(self, movement: Dict, **params) -> bool:
        """Execute a sequence of movements.
        
        Args:
            movement: Movement definition
            **params: Override parameters
            
        Returns:
            bool: True if all movements completed successfully, False otherwise
        """
        steps = movement.get('steps', [])
        if not steps:
            logger.warning("Empty movement sequence")
            return False
        
        # Execute each step in sequence
        for step in steps:
            if 'command' not in step:
                logger.warning("Step missing 'command' field")
                continue
            
            # Get the command and parameters
            command = step['command']
            step_params = {**step, **params}  # Merge step params with overrides
            
            # Remove the command from params to avoid recursion
            step_params.pop('command', None)
            
            # Execute the step
            if not await self.execute_movement(command, **step_params):
                logger.error(f"Sequence failed at step: {command}")
                return False
        
        return True
    
    async def stand(self, height: float = 120.0, duration: float = 2.0) -> bool:
        """Make the hexapod stand up.
        
        Args:
            height: Target height in mm
            duration: Time to complete the movement in seconds
            
        Returns:
            bool: True if movement completed successfully, False otherwise
        """
        logger.info(f"Standing to height: {height}mm over {duration}s")
        
        try:
            # Convert height to servo angles using inverse kinematics
            # This is a simplified example - replace with your actual IK implementation
            angles = self._calculate_stand_pose(height)
            
            # Move servos to target angles
            await self._move_servos(angles, duration)
            
            return True
            
        except Exception as e:
            logger.error(f"Error in stand movement: {e}")
            return False
    
    def _calculate_stand_pose(self, height: float) -> Dict[int, float]:
        """Calculate servo angles for standing at the specified height.
        
        Args:
            height: Target height in mm
            
        Returns:
            Dict[int, float]: Mapping of servo IDs to target angles
        """
        # This is a simplified example - replace with your actual IK calculations
        # The actual implementation will depend on your hexapod's geometry
        
        # Default angles (in degrees) for a standing pose
        # These values are placeholders - adjust based on your robot's configuration
        angles = {
            # Front right leg
            0: 90.0,  # Coxa
            1: 45.0,  # Femur
            2: 45.0,  # Tibia
            
            # Middle right leg
            3: 90.0,
            4: 45.0,
            5: 45.0,
            
            # Rear right leg
            6: 90.0,
            7: 45.0,
            8: 45.0,
            
            # Rear left leg
            9: 90.0,
            10: 135.0,
            11: 135.0,
            
            # Middle left leg
            12: 90.0,
            13: 135.0,
            14: 135.0,
            
            # Front left leg
            15: 90.0,
            16: 135.0,
            17: 135.0,
        }
        
        # Adjust angles based on height (simplified)
        # In a real implementation, you would use proper inverse kinematics here
        height_factor = (height - 60.0) / 60.0  # Normalize to 0-1 range for 60-120mm
        height_factor = max(0.0, min(1.0, height_factor))  # Clamp to 0-1
        
        for leg in range(6):
            # Adjust femur and tibia angles based on height
            base_angle = 45.0 if leg < 3 else 135.0
            angle = base_angle + (45.0 * height_factor)
            
            # Update angles for this leg
            angles[leg * 3 + 1] = angle        # Femur
            angles[leg * 3 + 2] = 180.0 - angle  # Tibia
        
        return angles
    
    async def _move_servos(self, angles: Dict[int, float], duration: float) -> None:
        """Move servos to the specified angles.
        
        Args:
            angles: Mapping of servo IDs to target angles
            duration: Time to complete the movement in seconds
        """
        # Convert duration to milliseconds
        duration_ms = int(duration * 1000)
        
        # Send commands to servos
        for servo_id, angle in angles.items():
            # Convert angle to servo pulse width (assuming 0.24us per degree, 1500us center)
            pulse_width = int(1500 + (angle - 90) * 10)  # Approximate conversion
            
            # Send command to servo
            # Replace with your actual servo control code
            # Example: self.client.send_servo_command(servo_id, pulse_width, duration_ms)
            pass
        
        # Wait for movement to complete
        await asyncio.sleep(duration)
    
    async def stop_movement(self) -> None:
        """Stop all current movements."""
        logger.info("Stopping all movements")
        
        # Stop the gait controller if running
        if hasattr(self, 'gait_controller') and self.gait_controller.is_running():
            await self.gait_controller.stop()
        
        # Clear the movement queue
        while not self._movement_queue.empty():
            try:
                self._movement_queue.get_nowait()
                self._movement_queue.task_done()
            except asyncio.QueueEmpty:
                break
        
        # Reset movement state
        self._is_moving = False
        self._current_movement = None
        self._stop_movement_flag = True
    
    def _update_movement_status(self) -> None:
        """Update the UI with current movement status."""
        if self._is_moving and self._current_movement:
            self.statusBar().showMessage(f"Moving: {self._current_movement}")
    
    def _on_movement_complete(self, movement_name: str, success: bool) -> None:
        """Handle movement completion.
        
        Args:
            movement_name: Name of the completed movement
            success: Whether the movement completed successfully
        """
        self._is_moving = False
        self._current_movement = None
        
        if success:
            logger.info(f"Movement completed: {movement_name}")
            self.statusBar().showMessage(f"Completed: {movement_name}", 3000)  # Show for 3 seconds
        else:
            logger.warning(f"Movement failed: {movement_name}")
            self.statusBar().showMessage(f"Failed: {movement_name}", 5000)  # Show for 5 seconds
    
    # Startup sequence methods
    
    async def start_startup_sequence(self) -> None:
        """Start the startup sequence."""
        if not hasattr(self, 'startup_manager'):
            logger.error("Startup manager not initialized")
            return
        
        try:
            # Disable UI controls during startup
            self._set_ui_enabled(False)
            
            # Show progress dialog
            self.startup_progress_dialog = QProgressDialog("Starting up...", "Cancel", 0, 100, self)
            self.startup_progress_dialog.setWindowTitle("Startup Sequence")
            self.startup_progress_dialog.setWindowModality(QtCore.Qt.WindowModal)
            self.startup_progress_dialog.canceled.connect(self.stop_startup_sequence)
            self.startup_progress_dialog.show()
            
            # Start the startup sequence
            await self.startup_manager.start()
            
        except Exception as e:
            logger.error(f"Error starting startup sequence: {e}")
            self.startup_progress_dialog.cancel()
            self._set_ui_enabled(True)
    
    async def stop_startup_sequence(self) -> None:
        """Stop the startup sequence."""
        if hasattr(self, 'startup_manager'):
            await self.startup_manager.stop()
        
        if hasattr(self, 'startup_progress_dialog'):
            self.startup_progress_dialog.close()
        
        self._set_ui_enabled(True)
    
    def _on_startup_progress(self, progress: int, message: str) -> None:
        """Update startup progress.
        
        Args:
            progress: Progress percentage (0-100)
            message: Status message
        """
        if hasattr(self, 'startup_progress_dialog'):
            self.startup_progress_dialog.setValue(progress)
            self.startup_progress_dialog.setLabelText(message)
    
    def _on_startup_complete(self, success: bool) -> None:
        """Handle startup sequence completion.
        
        Args:
            success: Whether the startup sequence completed successfully
        """
        if hasattr(self, 'startup_progress_dialog'):
            self.startup_progress_dialog.close()
        
        self._set_ui_enabled(True)
        
        if success:
            logger.info("Startup sequence completed successfully")
            self.statusBar().showMessage("Startup complete", 3000)
        else:
            logger.warning("Startup sequence failed")
            self.statusBar().showMessage("Startup failed", 5000)
    
    def _on_startup_state_change(self, state: str, message: str) -> None:
        """Handle startup state changes.
        
        Args:
            state: New state name
            message: State description
        """
        logger.info(f"Startup state changed: {state} - {message}")
    
    def _on_startup_error(self, error_code: str, error_message: str) -> None:
        """Handle startup errors.
        
        Args:
            error_code: Error code
            error_message: Error description
        """
        logger.error(f"Startup error {error_code}: {error_message}")
        QMessageBox.critical(self, "Startup Error", f"{error_code}: {error_message}")
    
    def _set_ui_enabled(self, enabled: bool) -> None:
        """Enable or disable UI controls.
        
        Args:
            enabled: Whether to enable the UI controls
        """
        # Enable/disable all buttons and controls
        for widget in self.findChildren((QPushButton, QSlider, QSpinBox, QDoubleSpinBox, QComboBox)):
            # Don't disable the stop button
            if widget is not self.Button_Stop:
                widget.setEnabled(enabled)
    
    def closeEvent(self, event: QCloseEvent) -> None:
        """Handle window close event with comprehensive cleanup."""
        logger.info("Application closing, performing cleanup...")
        
        try:
            # Stop any running startup sequence
            if hasattr(self, 'startup_manager'):
                asyncio.create_task(self.stop_startup_sequence())
            
            # Stop all movements
            if hasattr(self, '_movement_task') and not self._movement_task.done():
                self._movement_task.cancel()
            
            # Stop all timers
            for timer_name in ['video_timer', 'power_timer', 'sonic_timer', 'movement_timer', 'startup_timer']:
                timer = getattr(self, timer_name, None)
                if timer and timer.isActive():
                    try:
                        timer.stop()
                        timer.deleteLater()
                        logger.debug(f"Stopped timer: {timer_name}")
                    except Exception as e:
                        logger.error(f"Error stopping timer {timer_name}: {e}")
            
            # Stop all threads managed by thread manager
            if hasattr(self, '_thread_manager'):
                try:
                    self._thread_manager.stop_all()
                    logger.info("Stopped all managed threads")
                except Exception as e:
                    logger.error(f"Error stopping managed threads: {e}")
            
            # Disconnect from robot
            if hasattr(self, 'client') and hasattr(self.client, 'tcp_flag') and self.client.tcp_flag:
                try:
                    logger.info("Disconnecting from robot...")
                    self.client.turn_off_client()
                    logger.info("Disconnected from robot")
                except Exception as e:
                    logger.error(f"Error disconnecting from robot: {e}")
            
            # Clean up camera resources
            if hasattr(self, 'camera_recorder'):
                try:
                    self.camera_recorder.cleanup()
                    logger.debug("Cleaned up camera resources")
                except Exception as e:
                    logger.error(f"Error cleaning up camera resources: {e}")
            
            logger.info("Cleanup complete, closing application")
            
        except Exception as e:
            logger.error(f"Error during shutdown: {e}", exc_info=True)
            
        finally:
            # Always ensure the event is accepted to allow the application to close
            event.accept()
    
    def start_startup_sequence(self) -> None:
        """Start the startup movement sequence."""
        if not self.startup_sequence_running:
            self.startup_sequence_running = True
            self.startup_movement_index = 0
            self.startup_timer.start(100)
    
    def execute_startup_movement(self) -> None:
        """Execute the next movement in the startup sequence."""
        if self.startup_movement_index < len(self.startup_movements):
            movement = self.startup_movements[self.startup_movement_index]
            command = f"{movement['command']}#{';'.join(movement['params'])}\n"
            self.client.send_data(command)
            self.startup_movement_index += 1
            self.startup_timer.start(movement['delay'])
        else:
            self.startup_sequence_running = False
            self.startup_timer.stop()
    
    def toggle_startup_movements(self, enabled: bool) -> None:
        """Toggle startup movement sequence."""
        self.startup_movements_enabled = enabled
        logger.info(f"Startup movements {'enabled' if enabled else 'disabled'}")
    
            
# Helper function to safely stop threads
def stop_thread(thread):
    """Safely stop a running thread."""
    if thread is not None and thread.is_alive():
        thread.join(timeout=1.0)
        if thread.is_alive():
            # Force stop if thread doesn't respond to join
            thread._stop()

if __name__ == '__main__':
    try:
        app = QApplication(sys.argv)
        
        # Set application style
        app.setStyle('Fusion')
        
        # Create and show the main window
        controller = HexapodController()
        controller.show()
        
        # Start the application event loop
        sys.exit(app.exec_())
        
    except Exception as e:
        logger.critical(f"Fatal error: {e}", exc_info=True)
        sys.exit(1)
