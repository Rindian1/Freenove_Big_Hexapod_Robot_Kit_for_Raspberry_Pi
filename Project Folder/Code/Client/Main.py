# -*- coding: utf-8 -*-
import sys
import os
import math
import threading
import time
from typing import Optional, Callable
import cv2
import numpy as np
from src.ui.dialogs.led_dialog import Ui_led
from src.ui.dialogs.face_dialog import Ui_Face
from src.ui.main_window import Ui_client
from PyQt5 import QtCore, QtGui, QtWidgets
from PyQt5.QtCore import *
from PyQt5.QtWidgets import *
from PyQt5.QtGui import *
from src.core.client import Client
from src.ui.dialogs.calibration_dialog import Ui_calibration
from src.utils.camera_recorder import CameraRecorder
from src.core.thread import *
from src.core.command import COMMAND as cmd

# ==========================
# Module-level constants
# ==========================
HEAD_MIN: int = 50
HEAD_MAX: int = 180
HEAD_INIT: int = 90

SPEED_MIN: int = 2
SPEED_MAX: int = 10
SPEED_INIT: int = 8

ROLL_MIN: int = -15
ROLL_MAX: int = 15
ROLL_INIT: int = 0

Z_MIN: int = -20
Z_MAX: int = 20
Z_INIT: int = 0

TIMER_REFRESH_MS: int = 10
TIMER_POWER_MS: int = 3000
TIMER_SONIC_MS: int = 100
TIMER_PHOTO_MS: int = 100

from enum import Enum, auto
import socket
import time
import threading
from typing import Optional, Callable, Any, Tuple, Dict

from src.utils.exceptions import (
    RobotError,
    ConnectionError,
    NetworkError,
    TimeoutError,
    InvalidStateError
)
from src.core.thread_safe import ThreadSafeValue, ThreadSafeCounter
from src.utils.logging_config import get_logger
from src.utils.utils import retry, handle_errors, log_duration

logger = get_logger(__name__)


class ConnectionState(Enum):
    """Represents the connection state of the NetworkManager."""
    DISCONNECTED = auto()
    CONNECTING = auto()
    CONNECTED = auto()
    ERROR = auto()


class NetworkManager:
    """
    Manages network connections and background threads for the Hexapod Robot client.
    
    This class handles the lifecycle of network connections, including video streaming,
    command sending/receiving, and automatic reconnection with exponential backoff.
    """
    
    # Network timeouts in seconds
    CONNECT_TIMEOUT = 5.0
    SOCKET_TIMEOUT = 1.0
    RECONNECT_DELAY = 2.0
    MAX_RECONNECT_ATTEMPTS = 3
    
    def __init__(self, client: 'Client') -> None:
        """Initialize the NetworkManager with a client instance.
        
        Args:
            client: The Client instance to manage network connections for.
        """
        self.client = client
        self._state = ThreadSafeValue(ConnectionState.DISCONNECTED, name="connection_state")
        self._lock = threading.RLock()
        self._stop_event = threading.Event()
        self._reconnect_attempts = ThreadSafeCounter(name="reconnect_attempts")
        
        # Network components
        self.video_thread: Optional[threading.Thread] = None
        self.instruction_thread: Optional[threading.Thread] = None
        self.video_socket: Optional[socket.socket] = None
        self.instruction_socket: Optional[socket.socket] = None
        self.video_label: Optional[Any] = None
        
        # Connection details
        self.ip: Optional[str] = None
        self.port: Optional[int] = None
        self.video_port: Optional[int] = None
        
        # Thread control flags
        self._video_thread_running = False
        self._instruction_thread_running = False

    @property
    def is_connected(self) -> bool:
        """Thread-safe access to the connection status."""
        with self._lock:
            return self._is_connected

    @property
    def state(self) -> ConnectionState:
        """Get the current connection state."""
        return self._state.value
    
    def _set_state(self, new_state: ConnectionState) -> None:
        """Safely update the connection state.
        
        Args:
            new_state: The new connection state
        """
        old_state = self._state.value
        if old_state != new_state:
            self._state.value = new_state
            self._on_state_changed(old_state, new_state)
    
    def _on_state_changed(self, old_state: ConnectionState, new_state: ConnectionState) -> None:
        """Handle connection state changes.
        
        Args:
            old_state: Previous connection state
            new_state: New connection state
        """
        logger.info(f"Connection state changed: {old_state.name} -> {new_state.name}")
        
        # Notify client of state changes if needed
        if hasattr(self.client, 'on_connection_state_changed'):
            try:
                self.client.on_connection_state_changed(old_state, new_state)
            except Exception as e:
                logger.error(f"Error in connection state change handler: {e}", exc_info=True)
    
    @retry(max_attempts=3, delay=1.0, backoff=2.0, exceptions=(ConnectionError, socket.error, TimeoutError))
    def connect(self, ip: str, port: int, video_port: int) -> bool:
        """Establish a connection to the robot.
        
        Args:
            ip: Robot IP address
            port: Robot command port
            video_port: Robot video port
            
        Returns:
            bool: True if connection was successful, False otherwise
            
        Raises:
            ConnectionError: If connection fails after max retry attempts
            InvalidStateError: If already connected or in an invalid state
        """
        if self.state == ConnectionState.CONNECTED:
            raise InvalidStateError("Already connected to the robot")
            
        self._set_state(ConnectionState.CONNECTING)
        
        try:
            self.ip = ip
            self.port = port
            self.video_port = video_port
            
            # Connect to command port
            try:
                self.instruction_socket = self._create_socket()
                self.instruction_socket.connect((self.ip, self.port))
                logger.info(f"Connected to command port {self.port}")
            except (socket.error, OSError) as e:
                raise ConnectionError(f"Failed to connect to command port: {e}") from e
            
            # Connect to video port
            try:
                self.video_socket = self._create_socket()
                self.video_socket.connect((self.ip, self.video_port))
                logger.info(f"Connected to video port {self.video_port}")
            except (socket.error, OSError) as e:
                if self.instruction_socket:
                    self.instruction_socket.close()
                    self.instruction_socket = None
                raise ConnectionError(f"Failed to connect to video port: {e}") from e
            
            # Initialize client connection
            try:
                self.client.turn_on_client(ip)
                self.client.client_socket1 = self.instruction_socket
                self.client.client_socket = self.video_socket
                
                # Start communication threads
                self.start_threads()
                self._set_state(ConnectionState.CONNECTED)
                self._reconnect_attempts.value = 0
                return True
                
            except Exception as e:
                self.disconnect()
                raise ConnectionError(f"Failed to initialize client: {e}") from e
                
        except Exception as e:
            self._handle_connection_error(f"Connection failed: {e}")
            raise

    def _create_socket(self) -> socket.socket:
        """Create and configure a new socket.
        
        Returns:
            socket.socket: Configured socket
            
        Raises:
            NetworkError: If socket creation fails
        """
        try:
            sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
            sock.settimeout(self.SOCKET_TIMEOUT)
            sock.setsockopt(socket.SOL_SOCKET, socket.SO_KEEPALIVE, 1)
            return sock
        except socket.error as e:
            raise NetworkError(f"Failed to create socket: {e}") from e

    def disconnect(self) -> None:
        """Safely disconnect from the robot and clean up resources."""
        if self.state == ConnectionState.DISCONNECTED:
            return
            
        self._set_state(ConnectionState.DISCONNECTED)
        self._stop_event.set()
        
        # Stop and clean up threads
        self._stop_threads()
        
        # Close sockets
        sockets_to_close = [
            (self.instruction_socket, 'instruction'),
            (self.video_socket, 'video'),
            (getattr(self.client, 'client_socket', None), 'client_socket'),
            (getattr(self.client, 'client_socket1', None), 'client_socket1')
        ]
        
        for sock, name in sockets_to_close:
            if sock:
                try:
                    sock.shutdown(socket.SHUT_RDWR)
                    sock.close()
                    logger.debug(f"Closed {name} socket")
                except (OSError, AttributeError) as e:
                    logger.warning(f"Error closing {name} socket: {e}")
        
        # Reset socket references
        self.instruction_socket = None
        self.video_socket = None
        
        if hasattr(self.client, 'client_socket'):
            self.client.client_socket = None
        if hasattr(self.client, 'client_socket1'):
            self.client.client_socket1 = None
            
        self._stop_event.clear()
        logger.info("Disconnected from robot")

    def stop_threads(self) -> None:
        """Safely stop all background threads with proper cleanup."""
        with self._lock:
            if not self._video_thread_running and not self._instruction_thread_running:
                return
            
            self._stop_event.set()
            
            threads = []
            if self.video_thread and self.video_thread.is_alive():
                threads.append((self.video_thread, "video"))
            if self.instruction_thread and self.instruction_thread.is_alive():
                threads.append((self.instruction_thread, "instruction"))
            
            for thread, name in threads:
                try:
                    thread.join(timeout=2.0)
                    if thread.is_alive():
                        logger.warning(f"{name} thread did not terminate gracefully")
                except Exception as e:
                    logger.error(f"Error stopping {name} thread: {e}")
            
            self.video_thread = None
            self.instruction_thread = None
            self._video_thread_running = False
            self._instruction_thread_running = False

    def start_threads(self) -> None:
        """Start the video and instruction processing threads.
        
        Raises:
            InvalidStateError: If not connected or threads are already running
        """
        if self.state != ConnectionState.CONNECTED:
            raise InvalidStateError("Cannot start threads: Not connected to robot")
            
        with self._lock:
            if self._video_thread_running or self._instruction_thread_running:
                raise InvalidStateError("Threads are already running")
            
            # Start video thread
            self._stop_event.clear()
            self._video_thread_running = True
            self.video_thread = threading.Thread(
                target=self._video_thread_func,
                name="VideoThread"
            )
            self.video_thread.daemon = True
            self.video_thread.start()
            
            # Start instruction thread
            self._instruction_thread_running = True
            self.instruction_thread = threading.Thread(
                target=self._instruction_thread_func,
                name="InstructionThread"
            )
            self.instruction_thread.daemon = True
            self.instruction_thread.start()
            
            logger.info("Started network threads")

    def _video_thread_func(self) -> None:
        """Thread function for receiving and processing video frames."""
        if not self.video_socket:
            logger.error("Video socket not initialized")
            return
            
        buffer = b''
        while self._video_thread_running and not self._stop_event.is_set():
            try:
                # Check connection state
                if self.state != ConnectionState.CONNECTED:
                    time.sleep(0.1)
                    continue
                    
                # Receive data from socket
                try:
                    data = self.video_socket.recv(4096)
                    if not data:
                        raise ConnectionError("Connection closed by remote host")
                    buffer += data
                    
                    # Process complete frames (implementation depends on protocol)
                    # This is a simplified example - adjust based on actual protocol
                    while b'\n' in buffer:
                        frame_data, buffer = buffer.split(b'\n', 1)
                        self._process_video_frame(frame_data)
                        
                except socket.timeout:
                    continue
                except (socket.error, ConnectionError) as e:
                    if not self._stop_event.is_set():
                        self._handle_connection_error(f"Video thread error: {e}")
                    break
                    
            except Exception as e:
                logger.error(f"Unexpected error in video thread: {e}", exc_info=True)
                if not self._stop_event.is_set():
                    time.sleep(0.1)  # Prevent tight loop on errors

    def _instruction_thread_func(self) -> None:
        """Thread function for sending and receiving instructions."""
        if not self.instruction_socket:
            logger.error("Instruction socket not initialized")
            return
            
        while self._instruction_thread_running and not self._stop_event.is_set():
            try:
                # Check connection state
                if self.state != ConnectionState.CONNECTED:
                    time.sleep(0.1)
                    continue
                    
                # Receive data from socket
                try:
                    data = self.instruction_socket.recv(4096)
                    if not data:
                        raise ConnectionError("Connection closed by remote host")
                    
                    # Process received data (implementation depends on protocol)
                    # This is a simplified example - adjust based on actual protocol
                    self._process_instruction(data)
                    
                except socket.timeout:
                    continue
                except (socket.error, ConnectionError) as e:
                    if not self._stop_event.is_set():
                        self._handle_connection_error(f"Instruction thread error: {e}")
                    break
                    
            except Exception as e:
                logging.error(f"Unexpected error in instruction thread: {e}", exc_info=True)
                if not self._stop_event.is_set():
                    time.sleep(0.1)  # Prevent tight loop on errors

    def _process_video_frame(self, frame_data: bytes) -> None:
        """Process a received video frame.
        
        Args:
            frame_data: Raw video frame data
        """
        # TO DO: Implement video frame processing
        pass

    def _process_instruction(self, data: bytes) -> None:
        """Process received instruction data.
        
        Args:
            data: Raw instruction data
        """
        # TO DO: Implement instruction processing
        pass

    def _handle_connection_error(self, error: str) -> None:
        """Handle a connection-related error.
        
        Args:
            error: Error message
        """
        logging.error(error)
        self._set_state(ConnectionState.ERROR)
        self.stop_threads()

    def _create_socket(self) -> socket.socket:
        """Create a new socket with default settings."""
        sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        sock.settimeout(self.SOCKET_TIMEOUT)
        return sock

class VideoHandler:
    """Manages video refresh and photo capture for the main video label."""
    def __init__(self, client: 'Client', video_label: QLabel) -> None:
        self.client = client
        self.video_label = video_label
        self.camera_recorder = CameraRecorder(output_dir='Captures', video_label=video_label)

    def refresh_image(self) -> None:
        if self.client.video_flag == False:
            height, width, bytesPerComponent = self.client.image.shape
            cv2.cvtColor(self.client.image, cv2.COLOR_BGR2RGB, self.client.image)
            QImg = QImage(self.client.image.data.tobytes(), width, height, 3 * width, QImage.Format_RGB888)
            self.video_label.setPixmap(QPixmap.fromImage(QImg))
            self.client.video_flag = True

    def take_photo(self) -> None:
        try:
            try:
                self.client.send_data(cmd.CMD_BUZZER + '#1' + '\n') 
                self.client.send_data(cmd.CMD_LED + '#1' + '\n')
            except Exception as e:
                print(e)
            QtCore.QTimer.singleShot(120, self._capture_photo_and_buzz_off)
        except Exception as e:
            print(e)

    def _capture_photo_and_buzz_off(self) -> None:
        try:
            if hasattr(self.client, 'image') and len(self.client.image) > 0:
                pix = self.video_label.pixmap()
            else:
                pix = None
            saved_path = self.camera_recorder.capture(pixmap=pix if pix is not None else None)
            print('Photo saved to:', saved_path)
        except Exception as e:
            print(e)
        finally:
            try:
                self.client.send_data(cmd.CMD_BUZZER + '# 0' + '\n') 
                self.client.send_data(cmd.CMD_LED + '#0' + '\n')
            except Exception as e:
                print(e)

class UIManager:
    """Opens auxiliary windows bound to the same client (LED, Face, Calibration)."""
    def __init__(self, client: 'Client', parent_window: QMainWindow) -> None:
        self.client = client
        self.parent = parent_window
        self.calibration_window = None
        self.led_window = None
        self.face_window = None

    def show_calibration_window(self) -> None:
        command = cmd.CMD_CALIBRATION + '\n'
        self.client.send_data(command)
        self.calibration_window = CalibrationWindow(self.client)
        self.calibration_window.setWindowModality(Qt.ApplicationModal)
        self.calibration_window.show()

    def show_led_window(self) -> None:
        try:
            self.led_window = LedWindow(self.client)
            self.led_window.setWindowModality(Qt.ApplicationModal)
            self.led_window.show()
        except Exception as e:
            print(e)

    def show_face_window(self) -> None:
        try:
            self.face_window = FaceWindow(self.client)
            self.face_window.setWindowModality(Qt.ApplicationModal)
            self.face_window.show()
            self.client.fece_id = True
        except Exception as e:
            print(e)

class RobotController:
    """Maps UI state to robot commands (no UI ownership)."""
    def __init__(self, client: 'Client', ui: QMainWindow) -> None:
        self.client = client
        self.ui = ui
        self.action_flag = 1
        self.gait_flag = 1

    def map(self, value: float, fromLow: float, fromHigh: float, toLow: float, toHigh: float) -> float:
        return (toHigh - toLow) * (value - fromLow) / (fromHigh - fromLow) + toLow

    def move(self) -> None:
        try:
            x = self.map((self.ui.move_point[0]-325),0,100,0,35)
            y = self.map((635 - self.ui.move_point[1]),0,100,0,35)
            if self.action_flag == 1:
                angle = 0
            else:
                if x!=0 or y!=0:
                    angle=math.degrees(math.atan2(x,y))
                    if angle < -90 and angle >= -180:
                        angle=angle+360
                    if angle >= -90 and angle <=90:
                        angle = self.map(angle, -90, 90, -10, 10)
                    else:
                        angle = self.map(angle, 270, 90, 10, -10)
                else:
                    angle=0
            speed=self.client.move_speed
            command = cmd.CMD_MOVE+ "#"+str(self.gait_flag)+"#"+str(round(x))+"#"+str(round(y))\
                      +"#"+str(speed)+"#"+str(round(angle)) +'\n'
            print(command)
            self.client.send_data(command)
        except Exception as e:
            print(e)

    def relax(self) -> None:
        try:
            if self.ui.Button_Relax.text() == "Relax":
                self.ui.Button_Relax.setText("Relaxed")
                command = cmd.CMD_SERVOPOWER + "#" + "0" + '\n'
            else:
                self.ui.Button_Relax.setText("Relax")
                command = cmd.CMD_SERVOPOWER + "#" + "1" + '\n'
            print(command)
            self.client.send_data(command)
        except Exception as e:
            print(e)

    def attitude(self) -> None:
        r = self.map((self.ui.drawpoint[0][0]-800), -100, 100, -15, 15)
        p = self.map((180-self.ui.drawpoint[0][1]), -100, 100, -15, 15)
        y=self.ui.slider_roll.value()
        command = cmd.CMD_ATTITUDE+ "#" + str(round(r)) + "#" + str(round(p)) + "#" + str(round(y)) + '\n'
        print(command)
        self.client.send_data(command)

    def position(self) -> None:
        x = self.map((self.ui.drawpoint[1][0]-800), -100, 100, -40, 40)
        y = self.map((650-self.ui.drawpoint[1][1]), -100, 100, -40, 40)
        z=self.ui.slider_Z.value()
        command = cmd.CMD_POSITION+ "#" + str(round(x)) + "#" + str(round(y)) + "#" + str(round(z)) + '\n'
        print(command)
        self.client.send_data(command)

    def buzzer(self) -> None:
        if self.ui.Button_Buzzer.text() == 'Buzzer':
            command=cmd.CMD_BUZZER+'#1'+'\n'
            self.client.send_data(command)
            self.ui.Button_Buzzer.setText('Noise')
        else:
            command=cmd.CMD_BUZZER+'#0'+'\n'
            self.client.send_data(command)
            self.ui.Button_Buzzer.setText('Buzzer')

    def imu(self) -> None:
        if self.ui.Button_IMU.text()=='Balance':
            command=cmd.CMD_BALANCE+'#1'+'\n'
            self.client.send_data(command)
            self.ui.Button_IMU.setText("Close")
        else:
            command=cmd.CMD_BALANCE+'#0'+'\n'
            self.client.send_data(command)
            self.ui.Button_IMU.setText('Balance')

    def sonic(self) -> None:
        if self.ui.Button_Sonic.text() == 'Sonic':
            self.ui.timer_sonic.start(TIMER_SONIC_MS)
            self.ui.Button_Sonic.setText('Close')
        else:
            self.ui.timer_sonic.stop()
            self.ui.Button_Sonic.setText('Sonic')

    def get_sonic_data(self) -> None:
        command=cmd.CMD_SONIC+'\n'
        self.client.send_data(command)

    def power(self) -> None:
        try:
            command = cmd.CMD_POWER + '\n'
            self.client.send_data(command)
            self.ui.progress_Power1.setFormat(str(self.ui.power_value[0])+"V")
            self.ui.progress_Power2.setFormat(str(self.ui.power_value[1]) + "V")
            self.ui.progress_Power1.setValue(self.ui.restriction(round((float(self.ui.power_value[0]) - 5.00) / 3.40 * 100), 0, 100))
            self.ui.progress_Power2.setValue(self.ui.restriction(round((float(self.ui.power_value[1]) - 7.00) / 1.40 * 100), 0, 100))
        except Exception as e:
            print(e)
class MyWindow(QMainWindow,Ui_client):
    def __init__(self):
        super(MyWindow, self).__init__()
        self.setupUi(self)
        self._setup_ui_appearance()
        self._setup_handlers()
        self._setup_ui_components()
        self._setup_timers()
        self._setup_variables()
        self._setup_focus_handling()
    
    def _setup_ui_appearance(self):
        """Initialize the main window appearance and icons."""
        self.setWindowIcon(QIcon('Picture/logo_Mini.png'))
        self.Video.setScaledContents(True)
        self.Video.setPixmap(QPixmap('Picture/Spider_client.png'))
    
    def _setup_handlers(self):
        """Initialize and wire up all the handler classes."""
        # Initialize handlers
        self.video_handler = VideoHandler(client=None, video_label=self.Video)
        self.client = Client()
        self.ui_manager = UIManager(self.client, self)
        self.controller = RobotController(self.client, ui=self)
        self.network = NetworkManager(self.client)
        
        # Wire client into handlers
        self.video_handler.client = self.client
        
        # Load IP address
        try:
            with open('IP.txt', 'r') as file:
                self.lineEdit_IP_Adress.setText(str(file.readline().strip()))
        except Exception as e:
            print(f"Error loading IP address: {e}")
    
    def _setup_ui_components(self):
        """Set up all UI components including buttons, sliders, and their connections."""
        self._setup_buttons()
        self._setup_sliders()
        self._setup_radio_buttons()
    
    def _setup_buttons(self):
        """Set up button connections."""
        # Control buttons
        self.Button_Connect.clicked.connect(self.connect)
        self.Button_Video.clicked.connect(self.video)
        self.Button_IMU.clicked.connect(self.controller.imu)
        self.Button_Sonic.clicked.connect(self.controller.sonic)
        self.Button_Relax.clicked.connect(self.controller.relax)
        self.Button_Take_Photo.clicked.connect(self.video_handler.take_photo)
        self.Button_Face_Recognition.clicked.connect(self.face_recognition)
        
        # Buzzer button has press/release events
        self.Button_Buzzer.pressed.connect(self.controller.buzzer)
        self.Button_Buzzer.released.connect(self.controller.buzzer)
        
        # Window control buttons
        self.Button_Calibration.clicked.connect(self.ui_manager.show_calibration_window)
        self.Button_LED.clicked.connect(self.ui_manager.show_led_window)
        self.Button_Face_ID.clicked.connect(self.ui_manager.show_face_window)
    
    def _setup_sliders(self):
        """Set up slider controls with their ranges and connections."""
        # Head control sliders
        self._setup_slider(self.slider_head, HEAD_MIN, HEAD_MAX, HEAD_INIT, self.head_up_and_down)
        self._setup_slider(self.slider_head_1, 0, 180, HEAD_INIT, self.head_left_and_right)
        
        # Movement control sliders
        self._setup_slider(self.slider_speed, SPEED_MIN, SPEED_MAX, SPEED_INIT, self.speed)
        self._setup_slider(self.slider_roll, ROLL_MIN, ROLL_MAX, ROLL_INIT, self.set_roll)
        self._setup_slider(self.slider_Z, Z_MIN, Z_MAX, Z_INIT, self.set_z)
        
        # Set initial speed
        self.client.move_speed = str(self.slider_speed.value())
    
    def _setup_slider(self, slider, min_val, max_val, init_val, callback):
        """Helper method to configure a slider with common settings."""
        slider.setMinimum(min_val)
        slider.setMaximum(max_val)
        slider.setSingleStep(1)
        slider.setValue(init_val)
        slider.valueChanged.connect(callback)
    
    def _setup_radio_buttons(self):
        """Set up radio button groups and their connections."""
        # Action mode radio buttons
        self.ButtonActionMode1.setChecked(True)
        self.ButtonActionMode1.toggled.connect(lambda: self.action_mode(self.ButtonActionMode1))
        self.ButtonActionMode2.setChecked(False)
        self.ButtonActionMode2.toggled.connect(lambda: self.action_mode(self.ButtonActionMode2))
        
        # Gait mode radio buttons
        self.ButtonGaitMode1.setChecked(True)
        self.ButtonGaitMode1.toggled.connect(lambda: self.gait_mode(self.ButtonGaitMode1))
        self.ButtonGaitMode2.setChecked(False)
        self.ButtonGaitMode2.toggled.connect(lambda: self.gait_mode(self.ButtonGaitMode2))
    
    def _setup_timers(self):
        """Initialize and set up all timers."""
        # Video refresh timer
        self.timer = QTimer(self)
        self.timer.timeout.connect(self.video_handler.refresh_image)
        
        # Power monitoring timer
        self.timer_power = QTimer(self)
        self.timer_power.timeout.connect(self.controller.power)
        
        # Sonic sensor timer
        self.timer_sonic = QTimer(self)
        self.timer_sonic.timeout.connect(self.controller.get_sonic_data)
    
    def _setup_variables(self):
        """Initialize class variables and state."""
        # Keyboard state
        self.key_w = False
        self.key_a = False
        self.key_s = False
        self.key_d = False
        self.key_space = False
        
        # Thread handles
        self.video_thread = None
        self.instruction_thread = None
        
        # Movement and state variables
        self.power_value = [100, 100]
        self.move_point = [325, 635]
        self.move_flag = False
        self.drawpoint = [[800, 180], [800, 650]]
        self.action_flag = 1
        self.gait_flag = 1
    
    def _setup_focus_handling(self):
        """Set up focus handling for keyboard input."""
        try:
            # Main window should accept key focus
            self.setFocusPolicy(Qt.StrongFocus)
            
            # Allow the video label to take focus for keyboard handling
            self.Video.setFocusPolicy(Qt.StrongFocus)
            
            # Only focus the IP box when the user clicks it
            self.lineEdit_IP_Adress.setFocusPolicy(Qt.ClickFocus)
            
            # Start with focus outside the IP box
            self.lineEdit_IP_Adress.clearFocus()
            self.Video.setFocus(Qt.OtherFocusReason)
            
            # When user presses Enter in IP box, move focus to Video (so keys control robot)
            self.lineEdit_IP_Adress.returnPressed.connect(
                lambda: self.Video.setFocus(Qt.TabFocusReason))
        except Exception as e:
            print(f"Error setting up focus handling: {e}")
        self.slider_head_1.setSingleStep(1)
        self.slider_head_1.setValue(HEAD_INIT)
        self.slider_head_1.valueChanged.connect(self.head_left_and_right)

        self.slider_speed.setMinimum(SPEED_MIN)
        self.slider_speed.setMaximum(SPEED_MAX)
        self.slider_speed.setSingleStep(1)
        self.slider_speed.setValue(SPEED_INIT)
        self.slider_speed.valueChanged.connect(self.speed)
        self.client.move_speed = str(self.slider_speed.value())

        self.slider_roll.setMinimum(ROLL_MIN)
        self.slider_roll.setMaximum(ROLL_MAX)
        self.slider_roll.setSingleStep(1)
        self.slider_roll.setValue(ROLL_INIT)
        self.slider_roll.valueChanged.connect(self.set_roll)

        self.slider_Z.setMinimum(Z_MIN)
        self.slider_Z.setMaximum(Z_MAX)
        self.slider_Z.setSingleStep(1)
        self.slider_Z.setValue(Z_INIT)
        self.slider_Z.valueChanged.connect(self.set_z)

        #checkbox
        self.ButtonActionMode1.setChecked(True)
        self.ButtonActionMode1.toggled.connect(lambda: self.action_mode(self.ButtonActionMode1))
        self.ButtonActionMode2.setChecked(False)
        self.ButtonActionMode2.toggled.connect(lambda: self.action_mode(self.ButtonActionMode2))
        self.ButtonGaitMode1.setChecked(True)
        self.ButtonGaitMode1.toggled.connect(lambda: self.gait_mode(self.ButtonGaitMode1))
        self.ButtonGaitMode2.setChecked(False)
        self.ButtonGaitMode2.toggled.connect(lambda: self.gait_mode(self.ButtonGaitMode2))

        #Timer
        self.timer=QTimer(self)
        self.timer.timeout.connect(self.video_handler.refresh_image)

        self.timer_power = QTimer(self)
        self.timer_power.timeout.connect(self.controller.power)

        self.timer_sonic = QTimer(self)
        self.timer_sonic.timeout.connect(self.controller.get_sonic_data)

        #Variable
        self.power_value= [100,100]
        self.move_point = [325, 635]
        self.move_flag = False
        self.drawpoint = [[800, 180], [800, 650]]
        self.action_flag = 1
        self.gait_flag = 1

        # Focus handling: don't trap keys in the IP input by default
        try:
            # Main window should accept key focus
            self.setFocusPolicy(Qt.StrongFocus)
            # Allow the video label to take focus for keyboard handling
            self.Video.setFocusPolicy(Qt.StrongFocus)
            # Only focus the IP box when the user clicks it
            self.lineEdit_IP_Adress.setFocusPolicy(Qt.ClickFocus)
            # Start with focus outside the IP box
            self.lineEdit_IP_Adress.clearFocus()
            self.Video.setFocus(Qt.OtherFocusReason)
            # When user presses Enter in IP box, move focus to Video (so keys control robot)
            self.lineEdit_IP_Adress.returnPressed.connect(lambda: self.Video.setFocus(Qt.TabFocusReason))
        except Exception as e:
            print(e)

    # keyboard
    def keyPressEvent(self, event):
        if (event.key() == Qt.Key_C):
            print("C")
            self.connect()
        if (event.key() == Qt.Key_V):
            try:
                print("V")
                self.video()
            except Exception as e:
                print(e)

        if (event.key() == Qt.Key_R):
            print("R")
            self.relax()
        if (event.key() == Qt.Key_L):
            print("L")
            self.show_led_window()
        if (event.key() == Qt.Key_B):
            print("B")
            self.imu()
        if (event.key() == Qt.Key_F):
            print("F")
            self.face_recognition()
        if (event.key() == Qt.Key_U):
            print("U")
            self.sonic()
        if (event.key() == Qt.Key_I):
            print("I")
            self.show_face_window()
        if (event.key() == Qt.Key_T):
            print("T")
            self.show_calibration_window()
        if (event.key() == Qt.Key_Y):
            print("Y")
            self.buzzer()

        if event.isAutoRepeat():
            pass
        else:
            if event.key() == Qt.Key_W:
                self.key_w = True
                print("W")
                self.move_point = [325, 535]
                self.move()
            elif event.key() == Qt.Key_S:
                self.key_s = True
                print("S")
                self.move_point = [325, 735]
                self.move()
            elif event.key() == Qt.Key_A:
                self.key_a = True
                print("A")
                self.move_point = [225, 635]
                self.move()
            elif event.key() == Qt.Key_D:
                self.key_d = True
                print("D")
                self.move_point = [425, 635]
                self.move()

    def keyReleaseEvent(self, event):
        if (event.key() == Qt.Key_W):
            if not (event.isAutoRepeat()) and self.key_w == True:
                print("release W")
                self.key_w = False
                self.move_point = [325, 635]
                self.move()
        elif (event.key() == Qt.Key_A):
            if not (event.isAutoRepeat()) and self.key_a == True:
                print("release A")
                self.key_a = False
                self.move_point = [325, 635]
                self.move()
        elif (event.key() == Qt.Key_S):
            if not (event.isAutoRepeat()) and self.key_s == True:
                print("release S")
                self.key_s = False
                self.move_point = [325, 635]
                self.move()
        elif (event.key() == Qt.Key_D):
            if not (event.isAutoRepeat()) and self.key_d == True:
                print("release D")
                self.key_d = False
                self.move_point = [325, 635]
                self.move()
    def paintEvent(self,e):
        try:
            qp=QPainter()
            qp.begin(self)
            qp.setPen(QPen(Qt.white,2,Qt.SolidLine))
            qp.drawRect(700,80,200,200)
            qp.drawRect(700, 550, 200, 200)
            qp.setRenderHint(QPainter.Antialiasing)

            #steering wheel
            qp.setPen(Qt.NoPen)
            qp.setBrush(QBrush(Qt.gray))# QColor(0,138,255) Qt.white
            qp.drawEllipse(QPoint(325, 635), 100, 100)
            qp.setBrush(QBrush(QColor(0, 138, 255)))
            qp.drawEllipse(QPoint(self.move_point[0], self.move_point[1]), 15, 15)
            qp.setPen(QPen(QColor(0, 138, 255), 2, Qt.SolidLine))
            x1 = round(math.sqrt(100**2-(self.move_point[1]-635)**2)+325)
            y1 = round(math.sqrt(100 ** 2 - (self.move_point[0] - 325) ** 2) + 635)
            qp.drawLine(x1, self.move_point[1], 650-x1, self.move_point[1])
            qp.drawLine(self.move_point[0], 1270-y1, self.move_point[0], y1)

            #attitude
            qp.drawLine(self.drawpoint[0][0], 80, self.drawpoint[0][0], 280)
            qp.drawLine(700, self.drawpoint[0][1], 900, self.drawpoint[0][1])
            self.label_attitude.move(self.drawpoint[0][0] + 10, self.drawpoint[0][1] + 10)
            pitch = round((180-self.drawpoint[0][1]) / 100.0 * 15)
            yaw = round((self.drawpoint[0][0] - 800) / 100.0 * 15)
            self.label_attitude.setText(str((yaw, pitch)))

            #position
            qp.drawLine(self.drawpoint[1][0], 550, self.drawpoint[1][0],750)
            qp.drawLine(700, self.drawpoint[1][1], 900, self.drawpoint[1][1])
            self.label_position.move(self.drawpoint[1][0] + 10, self.drawpoint[1][1] + 10)
            y = round((650-self.drawpoint[1][1] ) / 100.0 * 40)
            x = round((self.drawpoint[1][0] - 800) / 100.0 * 40)
            self.label_position.setText(str((x, y)))
            qp.end()
        except Exception as e:
            print(e)

    def mouseMoveEvent(self, event):
        x = event.pos().x()
        y = event.pos().y()
        if x >= 700 and x <= 900:
            if y >= 80 and y <= 280:
                try:
                    self.drawpoint = [[800, 180], [800, 650]]
                    if  self.move_flag:
                        self.move_point = [325, 635]
                        self.move_flag = False
                        self.move()
                    if self.Button_IMU.text() == "Close":
                        self.Button_IMU.setText("Balance")

                    self.drawpoint[0][0] = x
                    self.drawpoint[0][1] = y
                    self.update()
                    self.attitude()
                except Exception as e:
                    print(e)
            elif y >= 550 and y <= 750:
                try:
                    self.drawpoint = [[800, 180], [800, 650]]
                    if self.move_flag:
                        self.move_point = [325, 635]
                        self.move_flag = False
                        self.move()
                    if self.Button_IMU.text() == "Close":
                        self.Button_IMU.setText("Balance")
                    self.move_point = [325, 635]
                    self.drawpoint[1][0] = x
                    self.drawpoint[1][1] = y
                    self.update()
                    self.position()
                except Exception as e:
                    print(e)
        elif x >= 225 and x <= 425 and y >= 550 and y <= 750:
            r = (x - 325) ** 2 + (635-y) ** 2
            self.drawpoint = [[800, 180], [800, 650]]
            if self.Button_IMU.text() == "Close":
                self.Button_IMU.setText("Balance")
            if r < 10000:
                self.move_flag = True
                self.move_point[0] = x
                self.move_point[1] = y
                self.move()
                self.update()
            else:
                x = x - 325
                y = 635 - y
                angle = math.atan2(y, x)
                self.move_point[0] = 100*math.cos(angle)+325
                self.move_point[1] = 635-100*math.sin(angle)
                self.move()
                self.update()
        elif self.move_flag == True:
            x = x - 325
            y = 635 - y
            angle = math.atan2(y, x)
            self.move_point[0] = 100 * math.cos(angle) + 325
            self.move_point[1] = 635 - 100 * math.sin(angle)
            self.move()
            self.update()

    def mousePressEvent(self, event):
        x = event.pos().x()
        y = event.pos().y()
        if x >= 700 and x <= 900:
            if y >= 80 and y <= 280:
                try:
                    self.drawpoint = [[800, 180], [800, 650]]
                    if self.move_flag:
                        self.move_point = [325, 635]
                        self.move_flag = False
                        self.move()
                    if self.Button_IMU.text() == "Close":
                        self.Button_IMU.setText("Balance")
                    self.drawpoint[0][0] = x
                    self.drawpoint[0][1] = y
                    self.update()
                    self.attitude()
                except Exception as e:
                    print(e)
            elif y >= 550 and y <= 750:
                try:
                    self.drawpoint = [[800, 180], [800, 650]]
                    if self.move_flag:
                        self.move_point = [325, 635]
                        self.move_flag = False
                        self.move()
                    if self.Button_IMU.text() == "Close":
                        self.Button_IMU.setText("Balance")
                    self.drawpoint[1][0] = x
                    self.drawpoint[1][1] = y
                    self.update()
                    self.position()
                except Exception as e:
                    print(e)
        elif x >= 225 and x <= 425 and y >= 550 and y <= 750:
            r = (x - 325) ** 2 + (635 - y) ** 2
            self.drawpoint = [[800, 180], [800, 650]]
            if self.Button_IMU.text() == "Close":
                self.Button_IMU.setText("Balance")
            if r < 10000:
                self.move_flag = True
                self.move_point[0] = x
                self.move_point[1] = y
                self.move()
                self.update()
            else:
                x = x - 325
                y = 635 - y
                angle = math.atan2(y, x)
                self.move_point[0] = 100 * math.cos(angle) + 325
                self.move_point[1] = 635 - 100 * math.sin(angle)
                self.move()
                self.update()
        elif self.move_flag == True:
            x = x - 325
            y = 635 - y
            angle = math.atan2(y, x)
            self.move_point[0] = 100 * math.cos(angle) + 325
            self.move_point[1] = 635 - 100 * math.sin(angle)
            self.move()
            self.update()

    def mouseReleaseEvent(self, event):
        x = event.pos().x()
        y = event.pos().y()
        #print(x,y)
        if self.move_flag:
            self.move_point = [325, 635]
            self.move_flag = False
            self.move()
        self.update()

    def map(self, value, fromLow, fromHigh, toLow, toHigh):
        return (toHigh - toLow) * (value - fromLow) / (fromHigh - fromLow) + toLow

    def face_recognition(self):
        try:
            if self.Button_Face_Recognition.text()=="Face Recog":
                self.client.fece_recognition_flag = True
                self.Button_Face_Recognition.setText("Close")
            elif self.Button_Face_Recognition.text() == "Close":
                self.client.fece_recognition_flag = False
                self.Button_Face_Recognition.setText("Face Recog")
        except Exception as e:
            print(e)

    def move(self):
        try:
            x = self.map((self.move_point[0]-325),0,100,0,35)
            y = self.map((635 - self.move_point[1]),0,100,0,35)
            if self.action_flag == 1:
                angle = 0
            else:
                if x!=0 or y!=0:
                    angle=math.degrees(math.atan2(x,y))

                    if angle < -90 and angle >= -180:
                        angle=angle+360
                    if angle >= -90 and angle <=90:
                        angle = self.map(angle, -90, 90, -10, 10)
                    else:
                        angle = self.map(angle, 270, 90, 10, -10)
                else:
                    angle=0
            speed=self.client.move_speed
            command = cmd.CMD_MOVE+ "#"+str(self.gait_flag)+"#"+str(round(x))+"#"+str(round(y))\
                      +"#"+str(speed)+"#"+str(round(angle)) +'\n'
            print(command)
            self.client.send_data(command)
        except Exception as e:
            print(e)
    def relax(self):
        try:
            if self.Button_Relax.text() == "Relax":
                self.Button_Relax.setText("Relaxed")
                command = cmd.CMD_SERVOPOWER + "#" + "0" + '\n'
            else:
                self.Button_Relax.setText("Relax")
                command = cmd.CMD_SERVOPOWER + "#" + "1" + '\n'
            print(command)
            self.client.send_data(command)
        except Exception as e:
            print(e)
    def attitude(self):
        r = self.map((self.drawpoint[0][0]-800), -100, 100, -15, 15)
        p = self.map((180-self.drawpoint[0][1]), -100, 100, -15, 15)
        y=self.slider_roll.value()
        command = cmd.CMD_ATTITUDE+ "#" + str(round(r)) + "#" + str(round(p)) + "#" + str(round(y)) + '\n'
        print(command)
        self.client.send_data(command)
    def position(self):
        x = self.map((self.drawpoint[1][0]-800), -100, 100, -40, 40)
        y = self.map((650-self.drawpoint[1][1]), -100, 100, -40, 40)
        z=self.slider_Z.value()
        command = cmd.CMD_POSITION+ "#" + str(round(x)) + "#" + str(round(y)) + "#" + str(round(z)) + '\n'
        print(command)
        self.client.send_data(command)
    def closeEvent(self,event):
        try:
            self.timer.stop()
            self.timer_power.stop()
        except Exception as e:
            print(e)
        # Delegate to NetworkManager
        try:
            self.network.disconnect()
        except Exception as e:
            print(e)
        QCoreApplication.instance().quit()
        #os._exit(0)

    def restriction(self,var,v_min,v_max):
        if var < v_min:
            return v_min
        elif var > v_max:
            return v_max
        else:
            return var

    def video(self):
        if self.Button_Video.text() == 'Open Video':
            self.timer.start(TIMER_REFRESH_MS)
            self.Button_Video.setText('Close Video')
        else:
            self.timer.stop()
            self.Button_Video.setText('Open Video')

    def power(self):
        try:
            command = cmd.CMD_POWER + '\n'
            self.client.send_data(command)
            self.progress_Power1.setFormat(str(self.power_value[0])+"V")
            self.progress_Power2.setFormat(str(self.power_value[1]) + "V")
            self.progress_Power1.setValue(self.restriction(round((float(self.power_value[0]) - 5.00) / 3.40 * 100), 0, 100))
            self.progress_Power2.setValue(self.restriction(round((float(self.power_value[1]) - 7.00) / 1.40 * 100), 0, 100))
            #print (command)
        except Exception as e:
            print(e)

    def receive_instruction(self,ip):
        try:
            self.client.client_socket1.connect((ip,5002))
            self.client.tcp_flag=True
            print ("Connecttion Successful !")
        except Exception as e:
            print ("Connect to server Faild!: Server IP is right? Server is opend?")
            self.client.tcp_flag=False 
        while True:
            try:
                alldata=self.client.receive_data()
            except:
                self.client.tcp_flag=False
                break
            #print(alldata)
            if alldata=='':
                break
            else:
                cmdArray=alldata.split('\n')
                #print(cmdArray)
                if cmdArray[-1] !="":
                    cmdArray==cmdArray[:-1]
            for oneCmd in cmdArray:
                data=oneCmd.split("#")
                print(data)
                if data=="":
                    self.client.tcp_flag=False
                    break
                elif data[0]==cmd.CMD_SONIC:
                    self.label_sonic.setText('Obstacle:'+data[1]+'cm')
                    #print('Obstacle:',data[1])
                elif data[0]==cmd.CMD_POWER:
                    try:
                        if len(data)==3:
                            self.power_value[0] = data[1]
                            self.power_value[1] = data[2]
                            #self.power_value[0] = self.restriction(round((float(data[1]) - 5.00) / 3.40 * 100),0,100)
                            #self.power_value[1] = self.restriction(round((float(data[2]) - 7.00) / 1.40 * 100),0,100)
                            #print('Power：',power_value1,power_value2)
                    except Exception as e:
                        print(e)

    #CONNECT
    def connect(self):
        try:
            file=open('IP.txt','w')
            file.write(self.lineEdit_IP_Adress.text())
            file.close()
            if self.Button_Connect.text()=='Connect':
                self.IP = self.lineEdit_IP_Adress.text()
                # Use NetworkManager to establish connection and spin up threads
                # Basic input validation: non-empty IP string
                if not isinstance(self.IP, str) or len(self.IP.strip()) == 0:
                    print('Invalid IP address input')
                    return
                self.network.connect(self.IP, self.receive_instruction)
                #self.face_thread = threading.Thread(target=self.client.face_recognition)
                #self.face_thread.start()
                self.Button_Connect.setText('Disconnect')
                #self.time_out.start(11000)
                self.timer_power.start(TIMER_POWER_MS)
                # Move focus away from IP field so keys control robot
                try:
                    self.lineEdit_IP_Adress.clearFocus()
                    self.Video.setFocus(Qt.OtherFocusReason)
                except Exception as _:
                    pass
            else:
                # Graceful disconnect via NetworkManager
                self.network.disconnect()
                self.Button_Connect.setText('Connect')
                self.timer_power.stop()
                # Ensure focus returns to Video after disconnect
                try:
                    self.lineEdit_IP_Adress.clearFocus()
                    self.Video.setFocus(Qt.OtherFocusReason)
                except Exception as _:
                    pass
        except Exception as e:
            print(e)
    #Mode
    #action_mode
    def action_mode(self,mode):
        if mode.text() == "Action Mode 1":
            if mode.isChecked() == True:
                #print(mode.text())
                self.ButtonActionMode1.setChecked(True)
                self.ButtonActionMode2.setChecked(False)
                self.action_flag = 1
        elif mode.text() == "Action Mode 2":
            if mode.isChecked() == True:
                #print(mode.text())
                self.ButtonActionMode1.setChecked(False)
                self.ButtonActionMode2.setChecked(True)
                self.action_flag = 2
    # gait_mode
    def gait_mode(self,mode):
        if mode.text() == "Gait Mode 1":
            if mode.isChecked() == True:
                #print(mode.text())
                self.ButtonGaitMode1.setChecked(True)
                self.ButtonGaitMode2.setChecked(False)
                self.gait_flag = 1
        elif mode.text() == "Gait Mode 2":
            if mode.isChecked() == True:
                #print(mode.text())
                self.ButtonGaitMode1.setChecked(False)
                self.ButtonGaitMode2.setChecked(True)
                self.gait_flag = 2
    #Slider
    def speed(self):
        self.client.move_speed=str(self.slider_speed.value())
        self.label_speed.setText(str(self.slider_speed.value()))
    def set_z(self):
        self.label_Z.setText(str(self.slider_Z.value()))
        self.position()
    def set_roll(self):
        self.label_roll.setText(str(self.slider_roll.value()))
        self.attitude()
    def head_up_and_down(self):
        try:
            angle = str(self.slider_head.value())
            self.label_head.setText(angle)
            command = cmd.CMD_HEAD + "#" +"0" +"#"+angle + '\n'
            self.client.send_data(command)
            print(command)
        except Exception as e:
            print(e)
    def head_left_and_right(self):
        try:
            angle = str(180-self.slider_head_1.value())
            self.label_head_1.setText(angle)
            command = cmd.CMD_HEAD + "#" +"1" +"#"+angle + '\n'
            self.client.send_data(command)
            print(command)
        except Exception as e:
            print(e)
    #BUZZER
    def buzzer(self):
        if self.Button_Buzzer.text() == 'Buzzer':
            command=cmd.CMD_BUZZER+'#1'+'\n'
            self.client.send_data(command)
            self.Button_Buzzer.setText('Noise')
            #print (command)
        else:
            command=cmd.CMD_BUZZER+'#0'+'\n'
            self.client.send_data(command)
            self.Button_Buzzer.setText('Buzzer')
            #print (command)
    #BALANCE
    def imu(self):
        if self.Button_IMU.text()=='Balance':
            command=cmd.CMD_BALANCE+'#1'+'\n'
            self.client.send_data(command)
            self.Button_IMU.setText("Close")
            #print (command)
        else:
            command=cmd.CMD_BALANCE+'#0'+'\n'
            self.client.send_data(command)
            self.Button_IMU.setText('Balance')
            #print (command)
    #SNOIC
    def sonic(self):
        if self.Button_Sonic.text() == 'Sonic':
            self.timer_sonic.start(100)
            self.Button_Sonic.setText('Close')

        else:
            self.timer_sonic.stop()
            self.Button_Sonic.setText('Sonic')
            #
    def get_sonic_data(self):
        command=cmd.CMD_SONIC+'\n'
        self.client.send_data(command)
        #print (command)

    def show_calibration_window(self):
        command = cmd.CMD_CALIBRATION + '\n'
        self.client.send_data(command)
        self.calibration_window = CalibrationWindow(self.client)
        self.calibration_window.setWindowModality(Qt.ApplicationModal)
        self.calibration_window.show()

    #LED
    def show_led_window(self):
        try:
            self.led_window = LedWindow(self.client)
            self.led_window.setWindowModality(Qt.ApplicationModal)
            self.led_window.show()
        except Exception as e:
            print(e)

    # Face
    def show_face_window(self):
        try:
            self.face_window = FaceWindow(self.client)
            self.face_window.setWindowModality(Qt.ApplicationModal)
            self.face_window.show()
            self.client.fece_id = True
        except Exception as e:
            print(e)

    def refresh_image(self):
        if self.client.video_flag == False:
            height, width, bytesPerComponent=self.client.image.shape
            #print (height, width, bytesPerComponent)
            cv2.cvtColor(self.client.image, cv2.COLOR_BGR2RGB, self.client.image)
            QImg = QImage(self.client.image.data.tobytes(), width, height, 3 * width, QImage.Format_RGB888)
            self.Video.setPixmap(QPixmap.fromImage(QImg))
            self.client.video_flag = True

    def take_photo(self):
        """Beep briefly, then save the current frame (or a blank image if none)."""
        try:
            # Turn buzzer ON for a short beep (non-blocking)
            try:
                self.client.send_data(cmd.CMD_BUZZER + '#1' + '\n')
                # Use a singleShot timer to turn off the buzzer after a delay
                # This ensures the timer is properly parented and cleaned up
                QtCore.QTimer.singleShot(200, lambda: self._turn_off_buzzer())
            except Exception as e:
                print(f"Error in take_photo: {e}")
            # Delay slightly so the beep precedes the shutter
            QtCore.QTimer.singleShot(120, self._capture_photo_and_buzz_off)
        except Exception as e:
            print(f"Error in take_photo: {e}")
            
    def _turn_off_buzzer(self):
        """Helper method to safely turn off the buzzer."""
        try:
            if hasattr(self, 'client') and self.client is not None:
                self.client.send_data(cmd.CMD_BUZZER + '#0' + '\n')
        except Exception as e:
            print(f"Error turning off buzzer: {e}")

    def _capture_photo_and_buzz_off(self):
        """Capture the current frame and save it as a photo."""
        try:
            # Consider feed available only if client.image has content
            if hasattr(self.client, 'image') and len(self.client.image) > 0:
                pix = self.Video.pixmap()
            else:
                pix = None
            # If pix is None or null, CameraRecorder will save a blank image
            saved_path = self.camera_recorder.capture(pixmap=pix if pix is not None else None)
            print('Photo saved to:', saved_path)
        except Exception as e:
            print(f"Error capturing photo: {e}")

class FaceWindow(QMainWindow,Ui_Face):
    def __init__(self,client):
        super(FaceWindow,self).__init__()
        self.setupUi(self)
        self.setWindowIcon(QIcon('Picture/logo_Mini.png'))
        self.Button_Read_Face.clicked.connect(self.read_face)
        self.client = client
        self.face_image=''
        self.photoCount=0
        self.timeout=0
        self.name = ''
        self.readFaceFlag=False
        # Timer
        self.timer1 = QTimer(self)
        self.timer1.timeout.connect(self.face_detection)
        self.timer1.start(10)

        self.timer2 = QTimer(self)
        self.timer2.timeout.connect(self.face_photo)

    def closeEvent(self, event):
        self.timer1.stop()
        self.client.fece_id = False

    def read_face(self):
        try:
            if self.Button_Read_Face.text() == "Read Face":
                self.Button_Read_Face.setText("Reading")
                self.readFaceFlag = True
                self.face_image = ''
                self.photoCount = 0
                self.timeout = time.time()
                self.timer2.start(TIMER_PHOTO_MS)  # Use constant for timer interval
            elif self.Button_Read_Face.text() == "Reading":
                self.Button_Read_Face.setText("Read Face")
                self.readFaceFlag = False
                self.timer2.stop()
        except Exception as e:
            print(e)

    def face_photo(self):
        try:
            if self.photoCount == 30:
                self.photoCount = 0
                self.timer2.stop()
                self.Button_Read_Face.setText("Read Face")
            else:
                if (len(self.client.image) > 0) and self.readFaceFlag:
                    gray = cv2.cvtColor(self.client.image, cv2.COLOR_BGR2GRAY)
                    faces = self.client.face.classifier.detectMultiScale(gray, 1.2, 5)
                    if len(faces) > 0:
                        x, y, w, h = faces[0]
                        self.face_image = self.client.image[y - 20:y + h + 20, x - 20:x + w + 20].copy()
                        self.save_face_photo()
                        self.timeout = time.time()
                        self.Button_Read_Face.setText("Reading " + str(1) + "S   " + str(self.photoCount) + "/30")
                    else:
                        self.Button_Read_Face.setText("Reading " + str(1) + "S   " + str(self.photoCount) + "/30")
                else:
                    second = int(time.time() - self.timeout)
                    if second > 1:
                        self.save_face_photo()
                        self.timeout = time.time()
                    else:
                        self.Button_Read_Face.setText("Reading " + str(1 - second) + "S   " + str(self.photoCount) + "/30")
        except Exception as e:
            print(e)

    def save_face_photo(self):
        cv2.cvtColor(self.face_image, cv2.COLOR_BGR2RGB, self.face_image)
        cv2.imwrite('Face/' + str(len(self.client.face.name)) + '.jpg', self.face_image)
        self.client.face.name.append([str(len(self.client.face.name)), str(self.name)])
        self.name = ''
        self.photoCount += 1
        self.Button_Read_Face.setText("Reading " + str(0) + " S " + str(self.photoCount) + "/30")

    def face_detection(self):
        try:
            if len(self.client.image) > 0:
                gray = cv2.cvtColor(self.client.image, cv2.COLOR_BGR2GRAY)
                faces = self.client.face.classifier.detectMultiScale(gray, 1.2, 5)
                if len(faces) > 0:
                    x, y, w, h = faces[0]
                    cv2.rectangle(self.client.image, (x - 20, y - 20), (x + w + 20, y + h + 20), (0, 255, 0), 2)
                if not self.client.video_flag:
                    height, width, bytesPerComponent = self.client.image.shape
                    cv2.cvtColor(self.client.image, cv2.COLOR_BGR2RGB, self.client.image)
                    QImg = QImage(self.client.image.data.tobytes(), width, height, 3 * width, QImage.Format_RGB888)
                    self.label_video.setPixmap(QPixmap.fromImage(QImg))
                    self.client.video_flag = True
        except Exception as e:
            print(e)

class CalibrationWindow(QMainWindow, Ui_calibration):
    def __init__(self, client):
        super(CalibrationWindow, self).__init__()
        self.setupUi(self)
        self.setWindowIcon(QIcon('Picture/logo_Mini.png'))
        self.label_picture.setScaledContents(True)
        self.label_picture.setPixmap(QPixmap('Picture/Spider_calibration.png'))
        self.point = self.read_from_txt('point')
        self.set_point(self.point)
        self.client = client
        self.leg = 'one'
        self.x = 0
        self.y = 0
        self.z = 0
        self.radioButton_one.setChecked(True)
        self.radioButton_one.toggled.connect(lambda: self.leg_point(self.radioButton_one))
        self.radioButton_two.setChecked(False)
        self.radioButton_two.toggled.connect(lambda: self.leg_point(self.radioButton_two))
        self.radioButton_three.setChecked(False)
        self.radioButton_three.toggled.connect(lambda: self.leg_point(self.radioButton_three))
        self.radioButton_four.setChecked(False)
        self.radioButton_four.toggled.connect(lambda: self.leg_point(self.radioButton_four))
        self.radioButton_five.setChecked(False)
        self.radioButton_five.toggled.connect(lambda: self.leg_point(self.radioButton_five))
        self.radioButton_six.setChecked(False)
        self.radioButton_six.toggled.connect(lambda: self.leg_point(self.radioButton_six))
        self.Button_Save.clicked.connect(self.save)
        self.Button_X1.clicked.connect(self.x1)
        self.Button_X2.clicked.connect(self.x2)
        self.Button_Y1.clicked.connect(self.y1)
        self.Button_Y2.clicked.connect(self.y2)
        self.Button_Z1.clicked.connect(self.z1)
        self.Button_Z2.clicked.connect(self.z2)

    def x1(self):
        self.get_point()
        self.x += 1
        command = cmd.CMD_CALIBRATION + '#' + self.leg + '#' + str(self.x) + '#' + str(self.y) + '#' + str(self.z) + '\n'
        self.client.send_data(command)
        self.set_point()

    def x2(self):
        self.get_point()
        self.x -= 1
        command = cmd.CMD_CALIBRATION + '#' + self.leg + '#' + str(self.x) + '#' + str(self.y) + '#' + str(self.z) + '\n'
        self.client.send_data(command)
        self.set_point()

    def y1(self):
        self.get_point()
        self.y += 1
        command = cmd.CMD_CALIBRATION + '#' + self.leg + '#' + str(self.x) + '#' + str(self.y) + '#' + str(self.z) + '\n'
        self.client.send_data(command)
        self.set_point()

    def y2(self):
        self.get_point()
        self.y -= 1
        command = cmd.CMD_CALIBRATION + '#' + self.leg + '#' + str(self.x) + '#' + str(self.y) + '#' + str(self.z) + '\n'
        self.client.send_data(command)
        self.set_point()

    def z1(self):
        self.get_point()
        self.z += 1
        command = cmd.CMD_CALIBRATION + '#' + self.leg + '#' + str(self.x) + '#' + str(self.y) + '#' + str(self.z) + '\n'
        self.client.send_data(command)
        self.set_point()

    def z2(self):
        self.get_point()
        self.z -= 1
        command = cmd.CMD_CALIBRATION + '#' + self.leg + '#' + str(self.x) + '#' + str(self.y) + '#' + str(self.z) + '\n'
        self.client.send_data(command)
        self.set_point()

    def set_point(self, data=None):
        if data is None:
            if self.leg == "one":
                self.one_x.setText(str(self.x))
                self.one_y.setText(str(self.y))
                self.one_z.setText(str(self.z))
                self.point[0][0] = self.x
                self.point[0][1] = self.y
                self.point[0][2] = self.z
            elif self.leg == "two":
                self.two_x.setText(str(self.x))
                self.two_y.setText(str(self.y))
                self.two_z.setText(str(self.z))
                self.point[1][0] = self.x
                self.point[1][1] = self.y
                self.point[1][2] = self.z
            elif self.leg == "three":
                self.three_x.setText(str(self.x))
                self.three_y.setText(str(self.y))
                self.three_z.setText(str(self.z))
                self.point[2][0] = self.x
                self.point[2][1] = self.y
                self.point[2][2] = self.z
            elif self.leg == "four":
                self.four_x.setText(str(self.x))
                self.four_y.setText(str(self.y))
                self.four_z.setText(str(self.z))
                self.point[3][0] = self.x
                self.point[3][1] = self.y
                self.point[3][2] = self.z
            elif self.leg == "five":
                self.five_x.setText(str(self.x))
                self.five_y.setText(str(self.y))
                self.five_z.setText(str(self.z))
                self.point[4][0] = self.x
                self.point[4][1] = self.y
                self.point[4][2] = self.z
            elif self.leg == "six":
                self.six_x.setText(str(self.x))
                self.six_y.setText(str(self.y))
                self.six_z.setText(str(self.z))
                self.point[5][0] = self.x
                self.point[5][1] = self.y
                self.point[5][2] = self.z
        else:
            self.one_x.setText(str(data[0][0]))
            self.one_y.setText(str(data[0][1]))
            self.one_z.setText(str(data[0][2]))
            self.two_x.setText(str(data[1][0]))
            self.two_y.setText(str(data[1][1]))
            self.two_z.setText(str(data[1][2]))
            self.three_x.setText(str(data[2][0]))
            self.three_y.setText(str(data[2][1]))
            self.three_z.setText(str(data[2][2]))
            self.four_x.setText(str(data[3][0]))
            self.four_y.setText(str(data[3][1]))
            self.four_z.setText(str(data[3][2]))
            self.five_x.setText(str(data[4][0]))
            self.five_y.setText(str(data[4][1]))
            self.five_z.setText(str(data[4][2]))
            self.six_x.setText(str(data[5][0]))
            self.six_y.setText(str(data[5][1]))
            self.six_z.setText(str(data[5][2]))

    def get_point(self):
        if self.leg == "one":
            self.x = int(self.one_x.text())
            self.y = int(self.one_y.text())
            self.z = int(self.one_z.text())
        elif self.leg == "two":
            self.x = int(self.two_x.text())
            self.y = int(self.two_y.text())
            self.z = int(self.two_z.text())
        elif self.leg == "three":
            self.x = int(self.three_x.text())
            self.y = int(self.three_y.text())
            self.z = int(self.three_z.text())
        elif self.leg == "four":
            self.x = int(self.four_x.text())
            self.y = int(self.four_y.text())
            self.z = int(self.four_z.text())
        elif self.leg == "five":
            self.x = int(self.five_x.text())
            self.y = int(self.five_y.text())
            self.z = int(self.five_z.text())
        elif self.leg == "six":
            self.x = int(self.six_x.text())
            self.y = int(self.six_y.text())
            self.z = int(self.six_z.text())

    def save(self):
        command = cmd.CMD_CALIBRATION + '#' + 'save' + '\n'
        self.client.send_data(command)

        self.point[0][0] = self.one_x.text()
        self.point[0][1] = self.one_y.text()
        self.point[0][2] = self.one_z.text()

        self.point[1][0] = self.two_x.text()
        self.point[1][1] = self.two_y.text()
        self.point[1][2] = self.two_z.text()

        self.point[2][0] = self.three_x.text()
        self.point[2][1] = self.three_y.text()
        self.point[2][2] = self.three_z.text()

        self.point[3][0] = self.four_x.text()
        self.point[3][1] = self.four_y.text()
        self.point[3][2] = self.four_z.text()

        self.point[4][0] = self.five_x.text()
        self.point[4][1] = self.five_y.text()
        self.point[4][2] = self.five_z.text()

        self.point[5][0] = self.six_x.text()
        self.point[5][1] = self.six_y.text()
        self.point[5][2] = self.six_z.text()

        self.save_to_txt(self.point, 'point')
        reply = QMessageBox.information(self,
                                        "Message",
                                        "Saved successfully",
                                        QMessageBox.Yes)

    def read_from_txt(self, filename):
        file1 = open(filename + ".txt", "r")
        list_row = file1.readlines()
        list_source = []
        for i in range(len(list_row)):
            list_source.append(list_row[i].split())
        file1.close()
        return list_source

    def save_to_txt(self, list, filename):
        file2 = open(filename + '.txt', 'w')
        for i in range(len(list)):
            for j in range(len(list[i])):
                if j == len(list[i]) - 1:
                    file2.write(str(list[i][j]) + "\n")
                else:
                    file2.write(str(list[i][j]) + '\t')
        file2.close()

    def leg_point(self, leg):
        if leg.text() == "One":
            if leg.isChecked():
                self.leg = "one"
        elif leg.text() == "Two":
            if leg.isChecked() == True:
                self.leg = "two"
        elif leg.text() == "Three":
            if leg.isChecked() == True:
                self.leg = "three"
        elif leg.text() == "Four":
            if leg.isChecked() == True:
                self.leg = "four"
        elif leg.text() == "Five":
            if leg.isChecked() == True:
                self.leg = "five"
        elif leg.text() == "Six":
            if leg.isChecked() == True:
                self.leg = "six"


class ColorDialog(QtWidgets.QColorDialog):
    def __init__(self, parent=None):
        super().__init__(parent)
        self.setOptions(self.options() | QtWidgets.QColorDialog.DontUseNativeDialog)
        for children in self.findChildren(QtWidgets.QWidget):
            classname = children.metaObject().className()
            if classname not in ("QColorPicker", "QColorLuminancePicker"):
                children.hide()
class LedWindow(QMainWindow,Ui_led):
    def __init__(self,client):
        super(LedWindow,self).__init__()
        self.setupUi(self)
        self.client = client
        self.setWindowIcon(QIcon('Picture/logo_Mini.png'))
        self.hsl = [0, 0, 1]
        self.rgb = [0, 0, 0]
        self.dial_color.setRange(0, 360)
        self.dial_color.setNotchesVisible(True)
        self.dial_color.setWrapping(True)
        self.dial_color.setPageStep(10)
        self.dial_color.setNotchTarget(10)
        self.dial_color.valueChanged.connect(self.dial_value_changed)
        composite_2f = lambda f, g: lambda t: g(f(t))
        self.hsl_to_rgb255 = composite_2f(self.hsl_to_rgb01, self.rgb01_to_rgb255)
        self.hsl_to_rgbhex = composite_2f(self.hsl_to_rgb255, self.rgb255_to_rgbhex)
        self.rgb255_to_hsl = composite_2f(self.rgb255_to_rgb01, self.rgb01_to_hsl)
        self.rgbhex_to_hsl = composite_2f(self.rgbhex_to_rgb255, self.rgb255_to_hsl)
        self.colordialog = ColorDialog()
        self.colordialog.currentColorChanged.connect(self.on_current_color_changed)
        lay = QtWidgets.QVBoxLayout(self.widget)
        lay.addWidget(self.colordialog, alignment=QtCore.Qt.AlignCenter)

        self.pushButtonLightsOut.clicked.connect(self.lights_out)
        self.radioButtonOne.setChecked(True)
        self.radioButtonOne.toggled.connect(lambda: self.led_mode(self.radioButtonOne))
        self.radioButtonTwo.setChecked(False)
        self.radioButtonTwo.toggled.connect(lambda: self.led_mode(self.radioButtonTwo))
        self.radioButtonThree.setChecked(False)
        self.radioButtonThree.toggled.connect(lambda: self.led_mode(self.radioButtonThree))
        self.radioButtonFour.setChecked(False)
        self.radioButtonFour.toggled.connect(lambda: self.led_mode(self.radioButtonFour))
        self.radioButtonFive.setChecked(False)
        self.radioButtonFive.toggled.connect(lambda: self.led_mode(self.radioButtonFive))

    def lights_out(self):
        command = cmd.CMD_LED_MOD + '#' + '0' + '\n'
        self.client.send_data(command)
    def led_mode(self,index):
        if index.text() == "Mode 1":
            if index.isChecked() == True:
                command = cmd.CMD_LED_MOD + '#' + '1' + '\n'
                self.client.send_data(command)
        elif index.text() == "Mode 2":
            if index.isChecked() == True:
                command = cmd.CMD_LED_MOD + '#' + '2' + '\n'
                self.client.send_data(command)
        elif index.text() == "Mode 3":
            if index.isChecked() == True:
                command = cmd.CMD_LED_MOD + '#' + '3' + '\n'
                self.client.send_data(command)
        elif index.text() == "Mode 4":
            if index.isChecked() == True:
                command = cmd.CMD_LED_MOD + '#' + '4' + '\n'
                self.client.send_data(command)
        elif index.text() == "Mode 5":
            if index.isChecked() == True:
                command = cmd.CMD_LED_MOD + '#' + '5' + '\n'
                self.client.send_data(command)
    def mode1_color(self):
        if (self.radioButtonOne.isChecked() == True) or (self.radioButtonThree.isChecked() == True):
            command = cmd.CMD_LED + '#' + str(self.rgb[0]) + '#' + str(self.rgb[1]) + '#' + str(self.rgb[2]) + '\n'
            self.client.send_data(command)
    def on_current_color_changed(self, color):
        try:
            self.rgb = self.rgbhex_to_rgb255(color.name())
            self.hsl = self.rgb255_to_hsl(self.rgb)
            self.change_hsl_text()
            self.change_rgb_text()
            self.mode1_color()
            self.update()
        except Exception as e:
            print(e)

    def paintEvent(self, e):
        try:
            qp = QPainter()
            qp.begin(self)
            brush = QBrush(QColor(self.rgb[0], self.rgb[1], self.rgb[2]))
            qp.setBrush(brush)
            qp.drawRect(20, 10, 80, 30)
            qp.end()
        except Exception as e:
            print(e)

    def dial_value_changed(self):
        try:
            self.lineEdit_H.setText(str(self.dial_color.value()))
            self.change_hsl()
            self.hex = self.hsl_to_rgbhex((self.hsl[0], self.hsl[1], self.hsl[2]))
            self.rgb = self.rgbhex_to_rgb255(self.hex)
            self.change_rgb_text()
            self.mode1_color()
            self.update()
        except Exception as e:
            print(e)

    def change_hsl(self):
        self.hsl[0] = float(self.lineEdit_H.text())
        self.hsl[1] = float(self.lineEdit_S.text())
        self.hsl[2] = float(self.lineEdit_L.text())

    def change_hsl_text(self):
        self.lineEdit_H.setText(str(int(self.hsl[0])))
        self.lineEdit_S.setText(str(round(self.hsl[1], 1)))
        self.lineEdit_L.setText(str(round(self.hsl[2], 1)))

    def change_rgb_text(self):
        self.lineEdit_R.setText(str(self.rgb[0]))
        self.lineEdit_G.setText(str(self.rgb[1]))
        self.lineEdit_B.setText(str(self.rgb[2]))

    def rgb255_to_rgbhex(self, rgb:np.array) -> str:
        f = lambda n: 0 if n < 0 else 255 if n > 255 else int(n)
        return '#%02x%02x%02x' % (f(rgb[0]), f(rgb[1]), f(rgb[2]))

    def rgbhex_to_rgb255(self, rgbhex: str) -> np.array:
        if rgbhex[0] == '#':
            rgbhex = rgbhex[1:]
        r = int(rgbhex[0:2], 16)
        g = int(rgbhex[2:4], 16)
        b = int(rgbhex[4:6], 16)
        return np.array((r, g, b))

    def rgb01_to_rgb255(self, rgb: np.array) -> np.array:
        return rgb * 255

    def rgb255_to_rgb01(self, rgb: np.array) -> np.array:
        return rgb / 255

    def rgb01_to_hsl(self, rgb: np.array) -> np.array:
        r, g, b = rgb
        lmin = min(r, g, b)
        lmax = max(r, g, b)
        if lmax == lmin:
            h = 0
        elif lmin == b:
            h = 60 + 60 * (g - r) / (lmax - lmin)
        elif lmin == r:
            h = 180 + 60 * (b - g) / (lmax - lmin)
        elif lmin == g:
            h = 300 + 60 * (r - b) / (lmax - lmin)
        else:
            h = 0
        s = lmax - lmin
        l = (lmax + lmin) / 2
        hsl = np.array((h, s, l))
        return hsl

    def hsl_to_rgb01(self, hsl: np.array) -> np.array:
        h, s, l = hsl
        lmin = l - s / 2
        lmax = l + s / 2
        ldif = lmax - lmin
        if h < 60:
            r, g, b = lmax, lmin + ldif * (0 + h) / 60, lmin
        elif h < 120:
            r, g, b = lmin + ldif * (120 - h) / 60, lmax, lmin
        elif h < 180:
            r, g, b = lmin, lmax, lmin + ldif * (h - 120) / 60
        elif h < 240:
            r, g, b = lmin, lmin + ldif * (240 - h) / 60, lmax
        elif h < 300:
            r, g, b = lmin + ldif * (h - 240) / 60, lmin, lmax
        else:
            r, g, b = lmax, lmin, lmin + ldif * (360 - h) / 60
        rgb = np.array((r, g, b))
        return rgb

def load_styles(app):
    """Load and apply styles from the external QSS file."""
    try:
        style_file = os.path.join(os.path.dirname(os.path.abspath(__file__)), 'styles/styles.qss')
        with open(style_file, 'r') as f:
            app.setStyleSheet(f.read())
    except Exception as e:
        print(f"Error loading stylesheet: {e}")

if __name__ == '__main__':
    app = QApplication(sys.argv)
    load_styles(app)
    myshow = MyWindow()
    myshow.show()
    sys.exit(app.exec_())
