"""Enhanced main application for Hexapod Robot Client with improved networking."""

import os
import sys
import time
import logging
from typing import Optional, Dict, Any, List, Tuple

from PyQt5.QtCore import Qt, QTimer, QPoint, QSize, QEvent, pyqtSignal, QObject
from PyQt5.QtGui import QIcon, QPixmap, QImage, QPainter, QPen, QColor, QFont, QCursor
from PyQt5.QtWidgets import (QApplication, QMainWindow, QMessageBox, QLabel, 
                           QPushButton, QLineEdit, QProgressBar, QSlider, QComboBox,
                           QHBoxLayout, QVBoxLayout, QWidget, QFrame, QGroupBox,
                           QRadioButton, QButtonGroup, QCheckBox, QFileDialog, QDialog,
                           QGridLayout, QSpacerItem, QSizePolicy, QSplitter, QTabWidget)

# Local imports
from .client_v2 import Client
from .Command import COMMAND as cmd
from .Face import Face
from .PID import Incremental_PID
from .logging_config import setup_logging
from .network_manager import NetworkManager, ConnectionState
from .exceptions import ConnectionError, NetworkError, TimeoutError

# Configure logging
setup_logging()
logger = logging.getLogger(__name__)

# Constants
DEFAULT_IP = "192.168.1.1"
DEFAULT_VIDEO_PORT = 8002
DEFAULT_COMMAND_PORT = 5002

# Try to import UI files, fall back to runtime generation if not available
try:
    from .ui_client import Ui_client
    from .ui_calibration import Ui_calibration
    from .ui_led import Ui_led
    UI_IMPORTED = True
except ImportError:
    UI_IMPORTED = False
    logger.warning("UI modules not found, using runtime UI generation")


class VideoHandler(QObject):
    """Handles video display and processing."""
    
    def __init__(self, client: Client, video_label: QLabel):
        super().__init__()
        self.client = client
        self.video_label = video_label
        self._frame = None
        self._lock = threading.RLock()
        self._stop_flag = False
        self._update_timer = QTimer()
        self._update_timer.timeout.connect(self.update_display)
        self._update_timer.start(33)  # ~30 FPS
    
    def update_frame(self, frame):
        """Update the current video frame."""
        with self._lock:
            self._frame = frame
    
    def update_display(self):
        """Update the video display with the current frame."""
        with self._lock:
            if self._frame is None:
                return
                
            try:
                # Convert frame to QImage
                height, width, channel = self._frame.shape
                bytes_per_line = 3 * width
                q_img = QImage(self._frame.data, width, height, 
                             bytes_per_line, QImage.Format_RGB888).rgbSwapped()
                
                # Scale image to fit the label while maintaining aspect ratio
                scaled_pixmap = QPixmap.fromImage(q_img).scaled(
                    self.video_label.size(), 
                    Qt.KeepAspectRatio,
                    Qt.SmoothTransformation
                )
                
                self.video_label.setPixmap(scaled_pixmap)
            except Exception as e:
                logger.error(f"Error updating video display: {e}")
    
    def stop(self):
        """Stop the video handler."""
        self._update_timer.stop()
        self._stop_flag = True


class RobotController:
    """Handles robot control commands and state management."""
    
    def __init__(self, client: Client, ui: QMainWindow):
        self.client = client
        self.ui = ui
        self.action_flag = 1
        self.gait_flag = 1
        self.move_point = [0, 0]
        self.power_value = [0, 0]
    
    def move(self) -> None:
        """Handle robot movement based on current state."""
        try:
            # Map UI coordinates to movement values
            x = self._map_value(self.move_point[0], 0, 100, -35, 35)
            y = self._map_value(100 - self.move_point[1], 0, 100, -35, 35)
            
            # Send movement command
            command = f"{cmd.CMD_MOVE} {x} {y}\n"
            self.client.send_data(command)
            
        except Exception as e:
            logger.error(f"Error in move: {e}")
    
    def set_action_mode(self, mode: int) -> None:
        """Set the action mode.
        
        Args:
            mode: Action mode (1 or 2)
        """
        try:
            self.action_flag = mode
            command = f"{cmd.CMD_ACTION_MODE} {mode}\n"
            self.client.send_data(command)
            logger.info(f"Action mode set to {mode}")
        except Exception as e:
            logger.error(f"Error setting action mode: {e}")
    
    def set_gait_mode(self, mode: int) -> None:
        """Set the gait mode.
        
        Args:
            mode: Gait mode (1 or 2)
        """
        try:
            self.gait_flag = mode
            command = f"{cmd.CMD_GAIT_MODE} {mode}\n"
            self.client.send_data(command)
            logger.info(f"Gait mode set to {mode}")
        except Exception as e:
            logger.error(f"Error setting gait mode: {e}")
    
    def power(self) -> None:
        """Update power display and status."""
        try:
            command = f"{cmd.CMD_POWER}\n"
            # Update UI with power values
            if hasattr(self.ui, 'progress_Power1') and hasattr(self.ui, 'progress_Power2'):
                self.ui.progress_Power1.setFormat(f"{self.power_value[0]:.1f}V")
                self.ui.progress_Power2.setFormat(f"{self.power_value[1]:.1f}V")
                
                # Map voltage to percentage for display
                power1_pct = self._constrain(int((self.power_value[0] - 5.00) / 3.40 * 100), 0, 100)
                power2_pct = self._constrain(int((self.power_value[1] - 7.00) / 1.40 * 100), 0, 100)
                
                self.ui.progress_Power1.setValue(power1_pct)
                self.ui.progress_Power2.setValue(power2_pct)
                
            # Request power status
            self.client.send_data(command)
            
        except Exception as e:
            logger.error(f"Error updating power status: {e}")
    
    def _map_value(self, value, in_min, in_max, out_min, out_max):
        """Map a value from one range to another."""
        return (value - in_min) * (out_max - out_min) / (in_max - in_min) + out_min
    
    def _constrain(self, value, min_val, max_val):
        """Constrain a value between min and max."""
        return max(min_val, min(value, max_val))


class HexapodClient(QMainWindow):
    """Main application window for the Hexapod Robot Client."""
    
    def __init__(self):
        super().__init__()
        
        # Initialize UI
        self._init_ui()
        
        # Initialize components
        self.client = Client(use_network_manager=True)
        self.controller = RobotController(self.client, self)
        self.video_handler = VideoHandler(self.client, self.video_label)
        
        # Connect signals
        self._connect_signals()
        
        # Load settings
        self._load_settings()
        
        logger.info("Hexapod Client initialized")
    
    def _init_ui(self):
        """Initialize the user interface."""
        self.setWindowTitle("Hexapod Robot Controller")
        self.setGeometry(100, 100, 1000, 800)
        
        # Create main widget and layout
        main_widget = QWidget()
        self.setCentralWidget(main_widget)
        
        layout = QHBoxLayout(main_widget)
        
        # Video display
        video_group = QGroupBox("Video Feed")
        video_layout = QVBoxLayout()
        
        self.video_label = QLabel()
        self.video_label.setAlignment(Qt.AlignCenter)
        self.video_label.setMinimumSize(640, 480)
        self.video_label.setStyleSheet("background-color: black;")
        
        video_layout.addWidget(self.video_label)
        video_group.setLayout(video_layout)
        
        # Control panel
        control_group = QGroupBox("Controls")
        control_layout = QVBoxLayout()
        
        # Connection controls
        conn_group = QGroupBox("Connection")
        conn_layout = QVBoxLayout()
        
        self.ip_edit = QLineEdit(DEFAULT_IP)
        self.connect_btn = QPushButton("Connect")
        self.video_btn = QPushButton("Start Video")
        
        conn_layout.addWidget(QLabel("IP Address:"))
        conn_layout.addWidget(self.ip_edit)
        conn_layout.addWidget(self.connect_btn)
        conn_layout.addWidget(self.video_btn)
        conn_group.setLayout(conn_layout)
        
        # Movement controls
        move_group = QGroupBox("Movement")
        move_layout = QGridLayout()
        
        # Add movement buttons (forward, back, left, right, etc.)
        # This is a simplified version - expand as needed
        self.forward_btn = QPushButton("↑")
        self.back_btn = QPushButton("↓")
        self.left_btn = QPushButton("←")
        self.right_btn = QPushButton("→")
        self.stop_btn = QPushButton("STOP")
        
        move_layout.addWidget(self.forward_btn, 0, 1)
        move_layout.addWidget(self.left_btn, 1, 0)
        move_layout.addWidget(self.stop_btn, 1, 1)
        move_layout.addWidget(self.right_btn, 1, 2)
        move_layout.addWidget(self.back_btn, 2, 1)
        
        move_group.setLayout(move_layout)
        
        # Status indicators
        status_group = QGroupBox("Status")
        status_layout = QVBoxLayout()
        
        self.connection_status = QLabel("Disconnected")
        self.video_status = QLabel("Video: Off")
        self.power_status = QLabel("Power: --V")
        
        status_layout.addWidget(self.connection_status)
        status_layout.addWidget(self.video_status)
        status_layout.addWidget(self.power_status)
        status_group.setLayout(status_layout)
        
        # Assemble control panel
        control_layout.addWidget(conn_group)
        control_layout.addWidget(move_group)
        control_layout.addWidget(status_group)
        control_layout.addStretch()
        control_group.setLayout(control_layout)
        control_group.setMaximumWidth(300)
        
        # Add to main layout
        layout.addWidget(video_group)
        layout.addWidget(control_group)
        
        # Set window icon
        self.setWindowIcon(QIcon(os.path.join('Picture', 'logo_Mini.png')))
    
    def _connect_signals(self):
        """Connect UI signals to slots."""
        # Connection controls
        self.connect_btn.clicked.connect(self._toggle_connection)
        self.video_btn.clicked.connect(self._toggle_video)
        
        # Movement controls
        self.forward_btn.pressed.connect(lambda: self._move_robot('forward'))
        self.forward_btn.released.connect(self._stop_robot)
        self.back_btn.pressed.connect(lambda: self._move_robot('back'))
        self.back_btn.released.connect(self._stop_robot)
        self.left_btn.pressed.connect(lambda: self._move_robot('left'))
        self.left_btn.released.connect(self._stop_robot)
        self.right_btn.pressed.connect(lambda: self._move_robot('right'))
        self.right_btn.released.connect(self._stop_robot)
        self.stop_btn.clicked.connect(self._stop_robot)
    
    def _toggle_connection(self):
        """Toggle connection to the robot."""
        if not self.client.tcp_flag:
            self._connect_to_robot()
        else:
            self._disconnect_from_robot()
    
    def _connect_to_robot(self):
        """Connect to the robot."""
        ip = self.ip_edit.text().strip()
        if not ip:
            QMessageBox.warning(self, "Error", "Please enter a valid IP address")
            return
        
        try:
            self.connect_btn.setEnabled(False)
            self.connect_btn.setText("Connecting...")
            
            # Connect to robot
            self.client.turn_on_client(ip)
            
            if self.client.tcp_flag:
                self.connection_status.setText("Connected")
                self.connect_btn.setText("Disconnect")
                self.video_btn.setEnabled(True)
                logger.info(f"Connected to robot at {ip}")
            else:
                QMessageBox.critical(self, "Error", "Failed to connect to robot")
                self.connect_btn.setText("Connect")
        
        except Exception as e:
            logger.error(f"Connection error: {e}")
            QMessageBox.critical(self, "Error", f"Failed to connect: {str(e)}")
            self.connect_btn.setText("Connect")
        
        finally:
            self.connect_btn.setEnabled(True)
    
    def _disconnect_from_robot(self):
        """Disconnect from the robot."""
        try:
            self.client.turn_off_client()
            self.connection_status.setText("Disconnected")
            self.video_status.setText("Video: Off")
            self.connect_btn.setText("Connect")
            self.video_btn.setEnabled(False)
            logger.info("Disconnected from robot")
        except Exception as e:
            logger.error(f"Error disconnecting: {e}")
    
    def _toggle_video(self):
        """Toggle video streaming."""
        if not hasattr(self, '_video_running') or not self._video_running:
            self._start_video()
        else:
            self._stop_video()
    
    def _start_video(self):
        """Start video streaming."""
        try:
            self.video_btn.setEnabled(False)
            self.video_btn.setText("Starting...")
            
            # Start video in a separate thread to avoid blocking the UI
            self.client.receiving_video(self.ip_edit.text().strip())
            
            self._video_running = True
            self.video_status.setText("Video: On")
            self.video_btn.setText("Stop Video")
            logger.info("Video streaming started")
            
        except Exception as e:
            logger.error(f"Failed to start video: {e}")
            QMessageBox.critical(self, "Error", f"Failed to start video: {str(e)}")
            self.video_btn.setText("Start Video")
        
        finally:
            self.video_btn.setEnabled(True)
    
    def _stop_video(self):
        """Stop video streaming."""
        try:
            self._video_running = False
            # The client will handle stopping the video stream
            self.video_status.setText("Video: Off")
            self.video_btn.setText("Start Video")
            logger.info("Video streaming stopped")
        except Exception as e:
            logger.error(f"Error stopping video: {e}")
    
    def _move_robot(self, direction: str):
        """Send movement command to the robot.
        
        Args:
            direction: Movement direction ('forward', 'back', 'left', 'right')
        """
        if not self.client.tcp_flag:
            return
        
        try:
            # Map direction to command
            commands = {
                'forward': f"{cmd.CMD_MOVE} 0 30\n",
                'back': f"{cmd.CMD_MOVE} 0 -30\n",
                'left': f"{cmd.CMD_MOVE} -30 0\n",
                'right': f"{cmd.CMD_MOVE} 30 0\n"
            }
            
            if direction in commands:
                self.client.send_data(commands[direction])
        
        except Exception as e:
            logger.error(f"Error sending movement command: {e}")
    
    def _stop_robot(self):
        """Send stop command to the robot."""
        if not self.client.tcp_flag:
            return
        
        try:
            self.client.send_data(f"{cmd.CMD_STOP}\n")
        except Exception as e:
            logger.error(f"Error sending stop command: {e}")
    
    def _load_settings(self):
        """Load application settings."""
        try:
            # Try to load last used IP address
            if os.path.exists('IP.txt'):
                with open('IP.txt', 'r') as f:
                    ip = f.readline().strip()
                    if ip:
                        self.ip_edit.setText(ip)
        except Exception as e:
            logger.warning(f"Failed to load settings: {e}")
    
    def closeEvent(self, event):
        """Handle application close event."""
        try:
            # Save current IP address
            with open('IP.txt', 'w') as f:
                f.write(self.ip_edit.text().strip())
            
            # Clean up
            if hasattr(self, '_video_running') and self._video_running:
                self._stop_video()
            
            if self.client.tcp_flag:
                self._disconnect_from_robot()
            
            logger.info("Application closed")
            
        except Exception as e:
            logger.error(f"Error during shutdown: {e}")
        
        event.accept()


def main():
    """Main application entry point."""
    try:
        # Set up high DPI scaling for better display on high-resolution screens
        if hasattr(Qt, 'AA_EnableHighDpiScaling'):
            QApplication.setAttribute(Qt.AA_EnableHighDpiScaling, True)
        if hasattr(Qt, 'AA_UseHighDpiPixmaps'):
            QApplication.setAttribute(Qt.AA_UseHighDpiPixmaps, True)
        
        # Create and show the main window
        app = QApplication(sys.argv)
        window = HexapodClient()
        window.show()
        
        # Set application style
        app.setStyle('Fusion')
        
        # Set application information
        app.setApplicationName("Hexapod Robot Controller")
        app.setApplicationVersion("2.0.0")
        app.setOrganizationName("Freenove")
        
        logger.info("Application started")
        
        # Start the event loop
        sys.exit(app.exec_())
        
    except Exception as e:
        logger.critical(f"Fatal error: {e}", exc_info=True)
        QMessageBox.critical(None, "Fatal Error", 
                           f"A fatal error occurred:\n{str(e)}\n\n"
                           "The application will now exit.")
        sys.exit(1)


if __name__ == "__main__":
    main()
