# -*- coding: utf-8 -*-
import sys
import math
import threading
import time
import cv2
import numpy as np
from abc import ABC, abstractmethod
from typing import List, Tuple, Optional, Dict
from dataclasses import dataclass
from enum import Enum

from ui_led import Ui_led
from ui_face import Ui_Face
from ui_client import Ui_client
from PyQt5 import QtCore, QtGui, QtWidgets
from PyQt5.QtCore import *
from PyQt5.QtWidgets import *
from PyQt5.QtGui import *
from Client import *
from Calibration import *
from camera_recording import CameraRecorder

# Constants
class UIConstants:
    MOVE_CENTER = [325, 635]
    ATTITUDE_CENTER = [800, 180]
    POSITION_CENTER = [800, 650]
    MOVE_RADIUS = 100
    TIMER_VIDEO = 10
    TIMER_POWER = 3000
    TIMER_SONIC = 100
    BUZZER_DELAY = 120

class RobotModes:
    ACTION_MODE_1 = 1
    ACTION_MODE_2 = 2
    GAIT_MODE_1 = 1
    GAIT_MODE_2 = 2

# Command Pattern Implementation
class Command(ABC):
    @abstractmethod
    def execute(self) -> str:
        pass

class MoveCommand(Command):
    def __init__(self, gait_flag: int, x: float, y: float, speed: str, angle: float):
        self.gait_flag = gait_flag
        self.x = x
        self.y = y
        self.speed = speed
        self.angle = angle

    def execute(self) -> str:
        return f"{cmd.CMD_MOVE}#{self.gait_flag}#{round(self.x)}#{round(self.y)}#{self.speed}#{round(self.angle)}\n"

class AttitudeCommand(Command):
    def __init__(self, roll: float, pitch: float, yaw: float):
        self.roll = roll
        self.pitch = pitch
        self.yaw = yaw

    def execute(self) -> str:
        return f"{cmd.CMD_ATTITUDE}#{round(self.roll)}#{round(self.pitch)}#{round(self.yaw)}\n"

class PositionCommand(Command):
    def __init__(self, x: float, y: float, z: float):
        self.x = x
        self.y = y
        self.z = z

    def execute(self) -> str:
        return f"{cmd.CMD_POSITION}#{round(self.x)}#{round(self.y)}#{round(self.z)}\n"

class BuzzerCommand(Command):
    def __init__(self, state: bool):
        self.state = state

    def execute(self) -> str:
        return f"{cmd.CMD_BUZZER}#{'1' if self.state else '0'}\n"

# State Pattern for Robot Control
class RobotState:
    def __init__(self):
        self.action_mode = RobotModes.ACTION_MODE_1
        self.gait_mode = RobotModes.GAIT_MODE_1
        self.move_speed = "8"
        self.head_position = [90, 90]
        self.attitude_values = [0, 0, 0]  # roll, pitch, yaw
        self.position_values = [0, 0, 0]  # x, y, z

@dataclass
class KeyState:
    w: bool = False
    a: bool = False
    s: bool = False
    d: bool = False
    space: bool = False

# Strategy Pattern for Input Handling
class InputHandler(ABC):
    @abstractmethod
    def handle_input(self, event, context) -> bool:
        pass

class KeyboardHandler(InputHandler):
    def __init__(self, command_processor):
        self.command_processor = command_processor

    def handle_input(self, event, context) -> bool:
        key_mappings = {
            Qt.Key_C: context.connect,
            Qt.Key_V: context.video,
            Qt.Key_R: context.relax,
            Qt.Key_L: context.show_led_window,
            Qt.Key_B: context.toggle_balance,
            Qt.Key_F: context.toggle_face_recognition,
            Qt.Key_U: context.toggle_sonic,
            Qt.Key_I: context.show_face_window,
            Qt.Key_T: context.show_calibration_window,
            Qt.Key_Y: context.buzzer
        }
        
        if event.key() in key_mappings:
            try:
                key_mappings[event.key()]()
                return True
            except Exception as e:
                print(f"Error handling key {event.key()}: {e}")
                return False
        return False

class MouseHandler(InputHandler):
    def __init__(self, command_processor):
        self.command_processor = command_processor

    def handle_input(self, event, context) -> bool:
        x, y = event.pos().x(), event.pos().y()
        
        if self._is_in_attitude_area(x, y):
            context.handle_attitude_control(x, y)
            return True
        elif self._is_in_position_area(x, y):
            context.handle_position_control(x, y)
            return True
        elif self._is_in_movement_area(x, y):
            context.handle_movement_control(x, y)
            return True
        return False

    def _is_in_attitude_area(self, x: int, y: int) -> bool:
        return 700 <= x <= 900 and 80 <= y <= 280

    def _is_in_position_area(self, x: int, y: int) -> bool:
        return 700 <= x <= 900 and 550 <= y <= 750

    def _is_in_movement_area(self, x: int, y: int) -> bool:
        return 225 <= x <= 425 and 550 <= y <= 750

# Command Processor
class CommandProcessor:
    def __init__(self, client):
        self.client = client

    def execute_command(self, command: Command):
        try:
            cmd_str = command.execute()
            print(cmd_str)
            self.client.send_data(cmd_str)
        except Exception as e:
            print(f"Error executing command: {e}")

    def send_raw_command(self, cmd_str: str):
        try:
            print(cmd_str)
            self.client.send_data(cmd_str)
        except Exception as e:
            print(f"Error sending command: {e}")

# Movement Controller
class MovementController:
    def __init__(self, command_processor: CommandProcessor, robot_state: RobotState):
        self.command_processor = command_processor
        self.robot_state = robot_state

    def move_robot(self, move_point: List[int]):
        x = self._map_value(move_point[0] - UIConstants.MOVE_CENTER[0], 0, 100, 0, 35)
        y = self._map_value(UIConstants.MOVE_CENTER[1] - move_point[1], 0, 100, 0, 35)
        
        angle = self._calculate_angle(x, y)
        
        move_cmd = MoveCommand(
            self.robot_state.gait_mode, x, y, 
            self.robot_state.move_speed, angle
        )
        self.command_processor.execute_command(move_cmd)

    def _calculate_angle(self, x: float, y: float) -> float:
        if self.robot_state.action_mode == RobotModes.ACTION_MODE_1:
            return 0
        
        if x == 0 and y == 0:
            return 0
            
        angle = math.degrees(math.atan2(x, y))
        
        if angle < -90 and angle >= -180:
            angle += 360
            
        if -90 <= angle <= 90:
            return self._map_value(angle, -90, 90, -10, 10)
        else:
            return self._map_value(angle, 270, 90, 10, -10)

    def _map_value(self, value: float, from_low: float, from_high: float, 
                   to_low: float, to_high: float) -> float:
        return (to_high - to_low) * (value - from_low) / (from_high - from_low) + to_low

# Camera Controller
class CameraController:
    def __init__(self, command_processor: CommandProcessor):
        self.command_processor = command_processor
        self.camera_recorder: Optional[CameraRecorder] = None

    def initialize_camera(self, output_dir: str, video_label):
        self.camera_recorder = CameraRecorder(output_dir=output_dir, video_label=video_label)

    def take_photo(self, video_widget) -> str:
        if not self.camera_recorder:
            raise ValueError("Camera not initialized")
            
        # Beep before capture
        buzzer_on = BuzzerCommand(True)
        self.command_processor.execute_command(buzzer_on)
        
        # Capture after delay
        QTimer.singleShot(UIConstants.BUZZER_DELAY, 
                         lambda: self._capture_and_buzz_off(video_widget))
        
        return "Photo capture initiated"

    def _capture_and_buzz_off(self, video_widget):
        try:
            pix = video_widget.pixmap()
            saved_path = self.camera_recorder.capture(pixmap=pix if pix else None)
            print(f'Photo saved to: {saved_path}')
        except Exception as e:
            print(f"Error capturing photo: {e}")
        finally:
            buzzer_off = BuzzerCommand(False)
            self.command_processor.execute_command(buzzer_off)

# Connection Manager
class ConnectionManager:
    def __init__(self, client, command_processor: CommandProcessor):
        self.client = client
        self.command_processor = command_processor
        self.video_thread: Optional[threading.Thread] = None
        self.instruction_thread: Optional[threading.Thread] = None

    def connect_to_robot(self, ip: str) -> bool:
        try:
            self.client.turn_on_client(ip)
            self.video_thread = threading.Thread(target=self.client.receiving_video, args=(ip,))
            self.instruction_thread = threading.Thread(target=self._receive_instructions, args=(ip,))
            
            self.video_thread.start()
            self.instruction_thread.start()
            return True
        except Exception as e:
            print(f"Connection failed: {e}")
            return False

    def disconnect_from_robot(self):
        try:
            if self.video_thread:
                stop_thread(self.video_thread)
            if self.instruction_thread:
                stop_thread(self.instruction_thread)
            
            self.client.tcp_flag = False
            self.client.turn_off_client()
        except Exception as e:
            print(f"Disconnect error: {e}")

    def _receive_instructions(self, ip: str):
        try:
            self.client.client_socket1.connect((ip, 5002))
            self.client.tcp_flag = True
            print("Connection Successful!")
        except Exception as e:
            print("Connect to server Failed!: Server IP is right? Server is opened?")
            self.client.tcp_flag = False
            return

        while True:
            try:
                alldata = self.client.receive_data()
                if not alldata:
                    break
                self._process_received_data(alldata)
            except:
                self.client.tcp_flag = False
                break

    def _process_received_data(self, data: str):
        cmd_array = data.split('\n')
        if cmd_array[-1] != "":
            cmd_array = cmd_array[:-1]
            
        for one_cmd in cmd_array:
            data_parts = one_cmd.split("#")
            if not data_parts or data_parts == [""]:
                self.client.tcp_flag = False
                break
            # Process different command types here
            print(data_parts)

# Main Window with improved OOP structure
class RobotControlWindow(QMainWindow, Ui_client):
    def __init__(self):
        super().__init__()
        self.setupUi(self)
        self._initialize_components()
        self._setup_ui()
        self._connect_signals()

    def _initialize_components(self):
        self.client = Client()
        self.robot_state = RobotState()
        self.key_state = KeyState()
        self.command_processor = CommandProcessor(self.client)
        self.movement_controller = MovementController(self.command_processor, self.robot_state)
        self.camera_controller = CameraController(self.command_processor)
        self.connection_manager = ConnectionManager(self.client, self.command_processor)
        
        # Input handlers
        self.keyboard_handler = KeyboardHandler(self.command_processor)
        self.mouse_handler = MouseHandler(self.command_processor)

    def _setup_ui(self):
        self.setWindowIcon(QIcon('Picture/logo_Mini.png'))
        self.Video.setScaledContents(True)
        self.Video.setPixmap(QPixmap('Picture/Spider_client.png'))
        
        # Initialize camera
        self.camera_controller.initialize_camera('Captures', self.Video)
        
        # Load IP from file
        self._load_ip_address()
        
        # Setup focus handling
        self._setup_focus_handling()
        
        # Initialize UI state
        self._initialize_ui_controls()
        
        # Setup timers
        self._setup_timers()

    def _load_ip_address(self):
        try:
            with open('IP.txt', 'r') as file:
                self.lineEdit_IP_Adress.setText(file.readline().strip())
        except FileNotFoundError:
            pass

    def _setup_focus_handling(self):
        try:
            self.setFocusPolicy(Qt.StrongFocus)
            self.Video.setFocusPolicy(Qt.StrongFocus)
            self.lineEdit_IP_Adress.setFocusPolicy(Qt.ClickFocus)
            self.lineEdit_IP_Adress.clearFocus()
            self.Video.setFocus(Qt.OtherFocusReason)
            self.lineEdit_IP_Adress.returnPressed.connect(
                lambda: self.Video.setFocus(Qt.TabFocusReason)
            )
        except Exception as e:
            print(f"Focus setup error: {e}")

    def _initialize_ui_controls(self):
        # Sliders
        self._setup_sliders()
        
        # Checkboxes
        self._setup_checkboxes()
        
        # UI state variables
        self.power_value = [100, 100]
        self.move_point = UIConstants.MOVE_CENTER.copy()
        self.move_flag = False
        self.drawpoint = [UIConstants.ATTITUDE_CENTER.copy(), UIConstants.POSITION_CENTER.copy()]

    def _setup_sliders(self):
        slider_configs = [
            (self.slider_head, 50, 180, 90, self.head_up_and_down),
            (self.slider_head_1, 0, 180, 90, self.head_left_and_right),
            (self.slider_speed, 2, 10, 8, self.speed_changed),
            (self.slider_roll, -15, 15, 0, self.roll_changed),
            (self.slider_Z, -20, 20, 0, self.z_changed)
        ]
        
        for slider, min_val, max_val, default, handler in slider_configs:
            slider.setMinimum(min_val)
            slider.setMaximum(max_val)
            slider.setSingleStep(1)
            slider.setValue(default)
            slider.valueChanged.connect(handler)

    def _setup_checkboxes(self):
        self.ButtonActionMode1.setChecked(True)
        self.ButtonActionMode1.toggled.connect(lambda: self.action_mode_changed(self.ButtonActionMode1))
        self.ButtonActionMode2.setChecked(False)
        self.ButtonActionMode2.toggled.connect(lambda: self.action_mode_changed(self.ButtonActionMode2))
        
        self.ButtonGaitMode1.setChecked(True)
        self.ButtonGaitMode1.toggled.connect(lambda: self.gait_mode_changed(self.ButtonGaitMode1))
        self.ButtonGaitMode2.setChecked(False)
        self.ButtonGaitMode2.toggled.connect(lambda: self.gait_mode_changed(self.ButtonGaitMode2))

    def _setup_timers(self):
        self.timer = QTimer(self)
        self.timer.timeout.connect(self.refresh_image)
        
        self.timer_power = QTimer(self)
        self.timer_power.timeout.connect(self.update_power_status)
        
        self.timer_sonic = QTimer(self)
        self.timer_sonic.timeout.connect(self.get_sonic_data)

    def _connect_signals(self):
        # Button connections
        self.Button_Connect.clicked.connect(self.toggle_connection)
        self.Button_Video.clicked.connect(self.toggle_video)
        self.Button_IMU.clicked.connect(self.toggle_balance)
        self.Button_Calibration.clicked.connect(self.show_calibration_window)
        self.Button_LED.clicked.connect(self.show_led_window)
        self.Button_Face_ID.clicked.connect(self.show_face_window)
        self.Button_Face_Recognition.clicked.connect(self.toggle_face_recognition)
        self.Button_Sonic.clicked.connect(self.toggle_sonic)
        self.Button_Take_Photo.clicked.connect(self.take_photo)
        self.Button_Relax.clicked.connect(self.toggle_relax)
        self.Button_Buzzer.pressed.connect(lambda: self.set_buzzer(True))
        self.Button_Buzzer.released.connect(lambda: self.set_buzzer(False))

    # Event handlers
    def keyPressEvent(self, event):
        if self.keyboard_handler.handle_input(event, self):
            return
            
        # Movement keys
        if not event.isAutoRepeat():
            movement_handled = self._handle_movement_keys(event, True)
            if movement_handled:
                return
        
        super().keyPressEvent(event)

    def keyReleaseEvent(self, event):
        if not event.isAutoRepeat():
            self._handle_movement_keys(event, False)
        super().keyReleaseEvent(event)

    def _handle_movement_keys(self, event, is_press: bool) -> bool:
        key_to_state = {
            Qt.Key_W: ('w', [UIConstants.MOVE_CENTER[0], UIConstants.MOVE_CENTER[1] - 100]),
            Qt.Key_S: ('s', [UIConstants.MOVE_CENTER[0], UIConstants.MOVE_CENTER[1] + 100]),
            Qt.Key_A: ('a', [UIConstants.MOVE_CENTER[0] - 100, UIConstants.MOVE_CENTER[1]]),
            Qt.Key_D: ('d', [UIConstants.MOVE_CENTER[0] + 100, UIConstants.MOVE_CENTER[1]])
        }
        
        if event.key() in key_to_state:
            state_attr, move_pos = key_to_state[event.key()]
            setattr(self.key_state, state_attr, is_press)
            
            if is_press:
                self.move_point = move_pos.copy()
                print(f"{event.key()} pressed")
            else:
                self.move_point = UIConstants.MOVE_CENTER.copy()
                print(f"{event.key()} released")
                
            self.movement_controller.move_robot(self.move_point)
            self.update()
            return True
        return False

    def mousePressEvent(self, event):
        self.mouse_handler.handle_input(event, self)
        super().mousePressEvent(event)

    def mouseMoveEvent(self, event):
        self.mouse_handler.handle_input(event, self)
        super().mouseMoveEvent(event)

    def mouseReleaseEvent(self, event):
        if self.move_flag:
            self.move_point = UIConstants.MOVE_CENTER.copy()
            self.move_flag = False
            self.movement_controller.move_robot(self.move_point)
        self.update()
        super().mouseReleaseEvent(event)

    # Control methods
    def handle_attitude_control(self, x: int, y: int):
        try:
            self.drawpoint[0] = [x, y]
            self.update()
            self._send_attitude_command()
        except Exception as e:
            print(f"Attitude control error: {e}")

    def handle_position_control(self, x: int, y: int):
        try:
            self.drawpoint[1] = [x, y]
            self.update()
            self._send_position_command()
        except Exception as e:
            print(f"Position control error: {e}")

    def handle_movement_control(self, x: int, y: int):
        try:
            center_x, center_y = UIConstants.MOVE_CENTER
            r = (x - center_x) ** 2 + (center_y - y) ** 2
            
            if r < UIConstants.MOVE_RADIUS ** 2:
                self.move_flag = True
                self.move_point = [x, y]
            else:
                angle = math.atan2(center_y - y, x - center_x)
                self.move_point = [
                    UIConstants.MOVE_RADIUS * math.cos(angle) + center_x,
                    center_y - UIConstants.MOVE_RADIUS * math.sin(angle)
                ]
            
            self.movement_controller.move_robot(self.move_point)
            self.update()
        except Exception as e:
            print(f"Movement control error: {e}")

    def _send_attitude_command(self):
        r = self._map_value(self.drawpoint[0][0] - 800, -100, 100, -15, 15)
        p = self._map_value(180 - self.drawpoint[0][1], -100, 100, -15, 15)
        y = self.slider_roll.value()
        
        attitude_cmd = AttitudeCommand(r, p, y)
        self.command_processor.execute_command(attitude_cmd)

    def _send_position_command(self):
        x = self._map_value(self.drawpoint[1][0] - 800, -100, 100, -40, 40)
        y = self._map_value(650 - self.drawpoint[1][1], -100, 100, -40, 40)
        z = self.slider_Z.value()
        
        position_cmd = PositionCommand(x, y, z)
        self.command_processor.execute_command(position_cmd)

    def _map_value(self, value: float, from_low: float, from_high: float, 
                   to_low: float, to_high: float) -> float:
        return (to_high - to_low) * (value - from_low) / (from_high - from_low) + to_low

    # Button event handlers
    def toggle_connection(self):
        try:
            if self.Button_Connect.text() == 'Connect':
                self._save_ip_address()
                ip = self.lineEdit_IP_Adress.text()
                
                if self.connection_manager.connect_to_robot(ip):
                    self.Button_Connect.setText('Disconnect')
                    self.timer_power.start(UIConstants.TIMER_POWER)
                    self._clear_ip_focus()
            else:
                self.connection_manager.disconnect_from_robot()
                self.Button_Connect.setText('Connect')
                self.timer_power.stop()
                self._clear_ip_focus()
        except Exception as e:
            print(f"Connection toggle error: {e}")

    def _save_ip_address(self):
        try:
            with open('IP.txt', 'w') as file:
                file.write(self.lineEdit_IP_Adress.text())
        except Exception as e:
            print(f"IP save error: {e}")

    def _clear_ip_focus(self):
        try:
            self.lineEdit_IP_Adress.clearFocus()
            self.Video.setFocus(Qt.OtherFocusReason)
        except Exception as e:
            print(f"Focus clear error: {e}")

    def toggle_video(self):
        if self.Button_Video.text() == 'Open Video':
            self.timer.start(UIConstants.TIMER_VIDEO)
            self.Button_Video.setText('Close Video')
        else:
            self.timer.stop()
            self.Button_Video.setText('Open Video')

    def take_photo(self):
        try:
            result = self.camera_controller.take_photo(self.Video)
            print(result)
        except Exception as e:
            print(f"Photo capture error: {e}")

    def toggle_relax(self):
        try:
            if self.Button_Relax.text() == "Relax":
                self.Button_Relax.setText("Relaxed")
                command = f"{cmd.CMD_SERVOPOWER}#0\n"
            else:
                self.Button_Relax.setText("Relax")
                command = f"{cmd.CMD_SERVOPOWER}#1\n"
            self.command_processor.send_raw_command(command)
        except Exception as e:
            print(f"Relax toggle error: {e}")

    def set_buzzer(self, state: bool):
        buzzer_cmd = BuzzerCommand(state)
        self.command_processor.execute_command(buzzer_cmd)
        self.Button_Buzzer.setText('Noise' if state else 'Buzzer')

    def toggle_balance(self):
        try:
            if self.Button_IMU.text() == 'Balance':
                command = f"{cmd.CMD_BALANCE}#1\n"
                self.Button_IMU.setText("Close")
            else:
                command = f"{cmd.CMD_BALANCE}#0\n"
                self.Button_IMU.setText('Balance')
            self.command_processor.send_raw_command(command)
        except Exception as e:
            print(f"Balance toggle error: {e}")

    def toggle_sonic(self):
        if self.Button_Sonic.text() == 'Sonic':
            self.timer_sonic.start(UIConstants.TIMER_SONIC)
            self.Button_Sonic.setText('Close')
        else:
            self.timer_sonic.stop()
            self.Button_Sonic.setText('Sonic')

    def toggle_face_recognition(self):
        try:
            if self.Button_Face_Recognition.text() == "Face Recog":
                self.client.fece_recognition_flag = True
                self.Button_Face_Recognition.setText("Close")
            elif self.Button_Face_Recognition.text() == "Close":
                self.client.fece_recognition_flag = False
                self.Button_Face_Recognition.setText("Face Recog")
        except Exception as e:
            print(f"Face recognition toggle error: {e}")

    # Slider event handlers
    def speed_changed(self):
        self.robot_state.move_speed = str(self.slider_speed.value())
        self.label_speed.setText(str(self.slider_speed.value()))

    def z_changed(self):
        self.label_Z.setText(str(self.slider_Z.value()))
        self._send_position_command()

    def roll_changed(self):
        self.label_roll.setText(str(self.slider_roll.value()))
        self._send_attitude_command()

    def head_up_and_down(self):
        try:
            angle = str(self.slider_head.value())
            self.label_head.setText(angle)
            command = f"{cmd.CMD_HEAD}#0#{angle}\n"
            self.command_processor.send_raw_command(command)
        except Exception as e:
            print(f"Head up/down error: {e}")

    def head_left_and_right(self):
        try:
            angle = str(180 - self.slider_head_1.value())
            self.label_head_1.setText(angle)
            command = f"{cmd.CMD_HEAD}#1#{angle}\n"
            self.command_processor.send_raw_command(command)
        except Exception as e:
            print(f"Head left/right error: {e}")

    # Mode change handlers
    def action_mode_changed(self, mode):
        if mode.text() == "Action Mode 1" and mode.isChecked():
            self.ButtonActionMode1.setChecked(True)
            self.ButtonActionMode2.setChecked(False)
            self.robot_state.action_mode = RobotModes.ACTION_MODE_1
        elif mode.text() == "Action Mode 2" and mode.isChecked():
            self.ButtonActionMode1.setChecked(False)
            self.ButtonActionMode2.setChecked(True)
            self.robot_state.action_mode = RobotModes.ACTION_MODE_2

    def gait_mode_changed(self, mode):
        if mode.text() == "Gait Mode 1" and mode.isChecked():
            self.ButtonGaitMode1.setChecked(True)
            self.ButtonGaitMode2.setChecked(False)
            self.robot_state.gait_mode = RobotModes.GAIT_MODE_1
        elif mode.text() == "Gait Mode 2" and mode.isChecked():
            self.ButtonGaitMode1.setChecked(False)
            self.ButtonGaitMode2.setChecked(True)
            self.robot_state.gait_mode = RobotModes.GAIT_MODE_2

    # Timer callbacks
    def refresh_image(self):
        if not self.client.video_flag:
            try:
                height, width, bytesPerComponent = self.client.image.shape
                cv2.cvtColor(self.client.image, cv2.COLOR_BGR2RGB, self.client.image)
                QImg = QImage(self.client.image.data.tobytes(), width, height, 
                            3 * width, QImage.Format_RGB888)
                self.Video.setPixmap(QPixmap.fromImage(QImg))
                self.client.video_flag = True
            except Exception as e:
                print(f"Image refresh error: {e}")
    
    def update_power_status(self):
        try:
            command = f"{cmd.CMD_POWER}\n"
            self.command_processor.send_raw_command(command)
            self.progressBar_Battery.setValue(self.client.power_value[0])
            self.progressBar_Adc.setValue(self.client.power_value[1])
        except Exception as e:
            print(f"Power status update error: {e}")

    def get_sonic_data(self):
        try:
            command = f"{cmd.CMD_SONIC}\n"
            self.command_processor.send_raw_command(command)
            self.label_Sonic.setText(f"{self.client.sonic} cm")
        except Exception as e:
            print(f"Sonic data error: {e}")

    # Window management methods
    def show_led_window(self):
        try:
            self.led_window = ledWindow(self.command_processor)
            self.led_window.show()
        except Exception as e:
            print(f"Error showing LED window: {e}")

    def show_face_window(self):
        try:
            self.face_window = faceWindow(self.command_processor)
            self.face_window.show()
        except Exception as e:
            print(f"Error showing face window: {e}")

    def show_calibration_window(self):
        try:
            self.calibration_window = Calibration(self.command_processor)
            self.calibration_window.show()
        except Exception as e:
            print(f"Error showing calibration window: {e}")

    # Paint event for drawing UI elements
    def paintEvent(self, event):
        try:
            painter = QPainter(self)
            painter.setRenderHint(QPainter.Antialiasing)
            
            # Draw movement area
            self._draw_movement_area(painter)
            
            # Draw attitude control area
            self._draw_attitude_control(painter)
            
            # Draw position control area
            self._draw_position_control(painter)
            
        except Exception as e:
            print(f"Paint error: {e}")

    def _draw_movement_area(self, painter):
        # Draw movement control circle
        painter.setPen(QPen(Qt.black, 2))
        painter.setBrush(Qt.white)
        painter.drawEllipse(
            UIConstants.MOVE_CENTER[0] - UIConstants.MOVE_RADIUS,
            UIConstants.MOVE_CENTER[1] - UIConstants.MOVE_RADIUS,
            UIConstants.MOVE_RADIUS * 2,
            UIConstants.MOVE_RADIUS * 2
        )
        
        # Draw center cross
        painter.setPen(QPen(Qt.red, 1))
        painter.drawLine(
            UIConstants.MOVE_CENTER[0] - 5, UIConstants.MOVE_CENTER[1],
            UIConstants.MOVE_CENTER[0] + 5, UIConstants.MOVE_CENTER[1]
        )
        painter.drawLine(
            UIConstants.MOVE_CENTER[0], UIConstants.MOVE_CENTER[1] - 5,
            UIConstants.MOVE_CENTER[0], UIConstants.MOVE_CENTER[1] + 5
        )
        
        # Draw current position
        if self.move_flag:
            painter.setPen(QPen(Qt.red, 1))
            painter.setBrush(Qt.red)
            painter.drawEllipse(
                self.move_point[0] - 5,
                self.move_point[1] - 5,
                10, 10
            )

    def _draw_attitude_control(self, painter):
        # Draw attitude control background
        painter.setPen(QPen(Qt.black, 2))
        painter.setBrush(Qt.white)
        painter.drawRect(700, 80, 200, 200)
        
        # Draw center cross
        painter.setPen(QPen(Qt.red, 1))
        center_x, center_y = UIConstants.ATTITUDE_CENTER
        painter.drawLine(center_x - 10, center_y, center_x + 10, center_y)
        painter.drawLine(center_x, center_y - 10, center_x, center_y + 10)
        
        # Draw current position
        painter.setPen(QPen(Qt.red, 1))
        painter.setBrush(Qt.red)
        painter.drawEllipse(
            self.drawpoint[0][0] - 5,
            self.drawpoint[0][1] - 5,
            10, 10
        )

    def _draw_position_control(self, painter):
        # Draw position control background
        painter.setPen(QPen(Qt.black, 2))
        painter.setBrush(Qt.white)
        painter.drawRect(700, 550, 200, 200)
        
        # Draw center cross
        painter.setPen(QPen(Qt.red, 1))
        center_x, center_y = UIConstants.POSITION_CENTER
        painter.drawLine(center_x - 10, center_y, center_x + 10, center_y)
        painter.drawLine(center_x, center_y - 10, center_x, center_y + 10)
        
        # Draw current position
        painter.setPen(QPen(Qt.red, 1))
        painter.setBrush(Qt.red)
        painter.drawEllipse(
            self.drawpoint[1][0] - 5,
            self.drawpoint[1][1] - 5,
            10, 10
        )

    def closeEvent(self, event: QCloseEvent) -> None:
        """Handle application close event safely."""
        try:
            self.timer.stop()
            self.timer_power.stop()
            self.timer_sonic.stop()
            self.connection_manager.disconnect_from_robot()
            event.accept()
        except Exception as e:
            print(f"Error during close: {e}")
            event.ignore()

# LED Control Window
class ledWindow(QMainWindow, Ui_led):
    def __init__(self, command_processor, parent=None):
        super().__init__(parent)
        self.setupUi(self)
        self.command_processor = command_processor
        self._setup_ui()
        self._connect_signals()

    def _setup_ui(self):
        self.setWindowIcon(QIcon('Picture/logo_Mini.png'))
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

    def _connect_signals(self):
        self.pushButton_Close.clicked.connect(self.close)
        self.pushButton_Color1.clicked.connect(lambda: self.set_color(1))
        self.pushButton_Color2.clicked.connect(lambda: self.set_color(2))
        self.pushButton_Color3.clicked.connect(lambda: self.set_color(3))
        self.pushButton_Color4.clicked.connect(lambda: self.set_color(4))
        self.pushButton_Color5.clicked.connect(lambda: self.set_color(5))
        self.pushButton_Color6.clicked.connect(lambda: self.set_color(6))
        self.pushButton_Color7.clicked.connect(lambda: self.set_color(7))
        self.pushButton_Color8.clicked.connect(lambda: self.set_color(8))
        self.pushButton_Color9.clicked.connect(lambda: self.set_color(9))
        self.pushButton_Color10.clicked.connect(lambda: self.set_color(10))
        self.pushButton_Color11.clicked.connect(lambda: self.set_color(11))
        self.pushButton_Color12.clicked.connect(lambda: self.set_color(12))
        self.pushButton_Color13.clicked.connect(lambda: self.set_color(13))
        self.pushButton_Color14.clicked.connect(lambda: self.set_color(14))
        self.pushButton_Color15.clicked.connect(lambda: self.set_color(15))
        self.pushButton_Color16.clicked.connect(lambda: self.set_color(16))

    def led_mode(self, index):
        if not index.isChecked():
            return
            
        mode = index.text().split()[-1]
        try:
            command = f"{cmd.CMD_LED_MOD}#{mode}\n"
            self.command_processor.send_raw_command(command)
        except Exception as e:
            print(f"LED mode error: {e}")

    def set_color(self, color_index: int):
        try:
            command = f"{cmd.CMD_LED}#{color_index}\n"
            self.command_processor.send_raw_command(command)
        except Exception as e:
            print(f"Set color error: {e}")

# Face Recognition Window
class faceWindow(QMainWindow, Ui_Face):
    def __init__(self, command_processor, parent=None):
        super().__init__(parent)
        self.setupUi(self)
        self.command_processor = command_processor
        self._setup_ui()

    def _setup_ui(self):
        self.setWindowIcon(QIcon('Picture/logo_Mini.png'))
        self.pushButton_Close.clicked.connect(self.close)
        self.pushButton_Save.clicked.connect(self.save_face)
        self.pushButton_Delete.clicked.connect(self.delete_face)
        self.pushButton_Refresh.clicked.connect(self.refresh_faces)

    def save_face(self):
        try:
            name = self.lineEdit_Name.text().strip()
            if not name:
                QMessageBox.warning(self, "Warning", "Please enter a name")
                return
                
            command = f"{cmd.CMD_FACE_SAVE}#{name}\n"
            self.command_processor.send_raw_command(command)
            QMessageBox.information(self, "Success", "Face saved successfully")
            self.refresh_faces()
        except Exception as e:
            QMessageBox.critical(self, "Error", f"Failed to save face: {e}")

    def delete_face(self):
        try:
            current_item = self.listWidget.currentItem()
            if not current_item:
                QMessageBox.warning(self, "Warning", "Please select a face to delete")
                return
                
            name = current_item.text()
            command = f"{cmd.CMD_FACE_DELETE}#{name}\n"
            self.command_processor.send_raw_command(command)
            self.refresh_faces()
            QMessageBox.information(self, "Success", "Face deleted successfully")
        except Exception as e:
            QMessageBox.critical(self, "Error", f"Failed to delete face: {e}")

    def refresh_faces(self):
        try:
            self.listWidget.clear()
            # This would be replaced with actual face list retrieval from the robot
            # For now, it's a placeholder
            command = f"{cmd.CMD_FACE_LIST}\n"
            self.command_processor.send_raw_command(command)
        except Exception as e:
            QMessageBox.critical(self, "Error", f"Failed to refresh faces: {e}")

# Main function
if __name__ == '__main__':
    app = QApplication(sys.argv)
    window = RobotControlWindow()
    window.show()
    sys.exit(app.exec_())