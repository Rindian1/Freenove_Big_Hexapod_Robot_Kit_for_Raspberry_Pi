# -*- coding: utf-8 -*-
"""
Client module for Freenove Big Hexapod Robot Kit

This module handles communication between the client (PC) and server (Raspberry Pi)
components of the hexapod robot system. It manages video streaming, command
sending, and face recognition functionality.
"""

import io
import math
import copy
import socket
import struct
import threading
import cv2
import numpy as np
import time
from PIL import Image, ImageDraw
from Thread import stop_thread
from PID import Incremental_PID
from Face import Face
from Command import COMMAND as cmd


class Client:
    """
    Client class for hexapod robot communication and control.
    
    Handles TCP/IP communication with the robot server, video streaming,
    command sending, and face recognition processing.
    """
    
    def __init__(self):
        """Initialize the Client with default settings and components."""
        # Initialize components
        self.face = Face()
        self.pid = Incremental_PID(1, 0, 0.0025)
        
        # Communication flags
        self.tcp_flag = False
        self.video_flag = True
        self.connection_established = False
        
        # Face recognition flags
        self.face_id_enabled = False
        self.face_recognition_enabled = False
        
        # Video and image data
        self.image = None
        self.video_thread = None
        self.command_thread = None
        
        # Movement settings
        self.move_speed = "8"
        self.last_command_time = time.time()
        
        # Connection settings
        self.video_socket = None
        self.command_socket = None
        self.video_connection = None
        self.command_connection = None
        
        # Error handling
        self.connection_retries = 3
        self.retry_delay = 2.0
        
    def turn_on_client(self, ip_address):
        """
        Initialize client sockets for connection to robot server.
        
        Args:
            ip_address (str): IP address of the robot server
        """
        try:
            # Create sockets for video and command communication
            self.video_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
            self.command_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
            
            # Set socket options for better performance
            self.video_socket.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
            self.command_socket.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
            
            # Set timeout for connections
            self.video_socket.settimeout(10.0)
            self.command_socket.settimeout(10.0)
            
            print(f"Client sockets initialized for IP: {ip_address}")
            return True
            
        except Exception as e:
            print(f"Error initializing client sockets: {e}")
            return False
    
    def turn_off_client(self):
        """Close all client connections and clean up resources."""
        try:
            # Set flag to stop threads
            self.tcp_flag = False
            
            # Close video connection
            if hasattr(self, 'video_connection') and self.video_connection:
                self.video_connection.close()
                
            # Close command connection
            if hasattr(self, 'command_connection') and self.command_connection:
                self.command_connection.close()
                
            # Close sockets
            if hasattr(self, 'video_socket') and self.video_socket:
                self.video_socket.close()
                
            if hasattr(self, 'command_socket') and self.command_socket:
                self.command_socket.close()
                
            # Stop threads
            if self.video_thread:
                stop_thread(self.video_thread)
            if self.command_thread:
                stop_thread(self.command_thread)
                
            self.connection_established = False
            print("Client connections closed successfully")
            
        except Exception as e:
            print(f"Error closing client connections: {e}")
    
    def connect_to_server(self, ip_address):
        """
        Establish connection to the robot server.
        
        Args:
            ip_address (str): IP address of the robot server
            
        Returns:
            bool: True if connection successful, False otherwise
        """
        try:
            # Connect to video stream port (8002)
            self.video_socket.connect((ip_address, 8002))
            self.video_connection = self.video_socket.makefile('rb')
            
            # Connect to command port (5002)
            self.command_socket.connect((ip_address, 5002))
            self.command_connection = self.command_socket
            
            self.tcp_flag = True
            self.connection_established = True
            print(f"Successfully connected to robot server at {ip_address}")
            return True
            
        except Exception as e:
            print(f"Failed to connect to server: {e}")
            self.connection_established = False
            return False
    
    def is_valid_image_4_bytes(self, buf):
        """
        Validate if a buffer contains a valid JPEG image.
        
        Args:
            buf (bytes): Image buffer to validate
            
        Returns:
            bool: True if valid JPEG image, False otherwise
        """
        try:
            # Check for JPEG file signature
            if buf[6:10] in (b'JFIF', b'Exif'):
                # Verify JPEG end marker
                if not buf.rstrip(b'\0\r\n').endswith(b'\xff\xd9'):
                    return False
            else:
                # Try to verify with PIL
                try:
                    Image.open(io.BytesIO(buf)).verify()
                except:
                    return False
            return True
            
        except Exception:
            return False
    
    def receiving_video(self, ip_address):
        """
        Receive and process video stream from the robot.
        
        Args:
            ip_address (str): IP address of the robot server
        """
        try:
            # Connect to video stream
            if not self.connect_to_server(ip_address):
                print("Failed to establish video connection")
                return
                
            print("Video stream connection established")
            
            while self.tcp_flag:
                try:
                    # Read frame length (4 bytes)
                    stream_bytes = self.video_connection.read(4)
                    if not stream_bytes:
                        break
                        
                    # Unpack frame length
                    frame_length = struct.unpack('<L', stream_bytes[:4])[0]
                    
                    # Read frame data
                    jpg_data = self.video_connection.read(frame_length)
                    
                    # Validate image data
                    if self.is_valid_image_4_bytes(jpg_data):
                        if self.video_flag:
                            # Decode image
                            self.image = cv2.imdecode(
                                np.frombuffer(jpg_data, dtype=np.uint8), 
                                cv2.IMREAD_COLOR
                            )
                            
                            # Process face recognition if enabled
                            if (not self.face_id_enabled and 
                                self.face_recognition_enabled and 
                                self.image is not None):
                                self.face.face_detect(self.image)
                                
                            self.video_flag = False
                            
                except Exception as e:
                    print(f"Error processing video frame: {e}")
                    break
                    
        except Exception as e:
            print(f"Video stream error: {e}")
        finally:
            print("Video stream connection closed")
    
    def send_data(self, data):
        """
        Send data to the robot server.
        
        Args:
            data (str): Data to send
        """
        if self.tcp_flag and self.command_connection:
            try:
                self.command_connection.send(data.encode('utf-8'))
                self.last_command_time = time.time()
            except Exception as e:
                print(f"Error sending data: {e}")
                self.tcp_flag = False
    
    def receive_data(self):
        """
        Receive data from the robot server.
        
        Returns:
            str: Received data or empty string if error
        """
        try:
            if self.tcp_flag and self.command_connection:
                data = self.command_connection.recv(1024).decode('utf-8')
                return data
        except Exception as e:
            print(f"Error receiving data: {e}")
            self.tcp_flag = False
        return ""
    
    def send_command(self, command, *parameters):
        """
        Send a formatted command to the robot.
        
        Args:
            command (str): Command type
            *parameters: Command parameters
        """
        formatted_command = cmd.format_command(command, *parameters)
        self.send_data(formatted_command)
    
    def get_robot_status(self):
        """
        Get current robot status including battery and sensors.
        
        Returns:
            dict: Robot status information
        """
        status = {
            'connected': self.tcp_flag,
            'battery_voltage': None,
            'distance': None,
            'last_command_time': self.last_command_time
        }
        
        # Request battery status
        self.send_command(cmd.CMD_POWER)
        time.sleep(0.1)
        
        # Request distance measurement
        self.send_command(cmd.CMD_SONIC)
        time.sleep(0.1)
        
        return status
    
    def enable_face_recognition(self, enable=True):
        """
        Enable or disable face recognition processing.
        
        Args:
            enable (bool): True to enable, False to disable
        """
        self.face_recognition_enabled = enable
        print(f"Face recognition {'enabled' if enable else 'disabled'}")
    
    def enable_face_id(self, enable=True):
        """
        Enable or disable face ID training mode.
        
        Args:
            enable (bool): True to enable, False to disable
        """
        self.face_id_enabled = enable
        print(f"Face ID training {'enabled' if enable else 'disabled'}")
    
    def get_current_image(self):
        """
        Get the most recent video frame.
        
        Returns:
            numpy.ndarray: Current image or None if not available
        """
        return self.image.copy() if self.image is not None else None
    
    def is_connected(self):
        """
        Check if client is connected to robot server.
        
        Returns:
            bool: True if connected, False otherwise
        """
        return self.tcp_flag and self.connection_established
    
    def get_connection_info(self):
        """
        Get connection information.
        
        Returns:
            dict: Connection status and information
        """
        return {
            'tcp_flag': self.tcp_flag,
            'connection_established': self.connection_established,
            'video_flag': self.video_flag,
            'face_recognition_enabled': self.face_recognition_enabled,
            'face_id_enabled': self.face_id_enabled
        }


def test_client():
    """Test function to demonstrate client functionality."""
    print("Testing Hexapod Robot Client...")
    
    client = Client()
    
    # Test socket initialization
    if client.turn_on_client("192.168.1.100"):
        print("✓ Socket initialization successful")
    else:
        print("✗ Socket initialization failed")
        return
    
    # Test command formatting
    try:
        client.send_command(cmd.CMD_SONIC)
        print("✓ Command sending test passed")
    except Exception as e:
        print(f"✗ Command sending test failed: {e}")
    
    # Clean up
    client.turn_off_client()
    print("✓ Client cleanup successful")


if __name__ == '__main__':
    print("Hexapod Robot Client Module")
    print("=" * 40)
    print("This module is meant to be imported by Main.py")
    print("To run the hexapod robot client, use: python Main.py")
    print()
    
    # Run test if executed directly
    test_client()
