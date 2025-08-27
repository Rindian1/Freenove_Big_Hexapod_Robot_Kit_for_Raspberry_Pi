"""
Improved Client Module for Freenove Hexapod Robot

This module provides an enhanced client implementation for communicating with the
Hexapod Robot server, including video streaming and command handling.
"""

import io
import logging
import socket
import struct
import numpy as np
from typing import Optional, Tuple, Any, Union, List, Dict

import cv2
from PIL import Image, ImageDraw

# Configure logging
logger = logging.getLogger(__name__)

class HexapodClient:
    """
    Enhanced client for communicating with the Hexapod Robot server.
    
    Handles both command communication and video streaming with proper
    error handling and resource management.
    """
    
    def __init__(self):
        """Initialize the Hexapod client with default settings."""
        # Connection attributes
        self.command_socket: Optional[socket.socket] = None
        self.video_socket: Optional[socket.socket] = None
        self.video_connection: Optional[io.BufferedReader] = None
        self.tcp_flag: bool = False
        
        # Video attributes
        self.video_flag: bool = True
        self.image: Optional[np.ndarray] = None
        
        # Face recognition
        self.face_recognition_enabled: bool = False
        
        # Initialize PID controller (if needed)
        self.pid = None  # Will be initialized if needed
        
        logger.info("HexapodClient initialized")
    
    def connect(self, ip: str, command_port: int = 5002, video_port: int = 8002, 
               timeout: float = 5.0) -> bool:
        """
        Connect to the Hexapod server.
        
        Args:
            ip: Server IP address
            command_port: Port for command communication
            video_port: Port for video streaming
            timeout: Connection timeout in seconds
            
        Returns:
            bool: True if both connections were successful, False otherwise
        """
        try:
            # Connect command socket
            self.command_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
            self.command_socket.settimeout(timeout)
            self.command_socket.connect((ip, command_port))
            
            # Connect video socket
            self.video_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
            self.video_socket.settimeout(timeout)
            self.video_socket.connect((ip, video_port))
            self.video_connection = self.video_socket.makefile('rb')
            
            self.tcp_flag = True
            logger.info(f"Connected to Hexapod server at {ip}")
            return True
            
        except socket.error as e:
            logger.error(f"Failed to connect to Hexapod server: {e}")
            self.disconnect()
            return False
    
    def disconnect(self) -> None:
        """Safely disconnect from the Hexapod server."""
        self.tcp_flag = False
        
        # Close video connection
        if hasattr(self, 'video_connection') and self.video_connection:
            try:
                self.video_connection.close()
            except Exception as e:
                logger.warning(f"Error closing video connection: {e}")
        
        # Close sockets
        for sock in [self.video_socket, self.command_socket]:
            if sock:
                try:
                    sock.shutdown(socket.SHUT_RDWR)
                    sock.close()
                except Exception as e:
                    logger.warning(f"Error closing socket: {e}")
        
        logger.info("Disconnected from Hexapod server")
    
    def send_command(self, command: str, timeout: float = 1.0) -> bool:
        """
        Send a command to the Hexapod server.
        
        Args:
            command: Command string to send
            timeout: Timeout for the operation in seconds
            
        Returns:
            bool: True if the command was sent successfully, False otherwise
        """
        if not self.tcp_flag or not self.command_socket:
            logger.warning("Not connected to server")
            return False
        
        try:
            if not command.endswith('\n'):
                command += '\n'
            self.command_socket.sendall(command.encode('utf-8'))
            return True
        except Exception as e:
            logger.error(f"Failed to send command: {e}")
            self.tcp_flag = False
            return False
    
    def receive_response(self, buffer_size: int = 1024, timeout: float = 1.0) -> Optional[str]:
        """
        Receive a response from the Hexapod server.
        
        Args:
            buffer_size: Maximum number of bytes to receive
            timeout: Timeout for the operation in seconds
            
        Returns:
            str: Received data or None if an error occurred
        """
        if not self.tcp_flag or not self.command_socket:
            return None
        
        try:
            self.command_socket.settimeout(timeout)
            data = self.command_socket.recv(buffer_size).decode('utf-8')
            return data.strip() if data else None
        except socket.timeout:
            logger.warning("Receive operation timed out")
            return None
        except Exception as e:
            logger.error(f"Error receiving data: {e}")
            self.tcp_flag = False
            return None
    
    def get_video_frame(self) -> Optional[np.ndarray]:
        """
        Get the next video frame from the server.
        
        Returns:
            Optional[np.ndarray]: The decoded image frame or None if an error occurred
        """
        if not self.tcp_flag or not self.video_connection:
            return None
        
        try:
            # Read the length of the image as a 4-byte unsigned long
            length_bytes = self.video_connection.read(4)
            if not length_bytes:
                return None
                
            # Convert the length to an integer
            length = struct.unpack('<L', length_bytes)[0]
            
            # Read the image data
            jpg_data = self.video_connection.read(length)
            
            # Verify the image data is valid
            if not self._is_valid_jpeg(jpg_data):
                logger.warning("Received invalid JPEG data")
                return None
            
            # Decode the image
            self.image = cv2.imdecode(
                np.frombuffer(jpg_data, dtype=np.uint8), 
                cv2.IMREAD_COLOR
            )
            
            return self.image
            
        except Exception as e:
            logger.error(f"Error receiving video frame: {e}")
            self.tcp_flag = False
            return None
    
    def _is_valid_jpeg(self, data: bytes) -> bool:
        """
        Check if the given data is a valid JPEG image.
        
        Args:
            data: Image data to validate
            
        Returns:
            bool: True if the data is a valid JPEG, False otherwise
        """
        try:
            # Check for JPEG magic numbers
            if data[6:10] in (b'JFIF', b'Exif'):
                return data.rstrip(b'\0\r\n').endswith(b'\xff\xd9')
            
            # Try to open with PIL
            Image.open(io.BytesIO(data)).verify()
            return True
            
        except Exception:
            return False
    
    def __enter__(self):
        """Context manager entry."""
        return self
    
    def __exit__(self, exc_type, exc_val, exc_tb):
        """Context manager exit - ensure resources are cleaned up."""
        self.disconnect()


if __name__ == '__main__':
    # Example usage
    import time
    
    # Configure logging
    logging.basicConfig(
        level=logging.INFO,
        format='%(asctime)s - %(name)s - %(levelname)s - %(message)s'
    )
    
    # Create and use the client
    with HexapodClient() as client:
        if client.connect('192.168.1.1'):  # Replace with actual IP
            print("Connected successfully!")
            
            # Example: Send a command
            if client.send_command("CMD_POWER"):
                response = client.receive_response()
                print(f"Power status: {response}")
            
            # Example: Get a video frame
            frame = client.get_video_frame()
            if frame is not None:
                print(f"Received frame with shape: {frame.shape}")
        else:
            print("Failed to connect to the Hexapod server")
