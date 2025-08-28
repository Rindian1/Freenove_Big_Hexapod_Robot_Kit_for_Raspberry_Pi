# -*- coding: utf-8 -*-
"""Client module for hexapod robot communication.

This module handles all network communication between the client and server,
including video streaming and command transmission.
"""

import io
import logging
import socket
import struct
import threading
from typing import Optional, Tuple

import cv2
import numpy as np
from PIL import Image

# Local imports
from src.core.command import COMMAND as cmd
from src.models.face import Face
from src.models.pid import Incremental_PID
from src.core.thread import *


class Client:
    """Handles network communication for the hexapod robot client.
    
    This class manages socket connections, video streaming, and data transmission
    between the client and server.
    """
    
    # Constants
    DEFAULT_VIDEO_PORT = 8002
    DEFAULT_COMMAND_PORT = 5002
    SOCKET_TIMEOUT = 5.0
    
    def __init__(self) -> None:
        """Initialize the client with default settings."""
        self.face = Face()
        self.pid = Incremental_PID(1, 0, 0.0025)
        self.tcp_flag = False
        self.video_flag = True
        self.face_id = False
        self.face_recognition_flag = False
        self.image = ''
        self._client_socket = None
        self._command_socket = None
        self._connection = None
        self._lock = threading.Lock()
        self._logger = self._setup_logger()
    
    def turn_on_client(self, ip: str) -> None:
        """Initialize client sockets.
        
        Args:
            ip: Server IP address
        """
        with self._lock:
            try:
                self._client_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
                self._command_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
                self._client_socket.settimeout(self.SOCKET_TIMEOUT)
                self._command_socket.settimeout(self.SOCKET_TIMEOUT)
                self.tcp_flag = True
                self._logger.info(f"Connected to server at {ip}")
            except socket.error as e:
                self._logger.error(f"Failed to create sockets: {e}")
                self.tcp_flag = False
                raise
    
    def turn_off_client(self) -> None:
        """Safely close all socket connections."""
        with self._lock:
            self.tcp_flag = False
            for sock in [self._client_socket, self._command_socket, self._connection]:
                if sock:
                    try:
                        sock.shutdown(socket.SHUT_RDWR)
                        sock.close()
                    except (OSError, AttributeError) as e:
                        self._logger.debug(f"Error closing socket: {e}")
                    finally:
                        sock = None
    
    def receiving_video(self, ip: str) -> None:
        """Handle video streaming from server.
        
        Args:
            ip: Server IP address
        """
        try:
            if not self.tcp_flag:
                self.turn_on_client(ip)
                
            self._client_socket.connect((ip, self.DEFAULT_VIDEO_PORT))
            self._connection = self._client_socket.makefile('rb')
            
            while True:
                try:
                    stream_bytes = self._connection.read(4)
                    if not stream_bytes:
                        break
                        
                    length = struct.unpack('<L', stream_bytes[:4])[0]
                    jpg = self._connection.read(length)
                    
                    if self._is_valid_image(jpg) and self.video_flag:
                        self._process_video_frame(jpg)
                        
                except (struct.error, ConnectionError) as e:
                    self._logger.error(f"Video stream error: {e}")
                    break
                    
        except Exception as e:
            self._logger.error(f"Video streaming failed: {e}")
            self.tcp_flag = False
        finally:
            self.turn_off_client()
    
    def _is_valid_image(self, buf: bytes) -> bool:
        """Validate image data.
        
        Args:
            buf: Binary image data
            
        Returns:
            bool: True if valid image, False otherwise
        """
        if buf[6:10] in (b'JFIF', b'Exif'):
            return buf.rstrip(b'\0\r\n').endswith(b'\xff\xd9')
        try:
            Image.open(io.BytesIO(buf)).verify()
            return True
        except Exception:
            return False
    
    def _process_video_frame(self, jpg: bytes) -> None:
        """Process a single video frame.
        
        Args:
            jpg: JPEG image data
        """
        self.image = cv2.imdecode(np.frombuffer(jpg, dtype=np.uint8), cv2.IMREAD_COLOR)
        if not self.face_id and self.face_recognition_flag:
            self.face.face_detect(self.image)
        self.video_flag = False
    
    def send_data(self, data: str) -> None:
        """Send data to the server.
        
        Args:
            data: Data to send
        """
        if not self.tcp_flag or not self._command_socket:
            self._logger.warning("Cannot send data: Not connected to server")
            return
            
        try:
            self._command_socket.sendall(data.encode('utf-8'))
        except (OSError, AttributeError) as e:
            self._logger.error(f"Failed to send data: {e}")
            self.tcp_flag = False
    
    def receive_data(self) -> str:
        """Receive data from the server.
        
        Returns:
            str: Received data or empty string if error
        """
        if not self.tcp_flag or not self._command_socket:
            return ""
            
        try:
            data = self._command_socket.recv(1024).decode('utf-8')
            return data
        except (OSError, AttributeError) as e:
            self._logger.error(f"Failed to receive data: {e}")
            self.tcp_flag = False
            return ""
    
    def _setup_logger(self) -> logging.Logger:
        """Configure and return a logger instance.
        
        Returns:
            logging.Logger: Configured logger instance
        """
        logger = logging.getLogger(__name__)
        logger.setLevel(logging.INFO)
        handler = logging.StreamHandler()
        formatter = logging.Formatter('%(asctime)s - %(name)s - %(levelname)s - %(message)s')
        handler.setFormatter(formatter)
        logger.addHandler(handler)
        return logger


if __name__ == '__main__':
    import logging
    logging.basicConfig(level=logging.INFO)
    
    try:
        client = Client()
        # Example usage
        client.turn_on_client("192.168.1.100")
    except KeyboardInterrupt:
        print("\nShutting down...")
    except Exception as e:
        logging.error(f"Fatal error: {e}")
    finally:
        if 'client' in locals():
            client.turn_off_client()