"""Enhanced Client class for hexapod robot communication.

This module provides an improved version of the Client class that integrates with
the new NetworkManager while maintaining backward compatibility with existing code.
"""

import io
import logging
import socket
import struct
import threading
from typing import Optional, Tuple, Any, Callable

import cv2
import numpy as np
from PIL import Image

# Local imports
from .Command import COMMAND as cmd
from .Face import Face
from .PID import Incremental_PID
from .network_manager import NetworkManager, ConnectionState
from .exceptions import ConnectionError, NetworkError, TimeoutError
from .logging_config import get_logger


class Client:
    """Handles network communication for the hexapod robot client.
    
    This enhanced version integrates with NetworkManager for better connection
    handling while maintaining backward compatibility with the original API.
    """
    
    # Constants
    DEFAULT_VIDEO_PORT = 8002
    DEFAULT_COMMAND_PORT = 5002
    SOCKET_TIMEOUT = 5.0
    
    def __init__(self, use_network_manager: bool = True) -> None:
        """Initialize the client with default settings.
        
        Args:
            use_network_manager: Whether to use the new NetworkManager (default: True)
        """
        self.face = Face()
        self.pid = Incremental_PID(1, 0, 0.0025)
        self.tcp_flag = False
        self.video_flag = True
        self.face_id = False
        self.face_recognition_flag = False
        self.image = ''
        
        # Legacy socket attributes (for backward compatibility)
        self._client_socket = None
        self._command_socket = None
        self._connection = None
        self._lock = threading.RLock()
        
        # New NetworkManager integration
        self._use_network_manager = use_network_manager
        self._network_manager = NetworkManager(self) if use_network_manager else None
        self._logger = get_logger(__name__)
        
        # Video processing thread
        self._video_thread = None
        self._stop_video_thread = threading.Event()
    
    # Legacy methods for backward compatibility
    def turn_on_client(self, ip: str) -> None:
        """Initialize client connection (legacy method).
        
        Args:
            ip: Server IP address
        """
        if self._use_network_manager and self._network_manager:
            try:
                self._network_manager.connect(ip, self.DEFAULT_COMMAND_PORT, self.DEFAULT_VIDEO_PORT)
                self.tcp_flag = True
            except (ConnectionError, TimeoutError) as e:
                self._logger.error(f"Connection failed: {e}")
                self.tcp_flag = False
                raise
        else:
            # Fall back to legacy implementation
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
        """Safely close all connections (legacy method)."""
        if self._use_network_manager and self._network_manager:
            self._network_manager.disconnect()
            self.tcp_flag = False
        else:
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
        """Start video streaming (legacy method)."""
        if self._use_network_manager and self._network_manager:
            if not self._video_thread or not self._video_thread.is_alive():
                self._stop_video_thread.clear()
                self._video_thread = threading.Thread(
                    target=self._video_stream_worker,
                    args=(ip,),
                    daemon=True
                )
                self._video_thread.start()
        else:
            # Fall back to legacy implementation
            self._legacy_receiving_video(ip)
    
    def _video_stream_worker(self, ip: str) -> None:
        """Worker thread for handling video streaming with NetworkManager."""
        try:
            if not self._network_manager or not self._network_manager.connected:
                self.turn_on_client(ip)
                
            while not self._stop_video_thread.is_set():
                try:
                    # Process video frames if available
                    if hasattr(self, 'process_video_frame'):
                        frame = self._network_manager.get_video_frame()
                        if frame is not None:
                            self.process_video_frame(frame)
                    time.sleep(0.03)  # ~30 FPS
                except Exception as e:
                    self._logger.error(f"Video processing error: {e}")
                    time.sleep(1)  # Prevent tight loop on errors
        except Exception as e:
            self._logger.error(f"Video streaming failed: {e}")
        finally:
            self.turn_off_client()
    
    def _legacy_receiving_video(self, ip: str) -> None:
        """Legacy video streaming implementation."""
        try:
            if not self.tcp_flag:
                self.turn_on_client(ip)
                
            self._client_socket.connect((ip, self.DEFAULT_VIDEO_PORT))
            self._connection = self._client_socket.makefile('rb')
            
            while not self._stop_video_thread.is_set():
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
    
    def send_data(self, data: str) -> None:
        """Send data to the server.
        
        Args:
            data: Data to send
        """
        if self._use_network_manager and self._network_manager:
            try:
                self._network_manager.send_command(data)
            except (ConnectionError, TimeoutError) as e:
                self._logger.error(f"Failed to send data: {e}")
                self.tcp_flag = False
        else:
            if not self.tcp_flag or not self._command_socket:
                self._logger.warning("Cannot send data: Not connected to server")
                return
                
            try:
                self._command_socket.sendall(data.encode('utf-8'))
            except (OSError, AttributeError) as e:
                self._logger.error(f"Failed to send data: {e}")
                self.tcp_flag = False
    
    def receive_data(self, timeout: float = None) -> str:
        """Receive data from the server.
        
        Args:
            timeout: Maximum time to wait for data (in seconds)
            
        Returns:
            str: Received data or empty string if error
        """
        if self._use_network_manager and self._network_manager:
            try:
                return self._network_manager.receive_command(timeout or self.SOCKET_TIMEOUT)
            except (ConnectionError, TimeoutError) as e:
                self._logger.error(f"Failed to receive data: {e}")
                self.tcp_flag = False
                return ""
        else:
            if not self.tcp_flag or not self._command_socket:
                return ""
                
            try:
                if timeout is not None:
                    self._command_socket.settimeout(timeout)
                data = self._command_socket.recv(1024).decode('utf-8')
                return data
            except (OSError, AttributeError) as e:
                self._logger.error(f"Failed to receive data: {e}")
                self.tcp_flag = False
                return ""
    
    # Helper methods
    def _is_valid_image(self, buf: bytes) -> bool:
        """Validate image data.
        
        Args:
            buf: Binary image data
            
        Returns:
            bool: True if valid image, False otherwise
        """
        if not buf:
            return False
            
        try:
            if buf[6:10] in (b'JFIF', b'Exif'):
                return buf.rstrip(b'\0\r\n').endswith(b'\xff\xd9')
            Image.open(io.BytesIO(buf)).verify()
            return True
        except Exception as e:
            self._logger.debug(f"Invalid image data: {e}")
            return False
    
    def _process_video_frame(self, jpg: bytes) -> None:
        """Process a single video frame.
        
        Args:
            jpg: JPEG image data
        """
        try:
            self.image = cv2.imdecode(np.frombuffer(jpg, dtype=np.uint8), cv2.IMREAD_COLOR)
            if not self.face_id and self.face_recognition_flag:
                self.face.face_detect(self.image)
            self.video_flag = False
        except Exception as e:
            self._logger.error(f"Error processing video frame: {e}")
    
    # New methods for NetworkManager integration
    def on_connection_state_changed(self, old_state: ConnectionState, new_state: ConnectionState) -> None:
        """Handle connection state changes.
        
        Args:
            old_state: Previous connection state
            new_state: New connection state
        """
        self.tcp_flag = (new_state == ConnectionState.CONNECTED)
        self._logger.info(f"Connection state changed from {old_state.name} to {new_state.name}")
    
    def process_instruction(self, data: bytes) -> None:
        """Process received instruction data.
        
        Args:
            data: Raw instruction data
        """
        try:
            # Process instruction data here
            # This is a placeholder - implement based on your protocol
            self._logger.debug(f"Received instruction: {data}")
        except Exception as e:
            self._logger.error(f"Error processing instruction: {e}")
    
    def __del__(self) -> None:
        """Clean up resources."""
        self._stop_video_thread.set()
        if self._video_thread and self._video_thread.is_alive():
            self._video_thread.join(timeout=2.0)
        self.turn_off_client()


# Maintain backward compatibility
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
