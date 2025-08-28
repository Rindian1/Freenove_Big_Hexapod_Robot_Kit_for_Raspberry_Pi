"""Network manager for the Hexapod Robot client.

This module provides the NetworkManager class which handles all network communication
with the Hexapod Robot, including connection management, command sending, and video streaming.
"""

import socket
import time
import threading
import logging
from enum import Enum, auto
from typing import Optional, Callable, Any, Dict, Tuple, Union

from .exceptions import (
    RobotError,
    ConnectionError,
    NetworkError,
    TimeoutError,
    InvalidStateError
)
from .thread_safe import ThreadSafeValue, ThreadSafeCounter
from .logging_config import get_logger
from .utils import retry, handle_errors, log_duration

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
    
    def _stop_threads(self) -> None:
        """Stop and clean up background threads."""
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
                    self._process_instruction(data)
                    
                except socket.timeout:
                    continue
                except (socket.error, ConnectionError) as e:
                    if not self._stop_event.is_set():
                        self._handle_connection_error(f"Instruction thread error: {e}")
                    break
                    
            except Exception as e:
                logger.error(f"Unexpected error in instruction thread: {e}", exc_info=True)
                if not self._stop_event.is_set():
                    time.sleep(0.1)  # Prevent tight loop on errors
    
    def _process_video_frame(self, frame_data: bytes) -> None:
        """Process a received video frame.
        
        Args:
            frame_data: Raw video frame data
        """
        try:
            if hasattr(self.client, 'process_video_frame'):
                self.client.process_video_frame(frame_data)
        except Exception as e:
            logger.error(f"Error processing video frame: {e}", exc_info=True)
    
    def _process_instruction(self, data: bytes) -> None:
        """Process received instruction data.
        
        Args:
            data: Raw instruction data
        """
        try:
            if hasattr(self.client, 'process_instruction'):
                self.client.process_instruction(data)
        except Exception as e:
            logger.error(f"Error processing instruction: {e}", exc_info=True)
    
    def _handle_connection_error(self, error_msg: str) -> None:
        """Handle a connection-related error.
        
        Args:
            error_msg: Error message
        """
        logger.error(error_msg)
        self._set_state(ConnectionState.ERROR)
        self.disconnect()
    
    def send_command(self, command: str, timeout: float = 1.0) -> str:
        """Send a command to the robot and wait for a response.
        
        Args:
            command: Command string to send
            timeout: Maximum time to wait for a response in seconds
            
        Returns:
            str: Response from the robot
            
        Raises:
            TimeoutError: If no response is received within the timeout
            ConnectionError: If not connected or connection is lost
        """
        if self.state != ConnectionState.CONNECTED or not self.instruction_socket:
            raise ConnectionError("Not connected to robot")
        
        try:
            # Send command
            self.instruction_socket.sendall(f"{command}\n".encode('utf-8'))
            
            # Wait for response with timeout
            self.instruction_socket.settimeout(timeout)
            response = self.instruction_socket.recv(4096).decode('utf-8').strip()
            
            # Reset timeout to default
            self.instruction_socket.settimeout(self.SOCKET_TIMEOUT)
            
            if not response:
                raise ConnectionError("Empty response from robot")
                
            return response
            
        except socket.timeout as e:
            raise TimeoutError(f"No response received within {timeout} seconds") from e
        except (socket.error, OSError) as e:
            self._handle_connection_error(f"Error sending command: {e}")
            raise ConnectionError(f"Failed to send command: {e}") from e
