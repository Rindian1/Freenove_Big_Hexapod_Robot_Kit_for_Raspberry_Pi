"""Enhanced NetworkManager for Hexapod Robot."""

import time
import threading
from typing import Optional, Callable, Any, Dict, Type

from .interfaces import IConnection, ConnectionState
from .exceptions import ConnectionError, TimeoutError
from .connections import SocketConnection
from .video import VideoStream
from .commands import CommandProcessor
from .thread_safe import ThreadSafeValue
from .logging_config import get_logger

logger = get_logger(__name__)

class NetworkManager:
    """Manages network connections and communication with the Hexapod Robot."""
    
    def __init__(
        self,
        connection_class: Type[IConnection] = SocketConnection,
        reconnect_attempts: int = 3,
        reconnect_delay: float = 1.0,
    ):
        """Initialize the NetworkManager.
        
        Args:
            connection_class: The connection class to use
            reconnect_attempts: Number of reconnection attempts
            reconnect_delay: Delay between reconnection attempts in seconds
        """
        self._connection_class = connection_class
        self._reconnect_attempts = reconnect_attempts
        self._reconnect_delay = reconnect_delay
        
        # Connection state
        self._state = ThreadSafeValue(ConnectionState.DISCONNECTED, name="network_manager_state")
        self._lock = threading.RLock()
        
        # Connections
        self._control_conn: Optional[IConnection] = None
        self._video_conn: Optional[IConnection] = None
        self._video_stream: Optional[VideoStream] = None
        self._command_processor: Optional[CommandProcessor] = None
        
        # Configuration
        self._host: Optional[str] = None
        self._control_port: int = 5002
        self._video_port: int = 8002
        
        # Thread management
        self._reconnect_thread: Optional[threading.Thread] = None
        self._stop_event = threading.Event()
    
    @property
    def state(self) -> ConnectionState:
        """Get the current connection state."""
        return self._state.value
    
    @state.setter
    def state(self, value: ConnectionState) -> None:
        """Set the connection state."""
        with self._lock:
            old_state = self._state.value
            self._state.value = value
            if old_state != value:
                logger.info(f"Connection state changed from {old_state.name} to {value.name}")
    
    @property
    def is_connected(self) -> bool:
        """Check if the network manager is connected."""
        return self.state == ConnectionState.CONNECTED
    
    @property
    def video_stream(self) -> Optional[VideoStream]:
        """Get the video stream instance."""
        return self._video_stream
    
    @property
    def command_processor(self) -> Optional[CommandProcessor]:
        """Get the command processor instance."""
        return self._command_processor
    
    def connect(self, host: str, control_port: int = 5002, video_port: int = 8002) -> None:
        """Connect to the robot.
        
        Args:
            host: The host to connect to
            control_port: The control port (default: 5002)
            video_port: The video port (default: 8002)
            
        Raises:
            ConnectionError: If connection fails after all retries
        """
        with self._lock:
            if self.state == ConnectionState.CONNECTED:
                logger.warning("Already connected")
                return
                
            self._host = host
            self._control_port = control_port
            self._video_port = video_port
            
            # Stop any existing connections
            self._stop_reconnect_thread()
            self._disconnect()
            
            # Start connection process
            self.state = ConnectionState.CONNECTING
            self._stop_event.clear()
            
            # Start connection in a separate thread to avoid blocking
            self._reconnect_thread = threading.Thread(
                target=self._connect_loop,
                name="NetworkManagerReconnectThread",
                daemon=True
            )
            self._reconnect_thread.start()
    
    def _connect_loop(self) -> None:
        """Main connection loop with retries."""
        attempt = 0
        
        while not self._stop_event.is_set() and attempt < self._reconnect_attempts:
            try:
                if attempt > 0:
                    logger.info(f"Reconnection attempt {attempt + 1}/{self._reconnect_attempts}")
                
                # Create and connect control connection
                control_conn = self._connection_class()
                control_conn.connect(self._host, self._control_port)
                
                # Create and connect video connection
                video_conn = self._connection_class()
                video_conn.connect(self._host, self._video_port)
                
                # Create command processor and video stream
                command_processor = CommandProcessor(control_conn)
                video_stream = VideoStream(video_conn)
                
                # Start components
                command_processor.start()
                video_stream.start()
                
                # Update state
                with self._lock:
                    self._control_conn = control_conn
                    self._video_conn = video_conn
                    self._command_processor = command_processor
                    self._video_stream = video_stream
                    self.state = ConnectionState.CONNECTED
                
                logger.info(f"Connected to {self._host}:{self._control_port} (control) and {self._video_port} (video)")
                return  # Success
                
            except ConnectionError as e:
                attempt += 1
                logger.error(f"Connection attempt {attempt} failed: {e}")
                
                # Clean up any partial connections
                self._cleanup_connections()
                
                if attempt >= self._reconnect_attempts:
                    logger.error(f"Failed to connect after {attempt} attempts")
                    self.state = ConnectionState.DISCONNECTED
                    break
                    
                # Wait before retrying
                time.sleep(self._reconnect_delay * (2 ** (attempt - 1)))  # Exponential backoff
                
            except Exception as e:
                logger.critical(f"Unexpected error during connection: {e}", exc_info=True)
                self.state = ConnectionState.DISCONNECTED
                self._cleanup_connections()
                break
    
    def disconnect(self) -> None:
        """Disconnect from the robot."""
        with self._lock:
            self._stop_reconnect_thread()
            self._disconnect()
    
    def _disconnect(self) -> None:
        """Internal disconnect implementation."""
        if self.state == ConnectionState.DISCONNECTED:
            return
            
        self.state = ConnectionState.DISCONNECTING
        
        # Stop video stream
        if self._video_stream:
            try:
                self._video_stream.stop()
            except Exception as e:
                logger.error(f"Error stopping video stream: {e}")
            self._video_stream = None
        
        # Stop command processor
        if self._command_processor:
            try:
                self._command_processor.stop()
            except Exception as e:
                logger.error(f"Error stopping command processor: {e}")
            self._command_processor = None
        
        # Close connections
        for conn in [self._control_conn, self._video_conn]:
            if conn:
                try:
                    conn.disconnect()
                    conn.close()
                except Exception as e:
                    logger.error(f"Error closing connection: {e}")
        
        self._control_conn = None
        self._video_conn = None
        self.state = ConnectionState.DISCONNECTED
        logger.info("Disconnected from robot")
    
    def _cleanup_connections(self) -> None:
        """Clean up any existing connections."""
        with self._lock:
            if self._video_stream:
                try:
                    self._video_stream.stop()
                except Exception:
                    pass
                self._video_stream = None
            
            if self._command_processor:
                try:
                    self._command_processor.stop()
                except Exception:
                    pass
                self._command_processor = None
            
            for conn in [self._control_conn, self._video_conn]:
                if conn:
                    try:
                        conn.disconnect()
                        conn.close()
                    except Exception:
                        pass
            
            self._control_conn = None
            self._video_conn = None
    
    def _stop_reconnect_thread(self) -> None:
        """Stop the reconnection thread if it's running."""
        if self._reconnect_thread and self._reconnect_thread.is_alive():
            self._stop_event.set()
            self._reconnect_thread.join(timeout=2.0)
            if self._reconnect_thread.is_alive():
                logger.warning("Reconnection thread did not stop gracefully")
        self._reconnect_thread = None
    
    def send_command(self, command: str, timeout: Optional[float] = None) -> Any:
        """Send a command to the robot.
        
        Args:
            command: The command to send
            timeout: Optional timeout in seconds
            
        Returns:
            The response from the robot
            
        Raises:
            ConnectionError: If not connected
            CommandError: If the command fails
            TimeoutError: If the operation times out
        """
        if not self.is_connected or not self._command_processor:
            raise ConnectionError("Not connected to robot")
            
        return self._command_processor.send_command(command, timeout)
    
    def get_video_frame(self, timeout: Optional[float] = None) -> Optional[bytes]:
        """Get the next video frame.
        
        Args:
            timeout: Optional timeout in seconds
            
        Returns:
            The video frame data, or None if no frame is available
        """
        if not self.is_connected or not self._video_stream:
            return None
            
        return self._video_stream.get_frame(timeout)
    
    def __enter__(self):
        """Context manager entry."""
        return self
    
    def __exit__(self, exc_type, exc_val, exc_tb):
        """Context manager exit."""
        self.disconnect()
    
    def __del__(self):
        """Ensure resources are cleaned up."""
        self.disconnect()
