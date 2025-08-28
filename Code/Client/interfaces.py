"""Interfaces for the Hexapod Robot network system."""

from abc import ABC, abstractmethod
from typing import Optional, Tuple, Any
from enum import Enum


class ConnectionState(Enum):
    """Connection state enumeration."""
    DISCONNECTED = 0
    CONNECTING = 1
    CONNECTED = 2
    RECONNECTING = 3


class IConnection(ABC):
    """Interface for network connections."""
    
    @property
    @abstractmethod
    def state(self) -> ConnectionState:
        """Get the current connection state."""
        pass
    
    @abstractmethod
    def connect(self, host: str, port: int) -> None:
        """Connect to the specified host and port.
        
        Args:
            host: The host to connect to
            port: The port to connect to
            
        Raises:
            ConnectionError: If connection fails
        """
        pass
    
    @abstractmethod
    def disconnect(self) -> None:
        """Disconnect from the remote host."""
        pass
    
    @abstractmethod
    def send(self, data: bytes, timeout: Optional[float] = None) -> int:
        """Send data over the connection.
        
        Args:
            data: The data to send
            timeout: Optional timeout in seconds
            
        Returns:
            int: Number of bytes sent
            
        Raises:
            ConnectionError: If not connected or send fails
            TimeoutError: If operation times out
        """
        pass
    
    @abstractmethod
    def receive(self, size: int = 4096, timeout: Optional[float] = None) -> bytes:
        """Receive data from the connection.
        
        Args:
            size: Maximum number of bytes to receive
            timeout: Optional timeout in seconds
            
        Returns:
            bytes: The received data
            
        Raises:
            ConnectionError: If not connected or receive fails
            TimeoutError: If operation times out
        """
        pass
    
    @abstractmethod
    def close(self) -> None:
        """Close the connection and release resources."""
        pass
    
    def __enter__(self):
        """Context manager entry."""
        return self
    
    def __exit__(self, exc_type, exc_val, exc_tb):
        """Context manager exit."""
        self.close()


class IVideoStream(ABC):
    """Interface for video streaming."""
    
    @abstractmethod
    def start(self) -> None:
        """Start the video stream."""
        pass
    
    @abstractmethod
    def stop(self) -> None:
        """Stop the video stream."""
        pass
    
    @abstractmethod
    def get_frame(self, timeout: Optional[float] = None) -> Optional[bytes]:
        """Get the next video frame.
        
        Args:
            timeout: Maximum time to wait for a frame
            
        Returns:
            Optional[bytes]: The frame data or None if no frame is available
        """
        pass


class ICommandProcessor(ABC):
    """Interface for command processing."""
    
    @abstractmethod
    def send_command(self, command: str, timeout: float = 5.0) -> str:
        """Send a command and wait for a response.
        
        Args:
            command: The command to send
            timeout: Maximum time to wait for a response
            
        Returns:
            str: The response from the robot
            
        Raises:
            CommandError: If the command fails
            TimeoutError: If the operation times out
        """
        pass
    
    @abstractmethod
    def process_response(self, data: bytes) -> Any:
        """Process a response from the robot.
        
        Args:
            data: The raw response data
            
        Returns:
            The processed response
        """
        pass
