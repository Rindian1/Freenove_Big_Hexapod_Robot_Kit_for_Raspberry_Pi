"""Connection implementations for the Hexapod Robot."""

import socket
import select
import time
from typing import Optional, Tuple, Union, Any

from .interfaces import IConnection, ConnectionState
from .exceptions import ConnectionError, TimeoutError
from .thread_safe import ThreadSafeValue
from .logging_config import get_logger

logger = get_logger(__name__)

class SocketConnection(IConnection):
    """Socket-based implementation of IConnection."""
    
    def __init__(self, sock: Optional[socket.socket] = None):
        """Initialize the socket connection.
        
        Args:
            sock: Optional existing socket to use
        """
        self._sock = sock or socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self._state = ThreadSafeValue(ConnectionState.DISCONNECTED, name="connection_state")
        self._lock = threading.RLock()
        self._host: Optional[str] = None
        self._port: Optional[int] = None
        self._timeout = 5.0
    
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
                logger.debug(f"Connection state changed from {old_state.name} to {value.name}")
    
    def connect(self, host: str, port: int, timeout: float = 5.0) -> None:
        """Connect to the specified host and port.
        
        Args:
            host: The host to connect to
            port: The port to connect to
            timeout: Connection timeout in seconds
            
        Raises:
            ConnectionError: If connection fails
        """
        with self._lock:
            if self.state == ConnectionState.CONNECTED:
                return
                
            self.state = ConnectionState.CONNECTING
            self._host = host
            self._port = port
            self._timeout = timeout
            
            try:
                # Set socket options
                self._sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
                self._sock.settimeout(timeout)
                
                # Connect
                self._sock.connect((host, port))
                self.state = ConnectionState.CONNECTED
                logger.info(f"Connected to {host}:{port}")
                
            except (socket.error, OSError) as e:
                self.state = ConnectionState.DISCONNECTED
                error_msg = f"Failed to connect to {host}:{port}: {e}"
                logger.error(error_msg)
                raise ConnectionError(error_msg) from e
    
    def disconnect(self) -> None:
        """Disconnect from the remote host."""
        with self._lock:
            if self.state == ConnectionState.DISCONNECTED:
                return
                
            try:
                if self._sock:
                    self._sock.shutdown(socket.SHUT_RDWR)
            except (OSError, AttributeError):
                pass  # Socket already closed
            finally:
                self.close()
    
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
        if self.state != ConnectionState.CONNECTED:
            raise ConnectionError("Not connected")
            
        if timeout is not None:
            self._sock.settimeout(timeout)
            
        try:
            return self._sock.sendall(data)
        except socket.timeout as e:
            raise TimeoutError("Send operation timed out") from e
        except (socket.error, OSError) as e:
            self.state = ConnectionState.DISCONNECTED
            raise ConnectionError(f"Send failed: {e}") from e
    
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
        if self.state != ConnectionState.CONNECTED:
            raise ConnectionError("Not connected")
            
        if timeout is not None:
            self._sock.settimeout(timeout)
            
        try:
            data = self._sock.recv(size)
            if not data:
                self.state = ConnectionState.DISCONNECTED
                raise ConnectionError("Connection closed by peer")
            return data
        except socket.timeout as e:
            raise TimeoutError("Receive operation timed out") from e
        except (socket.error, OSError) as e:
            self.state = ConnectionState.DISCONNECTED
            raise ConnectionError(f"Receive failed: {e}") from e
    
    def close(self) -> None:
        """Close the connection and release resources."""
        with self._lock:
            if self.state == ConnectionState.DISCONNECTED:
                return
                
            try:
                if self._sock:
                    self._sock.close()
            except (OSError, AttributeError):
                pass  # Socket already closed
            finally:
                self._sock = None
                self.state = ConnectionState.DISCONNECTED
                logger.info("Connection closed")
    
    def __del__(self):
        """Ensure resources are cleaned up."""
        self.close()


class DummyConnection(IConnection):
    """Dummy connection for testing purposes."""
    
    def __init__(self):
        """Initialize the dummy connection."""
        self._state = ThreadSafeValue(ConnectionState.DISCONNECTED, name="dummy_connection_state")
        self._lock = threading.RLock()
        self._buffer = bytearray()
    
    @property
    def state(self) -> ConnectionState:
        """Get the current connection state."""
        return self._state.value
    
    def connect(self, host: str, port: int) -> None:
        """Simulate connection."""
        with self._lock:
            self._state.value = ConnectionState.CONNECTED
    
    def disconnect(self) -> None:
        """Simulate disconnection."""
        with self._lock:
            self._state.value = ConnectionState.DISCONNECTED
    
    def send(self, data: bytes, timeout: Optional[float] = None) -> int:
        """Simulate sending data."""
        if self.state != ConnectionState.CONNECTED:
            raise ConnectionError("Not connected")
        with self._lock:
            self._buffer.extend(data)
            return len(data)
    
    def receive(self, size: int = 4096, timeout: Optional[float] = None) -> bytes:
        """Simulate receiving data."""
        if self.state != ConnectionState.CONNECTED:
            raise ConnectionError("Not connected")
        with self._lock:
            result = bytes(self._buffer[:size])
            self._buffer = self._buffer[size:]
            return result
    
    def close(self) -> None:
        """Clean up resources."""
        with self._lock:
            self._state.value = ConnectionState.DISCONNECTED
            self._buffer.clear()
