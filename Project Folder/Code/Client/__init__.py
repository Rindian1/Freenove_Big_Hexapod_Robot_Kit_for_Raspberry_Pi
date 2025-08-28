"""Hexapod Robot Control Client.

This package provides the client-side implementation for controlling the Hexapod Robot.
It includes modules for network communication, UI components, and robot control logic.
"""

# Import key components for easier access
from .Client import Client
from .exceptions import RobotError, ConnectionError, CommandError, NetworkError, TimeoutError, InvalidStateError
from .thread_safe import ThreadSafeValue, ThreadSafeCounter
from .logging_config import configure_logging, get_logger
from .utils import (
    retry,
    log_duration,
    handle_errors,
    validate_range,
    clamp,
    map_value,
    Singleton
)

# Configure default logging when the package is imported
configure_logging()

# Enhanced Network Components
from .network_manager_v2 import NetworkManager
from .connections import SocketConnection, DummyConnection
from .video import VideoStream
from .commands import CommandProcessor, CommandResponse
from .interfaces import IConnection, IVideoStream, ICommandProcessor, ConnectionState

__all__ = [
    # Main client class
    'Client',
    
    # Enhanced Network Components
    'NetworkManager',
    'SocketConnection',
    'DummyConnection',
    'VideoStream',
    'CommandProcessor',
    'CommandResponse',
    'IConnection',
    'IVideoStream',
    'ICommandProcessor',
    'ConnectionState',
    
    # Exceptions
    'RobotError',
    'ConnectionError',
    'NetworkError',
    'TimeoutError',
    'CommandError',
    'VideoError',
    'InvalidStateError',
    
    # Thread-safe utilities
    'ThreadSafeValue',
    'ThreadSafeCounter',
    
    # Logging
    'configure_logging',
    'get_logger',
    
    # Utility functions
    'retry',
    'log_duration',
    'handle_errors',
    'validate_range',
    'clamp',
    'map_value',
    'Singleton',
]
