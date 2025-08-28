"""Custom exceptions for the Hexapod Robot application."""

class RobotError(Exception):
    """Base exception for robot-related errors."""
    pass

class ConnectionError(RobotError):
    """Raised when connection to the robot fails."""
    pass

class CommandError(RobotError):
    """Raised when a command fails to execute."""
    pass

class NetworkError(RobotError):
    """Raised for network-related errors."""
    pass

class TimeoutError(RobotError):
    """Raised when an operation times out."""
    pass

class InvalidStateError(RobotError):
    """Raised when an operation is attempted in an invalid state."""
    pass
