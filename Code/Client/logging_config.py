"""Logging configuration for the Hexapod Robot application."""
import logging
import os
from logging.handlers import RotatingFileHandler
from typing import Optional, Dict, Any

# Log directory and file settings
LOG_DIR = 'logs'
LOG_FILE = os.path.join(LOG_DIR, 'hexapod_robot.log')
MAX_BYTES = 5 * 1024 * 1024  # 5 MB
BACKUP_COUNT = 3

# Log format
LOG_FORMAT = '%(asctime)s - %(name)s - %(levelname)s - %(message)s'
DATE_FORMAT = '%Y-%m-%d %H:%M:%S'

# Default log level (can be overridden by environment variable)
DEFAULT_LOG_LEVEL = 'INFO'


def configure_logging(log_level: Optional[str] = None) -> None:
    """Configure logging for the application.
    
    Args:
        log_level: Logging level as a string (e.g., 'DEBUG', 'INFO').
                  If None, uses the value from the environment variable 'LOG_LEVEL',
                  or falls back to DEFAULT_LOG_LEVEL.
    """
    # Ensure log directory exists
    os.makedirs(LOG_DIR, exist_ok=True)
    
    # Get log level from parameter, environment, or default
    level_str = (log_level or os.getenv('LOG_LEVEL', DEFAULT_LOG_LEVEL)).upper()
    level = getattr(logging, level_str, logging.INFO)
    
    # Configure root logger
    root_logger = logging.getLogger()
    root_logger.setLevel(level)
    
    # Clear any existing handlers
    for handler in root_logger.handlers[:]:
        root_logger.removeHandler(handler)
    
    # Create console handler
    console_handler = logging.StreamHandler()
    console_handler.setLevel(level)
    
    # Create file handler with rotation
    file_handler = RotatingFileHandler(
        LOG_FILE,
        maxBytes=MAX_BYTES,
        backupCount=BACKUP_COUNT,
        encoding='utf-8'
    )
    file_handler.setLevel(level)
    
    # Create formatter and add it to the handlers
    formatter = logging.Formatter(LOG_FORMAT, DATE_FORMAT)
    console_handler.setFormatter(formatter)
    file_handler.setFormatter(formatter)
    
    # Add handlers to the root logger
    root_logger.addHandler(console_handler)
    root_logger.addHandler(file_handler)
    
    # Set up third-party loggers
    _configure_third_party_loggers(level)
    
    logging.info(f"Logging configured with level: {logging.getLevelName(level)}")


def _configure_third_party_loggers(level: int) -> None:
    """Configure log levels for third-party libraries."""
    # Suppress overly verbose logs from libraries
    third_party_loggers: Dict[str, int] = {
        'urllib3': logging.WARNING,
        'matplotlib': logging.WARNING,
        'PIL': logging.WARNING,
        'asyncio': logging.WARNING,
        'PyQt5': logging.WARNING,
    }
    
    for name, lvl in third_party_loggers.items():
        logging.getLogger(name).setLevel(lvl)


def get_logger(name: Optional[str] = None) -> logging.Logger:
    """Get a logger with the given name.
    
    Args:
        name: Logger name. If None, returns the root logger.
        
    Returns:
        Configured logger instance.
    """
    return logging.getLogger(name)


# Configure logging when this module is imported
configure_logging()
