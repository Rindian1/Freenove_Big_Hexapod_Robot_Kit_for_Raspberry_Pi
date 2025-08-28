"""Utility functions and classes for the Hexapod Robot application."""
import functools
import logging
import time
import traceback
from typing import Any, Callable, Optional, Type, TypeVar, cast

from src.utils.exceptions import RobotError
from src.utils.logging_config import get_logger

T = TypeVar('T')
F = TypeVar('F', bound=Callable[..., Any])

logger = get_logger(__name__)

def retry(
    max_attempts: int = 3,
    delay: float = 1.0,
    backoff: float = 2.0,
    exceptions: tuple[Type[Exception], ...] = (Exception,),
):
    """Decorator that retries a function upon failure.
    
    Args:
        max_attempts: Maximum number of attempts before giving up.
        delay: Initial delay between attempts in seconds.
        backoff: Multiplier applied to delay between attempts.
        exceptions: Tuple of exceptions to catch and retry on.
    
    Returns:
        Decorated function with retry logic.
    """
    def decorator(func: F) -> F:
        @functools.wraps(func)
        def wrapper(*args: Any, **kwargs: Any) -> Any:
            current_delay = delay
            last_exception = None
            
            for attempt in range(1, max_attempts + 1):
                try:
                    return func(*args, **kwargs)
                except exceptions as e:
                    last_exception = e
                    if attempt == max_attempts:
                        break
                        
                    logger.warning(
                        f"Attempt {attempt}/{max_attempts} failed: {e}. "
                        f"Retrying in {current_delay:.2f}s..."
                    )
                    time.sleep(current_delay)
                    current_delay *= backoff
            
            # If we get here, all attempts failed
            raise RobotError(
                f"Failed after {max_attempts} attempts: {last_exception}"
            ) from last_exception
            
        return cast(F, wrapper)
    return decorator


def log_duration(level: int = logging.DEBUG):
    """Decorator to log the execution time of a function.
    
    Args:
        level: Logging level to use for the duration message.
    """
    def decorator(func: F) -> F:
        @functools.wraps(func)
        def wrapper(*args: Any, **kwargs: Any) -> Any:
            start_time = time.monotonic()
            try:
                return func(*args, **kwargs)
            finally:
                duration = time.monotonic() - start_time
                logger.log(
                    level,
                    f"Function {func.__qualname__} executed in {duration:.3f} seconds"
                )
        return cast(F, wrapper)
    return decorator


def handle_errors(
    reraise: bool = True,
    default_return: Any = None,
    log_level: int = logging.ERROR,
    exceptions: tuple[Type[Exception], ...] = (Exception,),
):
    """Decorator to handle and log exceptions in functions.
    
    Args:
        reraise: Whether to re-raise the exception after handling.
        default_return: Value to return if an exception occurs and reraise is False.
        log_level: Logging level to use for error messages.
        exceptions: Tuple of exceptions to catch.
    """
    def decorator(func: F) -> F:
        @functools.wraps(func)
        def wrapper(*args: Any, **kwargs: Any) -> Any:
            try:
                return func(*args, **kwargs)
            except exceptions as e:
                logger.log(
                    log_level,
                    f"Error in {func.__qualname__}: {e}\n"
                    f"{traceback.format_exc()}"
                )
                if reraise:
                    raise
                return default_return
        return cast(F, wrapper)
    return decorator


def validate_range(
    value: float,
    min_val: float,
    max_val: float,
    name: str = "value"
) -> float:
    """Validate that a value is within the specified range.
    
    Args:
        value: The value to validate.
        min_val: Minimum allowed value (inclusive).
        max_val: Maximum allowed value (inclusive).
        name: Name of the value for error messages.
        
    Returns:
        The validated value.
        
    Raises:
        ValueError: If the value is outside the valid range.
    """
    if not (min_val <= value <= max_val):
        raise ValueError(
            f"{name} must be between {min_val} and {max_val}, got {value}"
        )
    return value


def clamp(value: float, min_val: float, max_val: float) -> float:
    """Clamp a value between minimum and maximum bounds.
    
    Args:
        value: The value to clamp.
        min_val: Minimum allowed value.
        max_val: Maximum allowed value.
        
    Returns:
        The clamped value.
    """
    return max(min_val, min(value, max_val))


def map_value(
    value: float,
    in_min: float,
    in_max: float,
    out_min: float,
    out_max: float,
    clamp_output: bool = False
) -> float:
    """Map a value from one range to another.
    
    Args:
        value: The value to map.
        in_min: Minimum value of input range.
        in_max: Maximum value of input range.
        out_min: Minimum value of output range.
        out_max: Maximum value of output range.
        clamp_output: Whether to clamp the output to the output range.
        
    Returns:
        The mapped value.
    """
    if in_min == in_max:
        return (out_min + out_max) / 2
        
    result = (value - in_min) * (out_max - out_min) / (in_max - in_min) + out_min
    
    if clamp_output:
        return clamp(result, min(out_min, out_max), max(out_min, out_max))
    return result


class Singleton(type):
    """A metaclass that creates a Singleton base class when called."""
    _instances: dict[type, object] = {}

    def __call__(cls, *args: Any, **kwargs: Any) -> Any:
        if cls not in cls._instances:
            cls._instances[cls] = super().__call__(*args, **kwargs)
        return cls._instances[cls]
