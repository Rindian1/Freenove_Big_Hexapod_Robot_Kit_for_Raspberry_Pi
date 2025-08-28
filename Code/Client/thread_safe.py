"""Thread-safe data structures for concurrent access."""
from typing import Any, Generic, TypeVar, Optional
import threading

T = TypeVar('T')

class ThreadSafeValue(Generic[T]):
    """Thread-safe container for a value with change notifications.
    
    This class provides thread-safe access to a value and allows registering
    callbacks for value changes.
    """
    
    def __init__(self, initial_value: Optional[T] = None, name: str = '') -> None:
        """Initialize the thread-safe value.
        
        Args:
            initial_value: Initial value to store
            name: Optional name for debugging purposes
        """
        self._value = initial_value
        self._name = name
        self._lock = threading.RLock()
        self._callbacks = []
    
    @property
    def value(self) -> Optional[T]:
        """Get the current value."""
        with self._lock:
            return self._value
    
    @value.setter
    def value(self, new_value: T) -> None:
        """Set a new value and notify observers if it changed.
        
        Args:
            new_value: The new value to set
        """
        with self._lock:
            if self._value != new_value:
                old_value = self._value
                self._value = new_value
                self._notify_observers(old_value, new_value)
    
    def set_value_if(self, condition: bool, true_value: T, false_value: T) -> None:
        """Set value based on a condition.
        
        Args:
            condition: Boolean condition
            true_value: Value to set if condition is True
            false_value: Value to set if condition is False
        """
        self.value = true_value if condition else false_value
    
    def add_callback(self, callback: callable) -> None:
        """Add a callback to be called when the value changes.
        
        Args:
            callback: Function with signature (old_value, new_value)
        """
        with self._lock:
            if callback not in self._callbacks:
                self._callbacks.append(callback)
    
    def remove_callback(self, callback: callable) -> None:
        """Remove a previously registered callback.
        
        Args:
            callback: Callback to remove
        """
        with self._lock:
            if callback in self._callbacks:
                self._callbacks.remove(callback)
    
    def _notify_observers(self, old_value: T, new_value: T) -> None:
        """Notify all registered callbacks of a value change.
        
        Args:
            old_value: Previous value
            new_value: New value
        """
        with self._lock:
            for callback in self._callbacks[:]:  # Create a copy to allow modification during iteration
                try:
                    callback(old_value, new_value)
                except Exception as e:
                    import logging
                    logging.error(f"Error in callback for {self._name or 'unnamed value'}: {e}", exc_info=True)
    
    def __str__(self) -> str:
        return f"ThreadSafeValue({self._name or ''}): {self._value}"
    
    def __repr__(self) -> str:
        return f"<ThreadSafeValue name='{self._name}' value={repr(self._value)}>"


class ThreadSafeCounter:
    """Thread-safe counter with increment/decrement operations."""
    
    def __init__(self, initial_value: int = 0, name: str = '') -> None:
        """Initialize the counter.
        
        Args:
            initial_value: Starting value
            name: Optional name for debugging
        """
        self._value = initial_value
        self._name = name
        self._lock = threading.RLock()
    
    def increment(self, amount: int = 1) -> int:
        """Increment the counter and return the new value.
        
        Args:
            amount: Amount to increment by (can be negative)
            
        Returns:
            The new counter value
        """
        with self._lock:
            self._value += amount
            return self._value
    
    def decrement(self, amount: int = 1) -> int:
        """Decrement the counter and return the new value.
        
        Args:
            amount: Amount to decrement by (can be negative)
            
        Returns:
            The new counter value
        """
        return self.increment(-amount)
    
    @property
    def value(self) -> int:
        """Get the current counter value."""
        with self._lock:
            return self._value
    
    def __str__(self) -> str:
        return f"ThreadSafeCounter({self._name or ''}): {self._value}"
    
    def __repr__(self) -> str:
        return f"<ThreadSafeCounter name='{self._name}' value={self._value}>"
