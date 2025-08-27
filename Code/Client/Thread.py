"""
Thread management utilities for safe thread control in the Hexapod Robot system.

This module provides thread management utilities that are safer than directly
killing threads, which can lead to resource leaks and deadlocks.
"""

import threading
import ctypes
import inspect
import time
from typing import Optional, Callable, Any, List

class SafeThread(threading.Thread):
    """A thread class that supports safe stopping using an event flag."""
    
    def __init__(self, target: Optional[Callable] = None, name: Optional[str] = None, 
                 daemon: bool = True, **kwargs):
        """Initialize the thread with a stop event.
        
        Args:
            target: The function to run in the thread
            name: Thread name
            daemon: Whether to run as daemon thread
            **kwargs: Additional arguments to pass to the target function
        """
        super().__init__(target=target, name=name, daemon=daemon)
        self._stop_event = threading.Event()
        self._target = target
        self._args = kwargs.get('args', ())
        self._kwargs = kwargs.get('kwargs', {})
    
    def stop(self) -> None:
        """Signal the thread to stop."""
        self._stop_event.set()
    
    def stopped(self) -> bool:
        """Check if the thread has been signaled to stop."""
        return self._stop_event.is_set()
    
    def run(self) -> None:
        """Run the target function with stop checking."""
        if self._target is not None:
            # Call the target function with stop checking
            while not self.stopped():
                try:
                    self._target(*self._args, **self._kwargs)
                except Exception as e:
                    print(f"Error in thread {self.name}: {e}")
                    break

def _async_raise(tid, exctype):
    """Raises an exception in the thread with the given thread id.
    
    WARNING: This is a low-level function that can cause issues if not used carefully.
    Prefer using SafeThread with event-based stopping when possible.
    """
    if not inspect.isclass(exctype):
        exctype = type(exctype)
    res = ctypes.pythonapi.PyThreadState_SetAsyncExc(ctypes.c_long(tid), ctypes.py_object(exctype))
    if res == 0:
        raise ValueError("Invalid thread ID")
    elif res != 1:
        ctypes.pythonapi.PyThreadState_SetAsyncExc(ctypes.c_long(tid), None)
        raise SystemError("PyThreadState_SetAsyncExc failed")

def stop_thread(thread: threading.Thread, timeout: float = 1.0) -> None:
    """Safely stop a thread with a timeout.
    
    WARNING: This is a fallback method and should only be used when absolutely
    necessary, as it can lead to resource leaks. Prefer using SafeThread with
    event-based stopping when possible.
    
    Args:
        thread: The thread to stop
        timeout: Time to wait for thread to stop before forcing it
    """
    if not thread.is_alive():
        return
        
    # First try the safe way
    if isinstance(thread, SafeThread):
        thread.stop()
        thread.join(timeout)
        if not thread.is_alive():
            return
    
    # Fall back to more aggressive method if needed
    try:
        # Signal the thread to stop
        if hasattr(thread, 'stop'):
            thread.stop()
        
        # Try to join with timeout
        thread.join(timeout)
        
        # If still alive, try to raise an exception in the thread
        if thread.is_alive() and hasattr(thread, 'ident'):
            for _ in range(5):  # Try a few times
                try:
                    _async_raise(thread.ident, SystemExit)
                    thread.join(0.1)
                    if not thread.is_alive():
                        break
                except (ValueError, SystemError):
                    break
    except Exception as e:
        print(f"Error stopping thread: {e}")
    finally:
        # Last resort
        if hasattr(thread, 'is_alive') and thread.is_alive():
            print(f"Warning: Could not stop thread {getattr(thread, 'name', 'unknown')} cleanly")

class ThreadManager:
    """A manager for tracking and stopping multiple threads."""
    
    def __init__(self):
        self._threads: List[threading.Thread] = []
        self._lock = threading.Lock()
    
    def add_thread(self, thread: threading.Thread) -> None:
        """Add a thread to be managed."""
        with self._lock:
            self._threads.append(thread)
    
    def stop_all(self, timeout: float = 2.0) -> None:
        """Stop all managed threads."""
        with self._lock:
            for thread in self._threads[:]:  # Create a copy to iterate over
                if thread.is_alive():
                    try:
                        if hasattr(thread, 'stop'):
                            thread.stop()
                    except Exception as e:
                        print(f"Error stopping thread {thread.name}: {e}")
            
            # Wait for threads to stop
            end_time = time.time() + timeout
            for thread in self._threads[:]:
                if thread.is_alive():
                    remaining = max(0, end_time - time.time())
                    if remaining > 0:
                        thread.join(remaining)
                    if thread.is_alive():
                        print(f"Warning: Thread {thread.name} did not stop gracefully")
            
            self._threads = []
    
    def cleanup(self) -> None:
        """Clean up finished threads."""
        with self._lock:
            self._threads = [t for t in self._threads if t.is_alive()]