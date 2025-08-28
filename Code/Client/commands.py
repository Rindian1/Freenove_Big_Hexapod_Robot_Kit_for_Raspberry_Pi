"""Command processing for Hexapod Robot."""

import json
import time
import threading
from typing import Optional, Dict, Any, Callable, TypeVar, Generic, Type

from .interfaces import ICommandProcessor, IConnection
from .exceptions import CommandError, TimeoutError, ConnectionError
from .thread_safe import ThreadSafeDict, ThreadSafeValue
from .logging_config import get_logger

logger = get_logger(__name__)

T = TypeVar('T')

class CommandResponse(Generic[T]):
    """Represents a command response with typed data."""
    
    def __init__(self, success: bool, data: Optional[T] = None, error: Optional[str] = None):
        """Initialize the command response.
        
        Args:
            success: Whether the command was successful
            data: The response data (if successful)
            error: Error message (if failed)
        """
        self.success = success
        self.data = data
        self.error = error
    
    def __bool__(self) -> bool:
        """Return True if the command was successful."""
        return self.success
    
    def __str__(self) -> str:
        """Return a string representation of the response."""
        if self.success:
            return f"CommandResponse(success=True, data={self.data!r})"
        return f"CommandResponse(success=False, error={self.error!r})"


class CommandProcessor(ICommandProcessor):
    """Handles command processing for the Hexapod Robot."""
    
    def __init__(self, connection: IConnection, command_timeout: float = 5.0):
        """Initialize the command processor.
        
        Args:
            connection: The connection to use for sending/receiving commands
            command_timeout: Default timeout for command operations in seconds
        """
        self._conn = connection
        self._command_timeout = command_timeout
        self._command_lock = threading.RLock()
        self._response_handlers: Dict[str, Callable[[str], Any]] = {}
        self._pending_commands: Dict[str, threading.Event] = ThreadSafeDict()
        self._command_responses: Dict[str, str] = ThreadSafeDict()
        self._running = ThreadSafeValue(False, name="command_processor_running")
        self._thread: Optional[threading.Thread] = None
        
        # Register default response handlers
        self.register_response_handler("OK", lambda _: True)
        self.register_response_handler("ERROR", lambda msg: False)
    
    def start(self) -> None:
        """Start the command processor."""
        with self._command_lock:
            if self._running.value:
                logger.warning("Command processor is already running")
                return
                
            if self._conn.state != ConnectionState.CONNECTED:
                raise ConnectionError("Not connected to robot")
                
            self._running.value = True
            self._thread = threading.Thread(
                target=self._receive_loop,
                name="CommandProcessorThread",
                daemon=True
            )
            self._thread.start()
            logger.info("Command processor started")
    
    def stop(self) -> None:
        """Stop the command processor."""
        with self._command_lock:
            if not self._running.value:
                return
                
            self._running.value = False
            
            # Unblock any waiting commands
            for cmd_id, event in self._pending_commands.items():
                event.set()
            
            if self._thread and self._thread.is_alive():
                self._thread.join(timeout=2.0)
                if self._thread.is_alive():
                    logger.warning("Command processor thread did not stop gracefully")
            
            self._thread = None
            self._pending_commands.clear()
            self._command_responses.clear()
            logger.info("Command processor stopped")
    
    def register_response_handler(self, prefix: str, handler: Callable[[str], Any]) -> None:
        """Register a response handler for a command prefix.
        
        Args:
            prefix: The command prefix to handle (e.g., "OK", "ERROR")
            handler: A function that processes the response
        """
        with self._command_lock:
            self._response_handlers[prefix] = handler
    
    def send_command(self, command: str, timeout: Optional[float] = None) -> CommandResponse:
        """Send a command and wait for a response.
        
        Args:
            command: The command to send (without newline)
            timeout: Optional timeout in seconds (defaults to command_timeout)
            
        Returns:
            CommandResponse: The response from the robot
            
        Raises:
            CommandError: If the command fails
            TimeoutError: If the operation times out
        """
        if not command.endswith('\n'):
            command += '\n'
            
        cmd_id = str(id(command))
        event = threading.Event()
        self._pending_commands[cmd_id] = event
        
        try:
            # Send the command
            self._conn.send(command.encode('utf-8'))
            
            # Wait for response
            if not event.wait(timeout or self._command_timeout):
                raise TimeoutError(f"Command timed out: {command.strip()}")
            
            # Get and process the response
            response = self._command_responses.pop(cmd_id, None)
            if response is None:
                raise CommandError(f"No response received for command: {command.strip()}")
            
            # Parse the response
            for prefix, handler in self._response_handlers.items():
                if response.startswith(prefix):
                    result = handler(response[len(prefix):].strip())
                    if isinstance(result, bool):
                        return CommandResponse(result, error=None if result else "Command failed")
                    return CommandResponse(True, result)
            
            # No handler found, return raw response
            return CommandResponse(True, response)
            
        except ConnectionError as e:
            raise CommandError(f"Connection error: {e}") from e
        except Exception as e:
            raise CommandError(f"Command failed: {e}") from e
        finally:
            self._pending_commands.pop(cmd_id, None)
    
    def _receive_loop(self) -> None:
        """Main receive loop for processing responses."""
        buffer = ""
        
        try:
            while self._running.value:
                try:
                    # Receive data from the connection
                    data = self._conn.receive(4096, timeout=1.0).decode('utf-8', errors='ignore')
                    if not data:
                        continue
                    
                    # Add to buffer and process complete lines
                    buffer += data
                    lines = buffer.split('\n')
                    buffer = lines.pop()  # Keep incomplete line in buffer
                    
                    for line in lines:
                        line = line.strip()
                        if not line:
                            continue
                            
                        # Find a matching command ID in pending commands
                        matched = False
                        for cmd_id in list(self._pending_commands.keys()):
                            if line.startswith(cmd_id):
                                self._command_responses[cmd_id] = line[len(cmd_id):].strip()
                                self._pending_commands[cmd_id].set()
                                matched = True
                                break
                        
                        # If no command ID matched, process as a broadcast message
                        if not matched:
                            logger.debug(f"Received broadcast: {line}")
                            # TODO: Handle broadcast messages
                
                except TimeoutError:
                    continue  # No data received, continue waiting
                except ConnectionError as e:
                    logger.error(f"Connection error in receive loop: {e}")
                    self.stop()
                    break
                except Exception as e:
                    logger.error(f"Error in receive loop: {e}", exc_info=True)
                    time.sleep(0.1)  # Prevent tight loop on errors
        
        except Exception as e:
            logger.critical(f"Fatal error in receive loop: {e}", exc_info=True)
            self.stop()
            raise
        
        finally:
            self._running.value = False
            logger.info("Command processor loop ended")
    
    def __enter__(self):
        """Context manager entry."""
        self.start()
        return self
    
    def __exit__(self, exc_type, exc_val, exc_tb):
        """Context manager exit."""
        self.stop()
    
    def __del__(self):
        """Ensure resources are cleaned up."""
        self.stop()
