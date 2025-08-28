# Enhanced Network System for Hexapod Robot

This document describes the improved networking system for the Hexapod Robot client, which provides better reliability, error handling, and maintainability compared to the original implementation.

## Key Features

- **Robust Connection Management**: Automatic reconnection with exponential backoff
- **Thread Safety**: Thread-safe operations for concurrent access
- **Comprehensive Error Handling**: Detailed error reporting and recovery
- **Modular Design**: Separated concerns for different network operations
- **Backward Compatibility**: Maintains compatibility with existing code
- **Extensive Logging**: Detailed logging for debugging and monitoring

## Components

### 1. NetworkManager

The core class that manages all network operations:

- Manages connection state (DISCONNECTED, CONNECTING, CONNECTED, RECONNECTING)
- Handles automatic reconnection
- Manages video and instruction threads
- Provides thread-safe command sending and receiving

### 2. Client (client_v2.py)

Enhanced client class that uses NetworkManager:

- Maintains backward compatibility with the original API
- Provides a cleaner interface for application code
- Handles video streaming and command processing
- Integrates with the robot control system

### 3. Thread-Safe Utilities

- `ThreadSafeValue`: Thread-safe container for shared values
- `ThreadSafeCounter`: Thread-safe counter with atomic operations
- Decorators for thread synchronization

### 4. Error Handling

Custom exceptions for better error reporting:

- `ConnectionError`: Network connection failures
- `NetworkError`: General network-related errors
- `TimeoutError`: Operation timeouts
- `CommandError`: Command processing errors

## Usage

### Basic Usage

```python
from client_v2 import Client
from logging_config import setup_logging

# Configure logging
setup_logging(level=logging.INFO)

# Create client instance
client = Client(use_network_manager=True)

try:
    # Connect to the robot
    client.turn_on_client("192.168.1.100")
    
    # Send commands
    client.send_data("TEST_COMMAND\n")
    
    # Receive data
    response = client.receive_data(timeout=5.0)
    print(f"Response: {response}")
    
    # Start video streaming
    client.receiving_video("192.168.1.100")
    
    # Process frames (in a real application, this would be in a separate thread)
    while True:
        frame = client.get_video_frame()
        if frame:
            # Process frame
            pass
            
except KeyboardInterrupt:
    print("Shutting down...")
finally:
    client.turn_off_client()
```

### Using NetworkManager Directly

For more advanced usage, you can use NetworkManager directly:

```python
from network_manager import NetworkManager, ConnectionState
from client_v2 import Client

# Create a client instance
client = Client()

# Create NetworkManager
manager = NetworkManager(client)

try:
    # Connect to the robot
    manager.connect("192.168.1.100", 5002, 8002)
    
    # Send a command
    manager.send_command("TEST_COMMAND\n")
    
    # Receive a response
    response = manager.receive_command(timeout=5.0)
    print(f"Response: {response}")
    
    # Check connection state
    if manager.state == ConnectionState.CONNECTED:
        print("Connected to robot")
    
finally:
    # Clean up
    manager.disconnect()
```

## Error Handling

The system provides detailed error information through exceptions and logging. Always wrap network operations in try/except blocks:

```python
try:
    client.send_data("COMMAND\n")
    response = client.receive_data(timeout=5.0)
except ConnectionError as e:
    print(f"Connection error: {e}")
    # Handle reconnection
except TimeoutError as e:
    print(f"Operation timed out: {e}")
    # Handle timeout
except Exception as e:
    print(f"Unexpected error: {e}")
    # Handle other errors
```

## Logging

The system uses Python's built-in logging module. Configure it like this:

```python
from logging_config import setup_logging

# Basic configuration (console output)
setup_logging(level=logging.INFO)

# Or with a log file
setup_logging(level=logging.DEBUG, log_file="robot.log")
```

## Testing

### Unit Tests

Run the unit tests with:

```bash
python -m unittest test_network_manager.py -v
```

### Integration Tests

Test the client with a real robot:

```bash
python test_client_v2.py 192.168.1.100
```

## Performance Considerations

- The system is designed to minimize latency for real-time control
- Video streaming runs in a separate thread to avoid blocking the main thread
- Network operations use timeouts to prevent hanging
- Thread-safe operations ensure data consistency

## Troubleshooting

### Common Issues

1. **Connection refused**:
   - Check if the robot is powered on
   - Verify the IP address and port
   - Ensure the robot's network is functioning

2. **Video streaming issues**:
   - Check network bandwidth
   - Verify video port is open
   - Check for firewall issues

3. **Command timeouts**:
   - Check network connectivity
   - Verify the robot is responding
   - Increase timeout if necessary

### Debugging

Enable debug logging for more detailed information:

```python
import logging
logging.basicConfig(level=logging.DEBUG)
```

## License

This software is part of the Freenove Hexapod Robot Kit. See the main LICENSE file for license information.

## Contributing

Contributions are welcome! Please submit pull requests or open issues on the project repository.

---

*This document was generated on August 28, 2025.*
