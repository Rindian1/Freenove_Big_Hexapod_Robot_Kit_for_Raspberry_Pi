"""Simple test script to verify NetworkManager functionality."""

import sys
import os
import time
import logging

# Add parent directory to path for imports
sys.path.append(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from Client.network_manager import NetworkManager, ConnectionState
from Client.client_v2 import Client
from Client.logging_config import setup_logging

# Configure logging
setup_logging(level=logging.DEBUG)
logger = logging.getLogger(__name__)

def test_network_manager():
    """Test basic NetworkManager functionality."""
    print("Testing NetworkManager...")
    
    # Create a mock client
    mock_client = type('MockClient', (), {
        'turn_on_client': lambda self, ip: None,
        'turn_off_client': lambda self: None,
        'process_instruction': lambda self, data: logger.debug(f"Processing instruction: {data}")
    })()
    
    # Create NetworkManager instance
    manager = NetworkManager(mock_client)
    
    try:
        # Test initial state
        print(f"Initial state: {manager.state}")
        assert manager.state == ConnectionState.DISCONNECTED
        
        # Test connection (this will fail but shouldn't crash)
        print("\nTesting connection...")
        try:
            manager.connect("127.0.0.1", 5002, 8002)
            print(f"Connection state: {manager.state}")
        except Exception as e:
            print(f"Expected connection error (this is normal for test): {e}")
        
        # Test disconnection
        print("\nTesting disconnection...")
        manager.disconnect()
        print(f"State after disconnect: {manager.state}")
        assert manager.state == ConnectionState.DISCONNECTED
        
        print("\nBasic NetworkManager test completed successfully!")
        
    except Exception as e:
        print(f"\nTest failed: {e}")
        raise
    finally:
        # Ensure cleanup
        manager.disconnect()

if __name__ == "__main__":
    test_network_manager()
