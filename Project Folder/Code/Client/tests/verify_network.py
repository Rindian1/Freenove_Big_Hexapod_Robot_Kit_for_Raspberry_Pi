"""Simple verification script for the enhanced network implementation."""

import sys
import os
import time

# Add parent directory to path for imports
sys.path.append(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

# Import our enhanced network components
from Client.network_manager_v2 import NetworkManager
from Client.connections import DummyConnection
from Client.interfaces import ConnectionState

def main():
    """Run a simple test of the network implementation."""
    print("Testing enhanced network implementation...\n")
    
    # Create a network manager with dummy connection for testing
    print("Creating NetworkManager with DummyConnection...")
    manager = NetworkManager(connection_class=DummyConnection)
    
    try:
        # Test initial state
        print(f"Initial state: {manager.state.name}")
        
        # Test connection
        print("\nTesting connection...")
        manager.connect("127.0.0.1")
        print(f"State after connect: {manager.state.name}")
        
        # Test command sending
        print("\nTesting command sending...")
        response = manager.send_command("TEST")
        print(f"Command response: {response}")
        
        # Test video streaming
        print("\nTesting video streaming...")
        frame = manager.get_video_frame(timeout=1.0)
        print(f"Received frame: {'Yes' if frame is not None else 'No'}")
        
        # Test disconnection
        print("\nTesting disconnection...")
        manager.disconnect()
        print(f"State after disconnect: {manager.state.name}")
        
    except Exception as e:
        print(f"\nError during test: {e}", file=sys.stderr)
        raise
    
    print("\nTest completed successfully!")

if __name__ == "__main__":
    main()
