"""Test script for the enhanced Client class with NetworkManager integration."""

import sys
import time
import logging
from threading import Event
from typing import Optional

# Add parent directory to path for imports
sys.path.append('..')

# Local imports
from Client.client_v2 import Client
from Client.logging_config import setup_logging

# Configure logging
setup_logging()
logger = logging.getLogger(__name__)


def test_connection(ip: str, use_network_manager: bool = True) -> None:
    """Test the client connection and basic operations.
    
    Args:
        ip: Server IP address
        use_network_manager: Whether to use the new NetworkManager
    """
    logger.info(f"Testing Client with use_network_manager={use_network_manager}")
    
    # Create client instance
    client = Client(use_network_manager=use_network_manager)
    
    try:
        # Test connection
        logger.info(f"Connecting to {ip}...")
        client.turn_on_client(ip)
        
        if not client.tcp_flag:
            logger.error("Failed to connect to server")
            return
        
        logger.info("Connected successfully")
        
        # Test sending a simple command
        test_command = "TEST_COMMAND\n"
        logger.info(f"Sending test command: {test_command.strip()}")
        client.send_data(test_command)
        
        # Test receiving data (with timeout)
        logger.info("Waiting for response...")
        response = client.receive_data(timeout=5.0)
        if response:
            logger.info(f"Received response: {response.strip()}")
        else:
            logger.warning("No response received")
        
        # Test video streaming (just start it, won't process frames in this test)
        logger.info("Starting video stream...")
        client.receiving_video(ip)
        
        # Let it run for a bit
        time.sleep(5)
        
        logger.info("Test completed successfully")
        
    except Exception as e:
        logger.error(f"Test failed: {e}", exc_info=True)
    finally:
        # Clean up
        logger.info("Cleaning up...")
        client.turn_off_client()
        logger.info("Done")


if __name__ == '__main__':
    import argparse
    
    parser = argparse.ArgumentParser(description='Test Client with NetworkManager')
    parser.add_argument('ip', help='Server IP address')
    parser.add_argument('--legacy', action='store_true', 
                       help='Use legacy implementation (without NetworkManager)')
    
    args = parser.parse_args()
    
    test_connection(args.ip, use_network_manager=not args.legacy)
