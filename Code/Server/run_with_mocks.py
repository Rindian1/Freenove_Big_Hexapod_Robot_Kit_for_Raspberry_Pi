#!/usr/bin/env python3
"""
Freenove Big Hexapod Robot Kit - Mock Hardware Runner
This script sets up mock hardware and runs the server code on non-Raspberry Pi systems.
"""

import sys
import os

def setup_environment():
    """Set up the environment for running with mock hardware."""
    
    # Add current directory to Python path
    current_dir = os.path.dirname(os.path.abspath(__file__))
    if current_dir not in sys.path:
        sys.path.insert(0, current_dir)
    
    # Import and set up mock hardware
    try:
        from mock_hardware import setup_mock_hardware
        setup_mock_hardware()
        print("✓ Mock hardware system initialized")
        return True
    except ImportError as e:
        print(f"✗ Failed to import mock hardware: {e}")
        return False

def run_server():
    """Run the main server."""
    try:
        print("Starting hexapod robot server with mock hardware...")
        print("=" * 60)
        
        # Import and run the main server
        from main import main
        main()
        
    except ImportError as e:
        print(f"✗ Failed to import server modules: {e}")
        print("Make sure all required modules are available.")
        return False
    except Exception as e:
        print(f"✗ Server error: {e}")
        return False

def main():
    """Main entry point."""
    print("Freenove Big Hexapod Robot Kit - Development Mode")
    print("=" * 60)
    print("Running on non-Raspberry Pi system with mock hardware")
    print("This is for development and testing purposes only.")
    print("=" * 60)
    
    # Set up mock hardware
    if not setup_environment():
        print("Failed to set up mock hardware system.")
        sys.exit(1)
    
    # Run the server
    if not run_server():
        print("Failed to run server.")
        sys.exit(1)

if __name__ == '__main__':
    main()
