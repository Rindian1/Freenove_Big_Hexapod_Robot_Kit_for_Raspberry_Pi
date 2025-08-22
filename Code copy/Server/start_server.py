#!/usr/bin/env python3
"""
Freenove Big Hexapod Robot Kit - Universal Server Starter
This script automatically detects the platform and sets up appropriate hardware access.
"""

import sys
import os
import platform

def setup_mock_environment():
    """Set up mock hardware environment for non-Raspberry Pi systems."""
    
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

def check_hardware_availability():
    """Check if real hardware is available."""
    try:
        import smbus
        # Try to access I2C bus
        bus = smbus.SMBus(1)
        bus.close()
        return True
    except (ImportError, FileNotFoundError, OSError):
        return False

def run_server():
    """Run the main server."""
    try:
        print("Starting hexapod robot server...")
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
        import traceback
        traceback.print_exc()
        return False

def main():
    """Main entry point."""
    print("Freenove Big Hexapod Robot Kit - Server Starter")
    print("=" * 60)
    print(f"Platform: {platform.system()} {platform.release()}")
    print(f"Architecture: {platform.machine()}")
    print("=" * 60)
    
    # Check if we're on Raspberry Pi or need mocks
    if platform.system() == "Linux" and check_hardware_availability():
        print("✓ Real hardware detected - using actual hardware")
        print("Running on Raspberry Pi with real sensors and actuators")
    else:
        print("⚠ Development mode - using mock hardware")
        print("This is for development and testing purposes only.")
        print("Real robot functionality requires actual hardware.")
        
        if not setup_mock_environment():
            print("Failed to set up mock hardware system.")
            sys.exit(1)
    
    print("=" * 60)
    
    # Run the server
    if not run_server():
        print("Failed to run server.")
        sys.exit(1)

if __name__ == '__main__':
    main()
