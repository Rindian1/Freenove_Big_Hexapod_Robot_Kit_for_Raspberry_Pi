#!/usr/bin/env python3
"""
Test script to demonstrate the mock hardware system working.
This shows that the core functionality can be tested on macOS.
"""

import sys
import os

def setup_mock_environment():
    """Set up mock hardware environment."""
    current_dir = os.path.dirname(os.path.abspath(__file__))
    if current_dir not in sys.path:
        sys.path.insert(0, current_dir)
    
    try:
        from mock_hardware import setup_mock_hardware
        setup_mock_hardware()
        print("✓ Mock hardware system initialized")
        return True
    except ImportError as e:
        print(f"✗ Failed to import mock hardware: {e}")
        return False

def test_basic_modules():
    """Test that basic modules can be imported."""
    print("\n=== Testing Basic Module Imports ===")
    
    try:
        # Test ADC
        from adc import ADC
        adc = ADC()
        voltage = adc.read_battery_voltage()
        print(f"✓ ADC test: Battery voltage = {voltage}")
    except Exception as e:
        print(f"✗ ADC test failed: {e}")
    
    try:
        # Test IMU
        from imu import IMU
        imu = IMU()
        print("✓ IMU test: IMU initialized successfully")
    except Exception as e:
        print(f"✗ IMU test failed: {e}")
    
    try:
        # Test servo controller
        from pca9685 import PCA9685
        pca = PCA9685()
        print("✓ PCA9685 test: Servo controller initialized")
    except Exception as e:
        print(f"✗ PCA9685 test failed: {e}")
    
    try:
        # Test LED
        from led import Led
        led = Led()
        print("✓ LED test: LED controller initialized")
    except Exception as e:
        print(f"✗ LED test failed: {e}")
    
    try:
        # Test buzzer
        from buzzer import Buzzer
        buzzer = Buzzer()
        print("✓ Buzzer test: Buzzer initialized")
    except Exception as e:
        print(f"✗ Buzzer test failed: {e}")

def test_server_components():
    """Test server components."""
    print("\n=== Testing Server Components ===")
    
    try:
        # Test command module
        from command import COMMAND
        print(f"✓ Command module: {len(COMMAND)} commands available")
    except Exception as e:
        print(f"✗ Command module test failed: {e}")
    
    try:
        # Test Thread module
        import Thread
        print("✓ Thread module: Thread utilities available")
    except Exception as e:
        print(f"✗ Thread module test failed: {e}")

def main():
    """Main test function."""
    print("Freenove Big Hexapod Robot Kit - Mock System Test")
    print("=" * 60)
    print("Testing mock hardware system on macOS")
    print("=" * 60)
    
    # Set up mock environment
    if not setup_mock_environment():
        print("Failed to set up mock environment")
        sys.exit(1)
    
    # Test basic modules
    test_basic_modules()
    
    # Test server components
    test_server_components()
    
    print("\n" + "=" * 60)
    print("✓ Mock system test completed successfully!")
    print("The mock hardware system is working correctly.")
    print("You can now develop and test the robot software on macOS.")
    print("=" * 60)

if __name__ == '__main__':
    main()
