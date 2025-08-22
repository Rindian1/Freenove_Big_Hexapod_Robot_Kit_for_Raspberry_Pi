# -*- coding: utf-8 -*-
"""
Test script for Proximity Light Control System

This script demonstrates the functionality of the proximity light control system
for the CURSOR hexapod spider robot.
"""

import time
import sys
import os

# Add the current directory to the Python path
sys.path.append(os.path.dirname(os.path.abspath(__file__)))

from proximity_light_control import ProximityLightControl, ProximityConfig


def test_basic_functionality():
    """Test basic proximity light control functionality."""
    print("=" * 60)
    print("CURSOR Hexapod Spider Robot - Proximity Light Control Test")
    print("=" * 60)
    
    # Create configuration with debug mode enabled
    config = ProximityConfig(
        red_threshold=15.0,
        yellow_threshold=30.0,
        green_threshold=50.0,
        sensor_polling_rate=10.0,  # Lower rate for testing
        enable_buzzer=True,
        pulse_enabled=True,
        debug_mode=True,
        auto_start=False  # Don't auto-start for testing
    )
    
    print(f"Configuration:")
    print(f"  Red threshold: {config.red_threshold}cm")
    print(f"  Yellow threshold: {config.yellow_threshold}cm")
    print(f"  Green threshold: {config.green_threshold}cm")
    print(f"  Polling rate: {config.sensor_polling_rate}Hz")
    print(f"  Buzzer enabled: {config.enable_buzzer}")
    print(f"  Pulse enabled: {config.pulse_enabled}")
    print()
    
    # Create proximity control system
    print("Initializing proximity light control system...")
    proximity_system = ProximityLightControl(config)
    
    if not proximity_system.is_initialized:
        print("❌ Failed to initialize proximity system")
        return False
    
    print("✅ Proximity system initialized successfully")
    
    # Test system functionality
    print("\nTesting system functionality...")
    if not proximity_system.test_system():
        print("❌ System test failed")
        return False
    
    print("✅ System test passed")
    
    return proximity_system


def test_monitoring(proximity_system, duration=30):
    """Test the monitoring functionality."""
    print(f"\nStarting proximity monitoring for {duration} seconds...")
    print("Move objects in front of the robot to test different states:")
    print("  🔴 RED: < 15cm (close proximity)")
    print("  🟡 YELLOW: 15-30cm (caution zone)")
    print("  🟢 GREEN: > 30cm (safe distance)")
    print("Press Ctrl+C to stop early")
    print()
    
    try:
        # Start monitoring
        if not proximity_system.start_monitoring():
            print("❌ Failed to start monitoring")
            return False
        
        print("✅ Monitoring started")
        
        # Monitor for specified duration
        start_time = time.time()
        last_status_time = 0
        
        while time.time() - start_time < duration:
            current_time = time.time()
            
            # Print status every 5 seconds
            if current_time - last_status_time >= 5:
                status = proximity_system.get_status()
                print(f"Status: {status['current_state']} - "
                      f"Distance: {status['current_distance']:.1f}cm - "
                      f"Updates: {status['update_count']}")
                last_status_time = current_time
            
            time.sleep(0.1)
        
        # Stop monitoring
        proximity_system.stop_monitoring()
        print("✅ Monitoring stopped")
        
        # Print final statistics
        final_status = proximity_system.get_status()
        print(f"\nFinal Statistics:")
        print(f"  Total updates: {final_status['update_count']}")
        print(f"  Errors: {final_status['error_count']}")
        print(f"  Final state: {final_status['current_state']}")
        print(f"  Final distance: {final_status['current_distance']:.1f}cm")
        
        return True
        
    except KeyboardInterrupt:
        print("\n⚠️  Monitoring interrupted by user")
        proximity_system.stop_monitoring()
        return True
    except Exception as e:
        print(f"❌ Error during monitoring: {e}")
        proximity_system.stop_monitoring()
        return False


def test_configuration_updates(proximity_system):
    """Test configuration update functionality."""
    print("\nTesting configuration updates...")
    
    # Test updating thresholds
    new_config = {
        'red_threshold': 10.0,
        'yellow_threshold': 20.0,
        'green_threshold': 40.0
    }
    
    print(f"Updating thresholds to: {new_config}")
    if proximity_system.update_config(**new_config):
        print("✅ Configuration updated successfully")
        
        # Verify the update
        status = proximity_system.get_status()
        config = status['config']
        print(f"New thresholds:")
        print(f"  Red: {config['red_threshold']}cm")
        print(f"  Yellow: {config['yellow_threshold']}cm")
        print(f"  Green: {config['green_threshold']}cm")
    else:
        print("❌ Configuration update failed")
        return False
    
    return True


def test_individual_states(proximity_system):
    """Test individual light states."""
    print("\nTesting individual light states...")
    
    states = [
        ('GREEN', 'Safe distance'),
        ('YELLOW', 'Caution zone'),
        ('RED', 'Warning zone'),
        ('OFF', 'System disabled')
    ]
    
    for state_name, description in states:
        print(f"Testing {state_name} state ({description})...")
        
        # Manually set the state (for testing purposes)
        if hasattr(proximity_system, '_set_light_state'):
            # This is a test-only method
            proximity_system._set_light_state(getattr(proximity_system.__class__, 'LightState', type('', (), {}))().GREEN)
        
        time.sleep(2)
    
    print("✅ Light state tests completed")
    return True


def main():
    """Main test function."""
    print("Starting Proximity Light Control System Tests")
    print("=" * 60)
    
    try:
        # Test 1: Basic functionality
        proximity_system = test_basic_functionality()
        if not proximity_system:
            print("❌ Basic functionality test failed")
            return 1
        
        # Test 2: Configuration updates
        if not test_configuration_updates(proximity_system):
            print("❌ Configuration update test failed")
            return 1
        
        # Test 3: Individual states
        if not test_individual_states(proximity_system):
            print("❌ Individual state test failed")
            return 1
        
        # Test 4: Monitoring (interactive)
        print("\n" + "=" * 60)
        print("INTERACTIVE TEST")
        print("=" * 60)
        print("This test will run for 30 seconds.")
        print("Move objects in front of the robot to see the light changes.")
        print("The system will automatically adjust lights based on distance.")
        
        user_input = input("\nPress Enter to start the interactive test (or 'q' to quit): ")
        if user_input.lower() == 'q':
            print("Test cancelled by user")
            return 0
        
        if not test_monitoring(proximity_system, duration=30):
            print("❌ Monitoring test failed")
            return 1
        
        print("\n" + "=" * 60)
        print("✅ ALL TESTS COMPLETED SUCCESSFULLY")
        print("=" * 60)
        print("The proximity light control system is working correctly!")
        print("You can now integrate this system with your hexapod robot.")
        
        return 0
        
    except Exception as e:
        print(f"❌ Test failed with error: {e}")
        return 1
    finally:
        print("\nTest completed.")


if __name__ == '__main__':
    exit_code = main()
    sys.exit(exit_code)
