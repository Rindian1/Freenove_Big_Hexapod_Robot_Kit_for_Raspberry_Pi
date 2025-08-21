# -*- coding: utf-8 -*-
"""
ADC (Analog-to-Digital Converter) module for Freenove Big Hexapod Robot Kit

This module handles analog voltage measurements using the ADS7830 ADC chip
for battery voltage monitoring and other analog sensor readings.
"""

import smbus
import time
import threading
from typing import List, Tuple, Optional


class ADC:
    """
    ADC class for handling analog voltage measurements using ADS7830.
    
    Provides functionality for reading battery voltage, monitoring power levels,
    and scanning I2C devices. Supports multiple channels and voltage calibration.
    """
    
    # ADS7830 Configuration
    ADS7830_COMMAND = 0x84
    I2C_ADDRESS = 0x48
    I2C_BUS = 1
    
    # Channel definitions
    CHANNEL_BATTERY_1 = 0
    CHANNEL_BATTERY_2 = 4
    
    # Voltage thresholds (in volts)
    BATTERY_LOW_THRESHOLD = 6.0
    BATTERY_CRITICAL_THRESHOLD = 5.5
    
    def __init__(self, i2c_bus: int = 1, i2c_address: int = 0x48, voltage_coefficient: float = 3.0):
        """
        Initialize the ADC with configuration parameters.
        
        Args:
            i2c_bus (int): I2C bus number (default: 1)
            i2c_address (int): I2C address of ADS7830 (default: 0x48)
            voltage_coefficient (float): Voltage scaling coefficient (default: 3.0)
        """
        self.i2c_bus = None
        self.i2c_address = i2c_address
        self.voltage_coefficient = voltage_coefficient
        self.is_initialized = False
        
        # Battery monitoring
        self.battery_voltage_history = []
        self.max_history_size = 10
        self.last_reading_time = 0
        self.reading_interval = 1.0  # seconds
        
        # Thread safety
        self._lock = threading.Lock()
        
        # Initialize I2C connection
        self._initialize_i2c(i2c_bus)
    
    def _initialize_i2c(self, i2c_bus: int) -> bool:
        """
        Initialize I2C bus connection.
        
        Args:
            i2c_bus (int): I2C bus number
            
        Returns:
            bool: True if initialization successful, False otherwise
        """
        try:
            self.i2c_bus = smbus.SMBus(i2c_bus)
            self.is_initialized = True
            print(f"ADC initialized on I2C bus {i2c_bus}, address 0x{self.i2c_address:02X}")
            return True
            
        except Exception as e:
            print(f"Failed to initialize ADC: {e}")
            self.is_initialized = False
            return False
    
    def scan_i2c_bus(self) -> List[int]:
        """
        Scan the I2C bus for connected devices.
        
        Returns:
            List[int]: List of found I2C device addresses
        """
        if not self.is_initialized:
            print("ADC not initialized. Cannot scan I2C bus.")
            return []
        
        print("Scanning I2C bus for devices...")
        found_devices = []
        
        for device_address in range(128):
            try:
                # Try to read from the device
                self.i2c_bus.read_byte_data(device_address, 0)
                found_devices.append(device_address)
                print(f"Device found at address: 0x{device_address:02X}")
                
            except OSError:
                # Device not present at this address
                pass
            except Exception as e:
                print(f"Error scanning address 0x{device_address:02X}: {e}")
        
        if not found_devices:
            print("No I2C devices found.")
        else:
            print(f"Found {len(found_devices)} I2C device(s)")
        
        return found_devices
    
    def _read_stable_byte(self, max_attempts: int = 10) -> Optional[int]:
        """
        Read a stable byte from the ADC with retry mechanism.
        
        Args:
            max_attempts (int): Maximum number of read attempts
            
        Returns:
            int: Stable byte value or None if failed
        """
        if not self.is_initialized:
            return None
        
        for attempt in range(max_attempts):
            try:
                with self._lock:
                    value1 = self.i2c_bus.read_byte(self.i2c_address)
                    value2 = self.i2c_bus.read_byte(self.i2c_address)
                
                if value1 == value2:
                    return value1
                    
            except Exception as e:
                print(f"ADC read attempt {attempt + 1} failed: {e}")
                time.sleep(0.01)  # Short delay before retry
        
        print(f"Failed to read stable byte after {max_attempts} attempts")
        return None
    
    def read_channel_voltage(self, channel: int, samples: int = 3) -> Optional[float]:
        """
        Read voltage from a specific ADC channel with averaging.
        
        Args:
            channel (int): ADC channel number (0-7)
            samples (int): Number of samples to average
            
        Returns:
            float: Voltage reading in volts, or None if failed
        """
        if not self.is_initialized:
            return None
        
        if not 0 <= channel <= 7:
            print(f"Invalid channel number: {channel}. Must be 0-7.")
            return None
        
        readings = []
        
        for _ in range(samples):
            try:
                # Calculate command for the specified channel
                command = self.ADS7830_COMMAND | ((((channel << 2) | (channel >> 1)) & 0x07) << 4)
                
                with self._lock:
                    self.i2c_bus.write_byte(self.i2c_address, command)
                
                value = self._read_stable_byte()
                if value is not None:
                    # Convert ADC value to voltage
                    voltage = (value / 255.0) * 5.0 * self.voltage_coefficient
                    readings.append(voltage)
                
                time.sleep(0.01)  # Small delay between readings
                
            except Exception as e:
                print(f"Error reading channel {channel}: {e}")
        
        if readings:
            # Return average of readings
            average_voltage = sum(readings) / len(readings)
            return round(average_voltage, 2)
        
        return None
    
    def read_battery_voltage(self) -> Tuple[Optional[float], Optional[float]]:
        """
        Read battery voltage from both battery channels.
        
        Returns:
            Tuple[float, float]: (battery1_voltage, battery2_voltage) in volts
        """
        battery1 = self.read_channel_voltage(self.CHANNEL_BATTERY_1)
        battery2 = self.read_channel_voltage(self.CHANNEL_BATTERY_2)
        
        # Store in history for monitoring
        if battery1 is not None and battery2 is not None:
            self._update_battery_history(battery1, battery2)
        
        return battery1, battery2
    
    def _update_battery_history(self, battery1: float, battery2: float) -> None:
        """
        Update battery voltage history for monitoring.
        
        Args:
            battery1 (float): Battery 1 voltage
            battery2 (float): Battery 2 voltage
        """
        current_time = time.time()
        
        # Only update if enough time has passed
        if current_time - self.last_reading_time >= self.reading_interval:
            self.battery_voltage_history.append({
                'timestamp': current_time,
                'battery1': battery1,
                'battery2': battery2
            })
            
            # Keep only recent history
            if len(self.battery_voltage_history) > self.max_history_size:
                self.battery_voltage_history.pop(0)
            
            self.last_reading_time = current_time
    
    def get_battery_status(self) -> dict:
        """
        Get comprehensive battery status information.
        
        Returns:
            dict: Battery status including voltages, warnings, and trends
        """
        battery1, battery2 = self.read_battery_voltage()
        
        status = {
            'battery1_voltage': battery1,
            'battery2_voltage': battery2,
            'battery1_status': self._get_voltage_status(battery1),
            'battery2_status': self._get_voltage_status(battery2),
            'overall_status': 'unknown',
            'warnings': [],
            'history': self.battery_voltage_history.copy()
        }
        
        # Determine overall status
        if battery1 is not None and battery2 is not None:
            min_voltage = min(battery1, battery2)
            status['overall_status'] = self._get_voltage_status(min_voltage)
            
            # Generate warnings
            if battery1 < self.BATTERY_CRITICAL_THRESHOLD:
                status['warnings'].append(f"Battery 1 critically low: {battery1}V")
            elif battery1 < self.BATTERY_LOW_THRESHOLD:
                status['warnings'].append(f"Battery 1 low: {battery1}V")
                
            if battery2 < self.BATTERY_CRITICAL_THRESHOLD:
                status['warnings'].append(f"Battery 2 critically low: {battery2}V")
            elif battery2 < self.BATTERY_LOW_THRESHOLD:
                status['warnings'].append(f"Battery 2 low: {battery2}V")
        
        return status
    
    def _get_voltage_status(self, voltage: Optional[float]) -> str:
        """
        Get status string for a voltage reading.
        
        Args:
            voltage (float): Voltage reading in volts
            
        Returns:
            str: Status string ('good', 'low', 'critical', 'unknown')
        """
        if voltage is None:
            return 'unknown'
        elif voltage < self.BATTERY_CRITICAL_THRESHOLD:
            return 'critical'
        elif voltage < self.BATTERY_LOW_THRESHOLD:
            return 'low'
        else:
            return 'good'
    
    def calibrate_voltage_coefficient(self, known_voltage: float, channel: int = 0) -> bool:
        """
        Calibrate the voltage coefficient using a known voltage source.
        
        Args:
            known_voltage (float): Known voltage in volts
            channel (int): ADC channel to use for calibration
            
        Returns:
            bool: True if calibration successful, False otherwise
        """
        if not self.is_initialized:
            return False
        
        # Read raw ADC value
        try:
            command = self.ADS7830_COMMAND | ((((channel << 2) | (channel >> 1)) & 0x07) << 4)
            
            with self._lock:
                self.i2c_bus.write_byte(self.i2c_address, command)
            
            value = self._read_stable_byte()
            if value is not None:
                # Calculate new coefficient
                new_coefficient = (known_voltage * 255.0) / (value * 5.0)
                self.voltage_coefficient = new_coefficient
                print(f"Calibrated voltage coefficient to: {new_coefficient:.3f}")
                return True
                
        except Exception as e:
            print(f"Calibration failed: {e}")
        
        return False
    
    def get_voltage_trend(self, minutes: int = 5) -> dict:
        """
        Get voltage trend over the specified time period.
        
        Args:
            minutes (int): Time period in minutes
            
        Returns:
            dict: Voltage trend information
        """
        if not self.battery_voltage_history:
            return {'trend': 'no_data', 'change_rate': 0.0}
        
        cutoff_time = time.time() - (minutes * 60)
        recent_readings = [
            reading for reading in self.battery_voltage_history
            if reading['timestamp'] > cutoff_time
        ]
        
        if len(recent_readings) < 2:
            return {'trend': 'insufficient_data', 'change_rate': 0.0}
        
        # Calculate trend
        first_reading = recent_readings[0]
        last_reading = recent_readings[-1]
        
        time_diff = last_reading['timestamp'] - first_reading['timestamp']
        voltage_diff = min(last_reading['battery1'], last_reading['battery2']) - \
                      min(first_reading['battery1'], first_reading['battery2'])
        
        change_rate = voltage_diff / (time_diff / 60)  # V/min
        
        if change_rate > 0.1:
            trend = 'increasing'
        elif change_rate < -0.1:
            trend = 'decreasing'
        else:
            trend = 'stable'
        
        return {
            'trend': trend,
            'change_rate': round(change_rate, 3),
            'readings_count': len(recent_readings)
        }
    
    def close_i2c(self) -> None:
        """Close the I2C bus connection."""
        if self.is_initialized and self.i2c_bus:
            try:
                self.i2c_bus.close()
                self.is_initialized = False
                print("ADC I2C connection closed")
            except Exception as e:
                print(f"Error closing I2C connection: {e}")
    
    def __enter__(self):
        """Context manager entry point."""
        return self
    
    def __exit__(self, exc_type, exc_value, traceback):
        """Context manager exit point."""
        self.close_i2c()


def test_adc():
    """Test function to demonstrate ADC functionality."""
    print("Testing ADC Module...")
    print("=" * 40)
    
    with ADC() as adc:
        # Test I2C scan
        print("\n1. Scanning I2C bus...")
        devices = adc.scan_i2c_bus()
        
        # Test battery voltage reading
        print("\n2. Reading battery voltage...")
        battery1, battery2 = adc.read_battery_voltage()
        print(f"Battery 1: {battery1}V")
        print(f"Battery 2: {battery2}V")
        
        # Test battery status
        print("\n3. Getting battery status...")
        status = adc.get_battery_status()
        print(f"Overall status: {status['overall_status']}")
        if status['warnings']:
            print("Warnings:")
            for warning in status['warnings']:
                print(f"  - {warning}")
        
        # Test voltage trend
        print("\n4. Getting voltage trend...")
        trend = adc.get_voltage_trend()
        print(f"Trend: {trend['trend']}")
        print(f"Change rate: {trend['change_rate']} V/min")
        
        # Test individual channel reading
        print("\n5. Reading individual channels...")
        for channel in range(2):
            voltage = adc.read_channel_voltage(channel)
            print(f"Channel {channel}: {voltage}V")
    
    print("\n✓ ADC test completed successfully")


if __name__ == '__main__':
    print("ADC Module for Hexapod Robot")
    print("=" * 40)
    
    # Run test if executed directly
    test_adc()