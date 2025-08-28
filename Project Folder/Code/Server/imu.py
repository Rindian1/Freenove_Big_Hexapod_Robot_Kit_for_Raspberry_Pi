# -*- coding: utf-8 -*-
"""
IMU (Inertial Measurement Unit) module for Freenove Big Hexapod Robot Kit

This module handles the MPU6050 sensor for orientation detection and balance control.
It implements quaternion-based sensor fusion with Kalman filtering for accurate
attitude estimation.
"""

import time
import math
import os
import threading
from typing import Dict, List, Tuple, Optional, Any
from dataclasses import dataclass
from kalman import Kalman_filter
from mpu6050 import mpu6050


@dataclass
class IMUConfig:
    """Configuration parameters for IMU operation."""
    # Sensor settings
    I2C_ADDRESS: int = 0x68
    I2C_BUS: int = 1
    ACCEL_RANGE: int = mpu6050.ACCEL_RANGE_2G
    GYRO_RANGE: int = mpu6050.GYRO_RANGE_250DEG
    
    # Filter parameters
    KALMAN_Q: float = 0.001  # Process noise
    KALMAN_R: float = 0.1    # Measurement noise
    
    # Fusion parameters
    PROPORTIONAL_GAIN: float = 100.0
    INTEGRAL_GAIN: float = 0.002
    HALF_TIME_STEP: float = 0.001
    
    # Calibration parameters
    CALIBRATION_SAMPLES: int = 100
    CALIBRATION_DELAY: float = 0.01
    
    # Update parameters
    UPDATE_RATE: float = 100.0  # Hz
    MIN_UPDATE_INTERVAL: float = 1.0 / UPDATE_RATE


@dataclass
class IMUData:
    """Data structure for IMU readings."""
    # Raw sensor data
    accel_x: float = 0.0
    accel_y: float = 0.0
    accel_z: float = 0.0
    gyro_x: float = 0.0
    gyro_y: float = 0.0
    gyro_z: float = 0.0
    
    # Filtered data
    filtered_accel_x: float = 0.0
    filtered_accel_y: float = 0.0
    filtered_accel_z: float = 0.0
    filtered_gyro_x: float = 0.0
    filtered_gyro_y: float = 0.0
    filtered_gyro_z: float = 0.0
    
    # Orientation angles (degrees)
    roll: float = 0.0
    pitch: float = 0.0
    yaw: float = 0.0
    
    # Quaternion
    quaternion_w: float = 1.0
    quaternion_x: float = 0.0
    quaternion_y: float = 0.0
    quaternion_z: float = 0.0
    
    # Timestamp
    timestamp: float = 0.0


class IMU:
    """
    IMU class for handling MPU6050 sensor and attitude estimation.
    
    This class provides:
    - Sensor initialization and configuration
    - Automatic calibration
    - Kalman filtering for noise reduction
    - Quaternion-based sensor fusion
    - Real-time attitude estimation
    - Error handling and recovery
    """
    
    def __init__(self, config: Optional[IMUConfig] = None):
        """
        Initialize the IMU system.
        
        Args:
            config (IMUConfig, optional): Configuration parameters
        """
        self.config = config or IMUConfig()
        self.is_initialized = False
        self.is_calibrated = False
        self.last_update_time = 0.0
        
        # Sensor instance
        self.sensor = None
        
        # Current IMU data
        self.current_data = IMUData()
        
        # Calibration data
        self.error_accel_data = {'x': 0.0, 'y': 0.0, 'z': 0.0}
        self.error_gyro_data = {'x': 0.0, 'y': 0.0, 'z': 0.0}
        
        # Fusion state
        self.integral_error_x = 0.0
        self.integral_error_y = 0.0
        self.integral_error_z = 0.0
        
        # Kalman filters
        self.kalman_filters = {}
        
        # Threading
        self._lock = threading.Lock()
        self._update_thread = None
        self._running = False
        
        # Initialize the system
        self._initialize_sensor()
        self._initialize_filters()
        self._calibrate_sensor()
        
        print("IMU system initialized successfully")
    
    def _initialize_sensor(self) -> bool:
        """
        Initialize the MPU6050 sensor.
        
        Returns:
            bool: True if successful
        """
        try:
            self.sensor = mpu6050(
                address=self.config.I2C_ADDRESS,
                bus=self.config.I2C_BUS
            )
            
            # Configure sensor ranges
            self.sensor.set_accel_range(self.config.ACCEL_RANGE)
            self.sensor.set_gyro_range(self.config.GYRO_RANGE)
            
            # Test sensor communication
            test_data = self.sensor.get_accel_data()
            if test_data is None:
                raise Exception("Failed to read from MPU6050")
            
            self.is_initialized = True
            print("MPU6050 sensor initialized successfully")
            return True
            
        except Exception as e:
            print(f"Error initializing MPU6050 sensor: {e}")
            self._handle_sensor_error(e)
            return False
    
    def _initialize_filters(self) -> None:
        """Initialize Kalman filters for all sensor axes."""
        self.kalman_filters = {
            'accel_x': Kalman_filter(self.config.KALMAN_Q, self.config.KALMAN_R),
            'accel_y': Kalman_filter(self.config.KALMAN_Q, self.config.KALMAN_R),
            'accel_z': Kalman_filter(self.config.KALMAN_Q, self.config.KALMAN_R),
            'gyro_x': Kalman_filter(self.config.KALMAN_Q, self.config.KALMAN_R),
            'gyro_y': Kalman_filter(self.config.KALMAN_Q, self.config.KALMAN_R),
            'gyro_z': Kalman_filter(self.config.KALMAN_Q, self.config.KALMAN_R)
        }
        print("Kalman filters initialized")
    
    def _calibrate_sensor(self) -> bool:
        """
        Calibrate the sensor by calculating bias values.
        
        Returns:
            bool: True if successful
        """
        if not self.is_initialized:
            print("Cannot calibrate: sensor not initialized")
            return False
        
        try:
            print("Starting sensor calibration...")
            
            # Initialize sums
            accel_sums = {'x': 0.0, 'y': 0.0, 'z': 0.0}
            gyro_sums = {'x': 0.0, 'y': 0.0, 'z': 0.0}
            
            # Collect calibration samples
            for i in range(self.config.CALIBRATION_SAMPLES):
                accel_data = self.sensor.get_accel_data()
                gyro_data = self.sensor.get_gyro_data()
                
                # Accumulate sums
                for axis in ['x', 'y', 'z']:
                    accel_sums[axis] += accel_data[axis]
                    gyro_sums[axis] += gyro_data[axis]
                
                time.sleep(self.config.CALIBRATION_DELAY)
                
                # Progress indicator
                if (i + 1) % 20 == 0:
                    print(f"Calibration progress: {(i + 1) * 100 // self.config.CALIBRATION_SAMPLES}%")
            
            # Calculate averages
            for axis in ['x', 'y', 'z']:
                self.error_accel_data[axis] = accel_sums[axis] / self.config.CALIBRATION_SAMPLES
                self.error_gyro_data[axis] = gyro_sums[axis] / self.config.CALIBRATION_SAMPLES
            
            # Adjust for gravity (assuming Z-axis is vertical)
            self.error_accel_data['z'] -= 9.8
            
            self.is_calibrated = True
            print("Sensor calibration completed successfully")
            print(f"Accel bias: {self.error_accel_data}")
            print(f"Gyro bias: {self.error_gyro_data}")
            return True
            
        except Exception as e:
            print(f"Error during calibration: {e}")
            return False
    
    def calculate_average_sensor_data(self) -> Tuple[Dict[str, float], Dict[str, float]]:
        """
        Calculate average sensor data for calibration.
        
        Returns:
            Tuple[Dict[str, float], Dict[str, float]]: Average accel and gyro data
        """
        return self.error_accel_data, self.error_gyro_data
    
    def update_imu_state(self) -> Tuple[float, float, float]:
        """
        Update IMU state and return current orientation angles.
        
        Returns:
            Tuple[float, float, float]: (roll, pitch, yaw) in degrees
        """
        if not self.is_initialized or not self.is_calibrated:
            print("Warning: IMU not properly initialized or calibrated")
            return 0.0, 0.0, 0.0
        
        try:
            with self._lock:
                # Check update rate
                current_time = time.time()
                if current_time - self.last_update_time < self.config.MIN_UPDATE_INTERVAL:
                    return self.current_data.roll, self.current_data.pitch, self.current_data.yaw
                
                # Read raw sensor data
                accel_data = self.sensor.get_accel_data()
                gyro_data = self.sensor.get_gyro_data()
                
                # Apply Kalman filtering
                filtered_accel_x = self.kalman_filters['accel_x'].kalman(
                    accel_data['x'] - self.error_accel_data['x']
                )
                filtered_accel_y = self.kalman_filters['accel_y'].kalman(
                    accel_data['y'] - self.error_accel_data['y']
                )
                filtered_accel_z = self.kalman_filters['accel_z'].kalman(
                    accel_data['z'] - self.error_accel_data['z']
                )
                
                filtered_gyro_x = self.kalman_filters['gyro_x'].kalman(
                    gyro_data['x'] - self.error_gyro_data['x']
                )
                filtered_gyro_y = self.kalman_filters['gyro_y'].kalman(
                    gyro_data['y'] - self.error_gyro_data['y']
                )
                filtered_gyro_z = self.kalman_filters['gyro_z'].kalman(
                    gyro_data['z'] - self.error_gyro_data['z']
                )
                
                # Update current data
                self.current_data.accel_x = accel_data['x']
                self.current_data.accel_y = accel_data['y']
                self.current_data.accel_z = accel_data['z']
                self.current_data.gyro_x = gyro_data['x']
                self.current_data.gyro_y = gyro_data['y']
                self.current_data.gyro_z = gyro_data['z']
                
                self.current_data.filtered_accel_x = filtered_accel_x
                self.current_data.filtered_accel_y = filtered_accel_y
                self.current_data.filtered_accel_z = filtered_accel_z
                self.current_data.filtered_gyro_x = filtered_gyro_x
                self.current_data.filtered_gyro_y = filtered_gyro_y
                self.current_data.filtered_gyro_z = filtered_gyro_z
                
                # Perform sensor fusion
                self._update_quaternion(filtered_accel_x, filtered_accel_y, filtered_accel_z,
                                      filtered_gyro_x, filtered_gyro_y, filtered_gyro_z)
                
                # Convert quaternion to Euler angles
                self._quaternion_to_euler()
                
                self.current_data.timestamp = current_time
                self.last_update_time = current_time
                
                return self.current_data.roll, self.current_data.pitch, self.current_data.yaw
                
        except Exception as e:
            print(f"Error updating IMU state: {e}")
            self._handle_sensor_error(e)
            return 0.0, 0.0, 0.0
    
    def _update_quaternion(self, accel_x: float, accel_y: float, accel_z: float,
                          gyro_x: float, gyro_y: float, gyro_z: float) -> None:
        """
        Update quaternion using Madgwick/Mahony fusion algorithm.
        
        Args:
            accel_x, accel_y, accel_z: Filtered accelerometer data
            gyro_x, gyro_y, gyro_z: Filtered gyroscope data
        """
        # Normalize accelerometer data
        accel_norm = math.sqrt(accel_x * accel_x + accel_y * accel_y + accel_z * accel_z)
        if accel_norm > 0:
            accel_x /= accel_norm
            accel_y /= accel_norm
            accel_z /= accel_norm
        
        # Calculate quaternion from accelerometer
        quat_vx = 2 * (self.current_data.quaternion_x * self.current_data.quaternion_z - 
                       self.current_data.quaternion_w * self.current_data.quaternion_y)
        quat_vy = 2 * (self.current_data.quaternion_w * self.current_data.quaternion_x + 
                       self.current_data.quaternion_y * self.current_data.quaternion_z)
        quat_vz = (self.current_data.quaternion_w * self.current_data.quaternion_w - 
                   self.current_data.quaternion_x * self.current_data.quaternion_x - 
                   self.current_data.quaternion_y * self.current_data.quaternion_y + 
                   self.current_data.quaternion_z * self.current_data.quaternion_z)
        
        # Calculate error
        error_x = accel_y * quat_vz - accel_z * quat_vy
        error_y = accel_z * quat_vx - accel_x * quat_vz
        error_z = accel_x * quat_vy - accel_y * quat_vx
        
        # Update integral error
        self.integral_error_x += error_x * self.config.INTEGRAL_GAIN
        self.integral_error_y += error_y * self.config.INTEGRAL_GAIN
        self.integral_error_z += error_z * self.config.INTEGRAL_GAIN
        
        # Apply proportional and integral feedback
        gyro_x += self.config.PROPORTIONAL_GAIN * error_x + self.integral_error_x
        gyro_y += self.config.PROPORTIONAL_GAIN * error_y + self.integral_error_y
        gyro_z += self.config.PROPORTIONAL_GAIN * error_z + self.integral_error_z
        
        # Update quaternion
        qw = self.current_data.quaternion_w
        qx = self.current_data.quaternion_x
        qy = self.current_data.quaternion_y
        qz = self.current_data.quaternion_z
        
        self.current_data.quaternion_w += (-qx * gyro_x - qy * gyro_y - qz * gyro_z) * self.config.HALF_TIME_STEP
        self.current_data.quaternion_x += (qw * gyro_x + qy * gyro_z - qz * gyro_y) * self.config.HALF_TIME_STEP
        self.current_data.quaternion_y += (qw * gyro_y - qx * gyro_z + qz * gyro_x) * self.config.HALF_TIME_STEP
        self.current_data.quaternion_z += (qw * gyro_z + qx * gyro_y - qy * gyro_x) * self.config.HALF_TIME_STEP
        
        # Normalize quaternion
        norm = math.sqrt(
            self.current_data.quaternion_w * self.current_data.quaternion_w +
            self.current_data.quaternion_x * self.current_data.quaternion_x +
            self.current_data.quaternion_y * self.current_data.quaternion_y +
            self.current_data.quaternion_z * self.current_data.quaternion_z
        )
        
        if norm > 0:
            self.current_data.quaternion_w /= norm
            self.current_data.quaternion_x /= norm
            self.current_data.quaternion_y /= norm
            self.current_data.quaternion_z /= norm
    
    def _quaternion_to_euler(self) -> None:
        """Convert quaternion to Euler angles (roll, pitch, yaw)."""
        qw = self.current_data.quaternion_w
        qx = self.current_data.quaternion_x
        qy = self.current_data.quaternion_y
        qz = self.current_data.quaternion_z
        
        # Roll (x-axis rotation)
        sinr_cosp = 2 * (qw * qx + qy * qz)
        cosr_cosp = 1 - 2 * (qx * qx + qy * qy)
        self.current_data.roll = math.atan2(sinr_cosp, cosr_cosp) * 180 / math.pi
        
        # Pitch (y-axis rotation)
        sinp = 2 * (qw * qy - qz * qx)
        if abs(sinp) >= 1:
            self.current_data.pitch = math.copysign(math.pi / 2, sinp) * 180 / math.pi
        else:
            self.current_data.pitch = math.asin(sinp) * 180 / math.pi
        
        # Yaw (z-axis rotation)
        siny_cosp = 2 * (qw * qz + qx * qy)
        cosy_cosp = 1 - 2 * (qy * qy + qz * qz)
        self.current_data.yaw = math.atan2(siny_cosp, cosy_cosp) * 180 / math.pi
    
    def get_orientation(self) -> Tuple[float, float, float]:
        """
        Get current orientation angles.
        
        Returns:
            Tuple[float, float, float]: (roll, pitch, yaw) in degrees
        """
        return self.current_data.roll, self.current_data.pitch, self.current_data.yaw
    
    def get_quaternion(self) -> Tuple[float, float, float, float]:
        """
        Get current quaternion.
        
        Returns:
            Tuple[float, float, float, float]: (w, x, y, z) quaternion
        """
        return (self.current_data.quaternion_w, self.current_data.quaternion_x,
                self.current_data.quaternion_y, self.current_data.quaternion_z)
    
    def get_raw_data(self) -> Dict[str, float]:
        """
        Get raw sensor data.
        
        Returns:
            Dict[str, float]: Raw accelerometer and gyroscope data
        """
        return {
            'accel_x': self.current_data.accel_x,
            'accel_y': self.current_data.accel_y,
            'accel_z': self.current_data.accel_z,
            'gyro_x': self.current_data.gyro_x,
            'gyro_y': self.current_data.gyro_y,
            'gyro_z': self.current_data.gyro_z
        }
    
    def get_filtered_data(self) -> Dict[str, float]:
        """
        Get filtered sensor data.
        
        Returns:
            Dict[str, float]: Filtered accelerometer and gyroscope data
        """
        return {
            'accel_x': self.current_data.filtered_accel_x,
            'accel_y': self.current_data.filtered_accel_y,
            'accel_z': self.current_data.filtered_accel_z,
            'gyro_x': self.current_data.filtered_gyro_x,
            'gyro_y': self.current_data.filtered_gyro_y,
            'gyro_z': self.current_data.filtered_gyro_z
        }
    
    def get_status(self) -> Dict[str, Any]:
        """
        Get comprehensive IMU status.
        
        Returns:
            Dict[str, Any]: Complete IMU status information
        """
        return {
            'is_initialized': self.is_initialized,
            'is_calibrated': self.is_calibrated,
            'last_update_time': self.last_update_time,
            'update_rate': self.config.UPDATE_RATE,
            'orientation': {
                'roll': self.current_data.roll,
                'pitch': self.current_data.pitch,
                'yaw': self.current_data.yaw
            },
            'quaternion': {
                'w': self.current_data.quaternion_w,
                'x': self.current_data.quaternion_x,
                'y': self.current_data.quaternion_y,
                'z': self.current_data.quaternion_z
            },
            'calibration': {
                'accel_bias': self.error_accel_data,
                'gyro_bias': self.error_gyro_data
            }
        }
    
    def recalibrate(self) -> bool:
        """
        Recalibrate the sensor.
        
        Returns:
            bool: True if successful
        """
        print("Starting sensor recalibration...")
        self.is_calibrated = False
        success = self._calibrate_sensor()
        if success:
            print("Recalibration completed successfully")
        else:
            print("Recalibration failed")
        return success
    
    def reset_filters(self) -> None:
        """Reset all Kalman filters to initial state."""
        self._initialize_filters()
        print("Kalman filters reset")
    
    def _handle_sensor_error(self, exception: Exception) -> None:
        """
        Handle sensor errors and attempt recovery.
        
        Args:
            exception (Exception): The error that occurred
        """
        print(f"IMU sensor error: {exception}")
        
        # Try to detect I2C issues
        try:
            os.system("i2cdetect -y 1")
        except Exception as e:
            print(f"Error running i2cdetect: {e}")
        
        # Attempt sensor reinitialization
        print("Attempting sensor reinitialization...")
        if self._initialize_sensor():
            print("Sensor reinitialization successful")
            self._calibrate_sensor()
        else:
            print("Sensor reinitialization failed")
            raise exception
    
    def handle_exception(self, exception: Exception) -> None:
        """
        Handle exceptions (legacy method for compatibility).
        
        Args:
            exception (Exception): The exception to handle
        """
        self._handle_sensor_error(exception)
    
    def start_continuous_update(self) -> bool:
        """
        Start continuous IMU updates in a separate thread.
        
        Returns:
            bool: True if started successfully
        """
        if self._running:
            print("Continuous update already running")
            return False
        
        self._running = True
        self._update_thread = threading.Thread(target=self._update_loop, daemon=True)
        self._update_thread.start()
        print("Continuous IMU update started")
        return True
    
    def stop_continuous_update(self) -> None:
        """Stop continuous IMU updates."""
        self._running = False
        if self._update_thread:
            self._update_thread.join(timeout=1.0)
        print("Continuous IMU update stopped")
    
    def _update_loop(self) -> None:
        """Continuous update loop for background processing."""
        while self._running:
            try:
                self.update_imu_state()
                time.sleep(self.config.MIN_UPDATE_INTERVAL)
            except Exception as e:
                print(f"Error in update loop: {e}")
                time.sleep(0.1)
    
    def __enter__(self):
        """Context manager entry."""
        return self
    
    def __exit__(self, exc_type, exc_value, traceback):
        """Context manager exit."""
        self.stop_continuous_update()


if __name__ == '__main__':
    print("IMU Test and Calibration")
    print("=" * 40)
    
    # Create IMU instance
    imu = IMU()
    
    try:
        # Test continuous operation
        print("\nStarting continuous IMU monitoring...")
        print("Press Ctrl+C to stop")
        
        start_time = time.time()
        sample_count = 0
        
        while True:
            try:
                roll, pitch, yaw = imu.update_imu_state()
                
                # Print every 100th sample
                if sample_count % 100 == 0:
                    elapsed = time.time() - start_time
                    rate = sample_count / elapsed if elapsed > 0 else 0
                    print(f"Roll: {roll:6.2f}° | Pitch: {pitch:6.2f}° | Yaw: {yaw:6.2f}° | Rate: {rate:5.1f} Hz")
                
                sample_count += 1
                time.sleep(0.01)  # 100 Hz update rate
                
            except KeyboardInterrupt:
                print("\nStopping IMU monitoring...")
                break
            except Exception as e:
                print(f"Error: {e}")
                imu.handle_exception(e)
                break
        
        # Print final statistics
        elapsed = time.time() - start_time
        avg_rate = sample_count / elapsed if elapsed > 0 else 0
        print(f"\nFinal Statistics:")
        print(f"Total samples: {sample_count}")
        print(f"Total time: {elapsed:.2f} seconds")
        print(f"Average rate: {avg_rate:.1f} Hz")
        
    except Exception as e:
        print(f"Test failed: {e}")
    finally:
        print("IMU test completed")
