# -*- coding: utf-8 -*-
"""
Kalman Filter module for Freenove Big Hexapod Robot Kit

This module provides various Kalman filter implementations for sensor data filtering
and noise reduction. It includes 1D, 2D, and 3D Kalman filters with different
configurations for various sensor types.
"""

import numpy as np
import time
from typing import Union, List, Tuple, Optional, Dict, Any
from dataclasses import dataclass
from enum import Enum


class FilterType(Enum):
    """Types of Kalman filters available."""
    ONE_DIMENSIONAL = "1D"
    TWO_DIMENSIONAL = "2D"
    THREE_DIMENSIONAL = "3D"
    ADAPTIVE = "adaptive"


@dataclass
class KalmanConfig:
    """Configuration parameters for Kalman filter operation."""
    # Noise parameters
    process_noise: float = 0.001      # Process noise covariance (Q)
    measurement_noise: float = 0.1    # Measurement noise covariance (R)
    
    # Initial state
    initial_state: float = 0.0        # Initial state estimate
    initial_covariance: float = 1.0   # Initial state covariance
    
    # Adaptive parameters
    adaptive_threshold: float = 60.0  # Threshold for adaptive behavior
    adaptive_weight: float = 0.4      # Weight for adaptive estimation
    
    # Performance parameters
    max_history_size: int = 100       # Maximum history size for statistics
    enable_statistics: bool = True    # Enable performance statistics


class KalmanFilter:
    """
    Enhanced Kalman Filter implementation for sensor data filtering.
    
    This class provides a robust 1D Kalman filter with adaptive behavior,
    performance monitoring, and comprehensive error handling.
    """
    
    def __init__(self, config: Optional[KalmanConfig] = None):
        """
        Initialize the Kalman filter.
        
        Args:
            config (KalmanConfig, optional): Configuration parameters
        """
        self.config = config or KalmanConfig()
        self.is_initialized = False
        
        # State variables
        self.posterior_estimate = self.config.initial_state
        self.posterior_error_covariance = self.config.initial_covariance
        
        # Previous values
        self.previous_measurement = 0.0
        self.previous_output = 0.0
        
        # Performance tracking
        self.measurement_count = 0
        self.total_processing_time = 0.0
        self.history = []
        self.max_history_size = self.config.max_history_size
        
        # Statistics
        self.min_value = float('inf')
        self.max_value = float('-inf')
        self.sum_values = 0.0
        self.sum_squared_values = 0.0
        
        # Initialize the filter
        self._initialize_filter()
        
        print(f"Kalman filter initialized with Q={self.config.process_noise}, R={self.config.measurement_noise}")
    
    def _initialize_filter(self) -> None:
        """Initialize the filter state."""
        self.is_initialized = True
        self.measurement_count = 0
        self.total_processing_time = 0.0
        self.history.clear()
        
        # Reset statistics
        self.min_value = float('inf')
        self.max_value = float('-inf')
        self.sum_values = 0.0
        self.sum_squared_values = 0.0
    
    def kalman(self, measurement: float) -> float:
        """
        Process a measurement through the Kalman filter.
        
        Args:
            measurement (float): New measurement value
            
        Returns:
            float: Filtered output value
        """
        if not self.is_initialized:
            print("Warning: Kalman filter not initialized")
            return measurement
        
        start_time = time.time()
        
        try:
            # Store previous values
            self.previous_measurement = measurement
            previous_output = self.previous_output
            
            # Adaptive behavior for large changes
            if abs(previous_output - measurement) >= self.config.adaptive_threshold:
                self.posterior_estimate = (measurement * self.config.adaptive_weight + 
                                         previous_output * (1 - self.config.adaptive_weight))
            else:
                self.posterior_estimate = previous_output
            
            # Prediction step
            # P_k|k-1 = P_k-1|k-1 + Q
            estimated_error_covariance = (self.posterior_error_covariance + 
                                        self.config.process_noise)
            
            # Update step
            # K_k = P_k|k-1 / (P_k|k-1 + R)
            kalman_gain = (estimated_error_covariance / 
                          (estimated_error_covariance + self.config.measurement_noise))
            
            # x_k|k = x_k|k-1 + K_k * (z_k - x_k|k-1)
            filtered_output = (self.posterior_estimate + 
                              kalman_gain * (measurement - previous_output))
            
            # P_k|k = (1 - K_k) * P_k|k-1
            self.posterior_error_covariance = ((1 - kalman_gain) * 
                                              estimated_error_covariance)
            
            # Update state
            self.previous_output = filtered_output
            
            # Update statistics
            self._update_statistics(measurement, filtered_output, time.time() - start_time)
            
            return filtered_output
            
        except Exception as e:
            print(f"Error in Kalman filter: {e}")
            return measurement
    
    def _update_statistics(self, measurement: float, output: float, processing_time: float) -> None:
        """
        Update performance statistics.
        
        Args:
            measurement (float): Raw measurement
            output (float): Filtered output
            processing_time (float): Processing time in seconds
        """
        self.measurement_count += 1
        self.total_processing_time += processing_time
        
        # Update value statistics
        self.min_value = min(self.min_value, measurement, output)
        self.max_value = max(self.max_value, measurement, output)
        self.sum_values += output
        self.sum_squared_values += output * output
        
        # Update history
        if self.config.enable_statistics:
            history_entry = {
                'timestamp': time.time(),
                'measurement': measurement,
                'output': output,
                'processing_time': processing_time,
                'kalman_gain': self._get_current_kalman_gain()
            }
            
            self.history.append(history_entry)
            
            # Maintain history size
            if len(self.history) > self.max_history_size:
                self.history.pop(0)
    
    def _get_current_kalman_gain(self) -> float:
        """Get the current Kalman gain value."""
        estimated_error_covariance = (self.posterior_error_covariance + 
                                    self.config.process_noise)
        return (estimated_error_covariance / 
                (estimated_error_covariance + self.config.measurement_noise))
    
    def reset(self) -> None:
        """Reset the filter to initial state."""
        self.posterior_estimate = self.config.initial_state
        self.posterior_error_covariance = self.config.initial_covariance
        self.previous_measurement = 0.0
        self.previous_output = 0.0
        self._initialize_filter()
        print("Kalman filter reset")
    
    def set_noise_parameters(self, process_noise: float, measurement_noise: float) -> None:
        """
        Update noise parameters.
        
        Args:
            process_noise (float): New process noise covariance
            measurement_noise (float): New measurement noise covariance
        """
        self.config.process_noise = process_noise
        self.config.measurement_noise = measurement_noise
        print(f"Updated noise parameters: Q={process_noise}, R={measurement_noise}")
    
    def get_statistics(self) -> Dict[str, Any]:
        """
        Get comprehensive filter statistics.
        
        Returns:
            Dict[str, Any]: Filter statistics
        """
        if self.measurement_count == 0:
            return {
                'measurement_count': 0,
                'average_processing_time': 0.0,
                'min_value': 0.0,
                'max_value': 0.0,
                'mean_value': 0.0,
                'variance': 0.0,
                'current_kalman_gain': self._get_current_kalman_gain(),
                'history_size': len(self.history)
            }
        
        mean_value = self.sum_values / self.measurement_count
        variance = ((self.sum_squared_values / self.measurement_count) - 
                   (mean_value * mean_value))
        
        return {
            'measurement_count': self.measurement_count,
            'average_processing_time': self.total_processing_time / self.measurement_count,
            'min_value': self.min_value,
            'max_value': self.max_value,
            'mean_value': mean_value,
            'variance': variance,
            'current_kalman_gain': self._get_current_kalman_gain(),
            'history_size': len(self.history)
        }
    
    def get_recent_history(self, count: int = 10) -> List[Dict[str, Any]]:
        """
        Get recent filter history.
        
        Args:
            count (int): Number of recent entries to return
            
        Returns:
            List[Dict[str, Any]]: Recent history entries
        """
        return self.history[-count:] if self.history else []
    
    def get_state(self) -> Dict[str, float]:
        """
        Get current filter state.
        
        Returns:
            Dict[str, float]: Current state variables
        """
        return {
            'posterior_estimate': self.posterior_estimate,
            'posterior_error_covariance': self.posterior_error_covariance,
            'previous_measurement': self.previous_measurement,
            'previous_output': self.previous_output,
            'kalman_gain': self._get_current_kalman_gain()
        }


class KalmanFilter2D:
    """
    2D Kalman Filter for position and velocity estimation.
    
    This class implements a 2D Kalman filter suitable for tracking
    position and velocity in a single dimension.
    """
    
    def __init__(self, process_noise: float = 0.001, measurement_noise: float = 0.1,
                 initial_position: float = 0.0, initial_velocity: float = 0.0):
        """
        Initialize 2D Kalman filter.
        
        Args:
            process_noise (float): Process noise covariance
            measurement_noise (float): Measurement noise covariance
            initial_position (float): Initial position estimate
            initial_velocity (float): Initial velocity estimate
        """
        # State vector: [position, velocity]
        self.state = np.array([initial_position, initial_velocity], dtype=float)
        
        # State covariance matrix
        self.covariance = np.eye(2, dtype=float)
        
        # Process noise covariance matrix
        self.Q = np.array([[process_noise, 0], [0, process_noise]], dtype=float)
        
        # Measurement noise covariance
        self.R = measurement_noise
        
        # Measurement matrix (we only measure position)
        self.H = np.array([[1, 0]], dtype=float)
        
        # State transition matrix
        self.F = np.array([[1, 1], [0, 1]], dtype=float)  # Assuming dt=1
        
        self.measurement_count = 0
    
    def predict(self, dt: float = 1.0) -> None:
        """
        Prediction step.
        
        Args:
            dt (float): Time step
        """
        # Update state transition matrix for given dt
        self.F[0, 1] = dt
        
        # Predict state
        self.state = self.F @ self.state
        
        # Predict covariance
        self.covariance = self.F @ self.covariance @ self.F.T + self.Q
    
    def update(self, measurement: float) -> Tuple[float, float]:
        """
        Update step.
        
        Args:
            measurement (float): Position measurement
            
        Returns:
            Tuple[float, float]: (position, velocity) estimates
        """
        # Calculate Kalman gain
        S = self.H @ self.covariance @ self.H.T + self.R
        K = self.covariance @ self.H.T @ np.linalg.inv(S)
        
        # Update state
        y = measurement - self.H @ self.state
        self.state = self.state + K @ y
        
        # Update covariance
        I = np.eye(2)
        self.covariance = (I - K @ self.H) @ self.covariance
        
        self.measurement_count += 1
        
        return self.state[0], self.state[1]  # position, velocity
    
    def get_state(self) -> Tuple[float, float]:
        """
        Get current state estimates.
        
        Returns:
            Tuple[float, float]: (position, velocity)
        """
        return self.state[0], self.state[1]


class KalmanFilter3D:
    """
    3D Kalman Filter for 3D position and velocity estimation.
    
    This class implements a 3D Kalman filter suitable for tracking
    position and velocity in three dimensions.
    """
    
    def __init__(self, process_noise: float = 0.001, measurement_noise: float = 0.1):
        """
        Initialize 3D Kalman filter.
        
        Args:
            process_noise (float): Process noise covariance
            measurement_noise (float): Measurement noise covariance
        """
        # State vector: [x, y, z, vx, vy, vz]
        self.state = np.zeros(6, dtype=float)
        
        # State covariance matrix
        self.covariance = np.eye(6, dtype=float)
        
        # Process noise covariance matrix
        self.Q = np.eye(6, dtype=float) * process_noise
        
        # Measurement noise covariance matrix
        self.R = np.eye(3, dtype=float) * measurement_noise
        
        # Measurement matrix (we measure position only)
        self.H = np.zeros((3, 6), dtype=float)
        self.H[0, 0] = 1  # x position
        self.H[1, 1] = 1  # y position
        self.H[2, 2] = 1  # z position
        
        # State transition matrix
        self.F = np.eye(6, dtype=float)
        self.F[0, 3] = 1  # x += vx
        self.F[1, 4] = 1  # y += vy
        self.F[2, 5] = 1  # z += vz
        
        self.measurement_count = 0
    
    def predict(self, dt: float = 1.0) -> None:
        """
        Prediction step.
        
        Args:
            dt (float): Time step
        """
        # Update state transition matrix for given dt
        self.F[0, 3] = dt
        self.F[1, 4] = dt
        self.F[2, 5] = dt
        
        # Predict state
        self.state = self.F @ self.state
        
        # Predict covariance
        self.covariance = self.F @ self.covariance @ self.F.T + self.Q
    
    def update(self, measurement: np.ndarray) -> Tuple[np.ndarray, np.ndarray]:
        """
        Update step.
        
        Args:
            measurement (np.ndarray): Position measurement [x, y, z]
            
        Returns:
            Tuple[np.ndarray, np.ndarray]: (position, velocity) estimates
        """
        # Calculate Kalman gain
        S = self.H @ self.covariance @ self.H.T + self.R
        K = self.covariance @ self.H.T @ np.linalg.inv(S)
        
        # Update state
        y = measurement - self.H @ self.state
        self.state = self.state + K @ y
        
        # Update covariance
        I = np.eye(6)
        self.covariance = (I - K @ self.H) @ self.covariance
        
        self.measurement_count += 1
        
        position = self.state[:3]
        velocity = self.state[3:]
        return position, velocity
    
    def get_state(self) -> Tuple[np.ndarray, np.ndarray]:
        """
        Get current state estimates.
        
        Returns:
            Tuple[np.ndarray, np.ndarray]: (position, velocity)
        """
        return self.state[:3], self.state[3:]


# Legacy compatibility class
class Kalman_filter(KalmanFilter):
    """
    Legacy Kalman filter class for backward compatibility.
    
    This class maintains the original interface while providing
    enhanced functionality from the new KalmanFilter class.
    """
    
    def __init__(self, process_noise_covariance: float, measurement_noise_covariance: float):
        """
        Initialize legacy Kalman filter.
        
        Args:
            process_noise_covariance (float): Process noise covariance (Q)
            measurement_noise_covariance (float): Measurement noise covariance (R)
        """
        config = KalmanConfig(
            process_noise=process_noise_covariance,
            measurement_noise=measurement_noise_covariance
        )
        super().__init__(config)
        
        # Legacy variable names for compatibility
        self.process_noise_covariance = process_noise_covariance
        self.measurement_noise_covariance = measurement_noise_covariance
        self.estimated_error_covariance = 1.0
        self.kalman_gain = 0.0
        self.posterior_error_covariance = 1.0
        self.posterior_estimate = 0.0
        self.previous_adc_value = 0.0
        self.current_measurement = 0.0
        self.previous_kalman_output = 0.0
    
    def kalman(self, adc_value: float) -> float:
        """
        Process ADC value through Kalman filter (legacy method).
        
        Args:
            adc_value (float): ADC measurement value
            
        Returns:
            float: Filtered output value
        """
        # Update legacy variables for compatibility
        self.current_measurement = adc_value
        self.previous_adc_value = adc_value
        
        # Use enhanced filter
        filtered_output = super().kalman(adc_value)
        
        # Update legacy variables
        self.previous_kalman_output = filtered_output
        self.kalman_gain = self._get_current_kalman_gain()
        
        return filtered_output


if __name__ == '__main__':
    print("Kalman Filter Test Suite")
    print("=" * 40)
    
    # Test 1D Kalman Filter
    print("\n1. Testing 1D Kalman Filter")
    print("-" * 30)
    
    kalman_1d = KalmanFilter()
    
    # Simulate noisy measurements
    true_value = 50.0
    noise_std = 5.0
    
    print("Processing measurements...")
    for i in range(100):
        # Add noise to true value
        measurement = true_value + np.random.normal(0, noise_std)
        filtered_value = kalman_1d.kalman(measurement)
        
        if i % 20 == 0:
            print(f"Step {i:3d}: Raw={measurement:6.2f}, Filtered={filtered_value:6.2f}")
    
    # Print statistics
    stats = kalman_1d.get_statistics()
    print(f"\nFilter Statistics:")
    print(f"Measurements processed: {stats['measurement_count']}")
    print(f"Average processing time: {stats['average_processing_time']*1000:.3f} ms")
    print(f"Value range: {stats['min_value']:.2f} to {stats['max_value']:.2f}")
    print(f"Mean value: {stats['mean_value']:.2f}")
    print(f"Variance: {stats['variance']:.2f}")
    print(f"Current Kalman gain: {stats['current_kalman_gain']:.4f}")
    
    # Test 2D Kalman Filter
    print("\n2. Testing 2D Kalman Filter")
    print("-" * 30)
    
    kalman_2d = KalmanFilter2D()
    
    # Simulate position with constant velocity
    position = 0.0
    velocity = 2.0
    
    print("Processing 2D measurements...")
    for i in range(20):
        # Update true position
        position += velocity
        
        # Add noise to measurement
        measurement = position + np.random.normal(0, 1.0)
        
        # Predict and update
        kalman_2d.predict(dt=1.0)
        est_position, est_velocity = kalman_2d.update(measurement)
        
        if i % 5 == 0:
            print(f"Step {i:2d}: True=({position:6.2f}, {velocity:6.2f}), "
                  f"Meas=({measurement:6.2f}), Est=({est_position:6.2f}, {est_velocity:6.2f})")
    
    # Test Legacy Compatibility
    print("\n3. Testing Legacy Compatibility")
    print("-" * 30)
    
    legacy_filter = Kalman_filter(0.001, 0.1)
    
    for i in range(10):
        measurement = 50 + np.random.normal(0, 5)
        output = legacy_filter.kalman(measurement)
        print(f"Legacy {i:2d}: Input={measurement:6.2f}, Output={output:6.2f}")
    
    print("\n✓ All Kalman filter tests completed successfully")