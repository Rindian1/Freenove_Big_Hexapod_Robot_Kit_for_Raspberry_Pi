# encoding: UTF-8
"""
PID Controller for Hexapod Robot

This module provides an enhanced PID controller with additional features for
smooth control of the hexapod robot's movements, including rate limiting,
output constraints, and anti-windup.
"""

import time
from typing import Optional, Tuple, List, Callable

class Incremental_PID:
    """
    Enhanced PID controller with rate limiting, output constraints, and anti-windup.
    
    Features:
    - Standard PID control with configurable gains
    - Output constraints to limit control signal range
    - Rate limiting to prevent abrupt changes
    - Anti-windup for integral term
    - Reset functionality for safe reuse
    - Optional callback for monitoring internal state
    """
    
    def __init__(self, P: float = 0.0, I: float = 0.0, D: float = 0.0, 
                 setpoint: float = 0.0, sample_time: float = 0.01):
        """
        Initialize the PID controller.
        
        Args:
            P: Proportional gain
            I: Integral gain
            D: Derivative gain
            setpoint: Initial setpoint/target value
            sample_time: Time between updates in seconds
        """
        self.Kp = float(P)
        self.Ki = float(I)
        self.Kd = float(D)
        self.setpoint = float(setpoint)
        self.sample_time = max(0.001, float(sample_time))  # Minimum 1ms
        
        # State variables
        self._last_time = None
        self._last_output = 0.0
        self._last_input = 0.0
        self._last_error = 0.0
        self._integral = 0.0
        self._derivative = 0.0
        
        # Anti-windup and constraints
        self._integral_saturation = 10.0
        self._output_min = -float('inf')
        self._output_max = float('inf')
        self._rate_limit = float('inf')
        
        # Monitoring
        self._monitor_callback = None
    
    def compute(self, input_val: float, dt: Optional[float] = None) -> float:
        """
        Compute PID output for the given input.
        
        Args:
            input_val: Current process variable/feedback value
            dt: Time delta since last update. If None, uses internal timing.
                
        Returns:
            Control output value
        """
        now = time.time()
        
        # Handle first call
        if self._last_time is None:
            self._last_time = now - self.sample_time
            self._last_input = input_val
            self._last_output = 0.0
            self._last_error = self.setpoint - input_val
            return 0.0
        
        # Calculate time delta
        if dt is None:
            dt = now - self._last_time
            if dt < 0.001:  # Minimum 1ms
                return self._last_output
        
        # Calculate error terms
        error = self.setpoint - input_val
        d_input = input_val - self._last_input
        
        # Calculate PID terms
        p_term = self.Kp * error
        
        # Update integral with anti-windup
        self._integral += self.Ki * error * dt
        self._integral = max(-self._integral_saturation, 
                           min(self._integral_saturation, self._integral))
        
        # Calculate derivative (using derivative on measurement for smoother response)
        d_term = -self.Kd * d_input / dt if dt > 0 else 0.0
        
        # Calculate output
        output = p_term + self._integral + d_term
        
        # Apply output constraints
        output = max(self._output_min, min(self._output_max, output))
        
        # Apply rate limiting
        if self._rate_limit < float('inf'):
            max_change = self._rate_limit * dt
            output = max(self._last_output - max_change, 
                        min(self._last_output + max_change, output))
        
        # Update state
        self._last_output = output
        self._last_input = input_val
        self._last_error = error
        self._last_time = now
        
        # Call monitor callback if set
        if self._monitor_callback:
            self._monitor_callback({
                'time': now,
                'setpoint': self.setpoint,
                'input': input_val,
                'output': output,
                'error': error,
                'p_term': p_term,
                'i_term': self._integral,
                'd_term': d_term
            })
        
        return output
    
    # Alias for backward compatibility
    PID_compute = compute
    
    def set_setpoint(self, setpoint: float) -> None:
        """Set the target setpoint."""
        self.setpoint = float(setpoint)
    
    def set_output_limits(self, min_val: float, max_val: float) -> None:
        """Set minimum and maximum output values."""
        if min_val > max_val:
            raise ValueError("Minimum output must be less than maximum output")
        self._output_min = float(min_val)
        self._output_max = float(max_val)
    
    def set_integral_saturation(self, saturation: float) -> None:
        """Set the integral windup saturation limit."""
        self._integral_saturation = abs(float(saturation))
    
    def set_rate_limit(self, max_rate: float) -> None:
        """Set maximum rate of change of the output (units/second)."""
        self._rate_limit = abs(float(max_rate))
    
    def set_sample_time(self, sample_time: float) -> None:
        """Set the sample time in seconds."""
        self.sample_time = max(0.001, float(sample_time))
    
    def set_monitor_callback(self, callback: Optional[Callable[[dict], None]]) -> None:
        """Set a callback to monitor internal PID state."""
        self._monitor_callback = callback
    
    def reset(self) -> None:
        """Reset the controller state."""
        self._last_time = None
        self._last_output = 0.0
        self._last_input = 0.0
        self._last_error = 0.0
        self._integral = 0.0
        self._derivative = 0.0
    
    # Backward compatibility methods
    def setKp(self, proportional_gain: float) -> None:
        """Set proportional gain (legacy method)."""
        self.Kp = float(proportional_gain)
    
    def setKi(self, integral_gain: float) -> None:
        """Set integral gain (legacy method)."""
        self.Ki = float(integral_gain)
    
    def setKd(self, derivative_gain: float) -> None:
        """Set derivative gain (legacy method)."""
        self.Kd = float(derivative_gain)
    
    def setI_saturation(self, saturation_val: float) -> None:
        """Set integral saturation (legacy method)."""
        self.set_integral_saturation(saturation_val)
    
    @property
    def components(self) -> Tuple[float, float, float]:
        """Get the individual PID components (P, I, D)."""
        return (self.Kp * self._last_error, 
                self._integral, 
                self.Kd * (self._last_error - self._last_error) / self.sample_time)
    
    def __str__(self) -> str:
        """String representation of the PID controller."""
        return (f"PID(Kp={self.Kp:.2f}, Ki={self.Ki:.2f}, Kd={self.Kd:.2f}, "
                f"setpoint={self.setpoint:.2f})")