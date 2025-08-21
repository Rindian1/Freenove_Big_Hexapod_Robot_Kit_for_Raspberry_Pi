# -*- coding: utf-8 -*-
"""
Proximity Light Control System for Freenove Big Hexapod Robot Kit

This module implements a distance-based lighting system that provides visual
feedback based on obstacle proximity using ultrasonic sensors and RGB LEDs.
"""

import time
import threading
import math
from typing import Dict, List, Optional, Tuple, Any
from dataclasses import dataclass
from enum import Enum
from ultrasonic import Ultrasonic
from led import LED
from buzzer import Buzzer, BuzzerPattern


class LightState(Enum):
    """Light states for proximity indication."""
    GREEN = "green"      # Safe distance - clear path
    YELLOW = "yellow"    # Caution zone - medium distance
    RED = "red"          # Warning - close proximity
    OFF = "off"          # System disabled


@dataclass
class ProximityConfig:
    """Configuration for proximity light control system."""
    # Distance thresholds (in cm)
    red_threshold: float = 15.0      # Close proximity alert
    yellow_threshold: float = 30.0   # Caution zone
    green_threshold: float = 50.0    # Safe distance
    
    # Sensor settings
    sensor_polling_rate: float = 20.0  # Hz
    sensor_samples: int = 3           # Number of samples for averaging
    sensor_timeout: float = 1.0       # Timeout for sensor readings
    
    # Light settings
    enable_transitions: bool = True   # Smooth color transitions
    transition_duration: float = 0.5  # Transition time in seconds
    brightness_level: int = 255       # LED brightness (0-255)
    pulse_enabled: bool = True        # Enable pulsing for red warning
    
    # Alert settings
    enable_buzzer: bool = True        # Enable buzzer alerts
    buzzer_volume: float = 0.5        # Buzzer volume (0.0-1.0)
    
    # System settings
    auto_start: bool = True           # Auto-start monitoring
    debug_mode: bool = False          # Enable debug output


class ProximityLightControl:
    """
    Proximity-based light control system for hexapod robot.
    
    This system continuously monitors distance to obstacles and provides
    visual feedback through RGB LEDs based on proximity levels.
    """
    
    # Color definitions (RGB values)
    COLORS = {
        LightState.GREEN: [0, 255, 0],      # Green
        LightState.YELLOW: [255, 255, 0],   # Yellow
        LightState.RED: [255, 0, 0],        # Red
        LightState.OFF: [0, 0, 0]           # Off
    }
    
    def __init__(self, config: Optional[ProximityConfig] = None):
        """
        Initialize the proximity light control system.
        
        Args:
            config (ProximityConfig, optional): Configuration parameters
        """
        self.config = config or ProximityConfig()
        self.is_initialized = False
        self.is_running = False
        self.current_state = LightState.OFF
        self.previous_state = LightState.OFF
        self.current_distance = None
        self.distance_history = []
        self.max_history_size = 10
        
        # Initialize hardware components
        self.ultrasonic = None
        self.led = None
        self.buzzer = None
        
        # Threading components
        self.monitor_thread = None
        self.transition_thread = None
        self._lock = threading.Lock()
        self._stop_event = threading.Event()
        
        # Performance tracking
        self.last_update_time = 0.0
        self.update_count = 0
        self.error_count = 0
        
        self._initialize_hardware()
        
        if self.is_initialized and self.config.auto_start:
            self.start_monitoring()
    
    def _initialize_hardware(self) -> bool:
        """
        Initialize hardware components.
        
        Returns:
            bool: True if initialization successful
        """
        try:
            # Initialize ultrasonic sensor
            self.ultrasonic = Ultrasonic()
            
            # Initialize LED system
            self.led = LED()
            
            # Initialize buzzer if enabled
            if self.config.enable_buzzer:
                self.buzzer = Buzzer()
            
            self.is_initialized = True
            
            if self.config.debug_mode:
                print("Proximity light control system initialized successfully")
            
            return True
            
        except Exception as e:
            print(f"Error initializing proximity light control: {e}")
            self.is_initialized = False
            return False
    
    def start_monitoring(self) -> bool:
        """
        Start the proximity monitoring system.
        
        Returns:
            bool: True if started successfully
        """
        if not self.is_initialized:
            print("System not initialized")
            return False
        
        if self.is_running:
            print("Monitoring already running")
            return True
        
        try:
            self._stop_event.clear()
            self.is_running = True
            
            # Start monitoring thread
            self.monitor_thread = threading.Thread(
                target=self._monitor_loop,
                daemon=True,
                name="ProximityMonitor"
            )
            self.monitor_thread.start()
            
            if self.config.debug_mode:
                print("Proximity monitoring started")
            
            return True
            
        except Exception as e:
            print(f"Error starting monitoring: {e}")
            self.is_running = False
            return False
    
    def stop_monitoring(self) -> None:
        """Stop the proximity monitoring system."""
        if not self.is_running:
            return
        
        try:
            self._stop_event.set()
            self.is_running = False
            
            # Wait for thread to finish
            if self.monitor_thread and self.monitor_thread.is_alive():
                self.monitor_thread.join(timeout=2.0)
            
            # Turn off lights
            self._set_light_state(LightState.OFF)
            
            if self.config.debug_mode:
                print("Proximity monitoring stopped")
                
        except Exception as e:
            print(f"Error stopping monitoring: {e}")
    
    def _monitor_loop(self) -> None:
        """Main monitoring loop."""
        polling_interval = 1.0 / self.config.sensor_polling_rate
        
        while not self._stop_event.is_set():
            try:
                start_time = time.time()
                
                # Read distance
                distance = self._get_filtered_distance()
                
                if distance is not None:
                    # Update distance history
                    self._update_distance_history(distance)
                    
                    # Determine light state based on distance
                    new_state = self._determine_light_state(distance)
                    
                    # Update light state if changed
                    if new_state != self.current_state:
                        self._transition_to_state(new_state)
                    
                    # Update performance tracking
                    self.update_count += 1
                    self.last_update_time = time.time()
                    
                    if self.config.debug_mode and self.update_count % 50 == 0:
                        self._print_debug_info()
                
                # Calculate sleep time to maintain polling rate
                elapsed = time.time() - start_time
                sleep_time = max(0, polling_interval - elapsed)
                
                if sleep_time > 0:
                    time.sleep(sleep_time)
                    
            except Exception as e:
                self.error_count += 1
                print(f"Error in monitoring loop: {e}")
                time.sleep(0.1)
    
    def _get_filtered_distance(self) -> Optional[float]:
        """
        Get filtered distance reading.
        
        Returns:
            float: Filtered distance in cm, or None if error
        """
        try:
            # Take multiple samples for averaging
            samples = []
            for _ in range(self.config.sensor_samples):
                distance = self.ultrasonic.get_distance()
                if distance is not None:
                    samples.append(distance)
                time.sleep(0.01)  # Small delay between samples
            
            if not samples:
                return None
            
            # Apply median filter to remove outliers
            samples.sort()
            median_distance = samples[len(samples) // 2]
            
            # Apply moving average if we have history
            if self.distance_history:
                # Weighted average: 70% current, 30% history
                filtered_distance = 0.7 * median_distance + 0.3 * self.distance_history[-1]
            else:
                filtered_distance = median_distance
            
            return filtered_distance
            
        except Exception as e:
            if self.config.debug_mode:
                print(f"Error reading distance: {e}")
            return None
    
    def _update_distance_history(self, distance: float) -> None:
        """
        Update distance history for filtering.
        
        Args:
            distance (float): New distance reading
        """
        self.distance_history.append(distance)
        
        # Limit history size
        if len(self.distance_history) > self.max_history_size:
            self.distance_history.pop(0)
    
    def _determine_light_state(self, distance: float) -> LightState:
        """
        Determine light state based on distance.
        
        Args:
            distance (float): Distance in cm
            
        Returns:
            LightState: Appropriate light state
        """
        if distance <= self.config.red_threshold:
            return LightState.RED
        elif distance <= self.config.yellow_threshold:
            return LightState.YELLOW
        elif distance <= self.config.green_threshold:
            return LightState.GREEN
        else:
            return LightState.GREEN  # Safe distance
    
    def _transition_to_state(self, new_state: LightState) -> None:
        """
        Transition to a new light state.
        
        Args:
            new_state (LightState): New state to transition to
        """
        with self._lock:
            self.previous_state = self.current_state
            self.current_state = new_state
            
            if self.config.debug_mode:
                print(f"Transitioning from {self.previous_state.value} to {new_state.value}")
            
            # Handle state-specific actions
            if new_state == LightState.RED:
                self._handle_red_state()
            elif new_state == LightState.YELLOW:
                self._handle_yellow_state()
            elif new_state == LightState.GREEN:
                self._handle_green_state()
            elif new_state == LightState.OFF:
                self._handle_off_state()
    
    def _handle_red_state(self) -> None:
        """Handle red warning state."""
        try:
            # Set red light with pulsing if enabled
            if self.config.pulse_enabled:
                self._start_pulsing_red()
            else:
                self._set_light_state(LightState.RED)
            
            # Activate buzzer alert
            if self.config.enable_buzzer and self.buzzer:
                self.buzzer.play_pattern(BuzzerPattern.ALERT, block=False)
            
            if self.config.debug_mode:
                print("⚠️  RED WARNING: Close proximity detected!")
                
        except Exception as e:
            print(f"Error handling red state: {e}")
    
    def _handle_yellow_state(self) -> None:
        """Handle yellow caution state."""
        try:
            self._set_light_state(LightState.YELLOW)
            
            # Optional buzzer for yellow state
            if self.config.enable_buzzer and self.buzzer:
                self.buzzer.beep(0.1)
            
            if self.config.debug_mode:
                print("⚠️  YELLOW CAUTION: Medium distance detected")
                
        except Exception as e:
            print(f"Error handling yellow state: {e}")
    
    def _handle_green_state(self) -> None:
        """Handle green safe state."""
        try:
            self._set_light_state(LightState.GREEN)
            
            if self.config.debug_mode:
                print("✅ GREEN: Safe distance - clear path")
                
        except Exception as e:
            print(f"Error handling green state: {e}")
    
    def _handle_off_state(self) -> None:
        """Handle off state."""
        try:
            self._set_light_state(LightState.OFF)
            
            if self.config.debug_mode:
                print("🔴 OFF: System disabled")
                
        except Exception as e:
            print(f"Error handling off state: {e}")
    
    def _set_light_state(self, state: LightState) -> None:
        """
        Set the light to a specific state.
        
        Args:
            state (LightState): State to set
        """
        try:
            color = self.COLORS[state]
            
            if self.config.enable_transitions and state != LightState.OFF:
                # Start transition thread
                if self.transition_thread and self.transition_thread.is_alive():
                    return  # Transition already in progress
                
                self.transition_thread = threading.Thread(
                    target=self._transition_color,
                    args=(color,),
                    daemon=True
                )
                self.transition_thread.start()
            else:
                # Direct color change
                self.led.set_all_leds(color)
                
        except Exception as e:
            print(f"Error setting light state: {e}")
    
    def _transition_color(self, target_color: List[int]) -> None:
        """
        Smoothly transition to target color.
        
        Args:
            target_color (List[int]): Target RGB color
        """
        try:
            # Get current color
            current_color = self.COLORS.get(self.previous_state, [0, 0, 0])
            
            # Calculate transition steps
            steps = int(self.config.transition_duration * 30)  # 30 FPS
            step_duration = self.config.transition_duration / steps
            
            for i in range(steps + 1):
                if self._stop_event.is_set():
                    break
                
                # Interpolate between colors
                progress = i / steps
                interpolated_color = [
                    int(current_color[j] + (target_color[j] - current_color[j]) * progress)
                    for j in range(3)
                ]
                
                self.led.set_all_leds(interpolated_color)
                time.sleep(step_duration)
                
        except Exception as e:
            print(f"Error in color transition: {e}")
    
    def _start_pulsing_red(self) -> None:
        """Start pulsing red light for warning effect."""
        try:
            # Create pulsing effect
            base_color = self.COLORS[LightState.RED]
            max_brightness = self.config.brightness_level
            
            for brightness in range(50, max_brightness, 10):
                if self.current_state != LightState.RED:
                    break
                
                dimmed_color = [int(c * brightness / 255) for c in base_color]
                self.led.set_all_leds(dimmed_color)
                time.sleep(0.05)
            
            for brightness in range(max_brightness, 50, -10):
                if self.current_state != LightState.RED:
                    break
                
                dimmed_color = [int(c * brightness / 255) for c in base_color]
                self.led.set_all_leds(dimmed_color)
                time.sleep(0.05)
                
        except Exception as e:
            print(f"Error in pulsing red: {e}")
    
    def _print_debug_info(self) -> None:
        """Print debug information."""
        print(f"Proximity System - Updates: {self.update_count}, "
              f"Errors: {self.error_count}, "
              f"Distance: {self.current_distance:.1f}cm, "
              f"State: {self.current_state.value}")
    
    def get_status(self) -> Dict[str, Any]:
        """
        Get system status.
        
        Returns:
            Dict[str, Any]: Status information
        """
        return {
            'is_initialized': self.is_initialized,
            'is_running': self.is_running,
            'current_state': self.current_state.value,
            'current_distance': self.current_distance,
            'update_count': self.update_count,
            'error_count': self.error_count,
            'last_update_time': self.last_update_time,
            'config': {
                'red_threshold': self.config.red_threshold,
                'yellow_threshold': self.config.yellow_threshold,
                'green_threshold': self.config.green_threshold,
                'polling_rate': self.config.sensor_polling_rate,
                'enable_buzzer': self.config.enable_buzzer,
                'pulse_enabled': self.config.pulse_enabled
            }
        }
    
    def update_config(self, **kwargs) -> bool:
        """
        Update configuration parameters.
        
        Args:
            **kwargs: Configuration parameters to update
            
        Returns:
            bool: True if update successful
        """
        try:
            for key, value in kwargs.items():
                if hasattr(self.config, key):
                    setattr(self.config, key, value)
                else:
                    print(f"Unknown config parameter: {key}")
                    return False
            
            if self.config.debug_mode:
                print("Configuration updated")
            
            return True
            
        except Exception as e:
            print(f"Error updating configuration: {e}")
            return False
    
    def test_system(self) -> bool:
        """
        Test the proximity light control system.
        
        Returns:
            bool: True if test successful
        """
        try:
            print("Testing Proximity Light Control System...")
            
            # Test each light state
            for state in [LightState.GREEN, LightState.YELLOW, LightState.RED, LightState.OFF]:
                print(f"Testing {state.value} light...")
                self._set_light_state(state)
                time.sleep(1.0)
            
            # Test distance reading
            print("Testing distance sensor...")
            distance = self._get_filtered_distance()
            if distance is not None:
                print(f"Distance reading: {distance:.1f}cm")
            else:
                print("Distance sensor test failed")
                return False
            
            # Return to green state
            self._set_light_state(LightState.GREEN)
            
            print("✓ System test completed successfully")
            return True
            
        except Exception as e:
            print(f"System test failed: {e}")
            return False
    
    def __enter__(self):
        """Context manager entry."""
        return self
    
    def __exit__(self, exc_type, exc_value, traceback):
        """Context manager exit."""
        self.stop_monitoring()


if __name__ == '__main__':
    print("CURSOR Hexapod Spider Robot - Proximity Light Control System")
    print("=" * 60)
    
    # Create configuration
    config = ProximityConfig(
        red_threshold=15.0,
        yellow_threshold=30.0,
        green_threshold=50.0,
        sensor_polling_rate=20.0,
        enable_buzzer=True,
        pulse_enabled=True,
        debug_mode=True
    )
    
    # Create and test proximity control system
    with ProximityLightControl(config) as proximity_system:
        try:
            # Test the system
            if not proximity_system.test_system():
                print("System test failed, exiting...")
                exit(1)
            
            print("\nStarting proximity monitoring...")
            print("Move objects in front of the robot to test the system")
            print("Press Ctrl+C to stop")
            
            # Monitor for user input
            while True:
                time.sleep(1)
                
                # Print status every 10 seconds
                if int(time.time()) % 10 == 0:
                    status = proximity_system.get_status()
                    print(f"Status: {status['current_state']} - "
                          f"Distance: {status['current_distance']:.1f}cm")
        
        except KeyboardInterrupt:
            print("\nStopping proximity monitoring...")
        except Exception as e:
            print(f"Error: {e}")
        finally:
            print("Proximity light control system stopped")
