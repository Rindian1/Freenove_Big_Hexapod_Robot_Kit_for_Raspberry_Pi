# -*- coding: utf-8 -*-
"""
LED Control module for Freenove Big Hexapod Robot Kit

This module handles RGB LED strip control for the hexapod robot, providing
various lighting patterns and animations. It supports different PCB versions
and Raspberry Pi configurations.
"""

import time
import threading
from typing import List, Tuple, Optional, Dict, Any
from dataclasses import dataclass
from enum import Enum
from parameter import ParameterManager
from rpi_ledpixel import Freenove_RPI_WS281X
from spi_ledpixel import Freenove_SPI_LedPixel
import numpy as np


class LEDMode(Enum):
    """LED animation modes."""
    OFF = "0"
    SOLID_COLOR = "1"
    COLOR_WIPE = "2"
    THEATER_CHASE = "3"
    RAINBOW = "4"
    RAINBOW_CYCLE = "5"
    BREATHING = "6"
    STROBE = "7"
    FADE = "8"
    PULSE = "9"


@dataclass
class LEDConfig:
    """Configuration parameters for LED operation."""
    # Hardware settings
    led_count: int = 7
    brightness: int = 255
    default_color: List[int] = None
    
    # Animation settings
    default_wait_ms: int = 50
    breathing_speed: float = 0.05
    strobe_frequency: float = 10.0  # Hz
    fade_steps: int = 50
    
    # Performance settings
    max_brightness: int = 255
    min_brightness: int = 0
    enable_threading: bool = True
    
    def __post_init__(self):
        if self.default_color is None:
            self.default_color = [20, 0, 0]


class LED:
    """
    Enhanced LED control class for hexapod robot lighting.
    
    This class provides comprehensive LED strip control with support for:
    - Multiple PCB and Raspberry Pi versions
    - Various animation patterns
    - Threading for non-blocking animations
    - Color management and effects
    - Performance monitoring
    """
    
    def __init__(self, config: Optional[LEDConfig] = None):
        """
        Initialize the LED control system.
        
        Args:
            config (LEDConfig, optional): Configuration parameters
        """
        self.config = config or LEDConfig()
        self.is_initialized = False
        self.is_support_led_function = False
        
        # Hardware components
        self.param = ParameterManager()
        self.strip = None
        
        # Version information
        self.pcb_version = self.param.get_pcb_version()
        self.pi_version = self.param.get_raspberry_pi_version()
        
        # State variables
        self.led_mode = LEDMode.SOLID_COLOR.value
        self.current_color = self.config.default_color.copy()
        self.target_color = self.config.default_color.copy()
        
        # Animation control
        self.animation_thread = None
        self.animation_running = False
        self.animation_lock = threading.Lock()
        
        # Performance tracking
        self.frame_count = 0
        self.last_frame_time = 0.0
        self.animation_start_time = 0.0
        
        # Initialize hardware
        self._initialize_hardware()
        
        if self.is_support_led_function:
            print(f"LED system initialized - PCB v{self.pcb_version}, Pi v{self.pi_version}")
        else:
            print("LED function not supported for this hardware configuration")
    
    def _initialize_hardware(self) -> bool:
        """
        Initialize LED hardware based on PCB and Pi versions.
        
        Returns:
            bool: True if successful
        """
        try:
            # PCB v1.0 + Pi v1.0: Use RPI_WS281X
            if self.pcb_version == 1 and self.pi_version == 1:
                self.strip = Freenove_RPI_WS281X(
                    self.config.led_count, 
                    self.config.brightness, 
                    'RGB'
                )
                self.is_support_led_function = True
                print("Using RPI_WS281X driver")
            
            # PCB v2.0 + Pi v1.0/v2.0: Use SPI_LedPixel
            elif self.pcb_version == 2 and (self.pi_version == 1 or self.pi_version == 2):
                self.strip = Freenove_SPI_LedPixel(
                    self.config.led_count, 
                    self.config.brightness, 
                    'GRB'
                )
                self.is_support_led_function = True
                print("Using SPI_LedPixel driver")
            
            # PCB v1.0 + Pi v2.0: Not supported
            elif self.pcb_version == 1 and self.pi_version == 2:
                print("PCB Version 1.0 is not supported on Raspberry Pi 5")
                self.is_support_led_function = False
                return False
            
            else:
                print(f"Unsupported hardware combination: PCB v{self.pcb_version}, Pi v{self.pi_version}")
                self.is_support_led_function = False
                return False
            
            # Test LED strip
            if self.is_support_led_function:
                self.strip.set_led_rgb_data(0, [0, 0, 0])
                self.strip.show()
                self.is_initialized = True
            
            return self.is_support_led_function
            
        except Exception as e:
            print(f"Error initializing LED hardware: {e}")
            self.is_support_led_function = False
            return False
    
    def color_wipe(self, color: List[int], wait_ms: int = None) -> bool:
        """
        Wipe color across display a pixel at a time.
        
        Args:
            color (List[int]): RGB color [r, g, b]
            wait_ms (int, optional): Delay between pixels
            
        Returns:
            bool: True if successful
        """
        if not self._check_initialization():
            return False
        
        wait_ms = wait_ms or self.config.default_wait_ms
        
        try:
            for i in range(self.strip.get_led_count()):
                self.strip.set_led_rgb_data(i, color)
                self.strip.show()
                time.sleep(wait_ms / 1000.0)
            return True
        except Exception as e:
            print(f"Error in color_wipe: {e}")
            return False
    
    def wheel(self, pos: int) -> List[int]:
        """
        Generate rainbow colors across 0-255 positions.
        
        Args:
            pos (int): Position (0-255)
            
        Returns:
            List[int]: RGB color [r, g, b]
        """
        if pos < 0 or pos > 255:
            return [0, 0, 0]
        elif pos < 85:
            return [pos * 3, 255 - pos * 3, 0]
        elif pos < 170:
            pos -= 85
            return [255 - pos * 3, 0, pos * 3]
        else:
            pos -= 170
            return [0, pos * 3, 255 - pos * 3]
    
    def rainbow(self, wait_ms: int = 20, iterations: int = 1) -> bool:
        """
        Draw rainbow that fades across all pixels at once.
        
        Args:
            wait_ms (int): Delay between frames
            iterations (int): Number of iterations
            
        Returns:
            bool: True if successful
        """
        if not self._check_initialization():
            return False
        
        try:
            for j in range(256 * iterations):
                for i in range(self.strip.get_led_count()):
                    self.strip.set_led_rgb_data(i, self.wheel((i + j) & 255))
                self.strip.show()
                time.sleep(wait_ms / 1000.0)
            return True
        except Exception as e:
            print(f"Error in rainbow: {e}")
            return False
    
    def rainbow_cycle(self, wait_ms: int = 20, iterations: int = 1) -> bool:
        """
        Draw rainbow that uniformly distributes itself across all pixels.
        
        Args:
            wait_ms (int): Delay between frames
            iterations (int): Number of iterations
            
        Returns:
            bool: True if successful
        """
        if not self._check_initialization():
            return False
        
        try:
            for j in range(256 * iterations):
                for i in range(self.strip.get_led_count()):
                    self.strip.set_led_rgb_data(
                        i, 
                        self.wheel((int(i * 256 / self.strip.get_led_count()) + j) & 255)
                    )
                self.strip.show()
                time.sleep(wait_ms / 1000.0)
            return True
        except Exception as e:
            print(f"Error in rainbow_cycle: {e}")
            return False
    
    def theater_chase(self, color: List[int], wait_ms: int = 50) -> bool:
        """
        Movie theater light style chaser animation.
        
        Args:
            color (List[int]): RGB color [r, g, b]
            wait_ms (int): Delay between frames
            
        Returns:
            bool: True if successful
        """
        if not self._check_initialization():
            return False
        
        try:
            led_count = self.strip.get_led_count()
            for i in range(led_count):
                for q in range(3):
                    self.strip.set_led_rgb_data((i + q * 4) % led_count, color)
                self.strip.show()
                time.sleep(wait_ms / 1000.0)
                for q in range(3):
                    self.strip.set_led_rgb_data((i + q * 4) % led_count, [0, 0, 0])
            return True
        except Exception as e:
            print(f"Error in theater_chase: {e}")
            return False
    
    def breathing(self, color: List[int], duration: float = 2.0) -> bool:
        """
        Breathing effect with smooth brightness transitions.
        
        Args:
            color (List[int]): RGB color [r, g, b]
            duration (float): Duration of one breath cycle in seconds
            
        Returns:
            bool: True if successful
        """
        if not self._check_initialization():
            return False
        
        try:
            steps = int(duration * 1000 / 20)  # 20ms steps
            for i in range(steps):
                # Calculate brightness (sine wave)
                brightness = 0.5 + 0.5 * np.sin(2 * np.pi * i / steps)
                
                # Apply brightness to color
                dimmed_color = [int(c * brightness) for c in color]
                
                # Set all LEDs
                for j in range(self.strip.get_led_count()):
                    self.strip.set_led_rgb_data(j, dimmed_color)
                self.strip.show()
                
                time.sleep(0.02)
            return True
        except Exception as e:
            print(f"Error in breathing: {e}")
            return False
    
    def strobe(self, color: List[int], frequency: float = 10.0, duration: float = 1.0) -> bool:
        """
        Strobe light effect.
        
        Args:
            color (List[int]): RGB color [r, g, b]
            frequency (float): Strobe frequency in Hz
            duration (float): Duration in seconds
            
        Returns:
            bool: True if successful
        """
        if not self._check_initialization():
            return False
        
        try:
            period = 1.0 / frequency
            cycles = int(duration * frequency)
            
            for _ in range(cycles):
                # Flash on
                for i in range(self.strip.get_led_count()):
                    self.strip.set_led_rgb_data(i, color)
                self.strip.show()
                time.sleep(period / 2)
                
                # Flash off
                for i in range(self.strip.get_led_count()):
                    self.strip.set_led_rgb_data(i, [0, 0, 0])
                self.strip.show()
                time.sleep(period / 2)
            
            return True
        except Exception as e:
            print(f"Error in strobe: {e}")
            return False
    
    def fade_to_color(self, target_color: List[int], steps: int = 50) -> bool:
        """
        Smooth fade to target color.
        
        Args:
            target_color (List[int]): Target RGB color [r, g, b]
            steps (int): Number of fade steps
            
        Returns:
            bool: True if successful
        """
        if not self._check_initialization():
            return False
        
        try:
            start_color = self.current_color.copy()
            
            for i in range(steps + 1):
                # Interpolate between start and target colors
                ratio = i / steps
                fade_color = [
                    int(start_color[j] + (target_color[j] - start_color[j]) * ratio)
                    for j in range(3)
                ]
                
                # Set all LEDs
                for j in range(self.strip.get_led_count()):
                    self.strip.set_led_rgb_data(j, fade_color)
                self.strip.show()
                
                time.sleep(0.02)
            
            self.current_color = target_color.copy()
            return True
        except Exception as e:
            print(f"Error in fade_to_color: {e}")
            return False
    
    def led_index(self, index: int, r: int, g: int, b: int) -> bool:
        """
        Set specific LEDs based on bit pattern.
        
        Args:
            index (int): Bit pattern for LED selection
            r, g, b (int): RGB color values
            
        Returns:
            bool: True if successful
        """
        if not self._check_initialization():
            return False
        
        try:
            change_color = [r, g, b]
            for i in range(8):
                if index & 0x01 == 1:
                    self.strip.set_led_rgb_data(i, change_color)
                index = index >> 1
            self.strip.show()
            return True
        except Exception as e:
            print(f"Error in led_index: {e}")
            return False
    
    def set_all_leds(self, color: List[int]) -> bool:
        """
        Set all LEDs to the same color.
        
        Args:
            color (List[int]): RGB color [r, g, b]
            
        Returns:
            bool: True if successful
        """
        if not self._check_initialization():
            return False
        
        try:
            for i in range(self.strip.get_led_count()):
                self.strip.set_led_rgb_data(i, color)
            self.strip.show()
            self.current_color = color.copy()
            return True
        except Exception as e:
            print(f"Error in set_all_leds: {e}")
            return False
    
    def set_led(self, index: int, color: List[int]) -> bool:
        """
        Set a specific LED to a color.
        
        Args:
            index (int): LED index
            color (List[int]): RGB color [r, g, b]
            
        Returns:
            bool: True if successful
        """
        if not self._check_initialization():
            return False
        
        try:
            if 0 <= index < self.strip.get_led_count():
                self.strip.set_led_rgb_data(index, color)
                self.strip.show()
                return True
            else:
                print(f"LED index {index} out of range")
                return False
        except Exception as e:
            print(f"Error in set_led: {e}")
            return False
    
    def process_light_command(self, data: List[str]) -> bool:
        """
        Process light commands to control LED behavior.
        
        Args:
            data (List[str]): Command data
            
        Returns:
            bool: True if successful
        """
        if not self._check_initialization():
            return False
        
        try:
            old_mode = self.led_mode
            
            # Parse command data
            if len(data) < 4:
                self.led_mode = data[1]
            else:
                for i in range(3):
                    self.current_color[i] = int(data[i + 1])
            
            # Stop current animation
            self.stop_animation()
            
            # Execute mode
            if self.led_mode == LEDMode.OFF.value:
                self.set_all_leds([0, 0, 0])
                self.led_mode = old_mode
                return True
            
            elif self.led_mode == LEDMode.SOLID_COLOR.value:
                return self.led_index(0x7f, *self.current_color)
            
            elif self.led_mode == LEDMode.COLOR_WIPE.value:
                return self._start_animation_thread(self._color_wipe_loop)
            
            elif self.led_mode == LEDMode.THEATER_CHASE.value:
                return self._start_animation_thread(self._theater_chase_loop)
            
            elif self.led_mode == LEDMode.RAINBOW.value:
                return self._start_animation_thread(self._rainbow_loop)
            
            elif self.led_mode == LEDMode.RAINBOW_CYCLE.value:
                return self._start_animation_thread(self._rainbow_cycle_loop)
            
            elif self.led_mode == LEDMode.BREATHING.value:
                return self._start_animation_thread(self._breathing_loop)
            
            elif self.led_mode == LEDMode.STROBE.value:
                return self._start_animation_thread(self._strobe_loop)
            
            elif self.led_mode == LEDMode.FADE.value:
                return self._start_animation_thread(self._fade_loop)
            
            else:
                print(f"Unknown LED mode: {self.led_mode}")
                return False
                
        except Exception as e:
            print(f"Error processing light command: {e}")
            return False
    
    def _start_animation_thread(self, animation_func) -> bool:
        """
        Start animation in a separate thread.
        
        Args:
            animation_func: Animation function to run
            
        Returns:
            bool: True if started successfully
        """
        if not self.config.enable_threading:
            return False
        
        with self.animation_lock:
            if self.animation_running:
                return True
            
            self.animation_running = True
            self.animation_thread = threading.Thread(target=animation_func, daemon=True)
            self.animation_thread.start()
            return True
    
    def stop_animation(self) -> None:
        """Stop current animation."""
        with self.animation_lock:
            self.animation_running = False
            if self.animation_thread:
                self.animation_thread.join(timeout=1.0)
    
    def _color_wipe_loop(self) -> None:
        """Color wipe animation loop."""
        while self.animation_running:
            self.color_wipe([255, 0, 0])   # Red
            if not self.animation_running:
                break
            self.color_wipe([0, 255, 0])   # Green
            if not self.animation_running:
                break
            self.color_wipe([0, 0, 255])   # Blue
    
    def _theater_chase_loop(self) -> None:
        """Theater chase animation loop."""
        while self.animation_running:
            self.theater_chase(self.current_color)
    
    def _rainbow_loop(self) -> None:
        """Rainbow animation loop."""
        while self.animation_running:
            self.rainbow(wait_ms=5, iterations=1)
    
    def _rainbow_cycle_loop(self) -> None:
        """Rainbow cycle animation loop."""
        while self.animation_running:
            self.rainbow_cycle(wait_ms=5, iterations=1)
    
    def _breathing_loop(self) -> None:
        """Breathing animation loop."""
        while self.animation_running:
            self.breathing(self.current_color)
    
    def _strobe_loop(self) -> None:
        """Strobe animation loop."""
        while self.animation_running:
            self.strobe(self.current_color)
    
    def _fade_loop(self) -> None:
        """Fade animation loop."""
        while self.animation_running:
            # Fade to random color
            import random
            target_color = [random.randint(0, 255) for _ in range(3)]
            self.fade_to_color(target_color)
            if self.animation_running:
                time.sleep(1.0)
    
    def _check_initialization(self) -> bool:
        """
        Check if LED system is properly initialized.
        
        Returns:
            bool: True if initialized
        """
        if not self.is_initialized or not self.is_support_led_function:
            print("LED system not properly initialized")
            return False
        return True
    
    def get_status(self) -> Dict[str, Any]:
        """
        Get LED system status.
        
        Returns:
            Dict[str, Any]: Status information
        """
        return {
            'is_initialized': self.is_initialized,
            'is_support_led_function': self.is_support_led_function,
            'pcb_version': self.pcb_version,
            'pi_version': self.pi_version,
            'led_mode': self.led_mode,
            'current_color': self.current_color.copy(),
            'animation_running': self.animation_running,
            'led_count': self.strip.get_led_count() if self.strip else 0
        }
    
    def cleanup(self) -> None:
        """Clean up LED resources."""
        self.stop_animation()
        if self.is_initialized:
            self.set_all_leds([0, 0, 0])
        print("LED system cleaned up")
    
    def __enter__(self):
        """Context manager entry."""
        return self
    
    def __exit__(self, exc_type, exc_value, traceback):
        """Context manager exit."""
        self.cleanup()


if __name__ == '__main__':
    print("LED Control System Test")
    print("=" * 30)
    
    # Create LED instance
    led = LED()
    
    try:
        if not led.is_support_led_function:
            print("LED function not supported on this hardware")
            exit(1)
        
        print("Testing LED animations...")
        
        # Test basic animations
        print("1. Color wipe animation")
        led.color_wipe([255, 0, 0])  # Red
        led.color_wipe([0, 255, 0])  # Green
        led.color_wipe([0, 0, 255])  # Blue
        
        print("2. Rainbow animation")
        led.rainbow(wait_ms=5, iterations=1)
        
        print("3. Rainbow cycle animation")
        led.rainbow_cycle(wait_ms=5, iterations=1)
        
        print("4. Theater chase animation")
        led.theater_chase([255, 255, 0], wait_ms=50)  # Yellow
        
        print("5. Breathing animation")
        led.breathing([0, 255, 255], duration=2.0)  # Cyan
        
        print("6. Strobe animation")
        led.strobe([255, 0, 255], frequency=5.0, duration=1.0)  # Magenta
        
        print("7. Fade animation")
        led.fade_to_color([255, 255, 255], steps=50)  # White
        
        # Turn off all LEDs
        led.color_wipe([0, 0, 0], 10)
        
        print("All LED tests completed successfully")
        
    except KeyboardInterrupt:
        print("\nTest interrupted by user")
        led.cleanup()
    except Exception as e:
        print(f"Test failed: {e}")
        led.cleanup()
    finally:
        print("LED test completed")