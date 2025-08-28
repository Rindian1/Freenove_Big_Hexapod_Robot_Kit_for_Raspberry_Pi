# -*- coding: utf-8 -*-
"""
Buzzer module for Freenove Big Hexapod Robot Kit

This module handles buzzer control for audio feedback, alerts, and notifications
in the hexapod robot system. Supports various tone patterns and sequences.
"""

import time
import threading
from typing import List, Tuple, Optional
from gpiozero import OutputDevice, PWMOutputDevice
from enum import Enum


class BuzzerPattern(Enum):
    """Predefined buzzer patterns for different scenarios."""
    SINGLE_BEEP = "single_beep"
    DOUBLE_BEEP = "double_beep"
    TRIPLE_BEEP = "triple_beep"
    ALERT = "alert"
    SUCCESS = "success"
    ERROR = "error"
    STARTUP = "startup"
    SHUTDOWN = "shutdown"
    BATTERY_LOW = "battery_low"
    BATTERY_CRITICAL = "battery_critical"


class Buzzer:
    """
    Buzzer class for audio feedback and notifications.
    
    Provides functionality for simple on/off control, tone patterns,
    and custom audio sequences for robot status and alerts.
    """
    
    # GPIO Configuration
    BUZZER_PIN = 17
    
    # Timing configurations (in seconds)
    DEFAULT_BEEP_DURATION = 0.1
    DEFAULT_PAUSE_DURATION = 0.05
    DEFAULT_PATTERN_PAUSE = 0.2
    
    # Pattern definitions: (duration, pause, repeat_count)
    PATTERNS = {
        BuzzerPattern.SINGLE_BEEP: [(0.1, 0.0, 1)],
        BuzzerPattern.DOUBLE_BEEP: [(0.1, 0.1, 2)],
        BuzzerPattern.TRIPLE_BEEP: [(0.1, 0.1, 3)],
        BuzzerPattern.ALERT: [(0.2, 0.1, 3)],
        BuzzerPattern.SUCCESS: [(0.05, 0.05, 2), (0.1, 0.0, 1)],
        BuzzerPattern.ERROR: [(0.3, 0.1, 2)],
        BuzzerPattern.STARTUP: [(0.1, 0.1, 2), (0.2, 0.0, 1)],
        BuzzerPattern.SHUTDOWN: [(0.2, 0.0, 1), (0.1, 0.1, 2)],
        BuzzerPattern.BATTERY_LOW: [(0.1, 0.1, 2), (0.2, 0.2, 1)],
        BuzzerPattern.BATTERY_CRITICAL: [(0.1, 0.05, 5)]
    }
    
    def __init__(self, pin: int = BUZZER_PIN, use_pwm: bool = False):
        """
        Initialize the buzzer with specified configuration.
        
        Args:
            pin (int): GPIO pin number for buzzer (default: 17)
            use_pwm (bool): Use PWM for tone generation (default: False)
        """
        self.pin = pin
        self.use_pwm = use_pwm
        self.is_initialized = False
        
        # Initialize GPIO device
        if use_pwm:
            self.buzzer_device = PWMOutputDevice(pin, frequency=1000)
        else:
            self.buzzer_device = OutputDevice(pin)
        
        self.is_initialized = True
        
        # Threading for non-blocking operations
        self._pattern_thread = None
        self._is_playing = False
        
        print(f"Buzzer initialized on GPIO {pin} ({'PWM' if use_pwm else 'Digital'})")
    
    def set_state(self, state: bool) -> bool:
        """
        Set the buzzer on/off state.
        
        Args:
            state (bool): True to turn on, False to turn off
            
        Returns:
            bool: True if successful, False otherwise
        """
        if not self.is_initialized:
            print("Buzzer not initialized")
            return False
        
        try:
            if state:
                if self.use_pwm:
                    self.buzzer_device.on()
                else:
                    self.buzzer_device.on()
            else:
                self.buzzer_device.off()
            
            return True
            
        except Exception as e:
            print(f"Error setting buzzer state: {e}")
            return False
    
    def beep(self, duration: float = DEFAULT_BEEP_DURATION) -> bool:
        """
        Generate a single beep.
        
        Args:
            duration (float): Duration of the beep in seconds
            
        Returns:
            bool: True if successful, False otherwise
        """
        if not self.set_state(True):
            return False
        
        time.sleep(duration)
        return self.set_state(False)
    
    def beep_pattern(self, pattern: List[Tuple[float, float, int]], 
                    block: bool = True) -> bool:
        """
        Play a custom beep pattern.
        
        Args:
            pattern (List[Tuple]): List of (duration, pause, repeat_count) tuples
            block (bool): Block execution until pattern completes
            
        Returns:
            bool: True if successful, False otherwise
        """
        if not self.is_initialized:
            return False
        
        def play_pattern():
            try:
                for duration, pause, repeat_count in pattern:
                    for _ in range(repeat_count):
                        if not self._is_playing:
                            break
                        
                        self.set_state(True)
                        time.sleep(duration)
                        self.set_state(False)
                        
                        if pause > 0 and _ < repeat_count - 1:
                            time.sleep(pause)
                
                self._is_playing = False
                
            except Exception as e:
                print(f"Error playing buzzer pattern: {e}")
                self._is_playing = False
        
        if block:
            # Play pattern in current thread
            self._is_playing = True
            play_pattern()
        else:
            # Play pattern in background thread
            if self._pattern_thread and self._pattern_thread.is_alive():
                print("Pattern already playing")
                return False
            
            self._is_playing = True
            self._pattern_thread = threading.Thread(target=play_pattern)
            self._pattern_thread.daemon = True
            self._pattern_thread.start()
        
        return True
    
    def play_pattern(self, pattern: BuzzerPattern, block: bool = True) -> bool:
        """
        Play a predefined buzzer pattern.
        
        Args:
            pattern (BuzzerPattern): Predefined pattern to play
            block (bool): Block execution until pattern completes
            
        Returns:
            bool: True if successful, False otherwise
        """
        if pattern not in self.PATTERNS:
            print(f"Unknown pattern: {pattern}")
            return False
        
        return self.beep_pattern(self.PATTERNS[pattern], block)
    
    def morse_code(self, message: str, dot_duration: float = 0.1, 
                   dash_duration: float = 0.3, pause_duration: float = 0.1) -> bool:
        """
        Play a message in Morse code.
        
        Args:
            message (str): Message to encode (letters and numbers only)
            dot_duration (float): Duration of a dot in seconds
            dash_duration (float): Duration of a dash in seconds
            pause_duration (float): Duration of pause between characters
            
        Returns:
            bool: True if successful, False otherwise
        """
        # Morse code dictionary
        morse_dict = {
            'A': '.-', 'B': '-...', 'C': '-.-.', 'D': '-..', 'E': '.', 'F': '..-.',
            'G': '--.', 'H': '....', 'I': '..', 'J': '.---', 'K': '-.-', 'L': '.-..',
            'M': '--', 'N': '-.', 'O': '---', 'P': '.--.', 'Q': '--.-', 'R': '.-.',
            'S': '...', 'T': '-', 'U': '..-', 'V': '...-', 'W': '.--', 'X': '-..-',
            'Y': '-.--', 'Z': '--..',
            '0': '-----', '1': '.----', '2': '..---', '3': '...--', '4': '....-',
            '5': '.....', '6': '-....', '7': '--...', '8': '---..', '9': '----.'
        }
        
        pattern = []
        
        for char in message.upper():
            if char in morse_dict:
                morse_char = morse_dict[char]
                for symbol in morse_char:
                    if symbol == '.':
                        pattern.append((dot_duration, 0.05, 1))
                    elif symbol == '-':
                        pattern.append((dash_duration, 0.05, 1))
                
                # Add pause between characters
                if pause_duration > 0:
                    pattern.append((0, pause_duration, 1))
        
        return self.beep_pattern(pattern, block=True)
    
    def set_frequency(self, frequency: float) -> bool:
        """
        Set PWM frequency for tone generation (PWM mode only).
        
        Args:
            frequency (float): Frequency in Hz
            
        Returns:
            bool: True if successful, False otherwise
        """
        if not self.use_pwm:
            print("Frequency control requires PWM mode")
            return False
        
        if not self.is_initialized:
            return False
        
        try:
            self.buzzer_device.frequency = frequency
            return True
        except Exception as e:
            print(f"Error setting frequency: {e}")
            return False
    
    def play_tone(self, frequency: float, duration: float) -> bool:
        """
        Play a specific tone for a duration (PWM mode only).
        
        Args:
            frequency (float): Frequency in Hz
            duration (float): Duration in seconds
            
        Returns:
            bool: True if successful, False otherwise
        """
        if not self.use_pwm:
            print("Tone generation requires PWM mode")
            return False
        
        if not self.set_frequency(frequency):
            return False
        
        return self.beep(duration)
    
    def stop_pattern(self) -> bool:
        """
        Stop any currently playing pattern.
        
        Returns:
            bool: True if successful, False otherwise
        """
        self._is_playing = False
        
        if self._pattern_thread and self._pattern_thread.is_alive():
            self._pattern_thread.join(timeout=1.0)
        
        return self.set_state(False)
    
    def is_playing(self) -> bool:
        """
        Check if a pattern is currently playing.
        
        Returns:
            bool: True if pattern is playing, False otherwise
        """
        return self._is_playing
    
    def get_status(self) -> dict:
        """
        Get buzzer status information.
        
        Returns:
            dict: Buzzer status information
        """
        return {
            'initialized': self.is_initialized,
            'pin': self.pin,
            'pwm_mode': self.use_pwm,
            'is_playing': self._is_playing,
            'current_state': self.buzzer_device.value if self.is_initialized else None
        }
    
    def close(self) -> None:
        """Close the buzzer and release GPIO resources."""
        try:
            self.stop_pattern()
            if self.is_initialized:
                self.buzzer_device.close()
                self.is_initialized = False
                print("Buzzer closed")
        except Exception as e:
            print(f"Error closing buzzer: {e}")
    
    def __enter__(self):
        """Context manager entry point."""
        return self
    
    def __exit__(self, exc_type, exc_value, traceback):
        """Context manager exit point."""
        self.close()


def test_buzzer():
    """Test function to demonstrate buzzer functionality."""
    print("Testing Buzzer Module...")
    print("=" * 40)
    
    # Test digital buzzer
    print("\n1. Testing Digital Buzzer...")
    with Buzzer(use_pwm=False) as buzzer:
        # Test basic on/off
        print("  - Testing basic on/off...")
        buzzer.set_state(True)
        time.sleep(0.5)
        buzzer.set_state(False)
        
        # Test single beep
        print("  - Testing single beep...")
        buzzer.beep(0.2)
        
        # Test patterns
        print("  - Testing patterns...")
        for pattern in [BuzzerPattern.SINGLE_BEEP, BuzzerPattern.DOUBLE_BEEP, 
                       BuzzerPattern.SUCCESS]:
            print(f"    Playing {pattern.value}...")
            buzzer.play_pattern(pattern)
            time.sleep(0.5)
    
    # Test PWM buzzer (if supported)
    print("\n2. Testing PWM Buzzer...")
    try:
        with Buzzer(use_pwm=True) as buzzer:
            # Test tone generation
            print("  - Testing tone generation...")
            buzzer.play_tone(440, 0.5)  # A4 note
            time.sleep(0.2)
            buzzer.play_tone(523, 0.5)  # C5 note
            time.sleep(0.2)
            buzzer.play_tone(659, 0.5)  # E5 note
            
            # Test Morse code
            print("  - Testing Morse code...")
            buzzer.morse_code("SOS", dot_duration=0.1, dash_duration=0.3)
            
    except Exception as e:
        print(f"  PWM buzzer test failed: {e}")
    
    print("\n✓ Buzzer test completed successfully")


if __name__ == '__main__':
    print("Buzzer Module for Hexapod Robot")
    print("=" * 40)
    
    # Run test if executed directly
    test_buzzer()



