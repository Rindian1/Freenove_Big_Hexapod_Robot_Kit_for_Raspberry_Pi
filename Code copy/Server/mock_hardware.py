"""
Mock Hardware System for Freenove Big Hexapod Robot Kit
This module provides mock implementations of all hardware components
for development and testing on non-Raspberry Pi systems (like macOS).
"""

import time
import random
import math
import threading
from typing import Tuple, List, Optional
import numpy as np

# ============================================================================
# Mock SMBus for I2C Communication
# ============================================================================

class MockSMBus:
    """Mock SMBus class that simulates I2C communication."""
    
    def __init__(self, bus_number):
        self.bus_number = bus_number
        self.devices = {
            0x48: MockADCDevice(),  # ADC device
            0x68: MockMPU6050(),   # MPU6050 IMU
            0x40: MockPCA9685(),   # PCA9685 servo controller
        }
        print(f"Mock I2C bus {bus_number} initialized (simulation mode)")
    
    def read_byte_data(self, device, register):
        """Mock read operation."""
        if device in self.devices:
            return self.devices[device].read_register(register)
        return random.randint(0, 255)
    
    def read_byte(self, device):
        """Mock read operation."""
        if device in self.devices:
            return self.devices[device].read_byte()
        return random.randint(0, 255)
    
    def write_byte(self, device, value):
        """Mock write operation."""
        if device in self.devices:
            self.devices[device].write_register(0, value)
    
    def write_byte_data(self, device, register, value):
        """Mock write operation."""
        if device in self.devices:
            self.devices[device].write_register(register, value)
    
    def close(self):
        """Mock close operation."""
        print("Mock I2C bus closed")

# ============================================================================
# Mock Hardware Devices
# ============================================================================

class MockADCDevice:
    """Mock ADC device (ADS7830)."""
    
    def __init__(self):
        self.registers = {}
        self.voltage_values = {
            0: 7.5,  # Battery 1 voltage
            4: 8.2,  # Battery 2 voltage
        }
    
    def read_register(self, register):
        return random.randint(100, 200)
    
    def read_byte(self):
        return random.randint(100, 200)
    
    def write_register(self, register, value):
        self.registers[register] = value

class MockMPU6050:
    """Mock MPU6050 IMU sensor."""
    
    def __init__(self):
        self.registers = {}
        self.accel_data = [0, 0, 0]
        self.gyro_data = [0, 0, 0]
        self.temp = 25.0
        
        # Start a thread to simulate sensor movement
        self.running = True
        self.thread = threading.Thread(target=self._simulate_movement)
        self.thread.daemon = True
        self.thread.start()
    
    def _simulate_movement(self):
        """Simulate realistic IMU data."""
        while self.running:
            # Simulate slight movement and drift
            self.accel_data = [
                random.gauss(0, 0.1),
                random.gauss(0, 0.1),
                random.gauss(9.8, 0.1)  # Gravity
            ]
            self.gyro_data = [
                random.gauss(0, 0.01),
                random.gauss(0, 0.01),
                random.gauss(0, 0.01)
            ]
            self.temp += random.gauss(0, 0.1)
            time.sleep(0.01)
    
    def read_register(self, register):
        # Return appropriate mock data based on register
        if register == 0x3B:  # ACCEL_XOUT_H
            return int(self.accel_data[0] * 16384) >> 8
        elif register == 0x3C:  # ACCEL_XOUT_L
            return int(self.accel_data[0] * 16384) & 0xFF
        elif register == 0x41:  # TEMP_OUT_H
            return int((self.temp + 521) / 340 + 36.53) >> 8
        else:
            return random.randint(0, 255)
    
    def read_byte(self):
        return random.randint(0, 255)
    
    def write_register(self, register, value):
        self.registers[register] = value

class MockPCA9685:
    """Mock PCA9685 servo controller."""
    
    def __init__(self):
        self.registers = {}
        self.servo_positions = [90] * 16  # 16 servos, default 90 degrees
    
    def read_register(self, register):
        if register == 0x00:  # MODE1
            return 0x01
        elif register == 0x01:  # MODE2
            return 0x04
        else:
            return 0x00
    
    def read_byte(self):
        return 0x00
    
    def write_register(self, register, value):
        self.registers[register] = value

# ============================================================================
# Mock WS281X LED Library
# ============================================================================

class MockColor:
    """Mock Color class for WS281X LEDs."""
    
    def __init__(self, r, g, b, w=0):
        self.r = r
        self.g = g
        self.b = b
        self.w = w

class MockAdafruit_NeoPixel:
    """Mock NeoPixel LED strip."""
    
    def __init__(self, num, pin, freq_hz=800000, dma=10, invert=False, 
                 brightness=255, channel=0, strip_type=None):
        self.num = num
        self.pin = pin
        self.brightness = brightness
        self.pixels = [MockColor(0, 0, 0)] * num
        print(f"Mock NeoPixel strip initialized with {num} LEDs")
    
    def begin(self):
        print("Mock NeoPixel strip started")
    
    def setPixelColor(self, n, color):
        if 0 <= n < self.num:
            self.pixels[n] = color
    
    def show(self):
        # Simulate LED update
        pass
    
    def setBrightness(self, brightness):
        self.brightness = brightness
    
    def clear(self):
        self.pixels = [MockColor(0, 0, 0)] * self.num

# ============================================================================
# Mock Camera Library
# ============================================================================

class MockPicamera2:
    """Mock Picamera2 class."""
    
    def __init__(self):
        self.camera_config = {}
        self.preview_config = {}
        print("Mock Picamera2 initialized")
    
    def configure(self, config):
        self.camera_config = config
    
    def configure_preview(self, config):
        self.preview_config = config
    
    def start(self):
        print("Mock camera started")
    
    def stop(self):
        print("Mock camera stopped")
    
    def capture_array(self):
        # Return a mock image array
        return np.random.randint(0, 255, (480, 640, 3), dtype=np.uint8)

class MockPreview:
    """Mock Preview class."""
    
    def __init__(self, camera):
        self.camera = camera
        print("Mock camera preview initialized")
    
    def start(self):
        print("Mock camera preview started")
    
    def stop(self):
        print("Mock camera preview stopped")

class MockJpegEncoder:
    """Mock JPEG encoder for camera."""
    
    def __init__(self, camera):
        self.camera = camera
        print("Mock JPEG encoder initialized")
    
    def encode(self, stream):
        print("Mock JPEG encoding")
        return b'fake_jpeg_data'

class MockH264Encoder:
    """Mock H264 encoder for camera."""
    
    def __init__(self, camera):
        self.camera = camera
        print("Mock H264 encoder initialized")
    
    def encode(self, stream):
        print("Mock H264 encoding")
        return b'fake_h264_data'

class MockMPU6050Class:
    """Mock MPU6050 class for the mpu6050 module."""
    
    def __init__(self, address=0x68, bus=1):
        self.address = address
        self.bus = bus
        self.accel_data = [0, 0, 0]
        self.gyro_data = [0, 0, 0]
        self.temp = 25.0
        print(f"Mock MPU6050 initialized at address 0x{address:02X}")
    
    def get_accel_data(self):
        return self.accel_data
    
    def get_gyro_data(self):
        return self.gyro_data
    
    def get_temp(self):
        return self.temp

# ============================================================================
# Mock SPI Device
# ============================================================================

class MockSpiDev:
    """Mock SPI device for development."""
    
    def __init__(self):
        self.bus = 0
        self.device = 0
        self.max_speed_hz = 500000
        print("Mock SPI device initialized")
    
    def open(self, bus, device):
        self.bus = bus
        self.device = device
        print(f"Mock SPI opened: bus={bus}, device={device}")
    
    def close(self):
        print("Mock SPI device closed")
    
    def writebytes(self, data):
        print(f"Mock SPI write: {data}")
    
    def readbytes(self, length):
        return [random.randint(0, 255) for _ in range(length)]
    
    def xfer(self, data):
        print(f"Mock SPI transfer: {data}")
        return [random.randint(0, 255) for _ in range(len(data))]

# ============================================================================
# Hardware Detection and Import System
# ============================================================================

def detect_hardware():
    """Detect if we're running on real hardware or need mocks."""
    try:
        import smbus
        # Try to access I2C bus
        bus = smbus.SMBus(1)
        bus.close()
        return True  # Real hardware detected
    except (ImportError, FileNotFoundError, OSError):
        return False  # Need mocks

def setup_mock_imports():
    """Set up mock imports for hardware modules."""
    import sys
    import types
    
    # Create mock modules
    mock_smbus = types.ModuleType('smbus')
    mock_smbus.SMBus = MockSMBus
    sys.modules['smbus'] = mock_smbus
    
    mock_rpi_ws281x = types.ModuleType('rpi_ws281x')
    mock_rpi_ws281x.Adafruit_NeoPixel = MockAdafruit_NeoPixel
    mock_rpi_ws281x.Color = MockColor
    sys.modules['rpi_ws281x'] = mock_rpi_ws281x
    
    mock_picamera2 = types.ModuleType('picamera2')
    mock_picamera2.Picamera2 = MockPicamera2
    mock_picamera2.Preview = MockPreview
    sys.modules['picamera2'] = mock_picamera2
    
    # Mock SPI device
    mock_spidev = types.ModuleType('spidev')
    mock_spidev.SpiDev = MockSpiDev
    sys.modules['spidev'] = mock_spidev
    
    # Mock GPIO
    mock_RPi = types.ModuleType('RPi')
    mock_GPIO = types.ModuleType('RPi.GPIO')
    mock_GPIO.setmode = lambda x: None
    mock_GPIO.setup = lambda x, y: None
    mock_GPIO.OUT = 'OUT'
    mock_GPIO.IN = 'IN'
    mock_GPIO.HIGH = 1
    mock_GPIO.LOW = 0
    mock_GPIO.output = lambda x, y: None
    mock_GPIO.input = lambda x: 0
    mock_GPIO.cleanup = lambda: None
    mock_RPi.GPIO = mock_GPIO
    sys.modules['RPi'] = mock_RPi
    sys.modules['RPi.GPIO'] = mock_GPIO
    
    # Mock picamera2 submodules
    mock_encoders = types.ModuleType('picamera2.encoders')
    mock_encoders.JpegEncoder = MockJpegEncoder
    mock_encoders.H264Encoder = MockH264Encoder
    sys.modules['picamera2.encoders'] = mock_encoders
    
    mock_preview = types.ModuleType('picamera2.preview')
    mock_preview.Preview = MockPreview
    sys.modules['picamera2.preview'] = mock_preview
    
    # Mock mpu6050
    mock_mpu6050_module = types.ModuleType('mpu6050')
    mock_mpu6050_module.mpu6050 = MockMPU6050Class
    sys.modules['mpu6050'] = mock_mpu6050_module
    
    # Mock parameter module
    mock_parameter = types.ModuleType('parameter')
    from parameter_mock import ParameterManager
    mock_parameter.ParameterManager = ParameterManager
    sys.modules['parameter'] = mock_parameter

# ============================================================================
# Main Setup Function
# ============================================================================

def setup_mock_hardware():
    """Set up mock hardware system."""
    if not detect_hardware():
        print("Real hardware not detected, setting up mock hardware...")
        setup_mock_imports()
        print("Mock hardware system ready!")
        return True
    else:
        print("Real hardware detected, using actual hardware.")
        return False

if __name__ == '__main__':
    # Test the mock system
    setup_mock_hardware()
    
    # Test mock SMBus
    bus = MockSMBus(1)
    print(f"ADC reading: {bus.read_byte_data(0x48, 0)}")
    bus.close()
    
    # Test mock NeoPixel
    strip = MockAdafruit_NeoPixel(16, 18)
    strip.setPixelColor(0, MockColor(255, 0, 0))
    strip.show()
    
    # Test mock camera
    camera = MockPicamera2()
    image = camera.capture_array()
    print(f"Mock image shape: {image.shape}")
    
    print("Mock hardware system test completed!")
