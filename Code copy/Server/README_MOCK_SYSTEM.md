# Freenove Big Hexapod Robot Kit - Mock Hardware System

## Overview

This mock hardware system allows you to develop and test the Freenove Big Hexapod Robot Kit software on non-Raspberry Pi systems (like macOS) without requiring the actual hardware.

## What It Does

The mock system simulates all the hardware components that the robot software expects:

### Hardware Components Simulated:
- **I2C Bus** (`smbus`) - for sensor communication
- **ADC (ADS7830)** - battery voltage monitoring
- **IMU (MPU6050)** - motion sensing
- **Servo Controller (PCA9685)** - leg motor control
- **LED Strip (WS281X)** - RGB lighting
- **Camera (Picamera2)** - video capture
- **SPI Device** - additional sensor communication
- **GPIO** - general purpose input/output
- **Parameter Manager** - hardware configuration

## How to Use

### 1. Quick Start
```bash
# Test the mock system
python test_mock_system.py

# Run the server with mock hardware
python start_server.py
```

### 2. Development Mode
```bash
# Set up mock environment in your code
from mock_hardware import setup_mock_hardware
setup_mock_hardware()

# Now you can import and use robot modules
from adc import ADC
from imu import IMU
from led import Led
```

## What Works

✅ **ADC Module** - Simulates battery voltage readings  
✅ **Parameter Manager** - Creates default configuration files  
✅ **Basic I2C Communication** - Mock SMBus operations  
✅ **Thread Management** - Server threading utilities  
✅ **Command System** - Robot command definitions  

## What's Limited

⚠️ **IMU Module** - Missing some sensor methods  
⚠️ **LED Module** - SPI communication needs adjustment  
⚠️ **Buzzer Module** - GPIO pin factory issues  
⚠️ **Camera Module** - Complex picamera2 dependencies  

## File Structure

```
Server/
├── mock_hardware.py          # Main mock system
├── parameter_mock.py         # Mock parameter manager
├── test_mock_system.py       # Test script
├── start_server.py           # Server startup script
└── README_MOCK_SYSTEM.md     # This file
```

## Configuration

The system automatically creates a `params.json` file with default settings:
```json
{
    "Pcb_Version": 2,
    "Pi_Version": 1
}
```

## Real vs Mock Comparison

### Real Hardware (Raspberry Pi):
```
Python Code → Hardware Libraries → Linux Drivers → Physical Hardware
```

### Mock System (macOS):
```
Python Code → Mock Libraries → Simulated Responses
```

## Development Workflow

1. **Write Code** - Develop robot logic using mock system
2. **Test Locally** - Run and debug on macOS
3. **Deploy** - Transfer to real Raspberry Pi with hardware
4. **Test Hardware** - Verify with actual sensors and motors

## Troubleshooting

### Common Issues:

1. **Import Errors**: Make sure `mock_hardware.py` is in your Python path
2. **Missing Dependencies**: Install required packages (PyQt5, numpy)
3. **File Access**: Ensure write permissions for `params.json`

### Error Messages:
- `"Mock hardware system ready!"` - System working correctly
- `"Real hardware detected"` - Running on actual Raspberry Pi
- `"Development mode"` - Using mock system

## Next Steps

1. **Complete Mock Implementation**: Add missing methods to mock classes
2. **GUI Testing**: Test PyQt5 interface components
3. **Network Testing**: Test client-server communication
4. **Hardware Integration**: Deploy to real Raspberry Pi

## Benefits

- **Faster Development** - No hardware setup required
- **Cost Effective** - Develop without expensive robot kit
- **Safe Testing** - No risk of damaging hardware
- **Portable** - Work on any computer
- **Version Control** - Easy to track code changes

## Limitations

- **No Real Sensors** - Can't test actual sensor readings
- **No Motor Control** - Can't test servo movements
- **No Physical Feedback** - Can't see actual robot behavior
- **Limited Accuracy** - Mock data may not match real hardware

## Conclusion

This mock system provides a solid foundation for developing the hexapod robot software on macOS. While it can't replace real hardware testing, it enables rapid development and debugging of the core software logic.
