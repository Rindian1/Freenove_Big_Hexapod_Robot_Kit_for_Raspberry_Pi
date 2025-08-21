import time
import random

class MockSMBus:
    """Mock SMBus class that simulates I2C communication for development."""
    
    def __init__(self, bus_number):
        self.bus_number = bus_number
        print(f"Mock I2C bus {bus_number} initialized (simulation mode)")
    
    def read_byte_data(self, device, register):
        """Mock read operation - returns random data."""
        return random.randint(0, 255)
    
    def read_byte(self, device):
        """Mock read operation - returns random data."""
        return random.randint(0, 255)
    
    def write_byte(self, device, value):
        """Mock write operation."""
        pass
    
    def close(self):
        """Mock close operation."""
        print("Mock I2C bus closed")

class ADC:
    def __init__(self):
        """Initialize the ADC class with mock hardware."""
        self.ADS7830_COMMAND = 0x84
        self.adc_voltage_coefficient = 3
        self.I2C_ADDRESS = 0x48
        
        # Use mock SMBus for development
        try:
            import smbus
            self.i2c_bus = smbus.SMBus(1)
            print("Using real I2C hardware")
        except (ImportError, FileNotFoundError):
            self.i2c_bus = MockSMBus(1)
            print("Using mock I2C hardware (simulation mode)")

    def scan_i2c_bus(self) -> list:
        """Scan the I2C bus for connected devices."""
        print("Scanning I2C bus...")
        iic_addr = [None]
        
        if isinstance(self.i2c_bus, MockSMBus):
            # Simulate finding some devices
            simulated_devices = [0x48, 0x68, 0x70]  # Common I2C addresses
            for device in simulated_devices:
                print(f"Device found at address: 0x{device:02X}")
                iic_addr.append(device)
        else:
            # Real hardware scanning
            for device in range(128):
                try:
                    self.i2c_bus.read_byte_data(device, 0)
                    print(f"Device found at address: 0x{device:02X}")
                    iic_addr.append(device)
                except OSError:
                    pass
        
        return iic_addr

    def _read_stable_byte(self) -> int:
        """Read a stable byte from the ADC."""
        if isinstance(self.i2c_bus, MockSMBus):
            # Simulate stable reading with some noise
            base_value = random.randint(100, 200)
            noise = random.randint(-5, 5)
            return max(0, min(255, base_value + noise))
        else:
            # Real hardware reading
            while True:
                value1 = self.i2c_bus.read_byte(self.I2C_ADDRESS)
                value2 = self.i2c_bus.read_byte(self.I2C_ADDRESS)
                if value1 == value2:
                    return value1

    def read_channel_voltage(self, channel: int) -> float:
        """Read the ADC value for the specified channel using ADS7830."""
        command_set = self.ADS7830_COMMAND | ((((channel << 2) | (channel >> 1)) & 0x07) << 4)
        
        if not isinstance(self.i2c_bus, MockSMBus):
            self.i2c_bus.write_byte(self.I2C_ADDRESS, command_set)
        
        value = self._read_stable_byte()
        voltage = value / 255.0 * 5 * self.adc_voltage_coefficient
        return round(voltage, 2)

    def read_battery_voltage(self) -> tuple:
        """Read the battery voltage using ADS7830."""
        battery1 = self.read_channel_voltage(0)
        battery2 = self.read_channel_voltage(4)
        return battery1, battery2

    def close_i2c(self) -> None:
        """Close the I2C bus."""
        self.i2c_bus.close()

if __name__ == '__main__':
    print("ADC Test (Mock/Real Hardware)")
    print("=" * 40)
    
    adc = ADC()
    
    try:
        # Test I2C scanning
        print("\n1. Testing I2C bus scanning:")
        devices = adc.scan_i2c_bus()
        
        print("\n2. Testing battery voltage reading:")
        while True:
            power = adc.read_battery_voltage()
            print(f"Battery 1: {power[0]}V, Battery 2: {power[1]}V")
            time.sleep(1)
            
    except KeyboardInterrupt:
        print("\nEnd of program")
        adc.close_i2c()
