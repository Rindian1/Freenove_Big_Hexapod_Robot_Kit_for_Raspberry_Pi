# -*- coding: utf-8 -*-
"""
Parameter Management module for Freenove Big Hexapod Robot Kit

This module handles system parameter management, including hardware version
detection, parameter file management, and configuration validation.
"""

import os
import json
import subprocess
import time
from typing import Dict, Any, Optional, List, Union
from dataclasses import dataclass, field
from enum import Enum


class ParameterType(Enum):
    """Types of parameters supported."""
    INTEGER = "integer"
    FLOAT = "float"
    STRING = "string"
    BOOLEAN = "boolean"
    LIST = "list"
    DICT = "dict"


@dataclass
class ParameterDefinition:
    """Definition of a parameter with validation rules."""
    name: str
    param_type: ParameterType
    default_value: Any
    description: str = ""
    min_value: Optional[Union[int, float]] = None
    max_value: Optional[Union[int, float]] = None
    allowed_values: Optional[List[Any]] = None
    required: bool = True
    
    def validate(self, value: Any) -> bool:
        """
        Validate a parameter value.
        
        Args:
            value: Value to validate
            
        Returns:
            bool: True if valid
        """
        if value is None and self.required:
            return False
        
        if value is None:
            return True
        
        # Type validation
        try:
            if self.param_type == ParameterType.INTEGER:
                value = int(value)
            elif self.param_type == ParameterType.FLOAT:
                value = float(value)
            elif self.param_type == ParameterType.BOOLEAN:
                if isinstance(value, str):
                    value = value.lower() in ['true', '1', 'yes', 'on']
                else:
                    value = bool(value)
        except (ValueError, TypeError):
            return False
        
        # Range validation
        if self.min_value is not None and value < self.min_value:
            return False
        if self.max_value is not None and value > self.max_value:
            return False
        
        # Allowed values validation
        if self.allowed_values is not None and value not in self.allowed_values:
            return False
        
        return True


@dataclass
class ParameterConfig:
    """Configuration for parameter management."""
    # File settings
    default_file: str = 'params.json'
    backup_enabled: bool = True
    max_backups: int = 5
    
    # Validation settings
    strict_validation: bool = True
    auto_correct: bool = False
    
    # Hardware detection settings
    auto_detect_pi: bool = True
    fallback_pi_version: int = 1
    fallback_pcb_version: int = 2
    
    # Performance settings
    cache_enabled: bool = True
    cache_timeout: float = 300.0  # 5 minutes
    
    # Logging settings
    verbose_logging: bool = False


class ParameterManager:
    """
    Enhanced parameter management system for hexapod robot configuration.
    
    This class provides comprehensive parameter management with:
    - Hardware version detection
    - Parameter validation and type checking
    - File backup and recovery
    - Caching for performance
    - Error handling and logging
    """
    
    # Default parameter definitions
    DEFAULT_PARAMETERS = {
        'Pcb_Version': ParameterDefinition(
            name='Pcb_Version',
            param_type=ParameterType.INTEGER,
            default_value=2,
            description='PCB version (1 or 2)',
            min_value=1,
            max_value=2,
            allowed_values=[1, 2]
        ),
        'Pi_Version': ParameterDefinition(
            name='Pi_Version',
            param_type=ParameterType.INTEGER,
            default_value=1,
            description='Raspberry Pi version (1 for Pi 4 and earlier, 2 for Pi 5)',
            min_value=1,
            max_value=2,
            allowed_values=[1, 2]
        ),
        'Led_Count': ParameterDefinition(
            name='Led_Count',
            param_type=ParameterType.INTEGER,
            default_value=7,
            description='Number of LEDs in the strip',
            min_value=1,
            max_value=100
        ),
        'Servo_Count': ParameterDefinition(
            name='Servo_Count',
            param_type=ParameterType.INTEGER,
            default_value=18,
            description='Number of servos',
            min_value=1,
            max_value=50
        ),
        'Update_Rate': ParameterDefinition(
            name='Update_Rate',
            param_type=ParameterType.FLOAT,
            default_value=100.0,
            description='System update rate in Hz',
            min_value=1.0,
            max_value=1000.0
        ),
        'Debug_Mode': ParameterDefinition(
            name='Debug_Mode',
            param_type=ParameterType.BOOLEAN,
            default_value=False,
            description='Enable debug mode',
            required=False
        )
    }
    
    def __init__(self, config: Optional[ParameterConfig] = None):
        """
        Initialize the parameter manager.
        
        Args:
            config (ParameterConfig, optional): Configuration parameters
        """
        self.config = config or ParameterConfig()
        self.file_path = self.config.default_file
        self.cache = {}
        self.cache_timestamp = 0.0
        self.parameter_definitions = self.DEFAULT_PARAMETERS.copy()
        
        # Initialize parameter file
        if not self.file_exists() or not self.validate_params():
            self._handle_parameter_initialization()
        
        if self.config.verbose_logging:
            print(f"Parameter manager initialized with file: {self.file_path}")
    
    def file_exists(self, file_path: Optional[str] = None) -> bool:
        """
        Check if the specified file exists.
        
        Args:
            file_path (str, optional): Path to check
            
        Returns:
            bool: True if file exists
        """
        file_path = file_path or self.file_path
        return os.path.exists(file_path)
    
    def validate_params(self, file_path: Optional[str] = None) -> bool:
        """
        Validate that the parameter file contains valid parameters.
        
        Args:
            file_path (str, optional): Path to parameter file
            
        Returns:
            bool: True if parameters are valid
        """
        file_path = file_path or self.file_path
        
        if not self.file_exists(file_path):
            if self.config.verbose_logging:
                print(f"Parameter file {file_path} does not exist")
            return False
        
        try:
            with open(file_path, 'r') as file:
                params = json.load(file)
            
            # Validate all parameters
            for param_name, param_def in self.parameter_definitions.items():
                if param_name in params:
                    if not param_def.validate(params[param_name]):
                        if self.config.verbose_logging:
                            print(f"Invalid parameter {param_name}: {params[param_name]}")
                        return False
                elif param_def.required:
                    if self.config.verbose_logging:
                        print(f"Missing required parameter: {param_name}")
                    return False
            
            return True
            
        except json.JSONDecodeError as e:
            if self.config.verbose_logging:
                print(f"Error decoding JSON file: {e}")
            return False
        except Exception as e:
            if self.config.verbose_logging:
                print(f"Error reading parameter file: {e}")
            return False
    
    def get_param(self, param_name: str, file_path: Optional[str] = None) -> Any:
        """
        Get the value of a specified parameter.
        
        Args:
            param_name (str): Parameter name
            file_path (str, optional): Path to parameter file
            
        Returns:
            Any: Parameter value or None if not found
        """
        file_path = file_path or self.file_path
        
        # Check cache first
        if self.config.cache_enabled:
            cache_key = f"{file_path}:{param_name}"
            if (cache_key in self.cache and 
                time.time() - self.cache_timestamp < self.config.cache_timeout):
                return self.cache[cache_key]
        
        if not self.validate_params(file_path):
            return None
        
        try:
            with open(file_path, 'r') as file:
                params = json.load(file)
            
            value = params.get(param_name)
            
            # Validate value
            if param_name in self.parameter_definitions:
                param_def = self.parameter_definitions[param_name]
                if not param_def.validate(value):
                    if self.config.auto_correct:
                        value = param_def.default_value
                        self.set_param(param_name, value, file_path)
                    else:
                        if self.config.verbose_logging:
                            print(f"Invalid parameter value for {param_name}: {value}")
                        return None
            
            # Update cache
            if self.config.cache_enabled:
                cache_key = f"{file_path}:{param_name}"
                self.cache[cache_key] = value
                self.cache_timestamp = time.time()
            
            return value
            
        except Exception as e:
            if self.config.verbose_logging:
                print(f"Error getting parameter {param_name}: {e}")
            return None
    
    def set_param(self, param_name: str, value: Any, file_path: Optional[str] = None) -> bool:
        """
        Set the value of a specified parameter.
        
        Args:
            param_name (str): Parameter name
            value: Parameter value
            file_path (str, optional): Path to parameter file
            
        Returns:
            bool: True if successful
        """
        file_path = file_path or self.file_path
        
        # Validate parameter definition
        if param_name in self.parameter_definitions:
            param_def = self.parameter_definitions[param_name]
            if not param_def.validate(value):
                if self.config.verbose_logging:
                    print(f"Invalid value for parameter {param_name}: {value}")
                return False
        
        try:
            # Create backup if enabled
            if self.config.backup_enabled and self.file_exists(file_path):
                self._create_backup(file_path)
            
            # Load existing parameters
            params = {}
            if self.file_exists(file_path):
                with open(file_path, 'r') as file:
                    params = json.load(file)
            
            # Update parameter
            params[param_name] = value
            
            # Write back to file
            with open(file_path, 'w') as file:
                json.dump(params, file, indent=4)
            
            # Clear cache
            if self.config.cache_enabled:
                self._clear_cache()
            
            if self.config.verbose_logging:
                print(f"Parameter {param_name} set to {value}")
            
            return True
            
        except Exception as e:
            if self.config.verbose_logging:
                print(f"Error setting parameter {param_name}: {e}")
            return False
    
    def get_all_params(self, file_path: Optional[str] = None) -> Dict[str, Any]:
        """
        Get all parameters from the file.
        
        Args:
            file_path (str, optional): Path to parameter file
            
        Returns:
            Dict[str, Any]: All parameters
        """
        file_path = file_path or self.file_path
        
        if not self.validate_params(file_path):
            return {}
        
        try:
            with open(file_path, 'r') as file:
                return json.load(file)
        except Exception as e:
            if self.config.verbose_logging:
                print(f"Error reading all parameters: {e}")
            return {}
    
    def set_all_params(self, params: Dict[str, Any], file_path: Optional[str] = None) -> bool:
        """
        Set multiple parameters at once.
        
        Args:
            params (Dict[str, Any]): Parameters to set
            file_path (str, optional): Path to parameter file
            
        Returns:
            bool: True if successful
        """
        file_path = file_path or self.file_path
        
        try:
            # Validate all parameters
            for param_name, value in params.items():
                if param_name in self.parameter_definitions:
                    param_def = self.parameter_definitions[param_name]
                    if not param_def.validate(value):
                        if self.config.verbose_logging:
                            print(f"Invalid parameter {param_name}: {value}")
                        return False
            
            # Create backup if enabled
            if self.config.backup_enabled and self.file_exists(file_path):
                self._create_backup(file_path)
            
            # Write parameters
            with open(file_path, 'w') as file:
                json.dump(params, file, indent=4)
            
            # Clear cache
            if self.config.cache_enabled:
                self._clear_cache()
            
            if self.config.verbose_logging:
                print(f"Set {len(params)} parameters")
            
            return True
            
        except Exception as e:
            if self.config.verbose_logging:
                print(f"Error setting parameters: {e}")
            return False
    
    def delete_param_file(self, file_path: Optional[str] = None) -> bool:
        """
        Delete the parameter file.
        
        Args:
            file_path (str, optional): Path to parameter file
            
        Returns:
            bool: True if successful
        """
        file_path = file_path or self.file_path
        
        try:
            if self.file_exists(file_path):
                os.remove(file_path)
                self._clear_cache()
                if self.config.verbose_logging:
                    print(f"Deleted parameter file: {file_path}")
                return True
            else:
                if self.config.verbose_logging:
                    print(f"Parameter file does not exist: {file_path}")
                return False
        except Exception as e:
            if self.config.verbose_logging:
                print(f"Error deleting parameter file: {e}")
            return False
    
    def create_param_file(self, file_path: Optional[str] = None) -> bool:
        """
        Create a parameter file with default values.
        
        Args:
            file_path (str, optional): Path to parameter file
            
        Returns:
            bool: True if successful
        """
        file_path = file_path or self.file_path
        
        try:
            # Create default parameters
            default_params = {}
            for param_name, param_def in self.parameter_definitions.items():
                default_params[param_name] = param_def.default_value
            
            # Auto-detect Pi version if enabled
            if self.config.auto_detect_pi:
                default_params['Pi_Version'] = self.get_raspberry_pi_version()
            
            # Write to file
            with open(file_path, 'w') as file:
                json.dump(default_params, file, indent=4)
            
            if self.config.verbose_logging:
                print(f"Created parameter file: {file_path}")
            
            return True
            
        except Exception as e:
            if self.config.verbose_logging:
                print(f"Error creating parameter file: {e}")
            return False
    
    def get_raspberry_pi_version(self) -> int:
        """
        Get the version of the Raspberry Pi.
        
        Returns:
            int: Pi version (1 for Pi 4 and earlier, 2 for Pi 5)
        """
        try:
            # Try multiple methods to detect Pi version
            methods = [
                self._detect_pi_via_model,
                self._detect_pi_via_cpuinfo,
                self._detect_pi_via_proc_version
            ]
            
            for method in methods:
                try:
                    version = method()
                    if version is not None:
                        return version
                except Exception:
                    continue
            
            # Fallback to default
            if self.config.verbose_logging:
                print("Could not detect Pi version, using fallback")
            return self.config.fallback_pi_version
            
        except Exception as e:
            if self.config.verbose_logging:
                print(f"Error detecting Pi version: {e}")
            return self.config.fallback_pi_version
    
    def _detect_pi_via_model(self) -> Optional[int]:
        """Detect Pi version via device tree model."""
        try:
            result = subprocess.run(
                ['cat', '/sys/firmware/devicetree/base/model'], 
                capture_output=True, text=True, timeout=5
            )
            if result.returncode == 0:
                model = result.stdout.strip()
                if "Raspberry Pi 5" in model:
                    return 2
                elif "Raspberry Pi" in model:
                    return 1
        except Exception:
            pass
        return None
    
    def _detect_pi_via_cpuinfo(self) -> Optional[int]:
        """Detect Pi version via /proc/cpuinfo."""
        try:
            if os.path.exists('/proc/cpuinfo'):
                with open('/proc/cpuinfo', 'r') as f:
                    content = f.read()
                    if 'BCM2711' in content:  # Pi 4
                        return 1
                    elif 'BCM2712' in content:  # Pi 5
                        return 2
        except Exception:
            pass
        return None
    
    def _detect_pi_via_proc_version(self) -> Optional[int]:
        """Detect Pi version via /proc/version."""
        try:
            if os.path.exists('/proc/version'):
                with open('/proc/version', 'r') as f:
                    content = f.read()
                    if 'BCM2711' in content:
                        return 1
                    elif 'BCM2712' in content:
                        return 2
        except Exception:
            pass
        return None
    
    def _handle_parameter_initialization(self) -> None:
        """Handle parameter file initialization."""
        if not self.file_exists() or not self.validate_params():
            if self.config.verbose_logging:
                print(f"Parameter file {self.file_path} does not exist or contains invalid parameters.")
            
            # Check if user input is required
            user_input_required = True
            if self.file_exists() and self.validate_params():
                try:
                    user_choice = input("Do you want to re-enter the hardware versions? (yes/no): ").strip().lower()
                    user_input_required = user_choice == 'yes'
                except (EOFError, KeyboardInterrupt):
                    user_input_required = False
            
            if user_input_required:
                self._interactive_parameter_setup()
            else:
                if self.config.verbose_logging:
                    print("Using existing parameters")
    
    def _interactive_parameter_setup(self) -> None:
        """Interactive parameter setup."""
        print("Please enter the hardware versions.")
        
        # Get PCB version
        while True:
            try:
                pcb_version = int(input("Enter PCB Version (1 or 2): "))
                if pcb_version in [1, 2]:
                    break
                else:
                    print("Invalid PCB Version. Please enter 1 or 2.")
            except (ValueError, EOFError, KeyboardInterrupt):
                print("Using default PCB version 2")
                pcb_version = self.config.fallback_pcb_version
                break
        
        # Auto-detect Pi version
        pi_version = self.get_raspberry_pi_version()
        
        # Create parameter file
        self.create_param_file()
        self.set_param('Pcb_Version', pcb_version)
        self.set_param('Pi_Version', pi_version)
        
        if self.config.verbose_logging:
            print("Parameter setup completed")
    
    def _create_backup(self, file_path: str) -> None:
        """Create a backup of the parameter file."""
        try:
            timestamp = int(time.time())
            backup_path = f"{file_path}.backup.{timestamp}"
            
            # Limit number of backups
            if self.config.max_backups > 0:
                self._cleanup_old_backups(file_path)
            
            with open(file_path, 'r') as src, open(backup_path, 'w') as dst:
                dst.write(src.read())
            
            if self.config.verbose_logging:
                print(f"Created backup: {backup_path}")
                
        except Exception as e:
            if self.config.verbose_logging:
                print(f"Error creating backup: {e}")
    
    def _cleanup_old_backups(self, file_path: str) -> None:
        """Clean up old backup files."""
        try:
            backup_dir = os.path.dirname(file_path) or '.'
            base_name = os.path.basename(file_path)
            
            # Find backup files
            backup_files = []
            for file in os.listdir(backup_dir):
                if file.startswith(f"{base_name}.backup."):
                    backup_files.append(os.path.join(backup_dir, file))
            
            # Sort by modification time (oldest first)
            backup_files.sort(key=lambda x: os.path.getmtime(x))
            
            # Remove oldest backups
            while len(backup_files) >= self.config.max_backups:
                oldest = backup_files.pop(0)
                os.remove(oldest)
                if self.config.verbose_logging:
                    print(f"Removed old backup: {oldest}")
                    
        except Exception as e:
            if self.config.verbose_logging:
                print(f"Error cleaning up backups: {e}")
    
    def _clear_cache(self) -> None:
        """Clear the parameter cache."""
        self.cache.clear()
        self.cache_timestamp = 0.0
    
    def get_pcb_version(self) -> Optional[int]:
        """
        Get the PCB version from the parameter file.
        
        Returns:
            int: PCB version or None if not found
        """
        return self.get_param('Pcb_Version')
    
    def get_pi_version(self) -> Optional[int]:
        """
        Get the Raspberry Pi version from the parameter file.
        
        Returns:
            int: Pi version or None if not found
        """
        return self.get_param('Pi_Version')
    
    def get_status(self) -> Dict[str, Any]:
        """
        Get parameter manager status.
        
        Returns:
            Dict[str, Any]: Status information
        """
        return {
            'file_path': self.file_path,
            'file_exists': self.file_exists(),
            'parameters_valid': self.validate_params(),
            'cache_enabled': self.config.cache_enabled,
            'cache_size': len(self.cache),
            'backup_enabled': self.config.backup_enabled,
            'auto_detect_pi': self.config.auto_detect_pi,
            'verbose_logging': self.config.verbose_logging
        }
    
    def reset_to_defaults(self) -> bool:
        """
        Reset all parameters to default values.
        
        Returns:
            bool: True if successful
        """
        return self.create_param_file()
    
    def __enter__(self):
        """Context manager entry."""
        return self
    
    def __exit__(self, exc_type, exc_value, traceback):
        """Context manager exit."""
        self._clear_cache()


if __name__ == '__main__':
    print("Parameter Manager Test")
    print("=" * 30)
    
    # Create parameter manager with verbose logging
    config = ParameterConfig(verbose_logging=True)
    manager = ParameterManager(config)
    
    try:
        # Test parameter operations
        print("\n1. Testing parameter retrieval...")
        pcb_version = manager.get_pcb_version()
        pi_version = manager.get_pi_version()
        print(f"PCB Version: {pcb_version}")
        print(f"Pi Version: {pi_version}")
        
        print("\n2. Testing parameter setting...")
        success = manager.set_param('Debug_Mode', True)
        print(f"Set Debug_Mode: {success}")
        
        print("\n3. Testing parameter validation...")
        all_params = manager.get_all_params()
        print(f"All parameters: {all_params}")
        
        print("\n4. Testing status...")
        status = manager.get_status()
        print(f"Status: {status}")
        
        print("\n5. Testing Pi version detection...")
        detected_version = manager.get_raspberry_pi_version()
        print(f"Detected Pi version: {detected_version}")
        
        print("\n✓ All parameter manager tests completed successfully")
        
    except Exception as e:
        print(f"Test failed: {e}")
    finally:
        print("Parameter manager test completed")