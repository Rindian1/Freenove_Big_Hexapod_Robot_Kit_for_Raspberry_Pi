"""
Configuration Manager for Hexapod Controller
Handles loading, validating, and accessing configuration files.
"""
import json
import os
import logging
from pathlib import Path
from typing import Dict, Any, Optional, List, Union

class ConfigManager:
    def __init__(self, config_dir: str = "config"):
        """Initialize the configuration manager.
        
        Args:
            config_dir: Directory containing configuration files
        """
        self.config_dir = Path(config_dir)
        self.configs: Dict[str, Dict[str, Any]] = {}
        self.logger = logging.getLogger(__name__)
        
        # Create config directory if it doesn't exist
        self.config_dir.mkdir(parents=True, exist_ok=True)
        
    def load_config(self, config_name: str, config_type: str = "json") -> Dict[str, Any]:
        """Load a configuration file.
        
        Args:
            config_name: Name of the config file (without extension)
            config_type: File type/extension (default: json)
            
        Returns:
            Dictionary containing the configuration
            
        Raises:
            FileNotFoundError: If the config file doesn't exist
            json.JSONDecodeError: If the config file is not valid JSON
        """
        config_path = self.config_dir / f"{config_name}.{config_type}"
        
        if not config_path.exists():
            self.logger.error(f"Config file not found: {config_path}")
            raise FileNotFoundError(f"Config file not found: {config_path}")
            
        try:
            with open(config_path, 'r') as f:
                config = json.load(f)
                self.configs[config_name] = config
                self.logger.info(f"Loaded config: {config_name}")
                return config
                
        except json.JSONDecodeError as e:
            self.logger.error(f"Invalid JSON in config file {config_path}: {e}")
            raise
            
    def save_config(self, config_name: str, data: Dict[str, Any], config_type: str = "json") -> None:
        """Save a configuration to file.
        
        Args:
            config_name: Name of the config file (without extension)
            data: Configuration data to save
            config_type: File type/extension (default: json)
        """
        config_path = self.config_dir / f"{config_name}.{config_type}"
        
        try:
            with open(config_path, 'w') as f:
                json.dump(data, f, indent=4)
                self.configs[config_name] = data
                self.logger.info(f"Saved config: {config_name}")
                
        except (IOError, TypeError) as e:
            self.logger.error(f"Failed to save config {config_name}: {e}")
            raise
            
    def get_config(self, config_name: str) -> Dict[str, Any]:
        """Get a loaded configuration.
        
        Args:
            config_name: Name of the config to retrieve
            
        Returns:
            The requested configuration
            
        Raises:
            KeyError: If the config hasn't been loaded
        """
        if config_name not in self.configs:
            self.load_config(config_name)
        return self.configs[config_name]
    
    def get_value(self, config_name: str, key: str, default: Any = None) -> Any:
        """Get a value from a configuration.
        
        Args:
            config_name: Name of the config
            key: Configuration key to retrieve
            default: Default value if key not found
            
        Returns:
            The configuration value or default
        """
        try:
            config = self.get_config(config_name)
            return config.get(key, default)
        except (KeyError, FileNotFoundError):
            return default
            
    def list_configs(self) -> List[str]:
        """List all available configuration files.
        
        Returns:
            List of config file names (without extensions)
        """
        return [f.stem for f in self.config_dir.glob("*.*") if f.is_file()]

# Singleton instance
config_manager = ConfigManager()

def get_config_manager() -> ConfigManager:
    """Get the global ConfigManager instance."""
    return config_manager
