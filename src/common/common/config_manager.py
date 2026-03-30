#!/usr/bin/env python3
"""
Centralized Configuration Manager for Robot DC

This module provides a singleton ConfigManager for loading and accessing
robot configuration from YAML files. It supports:
- Multi-robot configuration (ur15, duco, etc.)
- Shared/common configuration
- Environment variable expansion
- Path resolution (relative to workspace root)
- Thread-safe singleton pattern
- Validation and type checking

Usage:
    from common.config_manager import ConfigManager
    
    # Get singleton instance
    config = ConfigManager()
    
    # Access robot-specific config
    ur15 = config.get_robot('ur15')
    ur15_ip = ur15.get('network.robot_ip')
    
    # Access shared config
    redis_host = config.get('shared.network.redis.host')
    
    # Direct access with dot notation
    ur15_ip = config.get('ur15.network.robot_ip')
"""

import os
import re
import yaml
import threading
from pathlib import Path
from typing import Any, Dict, List, Optional


class ConfigError(Exception):
    """Exception raised for configuration errors."""
    pass


class RobotConfig:
    """Wrapper for robot-specific configuration."""
    
    def __init__(self, name: str, data: dict, config_manager: 'ConfigManager'):
        """
        Initialize robot configuration wrapper.
        
        Args:
            name: Robot name (e.g., 'ur15', 'duco')
            data: Robot configuration dictionary
            config_manager: Parent ConfigManager instance
        """
        self.name = name
        self._data = data
        self._config_manager = config_manager
    
    def get(self, key: str, default: Any = None) -> Any:
        """
        Get configuration value using dot notation.
        
        Args:
            key: Configuration key with dot notation (e.g., 'network.robot_ip')
            default: Default value if key not found
            
        Returns:
            Configuration value or default
            
        Example:
            robot_ip = ur15.get('network.robot_ip')
            web_port = ur15.get('services.web', default=8030)
        """
        return self._get_nested(self._data, key.split('.'), default)
    
    def _get_nested(self, data: dict, keys: List[str], default: Any) -> Any:
        """Recursively get nested value from dictionary."""
        if not keys:
            return data
        
        key = keys[0]
        if not isinstance(data, dict) or key not in data:
            return default
        
        if len(keys) == 1:
            return data[key]
        
        return self._get_nested(data[key], keys[1:], default)
    
    def has(self, key: str) -> bool:
        """
        Check if configuration key exists.
        
        Args:
            key: Configuration key with dot notation
            
        Returns:
            True if key exists, False otherwise
        """
        return self.get(key) is not None
    
    def get_all(self) -> dict:
        """Get all configuration data for this robot."""
        return self._data.copy()
    
    def __getitem__(self, key: str) -> Any:
        """Support dict-style access: robot['network']['robot_ip']"""
        return self._data[key]
    
    def __repr__(self) -> str:
        return f"RobotConfig(name='{self.name}')"


class ConfigManager:
    """
    Singleton configuration manager for robot_dc workspace.
    
    Loads configuration from robot_config.yaml and provides convenient
    access to robot-specific and shared configuration.
    """
    
    _instance = None
    _lock = threading.Lock()
    _initialized = False
    
    # Config file search locations (in priority order)
    CONFIG_PATHS = [
        lambda: os.path.join(os.environ.get('ROBOT_DC_ROOT', ''), 'config', 'robot_config.yaml'),
        lambda: os.path.join(Path.home(), 'robot_dc', 'config', 'robot_config.yaml'),
        lambda: '/etc/robot_dc/robot_config.yaml',
    ]
    
    def __new__(cls):
        """Singleton pattern - only one instance exists."""
        if cls._instance is None:
            with cls._lock:
                if cls._instance is None:
                    cls._instance = super().__new__(cls)
        return cls._instance
    
    def __init__(self):
        """Initialize configuration manager (only once)."""
        if self._initialized:
            return
        
        with self._lock:
            if self._initialized:
                return
            
            self._config: Dict[str, Any] = {}
            self._config_path: Optional[Path] = None
            self._load_config()
            self._initialized = True
    
    def _find_config_file(self) -> Optional[Path]:
        """
        Find configuration file in standard locations.
        
        Returns:
            Path to config file or None if not found
        """
        # First try to find workspace root using existing utility
        try:
            from common.workspace_utils import get_workspace_root
            workspace_root = get_workspace_root()
            config_path = Path(workspace_root) / 'config' / 'robot_config.yaml'
            if config_path.exists():
                return config_path
        except (ImportError, Exception):
            pass
        
        # Try each predefined path
        for path_func in self.CONFIG_PATHS:
            try:
                path = Path(path_func())
                if path.exists():
                    return path
            except Exception:
                continue
        
        return None
    
    def _expand_env_vars(self, value: Any) -> Any:
        """
        Recursively expand environment variables in configuration values.
        
        Supports ${VAR_NAME} syntax in strings.
        
        Args:
            value: Configuration value (string, dict, list, or other)
            
        Returns:
            Value with environment variables expanded
        """
        if isinstance(value, str):
            # Match ${VAR_NAME} pattern
            pattern = r'\$\{([^}]+)\}'
            
            def replacer(match):
                var_name = match.group(1)
                return os.environ.get(var_name, match.group(0))
            
            return re.sub(pattern, replacer, value)
        
        elif isinstance(value, dict):
            return {k: self._expand_env_vars(v) for k, v in value.items()}
        
        elif isinstance(value, list):
            return [self._expand_env_vars(item) for item in value]
        
        return value
    
    def _resolve_paths(self, config: dict) -> dict:
        """
        Resolve relative paths to absolute paths based on workspace root.
        
        Args:
            config: Configuration dictionary
            
        Returns:
            Configuration with resolved paths
        """
        try:
            from common.workspace_utils import get_workspace_root
            workspace_root = Path(get_workspace_root())
        except (ImportError, Exception):
            # Fall back to config file parent's parent directory
            if self._config_path:
                workspace_root = self._config_path.parent.parent
            else:
                workspace_root = Path.cwd()
        
        # Recursively resolve paths in 'paths' sections
        def resolve_paths_recursive(data: Any) -> Any:
            if isinstance(data, dict):
                if 'paths' in data and isinstance(data['paths'], dict):
                    # Resolve paths in this section
                    resolved_paths = {}
                    for key, value in data['paths'].items():
                        if isinstance(value, str) and not Path(value).is_absolute():
                            resolved_paths[key] = str(workspace_root / value)
                        else:
                            resolved_paths[key] = value
                    data['paths'] = resolved_paths
                
                # Recurse into nested dicts
                return {k: resolve_paths_recursive(v) for k, v in data.items()}
            
            elif isinstance(data, list):
                return [resolve_paths_recursive(item) for item in data]
            
            return data
        
        return resolve_paths_recursive(config)
    
    def _load_config(self):
        """Load configuration from YAML file."""
        config_path = self._find_config_file()
        
        if config_path is None:
            raise ConfigError(
                "Configuration file not found. Please create config/robot_config.yaml\n"
                "Copy from template: cp config/robot_config.example.yaml config/robot_config.yaml"
            )
        
        self._config_path = config_path
        
        try:
            with open(config_path, 'r', encoding='utf-8') as f:
                config = yaml.safe_load(f)
            
            if config is None:
                raise ConfigError(f"Configuration file is empty: {config_path}")
            
            # Expand environment variables
            config = self._expand_env_vars(config)
            
            # Resolve relative paths
            config = self._resolve_paths(config)
            
            self._config = config
            
        except yaml.YAMLError as e:
            raise ConfigError(f"Failed to parse YAML configuration: {e}")
        except Exception as e:
            raise ConfigError(f"Failed to load configuration: {e}")
    
    def reload(self):
        """Reload configuration from file (useful for development)."""
        with self._lock:
            self._load_config()
    
    def get(self, key: str, default: Any = None) -> Any:
        """
        Get configuration value using dot notation.
        
        Args:
            key: Configuration key with dot notation 
                 Examples: 'ur15.network.robot_ip', 'shared.network.redis.host'
            default: Default value if key not found
            
        Returns:
            Configuration value or default
            
        Example:
            ur15_ip = config.get('ur15.network.robot_ip')
            redis_host = config.get('shared.network.redis.host', 'localhost')
        """
        keys = key.split('.')
        value = self._config
        
        for k in keys:
            if not isinstance(value, dict) or k not in value:
                return default
            value = value[k]
        
        return value
    
    def has(self, key: str) -> bool:
        """
        Check if configuration key exists.
        
        Args:
            key: Configuration key with dot notation
            
        Returns:
            True if key exists, False otherwise
        """
        keys = key.split('.')
        value = self._config
        
        for k in keys:
            if not isinstance(value, dict) or k not in value:
                return False
            value = value[k]
        
        return True
    
    def get_robot(self, robot_name: str) -> RobotConfig:
        """
        Get robot-specific configuration wrapper.
        
        Args:
            robot_name: Robot name (e.g., 'ur15', 'duco')
            
        Returns:
            RobotConfig object for convenient access
            
        Raises:
            ConfigError: If robot not found in configuration
            
        Example:
            ur15 = config.get_robot('ur15')
            ur15_ip = ur15.get('network.robot_ip')
        """
        if robot_name not in self._config:
            available = [k for k in self._config.keys() if k not in ['version', 'shared']]
            raise ConfigError(
                f"Robot '{robot_name}' not found in configuration. "
                f"Available robots: {available}"
            )
        
        return RobotConfig(robot_name, self._config[robot_name], self)
    
    def list_robots(self) -> List[str]:
        """
        Get list of configured robots.
        
        Returns:
            List of robot names (e.g., ['ur15', 'duco'])
        """
        # Exclude 'version' and 'shared' from robot list
        return [k for k in self._config.keys() if k not in ['version', 'shared']]
    
    def get_all(self) -> dict:
        """
        Get entire configuration dictionary.
        
        Returns:
            Complete configuration dictionary
        """
        return self._config.copy()
    
    def __getitem__(self, key: str) -> Any:
        """Support dict-style access: config['ur15']['network']['robot_ip']"""
        return self._config[key]
    
    def __repr__(self) -> str:
        robots = self.list_robots()
        return f"ConfigManager(robots={robots}, config_path='{self._config_path}')"


# Convenience function for quick access
def get_config() -> ConfigManager:
    """
    Get ConfigManager singleton instance.
    
    Returns:
        ConfigManager instance
    """
    return ConfigManager()
