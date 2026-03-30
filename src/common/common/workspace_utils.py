"""
Workspace utility functions for ROS 2 projects.

This module provides utilities for finding and working with ROS 2 workspace
directories, regardless of where the package is installed or executed from.
"""

import os
from typing import Optional


def get_workspace_root() -> Optional[str]:
    """
    Find the robot_dc project root directory using multiple methods.
    
    This function returns the PROJECT ROOT (robot_dc), not the colcon workspace.
    The project root contains: colcon_ws/, scripts/, temp/, dataset/, etc.
    
    This function attempts to locate the project root using several strategies:
    1. ROBOT_DC_ROOT environment variable (explicit override)
    2. ROS package discovery (for installed packages)
    3. Filesystem structure search (for development)
    4. COLCON_PREFIX_PATH environment variable
    5. Fallback path (for compatibility)
    
    Returns:
        str: Path to the project root (robot_dc), or None if not found.
        
    Example:
        >>> workspace_root = get_workspace_root()
        >>> if workspace_root:
        ...     temp_dir = os.path.join(workspace_root, 'temp')
        ...     scripts_dir = os.path.join(workspace_root, 'scripts')
        >>> else:
        ...     raise RuntimeError("Could not find workspace root")
    """
    
    # Method 0: Check explicit environment variable (highest priority)
    if 'ROBOT_DC_ROOT' in os.environ:
        robot_dc_root = os.environ['ROBOT_DC_ROOT']
        if os.path.exists(robot_dc_root) and os.path.exists(os.path.join(robot_dc_root, 'colcon_ws')):
            return robot_dc_root
    
    # Method 1: Use ROS package discovery
    # This works when the package is properly installed
    try:
        from ament_index_python.packages import get_package_share_directory
        
        # Try to get any known package in our workspace
        for package_name in ['common', 'ur15_web', 'camera_node', 'robot_status_redis']:
            try:
                package_dir = get_package_share_directory(package_name)
                current = package_dir
                while current != '/' and current:
                    parent = os.path.dirname(current)
                    # If we find 'install' directory, parent is colcon_ws, grandparent is robot_dc
                    if os.path.basename(current) == 'install':
                        colcon_ws = os.path.dirname(current)  # install -> colcon_ws
                        # Then go one more level up to get robot_dc (project root)
                        project_root = os.path.dirname(colcon_ws)  # colcon_ws -> robot_dc
                        # Verify this is robot_dc by checking for expected directories
                        if (os.path.exists(os.path.join(project_root, 'colcon_ws')) and
                            os.path.exists(os.path.join(project_root, 'scripts'))):
                            return project_root
                    current = parent
            except Exception:
                continue
    except Exception:
        pass
    
    # Method 2: Search from current file location
    # This works during development when running from source
    current = os.path.dirname(os.path.abspath(__file__))
    while current != '/' and current:
        # Check if we're in a colcon_ws directory - if so, parent is project root
        if os.path.basename(current) == 'colcon_ws':
            project_root = os.path.dirname(current)
            # Verify by checking for scripts directory
            if os.path.exists(os.path.join(project_root, 'scripts')):
                return project_root
        # Or check if current directory is the project root (has both colcon_ws and scripts)
        if (os.path.exists(os.path.join(current, 'colcon_ws')) and
            os.path.exists(os.path.join(current, 'scripts'))):
            return current
        # Check for robot_dc directory name as a marker
        if os.path.basename(current) == 'robot_dc':
            if (os.path.exists(os.path.join(current, 'colcon_ws')) and
                os.path.exists(os.path.join(current, 'scripts'))):
                return current
        current = os.path.dirname(current)
    
    # Method 3: Check COLCON_PREFIX_PATH environment variable
    # This works in various ROS setups where environment is properly sourced
    if 'COLCON_PREFIX_PATH' in os.environ:
        install_path = os.environ['COLCON_PREFIX_PATH'].split(':')[0]
        # install_path is typically: /path/to/robot_dc/colcon_ws/install
        colcon_ws = os.path.dirname(install_path)  # Get colcon_ws
        project_root = os.path.dirname(colcon_ws)  # Get robot_dc
        # Verify this is correct
        if (os.path.exists(os.path.join(project_root, 'colcon_ws')) and
            os.path.exists(os.path.join(project_root, 'scripts'))):
            return project_root
    
    # Method 4: Check ROS_WORKSPACE environment variable if set
    if 'ROS_WORKSPACE' in os.environ:
        workspace_path = os.environ['ROS_WORKSPACE']
        # ROS_WORKSPACE might point to colcon_ws or project root
        if os.path.basename(workspace_path) == 'colcon_ws':
            project_root = os.path.dirname(workspace_path)
        else:
            project_root = workspace_path
        # Verify
        if (os.path.exists(os.path.join(project_root, 'colcon_ws')) and
            os.path.exists(os.path.join(project_root, 'scripts'))):
            return project_root
    
    # Method 5: Fallback to common development paths
    fallback_paths = [
        os.path.expanduser('~/Documents/robot_dc'),
        os.path.expanduser('~/robot_dc'),
    ]
    
    for fallback_path in fallback_paths:
        if (os.path.exists(fallback_path) and 
            os.path.exists(os.path.join(fallback_path, 'colcon_ws')) and
            os.path.exists(os.path.join(fallback_path, 'scripts'))):
            return fallback_path
    
    return None


def get_temp_directory() -> str:
    """
    Get the temp directory path within the project root.
    
    This function finds the project root (robot_dc) and returns the temp directory path.
    The temp directory is automatically created if it doesn't exist.
    
    Returns:
        str: Path to the temp directory (robot_dc/temp).
        
    Raises:
        RuntimeError: If project root cannot be found.
        
    Example:
        >>> try:
        ...     temp_dir = get_temp_directory()
        ...     calibration_dir = os.path.join(temp_dir, 'calibration')
        ... except RuntimeError as e:
        ...     print(f"Error: {e}")
    """
    project_root = get_workspace_root()
    if project_root is None:
        raise RuntimeError(
            "Could not determine project root directory (robot_dc). "
            "Make sure you're running from within the robot_dc workspace or set the ROBOT_DC_ROOT environment variable."
        )
    
    # get_workspace_root() now always returns project root (robot_dc)
    temp_dir = os.path.join(project_root, 'temp')
    os.makedirs(temp_dir, exist_ok=True)
    return temp_dir


def get_scripts_directory() -> Optional[str]:
    """
    Get the scripts directory path within the workspace.
    
    Returns:
        str: Path to the scripts directory, or None if not found.
        
    Example:
        >>> scripts_dir = get_scripts_directory()
        >>> if scripts_dir:
        ...     calibration_toolkit = os.path.join(scripts_dir, 'ThirdParty', 'camera_calibration_toolkit')
    """
    workspace_root = get_workspace_root()
    if workspace_root is None:
        return None
    
    # Check in workspace root first
    scripts_dir = os.path.join(workspace_root, 'scripts')
    if os.path.exists(scripts_dir):
        return scripts_dir
    
    # Check in parent directory (for colcon_ws structure)
    parent_dir = os.path.dirname(workspace_root)
    parent_scripts_dir = os.path.join(parent_dir, 'scripts')
    if os.path.exists(parent_scripts_dir):
        return parent_scripts_dir
    
    return None


def get_calibration_images_directory() -> str:
    """
    Get the calibration images directory path within the workspace.
    
    This is a convenience function that returns the path where calibration
    images should be stored, typically in the workspace's calibration_images directory.
    
    Returns:
        str: Path to the calibration images directory.
        
    Raises:
        RuntimeError: If workspace root cannot be found.
    """
    workspace_root = get_workspace_root()
    if workspace_root is None:
        raise RuntimeError("Could not determine workspace root directory")
    
    calibration_dir = os.path.join(workspace_root, 'calibration_images')
    os.makedirs(calibration_dir, exist_ok=True)
    return calibration_dir
