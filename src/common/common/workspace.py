"""Workspace path utilities."""

import os
import subprocess


def get_workspace_root() -> str:
    """Get the root directory of the ur15_crisp workspace.

    Searches for the workspace root by looking for the 'src' directory
    containing known packages. Tries (in order):
      1. Git repo root (if inside a git repo)
      2. Walk up from this file's location
      3. Current working directory

    Returns:
        Absolute path to the workspace root.

    Raises:
        RuntimeError: If the workspace root cannot be determined.
    """
    # Method 1: git rev-parse
    try:
        root = subprocess.check_output(
            ["git", "rev-parse", "--show-toplevel"],
            stderr=subprocess.DEVNULL,
            text=True,
        ).strip()
        if _is_workspace_root(root):
            return root
    except (subprocess.CalledProcessError, FileNotFoundError):
        pass

    # Method 2: walk up from this file
    d = os.path.dirname(os.path.abspath(__file__))
    for _ in range(10):
        d = os.path.dirname(d)
        if _is_workspace_root(d):
            return d

    # Method 3: cwd
    cwd = os.getcwd()
    if _is_workspace_root(cwd):
        return cwd

    raise RuntimeError(
        "Cannot determine workspace root. "
        "Run from inside the ur15_crisp workspace or ensure it's a git repo."
    )


def get_config_path(filename: str = "ur15_controllers.yaml") -> str:
    """Get the path to a config file in the workspace config/ directory.

    Args:
        filename: Name of the config file.

    Returns:
        Absolute path to config/<filename>.
    """
    return os.path.join(get_workspace_root(), "config", filename)


def _is_workspace_root(path: str) -> bool:
    """Check if a path looks like the ur15_crisp workspace root."""
    return (
        os.path.isdir(os.path.join(path, "src"))
        and os.path.isdir(os.path.join(path, "config"))
    )
