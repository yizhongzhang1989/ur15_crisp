"""Launch file for Vision Tracker 6D.

Reads config/vision_tracker.yaml and:
  1. Spawns a usb_cam node for each camera that specifies a ``video_device``.
  2. Launches the vision tracker (web_server or headless tracker_node).
"""

import os
import shutil

import launch
import yaml
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def _find_config() -> str:
    """Locate (or create) config/vision_tracker.yaml."""
    import subprocess
    try:
        ws = subprocess.check_output(
            ["git", "rev-parse", "--show-toplevel"],
            stderr=subprocess.DEVNULL, text=True,
        ).strip()
    except Exception:
        ws = os.getcwd()

    cfg_path = os.path.join(ws, "config", "vision_tracker.yaml")
    if os.path.isfile(cfg_path):
        return cfg_path

    # Copy template from installed share
    try:
        from ament_index_python.packages import get_package_share_directory
        template = os.path.join(
            get_package_share_directory("vision_tracker_6d"),
            "config", "vision_tracker_config_template.yaml",
        )
        if os.path.isfile(template):
            os.makedirs(os.path.dirname(cfg_path), exist_ok=True)
            shutil.copy2(template, cfg_path)
            return cfg_path
    except Exception:
        pass

    return cfg_path


def _build_camera_nodes(cfg: dict) -> list:
    """Create a usb_cam Node for each camera with a video_device entry."""
    nodes = []
    for cam in cfg.get("cameras", []):
        if not cam.get("enabled", True):
            continue
        video_device = cam.get("video_device", "")
        if not video_device:
            continue

        name = cam["name"]
        topic = cam.get("image_topic", f"/{name}/image_raw")
        framerate = float(cam.get("framerate", 30))
        width = int(cam.get("image_width", 1280))
        height = int(cam.get("image_height", 720))

        nodes.append(
            Node(
                package="usb_cam",
                executable="usb_cam_node_exe",
                name=name,
                namespace=name,
                output="screen",
                parameters=[{
                    "video_device": video_device,
                    "framerate": framerate,
                    "image_width": width,
                    "image_height": height,
                    "pixel_format": "mjpeg2rgb",
                    "camera_name": name,
                    "frame_id": name,
                }],
                remappings=[
                    ("image_raw", topic),
                ],
            )
        )
    return nodes


def generate_launch_description():
    # Load vision tracker config
    cfg_path = _find_config()
    cfg = {}
    if os.path.isfile(cfg_path):
        with open(cfg_path, "r") as f:
            cfg = yaml.safe_load(f) or {}

    camera_nodes = _build_camera_nodes(cfg)

    tracker_nodes = [
        DeclareLaunchArgument(
            "headless", default_value="false",
            description="If true, run tracker node only (no web UI).",
        ),
        Node(
            package="vision_tracker_6d",
            executable="web_server",
            name="vision_tracker_web",
            output="screen",
            condition=launch.conditions.UnlessCondition(
                LaunchConfiguration("headless")
            ),
        ),
        Node(
            package="vision_tracker_6d",
            executable="tracker_node",
            name="vision_tracker_6d",
            output="screen",
            condition=launch.conditions.IfCondition(
                LaunchConfiguration("headless")
            ),
        ),
    ]

    return LaunchDescription(camera_nodes + tracker_nodes)
