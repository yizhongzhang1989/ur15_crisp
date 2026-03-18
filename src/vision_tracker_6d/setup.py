import os
from glob import glob
from setuptools import setup

package_name = "vision_tracker_6d"

setup(
    name=package_name,
    version="0.1.0",
    packages=[package_name],
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        (os.path.join("share", package_name, "launch"), glob("launch/*.py")),
        (os.path.join("share", package_name, "config"), glob("config/*.yaml") + glob("config/*.json")),
        (os.path.join("share", package_name, "static"), glob(package_name + "/static/*")),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    author="yizhong",
    description="6D chessboard pose tracking via camera(s) with web monitoring",
    license="MIT",
    entry_points={
        "console_scripts": [
            "tracker_node = vision_tracker_6d.tracker_node:main",
            "web_server = vision_tracker_6d.web_server:main",
        ],
    },
)
