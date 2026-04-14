import os
from glob import glob
from setuptools import setup

package_name = "joint_vla_control"

setup(
    name=package_name,
    version="0.1.0",
    packages=[package_name],
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        (os.path.join("share", package_name, "launch"), glob("launch/*.py")),
        (os.path.join("share", package_name, "static"), glob("joint_vla_control/static/*")),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    author="yizhong",
    description="VLA (Visual Language Action) joint control for UR15",
    license="MIT",
    entry_points={
        "console_scripts": [
            "vla_server = joint_vla_control.vla_server:main",
        ],
    },
)
