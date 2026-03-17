import os
from glob import glob
from setuptools import setup

package_name = "crisp_web_control"

setup(
    name=package_name,
    version="0.1.0",
    packages=[package_name],
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        (os.path.join("share", package_name, "launch"), glob("launch/*.py")),
        (os.path.join("share", package_name, "static"), glob("crisp_web_control/static/*")),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    author="yizhong",
    description="Web interface for CRISP controllers on UR15",
    license="MIT",
    entry_points={
        "console_scripts": [
            "web_server = crisp_web_control.web_server:main",
        ],
    },
)
