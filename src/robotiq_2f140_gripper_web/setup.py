from setuptools import setup
import os
from glob import glob

package_name = 'robotiq_2f140_gripper_web'

setup(
    name=package_name,
    version='0.1.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', glob('launch/*.launch.py')),
        ('share/' + package_name + '/templates', glob('robotiq_2f140_gripper_web/templates/*.html')),
        ('share/' + package_name + '/static', glob('robotiq_2f140_gripper_web/static/*.css')),
        ('share/' + package_name + '/static', glob('robotiq_2f140_gripper_web/static/*.js')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Your Name',
    maintainer_email='your_email@example.com',
    description='Web interface for monitoring and controlling Robotiq 2F-140 gripper',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'gripper_web_node = robotiq_2f140_gripper_web.gripper_web_node:main',
        ],
    },
)
