from setuptools import find_packages, setup

package_name = 'robotiq_2f140_gripper'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', [
            'launch/robotiq_gripper.launch.py',
            'launch/robotiq_gripper_web.launch.py'
        ]),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='robot',
    maintainer_email='yizhongzhang1989@gmail.com',
    description='ROS2 driver for Robotiq 2F-140 Gripper',
    license='MIT',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'robotiq_gripper_node = robotiq_2f140_gripper.robotiq_gripper_node:main',
        ],
    },
)
