import os
from glob import glob

from setuptools import find_packages, setup

package_name = 'parking_bot'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.py')),
        (os.path.join('share', package_name, 'worlds'), glob('worlds/*.sdf')),
        (os.path.join('share', package_name, 'urdf'), glob('urdf/*.xacro')),
        (os.path.join('share', package_name, 'config'), glob('config/*.yaml')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='jhkim',
    maintainer_email='2121jhkim@gmail.com',
    description=(
        '2026 대회 차량 규격 및 후진 주차장 치수를 반영한 ROS 2 + Gazebo 후진주차 시뮬레이션'
    ),
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'parking_controller = parking_bot.parking_controller:main',
        ],
    },
)
