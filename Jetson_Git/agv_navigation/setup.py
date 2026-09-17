from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'agv_navigation'

setup(
    name=package_name,
    version='1.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'),
            glob('launch/*.py')),
        (os.path.join('share', package_name, 'config'),
            glob('config/*.yaml')),
        (os.path.join('share', package_name, 'urdf'),
            glob('urdf/*')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='AGV Team',
    maintainer_email='maintainer@example.com',
    description='Jetson-side SLAM, Nav2, and YOLO safety for the AGV.',
    license='Proprietary',
    entry_points={
        'console_scripts': [
            'yolo_safety_node = agv_navigation.yolo_safety_node:main',
        ],
    },
)
