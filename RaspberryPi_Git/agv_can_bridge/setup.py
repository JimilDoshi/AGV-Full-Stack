from setuptools import setup

package_name = 'agv_can_bridge'

setup(
    name=package_name,
    version='0.1.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    entry_points={
        'console_scripts': [
            'cmd_vel_can_bridge = agv_can_bridge.cmd_vel_can_bridge:main',
        ],
    },
)
