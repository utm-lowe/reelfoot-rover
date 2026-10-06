from glob import glob
from setuptools import setup

package_name = 'reelfoot'

setup(
    name=package_name,
    version='0.1.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', glob('launch/*.launch.py')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Robert Lowe',
    maintainer_email='you@utm.edu',
    description='Reelfoot Rover sensors and bringup: sonar ring and lidar',
    license='MIT',
    entry_points={
        'console_scripts': [
            'sonar_ring = reelfoot.sonar_ring_node:main',
            'drive = reelfoot.drive_node:main',
        ],
    },
)
