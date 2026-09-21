from setuptools import setup
import os
from glob import glob

package_name = 'mapping_assignment'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name, f'{package_name}.scripts'],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
        (os.path.join('share', package_name, 'config'), glob('config/*.rviz')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    entry_points={
        'console_scripts': [
            'mapping_node = mapping_assignment.scripts.main:main',
            'play_node = mapping_assignment.scripts.play:main',
        ],
    },
)