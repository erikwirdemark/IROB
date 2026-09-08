from setuptools import setup, find_packages
from glob import glob
import os

package_name = 'kinematics_assignment'

setup(
    name=package_name,
    version='0.0.0',
    description='Kinematics assignment of the robotics course',
    maintainer='marco',
    maintainer_email='moletta@kth.se',
    license='BSD',
    install_requires=['setuptools'],
    zip_safe=True,
    # tests_require=['pytest'],

    # ---------- tell setuptools where the Python package lives -------------
    packages=find_packages(where='.'),          # finds   kinematics_assignment/
    package_dir={'': '.'},                      # root is current folder

    # ---------- non-Python assets copied into share/<pkg>/… -----------------
    data_files=[
        ('share/ament_index/resource_index/packages',
             [f'resource/{package_name}']),
        (f'share/{package_name}', ['package.xml']),
        (f'share/{package_name}/launch', glob('launch/*.py')),
        (f'share/{package_name}/urdf',   glob('urdf/*')),
        (f'share/{package_name}/rviz',   glob('rviz/*')),
        (f'share/{package_name}/paths',  glob('paths/*')),
    ],

    # ---------- wrapper scripts that ros2 run / ros2 launch look for --------
    entry_points = {
        "console_scripts": [
            "kuka_node = kinematics_assignment.kuka_node:main",
            "scara_node = kinematics_assignment.scara_node:main",
        ],
    },
)
