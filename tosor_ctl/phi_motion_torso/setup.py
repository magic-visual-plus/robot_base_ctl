from setuptools import setup
import os
from glob import glob

package_name = 'phi_motion_torso'

setup(
    name=package_name,
    version='0.1.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'),
            glob('launch/*.launch.py')),
        (os.path.join('share', package_name, 'config'),
            glob('config/*.yaml')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Phi Robot Team',
    maintainer_email='robot@example.com',
    description='Torso control node for Phi robot motion system',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'torso_node = phi_motion_torso.torso_node:main',
            'torso_teleop_node = phi_motion_torso.torso_teleop_node:main',
            'torso_teleop_pub = phi_motion_torso.torso_teleop_pub:main',
        ],
    },
)
