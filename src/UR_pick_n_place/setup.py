from setuptools import setup
import os
from glob import glob

package_name = 'UR_pick_n_place'

setup(
    name=package_name,
    version='0.1.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'config'),
            glob('config/*.yaml')),
        (os.path.join('share', package_name, 'launch'),
            glob('launch/*.py')),
    ],
    install_requires=['setuptools'],
    entry_points={
        'console_scripts': [
            'pnp_routine = UR_pick_n_place.pnp_node:main',
        ],
    },
)