from glob import glob
import os

from setuptools import find_packages, setup

package_name = 'c305_sim_adapter'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
         ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='wjg',
    maintainer_email='wjg@example.com',
    description='VehicleState to c305 Gazebo interface adapter.',
    license='Apache-2.0',
    entry_points={
        'console_scripts': [
            'sim_adapter = c305_sim_adapter.sim_adapter:main',
        ],
    },
)
