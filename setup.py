from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'comms_heartbeat_pkg'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'sounds'), glob(os.path.join('sounds', '*.*'))) # NEED TO ADD THIS FOR EVERY NEW LAUNCH FILE 

    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='jdb3',
    maintainer_email='jalen.beeman@gmail.com',
    description='TODO: Package description',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'heartbeat_reciever = comms_heartbeat_pkg.heartbeat_reciever:main',
            'heartbeat_transmitter = comms_heartbeat_pkg.heartbeat_transmitter:main'
        ],
    },
)
