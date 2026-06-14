import os
from glob import glob

from setuptools import find_packages, setup

package_name = 'omnidriver'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/assets', ['assets/dashboard.html']),
        ('share/' + package_name + '/assets/js', ['assets/js/chart.umd.min.js', 'assets/js/socket.io.min.js']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
        (os.path.join('share', package_name, 'config'), glob('config/*.yaml')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='roger',
    maintainer_email='joserogelioruiz12@gmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    entry_points={
        'console_scripts': [
                'odrive_dashboard = odrive_comm.odrive_dashboard:main',
                'odrive_serial_twist = odrive_comm.odrive_serial_twist:main',
                'simple_rx = odrive_comm.simple_rx:main',
        ],
    },
)
