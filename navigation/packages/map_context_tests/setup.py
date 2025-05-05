from setuptools import find_packages, setup
from glob import glob
package_name = 'map_context_tests'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', glob('launch/*.launch.py')),
        ('share/' + package_name + '/rviz', glob('rviz/*.rviz')),
        ('share/' + package_name + '/data', glob('data/*.yaml')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='roger',
    maintainer_email='joserogelioruiz12@gmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'map_tagger = map_context_tests.map_tagger:main'

        ],
    },
)
