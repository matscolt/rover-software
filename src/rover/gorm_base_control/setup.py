from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'gorm_base_control'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob(os.path.join('launch', '*launch.[pxy][yma]*'))),
        (os.path.join('share', package_name, 'config'), glob(os.path.join('config', '*'))),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Anton Bjørndahl Mortensen',
    maintainer_email='antonbm2008@gmail.com',
    description='The gorm_base_control package',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'ackermann_node = gorm_base_control.ackermann_node:main',
            'motor_driver_node = gorm_base_control.motor_driver_node:main',
        ],
    },
)
