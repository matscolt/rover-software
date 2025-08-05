from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'gorm_web_interface'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob(os.path.join('launch', '*launch.[pxy][yma]*'))),
        (os.path.join('share', package_name, 'web'), glob(os.path.join('web', '*'))),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='GORM Team',
    maintainer_email='todo@todo.com',
    description='Web interface for GORM rover motor control and monitoring',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'web_server = gorm_web_interface.web_server:main',
        ],
    },
)
