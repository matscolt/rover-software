from setuptools import find_packages, setup

package_name= 'my_ros2_pkg'

setup(
	name=package_name,
	version='0.0.1',
	packages=find_packages(exclude=['test']),
	data_files=[
		('share/ament_index/resource_index/packages',
			['resource/' + package_name]),
		('share/' + package_name, ['package.xml']),
	],
	install_requires=['setuptools'],
	zip_safe=True,
	maintainer='Victor',
	maintainer_email='dig@example.com',
	description='Min første ROS2 pakke i Docker',
	entry_points={
		'console_scripts':[
			'talker = my_ros2_pkg.publisher:main',
			'listener = my_ros2_pkg.subscriber:main',
		]
	}
)
