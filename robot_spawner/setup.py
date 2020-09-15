from setuptools import setup
from setuptools import find_packages
from glob import glob

package_name = 'robot_spawner'

setup(
    name=package_name,
    version='0.0.1',
    maintainer='OPT4SMART',
    maintainer_email='info@opt4smart.eu',
    description='Gazebo robot spawning utilities',
    license='GNU General Public License v3.0',
    packages=find_packages(),
    install_requires=['setuptools'],
    zip_safe=True,
    tests_require=['pytest'],
    data_files=[
	('share/ament_index/resource_index/packages', ['resource/' + package_name]),
    ('share/' + package_name + '/models', glob('resource/model*.sdf')),
	('share/' + package_name, ['package.xml']),
    ('share/' + package_name + '/launch/', glob('launch/*.launch.py')),
    ],
    entry_points={
        'console_scripts': [
            'spawn_turtlebot = robot_spawner.spawn_turtlebot:main',
        ],
    },
)
