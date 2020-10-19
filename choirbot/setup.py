from setuptools import find_packages
from setuptools import setup

package_name = 'choirbot'

setup(
    name=package_name,
    version='0.0.1',
    maintainer='OPT4SMART',
    maintainer_email='info@opt4smart.eu',
    description='ROS2 package for cooperative robotics',
    license='GNU General Public License v3.0',
    packages=find_packages(),
    # TODO data_files is deprecated, remove it
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    tests_require=['pytest'],
    install_requires=['setuptools', 'disropt>=0.1.7', 'recordclass>=0.5'],
    zip_safe=True,
    entry_points={
        'console_scripts': [
        ],
    },
    # we can also use "scripts" instead of declaring entry points:
    # scripts=['scripts/test_script']
    # (they must be standalone executables)
)
