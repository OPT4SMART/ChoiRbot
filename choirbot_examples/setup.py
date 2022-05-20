from setuptools import setup, find_packages
from glob import glob

package_name = 'choirbot_examples'
scripts = {
    'containment':      ['guidance', 'integrator', 'rviz'],
    'bearingformationcontrol': ['guidance', 'integrator' ],
    'quadrotorbearingformationcontrol': ['guidance', 'controller', 'integrator' ],
    'formationcontrol': ['guidance', 'controller'],
    'mpc':              ['guidance', 'integrator', 'rviz'],
    'taskassignment':   ['guidance', 'table', 'planner', 'controller'],
    }

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name, glob('launch/*.launch.py')),
        ('share/' + package_name, glob('resource/*.rviz')),
        ('share/' + package_name, glob('resource/*.sdf')),
    ],
    install_requires=['setuptools', 'choirbot'],
    zip_safe=True,
    maintainer='OPT4SMART',
    maintainer_email='info@opt4smart.eu',
    description='Example files for ChoiRbot',
    license='GNU General Public License v3.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'choirbot_{0}_{1} = choirbot_examples.{0}.{1}:main'.format(package, file)
            for package, files in scripts.items() for file in files
        ] + [
            'choirbot_turtlebot_spawner = choirbot_examples.turtlebot_spawner:main'
        ],
    },
)
