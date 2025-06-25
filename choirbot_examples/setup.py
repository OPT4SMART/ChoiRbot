from setuptools import setup, find_packages
from glob import glob

package_name = 'choirbot_examples'
scripts = {
    'containment':      ['guidance', 'integrator', 'rviz'],
    'bearingformationcontrol': ['guidance', 'integrator' ],
    'quadrotorbearingformationcontrol': ['guidance', 'controller', 'integrator' ],
    'formationcontrol': ['guidance', 'controller', 'closestrobotgetter', 'collision'],
    'mpc':              ['guidance', 'integrator', 'rviz'],
    'taskassignment':   ['guidance', 'table', 'planner', 'controller'],
    'webots':           ['guidance', 'controller'],
    }

setup(
    name=package_name,
    version='0.0.1',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name, glob('launch/*.launch.py')),
        ('share/' + package_name, glob('resource/*.rviz')),
        ('share/' + package_name, glob('resource/*.sdf')),
        ('share/' + package_name, glob('resource/*.*')),
        # ('share/' + package_name, glob('resource/*.xacro')),
        ('share/' + package_name + '/worlds', glob('worlds/*.wbt')),
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
        ]
    },
)
