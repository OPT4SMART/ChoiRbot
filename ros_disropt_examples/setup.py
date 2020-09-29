from setuptools import setup
from glob import glob

package_name = 'ros_disropt_examples'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name, glob('launch/*.launch.py')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='OPT4SMART',
    maintainer_email='info@opt4smart.eu',
    description='Example files for ros_disropt',
    license='GNU General Public License v3.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'ros_disropt_agent_i = scripts.main_i:main',
            'ros_disropt_guidance_i = scripts.guidance_i:main_guidance',
            'ros_disropt_table = scripts.guidance_i:main_table',
            'ros_disropt_planner= scripts.planner_i:main_planner',
            'ros_disropt_controller= scripts.controller:main',
            'ros_disropt_singleintplanner = scripts.singleint_planner_i:main'
        ],
    },
)
