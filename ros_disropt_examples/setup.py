from setuptools import setup, find_packages
from glob import glob

package_name = 'ros_disropt_examples'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name, glob('launch/*.launch.py')),
    ],
    install_requires=['setuptools', 'ros_disropt'],
    zip_safe=True,
    maintainer='OPT4SMART',
    maintainer_email='info@opt4smart.eu',
    description='Example files for ros_disropt',
    license='GNU General Public License v3.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'ros_disropt_agent_i = ros_disropt_examples.main_i:main',
            'ros_disropt_guidance_i = ros_disropt_examples.guidance_i:main_guidance',
            'ros_disropt_table = ros_disropt_examples.guidance_i:main_table',
            'ros_disropt_planner= ros_disropt_examples.planner_i:main_planner',
            'ros_disropt_controller= ros_disropt_examples.controller:main',
            'ros_disropt_singleintegrator = ros_disropt_examples.single_integrator_i:main',
            'ros_disropt_formationcontrol = ros_disropt_examples.formationcontrol_guidance_i:main',
            'ros_disropt_containment = ros_disropt_examples.containment_guidance_i:main',
            'ros_disropt_rviz = ros_disropt_examples.rviz_spawner_i:main'
        ],
    },
)
