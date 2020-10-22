from setuptools import setup, find_packages
from glob import glob

package_name = 'choirbot_examples'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name, glob('launch/*.launch.py')),
        ('share/' + package_name, glob('rviz/*.rviz')),
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
            'choirbot_task_guidance = choirbot_examples.task_assignment:main_guidance',
            'choirbot_table = choirbot_examples.task_assignment:main_table',
            'choirbot_planner= choirbot_examples.planner_i:main_planner',
            'choirbot_controller= choirbot_examples.controller:main',
            'choirbot_unicycle_vel= choirbot_examples.unicycle_controller:main',
            'choirbot_singleintegrator = choirbot_examples.single_integrator_i:main',
            'choirbot_unicycleintegrator = choirbot_examples.unicycle_integrator_i:main',
            'choirbot_formationcontrol = choirbot_examples.formationcontrol_guidance_i:main',
            'choirbot_containment = choirbot_examples.containment_guidance_i:main',
            'choirbot_rviz = choirbot_examples.rviz_spawner_i:main'
        ],
    },
)
