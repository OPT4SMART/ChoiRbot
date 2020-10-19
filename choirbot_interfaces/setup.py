from setuptools import setup

package_name = 'choirbot_interfaces'

setup(
    name=package_name,
    version='0.0.1',
    maintainer='OPT4SMART',
    maintainer_email='info@opt4smart.eu',
    description='Interfaces for ChoiRbot',
    license='GNU General Public License v3.0',
    packages=[package_name],
    data_files=[
        ('share/' + package_name, ['package.xml'])
    ],
    install_requires=['setuptools'],
    zip_safe=True
)
