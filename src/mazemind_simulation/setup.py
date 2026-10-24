from glob import glob

from setuptools import find_packages, setup

package_name = 'mazemind_simulation'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', glob('launch/*.launch.py')),
        ('share/' + package_name + '/config', glob('config/*')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Abhinav Rai',
    maintainer_email='raiabhinav2905@gmail.com',
    description='Procedural maze generation and Gazebo simulation for MazeMind.',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'generate_maze = mazemind_simulation.generate_cli:main',
        ],
    },
)
