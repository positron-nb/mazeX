from setuptools import find_packages, setup

package_name = 'mazemind_exploration'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Abhinav Rai',
    maintainer_email='raiabhinav2905@gmail.com',
    description='Frontier exploration and exit detection for MazeMind.',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'explorer = mazemind_exploration.explorer_node:main',
            'exit_detector = mazemind_exploration.exit_detector_node:main',
        ],
    },
)
