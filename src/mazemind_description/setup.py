from glob import glob

from setuptools import find_packages, setup

package_name = 'mazemind_description'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/urdf', glob('urdf/*')),
        ('share/' + package_name + '/launch', glob('launch/*.launch.py')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Abhinav Rai',
    maintainer_email='raiabhinav2905@gmail.com',
    description='URDF description of the MazeMind robot.',
    license='MIT',
    tests_require=['pytest'],
)
