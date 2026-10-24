from setuptools import find_packages, setup

package_name = 'mazemind_evaluation'

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
    description='Evaluation, experiment runner and analysis for MazeMind.',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'evaluator = mazemind_evaluation.evaluator_node:main',
            'run_experiments = mazemind_evaluation.run_experiments:main',
            'analyze_results = mazemind_evaluation.analyze_results:main',
            'media_recorder = mazemind_evaluation.media_recorder_node:main',
        ],
    },
)
