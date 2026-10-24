"""MazeMind full stack: Gazebo maze + robot, SLAM Toolbox, Nav2, exploration, evaluation, RViz.

    # Full autonomous run with GUI + RViz
    ros2 launch mazemind_bringup mazemind.launch.py difficulty:=easy seed:=7 strategy:=weighted

    # Manual Nav2 testing only (no exploration / evaluation)
    ros2 launch mazemind_bringup mazemind.launch.py explore:=false evaluate:=false

Information boundary: the ground-truth maze file produced here is passed
ONLY to the evaluator node. The explorer and exit detector receive the
same parameters regardless of which maze is loaded.
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (DeclareLaunchArgument, IncludeLaunchDescription, LogInfo,
                            OpaqueFunction, Shutdown, TimerAction)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

from mazemind_simulation.launch_utils import (MAZE_LAUNCH_ARGS, maze_from_launch_args,
                                              rviz_environment)


def _truthy(value: str) -> bool:
    return value.lower() in ('1', 'true', 'yes', 'on')


def _setup(context, *args, **kwargs):
    def get(name):
        return LaunchConfiguration(name).perform(context)

    bringup = get_package_share_directory('mazemind_bringup')
    maze = maze_from_launch_args(get)

    sim = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(
            get_package_share_directory('mazemind_simulation'), 'launch', 'sim.launch.py')),
        launch_arguments={'world': maze['world'], 'x': maze['x'], 'y': maze['y'],
                          'yaw': maze['yaw'], 'headless': get('headless'),
                          'render_mode': get('render_mode'),
                          'overhead_camera': get('record')}.items())

    slam = Node(
        package='slam_toolbox', executable='async_slam_toolbox_node', name='slam_toolbox',
        output='screen',
        parameters=[os.path.join(bringup, 'config', 'slam_toolbox.yaml'), {'use_sim_time': True}])

    nav2 = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(
            get_package_share_directory('nav2_bringup'), 'launch', 'navigation_launch.py')),
        launch_arguments={'use_sim_time': 'true', 'autostart': 'true',
                          'params_file': os.path.join(bringup, 'config', 'nav2.yaml')}.items())

    actions = [
        LogInfo(msg=f"[mazemind] maze world {maze['world']}"),
        sim,
        TimerAction(period=3.0, actions=[slam]),
        TimerAction(period=5.0, actions=[nav2]),
    ]

    if _truthy(get('explore')):
        explorer_params = os.path.join(bringup, 'config', 'exploration.yaml')
        actions.append(TimerAction(period=8.0, actions=[
            Node(package='mazemind_exploration', executable='exit_detector', name='exit_detector',
                 output='screen', parameters=[explorer_params, {'use_sim_time': True}]),
            Node(package='mazemind_exploration', executable='explorer', name='explorer',
                 output='screen',
                 parameters=[explorer_params, {'use_sim_time': True,
                                               'strategy': get('strategy'),
                                               'random_seed': int(get('seed')),
                                               'time_limit': float(get('time_limit'))}]),
        ]))

    if _truthy(get('evaluate')):
        end_run = _truthy(get('shutdown_on_finish'))
        actions.append(Node(
            package='mazemind_evaluation', executable='evaluator', name='evaluator', output='screen',
            parameters=[{'use_sim_time': True,
                         'ground_truth_file': maze['ground_truth'],
                         'run_id': get('run_id'),
                         'strategy': get('strategy'),
                         'results_dir': os.path.expanduser(get('results_dir')),
                         'time_limit': float(get('time_limit')) + 30.0}],
            on_exit=Shutdown(reason='evaluation finished') if end_run else None))

    if _truthy(get('record')):
        record_path = get('record_path') or os.path.join(
            '~', '.mazemind', 'media', f"{get('run_id')}_{get('strategy')}_s{get('seed')}.mp4")
        actions.append(Node(
            package='mazemind_evaluation', executable='media_recorder', name='media_recorder',
            output='screen',
            parameters=[{'use_sim_time': True, 'output': os.path.expanduser(record_path),
                         'timelapse': float(get('record_timelapse')),
                         'display_yaw': float(maze['yaw']),
                         'title': f"MazeMind | {get('difficulty')} maze, seed {get('seed')}"}]))

    if _truthy(get('rviz')):
        actions.append(Node(
            package='rviz2', executable='rviz2', name='rviz2', output='log',
            arguments=['-d', os.path.join(bringup, 'rviz', 'mazemind.rviz')],
            parameters=[{'use_sim_time': True}],
            additional_env=rviz_environment(get('rviz_render_mode'))))
    return actions


def generate_launch_description():
    args = [DeclareLaunchArgument(name, default_value=default, description=desc)
            for name, (default, desc) in MAZE_LAUNCH_ARGS.items()]
    args += [
        DeclareLaunchArgument('headless', default_value='false'),
        DeclareLaunchArgument('render_mode', default_value='auto'),
        DeclareLaunchArgument('rviz', default_value='true'),
        DeclareLaunchArgument('rviz_render_mode', default_value='auto',
                              description='auto (software on WSL) | gpu_override | software | native'),
        DeclareLaunchArgument('explore', default_value='true'),
        DeclareLaunchArgument('evaluate', default_value='true'),
        DeclareLaunchArgument('strategy', default_value='weighted',
                              description='random | nearest | weighted'),
        DeclareLaunchArgument('time_limit', default_value='600.0',
                              description='Exploration time limit (simulated seconds)'),
        DeclareLaunchArgument('run_id', default_value='manual'),
        DeclareLaunchArgument('results_dir', default_value='~/.mazemind/results'),
        DeclareLaunchArgument('record', default_value='false',
                              description='Record a side-by-side demo video (overhead sim view + SLAM)'),
        DeclareLaunchArgument('record_timelapse', default_value='4',
                              description='Video speed-up: video seconds = sim seconds / this'),
        DeclareLaunchArgument('record_path', default_value='',
                              description='Output .mp4 (default ~/.mazemind/media/<run>.mp4)'),
        DeclareLaunchArgument('shutdown_on_finish', default_value='false',
                              description='Stop the whole launch once the evaluator writes its result'),
    ]
    return LaunchDescription(args + [OpaqueFunction(function=_setup)])
