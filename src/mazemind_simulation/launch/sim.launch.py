"""Gazebo (Ignition Fortress) maze world + MazeMind robot + ROS bridge.

Either pass ``world:=/path/to/world.sdf x:= y:= yaw:=`` or let the launch
file generate a maze from ``difficulty``/``seed`` (see MAZE_LAUNCH_ARGS).

    ros2 launch mazemind_simulation sim.launch.py difficulty:=easy seed:=7 headless:=false
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (DeclareLaunchArgument, ExecuteProcess, IncludeLaunchDescription,
                            LogInfo, OpaqueFunction, Shutdown)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

from mazemind_simulation.launch_utils import (BRIDGE_TOPICS, GROUND_TRUTH_BRIDGE_TOPICS,
                                              MAZE_LAUNCH_ARGS, OVERHEAD_CAMERA_TOPIC,
                                              maze_from_launch_args, render_environment)
from mazemind_simulation.world_writer import WORLD_NAME


def _setup(context, *args, **kwargs):
    def get(name):
        return LaunchConfiguration(name).perform(context)

    world = get('world')
    if world:
        maze = {'world': world, 'ground_truth': '', 'x': get('x'), 'y': get('y'), 'yaw': get('yaw')}
    else:
        maze = maze_from_launch_args(get)

    headless = get('headless').lower() in ('1', 'true', 'yes')
    render_env = render_environment(get('render_mode'))
    gz_cmd = ['ign', 'gazebo', '-r']
    if headless:
        gz_cmd += ['-s', '--headless-rendering']
    gz_cmd.append(maze['world'])

    description = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(
            get_package_share_directory('mazemind_description'), 'launch', 'description.launch.py')),
        launch_arguments={'use_sim_time': 'true'}.items())

    spawn = Node(
        package='ros_gz_sim', executable='create', output='screen',
        arguments=['-world', WORLD_NAME, '-topic', 'robot_description', '-name', 'mazemind',
                   '-x', maze['x'], '-y', maze['y'], '-z', '0.01', '-Y', maze['yaw']])

    topics = BRIDGE_TOPICS + GROUND_TRUTH_BRIDGE_TOPICS
    if get('overhead_camera').lower() in ('1', 'true', 'yes'):
        topics = topics + [OVERHEAD_CAMERA_TOPIC]
    bridge = Node(
        package='ros_gz_bridge', executable='parameter_bridge', name='gz_bridge', output='screen',
        arguments=topics, parameters=[{'use_sim_time': True}])

    return [
        LogInfo(msg=f"[mazemind] world={maze['world']} "
                    f"spawn=({maze['x']}, {maze['y']}, yaw {maze['yaw']})"),
        LogInfo(msg=f'[mazemind] render environment: {render_env or "native"}'),
        ExecuteProcess(cmd=gz_cmd, output='screen', additional_env=render_env,
                       on_exit=Shutdown(reason='gazebo exited')),
        description,
        spawn,
        bridge,
    ]


def generate_launch_description():
    args = [DeclareLaunchArgument(name, default_value=default, description=desc)
            for name, (default, desc) in MAZE_LAUNCH_ARGS.items()]
    args += [
        DeclareLaunchArgument('world', default_value='', description='Existing world SDF (skips generation)'),
        DeclareLaunchArgument('x', default_value='0.5'),
        DeclareLaunchArgument('y', default_value='0.5'),
        DeclareLaunchArgument('yaw', default_value='0.0'),
        DeclareLaunchArgument('headless', default_value='false', description='Run Gazebo server only'),
        DeclareLaunchArgument('overhead_camera', default_value='false',
                              description='Bridge the top-down demo camera (recording only)'),
        DeclareLaunchArgument('render_mode', default_value='auto',
                              description='auto | gpu_override | software | native '
                                          '(see docs/troubleshooting.md)'),
    ]
    return LaunchDescription(args + [OpaqueFunction(function=_setup)])
