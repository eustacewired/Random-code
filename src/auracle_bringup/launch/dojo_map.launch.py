"""Starts dojo_map.py in --watch mode alongside the robot bringup.

Needs dojo_map.py installed into the package (see README, "Robot side, option B").
  ros2 launch auracle_bringup dojo_map.launch.py server:=http://<judges-ip>:8000
If the judges set a token, export DOJO_TOKEN=<token> first.
"""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('server', description='Judges server, e.g. http://192.168.1.10:8000'),
        DeclareLaunchArgument('team', default_value='Auracle'),
        DeclareLaunchArgument('topic', default_value='/map'),
        ExecuteProcess(
            cmd=['ros2', 'run', 'auracle_bringup', 'dojo_map.py',
                 '--server', LaunchConfiguration('server'),
                 '--team', LaunchConfiguration('team'),
                 '--topic', LaunchConfiguration('topic'),
                 '--watch'],
            output='screen'),
    ])
