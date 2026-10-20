import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():

    use_sim_time = LaunchConfiguration('use_sim_time')
    namespace = LaunchConfiguration('namespace')
    use_joy_node = LaunchConfiguration('use_joy_node')
    joy_params = os.path.join(
        get_package_share_directory('auracle_bringup'), 'config', 'joystick.yaml'
    )

    # Reads a controller plugged into / paired with THIS machine. Turn off
    # (use_joy_node:=false) when /joy comes from somewhere else, e.g. the
    # Windows->WSL UDP bridge, so it doesn't complain about a missing device.
    joy_node = Node(
        package='joy',
        executable='joy_node',
        name='joy_node',
        namespace=namespace,
        parameters=[joy_params, {'use_sim_time': use_sim_time}],
        condition=IfCondition(use_joy_node),
    )

    # Replaces teleop_twist_joy: triggers/d-pad/face buttons/stick -> smooth Twist.
    drive_node = Node(
        package='auracle_bringup',
        executable='joy_drive.py',
        name='joy_drive_node',
        namespace=namespace,
        parameters=[joy_params, {'use_sim_time': use_sim_time}],
        # twist_mux.yaml expects the joystick's twist on this relative
        # topic; feeds into twist_mux -> cmd_vel_unstamped -> twist_stamper.
        remappings=[('cmd_vel', 'cmd_vel_joy')],
        output='screen',
    )

    return LaunchDescription([
        DeclareLaunchArgument('use_sim_time', default_value='false'),
        DeclareLaunchArgument('namespace', default_value=''),
        DeclareLaunchArgument('use_joy_node', default_value='true',
                              description='false = /joy is published by something else'),
        joy_node,
        drive_node,
    ])