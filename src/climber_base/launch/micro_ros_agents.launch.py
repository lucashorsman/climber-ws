"""
micro_ros_agents.launch.py

Launches 4 micro-ROS agents — one per MCU arm (N, W, S, E).
Each agent bridges a serial port to the ROS 2 graph.

Launch arguments allow overriding the serial device for each arm
(defaults expect udev symlinks: /dev/climber_{n,w,s,e}).
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    arms = ['n', 'w', 's', 'e']
    # arms = ['n']

    declared_arguments = []
    agent_nodes = []

    for arm in arms:
        # Declare serial device argument for each arm
        arg_name = f'{arm}_serial_dev'
        declared_arguments.append(
            DeclareLaunchArgument(
                arg_name,
                default_value=f'/dev/climber_{arm}',
                description=f'Serial device for {arm.upper()} MCU'
            )
        )

        # Declare baud rate argument
        baud_arg = f'{arm}_baud'
        declared_arguments.append(
            DeclareLaunchArgument(
                baud_arg,
                default_value='115200',
                description=f'Baud rate for {arm.upper()} MCU serial link'
            )
        )

        # micro-ROS agent node
        agent_nodes.append(
            Node(
                package='micro_ros_agent',
                executable='micro_ros_agent',
                name=f'micro_ros_agent_{arm}',
                arguments=[
                    'serial',
                    '--dev', LaunchConfiguration(arg_name),
                    '-b', LaunchConfiguration(baud_arg),
                ],
                output='screen',
            )
        )


    return LaunchDescription(declared_arguments + agent_nodes)
