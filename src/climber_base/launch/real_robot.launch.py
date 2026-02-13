"""
real_robot.launch.py

Launches the full real-robot stack:
  1. Robot State Publisher (URDF with real hardware interface)
  2. ros2_control controller_manager (loads ClimberHardwareInterface)
  3. Joint State Broadcaster
  4. Velocity Controller (wheel spin)
  5. Position Controller (linear actuators)
  6. Cylinder Climb Controller (cmd_vel → wheel/actuator commands)
  7. micro-ROS agents (serial bridge to 4 MCUs)
  8. (Optional) RViz

The ClimberHardwareInterface subscribes to /mcu_*/arm_state and
publishes to /mcu_*/arm_cmd internally, bridging MCU data into the
ros2_control loop.
"""

import os
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    RegisterEventHandler,
)
from launch.conditions import IfCondition
from launch.event_handlers import OnProcessExit
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import (
    Command,
    FindExecutable,
    LaunchConfiguration,
    PathJoinSubstitution,
)
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch_ros.parameter_descriptions import ParameterValue
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    # ── Arguments ──────────────────────────────────────────────────
    declared_arguments = [
        DeclareLaunchArgument(
            'use_rviz', default_value='true',
            description='Launch RViz2 for visualization'),
        DeclareLaunchArgument(
            'launch_agents', default_value='true',
            description='Launch micro-ROS serial agents'),
    ]

    pkg_climber_base = get_package_share_directory('climber_base')
    controllers_yaml = os.path.join(pkg_climber_base, 'config', 'climber_controllers.yaml')

    # ── Robot description (real hardware) ──────────────────────────
    robot_description_content = Command([
        PathJoinSubstitution([FindExecutable(name='xacro')]),
        ' ',
        PathJoinSubstitution([
            FindPackageShare('climber_base'), 'urdf', 'climber_robot.urdf.xacro'
        ]),
        ' use_sim:=false',
        ' sim_gz:=false',
    ])

    robot_description = {
        'robot_description': ParameterValue(robot_description_content, value_type=str)
    }

    # ── Robot State Publisher ──────────────────────────────────────
    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[robot_description, {'use_sim_time': False}],
    )

    # ── ros2_control Controller Manager ───────────────────────────
    # When use_sim:=false the URDF loads ClimberHardwareInterface.
    # controller_manager reads the URDF from /robot_description
    # and loads the hardware plugin automatically.
    control_node = Node(
        package='controller_manager',
        executable='ros2_control_node',
        parameters=[robot_description, controllers_yaml],
        output='screen',
    )

    # ── Controller spawners ───────────────────────────────────────
    jsb_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['joint_state_broadcaster', '--controller-manager', '/controller_manager'],
        output='screen',
    )

    velocity_controller_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['velocity_controller', '--controller-manager', '/controller_manager'],
        output='screen',
    )

    position_controller_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['position_controller', '--controller-manager', '/controller_manager'],
        output='screen',
    )

    delay_controllers = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=jsb_spawner,
            on_exit=[velocity_controller_spawner, position_controller_spawner],
        )
    )

    # ── Cylinder Climb Controller ─────────────────────────────────
    climb_controller_node = Node(
        package='climber_base',
        executable='cylinder_climb_controller.py',
        name='cylinder_climb_controller',
        parameters=[
            {'use_sim_time': False},
            {'wheel_radius': 0.06},
            {'cylinder_radius': 0.075},
            {'use_real_hardware': True},
        ],
        output='screen',
    )

    delay_climb_controller = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=jsb_spawner,
            on_exit=[climb_controller_node],
        )
    )

    # ── micro-ROS Agents ──────────────────────────────────────────
    micro_ros_agents = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_climber_base, 'launch', 'micro_ros_agents.launch.py')
        ),
        condition=IfCondition(LaunchConfiguration('launch_agents')),
    )

    # ── RViz ──────────────────────────────────────────────────────
    rviz_config_file = PathJoinSubstitution([
        FindPackageShare('climber_base'), 'config', 'view_robot.rviz'
    ])
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', rviz_config_file],
        parameters=[{'use_sim_time': False}],
        condition=IfCondition(LaunchConfiguration('use_rviz')),
        output='screen',
    )

    return LaunchDescription(declared_arguments + [
        robot_state_publisher_node,
        control_node,
        jsb_spawner,
        delay_controllers,
        delay_climb_controller,
        micro_ros_agents,
        rviz_node,
    ])
