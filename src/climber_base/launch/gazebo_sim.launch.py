import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, RegisterEventHandler
from launch.conditions import IfCondition
from launch.event_handlers import OnProcessExit
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, FindExecutable, LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch_ros.parameter_descriptions import ParameterValue
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    # Declare arguments
    declared_arguments = []
    
    declared_arguments.append(
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='true',
            description='Use simulation (Gazebo) clock if true'
        )
    )
    
    declared_arguments.append(
        DeclareLaunchArgument(
            'use_rviz',
            default_value='true',
            description='Start RViz2 for visualization'
        )
    )
    
    declared_arguments.append(
        DeclareLaunchArgument(
            'world',
            default_value='empty.sdf',
            description='Gazebo world file'
        )
    )

    # Get paths
    pkg_climber_base = get_package_share_directory('climber_base')
    pkg_ros_gz_sim = get_package_share_directory('ros_gz_sim')
    
    # World file - use our cylinder climb world
    world_file = os.path.join(pkg_climber_base, 'urdf', 'cylinder_climb.world')
    
    # Get URDF via xacro
    robot_description_content = Command([
        PathJoinSubstitution([FindExecutable(name='xacro')]),
        ' ',
        PathJoinSubstitution([
            FindPackageShare('climber_base'),
            'urdf',
            'climber_robot.urdf.xacro'
        ]),
        ' use_sim:=true',
        ' sim_gz:=true',
    ])
    
    robot_description = {
        'robot_description': ParameterValue(robot_description_content, value_type=str)
    }
    
    # Robot State Publisher
    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[
            robot_description,
            {'use_sim_time': LaunchConfiguration('use_sim_time')}
        ]
    )
    
    # Gazebo Sim
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_ros_gz_sim, 'launch', 'gz_sim.launch.py')
        ),
        launch_arguments={
            'gz_args': ['-r -v4 ', world_file],
            'on_exit_shutdown': 'true'
        }.items()
    )
    
    # Spawn robot in Gazebo - position robot around the pole
    # The pole is at (0,0), spawn robot centered on it slightly above ground
    spawn_entity = Node(
        package='ros_gz_sim',
        executable='create',
        arguments=[
            '-name', 'climber_robot',
            '-topic', 'robot_description',
            '-x', '0.0',
            '-y', '0.0',
            '-z', '0.3',
        ],
        output='screen'
    )
    
    # Joint State Broadcaster spawner
    joint_state_broadcaster_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['joint_state_broadcaster', '--controller-manager', '/controller_manager'],
        output='screen',
    )
    
    # Velocity Controller spawner
    velocity_controller_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['velocity_controller', '--controller-manager', '/controller_manager'],
        output='screen',
    )
    
    # Position Controller spawner (linear actuators)
    position_controller_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['position_controller', '--controller-manager', '/controller_manager'],
        output='screen',
    )

    # Delay velocity + position controller spawners after joint_state_broadcaster
    delay_velocity_controller_spawner = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=joint_state_broadcaster_spawner,
            on_exit=[velocity_controller_spawner, position_controller_spawner],
        )
    )
    
    # ROS-Gazebo bridge
    ros_gz_bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        arguments=[
            '/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock',
        ],
        output='screen'
    )
    
    # Cylinder Climb Controller node (maps cmd_vel -> wheel velocities)
    climb_controller_node = Node(
        package='climber_base',
        executable='cylinder_climb_controller.py',
        name='cylinder_climb_controller',
        parameters=[
            {'use_sim_time': LaunchConfiguration('use_sim_time')},
            {'wheel_radius': 0.06},
            {'cylinder_radius': 0.075},
        ],
        output='screen',
    )
    
    # Delay climb controller until velocity controller is up
    delay_climb_controller = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=joint_state_broadcaster_spawner,
            on_exit=[climb_controller_node],
        )
    )
    
    # RViz
    rviz_config_file = PathJoinSubstitution([
        FindPackageShare('climber_base'),
        'config',
        'view_robot.rviz'
    ])
    
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', rviz_config_file],
        parameters=[{'use_sim_time': LaunchConfiguration('use_sim_time')}],
        condition=IfCondition(LaunchConfiguration('use_rviz')),
        output='screen'
    )
    
    nodes = [
        robot_state_publisher_node,
        gazebo,
        spawn_entity,
        joint_state_broadcaster_spawner,
        delay_velocity_controller_spawner,
        ros_gz_bridge,
        delay_climb_controller,
        rviz_node,
    ]
    
    return LaunchDescription(declared_arguments + nodes)
