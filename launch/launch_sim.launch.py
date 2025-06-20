import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument, TimerAction, Shutdown, RegisterEventHandler
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.event_handlers import OnProcessExit

from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    package_name = 'my_platform'

    rsp = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            os.path.join(
                get_package_share_directory(package_name),
                'launch',
                'rsp.launch.py' 
            )
        ]),
        launch_arguments={'use_sim_time': 'true'}.items() 
    )

    default_world = os.path.join(
        get_package_share_directory(package_name),
        'worlds',
        'empty.world'
    )

    obstacle_world = os.path.join(
        get_package_share_directory(package_name),
        'worlds',
        'obstacle.world'
    )

    world = LaunchConfiguration('world')

    world_arg = DeclareLaunchArgument(
        'world',
        default_value=default_world,
        description='World to load in Gazebo'
    )

    gazebo_params_path = os.path.join(get_package_share_directory(package_name), 'config', 'gazebo_params.yaml')
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            os.path.join(
                get_package_share_directory('ros_gz_sim'),
                'launch',
                'gz_sim.launch.py'
            )
        ]),
        launch_arguments={'gz_args': ['--render-engine ', 'ogre2 ', '-r ', world], 
                          'extra_gazebo_args': ' --ros-args --params-file ' + gazebo_params_path}.items()
    )

    spawn_entity = Node(
        package='ros_gz_sim',
        executable='create',
        arguments=['-world', 'empty_world', '-topic', '/robot_description', '-name', 'my_platform', '-z', '0.1'],
        output='screen'
    )

    bridge_params = os.path.join(get_package_share_directory(package_name), 'config', 'gz_bridge.yaml')
    ros_gz_bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        arguments=['--ros-args', '-p', f'config_file:={bridge_params}']
    )

    rviz_config_file = PathJoinSubstitution([
        FindPackageShare(package_name),
        'rviz',
        'my_platform.rviz'
    ])
    rviz = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='log',
        arguments=["-d", rviz_config_file]
    )

    diff_drive_spawner = TimerAction(
        period=2.0, # Wait for 2 seconds before spawning the robot
        actions=[
            Node(
                package="controller_manager",
                executable="spawner",
                arguments=["diff_cont"]
            )
        ]
    )

    joint_broad_spawner = TimerAction(
        period=2.0, # Wait for 2 seconds before spawning the joint state broadcaster
        actions=[
            Node(
                package="controller_manager",
                executable="spawner",
                arguments=["joint_broad"]
            )
        ]
    )

    joint_traj_spawner = TimerAction(
        period=2.0,
        actions=[
            Node(
                package="controller_manager",
                executable="spawner",
                arguments=["joint_trajectory_controller"]
            )
        ]
    )

    gripper_cont_spawner = TimerAction(
        period=2.0,
        actions=[
            Node(
                package="controller_manager",
                executable="spawner",
                arguments=["gripper_controller"]
            )
        ]
    )

    slam_launch = TimerAction(
        period=6.0,  # Wait for 3 seconds before starting SLAM
        actions=[
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource([
                    PathJoinSubstitution([
                        FindPackageShare('slam_toolbox'),
                        'launch',
                        'online_async_launch.py'
                    ])
                ]),
                launch_arguments={
                    'slam_params_file': PathJoinSubstitution([
                        FindPackageShare('my_platform'),
                        'config',
                        'mapper_params_online_async.yaml'
                    ]),
                    'use_sim_time': 'true'
                }.items()
            )   
        ]
    )

    nav2_launch = TimerAction(
        period=6.0,  # Wait for 6 seconds before starting Navigation2
        actions=[
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource([
                    PathJoinSubstitution([
                        FindPackageShare('nav2_bringup'),
                        'launch',
                        'navigation_launch.py'
                    ])
                ]),
                launch_arguments={
                    'use_sim_time': 'true'
                }.items()
            )
        ]
    )

    twist_mux = Node(
        package='twist_mux',
        executable='twist_mux',
        parameters=[PathJoinSubstitution([
            FindPackageShare('my_platform'),
            'config',
            'twist_mux.yaml'
        ]), 
        {'use_sim_time': True}],
        remappings=[('/cmd_vel_out', '/diff_cont/cmd_vel_unstamped')]
    )

    return LaunchDescription([
        rsp, # nodes
        world_arg,
        gazebo,
        spawn_entity,
        ros_gz_bridge,
        rviz,
        diff_drive_spawner,
        joint_broad_spawner,
        joint_traj_spawner,
        gripper_cont_spawner,
        slam_launch,
        nav2_launch,
        twist_mux
    ])
