import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.substitutions import LaunchConfiguration, Command, FindExecutable, PathJoinSubstitution
from launch.actions import DeclareLaunchArgument

from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

import xacro


def generate_launch_description():

    # Check if we're told to use sim time
    use_sim_time = LaunchConfiguration('use_sim_time')
    ur_type = LaunchConfiguration('ur_type')
    safety_limits = LaunchConfiguration('safety_limits')
    safety_pos_limits = LaunchConfiguration('safety_pos_limits')
    safety_k_position = LaunchConfiguration('safety_k_position')
    prefix = LaunchConfiguration('prefix')
    initial_joint_controller = LaunchConfiguration('initial_joint_controller')

    use_sim_time_arg = DeclareLaunchArgument(
        'use_sim_time',
        default_value='true',
        description='Use simulation time if true'
    )
    ur_type_arg = DeclareLaunchArgument(
        'ur_type',
        default_value='ur10',
    )
    safety_limits_arg = DeclareLaunchArgument(
        'safety_limits',
        default_value='true',
        description='Enables the safety limits for the robot'
    )
    safety_pos_limits_arg = DeclareLaunchArgument(
        'safety_pos_limits',
        default_value='0.15',
        description='The margin to lower and upper limits in the safety controller.'
    )
    safety_k_position_arg = DeclareLaunchArgument(
        'safety_k_position',
        default_value='20',
        description='k-position factor in the safety controller.'
    )
    # TODO: Need to change when we put all controllers in a single file
    runtime_config_package_arg = DeclareLaunchArgument(
        'runtime_config_package',
        default_value='my_ur',
        description='The package containing the runtime configuration files.'
    )
    controllers_file_arg = DeclareLaunchArgument(
        'controllers_file',
        default_value='ur_controllers.yaml',
        description='The file containing the controller configurations.'
    )
    description_package_arg = DeclareLaunchArgument(
        'description_package',
        default_value='my_platform',
        description='The package containing the robot description files.'
    )
    description_file_arg = DeclareLaunchArgument(
        'description_file',
        default_value='robot.urdf.xacro',
        description='The URDF/XACRO file for the robot description.'
    )
    prefix_arg = DeclareLaunchArgument(
        'prefix',
        default_value='""',
        description='Prefix for the robot name, useful for multi-robot setups.'
    )
    start_joint_controller_arg = DeclareLaunchArgument(
        'start_joint_controller',
        default_value='true',
        description='Enable headless mode for robot control.'
    )
    initial_joint_controller_arg = DeclareLaunchArgument(
        'initial_joint_controller',
        default_value='joint_trajectory_controller',
        description='Robot controller to start.'
    )

    # Process the URDF file
    pkg_path = os.path.join(get_package_share_directory('my_platform'))
    xacro_file = os.path.join(pkg_path, 'urdf','robot.urdf.xacro') # for original robot
    # robot_description_config = xacro.process_file(xacro_file)
    robot_description_content = Command(
        [
            PathJoinSubstitution([FindExecutable(name="xacro")]),
            " ",
            xacro_file,
            " ",
            "safety_limits:=", safety_limits,
            " ",
            "safety_pos_limits:=", safety_pos_limits,
            " ",
            "safety_k_position:=", safety_k_position,
            " ",
            "name:=ur",
            " ",
            "ur_type:=", ur_type,
            " ",
            "prefix:=", prefix,
            " ",
            "sim_ignition:=true",
            " ",
            "simulation_controllers:=", initial_joint_controller,
        ]
    )

    

    
    # Create a robot_state_publisher node
    # params = {'robot_description': robot_description_config.toxml(), 'use_sim_time': use_sim_time}
    robot_description = {"robot_description": robot_description_content}
    node_robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[{"use_sim_time": True}, robot_description],
    )


    # Launch!
    return LaunchDescription([
        use_sim_time_arg,
        ur_type_arg,
        safety_limits_arg,
        safety_pos_limits_arg,
        safety_k_position_arg,
        runtime_config_package_arg,
        controllers_file_arg,
        description_package_arg,
        description_file_arg,
        prefix_arg,
        start_joint_controller_arg,
        initial_joint_controller_arg,
        node_robot_state_publisher
    ])
