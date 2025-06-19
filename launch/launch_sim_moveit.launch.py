from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():

    ur_type = LaunchConfiguration("ur_type")
    safety_limits = LaunchConfiguration("safety_limits")
    runtime_config_package = LaunchConfiguration("runtime_config_package")
    controllers_file = LaunchConfiguration("controllers_file")
    description_package = LaunchConfiguration("description_package")
    description_file = LaunchConfiguration("description_file")
    moveit_config_package = LaunchConfiguration("moveit_config_package")
    moveit_config_file = LaunchConfiguration("moveit_config_file")
    prefix = LaunchConfiguration("prefix")

    ur_type_arg = DeclareLaunchArgument(
        "ur_type",
        default_value="ur10",
        description="Type of Universal Robot to use (e.g., ur10, ur5, etc.)",
    )

    safety_limits_arg = DeclareLaunchArgument(
        "safety_limits",
        default_value="true",
        description="Enable safety limits for the robot",
    )

    # General arguments
    runtime_config_package_arg = DeclareLaunchArgument(
        "runtime_config_package",
        default_value="my_platform",
        description="The package containing the runtime configuration files.",
    )

    controllers_file_arg = DeclareLaunchArgument(
        "controllers_file",
        default_value="controllers.yaml",
        description="The file containing the controller configurations.",
    )

    description_package_arg = DeclareLaunchArgument(
        "description_package",
        default_value="my_platform",
        description="The package containing the robot description files.",
    )

    description_file_arg = DeclareLaunchArgument(
        "description_file",
        default_value="robot.urdf.xacro",
        description="The file containing the robot description.",
    )

    moveit_config_package_arg = DeclareLaunchArgument(
        "moveit_config_package",
        default_value="ur_moveit_config",
        description="The package containing the MoveIt configuration files.",
    )
    moveit_config_file_arg = DeclareLaunchArgument(
        "moveit_config_file",
        default_value="ur.srdf.xacro",
        description="The file containing the MoveIt configuration.",
    )

    prefix_arg = DeclareLaunchArgument(
        "prefix",
        default_value='""',
        description="Prefix for the robot links and joints, useful for multi-robot setups.",
    )

    control_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            [FindPackageShare("my_platform"), "/launch", "/launch_sim.launch.py"]
        )
    )

    ur_moveit_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            [FindPackageShare("ur_moveit_config"), "/launch", "/ur_moveit.launch.py"]
        ),
        launch_arguments={
            "ur_type": ur_type,
            "safety_limits": safety_limits,
            "description_package": description_package,
            "description_file": description_file,
            "moveit_config_package": moveit_config_package,
            "moveit_config_file": moveit_config_file,
            "prefix": prefix,
            "use_sim_time": 'true',
            "launch_rviz": 'false',
        }.items(),
    )

    return LaunchDescription([
        ur_type_arg,
        safety_limits_arg,
        runtime_config_package_arg,
        controllers_file_arg,
        description_package_arg,
        description_file_arg,
        moveit_config_package_arg,
        moveit_config_file_arg,
        prefix_arg,
        control_launch,
        ur_moveit_launch
    ])