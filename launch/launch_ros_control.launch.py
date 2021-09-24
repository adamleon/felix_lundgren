from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.actions.include_launch_description import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, Command, ThisLaunchFileDir, PathJoinSubstitution, FindExecutable
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    declare_launch_arguments = [
        DeclareLaunchArgument(
            'robot_description_file',
            default_value="lundgren_cell.urdf.xacro",
            description="The file being loaded. Relative to <pachage_name>/urdf."
        ),

        DeclareLaunchArgument(
            "package_name",
            default_value="felix_lundgren",
            description="The package which contains all the information to launch the robot"
        )
    ]

    package_name = LaunchConfiguration("package_name")

    # Load the robot description file
    robot_description_file = LaunchConfiguration("robot_description_file")
    robot_description = {"robot_description":Command(
        [
            PathJoinSubstitution([FindExecutable(name="xacro")]),
            " ",
            PathJoinSubstitution([FindPackageShare(package_name),
            "urdf", robot_description_file]),
        ])}
    
    launch_robot = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([ThisLaunchFileDir(), "/launch_robot.launch.py"]),
        launch_arguments={
            "package_name": package_name,
            "gui": "false",
            "robot_description_file": robot_description_file,
        }.items(),
    )
    
    robot_controllers = PathJoinSubstitution(
        [FindPackageShare(package_name), "ros2_control/ros2_control_controllers.yaml"]
    )

    print(robot_controllers)

    # Nodes
    controller_manager = Node(
            package="controller_manager",
            executable="ros2_control_node",
            parameters=[robot_description, robot_controllers],
            output={
                "stdout": "screen",
                "stderr": "screen",
            }
        )

    joint_state_broadcaster = Node(
            package="controller_manager",
            executable="spawner.py",
            arguments=["lundgren_joint_state_broadcaster", "--controller-manager", "/controller_manager"],
        )

    position_trajectory_controller = Node(
            package="controller_manager",
            executable="spawner.py",
            arguments=["lundgren_position_trajectory_controller", "-c", "/controller_manager"],
        )         

    nodes_to_start = [controller_manager, joint_state_broadcaster, position_trajectory_controller]

    return LaunchDescription(declare_launch_arguments + [launch_robot] + nodes_to_start)