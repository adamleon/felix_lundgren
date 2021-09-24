from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, Command, FindExecutable, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    # Launch Arguments
    declare_launch_arguments = [
        DeclareLaunchArgument(
            'robot_description_file',
            default_value="lundgren_cell.urdf.xacro",
            description="The file being loaded. Relative to <pachage_name>/urdf."
        ),

        DeclareLaunchArgument(
            "gui",
            default_value="true",
            description="Launches the GUI"
        ),

        DeclareLaunchArgument(
            "package_name",
            default_value="felix_lundgren",
            description="The package which contains all the information to launch the robot"
        )
    ]

    # Get the package name
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
    
    # Check if the GUI should be launched
    # Typically if only this launch file is run
    launch_gui = IfCondition(LaunchConfiguration('gui'))

    # Nodes
    robot_state_publisher = Node(
            package="robot_state_publisher",
            executable="robot_state_publisher",
            name="robot_state_publisher",
            output="screen",
            parameters=[
                robot_description
            ]
        )
    
    # GUI that controls the joint states
    joint_state_gui = Node(
            package="joint_state_publisher_gui",
            executable="joint_state_publisher_gui",
            name="joint_state_publisher_gui",
            condition=launch_gui
        )

    rviz = Node(
            package="rviz2",
            executable="rviz2",
            name="rviz2",
            output="screen",
            condition=launch_gui
        )

    nodes_to_start = [robot_state_publisher, joint_state_gui, rviz]

    return LaunchDescription( declare_launch_arguments + nodes_to_start )