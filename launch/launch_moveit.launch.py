import os
import yaml
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, LogInfo
from launch.launch_context import LaunchContext
from launch_ros.actions import Node, node
from launch.actions.include_launch_description import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, Command, FindExecutable, PathJoinSubstitution, ThisLaunchFileDir, PythonExpression
from launch_ros.substitutions import FindPackageShare

def load_yaml(package_name, file_path):
    package_path = get_package_share_directory(package_name)
    absolute_file_path = os.path.join(package_path, file_path)

    try:
        with open(absolute_file_path) as file:
            return yaml.safe_load(file)
    except OSError:  # parent of IOError, OSError *and* WindowsError where available
        return None


def generate_launch_description():
    declare_launch_arguments = [
        DeclareLaunchArgument(
            'robot_description_file',
            default_value="lundgren_cell.urdf.xacro",
            description="The file being loaded. Relative to <pachage_name>/urdf."
        ),

        DeclareLaunchArgument(
            'robot_description_semantic_file',
            default_value="lundgren_cell.moveit2.srdf",
            description="The file being loaded. Relative to <pachage_name>/moveit2."
        ),

        DeclareLaunchArgument(
            "package_name",
            default_value="felix_lundgren",
            description="The package which contains all the information to launch the robot"
        )
    ]
    print("Hello World 1")
    package_name = LaunchConfiguration("package_name")
    print("Hello World 2")
    # Load the robot description file
    robot_description_file = LaunchConfiguration("robot_description_file")
    robot_description = {"robot_description":Command(
        [
            PathJoinSubstitution([FindExecutable(name="xacro")]),
            " ",
            PathJoinSubstitution([FindPackageShare(package_name),
            "urdf", robot_description_file]),
        ])}
    
    print("Hello World 3")
    launch_ros_control = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([ThisLaunchFileDir(), "/launch_ros_control.launch.py"]),
        launch_arguments={
            "package_name": package_name,
            "robot_description_file": robot_description_file,
        }.items(),
    )
    print("Hello World 4")
    # MoveIt Configuration
    robot_description_semantic_file = LaunchConfiguration("robot_description_semantic_file")
    robot_description_semantic = {"robot_description_semantic":Command(
        [
            PathJoinSubstitution([FindExecutable(name="xacro")]),
            " ",
            PathJoinSubstitution([FindPackageShare(package_name),
            "moveit2", robot_description_semantic_file]),
        ])}
    print("Hello World 5")
    # Kinematics
    kinematics_yaml = load_yaml("felix_lundgren", "moveit2/kinematics.yaml")
    robot_description_kinematics = {"robot_description_kinematics": kinematics_yaml}

    print("Hello World 6")
    # Planning Configuration
    ompl_planning_pipeline_config = {
        "move_group": {
            "planning_plugin": "ompl_interface/OMPLPlanner",
            "request_adapters": """default_planner_request_adapters/AddTimeOptimalParameterization default_planner_request_adapters/FixWorkspaceBounds default_planner_request_adapters/FixStartStateBounds default_planner_request_adapters/FixStartStateCollision default_planner_request_adapters/FixStartStatePathConstraints""",
            "start_state_max_bounds_error": 0.1,
        }
    }
    ompl_planning_yaml = load_yaml("felix_lundgren", "moveit2/ompl_planning.yaml")
    ompl_planning_pipeline_config["move_group"].update(ompl_planning_yaml)

    print("Hello World 7")
    # Trajectory Execution Configuration
    moveit_controllers_yaml = load_yaml("felix_lundgren", "moveit2/moveit2_controllers.yaml")
    moveit_controllers = {
        "moveit_simple_controller_manager": moveit_controllers_yaml,
        "moveit_controller_manager": "moveit_simple_controller_manager/MoveItSimpleControllerManager",
    }

    print("Hello World 8")
    trajectory_execution = {
        "moveit_manage_controllers": True,
        "trajectory_execution.allowed_execution_duration_scaling": 1.2,
        "trajectory_execution.allowed_goal_duration_margin": 0.5,
        "trajectory_execution.allowed_start_tolerance": 0.01,
    }

    # Planning scene parameters
    planning_scene_monitor_parameters = {
        "publish_planning_scene": True,
        "publish_geometry_updates": True,
        "publish_state_updates": True,
        "publish_transforms_updates": True,
        "planning_scene_monitor_options": {
            "name": "planning_scene_monitor",
            "robot_description": "robot_description",
            "joint_state_topic": "/joint_states",
            "attached_collision_object_topic": "/move_group/planning_scene_monitor",
            "publish_planning_scene_topic": "/move_group/publish_planning_scene",
            "monitored_planning_scene_topic": "/move_group/monitored_planning_scene",
            "wait_for_initial_state_timeout": 10.0,
        },
    }

    print("Hello World 9")
    # Nodes
    # Start the actual move_group node/action server
    move_group = Node(
            package="moveit_ros_move_group",
            executable="move_group",
            output="screen",
            parameters=[
                robot_description,
                robot_description_semantic,
                robot_description_kinematics,
                ompl_planning_pipeline_config,
                trajectory_execution,
                moveit_controllers,
                planning_scene_monitor_parameters,
            ],
        )

    rviz = Node(
            package="rviz2",
            executable="rviz2",
            name="rviz2_moveit",
            output="log",
            parameters=[
                robot_description,
                robot_description_semantic,
                ompl_planning_pipeline_config,
                robot_description_kinematics,
            ],
        )
    
    # Static TF
    static_tf = Node(
            package="tf2_ros",
            executable="static_transform_publisher",
            name="static_transform_publisher",
            output="log",
            arguments=[
                "0.0", "0.0", "0.0", 
                "0.0", "0.0", "0.0", 
                "world", "lundgren_base_link"],
        )

    print("Hello World 10")
    nodes_to_start = [
        move_group,
        rviz,
        static_tf
    ]

    print("Hello World 11")
    return LaunchDescription(
        declare_launch_arguments + 
        [launch_ros_control] + 
        nodes_to_start)