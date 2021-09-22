import os
import yaml
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration, Command

def load_yaml(file_path):
    package_path = get_package_share_directory('felix_lundgren')
    absolute_file_path = os.path.join(package_path, file_path)

    try:
        with open(absolute_file_path, 'r') as file:
            return yaml.safe_load(file)
    except EnvironmentError:  # parent of IOError, OSError *and* WindowsError where available
        return None


def generate_launch_description():
    use_sim_time = LaunchConfiguration('use_sim_time', default=False)

    robot_description_config = os.path.join(
        get_package_share_directory('felix_lundgren'),
        'urdf/lundgren_cell.urdf.xacro')
    robot_description = {'robot_description':Command(['xacro',' ', robot_description_config])}

    robot_controllers = os.path.join(
        get_package_share_directory('felix_lundgren'),
        'config/ros2_control/controllers.yaml')


    return LaunchDescription([
        # start up the robot state publisher.
        # runs xacro on the urdf before loading the file.
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            output='screen',
            parameters=[
                robot_description
            ]
        ),

        Node(
            package="controller_manager",
            executable="ros2_control_node",
            parameters=[robot_description, robot_controllers],
            output={
                "stdout": "screen",
                "stderr": "screen",
            }
        ),

        # start up the robot state publisher.
        # runs xacro on the urdf before loading the file.
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            output='screen',
            parameters=[
                {'use_sim_time': use_sim_time}
            ]
        ),

        Node(
            package="controller_manager",
            executable="spawner.py",
            arguments=["lundgren_joint_state_broadcaster", "--controller-manager", "/controller_manager"],
        ),

        Node(
            package="controller_manager",
            executable="spawner.py",
            arguments=["lundgren_position_trajectory_controller", "-c", "/controller_manager"],
        )         
    ])