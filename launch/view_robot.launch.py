import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, Command
from launch_ros.actions import Node


def generate_launch_description():
    use_sim_time = LaunchConfiguration('use_sim_time', default=False)

    robot_description_config = os.path.join(
        get_package_share_directory('felix_lundgren'),
        'urdf/lundgren_cell.urdf.xacro')
    robot_description = {'robot_description':Command(['xacro',' ', robot_description_config])}

    return LaunchDescription([
        # start up the robot state publisher.
        # runs xacro on the urdf before loading the file.
        Node(
            package='robot_state_publisher',
            namespace='lundgren',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            output='screen',
            parameters=[
                robot_description
            ]
        ),

        # starts the joint state publisher gui in case you want
        # control the joint states.
        Node(
            package='joint_state_publisher_gui',
            namespace='lundgren',
            executable='joint_state_publisher_gui',
            name='joint_state_publisher_gui'
        ),

        # start up the robot state publisher.
        # runs xacro on the urdf before loading the file.
        Node(
            package='rviz2',
            namespace='lundgren',
            executable='rviz2',
            name='rviz2',
            output='screen',
            parameters=[
                {'use_sim_time': use_sim_time}
            ]
        )
    ])