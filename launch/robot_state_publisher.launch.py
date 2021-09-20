import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, Command
from launch_ros.actions import Node


def generate_launch_description():
    urdf_file_name = 'urdf/kuka_kr16.urdf'

    print("urdf_file_name : {}".format(urdf_file_name))

    urdf = os.path.join(
        get_package_share_directory('felix_lundgren'),
        urdf_file_name)

    return LaunchDescription([
        # start up the robot state publisher.
        # runs xacro on the urdf before loading the file.
        Node(
            package='robot_state_publisher',
            namespace='lundgren',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            output='screen',
            parameters=[{
                'robot_description':Command(['xacro',' ', urdf])
                }]),

        # starts the joint state publisher gui in case you want
        # control the joint states.
        Node(
            package='joint_state_publisher_gui',
            namespace='lundgren',
            executable='joint_state_publisher_gui',
            name='joint_state_publisher_gui'
        )])