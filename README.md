# felix_lundgren
This is a configuration package for the Lundgren robot cell at MANULAB, MTP, NTNU.

It features an URDF of the robot.

## Future plans
- Full Unity integration for simulation
- MoveIt support
- Contstraint based planning.


## Installation
Make a ROS2 workspace and navigate to the `src` folder and run:
```
git clone git@github.com:adamleon/felix_lundgren.git
cd ..
colcon build --symlink-install
```
Now source your workspace if you haven't done it already

## Demos
A list of demos you can run.

### Robot Joint State Publisher
Run
```
ros2 launch felix_lundgren robot_joint_publisher.launch.py
```
You can start up Rviz2 or Unity and look at the robot move. There is an Rviz configuration in the launch folder that sets the plugins properly.

To run the Unity Endpoint, run:
```
ros2 run ros_tcp_endpoint default_server_endpoint --ros-args -p ROS_IP:=<Your IP>
```