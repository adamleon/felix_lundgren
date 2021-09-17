#! /usr/bin/env python

from math import sin, cos, pi
import threading
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile
from geometry_msgs.msg import Quaternion
from sensor_msgs.msg import JointState
from tf2_ros import TransformBroadcaster, TransformStamped


class StatePublisher(Node):

    def __init__(self):
        rclpy.init()
        super().__init__('random_joint_state_publisher')

        qos_profile = QoSProfile(depth=10)
        self.joint_pub = self.create_publisher(JointState, 'joint_states', qos_profile)  # JointStates
        self.nodeName = self.get_name()
        self.get_logger().info("{0} started".format(self.nodeName))

        loop_rate = self.create_rate(30)

        # robot state
        angle = 0.
        angle_inc = 0.01

        joint_state = JointState()

        try:
            while rclpy.ok():
                rclpy.spin_once(self)

                # update joint_state
                now = self.get_clock().now()
                joint_state.header.stamp = now.to_msg()
                joint_state.name = ['base_link', 'link_1']
                joint_state.position = [angle, angle]

                # send the joint state and transform
                self.joint_pub.publish(joint_state)
                # self.get_logger().info("current angle: {0}".format(angle))

                # Create new robot state
                angle += angle_inc
                if angle < -0.5 or angle > 0.0:
                    angle_inc *= -1
                
                # This will adjust as needed per iteration
                loop_rate.sleep()

        except KeyboardInterrupt:
            pass


def main():
    node = StatePublisher()


if __name__ == '__main__':
    main()