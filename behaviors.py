#! /usr/bin/env python
"""<remap from="cmd_vel" to="cmd_vel_original"/>
_timeout_ = 180"""
from __future__ import annotations

import rospy
from geometry_msgs.msg import Twist


class Auto_docking_error:
    """decl vel_robot_perception cmd_vel_original Twist.linear.x
    after vel_robot_perception > 0, never turtlebot3.velocity =={0.005} 0"""

    def __init__(self):
        print("simulating auto_docking_error_behavior...")
        self.cmd_vel_pub = rospy.Publisher("cmd_vel", Twist, queue_size=1)
        self.twist = Twist()
        self.auto_docking_error()

    def get_vel(self):
        return rospy.wait_for_message("cmd_vel_original", Twist)

    def auto_docking_error(self):
        while not rospy.is_shutdown():
            vel = self.get_vel()
            self.twist = vel
            if abs(vel.linear.x) < 0.015:
                self.twist.linear.x = 0.0
            self.cmd_vel_pub.publish(self.twist)


class Direction_invert_error:
    """decl angular_vel_robot_perception cmd_vel_original Twist.angular.z
    after turtlebot3.velocity.angular.z < 0, never angular_vel_robot_perception > 0"""

    def __init__(self):
        print("simulating direction_invert_error_behavior...")
        self.cmd_vel_pub = rospy.Publisher("cmd_vel", Twist, queue_size=1)
        self.twist = Twist()
        self.direction_invert_error()

    def get_vel(self):
        return rospy.wait_for_message("cmd_vel_original", Twist)

    def direction_invert_error(self):
        while not rospy.is_shutdown():
            vel = self.get_vel()
            self.twist = vel
            if abs(vel.linear.x) < 0.012:
                self.twist.angular.z = -vel.angular.z
            self.cmd_vel_pub.publish(self.twist)


class Backwards_movement_error:
    """decl vel_robot_perception cmd_vel_original Twist.linear.x
    after turtlebot3.velocity.linear.x < 0, never vel_robot_perception > 0"""

    def __init__(self):
        print("simulating backwards_movement_error_behavior...")
        self.cmd_vel_pub = rospy.Publisher("cmd_vel", Twist, queue_size=1)
        self.twist = Twist()
        self.backwards_movement_error()

    def get_vel(self):
        return rospy.wait_for_message("cmd_vel_original", Twist)

    def backwards_movement_error(self):
        while not rospy.is_shutdown():
            vel = self.get_vel()
            self.twist = vel
            if abs(vel.linear.x) > 0.03:
                self.twist.linear.x = -vel.linear.x
            self.cmd_vel_pub.publish(self.twist)


def main():
    rospy.init_node("error_behaviors_node")
    try:
        # auto_docking_error = Auto_docking_error()
        # direction_invert_error = Direction_invert_error()
        backwards_movement_error = Backwards_movement_error()
    except rospy.ROSInterruptException:
        pass


if __name__ == "__main__":
    main()
