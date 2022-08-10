#! /usr/bin/env python
from __future__ import annotations

import rospy
from geometry_msgs.msg import Twist

LINEAR_VEL = 0.02


class Auto_docking_error:
    def __init__(self):
        print("simulating auto_docking_error_behavior...")
        self.cmd_vel_pub = rospy.Publisher("cmd_vel", Twist, queue_size=1)
        self.auto_docking_error()

    def auto_docking_error(self):
        twist_msg = Twist()
        while not rospy.is_shutdown():
            twist_msg.linear.x = LINEAR_VEL
            twist_msg.angular.z = 0.0
            self.cmd_vel_pub.publish(twist_msg)


class Direction_invert_error:
    def __init__(self):
        print("simulating direction_invert_error_behavior...")
        self.cmd_vel_pub = rospy.Publisher("cmd_vel", Twist, queue_size=1)
        self.direction_invert_error()

    def get_vel(self):
        return rospy.wait_for_message("/cmd_vel", Twist)

    def direction_invert_error(self):
        while not rospy.is_shutdown():
            vel = self.get_vel()
            if abs(vel.linear.x) > 0.012:
                twist_msg = Twist()
                twist_msg = vel
                twist_msg.angular.z = -vel.angular.z
                self.cmd_vel_pub.publish(twist_msg)


def main():
    rospy.init_node("error_behaviors_node")
    try:
        # auto_docking_error = Auto_docking_error()
        direction_invert_error = Direction_invert_error()
    except rospy.ROSInterruptException:
        pass


if __name__ == "__main__":
    main()
