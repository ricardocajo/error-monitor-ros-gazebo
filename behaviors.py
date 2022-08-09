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


def main():
    rospy.init_node("error_behaviors_node")
    try:
        auto_docking_error = Auto_docking_error()
    except rospy.ROSInterruptException:
        pass


if __name__ == "__main__":
    main()
