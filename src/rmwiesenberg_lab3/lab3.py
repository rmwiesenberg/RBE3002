#!/usr/bin/env python
import rospy, Map, Astar, Robot

from geometry_msgs.msg import Twist, PoseStamped
rospy.init_node('rwiesenberg_lab3')

robot = Robot.Robot()

rospy.spin()