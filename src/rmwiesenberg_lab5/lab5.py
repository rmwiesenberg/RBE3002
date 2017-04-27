#!/usr/bin/env python
import rospy, Map, Astar, Robot, time

from geometry_msgs.msg import Twist, PoseStamped, PointStamped
rospy.init_node('rwiesenberg_lab3')

robot = Robot.Robot()
goal_sub = rospy.Subscriber('/clicked_point', PointStamped, robot.doWavefront, queue_size=1)

time.sleep(2)

rospy.spin()