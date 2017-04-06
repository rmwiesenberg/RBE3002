import rospy, tf, numpy, math
import Astar, Map
import geometry_msgs
from std_msgs.msg import String
from geometry_msgs.msg import Twist, PoseStamped, Point, Quaternion, Pose
from nav_msgs.msg import Odometry
from kobuki_msgs.msg import BumperEvent
from tf.transformations import euler_from_quaternion

class Robot(object):
	wheel_rad = 3.5/100
	wheel_base = 23/100

	stop_msg = Twist()
	stop_msg.linear.x = 0
	stop_msg.angular.z = 0

	max_ang = .2
	max_lin = .2
	marg_ang = .1
	marg_pos = .1

	def __init__(self):
		print "making robot"
		self.trans = tf.TransformerROS()

		self.pose = Pose()
		posn = Point()
		quat = Quaternion()
		self.pose.position = posn
		self.pose.orientation = quat
		self.pose.position.x = 0
		self.pose.position.y = 0
		self.pose.position.z = 0
		self.desired = [0,0,0]

		self.kp_lin = .5
		self.kp_ang = .5

		self.map = Map.Map()

		self.pub = rospy.Publisher('cmd_vel_mux/input/teleop', Twist, None, queue_size=10)
		self.goal_sub = rospy.Subscriber('move_base_simple/goal', PoseStamped, self.doAstar, queue_size=1)
		#self.bump_sub = rospy.Subscriber('mobile_base/events/bumper', BumperEvent, self.readBumper, queue_size=1)
		self.odom_sub = rospy.Subscriber('/odom', Odometry, self.readOdom)
		self.odom_lis = tf.TransformListener()
		self.odom_bro = tf.TransformBroadcaster()

	#drive to a goal subscribed as /move_base_simple/goal
	def navToPose(self, goal):
		keepRunning = True
		while keepRunning:
			self.calcDesired(goal)
			if(self.drive()):
				keepRunning = False
		print("Made it!")
		return True

	def calcDesired(self, goal):
		self.desired[0] = goal.position.x
		self.desired[1] = goal.position.y
		quat = goal.orientation
		quat = [quat.x, quat.y, quat.z, quat.w]
		roll, pitch, yaw = euler_from_quaternion(quat)
		if(self.desired[0] == self.pose.position.x) and (self.desired[1] == self.pose.position.y):
			self.desired[2] = yaw
		else:
			self.desired[2] = math.atan2((self.desired[1] - self.pose.position.y),
								(self.desired[0] - self.pose.position.y))
		if self.desired[2] < 0:
			self.desired[2] = self.desired[2] + 2*math.pi

	def drive(self):
		diff_ang = self.desired[2] - self.theta
		diff_pos = abs(self.desired[0] - self.pose.position.x) +  abs(self.desired[1] - self.pose.position.y)
		if(abs(diff_ang) > Robot.marg_ang):
			self.moveRobot(0, self.kp_ang*diff_ang)
			return False
		elif(diff_pos > Robot.marg_pos):
			self.driveStraight(self.kp_lin*diff_pos)
			return False
		else:
			self.stopRobot()
			return True

	def driveStraight(self, speed):
		self.moveRobot(speed, 0)

	#This function works the same as rotate how ever it does not publish linear velocities.
	def driveArc(self, radius, speed, angle):
		w = speed/radius
		v = speed
		initAng = self.posn[2]
		while(self.posn[2] < (initAng+angle)):
			self.moveRobot(v,w)
		self.stopRobot()
		pass  # Delete this 'pass' once implemented

	#Bumper Event Callback function
	def readBumper(self, msg):
		if (msg.state == 1):
			self.driveArc(.1, .2, 1)

	# Simple move and stop commands
	def moveRobot(self, linVel, angVel):
		move_msg = Twist()
		move_msg.linear.x = linVel
		move_msg.angular.z = angVel

		if(linVel < .1 and linVel != 0):
			move_msg.linear.x = .1
		if(angVel < .1 and angVel != 0):
			move_msg.angular.z = .1

		self.pub.publish(move_msg)
		return True

	def stopRobot(self):
		self.pub.publish(Robot.stop_msg)
		return True

	#get robot position
	#yay callbacks!
	def readOdom(self, msg):
		try:
			(trans, rot) = self.odom_lis.lookupTransform('map','base_footprint', rospy.Time(0))
			self.pose.position.x = trans[0]
			self.pose.position.y = trans[1]
			self.pose.position.z = trans[2]
			self.pose.orientation.x = rot[0]
			self.pose.orientation.y = rot[1]
			self.pose.orientation.z = rot[2]
			self.pose.orientation.w = rot[3]

			roll, pitch, yaw = euler_from_quaternion(rot)
			self.theta = yaw
		except:
			print "odom not ready"


	def doAstar(self, msg):
		start_pose = self.map.getCellPose(self.pose)
		goal_pose = self.map.getCellPose(msg.pose)
		astar = Astar.Astar(start_pose.position,self.map, goal_pose.position)
		astar.run()
		ways = astar.getWaypoints()
		print ways

		# for w in reversed(ways):
		# 	dPose = Pose()
		# 	dPose.position = w
		# 	dPose.orientation = self.pose.orientation
		# 	while self.navToPose(dPose):
		# 		pass

