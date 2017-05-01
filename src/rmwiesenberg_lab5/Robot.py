import rospy, tf, numpy, math, time
import Astar, Map, Wavefront
import geometry_msgs
from std_msgs.msg import String
from geometry_msgs.msg import Twist, PoseStamped, Point, Quaternion, Pose, PointStamped
from nav_msgs.msg import Odometry
from kobuki_msgs.msg import BumperEvent
from tf.transformations import euler_from_quaternion, quaternion_from_euler

class Robot(object):
	wheel_rad = 3.5/100
	wheel_base = 23/100

	stop_msg = Twist()
	stop_msg.linear.x = 0
	stop_msg.angular.z = 0

	max_ang = .5
	max_lin = .2
	min_ang = .05
	min_lin = .05
	marg_ang = .05
	marg_pos = .05

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

		self.initpos = [0,0,0]
		self.initort = [0,0,0,0]

		self.desired = [0,0,0]

		self.kp_lin = .25
		self.kp_ang = .25

		self.gotInit = False

		self.pub = rospy.Publisher('cmd_vel_mux/input/teleop', Twist, None, queue_size=10)
		#self.bump_sub = rospy.Subscriber('mobile_base/events/bumper', BumperEvent, self.readBumper, queue_size=1)
		self.odom_sub = rospy.Subscriber('/odom', Odometry, self.readOdom)
		self.odom_lis = tf.TransformListener()
		self.odom_bro = tf.TransformBroadcaster()
		print "made robot"

	

	#drive to a goal given by A*
	def navToPose(self, goal):
		keepRunning = True
		while keepRunning:
			self.calcDesired(goal)
			if(self.drive()):
				keepRunning = False
		print("Made it!")
		return True

	# calculate the desired theta to get to the goal
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
								(self.desired[0] - self.pose.position.x))
		if self.desired[2] < 0:
			self.desired[2] = self.desired[2] + 2*math.pi

	# fancy driving method that follows the following order:
	# while robot not pose:
	# 	Turn to face the goal
	# 	if there:
	# 		Drive Forward
	#		if there:
	#			Change Orientation
	def drive(self):
		diff_ang = math.tan((self.desired[2] - self.theta)/2)
		diff_pos = abs(self.desired[0] - self.pose.position.x) +  abs(self.desired[1] - self.pose.position.y)
		if(abs(diff_ang) > Robot.marg_ang):
			if diff_ang < 0:
				mod = -1
			else:
				mod = 1
			amt = mod*self.kp_ang*(diff_ang+2*math.pi)
			if abs(amt) < Robot.min_ang:
				self.moveRobot(0, mod*Robot.min_ang)
			elif abs(amt) > Robot.max_ang:
				self.moveRobot(0, mod*Robot.max_ang)
			else:
				self.moveRobot(0, amt)
			return False
		elif(diff_pos > Robot.marg_pos):
			amt = self.kp_lin*diff_pos
			if abs(amt) < Robot.min_ang:
				self.driveStraight(Robot.min_lin)
			elif abs(amt) > Robot.max_ang:
				self.driveStraight(Robot.max_lin)
			else:
				self.driveStraight(self.kp_lin*diff_pos)
			return False
		else:
			self.stopRobot()
			return True



	# Simple move and stop commands
	def driveStraight(self, speed):
		self.moveRobot(speed, 0)

	def moveRobot(self, linVel, angVel):
		move_msg = Twist()
		move_msg.linear.x = linVel
		move_msg.angular.z = angVel

		self.pub.publish(move_msg)
		return True

	def stopRobot(self):
		self.pub.publish(Robot.stop_msg)
		return True

	#get robot position
	def readOdom(self, msg):
		if not self.gotInit:
			try:
				(trans, rot) = self.odom_lis.lookupTransform('map','base_footprint', rospy.Time(0))
				self.initort = rot
				self.initpos = trans

				roll, pitch, yaw = euler_from_quaternion(rot)
				self.inittheta = yaw

				print "map ready"
				self.gotInit = True
			except:
				print "map not ready"
		else:
			(trans, rot) = self.odom_lis.lookupTransform('map','base_footprint', rospy.Time(0))
			self.pose.orientation.x = rot[0]
			self.pose.orientation.y = rot[1]
			self.pose.orientation.z = rot[2]
			self.pose.orientation.w = rot[3]
			self.pose.position.x = trans[0]
			self.pose.position.y = trans[1]
			self.pose.position.z = trans[2]

			roll, pitch, yaw = euler_from_quaternion(rot)
			self.theta = yaw 

			if(self.theta < 0):
				self.theta = self.theta+2*math.pi

	# start the wavefront chain
	# do wavefront until all unknowns are found
	# at every waypoint re-do the wavefront
	def doWavefront(self, msg):
		print("Will do!")
		wave = Wavefront.Wavefront()
		self.stopRobot()
		unk = False

		# search for unknowns
		try:
			unk = wave.run(self.pose.position)
		except:
			self.stopRobot()
			print("Explored!")
			self.goal_sub = rospy.Subscriber('move_base_simple/goal', PoseStamped, self.doAstar, queue_size=1)
			return

		#A* to the point
		astar = Astar.Astar()
		time.sleep(.2)
		astar.run(self.pose.position, unk)
		ways = astar.getWaypoints()

		# move through waypoints
		for w in reversed(ways):
			dPose = Pose()
			dPose.position = w
			if not len(ways) > 2:
				rot = [0,0,0,0]
				rot[0] = self.pose.orientation.x
				rot[1] = self.pose.orientation.y
				rot[2] = self.pose.orientation.z
				rot[3] = self.pose.orientation.w
				roll, pitch, yaw = euler_from_quaternion(rot)
				yaw = yaw + (math.pi/4)
				rot = quaternion_from_euler(roll, pitch, yaw)
				dPose.orientation.x = rot[0]
				dPose.orientation.y = rot[1]
				dPose.orientation.z = rot[2]
				dPose.orientation.w = rot[3]
			else:
				dPose.orientation = self.pose.orientation
			while not self.navToPose(dPose):
				pass
			self.stopRobot
			self.doWavefront(msg)
			break

	# start the A* chain
	# at every waypoint, redo A*
	def doAstar(self, msg):
		#A* to the point
		astar = Astar.Astar()
		time.sleep(.2)
		astar.run(self.pose.position, msg.pose.position)
		ways = astar.getWaypoints()

		# move through waypoints
		for w in reversed(ways):
			dPose = Pose()
			dPose.position = w
			rot = [0,0,0,0]
			rot[0] = self.pose.orientation.x
			rot[1] = self.pose.orientation.y
			rot[2] = self.pose.orientation.z
			rot[3] = self.pose.orientation.w
			roll, pitch, yaw = euler_from_quaternion(rot)
			yaw = yaw + math.pi
			rot = quaternion_from_euler(roll, pitch, yaw)
			dPose.orientation.x = rot[0]
			dPose.orientation.y = rot[1]
			dPose.orientation.z = rot[2]
			dPose.orientation.w = rot[3]
			while not self.navToPose(dPose):
				pass
			self.stopRobot
			if len(ways) > 2:
				self.doAstar(msg)
			break