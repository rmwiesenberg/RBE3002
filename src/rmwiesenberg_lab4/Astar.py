import rospy, tf, math,time
import Node
import numpy as np
from tf.transformations import euler_from_quaternion
from nav_msgs.msg import Odometry, Path, OccupancyGrid, MapMetaData, GridCells
from geometry_msgs.msg import Twist, PoseStamped

rad = .21

class Astar(object):
	def __init__(self):
		
		self.resolution = .21 	#yolo
		self.robot = .21 		#robot circumference should be a-star grd size

		self.map_sub = rospy.Subscriber("map", OccupancyGrid, self.mapstuff, queue_size=1)
		self.visit_pub = rospy.Publisher("move_base/local_costmap/visit", GridCells, queue_size=1)
		self.waypt_pub = rospy.Publisher("move_base/local_costmap/waypt", GridCells, queue_size=1)


	def run(self, start_posn, goal):
		self.start_posn = start_posn
		self.toVisit = [Node.Node(start_posn, None, 0)]
		self.visited = []
		self.goal = goal

		self.pub = GridCells()
		self.pub.header = self.map.header
		self.pub.cell_width = self.resolution
		self.pub.cell_height = self.resolution 

		self.way = GridCells()
		self.way.header = self.map.header
		self.way.cell_width = self.resolution
		self.way.cell_height = self.resolution

		while not self.toVisit[0].contains(self.goal, self.robot):
			going = self.toVisit.pop(0)
			if self.isVisit(going.posn):
				continue
			self.visited.append(going.posn)
			nodes = going.expand(self.resolution)
			for n in nodes:
				if self.isValid(n.posn):
					if not self.toVisit:
						self.toVisit.append(n)
					else:
						n_cost = n.dist + self.globaldist(n.posn, self.goal)					
						for idx, v in enumerate(self.toVisit):
							v_cost = v.dist + self.globaldist(v.posn, self.goal)
							if (n_cost < v_cost):
								self.toVisit.insert(idx, n)
								break
							elif (idx is len(self.toVisit)-1):
								self.toVisit.append(n)
								break

					self.pub.cells.append(n.posn)

			self.visit_pub.publish(self.pub)


		self.best = self.toVisit[0]
		print self.best.lineage()
		print "DID IT FAM"

	def globaldist(self, cur, goal):
		return math.sqrt((cur.x-goal.x)**2 + (cur.y-goal.y)**2 + (cur.z-goal.z)**2)

	def isValid(self, posn):
		space = self.robot+self.resolution/2
		for x in np.arange(posn.x - space, posn.x + space , self.map.info.resolution):
			for y in np.arange(posn.y - space, posn.y + space, self.map.info.resolution):
				if not ((x < self.map.info.origin.position.x) and (y < self.map.info.origin.position.y)):
					xloc = int((x-self.map.info.origin.position.x)/self.map.info.resolution)
					yloc = int((y-self.map.info.origin.position.y)/self.map.info.resolution)
			
					idx = xloc + self.map.info.width*yloc
					try:
						if self.map.data[idx] is 100:
							return False
					except:
						return False
		return True 

	def isVisit(self, posn):
		for p in self.visited:
			if posn.x >= p.x and posn.x < (p.x + self.resolution) and posn.y >= p.y and posn.y < (p.y + self.resolution):
				return True
		return False

	def getWaypoints(self):
		points = self.best.points()
		posn = points[0]
		curTheta = 100
		ways = [posn]
		for p in points:
			theta = math.atan2((p.y - posn.y),(p.x - posn.x)) 
			if abs(theta - curTheta) > .01:
				curTheta = theta
				ways.append(posn)
			posn = p
		self.way.cells = ways
		self.waypt_pub.publish(self.way)

		return ways

	def mapstuff(self, msg):
		self.map = msg
		print self.map.info

	def getCellPose(self, pose):
		posn = min(self.cel.cells, key=lambda i: 
			(abs(i.x-pose.position.x)+
			abs(i.y-pose.position.y)+
			abs(i.z-pose.position.z)))
		act_pose = Pose()
		act_pose.position = posn
		act_pose.orientation = pose.orientation
		return act_pose