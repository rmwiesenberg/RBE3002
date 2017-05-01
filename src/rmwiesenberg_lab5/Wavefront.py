import rospy, tf, math,time
import Node
import numpy as np
from tf.transformations import euler_from_quaternion
from nav_msgs.msg import Odometry, Path, OccupancyGrid, MapMetaData, GridCells
from geometry_msgs.msg import Twist, PoseStamped

# Wavefront looks for unknown areas and then returns that point to be used in A*
# A* expands its own obstacles and does not continue exploration there
# Resolution - define the desired wavefront resolution
# Robot - robot diameter for obstacle expansion

# map_sub - map subsciber
# *_pub - grid cell publishers for pretty GUI

class Wavefront(object):
	def __init__(self):
		
		self.resolution = .1
		self.robot = .22

		self.map_sub = rospy.Subscriber("map", OccupancyGrid, self.mapstuff, queue_size=1)
		self.wave_pub = rospy.Publisher("move_base/local_costmap/wave", GridCells, queue_size=1)

	# does wavefront
	def run(self, start_posn):
		self.start_posn = start_posn
		self.toVisit = [Node.Node(start_posn, None, 0)]
		self.visited = []

		# define grid cells to be published
		self.wave = GridCells()
		self.wave.header = self.map.header
		self.wave.cell_width = self.resolution
		self.wave.cell_height = self.resolution 

		# check if nodes are valid and have not been visited 
		# expands the remaining
		while 1:
			going = self.toVisit.pop(0)
			if self.isVisit(going.posn):
				continue
			if not self.isKno(going.posn):
				return going.posn
			self.visited.append(going.posn)
			nodes = going.expand(self.resolution)
			for n in nodes:
				n.lineage()
				if self.isValid(n.posn):
					if not self.toVisit:
						self.toVisit.append(n)
					else:
						n_cost = n.dist					
						for idx, v in enumerate(self.toVisit):
							v_cost = v.dist
							if (n_cost < v_cost):
								self.toVisit.insert(idx, n)
								break
							elif (idx is len(self.toVisit)-1):
								self.toVisit.append(n)
								break

					self.wave.cells.append(n.posn)
			self.wave_pub.publish(self.wave)

	# checks if position is an obstacle
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

	# checks if position is not an unknown
	def isKno(self, posn):
		space = self.robot+self.resolution/2
		for x in np.arange(posn.x - space, posn.x + space , self.map.info.resolution):
			for y in np.arange(posn.y - space, posn.y + space, self.map.info.resolution):
				if not ((x < self.map.info.origin.position.x) and (y < self.map.info.origin.position.y)):
					xloc = int((x-self.map.info.origin.position.x)/self.map.info.resolution)
					yloc = int((y-self.map.info.origin.position.y)/self.map.info.resolution)
			
					idx = xloc + self.map.info.width*yloc
					try:
						if self.map.data[idx] is -1:
							return False
					except:
						return False
		return True 

	# checks if the position has been visited by A*
	def isVisit(self, posn):
		for p in self.visited:
			if posn.x >= p.x and posn.x < (p.x + self.resolution) and posn.y >= p.y and posn.y < (p.y + self.resolution):
				return True
		return False

	# pass the map in to the object
	def mapstuff(self, msg):
		self.map = msg