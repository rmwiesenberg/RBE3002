import rospy, tf, numpy, math,time
import Node
from tf.transformations import euler_from_quaternion
from nav_msgs.msg import Odometry, Path, OccupancyGrid, MapMetaData, GridCells
from geometry_msgs.msg import Twist, PoseStamped

rad = .21

class Astar(object):
	def __init__(self, start_posn, map, goal):
		self.start_posn = start_posn
		self.toVisit = [Node.Node(start_posn, None, 0)]
		self.visited = [start_posn]
		self.goal = goal

		self.map = map

		self.pub = GridCells()
		self.pub.header = self.map.head
		self.pub.cell_width = self.map.meta.resolution 
		self.pub.cell_height = self.map.meta.resolution 

		self.way = GridCells()
		self.way.header = self.map.head
		self.way.cell_width = self.map.meta.resolution 
		self.way.cell_height = self.map.meta.resolution 

		self.visit_pub = rospy.Publisher("move_base/local_costmap/visit", GridCells, queue_size=1)
		self.waypt_pub = rospy.Publisher("move_base/local_costmap/waypt", GridCells, queue_size=1)


	def run(self):
		while not self.toVisit[0].contains(self.goal):
			nodes = self.toVisit.pop(0).expand(self.map.meta.resolution)

			for n in nodes:
				if self.isValid(n.posn) and not self.isVisit(n.posn):
					self.visited.append(n.posn)
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

					try:
						self.pub.cells.remove(cell_posn)
					except:
						pass
					self.pub.cells.append(n.posn)

			self.visit_pub.publish(self.pub)


		self.best = self.toVisit[0]
		print self.best.lineage()
		print "DID IT FAM"

	def globaldist(self, cur, goal):
		return math.sqrt((cur.x-goal.x)**2 + (cur.y-goal.y)**2 + (cur.z-goal.z)**2)

	def isValid(self, posn):
		for p in self.map.obs.cells:
			if math.sqrt((p.x - posn.x)**2 + (p.y - posn.y)**2 + (p.z - posn.z)**2) < (rad+self.map.meta.resolution/2):
				return False
		return True 

	def isVisit(self, posn):
		for p in self.visited:
			if abs(p.x - posn.x) + abs(p.y - posn.y) + abs(p.z - posn.z) < .01:
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
