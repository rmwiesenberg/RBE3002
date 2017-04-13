import math, geometry_msgs, time

from geometry_msgs.msg import Point

class Node(object):
	def __init__(self, posn, parent, dist):
		self.posn = posn
		self.parent = parent
		self.dist = dist
		
	def expand(self, resolution):
		toReturn = []

		for x in [-resolution,0,resolution]:
			for y in [-resolution,0,resolution]:
				posn = Point()
				posn.x = self.posn.x + x
				posn.y = self.posn.y + y
				posn.z = self.posn.z
				node = Node(posn, self, self.dist + math.sqrt(x**2 + y**2))

				toReturn.append(node)

		return toReturn

	def contains(self, posn, resolution):
		if (self.posn.x + resolution/2 > posn.x) and (self.posn.x - resolution/2 < posn.x) and (self.posn.y + resolution/2 > posn.y) and (self.posn.y - resolution/2 < posn.y):
			return True
		elif not self.parent:
			return False
		else:
			return self.parent.contains(posn, resolution)

	def lineage(self):
		if not self.parent:
			return [self.posn.x, self.posn.y, self.posn.z]
		else:
			return [self.posn.x, self.posn.y, self.posn.z, self.parent.lineage()]

	def points(self):
		points = [self.posn]
		cur = self.parent
		while cur:
			points.append(cur.posn)
			cur = cur.parent
		return points