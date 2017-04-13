import rospy, tf, numpy, math
import nav_msgs, geometry_msgs
from nav_msgs.msg import GridCells, OccupancyGrid
from geometry_msgs.msg import Point, Pose

from tf.transformations import euler_from_quaternion

class Map(object):
	def __init__(self):
		self.map_sub = rospy.Subscriber("map", OccupancyGrid, self.mapstuff, queue_size=1)
		self.obs_pub = rospy.Publisher("move_base/local_costmap/obstacles", GridCells, queue_size=1)
		self.emt_pub = rospy.Publisher("move_base/local_costmap/empty", GridCells, queue_size=1)
		self.unk_pub = rospy.Publisher("move_base/local_costmap/unknown", GridCells, queue_size=1)
		#self.mapmeta_sub = rospy.Subscriber("map_meta", MapMetaData, self.mapmetastuff)

		self.cel = GridCells()
		self.obs = GridCells()
		self.emt = GridCells()
		self.unk = GridCells()	

		self.newdata = False

	def mapstuff(self, msg):
		self.meta = msg.info
		self.head = msg.header
		posn = [self.meta.origin.position.x, 
				self.meta.origin.position.y, 
				self.meta.origin.position.z]
		quat = self.meta.origin.orientation
		quat = [quat.x, quat.y, quat.z, quat.w]
		roll, pitch, yaw = euler_from_quaternion(quat)

		self.cel.header = self.head
		self.cel.cell_width = self.meta.resolution 
		self.cel.cell_height = self.meta.resolution 

		self.obs.header = self.head
		self.obs.cell_width = self.meta.resolution 
		self.obs.cell_height = self.meta.resolution 

		self.emt.header = self.head
		self.emt.cell_width = self.meta.resolution 
		self.emt.cell_height = self.meta.resolution

		self.unk.header = self.head
		self.unk.cell_width = self.meta.resolution
		self.unk.cell_height = self.meta.resolution

		for idx, cell in enumerate(msg.data):
			cell_posn = Point()
			cell_posn.x = int(idx%self.meta.width)*self.meta.resolution + posn[0]*math.cos(yaw) + self.meta.resolution/2
			cell_posn.y = idx/self.meta.height*self.meta.resolution + posn[1]*math.sin(yaw) + self.meta.resolution/2
			cell_posn.z = posn[2]


			if(cell == -1):
				self.unk.cells.append(cell_posn)

				try:
					self.emt.cells.remove(cell_posn)
				except:
					pass

				try:
					self.obs.cells.remove(cell_posn)
				except:
					pass
					
			elif(cell == 100):
				self.obs.cells.append(cell_posn)
				
				try:
					self.emt.cells.remove(cell_posn)
				except:
					pass

				try:
					self.unk.cells.remove(cell_posn)
				except:
					pass
			else:
				self.emt.cells.append(cell_posn)
				
				try:
					self.unk.cells.remove(cell_posn)
				except:
					pass

				try:
					self.obs.cells.remove(cell_posn)
				except:
					pass

			try:
				self.cel.cells.remove(cell_posn)
			except:
				pass
			self.cel.cells.append(cell_posn)


		print(self.obs.cells)


		self.obs_pub.publish(self.obs)
		self.emt_pub.publish(self.emt)
		self.unk_pub.publish(self.unk)

		self.newdata = True

