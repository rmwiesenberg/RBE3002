import rospy, tf, numpy, math
from std_msgs.msg import String
from geometry_msgs.msg import Twist, PoseStamped
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
    max_lin = .5
    marg_ang = .1
    marg_pos = .1

    """docstring for Robot."""
    def __init__(self):
        rospy.init_node('rwiesenberg_lab2')

        self.pub = rospy.Publisher('cmd_vel_mux/input/teleop', Twist, None, queue_size=10)
        self.bump_sub = rospy.Subscriber('mobile_base/events/bumper', BumperEvent, self.readBumper, queue_size=1)
        self.goal_sub = rospy.Subscriber('move_base_simple/goal', PoseStamped, self.navToPose, queue_size=1)
        self.odom_sub = rospy.Subscriber('/odom', Odometry, self.readOdom)
        self.odom_lis = tf.TransformListener()
        self.odom_bro = tf.TransformBroadcaster()
        self.odom_bro.sendTransform((0,0,0),(0,0,0,1),rospy.Time.now(),"base_footprint","odom")

        rospy.sleep

        self.trans = tf.TransformerROS()

        self.posn = [0,0,0]
        self.desired = [0,0,0]

        self.kp_lin = 1
        self.kp_ang = 1

        rospy.spin()

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
        self.desired[0] = goal.pose.position.x
        self.desired[1] = goal.pose.position.y
        quat = goal.pose.orientation
        quat = [quat.x, quat.y, quat.z, quat.w]
        roll, pitch, yaw = euler_from_quaternion(quat)
        if(self.desired[0] == self.posn[0]) and (self.desired[1] == self.posn[1]):
            self.desired[2] = yaw
        else:
            self.desired[2] = math.atan2((self.desired[1] - self.posn[1]),
                                (self.desired[0] - self.posn[0]))
        if self.desired[2] < 0:
            self.desired[2] = self.desired[2] + 2*math.pi

    #This function accepts two wheel velocities and a time interval.
    def spinWheels(self, u1, u2, time):
        #calculate linear and angular velocities
        v = (r/2)*(u1+u2)
        w = (r/Robot.wheel_base)*(u1-u2)

        start = rospy.Time().now().secs
        while (rospy.Time().now().secs - start) < time and not rospy.is_shutdown():
            self.moveRobot(v,w)
        return True

    def drive(self):
        diff_ang = self.desired[2] - self.posn[2]
        diff_pos = abs(self.desired[0] - self.posn[0]) + abs(self.desired[1] - self.posn[1])
        print [self.posn, self.desired, diff_ang, diff_pos]
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
        self.pub.publish(move_msg)
        return True

    def stopRobot(self):
        self.pub.publish(Robot.stop_msg)
        return True

    #get robot position
    #yay callbacks!
    def readOdom(self, msg):
        (trans,rot) = self.odom_lis.lookupTransform('odom','base_footprint', rospy.Time(0))
        roll, pitch, yaw = euler_from_quaternion(rot)
        self.posn[0] = trans[0]
        self.posn[1] = trans[1]
        if yaw < 0:
            self.posn[2] = yaw + 2*math.pi
        else:
            self.posn[2] = yaw
