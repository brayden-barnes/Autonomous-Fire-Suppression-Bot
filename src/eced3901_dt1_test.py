import math
import rospy as ros
import sys
import time

from geometry_msgs.msg import Twist, Pose
from nav_msgs.msg import Odometry
from tf.transformations import euler_from_quaternion, quaternion_from_euler


__author__ = "Gabriel Urbain" 
__copyright__ = "Copyright 2018, IDLab, UGent"

__license__ = "MIT" 
__version__ = "1.0" 
__maintainer__ = "Gabriel Urbain"
__email__ = "gabriel.urbain@ugent.be" 
__status__ = "Education" 
__date__ = "October 15th, 2018"


class TriangleMove(object):
    """
    This class is an abstract class to control a square trajectory on the turtleBot.
    It mainly declare and subscribe to ROS topics in an elegant way.
    """

    def __init__(self):

        # Declare ROS subscribers and publishers
        self.node_name = "triangle_move"
        self.odom_sub_name = "/odom"
        self.vel_pub_name = "/cmd_vel"
        self.vel_pub = None
        self.odometry_sub = None

        # ROS params
        self.pub_rate = 0.1
        self.queue_size = 2

        # Variables containing the sensor information that can be used in the main program
        self.odom_pose = None

    def start_ros(self):

        # Create a ROS node with a name for our program
        ros.init_node(self.node_name, log_level=ros.INFO)

        # Define a callback to stop the robot when we interrupt the program (CTRL-C)
        ros.on_shutdown(self.stop_robot)

        # Create the Subscribers and Publishers
        self.odometry_sub = ros.Subscriber(self.odom_sub_name, Odometry, callback=self.__odom_ros_sub, queue_size=self.queue_size)
        self.vel_pub = ros.Publisher(self.vel_pub_name, Twist, queue_size=self.queue_size)

    def stop_robot(self):

        # Get the initial time
        self.t_init = time.time()

        # We publish for a second to be sure the robot receive the message
        while time.time() - self.t_init < 1 and not ros.is_shutdown():
            
            self.vel_ros_pub(Twist())
            time.sleep(self.pub_rate)

        sys.exit("The process has been interrupted by the user!")

    def move(self):
        """ To be surcharged in the inheriting class"""

        while not ros.is_shutdown():
            time.sleep(1)

    def __odom_ros_sub(self, msg):

        self.odom_pose = msg.pose.pose

    def vel_ros_pub(self, msg):

        self.vel_pub.publish(msg)



class TriangleMoveOdom(TriangleMove):
    """
    This class implements a semi closed-loop triangle trajectory based on relative position control,
    where only odometry is used. HOWTO:
     - Start the sensors on the turtlebot:
            $ roslaunch turtlebot3_gazebo turtlebot3_empty_world.launch 
     - Start this node on your computer:
            $ python eced3901_dt1.py odom
    """

    def __init__(self):

        super(TriangleMoveOdom, self).__init__()

        self.pub_rate = 0.1
    
    '''This function returns the current yaw of the robot'''
    def get_z_rotation(self, orientation):

        (roll, pitch, yaw) = euler_from_quaternion([orientation.x, orientation.y, orientation.z, orientation.w])
	
	#convert yaw to 0-2pi scale
	#if our yaw is less than zero, add 2pi to ensure our yaw remains within 0 to 2pi rather than -pi to pi
	if(yaw<0):
		while(yaw < 0):
			yaw = yaw + 2*math.pi
	#if our yaw is greater than 2pi, subtract 2pi to ensure our yaw remains within 0 to 2pi
	if(yaw>2*math.pi):
		while(yaw > 2*math.pi):
			yaw = yaw - 2*math.pi
	print roll, pitch, yaw
        
	return yaw
        
    '''This function moves forward the distance indicated'''
    def move_of(self, d, speed=0.1):
	
	#Set the current position of the robot
        x_init = self.odom_pose.position.x
        y_init = self.odom_pose.position.y

        # Set the velocity forward until distance is reached
        while math.sqrt((self.odom_pose.position.x - x_init)**2 + \
             (self.odom_pose.position.y - y_init)**2) < d and not ros.is_shutdown():

            sys.stdout.write("\r [MOVE] The robot has moved of {:.2f}".format(math.sqrt((self.odom_pose.position.x - x_init)**2 + \
            (self.odom_pose.position.y - y_init)**2)) +  "m over " + str(d) + "m")
            sys.stdout.flush()

            msg = Twist()
	    msg.angular.z = 0
            msg.linear.x = speed
            self.vel_ros_pub(msg)
            time.sleep(self.pub_rate)

        sys.stdout.write("\n")

    '''This function turns to the right (negative angle) or to the left (postive angle) through the angle specified'''
    def turn_of(self, a, ang_speed=0.2, thresh = 0.011):

        # Get the initial angle 
        a_init = self.get_z_rotation(self.odom_pose.orientation)
	
	# Set the target angle 
	target = a_init + a

	# Move to the right (negative) or the left (positive)

	# Move to the left (positive)
	if (a > 0):
		#if the target angle is greater than 2pi, subtract 2pi from the target angle to ensure angle reading remains between 0 and 2pi
		if target > (2*math.pi):
			target = target - 2*math.pi

	# Move to the right (negative)
	else:
		#if a is greater than zero, direct the robot to move clockwise (negative direction)
		ang_speed = ang_speed*-1
		# if the target angle is less than zero, add 2pi to the target to ensure angle reading remains between zero and 2pi
		if target < 0:
			target = 2*math.pi+ target

	
	#while the absolute value of our current angle minus our target angle is less than our threshold,continue turning
	while (abs(self.get_z_rotation(self.odom_pose.orientation)-target)>thresh and not ros.is_shutdown()):
		
		print self.get_z_rotation(self.odom_pose.orientation)-target
		msg = Twist()
		msg.angular.z = ang_speed
		msg.linear.x = 0
	        self.vel_ros_pub(msg)
	        time.sleep(self.pub_rate)

        sys.stdout.write("\n")

    '''This function defines the movement for the robot: a right angle triangle with 0.5 legs and a 0.707 hypotenuse
	The robot travels once CCW and then once CW'''
    def move(self):

        # Wait that our python program has received its first messages
        while self.odom_pose is None and not ros.is_shutdown():
            time.sleep(0.1)
	
	#Move CCW
	self.move_of(0.5)
	self.turn_of(3*math.pi/4)
	self.move_of(0.707)
	self.turn_of(3*math.pi/4)
	self.move_of(0.5)
	
	#Move CW
	self.turn_of(math.pi)
	self.move_of(0.5)
	self.turn_of(-3*math.pi/4)
	self.move_of(0.707)
	self.turn_of(-3*math.pi/4)
	self.move_of(0.5)
	self.turn_of(math.pi)
	self.stop_robot()


if __name__ == '__main__':

    # Choose the example you need to run in the command line
    if len(sys.argv) > 1:

        if sys.argv[1] == "vel":
            r = TriangleMoveVel()

        elif sys.argv[1] == "odom":
            r = TriangleMoveOdom()

        else:
            sys.exit(-1)

    else:
        sys.exit(-1)

    # Listen and Publish to ROS + execute moving instruction
    r.start_ros()
    r.move()






