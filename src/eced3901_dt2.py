#!/usr/bin/env python
import math
import rospy as ros
import sys
import time

from geometry_msgs.msg import Twist, Pose
from nav_msgs.msg import Odometry
from tf.transformations import euler_from_quaternion, quaternion_from_euler


__original_author__ = "Gabriel Urbain"
__modfiers__ = "Nikola Doelle", "Tanaka Akiyama", "Brayden Barnes" 
__copyright__ = "Copyright 2018, IDLab, UGent"

__license__ = "MIT" 
__version__ = "3.0" 
__maintainer__ = "Team33"
__status__ = "Education" 
__date__ = "June 9th, 2021"


class SquareMove(object):
    """
    This class is an abstract class to control a square trajectory on the robot.
    It mainly declares and subscribes to ROS topics in an elegant way.
    """

    def __init__(self):

        # Declare ROS subscribers and publishers
        self.node_name = "square_move"
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

        # We publish for a second to be sure the robot received the message
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



class SquareMoveOdom(SquareMove):
    """
    This class implements a CCW square trajectory based on relative position control,
    where only odometry is used. HOWTO:
     - Start the sensors on the turtlebot:
            $ roslaunch turtlebot3_gazebo turtlebot3_empty_world.launch 
     - Start this node on your computer:
            $ python eced3901_dt2.py odom
    """

    def __init__(self):

        super(SquareMoveOdom, self).__init__()

        self.pub_rate = 0.1
    
    def get_z_rotation(self, orientation):
	'''This function returns the current yaw of the robot'''

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
        
    def move_of(self, d, speed=0.1):
	'''This function moves the robot forward the indicated distance'''
	
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

    
    def turn_of(self, a, ang_speed=0.2, thresh = 0.011):
	'''This function causes the robot to turn right (negative angle) or left (postive angle) for a specified angle'''

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

    def move(self):
    	'''This function defines the path the robot will take in order to make a 1.1m x 1.1m square.
	The robot begins its path by moving forward and turns 90deg CCW every 1.1m,'''

        # Wait until our python program has received its first messages
        while self.odom_pose is None and not ros.is_shutdown():
            time.sleep(0.1)
	
	#Move CCW
	self.move_of(1.1)
	self.turn_of(math.pi/2)
	self.move_of(1.1)
	self.turn_of(math.pi/2)
	self.move_of(1.1)
	self.turn_of(math.pi/2)
	self.move_of(1.1)


if __name__ == '__main__':

    # Choose the example you need to run in the command line
    if len(sys.argv) > 1:

        if sys.argv[1] == "odom":
            r = SquareMoveOdom()

        else:
            sys.exit(-1)

    else:
        sys.exit(-1)

    # Listen and Publish to ROS + execute moving instruction
    r.start_ros()
    r.move()






