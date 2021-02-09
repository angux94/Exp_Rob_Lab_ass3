#!/usr/bin/env python

# Python libs
import sys
import time

# numpy and scipy
import numpy as np
from scipy.ndimage import filters

import imutils

# OpenCV
import cv2

# Ros libraries
import roslib
import rospy
import math

# Ros Messages
from sensor_msgs.msg import CompressedImage
from std_msgs.msg import Float64, String, Bool
from geometry_msgs.msg import Twist, Point, Pose
from nav_msgs.msg import Odometry
from tf import transformations

VERBOSE = False
cb_msg = None
cb_color = None

class image_feature:
    """Initialize ros publisher, ros subscriber
	
	Publishers:
		image_pub: publishes (sensor_msgs.CompressedImage) to /robot/output/image_raw/compressed

		vel_pub: publishes (grometry_msgs.Twist) to /robot/cmd_vel

		arrived_pub: publishes (std_msgs.Bool) to /arrived_play

		pub_point: published (geometry_msgs.Point) to /point_located

	Subscribers:
		sub: subscribes (std_msgs.String) to /gesture_request

		sub_color: subscribes (std_msgs.String) to /color

		sub_odom: subscribes (nav_msgs.Odom) to /odom

		subscriber: subscribes to (sensor_msgs.CompressedImage) /robot/camera1/image_raw/compressed   
    """
    def __init__(self):
	global cb_msg
        
        rospy.init_node('camera_ball', anonymous=True)
	
	self.wait_time = rospy.get_param('~wait_time',5)

        # topic where we publish
        self.image_pub = rospy.Publisher("/output/image_raw/compressed",
                                         CompressedImage, queue_size=1)
        self.vel_pub = rospy.Publisher("/cmd_vel",
                                       Twist, queue_size=1)

	self.arrived_pub = rospy.Publisher("/arrived_play",
                                       Bool, queue_size=1)
	
	self.pub_point = rospy.Publisher('/point_located', Point, queue_size=10)

        # subscribed Topic
	self.sub = rospy.Subscriber('/gesture_request', String, self.cb_request)
	
	self.sub_color = rospy.Subscriber('/color', String, self.clbk_color)

	self.sub_odom = rospy.Subscriber('/odom', Odometry, self.clbk_odom)

        self.subscriber = rospy.Subscriber("/camera1/image_raw/compressed",
                                           CompressedImage, self.callback,  queue_size=1)

	self.arrive = False
	self.start_time = time.time()

    def cb_request(self, data):
	""" Gets the command data
    	"""
	global cb_msg
	cb_msg = data.data

    def clbk_color(self, data):
	""" Gets the color information
    	"""
	global cb_color
	cb_color = data.data
	

    def clbk_odom(self, msg):
    	""" Gets the robot odometry data
    	"""
    	global position_
    	global pose_
    	global yaw_

    	# position
    	position_ = msg.pose.pose.position
    	pose_ = msg.pose.pose

    	# yaw
    	quaternion = (
        	msg.pose.pose.orientation.x,
        	msg.pose.pose.orientation.y,
        	msg.pose.pose.orientation.z,
        	msg.pose.pose.orientation.w)
    	euler = transformations.euler_from_quaternion(quaternion)
    	yaw_ = euler[2]

    def callback(self, ros_data):
	"""Callback function of subscribed topic. 
	Here images get converted and features detected
	
	Once the camera detects the ball, starts tracking it and when the robot arrives to the ball, turns it's neck to check its surroundings

	"""
	global cb_msg, position_, pose_, yaw_, cb_color
	
	if VERBOSE:
	        print ('received image of type: "%s"' % ros_data.format)

	#### direct conversion to CV2 ####
	np_arr = np.fromstring(ros_data.data, np.uint8)
	image_np = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)  # OpenCV >= 3.0:
	
	# Range of colors to search
	if cb_color == 'blue' :
		color_low = (100, 50, 50) 
		color_high = (130, 255, 255)
	if cb_color == 'red' :
		color_low = (0, 50, 50) 
		color_high = (5, 255, 255)
	if cb_color == 'green' :
		color_low = (50, 50, 50) 
		color_high = (70, 255, 255)
	if cb_color == 'yellow':
		color_low = (25, 50, 50) 
		color_high = (35, 255, 255)
	if cb_color == 'pink':
		color_low = (125, 50, 50)
		color_high =  (150, 255, 255)
	if cb_color == 'black':
		color_low = (0, 0, 0) 
		color_high = (5,50,50)
	elif cb_color == None:
		color_low = (50, 50, 50) 
		color_high = (70, 255, 255)

	# Image processing to reject posible noisy particles
	blurred = cv2.GaussianBlur(image_np, (11, 11), 0)
	hsv = cv2.cvtColor(blurred, cv2.COLOR_BGR2HSV)
	mask = cv2.inRange(hsv, color_low, color_high)
	mask = cv2.erode(mask, None, iterations=2)
	mask = cv2.dilate(mask, None, iterations=2)
	cnts = cv2.findContours(mask.copy(), cv2.RETR_EXTERNAL,
	                        cv2.CHAIN_APPROX_SIMPLE)
	cnts = imutils.grab_contours(cnts)
	center = None

	if cb_msg == 'play':
		# only proceed if at least one contour was found
		if len(cnts) > 0:
		    # find the largest contour in the mask, then use
		    # it to compute the minimum enclosing circle and
		    # centroid
		    c = max(cnts, key=cv2.contourArea)
		    ((x, y), radius) = cv2.minEnclosingCircle(c)
		    M = cv2.moments(c)
		    center = (int(M["m10"] / M["m00"]), int(M["m01"] / M["m00"]))

		    # only proceed if the radius meets a minimum size
		    if radius > 10:
			# draw the circle and centroid on the frame,
			# then update the list of tracked points
			cv2.circle(image_np, (int(x), int(y)), int(radius),
			           (0, 255, 255), 2)
			cv2.circle(image_np, center, 5, (0, 0, 255), -1)
			vel = Twist()
			vel.angular.z = -0.002*(center[0]-400)
			vel.linear.x = -0.01*(radius-100)
			self.vel_pub.publish(vel)
			
			#Only perform this, if we haven't arrived to the ball yet
			if(self.arrive == False):
				# Chech if the radius difference is small enough to consider we arrived
				rad_check = 2
				if(abs(radius-100) < rad_check):
					#Publish we arrived to the ball
					print("Reached the ball")
					self.arrived_pub.publish(True)
					self.arrive = True
					self.pub_point.publish(position_)
					print(position_)	
				

		    else:
			vel = Twist()
			vel.linear.x = 0.5
			self.vel_pub.publish(vel)

		else:
		    vel = Twist()
		    vel.angular.z = 0.5
		    self.vel_pub.publish(vel)
		    
		    # Reset the state to not arrived
		    self.arrive = False

		#cv2.imshow('window', image_np)
		cv2.waitKey(2)
	
	# If 'stop' command, start searching for the ball
	elif cb_msg == 'stop':
		cb_msg = None

	# Stop searching for the ball while going to a new location
	elif cb_msg == 'search':
		#cv2.imshow('window', image_np)
		cv2.waitKey(2)
	else:
		cv2.waitKey(2)
	# self.subscriber.unregister()


def main(args):
    """Initializes and cleanup ros node"""
    ic = image_feature()
    try:
        rospy.spin()
    except KeyboardInterrupt:
        print ("Shutting down ROS Image feature detector module")
    cv2.destroyAllWindows()


if __name__ == '__main__':
    main(sys.argv)
