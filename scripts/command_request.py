#!/usr/bin/env python

import roslib
import rospy
import smach
import smach_ros
import time
import random
from std_msgs.msg import String
from geometry_msgs.msg import Point
import numpy as np
import matplotlib.pyplot as plt

# Initialize status as normal
status = "normal"

def callback(data):
    """ callback to get the status received from the state machine
    """
    global status
    status = data.data

def main():
	"""Main code for command recognition

	Expects for the user to write the command 'play' or 'stop' to 

	'play' will make the robot play with the human
	'stop' will make the robot to stop playing

	Publishers:
		pub: publishes (std_msgs.String) to /command 

	Subscribers:
		sub: subscribes (std_msgs.String) to /status

	"""
	rospy.init_node('command_request')

	global status

	time.sleep(20)
	# Publishers and subscribers
	pub = rospy.Publisher('/command', String, queue_size=10)	
	sub = rospy.Subscriber('/status', String, callback)

	rate = rospy.Rate(10) # 10hz
	
	while not rospy.is_shutdown():
		if status == "normal":
			# Get command from user
			txt = raw_input("Write play to play with the robot: \n")
			if(txt == "Play" or txt == "play" or txt == "PLAY"):
				txt = "play"
				print('command play received')
				status = "play"
				pub.publish(txt)
			
			# Code for invalid comand and retry
			else:
				print("Your command '" + txt + "' is not valid.")
				print("Please write a valid command")
				print("")
				continue

		rate.sleep()

	rospy.spin()

if __name__ == '__main__':
	main()
