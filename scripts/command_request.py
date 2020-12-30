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


def main():
	"""Main code for command recognition

	Expects for the user to write the command 'play' or 'stop' to 

	'play' will ask for coordinates in order to make the ball appear in the environment
	'stop' will make the ball dissapear of the environment

	Publishers:
		pub: publishes (std_msgs.String) to /command 

	"""
	rospy.init_node('command_request')

	time.sleep(10)
	# Publishers and subscribers
	pub = rospy.Publisher('/command', String, queue_size=10)
	
	rate = rospy.Rate(10) # 10hz

	while not rospy.is_shutdown():

		# Get command from user
		txt = raw_input("Write play or stop to put or hide the ball in the environment: \n")
		if(txt == "Play" or txt == "play" or txt == "PLAY"):
			txt = "play"
			pub.publish(txt)
		
		elif(txt == "Stop" or txt == "stop" or txt == "STOP"):
			txt = "stop"
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
