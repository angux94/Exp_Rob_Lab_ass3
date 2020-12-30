#!/usr/bin/env python

import roslib
import rospy
import smach
import smach_ros
import time
import random
from std_msgs.msg import String
from geometry_msgs.msg import Point, Pose
import numpy as np
import matplotlib.pyplot as plt
import actionlib
import actionlib.msg
import motion_plan.msg



# Global variables
cb_msg = None


def coord_select():
	"""Function to display the grid and select the destination point

	Returns:
		xy
			X and Y coordinates of the point selected
	"""

	# 8x8 grid for selecting the destination
	plt.title('Select play destination')
	plt.figure(1)
	for i in range(-8,8):
		plt.plot([i, i], [-8,8], 'k')
		plt.plot([-8,8],[i, i], 'k')

	# Point input
	xy = plt.ginput(1)
	plt.close()
	plt.show()

	return xy

# Callback functions
def callback(data):
	global cb_msg
	cb_msg = data.data
	

def main():
	
	"""Main code for gesture recognition

	After receiving the "play" command, displays a 16x16 grid as the environment to 
        select a play destination to where the ball will go. Publishes the selected coordinates
	
	Subscribers:
		sub: subscriber (std_msgs.String) to /gesture_request
			reads when the play command arrives

	Actions:
		act_c: Client for action /reaching_goal
			calls the action to move the ball to the specified coordinates
	
			goal: geometry_msgs.PoseStamped

			result: geometry_msgs.Pose
	"""
	rospy.init_node('play_coords')

	# Publishers and Subscribers
	sub = rospy.Subscriber('/gesture_request', String, callback)

	#Actions
	act_c = actionlib.SimpleActionClient('/reaching_goal', motion_plan.msg.PlanningAction)
	
	#rospy.loginfo('Waiting for action server to start')
	act_c.wait_for_server()
	#rospy.loginfo('Server started')

	# Initialiizations
	global cb_msg

	goal = motion_plan.msg.PlanningGoal()
	play_coord = Point(x = 0, y = 0, z = 0)

	rate = rospy.Rate(10) # 10hz

	while not rospy.is_shutdown():
		
		# Check if we have a command
		if(cb_msg == "play"):
			# Clean variable
			cb_msg = None

			# Get destination coordinates
			xy = coord_select()

			#storage of the x and y coords of the point
			x = xy[0][0]
			y = xy[0][1]
			z = 0.5

			# Publish coordinates onto the action
			goal.target_pose.pose.position.x = x
			goal.target_pose.pose.position.y = y
			goal.target_pose.pose.position.z = z
			
			act_c.send_goal(goal)
			print('Ball to: ' + str(goal.target_pose.pose.position.x) + ', ' + str(goal.target_pose.pose.position.y)+ ', ' + str(goal.target_pose.pose.position.z))

			# Waits for the server to finish performing the action.
			act_c.wait_for_result()

		if(cb_msg == "stop"):
			# Clean variable
			cb_msg = None
			x = 0
			y = 0
			z = -2
			
			# Publish coordinates to hide ball onto the action
			goal.target_pose.pose.position.x = x
			goal.target_pose.pose.position.y = y
			goal.target_pose.pose.position.z = z

			act_c.send_goal(goal)
			print('(Ball sent to: ' + str(goal.target_pose.pose.position.x) + ', ' + str(goal.target_pose.pose.position.y)+ ', ' + str(goal.target_pose.pose.position.z) + ')')

			# Waits for the server to finish performing the action.
			act_c.wait_for_result()

			
		rate.sleep()
	rospy.spin()

if __name__ == '__main__':
	main()
