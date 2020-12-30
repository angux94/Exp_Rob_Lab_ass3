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
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal


# Global variables
cb_msg = None


def coord_select():
	"""Function to display the grid and select the destination point

	Returns:
		room
			The room where we want to play
	"""
	print("Human reached! Please tell me a room")
    	room = str(raw_input('x :'))
    	print("Thanks! Let's reach the " + room)

	return room

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
	act_c = actionlib.SimpleActionClient('/move_base', MoveBaseAction)
	
	#rospy.loginfo('Waiting for action server to start')
	act_c.wait_for_server()
	#rospy.loginfo('Server started')

	# Initialiizations
	global cb_msg

	human = MoveBaseGoal()
	play_coord = Point(x = 0, y = 0, z = 0)

	rate = rospy.Rate(10) # 10hz

	while not rospy.is_shutdown():
		
		# Check if we have a command
		if(cb_msg == "play"):
			# Clean variable
			cb_msg = None

			#Go to human
			human_x = -6
        		human_y = 8.5

			# Publish coordinates onto the action
			human.target_pose.pose.position.x = human_x
			human.target_pose.pose.position.y = human_y

			human.target_pose.header.frame_id = "map"
    			human.target_pose.pose.orientation.w = 1.0

			#Go towards the human
			act_c.send_goal(human)
			print('Going to human (' + str(human_x) + ', ' + str(human_y)+ ')')

			# Waits for the server to finish performing the action.
			act_c.wait_for_result()

			# Get destination coordinates
			xy = coord_select()


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
