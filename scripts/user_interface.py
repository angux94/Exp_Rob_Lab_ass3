#! /usr/bin/env python

# import ros stuff
import rospy
from std_srvs.srv import *
import time
from geometry_msgs.msg import PoseStamped
import actionlib
import actionlib.msg
import motion_plan.msg
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal

# service callback

coords = MoveBaseGoal() 
def set_new_pos():

    global coords
    print("Target reached! Please insert a new position")
    x = float(raw_input('x :'))
    y = float(raw_input('y :'))
    rospy.set_param("des_pos_x", x)
    rospy.set_param("des_pos_y", y)
    print("Thanks! Let's reach the next position")

    coords.target_pose.header.frame_id = "map"
    coords.target_pose.pose.position.x = x
    coords.target_pose.pose.position.y = y
    coords.target_pose.pose.orientation.w = 1.0

    act_c.send_goal(coords)
    # Waits for the server to finish performing the action.
    act_c.wait_for_result()
    return []


def main():
    global coords
    rospy.init_node('user_interface')

    act_c = actionlib.SimpleActionClient('/move_base', MoveBaseAction)
    act_c.wait_for_server()

    
    time.sleep(10)
    x = rospy.get_param("des_pos_x")
    y = rospy.get_param("des_pos_y")
    coords.target_pose.header.frame_id = "map"
    coords.target_pose.pose.position.x = x
    coords.target_pose.pose.position.y = y
    coords.target_pose.pose.orientation.w = 1.0

    act_c.send_goal(coords)
    print("Hi! We are reaching the first position: x = " +
          str(x) + ", y = " + str(y))
    # Waits for the server to finish performing the action.
    act_c.wait_for_result()
    
    rate = rospy.Rate(20)
    while not rospy.is_shutdown():
	set_new_pos()
        rate.sleep()


if __name__ == '__main__':
    main()
