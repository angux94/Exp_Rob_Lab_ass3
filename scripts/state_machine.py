#!/usr/bin/env python

import rospy
import smach
import smach_ros
import time
from std_msgs.msg import String, Bool
import random
from geometry_msgs.msg import Point
from tf import transformations
import math
import actionlib
import actionlib.msg
import motion_plan.msg
import time
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal

# define state NORMAL
class Normal(smach.State):
    """ Class for the NORMAL state

    Robot walks randomly for a random amount of times.
    If "play" command is received, ball appears and robot goes to PLAY state.
    If no command is received, eventually goes to SLEEP.

    State Machine:
    	NORMAL('play') -> PLAY (if play command received)
    	NORMAL('sleep') -> SLEEP (if no command received)

    Parameters:
    	normal_times: (int) Number of random walks to do

    Attributes:
    	normal_counter: (int)
    	coords: (motion_plan.msg.PlanningGoal)

    Subscribers:
    	sub_command: subscriber (std_msgs.String) to /command
		subscribe to get the play command to enter the PLAY state

    Actions:
    	act_c: Client for action /move_goal
		calls the action to move the robot to the specified coordinates
	
		goal: geometry_msgs.PoseStamped

		result: geometry_msgs.Pose
	
    """
    def __init__(self):
        smach.State.__init__(self, outcomes=['play','sleep'])
               
        #Publishers and subscribers
        self.sub_command = rospy.Subscriber('/command', String, cb_command)

	#Actions
	self.act_c = actionlib.SimpleActionClient('/move_base', MoveBaseAction)
	
	#rospy.loginfo('Waiting for action server to start')
	self.act_c.wait_for_server()
	#rospy.loginfo('Server started')

        # Initializations
        self.normal_counter = 1
	self.coords = MoveBaseGoal() 

    def execute(self, userdata):

        global sm_command

        # Restart the counter every time
        self.normal_counter = 1
        time.sleep(1)
        rospy.loginfo('Executing state NORMAL')

        # Check if there is previously a command in the buffer
        if sm_command == "play":
            print(sm_command)
            sm_command = None
            return 'play'
        
        # If not, proceed to randomly walk
        else:
	    # Amount of random walks before sleeping
            normal_times = rospy.get_param('normal_times',random.randrange(1,5))

            x = random.randrange(-6,6)
            y = random.randrange(-8,8)
	    z = 0

            normal_coord = Point(x = x, y = y, z = z)

	    self.coords.target_pose.pose.position.x = normal_coord.x
	    self.coords.target_pose.pose.position.y = normal_coord.y
	    self.coords.target_pose.pose.position.z = normal_coord.z

	    self.coords.target_pose.header.frame_id = "map"
    	    self.coords.target_pose.pose.orientation.w = 1.0

            # Status control
            print("Robot acting normal")
            print("Times: " + str(normal_times))
            print("Counter: " + str(self.normal_counter))
            print('Coords: ' + str(x) + ', ' + str(y))
            print('--------------------------')

	    #Go to the generated coords
	    self.act_c.send_goal(self.coords)
	    print('Goal: ' + str(x) + ', ' + str(y))

	    # Waits for the server to finish performing the action.
	    self.act_c.wait_for_result()

            while not rospy.is_shutdown():
		# Check if there was a play command in between random walks
                if sm_command == "play":
		    print(sm_command)
                    sm_command = None
                    return 'play'

                # If not, continue with the behavior
                if(self.normal_counter < normal_times):

                    x = random.randrange(-6,6)
                    y = random.randrange(-8,8)
		    z = 0

                    normal_coord = Point(x = x, y = y, z = z)
                    
		    self.coords.target_pose.pose.position.x = normal_coord.x
	    	    self.coords.target_pose.pose.position.y = normal_coord.y
	    	    self.coords.target_pose.pose.position.z = normal_coord.z

	    	    self.coords.target_pose.header.frame_id = "map"
    	    	    self.coords.target_pose.pose.orientation.w = 1.0

                    self.normal_counter = self.normal_counter + 1

                    # Status control
                    print("Times: " + str(normal_times))
                    print("Counter: " + str(self.normal_counter))
                    print('Coords: ' + str(x) + ', ' + str(y))
                    print('--------------------------')

		    #Go to the generated coords
	    	    self.act_c.send_goal(self.coords)
	    	    print('Goal: ' + str(x) + ', ' + str(y))

	    	    # Waits for the server to finish performing the action.
	    	    self.act_c.wait_for_result()

                else: return 'sleep'



# define state SLEEP
class Sleep(smach.State):
    """ Class for the SLEEP state

    The robot goes to the sleep coordinate, stays there for a while, then wakes up and goes into NORMAL state

    State Machine:
    	SLEEP('wait') -> NORMAL (after time passes)

    Parameters:
    	sleep_x: (double) Sleep x coordinate (-8,8)
    	sleep_y: (double) Sleep y coordinate (-8,8)
    	time_sleep: (int) Sleeping time (1,10)

    Attributes:
    	coords: (motion_plan.msg.PlanningGoal)

    Actions:
    	act_c: Client for action /move_goal
		calls the action to move the robot to the specified coordinates

		goal: geometry_msgs.PoseStamped

		result: geometry_msgs.Pose

    """
    def __init__(self):
        smach.State.__init__(self, outcomes=['wait'])      

	#Actions
	self.act_c = actionlib.SimpleActionClient('/move_base', MoveBaseAction)

	#rospy.loginfo('Waiting for action server to start')
	self.act_c.wait_for_server()
	#rospy.loginfo('Server started')

	#Initializations
	self.coords = MoveBaseGoal()  

    def execute(self, userdata):
        time.sleep(1)
        rospy.loginfo('Executing state SLEEP')

        # Coordinates of the sleep position
        sleep_x = rospy.get_param('sleep_x')
        sleep_y = rospy.get_param('sleep_y')
	sleep_z = 0
	
	sleep_coord = Point(x = sleep_x, y = sleep_y, z = sleep_z)

	self.coords.target_pose.pose.position.x = sleep_coord.x
	self.coords.target_pose.pose.position.y = sleep_coord.y
	self.coords.target_pose.pose.position.z = sleep_coord.z

	self.coords.target_pose.header.frame_id = "map"
    	self.coords.target_pose.pose.orientation.w = 1.0

        # Go to sleep position
	print("going to sleep")
	self.act_c.send_goal(self.coords)
	print('Sleep: ' + str(self.coords.target_pose.pose.position.x) + ', ' + str(self.coords.target_pose.pose.position.y))

	# Waits for the server to finish performing the action.
	self.act_c.wait_for_result()

	time_sleep = rospy.get_param('~time_sleep', 10)

        while not rospy.is_shutdown():

            # When arrived, sleep for a fixed time and then continue to NORMAL state
            print("Robot arrived to sleep")
            time.sleep(time_sleep)
            print("Robot woken")
            return 'wait'
        


# define state PLAY
class Play(smach.State):
    """ Class for the PLAY state

    Robot plays for as long as the ball is in the environment, the ball enters when the 'play' 
    command is received and the coordinates are given. Ball disapears once the command 'stop'
    arrives, which sends the ball out of the environment.

    State Machine:
    	PLAY('stop') -> NORMAL (if stop command is received)

    Attributes:
    	play_counter: int

    Publishers:
    	pub_command: publisher (std_msgs.String) to /gesture_request
		publishes the request to enter ball desired coordinates
    
    Subscribers:
    	sub_flag: subscriber (std_msgs.Bool) to /arrived_play
		checks if the robot reached the ball
    	sub_command: subscriber (std_msgs.String) to /command
		subscribe to get the stop command to exit the PLAY state
    """
    def __init__(self):
        smach.State.__init__(self, outcomes=['stop'])

        # Initialization
        self.play_counter = 1

        #Publishers and subscribers
        self.pub_command = rospy.Publisher('/gesture_request', String, queue_size=10)
	
	self.sub_command = rospy.Subscriber('/command', String, cb_command)
        self.sub_flag = rospy.Subscriber('/arrived_play', Bool, cb_flag)

    def execute(self, userdata):

	global sm_flag
        time.sleep(1)
        rospy.loginfo('Executing state PLAY')

	#Gets first play coordinates
	self.pub_command.publish("play")

	#We make sure we haven't arrived to the play destination
	sm_flag = False
        while not rospy.is_shutdown():       
                
		if sm_flag:
			sm_flag = False
                	self.pub_command.publish("play")
                        time.sleep(1)
		
		if(sm_command == "stop"):
			self.pub_command.publish("stop")
			print("Ball dissapeared!")
			time.sleep(10)
			return 'stop'

    

# Callback functions
sm_command = None
sm_flag = None

def cb_command(data):
    """ callback to get the command received on the terminal
    """
    global sm_command
    sm_command = data.data


def cb_flag(data):
    """ callback to set the arrived flag
    """
    global sm_flag
    sm_flag = data.data



# main
def main():
    """ State machine initialization

    Creates the state machine, add states and link their outputs.

    States:
    	NORMAL | PLAY | SLEEP
    
    Transitions:
    	NORMAL -> PLAY
    	
	NORMAL -> SLEEP

    	PLAY -> NORMAL

    	SLEEP -> NORMAL

    """
    global sm_command, sm_flag

    rospy.init_node('state_machine')

    # Create a SMACH state machine
    sm = smach.StateMachine(outcomes=['COORDS'])


    rate = rospy.Rate(10) # 10hz
    # Open the container
    with sm:
        # Add states to the container
        smach.StateMachine.add('NORMAL', Normal(), 
                                transitions={'play':'PLAY', 
                                            'sleep':'SLEEP'})

        smach.StateMachine.add('SLEEP', Sleep(), 
                               transitions={'wait':'NORMAL'})

        smach.StateMachine.add('PLAY', Play(), 
                               transitions={'stop':'NORMAL'})

    # Create and start the introspection server for visualization
    sis = smach_ros.IntrospectionServer('server_name', sm, '/SM_ROOT')
    sis.start()


    # Execute the state machine
    outcome = sm.execute()

    
    # Wait for ctrl-c to stop the application

    rospy.spin()
    sis.stop()


if __name__ == '__main__':
    main()
