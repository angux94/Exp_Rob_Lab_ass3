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
    If "play" command is received, robot goes to PLAY state.
    If no command is received, eventually goes to SLEEP.

    State Machine:
    	NORMAL('play') -> PLAY (if play command received)
    	NORMAL('sleep') -> SLEEP (if no command received)

    Attributes:
    	normal_counter: (int)
    	coords: (motion_plan.msg.PlanningGoal)
	good_coord: (bool)

    Publishers:
	pub: publishes (std_msgs.String) to /status

    Subscribers:
    	sub_command: subscriber (std_msgs.String) to /command
		subscribe to get the play command to enter the PLAY state

    Actions:
    	act_c: Client for action /move_base
		calls the action to move the robot to the specified coordinates
	
		goal: geometry_msgs.PoseStamped

		result: geometry_msgs.Pose
	
    """
    def __init__(self):
        smach.State.__init__(self, outcomes=['play','sleep'])
               
        #Publishers and subscribers
        self.sub_command = rospy.Subscriber('/command', String, cb_command)
	self.pub = rospy.Publisher('/status', String, queue_size=10)
	#Actions
	self.act_c = actionlib.SimpleActionClient('/move_base', MoveBaseAction)
	
	#rospy.loginfo('Waiting for action server to start')
	self.act_c.wait_for_server()
	#rospy.loginfo('Server started')

        # Initializations
        self.normal_counter = 1
	self.coords = MoveBaseGoal() 
	self.good_coord = False

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
	    #Let know to the command_recog node we can accept inputs
	    self.pub.publish('normal')
	    
	    # Amount of random walks before sleeping
            normal_times = random.randrange(1,5)

	    self.good_coord = False
	    while(self.good_coord == False):
            	x = random.randrange(-6,6)
            	y = random.randrange(-8,8)
	    	z = 0
		
		#Check if the random coordinates are reachable in the map
		if x >= 1 and y >= 4:
			self.good_coord = False
			print('Bad coord 1')
		elif x <= -1 and y <= -6:
			self.good_coord = False
			print('Bad coord 2')
		elif x >= 1 and x <=2 and y >= 1 and y <= 3:
			self.good_coord = False
			print('Bad coord 3')
		else:
			print('good coord')
			self.good_coord = True

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
		    
		    self.good_coord = False
                    while(self.good_coord == False):
            		x = random.randrange(-6,6)
            		y = random.randrange(-8,8)
	    		z = 0
		
			#Check if the random coordinates are reachable in the map
			if x >= 1 and y >= 4:
				self.good_coord = False
			elif x <= -1 and y <= -6:
				self.good_coord = False
			elif x >= 1 and x <=2 and y >= 1 and y <= 3:
				self.good_coord = False
			else:
				self.good_coord = True

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
    	sleep_x: (double) Sleep x coordinate 
    	sleep_y: (double) Sleep y coordinate
    	time_sleep: (int) Sleeping time (1,10)

    Attributes:
    	coords: (motion_plan.msg.PlanningGoal)

    Actions:
    	act_c: Client for action /move_base
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

    Robot plays for a said amount of times, going towards the human to know which room to find. If the room is known, goes to the room, if not, goes to FIND state to find the room.

    State Machine:
    	PLAY('find') -> FIND (if room is unknown)
	PLAY('stop') -> NORMAL (when behavior ends)

    State inputs:
	entrance_pin, closet_pin, living_room_pin, kitchen_pin, bathroom_pin, bedroom_pin

    State outputs:
	room_out

    Attributes:
    	play_counter: (int)
	play_times: (int)
	human: (geometry_msgs.Point) human coordinates
	coords: (motion_plan.msg.PlanningGoal)
    
    Subscribers:
    	sub_flag: subscriber (std_msgs.Bool) to /arrived_play
		checks if the robot reached the ball
    	sub_command: subscriber (std_msgs.String) to /command
		subscribe to get the stop command to exit the PLAY state

    Actions:
    	act_c: Client for action /move_base
		calls the action to move the robot to the specified coordinates

		goal: geometry_msgs.PoseStamped

		result: geometry_msgs.Pose
    """
    def __init__(self):
        smach.State.__init__(self, outcomes=['stop', 'find'],
				   input_keys=['entrance_pin', 'closet_pin', 'living_room_pin', 'kitchen_pin', 'bathroom_pin', 'bedroom_pin'],
				   output_keys=['room_out'])

        #Publishers and subscribers
	self.sub_command = rospy.Subscriber('/command', String, cb_command)
        self.sub_flag = rospy.Subscriber('/arrived_play', Bool, cb_flag)

	#Actions
	self.act_c = actionlib.SimpleActionClient('/move_base', MoveBaseAction)

	#rospy.loginfo('Waiting for action server to start')
	self.act_c.wait_for_server()
	#rospy.loginfo('Server started')

	#Initializations
	self.human = Point(x = -5, y = 8)
	self.coords = MoveBaseGoal()  

	self.play_counter = 0
	self.play_times = 0

    def room_select(self):
	"""Function to talk to human and select the destination point
	
	Rooms:
		entrance
		closet
		living room
		kitchen
		bathroom
		bedroom	
	
	Returns:
		room
			The room where we want to play
	"""
	
	print("Human reached! Please tell me a room")
    	room = str(raw_input('room: '))
	if room == 'stop':
		print('we stop playing then')
	else:
    		print("Thanks! Let's reach the " + room)

	return room		


    def execute(self, userdata):
	global sm_flag, room_select
        time.sleep(1)
        rospy.loginfo('Executing state PLAY')
	
	#Get play times each time we run the play state
	if(self.play_counter == 0):	
		self.play_times = random.randrange(1,4)
	
        while not rospy.is_shutdown():       
                
		# If not, continue with the behavior
                if(self.play_counter < self.play_times):
			
			# Increment the count
			self.play_counter += 1
			print("We are playing " + str(self.play_times) + " times")
			print("Counter: " + str(self.play_counter))

			#Go towards the human
			self.coords.target_pose.pose.position.x = self.human.x
			self.coords.target_pose.pose.position.y = self.human.y

			self.coords.target_pose.header.frame_id = "map"
    			self.coords.target_pose.pose.orientation.w = 1.0

			self.act_c.send_goal(self.coords)
			print('Going to human (' + str(self.human.x) + ', ' + str(self.human.y)+ ')')

			# Waits for the server to finish performing the action.
			self.act_c.wait_for_result()
			room = self.room_select()
			userdata.room_out = room

			if(room == "entrance"):
				#Check if the room position is known, if not, find it
				if(userdata.entrance_pin.x==0 and userdata.entrance_pin.y==0):
					print("Entrance is unknown, let's find it")
					return 'find'
				else:				
					print(userdata.entrance_pin)
					
					#Go towards the point
					self.coords.target_pose.pose.position.x = userdata.entrance_pin.x
					self.coords.target_pose.pose.position.y = userdata.entrance_pin.y

					self.coords.target_pose.header.frame_id = "map"
    					self.coords.target_pose.pose.orientation.w = 1.0
					self.act_c.send_goal(self.coords)

					# Waits for the server to finish performing the action.
					self.act_c.wait_for_result()

			if(room == "closet"):
				#Check if the room position is known, if not, find it
				if(userdata.closet_pin.x==0 and userdata.closet_pin.y==0):
					print("Closet is unknown, let's find it")
					return 'find'
				else:				
					print(userdata.closet_pin)

					#Go towards the point
					self.coords.target_pose.pose.position.x = userdata.closet_pin.x
					self.coords.target_pose.pose.position.y = userdata.closet_pin.y

					self.coords.target_pose.header.frame_id = "map"
    					self.coords.target_pose.pose.orientation.w = 1.0
					self.act_c.send_goal(self.coords)

					# Waits for the server to finish performing the action.
					self.act_c.wait_for_result()

			if(room == "living room"):
				#Check if the room position is known, if not, find it
				if(userdata.living_room_pin.x==0 and userdata.living_room_pin.y==0):
					print("Living room is unknown, let's find it")
					return 'find'
				else:				
					print(userdata.living_room_pin)

					#Go towards the point
					self.coords.target_pose.pose.position.x = userdata.living_room_pin.x
					self.coords.target_pose.pose.position.y = userdata.living_room_pin.y

					self.coords.target_pose.header.frame_id = "map"
    					self.coords.target_pose.pose.orientation.w = 1.0
					self.act_c.send_goal(self.coords)

					# Waits for the server to finish performing the action.
					self.act_c.wait_for_result()

			if(room == "kitchen"):
				#Check if the room position is known, if not, find it
				if(userdata.kitchen_pin.x==0 and userdata.kitchen_pin.y==0):
					print("Kitchen is unknown, let's find it")
					return 'find'
				else:				
					print(userdata.kitchen_pin)

					#Go towards the point
					self.coords.target_pose.pose.position.x = userdata.kitchen_pin.x
					self.coords.target_pose.pose.position.y = userdata.kitchen_pin.y

					self.coords.target_pose.header.frame_id = "map"
    					self.coords.target_pose.pose.orientation.w = 1.0
					self.act_c.send_goal(self.coords)

					# Waits for the server to finish performing the action.
					self.act_c.wait_for_result()

			if(room == "bathroom"):
				#Check if the room position is known, if not, find it
				if(userdata.bathroom_pin.x==0 and userdata.bathroom_pin.y==0):
					print("Bathroom is unknown, let's find it")
					return 'find'
				else:				
					print(userdata.bathroom_pin)

					#Go towards the point
					self.coords.target_pose.pose.position.x = userdata.bathroom_pin.x
					self.coords.target_pose.pose.position.y = userdata.bathroom_pin.y

					self.coords.target_pose.header.frame_id = "map"
    					self.coords.target_pose.pose.orientation.w = 1.0
					self.act_c.send_goal(self.coords)

					# Waits for the server to finish performing the action.
					self.act_c.wait_for_result()

			if(room == "bedroom"):
				#Check if the room position is known, if not, find it
				if(userdata.bedroom_pin.x==0 and userdata.bedroom_pin.y==0):
					print("Bedroom is unknown, let's find it")
					return 'find'
				else:				
					print(userdata.bedroom_pin)

					#Go towards the point
					self.coords.target_pose.pose.position.x = userdata.bedroom_pin.x
					self.coords.target_pose.pose.position.y = userdata.bedroom_pin.y

					self.coords.target_pose.header.frame_id = "map"
    					self.coords.target_pose.pose.orientation.w = 1.0
					self.act_c.send_goal(self.coords)

					# Waits for the server to finish performing the action.
					self.act_c.wait_for_result()

		else:
			#Reset play counter
			self.play_counter = 0
			return 'stop'

	
# define state FIND
class Find(smach.State):
    """ Class for the FIND state

    Robot tries to find the desired room going to a random location, and scanning the room until the desired color ball is found, if not, moves to another location and repeats.

    State Machine:
    	FIND('found') -> PLAY (if ball is found)

    State inputs:
	room_in

    State outputs:
	entrance_fout, closet_fout, living_room_fout, kitchen_fout, bathroom_fout, bedroom_fout

    Attributes:
    	go_random: (bool)
	good_coords: (bool)
	coords: (motion_plan.msg.PlanningGoal)
	my_time: (int)
	start_time: (int)

    Publishers:
    	pub_command: publisher (std_msgs.String) to /gesture_request
		publishes the request to enter ball desired coordinates
	pub_color: publisher (std_msgs.String) to /color
		publishes the desired ball color corresponding to the room
    
    Subscribers:
    	sub_flag: subscriber (std_msgs.Bool) to /arrived_play
		checks if the robot reached the ball
    	sub_point: subscriber (geometry_msgs.Point) to /point_located
		subscribe to get the point for which the ball is located

    Actions:
    	act_c: Client for action /move_base
		calls the action to move the robot to the specified coordinates

		goal: geometry_msgs.PoseStamped

		result: geometry_msgs.Pose
   
    """
    def __init__(self):
        smach.State.__init__(self, outcomes=['found'],
				   input_keys=['room_in'],
				   output_keys=['entrance_fout', 'closet_fout', 'living_room_fout', 'kitchen_fout', 'bathroom_fout', 'bedroom_fout'])
        
	#Publishers and subscribers
	self.pub_command = rospy.Publisher('/gesture_request', String, queue_size=10)
	self.pub_color = rospy.Publisher('/color', String, queue_size=10)
	
        self.sub_flag = rospy.Subscriber('/arrived_play', Bool, cb_flag)

	self.sub_point = rospy.Subscriber('/point_located', Point, cb_point)

	#Actions
	self.act_c = actionlib.SimpleActionClient('/move_base', MoveBaseAction)

	#rospy.loginfo('Waiting for action server to start')
	self.act_c.wait_for_server()
	#rospy.loginfo('Server started')

	#Initialization
	self.go_random = True
	self.good_coord = False
	self.coords = MoveBaseGoal() 
	
	self.my_time = 0
	self.start_time = 0

    def execute(self, userdata):

	global sm_flag, sm_point
        time.sleep(1)
        rospy.loginfo('Executing state FIND')

	#We make sure we haven't arrived to the play destination
	sm_flag = False
        while not rospy.is_shutdown():
                
		if(userdata.room_in == 'entrance'):

			if(self.go_random == True):

				self.go_random = False
				#userdata.living_room_fout = Point(x = -3, y = 7)
				self.good_coord = False
				while(self.good_coord == False):
            				x = random.randrange(-6,6)
            				y = random.randrange(-8,8)
	    				z = 0
		
					#Check if the random coordinates are reachable in the map
					if x >= 1 and y >= 4:
						self.good_coord = False
					elif x <= -1 and y <= -6:
						self.good_coord = False
					elif x >= 1 and x <=2 and y >= 1 and y <= 3:
						self.good_coord = False
					else:
						self.good_coord = True


				self.coords.target_pose.pose.position.x = x
				self.coords.target_pose.pose.position.y = y

				self.coords.target_pose.header.frame_id = "map"
    				self.coords.target_pose.pose.orientation.w = 1.0
				self.act_c.send_goal(self.coords)
				print("Going to a random point x: " + str(x) + " y: " + str(y))
				# Waits for the server to finish performing the action.
				self.act_c.wait_for_result()

				self.start_time = time.time()
				self.my_time = 0
			
			#Once it arrives, start searching for the ball
			else:
				self.my_time = time.time()-self.start_time
				
				# Start "watching" the camera
				self.pub_command.publish("play")
				# Tell which color to look for
				self.pub_color.publish("blue")
				if sm_flag:
					sm_flag = False
                        		time.sleep(1)
					self.pub_command.publish("stop")
					userdata.entrance_fout = Point(x = sm_point.x, y = sm_point.y)
					self.go_random = True
					print("Found at: " + str(self.my_time))
					print("Entrance located at x: " + str(sm_point.x) + " y: " + str(sm_point.y))
					return 'found'

				elif(self.my_time >= 120):
					print("Couldn't find the entrance, going to a new location to search")
					self.pub_command.publish("search")
					self.go_random = True

		if(userdata.room_in == 'closet'):
			
			if(self.go_random == True):

				self.go_random = False
				#userdata.living_room_fout = Point(x = -4, y = 2)
				self.good_coord = False
				while(self.good_coord == False):
            				x = random.randrange(-6,6)
            				y = random.randrange(-8,8)
	    				z = 0
		
					#Check if the random coordinates are reachable in the map
					if x >= 1 and y >= 4:
						self.good_coord = False
					elif x <= -1 and y <= -6:
						self.good_coord = False
					elif x >= 1 and x <=2 and y >= 1 and y <= 3:
						self.good_coord = False
					else:
						self.good_coord = True


				self.coords.target_pose.pose.position.x = x
				self.coords.target_pose.pose.position.y = y

				self.coords.target_pose.header.frame_id = "map"
    				self.coords.target_pose.pose.orientation.w = 1.0
				self.act_c.send_goal(self.coords)
				print("Going to a random point x: " + str(x) + " y: " + str(y))
				# Waits for the server to finish performing the action.
				self.act_c.wait_for_result()
				self.start_time = time.time()
				self.my_time = 0			
			#Once it arrives, start searching for the ball
			else:
				self.my_time = time.time()-self.start_time
				print(self.my_time)
				# Start "watching" the camera
				self.pub_command.publish("play")
				# Tell which color to look for
				self.pub_color.publish("red")
				if sm_flag:
					sm_flag = False
                        		time.sleep(1)
					self.pub_command.publish("stop")
					userdata.closet_fout = Point(x = sm_point.x, y = sm_point.y)
					self.go_random = True
					print("Found at: " + str(self.my_time))
					print("Closet located at x: " + str(sm_point.x) + " y: " + str(sm_point.y))
					return 'found'

				elif(self.my_time >= 120):
					print("Couldn't find the closet, going to a new location to search")
					self.go_random = True
					self.pub_command.publish("search")


		if(userdata.room_in == 'living room'):
						
			if(self.go_random == True):

				self.go_random = False
				#userdata.living_room_fout = Point(x = -3, y = -2)
				self.good_coord = False
				while(self.good_coord == False):
            				x = random.randrange(-6,6)
            				y = random.randrange(-8,8)
	    				z = 0
		
					#Check if the random coordinates are reachable in the map
					if x >= 1 and y >= 4:
						self.good_coord = False
					elif x <= -1 and y <= -6:
						self.good_coord = False
					elif x >= 1 and x <=2 and y >= 1 and y <= 3:
						self.good_coord = False
					else:
						self.good_coord = True


				self.coords.target_pose.pose.position.x = x
				self.coords.target_pose.pose.position.y = y

				self.coords.target_pose.header.frame_id = "map"
    				self.coords.target_pose.pose.orientation.w = 1.0
				self.act_c.send_goal(self.coords)
				print("Going to a random point x: " + str(x) + " y: " + str(y))
				# Waits for the server to finish performing the action.
				self.act_c.wait_for_result()
				self.start_time = time.time()
				self.my_time = 0
			#Once it arrives, start searching for the ball
			else:
				self.my_time = time.time()-self.start_time
				print(self.my_time)
				# Start "watching" the camera
				self.pub_command.publish("play")
				# Tell which color to look for
				self.pub_color.publish("green")
				if sm_flag:
					sm_flag = False
                        		time.sleep(1)
					self.pub_command.publish("stop")
					userdata.living_room_fout = Point(x = sm_point.x, y = sm_point.y)
					self.go_random = True
					print("Found at: " + str(self.my_time))
					print("Living room located at x: " + str(sm_point.x) + " y: " + str(sm_point.y))
					return 'found'

				elif(self.my_time >= 120):
					print("Couldn't find the living room, going to a new location to search")
					self.go_random = True
					self.pub_command.publish("search")

		if(userdata.room_in == 'kitchen'):

			if(self.go_random == True):

				self.go_random = False
				#userdata.living_room_fout = Point(x = 2, y = -7)
				self.good_coord = False
				while(self.good_coord == False):
            				x = random.randrange(-6,6)
            				y = random.randrange(-8,8)
	    				z = 0
		
					#Check if the random coordinates are reachable in the map
					if x >= 1 and y >= 4:
						self.good_coord = False
					elif x <= -1 and y <= -6:
						self.good_coord = False
					elif x >= 1 and x <=2 and y >= 1 and y <= 3:
						self.good_coord = False
					else:
						self.good_coord = True


				self.coords.target_pose.pose.position.x = x
				self.coords.target_pose.pose.position.y = y

				self.coords.target_pose.header.frame_id = "map"
    				self.coords.target_pose.pose.orientation.w = 1.0
				self.act_c.send_goal(self.coords)
				print("Going to a random point x: " + str(x) + " y: " + str(y))
				# Waits for the server to finish performing the action.
				self.act_c.wait_for_result()
				self.start_time = time.time()
				self.my_time = 0

			#Once it arrives, start searching for the ball
			else:
				self.my_time = time.time()-self.start_time
				print(self.my_time)
				# Start "watching" the camera
				self.pub_command.publish("play")
				# Tell which color to look for
				self.pub_color.publish("yellow")
				if sm_flag:
					sm_flag = False
                        		time.sleep(1)
					self.pub_command.publish("stop")
					userdata.kitchen_fout = Point(x = sm_point.x, y = sm_point.y)
					self.go_random = True
					print("Found at: " + str(self.my_time))
					print("Kitchen located at x: " + str(sm_point.x) + " y: " + str(sm_point.y))
					return 'found'

				elif(self.my_time >= 120):
					print("Couldn't find the kitchen, going to a new location to search")
					self.go_random = True
					self.pub_command.publish("search")

		if(userdata.room_in == 'bathroom'):
			
			if(self.go_random == True):

				self.go_random = False
				#userdata.living_room_fout = Point(x = 4, y = -4)
				self.good_coord = False
				while(self.good_coord == False):
            				x = random.randrange(-6,6)
            				y = random.randrange(-8,8)
	    				z = 0
		
					#Check if the random coordinates are reachable in the map
					if x >= 1 and y >= 4:
						self.good_coord = False
					elif x <= -1 and y <= -6:
						self.good_coord = False
					elif x >= 1 and x <=2 and y >= 1 and y <= 3:
						self.good_coord = False
					else:
						self.good_coord = True


				self.coords.target_pose.pose.position.x = x
				self.coords.target_pose.pose.position.y = y

				self.coords.target_pose.header.frame_id = "map"
    				self.coords.target_pose.pose.orientation.w = 1.0
				self.act_c.send_goal(self.coords)
				print("Going to a random point x: " + str(x) + " y: " + str(y))
				# Waits for the server to finish performing the action.
				self.act_c.wait_for_result()
				self.start_time = time.time()
				self.my_time = 0

			#Once it arrives, start searching for the ball
			else:
				self.my_time = time.time()-self.start_time
				print(self.my_time)
				# Start "watching" the camera
				self.pub_command.publish("play")
				# Tell which color to look for
				self.pub_color.publish("pink")
				if sm_flag:
					sm_flag = False
                        		time.sleep(1)
					self.pub_command.publish("stop")
					userdata.bathroom_fout = Point(x = sm_point.x, y = sm_point.y)
					self.go_random = True
					print("Found at: " + str(self.my_time))
					print("Bathroom located at x: " + str(sm_point.x) + " y: " + str(sm_point.y))
					return 'found'

				elif(self.my_time >= 120):
					print("Couldn't find the bathroom, going to a new location to search")
					self.go_random = True
					self.pub_command.publish("search")

		if(userdata.room_in == 'bedroom'):

			if(self.go_random == True):

				self.go_random = False
				#userdata.living_room_fout = Point(x = 4, y = 0)
				self.good_coord = False
				while(self.good_coord == False):
            				x = random.randrange(-6,6)
            				y = random.randrange(-8,8)
	    				z = 0
		
					#Check if the random coordinates are reachable in the map
					if x >= 1 and y >= 4:
						self.good_coord = False
					elif x <= -1 and y <= -6:
						self.good_coord = False
					elif x >= 1 and x <=2 and y >= 1 and y <= 3:
						self.good_coord = False
					else:
						self.good_coord = True


				self.coords.target_pose.pose.position.x = x
				self.coords.target_pose.pose.position.y = y

				self.coords.target_pose.header.frame_id = "map"
    				self.coords.target_pose.pose.orientation.w = 1.0
				self.act_c.send_goal(self.coords)
				print("Going to a random point x: " + str(x) + " y: " + str(y))
				# Waits for the server to finish performing the action.
				self.act_c.wait_for_result()
				self.start_time = time.time()
				self.my_time = 0

			#Once it arrives, start searching for the ball
			else:
				self.my_time = time.time()-self.start_time
				print(self.my_time)
				# Start "watching" the camera
				self.pub_command.publish("play")
				# Tell which color to look for
				self.pub_color.publish("black")
				if sm_flag:
					sm_flag = False
                        		time.sleep(1)
					self.pub_command.publish("stop")
					userdata.bedroom_fout = Point(x = sm_point.x, y = sm_point.y)
					self.go_random = True
					print("Found at: " + str(self.my_time))
					print("Bedroom located at x: " + str(sm_point.x) + " y: " + str(sm_point.y))
					return 'found'

				elif(self.my_time >= 120):
					print("Couldn't find the bedroom, going to a new location to search")
					self.go_random = True
					self.pub_command.publish("search")


# Callback functions
sm_command = None
sm_flag = None
sm_point = Point()
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

def cb_point(data):
    global sm_point
    sm_point.x = data.x
    sm_point.y = data.y


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

    # State machine transitions definitions
    sm.userdata.entrance = Point()
    sm.userdata.closet = Point()
    sm.userdata.living_room = Point()
    sm.userdata.kitchen = Point()
    sm.userdata.bathroom = Point()
    sm.userdata.bedroom = Point()
    sm.userdata.room = None

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
                               transitions={'stop':'NORMAL',
					    'find':'FIND'},
			       remapping={'entrance_pin':'entrance','closet_pin':'closet', 'living_room_pin':'living_room', 'kitchen_pin':'kitchen', 'bathroom_pin':'bathroom', 'bedroom_pin':'bedroom', 'room_out':'room'})

	smach.StateMachine.add('FIND', Find(), 
                               transitions={'found':'PLAY'},
			       remapping={'room_in':'room', 'entrance_fout':'entrance', 'closet_fout':'closet', 'living_room_fout':'living_room', 'kitchen_fout':'kitchen', 'bathroom_fout':'bathroom', 'bedroom_fout':'bedroom'})

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
