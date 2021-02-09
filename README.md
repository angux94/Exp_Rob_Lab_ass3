# Exp_Rob_Lab_ass3
Andres Gutierrez Experimental Robotics Lab Assignment 3 Repository

Introduction
--------------

This is the Assignment 3 of Experimental Robotics Laboratory.

The code simulates a "dog" robot which walks randomly on an environment with different rooms, where a colored ball is located in a corner of each room, linking the color to the physical space it represents. The robot goes to sleep if the user does not inputs a "play" command.
The play command can be input at any times and the robot will finish its movement and go to play.

When the "play" command is given the robot will finish its current action and then it will go to play with the human, which will provide the name of the room he wished the robot to go and play. If the room location is known, the robot will go directly to said location, if not, the robot will try to find it. The robot will locate the ball corresponding to the room and it will approach it, after it arrives, it will save that location, and the room position will be known from now on.

Software Architecture and States Diagrams
----------------------------------------

For the software architecture:
![](https://github.com/angux94/Exp_Rob_Lab_ass3/blob/main/Exp_Rob_Lab_Assignment3_Architecture.png)

The architecture shows how the program is structured and what is the working principle of the program. First the "command_request" node will wait for the user to give an input to the program, which will comunicate it to the "state_machine" node, which will be running the NORMAL state. "state_machine" is the brain of the program, this will come back and forth between NORMAL and SLEEP states, making use of the "move_base" action to move the robot through the environment. Once the command is received, it will trigger the PLAY state, and the robot will ask for a room to the user. If the last one is known, it will use "move_base" action to go to the location, if not, it will use it to go to a random location and then rely on the robot's camera through "camera_ball" to find the colored ball which represents each one of the rooms and obtain its position.

For the State Machine Diagram:

![](https://github.com/angux94/Exp_Rob_Lab_ass3/blob/main/State%20Machine%20diagram.png)

The State Machine has 4 states:
  - NORMAL: Main state in which the robot walks randomly until the "play" command arrives and goes to PLAY state, if not, goes to SLEEP state.
  - SLEEP: The robot goes to the sleep position and rests for a while, afterwards wakes up and goes to the NORMAL state.
  - PLAY: The robot plays for a set number of times, requesting the room to the human.
  - FIND: The robot starts searching for the colored ball respective to the room and stores its location once it finds it. If it takes too long, goes back to PLAY.
  
Messages
----------

The message types used in the project were:
  - geometry_msgs Point: for x,y coordinates of the robot.
  - geometry_msgs Pose: for pose data which includes positions.
  - geometry_msgs Twist: for robot velocities.
  - sensor_msgs CompressedImage: to get and show the image using opencv.
  - std_msgs String: for strings.
  - std_msgs Bool: for booleans.
  - MoveBaseGoal(): for actionlib action goal to move towards a location.
  - nav_msgs Odom: for odometry data.
  
Actions
-----------

The action used are located in the /action folder, it has the shape:
  - goal: geometry_msgs PoseStamped 
  - response: geomterty_msgs Pose 
  
This action message is used to move the robot to said coordinates which is the /move_base action.
  
Parameters
-------------

The parameter are to be specified just on the sm_assignment file:
  - sleep_x: Bed x coordinate 
  - sleep_y: Bed y coordinate 
  - time_sleep: Time the robot sleeps
  
Packages and File list
------------
- Launch:
The launch file is launcher.launch located inside /launch folder. Which is in charge of launching other .launch files such as gmapping.launch, move_base.launch and simulation.launch.

  -simulation.launch has the created nodes are at the end of the file, with their respective parameters, as well as the definition of the robot used and the human positions.

- Nodes:
All the executable files are in /scripts folder which are the .py files for each node.

  -camera_ball: Node in charge of controlling the camera installed on the robot. Once it enters in the FIND state, it starts searching on the image for a colored ball, after it founds it, starts approaching to it and when the robot arrives to the ball, stores the coordinates to link them to the room.
  
  -command_request: Node in charge of reading the inputs of the user, "play" or "stop" are the valid commands.
  
  -state_machine: Node in charge of managing the state machine of the program. Will switch through states and maganes the main behavior of the robot. It's also the client of /move_base actons. Is in charge on handling the transitions and checking conditions.
  
  -move_base: Node for moving the robot to the given position in the environment, takes into account the map information in order to avoid obstacles found in the map.

- The robot structure files can be found inside /urdf folder under the ex3_arm.xacro and ex3_arm.gazebo name files, which are the ones that define the structure of the robot, and its controlled joint.

- For the control file is can be found inside /config as ex3_motors.yaml.

- The /worlds folder contains the world definition, this file hasn't been modified after its delivery.

- The documentation can be found in the folder : /docs
In order to open it on firefox, open a terminal inside /docs folder and run: firefox _build/html/index.html

Intstallation and Running Procedure
-----------------

Download the project, or clone the repository in the desired folder on your computer.

Use: 
  - roslaunch exp_assignment3 launcher.launch

It will launch all the required nodes and the program will be fully functional. parameters can be specified on the same command or inside the .launch file located on the /launch folder.

Working Hypothesys and Environment
-------------

For the working hypothesys the robot should start in a NORMAL state, for which will move randomly until the play command arrives or the amount of times finishes and goes to SLEEP, giving the image of the camera all the time. If it goes to SLEEP, afterwards re-enters on the NORMAL behavior. If a "play" command is sent during the random walks, the robot will finish its current walk and then enter the PLAY state even if he hasn't finished all the random walks. Then the robot will approach the human and ask for a room to play in. If the room is known, the robot will go directly, if not, the robot will enter FIND state and search for the room using the camera information to detect the respective colored ball. The robot will arrive to a random location and check its environment, if the ball is detected it will approach it and store its location, finding the room and going back to the PLAY state. If it doesn't detects the ball, it will go to a new random location and repeat for a set amount of times. 

When a random point has been reached within the FIND state, the robot will look its surroundings for 150 seconds, this could be adjusted depending on the user's computation power. Then if the ball is not found and reached withing that time, the robot will go to a new location.

When the robot goes to SLEEP, he must wake up first in order to enter the play state, if a command is sent during the sleeping time, he will wake up, go to NORMAL state and directly to PLAY state.

The environment is an appartment, with a man and different rooms, each one linked to a colored ball, following the mapping:
  
  - entrance -> blue
  - closet -> red
  - living room -> green
  - kitchen -> yellow
  - bathroom -> pink
  - bedroom -> black
  
The robot has a specific sleep position to which it will go when perforing SLEEP state.

System's Features
------------
  - Commands can be input at any time.
  - Terminal shows the "usually" interesting data.
  
System's Limitations and technical improvements
------------

Due to the computational power needed to develop this project, the behavior of tracking unknown balls in the NORMAL state wasn't implemented, as it demanded to constantly run the algorithm to analyze the image. Instead, the image is just taken into account when performing the FIND state, in order to reduce the computational demand.

The project was tested on all the possible scenarios I imagined for miss-behaviors to happen, but due to the high computational demand and the slow performance during the testing, some situations might rise which I could not face as I couldn't run the program for "long periods of time" (It takes long real-world time to perform a small "simulation time" waiting up to 5-10 minutes for the robot just to go to a location on the NORMAL state, get the input command to switch to the PLAY state and come back to the human to get the desired room).

"Stop" command could be added afterwards, but was interfering with the room name request creating multiple input requests at some points.

Author and Contacts
------
Andres Gutierrez
email: andresgut94@gmail.com

EMARO+ master in Advanced Robotics
