#!/usr/bin/env python
import rospy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
import rospy
import math
from std_msgs.msg import Float64
import numpy as np                    #import numpy for trignometric function, arrays... etc
import sys                            #import sys for extracting input from termminal (input from user)
from nav_msgs.msg import Odometry
from tf.transformations import euler_from_quaternion, quaternion_from_euler

rospy.init_node('test', anonymous=True)
# global cmd_pub1
cmd_pub1 = rospy.Publisher("/"+str(name)+"/cmd_vel", Twist,queue_size=10)
global yaw
global cur_pos
global theta1
global nr_obst
global stuck
flag_initial_Pos = 0	#Initialize flag by zero
xcordinit = 0 
ycordinit = 0
thetayawinit = 0
xcord = 0
ycord = 0
thetayaw = 0
flag_initial_Pos_3 = 0	#Initialize flag by zero
xcordinit_3 = 0 
ycordinit_3 = 0
thetayawinit_3 = 0
xcord2 = 0
ycord2 = 0
thetayaw2 = 0
xcord3 = 0
ycord3 = 0
thetayaw3 = 0
xcord4 = 0
ycord4 = 0
thetayaw4 = 0
cur_pos = [xcord,ycord,xcord2,ycord2,xcord3,ycord3,xcord4,ycord4] # x and y positions of robots
theta1 = [thetayaw,thetayaw2,thetayaw3,thetayaw4] # theta orientations of the robots
theta_des=0
yaw=0
##Callback function for feedback the vehicle current position
#Callback function which is called when a new message of type Pose is received by the subscriber
def callback(data):
  global pos_msg	#Identify msg variable created as global variable
  global sub2		#Identify a subscriber as global variable
  global thetayaw
  global xcord
  global ycord
  
  #pos_msg = data				#Initialize pos_msg with data sent to the function
  cur_pos[0] = round(data.pose.pose.position.x, 4)		#Round the value of x to 4 decimal places
  cur_pos[1] = round(data.pose.pose.position.y, 4)		#Round the value of y to 4 decimal places
  quaternion = (data.pose.pose.orientation.x,
		data.pose.pose.orientation.y,
		data.pose.pose.orientation.z,
		data.pose.pose.orientation.w)
  euler = euler_from_quaternion(quaternion)
  theta1[0] = euler[2]
  #theta[0] = round(yaw, 4)	#Round the value of theta to 4 decimal places
if name == "robot1":  
  sub2 = rospy.Subscriber("/robot1/odom", Odometry, callback) #Identify the subscriber "sub2" to subscribe topic "/turtle1/pose" of type "Pose"
else:
  sub2 = rospy.Subscriber("/"+str(name)+"/odom", Odometry, callback)
#######################################################################
##Callback function for feedback the vehicle current position
#Callback function which is called when a new message of type Pose is received by the subscriber
def callforward(data):
  global pos_msg_3	#Identify msg variable created as global variable
  global sub2_3		#Identify a subscriber as global variable
  global thetayaw2
  global xcord2
  global ycord2
  
  #pos_msg = data				#Initialize pos_msg with data sent to the function
  cur_pos[2] = round(data.pose.pose.position.x, 4)		#Round the value of x to 4 decimal places
  cur_pos[3] = round(data.pose.pose.position.y, 4)		#Round the value of y to 4 decimal places
  quaternion = (data.pose.pose.orientation.x,
		data.pose.pose.orientation.y,
		data.pose.pose.orientation.z,
		data.pose.pose.orientation.w)
  euler = euler_from_quaternion(quaternion)
  theta1[1] = euler[2]
  #theta[1] = round(yaw, 4)	#Round the value of theta to 4 decimal places
if name == "robot2":
  sub1 = rospy.Subscriber("/robot1/odom", Odometry, callforward) #Identify the subscriber "sub2" to subscribe topic "/turtle1/pose" of type "Pose"
else:
  sub1 = rospy.Subscriber("/robot2/odom", Odometry, callforward) #Identify the subscriber "sub2" to subscribe topic "/turtle1/pose" of type "Pose"
#######################################################################
#Callback function which is called when a new message of type Pose is received by the subscriber
def callback3(data):
  global pos_msg	#Identify msg variable created as global variable
  global sub3		#Identify a subscriber as global variable
  global thetayaw3
  global xcord3
  global ycord3
  
  #pos_msg = data				#Initialize pos_msg with data sent to the function
  cur_pos[4] = round(data.pose.pose.position.x, 4)		#Round the value of x to 4 decimal places
  cur_pos[5] = round(data.pose.pose.position.y, 4)		#Round the value of y to 4 decimal places
  quaternion = (data.pose.pose.orientation.x,
		data.pose.pose.orientation.y,
		data.pose.pose.orientation.z,
		data.pose.pose.orientation.w)
  euler = euler_from_quaternion(quaternion)
  theta1[2] = euler[2]
  #theta[2] = round(yaw, 4)	#Round the value of theta to 4 decimal places
if name == "robot3":
  sub3 = rospy.Subscriber("/robot1/odom", Odometry, callback3) #Identify the subscriber "sub2" to subscribe topic "/turtle1/pose" of type "Pose"
else:
  sub3 = rospy.Subscriber("/robot3/odom", Odometry, callback3) #Identify the subscriber "sub2" to subscribe topic "/turtle1/pose" of type "Pose"
#######################################################################
#######################################################################
#Callback function which is called when a new message of type Pose is received by the subscriber
def callback4(data):
  global pos_msg	#Identify msg variable created as global variable
  global sub4		#Identify a subscriber as global variable
  global thetayaw4
  global xcord4
  global ycord4
  
  #pos_msg = data				#Initialize pos_msg with data sent to the function
  cur_pos[6] = round(data.pose.pose.position.x, 4)		#Round the value of x to 4 decimal places
  cur_pos[7] = round(data.pose.pose.position.y, 4)		#Round the value of y to 4 decimal places
  quaternion = (data.pose.pose.orientation.x,
		data.pose.pose.orientation.y,
		data.pose.pose.orientation.z,
		data.pose.pose.orientation.w)
  euler = euler_from_quaternion(quaternion)
  theta1[3] = euler[2]
  #theta[3] = round(yaw, 4)	#Round the value of theta to 4 decimal places
if name == "robot4":
  sub4 = rospy.Subscriber("/robot1/odom", Odometry, callback4) #Identify the subscriber "sub2" to subscribe topic "/turtle1/pose" of type "Pose"
else:
  sub4 = rospy.Subscriber("/robot4/odom", Odometry, callback4) #Identify the subscriber "sub2" to subscribe topic "/turtle1/pose" of type "Pose"


nr_obst = 3
global cmd_msg1
cmd_msg1 = Twist()

   
