# -*- coding: utf-8 -*-
"""
@author: arslan

File to initialize current position and final position
"""

from casadi import *
import numpy as NP
import matplotlib.pyplot as plt
from copy import deepcopy	
from matplotlib.ticker import MaxNLocator
import pdb	
import math
import os
import time
import scipy.io
from tf.transformations import euler_from_quaternion, quaternion_from_euler
class mainFb:
    def __init__(self,name,xF,yF):
        name = name
        time.sleep(1)
        execfile("Test.py") # gazebo turtlebot3 subscribers and publishers file
        # this parameter must be 0 because it is not a python simulation
        sim=0

        t_step = 0.4     #time step
        totalSteps=1000  
        tSensDelay=0.1
        end_time = t_step*totalSteps/2
        waitTime=t_step-tSensDelay
        FLAG=1

        ID_com = 1


        xInt1 = float(cur_pos[0])        #x1_initial Position of robot
        yInt1 = float(cur_pos[1])        #y1_initial Position  of robot
        thetaInt1=float(theta1[0])   #Robot initial Angle
        xFin1= float(xF)      #x1_final defined by C#
        yFin1=float(yF)       #y1_final defined by C#

        xInt = []; yInt =[]; thetaInit =[]
        nr_obst = (len(cur_pos)-2)/2
        for i in range(nr_obst):
            xInt.append(cur_pos[i+2])
            yInt.append(cur_pos[i+3])
            thetaInit.append(theta1[i+1])

        x1_0=xInt1
        y1_0=yInt1
        theta1_0=thetaInt1
        goal=NP.array([xFin1,yFin1])



        execfile("MsNmpc.py")
##################################################DESIGNED SCHEDULER THAT WILL BE CHANGED IN THE FUTURE ##########################################################
# function to obtain input docking point
def new_point(xF,yF):
    if(xF==3 and yF==0):
        xF = 3
        yF = 1
    elif(xF==3 and yF==6):
        xF = 3
        yF = 5
    elif(xF==0 and yF==3):
        xF = 1
        yF = 3
    elif(xF==6 and yF==3):
        xF = 5
        yF = 3
    return xF,yF

# function to obtain output docking point
def new_point1(xF,yF):
    if(xF==3 and yF==0):
        xF = 3
        yF = 0.5
    elif(xF==3 and yF==6):
        xF = 3
        yF = 5.5
    elif(xF==0 and yF==3):
        xF = 0.5
        yF = 3
    elif(xF==6 and yF==3):
        xF = 5.5
        yF = 3
    return xF,yF

# checking if destination is a station point
def bound(x,y):
    if((x==3 and y==0)or(x==0 and y==3)or(x==3 and y==6)or(x==6 and y==3)):
        return True
    else:
        return False

# checking if destination is free
def station(xF,yF):
    for i in range(nr_obst):
        if(sqrt((cur_pos[i+2] - xF) ** 2 + (cur_pos[i+3] - yF) ** 2) < 200 / 1000.0):
            return True 
        else:
            return False
# checking if robot is in a station
def in_station(xF,yF):
    if(sqrt((cur_pos[0] - 3) ** 2 + (cur_pos[1] - 0) ** 2) < 500 / 1000.0 or sqrt((cur_pos[0] - 0) ** 2 + (cur_pos[1] - 3) ** 2) < 500 / 1000.0  or sqrt((cur_pos[0] - 3) ** 2 + (cur_pos[1] - 6) ** 2) < 500 / 1000.0 or sqrt((cur_pos[0] - 6) ** 2 + (cur_pos[1] - 3) ** 2) < 500 / 1000.0):
        return True 
    else:
        return False

# finding free spots to position each robot
def free_spot(x,y):
    while(station(x,y)):
        if(x==6):
            x=x-1.5
        elif(x==0):
            x = x +1.5
        elif(y==6):
            y=y-1.5
        elif(y==0):
            y=y+1.5
        else:
            x = x + 1
            y = y + 1
    return x,y


name = sys.argv[1]    # name of robot to be controlled as input argument
time.sleep(1)        
execfile("Test.py")   # gazebo turtlebot3 subscribers and publishers file
time.sleep(1) 
# final destination
xF = float(sys.argv[2]) 
yF = float(sys.argv[3])
# going to output docking point if the robot is in station
if(in_station(cur_pos[0],cur_pos[1]) and sqrt((cur_pos[0] - xF) ** 2 + (cur_pos[1] - yF) ** 2)>500/1000):
    x1,y1 = new_point(cur_pos[0],cur_pos[1])
    mainFb(name,x1,y1)
# loop until reaching destination
while(sqrt((cur_pos[0] - xF) ** 2 + (cur_pos[1] - yF) ** 2) > 100 / 1000.0):
    x,y=free_spot(xF,yF)
    print(x)
    print(y)
    if(bound(x,y)):
        x1,y1 = new_point(x,y)
        print(x1)
        print(y1)
        mainFb(name,x1,y1)
        mainFb(name,x,y)
    else:
        mainFb(name,x,y)
#########################################################################################################################################################################
