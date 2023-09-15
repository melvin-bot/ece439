#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
title: Motors Node - ME439 Intro to robotics, wisc.edu
Author: Peter Adamczyk 
Updated 2021-02-16

motors_node.py
ROS Node to accept commands of "wheel_command_left" and "wheel_command_right" and make the motors run on a robot using the Pololu DRV8835 Raspberry Pi Hat 
"""

import rospy
import numpy as np
# "traceback" is a library that lets you track down errors. 
import traceback
# Import the message types we will need
from std_msgs.msg import Float32 
from basic_motors_and_sensors.msg import WheelCommands
# import the "motors" object and MAX_SPEED setting from the Pololu library for the motor driver. 
from pololu_drv8835_rpi import motors, MAX_SPEED  	# MAX_SPEED is 480 (hard-coded)

# Here we organize the two wheel commands as Global variables (simply by creating them in a Global context). 
# They will be updated by the Subscriber Callbacks and actually commanded to the motor driver at a constat update rate. 
wheel_command_left = 0
wheel_command_right = 0

# Initialize the Node. This can happen here (to be executed as the script is interpreted by Python) or inside a function, but it must take place before any ROS communications take place. 
rospy.init_node('motors_node',anonymous=False)

def listener(): 

    # Subscribe to the "wheel_command_left" topic
    ## 1st argument: topic; 2nd arg: message type; 3rd arg: callback function to with the incoming message will be passed)
    # subL = rospy.Subscriber('/wheel_command_left', Float32, set_wheel_command_left) 
    sub = rospy.Subscriber('/wheel_commands', WheelCommands, set_wheel_commands)
    #### CODE HERE ####
    # Add a Subscriber for the Right wheel
    # Subscribe to the "wheel_command_right" topic
    # subR = rospy.Subscriber('/wheel_command_right', Float32, set_wheel_command_right) 
    #### END CODE ####
    
    # Here set up a loop that will continue until ROS is shutdown (or until the program is interrupted). 
    # Regulate it with a rospy.Rate object to achieve a specific loop rate.
    r = rospy.Rate(10)
    while not rospy.is_shutdown():
        motors.setSpeeds(wheel_command_left, wheel_command_right)
        r.sleep()


# Callback for setting the command of the left wheel 
def set_wheel_commands(msg_in):
    # print(command_list)
    print(msg_in.commandL, msg_in.commandR)
    
    global wheel_command_left
    wheel_command_left = int(np.clip(msg_in.commandL,-MAX_SPEED,MAX_SPEED))
    
    global wheel_command_right
    wheel_command_right = int(np.clip(msg_in.commandR,-MAX_SPEED,MAX_SPEED))

# Section to start the execution, with Exception handling.     
if __name__ == "__main__":
    try: 
        listener()
    except : 
        traceback.print_exc()   # Print any error to the screen. 
        motors.setSpeeds(0,0)   # if there's an exception, stop the motors. 
    
    motors.setSpeeds(0,0)   # upon exit, stop the motors. 
