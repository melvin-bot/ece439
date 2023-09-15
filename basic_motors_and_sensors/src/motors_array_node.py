#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
title: Motors Node - ME439 Intro to robotics, wisc.edu
Author: Peter Adamczyk 
Updated 2022-10-04

motors_array_node.py
ROS Node to accept commands of "wheel_commands" (two-element Float32MultiArray of Left and Right commands) and make the motors run on a robot using the Pololu DRV8835 Raspberry Pi Hat 
"""

import rospy
# "traceback" is a library that lets you track down errors. 
import traceback
# Import the message types we will need
from std_msgs.msg import Float32MultiArray 
# import the "motors" object and MAX_SPEED setting from the Pololu library for the motor driver. 
from pololu_drv8835_rpi import motors, MAX_SPEED  	# MAX_SPEED is 480 (hard-coded)

# Initialize the Node. This can happen here (to be executed as the script is interpreted by Python) or inside a function, but it must take place before any ROS communications take place. 
rospy.init_node('motors_array_node',anonymous=False)

def listener(): 

    # Subscribe to the ***"wheel_commands"*** topic - type Float32MultiArray
    ## 1st argument: topic; 2nd arg: message type; 3rd arg: callback function to with the incoming message will be passed)
    sub = rospy.Subscriber('/wheel_commands_array', Float32MultiArray, set_wheel_commands) 
   
    rospy.spin()    # keep the node from exiting


# Callback for setting the command of the left wheel    
def set_wheel_commands(msg_in): 
    wheel_command_left = int(msg_in.data[0])
    wheel_command_right = int(msg_in.data[1])
    motors.setSpeeds(wheel_command_left,wheel_command_right)
    #    rospy.loginfo("left wheel set to {0}, right wheel set to {1}".format(wheel_command_left, wheel_command_right))   # could issue a Log message to ROS about it

# Section to start the execution, with Exception handling.     
if __name__ == "__main__":
    try: 
        listener()
    except : 
        traceback.print_exc()   # Print any error to the screen. 
        motors.setSpeeds(0,0)
        pass
    
    motors.setSpeeds(0,0)
    pass