#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
title: Motors Array Command Node - ME439 Intro to robotics, wisc.edu
Author: Peter Adamczyk 
Updated 2022-10-04

motors_array_command_node.py
ROS Node to get User Input of commands for wheel commands (left AND right) and publish them to a topic (type Float32MultiArray) to set wheel commands in another node.  
"""

import rospy
from std_msgs.msg import Float32MultiArray
import numpy as np

# Initialize the Node. This can happen here (to be executed as the script is interpreted by Python) or inside a function, but it must take place before any ROS communications take place. 
rospy.init_node('motors_array_command_node',anonymous=False)

def talker_for_wheel_commands():
    
    # Set up a Publisher
    ## 1st argument: topic; 2nd arg: message type; 3rd arg: queue_size - use 1 if you only want the latest to be read. See https://wiki.ros.org/rospy/Overview/Publishers%20and%20Subscribers)
    # Use a single publisher for both Left and Right wheel commands
    # Use Float32MultiArray type, pack the field "data" with a list [left, right]
    pub_wheel_commands_array = rospy.Publisher('wheel_commands_array',Float32MultiArray,queue_size=1)
    
    # Create the message object (empty at first) that will be populated and then published. Use the message type that was declared in the Publisher. 
    wheel_commands_array_msg = Float32MultiArray()
    
    # Code for the specific functions of this Node: 
    # Here a while loop that gets user input of desired wheel command and publishes it. 
    # The condition on the "while" (rospy.is_shutdown()) evaluates whether ROS is in the process of shutting down, or has shut down. See https://wiki.ros.org/rospy/Overview/Initialization%20and%20Shutdown 
    while not rospy.is_shutdown(): 
    
        # use the Python "input" command to get a value. Use the int() function to turn the string into an integer. 
        # Note the funky syntax for getting more than one value: 
        wheel_commands_array = [int(x) for x in input('Enter wheel commands (-480 to +480): cmdL,cmdR \n').split(',')]
        
        # Check for good inputs and fix them if bad (numpy.clip) 
        # and pack the message object with the current data.
        wheel_commands_array_msg.data = np.clip(wheel_commands_array,-480,480)
        # # Extra stuff that helps with packaging the message "correctly" - but isn't actually necessary if we are Unpacking it ourselves in the associated subscriber! 
        # # see https://answers.ros.org/question/325559/how-can-we-actually-use-float32multiarray-to-publish-2d-array-using-python/ 
        # wheel_commands_array_msg.layout.data_offset = 0
        # from std_msgs.msg import MultiArrayDimension
        # wheel_commands_array_msg.layout.dim = [MultiArrayDimension()]
        # wheel_commands_array_msg.layout.dim[0].label = "wheel_number"
        # wheel_commands_array_msg.layout.dim[0].size = 2
        # wheel_commands_array_msg.layout.dim[0].stride = 2
        
        # Publish the message. 
        pub_wheel_commands_array.publish(wheel_commands_array_msg)
        

# Section to start the execution, with Exception handling.         
if __name__ == "__main__": 
    try: 
        talker_for_wheel_commands()
    except rospy.ROSInterruptException: 
        pass
