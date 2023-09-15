#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Created on Thu Sep 14 18:21:03 2023

@author: pi
"""

import rospy
import encoders
import numpy as np
from matplotlib import pyplot as plt
from std_msgs.msg import Float32

# Initialize the Node. This can happen here (to be executed as the script is interpreted by Python) or inside a function, but it must take place before any ROS communications take place. 
rospy.init_node('motor_speed_test_command_node',anonymous=False)

def talker_for_wheel_commands():
    
    # Set up a Publisher - Left Wheel Command
    ## 1st argument: topic; 2nd arg: message type; 3rd arg: queue_size - use 1 if you only want the latest to be read. See https://wiki.ros.org/rospy/Overview/Publishers%20and%20Subscribers)
    pub_wheel_command_left = rospy.Publisher('/wheel_command_left',Float32,queue_size=2)
    
    # Create the message object (empty at first) that will be populated and then published. Use the message type that was declared in the Publisher. 
    wheel_command_left_msg = Float32()
    
    # Set up a Publisher and Message - Right Wheel Command
    pub_wheel_command_right = rospy.Publisher('/wheel_command_right',Float32,queue_size=2)
    
    # Create the message object (empty at first) that will be populated and then published. Use the message type that was declared in the Publisher.
    wheel_command_right_msg = Float32()
   
    
    # Code for the specific functions of this Node: 
    # Here a while loop that gets user input of desired wheel command and publishes it. 
    # The condition on the "while" (rospy.is_shutdown()) evaluates whether ROS is in the process of shutting down, or has shut down. See https://wiki.ros.org/rospy/Overview/Initialization%20and%20Shutdown 
    if not rospy.is_shutdown(): 
        
        # Wait for the publishers to come online
        rospy.sleep(1.0)
        
        # Time to measure wheel speed over, in seconds
        wheel_measure_delay = 5.0
        stabilization_time = 1.0
        
        # Set up speed command range
        wheel_command_range = np.arange(-480, 480.1, 48)
        #print(wheel_command_range)
        
        # Store encoder counts
        count_rates_left = []
        count_rates_right = []
        
        # Iterate over all command speeds
        for cmd_speed in wheel_command_range:
            
            # Assign speeds to message variables
            wheel_command_left_msg.data = cmd_speed
            wheel_command_right_msg.data = cmd_speed
            
            # Publish speed commands
            pub_wheel_command_left.publish(wheel_command_left_msg)
            pub_wheel_command_right.publish(wheel_command_right_msg)
            
            #print(f'Published command speed of {cmd_speed} to topics /wheel_command_left, /wheel_command_right')
            
            # Sleep for a bit to let the motor speed stabilize
            rospy.sleep(stabilization_time)
            
            # Get motor positions
            [left_enc_initial, right_enc_initial] = encoders.readEncoders() # They return as Ints
            
            #print(f'Got motor positions of {left_enc_initial}, {right_enc_initial} after stabilization delay')
            
            # Let the motors spin for a while
            rospy.sleep(wheel_measure_delay)
            
            # Get motor positions again
            [left_enc_final, right_enc_final] = encoders.readEncoders() # They return as Ints
            
            #print(f'Got motor positions of {left_enc_final}, {right_enc_final} after measurement delay')

            # Calculate counts/s
            count_rate_right = (right_enc_final - right_enc_initial) / wheel_measure_delay
            count_rate_left = (left_enc_final - left_enc_initial) / wheel_measure_delay

            # Store into array
            count_rates_left.append(count_rate_left)
            count_rates_right.append(count_rate_right)
            
            print(f'command speed: {cmd_speed}; left counts/s: {count_rate_left}; right counts/s: {count_rate_right}')
            #print(count_rates_left)
            #print(count_rates_right)
            
        
        # Turn off motors
        wheel_command_left_msg.data = 0
        wheel_command_right_msg.data = 0
        pub_wheel_command_left.publish(wheel_command_left_msg)
        pub_wheel_command_right.publish(wheel_command_right_msg)
        
        # Convert count rates to np arrays
        count_rates_left = np.array(count_rates_left)
        count_rates_right = np.array(count_rates_right)
        
        # Calculate wheel speeds in meters per second
        rev_per_count = 1/1440
        meters_per_rev = 3.1416 * 0.06
        meters_per_count = rev_per_count * meters_per_rev
        speeds_left = meters_per_count * count_rates_left
        speeds_right = meters_per_count * count_rates_right
        
        
        # Plot the results
        fig, ((ax1, ax2), (ax3, ax4)) = plt.subplots(2, 2)
        
        # Left wheel counts
        ax1.plot(wheel_command_range, count_rates_left)
        ax1.set_title("Left wheel counts/s")
        
        # Left wheel speeds
        ax2.plot(wheel_command_range, speeds_left)
        ax2.set_title("Left wheel meters/s")
        
        # Right wheel counts
        ax3.plot(wheel_command_range, count_rates_right)
        ax3.set_title("Right wheel counts/s")
        
        # Right wheel speeds
        ax4.plot(wheel_command_range, speeds_right)
        ax4.set_title("Right wheel meters/s")
        
        plt.show()
        


# Section to start the execution, with Exception handling.         
if __name__ == "__main__": 
    try: 
        talker_for_wheel_commands()
    except rospy.ROSInterruptException: 
        pass
