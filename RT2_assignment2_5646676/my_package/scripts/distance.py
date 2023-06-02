#!/usr/bin/env python3
import rospy
import time
import math
from my_package.msg import output_client
from nav_msgs.msg import Odometry


def sub_callback(msg):
    global last_print
    # Set the timing in miliseconds 
    timing = time.time()*1000
    #last_print = 0
    freq = 1.0 #frequency
    # Period in miliseconds
    period=(1/freq)*1000 

    # if we haven't printed anything in a time longer than our period, we print again
    if (timing - last_print) > period:
        # Get the desired position from the ros parameters 
        desired_x = float (rospy.get_param ("des_pos_x"))
        desired_y = float (rospy.get_param ("des_pos_y"))    

        # Get the actual position of the robot so that we can calculate the distance to the desired position
        position_x = msg.position_x
        position_y = msg.position_y
      
        # Get the average speed of the robot by combining the velocity in the x and y direction. 
        robot_av_speed =math.sqrt((msg.vel_x * msg.vel_x) + (msg.vel_y * msg.vel_y))
        
        # Get the distance to the target with the math.dist function 
        robot_target_distance = math.dist([desired_x, desired_y], [position_x, position_y])

        # Round up the values so that we don't have only 2 decimals  
        robot_av_speed = round(robot_av_speed,2)
        robot_target_distance = round(robot_target_distance,2)

        # Print the distance and the average speed 
        print ("Distance to desired position : ", robot_target_distance)
        print ("Average speed of the robot : ", robot_av_speed)

        # Update the time of the last printed information 
        timing =time.time()*1000
        last_print = timing



def main():
    # Initialize the node 
    global freq, last_print 
    last_print = 0.0
    # Set current time 
    #last_print = time.time()*10    00
    rospy.init_node('distance')
    
    # Find the frequency in the ros parameters 
    # freq = rospy.get_param ("frequency")

    # Subscribe to the robot's position and velocity message and call sub_callback when a new value is published
    sub_info = rospy.Subscriber("/robot_position", output_client, sub_callback)
    
    rospy.spin()


if __name__ == '__main__':
    main()