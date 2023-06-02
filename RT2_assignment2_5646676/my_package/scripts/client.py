#!/usr/bin/env python3

import rospy
import actionlib
import actionlib_msgs
import assignment_2_2022.msg
from nav_msgs.msg import Odometry
from my_package.msg import output_client 
from my_package.srv import target 
from geometry_msgs.msg import Twist, Pose, Point
import sys
import select


# This function call the service node if the user input is 'service'
def call_service ():
    
    # Wait for the node to respond
    rospy.wait_for_service('service')
    service = rospy.ServiceProxy('service', target)
    # Call the service
    response = service()
    # Print the number of goals reached and cancelled
    print("Goals reached: ", response.reached)
    print("Goals cancelled: ", response.cancelled)


# This function is used to publish the informations position and velocity
def publish_infos (msg):
        global pub 
        # Set Output with the custom msg structure 
        Output = output_client() 
        # Get position 
        position = msg.pose.pose.position 
        # Get twist 
        velocity = msg.twist.twist.linear 

        #Set the different parmaeters inside Output with the coresponding value. 
        Output.position_x = position.x
        Output.position_y = position.y
        Output.vel_x = velocity.x
        Output.vel_y = velocity.y

        # Publish the message on the topic
        pub.publish(Output) 
       

def client():

        # Initialize the action client
        client = actionlib.SimpleActionClient("reaching_goal", assignment_2_2022.msg.PlanningAction)
        # Wait for the server to be started
        client.wait_for_server()
        # initialize x and y (those are the user input)
        x = 0
        y = 0
        # This loop will run as long as rospy is running
        while not rospy.is_shutdown():
            # Get the input from user 
            # Input has to be a float or 'service'
            print ("Enter 'service' to access the service node ")
            x = input("Enter a goal x: ")
            y = input("Enter a goal y: ")

            
            if (x == "service") or (y == "service"):
                call_service() 
            else : 
                try : 
                    # Check if x and y are valid float values. 
                    x = float (x)
                    y = float (y)

                except ValueError:
                    # If an error is raised, print this message.
                    print (" Error, input was not a float number. Please input a number or 'service'")
                    # This will go back to the main loop without crashing the programm
                    # That way the user can try to input another number
                    continue
                
                # Set up the target position
                goal = assignment_2_2022.msg.PlanningGoal()

                goal.target_pose.pose.position.x = x
                goal.target_pose.pose.position.y = y
                goal.target_pose.pose.orientation.w = 1.0
                
                # Send the goal to the server
                client.send_goal(goal)

                # Let 3 seconds to cancel the target. 
                print("press x to cancel the target")
                cancel=select.select ([sys.stdin],  [], [], 3) [0] 
                if cancel:
                    i = sys.stdin.readline().rstrip()
                    if(i=="x"):
                        client.cancel_goal()
                        print("The goal was cancelled")
                       

def main():
    # Initialize the node 
    rospy.init_node("client")

    # Set the pub variable as global so that we can access it from every function
    global pub

    # Publisher to publish the informations velocity and postion with the custom message structure
    pub = rospy.Publisher("/robot_position", output_client, queue_size=10)
   
    # Subscriber to get the informations from /odom
    odom_sub = rospy.Subscriber("/odom", Odometry, publish_infos)

    # Call the client function
    client()


if __name__ == '__main__':
    main()  