#! /usr/bin/env python3

import rospy
import actionlib
import actionlib_msgs
import assignment_2_2022.msg
from my_package.srv import target, targetResponse


def result(msg):
    # Check the status of the robot 
    status=msg.status.status
    # If status is 2, it means that a goal has been cancelled 
    if status == 2: 
        goal.cancelled += 1
    # Status  = 3 means that a goal has been reached.
    elif status == 3: 
        goal.reached += 1

def goal_callback(req):
    # Print the number of goals in this window but also return it to the client that called this service node. 
    print ("Number of goals reached:", goal.reached )
    print ("Number of goals cancelled:", goal.cancelled )
    return goal


def main():
    # Set the goal as a global variable so it can be used in every function
    global goal

    # Set up goal with the correct service structure defined in /srv/target.srv
    goal = targetResponse()

    # Init the node 
    rospy.init_node('service')
    # Set up the service with the service structure and call a goal_callback when triggered.
    srv= rospy.Service('service', target, goal_callback)

    # Subscriber to this topic to get the state of the robot.
    # Based on the state, we will be able to know how many goals were reached or cancelled 
    subscriber_result = rospy.Subscriber('/reaching_goal/result', assignment_2_2022.msg.PlanningActionResult, result)
    rospy.spin()

if __name__ == "__main__":
    main()

