# Assignment 2 

- Lucas Gigondan's solution to the second assignment of research track 1 
- Class : Research Track 1 
- Professor : Carmine Tommaso Recchiuto
- Year : 2022-2023
- Student : Lucas Gigondan 
- Student ID : s5646676

## A. How to run the code 

- First, clone this repository in the src folder of your ros workspace. 
This github repository contains two ros packages : 
    - assignment_2_2022 (the base of our work published by our professor)
      Be carefull to take my version because I did a slight modification in the launch file. 
    - my-package : my solution to this assignment with the three required nodes. 

- If you haven't setup your workspace yet you can follow this tutorial : http://wiki.ros.org/catkin/Tutorials/create_a_workspace

- Then you will need to build the ros package you just cloned. To do so, run this command : 
'catkin_make' (in the root directory of your workspace)

- Then you can launch the different nodes with this command : 
'roslaunch assignment_2_2022 assignment1.launch'

- If it is working correctly, two terminal (client and distance), rviz and gazebo should open. 

## B. Code explanation 
In this code explanation, I will explain how my_package works and the modification I did to the launch files. 

### 1. Client node (a) : 

The client has a first job : get the user input goal for the robot.
To do so, it opens a window asking the user for a 'x' and 'y' value. 
Once this value are entered, the node checks that they are actually float numbers.
If they are of type float, the node will send this goal to the server.

The second job is to publish data on a specific topic. 
The data to publish are : 
- velocity in X 
- velocity in Y 
- position in X 
- position in Y 
To find those values, the node will be subscribed to the /odom topic. 
Here the code is fairly simple : get the information from /odom and publish it back on /robot_position.
To be published in /robot_position correctly, we defined a special msg structure : output_client.

The third job of the node is to call the service node. 
If needed, the user can input 'user' and not a float number. 
In this case the service node is called. 

Here is the flow chart of this node : 


![image](https://user-images.githubusercontent.com/58549218/214709933-fca1b27b-6ad2-4cd7-b89c-e57193cf590f.png)




### 2. Service node (b) : 

The service node has to return the number of goals reached / cancelled. 
To do so, if analyses the state of the robot. The states of the robots can be obtained by subscribing to this topic : /reaching_goal/result
Here are the different states of the robot : 
- 0 Robot is going to a point
- 1 Robot is following a wall 
- 2 Goal has been cancelled
- 3 Goal has been reached 

By counting the number of times the state changes to either 2 and 3, we get the number of goals reached / cancelled.
Then, it returns this data when it is called. 
That way we can print those values on the client terminal. 


### 3. Distance node (c) : 

The distance node has to print (at a certain frequence) the current distance to the desired goal and the average velocity of the robot. 
To get those values, we subscribe to the topic /robot_position created previously.
This gives us the speed in x and y of the robot. 
We can then combine them with a simple mathematical formula to get the average speed. 
To print the distance to the goal, we get the position of the goal as a ros parameter. 
The current position of the robot can be found in /robot_position too. 
Knowing this, we could use pythagoras and find the distance that way but it is more efficient with the math.dist function. 

To set the frequency that we want to print the messages to the terminal, we set up a variable called frequency. 

This variable will then be used to get the period in miliseconds in which it should be printed. 
Then we just need to check if the (current time) - ( last time we printed) is bigger than the desired period.
If yes we print again, otherwise we wait. 

### 4. Launch files

As I am not working in the main package (assignment_2_2022), a new launch file was required. 
This launch file called my_package.launch starts the three nodes, two of them have a terminal the service one just runs in the background. 
You don't have to call this launch file manually as it is called when you call the launch file assignment1.launch. 


## C. Possible upgrades 

- Create a possibility to update the frequency at which informations are printed in the 'distance' node while the robot is moving / active. 

- Developping a better user interface could create a better user experience. 
Right now the code is not really comprehensive : if you type 'Service' instead of 'service' it will just output an error. 
It would be easier if you could call the service node by clicking a button. 

- Make the robot faster by changing his speeds but also the way he handles an obstacle. 

- Print more informations to the logs for example the service node could also register the positions of the reached / cancelled goals : 
For example : 

3 Goals reached : 
- [1,1]
- [7,5]
- [-6,-2]

2 Goals cancelled : 
- [4,-3]
- [1,2]

This could even allow in the future to draw a map to show all the movements the robot did. 
Imagine this robot in a warehouse, moving pallets around to different locations.
This map could help you optimize the shipping / storing process. 

If anything is unclear, feel free to contact me :
lucas.gigondan@gmail.com
