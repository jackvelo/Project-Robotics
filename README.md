# Project-Robotics

In this project a robotic agent able to carry out several tasks is designed. The robot manages to gather information about its environment using the sensors, plan a set of actions 
to respond appropriately to sensed data based on a pre-existing strategy, and execute a set of motor commands to carry out the actions that the plan calls for. The robot is simulated in the robot simulator CoppeliaSim.

The following milestones were completed:

1. Exploration of the whole map (building an appropriate representation of the latter)

- by accessing the GPS coordinates (Main Code).

- by utilizing the distance to three beacons which are sending radio signals through the sensor. The beacons' signal is affected by noise. A particle filter is developed to 
increase the accuracy of the results (Milestone1B). 

2. Manipulation

The youBot reaches a “TargetTable” (identified in the exploration milestone) and identifies the position of each objects thanks to its sensors. The youBot is so grabbing all the 
objects on the table, without any falling on the ground, and put them on the target table (Main Code).

The project is realized in Python. 
